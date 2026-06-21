#include "hbos_engine.h"
#include "hbos_types.h"
#include "hbos_math.h"
#include <hls_stream.h>

static count_t hist[NR_SENSORS][NR_BINS];
static count_t d_hist[NR_SENSORS][NR_DELTA_BINS];
static count_t score_hist[2048];

static delta_addr_t  delta_th[NR_SENSORS];
static weight_t      sensor_weights[NR_SENSORS] = {50, 93, 58, 55};
static spike_t       spike_penalty = 5632;
static total_score_t global_threshold = 32767;

static bin_addr_t   hb_last_addr[NR_SENSORS]  = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
static count_t      hb_last_val[NR_SENSORS]    = {0, 0, 0, 0};
static delta_addr_t hb_last_d_addr[NR_SENSORS] = {0xFF, 0xFF, 0xFF, 0xFF};
static count_t      hb_last_d_val[NR_SENSORS]  = {0, 0, 0, 0};

static void convert_hist_to_score(count_t train_count) {
    #pragma HLS INLINE off
    ap_uint<16> log2_denom = aprox_log2(train_count + 2048);
    for (int i = 0; i < NR_SENSORS; i++) {
        for (int j = 0; j < NR_BINS; j++) {
            #pragma HLS PIPELINE II=2
            ap_uint<16> log2_num = aprox_log2(hist[i][j] + 1);
            if (log2_denom > log2_num) {
                hist[i][j] = (count_t)(log2_denom - log2_num);
            } else {
                hist[i][j] = 0;
            }
        }
        count_t target = train_count - (train_count >> 10);
        count_t cumulative = 0;
        delta_addr_t delta_bin = 255;
        for (int d = 0; d < NR_DELTA_BINS; d++) {
            #pragma HLS PIPELINE II=1
            cumulative += d_hist[i][d];
            if (cumulative >= target && delta_bin == 255) {
                delta_bin = d;
            }
        }
        delta_th[i] = delta_bin;
    }
}

static void finalize_global_threshold(count_t calib_count) {
    #pragma HLS INLINE off
    count_t target = calib_count - (calib_count >> 9);
    count_t cumulative = 0;
    ap_uint<11> threshold_bin = 2047;
    for (int i = 0; i < 2048; i++) {
        #pragma HLS PIPELINE II=1
        cumulative += score_hist[i];
        if (cumulative >= target && threshold_bin == 2047) {
            threshold_bin = i;
        }
    }
    global_threshold = (total_score_t)threshold_bin << 4;
}

static total_score_t engine_score(addr_packet_t &pkt) {
    #pragma HLS INLINE
    total_score_t total = 0;
    for (int i = 0; i < NR_SENSORS; i++) {
        #pragma HLS UNROLL factor=K_PARALLEL

        if (i < (int)pkt.active_count) {
            bin_addr_t   addr   = pkt.addr[i];
            delta_addr_t d_addr = pkt.d_addr[i];
            hbos_score_t base_score = hist[i][addr];
            if (d_addr > delta_th[i]) {
                base_score += (hbos_score_t)spike_penalty;
            }
            total += (total_score_t)((base_score * sensor_weights[i]) >> 8);
        }
    }
    return total;
}

static void histogram_builder(addr_packet_t &pkt, bool is_clean) {
    #pragma HLS INLINE
    #pragma HLS dependence variable=hist   type=inter direction=RAW dependent=false
    #pragma HLS dependence variable=hist   type=intra direction=RAW dependent=false
    #pragma HLS dependence variable=d_hist type=inter direction=RAW dependent=false
    #pragma HLS dependence variable=d_hist type=intra direction=RAW dependent=false

    for (int i = 0; i < NR_SENSORS; i++) {
        #pragma HLS UNROLL factor=K_PARALLEL
        if (is_clean && i < (int)pkt.active_count) {
            bin_addr_t curr_addr = pkt.addr[i];
            count_t curr_val;
            if (curr_addr == hb_last_addr[i]) {
                curr_val = hb_last_val[i] + 1;
            } else {
                curr_val = hist[i][curr_addr] + 1;
            }
            hist[i][curr_addr] = curr_val;
            hb_last_addr[i] = curr_addr;
            hb_last_val[i]  = curr_val;

            delta_addr_t curr_d_addr = pkt.d_addr[i];
            count_t curr_d_val;
            if (curr_d_addr == hb_last_d_addr[i]) {
                curr_d_val = hb_last_d_val[i] + 1;
            } else {
                curr_d_val = d_hist[i][curr_d_addr] + 1;
            }
            d_hist[i][curr_d_addr] = curr_d_val;
            hb_last_d_addr[i] = curr_d_addr;
            hb_last_d_val[i]  = curr_d_val;
        }
    }
}

void hbos_engine(
    hls::stream<addr_packet_t>&    in_stream,
    hls::stream<anomaly_packet_t>& anomaly_out
) {
    #pragma HLS INTERFACE axis port=in_stream
    #pragma HLS INTERFACE axis port=anomaly_out
    #pragma HLS INTERFACE ap_ctrl_none port=return
    #pragma HLS ARRAY_PARTITION variable=hist   cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=d_hist cyclic factor=K_PARALLEL dim=1
    #pragma HLS BIND_STORAGE   variable=hist   type=ram_2p
    #pragma HLS BIND_STORAGE   variable=d_hist type=ram_2p
    #pragma HLS ARRAY_PARTITION variable=delta_th       cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=sensor_weights cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=hb_last_addr   cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=hb_last_val    cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=hb_last_d_addr cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=hb_last_d_val  cyclic factor=K_PARALLEL dim=1

    static count_t       train_count = 0;
    static count_t       calib_count = 0;
    static opcode_t      last_opcode = OP_TRAIN;
    static bool          calib_done       = false;
    static bool          hist_converted   = false;
    static bool          config_written   = false;
    static bool          threshold_ready  = false;
    static ap_uint<32>   total_rx_train = 0;
    static ap_uint<32>   total_rx_calib = 0;

    static ap_uint<8>    telem[5];
    #pragma HLS ARRAY_PARTITION variable=telem complete
    static ap_uint<5>    config_dump_state = 0;

    addr_packet_t pkt = in_stream.read();
    opcode_t opcode = pkt.opcode;

    bool       do_write   = false;
    ap_uint<8> write_data = 0;

    if (pkt.frame_ok) {

        if (opcode == OP_TRAIN && last_opcode != OP_TRAIN) {
            train_count = 0; calib_count = 0;
            calib_done = false; hist_converted = false;
            config_written = false; threshold_ready = false;
            total_rx_train = 0; total_rx_calib = 0;
            config_dump_state = 0;
            for (int i = 0; i < NR_SENSORS; i++) {
                #pragma HLS UNROLL factor=K_PARALLEL
                hb_last_addr[i] = 0xFFFF; hb_last_val[i] = 0;
                hb_last_d_addr[i] = 0xFF;  hb_last_d_val[i] = 0;
            }
        }

        if (opcode == OP_RESET) {
            for (int j = 0; j < NR_BINS; j++) {
                #pragma HLS PIPELINE II=1
                for (int i = 0; i < NR_SENSORS; i++) {
                    #pragma HLS UNROLL factor=K_PARALLEL
                    hist[i][j] = 0;
                }
            }
            for (int j = 0; j < NR_DELTA_BINS; j++) {
                #pragma HLS PIPELINE II=1
                for (int i = 0; i < NR_SENSORS; i++) {
                    #pragma HLS UNROLL factor=K_PARALLEL
                    d_hist[i][j] = 0;
                }
            }
            for (int j = 0; j < 2048; j++) {
                #pragma HLS PIPELINE II=1
                score_hist[j] = 0;
            }
            for (int i = 0; i < NR_SENSORS; i++) {
                #pragma HLS UNROLL factor=K_PARALLEL
                delta_th[i]       = 0;
                hb_last_addr[i]   = 0xFFFF; hb_last_val[i]   = 0;
                hb_last_d_addr[i] = 0xFF;   hb_last_d_val[i] = 0;
            }
            train_count = 0; calib_count = 0;
            calib_done = false; hist_converted = false;
            config_written = false; threshold_ready = false;
            total_rx_train = 0; total_rx_calib = 0;
            global_threshold = 32767; config_dump_state = 0;
        }
        else if (opcode == OP_CONFIG) {
            for (int i = 0; i < NR_SENSORS; i++) {
                #pragma HLS UNROLL factor=K_PARALLEL
                sensor_weights[i] = (weight_t)pkt.d_addr[i];
            }
            spike_penalty = ((spike_t)pkt.addr[1] << 11) | (spike_t)pkt.addr[0];

            config_written = false;
        }
        else if (opcode == OP_TRAIN) {
            total_rx_train++;
            bool is_clean = (pkt.tlast == 0);
            if (is_clean) train_count++;
            histogram_builder(pkt, is_clean);
        }
        else if (opcode == OP_CALIB) {
            total_rx_calib++;
            calib_done = true;
            if (!hist_converted) {
                convert_hist_to_score(train_count);
                hist_converted = true;
                calib_count = 0;
                for (int j = 0; j < 2048; j++) {
                    #pragma HLS PIPELINE II=1
                    score_hist[j] = 0;
                }
                for (int j = 0; j < NR_DELTA_BINS; j++) {
                    #pragma HLS PIPELINE II=1
                    for (int i = 0; i < NR_SENSORS; i++) {
                        #pragma HLS UNROLL factor=K_PARALLEL
                        d_hist[i][j] = 0;
                    }
                }
            }
            bool is_clean = (pkt.tlast == 0);
            if (is_clean) {
                calib_count++;
                total_score_t score = engine_score(pkt);
                ap_uint<22> s_idx_full = (ap_uint<22>)(score >> 4);
                bin_addr_t s_idx = (s_idx_full >= 2048) ? (bin_addr_t)2047 : (bin_addr_t)s_idx_full;
                score_hist[s_idx]++;
            }
        }
        else if (opcode == OP_DUMP) {
            if (calib_done && !config_written) {

                finalize_global_threshold(calib_count);
                telem[0] = 0xFE;
                telem[1] = (ap_uint<8>)(global_threshold & 0xFF);
                telem[2] = (ap_uint<8>)((global_threshold >> 8) & 0xFF);
                telem[3] = (ap_uint<8>)((global_threshold >> 16) & 0xFF);
                telem[4] = 0xFF;
                config_written  = true;
                threshold_ready = true;
                config_dump_state = 1;
                do_write   = true;
                write_data = 0xFF;
            } else if (config_dump_state > 0) {
                do_write   = true;
                write_data = telem[config_dump_state - 1];
                config_dump_state = (config_dump_state >= 5)
                                  ? (ap_uint<5>)0
                                  : (ap_uint<5>)(config_dump_state + 1);
            }
        }
        else if (opcode == OP_DETECT && threshold_ready) {
            total_score_t total = engine_score(pkt);
            bool is_anomaly = (total >= global_threshold);
            do_write   = true;
            write_data = is_anomaly ? (ap_uint<8>)0x01 : (ap_uint<8>)0x00;
        }

        if (opcode != OP_DUMP) {
            last_opcode = opcode;
        }
    }

    if (do_write) {
        anomaly_packet_t out_pkt;
        out_pkt.data = write_data;
        out_pkt.keep = -1;
        out_pkt.strb = -1;
        out_pkt.last = 1;
        anomaly_out.write(out_pkt);
    }
}
