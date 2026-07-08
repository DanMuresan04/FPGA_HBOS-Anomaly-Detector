#include "hbos_top.h"
count_t d_hist[NR_SENSORS][NR_DELTA_BINS] = {0};
count_t score_hist[2048] = {0};
delta_addr_t delta_th[NR_SENSORS] = {0};

static bin_addr_t   hb_last_addr[NR_SENSORS]   = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
static count_t      hb_last_val[NR_SENSORS]     = {0, 0, 0, 0};
static delta_addr_t hb_last_d_addr[NR_SENSORS]  = {0xFF, 0xFF, 0xFF, 0xFF};
static count_t      hb_last_d_val[NR_SENSORS]   = {0, 0, 0, 0};

static weight_t sensor_weights[NR_SENSORS] = {50, 93, 58, 55};
static spike_t  spike_penalty = 5632;

static ap_uint<5> calib_shift = 9;

#include "hbos_math.h"
void convert_hist_to_score(count_t hist[NR_SENSORS][NR_BINS], count_t train_count) {
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

void finalize_global_threshold(count_t calib_count, total_score_t &threshold) {

    count_t target = calib_count - (calib_count >> calib_shift);
    count_t cumulative = 0;
    ap_uint<11> threshold_bin = 2047;
    for (int i = 0; i < 2048; i++) {
        #pragma HLS PIPELINE II=1
        cumulative += score_hist[i];
        if (cumulative >= target && threshold_bin == 2047) {
            threshold_bin = i;
        }
    }
    threshold = (total_score_t)threshold_bin << 4;
}

total_score_t engine_score(count_t hist[NR_SENSORS][NR_BINS], addr_packet_t &pkt) {
    #pragma HLS INLINE
    total_score_t total = 0;
    for (int i = 0; i < NR_SENSORS; i++) {
        #pragma HLS UNROLL factor=K_PARALLEL
        bin_addr_t addr = pkt.addr[i];
        delta_addr_t d_addr = pkt.d_addr[i];
        hbos_score_t base_score = hist[i][addr];
        if (d_addr > delta_th[i]) {
            base_score += (hbos_score_t)spike_penalty;
        }
        total += (total_score_t)((base_score * sensor_weights[i]) >> 8);
    }
    return total;
}

void histogram_builder(count_t hist[NR_SENSORS][NR_BINS], addr_packet_t &pkt, bool is_clean) {
    #pragma HLS INLINE

    bin_addr_t   (&last_addr)[NR_SENSORS]   = hb_last_addr;
    count_t      (&last_val)[NR_SENSORS]    = hb_last_val;
    delta_addr_t (&last_d_addr)[NR_SENSORS] = hb_last_d_addr;
    count_t      (&last_d_val)[NR_SENSORS]  = hb_last_d_val;

    #pragma HLS dependence variable=hist type=inter direction=RAW dependent=false
    #pragma HLS dependence variable=hist type=intra direction=RAW dependent=false
    #pragma HLS dependence variable=d_hist type=inter direction=RAW dependent=false
    #pragma HLS dependence variable=d_hist type=intra direction=RAW dependent=false

    for (int i = 0; i < NR_SENSORS; i++) {
        #pragma HLS UNROLL factor=K_PARALLEL
        if (is_clean) {

            bin_addr_t curr_addr = pkt.addr[i];
            count_t curr_val;
            if (curr_addr == last_addr[i]) {
                curr_val = last_val[i] + 1;
            } else {
                curr_val = hist[i][curr_addr] + 1;
            }
            hist[i][curr_addr] = curr_val;
            last_addr[i] = curr_addr;
            last_val[i] = curr_val;

            delta_addr_t curr_d_addr = pkt.d_addr[i];
            count_t curr_d_val;
            if (curr_d_addr == last_d_addr[i]) {
                curr_d_val = last_d_val[i] + 1;
            } else {
                curr_d_val = d_hist[i][curr_d_addr] + 1;
            }
            d_hist[i][curr_d_addr] = curr_d_val;
            last_d_addr[i] = curr_d_addr;
            last_d_val[i] = curr_d_val;
        }
    }
}

void hbos_top(hls::stream<addr_packet_t>& in_stream, count_t hist[NR_SENSORS][NR_BINS], hls::stream<ap_uint<32>>& config_out) {
    #pragma HLS INTERFACE axis port=in_stream
    #pragma HLS INTERFACE bram port=hist storage_type=ram_2p
    #pragma HLS INTERFACE axis port=config_out
    #pragma HLS INTERFACE ap_ctrl_none port=return
    #pragma HLS ARRAY_PARTITION variable=hist cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=d_hist cyclic factor=K_PARALLEL dim=1
    #pragma HLS BIND_STORAGE variable=d_hist type=ram_2p
    #pragma HLS ARRAY_PARTITION variable=delta_th cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=sensor_weights cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=hb_last_addr   cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=hb_last_val     cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=hb_last_d_addr  cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=hb_last_d_val   cyclic factor=K_PARALLEL dim=1

    static count_t train_count = 0;
    static count_t calib_count = 0;
    static total_score_t global_threshold = 32767;
    static opcode_t last_opcode = OP_TRAIN;
    static bool calib_done = false;
    static bool hist_converted = false;

    static bool config_written = false;

    static ap_uint<32> total_rx_train = 0;
    static ap_uint<32> total_rx_calib = 0;

    addr_packet_t pkt;
    bool has_slow_packet = false;

    while (!in_stream.empty()) {
        #pragma HLS PIPELINE II=1
        pkt = in_stream.read();

        if (pkt.frame_ok && pkt.opcode != OP_TRAIN) {
            has_slow_packet = true;
            break;
        }

        if (!pkt.frame_ok) {
            continue;
        }

        opcode_t opcode = pkt.opcode;
        bool is_clean = (pkt.tlast == 0);

        if (opcode == OP_TRAIN && last_opcode != OP_TRAIN) {
            train_count = 0;
            calib_count = 0;
            calib_done = false;
            hist_converted = false;
            total_rx_train = 0;
            total_rx_calib = 0;
            config_written = false;
        }

        total_rx_train++;
        if (is_clean) {
            train_count++;
        }
        histogram_builder(hist, pkt, is_clean);

        last_opcode = opcode;
    }

    if (has_slow_packet) {
        opcode_t opcode = pkt.opcode;
        bool is_clean = (pkt.tlast == 0);

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
                hb_last_addr[i]   = 0xFFFF;
                hb_last_val[i]    = 0;
                hb_last_d_addr[i] = 0xFF;
                hb_last_d_val[i]  = 0;
            }
            train_count      = 0;
            calib_count      = 0;
            calib_done       = false;
            hist_converted   = false;
            config_written   = false;
            total_rx_train   = 0;
            total_rx_calib   = 0;
            global_threshold = 32767;
            calib_shift      = 9;
#ifndef __SYNTHESIS__
            printf("[OP_RESET] full state flush\n");
#endif
        }
        else if (opcode == OP_CONFIG) {

            for (int i = 0; i < NR_SENSORS; i++) {
                #pragma HLS UNROLL factor=K_PARALLEL
                sensor_weights[i] = (weight_t)pkt.d_addr[i];
            }
            spike_penalty = ((spike_t)pkt.addr[1] << 11) | (spike_t)pkt.addr[0];

            {
                ap_uint<5> cs = (ap_uint<5>)pkt.addr[2];
                calib_shift = (cs == 0) ? (ap_uint<5>)9 : cs;
            }
            config_written = false;
#ifndef __SYNTHESIS__
            printf("[OP_CONFIG] weights={%d,%d,%d,%d} spike_penalty=%d calib_shift=%d\n",
                   (int)sensor_weights[0], (int)sensor_weights[1],
                   (int)sensor_weights[2], (int)sensor_weights[3],
                   (int)spike_penalty, (int)calib_shift);
#endif
        }
        else if (opcode == OP_DUMP && calib_done && !config_written) {

#ifndef __SYNTHESIS__
            printf("DUMP finalize: calib_count=%d\n", (int)calib_count);
#endif
            finalize_global_threshold(calib_count, global_threshold);
            ap_uint<32> packed_deltas = 0;
            ap_uint<32> packed_weights = 0;
            for (int i = 0; i < NR_SENSORS; i++) {
                #pragma HLS UNROLL factor=K_PARALLEL
                packed_deltas  |= ((ap_uint<32>)delta_th[i]      << (i * 8));
                packed_weights |= ((ap_uint<32>)sensor_weights[i] << (i * 8));
            }

            config_out.write((ap_uint<32>)global_threshold);
            config_out.write(packed_deltas);
            config_out.write(total_rx_train);
            config_out.write(total_rx_calib);
            config_out.write(packed_weights);
            config_out.write((ap_uint<32>)spike_penalty);
            config_written = true;
        }
        else if (opcode == OP_CALIB) {
            total_rx_calib++;
            calib_done = true;
            if (!hist_converted) {
                convert_hist_to_score(hist, train_count);
                hist_converted = true;
                calib_count = 0;
                config_written = false;

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
            if (is_clean) {
                calib_count++;
                total_score_t score = engine_score(hist, pkt);

                ap_uint<22> s_idx_full = (ap_uint<22>)(score >> 4);
                bin_addr_t s_idx = (s_idx_full >= 2048) ? (bin_addr_t)2047 : (bin_addr_t)s_idx_full;
                score_hist[s_idx]++;
            }
        }

        last_opcode = opcode;
    }
}
