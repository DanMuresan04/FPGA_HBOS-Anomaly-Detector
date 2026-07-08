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
static ap_uint<5>    calib_shift = 9;

#define FWD_DEPTH 3
static bin_addr_t   fwd_h_addr[NR_SENSORS][FWD_DEPTH];
static count_t      fwd_h_val [NR_SENSORS][FWD_DEPTH];
static bool         fwd_h_ok  [NR_SENSORS][FWD_DEPTH];
static delta_addr_t fwd_d_addr[NR_SENSORS][FWD_DEPTH];
static count_t      fwd_d_val [NR_SENSORS][FWD_DEPTH];
static bool         fwd_d_ok  [NR_SENSORS][FWD_DEPTH];
static ap_uint<11>  fwd_s_addr[FWD_DEPTH];
static count_t      fwd_s_val [FWD_DEPTH];
static bool         fwd_s_ok  [FWD_DEPTH];

static void fwd_invalidate_hist() {
    #pragma HLS INLINE
    for (int i = 0; i < NR_SENSORS; i++) {
        #pragma HLS UNROLL
        for (int k = 0; k < FWD_DEPTH; k++) {
            #pragma HLS UNROLL
            fwd_h_ok[i][k] = false;
            fwd_d_ok[i][k] = false;
        }
    }
}

static void fwd_invalidate_score() {
    #pragma HLS INLINE
    for (int k = 0; k < FWD_DEPTH; k++) {
        #pragma HLS UNROLL
        fwd_s_ok[k] = false;
    }
}

static count_t train_count = 0;

enum {
    M_IDLE   = 0,
    M_ZERO   = 1,
    M_DSCAN  = 2,
    M_ZCALIB = 3,
    M_FINAL  = 4
};

#define MAINT_SETTLE 24

static ap_uint<3>  m_state  = M_IDLE;
static ap_uint<12> m_idx    = 0;
static ap_uint<5>  m_settle = 0;

static addr_packet_t pending_pkt;
static bool          pending_valid = false;

static count_t      dscan_cum[NR_SENSORS];
static delta_addr_t dscan_bin[NR_SENSORS];
static count_t      dscan_target;

static count_t     fin_cum;
static ap_uint<11> fin_bin;
static count_t     fin_target;
static bool        fin_done = false;

static count_t     calib_count = 0;
static opcode_t    last_opcode = OP_TRAIN;
static bool        calib_done       = false;
static bool        hist_converted   = false;
static bool        config_written   = false;
static bool        threshold_ready  = false;
static ap_uint<32> total_rx_train = 0;
static ap_uint<32> total_rx_calib = 0;
static ap_uint<8>  telem[5];
static ap_uint<5>  config_dump_state = 0;

static total_score_t engine_score(addr_packet_t &pkt) {
    #pragma HLS INLINE

    ap_uint<16> log2_denom = aprox_log2((ap_uint<32>)train_count + 2048);
    total_score_t total = 0;
    for (int i = 0; i < NR_SENSORS; i++) {
        #pragma HLS UNROLL factor=K_PARALLEL

        if (i < (int)pkt.active_count) {
            bin_addr_t   addr   = pkt.addr[i];
            delta_addr_t d_addr = pkt.d_addr[i];
            count_t      cnt    = hist[i][addr];
            ap_uint<16>  log2_num = aprox_log2((ap_uint<32>)cnt + 1);
            hbos_score_t base_score = (log2_denom > log2_num)
                                    ? (hbos_score_t)(log2_denom - log2_num)
                                    : (hbos_score_t)0;
            if (d_addr > delta_th[i]) {
                base_score += (hbos_score_t)spike_penalty;
            }
            total += (total_score_t)((base_score * sensor_weights[i]) >> 8);
        }
    }
    return total;
}

static void hbos_detect_one(
    addr_packet_t&               pkt,
    hls::stream<verdict_beat_t>& anomaly_out
) {

    #pragma HLS INLINE
    total_score_t total = engine_score(pkt);
    bool is_anomaly = (total >= global_threshold);

    verdict_beat_t v;
    v.data = ((ap_uint<32>)(is_anomaly ? 0x01 : 0x00) << 24)
           | (ap_uint<32>)(pkt.seq & 0xFFFFFF);
    v.keep = 0xF;
    v.strb = 0xF;
    v.last = 1;
    anomaly_out.write(v);
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

            count_t cand = hist[i][curr_addr];
            for (int k = FWD_DEPTH - 1; k >= 0; k--) {
                #pragma HLS UNROLL
                if (fwd_h_ok[i][k] && fwd_h_addr[i][k] == curr_addr) {
                    cand = fwd_h_val[i][k];
                }
            }
            count_t curr_val = cand + 1;
            hist[i][curr_addr] = curr_val;
            for (int k = FWD_DEPTH - 1; k > 0; k--) {
                #pragma HLS UNROLL
                fwd_h_addr[i][k] = fwd_h_addr[i][k-1];
                fwd_h_val [i][k] = fwd_h_val [i][k-1];
                fwd_h_ok  [i][k] = fwd_h_ok  [i][k-1];
            }
            fwd_h_addr[i][0] = curr_addr;
            fwd_h_val [i][0] = curr_val;
            fwd_h_ok  [i][0] = true;

            delta_addr_t curr_d_addr = pkt.d_addr[i];
            count_t d_cand = d_hist[i][curr_d_addr];
            for (int k = FWD_DEPTH - 1; k >= 0; k--) {
                #pragma HLS UNROLL
                if (fwd_d_ok[i][k] && fwd_d_addr[i][k] == curr_d_addr) {
                    d_cand = fwd_d_val[i][k];
                }
            }
            count_t curr_d_val = d_cand + 1;
            d_hist[i][curr_d_addr] = curr_d_val;
            for (int k = FWD_DEPTH - 1; k > 0; k--) {
                #pragma HLS UNROLL
                fwd_d_addr[i][k] = fwd_d_addr[i][k-1];
                fwd_d_val [i][k] = fwd_d_val [i][k-1];
                fwd_d_ok  [i][k] = fwd_d_ok  [i][k-1];
            }
            fwd_d_addr[i][0] = curr_d_addr;
            fwd_d_val [i][0] = curr_d_val;
            fwd_d_ok  [i][0] = true;
        }
    }
}

static void maintenance_step() {
    #pragma HLS INLINE
    switch ((int)m_state) {

    case M_ZERO: {
        int j = (int)m_idx;
        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL
            hist[i][j] = 0;
            if (j < NR_DELTA_BINS) d_hist[i][j] = 0;
        }
        score_hist[j] = 0;
        if (m_idx == NR_BINS - 1) {
            fwd_invalidate_hist();
            fwd_invalidate_score();
            m_state = M_IDLE; m_idx = 0;
            m_settle = MAINT_SETTLE;
        } else m_idx++;
        break;
    }

    case M_DSCAN: {
        int d = (int)m_idx;
        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL
            dscan_cum[i] = dscan_cum[i] + d_hist[i][d];
            if (dscan_cum[i] >= dscan_target && dscan_bin[i] == 255) {
                dscan_bin[i] = (delta_addr_t)d;
            }
        }
        if (m_idx == NR_DELTA_BINS - 1) {
            for (int i = 0; i < NR_SENSORS; i++) {
                #pragma HLS UNROLL
                delta_th[i] = dscan_bin[i];
            }
            m_state = M_ZCALIB; m_idx = 0;
            m_settle = MAINT_SETTLE;
        } else m_idx++;
        break;
    }

    case M_ZCALIB: {
        int j = (int)m_idx;
        score_hist[j] = 0;
        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL
            if (j < NR_DELTA_BINS) d_hist[i][j] = 0;
        }
        if (m_idx == NR_BINS - 1) {
            fwd_invalidate_hist();
            fwd_invalidate_score();
            m_state = M_IDLE; m_idx = 0;
            m_settle = MAINT_SETTLE;
        } else m_idx++;
        break;
    }

    case M_FINAL: {
        fin_cum = fin_cum + score_hist[m_idx];
        if (fin_cum >= fin_target && fin_bin == 2047) {
            fin_bin = (ap_uint<11>)m_idx;
        }
        if (m_idx == NR_BINS - 1) {
            global_threshold = (total_score_t)fin_bin << 4;
            fin_done = true;
            m_state = M_IDLE; m_idx = 0;
            m_settle = MAINT_SETTLE;
        } else m_idx++;
        break;
    }

    default:
        m_state = M_IDLE;
        break;
    }
}

static void process_packet(addr_packet_t &pkt,
                           hls::stream<verdict_beat_t>& anomaly_out) {
    #pragma HLS INLINE
    opcode_t opcode = pkt.opcode;

    bool       do_write   = false;
    ap_uint<8> write_data = 0;

    if (pkt.frame_ok) {

        if (opcode == OP_TRAIN && last_opcode != OP_TRAIN) {
            train_count = 0; calib_count = 0;
            calib_done = false; hist_converted = false;
            config_written = false; threshold_ready = false; fin_done = false;
            total_rx_train = 0; total_rx_calib = 0;
            config_dump_state = 0;
            last_opcode = OP_TRAIN;
            pending_pkt = pkt; pending_valid = true;
            m_state = M_ZERO; m_idx = 0; m_settle = MAINT_SETTLE;
            return;
        }

        if (opcode == OP_RESET) {
            for (int i = 0; i < NR_SENSORS; i++) {
                #pragma HLS UNROLL factor=K_PARALLEL
                delta_th[i] = 0;
            }
            train_count = 0; calib_count = 0;
            calib_done = false; hist_converted = false;
            config_written = false; threshold_ready = false; fin_done = false;
            total_rx_train = 0; total_rx_calib = 0;
            global_threshold = 32767; config_dump_state = 0;
            calib_shift = 9;
            last_opcode = OP_RESET;
            m_state = M_ZERO; m_idx = 0; m_settle = MAINT_SETTLE;
            return;
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
            fin_done       = false;
        }
        else if (opcode == OP_TRAIN) {
            total_rx_train++;
            bool is_clean = (pkt.tlast == 0);
            if (is_clean) train_count++;
            histogram_builder(pkt, is_clean);
        }
        else if (opcode == OP_CALIB) {
            if (!hist_converted) {

                hist_converted = true;
                calib_count = 0;
                dscan_target = train_count - (train_count >> 10);
                for (int i = 0; i < NR_SENSORS; i++) {
                    #pragma HLS UNROLL
                    dscan_cum[i] = 0;
                    dscan_bin[i] = 255;
                }
                last_opcode = OP_CALIB;
                pending_pkt = pkt; pending_valid = true;
                m_state = M_DSCAN; m_idx = 0; m_settle = MAINT_SETTLE;
                return;
            }
            total_rx_calib++;
            calib_done = true;
            bool is_clean = (pkt.tlast == 0);
            if (is_clean) {
                calib_count++;
                total_score_t score = engine_score(pkt);
                ap_uint<22> s_idx_full = (ap_uint<22>)(score >> 4);
                bin_addr_t s_idx = (s_idx_full >= 2048) ? (bin_addr_t)2047 : (bin_addr_t)s_idx_full;

                count_t s_cand = score_hist[s_idx];
                for (int k = FWD_DEPTH - 1; k >= 0; k--) {
                    #pragma HLS UNROLL
                    if (fwd_s_ok[k] && fwd_s_addr[k] == (ap_uint<11>)s_idx) {
                        s_cand = fwd_s_val[k];
                    }
                }
                count_t s_new = s_cand + 1;
                score_hist[s_idx] = s_new;
                for (int k = FWD_DEPTH - 1; k > 0; k--) {
                    #pragma HLS UNROLL
                    fwd_s_addr[k] = fwd_s_addr[k-1];
                    fwd_s_val [k] = fwd_s_val [k-1];
                    fwd_s_ok  [k] = fwd_s_ok  [k-1];
                }
                fwd_s_addr[0] = (ap_uint<11>)s_idx;
                fwd_s_val [0] = s_new;
                fwd_s_ok  [0] = true;
            }
        }
        else if (opcode == OP_DUMP) {
            if (calib_done && !config_written) {
                if (!fin_done) {

                    fin_target = calib_count - (calib_count >> calib_shift);
                    fin_cum = 0; fin_bin = 2047;
                    pending_pkt = pkt; pending_valid = true;
                    m_state = M_FINAL; m_idx = 0; m_settle = MAINT_SETTLE;
                    return;
                }
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
            hbos_detect_one(pkt, anomaly_out);
        }

        if (opcode != OP_DUMP) {
            last_opcode = opcode;
        }
    }

    if (do_write) {

        verdict_beat_t out_pkt;
        out_pkt.data = (ap_uint<32>)write_data;
        out_pkt.keep = 0x1;
        out_pkt.strb = 0x1;
        out_pkt.last = 1;
        anomaly_out.write(out_pkt);
    }
}

void hbos_engine(
    hls::stream<addr_ctrl_t>&    in_ctrl,
    hls::stream<addr_data_t>&    in_data,
    hls::stream<verdict_beat_t>& anomaly_out
) {
    #pragma HLS INTERFACE axis port=in_ctrl
    #pragma HLS INTERFACE axis port=in_data
    #pragma HLS INTERFACE axis port=anomaly_out
    #pragma HLS INTERFACE ap_ctrl_none port=return
    #pragma HLS ARRAY_PARTITION variable=hist   cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=d_hist cyclic factor=K_PARALLEL dim=1

    #pragma HLS BIND_STORAGE   variable=hist   type=ram_t2p
    #pragma HLS BIND_STORAGE   variable=d_hist type=ram_t2p

    #pragma HLS dependence variable=hist       type=inter dependent=false
    #pragma HLS dependence variable=d_hist     type=inter dependent=false
    #pragma HLS dependence variable=score_hist type=inter dependent=false
    #pragma HLS ARRAY_PARTITION variable=delta_th       cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=sensor_weights cyclic factor=K_PARALLEL dim=1

    #pragma HLS ARRAY_PARTITION variable=fwd_h_addr complete dim=0
    #pragma HLS ARRAY_PARTITION variable=fwd_h_val  complete dim=0
    #pragma HLS ARRAY_PARTITION variable=fwd_h_ok   complete dim=0
    #pragma HLS ARRAY_PARTITION variable=fwd_d_addr complete dim=0
    #pragma HLS ARRAY_PARTITION variable=fwd_d_val  complete dim=0
    #pragma HLS ARRAY_PARTITION variable=fwd_d_ok   complete dim=0
    #pragma HLS ARRAY_PARTITION variable=fwd_s_addr complete dim=0
    #pragma HLS ARRAY_PARTITION variable=fwd_s_val  complete dim=0
    #pragma HLS ARRAY_PARTITION variable=fwd_s_ok   complete dim=0
    #pragma HLS BIND_STORAGE    variable=score_hist type=ram_t2p
    #pragma HLS ARRAY_PARTITION variable=telem      complete
    #pragma HLS ARRAY_PARTITION variable=dscan_cum  complete dim=1
    #pragma HLS ARRAY_PARTITION variable=dscan_bin  complete dim=1

    #pragma HLS PIPELINE II=1

    if (m_settle != 0) {
        m_settle = m_settle - 1;
        return;
    }
    if (m_state != M_IDLE) {
        maintenance_step();
        return;
    }

    addr_packet_t pkt;
    if (pending_valid) {

        pending_valid = false;
        pkt = pending_pkt;
    } else {
        addr_ctrl_t ctrl;
        if (!in_ctrl.read_nb(ctrl)) return;
        addr_data_t dat = in_data.read();
        pkt.opcode       = ctrl.opcode;
        pkt.tlast        = ctrl.tlast;
        pkt.active_count = ctrl.active_count;
        pkt.seq          = ctrl.seq;
        pkt.frame_ok     = ctrl.frame_ok;
        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL
            pkt.addr[i]   = dat.addr[i];
            pkt.d_addr[i] = dat.d_addr[i];
        }
    }

    process_packet(pkt, anomaly_out);

#ifndef __SYNTHESIS__

    while (m_state != M_IDLE || m_settle != 0 || pending_valid) {
        if (m_settle != 0)          { m_settle = m_settle - 1; }
        else if (m_state != M_IDLE) { maintenance_step(); }
        else {
            addr_packet_t p2 = pending_pkt;
            pending_valid = false;
            process_packet(p2, anomaly_out);
        }
    }
#endif
}
