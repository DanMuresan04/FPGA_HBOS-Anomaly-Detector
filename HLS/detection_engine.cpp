#include "hbos_top.h"
#include "hbos_types.h"
#include "hbos_math.h"
#include <hls_stream.h>

void detection_engine(
    hls::stream<addr_packet_t>&    in_stream,
    hls::stream<ap_uint<32>>&      config_in,
    count_t hist[NR_SENSORS][NR_BINS],
    hls::stream<anomaly_packet_t>& anomaly_out
) {
    #pragma HLS INTERFACE axis port=in_stream
    #pragma HLS INTERFACE axis port=config_in
    #pragma HLS INTERFACE axis port=anomaly_out
    #pragma HLS INTERFACE bram port=hist
    #pragma HLS INTERFACE ap_ctrl_none port=return
    #pragma HLS ARRAY_PARTITION variable=hist complete dim=1

    static total_score_t global_threshold = 0;
    static delta_addr_t  delta_th[NR_SENSORS];
    static ap_uint<32>   total_rx_train = 0;
    static ap_uint<32>   total_rx_calib = 0;
    static bool          inference_enabled  = false;
    static bool          calib_phase_seen   = false;
    static bool          dump_ack_pending   = false;
    static ap_uint<5>    config_dump_state  = 0;
    // ap_uint<3> for cfg_rx_cnt, but index into cfg_buf via (int) cast to
    // avoid the bit-extension penalty on array access (HLS 214-358).
    static ap_uint<3>    cfg_rx_cnt = 0;
    static ap_uint<32>   cfg_buf[6];
    #pragma HLS ARRAY_PARTITION variable=cfg_buf    complete
    #pragma HLS ARRAY_PARTITION variable=delta_th   complete dim=1

    static weight_t sensor_weights[NR_SENSORS] = {50, 93, 58, 55};
    #pragma HLS ARRAY_PARTITION variable=sensor_weights complete dim=1
    static spike_t spike_penalty = 5632;

    // Forwarding cache: mirrors histogram_builder's last_addr/last_val pattern.
    // Bypasses the BRAM entirely when consecutive packets land on the same bin.
    // cache_valid guards against stale hits on first use and after retraining.
    static bin_addr_t   last_hist_addr[NR_SENSORS];
    static hbos_score_t last_hist_score[NR_SENSORS];
    static bool         cache_valid[NR_SENSORS] = {false, false, false, false};
    #pragma HLS ARRAY_PARTITION variable=last_hist_addr  complete dim=1
    #pragma HLS ARRAY_PARTITION variable=last_hist_score complete dim=1
    #pragma HLS ARRAY_PARTITION variable=cache_valid     complete dim=1

    // Pipeline the whole function invocation; target II=1.
    // hist is read-only here so there is no inter-invocation RAW hazard on the BRAM.
    // The forwarding cache arrays have genuine RAW dependencies at distance=1.
    #pragma HLS PIPELINE II=1
    #pragma HLS dependence variable=hist            type=inter direction=RAW dependent=false
    #pragma HLS dependence variable=last_hist_addr  type=inter direction=RAW dependent=true distance=1
    #pragma HLS dependence variable=last_hist_score type=inter direction=RAW dependent=true distance=1
    #pragma HLS dependence variable=cache_valid     type=inter direction=RAW dependent=true distance=1

    addr_packet_t pkt = in_stream.read();
    opcode_t opcode = pkt.opcode;

    // Single write-point architecture: collect output into these locals then
    // call anomaly_out.write() exactly once at the bottom (if do_write is set).
    // Having a single write call on one AXI stream port removes the II=2
    // carried-dependence violation that arises from multiple write sites.
    bool       do_write   = false;
    ap_uint<8> write_data = 0;

    // ── config word accumulation ─────────────────────────────────────────────
    // hbos_top writes 6 words at DUMP: threshold, packed_deltas, rx_train,
    // rx_calib, packed_weights, spike_penalty.  One word is read per invocation
    // via read_nb; all six are latched atomically on the 6th arrival.
    {
        ap_uint<32> w;
        if (config_in.read_nb(w)) {
            int idx = (int)cfg_rx_cnt;    // int index avoids bit-extension penalty
            cfg_buf[idx] = w;
            if (cfg_rx_cnt == 5) {
                cfg_rx_cnt = 0;
                global_threshold = (total_score_t)cfg_buf[0];
                ap_uint<32> packed_deltas  = cfg_buf[1];
                ap_uint<32> packed_weights = cfg_buf[4];
                for (int i = 0; i < NR_SENSORS; i++) {
                    #pragma HLS UNROLL
                    delta_th[i]       = (delta_addr_t)((packed_deltas  >> (i * 8)) & 0xFF);
                    sensor_weights[i] = (weight_t)    ((packed_weights >> (i * 8)) & 0xFF);
                }
                total_rx_train = cfg_buf[2];
                total_rx_calib = cfg_buf[3];
                spike_penalty  = (spike_t)cfg_buf[5];
#ifndef __SYNTHESIS__
                printf("\n=====================================\n");
                printf("  [LATCHED HBOS CONFIGURATION]\n");
                printf("  Global Threshold: %d\n", (int)global_threshold);
                for (int i = 0; i < NR_SENSORS; i++) {
                    printf("  Sensor %d Delta Threshold: %d  Weight: %d\n",
                           i, (int)delta_th[i], (int)sensor_weights[i]);
                }
                printf("  Spike Penalty: %d\n", (int)spike_penalty);
                printf("  FPGA RX TRAIN frames: %u\n", (unsigned)total_rx_train);
                printf("  FPGA RX CALIB frames: %u\n", (unsigned)total_rx_calib);
                printf("=====================================\n\n");
#endif
                inference_enabled = true;
                dump_ack_pending = false;
                if (config_dump_state == 0) {
                    config_dump_state = 1;
                }
                // Immediate config-latched ack — takes priority over all other outputs.
                do_write   = true;
                write_data = 0xFF;
            } else {
                cfg_rx_cnt++;
            }
        }
    }

    // ── dump telemetry state machine ─────────────────────────────────────────
    // Advances one byte per OP_DUMP poll; yields to the config ack if both
    // happen to fire in the same invocation (do_write guard).
    // Layout: 0xFE | threshold[23:0] LE | delta_th[0..3] |
    //         rx_train[31:0] LE | rx_calib[31:0] LE | 0xFF
    if (!do_write && config_dump_state > 0 && opcode == OP_DUMP) {
        do_write = true;
        if (config_dump_state == 1) {
            write_data = 0xFE;
        } else if (config_dump_state == 2) {
            write_data = (ap_uint<8>)(global_threshold & 0xFF);
        } else if (config_dump_state == 3) {
            write_data = (ap_uint<8>)((global_threshold >> 8) & 0xFF);
        } else if (config_dump_state == 4) {
            write_data = (ap_uint<8>)((global_threshold >> 16) & 0xFF);
        } else if (config_dump_state == 5) {
            write_data = (ap_uint<8>)(delta_th[0]);
        } else if (config_dump_state == 6) {
            write_data = (ap_uint<8>)(delta_th[1]);
        } else if (config_dump_state == 7) {
            write_data = (ap_uint<8>)(delta_th[2]);
        } else if (config_dump_state == 8) {
            write_data = (ap_uint<8>)(delta_th[3]);
        } else if (config_dump_state == 9) {
            write_data = (ap_uint<8>)(total_rx_train & 0xFF);
        } else if (config_dump_state == 10) {
            write_data = (ap_uint<8>)((total_rx_train >> 8) & 0xFF);
        } else if (config_dump_state == 11) {
            write_data = (ap_uint<8>)((total_rx_train >> 16) & 0xFF);
        } else if (config_dump_state == 12) {
            write_data = (ap_uint<8>)((total_rx_train >> 24) & 0xFF);
        } else if (config_dump_state == 13) {
            write_data = (ap_uint<8>)(total_rx_calib & 0xFF);
        } else if (config_dump_state == 14) {
            write_data = (ap_uint<8>)((total_rx_calib >> 8) & 0xFF);
        } else if (config_dump_state == 15) {
            write_data = (ap_uint<8>)((total_rx_calib >> 16) & 0xFF);
        } else if (config_dump_state == 16) {
            write_data = (ap_uint<8>)((total_rx_calib >> 24) & 0xFF);
        } else {
            write_data = 0xFF;
            config_dump_state = 0;
        }
        if (config_dump_state != 0) {
            config_dump_state++;
        }
    }

    // ── opcode dispatch ───────────────────────────────────────────────────────
    if (!pkt.frame_ok) {
    }
    else if (opcode == OP_TRAIN) {
        // hist is being rebuilt; invalidate the forwarding cache so the first
        // OP_DETECT after retraining reads fresh log-scores from BRAM.
        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL
            cache_valid[i] = false;
        }
        // Disable inference until the new config is latched via DUMP→CALIB pump.
        // Without this, stale thresholds/weights from the previous run would
        // continue serving OP_DETECT packets during the retrain period.
        inference_enabled = false;
    }
    else if (opcode == OP_CALIB) {
        calib_phase_seen = true;
    }
    else if (opcode == OP_DUMP && calib_phase_seen && !dump_ack_pending && config_dump_state == 0) {
        dump_ack_pending = true;
    }
    else if (opcode == OP_DETECT && inference_enabled && !do_write) {
        total_score_t total = 0;

        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL
            bin_addr_t   addr   = pkt.addr[i];
            delta_addr_t d_addr = pkt.d_addr[i];

            hbos_score_t base_score;
            if (cache_valid[i] && addr == last_hist_addr[i]) {
                base_score = last_hist_score[i];   // register forward — no BRAM access
            } else {
                base_score = hist[i][addr];         // BRAM read (1cc latency)
            }
            // Update cache with the raw hist value (before spike penalty so the
            // cached value is reusable regardless of the next packet's d_addr).
            last_hist_addr[i]  = addr;
            last_hist_score[i] = base_score;
            cache_valid[i]     = true;

            if (d_addr > delta_th[i]) {
                base_score += (hbos_score_t)spike_penalty;
            }
            total += (total_score_t)((base_score * sensor_weights[i]) >> 8);
        }

        bool is_anomaly = (total >= global_threshold);
        do_write   = true;
        write_data = is_anomaly ? (ap_uint<8>)0x01 : (ap_uint<8>)0x00;
    }

    // ── single output write ───────────────────────────────────────────────────
    if (do_write) {
        anomaly_packet_t out_pkt;
        out_pkt.data = write_data;
        out_pkt.keep = -1;
        out_pkt.strb = -1;
        out_pkt.last = 1;
        anomaly_out.write(out_pkt);
    }
}
