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
    // Minimal host readback: 0xFE | global_threshold[23:0] LE | 0xFF.
    // Filled once when config latches; the dump "FSM" just walks this LUT, so
    // config_dump_state is a plain counter (no wide comparison ladder).
    static ap_uint<8>    telem[5];
    #pragma HLS ARRAY_PARTITION variable=telem      complete
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

    // Full-throughput pipeline: one verdict per cycle.
    // II=1 is recovered by decoupling the control-plane recurrences (see the
    // cds_snap/dap_snap snapshots below): the dump readback and the dump-ack
    // dispatch read the REGISTERED config_dump_state/dump_ack_pending instead of
    // each other's same-cycle updates, so the config_dump_state -> dispatch ->
    // dump_ack_pending chain (the ~12.5 ns recurrence that capped II=1 Fmax) is
    // broken into short, independent recurrences.
    // hist is read-only here so there is no inter-invocation RAW hazard on the BRAM.
    // The forwarding cache arrays have genuine RAW dependencies at distance=1.
    #pragma HLS PIPELINE II=1
    #pragma HLS dependence variable=hist            type=inter direction=RAW dependent=false
    #pragma HLS dependence variable=last_hist_addr  type=inter direction=RAW dependent=true distance=1
    #pragma HLS dependence variable=last_hist_score type=inter direction=RAW dependent=true distance=1
    #pragma HLS dependence variable=cache_valid     type=inter direction=RAW dependent=true distance=1

    addr_packet_t pkt = in_stream.read();
    opcode_t opcode = pkt.opcode;

    // Snapshot the control-FSM state at the start of the iteration. The readback
    // and the dump-ack dispatch both key off config_dump_state; reading these
    // registered snapshots (instead of values another branch may rewrite this
    // same cycle) keeps each control recurrence short and independent, which is
    // what lets the function close timing at II=1. The dump readback (cds>0) and
    // the dump-ack branch (cds==0) are mutually exclusive, so using the snapshot
    // is behaviourally identical for every real train/detect flow.
    ap_uint<5> cds_snap = config_dump_state;
    bool       dap_snap = dump_ack_pending;

    // Each output source decides INDEPENDENTLY whether it wants to drive the
    // anomaly_out byte this cycle; a parallel priority mux at the bottom picks
    // one (latch ack > threshold readback > detect verdict) and issues the
    // single write. This replaces the old serial `do_write` thread that chained
    // config-latch -> readback -> detect and fed the inference_enabled
    // recurrence — that chain was the II=1 critical path.
    bool       latch_wr  = false;                 // config-latched 0xFF ack
    bool       dump_wr   = false; ap_uint<8> dump_data = 0;   // threshold byte
    bool       det_wr    = false; ap_uint<8> det_data  = 0;   // verdict 0x00/0x01

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
                // Pre-pack the threshold readback frame (output off the
                // config_dump_state recurrence path).
                telem[0] = 0xFE;
                telem[1] = (ap_uint<8>)(global_threshold & 0xFF);
                telem[2] = (ap_uint<8>)((global_threshold >> 8) & 0xFF);
                telem[3] = (ap_uint<8>)((global_threshold >> 16) & 0xFF);
                telem[4] = 0xFF;
                if (cds_snap == 0) {
                    config_dump_state = 1;
                }
                // Immediate config-latched ack — highest output priority.
                latch_wr = true;
            } else {
                cfg_rx_cnt++;
            }
        }
    }

    // ── threshold readback ───────────────────────────────────────────────────
    // Emits the 5-byte frame 0xFE | global_threshold[23:0] LE | 0xFF, one byte
    // per OP_DUMP poll. config_dump_state is now just a 1..5 counter walking the
    // pre-packed telem[] LUT, so its loop-carried recurrence is a short
    // increment/wrap instead of the old 16-way comparison ladder.
    if (cds_snap > 0 && opcode == OP_DUMP) {
        dump_wr   = true;
        dump_data = telem[cds_snap - 1];
        config_dump_state = (cds_snap >= 5)
                          ? (ap_uint<5>)0
                          : (ap_uint<5>)(cds_snap + 1);
    }

    // ── opcode dispatch ───────────────────────────────────────────────────────
    if (!pkt.frame_ok) {
    }
    else if (opcode == OP_RESET) {
        // Full flush: drop inference, invalidate the forwarding cache, and reset
        // the config-latch state machine so a stale partial frame or phase flag
        // can't survive into the next train/latch cycle.
        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL
            cache_valid[i] = false;
            delta_th[i]    = 0;
        }
        inference_enabled = false;
        calib_phase_seen  = false;
        dump_ack_pending  = false;
        config_dump_state = 0;
        cfg_rx_cnt        = 0;
        global_threshold  = 0;
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
    else if (opcode == OP_DUMP && calib_phase_seen && !dap_snap && cds_snap == 0) {
        dump_ack_pending = true;
    }
    else if (opcode == OP_DETECT && inference_enabled) {
        // Per-sensor weighted scores into an array, then a balanced adder tree.
        // The 16x8 multiply was mapped to LUTs (~5.6 ns) and the whole
        // spike-add -> mul -> accumulate -> compare ran in one ~18 ns
        // combinational stage (Fmax ~56 MHz). Binding the multiply to a
        // pipelined DSP and reducing with a tree lets HLS spread the MAC across
        // pipeline stages: same II=1, higher latency, much shorter critical path.
        total_score_t prod[NR_SENSORS];
        #pragma HLS ARRAY_PARTITION variable=prod complete dim=1

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
            // Pipelined DSP multiply (vs the LUT-mapped combinational mul on the
            // old critical path). latency=2 registers it across stages.
            ap_uint<24> p = base_score * sensor_weights[i];
            #pragma HLS BIND_OP variable=p op=mul impl=dsp latency=2
            prod[i] = (total_score_t)(p >> 8);
        }

        // Balanced adder tree instead of a serial += chain (NR_SENSORS == 4).
        total_score_t total = (prod[0] + prod[1]) + (prod[2] + prod[3]);
        bool is_anomaly = (total >= global_threshold);
        det_wr   = true;
        det_data = is_anomaly ? (ap_uint<8>)0x01 : (ap_uint<8>)0x00;
    }

    // ── single output write (parallel priority mux) ───────────────────────────
    // Sources are mutually exclusive in every real flow (config_in event vs
    // OP_DUMP vs OP_DETECT); the priority order only matters for safety.
    bool       do_write   = latch_wr || dump_wr || det_wr;
    ap_uint<8> write_data = latch_wr ? (ap_uint<8>)0xFF
                          : dump_wr  ? dump_data
                          :            det_data;
    if (do_write) {
        anomaly_packet_t out_pkt;
        out_pkt.data = write_data;
        out_pkt.keep = -1;
        out_pkt.strb = -1;
        out_pkt.last = 1;
        anomaly_out.write(out_pkt);
    }
}
