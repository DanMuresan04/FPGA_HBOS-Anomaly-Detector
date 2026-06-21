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
    #pragma HLS ARRAY_PARTITION variable=hist cyclic factor=K_PARALLEL dim=1

    static total_score_t global_threshold = 0;
    static delta_addr_t  delta_th[NR_SENSORS];
    static ap_uint<32>   total_rx_train = 0;
    static ap_uint<32>   total_rx_calib = 0;
    static bool          inference_enabled  = false;
    static bool          calib_phase_seen   = false;
    static bool          dump_ack_pending   = false;
    static ap_uint<5>    config_dump_state  = 0;

    static ap_uint<3>    cfg_rx_cnt = 0;
    static ap_uint<32>   cfg_buf[6];
    #pragma HLS ARRAY_PARTITION variable=cfg_buf    complete

    static ap_uint<8>    telem[5];
    #pragma HLS ARRAY_PARTITION variable=telem      complete
    #pragma HLS ARRAY_PARTITION variable=delta_th   cyclic factor=K_PARALLEL dim=1

    static weight_t sensor_weights[NR_SENSORS] = {50, 93, 58, 55};
    #pragma HLS ARRAY_PARTITION variable=sensor_weights cyclic factor=K_PARALLEL dim=1
    static spike_t spike_penalty = 5632;

    static bin_addr_t   last_hist_addr[NR_SENSORS];
    static hbos_score_t last_hist_score[NR_SENSORS];
    static bool         cache_valid[NR_SENSORS] = {false, false, false, false};
    #pragma HLS ARRAY_PARTITION variable=last_hist_addr  cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=last_hist_score cyclic factor=K_PARALLEL dim=1
    #pragma HLS ARRAY_PARTITION variable=cache_valid     cyclic factor=K_PARALLEL dim=1

    #pragma HLS PIPELINE II=1
    #pragma HLS dependence variable=hist            type=inter direction=RAW dependent=false
    #pragma HLS dependence variable=last_hist_addr  type=inter direction=RAW dependent=true distance=1
    #pragma HLS dependence variable=last_hist_score type=inter direction=RAW dependent=true distance=1
    #pragma HLS dependence variable=cache_valid     type=inter direction=RAW dependent=true distance=1

    addr_packet_t pkt = in_stream.read();
    opcode_t opcode = pkt.opcode;

    ap_uint<5> cds_snap = config_dump_state;
    bool       dap_snap = dump_ack_pending;

    bool       latch_wr  = false;
    bool       dump_wr   = false; ap_uint<8> dump_data = 0;
    bool       det_wr    = false; ap_uint<8> det_data  = 0;

    {
        ap_uint<32> w;
        if (config_in.read_nb(w)) {
            int idx = (int)cfg_rx_cnt;
            cfg_buf[idx] = w;
            if (cfg_rx_cnt == 5) {
                cfg_rx_cnt = 0;
                global_threshold = (total_score_t)cfg_buf[0];
                ap_uint<32> packed_deltas  = cfg_buf[1];
                ap_uint<32> packed_weights = cfg_buf[4];
                for (int i = 0; i < NR_SENSORS; i++) {
                    #pragma HLS UNROLL factor=K_PARALLEL
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

                telem[0] = 0xFE;
                telem[1] = (ap_uint<8>)(global_threshold & 0xFF);
                telem[2] = (ap_uint<8>)((global_threshold >> 8) & 0xFF);
                telem[3] = (ap_uint<8>)((global_threshold >> 16) & 0xFF);
                telem[4] = 0xFF;
                if (cds_snap == 0) {
                    config_dump_state = 1;
                }

                latch_wr = true;
            } else {
                cfg_rx_cnt++;
            }
        }
    }

    if (cds_snap > 0 && opcode == OP_DUMP) {
        dump_wr   = true;
        dump_data = telem[cds_snap - 1];
        config_dump_state = (cds_snap >= 5)
                          ? (ap_uint<5>)0
                          : (ap_uint<5>)(cds_snap + 1);
    }

    if (!pkt.frame_ok) {
    }
    else if (opcode == OP_RESET) {

        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL factor=K_PARALLEL
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

        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL factor=K_PARALLEL
            cache_valid[i] = false;
        }

        inference_enabled = false;
    }
    else if (opcode == OP_CALIB) {
        calib_phase_seen = true;
    }
    else if (opcode == OP_DUMP && calib_phase_seen && !dap_snap && cds_snap == 0) {
        dump_ack_pending = true;
    }
    else if (opcode == OP_DETECT && inference_enabled) {

        total_score_t prod[NR_SENSORS];
        #pragma HLS ARRAY_PARTITION variable=prod cyclic factor=K_PARALLEL dim=1

        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL factor=K_PARALLEL
            bin_addr_t   addr   = pkt.addr[i];
            delta_addr_t d_addr = pkt.d_addr[i];

            hbos_score_t base_score;
            if (cache_valid[i] && addr == last_hist_addr[i]) {
                base_score = last_hist_score[i];
            } else {
                base_score = hist[i][addr];
            }

            last_hist_addr[i]  = addr;
            last_hist_score[i] = base_score;
            cache_valid[i]     = true;

            if (d_addr > delta_th[i]) {
                base_score += (hbos_score_t)spike_penalty;
            }

            ap_uint<24> p = base_score * sensor_weights[i];
            #pragma HLS BIND_OP variable=p op=mul impl=dsp latency=2
            prod[i] = (total_score_t)(p >> 8);
        }

        total_score_t total = (prod[0] + prod[1]) + (prod[2] + prod[3]);
        bool is_anomaly = (total >= global_threshold);
        det_wr   = true;
        det_data = is_anomaly ? (ap_uint<8>)0x01 : (ap_uint<8>)0x00;
    }

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
