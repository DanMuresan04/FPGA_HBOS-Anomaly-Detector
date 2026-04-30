#include "hbos_types.h"

count_t hist[NR_SENSORS][NR_BINS];
count_t d_hist[NR_SENSORS][NR_DELTA_BINS];
hbos_score_t lut[NR_SENSORS][NR_BINS];
sensor_config_t config[NR_SENSORS];

void histogram_builder(sensor_t data[NR_SENSORS]) {
    #pragma HLS ARRAY_PARTITION variable=hist complete dim=1
    #pragma HLS ARRAY_PARTITION variable=d_hist complete dim=1
    #pragma HLS ARRAY_PARTITION variable=config complete dim=1

    static bool sensor_init[NR_SENSORS] = {false, false, false, false};
    #pragma HLS ARRAY_PARTITION variable=sensor_init complete

    SENSOR_LOOP: for (int i = 0; i < NR_SENSORS; i++) {
        #pragma HLS UNROLL

        sensor_t v = data[i];

        if (!sensor_init[i]) {
            config[i].min_v = v;
            config[i].shamt = 0;
            config[i].d_shamt = 0;
            config[i].prev_val = v;
            sensor_init[i] = true;
        }

        while (v < config[i].min_v || v >= config[i].min_v + (((sensor_t)NR_BINS) << config[i].shamt)) {
            #pragma HLS LOOP_TRIPCOUNT min=0 max=3
            if (config[i].shamt >= 31) break;

            if (v < config[i].min_v) {
                config[i].min_v -= ((sensor_t)(NR_BINS / 2)) << config[i].shamt;
                for (int b = (NR_BINS / 2) - 1; b >= 0; b--)
                    hist[i][b + (NR_BINS / 2)] = hist[i][2*b] + hist[i][2*b+1];
                for (int b = 0; b < (NR_BINS / 2); b++)
                    hist[i][b] = 0;
            } else {
                for (int b = 0; b < (NR_BINS / 2); b++)
                    hist[i][b] = hist[i][2*b] + hist[i][2*b+1];
                for (int b = (NR_BINS / 2); b < NR_BINS; b++)
                    hist[i][b] = 0;
            }
            config[i].shamt++;
        }

        hist[i][(v - config[i].min_v) >> config[i].shamt]++;

        sensor_t diff = (v > config[i].prev_val) ? (sensor_t)(v - config[i].prev_val) : (sensor_t)(config[i].prev_val - v);

        while (diff >= (((sensor_t)NR_DELTA_BINS) << config[i].d_shamt)) {
            #pragma HLS LOOP_TRIPCOUNT min=0 max=3
            if (config[i].d_shamt >= 31) break;
            for (int b = 0; b < (NR_DELTA_BINS / 2); b++)
                d_hist[i][b] = d_hist[i][2*b] + d_hist[i][2*b+1];
            for (int b = (NR_DELTA_BINS / 2); b < NR_DELTA_BINS; b++)
                d_hist[i][b] = 0;
            config[i].d_shamt++;
        }

        d_hist[i][diff >> config[i].d_shamt]++;
        config[i].prev_val = v;
    }
}

void hbos_top(hls::stream<sensor_packet_t>& in_stream, hls::stream<bool>& anomaly_out) {
    #pragma HLS INTERFACE axis port=in_stream
    #pragma HLS INTERFACE axis port=anomaly_out
    #pragma HLS INTERFACE ap_ctrl_none port=return

    sensor_packet_t pkt = in_stream.read();

    if (pkt.opcode == OP_TRAIN) {
        histogram_builder(pkt.data);
    }

    anomaly_out.write(false);
}