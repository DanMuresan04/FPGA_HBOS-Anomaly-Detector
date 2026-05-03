#include "hbos_top.h"

count_t hist[NR_SENSORS][NR_BINS];
count_t d_hist[NR_SENSORS][NR_DELTA_BINS];
hbos_score_t score_lut[NR_SENSORS][NR_BINS];
sensor_config_t config[NR_SENSORS];
count_t score_hist[2048];
sensor_t history[NR_SENSORS][5];

static const ap_uint<10> log2_frac_lut[16] = {
    0, 22, 43, 63, 82, 101, 119, 136, 153, 170, 186, 202, 217, 232, 246, 260
};

static const ap_uint<8> weights[NR_SENSORS] = {50, 93, 58, 55};
#define SPIKE_PENALTY 5632 

ap_uint<16> aprox_log2(ap_uint<32> x) {
    #pragma HLS INLINE
    if (x == 0) {
        return 0;
    }
    unsigned int lz = __builtin_clz((unsigned int)x);
    ap_uint<8> msb = 31 - lz;
    ap_uint<4> frac_bits;
    if (msb >= 4) {
        frac_bits = (x >> (msb - 4)) & 0xF;
    } else {
        frac_bits = (x << (4 - msb)) & 0xF;
    }
    return ((ap_uint<16>)msb << 8) + log2_frac_lut[frac_bits];
}

void convert_hist_to_score(count_t train_count) {
    ap_uint<16> log2_denom = aprox_log2(train_count + 2048);
    for (int i = 0; i < NR_SENSORS; i++) {
        for (int j = 0; j < 2048; j++) {
            #pragma HLS PIPELINE II=1
            ap_uint<16> log2_num = aprox_log2(hist[i][j] + 1);
            if (log2_denom > log2_num) {
                score_lut[i][j] = (hbos_score_t)(log2_denom - log2_num);
            } else {
                score_lut[i][j] = (hbos_score_t)0;
            }
        }
        count_t target = train_count - (train_count >> 10);
        count_t cumulative = 0;
        ap_uint<8> delta_bin = 255;
        for (int d = 0; d < 256; d++) {
            #pragma HLS PIPELINE II=1
            cumulative += d_hist[i][d];
            if (cumulative >= target) {
                if (delta_bin == 255) {
                    delta_bin = d;
                }
            }
        }
        config[i].delta_th = ((sensor_t)delta_bin) << config[i].d_shamt;
    }
}

void finalize_global_threshold(count_t calib_count, total_score_t &threshold) {
    count_t target = calib_count - (calib_count >> 9);
    count_t cumulative = 0;
    ap_uint<11> threshold_bin = 2047;
    for (int i = 0; i < 2048; i++) {
        #pragma HLS PIPELINE II=1
        cumulative += score_hist[i];
        if (cumulative >= target) {
            if (threshold_bin == 2047) {
                threshold_bin = i;
            }
        }
    }
    threshold = (total_score_t)threshold_bin << 4;
}

total_score_t engine_score(sensor_t data[NR_SENSORS]) {
    #pragma HLS INLINE
    total_score_t total = 0;
    SENSOR_SCORE_LOOP: for (int i = 0; i < NR_SENSORS; i++) {
        #pragma HLS UNROLL
        sensor_t v = data[i];
        
        sensor_t addr_raw = (v - config[i].min_v) >> config[i].shamt;
        bin_addr_t addr;
        if (addr_raw < 0) {
            addr = 0;
        } else if (addr_raw >= NR_BINS) {
            addr = NR_BINS - 1;
        } else {
            addr = (bin_addr_t)addr_raw;
        }

        hbos_score_t final_sensor_score = score_lut[i][addr];

        sensor_t old_v = history[i][0];
        sensor_t diff;
        if (v > old_v) {
            diff = (sensor_t)(v - old_v);
        } else {
            diff = (sensor_t)(old_v - v);
        }
        
        if (diff > config[i].delta_th) {
            final_sensor_score += SPIKE_PENALTY;
        }

        total += (total_score_t)((final_sensor_score * weights[i]) >> 8);
        
        for (int h = 0; h < 4; h++) {
            history[i][h] = history[i][h+1];
        }
        history[i][4] = v;
    }
    return total;
}

void histogram_builder(sensor_t data[NR_SENSORS], bool is_clean, int index) {
    #pragma HLS INLINE
    static bool sensor_init[NR_SENSORS] = {false, false, false, false};
    #pragma HLS ARRAY_PARTITION variable=sensor_init complete

    SENSOR_LOOP: for (int i = 0; i < NR_SENSORS; i++) {
        #pragma HLS UNROLL
        sensor_t v = data[i];
        if (sensor_init[i] == false) {
            config[i].min_v = v;
            config[i].shamt = 0;
            config[i].d_shamt = 0;
            for (int h = 0; h < 5; h++) {
                history[i][h] = v;
            }
            sensor_init[i] = true;
        }

        if (is_clean == true) {
            if (index % 5 == 0) {
                while (v < config[i].min_v || v >= config[i].min_v + (((sensor_t)NR_BINS) << config[i].shamt)) {
                    #pragma HLS LOOP_TRIPCOUNT min=0 max=3
                    if (v < config[i].min_v) {
                        config[i].min_v -= ((sensor_t)(NR_BINS / 2)) << config[i].shamt;
                        for (int b = (NR_BINS / 2) - 1; b >= 0; b--) {
                            hist[i][b + (NR_BINS / 2)] = hist[i][2 * b] + hist[i][2 * b + 1];
                        }
                        for (int b = 0; b < (NR_BINS / 2); b++) {
                            hist[i][b] = 0;
                        }
                    } else {
                        for (int b = 0; b < (NR_BINS / 2); b++) {
                            hist[i][b] = hist[i][2 * b] + hist[i][2 * b + 1];
                        }
                        for (int b = (NR_BINS / 2); b < NR_BINS; b++) {
                            hist[i][b] = 0;
                        }
                    }
                    config[i].shamt++;
                }
                hist[i][(v - config[i].min_v) >> config[i].shamt]++;

                sensor_t old_v = history[i][0];
                sensor_t diff;
                if (v > old_v) {
                    diff = (sensor_t)(v - old_v);
                } else {
                    diff = (sensor_t)(old_v - v);
                }
                while (diff >= (((sensor_t)NR_DELTA_BINS) << config[i].d_shamt)) {
                    #pragma HLS LOOP_TRIPCOUNT min=0 max=3
                    for (int b = 0; b < (NR_DELTA_BINS / 2); b++) {
                        d_hist[i][b] = d_hist[i][2 * b] + d_hist[i][2 * b + 1];
                    }
                    for (int b = (NR_DELTA_BINS / 2); b < NR_DELTA_BINS; b++) {
                        d_hist[i][b] = 0;
                    }
                    config[i].d_shamt++;
                }
                d_hist[i][diff >> config[i].d_shamt]++;
            }
        }
        
        for (int h = 0; h < 4; h++) {
            history[i][h] = history[i][h + 1];
        }
        history[i][4] = v;
    }
}

void hbos_top(hls::stream<sensor_packet_t>& in_stream, hls::stream<bool>& anomaly_out) {
    #pragma HLS INTERFACE axis port=in_stream
    #pragma HLS INTERFACE axis port=anomaly_out
    #pragma HLS INTERFACE ap_ctrl_none port=return

    static count_t train_count = 0;
    static count_t calib_count = 0;
    static total_score_t global_threshold = 32767;
    static opcode_t last_opcode = OP_TRAIN;
    static count_t sample_index = 0;

    sensor_packet_t pkt = in_stream.read();
    opcode_t opcode = pkt.opcode;
    bool is_clean;
    if (pkt.tlast == 0) {
        is_clean = true;
    } else {
        is_clean = false;
    }
    
    if (opcode == OP_CALIB) {
        if (last_opcode == OP_TRAIN) {
            convert_hist_to_score(train_count);
        }
    }
    if (opcode == OP_DETECT) {
        if (last_opcode == OP_CALIB) {
            finalize_global_threshold(calib_count, global_threshold);
        }
    }
    last_opcode = opcode;

    if (opcode == OP_TRAIN) {
        if (is_clean == true) {
            if (sample_index % 5 == 0) {
                train_count++;
            }
        }
        histogram_builder(pkt.data, is_clean, sample_index);
        anomaly_out.write(false);
    } else if (opcode == OP_CALIB) {
        total_score_t score = engine_score(pkt.data);
        if (is_clean == true) {
            if (sample_index % 5 == 1) {
                calib_count++;
                score_hist[(bin_addr_t)(score >> 4)]++;
            }
        }
        anomaly_out.write(false);
    } else {
        total_score_t score = engine_score(pkt.data);
        if (score >= global_threshold) {
            anomaly_out.write(true);
        } else {
            anomaly_out.write(false);
        }
    }

    sample_index++;
    if (opcode == OP_DETECT) {
        if (last_opcode == OP_CALIB) {
            sample_index = 0;
        }
    }
}