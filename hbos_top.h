#ifndef HBOS_TOP_H
#define HBOS_TOP_H

#include "hbos_types.h"

void hbos_top(hls::stream<sensor_packet_t>& in_stream, hls::stream<bool>& anomaly_out);

extern count_t hist[NR_SENSORS][NR_BINS];
extern count_t d_hist[NR_SENSORS][NR_DELTA_BINS];
extern hbos_score_t score_lut[NR_SENSORS][NR_BINS];
extern sensor_config_t config[NR_SENSORS];
extern count_t score_hist[2048];

#endif
