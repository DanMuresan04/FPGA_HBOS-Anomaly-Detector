#ifndef HBOS_TOP_H
#define HBOS_TOP_H

#include "hbos_types.h"
extern count_t hist[NR_SENSORS][NR_BINS];
extern sensor_config_t config[NR_SENSORS];
void hbos_top(hls::stream<sensor_packet_t>& in_stream, hls::stream<bool>& anomaly_out);

#endif
