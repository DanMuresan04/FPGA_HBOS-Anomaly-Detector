#ifndef HBOS_TOP_H
#define HBOS_TOP_H

#include "hbos_types.h"

void hbos_top(hls::stream<addr_packet_t>& in_stream, count_t hist[NR_SENSORS][NR_BINS], hls::stream<ap_uint<32>>& config_out);

extern count_t d_hist[NR_SENSORS][NR_DELTA_BINS];
extern count_t score_hist[2048];

#endif
