#ifndef HBOS_TOP_H
#define HBOS_TOP_H

#include "hbos_types.h"

// ============================================================================
// hbos_top — legacy split-design trainer (superseded by hbos_engine).
//
// The training/calibration half of the original two-IP design: it builds the
// value/delta histograms in an EXTERNAL shared BRAM and, at OP_DUMP, finalizes
// the threshold and streams the config words out to detection_engine over a
// FIFO. Kept for reference/regression; new work uses the merged hbos_engine,
// which holds the histograms internally and needs no shared BRAM or FIFO.
//
// Params:
//   in_stream   addr_packet_t stream (bin addresses + opcode).
//   hist        external value-histogram BRAM, shared with detection_engine.
//   config_out  FIFO of 6 config words handed to detection_engine at OP_DUMP
//               (threshold, packed deltas, rx counts, packed weights, spike).
// d_hist/score_hist are file-scope state exposed so OP_RESET can clear them.
// ============================================================================
void hbos_top(hls::stream<addr_packet_t>& in_stream, count_t hist[NR_SENSORS][NR_BINS], hls::stream<ap_uint<32>>& config_out);

extern count_t d_hist[NR_SENSORS][NR_DELTA_BINS];
extern count_t score_hist[2048];

#endif
