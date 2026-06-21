#ifndef HBOS_ENGINE_H
#define HBOS_ENGINE_H

#include "hbos_types.h"
#include <hls_stream.h>

// ============================================================================
// hbos_engine — merged single-kernel HBOS detector.
//
// Idea: fuse train + calibrate + detect into ONE opcode-dispatched top-level
// function that owns the value/delta/score histograms as INTERNAL on-chip
// memory. This removes the legacy split design's external shared-BRAM plumbing
// (bram_quad_mux / bram_addr_shift / hist_sel_train) and the hbos_top->
// detection_engine config FIFO — so the block design is a flat AXIS pipeline
// with no memories on the canvas, and the same kernel reads internal memory
// directly (no cross-IP read path), which also removes that class of mismatch.
//
// One frame is processed per invocation (ap_ctrl_none). Behaviour by opcode:
//   OP_RESET   zero all histograms/accumulators/caches.
//   OP_CONFIG  latch per-sensor weights + spike penalty.
//   OP_TRAIN   add clean samples to the value/delta histograms.
//   OP_CALIB   on first calib, convert histograms to log-rarity scores; then
//              score clean samples to build the threshold distribution.
//   OP_DUMP    first poll finalizes the global threshold and ACKs (0xFF); each
//              later poll returns one telemetry byte (0xFE | thr[23:0] | 0xFF).
//   OP_DETECT  score the sample; emit 0x01 if score >= threshold else 0x00.
//
// Scaling: per-sensor work is gated on active_count (runtime 1..NR_SENSORS) and
// parallelised by K_PARALLEL (see hbos_types.h) — fixed-uniform HW, runtime
// sensor selection, no resynthesis to change how many sensors are live.
//
// Params:
//   in_stream    addr_packet_t stream from address_engine (bin addresses +
//                opcode + active_count); OP_CONFIG packs weights/spike here.
//   anomaly_out  8-bit AXIS: detect verdicts, the DUMP ACK, and telemetry bytes.
// ============================================================================
void hbos_engine(
    hls::stream<addr_packet_t>&    in_stream,
    hls::stream<anomaly_packet_t>& anomaly_out
);

#endif
