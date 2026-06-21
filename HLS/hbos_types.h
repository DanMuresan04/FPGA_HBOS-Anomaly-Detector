#ifndef HBOS_TYPES_H
#define HBOS_TYPES_H

// ============================================================================
// HBOS streaming anomaly detector — shared types, constants and module API.
//
// Pipeline (merged design):
//   UART/PCIe bytes -> packet_assembler -> address_engine -> hbos_engine -> verdict
//
//   * packet_assembler  gathers one frame's N int32 sensor values and emits a
//                       single wide sensor_packet_t (the parallel-ingress point;
//                       swapping UART for PCIe only changes how bytes arrive).
//   * address_engine    maps each sensor value to a log-linear histogram bin
//                       address and a delta (vs the previous sample) address.
//   * hbos_engine       trains/calibrates/detects against per-sensor histograms;
//                       the anomaly score is a weighted sum of per-bin rarities.
//
// HBOS score for one sample = sum over active sensors of
//     (hist[s][addr] (+ spike_penalty if delta is rare)) * weight[s] >> 8
// A sample is anomalous when that score >= the calibrated global_threshold.
// ============================================================================

#include <ap_fixed.h>
#include <ap_int.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>

#define NR_BINS       2048   // value-histogram bins per sensor (log-linear)
#define NR_DELTA_BINS 256    // delta-histogram bins per sensor
#define NR_SENSORS    16     // fixed, uniform HW capacity (max channels)

// ---------------------------------------------------------------------------
// K_PARALLEL — parallelism knob: number of sensors processed per cycle.
// Per-sensor loops use `#pragma HLS UNROLL factor=K_PARALLEL` and per-sensor
// memories `... ARRAY_PARTITION cyclic factor=K_PARALLEL`, so a single source
// scales between two endpoints with no code change:
//   K_PARALLEL = 1            -> one shared datapath, single-port histogram (TDM)
//   K_PARALLEL = NR_SENSORS   -> fully parallel: N address units + N hist banks
// Must divide NR_SENSORS. Override per build with -DK_PARALLEL=<n>.
// ---------------------------------------------------------------------------
#ifndef K_PARALLEL
#define K_PARALLEL 1
#endif

// Log-linear address format: exponent + mantissa bit-widths for value bins (H)
// and delta bins (D). An address packs {sign?, exponent, mantissa} so that
// bins are dense near the per-sensor centre and coarsen geometrically outward.
#define H_EXP_BITS  5
#define H_MANT_BITS 5
#define D_EXP_BITS  5
#define D_MANT_BITS 3

typedef ap_int<32>  sensor_t;       // raw signed sensor value
typedef ap_uint<16> hbos_score_t;   // per-sensor (pre-weight) bin rarity score
typedef ap_uint<26> total_score_t;  // accumulated weighted sample score
typedef ap_uint<16> count_t;        // histogram bin counter
typedef ap_uint<11> bin_addr_t;     // value-histogram bin index (0..NR_BINS-1)
typedef ap_uint<8>  delta_addr_t;   // delta-histogram bin index
typedef ap_uint<3>  opcode_t;       // frame opcode (see OP_* below)
typedef ap_uint<8>  weight_t;       // per-sensor importance weight
typedef ap_uint<16> spike_t;        // penalty added when a delta is rare

// Frame opcodes (host -> engine). One opcode per frame.
#define OP_TRAIN  0   // accumulate clean samples into the value/delta histograms
#define OP_CALIB  1   // score clean samples to build the threshold distribution
#define OP_DETECT 2   // score a sample and emit a 0/1 anomaly verdict
#define OP_DUMP   3   // finalize threshold; readback telemetry one byte per poll
#define OP_CONFIG 4   // latch per-sensor weights + spike penalty
#define OP_RESET  5   // full-state flush: zero all histograms/accumulators/caches
                      // (host sends this before any re-training so a new run never
                      //  accumulates onto the previous run's state)

// Frame validity markers (last two bytes of every frame).
#define FRAME_MAGIC_LO 0xA5
#define FRAME_MAGIC_HI 0x5A

// Per-sensor learned configuration (centre value + delta threshold + history).
struct sensor_config_t {
    sensor_t     center;
    delta_addr_t delta_th;
    sensor_t     prev_val;
};

// Wide sample packet emitted by packet_assembler.
//   data[]        one int32 per channel; only [0..active_count-1] are live.
//   active_count  number of live sensors this frame (1..NR_SENSORS); downstream
//                 engines gate every per-sensor loop on i < active_count, so
//                 unused channels never affect the score or the threshold.
//   tlast         marks the sample's CSV label (used to gate clean training).
//   frame_ok      magic-bytes check passed.
struct sensor_packet_t {
    sensor_t    data[NR_SENSORS];
    opcode_t    opcode;
    bool        tlast;
    ap_uint<5>  active_count;
    ap_uint<16> reserve;
    bool        frame_ok;
};

// 8-bit AXI-Stream beats: anomaly verdict / telemetry out, raw UART bytes in.
typedef ap_axiu<8, 0, 0, 0> anomaly_packet_t;
typedef ap_axiu<8, 0, 0, 0> rx_byte_axis_t;

// Address packet emitted by address_engine (carries bin addresses, not raw values).
//   addr[s]   value-histogram bin for sensor s
//   d_addr[s] delta-histogram bin for sensor s
//   (OP_CONFIG reuses these fields to carry packed weights + spike penalty.)
struct addr_packet_t {
    bin_addr_t   addr[NR_SENSORS];
    delta_addr_t d_addr[NR_SENSORS];
    opcode_t     opcode;
    bool         tlast;
    ap_uint<5>   active_count;
    bool         frame_ok;
};

// ---------------------------------------------------------------------------
// Module API.
// ---------------------------------------------------------------------------

// Reassemble a count-prefixed UART/byte frame into one wide sensor_packet_t.
// Wire frame: [n_words][active_count][opcode][tlast][ n_words LE int32 ][A5][5A].
// Waits for all n_words ints, then bursts the wide packet (parallel-ingress PoC).
void packet_assembler(
    hls::stream<rx_byte_axis_t>&  rx_in,
    hls::stream<sensor_packet_t>& packet_out
);

// Map each live sensor value to its value-bin and delta-bin address.
// Tracks per-sensor centre/previous-value state; gates work on active_count.
void address_engine(
    hls::stream<sensor_packet_t>& in_stream,
    hls::stream<addr_packet_t>&   out_stream
);

// Legacy split-design detector (paired with hbos_top via an external shared
// histogram BRAM + config FIFO). Superseded by hbos_engine (see hbos_engine.h).
void detection_engine(
    hls::stream<addr_packet_t>&    in_stream,
    hls::stream<ap_uint<32>>&      config_in,
    count_t hist[NR_SENSORS][NR_BINS],
    hls::stream<anomaly_packet_t>& anomaly_out
);

#endif
