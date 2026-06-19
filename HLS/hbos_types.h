#ifndef HBOS_TYPES_H
#define HBOS_TYPES_H

#include <ap_fixed.h>
#include <ap_int.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>

#define NR_BINS 2048
#define NR_DELTA_BINS 256
#define NR_SENSORS 4

#define H_EXP_BITS 5
#define H_MANT_BITS 5
#define D_EXP_BITS 5
#define D_MANT_BITS 3

typedef ap_int<32>  sensor_t;
typedef ap_uint<16> hbos_score_t;
typedef ap_uint<26> total_score_t;
typedef ap_uint<16> count_t;
typedef ap_uint<11> bin_addr_t;
typedef ap_uint<8>  delta_addr_t;
typedef ap_uint<3>  opcode_t;
typedef ap_uint<8>  weight_t;
typedef ap_uint<16> spike_t;

#define OP_TRAIN  0
#define OP_CALIB  1
#define OP_DETECT 2
#define OP_DUMP   3
#define OP_CONFIG 4
// Explicit full-state flush. Sent by the host before (re)training so every
// engine zeroes its accumulators/caches; without it a second TRAIN accumulates
// onto the previous run's converted score histogram. opcode_t is 3 bits (0-7).
#define OP_RESET  5

#define FRAME_MAGIC_LO 0xA5
#define FRAME_MAGIC_HI 0x5A

struct sensor_config_t {
    sensor_t    center;
    delta_addr_t delta_th;
    sensor_t    prev_val;
};

struct sensor_packet_t {
    sensor_t    data[NR_SENSORS];
    opcode_t    opcode;
    bool        tlast;
    ap_uint<16> reserve;
    bool        frame_ok;
};

typedef ap_axiu<8, 0, 0, 0> anomaly_packet_t;
typedef ap_axiu<8, 0, 0, 0> rx_byte_axis_t;

struct addr_packet_t {
    bin_addr_t   addr[NR_SENSORS];
    delta_addr_t d_addr[NR_SENSORS];
    opcode_t     opcode;
    bool         tlast;
    bool         frame_ok;
};

void address_engine(
    hls::stream<sensor_packet_t>& in_stream,
    hls::stream<addr_packet_t>&   out_stream
);

void packet_assembler(
    hls::stream<rx_byte_axis_t>& rx_in,
    hls::stream<sensor_packet_t>& packet_out
);

void detection_engine(
    hls::stream<addr_packet_t>&    in_stream,
    hls::stream<ap_uint<32>>&      config_in,
    count_t hist[NR_SENSORS][NR_BINS],
    hls::stream<anomaly_packet_t>& anomaly_out
);

#endif
