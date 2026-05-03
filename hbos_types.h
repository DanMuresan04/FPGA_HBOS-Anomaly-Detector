#ifndef HBOS_TYPES_H
#define HBOS_TYPES_H

#include <ap_fixed.h>
#include <ap_int.h>
#include <hls_stream.h>

#define NR_BINS 2048
#define NR_DELTA_BINS 256
#define NR_SENSORS 4

typedef ap_int<32> sensor_t;      
typedef ap_int<32> hbos_score_t;  
typedef ap_int<64> total_score_t;
typedef ap_uint<32> count_t;
typedef ap_uint<11> bin_addr_t;
typedef ap_uint<8> delta_addr_t;
typedef ap_uint<2> opcode_t;

#define OP_TRAIN 0
#define OP_CALIB 1
#define OP_DETECT 2

struct sensor_config_t {
    sensor_t min_v;
    ap_uint<8> shamt;
    ap_uint<8> d_shamt;
    sensor_t delta_th;
    sensor_t prev_val;
};

struct sensor_packet_t {
    sensor_t data[NR_SENSORS];
    opcode_t opcode;
    bool tlast;
    ap_uint<16> reserve;
};

#endif
