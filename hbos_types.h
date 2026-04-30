#ifndef HBOS_TYPES
#define HBOS_TYPES

#include <ap_fixed.h>
#include <ap_int.h>
#include <hls_math.h>

#define NR_BINS 2048
#define F_BITS 8
#define NR_SENSORS 4
#define Q_SCALE (1 << F_BITS)

typedef ap_fixed<10, 8> sensor_t;
typedef ap_fixed<10, 8> hbos_score_t;
typedef ap_uint<32> count_t;
typedef ap_uint<11> bin_addr_t;

struct sensor_config_t {
    sensor_t min_v;
    ap_uint<4> shamt; 
    hbos_score_t delta_th; 
};

#endif