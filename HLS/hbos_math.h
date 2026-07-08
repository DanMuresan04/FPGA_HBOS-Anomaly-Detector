#ifndef HBOS_MATH_H
#define HBOS_MATH_H

#include "hbos_types.h"

static const ap_uint<10> log2_frac_lut[16] = {
    0, 22, 43, 63, 82, 101, 119, 136, 153, 170, 186, 202, 217, 232, 246, 260
};

static inline ap_uint<16> aprox_log2(ap_uint<32> x) {
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

static inline bin_addr_t log_linear_addr(sensor_t v, sensor_t center) {
    #pragma HLS INLINE
    sensor_t diff;
    ap_uint<1> sign;
    if (v >= center) {
        diff = v - center;
        sign = 0;
    } else {
        diff = center - v;
        sign = 1;
    }
    if (diff == 0) {
        diff = 1;
    }
    unsigned int lz = __builtin_clz((unsigned int)(ap_uint<32>)diff);
    ap_uint<5> msb = 31 - lz;
    ap_uint<H_EXP_BITS> exp;
    if (msb > ((1 << H_EXP_BITS) - 1)) {
        exp = (1 << H_EXP_BITS) - 1;
    } else {
        exp = msb;
    }
    ap_uint<H_MANT_BITS> mantissa;
    if (msb >= H_MANT_BITS) {
        mantissa = (diff >> (msb - H_MANT_BITS)) & ((1 << H_MANT_BITS) - 1);
    } else {
        mantissa = (diff << (H_MANT_BITS - msb)) & ((1 << H_MANT_BITS) - 1);
    }
    bin_addr_t addr = ((bin_addr_t)sign << (H_EXP_BITS + H_MANT_BITS)) |
                      ((bin_addr_t)exp  << H_MANT_BITS) |
                      (bin_addr_t)mantissa;
    return addr;
}

static inline delta_addr_t delta_log_linear_addr(ap_uint<32> diff) {
    #pragma HLS INLINE
    if (diff == 0) {
        diff = 1;
    }
    unsigned int lz = __builtin_clz((unsigned int)diff);
    ap_uint<5> msb = 31 - lz;
    ap_uint<D_EXP_BITS> exp;
    if (msb > ((1 << D_EXP_BITS) - 1)) {
        exp = (1 << D_EXP_BITS) - 1;
    } else {
        exp = msb;
    }
    ap_uint<D_MANT_BITS> mantissa;
    if (msb >= D_MANT_BITS) {
        mantissa = (diff >> (msb - D_MANT_BITS)) & ((1 << D_MANT_BITS) - 1);
    } else {
        mantissa = (diff << (D_MANT_BITS - msb)) & ((1 << D_MANT_BITS) - 1);
    }
    delta_addr_t addr = ((delta_addr_t)exp << D_MANT_BITS) | (delta_addr_t)mantissa;
    return addr;
}

#endif
