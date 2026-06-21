#ifndef HBOS_MATH_H
#define HBOS_MATH_H

// ============================================================================
// Fixed-point log helpers for HBOS binning.
//
// Idea: a value's "rarity" in HBOS is driven by counts on a log scale, and the
// histogram should be dense near a sensor's centre and coarsen geometrically
// outward. Both are served by cheap integer log approximations (no DSP/float):
// the bin address is essentially a floating-point encoding {sign, exponent,
// mantissa} of the distance from the centre.
// ============================================================================

#include "hbos_types.h"

// 8.8 fixed-point fractional part of log2, indexed by the top 4 mantissa bits.
static const ap_uint<10> log2_frac_lut[16] = {
    0, 22, 43, 63, 82, 101, 119, 136, 153, 170, 186, 202, 217, 232, 246, 260
};

// Approximate log2(x) in 8.8 fixed-point (integer part << 8 | fractional LUT).
// Returns 0 for x == 0. Used to turn raw bin counts into log-rarity scores.
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

// Value-histogram bin for `v` relative to the sensor's learned `center`.
// Encodes |v - center| as {sign, exponent(H_EXP_BITS), mantissa(H_MANT_BITS)}
// so bins are fine near the centre and grow geometrically with distance.
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

// Delta-histogram bin for an unsigned magnitude `diff` (|sample - previous|).
// Same exponent/mantissa encoding as above but unsigned (D_EXP/D_MANT bits);
// a delta landing above the learned delta_th[] adds the spike penalty.
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
