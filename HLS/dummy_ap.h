#include <stdint.h>
template<int W> struct ap_uint {
    uint32_t val;
    ap_uint() : val(0) {}
    ap_uint(uint32_t v) : val(v & ((1ULL<<W)-1)) {}
    operator uint32_t() const { return val; }
};
template<int W> struct ap_int {
    int32_t val;
    ap_int() : val(0) {}
    ap_int(int32_t v) : val((v << (32-W)) >> (32-W)) {}
    operator int32_t() const { return val; }
};
template<int W, int I, int Q, int O, int N> struct ap_axiu {
    ap_uint<W> data;
    ap_uint<1> last;
    ap_uint<1> keep;
    ap_uint<1> strb;
};
#define ap_ctrl_none
#define ap_fixed
