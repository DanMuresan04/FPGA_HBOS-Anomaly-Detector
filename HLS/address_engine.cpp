#include "hbos_types.h"
#include "hbos_math.h"
#include <hls_stream.h>

struct ae_mid_t {
    addr_ctrl_t  ctrl;
    bool         raw;
    bool         active[NR_SENSORS];
    sensor_t     v[NR_SENSORS];
    sensor_t     center[NR_SENSORS];
    sensor_t     ref[NR_SENSORS];
    bin_addr_t   raw_addr[NR_SENSORS];
    delta_addr_t raw_d_addr[NR_SENSORS];
};

static void stage_select(
    hls::stream<sensor_packet_t>& in_stream,
    hls::stream<ae_mid_t>&        mid
) {

    #pragma HLS PIPELINE II=1

    static sensor_t center[NR_SENSORS];
    #pragma HLS ARRAY_PARTITION variable=center complete dim=1

    static sensor_t val_hist[NR_SENSORS][DELTA_MAX_STRIDE];
    #pragma HLS ARRAY_PARTITION variable=val_hist complete dim=2
    #pragma HLS ARRAY_PARTITION variable=val_hist complete dim=1

    static bool initialized[NR_SENSORS];
    #pragma HLS ARRAY_PARTITION variable=initialized complete dim=1

    static ap_uint<4> delta_stride = 1;

    static opcode_t last_opcode_ae = OP_TRAIN;

    sensor_packet_t pkt = in_stream.read();

    ae_mid_t m;

    #pragma HLS ARRAY_PARTITION variable=pkt.data     complete dim=1
    #pragma HLS ARRAY_PARTITION variable=m.active      complete dim=1
    #pragma HLS ARRAY_PARTITION variable=m.v           complete dim=1
    #pragma HLS ARRAY_PARTITION variable=m.center      complete dim=1
    #pragma HLS ARRAY_PARTITION variable=m.ref         complete dim=1
    #pragma HLS ARRAY_PARTITION variable=m.raw_addr    complete dim=1
    #pragma HLS ARRAY_PARTITION variable=m.raw_d_addr  complete dim=1

    m.ctrl.opcode       = pkt.opcode;
    m.ctrl.tlast        = pkt.tlast;
    m.ctrl.active_count = pkt.active_count;
    m.ctrl.seq          = pkt.seq;
    m.ctrl.frame_ok     = pkt.frame_ok;

    if (pkt.opcode == OP_RESET) {
        m.raw = true;
        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL factor=K_PARALLEL
            center[i]      = 0;
            initialized[i] = false;
            for (int k = 0; k < DELTA_MAX_STRIDE; k++) {
                #pragma HLS UNROLL
                val_hist[i][k] = 0;
            }
            m.raw_addr[i]   = 0;
            m.raw_d_addr[i] = 0;
        }
        last_opcode_ae = OP_RESET;
    } else if (pkt.opcode == OP_CONFIG) {
        m.raw = true;
        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL factor=K_PARALLEL
            int word = i >> 2, byte = i & 3;
            m.raw_d_addr[i] = (delta_addr_t)((pkt.data[word] >> (byte * 8)) & 0xFF);
            m.raw_addr[i]   = 0;
        }
        ap_uint<16> sp = (ap_uint<16>)(pkt.data[4] & 0xFFFF);
        m.raw_addr[0] = (bin_addr_t)(sp & 0x7FF);
        m.raw_addr[1] = (bin_addr_t)((sp >> 11) & 0x1F);

        m.raw_addr[2] = (bin_addr_t)(pkt.data[6] & 0x1F);

        ap_uint<32> s = (ap_uint<32>)pkt.data[5];
        if (s < 1)                     delta_stride = 1;
        else if (s > DELTA_MAX_STRIDE) delta_stride = DELTA_MAX_STRIDE;
        else                           delta_stride = (ap_uint<4>)s;

    } else {
        m.raw = false;
        bool phase_change = (pkt.opcode != OP_DUMP) && (pkt.opcode != last_opcode_ae);
        bool do_shift     = (pkt.opcode != OP_DUMP);

        ap_uint<4> eff_stride = (pkt.opcode == OP_DETECT) ? delta_stride : (ap_uint<4>)1;

        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS PIPELINE II=1
            #pragma HLS UNROLL factor=K_PARALLEL

            if (i < (int)pkt.active_count) {
                sensor_t v = pkt.data[i];

                if (!initialized[i]) {
                    center[i] = v;
                    initialized[i] = true;
                    for (int k = 0; k < DELTA_MAX_STRIDE; k++) {
                        #pragma HLS UNROLL
                        val_hist[i][k] = v;
                    }
                } else if (phase_change) {
                    for (int k = 0; k < DELTA_MAX_STRIDE; k++) {
                        #pragma HLS UNROLL
                        val_hist[i][k] = v;
                    }
                }

                m.active[i] = true;
                m.v[i]      = v;
                m.center[i] = center[i];
                m.ref[i]    = val_hist[i][eff_stride - 1];

                if (do_shift) {
                    for (int k = DELTA_MAX_STRIDE - 1; k > 0; k--) {
                        #pragma HLS UNROLL
                        val_hist[i][k] = val_hist[i][k-1];
                    }
                    val_hist[i][0] = v;
                }
            } else {
                m.active[i] = false;
                m.v[i]      = 0;
                m.center[i] = 0;
                m.ref[i]    = 0;
            }
        }

        if (pkt.opcode != OP_DUMP) {
            last_opcode_ae = pkt.opcode;
        }
    }

    mid.write(m);
}

static void stage_encode(
    hls::stream<ae_mid_t>&    mid,
    hls::stream<addr_ctrl_t>& out_ctrl,
    hls::stream<addr_data_t>& out_data
) {

    #pragma HLS PIPELINE II=1

    ae_mid_t m = mid.read();

    #pragma HLS ARRAY_PARTITION variable=m.active      complete dim=1
    #pragma HLS ARRAY_PARTITION variable=m.v           complete dim=1
    #pragma HLS ARRAY_PARTITION variable=m.center      complete dim=1
    #pragma HLS ARRAY_PARTITION variable=m.ref         complete dim=1
    #pragma HLS ARRAY_PARTITION variable=m.raw_addr    complete dim=1
    #pragma HLS ARRAY_PARTITION variable=m.raw_d_addr  complete dim=1

    addr_data_t data;
    #pragma HLS ARRAY_PARTITION variable=data.addr     complete dim=1
    #pragma HLS ARRAY_PARTITION variable=data.d_addr   complete dim=1
    for (int i = 0; i < NR_SENSORS; i++) {
        #pragma HLS PIPELINE II=1
        #pragma HLS UNROLL factor=K_PARALLEL
        if (m.raw) {
            data.addr[i]   = m.raw_addr[i];
            data.d_addr[i] = m.raw_d_addr[i];
        } else if (m.active[i]) {
            data.addr[i] = log_linear_addr(m.v[i], m.center[i]);
            sensor_t delta = (m.v[i] > m.ref[i]) ? (sensor_t)(m.v[i] - m.ref[i])
                                                 : (sensor_t)(m.ref[i] - m.v[i]);
            data.d_addr[i] = delta_log_linear_addr((ap_uint<32>)delta);
        } else {
            data.addr[i]   = 0;
            data.d_addr[i] = 0;
        }
    }

    out_ctrl.write(m.ctrl);
    out_data.write(data);
}

void address_engine(
    hls::stream<sensor_packet_t>& in_stream,
    hls::stream<addr_ctrl_t>&     out_ctrl,
    hls::stream<addr_data_t>&     out_data
) {
    #pragma HLS INTERFACE axis port=in_stream
    #pragma HLS INTERFACE axis port=out_ctrl
    #pragma HLS INTERFACE axis port=out_data
    #pragma HLS INTERFACE ap_ctrl_none port=return
    #pragma HLS DATAFLOW

    hls::stream<ae_mid_t> mid;
    #pragma HLS STREAM variable=mid depth=2

    stage_select(in_stream, mid);
    stage_encode(mid, out_ctrl, out_data);
}
