#include "hbos_types.h"
#include "hbos_math.h"
#include <hls_stream.h>

void address_engine(
    hls::stream<sensor_packet_t>& in_stream,
    hls::stream<addr_packet_t>&   out_stream
) {
    #pragma HLS INTERFACE axis port=in_stream
    #pragma HLS INTERFACE axis port=out_stream
    #pragma HLS INTERFACE ap_ctrl_none port=return

    static sensor_t center[NR_SENSORS];
    #pragma HLS ARRAY_PARTITION variable=center cyclic factor=K_PARALLEL dim=1

    static sensor_t prev_val[NR_SENSORS];
    #pragma HLS ARRAY_PARTITION variable=prev_val cyclic factor=K_PARALLEL dim=1

    static bool initialized[NR_SENSORS];
    #pragma HLS ARRAY_PARTITION variable=initialized cyclic factor=K_PARALLEL dim=1

    static opcode_t last_opcode_ae = OP_TRAIN;

    sensor_packet_t pkt = in_stream.read();

    addr_packet_t out;
    out.opcode = pkt.opcode;
    out.tlast  = pkt.tlast;
    out.active_count = pkt.active_count;
    out.frame_ok = pkt.frame_ok;

    if (pkt.opcode == OP_RESET) {

        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL factor=K_PARALLEL
            center[i]      = 0;
            prev_val[i]    = 0;
            initialized[i] = false;
            out.addr[i]    = 0;
            out.d_addr[i]  = 0;
        }
        last_opcode_ae = OP_RESET;
    } else if (pkt.opcode == OP_CONFIG) {

        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL factor=K_PARALLEL
            int word = i >> 2, byte = i & 3;
            out.d_addr[i] = (delta_addr_t)((pkt.data[word] >> (byte * 8)) & 0xFF);
            out.addr[i]   = 0;
        }
        ap_uint<16> sp = (ap_uint<16>)(pkt.data[4] & 0xFFFF);
        out.addr[0] = (bin_addr_t)(sp & 0x7FF);
        out.addr[1] = (bin_addr_t)((sp >> 11) & 0x1F);

    } else {
        bool phase_change = (pkt.opcode != OP_DUMP) && (pkt.opcode != last_opcode_ae);

        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL factor=K_PARALLEL

            if (i < (int)pkt.active_count) {
                sensor_t v = pkt.data[i];

                if (!initialized[i]) {
                    center[i] = v;
                    prev_val[i] = v;
                    initialized[i] = true;
                } else if (phase_change) {
                    prev_val[i] = v;
                }

                out.addr[i] = log_linear_addr(v, center[i]);

                sensor_t delta;
                if (v > prev_val[i]) {
                    delta = v - prev_val[i];
                } else {
                    delta = prev_val[i] - v;
                }
                out.d_addr[i] = delta_log_linear_addr((ap_uint<32>)delta);
            } else {
                out.addr[i]   = 0;
                out.d_addr[i] = 0;
            }
        }

        if (pkt.opcode != OP_DUMP) {
            for (int i = 0; i < NR_SENSORS; i++) {
                #pragma HLS UNROLL factor=K_PARALLEL
                if (i < (int)pkt.active_count) prev_val[i] = pkt.data[i];
            }
            last_opcode_ae = pkt.opcode;
        }
    }

    out_stream.write(out);
}
