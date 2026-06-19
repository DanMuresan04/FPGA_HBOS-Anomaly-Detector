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
    #pragma HLS ARRAY_PARTITION variable=center complete dim=1

    static sensor_t prev_val[NR_SENSORS];
    #pragma HLS ARRAY_PARTITION variable=prev_val complete dim=1

    static bool initialized[NR_SENSORS];
    #pragma HLS ARRAY_PARTITION variable=initialized complete dim=1

    static opcode_t last_opcode_ae = OP_TRAIN;

    sensor_packet_t pkt = in_stream.read();

    addr_packet_t out;
    out.opcode = pkt.opcode;
    out.tlast  = pkt.tlast;
    out.frame_ok = pkt.frame_ok;

    if (pkt.opcode == OP_RESET) {
        // Full flush: forget the learned center and the delta history so the
        // next TRAIN re-initialises from its first sample (initialized[]=false
        // makes the next data packet reload center[]/prev_val[]).
        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL
            center[i]      = 0;
            prev_val[i]    = 0;
            initialized[i] = false;
            out.addr[i]    = 0;
            out.d_addr[i]  = 0;
        }
        last_opcode_ae = OP_RESET;
    } else if (pkt.opcode == OP_CONFIG) {
        // Encode config payload into addr/d_addr for hbos_top to latch.
        // Packet layout (bytes):  [0]=w[0] [1]=w[1] [2]=w[2] [3]=w[3]
        //                         [4..5]=spike_penalty LE  [6..15]=padding
        // pkt.data[0] LE packs all four weights; pkt.data[1] LE holds spike_penalty.
        out.d_addr[0] = (delta_addr_t)((pkt.data[0] >>  0) & 0xFF);
        out.d_addr[1] = (delta_addr_t)((pkt.data[0] >>  8) & 0xFF);
        out.d_addr[2] = (delta_addr_t)((pkt.data[0] >> 16) & 0xFF);
        out.d_addr[3] = (delta_addr_t)((pkt.data[0] >> 24) & 0xFF);
        ap_uint<16> sp = (ap_uint<16>)(pkt.data[1] & 0xFFFF);
        out.addr[0] = (bin_addr_t)(sp & 0x7FF);          // low 11 bits
        out.addr[1] = (bin_addr_t)((sp >> 11) & 0x1F);   // high 5 bits
        out.addr[2] = 0;
        out.addr[3] = 0;
        // Do NOT touch center[], prev_val[], or initialized[]: config is not sensor data.
    } else {
        bool phase_change = (pkt.opcode != OP_DUMP) && (pkt.opcode != last_opcode_ae);

        for (int i = 0; i < NR_SENSORS; i++) {
            #pragma HLS UNROLL

            sensor_t v = pkt.data[i];

            if (!initialized[i]) {
                center[i] = v;
                prev_val[i] = v;
                initialized[i] = true;
            } else if (phase_change) {
                prev_val[i] = v;
            }

            out.addr[i] = log_linear_addr(v, center[i]);

            // Delta vs the immediately preceding sample.
            sensor_t delta;
            if (v > prev_val[i]) {
                delta = v - prev_val[i];
            } else {
                delta = prev_val[i] - v;
            }
            out.d_addr[i] = delta_log_linear_addr((ap_uint<32>)delta);
        }

        // Store the current sample as the previous one for the next packet.
        if (pkt.opcode != OP_DUMP) {
            for (int i = 0; i < NR_SENSORS; i++) {
                #pragma HLS UNROLL
                prev_val[i] = pkt.data[i];
            }
            last_opcode_ae = pkt.opcode;
        }
    }

    out_stream.write(out);
}
