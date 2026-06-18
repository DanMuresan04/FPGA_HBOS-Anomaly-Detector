#include "hbos_types.h"
#include <hls_stream.h>

#define MAX_DRAIN_BYTES 64

void packet_assembler(
    hls::stream<rx_byte_axis_t>& rx_in,
    hls::stream<sensor_packet_t>& packet_out
) {
    #pragma HLS INTERFACE axis port=rx_in
    #pragma HLS INTERFACE axis port=packet_out
    #pragma HLS INTERFACE ap_ctrl_none port=return

    ap_uint<8> buffer[20];
    #pragma HLS ARRAY_PARTITION variable=buffer complete

    bool last_on_byte19 = false;
    bool early_last = false;

    for (int i = 0; i < 20; i++) {
        #pragma HLS PIPELINE II=1
        rx_byte_axis_t beat = rx_in.read();
        buffer[i] = beat.data;
        if (beat.last) {
            if (i == 19) {
                last_on_byte19 = true;
            } else {
                early_last = true;
            }
        }
    }

    if (!early_last && !last_on_byte19) {
        for (int d = 0; d < MAX_DRAIN_BYTES; d++) {
            #pragma HLS PIPELINE II=1
            rx_byte_axis_t tail = rx_in.read();
            if (tail.last) {
                break;
            }
        }
    }

    sensor_packet_t pkt;

    for (int s = 0; s < 4; s++) {
        #pragma HLS UNROLL
        int base = s * 4;
        pkt.data[s] = ((sensor_t)buffer[base + 3] << 24) |
                      ((sensor_t)buffer[base + 2] << 16) |
                      ((sensor_t)buffer[base + 1] << 8)  |
                      ((sensor_t)buffer[base]);
    }

    pkt.opcode = (opcode_t)(buffer[16] & 0x07);
    pkt.tlast = (bool)(buffer[17] & 0x01);
    pkt.reserve = ((ap_uint<16>)buffer[19] << 8) | (ap_uint<16>)buffer[18];
    pkt.frame_ok = last_on_byte19 &&
                   (buffer[18] == FRAME_MAGIC_LO) && (buffer[19] == FRAME_MAGIC_HI);

    packet_out.write(pkt);
}
