#include "hbos_types.h"
#include <hls_stream.h>

void packet_assembler(
    hls::stream<rx_byte_axis_t>& rx_in,
    hls::stream<sensor_packet_t>& packet_out
) {
    #pragma HLS INTERFACE axis port=rx_in
    #pragma HLS INTERFACE axis port=packet_out
    #pragma HLS INTERFACE ap_ctrl_none port=return

    ap_uint<8> h_nwords = rx_in.read().data;
    ap_uint<8> h_active = rx_in.read().data;
    ap_uint<8> h_opcode = rx_in.read().data;
    ap_uint<8> h_tlast  = rx_in.read().data;

    ap_uint<8> h_seq0   = rx_in.read().data;
    ap_uint<8> h_seq1   = rx_in.read().data;
    ap_uint<8> h_seq2   = rx_in.read().data;

    int n_words = (int)h_nwords;
    if (n_words > PKT_WORDS) n_words = PKT_WORDS;

    sensor_packet_t pkt;

    #pragma HLS ARRAY_PARTITION variable=pkt.data complete dim=1
    pkt.opcode       = (opcode_t)(h_opcode & 0x0F);
    pkt.tlast        = (bool)(h_tlast & 0x01);
    pkt.active_count = (ap_uint<5>)h_active;
    pkt.seq          = ((ap_uint<24>)h_seq2 << 16) |
                       ((ap_uint<24>)h_seq1 << 8)  |
                        (ap_uint<24>)h_seq0;
    pkt.reserve      = 0;

    for (int i = 0; i < n_words; i++) {
        #pragma HLS LOOP_TRIPCOUNT min=1 max=16
        #pragma HLS PIPELINE II=4
        ap_uint<8> b0 = rx_in.read().data;
        ap_uint<8> b1 = rx_in.read().data;
        ap_uint<8> b2 = rx_in.read().data;
        ap_uint<8> b3 = rx_in.read().data;
        pkt.data[i] = ((sensor_t)b3 << 24) | ((sensor_t)b2 << 16) |
                      ((sensor_t)b1 << 8)  |  (sensor_t)b0;
    }
    for (int i = 0; i < PKT_WORDS; i++) {
        #pragma HLS UNROLL
        if (i >= n_words) pkt.data[i] = 0;
    }

    ap_uint<8> m0 = rx_in.read().data;
    ap_uint<8> m1 = rx_in.read().data;
    pkt.frame_ok = (m0 == FRAME_MAGIC_LO) && (m1 == FRAME_MAGIC_HI);

    packet_out.write(pkt);
}
