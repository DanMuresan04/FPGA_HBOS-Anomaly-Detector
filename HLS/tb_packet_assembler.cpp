#include <cstdio>
#include <cstdlib>
#include "hbos_types.h"

static void feed_frame(
    hls::stream<rx_byte_axis_t>& rx,
    const ap_uint<8> buf[20],
    int pad_after
) {
    for (int i = 0; i < 20; i++) {
        rx_byte_axis_t beat;
        beat.data = buf[i];
        beat.keep = 1;
        beat.strb = 1;
        beat.last = (pad_after == 0 && i == 19);
        rx.write(beat);
    }
    for (int p = 0; p < pad_after; p++) {
        rx_byte_axis_t beat;
        beat.data = 0;
        beat.keep = 1;
        beat.strb = 1;
        beat.last = (p == pad_after - 1);
        rx.write(beat);
    }
}

static int run_one(const char* label, const ap_uint<8> buf[20], int pad_after, bool expect_ok) {
    hls::stream<rx_byte_axis_t> rx;
    hls::stream<sensor_packet_t> out;
    feed_frame(rx, buf, pad_after);
    packet_assembler(rx, out);
    sensor_packet_t pkt = out.read();
    if (pkt.frame_ok != expect_ok) {
        printf("FAIL %s: frame_ok=%d expected %d\n", label, (int)pkt.frame_ok, (int)expect_ok);
        return 1;
    }
    printf("PASS %s: opcode=%d frame_ok=%d\n", label, (int)pkt.opcode, (int)pkt.frame_ok);
    return 0;
}

int main() {
    ap_uint<8> good[20] = {0};
    good[16] = OP_CALIB;
    good[18] = FRAME_MAGIC_LO;
    good[19] = FRAME_MAGIC_HI;

    ap_uint<8> bad[20] = {0};
    bad[16] = OP_CALIB;

    int err = 0;
    err |= run_one("good_20B", good, 0, true);
    {
        hls::stream<rx_byte_axis_t> rx;
        hls::stream<sensor_packet_t> out;
        feed_frame(rx, good, 4);
        packet_assembler(rx, out);
        if (out.read().frame_ok) {
            printf("FAIL padded_drop: expected frame_ok=0\n");
            err = 1;
        }
        feed_frame(rx, good, 0);
        packet_assembler(rx, out);
        if (!out.read().frame_ok) {
            printf("FAIL padded_resync: second frame not ok\n");
            err = 1;
        } else {
            printf("PASS padded_resync\n");
        }
    }
    err |= run_one("bad_magic", bad, 0, false);
    return err ? 1 : 0;
}
