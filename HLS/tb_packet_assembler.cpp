#include <cstdio>
#include <cstdlib>
#include <vector>
#include "hbos_types.h"

static void push(hls::stream<rx_byte_axis_t>& rx, ap_uint<8> b) {
    rx_byte_axis_t beat;
    beat.data = b;
    beat.keep = 1;
    beat.strb = 1;
    beat.last = 0;
    rx.write(beat);
}

static void feed_frame(
    hls::stream<rx_byte_axis_t>& rx,
    int opcode, int active, unsigned seq,
    const std::vector<int>& words,
    ap_uint<8> magic_lo, ap_uint<8> magic_hi
) {
    push(rx, words.size());
    push(rx, active);
    push(rx, opcode);
    push(rx, 0);
    push(rx, seq & 0xFF);
    push(rx, (seq >> 8) & 0xFF);
    push(rx, (seq >> 16) & 0xFF);
    for (int w : words) {
        push(rx, w & 0xFF);
        push(rx, (w >> 8) & 0xFF);
        push(rx, (w >> 16) & 0xFF);
        push(rx, (w >> 24) & 0xFF);
    }
    push(rx, magic_lo);
    push(rx, magic_hi);
}

static int failures = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s\n", msg); failures++; } } while (0)

int main() {

    {
        hls::stream<rx_byte_axis_t> rx;
        hls::stream<sensor_packet_t> out;
        std::vector<int> words = {1, 2, 3, 4};
        feed_frame(rx, OP_CALIB, 4, 0x010203u, words, FRAME_MAGIC_LO, FRAME_MAGIC_HI);
        packet_assembler(rx, out);
        sensor_packet_t p = out.read();
        CHECK(p.frame_ok == true,        "good: frame_ok set on valid magic");
        CHECK(p.opcode == OP_CALIB,      "good: opcode decoded");
        CHECK(p.active_count == 4,       "good: active_count decoded");
        CHECK(p.seq == 0x010203u,        "good: 24-bit seq little-endian");
        CHECK(p.data[0] == 1 && p.data[1] == 2 &&
              p.data[2] == 3 && p.data[3] == 4, "good: payload decoded");
        CHECK(out.empty(),               "good: exactly one packet emitted");
    }

    {
        hls::stream<rx_byte_axis_t> rx;
        hls::stream<sensor_packet_t> out;
        feed_frame(rx, OP_CALIB, 4, 0, {1, 2, 3, 4}, 0x00, 0x00);
        packet_assembler(rx, out);
        sensor_packet_t p = out.read();
        CHECK(p.frame_ok == false,       "bad_magic: frame_ok clear on wrong magic");
    }

    {
        hls::stream<rx_byte_axis_t> rx;
        hls::stream<sensor_packet_t> out;
        feed_frame(rx, OP_TRAIN,  4, 0, {10, 20, 30, 40}, FRAME_MAGIC_LO, FRAME_MAGIC_HI);
        feed_frame(rx, OP_DETECT, 4, 7, {50, 60, 70, 80}, FRAME_MAGIC_LO, FRAME_MAGIC_HI);
        packet_assembler(rx, out);
        sensor_packet_t a = out.read();
        packet_assembler(rx, out);
        sensor_packet_t b = out.read();
        CHECK(a.frame_ok && a.opcode == OP_TRAIN && a.data[0] == 10,
              "resync: first frame parsed");
        CHECK(b.frame_ok && b.opcode == OP_DETECT && b.seq == 7 && b.data[0] == 50,
              "resync: second frame parsed (alignment preserved)");
    }

    {
        hls::stream<rx_byte_axis_t> rx;
        hls::stream<sensor_packet_t> out;
        std::vector<int> words(NR_SENSORS, 5);
        feed_frame(rx, OP_CALIB, NR_SENSORS, 0, words, FRAME_MAGIC_LO, FRAME_MAGIC_HI);
        packet_assembler(rx, out);
        sensor_packet_t p = out.read();
        CHECK(p.frame_ok == true,  "clamp: full-width frame stays aligned, magic found");
        CHECK(out.empty(),         "clamp: exactly one packet emitted");
    }

    printf(failures ? "\n=== %d FAILURE(S) ===\n" : "\n=== ALL PASS ===\n", failures);
    return failures ? 1 : 0;
}
