#include <cstdio>
#include <cstdlib>
#include "hbos_types.h"

static void write_addr(
    hls::stream<addr_packet_t>& s,
    opcode_t op,
    bool frame_ok
) {
    addr_packet_t p = {};
    for (int i = 0; i < NR_SENSORS; i++) {
        p.addr[i] = 100 + i;
        p.d_addr[i] = 10;
    }
    p.opcode = op;
    p.tlast = 0;
    p.frame_ok = frame_ok;
    s.write(p);
}

int main() {
    hls::stream<addr_packet_t> in;
    hls::stream<ap_uint<32>> config_in;
    hls::stream<anomaly_packet_t> anomaly_out;
    count_t hist[NR_SENSORS][NR_BINS] = {};

    int err = 0;

    write_addr(in, OP_TRAIN, true);
    detection_engine(in, config_in, hist, anomaly_out);
    if (!anomaly_out.empty()) {
        printf("FAIL: reply during TRAIN\n");
        err = 1;
    }

    write_addr(in, OP_CALIB, true);
    detection_engine(in, config_in, hist, anomaly_out);
    if (!anomaly_out.empty()) {
        printf("FAIL: reply during CALIB\n");
        err = 1;
    }

    // 6 config words matching hbos_top's OP_DUMP output:
    // [0]=threshold [1]=packed_deltas [2]=rx_train [3]=rx_calib [4]=packed_weights [5]=spike_penalty
    config_in.write((ap_uint<32>)2288);
    config_in.write((ap_uint<32>)((105) | (165 << 8) | (144 << 16) | (87 << 24)));
    config_in.write((ap_uint<32>)1000);
    config_in.write((ap_uint<32>)500);
    config_in.write((ap_uint<32>)((50) | (93 << 8) | (58 << 16) | (55 << 24)));
    config_in.write((ap_uint<32>)5632);

    // OP_DUMP reads word 0 (cfg_rx_cnt: 0→1), sets dump_ack_pending.
    write_addr(in, OP_DUMP, true);
    detection_engine(in, config_in, hist, anomaly_out);
    if (!anomaly_out.empty()) {
        printf("FAIL: ack before pump beat\n");
        err = 1;
    }

    // 4 pump calls read words 1-4 (cfg_rx_cnt: 1→5); no latch yet.
    for (int d = 0; d < 4; d++) {
        write_addr(in, OP_CALIB, true);
        detection_engine(in, config_in, hist, anomaly_out);
        if (!anomaly_out.empty()) {
            printf("FAIL: premature output during pump %d\n", d);
            err = 1;
        }
    }

    // Final pump reads word 5 (cfg_rx_cnt==5): latches config and writes 0xFF ack.
    write_addr(in, OP_CALIB, true);
    detection_engine(in, config_in, hist, anomaly_out);
    if (anomaly_out.empty()) {
        printf("FAIL: no DUMP ack after pump\n");
        err = 1;
    } else {
        anomaly_packet_t ack = anomaly_out.read();
        if (ack.data != 0xFF) {
            printf("FAIL: DUMP ack 0x%02X\n", (int)ack.data);
            err = 1;
        } else {
            printf("PASS: DUMP ack 0xFF\n");
        }
    }

    write_addr(in, OP_DETECT, true);
    detection_engine(in, config_in, hist, anomaly_out);
    if (anomaly_out.empty()) {
        printf("FAIL: no DETECT reply\n");
        err = 1;
    } else {
        printf("PASS: DETECT reply 0x%02X\n", (int)anomaly_out.read().data);
    }

    // ── retrain: OP_TRAIN must disable inference ──────────────────────────────
    // After OP_TRAIN arrives, inference_enabled must be false so that OP_DETECT
    // produces no output until a new config is latched (DUMP→CALIB pump).
    write_addr(in, OP_TRAIN, true);
    detection_engine(in, config_in, hist, anomaly_out);
    if (!anomaly_out.empty()) {
        printf("FAIL: unexpected output during retrain OP_TRAIN\n");
        err = 1;
    }

    write_addr(in, OP_DETECT, true);
    detection_engine(in, config_in, hist, anomaly_out);
    if (!anomaly_out.empty()) {
        printf("FAIL: OP_DETECT should be disabled after retrain OP_TRAIN\n");
        err = 1;
    } else {
        printf("PASS: inference correctly disabled after retrain OP_TRAIN\n");
    }

    return err ? 1 : 0;
}
