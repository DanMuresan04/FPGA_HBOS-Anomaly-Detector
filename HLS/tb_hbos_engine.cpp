#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include "hbos_types.h"
#include "hbos_engine.h"
#include "tb_verdict.h"

void packet_assembler(hls::stream<rx_byte_axis_t>&, hls::stream<sensor_packet_t>&);
void address_engine(hls::stream<sensor_packet_t>&, hls::stream<addr_ctrl_t>&, hls::stream<addr_data_t>&);

static void send_byte(hls::stream<rx_byte_axis_t>& rx, ap_uint<8> b, bool last) {
    rx_byte_axis_t beat; beat.data = b; beat.keep = 1; beat.strb = 1; beat.last = last;
    rx.write(beat);
}

static void send_frame(hls::stream<rx_byte_axis_t>& rx,
                       const long *vals, int n, int active, int opcode, int tlast) {
    send_byte(rx, (ap_uint<8>)n,       false);
    send_byte(rx, (ap_uint<8>)active,  false);
    send_byte(rx, (ap_uint<8>)opcode,  false);
    send_byte(rx, (ap_uint<8>)tlast,   false);
    send_byte(rx, 0, false); send_byte(rx, 0, false); send_byte(rx, 0, false);
    for (int i = 0; i < n; i++) {
        long v = vals[i];
        send_byte(rx, (ap_uint<8>)(v & 0xFF),         false);
        send_byte(rx, (ap_uint<8>)((v >> 8) & 0xFF),  false);
        send_byte(rx, (ap_uint<8>)((v >> 16) & 0xFF), false);
        send_byte(rx, (ap_uint<8>)((v >> 24) & 0xFF), false);
    }
    send_byte(rx, FRAME_MAGIC_LO, false);
    send_byte(rx, FRAME_MAGIC_HI, true);
}

static void pipeline_one_packet(
    hls::stream<rx_byte_axis_t>& rx_byte_stream,
    hls::stream<anomaly_packet_t>& anomaly_out,
    const long *vals, int n, int active, int opcode, int tlast
) {
    hls::stream<sensor_packet_t> raw_in;
    hls::stream<addr_ctrl_t>     ctrl_out;
    hls::stream<addr_data_t>     data_out;
    static hls::stream<verdict_beat_t> vout;
    send_frame(rx_byte_stream, vals, n, active, opcode, tlast);
    packet_assembler(rx_byte_stream, raw_in);
    address_engine(raw_in, ctrl_out, data_out);
    hbos_engine(ctrl_out, data_out, vout);
    tb_unpack_verdicts(vout, anomaly_out);
}

int main() {

    const char* root_env = std::getenv("LICENTA_ROOT");
    std::string csv_path = (root_env ? std::string(root_env) : std::string(".."))
                          + "/training_engine/hls_test_stream.csv";
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open " << csv_path << std::endl;
        return 1;
    }

    hls::stream<rx_byte_axis_t>   rx_byte_stream;
    hls::stream<anomaly_packet_t> anomaly_out;

    const int ACTIVE = 4;
    printf("Merged hbos_engine PoC: NR_SENSORS=%d, active_count=%d\n", NR_SENSORS, ACTIVE);
    printf("(defaults weights {50,93,58,55} + spike 5632 match the golden config)\n\n");

    for (int pass = 0; pass < 2; pass++) {
        const char *name = (pass == 0) ? "TRAINING" : "CALIBRATION";
        printf("--- PASS %d: %s ---\n", pass, name);
        file.clear(); file.seekg(0);
        std::string line; int count = 0;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            long vals[NR_SENSORS] = {0};
            for (int i = 0; i < ACTIVE; i++) {
                std::getline(ss, cell, ',');
                vals[i] = std::stol(cell);
            }
            std::getline(ss, cell, ',');
            int label = std::stoi(cell);
            pipeline_one_packet(rx_byte_stream, anomaly_out, vals, ACTIVE, ACTIVE,
                                pass, (label == 0) ? 0 : 1);
            count++;
        }
        printf("  Processed %d samples\n", count);
    }

    printf("\n--- OP_DUMP ---\n");
    {
        long zeros[NR_SENSORS] = {0};
        pipeline_one_packet(rx_byte_stream, anomaly_out, zeros, ACTIVE, ACTIVE, OP_DUMP, 0);
    }

    if (anomaly_out.empty()) { std::cerr << "ERROR: No DUMP ACK!" << std::endl; return 1; }
    anomaly_packet_t ack = anomaly_out.read();
    printf("Received ACK: 0x%02X %s\n", (int)ack.data, ack.data == 0xFF ? "[SUCCESS]" : "[UNEXPECTED]");
    if (ack.data != 0xFF) return 1;

    printf("\n--- OP_DETECT ---\n");
    file.clear(); file.seekg(0);
    int total_responses = 0, tp = 0, fp = 0, fn = 0, tn = 0;
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        long vals[NR_SENSORS] = {0};
        for (int i = 0; i < ACTIVE; i++) {
            std::getline(ss, cell, ',');
            vals[i] = std::stol(cell);
        }
        std::getline(ss, cell, ',');
        int label = std::stoi(cell);
        pipeline_one_packet(rx_byte_stream, anomaly_out, vals, ACTIVE, ACTIVE, OP_DETECT, (label == 0) ? 0 : 1);

        while (!anomaly_out.empty()) {
            anomaly_packet_t res = anomaly_out.read();
            total_responses++;
            bool predicted = (res.data == 0x01);
            bool actual    = (label != 0);
            if (predicted && actual) tp++;
            else if (predicted && !actual) fp++;
            else if (!predicted && actual) fn++;
            else tn++;
        }
    }

    double precision = (tp + fp > 0) ? (double)tp / (tp + fp) : 0.0;
    double recall    = (tp + fn > 0) ? (double)tp / (tp + fn) : 0.0;
    double f1 = (precision + recall > 0.0) ? 2.0 * (precision * recall) / (precision + recall) : 0.0;

    printf("\n=== RESULTS ===\n");
    printf("Total Responses:      %d\n", total_responses);
    printf("True Positives (TP):  %d\n", tp);
    printf("False Positives (FP): %d\n", fp);
    printf("False Negatives (FN): %d\n", fn);
    printf("True Negatives (TN):  %d\n", tn);
    printf("Precision: %.4f\n", precision);
    printf("Recall:    %.4f\n", recall);
    printf("F1 Score:  %.4f\n", f1);
    file.close();
    printf("\nSimulation Complete.\n");
    return 0;
}
