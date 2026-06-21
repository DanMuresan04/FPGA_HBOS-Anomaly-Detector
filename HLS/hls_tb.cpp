#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdio>
#include "hbos_top.h"

static void write_udp_frame(hls::stream<rx_byte_axis_t>& rx, const ap_uint<8> buf[20]) {
    for (int i = 0; i < 20; i++) {
        rx_byte_axis_t beat;
        beat.data = buf[i];
        beat.keep = 1;
        beat.strb = 1;
        beat.last = (i == 19);
        rx.write(beat);
    }
}

static void serialize_packet(sensor_packet_t &pkt, ap_uint<8> buffer[20]) {
    for (int s = 0; s < 4; s++) {
        int base = s * 4;
        buffer[base]     = (ap_uint<8>)(pkt.data[s] & 0xFF);
        buffer[base + 1] = (ap_uint<8>)((pkt.data[s] >> 8) & 0xFF);
        buffer[base + 2] = (ap_uint<8>)((pkt.data[s] >> 16) & 0xFF);
        buffer[base + 3] = (ap_uint<8>)((pkt.data[s] >> 24) & 0xFF);
    }
    buffer[16] = (ap_uint<8>)(pkt.opcode & 0x07);
    buffer[17] = (ap_uint<8>)(pkt.tlast ? 1 : 0);
    buffer[18] = FRAME_MAGIC_LO;
    buffer[19] = FRAME_MAGIC_HI;
}

static void pipeline_one_packet(
    hls::stream<rx_byte_axis_t>& rx_byte_stream,
    count_t hist[NR_SENSORS][NR_BINS],
    hls::stream<ap_uint<32>>& config_fifo,
    hls::stream<anomaly_packet_t>& anomaly_out,
    ap_uint<8> buffer[20]
) {
    hls::stream<sensor_packet_t> raw_in;
    hls::stream<addr_packet_t>   addr_out;

    write_udp_frame(rx_byte_stream, buffer);
    packet_assembler(rx_byte_stream, raw_in);
    address_engine(raw_in, addr_out);

    addr_packet_t pkt = addr_out.read();

    hls::stream<addr_packet_t> hbos_in;
    hls::stream<addr_packet_t> det_in;
    hbos_in.write(pkt);
    det_in.write(pkt);

    hbos_top(hbos_in, hist, config_fifo);
    detection_engine(det_in, config_fifo, hist, anomaly_out);
}

int main() {
    std::ifstream file("hls_test_stream.csv");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open hls_test_stream.csv" << std::endl;
        return 1;
    }

    hls::stream<rx_byte_axis_t>  rx_byte_stream;
    hls::stream<ap_uint<32>>     config_fifo;
    hls::stream<anomaly_packet_t> anomaly_out;
    count_t hist[NR_SENSORS][NR_BINS];

    for (int i = 0; i < NR_SENSORS; i++)
        for (int j = 0; j < NR_BINS; j++)
            hist[i][j] = 0;

    printf("Starting Training Engine C-Simulation...\n");
    printf("Both hbos_top and detection_engine receive every packet (broadcaster model).\n\n");

    {
        ap_uint<8> cfg_buf[20] = {0};
        cfg_buf[0] = 50;
        cfg_buf[1] = 93;
        cfg_buf[2] = 58;
        cfg_buf[3] = 55;
        cfg_buf[4] = (ap_uint<8>)(5632 & 0xFF);
        cfg_buf[5] = (ap_uint<8>)((5632 >> 8) & 0xFF);
        cfg_buf[16] = OP_CONFIG;
        cfg_buf[17] = 0;
        cfg_buf[18] = FRAME_MAGIC_LO;
        cfg_buf[19] = FRAME_MAGIC_HI;
        pipeline_one_packet(rx_byte_stream, hist, config_fifo, anomaly_out, cfg_buf);
        printf("OP_CONFIG delivered.\n\n");
    }

    for (int pass = 0; pass < 2; pass++) {
        const char *name = (pass == 0) ? "TRAINING" : "CALIBRATION";
        printf("--- PASS %d: %s ---\n", pass, name);
        file.clear();
        file.seekg(0);
        std::string line;
        int count = 0;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            sensor_packet_t pkt;
            for (int i = 0; i < NR_SENSORS; i++) {
                std::getline(ss, cell, ',');
                pkt.data[i] = (sensor_t)std::stol(cell);
            }
            std::getline(ss, cell, ',');
            int label = std::stoi(cell);
            pkt.opcode = (opcode_t)pass;
            pkt.tlast  = (label == 0) ? 0 : 1;
            pkt.reserve = 0;

            ap_uint<8> buffer[20];
            serialize_packet(pkt, buffer);
            pipeline_one_packet(rx_byte_stream, hist, config_fifo, anomaly_out, buffer);
            count++;
        }
        printf("  Processed %d samples\n", count);
    }

    printf("\n--- PASS 2: OP_DUMP ---\n");

    sensor_packet_t dump_pkt;
    dump_pkt.opcode = OP_DUMP;
    dump_pkt.tlast = 0;
    dump_pkt.reserve = 0;
    for (int i = 0; i < NR_SENSORS; i++) dump_pkt.data[i] = 0;

    ap_uint<8> dump_buffer[20];
    serialize_packet(dump_pkt, dump_buffer);
    pipeline_one_packet(rx_byte_stream, hist, config_fifo, anomaly_out, dump_buffer);
    printf("  OP_DUMP delivered. hbos_top wrote config, detection_engine armed.\n");

    sensor_packet_t trigger_pkt;
    trigger_pkt.opcode = OP_CALIB;
    trigger_pkt.tlast = 0;
    trigger_pkt.reserve = 0;
    for (int i = 0; i < NR_SENSORS; i++) trigger_pkt.data[i] = 0;

    ap_uint<8> trigger_buffer[20];
    serialize_packet(trigger_pkt, trigger_buffer);

    printf("  Draining 5 config words (5x OP_CALIB)...\n");
    for (int d = 0; d < 5; d++) {
        pipeline_one_packet(rx_byte_stream, hist, config_fifo, anomaly_out, trigger_buffer);
    }

    printf("\n=== DUMP ACK CHECK ===\n");
    if (anomaly_out.empty()) {
        std::cerr << "ERROR: No DUMP ACK!" << std::endl;
        return 1;
    }
    anomaly_packet_t ack = anomaly_out.read();
    printf("Received ACK: 0x%02X", (int)ack.data);
    if (ack.data == 0xFF) {
        printf(" [SUCCESS]\n");
    } else {
        printf(" [UNEXPECTED]\n");
        return 1;
    }

    printf("\n--- PASS 3: OP_DETECT ---\n");
    file.clear();
    file.seekg(0);
    int total_responses = 0;
    int tp = 0, fp = 0, fn = 0, tn = 0;
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        sensor_packet_t pkt;
        for (int i = 0; i < NR_SENSORS; i++) {
            std::getline(ss, cell, ',');
            pkt.data[i] = (sensor_t)std::stol(cell);
        }
        std::getline(ss, cell, ',');
        int label = std::stoi(cell);
        pkt.opcode = OP_DETECT;
        pkt.tlast  = (label == 0) ? 0 : 1;
        pkt.reserve = 0;

        ap_uint<8> buffer[20];
        serialize_packet(pkt, buffer);
        pipeline_one_packet(rx_byte_stream, hist, config_fifo, anomaly_out, buffer);

        while (!anomaly_out.empty()) {
            anomaly_packet_t res = anomaly_out.read();
            total_responses++;
            bool predicted_anomaly = (res.data == 0x01);
            bool actual_anomaly = (label != 0);

            if (predicted_anomaly && actual_anomaly) {
                tp++;
            } else if (predicted_anomaly && !actual_anomaly) {
                fp++;
            } else if (!predicted_anomaly && actual_anomaly) {
                fn++;
            } else {
                tn++;
            }
        }
    }

    double precision = (tp + fp > 0) ? (double)tp / (tp + fp) : 0.0;
    double recall = (tp + fn > 0) ? (double)tp / (tp + fn) : 0.0;
    double f1 = (precision + recall > 0.0) ? 2.0 * (precision * recall) / (precision + recall) : 0.0;

    printf("\n=== RESULTS ===\n");
    printf("Total Responses:      %d\n", total_responses);
    printf("True Positives (TP):  %d (Correctly detected anomalies)\n", tp);
    printf("False Positives (FP): %d (False alarms)\n", fp);
    printf("False Negatives (FN): %d (Missed anomalies)\n", fn);
    printf("True Negatives (TN):  %d (Correctly ignored normal samples)\n", tn);
    printf("\n--- METRICS ---\n");
    printf("Precision: %.4f (%% of detected anomalies that are real)\n", precision);
    printf("Recall:    %.4f (%% of real anomalies detected)\n", recall);
    printf("F1 Score:  %.4f\n", f1);
    file.close();
    printf("\nSimulation Complete.\n");
    return 0;
}
