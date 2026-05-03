#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdio>
#include "hbos_top.h"

int main() {
    std::ifstream file("hls_test_stream.csv");
    if (file.is_open() == false) {
        std::cerr << "Error: Could not open hls_test_stream.csv" << std::endl;
        return 1;
    }

    hls::stream<sensor_packet_t> in_stream;
    hls::stream<bool> out_stream;

    int true_pos = 0;
    int false_pos = 0;
    int true_neg = 0;
    int false_neg = 0;
    int total_processed = 0;

    printf("Starting optimized 3-pass C-Simulation...\n");

    for (int pass = 0; pass < 3; pass++) {
        printf("--- PASS %d (Opcode %d) ---\n", pass, pass);
        
        file.clear();
        file.seekg(0);

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

            std::getline(ss, cell, ',');
            int idx = std::stoi(cell);

            pkt.opcode = (opcode_t)pass;
            if (label == 0) {
                pkt.tlast = 0;
            } else {
                pkt.tlast = 1;
            }

            in_stream.write(pkt);
            hbos_top(in_stream, out_stream);
            bool result = out_stream.read();

            if (pass == 2) {
                bool actual;
                if (label == 1) {
                    actual = true;
                } else {
                    actual = false;
                }

                if (result == true) {
                    if (actual == true) {
                        true_pos++;
                    } else {
                        false_pos++;
                    }
                } else {
                    if (actual == true) {
                        false_neg++;
                    } else {
                        true_neg++;
                    }
                }
            }

            total_processed++;
            if (total_processed % 50000 == 0) {
                printf("Processed %d samples...\n", total_processed);
            }
        }
    }
    file.close();

    double precision = 0.0;
    if (true_pos + false_pos > 0) {
        precision = (double)true_pos / (true_pos + false_pos);
    }

    double recall = 0.0;
    if (true_pos + false_neg > 0) {
        recall = (double)true_pos / (true_pos + false_neg);
    }

    double f1 = 0.0;
    if (precision + recall > 0) {
        f1 = 2.0 * precision * recall / (precision + recall);
    }

    printf("\n=== FINAL PERFORMANCE ===\n");
    printf("  TP: %d, FP: %d, TN: %d, FN: %d\n", true_pos, false_pos, true_neg, false_neg);
    printf("  Precision: %.4f\n", precision);
    printf("  Recall:    %.4f\n", recall);
    printf("  F1 Score:  %.4f\n", f1);

    if (f1 > 0.75) {
        return 0;
    } else {
        return 1;
    }
}
