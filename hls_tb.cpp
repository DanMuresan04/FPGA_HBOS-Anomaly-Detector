#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdio>
#include "hbos_top.h"

int main() {
    std::cout << "Starting HLS C-Simulation" << std::endl;

    std::ifstream file("hls_test_stream.csv");
    if (!file.is_open()) {
        std::cout << "Error: Could not open hls_test_stream.csv" << std::endl;
        return 1;
    }

    hls::stream<sensor_packet_t> in_stream;
    hls::stream<bool> out_stream;

    std::string line;
    int count = 0;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        sensor_packet_t pkt;

        for (int i = 0; i < NR_SENSORS; i++) {
            std::getline(ss, cell, ',');
            long raw = std::stol(cell);
            pkt.data[i] = (sensor_t)raw;
            if (count == 0) {
                printf("TB sensor[%d]: string='%s' long=%ld ap_int=%d\n",
                    i, cell.c_str(), raw, (int)pkt.data[i]);
            }
        }
        std::getline(ss, cell, ',');
        pkt.opcode = (opcode_t)std::stoi(cell);
        pkt.tlast = false;

        in_stream.write(pkt);
        hbos_top(in_stream, out_stream);
        out_stream.read();
        count++;
    }
    
    std::cout << "--- Final Hardware State ---" << std::endl;
    for (int i = 0; i < NR_SENSORS; i++) {
        std::cout << "Sensor [" << i << "]:" << std::endl;
        std::cout << "  Min Value: " << (long)config[i].min_v << std::endl;
        std::cout << "  Value Zoom (shamt): " << (int)config[i].shamt << std::endl;
        std::cout << "  Delta Zoom (d_shamt): " << (int)config[i].d_shamt << std::endl;
        
        std::cout << "  First Bins: ";
        int found = 0;
        for(int b=0; b<NR_BINS && found < 5; b++) {
            if(hist[i][b] > 0) {
                std::cout << "Bin[" << b << "]=" << (int)hist[i][b] << " ";
                found++;
            }
        }
        std::cout << std::endl;
    }
    std::cout << "Simulation finished. Processed " << count << " samples." << std::endl;
    return 0;
}
