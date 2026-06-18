#include <cstdio>
#include "hbos_types.h"

int main() {
    hls::stream<sensor_packet_t> in;
    hls::stream<addr_packet_t> out;

    sensor_packet_t pkt = {};
    pkt.data[0] = 1000;
    pkt.data[1] = 2000;
    pkt.data[2] = 3000;
    pkt.data[3] = 4000;
    pkt.opcode = OP_CALIB;
    pkt.tlast = 0;
    pkt.frame_ok = true;
    in.write(pkt);

    address_engine(in, out);
    addr_packet_t a = out.read();

    if (!a.frame_ok) {
        printf("FAIL: frame_ok not passed through\n");
        return 1;
    }
    printf("PASS: frame_ok=1 addrs=%d,%d,%d,%d\n",
           (int)a.addr[0], (int)a.addr[1], (int)a.addr[2], (int)a.addr[3]);
    return 0;
}
