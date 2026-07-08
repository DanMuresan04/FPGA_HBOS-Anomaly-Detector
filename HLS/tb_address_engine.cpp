#include <cstdio>
#include "hbos_types.h"

int main() {
    hls::stream<sensor_packet_t> in;
    hls::stream<addr_ctrl_t>     ctrl;
    hls::stream<addr_data_t>     data;

    sensor_packet_t pkt = {};
    pkt.data[0] = 1000;
    pkt.data[1] = 2000;
    pkt.data[2] = 3000;
    pkt.data[3] = 4000;
    pkt.opcode = OP_CALIB;
    pkt.tlast = 0;
    pkt.frame_ok = true;
    in.write(pkt);

    address_engine(in, ctrl, data);
    addr_ctrl_t c = ctrl.read();
    addr_data_t d = data.read();

    if (!c.frame_ok) {
        printf("FAIL: frame_ok not passed through\n");
        return 1;
    }
    printf("PASS: frame_ok=1 addrs=%d,%d,%d,%d\n",
           (int)d.addr[0], (int)d.addr[1], (int)d.addr[2], (int)d.addr[3]);
    return 0;
}
