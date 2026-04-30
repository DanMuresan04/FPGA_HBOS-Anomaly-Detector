#include "hbos_types.h"
#include <hls_stream.h>

struct sensor_packet_t{
    sensor_t data[NR_SENSORS];
    bool tlast;
};

void find_boundaries(
    hls::stream<sensor_packet_t> &input_stream,
    sensor_config_t[NR_SENSORS],
    sensor_t 
)