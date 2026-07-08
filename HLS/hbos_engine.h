#ifndef HBOS_ENGINE_H
#define HBOS_ENGINE_H

#include "hbos_types.h"
#include <hls_stream.h>

void hbos_engine(
    hls::stream<addr_ctrl_t>&    in_ctrl,
    hls::stream<addr_data_t>&    in_data,
    hls::stream<verdict_beat_t>& anomaly_out
);

#endif
