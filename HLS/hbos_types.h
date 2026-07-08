#ifndef HBOS_TYPES_H
#define HBOS_TYPES_H

#include <ap_fixed.h>
#include <ap_int.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>

#define NR_BINS       2048
#define NR_DELTA_BINS 256
#define NR_SENSORS    4

#define DELTA_MAX_STRIDE 8

#define NR_CFG_WORDS 7
#define PKT_WORDS    (NR_SENSORS > NR_CFG_WORDS ? NR_SENSORS : NR_CFG_WORDS)

#ifndef K_PARALLEL
#define K_PARALLEL 4
#endif

#define H_EXP_BITS  5
#define H_MANT_BITS 5
#define D_EXP_BITS  5
#define D_MANT_BITS 3

typedef ap_int<32>  sensor_t;
typedef ap_uint<16> hbos_score_t;
typedef ap_uint<26> total_score_t;
typedef ap_uint<16> count_t;
typedef ap_uint<11> bin_addr_t;
typedef ap_uint<8>  delta_addr_t;
typedef ap_uint<4>  opcode_t;
typedef ap_uint<8>  weight_t;
typedef ap_uint<16> spike_t;

#define DDR_WORD_BITS 128
typedef ap_uint<DDR_WORD_BITS> ddr_word_t;

#define OP_TRAIN  0
#define OP_CALIB  1
#define OP_DETECT 2
#define OP_DUMP   3
#define OP_CONFIG 4
#define OP_RESET  5

#define OP_LOAD_TRAIN  6
#define OP_LOAD_TEST   7
#define OP_LOAD_STATUS 8

#define FRAME_MAGIC_LO 0xA5
#define FRAME_MAGIC_HI 0x5A

#define DMA_DATA_MAGIC   0xDA
#define DMA_STATUS_MAGIC 0xB5
#define DMA_MAX_B        90
#define DMA_MAX_BLOCKS   16384
#define DMA_BITMAP_WORDS (DMA_MAX_BLOCKS / 32)
#define DMA_MISS_CAP     400
#define DMA_MISS_NO_SESSION 0x7FFF

struct sensor_config_t {
    sensor_t     center;
    delta_addr_t delta_th;
    sensor_t     prev_val;
};

struct sensor_packet_t {
    sensor_t    data[PKT_WORDS];
    opcode_t    opcode;
    bool        tlast;
    ap_uint<5>  active_count;
    ap_uint<24> seq;
    ap_uint<16> reserve;
    bool        frame_ok;
};

typedef ap_axiu<8, 0, 0, 0> anomaly_packet_t;
typedef ap_axiu<8, 0, 0, 0> rx_byte_axis_t;

typedef ap_axiu<32, 0, 0, 0> verdict_beat_t;

struct addr_packet_t {
    bin_addr_t   addr[NR_SENSORS];
    delta_addr_t d_addr[NR_SENSORS];
    opcode_t     opcode;
    bool         tlast;
    ap_uint<5>   active_count;
    ap_uint<24>  seq;
    bool         frame_ok;
};

struct addr_ctrl_t {
    opcode_t    opcode;
    bool        tlast;
    ap_uint<5>  active_count;
    ap_uint<24> seq;
    bool        frame_ok;
};

struct addr_data_t {
    bin_addr_t   addr[NR_SENSORS];
    delta_addr_t d_addr[NR_SENSORS];
};

void packet_assembler(
    hls::stream<rx_byte_axis_t>&  rx_in,
    hls::stream<sensor_packet_t>& packet_out
);

void address_engine(
    hls::stream<sensor_packet_t>& in_stream,
    hls::stream<addr_ctrl_t>&     out_ctrl,
    hls::stream<addr_data_t>&     out_data
);

void dataset_dma(
    hls::stream<sensor_packet_t>&  in_stream,
    hls::stream<rx_byte_axis_t>&   bulk_in,
    hls::stream<sensor_packet_t>&  out_stream,
    hls::stream<anomaly_packet_t>& status_out,
    ddr_word_t*                    mem
);

void detection_engine(
    hls::stream<addr_packet_t>&    in_stream,
    hls::stream<ap_uint<32>>&      config_in,
    count_t hist[NR_SENSORS][NR_BINS],
    hls::stream<anomaly_packet_t>& anomaly_out
);

#endif
