# FPGA HBOS Anomaly Detector

HBOS (Histogram-Based Outlier Score) anomaly-detection pipeline for the
Nexys A7, plus its host-side tooling. This repository holds the curated
sources only; Vivado/HLS build workspaces, datasets, and generated artifacts
are not tracked.

## Layout

- **`HLS/`** — Vitis HLS C++ sources for the detection pipeline
  (`address_engine`, `detection_engine`, `packet_assembler`, `hbos_top`),
  their headers, testbenches, and build/sim config (`hls_config.cfg`,
  `run_csim.sh`, `run_sim.tcl`, `sim.py`, `vitis-comp.json`).
- **`VHDL/`** — hand-written RTL (`bram_addr_shift`, `bram_quad_mux_infer`).
- **`software/`** — Python host tooling: the live stream viewer GUI
  (`stream_viewer.py`), transports (`uart_client.py`, `fpga_client.py`,
  `mock_client.py`), training/comparison helpers, and tests.
