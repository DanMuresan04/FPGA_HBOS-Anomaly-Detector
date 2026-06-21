## build_demo_bd.tcl
## Creates a fresh Vivado project "demo_conferinta" with the Stage-2 merged
## linear pipeline:
##
##   uart_rx -> uart_rx_stream -> packet_assembler -> address_engine
##           -> hbos_engine -> uart_tx_stream -> uart_tx
##
## No histogram BRAMs, no bram_quad_mux / bram_addr_shift / hist_sel_train,
## no axis_broadcaster, no config FIFO — the merged hbos_engine owns the
## histograms internally. Your working "demo" project is left untouched.
##
## Run (from a shell, NOT inside the locked demo GUI):
##   vivado -mode batch -source build_demo_bd.tcl
## or in a fresh Vivado Tcl console:
##   source /home/dan/HLS/VivadoProjects/licenta/training_engine/build_demo_bd.tcl

# ── paths / names ────────────────────────────────────────────────────────────
set proj_name   demo_conferinta
set proj_dir    /home/dan/HLS/VivadoProjects/demo_conferinta
set part        xc7a100tcsg324-1
set bd_name     demo
set ip_repo     /home/dan/HLS/IP_REPOSITORY

# Reuse the proven UART RTL + pin constraints from the existing demo project.
# The *_stream wrappers instantiate `entity work.uart_rx` / `work.uart_tx`, so the
# underlying cores (uart_rx.vhd / uart_tx.vhd) MUST be imported too or synth fails.
set src_demo    /home/dan/HLS/VivadoProjects/demo/demo.srcs
set uart_vhd    [list \
    $src_demo/sources_1/new/uart_rx.vhd \
    $src_demo/sources_1/new/uart_tx.vhd \
    $src_demo/sources_1/new/uart_rx_stream.vhd \
    $src_demo/sources_1/new/uart_tx_stream.vhd ]
set uart_xdc    $src_demo/constrs_1/new/demo_uart.xdc

if {[file exists $proj_dir]} {
    error "Project dir already exists: $proj_dir  (delete it or change \$proj_dir)"
}

# ── project ──────────────────────────────────────────────────────────────────
create_project $proj_name $proj_dir -part $part
set_property ip_repo_paths $ip_repo [current_project]
update_ip_catalog

import_files -norecurse $uart_vhd
import_files -fileset constrs_1 -norecurse $uart_xdc
update_compile_order -fileset sources_1

# ── block design ─────────────────────────────────────────────────────────────
create_bd_design $bd_name

# HLS IPs
create_bd_cell -type ip -vlnv xilinx.com:hls:packet_assembler:1.0 packet_assembler_0
create_bd_cell -type ip -vlnv xilinx.com:hls:address_engine:1.0   address_engine_0
create_bd_cell -type ip -vlnv xilinx.com:hls:hbos_engine:1.0       hbos_engine_0

# UART module_ref blocks (from the imported VHDL)
create_bd_cell -type module -reference uart_rx_stream uart_rx_stream_0
create_bd_cell -type module -reference uart_tx_stream uart_tx_stream_0

# Reset bridge. proc_sys_reset infers ext-reset polarity from the connected
# port's CONFIG.POLARITY (CPU_RESETN is ACTIVE_LOW below), so no manual param.
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0

# ── external ports ───────────────────────────────────────────────────────────
create_bd_port -dir I -type clk -freq_hz 100000000 clk_100MHz
create_bd_port -dir I -type rst CPU_RESETN
set_property CONFIG.POLARITY ACTIVE_LOW [get_bd_ports CPU_RESETN]
create_bd_port -dir I uart_rx
create_bd_port -dir O uart_tx

# ── AXIS dataflow (interface nets) ───────────────────────────────────────────
connect_bd_intf_net [get_bd_intf_pins uart_rx_stream_0/m_axis]   [get_bd_intf_pins packet_assembler_0/rx_in]
connect_bd_intf_net [get_bd_intf_pins packet_assembler_0/packet_out] [get_bd_intf_pins address_engine_0/in_stream]
connect_bd_intf_net [get_bd_intf_pins address_engine_0/out_stream]   [get_bd_intf_pins hbos_engine_0/in_stream]

# hbos_engine.anomaly_out -> uart_tx_stream.s_axis (interface-level; the master's
# extra TKEEP/TSTRB are simply left open against the uart slave).
connect_bd_intf_net [get_bd_intf_pins hbos_engine_0/anomaly_out] [get_bd_intf_pins uart_tx_stream_0/s_axis]

# ── UART pins ────────────────────────────────────────────────────────────────
connect_bd_net [get_bd_ports uart_rx] [get_bd_pins uart_rx_stream_0/uart_rx]
connect_bd_net [get_bd_ports uart_tx] [get_bd_pins uart_tx_stream_0/uart_tx]

# ── clock fan-out ────────────────────────────────────────────────────────────
connect_bd_net [get_bd_ports clk_100MHz] \
    [get_bd_pins proc_sys_reset_0/slowest_sync_clk] \
    [get_bd_pins uart_rx_stream_0/aclk] \
    [get_bd_pins uart_tx_stream_0/aclk] \
    [get_bd_pins packet_assembler_0/ap_clk] \
    [get_bd_pins address_engine_0/ap_clk] \
    [get_bd_pins hbos_engine_0/ap_clk]

# ── reset fan-out ────────────────────────────────────────────────────────────
connect_bd_net [get_bd_ports CPU_RESETN] [get_bd_pins proc_sys_reset_0/ext_reset_in]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
    [get_bd_pins uart_rx_stream_0/aresetn] \
    [get_bd_pins uart_tx_stream_0/aresetn] \
    [get_bd_pins packet_assembler_0/ap_rst_n] \
    [get_bd_pins address_engine_0/ap_rst_n] \
    [get_bd_pins hbos_engine_0/ap_rst_n]

# ── finalize ─────────────────────────────────────────────────────────────────
regenerate_bd_layout
validate_bd_design
save_bd_design

make_wrapper -files [get_files $bd_name.bd] -top
add_files -norecurse $proj_dir/$proj_name.gen/sources_1/bd/$bd_name/hdl/${bd_name}_wrapper.v
set_property top ${bd_name}_wrapper [current_fileset]
update_compile_order -fileset sources_1

puts "================================================================"
puts " demo_conferinta created at: $proj_dir"
puts " BD '$bd_name' built + validated. Top = ${bd_name}_wrapper."
puts " Next: Generate Bitstream (or launch_runs impl_1 -to_step write_bitstream)."
puts "================================================================"
