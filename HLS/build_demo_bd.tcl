## build_demo_bd.tcl
## Creates a fresh Vivado project "demo_conferinta" with the merged linear
## pipeline, DDR2 dataset staging (DMA), and UDP transport (LiteEth RMII core,
## replaces the UART — see licenta/eth_core/UDP_MIGRATION_PLAN.md):
##
##   eth(:1234 ctrl) -> bridge.m_ctrl -> rx_fifo -> packet_assembler -> dataset_dma
##                   -> address_engine -> hbos_engine -> tx_fifo ----\
##   eth(:1235 data) -> bridge.m_bulk -> dataset_dma.bulk_in          |
##   dataset_dma.status_out -> status_fifo ------------> udp_tx_packetizer
##                             (dst = sender latched by the eth core) |
##   eth TX <- bridge.s_tx <--------------------------------------- m_data
##
##   dataset_dma.m_axi -> axi_smartconnect -> mig_7series.S_AXI -> DDR2 (128 MiB)
##
## dataset_dma stages each dataset in DDR2 once (OP_LOAD_TRAIN/OP_LOAD_TEST) and
## replays it on command (OP_TRAIN/OP_CALIB/OP_DETECT) at engine clock. No CPU:
## everything is opcode-driven (ap_ctrl_none). The MIG (Micron MT47H64M16, the
## Nexys A7 DDR2) is configured from a saved .prj; its 200 MHz IDELAYCTRL ref and
## a clean 100 MHz sys clock come from a clk_wiz off the board's 100 MHz. The user
## logic is held in reset until the MIG asserts init_calib_complete.
##
## Run (NOT inside a locked GUI):
##   vivado -mode batch -source build_demo_bd.tcl

# ── paths / names ────────────────────────────────────────────────────────────
# All repo-relative paths are derived from this script's own location, so the
# build works from any checkout without editing paths by hand.
set script_dir       [file dirname [file normalize [info script]]]
set licenta_root      [file normalize [file join $script_dir ".."]]
set vivado_projects_root [file dirname $licenta_root]

set proj_name   demo_conferinta
# Output dir is overridable so a verification build can target a throwaway path
# without clobbering an existing project:  set ::env(DEMO_PROJ_DIR) /tmp/foo
if {[info exists ::env(DEMO_PROJ_DIR)]} {
    set proj_dir $::env(DEMO_PROJ_DIR)
} else {
    set proj_dir [file join $vivado_projects_root demo_conferinta]
}
set part        xc7a100tcsg324-1
set bd_name     demo
# IP_REPOSITORY lives outside the repo (built IP zips); override with
# ::env(IP_REPOSITORY) if it is not a sibling of VivadoProjects/.
if {[info exists ::env(IP_REPOSITORY)]} {
    set ip_repo $::env(IP_REPOSITORY)
} else {
    set ip_repo [file join [file dirname $vivado_projects_root] IP_REPOSITORY]
}
set mig_prj     [file join $licenta_root HLS mig nexys_a7_ddr2.prj]
set timing_xdc  [file join $licenta_root HLS demo_timing.xdc]
set pins_xdc    [file join $licenta_root HLS demo_pins.xdc]
set eth_xdc     [file join $licenta_root HLS demo_eth.xdc]

# LiteEth UDP core (regenerated with :1234 ctrl + :1235 data + sender latch)
# and the raw-wire<->AXIS bridge.
set eth_dir     [file join $licenta_root eth_core]
set eth_rtl     [list \
    $eth_dir/liteeth_rmii_core.v \
    $eth_dir/udp_axis_bridge.v ]

if {[file exists $proj_dir]} {
    error "Project dir already exists: $proj_dir  (delete it or change \$proj_dir)"
}

# ── project ──────────────────────────────────────────────────────────────────
create_project $proj_name $proj_dir -part $part
set_property ip_repo_paths $ip_repo [current_project]
update_ip_catalog

import_files -norecurse $eth_rtl
import_files -fileset constrs_1 -norecurse $eth_xdc
# Async clock-group constraint — implementation-only (the generated clock names
# it references don't exist until the clocks are propagated at impl).
import_files -fileset constrs_1 -norecurse $timing_xdc
set_property used_in_synthesis false [get_files demo_timing.xdc]
# Pin LOC/IOSTANDARD for demo_conferinta-only ports (ddr2_calib_done LED).
import_files -fileset constrs_1 -norecurse $pins_xdc
update_compile_order -fileset sources_1

# ── block design ─────────────────────────────────────────────────────────────
create_bd_design $bd_name

# HLS IPs (engine pipeline + DDR2 stager + UDP TX packetizer)
create_bd_cell -type ip -vlnv xilinx.com:hls:packet_assembler:1.0  packet_assembler_0
create_bd_cell -type ip -vlnv xilinx.com:hls:dataset_dma:1.0        dataset_dma_0
create_bd_cell -type ip -vlnv xilinx.com:hls:address_engine:1.0     address_engine_0
create_bd_cell -type ip -vlnv xilinx.com:hls:hbos_engine:1.0        hbos_engine_0
create_bd_cell -type ip -vlnv xilinx.com:hls:udp_tx_packetizer:1.0  udp_tx_packetizer_0

# LiteEth UDP core + raw-wire<->AXIS bridge (module refs from imported RTL)
create_bd_cell -type module -reference liteeth_rmii_core liteeth_rmii_core_0
create_bd_cell -type module -reference udp_axis_bridge   udp_axis_bridge_0

# Reset bridge for the 100 MHz user-logic domain.
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0
# Reset bridge for the 50 MHz eth reference domain (active-high into the core).
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_eth_50MHz

# AXIS data FIFOs decouple the transport from the engine. rx_fifo absorbs
# control bytes while the engine emits; tx_fifo absorbs verdict beats (one
# 32-bit TKEEP-marked beat per verdict since the II=1 widening; TDATA width
# propagates from the pins) so the engine never blocks; status_fifo holds a
# full LOAD_STATUS reply (<=1205 B) so dataset_dma never stalls on a busy
# packetizer.
create_bd_cell -type ip -vlnv xilinx.com:ip:axis_data_fifo:2.0 rx_fifo
set_property CONFIG.FIFO_DEPTH {4096} [get_bd_cells rx_fifo]
create_bd_cell -type ip -vlnv xilinx.com:ip:axis_data_fifo:2.0 tx_fifo
set_property CONFIG.FIFO_DEPTH {1024} [get_bd_cells tx_fifo]
create_bd_cell -type ip -vlnv xilinx.com:ip:axis_data_fifo:2.0 status_fifo
set_property CONFIG.FIFO_DEPTH {2048} [get_bd_cells status_fifo]

# ── DDR2 subsystem ───────────────────────────────────────────────────────────
# clk_wiz: board 100 MHz -> 100 MHz (sys + user logic), 200 MHz (MIG ref clk),
# 50 MHz 0deg (RMII reference, forwarded to the PHY) and 50 MHz 90deg (RMII RX
# capture — centres the sample in the RXD eye; see eth_core/build_rmii.py).
create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz:6.0 clk_wiz_0
set_property -dict [list \
    CONFIG.PRIM_IN_FREQ {100.000} \
    CONFIG.CLKOUT2_USED {true} \
    CONFIG.CLKOUT3_USED {true} \
    CONFIG.CLKOUT4_USED {true} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {100.000} \
    CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {200.000} \
    CONFIG.CLKOUT3_REQUESTED_OUT_FREQ {50.000} \
    CONFIG.CLKOUT4_REQUESTED_OUT_FREQ {50.000} \
    CONFIG.CLKOUT4_REQUESTED_PHASE {90.000} \
    CONFIG.RESET_TYPE {ACTIVE_LOW} \
    CONFIG.USE_LOCKED {true} \
] [get_bd_cells clk_wiz_0]

# MIG 7-series DDR2 controller, configured from the saved Nexys A7 .prj (it
# carries the full DDR2 pinout, so no board part is needed). System + reference
# clocks are "No Buffer" -> driven from clk_wiz internal nets.
create_bd_cell -type ip -vlnv xilinx.com:ip:mig_7series:4.2 mig_7series_0
set mig_ipdir [get_property IP_DIR [get_ips [get_property CONFIG.Component_Name [get_bd_cells mig_7series_0]]]]
file copy -force $mig_prj ${mig_ipdir}/nexys_a7_ddr2.prj
set_property -dict [list \
    CONFIG.MIG_DONT_TOUCH_PARAM {Custom} \
    CONFIG.RESET_BOARD_INTERFACE {Custom} \
    CONFIG.XML_INPUT_FILE {nexys_a7_ddr2.prj} \
] [get_bd_cells mig_7series_0]

# SmartConnect bridges the 512-bit/100 MHz HLS master to the 128-bit/ui_clk MIG
# slave: it handles BOTH the data-width conversion and the clock crossing.
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 axi_smc
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1} CONFIG.NUM_CLKS {2}] [get_bd_cells axi_smc]

# Reset bridge for the MIG's ui_clk domain.
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 mig_sys_reset

# "System ready" gate: clk_wiz locked AND DDR2 init_calib_complete. Fed into
# proc_sys_reset's dcm_locked (see reset section) so the user-logic reset both
# waits for DDR2 calibration AND is synchronized into the 100 MHz domain by
# proc_sys_reset — no cross-domain combinational reset fan-out to mis-time.
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic:2.0 calib_gate
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {and}] [get_bd_cells calib_gate]

# ── external ports ───────────────────────────────────────────────────────────
create_bd_port -dir I -type clk -freq_hz 100000000 clk_100MHz
create_bd_port -dir I -type rst CPU_RESETN
set_property CONFIG.POLARITY ACTIVE_LOW [get_bd_ports CPU_RESETN]
create_bd_port -dir O ddr2_calib_done
# RMII PHY pins
create_bd_port -dir O ETH_REFCLK
create_bd_port -dir O ETH_RSTN
create_bd_port -dir O ETH_TXEN
create_bd_port -dir O -from 1 -to 0 ETH_TXD
create_bd_port -dir I ETH_CRSDV
create_bd_port -dir I -from 1 -to 0 ETH_RXD
# DDR2 physical bus -> external (auto-typed from the MIG interface).
make_bd_intf_pins_external [get_bd_intf_pins mig_7series_0/DDR2]

# ── AXIS dataflow ────────────────────────────────────────────────────────────
# eth ctrl -> bridge -> rx_fifo -> packet_assembler -> dataset_dma
#          -> address_engine -> hbos_engine -> tx_fifo -> packetizer -> eth tx
# eth data -> bridge -> dataset_dma.bulk_in ; dataset_dma.status_out
#          -> status_fifo -> packetizer
connect_bd_intf_net [get_bd_intf_pins udp_axis_bridge_0/m_ctrl]       [get_bd_intf_pins rx_fifo/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins rx_fifo/M_AXIS]                [get_bd_intf_pins packet_assembler_0/rx_in]
connect_bd_intf_net [get_bd_intf_pins packet_assembler_0/packet_out] [get_bd_intf_pins dataset_dma_0/in_stream]
connect_bd_intf_net [get_bd_intf_pins udp_axis_bridge_0/m_bulk]      [get_bd_intf_pins dataset_dma_0/bulk_in]
connect_bd_intf_net [get_bd_intf_pins dataset_dma_0/out_stream]      [get_bd_intf_pins address_engine_0/in_stream]
connect_bd_intf_net [get_bd_intf_pins dataset_dma_0/status_out]      [get_bd_intf_pins status_fifo/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins address_engine_0/out_ctrl]     [get_bd_intf_pins hbos_engine_0/in_ctrl]
connect_bd_intf_net [get_bd_intf_pins address_engine_0/out_data]     [get_bd_intf_pins hbos_engine_0/in_data]
connect_bd_intf_net [get_bd_intf_pins hbos_engine_0/anomaly_out]     [get_bd_intf_pins tx_fifo/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins status_fifo/M_AXIS]            [get_bd_intf_pins udp_tx_packetizer_0/s_dma]
connect_bd_intf_net [get_bd_intf_pins tx_fifo/M_AXIS]                [get_bd_intf_pins udp_tx_packetizer_0/s_eng]
connect_bd_intf_net [get_bd_intf_pins udp_tx_packetizer_0/m_data]    [get_bd_intf_pins udp_axis_bridge_0/s_tx]

# ── DDR2 datapath ────────────────────────────────────────────────────────────
connect_bd_intf_net [get_bd_intf_pins dataset_dma_0/m_axi_gmem] [get_bd_intf_pins axi_smc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_smc/M00_AXI]          [get_bd_intf_pins mig_7series_0/S_AXI]

# ── eth core raw wiring ──────────────────────────────────────────────────────
# bridge <-> core streams
foreach {a b} {
    rx_valid rx_valid   rx_ready rx_ready   rx_last rx_last   rx_data rx_data
    rx2_valid rx2_valid rx2_ready rx2_ready rx2_last rx2_last rx2_data rx2_data
    tx_valid tx_valid   tx_ready tx_ready   tx_last tx_last   tx_data tx_data
} {
    connect_bd_net [get_bd_pins liteeth_rmii_core_0/$a] [get_bd_pins udp_axis_bridge_0/$b]
}
# packetizer datagram metadata -> core TX; latched sender -> packetizer dst
connect_bd_net [get_bd_pins udp_tx_packetizer_0/tx_length]   [get_bd_pins liteeth_rmii_core_0/tx_length]
connect_bd_net [get_bd_pins udp_tx_packetizer_0/tx_ip]       [get_bd_pins liteeth_rmii_core_0/tx_ip_address]
connect_bd_net [get_bd_pins udp_tx_packetizer_0/tx_dst_port] [get_bd_pins liteeth_rmii_core_0/tx_dst_port]
connect_bd_net [get_bd_pins udp_tx_packetizer_0/tx_src_port] [get_bd_pins liteeth_rmii_core_0/tx_src_port]
connect_bd_net [get_bd_pins liteeth_rmii_core_0/last_src_ip]   [get_bd_pins udp_tx_packetizer_0/dst_ip]
connect_bd_net [get_bd_pins liteeth_rmii_core_0/last_src_port] [get_bd_pins udp_tx_packetizer_0/dst_port]

# ── RMII PHY pins ────────────────────────────────────────────────────────────
connect_bd_net [get_bd_pins liteeth_rmii_core_0/eth_ref_clk] [get_bd_ports ETH_REFCLK]
connect_bd_net [get_bd_pins liteeth_rmii_core_0/eth_rst_n]   [get_bd_ports ETH_RSTN]
connect_bd_net [get_bd_pins liteeth_rmii_core_0/eth_tx_en]   [get_bd_ports ETH_TXEN]
connect_bd_net [get_bd_pins liteeth_rmii_core_0/eth_tx_data] [get_bd_ports ETH_TXD]
connect_bd_net [get_bd_ports ETH_CRSDV] [get_bd_pins liteeth_rmii_core_0/eth_crs_dv]
connect_bd_net [get_bd_ports ETH_RXD]   [get_bd_pins liteeth_rmii_core_0/eth_rx_data]

# ── clocking ─────────────────────────────────────────────────────────────────
connect_bd_net [get_bd_ports clk_100MHz] [get_bd_pins clk_wiz_0/clk_in1]

# clk_out1 (100 MHz): user-logic + eth core sys domain + MIG sys clock +
# SmartConnect primary clock. The LiteEth UDP user ports run in this same
# domain (clk_freq=100e6 in build_rmii.py) — no CDC toward the pipeline.
connect_bd_net [get_bd_pins clk_wiz_0/clk_out1] \
    [get_bd_pins proc_sys_reset_0/slowest_sync_clk] \
    [get_bd_pins liteeth_rmii_core_0/sys_clk] \
    [get_bd_pins udp_axis_bridge_0/aclk] \
    [get_bd_pins packet_assembler_0/ap_clk] \
    [get_bd_pins dataset_dma_0/ap_clk] \
    [get_bd_pins address_engine_0/ap_clk] \
    [get_bd_pins hbos_engine_0/ap_clk] \
    [get_bd_pins udp_tx_packetizer_0/ap_clk] \
    [get_bd_pins rx_fifo/s_axis_aclk] \
    [get_bd_pins tx_fifo/s_axis_aclk] \
    [get_bd_pins status_fifo/s_axis_aclk] \
    [get_bd_pins axi_smc/aclk] \
    [get_bd_pins mig_7series_0/sys_clk_i]

# clk_out2 (200 MHz): MIG IDELAYCTRL reference.
connect_bd_net [get_bd_pins clk_wiz_0/clk_out2] [get_bd_pins mig_7series_0/clk_ref_i]

# clk_out3 (50 MHz 0deg): RMII reference domain (also forwarded to the PHY pin
# via the core's ODDR). clk_out4 (50 MHz 90deg): RMII RX capture.
connect_bd_net [get_bd_pins clk_wiz_0/clk_out3] \
    [get_bd_pins liteeth_rmii_core_0/eth_clk] \
    [get_bd_pins rst_eth_50MHz/slowest_sync_clk]
connect_bd_net [get_bd_pins clk_wiz_0/clk_out4] [get_bd_pins liteeth_rmii_core_0/eth_rx_clk]

# ui_clk (~81 MHz): SmartConnect secondary clock + MIG-domain reset.
connect_bd_net [get_bd_pins mig_7series_0/ui_clk] \
    [get_bd_pins axi_smc/aclk1] \
    [get_bd_pins mig_sys_reset/slowest_sync_clk]
connect_bd_net [get_bd_pins mig_7series_0/ui_clk_sync_rst] [get_bd_pins mig_sys_reset/ext_reset_in]

# ── resets ───────────────────────────────────────────────────────────────────
connect_bd_net [get_bd_ports CPU_RESETN] \
    [get_bd_pins proc_sys_reset_0/ext_reset_in] \
    [get_bd_pins rst_eth_50MHz/ext_reset_in] \
    [get_bd_pins clk_wiz_0/resetn]

# clk_wiz lock: gates the MIG-domain reset directly, drives the MIG's own
# sys_rst (see below), and is one input of the "system ready" gate for the
# user domain.
connect_bd_net [get_bd_pins clk_wiz_0/locked] \
    [get_bd_pins mig_sys_reset/dcm_locked] \
    [get_bd_pins rst_eth_50MHz/dcm_locked] \
    [get_bd_pins calib_gate/Op1] \
    [get_bd_pins mig_7series_0/sys_rst]

# eth-domain reset (active high into the core's 50 MHz domain).
connect_bd_net [get_bd_pins rst_eth_50MHz/peripheral_reset] \
    [get_bd_pins liteeth_rmii_core_0/eth_rst]

# System ready = locked AND init_calib_complete, fed into proc_sys_reset/dcm_locked.
# proc_sys_reset SYNCHRONIZES this ui_clk-domain signal into the 100 MHz domain and
# only deasserts the user reset once DDR2 is calibrated — so there is no cross-domain
# combinational reset fan-out (the path that broke timing). Also drives a status LED.
connect_bd_net [get_bd_pins mig_7series_0/init_calib_complete] \
    [get_bd_pins calib_gate/Op2] \
    [get_bd_ports ddr2_calib_done]
connect_bd_net [get_bd_pins calib_gate/Res] [get_bd_pins proc_sys_reset_0/dcm_locked]

# MIG S_AXI interface reset (ui_clk domain) — aresetn ONLY.
#
# sys_rst must NOT be driven from here: mig_sys_reset is clocked by the MIG's own
# ui_clk, which is stopped while sys_rst is asserted. Feeding sys_rst from this
# synchronizer deadlocks at power-up (ui_clk never starts -> reset never lifts ->
# MIG never calibrates -> init_calib_complete stays low). sys_rst is instead
# driven by clk_wiz/locked above (async ACTIVE-LOW reset, released once the
# sys/ref clocks are stable). aresetn stays here: it is the AXI reset for the
# already-running ui_clk domain, which is exactly what this synchronizer provides.
connect_bd_net [get_bd_pins mig_sys_reset/peripheral_aresetn] \
    [get_bd_pins mig_7series_0/aresetn]

# Single synchronous active-low reset for the whole 100 MHz user-logic cluster.
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
    [get_bd_pins packet_assembler_0/ap_rst_n] \
    [get_bd_pins dataset_dma_0/ap_rst_n] \
    [get_bd_pins address_engine_0/ap_rst_n] \
    [get_bd_pins hbos_engine_0/ap_rst_n] \
    [get_bd_pins udp_tx_packetizer_0/ap_rst_n] \
    [get_bd_pins rx_fifo/s_axis_aresetn] \
    [get_bd_pins tx_fifo/s_axis_aresetn] \
    [get_bd_pins status_fifo/s_axis_aresetn] \
    [get_bd_pins axi_smc/aresetn]

# eth core sys-domain reset is ACTIVE HIGH and, like the rest of the user
# logic, waits for DDR2 calibration (peripheral_reset = !peripheral_aresetn).
# Early host packets during calibration are simply lost; the host's
# status/echo retries cover bring-up.
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_reset] \
    [get_bd_pins liteeth_rmii_core_0/sys_rst]

# ── address map ──────────────────────────────────────────────────────────────
# dataset_dma issues byte addresses index*64 from 0 (m_axi offset=off), so the
# DDR2 segment must sit at 0x0.
assign_bd_address -offset 0x00000000 -range 0x08000000 \
    -target_address_space [get_bd_addr_spaces dataset_dma_0/Data_m_axi_gmem] \
    [get_bd_addr_segs mig_7series_0/memmap/memaddr] -force

# ── finalize ─────────────────────────────────────────────────────────────────
regenerate_bd_layout
validate_bd_design
save_bd_design

make_wrapper -files [get_files $bd_name.bd] -top
add_files -norecurse $proj_dir/$proj_name.gen/sources_1/bd/$bd_name/hdl/${bd_name}_wrapper.v
set_property top ${bd_name}_wrapper [current_fileset]
update_compile_order -fileset sources_1

# Enable physical optimization in implementation — replicates the high-fanout
# register-slice control nets on the wide (~570-bit) AXIS buses, which is the
# residual intra-100 MHz timing pressure around dataset_dma.
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]

puts "================================================================"
puts " demo_conferinta created at: $proj_dir"
puts " BD '$bd_name' built + validated (engine pipeline + DDR2 staging)."
puts " Top = ${bd_name}_wrapper."
puts " Next: Generate Bitstream (or launch_runs impl_1 -to_step write_bitstream)."
puts "================================================================"
