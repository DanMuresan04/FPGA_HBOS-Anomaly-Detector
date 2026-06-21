# reconnect_hls_bram.tcl
# Recreates the histogram-BRAM wiring between the HLS engines (hbos_top_0,
# detection_engine_0) and the custom bram_quad_mux_0 / bram_addr_shift_* blocks.
#
# WHY THIS IS NEEDED: Vivado prunes these nets whenever the HLS IPs are upgraded
# with a changed hist port structure. A K=1 build collapses hist_0..3 into a
# single `hist` port, so upgrading the BD instance to it deletes every net that
# touched hist_0..3. Re-deploying the K=4 IP restores the ports but NOT the
# deleted nets -> bram_quad_mux/bram_addr_shift inputs dangle (BD 41-759).
#
# Run this AFTER the IPs are back at K=4 (hist_0..3, hbos_top dual-port A/B).
#
# Usage (Vivado Tcl console):
#   source /home/dan/HLS/VivadoProjects/licenta/training_engine/reconnect_hls_bram.tcl

if {[catch {current_bd_design} bd] || $bd eq ""} {
    open_bd_design [lindex [get_files demo.bd] 0]
}

set ::_made 0
set ::_skipped 0
proc _wire {a b} {
    if {[catch {connect_bd_net [get_bd_pins $a] [get_bd_pins $b]} err]} {
        puts "  skip : $a <-> $b   ($err)"
        incr ::_skipped
    } else {
        puts "  wired: $a <-> $b"
        incr ::_made
    }
}

for {set n 0} {$n < 4} {incr n} {
    # --- train path: hbos_top port A -> bram_quad_mux train_$n ---
    _wire hbos_top_0/hist_${n}_Addr_A   bram_quad_mux_0/train_${n}_ADDR
    _wire hbos_top_0/hist_${n}_Din_A    bram_quad_mux_0/train_${n}_DIN
    _wire hbos_top_0/hist_${n}_WEN_A    bram_quad_mux_0/train_${n}_WE
    _wire hbos_top_0/hist_${n}_EN_A     bram_quad_mux_0/train_${n}_EN
    _wire bram_quad_mux_0/train_${n}_DOUT hbos_top_0/hist_${n}_Dout_A

    # --- train path: hbos_top port B -> bram_addr_shift_$n ---
    _wire hbos_top_0/hist_${n}_Addr_B   bram_addr_shift_${n}/hls_addr
    _wire hbos_top_0/hist_${n}_Din_B    bram_addr_shift_${n}/hls_din
    _wire hbos_top_0/hist_${n}_WEN_B    bram_addr_shift_${n}/hls_we
    _wire hbos_top_0/hist_${n}_EN_B     bram_addr_shift_${n}/hls_en
    _wire bram_addr_shift_${n}/hls_dout hbos_top_0/hist_${n}_Dout_B

    # --- detect path: detection_engine port A -> bram_quad_mux det_$n ---
    _wire detection_engine_0/hist_${n}_Addr_A bram_quad_mux_0/det_${n}_ADDR
    _wire detection_engine_0/hist_${n}_EN_A   bram_quad_mux_0/det_${n}_EN
    _wire bram_quad_mux_0/det_${n}_DOUT       detection_engine_0/hist_${n}_Dout_A
}

puts "reconnect_hls_bram: $::_made wired, $::_skipped skipped (already-connected/missing)"
validate_bd_design
save_bd_design
puts "Done. If validation is clean, regenerate output products + re-run synthesis."
