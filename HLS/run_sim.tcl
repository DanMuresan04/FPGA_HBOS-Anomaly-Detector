open_project hls_proj -reset
set_top hbos_top
add_files hbos_top.cpp
add_files address_engine.cpp
add_files packet_assembler.cpp
add_files detection_engine.cpp
add_files -tb hls_tb.cpp
add_files -tb hls_test_stream.csv
open_solution "solution1"
set_part {xc7a100tcsg324-1}
create_clock -period 10 -name default
csim_design
exit
