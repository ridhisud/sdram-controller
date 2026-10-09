# change the part to your board's FPGA (xc7a35tcpg236-1 = Basys3 Artix-7)
set part xc7a35tcpg236-1
read_verilog ../rtl/sdram_ctrl.v
synth_design -top sdram_ctrl -part $part -mode out_of_context
create_clock -period 10.000 -name clk [get_ports clk]
opt_design
place_design
route_design
report_timing_summary -file ../results/vivado_timing_summary.txt
report_timing -max_paths 3 -file ../results/vivado_critical_paths.txt
report_utilization -file ../results/vivado_utilization.txt
puts "DONE"
