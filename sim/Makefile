# needs: sudo apt install iverilog gtkwave
all: sim

sim:
	iverilog -g2005 -o tb.vvp ../rtl/sdram_ctrl.v sdram_model.v tb_sdram_ctrl.v
	vvp tb.vvp | tee ../results/sim_log.txt

# lots of refreshes (every 100 cycles) to stress the refresh logic
stress:
	iverilog -g2005 -Ptb_sdram_ctrl.REF_INTERVAL=100 -o tb_stress.vvp ../rtl/sdram_ctrl.v sdram_model.v tb_sdram_ctrl.v
	vvp tb_stress.vvp | tee ../results/sim_log_stress.txt

waves:
	gtkwave ../results/waves.vcd &

clean:
	rm -f *.vvp
