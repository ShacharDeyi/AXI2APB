# AXI2APB
Shachar and Sahar's final project

Final project in university - axi2apb bridge (verilog). 
requirments: 
•	Compatibility with AXI3 and APB4 standard
•	Features:
o	AXI data width wider or equal to APB (power of 2)
o	INCR transactions only
o	including NARROW transactions.
o	Outstanding level TBD (Default – 2 reads/ 2writes)
o	Maximum burst size - 64 bytes.
•	Clock Frequency: 150 MHz


HOW TO RUN
1. vlogan -kdb -sverilog -full64 struct_types.sv apb_interface.sv apb_master.sv axi_interface.sv axi_slave_rd.sv axi_slave_wr.sv axi_resp_builder.sv apb_req_builder.sv disassembler.sv assembler.sv manager.sv register_file.sv top_module.sv tb.sv DW_fifo_s1_sf.v DW03_reg_s_pl.v --Analyze
2. vcs -kdb -debug_access+all -full64 tb --elaborate
3. simv -gui