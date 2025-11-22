# AXI2APB
Shachar and Sahar's final project

TODO
* write a testbench
* Error handling
* RLAST handling - how do we know?
* RRESP in AXI, what is it? p.67 in spec


HOW TO RUN
1. vlogan -full64 -sverilog -kdb +v2k apb_interface.sv axi_interface.sv apb_master.sv axi_slave_rd.sv axi_slave_wr.sv disassembler.sv top_module.sv tb.sv --Analyze
2. vcs -kdb -debug_access+all -full64 tb --elaborate
3. simv -gui