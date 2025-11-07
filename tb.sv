//------------------------------------------------------------------------------
// File : tb.sv
// Description : Simple testbench for top_module
//------------------------------------------------------------------------------

module tb;

	// Generate clock and reset
	logic clk;
	logic reset;

	initial begin
		clk = 0;
		forever #5 clk = ~clk; // 100 MHz clock
	end

	initial begin
		reset = 1;
		#20;
		reset = 0;
	end

	// === Instantiate interfaces ===
	axi_interface axi_if();   // instance of your AXI interface
	apb_interface apb_if();   // instance of your APB interface

	// === Instantiate DUT ===
	top_module u_top (
		.clk(clk),
		.reset(reset),
		.axi(axi_if.slave),   // connect to slave modport
		.apb(apb_if.master)   // connect to master modport
	);

	// === (Optional) APB or AXI agents, monitors, stimulus ===
	// You can drive or monitor apb_if and axi_if here.

endmodule
