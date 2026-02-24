//------------------------------------------------------------------------------
// File : tb.sv
// Description : Simple testbench for top_module
//------------------------------------------------------------------------------

module tb;

	// Generate clock and reset
	logic clk;
	logic reset;

	initial begin
		clk = 1'b0;
		forever #5 clk = ~clk; // 100 MHz clock
	end

	initial begin
		reset = 1'b1;
		#20;
		reset = 1'b0;
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

	task automatic axi_write_burst(input logic [31:0] addr, input logic [31:0] id, input int beat_count, input logic [63:0] start_data);
		int i;
		logic aw_seen = 1'b0;
		@(posedge clk);
		axi_if.awaddr <= addr;
		axi_if.awid <= id;
		axi_if.awvalid <= 1'b1;
		while (!aw_seen) begin
			@(posedge clk);
			if (axi_if.awready && axi_if.awvalid) begin
				aw_seen = 1'b1;
			end
		end
		@(posedge clk);
		axi_if.awvalid <= 1'b0;
		
		for (i = 0; i < beat_count; i++) begin
			@(posedge clk);
			axi_if.wdata <= start_data + i;
			axi_if.wvalid <= 1'b1;
			axi_if.wlast <= (i == beat_count-1) ? 1'b1 : 1'b0;
			
			while(!(axi_if.wready && axi_if.wvalid)) begin
				@(posedge clk);
			end
			
			@(posedge clk);
			axi_if.wvalid <= 1'b0;
			axi_if.wlast <= 1'b0;
		end 
		
		axi_if.bready <= 1'b0;
		@(posedge clk);
		while (!axi_if.bvalid) begin
			@(posedge clk);
		end
		
		axi_if.bready <= 1'b0;
	endtask 
	
	
	task automatic axi_read_burst(input logic [31:0] addr, input logic [31:0] id, input int beat_count);
		int cnt;
		logic ar_seen;
		@(posedge clk);
		axi_if.araddr <= addr;
		axi_if.arid <= id;
		axi_if.arvalid <= 1'b1;
		ar_seen = 1'b0;
		while (!ar_seen) begin
			@(posedge clk);
			if (axi_if.arready && axi_if.arvalid) begin
				ar_seen = 1'b1;
			end
		end
		@(posedge clk);
		axi_if.arvalid <= 1'b0;
		
		axi_if.rready <= 1'b1;
		while (cnt < beat_count) begin
			@(posedge clk);
			if(axi_if.rvalid) begin
				cnt++;
			end
		end
		axi_if.rready <= 1'b0;
		
	endtask 
		
	initial begin
		axi_if.awvalid = 1'b0; axi_if.awaddr = 32'b0; axi_if.awid = 32'b0;
		axi_if.wvalid = 1'b0; axi_if.wdata = 64'b0; axi_if.wlast = 1'b0;
		axi_if.bready = 1'b0;
		axi_if.arvalid = 1'b0; axi_if.araddr = 32'b0; axi_if.arid = 32'b0;
		axi_if.rready = 1'b0;
		
		wait (reset == 0);
		@(posedge clk);
		
//		axi_write_burst(32'h000_1000, 32'h1, 4, 64'hDEAD_BEEF_0000_0000);
//		repeat(4) @(posedge clk);
		axi_read_burst(32'h0000_1000, 32'h1, 4);
		
		$finish;
		
	end
	
endmodule
