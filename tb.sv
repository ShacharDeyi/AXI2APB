`timescale 1ns/1ps

/*------------------------------------------------------------------------------
 * File          : tb.sv
 * Description   : Testbench for the AXI-to-APB bridge.
 *
 * Fix: removed direct procedural/assign mixed drivers on interface signals.
 * All signals driven by the TB are now driven through intermediate logic
 * variables that are connected to the interface via continuous assign at the
 * top of the module. This gives each net exactly one driver.
 *------------------------------------------------------------------------------*/

module tb;

import struct_types::*;

/*=========================================================================*/
/*  Clock / reset                                                           */
/*=========================================================================*/
logic clk = 0;
logic rst_n;

always #5 clk = ~clk;  // 100 MHz

initial begin
	rst_n = 0;
	repeat(4) @(posedge clk);
	@(negedge clk);
	rst_n = 1;
end

/*=========================================================================*/
/*  Interface instances                                                     */
/*=========================================================================*/
axi_interface axi_if();
apb_interface apb_if();

/*=========================================================================*/
/*  Intermediate signals  TB drives these, assigns feed the interfaces    */
/*=========================================================================*/

// AXI master side (TB -> DUT)
logic                    awvalid_tb = 0;
logic [ID_WIDTH-1:0]     awid_tb    = '0;
logic [ADDR_WIDTH-1:0]   awaddr_tb  = '0;
logic [7:0]              awlen_tb   = '0;
logic                    wvalid_tb  = 0;
logic [DATA_WIDTH-1:0]   wdata_tb   = '0;
logic [WSTRB_WIDTH-1:0]  wstrb_tb   = '0;
logic                    wlast_tb   = 0;
logic                    bready_tb  = 0;
logic                    arvalid_tb = 0;
logic [ID_WIDTH-1:0]     arid_tb    = '0;
logic [ADDR_WIDTH-1:0]   araddr_tb  = '0;
logic [7:0]              arlen_tb   = '0;
logic                    rready_tb  = 0;

assign axi_if.awvalid = awvalid_tb;
assign axi_if.awid    = awid_tb;
assign axi_if.awaddr  = awaddr_tb;
assign axi_if.awlen   = awlen_tb;
assign axi_if.wvalid  = wvalid_tb;
assign axi_if.wdata   = wdata_tb;
assign axi_if.wstrb   = wstrb_tb;
assign axi_if.wlast   = wlast_tb;
assign axi_if.bready  = bready_tb;
assign axi_if.arvalid = arvalid_tb;
assign axi_if.arid    = arid_tb;
assign axi_if.araddr  = araddr_tb;
assign axi_if.arlen   = arlen_tb;
assign axi_if.rready  = rready_tb;

// APB slave side (TB -> DUT, response direction)
// No inline initializers  these are driven exclusively by the always_ff
// below. VCS forbids mixing an initializer with always_ff on the same var.
logic        pready_tb;
logic [31:0] prdata_tb;
logic        pslverr_tb;

assign apb_if.pready  = pready_tb;
assign apb_if.prdata  = prdata_tb;
assign apb_if.pslverr = pslverr_tb;

/*=========================================================================*/
/*  DUT                                                                     */
/*=========================================================================*/
top_module u_dut (
	.clk   (clk),
	.rst_n (rst_n),
	.axi   (axi_if.slave),
	.apb   (apb_if.master)
);

/*=========================================================================*/
/*  AXI master driver tasks                                                 */
/*=========================================================================*/

task automatic axi_write(
	input logic [ID_WIDTH-1:0]    id,
	input logic [ADDR_WIDTH-1:0]  addr,
	input logic [DATA_WIDTH-1:0]  data,
	input logic [WSTRB_WIDTH-1:0] strb
);
	// Write address channel
	@(negedge clk);
	awvalid_tb = 1;
	awid_tb    = id;
	awaddr_tb  = addr;
	awlen_tb   = 8'h00;
	@(posedge clk);
	while (!axi_if.awready) @(posedge clk);
	@(negedge clk);
	awvalid_tb = 0;

	// Write data channel
	wvalid_tb = 1;
	wdata_tb  = data;
	wstrb_tb  = strb;
	wlast_tb  = 1;
	@(posedge clk);
	while (!axi_if.wready) @(posedge clk);
	@(negedge clk);
	wvalid_tb = 0;
	wlast_tb  = 0;

	// Write response channel
	bready_tb = 1;
	@(posedge clk);
	while (!axi_if.bvalid) @(posedge clk);
	$display("[%0t] WRITE RESP  bid=%0h  bresp=%0b  (%s)",
			 $time, axi_if.bid, axi_if.bresp,
			 (axi_if.bresp == 2'b00) ? "OKAY" : "SLVERR");
	@(negedge clk);
	bready_tb = 0;
endtask

task automatic axi_read(
	input logic [ID_WIDTH-1:0]   id,
	input logic [ADDR_WIDTH-1:0] addr
);
	// Read address channel
	@(negedge clk);
	arvalid_tb = 1;
	arid_tb    = id;
	araddr_tb  = addr;
	arlen_tb   = 8'h00;
	@(posedge clk);
	while (!axi_if.arready) @(posedge clk);
	@(negedge clk);
	arvalid_tb = 0;

	// Read data channel
	rready_tb = 1;
	@(posedge clk);
	while (!axi_if.rvalid) @(posedge clk);
	$display("[%0t] READ  DATA   rid=%0h  rdata=%0h  rresp=%0b  rlast=%0b  (%s)",
			 $time, axi_if.rid, axi_if.rdata, axi_if.rresp, axi_if.rlast,
			 (axi_if.rresp == 2'b00) ? "OKAY" : "SLVERR");

	if (axi_if.rdata === 64'hDEAD_BEEF_CAFE_1234)
		$display("[%0t] CHECK PASS: rdata matches 0xDEAD_BEEF_CAFE_1234", $time);
	else
		$display("[%0t] CHECK FAIL: rdata=%0h, expected 0xDEAD_BEEF_CAFE_1234",
				 $time, axi_if.rdata);

	@(negedge clk);
	rready_tb = 0;
endtask

/*=========================================================================*/
/*  Inline APB slave model                                                  */
/*=========================================================================*/
// apb_mem has no inline initializer  it is written only by the always_ff
// below (both in the reset branch and during normal operation).
logic [31:0] apb_mem [0:3];

always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		pready_tb  <= 0;
		prdata_tb  <= 0;
		pslverr_tb <= 0;
		foreach (apb_mem[i]) apb_mem[i] <= 32'hDEAD_DEAD;
	end else begin
		pready_tb  <= 0;  // default deassert

		if (apb_if.psel && apb_if.penable && !pready_tb) begin
			pready_tb  <= 1;
			pslverr_tb <= 0;

			if (apb_if.pwrite) begin
				for (int b = 0; b < 4; b++)
					if (apb_if.pstrb[b])
						apb_mem[(apb_if.paddr >> 2) & 3][b*8 +: 8] <=
							apb_if.pwdata[b*8 +: 8];
				$display("[%0t] APB WRITE  paddr=0x%0h  pwdata=0x%0h  pstrb=%0b",
						 $time, apb_if.paddr, apb_if.pwdata, apb_if.pstrb);
			end else begin
				prdata_tb <= apb_mem[(apb_if.paddr >> 2) & 3];
				$display("[%0t] APB READ   paddr=0x%0h  prdata=0x%0h",
						 $time, apb_if.paddr,
						 apb_mem[(apb_if.paddr >> 2) & 3]);
			end
		end
	end
end

/*=========================================================================*/
/*  Stimulus                                                                */
/*=========================================================================*/
initial begin
	$dumpfile("tb.vcd");
	$dumpvars(0, tb);

	@(posedge rst_n);
	repeat(2) @(posedge clk);

	$display("\n=== AXI WRITE: addr=0x1000, data=0xDEAD_BEEF_CAFE_1234 ===");
	axi_write(32'h1, 32'h0000_1000, 64'hDEAD_BEEF_CAFE_1234, 8'hFF);

	repeat(4) @(posedge clk);

	$display("\n=== AXI READ: addr=0x1000 ===");
	axi_read(32'h2, 32'h0000_1000);

	repeat(8) @(posedge clk);
	$display("\n=== Simulation complete ===");
	$finish;
end

initial begin
	#100_000;
	$display("TIMEOUT");
	$finish;
end

endmodule