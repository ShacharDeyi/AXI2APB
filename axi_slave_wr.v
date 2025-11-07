/*------------------------------------------------------------------------------
 * File          : top_module.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Jul 5, 2025
 * Description   :
 *------------------------------------------------------------------------------*/

module top_module #() (
	input logic clk,
	input logic reset,
	
	// Interface declarations using your specified syntax
	axi_interface.slave  axi,
	apb_interface.master apb
	
);

apb_master u_apb_master (
	.clk(clk),
	.rst_n(~reset),
	.apb(apb),
	// Connect dummy signals for now
	.req_valid(1'b0),
	.req_addr(64'h0),
	.req_wdata(32'h0),
	.req_write(1'b0),
	.req_ready(),
	.rsp_rdata(),
	.rsp_valid(),
	.rsp_error()
  );

endmodule : top_module
