/*------------------------------------------------------------------------------
 * File          : assembler.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 24, 2026
 * Description   :
 *------------------------------------------------------------------------------*/

module assembler 
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	
	// Ports to talk to the input FIFO (from apb_slave) sitting in the Top Module
	output 	logic						apb_data_pop_n,	
	input	logic						apb_resp_empty,	
	input	struct_types::apb_struct	apb_resp_fifo_out, 

	
	// Ports to talk to the output FIFO (to axi_master) sitting in the Top Module
	output 	logic               		resp_push_n,
	output 	struct_types::axi_rd_data 	rd_data_fifo_in,
	output 	struct_types::axi_wr_resp 	wr_resp_fifo_in,
	input  	logic              			rd_data_full,
	input  	logic              			wr_resp_full
	
	
);
/*------------------------------INPUT----------------------------------*/
	// AXI Protocol Logic: Ready only if the FIFO has room
	assign axi.arready = !req_fifo_full;

	// Prepare the packet
	always_comb begin
		req_fifo_data_out.arid    = axi.arid;
		req_fifo_data_out.araddr  = axi.araddr;
		req_fifo_data_out.arlen   = axi.arlen;
	end

	// We push only when Valid and Ready handshake occurs
	assign req_fifo_push_n = !(axi.arvalid && axi.arready);
	
	
/*------------------------------OUTPUT----------------------------------*/	
	// AXI Protocol Logic: Ready only if the FIFO has room
	assign axi.rvalid = !data_fifo_empty;

	// Prepare the packet
	always_comb begin
		axi.rid = data_fifo_data_in.rid;
		axi.rdata = data_fifo_data_in.rdata;
		axi.rresp = data_fifo_data_in.rresp;
		axi.rlast = data_fifo_data_in.rlast;
	end

	// We push only when Valid and Ready handshake occurs
	assign data_fifo_pop_n = !(axi.rvalid && axi.rready);

endmodule