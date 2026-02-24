/*------------------------------------------------------------------------------
 * File          : disassembler.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Oct 25, 2025
 * Description   :
 *------------------------------------------------------------------------------*/

module disassembler 
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	
	// Ports to talk to the input FIFO (from axi_slave) sitting in the Top Module
	output 	logic						req_pop_n,		
	output 	logic						wr_data_pop_n,	
	
	input 	logic						is_wr, 			// wr = 1, rd = 0
	input	logic						req_empty,		
	input	logic						wr_data_empty,	
	
	input	struct_types::axi_rd_req	rd_req_fifo_out, 
	input	struct_types::axi_wr_req	wr_req_fifo_out,
	input 	struct_types::axi_wr_data 	wr_data_fifo_out,
	
	// Ports to talk to the output FIFO (to apb_master) sitting in the Top Module
	output 	logic               		apb_req_push_n,
	output 	struct_types::apb_struct 	apb_req_fifo_in,
	input  	logic              			apb_req_full
	
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