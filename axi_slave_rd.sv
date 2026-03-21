/*------------------------------------------------------------------------------
 * File          : axi_slave_rd.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 * Description   :
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module axi_slave_rd 
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	axi_interface.slave axi,
	
	// Ports to talk to the FIFO sitting in the Top Module
	output logic               		req_fifo_push_n,
	output struct_types::axi_rd_req req_fifo_data_out,
	input  logic              		req_fifo_full,
	output logic               		data_fifo_pop_n,
	input struct_types::axi_rd_data data_fifo_data_in,
	input  logic              		data_fifo_empty
);
/*------------------------------READ REQ----------------------------------*/
	// AXI Protocol Logic: Ready only if the FIFO has room
	assign axi.arready = !req_fifo_full;

	// Prepare the packet
	always_comb begin
		req_fifo_data_out.arid    = axi.arid;
		req_fifo_data_out.araddr  = axi.araddr;
		req_fifo_data_out.arlen   = axi.arlen;
		req_fifo_data_out.arsize  = axi.arsize;
	end

	// We push only when Valid and Ready handshake occurs
	assign req_fifo_push_n = !(axi.arvalid && axi.arready);
	
	
/*------------------------------READ DATA----------------------------------*/	
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