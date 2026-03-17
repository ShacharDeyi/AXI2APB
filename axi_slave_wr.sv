/*------------------------------------------------------------------------------
 * File          : axi_slave_wr.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 * Description   : pushes write requests
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module axi_slave_wr 
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	axi_interface.slave axi,
	
	// Ports to talk to the FIFO sitting in the Top Module
	output logic               			req_fifo_push_n,
	output struct_types::axi_wr_req 	req_fifo_data_out,
	input  logic              			req_fifo_full,
	output logic               			data_fifo_push_n,
	output struct_types::axi_wr_data 	data_fifo_data_out,
	input  logic              			data_fifo_full,
	output logic               			resp_fifo_pop_n,
	input struct_types::axi_wr_resp 	resp_fifo_data_out,
	input  logic              			resp_fifo_empty
);
/*------------------------------WRITE REQ----------------------------------*/
	// AXI Protocol Logic: Ready only if the FIFO has room
	assign axi.awready = !req_fifo_full;

	// Prepare the packet
	always_comb begin
		req_fifo_data_out.awid    = axi.awid;
		req_fifo_data_out.awaddr  = axi.awaddr;
		req_fifo_data_out.awlen   = axi.awlen;
	end

	// We push only when Valid and Ready handshake occurs
	assign req_fifo_push_n = !(axi.awvalid && axi.awready);
	
/*------------------------------WRITE DATA----------------------------------*/
	assign axi.wready = !data_fifo_full;

	// Prepare the packet
	always_comb begin
		data_fifo_data_out.wdata    = axi.wdata;
		data_fifo_data_out.wstrb  = axi.wstrb;
		data_fifo_data_out.wlast   = axi.wlast;
	end

	// We push only when Valid and Ready handshake occurs
	assign data_fifo_push_n = !(axi.wvalid && axi.wready);
	
	/*------------------------------WRITE RESP----------------------------------*/
	// AXI Protocol Logic: Ready only if the FIFO has room
	assign axi.bvalid = !resp_fifo_empty;

	// Prepare the packet
	always_comb begin
		axi.bid 	= resp_fifo_data_out.bid;
		axi.bresp 	= resp_fifo_data_out.bresp;
	end

	// We pop only when Valid and Ready handshake occurs
	assign resp_fifo_pop_n = !(axi.bvalid && axi.bready);

endmodule