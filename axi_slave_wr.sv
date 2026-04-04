/*------------------------------------------------------------------------------
 * File          : axi_slave_wr.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 * Description   : AXI write-side interface adapter.
 *
 *   Bridges the three AXI write channels (AW, W, B) to the FIFO interfaces
 *   used by the rest of the bridge:
 *
 *     AW channel : accepts write addresses; pushes axi_wr_req structs into
 *                  the write-request FIFO. awready is driven from !full.
 *
 *     W channel  : accepts write data beats; pushes axi_wr_data structs into
 *                  the write-data FIFO. wready is driven from !full.
 *
 *     B channel  : drives write responses back to the AXI master from the
 *                  write-response FIFO. bvalid is driven from !empty;
 *                  the FIFO is popped on the AXI handshake.
 *
 *   This module is stateless — it contains no FSMs or counters, only
 *   combinational glue between the AXI bus and the FIFO control signals.
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
	// awready is high whenever the request FIFO has room.
	assign axi.awready = !req_fifo_full;

	// Pack the incoming AXI AW signals into a struct for the FIFO.
	always_comb begin
		req_fifo_data_out.awid    = axi.awid;
		req_fifo_data_out.awaddr  = axi.awaddr;
		req_fifo_data_out.awlen   = axi.awlen;
		req_fifo_data_out.awsize  = axi.awsize;
	end

	// Push on every accepted AW handshake.
	assign req_fifo_push_n = !(axi.awvalid && axi.awready);
	
/*------------------------------WRITE DATA----------------------------------*/
	// wready is high whenever the data FIFO has room.
	assign axi.wready = !data_fifo_full;

	// Pack the incoming AXI W signals into a struct for the FIFO.
	always_comb begin
		data_fifo_data_out.wdata  = axi.wdata;
		data_fifo_data_out.wstrb  = axi.wstrb;
		data_fifo_data_out.wlast  = axi.wlast;
	end

	// Push on every accepted W handshake.
	assign data_fifo_push_n = !(axi.wvalid && axi.wready);
	
	/*------------------------------WRITE RESP----------------------------------*/
	// bvalid is high whenever the response FIFO has an entry waiting.
	assign axi.bvalid = !resp_fifo_empty;

	// Forward the FIFO head directly onto the AXI B channel.
	always_comb begin
		axi.bid 	= resp_fifo_data_out.bid;
		axi.bresp 	= resp_fifo_data_out.bresp;
	end

	// Pop the FIFO on every accepted B-channel handshake.
	assign resp_fifo_pop_n = !(axi.bvalid && axi.bready);

endmodule