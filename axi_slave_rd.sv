/*------------------------------------------------------------------------------
 * File          : axi_slave_rd.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 * Description   : AXI read-side interface adapter.
 *
 *   Bridges the two AXI read channels (AR, R) to the FIFO interfaces
 *   used by the rest of the bridge:
 *
 *     AR channel : accepts read addresses; pushes axi_rd_req structs into
 *                  the read-request FIFO. arready is driven from !full.
 *
 *     R channel  : drives read data beats back to the AXI master from the
 *                  read-data FIFO. rvalid is driven from !empty; the FIFO
 *                  is popped on the AXI handshake.
 *
 *   This module is stateless — it contains no FSMs or counters, only
 *   combinational glue between the AXI bus and the FIFO control signals.
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
	// arready is high whenever the request FIFO has room.
	// The AXI AR handshake completes when arvalid && arready.
	assign axi.arready = !req_fifo_full;

	// Pack the incoming AXI AR signals into a struct for the FIFO.
	always_comb begin
		req_fifo_data_out.arid    = axi.arid;
		req_fifo_data_out.araddr  = axi.araddr;
		req_fifo_data_out.arlen   = axi.arlen;
		req_fifo_data_out.arsize  = axi.arsize;
	end

	// Push on every accepted handshake.
	assign req_fifo_push_n = !(axi.arvalid && axi.arready);
	
	
/*------------------------------READ DATA----------------------------------*/	
	// rvalid is high whenever the data FIFO has an entry waiting.
	assign axi.rvalid = !data_fifo_empty;

	// Forward the FIFO head directly onto the AXI R channel.
	always_comb begin
		axi.rid = data_fifo_data_in.rid;
		axi.rdata = data_fifo_data_in.rdata;
		axi.rresp = data_fifo_data_in.rresp;
		axi.rlast = data_fifo_data_in.rlast;
	end

	// Pop the FIFO on every accepted R-channel handshake.
	assign data_fifo_pop_n = !(axi.rvalid && axi.rready);

endmodule