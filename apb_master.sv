`timescale 1ns/1ps

/*------------------------------------------------------------------------------
 * File          : apb_master.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 *
 * Timing fix: psel/penable must remain asserted on the cycle pready arrives
 * so that in_access && pready are simultaneously true for the response push
 * and the busy/psel/penable clear. They are now cleared the cycle AFTER
 * pready is seen, using a registered 'completing' pulse.
 *------------------------------------------------------------------------------*/

module apb_master
import struct_types::*;
(
	input  logic                    clk,
	input  logic                    rst_n,
	apb_interface.master            apb,

	output logic                    req_fifo_pop_n,
	input  struct_types::apb_struct req_fifo_data_out,
	input  logic                    req_fifo_empty,
	output logic                    resp_fifo_push_n,
	output struct_types::apb_struct resp_fifo_data_in,
	input  logic                    resp_fifo_full
);

	struct_types::apb_struct req_lat;
	logic busy;

	wire can_start = !busy && !req_fifo_empty && !resp_fifo_full;
	wire in_setup  =  apb.psel && !apb.penable;
	wire in_access =  apb.psel &&  apb.penable;

	// handshake: pready seen while in ACCESS  this is the completion event.
	// psel/penable stay asserted this cycle so in_access is still true,
	// ensuring resp_fifo_push_n fires correctly.
	wire completing = in_access && apb.pready;

	// Pop FIFO exactly once per transaction
	assign req_fifo_pop_n = !can_start;

	// Push response on the cycle pready arrives (psel/penable still high)
	assign resp_fifo_push_n = !(completing && !resp_fifo_full);

	// busy: set when we start, clear the cycle AFTER completing
	// (one extra register stage so psel/penable don't drop until next cycle)
	logic completing_r;
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) completing_r <= 1'b0;
		else        completing_r <= completing;
	end

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)        busy <= 1'b0;
		else if (can_start) busy <= 1'b1;
		else if (completing_r) busy <= 1'b0;
	end

	// Latch request when popping
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)         req_lat <= '0;
		else if (can_start) req_lat <= req_fifo_data_out;
	end

	// psel: set on can_start, clear cycle after completing
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)            apb.psel <= 1'b0;
		else if (can_start)    apb.psel <= 1'b1;
		else if (completing_r) apb.psel <= 1'b0;
	end

	// penable: set when in SETUP, clear cycle after completing
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)            apb.penable <= 1'b0;
		else if (in_setup)     apb.penable <= 1'b1;
		else if (completing_r) apb.penable <= 1'b0;
	end

	// APB request bus driven from latched request
	assign apb.paddr  = req_lat.paddr;
	assign apb.pwdata = req_lat.pwdata;
	assign apb.pwrite = req_lat.pwrite;
	assign apb.pstrb  = req_lat.pstrb;

	// Response back to manager
	always_comb begin
		resp_fifo_data_in         = '0;
		resp_fifo_data_in.prdata  = apb.prdata;
		resp_fifo_data_in.pslverr = apb.pslverr;
		resp_fifo_data_in.paddr   = req_lat.paddr;
	end

endmodule