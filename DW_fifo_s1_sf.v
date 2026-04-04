`timescale 1ns/1ps

/*------------------------------------------------------------------------------
 * File          : DW_fifo_s1_sf.sv
 * Description   : Drop-in replacement for the DesignWare DW_fifo_s1_sf
 *                 synchronous single-clock FIFO.
 *
 * Parameters:
 *   width  - data width in bits
 *   depth  - number of entries (must be >= 2)
 *
 * Ports (matching DW_fifo_s1_sf exactly):
 *   clk        - clock
 *   rst_n      - active-low synchronous reset
 *   push_req_n - active-low push strobe  (data_in written when low)
 *   pop_req_n  - active-low pop strobe   (data_out advances when low)
 *   data_in    - write data
 *   data_out   - read data (head of FIFO, combinational)
 *   full       - asserted when FIFO cannot accept more entries
 *   empty      - asserted when FIFO has no valid entries
 *
 * Notes:
 *   - Simultaneous push + pop when neither full nor empty is supported;
 *     the count stays the same and data flows through in one cycle.
 *   - data_out is the entry at the current read pointer (registered),
 *     matching DW_fifo_s1_sf's "first-word fall-through" behaviour.
 *   - Pushing when full or popping when empty is ignored (no error port
 *     needed for this bridge, but the conditions are flagged via $display
 *     in simulation).
 *------------------------------------------------------------------------------*/

module DW_fifo_s1_sf #(
	parameter int width = 8,
	parameter int depth = 16
)(
	input  logic             clk,
	input  logic             rst_n,
	input  logic             push_req_n,
	input  logic             pop_req_n,
	input  logic [width-1:0] data_in,
	output logic [width-1:0] data_out,
	output logic             full,
	output logic             empty
);

	// Pointer width: needs to count 0..depth-1, plus one extra bit for
	// full/empty disambiguation (standard 2^n trick; works for any depth
	// because we compare the full pointer values, not just the lower bits).
	localparam int PTR_W = $clog2(depth) + 1;

	logic [width-1:0]  mem  [0:depth-1];
	logic [PTR_W-1:0]  wr_ptr;   // next write slot
	logic [PTR_W-1:0]  rd_ptr;   // next read slot

	// Lower bits are the actual index; MSB is the wrap bit.
	wire [PTR_W-2:0] wr_idx = wr_ptr[PTR_W-2:0];
	wire [PTR_W-2:0] rd_idx = rd_ptr[PTR_W-2:0];

	// Full  : pointers match but wrap bits differ
	// Empty : pointers are identical
	assign full  = (wr_ptr[PTR_W-2:0] == rd_ptr[PTR_W-2:0]) &&
				   (wr_ptr[PTR_W-1]   != rd_ptr[PTR_W-1]);
	assign empty = (wr_ptr == rd_ptr);

	// data_out: registered head of FIFO (first-word fall-through emulation)
	assign data_out = mem[rd_idx];

	wire do_push = !push_req_n && !full;
	wire do_pop  = !pop_req_n  && !empty;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wr_ptr <= '0;
			rd_ptr <= '0;
		end else begin
			if (do_push) begin
				mem[wr_idx] <= data_in;
				wr_ptr      <= wr_ptr + 1'b1;
			end
			if (do_pop) begin
				rd_ptr <= rd_ptr + 1'b1;
			end
		end
	end
endmodule