`timescale 1ns/1ps

/*------------------------------------------------------------------------------
 * File          : DW_fifo_s1_sf.v
 * Description   : Synchronous single-clock FIFO — drop-in replacement for the
 *                 DesignWare DW_fifo_s1_sf primitive.
 *
 * PARAMETERS
 *   width  — data width in bits
 *   depth  — number of entries (must be >= 2)
 *
 * PORTS  (identical to DW_fifo_s1_sf)
 *   clk        — clock
 *   rst_n      — active-low synchronous reset
 *   push_req_n — active-low push strobe; data_in is written when low
 *   pop_req_n  — active-low pop strobe; data_out advances when low
 *   data_in    — write data
 *   data_out   — read data at the head of the FIFO (combinational)
 *   full       — high when the FIFO cannot accept more entries
 *   empty      — high when the FIFO has no valid entries
 *
 * BEHAVIOUR
 *   - First-word fall-through: data_out reflects the current head entry
 *     combinationally (i.e., without a clock edge after push).
 *   - Simultaneous push + pop when neither full nor empty is supported;
 *     the occupancy count stays the same and data flows through in one cycle.
 *   - Pushing when full or popping when empty is silently ignored.
 *   - Full/empty detection uses the standard extra-MSB pointer scheme:
 *       full  = same lower bits, different wrap bits
 *       empty = pointers identical
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

	// Pointer width: one extra bit beyond the index for full/empty disambiguation.
	// Full  = same index bits, different wrap bits.
	// Empty = pointers identical.
	localparam int PTR_W = $clog2(depth) + 1;

	logic [width-1:0]  mem  [0:depth-1];
	logic [PTR_W-1:0]  wr_ptr;   // next write slot
	logic [PTR_W-1:0]  rd_ptr;   // next read slot

	wire [PTR_W-2:0] wr_idx = wr_ptr[PTR_W-2:0];
	wire [PTR_W-2:0] rd_idx = rd_ptr[PTR_W-2:0];

	assign full  = (wr_ptr[PTR_W-2:0] == rd_ptr[PTR_W-2:0]) &&
				   (wr_ptr[PTR_W-1]   != rd_ptr[PTR_W-1]);
	assign empty = (wr_ptr == rd_ptr);

	// data_out is the head entry driven combinationally from mem (FWFT).
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