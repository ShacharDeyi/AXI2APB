`timescale 1ns/1ps

/*------------------------------------------------------------------------------
 * File          : apb_master.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 * Description   : APB bus driver — executes one APB transaction at a time.
 *
 * OPERATION
 * =========
 * Pops one entry from the APB request FIFO when idle and the response FIFO
 * has space. Drives the two-phase APB protocol:
 *
 *   SETUP  phase : psel=1, penable=0 (one clock cycle)
 *   ACCESS phase : psel=1, penable=1, waits until the slave asserts pready
 *
 * On pready: captures the slave's response (prdata, pslverr) and pushes it
 * into the APB response FIFO together with paddr (used by the manager to
 * identify the transaction).
 *
 * FIFO INTERFACE
 * ==============
 * req_fifo  : carries apb_struct (address, direction, write data, byte enables)
 * resp_fifo : carries apb_struct (address echoed back, prdata, pslverr)
 *
 * The manager pops the response FIFO and uses the sideband tag FIFO (managed
 * separately) to route each response to the correct transaction slot and half.
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

	// Transaction completes when the slave asserts pready during the ACCESS phase.
	wire completing = in_access && apb.pready;

	// Pop the request FIFO the same cycle we start a new transaction.
	assign req_fifo_pop_n = !can_start;

	// Push the response FIFO the cycle pready arrives.
	// psel/penable are still high this cycle; they drop on the next clock edge.
	assign resp_fifo_push_n = !(completing && !resp_fifo_full);

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)         busy <= 1'b0;
		else if (can_start)  busy <= 1'b1;
		else if (completing) busy <= 1'b0;
	end

	// Latch the request when it is popped from the FIFO.
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)         req_lat <= '0;
		else if (can_start) req_lat <= req_fifo_data_out;
	end

	// psel/penable: raised when a transaction starts, lowered the cycle after pready.
	// The non-blocking assignment means psel/penable drop one clock after completing,
	// which is correct — the slave must see them high on the pready cycle.
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)          apb.psel <= 1'b0;
		else if (can_start)  apb.psel <= 1'b1;
		else if (completing) apb.psel <= 1'b0;
	end

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)          apb.penable <= 1'b0;
		else if (in_setup)   apb.penable <= 1'b1;
		else if (completing) apb.penable <= 1'b0;
	end

	// Drive APB request signals from the latched request.
	assign apb.paddr  = req_lat.paddr;
	assign apb.pwdata = req_lat.pwdata;
	assign apb.pwrite = req_lat.pwrite;
	assign apb.pstrb  = req_lat.pstrb;

	// Pack the slave's response for the response FIFO.
	// paddr is echoed so the manager can identify the transaction.
	always_comb begin
		resp_fifo_data_in         = '0;
		resp_fifo_data_in.prdata  = apb.prdata;
		resp_fifo_data_in.pslverr = apb.pslverr;
		resp_fifo_data_in.paddr   = req_lat.paddr;
	end

endmodule