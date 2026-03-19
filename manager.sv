`timescale 1ns/1ps

/*------------------------------------------------------------------------------
 * File          : manager.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 27, 2026
 * Description   : Manager module coordinating data flow between AXI/APB domains
 *------------------------------------------------------------------------------*/

module manager
import struct_types::*;
(
	input  logic      clk,
	input  logic      rst_n,

	output	logic						rd_req_pop_n,
	input	logic						rd_req_empty,
	input	struct_types::axi_rd_req	rd_req_fifo_out,

	output	logic						wr_req_pop_n,
	input	logic						wr_req_empty,
	input	struct_types::axi_wr_req	wr_req_fifo_out,

	output	logic						wr_data_pop_n,
	input	logic						wr_data_empty,
	input	struct_types::axi_wr_data	wr_data_fifo_out,

	output	logic						apb_req_push_n,
	output	struct_types::apb_struct	apb_req_fifo_in,
	input	logic						apb_req_full,

	output	logic						apb_data_pop_n,
	input	logic						apb_resp_empty,
	input	struct_types::apb_struct	apb_resp_fifo_out,

	output	logic						rd_data_push_n,
	output	struct_types::axi_rd_data	rd_data_fifo_in,
	input	logic						rd_data_full,

	output	logic						wr_resp_push_n,
	output	struct_types::axi_wr_resp	wr_resp_fifo_in,
	input	logic						wr_resp_full
);

	/*--------- INTERNAL SIGNALS ---------*/

	logic						dis_valid_msb, dis_valid_lsb;
	struct_types::apb_struct	dis_apb_msb, dis_apb_lsb;
	logic						dis_is_wr;

	struct_types::axi_rd_req	rd_req_stored;
	struct_types::axi_wr_req	wr_req_stored;
	struct_types::axi_wr_data	wr_data_stored;

	// req_latched: stays high for the entire DISPATCH state after the first pop.
	// Used as the push gate instead of !fifo_empty (which goes low after pop).
	logic						rd_req_latched, wr_req_latched;

	logic [1:0]					apb_resp_count;
	logic						rd_resp_valid, wr_resp_valid;

	struct_types::apb_struct	apb_resp_msb, apb_resp_lsb;
	logic						apb_resp_msb_valid, apb_resp_lsb_valid;
	logic						apb_pop_pending;

	struct_types::axi_rd_data	assembler_rd_data;
	struct_types::axi_wr_resp	assembler_wr_resp;
	logic						assembler_is_wr;
	logic						assembler_is_last;

	// lsb_sent/msb_sent: registered, track which halves have been pushed this beat
	logic						lsb_sent, msb_sent;

	logic [7:0]					rd_beat_count;
	logic						wr_burst_err;

	typedef enum logic [2:0] {
		IDLE,
		DISPATCH_WR,
		DISPATCH_RD,
		WAIT_WR_RESP,
		WAIT_RD_RESP
	} state_t;

	state_t current_state, next_state;

	/*=====================================================================*/
	/*                     DISASSEMBLER INSTANTIATION                      */
	/*=====================================================================*/

	disassembler u_disassembler (
		.clk                 (clk),
		.rst_n               (rst_n),
		.is_wr               (dis_is_wr),
		// Feed from stored registers (stable after pop) not live FIFO outputs
		// (which go stale the cycle after the pop fires).
		.rd_req_fifo_out     (rd_req_stored),
		.wr_req_fifo_out     (wr_req_stored),
		.wr_data_fifo_out    (wr_data_stored),
		.apb_req_fifo_in_msb (dis_apb_msb),
		.valid_msb           (dis_valid_msb),
		.apb_req_fifo_in_lsb (dis_apb_lsb),
		.valid_lsb           (dis_valid_lsb)
	);

	/*=====================================================================*/
	/*                      ASSEMBLER INSTANTIATION                        */
	/*=====================================================================*/

	assembler u_assembler (
		.clk                  (clk),
		.rst_n                (rst_n),
		.is_wr                (assembler_is_wr),
		.is_last              (assembler_is_last),
		.transaction_id       (assembler_is_wr ? wr_req_stored.awid : rd_req_stored.arid),
		.apb_resp_fifo_out_msb(apb_resp_msb),
		.apb_resp_fifo_out_lsb(apb_resp_lsb),
		.rd_data_fifo_in      (assembler_rd_data),
		.wr_resp_fifo_in      (assembler_wr_resp)
	);

	/*=====================================================================*/
	/*                        STATE MACHINE                                */
	/*=====================================================================*/

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) current_state <= IDLE;
		else        current_state <= next_state;
	end

	always_comb begin
		next_state = current_state;
		case (current_state)
			IDLE: begin
				if (!wr_req_empty && !wr_data_empty) next_state = DISPATCH_WR;
				else if (!rd_req_empty)              next_state = DISPATCH_RD;
			end

			// Exit DISPATCH the cycle the last half is pushed.
			// We compute "will lsb/msb be done after this cycle?" combinationally
			// by peeking at apb_req_push_n (already driven this cycle in always_comb below).
			// lsb done = already sent OR being sent right now (push fires and it's the lsb)
			// msb done = already sent OR being sent right now (push fires and it's the msb)
			DISPATCH_WR: begin
				if (( lsb_sent || (!lsb_sent && !apb_req_push_n &&  dis_valid_lsb)) &&
					( msb_sent || ( lsb_sent && !msb_sent && !apb_req_push_n && dis_valid_msb)) )
					next_state = WAIT_WR_RESP;
				else if ((lsb_sent || (!lsb_sent && !apb_req_push_n && dis_valid_lsb)) && !dis_valid_msb)
					next_state = WAIT_WR_RESP;
				else if (!dis_valid_lsb && !dis_valid_msb)
					next_state = WAIT_WR_RESP;
			end

			DISPATCH_RD: begin
				if (( lsb_sent || (!lsb_sent && !apb_req_push_n &&  dis_valid_lsb)) &&
					( msb_sent || ( lsb_sent && !msb_sent && !apb_req_push_n && dis_valid_msb)) )
					next_state = WAIT_RD_RESP;
				else if ((lsb_sent || (!lsb_sent && !apb_req_push_n && dis_valid_lsb)) && !dis_valid_msb)
					next_state = WAIT_RD_RESP;
				else if (!dis_valid_lsb && !dis_valid_msb)
					next_state = WAIT_RD_RESP;
			end

			WAIT_WR_RESP: begin
				if (wr_resp_valid && !wr_resp_full) begin
					if (wr_data_stored.wlast) next_state = IDLE;
					else                      next_state = DISPATCH_WR;
				end
			end

			WAIT_RD_RESP: begin
				if (rd_resp_valid && !rd_data_full) begin
					if (rd_beat_count >= rd_req_stored.arlen) next_state = IDLE;
					else                                      next_state = DISPATCH_RD;
				end
			end

			default: next_state = IDLE;
		endcase
	end

	/*=====================================================================*/
	/*                   AXI REQUEST FIFO CONTROL (POP)                    */
	/*=====================================================================*/
	// Pop fires on the first cycle of DISPATCH (before wr_req_latched goes high).
	// apb_req_full does NOT block the pop  we are just latching the request
	// into wr_req_stored/wr_data_stored. The push to APB is gated separately.

	always_comb begin
		rd_req_pop_n  = 1'b1;
		wr_req_pop_n  = 1'b1;
		wr_data_pop_n = 1'b1;
		case (current_state)
			DISPATCH_WR: begin
				if (!wr_req_latched && !wr_req_empty && !wr_data_empty)
					begin wr_req_pop_n = 1'b0; wr_data_pop_n = 1'b0; end
			end
			DISPATCH_RD: begin
				if (!rd_req_latched && !rd_req_empty)
					rd_req_pop_n = 1'b0;
			end
		endcase
	end

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rd_req_latched <= 1'b0;
			wr_req_latched <= 1'b0;
		end else begin
			case (current_state)
				DISPATCH_WR: if (!wr_req_pop_n) wr_req_latched <= 1'b1;
				DISPATCH_RD: if (!rd_req_pop_n) rd_req_latched <= 1'b1;
				default: begin
					rd_req_latched <= 1'b0;
					wr_req_latched <= 1'b0;
				end
			endcase
		end
	end

	/*=====================================================================*/
	/*               APB REQUEST FIFO CONTROL (PUSH) & DATA MUX            */
	/*=====================================================================*/
	// Gate on req_latched, NOT on !fifo_empty.
	// After the pop, the FIFOs go empty but req_latched stays high and the
	// disassembler outputs remain stable (driven from wr_req_fifo_out which
	// holds the last value until overwritten, and the stored regs are latched).

	always_comb begin
		apb_req_push_n  = 1'b1;
		apb_req_fifo_in = '0;
		dis_is_wr       = 1'b0;

		case (current_state)
			DISPATCH_WR: begin
				if (wr_req_latched && !apb_req_full) begin
					dis_is_wr = 1'b1;
					if (!lsb_sent && dis_valid_lsb) begin
						apb_req_push_n  = 1'b0;
						apb_req_fifo_in = dis_apb_lsb;
					end else if (lsb_sent && !msb_sent && dis_valid_msb) begin
						apb_req_push_n  = 1'b0;
						apb_req_fifo_in = dis_apb_msb;
					end
				end
			end

			DISPATCH_RD: begin
				if (rd_req_latched && !apb_req_full) begin
					dis_is_wr = 1'b0;
					if (!lsb_sent && dis_valid_lsb) begin
						apb_req_push_n  = 1'b0;
						apb_req_fifo_in = dis_apb_lsb;
					end else if (lsb_sent && !msb_sent && dis_valid_msb) begin
						apb_req_push_n  = 1'b0;
						apb_req_fifo_in = dis_apb_msb;
					end
				end
			end
		endcase
	end

	// Track which halves have been dispatched this beat
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			lsb_sent <= 1'b0;
			msb_sent <= 1'b0;
		end else begin
			case (current_state)
				DISPATCH_WR, DISPATCH_RD: begin
					if (!apb_req_push_n) begin
						if (!lsb_sent && dis_valid_lsb)
							lsb_sent <= 1'b1;
						else if (lsb_sent && !msb_sent && dis_valid_msb)
							msb_sent <= 1'b1;
					end
				end
				WAIT_RD_RESP: if (rd_resp_valid && !rd_data_full) begin lsb_sent <= 1'b0; msb_sent <= 1'b0; end
				WAIT_WR_RESP: if (wr_resp_valid && !wr_resp_full) begin lsb_sent <= 1'b0; msb_sent <= 1'b0; end
				default:      begin lsb_sent <= 1'b0; msb_sent <= 1'b0; end
			endcase
		end
	end

	/*=====================================================================*/
	/*            APB RESPONSE FIFO CONTROL (POP) & TRACKING               */
	/*=====================================================================*/

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			apb_resp_count    <= '0;
			apb_resp_msb_valid <= 1'b0;
			apb_resp_lsb_valid <= 1'b0;
			apb_resp_msb      <= '0;
			apb_resp_lsb      <= '0;
			rd_resp_valid     <= 1'b0;
			wr_resp_valid     <= 1'b0;
			rd_beat_count     <= '0;
			assembler_is_wr   <= 1'b0;
			assembler_is_last <= 1'b0;
			apb_pop_pending   <= 1'b0;
			wr_burst_err      <= 1'b0;
		end else begin
			apb_pop_pending <= 1'b0;

			if (!apb_resp_empty &&
				(current_state == WAIT_RD_RESP || current_state == WAIT_WR_RESP) &&
				apb_resp_count < 2'd2 &&
				!apb_pop_pending) begin

				apb_pop_pending <= 1'b1;

				if (!apb_resp_fifo_out.paddr[2]) begin
					apb_resp_lsb       <= apb_resp_fifo_out;
					apb_resp_lsb_valid <= 1'b1;
				end else begin
					apb_resp_msb       <= apb_resp_fifo_out;
					apb_resp_msb_valid <= 1'b1;
				end

				apb_resp_count <= apb_resp_count + 1'b1;

				if (apb_resp_count == 2'd1) begin
					assembler_is_wr   <= (current_state == WAIT_WR_RESP);
					assembler_is_last <= (current_state == WAIT_RD_RESP) ?
										 (rd_beat_count == rd_req_stored.arlen) :
										 wr_data_stored.wlast;

					if (current_state == WAIT_RD_RESP) begin
						rd_resp_valid <= 1'b1;
						rd_beat_count <= rd_beat_count + 1'b1;
					end else begin
						wr_burst_err <= wr_burst_err |
										apb_resp_fifo_out.pslverr |
										(apb_resp_fifo_out.paddr[2] ?
											apb_resp_lsb.pslverr :
											apb_resp_msb.pslverr);
						if (wr_data_stored.wlast)
							wr_resp_valid <= 1'b1;
					end

					apb_resp_count <= '0;
				end
			end

			if (rd_resp_valid && !rd_data_full) begin
				rd_resp_valid      <= 1'b0;
				apb_resp_msb_valid <= 1'b0;
				apb_resp_lsb_valid <= 1'b0;
				apb_resp_count     <= '0;
			end

			if (wr_resp_valid && !wr_resp_full) begin
				wr_resp_valid      <= 1'b0;
				wr_burst_err       <= 1'b0;
				apb_resp_msb_valid <= 1'b0;
				apb_resp_lsb_valid <= 1'b0;
				apb_resp_count     <= '0;
			end

			if ((current_state == WAIT_RD_RESP && next_state == IDLE) ||
				(current_state == WAIT_WR_RESP && next_state == IDLE))
				rd_beat_count <= '0;
		end
	end

	assign apb_data_pop_n = !apb_pop_pending;

	/*=====================================================================*/
	/*             AXI RESPONSE FIFO CONTROL (PUSH) & ASSEMBLY             */
	/*=====================================================================*/

	assign rd_data_push_n  = !(rd_resp_valid && !rd_data_full);
	assign rd_data_fifo_in  = assembler_rd_data;
	assign wr_resp_push_n  = !(wr_resp_valid && !wr_resp_full);
	assign wr_resp_fifo_in  = assembler_wr_resp;

	/*=====================================================================*/
	/*                    STORE TRANSACTION INFO ON POP                    */
	/*=====================================================================*/

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rd_req_stored  <= '0;
			wr_req_stored  <= '0;
			wr_data_stored <= '0;
		end else begin
			if (!rd_req_pop_n)  rd_req_stored  <= rd_req_fifo_out;
			if (!wr_req_pop_n)  wr_req_stored  <= wr_req_fifo_out;
			if (!wr_data_pop_n) wr_data_stored <= wr_data_fifo_out;
		end
	end

endmodule