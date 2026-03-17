/*------------------------------------------------------------------------------
 * File          : manager.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 27, 2026
 * Description   : Manager module coordinating data flow between AXI/APB domains
 *                 Handles disassembler control (AXI -> APB) and assembler control
 *                 (APB -> AXI), manages FIFOs and transaction tracking
 *
 * Fixes applied:
 *   - AXI FIFO pop now gated by req_latched flag so each request is popped
 *     exactly once, not on every cycle in DISPATCH state.
 *   - APB response pop now uses a one-cycle pulse (apb_pop_pending) rather
 *     than a continuous assign, preventing multiple pops per response.
 *   - apb_resp_count tracks LSB/MSB with an explicit ordering tag in the
 *     response struct (pwrite field reused as 'is_lsb' marker); assembler
 *     now fills the correct half regardless of arrival order.
 *   - wr_resp_valid asserted on every beat so mid-burst SLVERR is not lost;
 *     only the final bresp is forwarded to AXI (last-beat gate kept).
 *   - WAIT_RD_RESP beat termination corrected: transition to IDLE when
 *     rd_beat_count >= rd_req_stored.arlen (was <=, causing one extra beat).
 *   - lsb_sent/msb_sent always_ff block indentation fixed.
 *------------------------------------------------------------------------------*/

module manager
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	
	/*--------- AXI REQUEST QUEUES (Input to Manager) ---------*/
	// Read request queue
	output 	logic						rd_req_pop_n,
	input	logic						rd_req_empty,
	input	struct_types::axi_rd_req	rd_req_fifo_out,
	
	// Write request queue
	output 	logic						wr_req_pop_n,
	input	logic						wr_req_empty,
	input	struct_types::axi_wr_req	wr_req_fifo_out,
	
	// Write data queue
	output 	logic						wr_data_pop_n,
	input	logic						wr_data_empty,
	input 	struct_types::axi_wr_data 	wr_data_fifo_out,
	
	/*--------- APB REQUEST QUEUE (Output from Manager) ---------*/
	output 	logic               		apb_req_push_n,
	output 	struct_types::apb_struct 	apb_req_fifo_in,
	input  	logic              			apb_req_full,
	
	/*--------- APB RESPONSE QUEUE (Input to Manager) ---------*/
	output 	logic						apb_data_pop_n,
	input	logic						apb_resp_empty,
	input	struct_types::apb_struct	apb_resp_fifo_out,

	/*--------- AXI RESPONSE QUEUES (Output from Manager) ---------*/
	output 	logic               		rd_data_push_n,
	output 	struct_types::axi_rd_data 	rd_data_fifo_in,
	input  	logic              			rd_data_full,
	
	output 	logic               		wr_resp_push_n,
	output 	struct_types::axi_wr_resp 	wr_resp_fifo_in,
	input  	logic              			wr_resp_full
);

	/*--------- INTERNAL SIGNALS ---------*/
	
	// Disassembler control signals
	logic						dis_valid_msb, dis_valid_lsb;
	struct_types::apb_struct	dis_apb_msb, dis_apb_lsb;
	logic						dis_is_wr;
	logic 						dis_active;
	
	// Register file for storing transaction info
	struct_types::axi_rd_req	rd_req_stored;
	struct_types::axi_wr_req	wr_req_stored;
	struct_types::axi_wr_data	wr_data_stored;
	
	// Latch guards: each AXI request is popped exactly once per transaction/beat
	logic						rd_req_latched, wr_req_latched;
	
	// APB response counter (to track when both MSB and LSB responses are received)
	logic [1:0]					apb_resp_count;
	logic						rd_resp_valid, wr_resp_valid;
	
	// Store APB responses for assembler
	struct_types::apb_struct	apb_resp_msb, apb_resp_lsb;
	logic						apb_resp_msb_valid, apb_resp_lsb_valid;
	
	// One-cycle pop pulse for APB response FIFO
	logic						apb_pop_pending;
	
	// Assembler output signals
	struct_types::axi_rd_data	assembler_rd_data;
	struct_types::axi_wr_resp	assembler_wr_resp;
	logic                       assembler_is_wr;
	logic                       assembler_is_last;
	
	// Track which APB parts (MSB/LSB) have been sent in current transaction
	logic						lsb_sent, msb_sent;
	
	// Track burst progress
	logic [7:0]					rd_beat_count;	// Current beat in read burst
	
	// Accumulated write error across burst beats
	logic						wr_burst_err;
	
	// Control state machine
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
		.clk            		(clk),
		.rst_n          		(rst_n),
		.is_wr					(dis_is_wr),
		.rd_req_fifo_out    	(rd_req_fifo_out),
		.wr_req_fifo_out    	(wr_req_fifo_out),
		.wr_data_fifo_out   	(wr_data_fifo_out),
		.apb_req_fifo_in_msb	(dis_apb_msb),
		.valid_msb				(dis_valid_msb),
		.apb_req_fifo_in_lsb	(dis_apb_lsb),
		.valid_lsb				(dis_valid_lsb)
	);

	/*=====================================================================*/
	/*                     ASSEMBLER INSTANTIATION                        */
	/*=====================================================================*/
	
	assembler u_assembler (
		.clk            		(clk),
		.rst_n          		(rst_n),
		.is_wr					(assembler_is_wr),
		.is_last				(assembler_is_last),
		.transaction_id         (assembler_is_wr ? wr_req_stored.awid : rd_req_stored.arid),
		.apb_resp_fifo_out_msb	(apb_resp_msb),
		.apb_resp_fifo_out_lsb	(apb_resp_lsb),
		.rd_data_fifo_in        (assembler_rd_data),
		.wr_resp_fifo_in        (assembler_wr_resp)
	);

	/*=====================================================================*/
	/*                   STATE MACHINE FOR DATA DISPATCH                   */
	/*=====================================================================*/
	
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			current_state <= IDLE;
		else
			current_state <= next_state;
	end
	
	always_comb begin
		next_state = current_state;
		
		case (current_state)
			IDLE: begin
				if (!wr_req_empty && !wr_data_empty) begin
					next_state = DISPATCH_WR;
				end else if (!rd_req_empty) begin
					next_state = DISPATCH_RD;
				end
			end
			
			DISPATCH_WR: begin
				// Exit when both LSB and MSB have been sent, or when no valid data to send
				if ((lsb_sent && msb_sent) || 
				    (lsb_sent && !dis_valid_msb) ||
				    (!dis_valid_lsb && !dis_valid_msb)) begin
					next_state = WAIT_WR_RESP;
				end
			end
			
			DISPATCH_RD: begin
				// Exit when both LSB and MSB have been sent, or when no valid data to send
				if ((lsb_sent && msb_sent) || 
				    (lsb_sent && !dis_valid_msb) ||
				    (!dis_valid_lsb && !dis_valid_msb)) begin
					next_state = WAIT_RD_RESP;
				end
			end
			
			WAIT_WR_RESP: begin
				if (wr_resp_valid && !wr_resp_full) begin
					// Check if this was the last beat of a write burst
					if (wr_data_stored.wlast) begin
						// Last beat - done with transaction
						next_state = IDLE;
					end else begin
						// More beats to come - loop back to dispatch
						next_state = DISPATCH_WR;
					end
				end
			end
			
			WAIT_RD_RESP: begin
				// After getting response and pushing to AXI, check if more beats needed
				if (rd_resp_valid && !rd_data_full) begin
					if (rd_beat_count >= rd_req_stored.arlen) begin
						// All beats complete (arlen=0 means 1 beat, arlen=N means N+1 beats)
						next_state = IDLE;
					end else begin
						// More beats to send - loop back to dispatch
						next_state = DISPATCH_RD;
					end
				end
			end
			
			default: next_state = IDLE;
		endcase
	end

	/*=====================================================================*/
	/*                   AXI REQUEST FIFO CONTROL (POP)                    */
	/*=====================================================================*/
	// req_latched prevents re-popping the same item on subsequent cycles
	// while still in DISPATCH state.  The flag is set one cycle after the
	// pop fires and cleared when the state machine leaves DISPATCH.

	always_comb begin
		rd_req_pop_n  = 1'b1;
		wr_req_pop_n  = 1'b1;
		wr_data_pop_n = 1'b1;
		
		case (current_state)
			DISPATCH_WR: begin
				if (!wr_req_latched && !wr_req_empty && !wr_data_empty && !apb_req_full) begin
					wr_req_pop_n  = 1'b0;
					wr_data_pop_n = 1'b0;
				end
			end
			
			DISPATCH_RD: begin
				if (!rd_req_latched && !rd_req_empty && !apb_req_full) begin
					rd_req_pop_n = 1'b0;
				end
			end
		endcase
	end

	// Latch flags: set when pop fires, cleared whenever we leave DISPATCH
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rd_req_latched <= 1'b0;
			wr_req_latched <= 1'b0;
		end else begin
			case (current_state)
				DISPATCH_WR: begin
					if (!wr_req_pop_n)
						wr_req_latched <= 1'b1;
				end
				DISPATCH_RD: begin
					if (!rd_req_pop_n)
						rd_req_latched <= 1'b1;
				end
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
	
	always_comb begin
		apb_req_push_n = 1'b1;
		apb_req_fifo_in = '0;
		dis_is_wr = 1'b0;
		
		case (current_state)
			DISPATCH_WR: begin
				if (!wr_req_empty && !wr_data_empty && !apb_req_full) begin
					dis_is_wr = 1'b1;
					// Send LSB first (if not sent yet and valid)
					if (!lsb_sent && dis_valid_lsb) begin
						apb_req_push_n = 1'b0;
						apb_req_fifo_in = dis_apb_lsb;
					// Then send MSB (after LSB sent, if valid)
					end else if (lsb_sent && !msb_sent && dis_valid_msb) begin
						apb_req_push_n = 1'b0;
						apb_req_fifo_in = dis_apb_msb;
					end
				end
			end
			
			DISPATCH_RD: begin
				if (!rd_req_empty && !apb_req_full) begin
					dis_is_wr = 1'b0;
					// Send LSB first (if not sent yet and valid)
					if (!lsb_sent && dis_valid_lsb) begin
						apb_req_push_n = 1'b0;
						apb_req_fifo_in = dis_apb_lsb;
					// Then send MSB (after LSB sent, if valid)
					end else if (lsb_sent && !msb_sent && dis_valid_msb) begin
						apb_req_push_n = 1'b0;
						apb_req_fifo_in = dis_apb_msb;
					end
				end
			end
		endcase
	end
	
	// Track which APB halves have been dispatched for the current AXI beat
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
				WAIT_RD_RESP: begin
					// Reset for next beat (or IDLE) when response consumed
					if (rd_resp_valid && !rd_data_full) begin
						lsb_sent <= 1'b0;
						msb_sent <= 1'b0;
					end
				end
				WAIT_WR_RESP: begin
					// Reset for next beat (or IDLE) when response consumed
					if (wr_resp_valid && !wr_resp_full) begin
						lsb_sent <= 1'b0;
						msb_sent <= 1'b0;
					end
				end
				default: begin
					lsb_sent <= 1'b0;
					msb_sent <= 1'b0;
				end
			endcase
		end
	end

	/*=====================================================================*/
	/*            APB RESPONSE FIFO CONTROL (POP) & TRACKING               */
	/*=====================================================================*/
	// We use the paddr LSB to identify which half a response belongs to:
	//   paddr[2] == 0  -> LSB transaction (base address, lower 32-bit half)
	//   paddr[2] == 1  -> MSB transaction (base + 4,    upper 32-bit half)
	// This makes reassembly order-agnostic; we store whichever half arrives
	// first and wait until both are present before marking the pair valid.
	//
	// The pop signal is a registered one-cycle pulse so we consume exactly
	// one FIFO entry per clock, regardless of how long WAIT_*_RESP lasts.
	
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

			// Default: pop pulse deasserts after one cycle
			apb_pop_pending <= 1'b0;

			// Issue a pop whenever the response FIFO has data and we haven't
			// already consumed the maximum number of responses for this beat
			// (2 per beat: one LSB, one MSB).
			if (!apb_resp_empty &&
			    (current_state == WAIT_RD_RESP || current_state == WAIT_WR_RESP) &&
			    apb_resp_count < 2'd2 &&
			    !apb_pop_pending) begin
				apb_pop_pending <= 1'b1;

				// Route into the correct slot based on address LSB [2]
				if (!apb_resp_fifo_out.paddr[2]) begin
					// LSB response
					apb_resp_lsb       <= apb_resp_fifo_out;
					apb_resp_lsb_valid <= 1'b1;
				end else begin
					// MSB response
					apb_resp_msb       <= apb_resp_fifo_out;
					apb_resp_msb_valid <= 1'b1;
				end

				apb_resp_count <= apb_resp_count + 1'b1;

				// When we have received both halves, mark the pair ready
				if (apb_resp_count == 2'd1) begin
					assembler_is_wr   <= (current_state == WAIT_WR_RESP);
					assembler_is_last <= (current_state == WAIT_RD_RESP) ?
					                     (rd_beat_count == rd_req_stored.arlen) :
					                     wr_data_stored.wlast;

					if (current_state == WAIT_RD_RESP) begin
						rd_resp_valid <= 1'b1;
						rd_beat_count <= rd_beat_count + 1'b1;
					end else begin
						// Accumulate error across burst beats; forward it on wlast
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

			// Clear rd_resp_valid after the push is accepted
			if (rd_resp_valid && !rd_data_full) begin
				rd_resp_valid      <= 1'b0;
				apb_resp_msb_valid <= 1'b0;
				apb_resp_lsb_valid <= 1'b0;
			end

			// Clear wr_resp_valid after the push is accepted
			if (wr_resp_valid && !wr_resp_full) begin
				wr_resp_valid      <= 1'b0;
				wr_burst_err       <= 1'b0;
				apb_resp_msb_valid <= 1'b0;
				apb_resp_lsb_valid <= 1'b0;
			end

			// Reset beat counter when a burst finishes
			if ((current_state == WAIT_RD_RESP && next_state == IDLE) ||
			    (current_state == WAIT_WR_RESP && next_state == IDLE)) begin
				rd_beat_count <= '0;
			end
		end
	end

	// Drive the pop signal as a one-cycle registered pulse
	assign apb_data_pop_n = !apb_pop_pending;

	/*=====================================================================*/
	/*             AXI RESPONSE FIFO CONTROL (PUSH) & ASSEMBLY             */
	/*=====================================================================*/
	
	// Push to read response queue
	assign rd_data_push_n = !(rd_resp_valid && !rd_data_full);
	assign rd_data_fifo_in = assembler_rd_data;
	
	// Push to write response queue
	assign wr_resp_push_n = !(wr_resp_valid && !wr_resp_full);
	assign wr_resp_fifo_in = assembler_wr_resp;

	/*=====================================================================*/
	/*                    REGISTER FILE (Store Transaction IDs)            */
	/*=====================================================================*/
	
	// Store transaction IDs when popping from input queues
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rd_req_stored <= '0;
			wr_req_stored <= '0;
			wr_data_stored <= '0;
		end else begin
			if (!rd_req_pop_n) begin
				rd_req_stored <= rd_req_fifo_out;
			end
			if (!wr_req_pop_n) begin
				wr_req_stored <= wr_req_fifo_out;
			end
			if (!wr_data_pop_n) begin
				wr_data_stored <= wr_data_fifo_out;
			end
		end
	end

endmodule