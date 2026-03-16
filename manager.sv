/*------------------------------------------------------------------------------
 * File          : manager.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 27, 2026
 * Description   : Manager module coordinating data flow between AXI/APB domains
 *                 Handles disassembler control (AXI -> APB) and assembler control
 *                 (APB -> AXI), manages FIFOs and transaction tracking
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
	
	// APB response counter (to track when both MSB and LSB responses are received)
	logic [1:0]					apb_resp_count;
	logic						rd_resp_valid, wr_resp_valid;
	
	// Store APB responses for assembler
	struct_types::apb_struct	apb_resp_msb, apb_resp_lsb;
	logic						apb_resp_msb_valid, apb_resp_lsb_valid;
	
	// Assembler output signals
	struct_types::axi_rd_data	assembler_rd_data;
	struct_types::axi_wr_resp	assembler_wr_resp;
	logic                       assembler_is_wr;
	logic                       assembler_is_last;
	
	// Track which APB parts (MSB/LSB) have been sent in current transaction
	logic						lsb_sent, msb_sent;
	
	// Track burst progress
	logic [7:0]					rd_beat_count;	// Current beat in read burst
	
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
					if (rd_beat_count <= rd_req_stored.arlen) begin
						// More beats to send - loop back to dispatch
						next_state = DISPATCH_RD;
					end else begin
						// All beats complete - back to idle
						next_state = IDLE;
					end
				end
			end
			
			default: next_state = IDLE;
		endcase
	end

	/*=====================================================================*/
	/*                   AXI REQUEST FIFO CONTROL (POP)                    */
	/*=====================================================================*/
	
	always_comb begin
		rd_req_pop_n = 1'b1;
		wr_req_pop_n = 1'b1;
		wr_data_pop_n = 1'b1;
		
		case (current_state)
			DISPATCH_WR: begin
				if (!wr_req_empty && !wr_data_empty && !apb_req_full) begin
					wr_req_pop_n = 1'b0;
					wr_data_pop_n = 1'b0;
				end
			end
			
			DISPATCH_RD: begin
				if (!rd_req_empty && !apb_req_full) begin
					rd_req_pop_n = 1'b0;
				end
			end
		endcase
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
	
	// Track which parts have been sent
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			lsb_sent <= 1'b0;
			msb_sent <= 1'b0;
		end else begin
			case (current_state)
				DISPATCH_WR, DISPATCH_RD: begin
					if (!apb_req_push_n) begin
						// Track which part was just sent
						//TODO: if lsb wasnt valid, msb wont be sent
						if (!lsb_sent && dis_valid_lsb) begin
							lsb_sent <= 1'b1;
						end else if (lsb_sent && !msb_sent && dis_valid_msb) begin
							msb_sent <= 1'b1;
						end
					end
				end
				WAIT_RD_RESP: begin
					// Reset sent flags when transitioning back to DISPATCH_RD for next beat
					if (rd_resp_valid && !rd_data_full && rd_beat_count <= rd_req_stored.arlen) begin
						lsb_sent <= 1'b0;
						msb_sent <= 1'b0;
					end else if (rd_resp_valid && !rd_data_full && rd_beat_count > rd_req_stored.arlen) begin
						// Burst complete - reset for next transaction
						lsb_sent <= 1'b0;
						msb_sent <= 1'b0;
					end
				end			WAIT_WR_RESP: begin
				// Reset sent flags when transitioning back to DISPATCH_WR for next beat
				if (wr_resp_valid && !wr_resp_full && !wr_data_stored.wlast) begin
					lsb_sent <= 1'b0;
					msb_sent <= 1'b0;
				end else if (wr_resp_valid && !wr_resp_full && wr_data_stored.wlast) begin
					// Burst complete - reset for next transaction
					lsb_sent <= 1'b0;
					msb_sent <= 1'b0;
				end
			end				default: begin
					lsb_sent <= 1'b0;
					msb_sent <= 1'b0;
				end
			endcase
		end
	end

	/*=====================================================================*/
	/*            APB RESPONSE FIFO CONTROL (POP) & TRACKING               */
	/*=====================================================================*/
	
	// Track APB responses for reassembly
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			apb_resp_count <= '0;
			apb_resp_msb_valid <= 1'b0;
			apb_resp_lsb_valid <= 1'b0;
			rd_resp_valid <= 1'b0;
			wr_resp_valid <= 1'b0;
			rd_beat_count <= '0;
			assembler_is_wr <= 1'b0;
			assembler_is_last <= 1'b0;
		end else begin
			// Pop from APB response queue and collect MSB/LSB
			if (!apb_resp_empty && (current_state == WAIT_RD_RESP || current_state == WAIT_WR_RESP)) begin
				if (apb_resp_count == 2'd0) begin
					// First response is MSB
					apb_resp_msb <= apb_resp_fifo_out;
					apb_resp_msb_valid <= 1'b1;
					apb_resp_count <= apb_resp_count + 1'b1;
				end else if (apb_resp_count == 2'd1) begin
					// Second response is LSB - now we have both
					apb_resp_lsb <= apb_resp_fifo_out;
					apb_resp_lsb_valid <= 1'b1;
					
					// Set assembler control signals
					assembler_is_wr <= (current_state == WAIT_WR_RESP);
					assembler_is_last <= (current_state == WAIT_RD_RESP) ? 
										 (rd_beat_count == rd_req_stored.arlen) : 
										 wr_data_stored.wlast;
					
					// Mark response as valid (assembler will output valid data)
					if (current_state == WAIT_RD_RESP) begin
						rd_resp_valid <= 1'b1;
						rd_beat_count <= rd_beat_count + 1'b1;
					end else if (current_state == WAIT_WR_RESP && wr_data_stored.wlast) begin
						// Only assert valid for writes on last beat
						wr_resp_valid <= 1'b1;
					end
					
					apb_resp_count <= '0;
				end
			end
			
			// Clear valid signals after response is pushed
			if (rd_resp_valid && !rd_data_full) begin
				rd_resp_valid <= 1'b0;
				apb_resp_msb_valid <= 1'b0;
				apb_resp_lsb_valid <= 1'b0;
			end
			if (wr_resp_valid && !wr_resp_full) begin
				wr_resp_valid <= 1'b0;
				apb_resp_msb_valid <= 1'b0;
				apb_resp_lsb_valid <= 1'b0;
			end
			
			// Reset beat counters only when burst is fully complete (transitioning to IDLE)
			if ((current_state == WAIT_RD_RESP && next_state == IDLE) ||
			    (current_state == WAIT_WR_RESP && next_state == IDLE)) begin
				rd_beat_count <= '0;
			end
		end
	end
	
	// Pop from APB response queue whenever in response wait states
	assign apb_data_pop_n = !((!apb_resp_empty) && 
	                           ((current_state == WAIT_RD_RESP) || (current_state == WAIT_WR_RESP)));

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