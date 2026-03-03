/*------------------------------------------------------------------------------
 * File          : manager.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 27, 2026
 * Description   :
 *------------------------------------------------------------------------------*/

module manager
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	
	// Ports to talk to the input FIFO (from axi_slave) sitting in the Top Module
	output 	logic						rd_req_pop_n,	
	output 	logic						wr_req_pop_n,		
	output 	logic						wr_data_pop_n,	
	
	input 	logic						is_wr, 			// wr = 1, rd = 0
	input	logic						rd_req_empty,
	input	logic						wr_req_empty,
	input	logic						wr_data_empty,	
	
	input	struct_types::axi_rd_req	rd_req_fifo_out, 
	input	struct_types::axi_wr_req	wr_req_fifo_out,
	input 	struct_types::axi_wr_data 	wr_data_fifo_out,
	
	// Ports to talk to the output FIFO (to apb_master) sitting in the Top Module
	output 	logic               		apb_req_push_n,
	output 	struct_types::apb_struct 	apb_req_fifo_in,
	input  	logic              			apb_req_full,
	
	// Ports to talk to the input FIFO (from apb_slave) sitting in the Top Module
	output 	logic						apb_data_pop_n,	
	input	logic						apb_resp_empty,	
	input	struct_types::apb_struct	apb_resp_fifo_out, 

	
	// Ports to talk to the output FIFO (to axi_master) sitting in the Top Module
	output 	logic               		resp_push_n,
	output 	struct_types::axi_rd_data 	rd_data_fifo_in,
	output 	struct_types::axi_wr_resp 	wr_resp_fifo_in,
	input  	logic              			rd_data_full,
	input  	logic              			wr_resp_full
	
);

//1.pop from req queue 
	always_comb begin
		 if (!wr_req_empty) begin
			 wr_req_pop_n = !wr_req_empty;
			 if (!wr_data_empty) begin
				 
			 end
			 
		 end
	end
//2. if wr req pop from data queue

//3. send to disassembler

//4. send only valid:

//	a.to apb_wrapper (?)

//	b.to apb incoming queue


/*------------------------------------READ---------------------------------*/
	always_comb begin
		if (!rd_req_empty) begin
			rd_req_pop_n = !rd_req_empty;

			
		end
   end

	disassembler u_disassembler_rd (
		.clk            		(clk),
		.rst_n          		(rst_n),
		// Connections to the FIFO ports
		.is_wr					(1'b0),
		.rd_req_fifo_out    	(rd_req_fifo_out),
		.wr_req_fifo_out    	('b0), //what to put here?
		.wr_data_fifo_out   	('b0), //what to put here?
		.apb_req_fifo_in_msb	(),
		.valid_msb				(valid_msb),
		.apb_req_fifo_in_lsb	(),
		.valid_lsb				()
	);
endmodule