/*------------------------------------------------------------------------------
 * File          : assembler.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 24, 2026
 * Description   : Assembler module that combines 2 APB responses into 1 AXI 
 *                 response. Handles both read (combine data) and write (pass response)
 *------------------------------------------------------------------------------*/

module assembler 
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	
	// Control signals
	input  logic                            is_wr,          // 1 for write, 0 for read
	input  logic                            is_last,        // Last beat of burst
	input  logic [ID_WIDTH-1:0]             transaction_id, // bid or rid
	
	// Ports to talk to the input FIFO (from apb_master) sitting in the Top Module
	input	struct_types::apb_struct	apb_resp_fifo_out_msb, 
	input	struct_types::apb_struct	apb_resp_fifo_out_lsb, 
	
	// Ports to talk to the output FIFO (to axi_master) sitting in the Top Module
	output 	struct_types::axi_rd_data 	rd_data_fifo_in,
	output 	struct_types::axi_wr_resp 	wr_resp_fifo_in
	
);

	/*=====================================================================*/
	/*                    ASSEMBLE AXI RESPONSES                          */
	/*=====================================================================*/
	
	// Read response: combine MSB and LSB APB responses into 64-bit read data
	always_comb begin
		rd_data_fifo_in.rid = transaction_id;
		rd_data_fifo_in.rdata = {apb_resp_fifo_out_msb.prdata, apb_resp_fifo_out_lsb.prdata};
		rd_data_fifo_in.rlast = is_last;
		// Response is OKAY (2'b00) unless either MSB or LSB has error
		rd_data_fifo_in.rresp = (apb_resp_fifo_out_msb.pslverr | apb_resp_fifo_out_lsb.pslverr) ? 2'b10 : 2'b00;
	end
	
	// Write response: pass through the response from APB (both should be same)
	always_comb begin
		wr_resp_fifo_in.bid = transaction_id;
		// Response is OKAY (2'b00) unless either MSB or LSB has error
		wr_resp_fifo_in.bresp = (apb_resp_fifo_out_msb.pslverr | apb_resp_fifo_out_lsb.pslverr) ? 2'b10 : 2'b00;
	end

endmodule