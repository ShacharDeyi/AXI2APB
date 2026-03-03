/*------------------------------------------------------------------------------
 * File          : axi_wrapper.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 27, 2026
 * Description   :
 *------------------------------------------------------------------------------*/

module axi_wrapper
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	
	input 	logic						is_wr, 			// wr = 1, rd = 0
	input	struct_types::apb_struct	apb_resp_msb, 
	input	struct_types::apb_struct	apb_resp_lsb, 

	
	// Ports to talk to the output FIFO (to axi_master) sitting in the Top Module
	output 	struct_types::axi_rd_data 	rd_data_fifo_in,
	output 	struct_types::axi_wr_resp 	wr_resp_fifo_in	
);

/*------------------------------Wrap As AXI----------------------------------*/	
	always_comb begin
		if (is_wr) begin
			wr_resp_fifo_in.bid = 
		end
			
	end



endmodule