/*------------------------------------------------------------------------------
 * File          : apb_wrapper.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 27, 2026
 * Description   :
 *------------------------------------------------------------------------------*/

module apb_wrapper
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	
	input 	logic						is_wr, 			// wr = 1, rd = 0
	
	input	struct_types::axi_rd_req	rd_req_fifo_out, 
	input	struct_types::axi_wr_req	wr_req_fifo_out,
	input 	struct_types::axi_wr_data 	wr_data_fifo_out,
	
	// Ports to talk to the output FIFO (to apb_master) sitting in the Top Module
	output 	struct_types::apb_struct 	apb_req_fifo_in
	
);
/*------------------------------Wrap As APB----------------------------------*/	

	// Repack as apb packet
	// Prepare the packet
	always_comb begin
		apb_req_fifo_in.paddr  = is_wr ? wr_req_fifo_out.awaddr :  rd_req_fifo_out.araddr;
		apb_req_fifo_in.pwrite = is_wr;
		apb_req_fifo_in.pwdata = wr_data_fifo_out.wdata;
		apb_req_fifo_in.pstrb = wr_data_fifo_out.wstrb;
	end

endmodule