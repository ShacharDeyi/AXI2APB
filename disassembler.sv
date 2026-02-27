/*------------------------------------------------------------------------------
 * File          : disassembler.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Oct 25, 2025
 * Description   :
 *------------------------------------------------------------------------------*/

module disassembler 
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,

	input 	logic						is_wr, 			// wr = 1, rd = 0
	
	input	struct_types::axi_rd_req	rd_req_fifo_out, 
	input	struct_types::axi_wr_req	wr_req_fifo_out,
	input 	struct_types::axi_wr_data 	wr_data_fifo_out,
	
	output	struct_types::apb_struct 	apb_req_fifo_in_msb,
	output 	logic						valid_msb,
	output	struct_types::apb_struct 	apb_req_fifo_in_lsb,
	output  logic 						valid_lsb

	
);
/*------------------------------Save in RF----------------------------------*/

	always_comb begin
		
		req.id 	= is_wr ? wr_req_fifo_out.awid : rd_req_fifo_out.arid;
		req.addr  	= is_wr ? wr_req_fifo_out.awaddr :  rd_req_fifo_out.araddr;
		req.wdata 	= wr_data_fifo_out.wdata;
		
	end
	
/*------------------------------Split into 2 apb requests----------------------------------*/

	always_comb begin
		apb_req_fifo_in_msb.paddr  = (is_wr ? wr_req_fifo_out.awaddr :  rd_req_fifo_out.araddr) + OFFSET_APB;
		apb_req_fifo_in_msb.pwdata = wr_data_fifo_out.wdata[DATA_WIDTH-1:PDATA_WIDTH];
		apb_req_fifo_in_msb.pstrb = wr_data_fifo_out.wstrb[WSTRB_WIDTH-1:PSTRB_WIDTH];
		valid_msb = |wr_data_fifo_out.wstrb[WSTRB_WIDTH-1:PSTRB_WIDTH] | !is_wr; //read axi req translate always to 2 apb transactions

		apb_req_fifo_in_lsb.paddr  = is_wr ? wr_req_fifo_out.awaddr :  rd_req_fifo_out.araddr;
		apb_req_fifo_in_lsb.pwdata = wr_data_fifo_out.wdata[PDATA_WIDTH-1:0];
		apb_req_fifo_in_lsb.pstrb = wr_data_fifo_out.wstrb[PSTRB_WIDTH-1:0];
		valid_lsb = |wr_data_fifo_out.wstrb[PSTRB_WIDTH-1:0] | !is_wr;
	end


endmodule