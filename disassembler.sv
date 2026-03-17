/*------------------------------------------------------------------------------
 * File          : disassembler.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Oct 25, 2025
 * Description   : Splits one 64-bit AXI request into two 32-bit APB requests.
 *                 LSB covers the lower 32 bits (original address).
 *                 MSB covers the upper 32 bits (original address + 4).
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

/*------------------------------Split into 2 APB requests----------------------------------*/
// The 64-bit AXI word is split into two 32-bit APB words:
//   LSB transaction -> original AXI address       (data[31:0])
//   MSB transaction -> original AXI address + 4   (data[63:32])
//
// valid_lsb / valid_msb for writes: true only if the corresponding byte-enable
// lanes are non-zero (i.e. there is data to write in that half).
// For reads: both halves are always requested.

	always_comb begin
		// ---- LSB (lower 32 bits, lower 4 byte lanes) ----
		apb_req_fifo_in_lsb.paddr   = is_wr ? wr_req_fifo_out.awaddr : rd_req_fifo_out.araddr;
		apb_req_fifo_in_lsb.pwrite  = is_wr;
		apb_req_fifo_in_lsb.pwdata  = wr_data_fifo_out.wdata[PDATA_WIDTH-1:0];
		apb_req_fifo_in_lsb.pstrb   = wr_data_fifo_out.wstrb[PSTRB_WIDTH-1:0];
		// Response fields are driven by the APB slave; zero them on the request path
		apb_req_fifo_in_lsb.pready  = 1'b0;
		apb_req_fifo_in_lsb.prdata  = '0;
		apb_req_fifo_in_lsb.pslverr = 1'b0;
		valid_lsb = |wr_data_fifo_out.wstrb[PSTRB_WIDTH-1:0] | !is_wr;

		// ---- MSB (upper 32 bits, upper 4 byte lanes) ----
		// Address is base + 4 bytes (not OFFSET_APB=32, which pointed 28 bytes too far)
		apb_req_fifo_in_msb.paddr   = (is_wr ? wr_req_fifo_out.awaddr : rd_req_fifo_out.araddr) + 32'd4;
		apb_req_fifo_in_msb.pwrite  = is_wr;
		apb_req_fifo_in_msb.pwdata  = wr_data_fifo_out.wdata[DATA_WIDTH-1:PDATA_WIDTH];
		apb_req_fifo_in_msb.pstrb   = wr_data_fifo_out.wstrb[WSTRB_WIDTH-1:PSTRB_WIDTH];
		apb_req_fifo_in_msb.pready  = 1'b0;
		apb_req_fifo_in_msb.prdata  = '0;
		apb_req_fifo_in_msb.pslverr = 1'b0;
		valid_msb = |wr_data_fifo_out.wstrb[WSTRB_WIDTH-1:PSTRB_WIDTH] | !is_wr;
	end

endmodule