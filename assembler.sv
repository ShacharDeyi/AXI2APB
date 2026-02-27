/*------------------------------------------------------------------------------
 * File          : assembler.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 24, 2026
 * Description   :
 *------------------------------------------------------------------------------*/

module assembler 
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	
	// Ports to talk to the input FIFO (from apb_slave) sitting in the Top Module
	input	struct_types::apb_struct	apb_resp_fifo_out_msb, 
	input	struct_types::apb_struct	apb_resp_fifo_out_lsb, 
	
	// Ports to talk to the output FIFO (to axi_master) sitting in the Top Module
	output 	struct_types::axi_rd_data 	rd_data_fifo_in,
	output 	struct_types::axi_wr_resp 	wr_resp_fifo_in
	
);
/*------------------------------Restore from RF----------------------------------*/

	// Prepare the packet
	always_comb begin
		if (is_wr) begin
			wr_resp_fifo_in.arid    = axi.bid;
			wr_resp_fifo_in.araddr  = axi.araddr;
			wr_resp_fifo_in.arlen   = axi.arlen;
		end

	end
	
	
/*------------------------------Pack into 1 AXI response----------------------------------*/	
 
	// Prepare the packet
	always_comb begin
		if (is_wr) begin
			axi_wr_resp.bid = 
		end
		axi.rid = data_fifo_data_in.rid;
		axi.rdata = data_fifo_data_in.rdata;
		axi.rresp = data_fifo_data_in.rresp;
		axi.rlast = data_fifo_data_in.rlast;
	end

	// We push only when Valid and Ready handshake occurs
	assign data_fifo_pop_n = !(axi.rvalid && axi.rready);

endmodule