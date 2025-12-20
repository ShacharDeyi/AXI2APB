/*------------------------------------------------------------------------------
 * File          : axi_slave_rd.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 * Description   :
 *------------------------------------------------------------------------------*/
module axi_slave_rd 
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	axi_interface.slave axi,
	
	// Ports to talk to the FIFO sitting in the Top Module
	output logic               		fifo_push_n,
	output struct_types::axi_rd_req fifo_data_out,
	input  logic              		fifo_full
);

	// AXI Protocol Logic: Ready only if the FIFO has room
	assign axi.arready = !fifo_full;

	// Prepare the packet
	always_comb begin
		fifo_data_out.arid    = axi.arid;
		fifo_data_out.araddr  = axi.araddr;
		fifo_data_out.arlen   = axi.arlen;
	end

	// We push only when Valid and Ready handshake occurs
	assign fifo_push_n = !(axi.arvalid && axi.arready);

endmodule