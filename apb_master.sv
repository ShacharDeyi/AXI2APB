/*------------------------------------------------------------------------------
 * File          : apb_master.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 * Description   :
 *------------------------------------------------------------------------------*/


module apb_master 
import struct_types::*;
(
	input  logic                clk,
	input  logic                rst_n,
	apb_interface.master apb,

	// Ports to talk to the FIFO sitting in the Top Module
	output logic               		req_fifo_pop_n,
	input struct_types::apb_struct 	req_fifo_data_out,
	input  logic              		req_fifo_empty,
	output logic               		resp_fifo_push_n,
	output struct_types::apb_struct resp_fifo_data_in,
	input  logic              		resp_fifo_full
);
/*------------------------------REQ----------------------------------*/
// APB Protocol Logic: Ready only if the FIFO has room
always_ff @(posedge clk) begin
	if(!req_fifo_empty && !resp_fifo_full)
		apb.psel <= 1'b1;
	if(apb.psel)
		apb.penable <= 1'b1;
end

// Prepare the packet
always_comb begin
	apb.paddr = 	req_fifo_data_out.paddr;
	apb.pwdata = 	req_fifo_data_out.pwdata;
	apb.pwrite = 	req_fifo_data_out.pwrite;
	apb.pstrb = 	req_fifo_data_out.pstrb;

end

// We push only when Valid and Ready handshake occurs
assign req_fifo_pop_n = !(apb.psel && apb.penable && apb.pready);

/*------------------------------RESP----------------------------------*/
// APB Protocol Logic: Ready only if the FIFO has room
always_ff @(posedge clk) begin
	if(apb.pready) begin
		apb.psel <= 1'b0;
		apb.penable <= 1'b0;
	end
end

// Prepare the packet
always_comb begin
	resp_fifo_data_in.prdata =  apb.prdata;
	resp_fifo_data_in.pslverr = apb.pslverr;

end

// We push only when Valid and Ready handshake occurs
assign resp_fifo_push_n = !(apb.psel && apb.penable && apb.pready);
endmodule
	