/*------------------------------------------------------------------------------
 * File          : top_module.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Jul 5, 2025
 * Description   : AXI-to-APB bridge top level.
 *------------------------------------------------------------------------------*/

module top_module 
import struct_types::*;
(
	input logic clk,
	input logic rst_n,          // Active-low reset (was 'reset' active-high -- fixed)
	axi_interface.slave  axi,   // External AXI bus
	apb_interface.master apb    // External APB bus
);

/*------------------------------AXI WRITE REQ & DATA & RESP----------------------------------*/
	logic wr_req_push_n;
	axi_wr_req wr_req_fifo_in;
	logic wr_req_full;
	axi_wr_req wr_req_fifo_out;
	logic wr_req_empty;
	logic wr_req_pop_n;
	
	logic wr_data_push_n;
	axi_wr_data wr_data_fifo_in;
	logic wr_data_full;
	axi_wr_data wr_data_fifo_out;
	logic wr_data_empty;
	logic wr_data_pop_n;
	
	logic wr_resp_push_n;
	logic wr_resp_pop_n;
	axi_wr_resp wr_resp_fifo_in;
	logic wr_resp_full;
	axi_wr_resp wr_resp_fifo_out;
	logic wr_resp_empty;

/*------------------------------AXI READ REQ & DATA----------------------------------*/
	logic rd_req_push_n;
	axi_rd_req rd_req_fifo_in;
	logic rd_req_full;
	axi_rd_req rd_req_fifo_out;
	logic rd_req_empty;
	logic rd_req_pop_n;

	logic rd_data_push_n;
	logic rd_data_pop_n;
	axi_rd_data rd_data_fifo_in;
	logic rd_data_full;
	axi_rd_data rd_data_fifo_out;
	logic rd_data_empty;

/*------------------------------APB REQ & RESP----------------------------------*/
	logic       apb_req_push_n;
	logic       apb_req_pop_n;
	apb_struct  apb_req_fifo_in;
	logic       apb_req_full;
	apb_struct  apb_req_fifo_out;
	logic       apb_req_empty;

	logic       apb_resp_push_n;
	apb_struct  apb_resp_fifo_in;
	logic       apb_resp_full;
	apb_struct  apb_resp_fifo_out;
	logic       apb_resp_empty;
	logic       apb_data_pop_n;

/*------------------------------AXI WRITE SLAVE + FIFOs----------------------------------*/

	axi_slave_wr u_axi_slave_wr (
		.clk                (clk),
		.rst_n              (rst_n),
		.axi                (axi),
		.req_fifo_push_n    (wr_req_push_n),
		.req_fifo_data_out  (wr_req_fifo_in),
		.req_fifo_full      (wr_req_full),
		.data_fifo_push_n   (wr_data_push_n),
		.data_fifo_data_out (wr_data_fifo_in),
		.data_fifo_full     (wr_data_full),
		.resp_fifo_pop_n    (wr_resp_pop_n),
		.resp_fifo_data_out (wr_resp_fifo_out),
		.resp_fifo_empty    (wr_resp_empty)
	);

	DW_fifo_s1_sf #(
		.width($bits(axi_wr_req)),
		.depth(16)
	) axi_wr_req_queue (
		.clk        (clk),
		.rst_n      (rst_n),
		.push_req_n (wr_req_push_n),
		.data_in    (wr_req_fifo_in),
		.full       (wr_req_full),
		.data_out   (wr_req_fifo_out),
		.empty      (wr_req_empty),
		.pop_req_n  (wr_req_pop_n)
	);

	DW_fifo_s1_sf #(
		.width($bits(axi_wr_data)),
		.depth(16)
	) axi_wr_data_queue (
		.clk        (clk),
		.rst_n      (rst_n),
		.push_req_n (wr_data_push_n),
		.data_in    (wr_data_fifo_in),
		.full       (wr_data_full),
		.data_out   (wr_data_fifo_out),
		.empty      (wr_data_empty),
		.pop_req_n  (wr_data_pop_n)
	);

	DW_fifo_s1_sf #(
		.width($bits(axi_wr_resp)),
		.depth(16)
	) axi_wr_resp_queue (
		.clk        (clk),
		.rst_n      (rst_n),
		.push_req_n (wr_resp_push_n),
		.data_in    (wr_resp_fifo_in),
		.full       (wr_resp_full),
		.data_out   (wr_resp_fifo_out),
		.empty      (wr_resp_empty),
		.pop_req_n  (wr_resp_pop_n)
	);

/*------------------------------AXI READ SLAVE + FIFOs----------------------------------*/

	axi_slave_rd u_axi_slave_rd (
		.clk                (clk),
		.rst_n              (rst_n),
		.axi                (axi),
		.req_fifo_push_n    (rd_req_push_n),
		.req_fifo_data_out  (rd_req_fifo_in),
		.req_fifo_full      (rd_req_full),
		.data_fifo_pop_n    (rd_data_pop_n),
		.data_fifo_data_in  (rd_data_fifo_out),
		.data_fifo_empty    (rd_data_empty)
	);

	DW_fifo_s1_sf #(
		.width($bits(axi_rd_req)),
		.depth(16)
	) axi_rd_req_queue (
		.clk        (clk),
		.rst_n      (rst_n),
		.push_req_n (rd_req_push_n),
		.data_in    (rd_req_fifo_in),
		.full       (rd_req_full),
		.data_out   (rd_req_fifo_out),
		.empty      (rd_req_empty),
		.pop_req_n  (rd_req_pop_n)
	);

	DW_fifo_s1_sf #(
		.width($bits(axi_rd_data)),
		.depth(16)
	) axi_rd_data_queue (
		.clk        (clk),
		.rst_n      (rst_n),
		.push_req_n (rd_data_push_n),
		.data_in    (rd_data_fifo_in),
		.full       (rd_data_full),
		.data_out   (rd_data_fifo_out),
		.empty      (rd_data_empty),
		.pop_req_n  (rd_data_pop_n)
	);

/*------------------------------APB MASTER + FIFOs----------------------------------*/

	DW_fifo_s1_sf #(
		.width($bits(apb_struct)),
		.depth(16)
	) apb_req_queue (
		.clk        (clk),
		.rst_n      (rst_n),
		.push_req_n (apb_req_push_n),
		.data_in    (apb_req_fifo_in),
		.full       (apb_req_full),
		.data_out   (apb_req_fifo_out),
		.empty      (apb_req_empty),
		.pop_req_n  (apb_req_pop_n)
	);

	apb_master u_apb_master (
		.clk                (clk),
		.rst_n              (rst_n),
		.apb                (apb),
		.req_fifo_pop_n     (apb_req_pop_n),
		.req_fifo_data_out  (apb_req_fifo_out),
		.req_fifo_empty     (apb_req_empty),
		.resp_fifo_push_n   (apb_resp_push_n),
		.resp_fifo_data_in  (apb_resp_fifo_in),
		.resp_fifo_full     (apb_resp_full)
	);

	DW_fifo_s1_sf #(
		.width($bits(apb_struct)),
		.depth(16)
	) apb_resp_queue (
		.clk        (clk),
		.rst_n      (rst_n),
		.push_req_n (apb_resp_push_n),
		.data_in    (apb_resp_fifo_in),
		.full       (apb_resp_full),
		.data_out   (apb_resp_fifo_out),
		.empty      (apb_resp_empty),
		.pop_req_n  (apb_data_pop_n)
	);

/*------------------------------MANAGER (disassembler + assembler)----------------------------------*/
// The manager owns the unified disassembler and assembler internally.
// It arbitrates between read and write transactions, slices 64-bit AXI
// transfers into pairs of 32-bit APB transactions, and reassembles the
// two APB responses back into a single AXI response.

	manager u_manager (
		.clk                (clk),
		.rst_n              (rst_n),

		// AXI read request queue
		.rd_req_pop_n       (rd_req_pop_n),
		.rd_req_empty       (rd_req_empty),
		.rd_req_fifo_out    (rd_req_fifo_out),

		// AXI write request queue
		.wr_req_pop_n       (wr_req_pop_n),
		.wr_req_empty       (wr_req_empty),
		.wr_req_fifo_out    (wr_req_fifo_out),

		// AXI write data queue
		.wr_data_pop_n      (wr_data_pop_n),
		.wr_data_empty      (wr_data_empty),
		.wr_data_fifo_out   (wr_data_fifo_out),

		// APB request queue (manager -> apb_master)
		.apb_req_push_n     (apb_req_push_n),
		.apb_req_fifo_in    (apb_req_fifo_in),
		.apb_req_full       (apb_req_full),

		// APB response queue (apb_master -> manager)
		.apb_data_pop_n     (apb_data_pop_n),
		.apb_resp_empty     (apb_resp_empty),
		.apb_resp_fifo_out  (apb_resp_fifo_out),

		// AXI read data queue (manager -> axi_slave_rd)
		.rd_data_push_n     (rd_data_push_n),
		.rd_data_fifo_in    (rd_data_fifo_in),
		.rd_data_full       (rd_data_full),

		// AXI write response queue (manager -> axi_slave_wr)
		.wr_resp_push_n     (wr_resp_push_n),
		.wr_resp_fifo_in    (wr_resp_fifo_in),
		.wr_resp_full       (wr_resp_full)
	);

endmodule
