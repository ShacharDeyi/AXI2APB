/*------------------------------------------------------------------------------
 * File          : top_module.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Jul 5, 2025
 * Description   :
 *------------------------------------------------------------------------------*/


module top_module 
import struct_types::*;
(
	input logic clk,
	input logic reset,
	axi_interface.slave  axi, // External AXI bus
	apb_interface.master apb  // External APB bus
);
/*------------------------------WRITE REQ & DATA----------------------------------*/
// 1. Handling write requests from master to queue:
	logic wr_req_push_n;
	axi_wr_req wr_req_fifo_in;
	logic wr_req_full;
	axi_wr_req wr_req_fifo_out;
	logic wr_req_empty;
	
	logic wr_data_push_n;
	axi_wr_data wr_data_fifo_in;
	logic wr_data_full;
	axi_wr_data wr_data_fifo_out;
	logic wr_data_empty;

	// 2. Instantiate the AXI Slave Write Controller
	// This module decides WHEN to push, but doesn't contain the FIFO
	axi_slave_wr u_axi_slave_wr (
		.clk            (clk),
		.rst_n          (!reset),
		.axi            (axi),
		// Connections to the FIFO ports
		.req_fifo_push_n    (wr_req_push_n),
		.req_fifo_data_out  (wr_req_fifo_in),
		.req_fifo_full      (wr_req_full),
		.data_fifo_push_n    (wr_data_push_n),
		.data_fifo_data_out  (wr_data_fifo_in),
		.data_fifo_full      (wr_data_full)
		
	);

	// 3. Instantiate the FIFO (The "Queue" from your diagram)
	// Using the parameters from your FIFO description
	DW_fifo_s1_sf #(
		.width($bits(axi_wr_req)),
		.depth(16) 
	) axi_wr_req_queue (
		.clk            (clk),
		.rst_n          (!reset),
		.push_req_n     (wr_req_push_n),   // Driven by Slave Controller
		.data_in        (wr_req_fifo_in),  // Driven by Slave Controller
		.full           (wr_req_full),     // Feedback to Slave Controller
		.data_out       (wr_req_fifo_out), // Goes to the Disassembler
		.empty          (wr_req_empty),    // Goes to the Disassembler
		.pop_req_n      (wr_req_pop_n)     // Driven by Disassembler
	);
	
	DW_fifo_s1_sf #(
		.width($bits(axi_wr_req)),
		.depth(16) 
	) axi_wr_data_queue (
		.clk            (clk),
		.rst_n          (!reset),
		.push_req_n     (wr_data_push_n),   // Driven by Slave Controller
		.data_in        (wr_data_fifo_in),  // Driven by Slave Controller
		.full           (wr_data_full),     // Feedback to Slave Controller
		.data_out       (wr_data_fifo_out), // Goes to the Disassembler
		.empty          (wr_data_empty),    // Goes to the Disassembler
		.pop_req_n      (wr_data_pop_n)     // Driven by Disassembler
	);

	/*------------------------------READ REQ----------------------------------*/
	//1. Handling read requests from master to queue:
	logic rd_req_push_n;
	axi_rd_req rd_req_fifo_in;
	logic rd_req_full;
	axi_rd_req rd_req_fifo_out;
	logic rd_req_empty;

	// 2. Instantiate the AXI Slave Write Controller
	// This module decides WHEN to push, but doesn't contain the FIFO
	axi_slave_rd u_axi_slave_rd (
		.clk            (clk),
		.rst_n          (!reset),
		.axi            (axi),
		// Connections to the FIFO ports
		.fifo_push_n    (rd_req_push_n),
		.fifo_data_out  (rd_req_fifo_in),
		.fifo_full      (rd_req_full)
	);

	// 3. Instantiate the FIFO (The "Queue" from your diagram)
	// Using the parameters from your FIFO description
	DW_fifo_s1_sf #(
		.width($bits(axi_rd_req)),
		.depth(16) 
	) axi_rd_req_queue (
		.clk            (clk),
		.rst_n          (!reset),
		.push_req_n     (rd_req_push_n),   // Driven by Slave Controller
		.data_in        (rd_req_fifo_in),  // Driven by Slave Controller
		.full           (rd_req_full),     // Feedback to Slave Controller
		.data_out       (rd_req_fifo_out), // Goes to the Disassembler
		.empty          (rd_req_empty),    // Goes to the Disassembler
		.pop_req_n      (rd_req_pop_n)     // Driven by Disassembler
	);
	// ... Other modules like Manager and Disassembler go here
endmodule