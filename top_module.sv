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
/*------------------------------AXI WRITE REQ & DATA & RESP----------------------------------*/
// 1. Handling write requests from master to queue:
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
	//1. Handling read requests from master to queue:
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
	//1. Handling read requests from master to queue:
	logic 		apb_req_push_n;
	logic 		apb_req_pop_n;
	apb_struct 	apb_req_fifo_in;
	logic 		apb_req_full;
	apb_struct 	apb_req_fifo_out;
	logic 		apb_req_empty;

	logic 		apb_resp_push_n;
	apb_struct 	apb_resp_fifo_in;
	logic 		apb_resp_full;
	apb_struct 	apb_resp_fifo_out;
	logic 		apb_resp_empty;
	logic 		apb_data_pop_n;
	
	
	/*------------------------------AXI WRITE REQ & DATA & RESP----------------------------------*/

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
		.data_fifo_push_n   (wr_data_push_n),
		.data_fifo_data_out (wr_data_fifo_in),
		.data_fifo_full     (wr_data_full),
		.resp_fifo_pop_n    (wr_resp_pop_n),
		.resp_fifo_data_out	(wr_resp_fifo_out),
		.resp_fifo_empty	(wr_resp_empty)		
		
	);

	// 3. Instantiate the FIFO 
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
		.width($bits(axi_wr_data)),
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
	
	//send to disassembler
	
	disassembler u_disassembler_wr (
		.clk            	(clk),
		.rst_n          	(!reset),
		// Connections to the FIFO ports
		.req_pop_n    		(wr_req_pop_n),
		.wr_data_pop_n  	(wr_data_pop_n),
		.is_wr				(1'b1), //check what to put here
		.req_empty      	(wr_req_full),
		.wr_data_empty   	(wr_data_empty),
		.rd_req_fifo_out 	('b0), // what to put here?
		.wr_req_fifo_out    (wr_req_fifo_out),
		.wr_data_fifo_out   (wr_data_fifo_out),
		.apb_req_push_n		(apb_req_push_n),
		.apb_req_fifo_in	(apb_req_fifo_in),
		.apb_req_full		(apb_req_full)
	);
	
	//send wr resp to axi

	DW_fifo_s1_sf #(
		.width($bits(axi_wr_resp)),
		.depth(16) 
	) axi_wr_resp_queue (
		.clk            (clk),
		.rst_n          (!reset),
		.push_req_n     (wr_resp_push_n),   // Driven by Assembler 
		.data_in        (wr_resp_fifo_in),  // Driven by Assembler 
		.full           (wr_resp_full),     // Feedback to Assembler
		.data_out       (wr_resp_fifo_out), // Goes to the Slave Controller
		.empty          (wr_resp_empty),    // Goes to the Slave Controller
		.pop_req_n      (wr_resp_pop_n)     // Driven by Slave Controller 
	);


	/*------------------------------AXI READ REQ & DATA----------------------------------*/
	// 2. Instantiate the AXI Slave Write Controller
	// This module decides WHEN to push, but doesn't contain the FIFO
	axi_slave_rd u_axi_slave_rd (
		.clk            (clk),
		.rst_n          (!reset),
		.axi            (axi),
		// Connections to the FIFO ports
		.req_fifo_push_n    (rd_req_push_n),
		.req_fifo_data_out  (rd_req_fifo_in),
		.req_fifo_full      (rd_req_full),
		.data_fifo_pop_n	(rd_data_pop_n),
		.data_fifo_data_in 	(rd_data_fifo_in),
		.data_fifo_empty	(rd_data_empty)
	);

	// 3. Instantiate the FIFO
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
	
	disassembler u_disassembler_rd (
		.clk            	(clk),
		.rst_n          	(!reset),
		// Connections to the FIFO ports
		.req_pop_n    		(rd_req_pop_n),
		.wr_data_pop_n  	(1'b0), // what to put here?
		.is_wr				(1'b0), //check what to put here
		.req_empty      	(rd_req_full),
		.wr_data_empty   	('b0), // what to put here?
		.rd_req_fifo_out 	(rd_req_fifo_out),
		.wr_req_fifo_out    ('b0), // what to put here?
		.wr_data_fifo_out   ('b0), // what to put here?
		.apb_req_push_n		(apb_req_push_n),
		.apb_req_fifo_in	(apb_req_fifo_in),
		.apb_req_full		(apb_req_full)
	);
	
	DW_fifo_s1_sf #(
		.width($bits(axi_rd_data)),
		.depth(16) 
	) axi_rd_data_queue (
		.clk            (clk),
		.rst_n          (!reset),
		.push_req_n     (rd_data_push_n),  	// Driven by Assembler
		.data_in        (rd_data_fifo_in),  // Driven by Assembler
		.full           (rd_data_full),     // Feedback to Assembler
		.data_out       (rd_data_fifo_out), // Goes to the Slave Controller
		.empty          (rd_data_empty),    // Goes to the Slave Controller
		.pop_req_n      (rd_data_pop_n)     // Driven by Slave Controller
	);
	// ... Other modules like Manager and Disassembler go here
	
	
	
/*------------------------------APB REQ & RESP----------------------------------*/
	
	//queue from disassembler to apb_master
	DW_fifo_s1_sf #(
		.width($bits(apb_struct)),
		.depth(16) 
	) apb_req_queue (
		.clk            (clk),
		.rst_n          (!reset),
		.push_req_n     (apb_req_push_n),   // Driven by Disassembler 
		.data_in        (apb_req_fifo_in),  // Driven by Disassembler
		.full           (apb_req_full),     // Feedback to Disassembler
		.data_out       (apb_req_fifo_out), // Goes to the Master Controller
		.empty          (apb_req_empty),    // Goes to the Master Controller
		.pop_req_n      (apb_req_pop_n)     // Driven by Master Controller
	);
	
	
	//apb master module, communicates with queue from disassembler and queue into assembler
	// This module decides WHEN to push, but doesn't contain the FIFO
	apb_master u_apb_master (
		.clk            (clk),
		.rst_n          (!reset),
		.apb            (apb),
		// Connections to the FIFO ports
		.req_fifo_pop_n    	(apb_req_pop_n),
		.req_fifo_data_out  (apb_req_fifo_out),
		.req_fifo_empty     (apb_req_empty),
		.resp_fifo_push_n	(apb_resp_push_n),
		.resp_fifo_data_in 	(apb_resp_fifo_in),
		.resp_fifo_full		(apb_resp_full)
	);
	

	
	//input from apb_master into queue waiting to go to assembler
	
	DW_fifo_s1_sf #(
		.width($bits(apb_struct)),
		.depth(16) 
	) apb_resp_queue (
		.clk            (clk),
		.rst_n          (!reset),
		.push_req_n     (apb_resp_push_n),   // Driven by Master Controller
		.data_in        (apb_resp_fifo_in),  // Driven by Master Controller
		.full           (apb_resp_full),     // Feedback to Master Controller
		.data_out       (apb_resp_fifo_out), // Goes to the Assembler
		.empty          (apb_resp_empty),    // Goes to the Assembler
		.pop_req_n      (apb_data_pop_n)     // Driven by Assembler
	);
	
	assembler u_assembler (
		.clk            	(clk),
		.rst_n          	(!reset),
		// Connections to the FIFO ports
		.apb_data_pop_n    	(apb_data_pop_n),
		.apb_resp_empty  	(apb_resp_empty), 
		.apb_resp_fifo_out  (apb_resp_fifo_out),
		.resp_push_n   		('b0), // needs to be adjusted by manager to feed into correct fifo?
		.rd_data_fifo_in 	(rd_data_fifo_in),
		.wr_resp_fifo_in    (wr_resp_fifo_in),
		.rd_data_full 		(rd_data_full),
		.wr_resp_full		(wr_resp_full)
	);
	
	
endmodule