/*------------------------------------------------------------------------------
 * File          : axi_interface.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Jul 5, 2025
 * Description   : AXI bus interface definition.
 *
 *   Declares all five AXI channels (AW, W, B, AR, R) with parameterisable
 *   widths and exposes two modports:
 *     - master : used by the AXI master / testbench model
 *     - slave  : used by axi_slave_wr and axi_slave_rd (receives requests,
 *                drives responses)
 *
 *   Using a SystemVerilog interface enforces correct signal directions at
 *   compile time and simplifies port lists throughout the design.
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

interface axi_interface #(
	parameter ADDR_WIDTH    = 32,
	parameter DATA_WIDTH    = 64,
	parameter ID_WIDTH      = 32,
	parameter RESP_WIDTH   	= 2,
	parameter MAX_SIZE		= 3,
	parameter MAX_LEN		= 8
);

	// Write Request Channel (AW)
	logic                      awvalid; // Master has a valid write address
	logic                      awready; // Slave is ready to accept the address
	logic [ID_WIDTH-1:0]       awid;    // Write transaction identifier
	logic [ADDR_WIDTH-1:0]     awaddr;  // Write burst base address
	logic [MAX_LEN-1:0]		   awlen;   // Burst length minus 1 (0=1 beat, 7=8 beats)
	logic [MAX_SIZE-1:0]       awsize;  // Bytes per beat, encoded as log2 (3=8 B, 2=4 B, …)

	// Write Data Channel (W)
	logic                      	wvalid; // Master has valid write data
	logic                      	wready; // Slave is ready to accept write data
	logic [DATA_WIDTH-1:0]     	wdata;  // 64-bit write data
	logic [DATA_WIDTH/8-1:0]	wstrb;  // Byte enables (1 bit per byte lane)
	logic                      	wlast;  // Asserted on the last beat of a burst

	// Write Response Channel (B)
	logic                      	bvalid; // Slave has a valid write response
	logic                      	bready; // Master is ready to accept the response
	logic [ID_WIDTH-1:0]       	bid;    // Echoes the awid of the completed transaction
	logic [RESP_WIDTH-1:0]    	bresp;  // Response code (OKAY=2'b00, SLVERR=2'b10)

	// Read Request Channel (AR)
	logic                      	arvalid; // Master has a valid read address
	logic                      	arready; // Slave is ready to accept the address
	logic [ID_WIDTH-1:0]       	arid;    // Read transaction identifier
	logic [ADDR_WIDTH-1:0]     	araddr;  // Read burst base address
	logic [MAX_LEN-1:0]			arlen;   // Burst length minus 1
	logic [MAX_SIZE-1:0]		arsize;  // Bytes per beat, encoded as log2

	// Read Data Channel (R)
	logic                      rvalid;  // Slave has valid read data
	logic                      rready;  // Master is ready to accept read data
	logic [ID_WIDTH-1:0]       rid;     // Echoes the arid of the corresponding request
	logic [DATA_WIDTH-1:0]     rdata;   // 64-bit read data
	logic [RESP_WIDTH-1:0]    rresp;   // Response code
	logic                      rlast;   // Asserted on the last beat of a read burst




	// Master modport
	modport master (
		output awvalid,
		output awaddr,
		output awid,
		output awlen,
		output awsize,
		input  awready,

		output wvalid,
		output wdata,
		output wstrb,
		output wlast,
		input  wready,

		input  bvalid,
		input  bresp,
		input  bid,
		output bready,

		output arvalid,
		output araddr,
		output arid,
		output arlen,
		output arsize,
		input  arready,

		input  rvalid,
		input  rdata,
		input  rlast,
		input  rresp,
		input  rid,
		output rready
	);

	// Slave modport
	modport slave (
		input  awvalid,
		input  awaddr,
		input  awid,
		input  awlen,
		input  awsize,
		output awready,

		input  wvalid,
		input  wdata,
		input  wstrb,
		input  wlast,
		output wready,

		output bvalid,
		output bresp,
		output bid,
		input  bready,

		input  arvalid,
		input  araddr,
		input  arid,
		input  arlen,
		input  arsize,
		output arready,

		output rvalid,
		output rdata,
		output rlast,
		output rresp,
		output rid,
		input  rready
	);

endinterface : axi_interface
