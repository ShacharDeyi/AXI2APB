/*------------------------------------------------------------------------------
 * File          : axi_interface.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Jul 5, 2025
 * Description   :
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

interface axi_interface #(
	parameter ADDR_WIDTH    = 32,
	parameter DATA_WIDTH    = 64,
	parameter ID_WIDTH      = 32,
	parameter RRESP_WIDTH   = 2,
	parameter BRESP_WIDTH   = 2,
	parameter MAX_SIZE		= 3,
	parameter MAX_LEN		= 8
);

	// Write Request Channel
	logic                      awvalid; /*Valid indicator*/
	logic                      awready; /*Ready indicator*/
	logic [ID_WIDTH-1:0]       awid; /*Transaction identifier for the write channels*/
	logic [ADDR_WIDTH-1:0]     awaddr; /*Transaction address*/
	logic [MAX_LEN-1:0]		   awlen; /*Defines the number of data transfers in a transaction (AXI4: 0=1 beat, 255=256 beats)*/
	logic [MAX_SIZE-1:0]       awsize; /*Size indicates the maximum number of bytes in each data transfer, 8bytes = 011, 64byes = 110*/

	// Write Data Channel
	logic                      	wvalid; /*Valid indicator*/ 
	logic                      	wready; /*Ready indicator*/
	logic [DATA_WIDTH-1:0]     	wdata; /*DATA_WIDTH*/
	logic [DATA_WIDTH/8-1:0]	wstrb; /*PSTRB indicates which byte lanes to update during a write transfer*/
	logic                      	wlast; /*Last write data*/

	// Write Response Channel 
	logic                      	bvalid; /*Valid indicator*/
	logic                      	bready; /*Ready indicator*/
	logic [ID_WIDTH-1:0]       	bid; /*Transaction identifier for the write channels*/
	logic [BRESP_WIDTH-1:0]    	bresp; /*Write response*/



	// Read Request Channel
	logic                      	arvalid; /*Valid indicator*/
	logic                      	arready; /*Ready indicator*/
	logic [ID_WIDTH-1:0]       	arid; /*Transaction identifier for the write channels*/
	logic [ADDR_WIDTH-1:0]     	araddr; /*Transaction address*/
	logic [MAX_LEN-1:0]			arlen; /*Defines the number of data transfers in a transaction (AXI4: 0=1 beat, 255=256 beats)*/
	logic [MAX_SIZE-1:0]		arsize;/*Size indicates the maximum number of bytes in each data transfer, 8bytes = 011, 64byes = 110*/

	// Read Data Channel
	logic                      rvalid;
	logic                      rready;
	logic [ID_WIDTH-1:0]       rid;
	logic [DATA_WIDTH-1:0]     rdata;
	logic [RRESP_WIDTH-1:0]    rresp;
	logic                      rlast;




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
