/*------------------------------------------------------------------------------
 * File          : axi_interface.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Jul 5, 2025
 * Description   :
 *------------------------------------------------------------------------------*/

interface axi_interface #(
	parameter ADDR_WIDTH    = 32,
	parameter DATA_WIDTH    = 64,
	parameter ID_WIDTH      = 32,
	parameter RRESP_WIDTH   = 2,
	parameter BRESP_WIDTH   = 2
);

	// Write Address Channel
	logic                      awvalid; /*Valid indicator*/
	logic [ADDR_WIDTH-1:0]     awaddr; /*Transaction address*/
	logic [ID_WIDTH-1:0]       awid; /*Transaction identifier for the write channels*/
	logic                      awready; /*Ready indicator*/

	// Write Data Channel
	logic                      wvalid; /*Valid indicator*/ 
	logic [DATA_WIDTH-1:0]     wdata; /*DATA_WIDTH*/
	logic                      wlast; /*Last write data*/
	logic                      wready; /*Ready indicator*/

	// Write Response Channel 
	logic                      bvalid; /*Valid indicator*/
	logic [BRESP_WIDTH-1:0]    bresp; /*Write response*/
	logic [ID_WIDTH-1:0]       bid; /*Transaction identifier for the write channels*/
	logic                      bready; /*Ready indicator*/

	// Read Address Channel
	logic                      arvalid; /*Valid indicator*/
	logic [ADDR_WIDTH-1:0]     araddr; /*Transaction address*/
	logic [ID_WIDTH-1:0]       arid; /*Transaction identifier for the write channels*/
	logic                      arready; /*Ready indicator*/

	// Read Data Channel
	logic                      rvalid;
	logic [DATA_WIDTH-1:0]     rdata;
	logic                      rlast;
	logic [RRESP_WIDTH-1:0]    rresp;
	logic [ID_WIDTH-1:0]       rid;
	logic                      rready;

	// Master modport
	modport master (
		output awvalid,
		output awaddr,
		output awid,
		input  awready,

		output wvalid,
		output wdata,
		output wlast,
		input  wready,

		input  bvalid,
		input  bresp,
		input  bid,
		output bready,

		output arvalid,
		output araddr,
		output arid,
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
		output awready,

		input  wvalid,
		input  wdata,
		input  wlast,
		output wready,

		output bvalid,
		output bresp,
		output bid,
		input  bready,

		input  arvalid,
		input  araddr,
		input  arid,
		output arready,

		output rvalid,
		output rdata,
		output rlast,
		output rresp,
		output rid,
		input  rready
	);

endinterface : axi_interface