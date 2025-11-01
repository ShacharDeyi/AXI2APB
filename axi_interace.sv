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
	wire                      awvalid; /*Valid indicator*/
	wire [ADDR_WIDTH-1:0]     awaddr; /*Transaction address*/
	wire [ID_WIDTH-1:0]       awid; /*Transaction identifier for the write channels*/
	wire                      awready; /*Ready indicator*/

	// Write Data Channel
	wire                      wvalid; /*Valid indicator*/ 
	wire [DATA_WIDTH-1:0]     wdata; /*DATA_WIDTH*/
	wire                      wlast; /*Last write data*/
	wire                      wready; /*Ready indicator*/

	// Write Response Channel 
	wire                      bvalid; /*Valid indicator*/
	wire [BRESP_WIDTH-1:0]    bresp; /*Write response*/
	wire [ID_WIDTH-1:0]       bid; /*Transaction identifier for the write channels*/
	wire                      bready; /*Ready indicator*/

	// Read Address Channel
	wire                      arvalid; /*Valid indicator*/
	wire [ADDR_WIDTH-1:0]     araddr; /*Transaction address*/
	wire [ID_WIDTH-1:0]       arid; /*Transaction identifier for the write channels*/
	wire                      arready; /*Ready indicator*/

	// Read Data Channel
	wire                      rvalid;
	wire [DATA_WIDTH-1:0]     rdata;
	wire                      rlast;
	wire [RRESP_WIDTH-1:0]    rresp;
	wire [ID_WIDTH-1:0]       rid;
	wire                      rready;

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
