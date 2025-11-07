/*------------------------------------------------------------------------------
 * File          : apb_master.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 * Description   :
 *------------------------------------------------------------------------------*/


module apb_master #(
	parameter PADDR_WIDTH = 32,
	parameter PDATA_WIDTH = 32
	
  )(
	input  logic                clk,
	input  logic                rst_n,

	// Connect to the APB interface
	apb_interface.master        apb,

	// AXI bridge controller
	input  logic                	req_valid,   // new request available
	input  logic [PADDR_WIDTH-1:0] 	req_addr,
	input  logic [PDATA_WIDTH-1:0] 	req_wdata,
	input  logic                	req_write,   // 1=write, 0=read
	output logic                	req_ready,   // can accept new request

	// Response back to controller
	output logic [PDATA_WIDTH-1:0] 	rsp_rdata,
	output logic                	rsp_valid,
	output logic                	rsp_error
);

typedef enum logic [1:0] {
	IDLE,
	SETUP,
	ENABLE
} apb_state_e;

apb_state_e state, next_state;

always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n)
	  state <= IDLE;
	else
	  state <= next_state;
end

always_comb begin
	// Default outputs
	apb.psel    = 1'b0;
	apb.penable = 1'b0;
	apb.pwrite  = 1'b0;
	apb.paddr   = '0;
	apb.pwdata  = '0;
	apb.pstrb   = '0;
	apb.pprot   = 3'b000;
	apb.puser   = '0;

	rsp_valid   = 1'b0;
	rsp_error   = 1'b0;
	rsp_rdata   = '0;
	req_ready   = 1'b0;
	next_state  = state;

	case (state)
	  IDLE: begin
		req_ready = 1'b1; // can accept a new transaction
		if (req_valid) begin
		  next_state = SETUP;
		end
	  end

	  SETUP: begin
		apb.psel   = 1'b1;
		apb.paddr  = req_addr;
		apb.pwrite = req_write;
		apb.pwdata = req_wdata;
		apb.pstrb  = {PDATA_WIDTH/8{1'b1}};
		next_state = ENABLE;
	  end

	  ENABLE: begin
		apb.psel    = 1'b1;
		apb.penable = 1'b1;
		apb.paddr   = req_addr;
		apb.pwrite  = req_write;
		apb.pwdata  = req_wdata;
		apb.pstrb   = {PDATA_WIDTH/8{1'b1}};

		if (apb.pready) begin
		  rsp_valid = 1'b1;
		  rsp_error = apb.pslverr;
		  rsp_rdata = apb.prdata;
		  next_state = IDLE;
		end
	  end

	  default: next_state = IDLE;
	endcase
  end
endmodule : apb_master