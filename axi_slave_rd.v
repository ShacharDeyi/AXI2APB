/*------------------------------------------------------------------------------
 * File          : axi_slave_rd.v
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 1, 2025
 * Description   :
 *------------------------------------------------------------------------------*/

module axi_slave_rd #(
	parameter ADDR_WIDTH    = 32,
	parameter DATA_WIDTH    = 64,
	parameter ID_WIDTH      = 32
	
)(
	input  logic                clk,
	input  logic                rst_n,
	
	axi_interface.slave			axi,
	
	// Response from controller
	input logic [DATA_WIDTH-1:0] 	rsp_rdata,
	input logic                		rsp_valid,
	input logic                		rsp_error,
	input logic	[ID_WIDTH-1:0]		rsp_id,

	// APB bridge controller
	input 	logic 					req_ready, //can send new request
	output  logic                	req_valid,   // new request available
	output  logic [ADDR_WIDTH-1:0] 	req_addr,
	output  logic                	req_read,   // 1=write, 0=read
	output 	logic [ID_WIDTH-1:0] 	req_id
	
);

typedef enum logic [1:0] {
	IDLE,
	REQUEST,
	RESPONSE
} axi_state_e;

axi_state_e state, next_state;

always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n)
	  state <= IDLE;
	else
	  state <= next_state;
end

always_comb begin
	axi.arready = 1'b0; 
	axi.rvalid = 1'b0;
	axi.rdata =  '0;
	axi.rlast = 1'b0;
	axi.rresp = '0;
	axi.rid = '0;
	
	req_valid = 1'b0;
	req_addr = '0;
	req_read = 1'b0;
	req_id = '0;
	next_state  = state;

	
	case(state)
		IDLE: begin
			axi.arready = 1'b1; 
			req_valid = axi.arvalid;
			if (axi.arvalid) begin
				next_state = REQUEST;
			end
		end
		
		REQUEST: begin
			req_addr = axi.araddr;
			req_id = axi.arid;
			next_state = RESPONSE;			
		end
		
		RESPONSE: begin
			axi.rvalid = rsp_valid;
			if(axi.rvalid) begin
				if (axi.rready) begin
					axi.rdata = rsp_rdata;
					//axi.rresp = 
					axi.rid = rsp_id;
//					if() begin
//						axi.rlast = 
//						next_state = IDLE;
//					end
				end
			end
			
		end
	endcase
end
	
endmodule