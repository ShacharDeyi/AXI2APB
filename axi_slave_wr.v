/*------------------------------------------------------------------------------
 * File          : axi_slave_wr.v
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 1, 2025
 * Description   :
 *------------------------------------------------------------------------------*/

module axi_slave_wr #(
	parameter ADDR_WIDTH    = 32,
	parameter DATA_WIDTH    = 64,
	parameter ID_WIDTH      = 32
	
)(
	input  logic                clk,
	input  logic                rst_n,
	
	axi_interface.slave			axi,
	
	// Response from controller
	input logic                		rsp_valid,
	input logic                		rsp_error,
	input logic [ID_WIDTH-1:0] 		rsp_id,
	// APB bridge controller
	input logic 					req_ready, //can send new request
	output  logic                	req_valid,   // new request available
	output  logic [ADDR_WIDTH-1:0] 	req_addr,
	output  logic [DATA_WIDTH-1:0] 	req_wdata,
	output  logic                	req_write,   // 1=write, 0=read
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
	
	axi.awready = 1'b0;
	axi.wready = 1'b0;
	axi.bvalid = 1'b0;
	axi.bresp = '0;
	axi.bid = '0;
	
	req_valid = 1'b0;
	req_addr = 1'b0;
	req_wdata = 1'b0;
	req_write = 1'b1;
	req_id = '0;
	next_state  = state;
	
	case (state)
		IDLE: begin
			axi.awready = 1'b1;
			axi.wready = 1'b1;
			req_valid = axi.awvalid;
			if (axi.awvalid && axi.wvalid) begin
				next_state = REQUEST;
			end
		end
		
		REQUEST: begin
			req_addr = axi.awaddr;
			req_wdata = axi.wdata;
			req_id = axi.awid;
			if(axi.wlast) begin
				next_state = RESPONE;
			end	
		end
		
		RESPONE: begin
			axi.bvalid = rsp_valid;
			if(rsp_valid) begin
				if(axi.bready) begin
					axi.bresp = 3'b000;   //'okay'
					axi.bid = rsp_id;
					next_state = IDLE;
				end
			end
			
		end
	endcase
end
	
endmodule