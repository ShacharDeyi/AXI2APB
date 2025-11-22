/*------------------------------------------------------------------------------
 * File          : axi_slave_wr.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
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
	input logic [ID_WIDTH-1:0] 		rsp_id,
	// APB bridge controller
	input 	logic 					req_ready, //can send new request
	output  logic                	req_valid,   // new request available
	output  logic [ADDR_WIDTH-1:0] 	req_addr,
	output  logic [DATA_WIDTH-1:0] 	req_wdata,
	output  logic                	req_write,   // 1=write, 0=read
	output 	logic [ID_WIDTH-1:0] 	req_id

	
);

logic is_last_i;


typedef enum logic [1:0] {
	IDLE,
	REQUEST,
	DATA,
	RESPONSE
} axi_state_e;

axi_state_e state, next_state;

always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
	  state <= IDLE;
	end
	else
	  state <= next_state;
end

always_comb begin
	
	axi.awready = 1'b0;
	axi.wready = 1'b0;
	axi.bvalid = axi.bvalid;
	axi.bresp = '0;
	axi.bid = '0;
	
	req_valid = 1'b0;
	req_addr = req_addr;
	req_wdata = req_wdata;
	req_write = 1'b1;
	req_id = req_id;
	next_state  = state;
	is_last_i = is_last_i;

	
	
	case (state)
		IDLE: begin
			is_last_i = 1'b0;
			axi.awready = 1'b1;
			axi.bvalid = 1'b0;
			req_valid = axi.awvalid;
			req_addr = 32'b0;
			req_wdata = 64'b0;
			req_id = 32'b0;
			if (axi.awvalid) begin
				req_addr = axi.awaddr;
				req_id = axi.awid;	
				next_state = REQUEST;
			end
		end
		
		REQUEST: begin

			axi.wready = 1'b1;
			if(axi.wvalid) begin
				req_wdata = axi.wdata;
				is_last_i = axi.wlast;
				next_state = DATA;
			end
		end
		
		DATA: begin
			//prepare data
//			if(req_ready) begin 
				req_valid = 1'b1;
				if(!is_last_i) begin // add response from queue?
					//push the queue
					next_state = REQUEST;
				end
				else begin
					next_state = RESPONSE;
				end	
//			end
		end
		
		
		RESPONSE: begin
			if(rsp_valid) begin
				axi.bvalid = rsp_valid;
			end

			if(rsp_valid) begin
				if(axi.bready) begin
					axi.bresp = 2'b0;   //'okay'
					axi.bid = rsp_id;
					next_state = IDLE;
				end
			end
			
		end
	endcase
end
	
endmodule