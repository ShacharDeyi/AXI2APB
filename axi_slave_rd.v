/*------------------------------------------------------------------------------
 * File          : axi_slave_rd.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 * Description   :
 *------------------------------------------------------------------------------*/

module axi_slave_rd #(
	parameter ADDR_WIDTH    = 32,
	parameter DATA_WIDTH    = 64,
	parameter RRESP_WIDTH   = 2,
	parameter ID_WIDTH      = 32
	
)(
	input  logic                clk,
	input  logic                rst_n,
	
	axi_interface.slave			axi,
	
	// Response from controller
	input logic [DATA_WIDTH-1:0] 	rsp_rdata,
	input logic                		rsp_valid,
	input logic                		is_last,
	input logic	[ID_WIDTH-1:0]		rsp_id,
	input logic [RRESP_WIDTH-1:0]	rsp_rresp,

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
	RESPONSE,
	DATA
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
	axi.rvalid = axi.rvalid;
	axi.rdata =  axi.rdata;
	axi.rlast = 1'b0;
	axi.rresp = axi.rresp;
	axi.rid = axi.rid;

	
	req_valid = 1'b0;
	req_addr = req_addr;
	req_read = 1'b0;
	req_id = req_id;
	next_state  = state;

	
	case(state)
		IDLE: begin
			axi.arready = 1'b1; 
			req_addr = '0;
			req_id = '0;
			axi.rvalid = '0;
			axi.rdata = '0;
			axi.rresp  = '0;
			axi.rid = '0;
			
			req_valid = axi.arvalid; //pass forward
			if (axi.arvalid) begin
				req_addr = axi.araddr;
				req_id = axi.arid;
				next_state = REQUEST;
			end
		end
		
		REQUEST: begin
//			if(req_ready) begin 
				req_valid = 1'b1;
				next_state = RESPONSE;	
// 			end
		end
		//wait for queue 
		RESPONSE: begin
			//tell queue I'm ready to receive
			if(rsp_valid) begin
				//pop the queue, check how to actually remove from queue
				axi.rdata = rsp_rdata;
				axi.rid = rsp_id;
				axi.rresp = rsp_rresp;
				next_state = DATA;
			end	
		end		
		
		DATA: begin
			axi.rvalid = 1'b1;
			if (axi.rready) begin
				if(is_last) begin
					axi.rlast = 1'b1;
					next_state = IDLE;
				end
				else begin
					next_state = RESPONSE;
				end
			end
		end
		
	endcase
end
	
endmodule