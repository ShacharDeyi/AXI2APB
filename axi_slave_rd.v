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
	output  logic                	rsp_ready,   // ready to receive the response from fifo
	output  logic                	req_valid,   // new request available
	output  logic [ADDR_WIDTH-1:0] 	req_addr,
	output  logic                	req_read,   // 1=write, 0=read
	output 	logic [ID_WIDTH-1:0] 	req_id
	
);

parameter FSM_WIDTH = 2;
parameter IDLE = 2'b00;
parameter REQUEST = 2'b01;
parameter RESPONSE = 2'b10;
parameter DATA = 2'b11;

logic 	[FSM_WIDTH-1:0] prev_state;
logic	[FSM_WIDTH-1:0] next_state;

wire idle2request =  axi.arvalid & axi.arready;
wire request2response = req_ready; 
wire response2data = rsp_valid;
wire data2reponse = axi.rready & ~is_last;
wire data2idle = axi.rready & is_last;

always @(prev_state or idle2request or request2response or response2data or data2reponse or data2idle)
	case (prev_state)
		IDLE: next_state = idle2request ? REQUEST : IDLE;
		REQUEST: next_state = request2response ? RESPONSE : REQUEST;
		RESPONSE: next_state = response2data ? DATA : RESPONSE;
		DATA: next_state = 	data2reponse ? RESPONSE :
							data2idle ? IDLE : DATA;
		default: next_state = {FSM_WIDTH{1'bx}};
	endcase

always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n)
		prev_state <= IDLE;
	else
		prev_state <= next_state;
end


always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		
		axi.arready <= 1'b0;
		axi.rvalid <= 1'b0;
		axi.rdata <= '0;
		axi.rresp <= '0;
		axi.rid <= '0;
		axi.rlast <= 1'b0;
		
		req_valid <= 1'b0;
		req_addr <= '0;
		req_id <= '0;
		req_read <= 1'b0;
		rsp_ready <= 1'b0;
		
	end else begin
		axi.arready <= 1'b0;
		axi.rvalid <= 1'b0;
		axi.rlast <= 1'b0;
		
		req_valid <= 1'b0;
		rsp_ready <= 1'b0;
		
		case (prev_state)
			IDLE: begin
				axi.arready <= 1'b1;	
				axi.rdata <= '0;
				if(idle2request) begin
					req_valid <= 1'b1;
					req_id <= axi.arid;
					req_addr <= axi.araddr;
				end
			end
			REQUEST: begin
				req_valid <= 1'b1;
				//add fifo logic - to send package
			end
			RESPONSE: begin
				//add fifo logic - to receive package
				rsp_ready <= 1'b1;
				if(response2data) begin
					axi.rdata <= rsp_rdata;
					axi.rid <= rsp_id;
					axi.rresp <= rsp_rresp;
				end
			end
			DATA: begin
				axi.rvalid <= 1'b1;
				if (data2idle)
					axi.rlast <= 1'b1;
			end
		endcase
	end
end
endmodule