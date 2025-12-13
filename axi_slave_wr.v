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
	output 	logic [ID_WIDTH-1:0] 	req_id,
	output 	logic 					rsp_ready

	
);

logic is_last_i;

parameter FSM_WIDTH = 3;
parameter IDLE = 3'b000;
parameter REQUEST = 3'b001;
parameter DATA = 3'b010;
parameter RECEIVE = 3'b011;
parameter RESPONSE = 3'b100;

logic 	[FSM_WIDTH-1:0] prev_state;
logic	[FSM_WIDTH-1:0] next_state;

wire idle2request =  axi.awvalid & axi.awready;
wire request2data = axi.wready & axi.wvalid; 
wire data2request = req_ready & ~axi.wlast;
wire data2receive = req_ready & axi.wlast;
wire receive2response = rsp_ready & rsp_valid;
wire reponse2idle = axi.bready & axi.bvalid;

always @(prev_state or idle2request or request2data or data2request or data2receive or receive2response or reponse2idle)
	case (prev_state)
		IDLE: next_state = idle2request ? REQUEST : IDLE;
		REQUEST: next_state = request2data ? DATA : REQUEST;
		DATA: next_state = 	data2request ? REQUEST : 
							data2receive ? RECEIVE : DATA;
		RECEIVE: next_state = receive2response ? RESPONSE : RECEIVE;
		RESPONSE: next_state = 	reponse2idle ? IDLE : RESPONSE;
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
		
		axi.awready <= 1'b0;
		axi.bvalid <= 1'b0;
		axi.wready <= 1'b0;
		axi.bresp <= '0;
		axi.bid <= '0;
				
		req_valid <= 1'b0;
		req_addr <= '0;
		req_wdata <= '0;
		req_write <= 1'b1;
		req_id <= '0;
		
	end else begin
		axi.awready <= 1'b0;
		axi.bvalid <= 1'b0;	
		
		req_valid <= 1'b0;
		rsp_ready <= 1'b0;
		
		case(prev_state)
			IDLE: begin
				axi.awready <= 1'b1;
				if (idle2request) begin
					req_addr <= axi.awaddr;
					req_id <= axi.awid;	
				end
			end
			REQUEST: begin
				req_valid <= 1'b1;
				axi.wready <= 1'b1;
				if (request2data) begin
					req_wdata <= axi.wdata;
				end
			end
			DATA: begin
				req_valid = 1'b1;
				if (req_ready) begin
				//push queue
				end
			end
			RECEIVE: begin
				rsp_ready <= 1'b1;
				if (receive2response) begin
					axi.bvalid = 1'b1;
					axi.bresp = 2'b0;   //'okay'
					axi.bid = rsp_id;
				end
			end
			RESPONSE: begin
				axi.bvalid = 1'b1;
			end
		endcase
	end
end
endmodule