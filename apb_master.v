/*------------------------------------------------------------------------------
 * File          : apb_master.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 * Description   :
 *------------------------------------------------------------------------------*/


module apb_master #(
	parameter PADDR_WIDTH = 32,
	parameter PDATA_WIDTH = 32 // should be 32
	
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


parameter FSM_WIDTH = 2;
parameter IDLE = 2'b00;
parameter REQUEST = 2'b01;
parameter RESPONSE = 2'b10;
//parameter DATA = 2'b11;
//error state?


logic 	[FSM_WIDTH-1:0] prev_state;
logic	[FSM_WIDTH-1:0] next_state;

wire idle2request =  	req_ready 	& req_valid;
wire request2response = apb.psel 	& ~apb.penable; 
//wire response2data = 	apb.pready 	& apb.prdata;
wire response2idle = 	apb.pready; 	//& ~apb.prdata;

always @(prev_state or idle2request or request2response)
	case (prev_state)
		IDLE: 		next_state = 	idle2request 		? 	REQUEST 	: IDLE;
		REQUEST: 	next_state = 	request2response 	? 	RESPONSE 	: REQUEST;
		RESPONSE: 	next_state =	response2idle 		? 	IDLE 		: RESPONSE;
		default: 	next_state = 	{FSM_WIDTH{1'bx}};
	endcase

always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n)
		prev_state <= IDLE;
	else
		prev_state <= next_state;
end

always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		apb.psel    <= 1'b0;
		apb.penable <= 1'b0;
		apb.paddr   <= '0;
		apb.pwdata  <= '0;
		apb.pstrb   <= '0;
		apb.pprot   <= 3'b000;
		apb.puser   <= '0;

		rsp_valid   <= 1'b0;
		rsp_error   <= 1'b0;
		rsp_rdata   <= '0;
		req_ready   <= 1'b0;
	end else begin
		apb.psel    <= 1'b0;
		rsp_valid   <= 1'b0;
		rsp_error   <= 1'b0;
		rsp_rdata   <= '0;
		req_ready   <= 1'b0;
		
		case (prev_state)
			IDLE: begin
				req_ready <= 1'b1; // can accept a new transaction
				if(req_valid) begin
					apb.psel   <= 1'b1;
					apb.paddr  <= req_addr;
					apb.pwrite <= req_write;
					apb.pwdata <= req_wdata;
					apb.pstrb  <= {PDATA_WIDTH/8{1'b1}};
					//pop from queue
				end
			end
			REQUEST: begin
				apb.psel   <= 1'b1;
				apb.penable <= 1'b1;
			end
			RESPONSE: begin
				apb.psel    <= 1'b1;
				apb.penable <= 1'b1;
				rsp_valid 	<= 1'b1;
				rsp_error 	<= apb.pslverr;
				rsp_rdata 	<= apb.prdata;
			end
		endcase
	end	
end

endmodule
	