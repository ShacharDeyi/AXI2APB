/*------------------------------------------------------------------------------
 * File          : top_module.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Jul 5, 2025
 * Description   :
 *------------------------------------------------------------------------------*/


module top_module #() (
	input logic clk,
	input logic reset,
	
	// Interface declarations
	axi_interface.slave  axi,
	apb_interface.master apb
	
);

	// Internal signals from AXI slave write to APB master
	logic                	req_valid_wr;
	logic [31:0]         	req_addr_wr;
	logic [63:0]         	req_wdata_wr;
	logic                	req_write_wr;
	logic                	req_ready;
	logic [31:0]         	rsp_rdata;
	logic                	rsp_valid;
	logic                	rsp_error;
	
	// Internal signals from AXI slave read to APB master
	logic                	req_valid_rd;
	logic [31:0]         	req_addr_rd;
	logic                	req_write_rd;

	// AXI Slave Write Module (converts AXI write requests to APB)
	axi_slave_wr u_axi_slave_wr (
		.clk(clk),
		.rst_n(~reset),
		.axi(axi),
		
		// Outputs to APB master
		.req_valid(req_valid_wr),
		.req_addr(req_addr_wr),
		.req_wdata(req_wdata_wr),
		.req_write(req_write_wr),
		.req_ready(req_ready),
		
		// Response from APB master
		.rsp_valid(rsp_valid)
	);

	// AXI Slave Read Module (converts AXI read requests to APB)
	axi_slave_rd u_axi_slave_rd (
		.clk(clk),
		.rst_n(~reset),
		.axi(axi),
		
		// Outputs to APB master
		.req_valid(req_valid_rd),
		.req_addr(req_addr_rd),
		.req_read(req_write_rd),
		.req_ready(req_ready),
		
		// Response from APB master
		.rsp_rdata(rsp_rdata),
		.rsp_valid(rsp_valid)
	);

	// APB Master Module (arbitrates between read and write requests)
	apb_master u_apb_master (
		.clk(clk),
		.rst_n(~reset),
		.apb(apb),
		
		// Inputs from AXI slaves (write has priority)
		.req_valid(req_valid_wr | req_valid_rd),
		.req_addr(req_valid_wr ? req_addr_wr : req_addr_rd),
		.req_wdata(req_wdata_wr),
		.req_write(req_valid_wr ? req_write_wr : req_write_rd),
		.req_ready(req_ready),
		
		// Outputs to AXI slaves
		.rsp_rdata(rsp_rdata),
		.rsp_valid(rsp_valid),
		.rsp_error(rsp_error)
	);

endmodule : top_module