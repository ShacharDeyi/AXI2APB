/*------------------------------------------------------------------------------
 * File          : register_file.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Mar 3, 2026
 * Description   : Register storage for AXI response reassembly.
 *------------------------------------------------------------------------------*/

module register_file 
import struct_types::*;

(
	input  logic   clk,
	input  logic   rst_n,
	input  logic   enable_rd,
	input  logic   enable_wr,
	input  logic   done_wr,
	input  logic   done_rd,

	input  struct_types::axi_wr_resp rf_axi_wr_resp_in,
	output struct_types::axi_wr_resp rf_axi_wr_resp_out,

	input  struct_types::axi_rd_data rf_axi_rd_data_in,
	output struct_types::axi_rd_data rf_axi_rd_data_out
);

/*------------------------------WRITE RESPONSE REGISTERS----------------------------------*/

	logic write_sel;

	always_ff @(posedge done_wr or negedge rst_n) begin
		if (!rst_n)
			write_sel <= 1'b0;
		else
			write_sel <= ~write_sel;   // toggle between 0 and 1
	end

	struct_types::axi_wr_resp wr_resp_q0, wr_resp_q1;

	DW03_reg_s_pl #(
		.width($bits(axi_wr_resp))
	) u_reg_wr_resp0 (
		.clk     (clk),
		.reset_N (rst_n),
		.enable  (enable_wr & ~write_sel),
		.d       (rf_axi_wr_resp_in),
		.q       (wr_resp_q0)
	);

	DW03_reg_s_pl #(
		.width($bits(axi_wr_resp))
	) u_reg_wr_resp1 (
		.clk     (clk),
		.reset_N (rst_n),
		.enable  (enable_wr & write_sel),
		.d       (rf_axi_wr_resp_in),
		.q       (wr_resp_q1)
	);

	// Mux output: select the register that was most recently written
	assign rf_axi_wr_resp_out = write_sel ? wr_resp_q1 : wr_resp_q0;

/*------------------------------READ RESPONSE REGISTERS----------------------------------*/
`timescale 1ns/1ps

	logic read_sel;

	always_ff @(posedge done_rd or negedge rst_n) begin
		if (!rst_n)
			read_sel <= 1'b0;
		else
			read_sel <= ~read_sel;
	end

	struct_types::axi_rd_data rd_data_q0, rd_data_q1;

	DW03_reg_s_pl #(
		.width($bits(axi_rd_data))
	) u_reg_rd_data0 (
		.clk     (clk),
		.reset_N (rst_n),
		.enable  (enable_rd & ~read_sel),
		.d       (rf_axi_rd_data_in),
		.q       (rd_data_q0)
	);

	DW03_reg_s_pl #(
		.width($bits(axi_rd_data))
	) u_reg_rd_data1 (
		.clk     (clk),
		.reset_N (rst_n),
		.enable  (enable_rd & read_sel),
		.d       (rf_axi_rd_data_in),
		.q       (rd_data_q1)
	);

	assign rf_axi_rd_data_out = read_sel ? rd_data_q1 : rd_data_q0;

endmodule