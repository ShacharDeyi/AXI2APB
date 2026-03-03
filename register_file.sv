/*------------------------------------------------------------------------------
 * File          : register_file.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Mar 3, 2026
 * Description   : Register storage for AXI response reassembly
 *------------------------------------------------------------------------------*/

module register_file 
import struct_types::*;

(
	input  logic      clk,
	input  logic      rst_n,
	
	// Write response storage (from APB)
	input	struct_types::axi_wr_resp	wr_resp_in,
	output	struct_types::axi_wr_resp	wr_resp_out,
	
	// Read response storage (from APB)
	input	struct_types::axi_rd_data	rd_data_in,
	output	struct_types::axi_rd_data	rd_data_out
);

	*------------------------------------------------------------------------------
 * File          : register_file.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Mar 3, 2026
 * Description   :
 *------------------------------------------------------------------------------*/

module register_file 
import struct_types::*;

(
	input  	logic   clk,
	input  	logic   rst_n,
	input	logic 	enable_rd,
	input	logic 	enable_wr,
	input 	logic	done_wr,
	input	logic	done_rd,
	
	input	struct_types::axi_wr_resp rf_axi_wr_resp_in,
	output	struct_types::axi_wr_resp rf_axi_wr_resp_out,
	
	input	struct_types::axi_rd_data rf_axi_rd_data_in,
	output	struct_types::axi_rd_data rf_axi_rd_data_out

);



/*------------------------------WRITE RESPONSE REGISTERS----------------------------------*/

int write_sel;
always_ff @(posedge done_wr or negedge rst_n) begin
	if (!rst_n)
		write_sel <= '0; // Reset to 0
	else
		write_sel <= (write_sel + 1'b1) % N; 
end

DW03_reg_s_pl #(
	.width($bits(axi_wr_resp))
) u_reg_wr_resp1 (
	.clk(clk),
	.reset_N(rst_n),
	.enable(enable_wr & !write_sel),
	.d(rf_axi_wr_resp_in),
	.q(rf_axi_wr_resp_out)
);

DW03_reg_s_pl #(
	.width($bits(axi_wr_resp))
) u_reg_wr_resp2 (
	.clk(clk),
	.reset_N(rst_n),
	.enable(enable_wr & write_sel),
	.d(rf_axi_wr_resp_in),
	.q(rf_axi_wr_resp_out)
);

/*------------------------------READ RESPONSE REGISTERS----------------------------------*/
int read_sel;
always_ff @(posedge done_rd or negedge rst_n) begin
	if (!rst_n)
		read_sel <= '0; // Reset to 0
	else
		read_sel <= (read_sel + 1'b1) % N;
end


DW03_reg_s_pl #(
	.width($bits(axi_rd_data))
) u_reg_rd_data1 (
	.clk(clk),
	.reset_N(rst_n),
	.enable(enable_rd & read_sel),
	.d(rf_axi_rd_data_in),
	.q(rf_axi_rd_data_out)
);

DW03_reg_s_pl #(
	.width($bits(axi_rd_data))
) u_reg_rd_data2 (
	.clk(clk),
	.reset_N(rst_n),
	.enable(enable_rd & !read_sel),
	.d(rf_axi_rd_data_in),
	.q(rf_axi_rd_data_out)
);

endmodule
