/*------------------------------------------------------------------------------
 * File          : apb_req_builder.sv
 * Project       : RTL
 * Author        : epsdso
 * Description   : Packs raw disassembler outputs into apb_struct records.
 *
 * RESPONSIBILITY
 * ==============
 * The disassembler produces raw fields (addr, wdata, pstrb, valid) for each
 * APB half. This module packs those fields into apb_struct records ready to
 * push into the APB request FIFO.
 *
 * This module is purely combinational and owns no state.
 *
 * PIPELINE POSITION
 * =================
 *
 *   AXI beat
 *      |
 *      v
 *   disassembler        <- splits beat into raw LSB/MSB fields
 *      |
 *      v
 *   apb_req_builder     <- packs fields into apb_struct (this module)
 *      |
 *      v
 *   manager             <- sequences pushes into the APB request FIFO
 *
 * RESPONSE FIELDS
 * ===============
 * pready, prdata, and pslverr are response fields filled in by the APB slave.
 * On the request path they are initialised to 0 here; apb_master overwrites
 * them when the slave responds.
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module apb_req_builder
import struct_types::*;
(
	input  logic                    is_wr,      // passed through to pwrite

	// Raw fields from disassembler
	input  logic [ADDR_WIDTH-1:0]   lsb_addr,
	input  logic [PDATA_WIDTH-1:0]  lsb_wdata,
	input  logic [PSTRB_WIDTH-1:0]  lsb_pstrb,

	input  logic [ADDR_WIDTH-1:0]   msb_addr,
	input  logic [PDATA_WIDTH-1:0]  msb_wdata,
	input  logic [PSTRB_WIDTH-1:0]  msb_pstrb,

	// Packed apb_struct outputs, ready to push into the APB request FIFO
	output struct_types::apb_struct apb_lsb,
	output struct_types::apb_struct apb_msb
);

	always_comb begin
		// ---- LSB packet ----
		apb_lsb.paddr   = lsb_addr;
		apb_lsb.pwrite  = is_wr;
		apb_lsb.pwdata  = lsb_wdata;
		apb_lsb.pstrb   = lsb_pstrb;
		// Response fields: zeroed here; filled in by the APB slave via apb_master.
		apb_lsb.pready  = 1'b0;
		apb_lsb.prdata  = '0;
		apb_lsb.pslverr = 1'b0;

		// ---- MSB packet ----
		apb_msb.paddr   = msb_addr;
		apb_msb.pwrite  = is_wr;
		apb_msb.pwdata  = msb_wdata;
		apb_msb.pstrb   = msb_pstrb;
		apb_msb.pready  = 1'b0;
		apb_msb.prdata  = '0;
		apb_msb.pslverr = 1'b0;
	end

endmodule