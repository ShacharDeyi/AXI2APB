/*------------------------------------------------------------------------------
 * File          : apb_req_builder.sv
 * Project       : RTL
 * Author        : epsdso
 * Description   : Packs raw disassembler outputs into apb_struct format.
 *
 * RESPONSIBILITY
 * ==============
 * The disassembler answers "what are the two sub-transactions?", producing
 * raw fields (addr, wdata, pstrb, valid).
 * This module answers "what does each APB request packet look like?",
 * assembling those fields into apb_struct records ready for the FIFO.
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
 * The manager instantiates both disassembler and apb_req_builder.
 *
 * RESPONSE FIELDS
 * ===============
 * pready, prdata, pslverr are response fields driven by the APB slave.
 * On the request path they are initialized to 0. The APB master will
 * overwrite them when the slave responds.
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
		// Response fields: initialized to 0 on the request path;
		// filled in by the APB slave after the transaction completes.
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