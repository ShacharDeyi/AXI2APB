/*------------------------------------------------------------------------------
 * File          : assembler.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 24, 2026
 * Description   : Merges two 32-bit APB responses into one 64-bit AXI beat.
 *
 * RESPONSIBILITY
 * ==============
 * Given the two APB slave responses (LSB and MSB halves), produces the
 * assembled 64-bit AXI read data and a single AXI response code.
 * Outputs raw fields only — no struct packing.
 * axi_resp_builder packs these fields (plus manager metadata) into
 * axi_rd_data / axi_wr_resp structs.
 *
 * PIPELINE POSITION (return path)
 * =================================
 *
 *   APB slave responses (LSB + MSB)
 *          |
 *          v
 *      assembler         <- merges prdata + pslverr (this module)
 *          |
 *          v
 *      axi_resp_builder  <- packs into axi_rd_data / axi_wr_resp structs
 *          |
 *          v
 *      manager           <- pushes into AXI response FIFOs
 *
 * NARROW TRANSACTION HANDLING
 * ===========================
 * For narrow beats only one half was requested (valid_lsb XOR valid_msb).
 * The unused half's prdata is zeroed so the AXI master never sees stale
 * data in unrequested byte lanes. The manager must pass the same valid_lsb /
 * valid_msb flags that the disassembler produced when the request was issued.
 *
 * ERROR MERGING
 * =============
 * AXI SLVERR (rresp/bresp = 2'b10) is asserted if either active half returned
 * pslverr=1. For single-half narrow transactions only the active half's
 * pslverr matters; the manager must drive the inactive half's pslverr input
 * to 0 since no APB transaction was issued for it.
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module assembler
import struct_types::*;
(
	// Which halves were actually requested (from disassembler, preserved by manager)
	input  logic                    valid_lsb,      // LSB APB transaction was issued
	input  logic                    valid_msb,      // MSB APB transaction was issued

	// LSB APB slave response fields
	input  logic [PDATA_WIDTH-1:0]  lsb_prdata,     // read data from lower half
	input  logic                    lsb_pslverr,    // error flag from lower half

	// MSB APB slave response fields
	input  logic [PDATA_WIDTH-1:0]  msb_prdata,     // read data from upper half
	input  logic                    msb_pslverr,    // error flag from upper half

	// Assembled outputs -- raw fields, no struct packing yet
	output logic [DATA_WIDTH-1:0]   rdata,          // 64-bit read data (reads only)
	output logic [RESP_WIDTH-1:0]  resp           // AXI  response code
);

	/*=========================================================================*/
	/*  Read data assembly                                                     */
	/*=========================================================================*/
	//
	// Concatenate the two 32-bit halves into one 64-bit word:
	//   rdata[63:32] = msb_prdata   (upper half, address+4)
	//   rdata[31:0]  = lsb_prdata   (lower half, original address)
	//
	// For narrow reads where only one half was requested, zero the unused half
	// to prevent stale data from appearing in the AXI master's byte lanes.
	//
	always_comb begin
		rdata[PDATA_WIDTH-1:0]              = valid_lsb ? lsb_prdata : '0;
		rdata[DATA_WIDTH-1:PDATA_WIDTH]     = valid_msb ? msb_prdata : '0;
	end

	/*=========================================================================*/
	/*  Error merging                                                          */
	/*=========================================================================*/
	//
	// AXI response codes:
	//   2'b00  OKAY   — normal successful transfer
	//   2'b10  SLVERR — slave error
	//
	// Assert SLVERR if either active half returned pslverr=1.
	// The manager drives the inactive half's pslverr input to 0
	// (no transaction was issued, so there is no error to report).
	//
	wire any_err = (valid_lsb & lsb_pslverr) | (valid_msb & msb_pslverr);

	always_comb begin
		resp = any_err ? 2'b10 : 2'b00;
	end

endmodule