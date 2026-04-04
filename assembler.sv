/*------------------------------------------------------------------------------
 * File          : assembler.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 24, 2026
 * Description   : Recombines two 32-bit APB responses into one 64-bit AXI beat.
 *
 * RESPONSIBILITY
 * ==============
 * This module answers one question: given the two APB slave responses
 * (LSB and MSB halves), what are the assembled AXI response fields?
 *
 * It operates on RAW FIELDS only -- no struct packing.
 * axi_resp_builder is responsible for assembling those fields into
 * axi_rd_data / axi_wr_resp structs.
 * The manager instantiates both modules and controls push timing.
 *
 * PIPELINE POSITION (return path, mirror of forward path)
 * =======================================================
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
 * METADATA SEPARATION
 * ===================
 * transaction_id (rid/bid) and is_last (rlast) are manager-side metadata --
 * they are NOT derived from APB response fields. They belong in
 * axi_resp_builder where the struct is packed, not here.
 * This module only touches what actually comes back from the APB slave.
 *
 * NARROW TRANSACTION HANDLING
 * ===========================
 * For narrow beats only one half is requested (valid_lsb XOR valid_msb).
 * The unused half's prdata must be zeroed in rdata so the AXI master
 * never sees stale data in the byte lanes it didn't ask for.
 * valid_lsb / valid_msb are the same flags produced by the disassembler
 * when the request was issued; the manager must preserve and pass them here.
 *
 * ERROR MERGING
 * =============
 * AXI SLVERR (rresp/bresp = 2'b10) is asserted if EITHER half returned
 * pslverr=1. For a single-half narrow transaction, only the active half's
 * pslverr matters; the inactive half's pslverr input should be driven 0
 * by the manager (since no transaction was sent to that half).
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
	// For narrow reads where only one half was requested, zero the unused
	// half so the AXI master never sees stale data in unrequested byte lanes.
	// The AXI master will only sample the bytes matching the beat's address
	// and size, but driving clean zeros is safer and cleaner in simulation.
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
	//   2'b00  OKAY   -- normal successful transfer
	//   2'b10  SLVERR -- slave error
	//
	// Assert SLVERR if either active half returned pslverr=1.
	// Inactive halves: the manager drives their pslverr input as 0
	// (no transaction was issued so there is no error to report).
	//
	// rresp and bresp use identical error logic -- both just report
	// whether anything went wrong on the APB side.
	//
	wire any_err = (valid_lsb & lsb_pslverr) | (valid_msb & msb_pslverr);

	always_comb begin
		resp = any_err ? 2'b10 : 2'b00;
	end

endmodule