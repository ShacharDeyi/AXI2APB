/*------------------------------------------------------------------------------
 * File          : disassembler.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Oct 25, 2025
 * Description   : Splits one 64-bit AXI beat into two independent 32-bit APB
 *                 sub-transactions.
 *
 * RESPONSIBILITY
 * ==============
 * Given a beat's address, size, direction, and write data, determines which
 * of the two 32-bit APB halves (LSB and/or MSB) need to be issued and computes
 * their individual address, data, and strobe fields.
 * Outputs raw fields only — no apb_struct packing.
 * apb_req_builder packs these fields into apb_structs for the FIFO.
 *
 * HALF SELECTION
 * ==============
 * size=3 (8-byte beat): the beat spans the full 64-bit word → both halves active.
 * size<3 (narrow beat): the beat fits in one 32-bit half, selected by beat_addr[2]:
 *   beat_addr[2]=0 → LSB half only (address unchanged)
 *   beat_addr[2]=1 → MSB half only (address already points to the upper word)
 *
 * VALID FLAGS
 * ===========
 * Reads:  a half is valid when it is active.
 * Writes: a half is valid when it is active AND at least one of its strobe
 *         lanes is set (no point issuing a write that touches no bytes).
 *
 * ADDRESS GENERATION
 * ==================
 * The manager computes beat_addr each beat as:
 *   beat_addr = base_addr + beat_index * (1 << beat_size)
 * LSB address = beat_addr (offset +0).
 * MSB address = beat_addr + 4 when both halves are active;
 *               = beat_addr when only MSB is active (narrow beat, addr already
 *               points to the upper word per the AXI narrow-transfer rules).
 *
 * WRITE STROBE MAPPING  (AXI wstrb[7:0] → APB pstrb[3:0])
 * ====================
 *   LSB: wstrb[3:0] → pstrb[3:0]  (direct mapping)
 *   MSB: wstrb[7:4] → pstrb[3:0]  (shifted down by 4 because APB pstrb always
 *        indexes bytes within its own 32-bit word; AXI byte-lane 4 becomes
 *        APB byte-lane 0 from the slave's perspective)
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module disassembler
import struct_types::*;
(
	// Direction
	input  logic                        is_wr,          // 1 = write, 0 = read

	// Beat descriptor (computed by manager before each beat)
	input  logic [ADDR_WIDTH-1:0]       beat_addr,      // base + beat_idx*(1<<size)
	input  logic [MAX_SIZE-1:0]         beat_size,      // arsize or awsize

	// Write data for this beat (ignored when is_wr = 0)
	input  logic [DATA_WIDTH-1:0]       wdata,          // full 64-bit data word
	input  logic [WSTRB_WIDTH-1:0]      wstrb,          // 8-bit byte enables

	// ---- LSB outputs (lower 32-bit half) ----
	output logic [ADDR_WIDTH-1:0]       lsb_addr,       // APB address for lower half
	output logic [PDATA_WIDTH-1:0]      lsb_wdata,      // lower 32 bits of wdata
	output logic [PSTRB_WIDTH-1:0]      lsb_pstrb,      // wstrb[3:0], zeroed for reads
	output logic                        valid_lsb,      // this half needs an APB txn

	// ---- MSB outputs (upper 32-bit half) ----
	output logic [ADDR_WIDTH-1:0]       msb_addr,       // APB address for upper half
	output logic [PDATA_WIDTH-1:0]      msb_wdata,      // upper 32 bits of wdata
	output logic [PSTRB_WIDTH-1:0]      msb_pstrb,      // wstrb[7:4] shifted to [3:0]
	output logic                        valid_msb       // this half needs an APB txn
);

	/*=========================================================================*/
	/*  Half-activation                                                        */
	/*=========================================================================*/
	// size==3: beat is 8 bytes — spans both APB 32-bit halves.
	// size< 3: beat fits in one half, selected by beat_addr[2].
	wire full_width = (beat_size == 3'd3);
	wire lsb_active = full_width | (beat_addr[2] == 1'b0);
	wire msb_active = full_width | (beat_addr[2] == 1'b1);

	/*=========================================================================*/
	/*  Data and strobe splitting                                              */
	/*=========================================================================*/

	always_comb begin
		// For reads, data and strobes are irrelevant — zero them.

		// ---- LSB half ----
		lsb_wdata = is_wr ? wdata[PDATA_WIDTH-1:0]          : '0;
		lsb_pstrb = is_wr ? wstrb[PSTRB_WIDTH-1:0]          : '0;

		// ---- MSB half ----
		// wstrb[7:4] shifts down to pstrb[3:0] because APB pstrb indexes
		// bytes within its own 32-bit word.
		msb_wdata = is_wr ? wdata[DATA_WIDTH-1:PDATA_WIDTH]  : '0;
		msb_pstrb = is_wr ? wstrb[WSTRB_WIDTH-1:PSTRB_WIDTH] : '0;
	end

	/*=========================================================================*/
	/*  Valid flags & address generation                                       */
	/*=========================================================================*/

	always_comb begin
		// A half is valid when:
		//   (a) the beat covers that half (lsb/msb_active), AND
		//   (b) for writes: at least one strobe byte is set in that half;
		//       for reads: always valid if the half is active.
		valid_lsb = lsb_active & (is_wr ? |wstrb[PSTRB_WIDTH-1:0]          : 1'b1);
		valid_msb = msb_active & (is_wr ? |wstrb[WSTRB_WIDTH-1:PSTRB_WIDTH] : 1'b1);
		lsb_addr = beat_addr;  // offset +0
		// MSB address: +4 when both halves are active (full-width beat).
		// When only MSB is active (narrow beat), beat_addr already points to
		// the upper 32-bit word, so no offset is added.
		msb_addr = (valid_msb & !valid_lsb) ? beat_addr : (beat_addr + 32'd4);

	end


endmodule