/*------------------------------------------------------------------------------
 * File          : disassembler.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Oct 25, 2025
 * Description   : Splits one 64-bit AXI beat into two independent 32-bit halves.
 *
 * RESPONSIBILITY
 * ==============
 * This module answers one question: given a beat's address, size, direction,
 * and write data, what are the two 32-bit sub-transactions that need to happen?
 *
 * It outputs RAW FIELDS only -- no apb_struct packing.
 * apb_req_builder is responsible for assembling those fields into apb_structs.
 * The manager instantiates both modules and controls push/pop timing.
 *
 * THEORY OF OPERATION
 * ===================
 * AXI data width = 64 bits. APB data width = 32 bits.
 * Every AXI beat maps to an 8-byte-aligned block in memory containing
 * exactly two 32-bit APB words:
 *
 *   LSB word  ->  address with bit[2] = 0  (lower 4 bytes, offset +0)
 *   MSB word  ->  address with bit[2] = 1  (upper 4 bytes, offset +4)
 *
 * How many APB transactions are needed per beat depends on beat_size:
 *
 *   size = 3 (8B) : beat spans the full 64-bit word  -> both halves active
 *   size < 3      : beat fits in one 32-bit half
 *                   bit[2] of beat_addr selects which half:
 *                     0  ->  LSB only
 *                     1  ->  MSB only
 *
 * BEAT ADDRESS
 * ============
 * The manager computes beat_addr each beat as:
 *   beat_addr = base_addr + beat_index * (1 << beat_size)
 * This module receives beat_addr directly and does NOT need base_addr or
 * beat_index separately.
 *
 * APB ADDRESS DERIVATION
 * ======================
 *   lsb_addr = { beat_addr[31:3], 3'b000 }  -- 4-byte aligned, lower word
 *   msb_addr = { beat_addr[31:3], 3'b100 }  -- 4-byte aligned, upper word (+4)
 *
 * WRITE STROBE MAPPING  (AXI wstrb[7:0] -> APB pstrb[3:0])
 * ====================
 *   LSB: wstrb[3:0] -> pstrb[3:0]  (direct, same byte lanes)
 *   MSB: wstrb[7:4] -> pstrb[3:0]  (shift DOWN by 4;
 *        APB pstrb always indexes within its own 32-bit word,
 *        so AXI byte-lane 4 becomes APB byte-lane 0 of the MSB transaction)
 *
 * VALID FLAGS
 * ===========
 *   Reads:  valid = half is active
 *   Writes: valid = half is active AND at least one strobe lane is set
 *           (no point issuing a write that touches no bytes)
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
	//
	// size==3: AXI bus is 64-bit (8 bytes) and our APB bus is 32-bit (4 bytes).
	//			is the only case where the beat is wide enough to touch **both** APB halves. 
	//			Every smaller size fits entirely within one 32-bit half.
	// 			
	// size< 3: need to select which half is actived
	//			do it by address alignment (to 8bytes)
	wire full_width = (beat_size == 3'd3);
	wire lsb_active = full_width | (beat_addr[2] == 1'b0);
	wire msb_active = full_width | (beat_addr[2] == 1'b1);

	/*=========================================================================*/
	/*  Address generation                                                     */
	/*=========================================================================*/
	//
	// Strip the low 3 bits and re-attach fixed offsets.
	// This guarantees 4-byte (as needed in apb) alignment regardless of the incoming beat_addr.
	//
	always_comb begin
		lsb_addr = {beat_addr[ADDR_WIDTH-1:3], 3'b000};  // offset +0
		msb_addr = {beat_addr[ADDR_WIDTH-1:3], 3'b100};  // offset +4
	end

	/*=========================================================================*/
	/*  Data and strobe splitting                                              */
	/*=========================================================================*/

	always_comb begin
		// Zero everything for reads since not needed
		
		// ---- LSB half ----
		// wdata[31:0] and wstrb[3:0] are the lower lanes by default.
		lsb_wdata = is_wr ? wdata[PDATA_WIDTH-1:0]          : '0;
		lsb_pstrb = is_wr ? wstrb[PSTRB_WIDTH-1:0]          : '0;

		// ---- MSB half ----
		// wdata[63:32] goes directly into the MSB APB word.
		// wstrb[7:4] SHIFTS DOWN to pstrb[3:0] because APB pstrb indexes
		// bytes within its own 32-bit word (byte 4 of the AXI bus becomes byte 0 from the APB slave's perspective).
		msb_wdata = is_wr ? wdata[DATA_WIDTH-1:PDATA_WIDTH]  : '0;
		msb_pstrb = is_wr ? wstrb[WSTRB_WIDTH-1:PSTRB_WIDTH] : '0;
	end

	/*=========================================================================*/
	/*  Valid flags                                                            */
	/*=========================================================================*/

	always_comb begin
		// A half is valid when:
		//   (a) the beat contains that half (lsb/msb_active), AND
		//   (b) for writes: at least one strobe byte is set in that half
		//       for reads: always proceed if the half is active
		valid_lsb = lsb_active & (is_wr ? |wstrb[PSTRB_WIDTH-1:0]          : 1'b1);
		valid_msb = msb_active & (is_wr ? |wstrb[WSTRB_WIDTH-1:PSTRB_WIDTH] : 1'b1);
	end

endmodule