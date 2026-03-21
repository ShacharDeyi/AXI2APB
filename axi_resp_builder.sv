/*------------------------------------------------------------------------------
 * File          : axi_resp_builder.sv
 * Project       : RTL
 * Author        : epsdso
 * Description   : Packs raw assembler outputs into AXI response structs.
 *
 * RESPONSIBILITY
 * ==============
 * The assembler answers "what are the response fields?", producing raw
 * signals (rdata, rresp, bresp).
 * This module answers "what does each AXI response packet look like?",
 * adding manager metadata (transaction_id, is_last) and packing everything
 * into axi_rd_data / axi_wr_resp structs ready to push into the FIFOs.
 *
 * This module is purely combinational and owns no state.
 *
 * PIPELINE POSITION (return path)
 * ================================
 *
 *   APB slave responses
 *          |
 *          v
 *      assembler           <- merges prdata + pslverr into raw fields
 *          |
 *          v
 *      axi_resp_builder    <- adds metadata, packs structs (this module)
 *          |
 *          v
 *      manager             <- pushes into axi_rd_data / axi_wr_resp FIFOs
 *
 * METADATA
 * ========
 * transaction_id : the AXI ID of the original request (arid or awid),
 *                  preserved by the manager across the APB round-trip.
 * is_last        : rlast flag for the read channel, asserted on the final
 *                  beat of a burst. Comes from the manager's beat counter.
 *                  Not needed for write responses (no wlast equivalent on B channel).
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module axi_resp_builder
import struct_types::*;
(
	// Manager metadata
	input  logic [ID_WIDTH-1:0]         transaction_id, // arid or awid of this txn
	input  logic                        is_last,        // last beat of burst (reads only)

	// Raw fields from assembler
	input  logic [DATA_WIDTH-1:0]       rdata,          // assembled 64-bit read data
	input  logic [RRESP_WIDTH-1:0]      rresp,          // AXI read response code
	input  logic [BRESP_WIDTH-1:0]      bresp,          // AXI write response code

	// Packed AXI response structs, ready to push into response FIFOs
	output struct_types::axi_rd_data    rd_data,        // to axi_rd_data FIFO
	output struct_types::axi_wr_resp    wr_resp         // to axi_wr_resp FIFO
);

	always_comb begin
		// ---- Read response ----
		rd_data.rid   = transaction_id;
		rd_data.rdata = rdata;
		rd_data.rresp = rresp;
		rd_data.rlast = is_last;

		// ---- Write response ----
		// No rdata or rlast on the write response channel -- just ID and response code.
		wr_resp.bid   = transaction_id;
		wr_resp.bresp = bresp;
	end

endmodule