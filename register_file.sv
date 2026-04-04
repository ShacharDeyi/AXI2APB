/*------------------------------------------------------------------------------
 * File          : register_file.sv
 * Project       : RTL
 * Description   : 4-slot transaction store for the AXI-to-APB bridge.
 *
 * OVERVIEW
 * ========
 * Provides 2 write slots and 2 read slots. Each slot holds one AXI transaction
 * from the moment its request header is stored (allocation) until the assembled
 * AXI response has been delivered (done).
 *
 * SLOT LIFECYCLE
 * ==============
 *   1. ALLOCATION   Manager asserts alloc_wr / alloc_rd with the target slot
 *                   index and the request header. The header is stored in a
 *                   DW03_reg_s_pl instance so the manager can read it back
 *                   every cycle to compute beat addresses.
 *                   Slot state: FREE -> ACTIVE.
 *
 *   2. RESPONSE     As APB responses arrive, the manager writes the assembled
 *    ACCUMULATION   axi_wr_resp into the slot via resp_wr_en / resp_slot_wr.
 *                   (Read responses bypass this module entirely and are pushed
 *                   directly to rd_data_fifo by the manager.)
 *
 *   3. COMPLETION   Manager asserts resp_valid_set_wr to mark the slot
 *                   response-ready after the burst's final beat completes.
 *                   Slot state: ACTIVE -> READY.
 *
 *   4. DRAIN        Manager pushes the response to wr_resp_fifo when
 *                   resp_ready_wr is high, then asserts done_wr. The slot
 *                   returns to FREE and the read pointer advances.
 *
 * POINTER SCHEME
 * ==============
 * Each direction (write / read) has an independent allocation pointer (in the
 * manager) and a drain pointer (here), both toggling between 0 and 1. This is
 * the standard 2-entry in-order FIFO scheme: slots are always allocated and
 * drained in order.
 *
 * STORAGE
 * =======
 * Request headers and write response data are stored in DW03_reg_s_pl
 * instances (synchronous FF with active-low reset and clock enable).
 * Per-slot control bits (valid, resp_ready) use plain always_ff registers.
 *
 * PROTOCOL CONTRACT (manager must obey)
 * ======================================
 *   - alloc_wr/rd must not be asserted when the target slot is not free
 *     (slot_free_wr/rd[slot] == 0).
 *   - resp_valid_set_wr must only be asserted for an already-active slot.
 *   - done_wr must only be asserted when resp_ready_wr is high.
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module register_file
import struct_types::*;
(
	input  logic   clk,
	input  logic   rst_n,

	/*--------------------------------------------------------------------*/
	/* Write direction — 2 slots                                          */
	/*--------------------------------------------------------------------*/

	// Allocation: manager pops wr_req_fifo and stores the header immediately.
	input  logic                        alloc_wr,           // 1 = allocate this cycle
	input  logic                        alloc_slot_wr,      // target slot (0 or 1)
	input  struct_types::axi_wr_req     alloc_req_wr,       // request header to store

	// Header readback: manager reads these every cycle for beat-address computation.
	output struct_types::axi_wr_req     req_out_wr  [0:1],

	// Response write: manager writes the assembled bresp as beats complete.
	input  logic                        resp_wr_en,         // 1 = write this cycle
	input  logic                        resp_slot_wr,       // target slot
	input  struct_types::axi_wr_resp    resp_in_wr,         // assembled response

	// Completion: manager signals the slot's response is final (last beat done).
	input  logic                        resp_valid_set_wr,  // 1 = mark slot ready
	input  logic                        resp_valid_slot_wr, // target slot

	// Drain: AXI slave has consumed the response; advance the read pointer.
	input  logic                        done_wr,
	input  logic                        done_rd,

	// Outputs to AXI slave (always driven from the read-pointer slot).
	output struct_types::axi_wr_resp    resp_out_wr,
	output logic                        resp_ready_wr,      // read-pointer slot has a final response

	// Free flags: manager checks these before allocating.
	output logic                        slot_free_wr [0:1],

	/*--------------------------------------------------------------------*/
	/* Read direction — 2 slots (symmetric)                               */
	/* Read responses bypass this module entirely and are pushed directly  */
	/* to rd_data_fifo by the manager. Only the request header and         */
	/* slot-free tracking are used here for reads.                         */
	/*--------------------------------------------------------------------*/

	input  logic                        alloc_rd,
	input  logic                        alloc_slot_rd,
	input  struct_types::axi_rd_req     alloc_req_rd,

	output struct_types::axi_rd_req     req_out_rd  [0:1],

	output logic                        slot_free_rd [0:1]
);

/*=========================================================================*/
/*  Write direction                                                         */
/*=========================================================================*/

	// Drain-side read pointer; the allocation pointer lives in the manager.
	logic wr_read_ptr;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) wr_read_ptr <= 1'b0;
		else if (done_wr) wr_read_ptr <= ~wr_read_ptr;
	end

	// Per-slot control bits.
	logic wr_slot_valid    [0:1]; // slot is allocated (active or ready)
	logic wr_resp_ready_q  [0:1]; // slot has a final response waiting to drain

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wr_slot_valid[0]   <= 1'b0;
			wr_resp_ready_q[0] <= 1'b0;
			wr_slot_valid[1]   <= 1'b0;
			wr_resp_ready_q[1] <= 1'b0;
		end else begin
			for (int s = 0; s < 2; s++) begin
				if (alloc_wr && alloc_slot_wr == logic'(s))
					wr_slot_valid[s] <= 1'b1;
				else if (done_wr && wr_read_ptr == logic'(s))
					wr_slot_valid[s] <= 1'b0;

				if (resp_valid_set_wr && resp_valid_slot_wr == logic'(s))
					wr_resp_ready_q[s] <= 1'b1;
				else if (done_wr && wr_read_ptr == logic'(s))
					wr_resp_ready_q[s] <= 1'b0;
			end
		end
	end

	// Request header storage (DW03_reg_s_pl per slot).
	struct_types::axi_wr_req wr_req_q [0:1];

	DW03_reg_s_pl #(
		.width       ($bits(axi_wr_req)),
		.reset_value (0)
	) u_wr_req0 (
		.clk     (clk),
		.reset_N (rst_n),
		.enable  (alloc_wr & ~alloc_slot_wr),
		.d       (alloc_req_wr),
		.q       (wr_req_q[0])
	);

	DW03_reg_s_pl #(
		.width       ($bits(axi_wr_req)),
		.reset_value (0)
	) u_wr_req1 (
		.clk     (clk),
		.reset_N (rst_n),
		.enable  (alloc_wr & alloc_slot_wr),
		.d       (alloc_req_wr),
		.q       (wr_req_q[1])
	);

	// Response storage (DW03_reg_s_pl per slot).
	struct_types::axi_wr_resp wr_resp_q [0:1];

	DW03_reg_s_pl #(
		.width       ($bits(axi_wr_resp)),
		.reset_value (0)
	) u_wr_resp0 (
		.clk     (clk),
		.reset_N (rst_n),
		.enable  (resp_wr_en & ~resp_slot_wr),
		.d       (resp_in_wr),
		.q       (wr_resp_q[0])
	);

	DW03_reg_s_pl #(
		.width       ($bits(axi_wr_resp)),
		.reset_value (0)
	) u_wr_resp1 (
		.clk     (clk),
		.reset_N (rst_n),
		.enable  (resp_wr_en & resp_slot_wr),
		.d       (resp_in_wr),
		.q       (wr_resp_q[1])
	);

	// Outputs  always driven from the read-pointer slot.
	assign req_out_wr    = wr_req_q;                       // both slots visible
	assign resp_out_wr   = wr_resp_q[wr_read_ptr];
	assign resp_ready_wr = wr_resp_ready_q[wr_read_ptr];

	generate
		for (genvar s = 0; s < 2; s++)
			assign slot_free_wr[s] = !wr_slot_valid[s];
	endgenerate

/*=========================================================================*/
/*  Read direction                                                          */
/*  Read responses bypass this module — the manager pushes each beat       */
/*  directly to rd_data_fifo. Only the request header and slot-free        */
/*  state are tracked here.                                                 */
/*=========================================================================*/

	logic rd_read_ptr;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) rd_read_ptr <= 1'b0;
		else if (done_rd) rd_read_ptr <= ~rd_read_ptr;
	end

	logic rd_slot_valid [0:1];

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rd_slot_valid[0] <= 1'b0;
			rd_slot_valid[1] <= 1'b0;
		end else begin
			for (int s = 0; s < 2; s++) begin
				if (alloc_rd && alloc_slot_rd == logic'(s))
					rd_slot_valid[s] <= 1'b1;
				else if (done_rd && rd_read_ptr == logic'(s))
					rd_slot_valid[s] <= 1'b0;
			end
		end
	end

	struct_types::axi_rd_req rd_req_q [0:1];

	DW03_reg_s_pl #(
		.width       ($bits(axi_rd_req)),
		.reset_value (0)
	) u_rd_req0 (
		.clk     (clk),
		.reset_N (rst_n),
		.enable  (alloc_rd & ~alloc_slot_rd),
		.d       (alloc_req_rd),
		.q       (rd_req_q[0])
	);

	DW03_reg_s_pl #(
		.width       ($bits(axi_rd_req)),
		.reset_value (0)
	) u_rd_req1 (
		.clk     (clk),
		.reset_N (rst_n),
		.enable  (alloc_rd & alloc_slot_rd),
		.d       (alloc_req_rd),
		.q       (rd_req_q[1])
	);

	assign req_out_rd = rd_req_q;

	generate
		for (genvar s = 0; s < 2; s++)
			assign slot_free_rd[s] = !rd_slot_valid[s];
	endgenerate

endmodule