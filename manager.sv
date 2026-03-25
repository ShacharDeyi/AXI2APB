/*------------------------------------------------------------------------------
 * File          : manager.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 27, 2026
 * Description   : 2-outstanding AXI-to-APB bridge manager.
 *
 * ============================================================================
 * CHANGES vs. PREVIOUS REVISION
 * ============================================================================
 *
 * 5. WAR / RAW COLLISION DETECTION (cache-line granularity)
 *    Previously only Write-after-Write (WaW) pairs on the same cache line
 *    were serialised.  Read-write and write-read pairs were always allowed
 *    to proceed concurrently.
 *
 *    This revision adds two new hazard classes:
 *      RaW (Read-after-Write): an incoming WRITE is held in SLOT_IDLE while
 *        any in-flight slot is a READ whose address range overlaps the same
 *        cache line(s) as the incoming write.
 *      WaR (Write-after-Read): an incoming READ is held in SLOT_IDLE while
 *        any in-flight slot is a WRITE whose address range overlaps the same
 *        cache line(s) as the incoming read.
 *
 *    Implementation:
 *      - inc_rd_cl_start/end: cache-line range for the incoming read (FIFO head).
 *        Mirrors the existing inc_cl_start/end for writes.
 *      - slot_rd_cl_start/end: cache-line range for each slot's LATCHED read
 *        request.  Derived from slot_rd_req[gc].araddr/arlen/arsize.
 *      - slot_raw_collision[gc]: incoming write overlaps in-flight read in slot gc.
 *      - slot_war_collision[gc]: incoming read overlaps in-flight write in slot gc.
 *      - raw_collision / war_collision: OR of per-slot flags.
 *      - Write-accept condition in SLOT_IDLE now also gated by raw_collision.
 *      - Read-accept condition in SLOT_IDLE now also gated by war_collision.
 *      - s0_takes_wr / s0_takes_rd propagate the new gates consistently.
 *      - gen_pop_wants updated to reflect the new gates.
 *      - wr_accepted_this_cycle updated to include raw_collision guard.
 *      - rd_accepted_this_cycle updated to include war_collision guard.
 *    No new sequential state is added; all logic is purely combinational.
 *
 * 1. DUAL DISASSEMBLERS
 *    Previously one shared disassembler was muxed between the two slots.
 *    Now there are two dedicated instances:
 *      u_disassembler_wr  -- always driven with is_wr=1, connected to the
 *                            write-slot's beat_addr / wdata / wstrb.
 *      u_disassembler_rd  -- always driven with is_wr=0, wdata/wstrb tied 0,
 *                            connected to the read-slot's beat_addr.
 *    Similarly, two apb_req_builder instances pack the outputs.
 *    The push-arbitration mux (dis_slot) now selects WHICH builder's output
 *    to forward to the APB req FIFO; it no longer needs to steer inputs.
 *    This removes the combinational mux on the disassembler inputs and makes
 *    the timing path cleaner.
 *
 * 4. WRITE-WRITE COLLISION DETECTION (cache-line granularity)
 *    When a new write burst arrives and an in-flight write burst touches
 *    the same cache line, the new write is held in SLOT_IDLE until the
 *    in-flight burst completes.
 *    Controlled by CACHE_LINE_SIZE parameter (default 64 bytes).
 *    Detection is purely combinational; no new state is added.
 *
 * 3. ANTI-STARVATION: read priority latch + starvation counter
 *    Writes are still favoured by default.  After WR_STARVE_THRESH writes
 *    are accepted while rd_req is non-empty, rd_priority is latched.
 *    While latched, SLOT_IDLE inverts the priority: reads win, writes are
 *    only accepted if no read is pending.  rd_priority clears the cycle
 *    a read is actually accepted, and the counter resets.
 *    Set WR_STARVE_THRESH=0 to disable entirely.
 *
 * 2. BUG FIX: slot_expected_lsb/msb cleared too early in SLOT_DISPATCH
 *    The unconditional
 *        if (!dis_valid_lsb) slot_expected_lsb[gi] <= 1'b0;
 *    fired every DISPATCH cycle, including cycles where dis_slot pointed at
 *    the OTHER slot. dis_valid_lsb/msb then reflected the other slot's
 *    disassembler outputs and could spuriously clear expected flags.
 *    Fixed: these clears are now gated on the slot-specific valid signals
 *    (wr_valid_lsb[gi] / rd_valid_lsb[gi]) derived directly from the
 *    dedicated disassembler for that slot's direction.
 *
 * ============================================================================
 * OVERVIEW (unchanged)
 * ============================================================================
 *
 * The manager coordinates the full data path between the AXI FIFOs
 * (populated by axi_slave_wr/rd) and the APB FIFOs (consumed by apb_master).
 * It supports two simultaneously in-flight transactions via two independent
 * transaction slots (slot 0 and slot 1).
 *
 * Internal sub-modules instantiated here:
 *   - disassembler (x2)  : splits 64-bit AXI beat into raw 32-bit LSB/MSB fields
 *   - apb_req_builder(x2): packs raw fields into apb_struct for the APB req FIFO
 *   - assembler           : merges two 32-bit APB responses into 64-bit AXI fields
 *   - axi_resp_builder    : packs raw fields into axi_rd_data / axi_wr_resp structs
 *   - register_file       : 2-slot ping-pong buffer for assembled AXI responses
 *
 * ============================================================================
 * HOW EACH REQUEST IS TRACKED
 * ============================================================================
 * ... (see previous revision for full narrative)
 *
 * ============================================================================
 * APB REQUEST ARBITRATION  (updated)
 * ============================================================================
 *
 * Both slots may want to push APB requests simultaneously.
 * Only one push is allowed per cycle.
 *
 * Fixed priority: slot 0 wins over slot 1.
 * dis_slot now selects which builder's LSB/MSB outputs to forward.
 *
 * Write slot drives u_disassembler_wr / u_apb_req_builder_wr.
 * Read  slot drives u_disassembler_rd / u_apb_req_builder_rd.
 *
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module manager
import struct_types::*;
#(
	parameter NUM_SLOTS        = 2, // number of in-flight transaction slots
	// Anti-starvation: after this many writes are accepted while a read is
	// waiting, rd_priority is latched and the next idle slot is forced to
	// take a read.  Writes are still favoured in the normal (non-priority)
	// path.  Set to 0 to disable (writes always win).
	parameter WR_STARVE_THRESH = 4, // 0 = disabled
	// Collision detection: two concurrent transactions that touch the same
	// cache line and have a RAW/WAR/WAW dependency are serialised -- the
	// second waits in SLOT_IDLE until the first completes.
	// Must be a power of 2 and >= 8 (one AXI beat).
	parameter CACHE_LINE_SIZE  = 64 // bytes
)
(
	input  logic        clk,
	input  logic        rst_n,

	/*------------------------------------------------------------------*/
	/* AXI read request FIFO                                            */
	/*------------------------------------------------------------------*/
	output logic                        rd_req_pop_n,
	input  logic                        rd_req_empty,
	input  struct_types::axi_rd_req     rd_req_fifo_out,

	/*------------------------------------------------------------------*/
	/* AXI write request FIFO                                           */
	/*------------------------------------------------------------------*/
	output logic                        wr_req_pop_n,
	input  logic                        wr_req_empty,
	input  struct_types::axi_wr_req     wr_req_fifo_out,

	/*------------------------------------------------------------------*/
	/* AXI write data FIFO                                              */
	/*------------------------------------------------------------------*/
	output logic                        wr_data_pop_n,
	input  logic                        wr_data_empty,
	input  struct_types::axi_wr_data    wr_data_fifo_out,

	/*------------------------------------------------------------------*/
	/* APB request FIFO + sideband tag FIFO (always pushed together)    */
	/*------------------------------------------------------------------*/
	output logic                        apb_req_push_n,
	output struct_types::apb_struct     apb_req_fifo_in,
	input  logic                        apb_req_full,

	output logic                        apb_tag_push_n,
	output struct_types::apb_tag_t      apb_tag_fifo_in,
	input  logic                        apb_tag_full,

	/*------------------------------------------------------------------*/
	/* APB response FIFO + sideband tag FIFO (always popped together)   */
	/*------------------------------------------------------------------*/
	output logic                        apb_resp_pop_n,
	input  logic                        apb_resp_empty,
	input  struct_types::apb_struct     apb_resp_fifo_out,

	output logic                        apb_tag_pop_n,
	input  logic                        apb_tag_empty,
	input  struct_types::apb_tag_t      apb_tag_fifo_out,

	/*------------------------------------------------------------------*/
	/* AXI read data FIFO                                               */
	/*------------------------------------------------------------------*/
	output logic                        rd_data_push_n,
	output struct_types::axi_rd_data    rd_data_fifo_in,
	input  logic                        rd_data_full,

	/*------------------------------------------------------------------*/
	/* AXI write response FIFO                                          */
	/*------------------------------------------------------------------*/
	output logic                        wr_resp_push_n,
	output struct_types::axi_wr_resp    wr_resp_fifo_in,
	input  logic                        wr_resp_full
);

	/*=====================================================================*/
	/*  Per-slot state encoding                                            */
	/*=====================================================================*/

	typedef enum logic [1:0] {
		SLOT_IDLE,          // slot is free, waiting for a new transaction
		SLOT_DISPATCH,      // pushing APB requests for the current beat
		SLOT_WAIT_RESP      // waiting for APB responses for the current beat
	} slot_state_t;

	/*=====================================================================*/
	/*  Per-slot state arrays  [indexed 0..NUM_SLOTS-1]                   */
	/*=====================================================================*/

	slot_state_t              slot_state          [0:NUM_SLOTS-1];
	logic                     slot_is_wr          [0:NUM_SLOTS-1];

	// AXI request storage
	struct_types::axi_wr_req  slot_wr_req         [0:NUM_SLOTS-1]; // stable whole burst
	struct_types::axi_wr_data slot_wr_data        [0:NUM_SLOTS-1]; // refreshed every beat
	struct_types::axi_rd_req  slot_rd_req         [0:NUM_SLOTS-1]; // stable whole burst
	logic                     slot_wr_req_latched [0:NUM_SLOTS-1]; // wr_req popped flag
	logic                     slot_rd_req_latched [0:NUM_SLOTS-1]; // rd_req popped flag
	// Set when wr_data is valid for the current beat; cleared on the transition
	// back to SLOT_DISPATCH so the next beat's data can be fetched.
	logic                     slot_wr_data_latched[0:NUM_SLOTS-1];

	// Beat tracking
	logic [7:0]               slot_beat_index     [0:NUM_SLOTS-1];

	// Dispatch tracking
	logic                     slot_lsb_sent       [0:NUM_SLOTS-1];
	logic                     slot_msb_sent       [0:NUM_SLOTS-1];
	logic                     slot_expected_lsb   [0:NUM_SLOTS-1];
	logic                     slot_expected_msb   [0:NUM_SLOTS-1];

	// APB response capture
	logic [PDATA_WIDTH-1:0]   slot_lsb_prdata     [0:NUM_SLOTS-1];
	logic                     slot_lsb_pslverr    [0:NUM_SLOTS-1];
	logic                     slot_lsb_valid      [0:NUM_SLOTS-1];
	logic [PDATA_WIDTH-1:0]   slot_msb_prdata     [0:NUM_SLOTS-1];
	logic                     slot_msb_pslverr    [0:NUM_SLOTS-1];
	logic                     slot_msb_valid      [0:NUM_SLOTS-1];

	// Write burst error accumulation
	logic                     slot_wr_burst_err   [0:NUM_SLOTS-1];

	/*=====================================================================*/
	/*  Anti-starvation state                                              */
	/*=====================================================================*/
	// wr_starve_cnt: counts write transactions accepted while rd_req is
	//   non-empty.  Saturates at WR_STARVE_THRESH.
	// rd_priority: latched flag -- forces the next idle slot to take a
	//   read.  Cleared the cycle a read is actually accepted.
	// Both are no-ops when WR_STARVE_THRESH==0.

	localparam STARVE_W = (WR_STARVE_THRESH > 0)
						  ? $clog2(WR_STARVE_THRESH + 1)
						  : 1;   // at least 1 bit to keep the port legal

	logic [STARVE_W-1:0]      wr_starve_cnt;
	logic                     rd_priority;

	/*=====================================================================*/
	/*  Collision detection state (WaW, RaW, WaR)                        */
	/*=====================================================================*/
	// CL_BITS: number of low address bits that identify position within
	//   a cache line.  Two addresses share a cache line iff they agree on
	//   all bits above CL_BITS.
	//
	// Incoming WRITE range (FIFO head):
	//   inc_cl_start/end  -- cache-line index range of the pending write
	//
	// Incoming READ range (FIFO head):
	//   inc_rd_cl_start/end -- cache-line index range of the pending read
	//
	// Per-slot WRITE range:
	//   slot_cl_start/end -- derived from slot_wr_req (awaddr/awlen/awsize)
	//
	// Per-slot READ range:
	//   slot_rd_cl_start/end -- derived from slot_rd_req (araddr/arlen/arsize)
	//
	// Collision flags (one per in-flight slot):
	//   slot_wr_collision[gc] : WaW -- incoming write vs in-flight write
	//   slot_raw_collision[gc]: RaW -- incoming write vs in-flight read
	//   slot_war_collision[gc]: WaR -- incoming read  vs in-flight write
	//
	// Global OR flags gate the SLOT_IDLE accept conditions:
	//   wr_collision  -- blocks a new write  (WaW hazard present)
	//   raw_collision -- blocks a new write  (RaW hazard present)
	//   war_collision -- blocks a new read   (WaR hazard present)

	localparam CL_BITS = $clog2(CACHE_LINE_SIZE);

	// Cache-line range of the incoming write (FIFO head)
	logic [ADDR_WIDTH-1:0]  inc_cl_start;       // lowest  CL index touched by incoming write
	logic [ADDR_WIDTH-1:0]  inc_cl_end;         // highest CL index touched by incoming write

	// Cache-line range of the incoming read (FIFO head)
	logic [ADDR_WIDTH-1:0]  inc_rd_cl_start;    // lowest  CL index touched by incoming read
	logic [ADDR_WIDTH-1:0]  inc_rd_cl_end;      // highest CL index touched by incoming read

	// Cache-line range of each in-flight slot's write burst
	logic [ADDR_WIDTH-1:0]  slot_cl_start    [0:NUM_SLOTS-1];
	logic [ADDR_WIDTH-1:0]  slot_cl_end      [0:NUM_SLOTS-1];

	// Cache-line range of each in-flight slot's read burst
	logic [ADDR_WIDTH-1:0]  slot_rd_cl_start [0:NUM_SLOTS-1];
	logic [ADDR_WIDTH-1:0]  slot_rd_cl_end   [0:NUM_SLOTS-1];

	// Per-slot collision flags
	logic                   slot_wr_collision  [0:NUM_SLOTS-1]; // WaW
	logic                   slot_raw_collision [0:NUM_SLOTS-1]; // RaW: incoming write vs in-flight read
	logic                   slot_war_collision [0:NUM_SLOTS-1]; // WaR: incoming read  vs in-flight write

	// Global collision flags
	logic                   wr_collision;   // OR of slot_wr_collision  -- gates write accept
	logic                   raw_collision;  // OR of slot_raw_collision -- gates write accept
	logic                   war_collision;  // OR of slot_war_collision -- gates read  accept

	/*=====================================================================*/
	/*  Combinational per-slot derived signals                            */
	/*=====================================================================*/

	logic                     resp_ready          [0:NUM_SLOTS-1];
	logic [ADDR_WIDTH-1:0]    slot_beat_addr      [0:NUM_SLOTS-1];
	logic [MAX_SIZE-1:0]      slot_beat_size      [0:NUM_SLOTS-1];
	logic [ADDR_WIDTH-1:0]    slot_base_addr      [0:NUM_SLOTS-1];
	logic                     slot_is_last        [0:NUM_SLOTS-1];

	/*=====================================================================*/
	/*  Dual disassembler wires                                           */
	/*                                                                     */
	/*  _wr suffix = write disassembler (is_wr=1, wired to whichever     */
	/*               slot is currently a write transaction)               */
	/*  _rd suffix = read disassembler  (is_wr=0, wdata/wstrb=0, wired  */
	/*               to whichever slot is currently a read transaction)   */
	/*                                                                     */
	/*  dis_wr_slot / dis_rd_slot: which slot feeds each disassembler.   */
	/*  These are set in the always_comb mux below and used only to       */
	/*  drive the disassembler inputs -- they do NOT gate pushes.         */
	/*  The push-arbitration dis_slot (unchanged logic) still selects     */
	/*  which builder output goes into the FIFO.                          */
	/*=====================================================================*/

	logic                     dis_wr_valid_lsb;
	logic                     dis_wr_valid_msb;
	logic [ADDR_WIDTH-1:0]    dis_wr_lsb_addr;
	logic [ADDR_WIDTH-1:0]    dis_wr_msb_addr;
	logic [PDATA_WIDTH-1:0]   dis_wr_lsb_wdata;
	logic [PDATA_WIDTH-1:0]   dis_wr_msb_wdata;
	logic [PSTRB_WIDTH-1:0]   dis_wr_lsb_pstrb;
	logic [PSTRB_WIDTH-1:0]   dis_wr_msb_pstrb;

	logic                     dis_rd_valid_lsb;
	logic                     dis_rd_valid_msb;
	logic [ADDR_WIDTH-1:0]    dis_rd_lsb_addr;
	logic [ADDR_WIDTH-1:0]    dis_rd_msb_addr;

	// Disassembler input mux wires
	logic                     dis_wr_slot;
	logic                     dis_rd_slot;
	logic [ADDR_WIDTH-1:0]    dis_wr_beat_addr;
	logic [MAX_SIZE-1:0]      dis_wr_beat_size;
	logic [DATA_WIDTH-1:0]    dis_wr_wdata;
	logic [DATA_WIDTH/8-1:0]  dis_wr_wstrb;
	logic [ADDR_WIDTH-1:0]    dis_rd_beat_addr;
	logic [MAX_SIZE-1:0]      dis_rd_beat_size;

	// Per-slot valid flags resolved to the correct disassembler
	logic                     slot_valid_lsb [0:NUM_SLOTS-1];
	logic                     slot_valid_msb [0:NUM_SLOTS-1];

	// Builder outputs
	struct_types::apb_struct  builder_wr_apb_lsb;
	struct_types::apb_struct  builder_wr_apb_msb;
	struct_types::apb_struct  builder_rd_apb_lsb;
	struct_types::apb_struct  builder_rd_apb_msb;

	// Push-arbitration mux: which slot's builder feeds the FIFO
	logic                     dis_slot;

	/*=====================================================================*/
	/*  Assembler wires                                                    */
	/*=====================================================================*/

	logic                     asm_slot;
	logic [DATA_WIDTH-1:0]    asm_rdata;
	logic [RRESP_WIDTH-1:0]   asm_rresp;
	logic [BRESP_WIDTH-1:0]   asm_bresp;

	/*=====================================================================*/
	/*  axi_resp_builder wires                                             */
	/*=====================================================================*/

	struct_types::axi_rd_data rb_rd_data;
	struct_types::axi_wr_resp rb_wr_resp;

	/*=====================================================================*/
	/*  register_file wires                                                */
	/*=====================================================================*/

	logic                     rf_enable_wr;
	logic                     rf_done_wr;
	logic [ID_WIDTH-1:0]      rf_txn_id_wr;
	struct_types::axi_wr_resp rf_axi_wr_resp_in;
	struct_types::axi_wr_resp rf_axi_wr_resp_out;
	logic [ID_WIDTH-1:0]      rf_wr_id_out;
	logic                     rf_wr_valid;

	logic                     rf_enable_rd;
	logic                     rf_done_rd;
	logic [ID_WIDTH-1:0]      rf_txn_id_rd;
	struct_types::axi_rd_data rf_axi_rd_data_in;
	struct_types::axi_rd_data rf_axi_rd_data_out;
	logic [ID_WIDTH-1:0]      rf_rd_id_out;
	logic                     rf_rd_valid;

	/*=====================================================================*/
	/*  s0_takes_wr/rd -- slot-0-wins signals for slot 1                  */
	/*  slot_req_latched, slot_lsb_done, slot_msb_done                    */
	/*  slot_wants_*_pop                                                   */
	/*=====================================================================*/

	logic s0_takes_wr        [0:NUM_SLOTS-1];
	logic s0_takes_rd        [0:NUM_SLOTS-1];
	logic slot_req_latched   [0:NUM_SLOTS-1];
	logic slot_lsb_done      [0:NUM_SLOTS-1];
	logic slot_msb_done      [0:NUM_SLOTS-1];
	logic slot_wants_wr_req_pop  [0:NUM_SLOTS-1];
	logic slot_wants_rd_req_pop  [0:NUM_SLOTS-1];
	logic slot_wants_wr_data_pop [0:NUM_SLOTS-1];
	logic s0_dispatching;

	/*=====================================================================*/
	/*  Sub-module instantiations                                          */
	/*=====================================================================*/

	disassembler u_disassembler_wr (
		.is_wr      (1'b1),
		.beat_addr  (dis_wr_beat_addr),
		.beat_size  (dis_wr_beat_size),
		.wdata      (dis_wr_wdata),
		.wstrb      (dis_wr_wstrb),
		.valid_lsb  (dis_wr_valid_lsb),
		.valid_msb  (dis_wr_valid_msb),
		.lsb_addr   (dis_wr_lsb_addr),
		.msb_addr   (dis_wr_msb_addr),
		.lsb_wdata  (dis_wr_lsb_wdata),
		.msb_wdata  (dis_wr_msb_wdata),
		.lsb_pstrb  (dis_wr_lsb_pstrb),
		.msb_pstrb  (dis_wr_msb_pstrb)
	);

	disassembler u_disassembler_rd (
		.is_wr      (1'b0),
		.beat_addr  (dis_rd_beat_addr),
		.beat_size  (dis_rd_beat_size),
		.wdata      ('0),
		.wstrb      ('0),
		.valid_lsb  (dis_rd_valid_lsb),
		.valid_msb  (dis_rd_valid_msb),
		.lsb_addr   (dis_rd_lsb_addr),
		.msb_addr   (dis_rd_msb_addr),
		.lsb_wdata  (),
		.msb_wdata  (),
		.lsb_pstrb  (),
		.msb_pstrb  ()
	);

	apb_req_builder u_apb_req_builder_wr (
		.is_wr      (1'b1),
		.lsb_addr   (dis_wr_lsb_addr),
		.lsb_wdata  (dis_wr_lsb_wdata),
		.lsb_pstrb  (dis_wr_lsb_pstrb),
		.msb_addr   (dis_wr_msb_addr),
		.msb_wdata  (dis_wr_msb_wdata),
		.msb_pstrb  (dis_wr_msb_pstrb),
		.apb_lsb    (builder_wr_apb_lsb),
		.apb_msb    (builder_wr_apb_msb)
	);

	apb_req_builder u_apb_req_builder_rd (
		.is_wr      (1'b0),
		.lsb_addr   (dis_rd_lsb_addr),
		.lsb_wdata  ('0),
		.lsb_pstrb  ('0),
		.msb_addr   (dis_rd_msb_addr),
		.msb_wdata  ('0),
		.msb_pstrb  ('0),
		.apb_lsb    (builder_rd_apb_lsb),
		.apb_msb    (builder_rd_apb_msb)
	);

	assembler u_assembler (
		.valid_lsb   (slot_expected_lsb[asm_slot]),
		.valid_msb   (slot_expected_msb[asm_slot]),
		.lsb_prdata  (slot_lsb_prdata  [asm_slot]),
		.lsb_pslverr (slot_lsb_pslverr [asm_slot]),
		.msb_prdata  (slot_msb_prdata  [asm_slot]),
		.msb_pslverr (slot_msb_pslverr [asm_slot]),
		.rdata       (asm_rdata),
		.rresp       (asm_rresp),
		.bresp       (asm_bresp)
	);

	axi_resp_builder u_axi_resp_builder (
		.transaction_id (slot_is_wr[asm_slot]
							? slot_wr_req[asm_slot].awid
							: slot_rd_req[asm_slot].arid),
		.is_last        (slot_is_last[asm_slot]),
		.rdata          (asm_rdata),
		.rresp          (asm_rresp),
		.bresp          (asm_bresp),
		.rd_data        (rb_rd_data),
		.wr_resp        (rb_wr_resp)
	);

	register_file u_register_file (
		.clk                (clk),
		.rst_n              (rst_n),
		.enable_wr          (rf_enable_wr),
		.done_wr            (rf_done_wr),
		.rf_txn_id_wr       (rf_txn_id_wr),
		.rf_axi_wr_resp_in  (rf_axi_wr_resp_in),
		.rf_axi_wr_resp_out (rf_axi_wr_resp_out),
		.rf_wr_id_out       (rf_wr_id_out),
		.rf_wr_valid        (rf_wr_valid),
		.enable_rd          (rf_enable_rd),
		.done_rd            (rf_done_rd),
		.rf_txn_id_rd       (rf_txn_id_rd),
		.rf_axi_rd_data_in  (rf_axi_rd_data_in),
		.rf_axi_rd_data_out (rf_axi_rd_data_out),
		.rf_rd_id_out       (rf_rd_id_out),
		.rf_rd_valid        (rf_rd_valid)
	);

	/*=====================================================================*/
	/*  register_file enable / done wiring                                */
	/*=====================================================================*/

	always_comb begin
		rf_enable_wr       = 1'b0;
		rf_enable_rd       = 1'b0;
		rf_txn_id_wr       = '0;
		rf_txn_id_rd       = '0;
		rf_axi_wr_resp_in  = '0;
		rf_axi_rd_data_in  = '0;

		for (int s = 0; s < NUM_SLOTS; s++) begin
			if (slot_state[s] == SLOT_WAIT_RESP && resp_ready[s] &&
				asm_slot == logic'(s)) begin
				if (slot_is_wr[s]) begin
				if (slot_is_last[s]) begin   // <-- only commit on wlast beat
						rf_enable_wr      = 1'b1;
						rf_txn_id_wr      = slot_wr_req[s].awid;
						rf_axi_wr_resp_in = rb_wr_resp;
					rf_axi_wr_resp_in.bresp = rb_wr_resp.bresp | {slot_wr_burst_err[s], 1'b0};
				end
				end else begin
					rf_enable_rd      = 1'b1;
					rf_txn_id_rd      = slot_rd_req[s].arid;
					rf_axi_rd_data_in = rb_rd_data;
				end
			end
		end
	end
	
	assign wr_resp_push_n  = !(rf_wr_valid && !wr_resp_full);
	assign wr_resp_fifo_in = rf_axi_wr_resp_out;
	assign rf_done_wr      = !wr_resp_push_n;

	assign rd_data_push_n  = !(rf_rd_valid && !rd_data_full);
	assign rd_data_fifo_in = rf_axi_rd_data_out;
	assign rf_done_rd      = !rd_data_push_n;

	/*=====================================================================*/
	/*  Collision detection logic (WaW, RaW, WaR)                        */
	/*=====================================================================*/

	// -----------------------------------------------------------------------
	// Cache-line range for the INCOMING WRITE (FIFO head)
	// burst_bytes = (awlen+1) << awsize
	// end_addr    = awaddr + burst_bytes - 1
	// -----------------------------------------------------------------------
	always_comb begin : comb_inc_wr_cl
		logic [ADDR_WIDTH-1:0] inc_burst_bytes;
		logic [ADDR_WIDTH-1:0] inc_end_addr;
		inc_burst_bytes = ADDR_WIDTH'(wr_req_fifo_out.awlen + 1)
						  << wr_req_fifo_out.awsize;
		inc_end_addr    = wr_req_fifo_out.awaddr + inc_burst_bytes - 1;
		inc_cl_start    = wr_req_fifo_out.awaddr >> CL_BITS;
		inc_cl_end      = inc_end_addr            >> CL_BITS;
	end

	// -----------------------------------------------------------------------
	// Cache-line range for the INCOMING READ (FIFO head)
	// burst_bytes = (arlen+1) << arsize
	// end_addr    = araddr + burst_bytes - 1
	// -----------------------------------------------------------------------
	always_comb begin : comb_inc_rd_cl
		logic [ADDR_WIDTH-1:0] inc_rd_burst_bytes;
		logic [ADDR_WIDTH-1:0] inc_rd_end_addr;
		inc_rd_burst_bytes = ADDR_WIDTH'(rd_req_fifo_out.arlen + 1)
							 << rd_req_fifo_out.arsize;
		inc_rd_end_addr    = rd_req_fifo_out.araddr + inc_rd_burst_bytes - 1;
		inc_rd_cl_start    = rd_req_fifo_out.araddr >> CL_BITS;
		inc_rd_cl_end      = inc_rd_end_addr        >> CL_BITS;
	end

	// -----------------------------------------------------------------------
	// Per-slot cache-line ranges and collision flags
	// Ranges [a,b] and [c,d] overlap iff NOT (b < c OR d < a).
	// -----------------------------------------------------------------------
	generate
		for (genvar gc = 0; gc < NUM_SLOTS; gc++) begin : gen_cl_check
			always_comb begin : comb_slot_cl
				logic [ADDR_WIDTH-1:0] slot_burst_bytes;
				logic [ADDR_WIDTH-1:0] slot_end_addr;
				logic [ADDR_WIDTH-1:0] slot_rd_burst_bytes;
				logic [ADDR_WIDTH-1:0] slot_rd_end_addr;

				// --- Write CL range for this slot ---
				slot_burst_bytes     = ADDR_WIDTH'(slot_wr_req[gc].awlen + 1)
									   << slot_wr_req[gc].awsize;
				slot_end_addr        = slot_wr_req[gc].awaddr + slot_burst_bytes - 1;
				slot_cl_start[gc]    = slot_wr_req[gc].awaddr >> CL_BITS;
				slot_cl_end  [gc]    = slot_end_addr           >> CL_BITS;

				// --- Read CL range for this slot ---
				slot_rd_burst_bytes     = ADDR_WIDTH'(slot_rd_req[gc].arlen + 1)
										  << slot_rd_req[gc].arsize;
				slot_rd_end_addr        = slot_rd_req[gc].araddr + slot_rd_burst_bytes - 1;
				slot_rd_cl_start[gc]    = slot_rd_req[gc].araddr >> CL_BITS;
				slot_rd_cl_end  [gc]    = slot_rd_end_addr       >> CL_BITS;

				// --- WaW: incoming WRITE vs in-flight WRITE ---
				slot_wr_collision[gc] =
					(slot_state[gc] != SLOT_IDLE) &&
					slot_is_wr[gc]                &&
					!(inc_cl_end         < slot_cl_start[gc] ||
					  slot_cl_end  [gc]  < inc_cl_start);

				// --- RaW: incoming WRITE vs in-flight READ ---
				// Hold a new write if a currently active READ touches the
				// same cache line(s).  Compare incoming write range against
				// the in-flight slot's READ range.
				slot_raw_collision[gc] =
					(slot_state[gc] != SLOT_IDLE) &&
					!slot_is_wr[gc]               &&
					!(inc_cl_end            < slot_rd_cl_start[gc] ||
					  slot_rd_cl_end  [gc]  < inc_cl_start);

				// --- WaR: incoming READ vs in-flight WRITE ---
				// Hold a new read if a currently active WRITE touches the
				// same cache line(s).  Compare incoming read range against
				// the in-flight slot's WRITE range.
				slot_war_collision[gc] =
					(slot_state[gc] != SLOT_IDLE) &&
					slot_is_wr[gc]                &&
					!(inc_rd_cl_end       < slot_cl_start[gc] ||
					  slot_cl_end  [gc]   < inc_rd_cl_start);
			end
		end
	endgenerate

	// Global OR flags
	always_comb begin
		wr_collision  = 1'b0;
		raw_collision = 1'b0;
		war_collision = 1'b0;
		for (int c = 0; c < NUM_SLOTS; c++) begin
			wr_collision  = wr_collision  | slot_wr_collision [c];
			raw_collision = raw_collision | slot_raw_collision[c];
			war_collision = war_collision | slot_war_collision[c];
		end
	end

	/*=====================================================================*/
	/*  Combinational intermediates for per-slot FSM                      */
	/*=====================================================================*/

	always_comb begin
		for (int i = 0; i < NUM_SLOTS; i++) begin
			// s0_takes_wr/rd: true when slot 0 is also idle this cycle and
			// will win the same direction, so slot 1 must not also pop that
			// FIFO.  rd_priority inverts the normal write-first preference.
			//
			// Write gate: blocked by WaW (wr_collision) OR RaW (raw_collision).
			// Read  gate: blocked by WaR (war_collision).
			s0_takes_wr[i] = (i == 1) &&
							  (slot_state[0] == SLOT_IDLE) &&
							  !wr_req_empty && !wr_data_empty &&
							  !rd_priority &&
							  !wr_collision &&    // no WaW hazard
							  !raw_collision;     // no RaW hazard

			s0_takes_rd[i] = (i == 1) &&
							  (slot_state[0] == SLOT_IDLE) &&
							  !rd_req_empty &&
							  !war_collision &&   // no WaR hazard
							  (rd_priority ||
							   (wr_req_empty || wr_data_empty));

			slot_req_latched[i] = slot_is_wr[i]
								 ? slot_wr_req_latched[i]
								 : slot_rd_req_latched[i];

			slot_lsb_done[i] = !slot_valid_lsb[i] || slot_lsb_sent[i] ||
								(!apb_req_push_n && dis_slot == logic'(i) &&
								 !slot_lsb_sent[i] && slot_valid_lsb[i]);

			slot_msb_done[i] = !slot_valid_msb[i] || slot_msb_sent[i] ||
								(!apb_req_push_n && dis_slot == logic'(i) &&
								 (slot_lsb_sent[i] || !slot_valid_lsb[i]) &&
								 !slot_msb_sent[i] && slot_valid_msb[i]);
		end
	end

	/*=====================================================================*/
	/*  Anti-starvation counter and rd_priority latch                     */
	/*=====================================================================*/
	// Runs independently of the per-slot FSM.
	//
	// COUNTER RULES
	//   Increments: any cycle a slot in SLOT_IDLE accepts a WRITE while
	//               rd_req is non-empty (i.e. a read was passed over).
	//   Saturates:  at WR_STARVE_THRESH (never wraps).
	//   Resets:     when a read is accepted on any slot, OR rd_req empties.
	//
	// LATCH RULES
	//   rd_priority asserts: counter reaches WR_STARVE_THRESH.
	//   rd_priority clears:  a read is accepted on any slot.
	//
	// Disabled (all zeros) when WR_STARVE_THRESH==0.

	logic wr_accepted_this_cycle;
	logic rd_accepted_this_cycle;

	always_comb begin
		wr_accepted_this_cycle = 1'b0;
		rd_accepted_this_cycle = 1'b0;
		for (int s = 0; s < NUM_SLOTS; s++) begin
			if (slot_state[s] == SLOT_IDLE) begin
				// Write accepted: write condition fires
				// (same guards as the FSM write condition below)
				if (!wr_req_empty && !wr_data_empty &&
					(!rd_priority || rd_req_empty) &&
					!wr_collision && !raw_collision &&
					!(s == 1 && s0_takes_wr[1]))
					wr_accepted_this_cycle = 1'b1;
				// Read accepted: read condition fires
				// (same guards as the FSM read condition below)
				if (!rd_req_empty &&
					!war_collision &&
					(rd_priority || wr_req_empty || wr_data_empty) &&
					!(s == 1 && s0_takes_rd[1]))
					rd_accepted_this_cycle = 1'b1;
			end
		end
	end

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wr_starve_cnt <= '0;
			rd_priority   <= 1'b0;
		end else if (WR_STARVE_THRESH == 0) begin
			wr_starve_cnt <= '0;
			rd_priority   <= 1'b0;
		end else begin
			if (rd_accepted_this_cycle) begin
				rd_priority   <= 1'b0;
				wr_starve_cnt <= '0;
			end else if (rd_req_empty) begin
				wr_starve_cnt <= '0;
				rd_priority   <= 1'b0;
			end else if (wr_accepted_this_cycle && !rd_priority) begin
				if (wr_starve_cnt == STARVE_W'(WR_STARVE_THRESH - 1)) begin
					rd_priority   <= 1'b1;
					wr_starve_cnt <= '0;
				end else begin
					wr_starve_cnt <= wr_starve_cnt + 1'b1;
				end
			end
		end
	end

	/*=====================================================================*/
	/*  Per-slot sequential state machine                                  */
	/*=====================================================================*/

	generate
		for (genvar gi = 0; gi < NUM_SLOTS; gi++) begin : gen_slot_ff

			always_ff @(posedge clk or negedge rst_n) begin
				if (!rst_n) begin
					slot_state          [gi] <= SLOT_IDLE;
					slot_is_wr          [gi] <= 1'b0;
					slot_wr_req         [gi] <= '0;
					slot_wr_data        [gi] <= '0;
					slot_rd_req         [gi] <= '0;
					slot_wr_req_latched [gi] <= 1'b0;
					slot_rd_req_latched [gi] <= 1'b0;
					slot_wr_data_latched[gi] <= 1'b0;
					slot_beat_index     [gi] <= '0;
					slot_lsb_sent       [gi] <= 1'b0;
					slot_msb_sent       [gi] <= 1'b0;
					slot_expected_lsb   [gi] <= 1'b0;
					slot_expected_msb   [gi] <= 1'b0;
					slot_lsb_prdata     [gi] <= '0;
					slot_lsb_pslverr    [gi] <= 1'b0;
					slot_lsb_valid      [gi] <= 1'b0;
					slot_msb_prdata     [gi] <= '0;
					slot_msb_pslverr    [gi] <= 1'b0;
					slot_msb_valid      [gi] <= 1'b0;
					slot_wr_burst_err   [gi] <= 1'b0;
				end else begin

					case (slot_state[gi])

						/*---------------------------------------------------*/
						SLOT_IDLE: begin
						/*---------------------------------------------------*/
						// Normal path   (rd_priority=0): write beats read.
						// Priority path (rd_priority=1): read beats write.
						// In both cases the losing direction is still accepted
						// when the winning direction has nothing pending.
						//
						// Hazard gates (new in this revision):
						//   Write is additionally blocked by:
						//     wr_collision  -- WaW: in-flight write on same CL
						//     raw_collision -- RaW: in-flight read  on same CL
						//   Read is additionally blocked by:
						//     war_collision -- WaR: in-flight write on same CL

							// --- Write condition ---
							if (!wr_req_empty && !wr_data_empty &&
								(!rd_priority || rd_req_empty) &&
								!s0_takes_wr[gi] &&
								!wr_collision  &&   // no WaW cache-line hazard
								!raw_collision) begin  // no RaW cache-line hazard
								slot_state           [gi] <= SLOT_DISPATCH;
								slot_is_wr           [gi] <= 1'b1;
								slot_wr_req          [gi] <= wr_req_fifo_out;
								slot_wr_req_latched  [gi] <= 1'b1;
								slot_wr_data         [gi] <= wr_data_fifo_out;
								slot_wr_data_latched [gi] <= 1'b1;

							// --- Read condition ---
							end else if (!rd_req_empty &&
										 !war_collision &&   // no WaR cache-line hazard
										 (rd_priority ||
										  wr_req_empty || wr_data_empty) &&
										 !s0_takes_rd[gi]) begin
								slot_state           [gi] <= SLOT_DISPATCH;
								slot_is_wr           [gi] <= 1'b0;
								slot_rd_req          [gi] <= rd_req_fifo_out;
								slot_rd_req_latched  [gi] <= 1'b1;
							end
						end

						/*---------------------------------------------------*/
						SLOT_DISPATCH: begin
						/*---------------------------------------------------*/

							if (!wr_data_pop_n && slot_wants_wr_data_pop[gi]) begin
								slot_wr_data        [gi] <= wr_data_fifo_out;
								slot_wr_data_latched[gi] <= 1'b1;
							end

							if (!apb_req_push_n && dis_slot == logic'(gi)) begin
								if (!slot_lsb_sent[gi] && slot_valid_lsb[gi]) begin
									slot_lsb_sent    [gi] <= 1'b1;
									slot_expected_lsb[gi] <= 1'b1;
								end else if ((slot_lsb_sent[gi] || !slot_valid_lsb[gi]) &&
											 !slot_msb_sent[gi] && slot_valid_msb[gi]) begin
									slot_msb_sent    [gi] <= 1'b1;
									slot_expected_msb[gi] <= 1'b1;
								end
							end

							// BUG FIX: Clear expected flags only when THIS slot's
							// disassembler says the half is not valid.
							if (!slot_valid_lsb[gi]) slot_expected_lsb[gi] <= 1'b0;
							if (!slot_valid_msb[gi]) slot_expected_msb[gi] <= 1'b0;

							if (slot_req_latched[gi] && slot_lsb_done[gi] && slot_msb_done[gi])
								slot_state[gi] <= SLOT_WAIT_RESP;
						end

						/*---------------------------------------------------*/
						SLOT_WAIT_RESP: begin
						/*---------------------------------------------------*/

							if (!apb_resp_pop_n &&
								apb_tag_fifo_out.slot == logic'(gi)) begin
								if (!apb_tag_fifo_out.is_msb) begin
									slot_lsb_prdata [gi] <= apb_resp_fifo_out.prdata;
									slot_lsb_pslverr[gi] <= apb_resp_fifo_out.pslverr;
									slot_lsb_valid  [gi] <= 1'b1;
								end else begin
									slot_msb_prdata [gi] <= apb_resp_fifo_out.prdata;
									slot_msb_pslverr[gi] <= apb_resp_fifo_out.pslverr;
									slot_msb_valid  [gi] <= 1'b1;
								end
							end

							if (resp_ready[gi] && asm_slot == logic'(gi)) begin

								slot_lsb_valid   [gi] <= 1'b0;
								slot_msb_valid   [gi] <= 1'b0;
								slot_lsb_sent    [gi] <= 1'b0;
								slot_msb_sent    [gi] <= 1'b0;
								slot_expected_lsb[gi] <= 1'b0;
								slot_expected_msb[gi] <= 1'b0;

								if (slot_is_wr[gi]) begin
									slot_wr_burst_err[gi] <=
										slot_wr_data[gi].wlast ? 1'b0
										: slot_wr_burst_err[gi] | asm_bresp[1];

									if (slot_wr_data[gi].wlast) begin
										slot_state           [gi] <= SLOT_IDLE;
										slot_wr_req_latched  [gi] <= 1'b0;
										slot_wr_data_latched [gi] <= 1'b0;
										slot_beat_index      [gi] <= '0;
										slot_wr_burst_err    [gi] <= 1'b0;
									end else begin
										slot_state           [gi] <= SLOT_DISPATCH;
										slot_wr_data_latched [gi] <= 1'b0;
										slot_beat_index      [gi] <= slot_beat_index[gi] + 1'b1;
									end

								end else begin
									if (slot_beat_index[gi] == slot_rd_req[gi].arlen) begin
										slot_state          [gi] <= SLOT_IDLE;
										slot_rd_req_latched [gi] <= 1'b0;
										slot_beat_index     [gi] <= '0;
									end else begin
										slot_state      [gi] <= SLOT_DISPATCH;
										slot_beat_index [gi] <= slot_beat_index[gi] + 1'b1;
									end
								end
							end
						end

						default: slot_state[gi] <= SLOT_IDLE;

					endcase
				end
			end
		end
	endgenerate

	/*=====================================================================*/
	/*  Combinational per-slot derived signals (generate)                 */
	/*=====================================================================*/

	generate
		for (genvar gs = 0; gs < NUM_SLOTS; gs++) begin : gen_slot_comb
			always_comb begin
				slot_beat_size[gs] = slot_is_wr[gs]
								   ? slot_wr_req[gs].awsize
								   : slot_rd_req[gs].arsize;
				slot_base_addr[gs] = slot_is_wr[gs]
								   ? slot_wr_req[gs].awaddr
								   : slot_rd_req[gs].araddr;

				slot_beat_addr[gs] = slot_base_addr[gs]
								   + (ADDR_WIDTH'(slot_beat_index[gs])
									  << slot_beat_size[gs]);

				slot_is_last[gs] = slot_is_wr[gs]
								 ? slot_wr_data[gs].wlast
								 : (slot_beat_index[gs] == slot_rd_req[gs].arlen);

				resp_ready[gs] = (!slot_expected_lsb[gs] || slot_lsb_valid[gs]) &&
								 (!slot_expected_msb[gs] || slot_msb_valid[gs]);
			end
		end
	endgenerate

	/*=====================================================================*/
	/*  Disassembler input muxes                                          */
	/*=====================================================================*/

	always_comb begin
		// --- Write disassembler input mux ---
		dis_wr_slot = 1'b0;
		for (int s = NUM_SLOTS-1; s >= 0; s--) begin
			if (slot_state[s] == SLOT_DISPATCH && slot_is_wr[s] &&
				slot_wr_req_latched[s])
				dis_wr_slot = logic'(s);
		end

		dis_wr_beat_addr = slot_beat_addr[dis_wr_slot];
		dis_wr_beat_size = slot_beat_size[dis_wr_slot];
		dis_wr_wdata     = slot_wr_data[dis_wr_slot].wdata;
		dis_wr_wstrb     = slot_wr_data[dis_wr_slot].wstrb;

		// --- Read disassembler input mux ---
		dis_rd_slot = 1'b0;
		for (int s = NUM_SLOTS-1; s >= 0; s--) begin
			if (slot_state[s] == SLOT_DISPATCH && !slot_is_wr[s] &&
				slot_rd_req_latched[s])
				dis_rd_slot = logic'(s);
		end

		dis_rd_beat_addr = slot_beat_addr[dis_rd_slot];
		dis_rd_beat_size = slot_beat_size[dis_rd_slot];
	end

	/*=====================================================================*/
	/*  Per-slot valid_lsb/msb (from the appropriate disassembler)       */
	/*=====================================================================*/

	always_comb begin
		for (int s = 0; s < NUM_SLOTS; s++) begin
			if (slot_is_wr[s]) begin
				slot_valid_lsb[s] = dis_wr_valid_lsb && (dis_wr_slot == logic'(s));
				slot_valid_msb[s] = dis_wr_valid_msb && (dis_wr_slot == logic'(s));
			end else begin
				slot_valid_lsb[s] = dis_rd_valid_lsb && (dis_rd_slot == logic'(s));
				slot_valid_msb[s] = dis_rd_valid_msb && (dis_rd_slot == logic'(s));
			end
		end
	end

	/*=====================================================================*/
	/*  Push arbitration: dis_slot                                        */
	/*=====================================================================*/

	always_comb begin
		s0_dispatching = (slot_state[0] == SLOT_DISPATCH) &&
						 (slot_wr_req_latched[0] || slot_rd_req_latched[0]) &&
						 (!slot_lsb_sent[0] || !slot_msb_sent[0]);

		dis_slot = s0_dispatching ? 1'b0 : 1'b1;
	end

	/*=====================================================================*/
	/*  Assembler input mux                                                */
	/*=====================================================================*/

	always_comb begin
		asm_slot = (slot_state[0] == SLOT_WAIT_RESP && resp_ready[0]) ? 1'b0 : 1'b1;
	end

	/*=====================================================================*/
	/*  FIFO pop want signals (per slot, combinational)                   */
	/*=====================================================================*/

	generate
		for (genvar gp = 0; gp < NUM_SLOTS; gp++) begin : gen_pop_wants
			always_comb begin
				slot_wants_wr_req_pop [gp] = 1'b0;
				slot_wants_rd_req_pop [gp] = 1'b0;
				slot_wants_wr_data_pop[gp] = 1'b0;

				if (slot_state[gp] == SLOT_IDLE) begin
					// Write pop: same guards as the FSM write condition
					if (!wr_req_empty && !wr_data_empty &&
						!wr_collision && !raw_collision &&
						!s0_takes_wr[gp]) begin
						slot_wants_wr_req_pop [gp] = 1'b1;
						slot_wants_wr_data_pop[gp] = 1'b1;
					// Read pop: same guards as the FSM read condition
					end else if ((wr_req_empty || wr_data_empty || wr_collision || raw_collision) &&
								 !rd_req_empty && !war_collision && !s0_takes_rd[gp]) begin
						slot_wants_rd_req_pop[gp] = 1'b1;
					end
				end

				if (slot_state[gp] == SLOT_DISPATCH) begin
					if (slot_is_wr[gp]) begin
						if (!wr_data_empty && slot_wr_req_latched[gp] &&
							!slot_wr_data_latched[gp])
							slot_wants_wr_data_pop[gp] = 1'b1;
					end
				end
			end
		end
	endgenerate

	/*=====================================================================*/
	/*  FIFO pop arbitration                                               */
	/*=====================================================================*/

	always_comb begin
		wr_req_pop_n  = 1'b1;
		rd_req_pop_n  = 1'b1;
		wr_data_pop_n = 1'b1;

		if      (slot_wants_wr_req_pop[0])  wr_req_pop_n  = 1'b0;
		else if (slot_wants_wr_req_pop[1])  wr_req_pop_n  = 1'b0;

		if      (slot_wants_rd_req_pop[0])  rd_req_pop_n  = 1'b0;
		else if (slot_wants_rd_req_pop[1])  rd_req_pop_n  = 1'b0;

		if      (slot_wants_wr_data_pop[0]) wr_data_pop_n = 1'b0;
		else if (slot_wants_wr_data_pop[1]) wr_data_pop_n = 1'b0;
	end

	/*=====================================================================*/
	/*  APB request push + sideband tag push                              */
	/*=====================================================================*/

	always_comb begin
		apb_req_push_n  = 1'b1;
		apb_req_fifo_in = '0;
		apb_tag_push_n  = 1'b1;
		apb_tag_fifo_in = '0;

		if (!apb_req_full && !apb_tag_full) begin
			for (int s = 0; s < NUM_SLOTS; s++) begin
				if (apb_req_push_n &&
					slot_state[s] == SLOT_DISPATCH &&
					(slot_wr_req_latched[s] || slot_rd_req_latched[s]) &&
					dis_slot == logic'(s)) begin

					if (!slot_lsb_sent[s] && slot_valid_lsb[s]) begin
						apb_req_push_n  = 1'b0;
						apb_req_fifo_in = slot_is_wr[s]
										? builder_wr_apb_lsb
										: builder_rd_apb_lsb;
						apb_tag_push_n  = 1'b0;
						apb_tag_fifo_in = '{slot: logic'(s), is_msb: 1'b0};
					end else if ((slot_lsb_sent[s] || !slot_valid_lsb[s]) &&
								 !slot_msb_sent[s] && slot_valid_msb[s]) begin
						apb_req_push_n  = 1'b0;
						apb_req_fifo_in = slot_is_wr[s]
										? builder_wr_apb_msb
										: builder_rd_apb_msb;
						apb_tag_push_n  = 1'b0;
						apb_tag_fifo_in = '{slot: logic'(s), is_msb: 1'b1};
					end
				end
			end
		end
	end

	/*=====================================================================*/
	/*  APB response pop                                                   */
	/*=====================================================================*/

	assign apb_resp_pop_n = apb_resp_empty || apb_tag_empty;
	assign apb_tag_pop_n  = apb_resp_pop_n;

endmodule