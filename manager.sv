/*------------------------------------------------------------------------------
 * File          : manager.sv
 * Project       : RTL
 * Description   : AXI-to-APB bridge manager — separate write and read
 *                 pipelines, each with 2 register-file slots.
 *
 * ============================================================================
 * ARCHITECTURE OVERVIEW
 * ============================================================================
 *
 *   Write pipeline   handles AXI write transactions exclusively.
 *   Read  pipeline   handles AXI read  transactions exclusively.
 *
 * Both pipelines can be in DISPATCH or WAIT_RESP simultaneously, pushing
 * APB sub-requests to the shared APB request FIFO. Per-cycle push arbitration
 * gives writes priority over reads (subject to the hazard and beat-atomicity
 * rules described below).
 *
 * ============================================================================
 * SLOT LIFECYCLE (per pipeline)
 * ============================================================================
 *
 *  IDLE       No active slot. Waits for a request in the relevant AXI FIFO
 *             and a free register-file slot. The next request is not accepted
 *             until the previous slot's last beat is in the APB request FIFO
 *             (all_beats_pushed), guaranteeing in-order APB issue.
 *
 *  DISPATCH   Pushes LSB then MSB APB sub-requests for the current beat.
 *             When all sub-requests of the current beat are pushed, transitions
 *             to WAIT_RESP. If this was the last beat, sets all_beats_pushed
 *             first so the next request can be accepted by wr/rd_can_accept_next.
 *
 *  WAIT_RESP  Collects APB responses routed back via the sideband tag.
 *             On beat complete: assembles AXI response; for writes, stores in
 *             register file; for reads, pushes directly to rd_data_fifo.
 *             Non-last beat: returns to DISPATCH for the next beat.
 *             Last beat: frees the slot; write responses drain independently
 *             via the register file.
 *
 * ============================================================================
 * SECOND-SLOT PIPELINING
 * ============================================================================
 *
 * wr_can_accept_next fires when the FSM is in WAIT_RESP or DISPATCH, the
 * active slot's last beat has already been pushed (all_beats_pushed), the
 * other slot is free, and no hazard blocks the new request. The second slot
 * is allocated immediately and its FSM moves to DISPATCH while the first
 * slot continues collecting APB responses in the background.
 * rd_can_accept_next is the symmetric condition for the read pipeline.
 *
 * ============================================================================
 * APB SIDEBAND TAG
 * ============================================================================
 *
 * A 3-bit tag is pushed alongside every APB request and popped alongside
 * every APB response, routing each response to the correct slot and half:
 *
 *   { is_wr [2], slot_idx [1], is_msb [0] }
 *
 *   is_wr    : 1 = write pipeline, 0 = read pipeline.
 *   slot_idx : which of the 2 register-file slots for that direction.
 *   is_msb   : 0 = LSB half, 1 = MSB half of the 64->32 disassembly.
 *
 * Because APB is strictly ordered (responses arrive in request order),
 * the tag FIFOs remain in sync with the data FIFOs automatically.
 *
 * ============================================================================
 * HAZARD POLICY  (first-come-first-served, same cache line)
 * ============================================================================
 *
 *   When a read and a write target the same 64-byte cache line (addr[31:6]),
 *   the pipeline that arrived first completes all of its APB beats before the
 *   other is allowed to start. "Arrived first" means entering DISPATCH one
 *   or more cycles earlier.
 *
 *   Enforcement has two layers:
 *     1. FSM gate (pre-DISPATCH): rd_addr_hazard blocks rd_can_accept while
 *        any live write slot covers the same cache line; wr_addr_hazard does
 *        the symmetric check. Both signals include a combinational forward-look
 *        (wr/rd_slot_occupied) so the hazard is visible the same cycle the
 *        conflicting transaction is first accepted, preventing simultaneous
 *        entry into DISPATCH.
 *     2. Push-arbiter gate (in-DISPATCH): rd_addr_hazard also suppresses read
 *        APB pushes when a conflicting write is in DISPATCH. A read beat that
 *        is already in progress (LSB pushed, MSB pending) is always allowed to
 *        complete atomically to preserve LSB/MSB ordering in the APB FIFO.
 *
 *   True simultaneous arrival (both IDLE, same cycle, colliding address):
 *   write wins the tie because rd_addr_hazard sees wr_slot_occupied one cycle
 *   before rd_slot_occupied is visible.
 *
 * ============================================================================
 * INTERNAL SUB-MODULES
 * ============================================================================
 *
 *   u_disassembler_wr / u_apb_req_builder_wr   write pipeline (is_wr=1)
 *   u_disassembler_rd / u_apb_req_builder_rd   read  pipeline (is_wr=0)
 *   u_assembler / u_axi_resp_builder            response assembly
 *   u_register_file                             4-slot request/response store
 *
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module manager
import struct_types::*;
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
	/* APB request FIFO + sideband tag FIFO                             */
	/*------------------------------------------------------------------*/
	output logic                        apb_req_push_n,
	output struct_types::apb_struct     apb_req_fifo_in,
	input  logic                        apb_req_full,

	output logic                        apb_tag_push_n,
	output struct_types::apb_tag_t      apb_tag_fifo_in,
	input  logic                        apb_tag_full,

	/*------------------------------------------------------------------*/
	/* APB response FIFO + sideband tag FIFO                            */
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
	/*  FSM state encoding                                                 */
	/*=====================================================================*/

	typedef enum logic [1:0] {
		ST_IDLE,
		ST_DISPATCH,
		ST_WAIT_RESP
	} pipe_state_t;

	/*=====================================================================*/
	/*  Write pipeline FSM + alloc pointer                                */
	/*=====================================================================*/

	pipe_state_t  wr_state;
	logic         wr_active_slot;   // slot currently in DISPATCH or WAIT_RESP
	logic         wr_alloc_ptr;     // next slot to allocate (toggles on each pop)

	/*=====================================================================*/
	/*  Read pipeline FSM + alloc pointer                                 */
	/*=====================================================================*/

	pipe_state_t  rd_state;
	logic         rd_active_slot;
	logic         rd_alloc_ptr;

	/*=====================================================================*/
	/*  Per-slot beat state write pipeline  [0:1]                       */
	/*  Indexed by slot_idx, not by wr_active_slot alone, because slot 0  */
	/*  may be in WAIT_RESP while slot 1 is in DISPATCH simultaneously.   */
	/*=====================================================================*/

	logic [MAX_LEN-1:0]       wr_beat_index       [0:1];
	logic                     wr_burst_err        [0:1]; // accumulated bresp error
	logic                     wr_req_lsb_pushed   [0:1];
	logic                     wr_req_msb_pushed   [0:1];
	logic                     wr_req_lsb_pending  [0:1]; // expect APB resp for LSB
	logic                     wr_req_msb_pending  [0:1]; // expect APB resp for MSB
	logic [PDATA_WIDTH-1:0]   wr_resp_lsb_data    [0:1];
	logic                     wr_resp_lsb_err     [0:1];
	logic                     wr_resp_lsb_done    [0:1];
	logic [PDATA_WIDTH-1:0]   wr_resp_msb_data    [0:1];
	logic                     wr_resp_msb_err     [0:1];
	logic                     wr_resp_msb_done    [0:1];
	logic                     wr_all_beats_pushed [0:1]; // last beat in APB FIFO
	logic                     wr_data_latched     [0:1]; // wr_data valid for beat
	
	struct_types::axi_wr_data wr_data_q [0:1]; // wr_data storage per slot (refreshed each beat)

	/*=====================================================================*/
	/*  Per-slot beat state  read pipeline  [0:1]                        */
	/*=====================================================================*/

	logic [MAX_LEN-1:0]       rd_beat_index      [0:1];
	logic                     rd_req_lsb_pushed  [0:1];
	logic                     rd_req_msb_pushed  [0:1];
	logic                     rd_req_lsb_pending [0:1];
	logic                     rd_req_msb_pending [0:1];
	logic [PDATA_WIDTH-1:0]   rd_resp_lsb_data   [0:1];
	logic                     rd_resp_lsb_err    [0:1];
	logic                     rd_resp_lsb_done   [0:1];
	logic [PDATA_WIDTH-1:0]   rd_resp_msb_data   [0:1];
	logic                     rd_resp_msb_err    [0:1];
	logic                     rd_resp_msb_done   [0:1];
	logic                     rd_all_beats_pushed[0:1];

	/*=====================================================================*/
	/*  Register file wires                                                */
	/*=====================================================================*/

	// Write direction
	logic                       rf_alloc_wr;
	logic                       rf_alloc_slot_wr;
	struct_types::axi_wr_req    rf_req_out_wr [0:1];

	logic                       rf_resp_ready_wr;
	logic                       rf_slot_free_wr [0:1];

	// Read direction
	logic                       rf_alloc_rd;
	logic                       rf_alloc_slot_rd;
	struct_types::axi_rd_req    rf_req_out_rd [0:1];
	logic                       rf_done_rd;
	logic                       rf_slot_free_rd [0:1];

	/*=====================================================================*/
	/*  Disassembler wires write                                           */
	/*=====================================================================*/

	logic [ADDR_WIDTH-1:0]   dis_wr_beat_addr;
	logic [MAX_SIZE-1:0]     dis_wr_beat_size;
	logic [DATA_WIDTH-1:0]   dis_wr_wdata;
	logic [WSTRB_WIDTH-1:0]  dis_wr_wstrb;

	logic [ADDR_WIDTH-1:0]   dis_wr_lsb_addr,  dis_wr_msb_addr;
	logic [PDATA_WIDTH-1:0]  dis_wr_lsb_wdata, dis_wr_msb_wdata;
	logic [PSTRB_WIDTH-1:0]  dis_wr_lsb_pstrb, dis_wr_msb_pstrb;
	logic                    dis_wr_valid_lsb,  dis_wr_valid_msb;

	struct_types::apb_struct  builder_wr_apb_lsb, builder_wr_apb_msb;

	/*=====================================================================*/
	/*  Disassembler wires  read                                          */
	/*=====================================================================*/

	logic [ADDR_WIDTH-1:0]   dis_rd_beat_addr;
	logic [MAX_SIZE-1:0]     dis_rd_beat_size;

	logic [ADDR_WIDTH-1:0]   dis_rd_lsb_addr,  dis_rd_msb_addr;
	logic                    dis_rd_valid_lsb,  dis_rd_valid_msb;

	struct_types::apb_struct  builder_rd_apb_lsb, builder_rd_apb_msb;

	/*=====================================================================*/
	/*  Assembler / resp-builder wires                            */
	/*=====================================================================*/

	logic [1:0]               asm_resp;
	logic [DATA_WIDTH-1:0]	  asm_rdata;
	struct_types::axi_wr_resp axi_wr_resp_assembled;

	/*=====================================================================*/
	/*  Combinational derived signals                                      */
	/*=====================================================================*/

	// Beat address for each pipeline slot
	logic [ADDR_WIDTH-1:0] wr_beat_addr [0:1];
	logic [ADDR_WIDTH-1:0] rd_beat_addr [0:1];

	// "All done" for current beat (both expected halves have responses)
	logic wr_resp_all_done [0:1];
	logic rd_resp_all_done [0:1];

	// is_last for a slot's current beat
	logic wr_is_last [0:1]; // wlast from captured wr_data
	logic rd_is_last [0:1]; // beat_index == arlen

	// lsb/msb_done combinational (includes the push-this-cycle case)
	logic wr_lsb_done [0:1], wr_msb_done [0:1];
	logic rd_lsb_done [0:1], rd_msb_done [0:1];

	// Per-slot beat error: any active half returned pslverr this beat.
	// Used in the ST_WAIT_RESP for-loop instead of asm_wr_bresp (which is
	// wired to wr_completing_slot and would give the wrong value for the
	// background slot when both complete simultaneously).
	logic wr_beat_err [0:1];

	// Hazard: read beat suppressed in push arbiter
	logic rd_addr_hazard;
	logic wr_addr_hazard;


	// Can each pipeline accept a new request?
	// wr_can_accept      : first request  FSM is in IDLE.
	// wr_can_accept_next : second request  FSM is in WAIT_RESP and the active
	//                      slot's beats are all in the APB FIFO (all_beats_pushed).
	//                      The new slot is allocated immediately and DISPATCH starts.
	logic wr_can_accept;
	logic wr_can_accept_next;
	logic rd_can_accept;
	logic rd_can_accept_next;

	/*=====================================================================*/
	/*  Beat address, response-done, is_last, beat-error  combinational    */
	/*  Also muxes active-slot signals into the disassembler inputs.       */
	/*=====================================================================*/

	always_comb begin
		for (int s = 0; s < 2; s++) begin
			wr_beat_addr[s] = rf_req_out_wr[s].awaddr + (ADDR_WIDTH'(wr_beat_index[s]) << rf_req_out_wr[s].awsize);
			rd_beat_addr[s] = rf_req_out_rd[s].araddr + (ADDR_WIDTH'(rd_beat_index[s]) << rf_req_out_rd[s].arsize);
			wr_is_last[s] = wr_data_q[s].wlast;
			rd_is_last[s] = (rd_beat_index[s] == rf_req_out_rd[s].arlen);
			wr_resp_all_done[s] = (!wr_req_lsb_pending[s] || wr_resp_lsb_done[s]) && (!wr_req_msb_pending[s] || wr_resp_msb_done[s]);
			rd_resp_all_done[s] = (!rd_req_lsb_pending[s] || rd_resp_lsb_done[s]) && (!rd_req_msb_pending[s] || rd_resp_msb_done[s]);
			wr_beat_err[s] = (wr_req_lsb_pending[s] && wr_resp_lsb_err[s]) || (wr_req_msb_pending[s] && wr_resp_msb_err[s]);
		end

		// Disassembler inputs: muxed to the active slot for each pipeline.
		dis_wr_beat_addr = wr_beat_addr[wr_active_slot];
		dis_wr_beat_size = rf_req_out_wr[wr_active_slot].awsize;
		dis_wr_wdata     = wr_data_q[wr_active_slot].wdata;
		dis_wr_wstrb     = wr_data_q[wr_active_slot].wstrb;
		dis_rd_beat_addr = rd_beat_addr[rd_active_slot];
		dis_rd_beat_size = rf_req_out_rd[rd_active_slot].arsize;
	end

	/*=====================================================================*/
	/*  lsb/msb done combinational                                         */
	/*  Includes "pushed this cycle" so DISPATCH can advance in one cycle. */
	/*=====================================================================*/

	always_comb begin
		for (int s = 0; s < 2; s++) begin
			// Write
			wr_lsb_done[s] = wr_req_lsb_pushed[s] ||
				(!apb_req_push_n && apb_tag_fifo_in.is_wr &&
				 apb_tag_fifo_in.slot_idx == logic'(s) &&
				 !apb_tag_fifo_in.is_msb);

			wr_msb_done[s] = wr_req_msb_pushed[s] ||
				(!apb_req_push_n && apb_tag_fifo_in.is_wr &&
				 apb_tag_fifo_in.slot_idx == logic'(s) &&
				 apb_tag_fifo_in.is_msb);

			// Read
			rd_lsb_done[s] = rd_req_lsb_pushed[s] ||
				(!apb_req_push_n && !apb_tag_fifo_in.is_wr &&
				 apb_tag_fifo_in.slot_idx == logic'(s) &&
				 !apb_tag_fifo_in.is_msb);

			rd_msb_done[s] = rd_req_msb_pushed[s] ||
				(!apb_req_push_n && !apb_tag_fifo_in.is_wr &&
				 apb_tag_fifo_in.slot_idx == logic'(s) &&
				 apb_tag_fifo_in.is_msb);
		end
	end

	/*=====================================================================*/
	/* Hazard Check                                                        */
	/*=====================================================================*/
	//
	// rd_addr_hazard: a live write slot (occupied and not yet fully pushed)
	//   covers the same 64-byte cache line as the incoming read request.
	//   Before the read is allocated (rd FSM in IDLE), the FIFO-head address
	//   is compared so the hazard is visible before DISPATCH is entered.
	//   After the read is allocated (rd FSM in DISPATCH or WAIT_RESP), all
	//   allocated read slots are compared against the write slot's address.
	//
	// wr_addr_hazard: symmetric — a live read slot covers the same cache line
	//   as the incoming write request. Used only in the push arbiter and to
	//   block wr_can_accept_next; the write FSM itself is never held in IDLE
	//   by wr_addr_hazard (writes always win the tie).
	//
	// wr_slot_occupied[s]: true when write slot s is already allocated OR is
	//   being allocated THIS cycle. This forward-look is necessary because
	//   rf_slot_free_wr is a registered output — it only goes low the cycle
	//   AFTER alloc_wr fires. Without the forward-look, both FSMs could see
	//   slot_free==1 on the same cycle and enter DISPATCH simultaneously.
	logic wr_slot_occupied [0:1];
	assign wr_slot_occupied[0] = !rf_slot_free_wr[0] || (rf_alloc_wr && (rf_alloc_slot_wr == 1'b0));
	assign wr_slot_occupied[1] = !rf_slot_free_wr[1] || (rf_alloc_wr && (rf_alloc_slot_wr == 1'b1));

	always_comb begin
		rd_addr_hazard = 1'b0;
		for (int s = 0; s < 2; s++) begin
			// A write slot is "live" when it is occupied (or being allocated
			// this very cycle) and has not yet pushed all its beats.
			// wr_all_beats_pushed[s] is 0 at allocation time (cleared in IDLE
			// accept block), so the !wr_all_beats_pushed guard is safe to combine
			// with the forward-declared occupied signal.
			if (wr_slot_occupied[s] && !wr_all_beats_pushed[s]) begin
				// Pre-allocation check (rd FSM still in IDLE):
				// Compare the incoming AR FIFO head address against the write
				// slot's address.  For a slot being allocated this cycle we read
				// wr_req_fifo_out (the head of the AW FIFO, which is the address
				// being latched into the slot right now).  For an already-occupied
				// slot we read rf_req_out_wr[s] (the stored header).
				if (rd_state == ST_IDLE) begin
					logic [ADDR_WIDTH-1:0] wr_slot_addr;
					// If the slot is being freshly allocated this cycle, 
					//its header hasn't been written to rf_req_out_wr yet use the FIFO head.
					wr_slot_addr = (rf_alloc_wr && (rf_alloc_slot_wr == logic'(s)) &&
									rf_slot_free_wr[s]) ? wr_req_fifo_out.awaddr : rf_req_out_wr[s].awaddr;
					if (rd_req_fifo_out.araddr[ADDR_WIDTH-1:6] == wr_slot_addr[ADDR_WIDTH-1:6])
						rd_addr_hazard = 1'b1;
				end
				// Post-allocation check (rd FSM in DISPATCH or WAIT_RESP):
				// The read is already allocated; compare the write slot address
				// against all allocated read slots (active and background).
				if (rd_state != ST_IDLE) begin
					for (int r = 0; r < 2; r++) begin
						if (!rf_slot_free_rd[r] &&
							rf_req_out_rd[r].araddr[ADDR_WIDTH-1:6] == rf_req_out_wr[s].awaddr[ADDR_WIDTH-1:6])
							rd_addr_hazard = 1'b1;
					end
				end
			end
		end
	end

	logic rd_slot_occupied [0:1];
	assign rd_slot_occupied[0] = !rf_slot_free_rd[0] || (rf_alloc_rd && (rf_alloc_slot_rd == 1'b0));
	assign rd_slot_occupied[1] = !rf_slot_free_rd[1] || (rf_alloc_rd && (rf_alloc_slot_rd == 1'b1));

	always_comb begin
		// wr_addr_hazard: a live read slot covers the same cache line as the
		// incoming write request (AW FIFO head when wr_state==ST_IDLE,
		// or the stored header once the write is allocated).
		// Used in two places:
		//   1. wr_can_accept / wr_can_accept_next: blocks write from leaving
		//      IDLE when a conflicting read is already in DISPATCH (FCFS rule).
		//   2. Push arbiter: protects beat atomicity if a read LSB is in-flight.
		wr_addr_hazard = 1'b0;
		for (int s = 0; s < 2; s++) begin
			if (rd_slot_occupied[s] && !rd_all_beats_pushed[s]) begin
				logic [ADDR_WIDTH-1:0] wr_incoming_addr;
				wr_incoming_addr = (wr_state == ST_IDLE) ? wr_req_fifo_out.awaddr : rf_req_out_wr[wr_active_slot].awaddr;
				if (wr_incoming_addr[ADDR_WIDTH-1:6] == rf_req_out_rd[s].araddr[ADDR_WIDTH-1:6])
					wr_addr_hazard = 1'b1;
			end
		end
	end

	/*=====================================================================*/
	/*  Accept conditions                                                   */
	/*=====================================================================*/
	// Write first request: FSM in IDLE, alloc_ptr slot is free, no hazard.
	// Write wins any simultaneous-arrival tie with a read by construction
	// (see hazard policy above).
	assign wr_can_accept = (wr_state == ST_IDLE) &&
						   !wr_req_empty && !wr_data_empty &&
						   rf_slot_free_wr[wr_alloc_ptr] &&
						   !wr_addr_hazard;

	// Write second request: FSM in WAIT_RESP or DISPATCH, active slot last
	// beat already pushed, other slot free, no conflicting read still pushing.
	assign wr_can_accept_next = (wr_state == ST_WAIT_RESP || wr_state == ST_DISPATCH) &&
								!wr_req_empty && !wr_data_empty &&
								wr_all_beats_pushed[wr_active_slot] &&
								rf_slot_free_wr[wr_alloc_ptr] &&
								!wr_addr_hazard;

	// Read first request: FSM in IDLE, alloc_ptr slot is free, no live write
	// covers the same cache line (rd_addr_hazard blocks entry into DISPATCH).
	assign rd_can_accept = (rd_state == ST_IDLE) &&
						   !rd_req_empty && rf_slot_free_rd[rd_alloc_ptr] &&
						   !rd_addr_hazard;

	// Read second request: FSM in WAIT_RESP or DISPATCH, active slot done
	// pushing, other slot free, no conflicting write in flight.
	assign rd_can_accept_next = (rd_state == ST_WAIT_RESP || rd_state == ST_DISPATCH) &&
								!rd_req_empty && rd_all_beats_pushed[rd_active_slot] &&
								rf_slot_free_rd[rd_alloc_ptr] &&
								!rd_addr_hazard;

	/*=====================================================================*/
	/*  Sub-module: write disassembler                                     */
	/*=====================================================================*/

	disassembler u_disassembler_wr (
		.is_wr     (1'b1),
		.beat_addr (dis_wr_beat_addr),
		.beat_size (dis_wr_beat_size),
		.wdata     (dis_wr_wdata),
		.wstrb     (dis_wr_wstrb),
		.lsb_addr  (dis_wr_lsb_addr),
		.lsb_wdata (dis_wr_lsb_wdata),
		.lsb_pstrb (dis_wr_lsb_pstrb),
		.valid_lsb (dis_wr_valid_lsb),
		.msb_addr  (dis_wr_msb_addr),
		.msb_wdata (dis_wr_msb_wdata),
		.msb_pstrb (dis_wr_msb_pstrb),
		.valid_msb (dis_wr_valid_msb)
	);

	/*=====================================================================*/
	/*  Sub-module: read disassembler                                      */
	/*=====================================================================*/

	disassembler u_disassembler_rd (
		.is_wr     (1'b0),
		.beat_addr (dis_rd_beat_addr),
		.beat_size (dis_rd_beat_size),
		.wdata     ('0),
		.wstrb     ('0),
		.lsb_addr  (dis_rd_lsb_addr),
		.lsb_wdata (),
		.lsb_pstrb (),
		.valid_lsb (dis_rd_valid_lsb),
		.msb_addr  (dis_rd_msb_addr),
		.msb_wdata (),
		.msb_pstrb (),
		.valid_msb (dis_rd_valid_msb)
	);

	/*=====================================================================*/
	/*  Sub-module: write apb_req_builder                                  */
	/*=====================================================================*/

	apb_req_builder u_apb_req_builder_wr (
		.is_wr     (1'b1),
		.lsb_addr  (dis_wr_lsb_addr),
		.lsb_wdata (dis_wr_lsb_wdata),
		.lsb_pstrb (dis_wr_lsb_pstrb),
		.msb_addr  (dis_wr_msb_addr),
		.msb_wdata (dis_wr_msb_wdata),
		.msb_pstrb (dis_wr_msb_pstrb),
		.apb_lsb   (builder_wr_apb_lsb),
		.apb_msb   (builder_wr_apb_msb)
	);

	/*=====================================================================*/
	/*  Sub-module: read apb_req_builder                                   */
	/*=====================================================================*/

	apb_req_builder u_apb_req_builder_rd (
		.is_wr     (1'b0),
		.lsb_addr  (dis_rd_lsb_addr),
		.lsb_wdata ('0),
		.lsb_pstrb ('0),
		.msb_addr  (dis_rd_msb_addr),
		.msb_wdata ('0),
		.msb_pstrb ('0),
		.apb_lsb   (builder_rd_apb_lsb),
		.apb_msb   (builder_rd_apb_msb)
	);

	/*=====================================================================*/
	/*  Completing slot selection                                          */
	/*  wr_completing_slot / wr_completing: which write slot has a beat   */
	/*  ready to assemble this cycle, and whether any slot does at all.   */
	/*  Priority: slot 0 over slot 1 on simultaneous completion (rare).   */
	/*  Fallback: wr_active_slot so the mux never points at an            */
	/*  unallocated slot when nothing is completing.                       */
	/*  Derived from registered state only — NOT from apb_tag_fifo_out   */
	/*  or apb_resp_pop_n — so the mux select is stable on the cycle the  */
	/*  final response arrives (the _done flags clock in that same cycle). */
	/*  rd_completing_slot: symmetric for the read pipeline.              */
	/*=====================================================================*/

	logic wr_completing_slot;
	logic wr_completing;
	logic rd_completing_slot;

	always_comb begin : completing_slots
		// Write
		wr_completing_slot = wr_active_slot; // safe fallback
		wr_completing      = 1'b0;
		for (int s = 0; s < 2; s++) begin
		if ((wr_req_lsb_pending[s] || wr_req_msb_pending[s]) &&
			wr_resp_all_done[s] && (logic'(s) == wr_active_slot ? wr_state == ST_WAIT_RESP : wr_all_beats_pushed[s])) begin
				wr_completing_slot = logic'(s);
				wr_completing      = 1'b1;
				break; // slot 0 wins tie (loop goes 0 first)
			end
		end

		// Read
		rd_completing_slot = rd_active_slot; // safe fallback
		for (int s = 0; s < 2; s++) begin
			if ((rd_req_lsb_pending[s] || rd_req_msb_pending[s]) &&
				rd_resp_all_done[s]) begin
				rd_completing_slot = logic'(s);
				break; // slot 0 wins tie
			end
		end
	end

	/*=====================================================================*/
	/*  Sub-module: register file                                          */
	/*=====================================================================*/

	register_file u_register_file (
		.clk                 (clk),
		.rst_n               (rst_n),

		// Write direction
		.alloc_wr            (rf_alloc_wr),
		.alloc_slot_wr       (rf_alloc_slot_wr),
		.alloc_req_wr        (wr_req_fifo_out),
		.req_out_wr          (rf_req_out_wr),
		.resp_wr_en          (wr_completing),
		.resp_slot_wr        (wr_completing_slot),
		.resp_in_wr          (axi_wr_resp_assembled),
		.resp_valid_set_wr   (wr_completing && wr_is_last[wr_completing_slot]),
		.resp_valid_slot_wr  (wr_completing_slot),
		.done_wr             (!wr_resp_push_n),
		.resp_out_wr         (wr_resp_fifo_in),
		.resp_ready_wr       (rf_resp_ready_wr),
		.slot_free_wr        (rf_slot_free_wr),

		// Read direction  reads bypass the reg file, going directly to rd_data_fifo.
		.alloc_rd            (rf_alloc_rd),
		.alloc_slot_rd       (rf_alloc_slot_rd),
		.alloc_req_rd        (rd_req_fifo_out),
		.req_out_rd          (rf_req_out_rd),
		.done_rd             (rf_done_rd),
		.slot_free_rd        (rf_slot_free_rd)
	);

	/*=====================================================================*/
	/*  AXI FIFO pop signals                                               */
	/*=====================================================================*/

	assign wr_req_pop_n  = !(wr_can_accept || wr_can_accept_next);
	assign rd_req_pop_n  = !(rd_can_accept || rd_can_accept_next);

	// wr_data: popped when write pipeline accepts a new request (beat 0)
	// or when returning to DISPATCH for the next beat (data_latched goes low).
	assign wr_data_pop_n = !(wr_can_accept || wr_can_accept_next ||
							(wr_state == ST_DISPATCH && !wr_data_latched[wr_active_slot] 
							&& !wr_data_empty));

	/*=====================================================================*/
	/*  APB request push + tag push arbitration                        	   */
	/*                                                                     */
	/*  Write pipeline wins over read pipeline every cycle, UNLESS the     */
	/*  read pipeline has already pushed its LSB and is waiting to push    */
	/*  its MSB (rd_beat_in_progress). Interrupting a partially-pushed    */
	/*  beat would interleave LSB/MSB pairs from different pipelines in    */
	/*  the APB req FIFO, causing the response router to deadlock because  */
	/*  a response arrives for pipeline A while pipeline B is waiting.     */
	/*                                                                     */
	/*=====================================================================*/

	// True when the read pipeline has pushed LSB but not yet MSB for the
	// current beat the pair must complete atomically in the APB req FIFO.
	// (wr_beat_in_progress is not needed: writes win arbitration by default
	// and are never stalled waiting on the read side's MSB slot.)
	logic rd_beat_in_progress;

	assign rd_beat_in_progress = (rd_state == ST_DISPATCH) &&
								  rd_lsb_done[rd_active_slot] &&
								  !rd_req_msb_pushed[rd_active_slot] &&
								  dis_rd_valid_msb;

	always_comb begin
		apb_req_push_n  = 1'b1;
		apb_req_fifo_in = '0;
		apb_tag_push_n  = 1'b1;
		apb_tag_fifo_in = '0;

		if (!apb_req_full && !apb_tag_full) begin

			// ---- Write pipeline ----
			// Blocked only if the read pipeline has already pushed its LSB this
			// beat and is still waiting to push MSB (must not interleave the pair).
			if (wr_state == ST_DISPATCH && wr_data_latched[wr_active_slot] && !rd_beat_in_progress) begin
				if (!wr_req_lsb_pushed[wr_active_slot] && dis_wr_valid_lsb) begin
					apb_req_push_n  = 1'b0;
					apb_req_fifo_in = builder_wr_apb_lsb;
					apb_tag_push_n  = 1'b0;
					apb_tag_fifo_in = '{is_wr: 1'b1, slot_idx: wr_active_slot, is_msb: 1'b0};

				end else if ((wr_lsb_done[wr_active_slot] || !dis_wr_valid_lsb) &&
							 !wr_req_msb_pushed[wr_active_slot] &&
							 dis_wr_valid_msb) begin
					apb_req_push_n  = 1'b0;
					apb_req_fifo_in = builder_wr_apb_msb;
					apb_tag_push_n  = 1'b0;
					apb_tag_fifo_in = '{is_wr: 1'b1, slot_idx: wr_active_slot, is_msb: 1'b1};
				end
			end

			// ---- Read pipeline (only if write did not push this cycle) ----
			// Allowed when:
			//   1. Write pipeline did not push this cycle (apb_req_push_n still 1).
			//   2. Read FSM is in DISPATCH.
			//   3. No address hazard, OR the current read beat is already in progress
			//      (LSB pushed, MSB pending): once the LSB is in the FIFO the MSB
			//      MUST follow or the tag router will deadlock waiting for the pair.
			if (apb_req_push_n && rd_state == ST_DISPATCH && (!rd_addr_hazard || rd_beat_in_progress)) begin

				if (!rd_req_lsb_pushed[rd_active_slot] && dis_rd_valid_lsb) begin
					apb_req_push_n  = 1'b0;
					apb_req_fifo_in = builder_rd_apb_lsb;
					apb_tag_push_n  = 1'b0;
					apb_tag_fifo_in = '{is_wr: 1'b0, slot_idx: rd_active_slot, is_msb: 1'b0};

				end else if ((rd_lsb_done[rd_active_slot] || !dis_rd_valid_lsb) &&
							 !rd_req_msb_pushed[rd_active_slot] && dis_rd_valid_msb) begin
					apb_req_push_n  = 1'b0;
					apb_req_fifo_in = builder_rd_apb_msb;
					apb_tag_push_n  = 1'b0;
					apb_tag_fifo_in = '{is_wr: 1'b0, slot_idx: rd_active_slot, is_msb: 1'b1};
				end
			end
		end
	end

	/*=====================================================================*/
	/*  APB response pop                                                   */
	/*  Pop both FIFOs together whenever both are non-empty. Because the  */
	/*  tag and data FIFOs are pushed in lockstep they are always in sync. */
	/*=====================================================================*/

	assign apb_resp_pop_n = apb_resp_empty || apb_tag_empty;
	assign apb_tag_pop_n  = apb_resp_empty || apb_tag_empty;

	/*=====================================================================*/
	/*  Register file drive  allocation signals                           */
	/*=====================================================================*/

	assign rf_alloc_wr      = wr_can_accept || wr_can_accept_next;
	assign rf_alloc_slot_wr = wr_alloc_ptr;
	assign rf_alloc_rd      = rd_can_accept || rd_can_accept_next;
	assign rf_alloc_slot_rd = rd_alloc_ptr;

	/*=====================================================================*/
	/*  Sub-module: assembler + resp builder         	                   */
	/*=====================================================================*/

	assembler u_assembler (
		.valid_lsb   (wr_completing ? wr_req_lsb_pending[wr_completing_slot] : rd_req_lsb_pending[rd_completing_slot]),
		.valid_msb   (wr_completing ? wr_req_msb_pending[wr_completing_slot] : rd_req_msb_pending[rd_completing_slot]),
		.lsb_prdata  (wr_completing ? wr_resp_lsb_data  [wr_completing_slot] : rd_resp_lsb_data  [rd_completing_slot]),
		.lsb_pslverr (wr_completing ? wr_resp_lsb_err   [wr_completing_slot] : rd_resp_lsb_err   [rd_completing_slot]),
		.msb_prdata  (wr_completing ? wr_resp_msb_data  [wr_completing_slot] : rd_resp_msb_data  [rd_completing_slot]),
		.msb_pslverr (wr_completing ? wr_resp_msb_err   [wr_completing_slot] : rd_resp_msb_err   [rd_completing_slot]),
		.rdata       (asm_rdata),
		.resp        (asm_resp)
	);

	axi_resp_builder u_axi_resp_builder (
		.transaction_id (wr_completing ? rf_req_out_wr[wr_completing_slot].awid : rf_req_out_rd[rd_completing_slot].arid),
		.is_last        (wr_completing ? wr_is_last[wr_completing_slot] : rd_is_last[rd_completing_slot]),
		.rdata          (asm_rdata),
		.resp           (wr_completing ? (wr_burst_err[wr_completing_slot] ? 2'b10 : asm_resp) : asm_resp),
		.rd_data        (rd_data_fifo_in),
		.wr_resp        (axi_wr_resp_assembled)
	);

	/*=====================================================================*/
	/*  AXI response drain to slave                                        */
	/*=====================================================================*/

	// Write responses: held in register file until burst complete, then drained.
	// wr_resp_fifo_in is driven directly from the register file's resp_out_wr port.
	assign wr_resp_push_n = !(rf_resp_ready_wr && !wr_resp_full);

	// Read responses: each beat pushed directly to rd_data_fifo as it completes.
	// rd_data_fifo_in is driven directly from u_axi_resp_builder's rd_data port.
	assign rd_data_push_n = !(rd_state == ST_WAIT_RESP && rd_resp_all_done[rd_completing_slot] && !rd_data_full);

	// Free the read register-file slot on the last beat push.
	assign rf_done_rd = (rd_state == ST_WAIT_RESP) && !rd_data_push_n && rd_is_last[rd_completing_slot];

	/*=====================================================================*/
	/*  Write pipeline sequential FSM                                      */
	/*=====================================================================*/

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wr_state       <= ST_IDLE;
			wr_active_slot <= 1'b0;
			wr_alloc_ptr   <= 1'b0;

			for (int s = 0; s < 2; s++) begin
				wr_beat_index      [s] <= '0;
				wr_burst_err       [s] <= 1'b0;
				wr_req_lsb_pushed  [s] <= 1'b0;
				wr_req_msb_pushed  [s] <= 1'b0;
				wr_req_lsb_pending [s] <= 1'b0;
				wr_req_msb_pending [s] <= 1'b0;
				wr_resp_lsb_done   [s] <= 1'b0;
				wr_resp_msb_done   [s] <= 1'b0;
				wr_all_beats_pushed[s] <= 1'b0;
				wr_resp_lsb_data  [s] <= '0;
				wr_resp_lsb_err   [s] <= 1'b0;
				wr_resp_msb_data  [s] <= '0;
				wr_resp_msb_err   [s] <= 1'b0;
				wr_data_latched   [s] <= 1'b0;
				wr_data_q         [s] <= '0;
			end
		end else begin

			case (wr_state)

				/*-----------------------------------------------------------*/
				ST_IDLE: begin
				/*-----------------------------------------------------------*/
					if (wr_can_accept) begin
						// Latch beat-0 data (popped simultaneously with req)
						wr_data_q      [wr_alloc_ptr] <= wr_data_fifo_out;
						wr_data_latched[wr_alloc_ptr] <= 1'b1;
						// Reset beat state for the new slot
						wr_beat_index      [wr_alloc_ptr] <= '0;
						wr_burst_err       [wr_alloc_ptr] <= 1'b0;
						wr_req_lsb_pushed  [wr_alloc_ptr] <= 1'b0;
						wr_req_msb_pushed  [wr_alloc_ptr] <= 1'b0;
						wr_req_lsb_pending [wr_alloc_ptr] <= 1'b0;
						wr_req_msb_pending [wr_alloc_ptr] <= 1'b0;
						wr_resp_lsb_done   [wr_alloc_ptr] <= 1'b0;
						wr_resp_msb_done   [wr_alloc_ptr] <= 1'b0;
						wr_all_beats_pushed[wr_alloc_ptr] <= 1'b0;
						// Advance pointers
						wr_active_slot <= wr_alloc_ptr;
						wr_alloc_ptr   <= ~wr_alloc_ptr;
						wr_state       <= ST_DISPATCH;
					end
				end

				/*-----------------------------------------------------------*/
				ST_DISPATCH: begin
				/*-----------------------------------------------------------*/
					// Latch wr_data for subsequent beats (pop fired last cycle)
					if (!wr_data_pop_n && wr_data_latched[wr_active_slot] == 1'b0) begin
						wr_data_q      [wr_active_slot] <= wr_data_fifo_out;
						wr_data_latched[wr_active_slot] <= 1'b1;
					end

					// Track which halves have been pushed
					if (!apb_req_push_n && apb_tag_fifo_in.is_wr && apb_tag_fifo_in.slot_idx == wr_active_slot) begin
						if (!apb_tag_fifo_in.is_msb) begin
							wr_req_lsb_pushed [wr_active_slot] <= 1'b1;
							wr_req_lsb_pending[wr_active_slot] <= 1'b1;
						end else begin
							wr_req_msb_pushed [wr_active_slot] <= 1'b1;
							wr_req_msb_pending[wr_active_slot] <= 1'b1;
						end
					end

					// Both halves dispatched (or not needed) -> move to WAIT_RESP.
					// Each half is "done" if it was pushed OR it was not valid for
					// this beat (narrow transaction with only one active half).
					// pending is always 0 at beat start (cleared in WAIT_RESP beat-done),
					// so no explicit clear is needed for invalid halves.
					if (wr_data_latched[wr_active_slot] &&
						(wr_lsb_done[wr_active_slot] || !dis_wr_valid_lsb) &&
						(wr_msb_done[wr_active_slot] || !dis_wr_valid_msb)) begin

						// If this beat's push is also the last beat, mark all done
						if (wr_is_last[wr_active_slot])
							wr_all_beats_pushed[wr_active_slot] <= 1'b1;

						wr_state <= ST_WAIT_RESP;
					end
				end

				/*-----------------------------------------------------------*/
				ST_WAIT_RESP: begin
				/*-----------------------------------------------------------*/
					// Capture APB responses into whichever slot the tag identifies —
					// NOT necessarily wr_active_slot. With pipelined slots, slot 0
					// responses can arrive while slot 1 is the active dispatching slot.
					if (!apb_resp_pop_n && apb_tag_fifo_out.is_wr) begin
						if (!apb_tag_fifo_out.is_msb) begin
							wr_resp_lsb_data[apb_tag_fifo_out.slot_idx] <= apb_resp_fifo_out.prdata;
							wr_resp_lsb_err [apb_tag_fifo_out.slot_idx] <= apb_resp_fifo_out.pslverr;
							wr_resp_lsb_done[apb_tag_fifo_out.slot_idx] <= 1'b1;
						end else begin
							wr_resp_msb_data[apb_tag_fifo_out.slot_idx] <= apb_resp_fifo_out.prdata;
							wr_resp_msb_err [apb_tag_fifo_out.slot_idx] <= apb_resp_fifo_out.pslverr;
							wr_resp_msb_done[apb_tag_fifo_out.slot_idx] <= 1'b1;
						end
					end

					// Beat-done check: iterate over both slots independently.
					// Either slot may complete a beat while the other is still waiting.
					// Per-slot error is computed inline from the raw response arrays
					// because asm_wr_bresp is muxed to wr_completing_slot and would
					// give the wrong value for the background slot.
					for (int s = 0; s < 2; s++) begin
						if ((wr_req_lsb_pending[s] || wr_req_msb_pending[s]) && wr_resp_all_done[s]) begin

							wr_burst_err[s] <= wr_burst_err[s] | wr_beat_err[s];

							wr_resp_lsb_done  [s] <= 1'b0;
							wr_resp_msb_done  [s] <= 1'b0;
							wr_req_lsb_pushed [s] <= 1'b0;
							wr_req_msb_pushed [s] <= 1'b0;
							wr_req_lsb_pending[s] <= 1'b0;
							wr_req_msb_pending[s] <= 1'b0;

							if (wr_is_last[s]) begin
								wr_beat_index  [s] <= '0;
								wr_burst_err   [s] <= 1'b0;
								wr_data_latched[s] <= 1'b0;
								// Only transition FSM state when the active slot finishes
								if (logic'(s) == wr_active_slot)
									wr_state <= ST_IDLE;
							end else begin
								wr_beat_index  [s] <= wr_beat_index[s] + 1'b1;
								wr_data_latched[s] <= 1'b0;
								if (logic'(s) == wr_active_slot)
									wr_state <= ST_DISPATCH;
							end
						end
					end

					// Accept a new write request into the other slot while
					// this slot is still waiting for APB responses, as long
					// as wr_can_accept_next is true (all_beats_pushed for active slot).
					if (wr_can_accept_next) begin
						wr_data_q          [wr_alloc_ptr] <= wr_data_fifo_out;
						wr_data_latched    [wr_alloc_ptr] <= 1'b1;
						wr_beat_index      [wr_alloc_ptr] <= '0;
						wr_burst_err       [wr_alloc_ptr] <= 1'b0;
						wr_req_lsb_pushed  [wr_alloc_ptr] <= 1'b0;
						wr_req_msb_pushed  [wr_alloc_ptr] <= 1'b0;
						wr_req_lsb_pending [wr_alloc_ptr] <= 1'b0;
						wr_req_msb_pending [wr_alloc_ptr] <= 1'b0;
						wr_resp_lsb_done   [wr_alloc_ptr] <= 1'b0;
						wr_resp_msb_done   [wr_alloc_ptr] <= 1'b0;
						wr_all_beats_pushed[wr_alloc_ptr] <= 1'b0;
						wr_alloc_ptr   <= ~wr_alloc_ptr;
						wr_active_slot <= wr_alloc_ptr;
						wr_state       <= ST_DISPATCH;
					end
				end

				default: wr_state <= ST_IDLE;
			endcase
		end
	end

	/*=====================================================================*/
	/*  Read pipeline sequential FSM  (symmetric to write)                */
	/*=====================================================================*/

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rd_state       <= ST_IDLE;
			rd_active_slot <= 1'b0;
			rd_alloc_ptr   <= 1'b0;

			for (int s = 0; s < 2; s++) begin
				rd_beat_index      [s] <= '0;
				rd_req_lsb_pushed  [s] <= 1'b0;
				rd_req_msb_pushed  [s] <= 1'b0;
				rd_req_lsb_pending [s] <= 1'b0;
				rd_req_msb_pending [s] <= 1'b0;
				rd_resp_lsb_done   [s] <= 1'b0;
				rd_resp_msb_done   [s] <= 1'b0;
				rd_all_beats_pushed[s] <= 1'b0;
				rd_resp_lsb_data  [s] <= '0;
				rd_resp_lsb_err   [s] <= 1'b0;
				rd_resp_msb_data  [s] <= '0;
				rd_resp_msb_err   [s] <= 1'b0;
			end
		end else begin

			case (rd_state)

				/*-----------------------------------------------------------*/
				ST_IDLE: begin
				/*-----------------------------------------------------------*/
					if (rd_can_accept) begin
						rd_beat_index      [rd_alloc_ptr] <= '0;
						rd_req_lsb_pushed  [rd_alloc_ptr] <= 1'b0;
						rd_req_msb_pushed  [rd_alloc_ptr] <= 1'b0;
						rd_req_lsb_pending [rd_alloc_ptr] <= 1'b0;
						rd_req_msb_pending [rd_alloc_ptr] <= 1'b0;
						rd_resp_lsb_done   [rd_alloc_ptr] <= 1'b0;
						rd_resp_msb_done   [rd_alloc_ptr] <= 1'b0;
						rd_all_beats_pushed[rd_alloc_ptr] <= 1'b0;
						rd_active_slot <= rd_alloc_ptr;
						rd_alloc_ptr   <= ~rd_alloc_ptr;
						rd_state       <= ST_DISPATCH;
					end
				end

				/*-----------------------------------------------------------*/
				ST_DISPATCH: begin
				/*-----------------------------------------------------------*/
					if (!apb_req_push_n &&
						!apb_tag_fifo_in.is_wr &&
						apb_tag_fifo_in.slot_idx == rd_active_slot) begin

						if (!apb_tag_fifo_in.is_msb) begin
							rd_req_lsb_pushed [rd_active_slot] <= 1'b1;
							rd_req_lsb_pending[rd_active_slot] <= 1'b1;
						end else begin
							rd_req_msb_pushed [rd_active_slot] <= 1'b1;
							rd_req_msb_pending[rd_active_slot] <= 1'b1;
						end
					end

					// Symmetric exit: each half done if pushed OR not valid for beat.
					// pending is always 0 at beat start (cleared in WAIT_RESP beat-done),
					// so no explicit clear is needed for invalid halves.
					if ((rd_lsb_done[rd_active_slot] || !dis_rd_valid_lsb) &&
						(rd_msb_done[rd_active_slot] || !dis_rd_valid_msb)) begin

						if (rd_is_last[rd_active_slot])
							rd_all_beats_pushed[rd_active_slot] <= 1'b1;

						rd_state <= ST_WAIT_RESP;
					end
				end

				/*-----------------------------------------------------------*/
				ST_WAIT_RESP: begin
				/*-----------------------------------------------------------*/
					// Capture APB responses into the slot the tag identifies.
					if (!apb_resp_pop_n && !apb_tag_fifo_out.is_wr) begin
						if (!apb_tag_fifo_out.is_msb) begin
							rd_resp_lsb_data[apb_tag_fifo_out.slot_idx] <= apb_resp_fifo_out.prdata;
							rd_resp_lsb_err [apb_tag_fifo_out.slot_idx] <= apb_resp_fifo_out.pslverr;
							rd_resp_lsb_done[apb_tag_fifo_out.slot_idx] <= 1'b1;
						end else begin
							rd_resp_msb_data[apb_tag_fifo_out.slot_idx] <= apb_resp_fifo_out.prdata;
							rd_resp_msb_err [apb_tag_fifo_out.slot_idx] <= apb_resp_fifo_out.pslverr;
							rd_resp_msb_done[apb_tag_fifo_out.slot_idx] <= 1'b1;
						end
					end

					// Beat-done: check both slots independently.
					// rd_data_fifo_in is muxed to the completing slot; the FSM state
					// transition only fires for the active slot; the background slot's
					// beat state is cleared without a state change.
					for (int s = 0; s < 2; s++) begin
						if ((rd_req_lsb_pending[s] || rd_req_msb_pending[s]) &&
							rd_resp_all_done[s] &&
							!rd_data_full) begin

							rd_resp_lsb_done  [s] <= 1'b0;
							rd_resp_msb_done  [s] <= 1'b0;
							rd_req_lsb_pushed [s] <= 1'b0;
							rd_req_msb_pushed [s] <= 1'b0;
							rd_req_lsb_pending[s] <= 1'b0;
							rd_req_msb_pending[s] <= 1'b0;

							if (rd_is_last[s]) begin
								rd_beat_index[s] <= '0;
								if (logic'(s) == rd_active_slot)
									rd_state <= ST_IDLE;
							end else begin
								rd_beat_index[s] <= rd_beat_index[s] + 1'b1;
								if (logic'(s) == rd_active_slot)
									rd_state <= ST_DISPATCH;
							end
						end
					end

					// Accept next read into the other slot while this one is in WAIT_RESP.
					if (rd_can_accept_next) begin
						rd_beat_index      [rd_alloc_ptr] <= '0;
						rd_req_lsb_pushed  [rd_alloc_ptr] <= 1'b0;
						rd_req_msb_pushed  [rd_alloc_ptr] <= 1'b0;
						rd_req_lsb_pending [rd_alloc_ptr] <= 1'b0;
						rd_req_msb_pending [rd_alloc_ptr] <= 1'b0;
						rd_resp_lsb_done   [rd_alloc_ptr] <= 1'b0;
						rd_resp_msb_done   [rd_alloc_ptr] <= 1'b0;
						rd_all_beats_pushed[rd_alloc_ptr] <= 1'b0;
						rd_alloc_ptr   <= ~rd_alloc_ptr;
						rd_active_slot <= rd_alloc_ptr;
						rd_state       <= ST_DISPATCH;
					end
				end

				default: rd_state <= ST_IDLE;
			endcase
		end
	end

endmodule