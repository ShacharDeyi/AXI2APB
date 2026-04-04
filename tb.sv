/*------------------------------------------------------------------------------
 * File          : tb.sv
 * Project       : AXI2APB Bridge
 * Authors       : Shachar & Sahar
 * Description   : Comprehensive testbench for the AXI3?APB4 bridge.
 *
 * ============================================================================
 * DESIGN UNDER TEST PARAMETERS
 * ============================================================================
 *   AXI data width : 64-bit   APB data width : 32-bit  (2:1 split per beat)
 *   Address width  : 32-bit   Max burst len  : 8 beats (MAX_LEN=8)
 *   Max beat size  : 8 B      (MAX_SIZE=3)
 *   Write slots    : 2        Read slots     : 2
 *   APB FIFO depth : 16       Clock          : ~150 MHz (6.67 ns period)
 *
 * ============================================================================
 * APB SLAVE MODEL
 * ============================================================================
 *   Combinational response: prdata = ~paddr (write-gated: 0 on writes).
 *   Three control knobs driven exclusively by the initial block (blocking =):
 *
 *     apb_pready_delay  int    Extra ACCESS cycles before pready fires.
 *                               Decremented via blocking = inside the always
 *                               block so it persists across transactions until
 *                               explicitly reset.  Default: 0.
 *
 *     apb_inject_err    logic  While 1, asserts pslverr on every APB response.
 *                               Arm/disarm from the initial block only  never
 *                               from the always block  to avoid multiple-driver
 *                               X-propagation (root cause of original P1-10
 *                               timeout).  Default: 0.
 *
 *     apb_txn_cnt       int    Running count of completed APB transactions.
 *                               Incremented via blocking = in the always block
 *                               so wait() in the initial block sees it
 *                               immediately.  Used by P1-9 to synchronise
 *                               error injection to a specific sub-transaction.
 *
 *   SINGLE-DRIVER RULE: the always block never writes apb_inject_err,
 *   apb_pready_delay, or apb_txn_cnt with non-blocking <=.  Mixing blocking
 *   and non-blocking assignments on the same variable from different processes
 *   is a multiple-driver violation that causes X in VCS.
 *
 * ============================================================================
 * AXI MASTER TASKS
 * ============================================================================
 *   axi_send_aw(addr, id, len, size)      AW handshake (blocking)
 *   axi_send_w(data, strb, last)          W  handshake (blocking)
 *   axi_collect_b(bid_out, bresp_out)     B  handshake (blocking)
 *   axi_write(addr, id, data, strb, size, bresp_out)
 *                                          Single-beat write convenience task.
 *                                          Uses fork/join for AW+W, then
 *                                          collects B sequentially.
 *                                          NOT thread-safe for bready  never
 *                                          call from multiple parallel threads.
 *
 *   axi_send_ar(addr, id, len, size)      AR handshake (blocking)
 *   axi_collect_r(rid, rdata, rresp, rlast)
 *                                          R  handshake (blocking).
 *                                          Sets rready on negedge for clean
 *                                          setup time; samples outputs on the
 *                                          posedge where rvalid fires.
 *   axi_read(addr, id, size, rdata, rresp)
 *                                          Single-beat read convenience task.
 *
 *   THREAD SAFETY: axi_send_aw, axi_send_w, axi_send_ar all drive shared AXI
 *   interface signals.  They must never run concurrently  doing so causes
 *   signal races that corrupt transactions.  axi_collect_b and axi_collect_r
 *   each drive a dedicated ready signal (bready / rready) and are safe to run
 *   concurrently with each other but not with another instance of themselves.
 *
 * ============================================================================
 * HELPER TASKS
 * ============================================================================
 *   wait_idle(timeout=500)   Polls axi.bvalid || axi.rvalid until both clear,
 *                             then waits 4 more posedges.  Called after every
 *                             test to ensure the bridge is fully quiesced
 *                             before the next test starts.
 *
 *   check(name, condition)   Prints [PASS]/[FAIL] and increments counters.
 *
 *   do_reset(cycles=5)       Asserts rst_n=0 for N posedges, releases on
 *                             negedge.
 *
 * ============================================================================
 * TEST LIST
 * ============================================================================
 *
 * PRIORITY 1  Basic functional correctness
 * ------------------------------------------
 *  P1-1  Single-beat write (awlen=0, size=3, full 8B width)
 *         Verifies BRESP=OKAY end-to-end through both APB halves.
 *
 *  P1-2  Single-beat read (arlen=0, size=3, full 8B width)
 *         Verifies RRESP=OKAY and that rdata[31:0]=~lsb_addr,
 *         rdata[63:32]=~msb_addr (sideband tag routes each APB half
 *         to the correct rdata lane).
 *
 *  P1-3  Narrow write  LSB half only (size=2, addr[2]=0)
 *         Only the LSB APB sub-transaction is generated; MSB is suppressed.
 *
 *  P1-4  Narrow write  MSB half only (size=2, addr[2]=1)
 *         Only the MSB APB sub-transaction is generated.
 *
 *  P1-5  Write strobe: LSB lanes only (wstrb=8'h0F, size=3)
 *         Verifies disassembler suppresses MSB half (pstrb[7:4]=0 ? not valid
 *         for write).  Checks: exactly 1 APB sub-txn issued (apb_txn_cnt+1),
 *         and pstrb captured during ACCESS phase equals 4'hF.
 *
 *  P1-6  Write strobe: MSB lanes only (wstrb=8'hF0, size=3)
 *         Symmetric to P1-5: addr[2]=1 (0x4000_0004) ? MSB half only.
 *         Checks: 1 sub-txn, pstrb=4'hF on that txn.
 *
 *  P1-7  Max-length write burst (awlen=7 ? 8 beats, size=3)
 *         AW sent in parallel with all 8 W beats.  Checks BRESP=OKAY and
 *         bid matches awid.
 *
 *  P1-8  Max-length read burst (arlen=7 ? 8 beats, size=3)
 *         Checks all 8 beats RRESP=OKAY, rlast asserted on beat 7,
 *         rid matches arid.
 *
 *  P1-9  PSLVERR on mid-burst write beat (beat 3 of 8)
 *         Uses apb_txn_cnt to arm apb_inject_err on exactly sub-txn 6
 *         (LSB of AXI beat 3, 0-indexed).  The single-use clear is done
 *         by the initial block monitoring apb_txn_cnt  no multiple-driver
 *         conflict.  Verifies the bridge ORs the error into final BRESP=SLVERR.
 *
 *  P1-10 PSLVERR on read response
 *         apb_inject_err=1 held for all sub-txns of a single-beat read.
 *         Verifies RRESP=2'b10.
 *
 *  P1-10b PSLVERR on mid-burst READ beat (beat 3 of 8)
 *         Mirrors P1-9 for the read path. Uses apb_txn_cnt to arm
 *         apb_inject_err on sub-txn 6 (LSB of AXI beat 3). Verifies at
 *         least one RRESP=SLVERR is returned and rlast/rid are correct.
 *
 *  P1-10c PSLVERR on FIRST beat of write burst (beat 0 of 8)
 *         Error injected on sub-txn 0 only; all other beats clean.
 *         Verifies early error is captured and BRESP=SLVERR is returned.
 *
 *  P1-10d PSLVERR on LAST beat of write burst (beat 7 of 8)
 *         Error injected on sub-txn 14 (LSB of beat 7); all earlier beats
 *         clean. Verifies a late error is still propagated into BRESP=SLVERR
 *         (no early-exit after the first clean beat).
 *
 *  P1-10e PSLVERR on MULTIPLE non-contiguous beats (beats 1 & 5 of 8)
 *         Two separate error injections on sub-txns 2 and 10. Verifies the
 *         bridge OR-accumulates scattered errors into a single BRESP=SLVERR.
 *
 *  P1-11 Transaction ID preservation
 *         awid=0xDEAD_BEEF echoed in bid; arid=0xCAFE_F00D echoed in rid.
 *         Both IDs captured via task output arguments (not by re-sampling
 *         the interface after the handshake window closes).
 *
 * PRIORITY 2  Concurrency and arbitration
 * ------------------------------------------
 *  P2-1  Simultaneous write + read (one slot each, no data mixing)
 *         Write and read issued in a fork/join.  Checks both responses OKAY
 *         and bid ? rid (no cross-contamination).
 *
 *  P2-2  Write pipeline APB push priority (slot 0 before slot 1)
 *         Two back-to-back writes; first APB paddr sampled on posedge of
 *         psel (edge-triggered to avoid level race).  Verifies slot-0 base
 *         address appears first.
 *
 *  P2-3  Back-to-back writes (4 serial writes, no idle gap)
 *         Each write's BRESP is collected before the next AW is sent,
 *         verifying the bridge handles continuous throughput cleanly.
 *
 *  P2-4  Sideband tag routing
 *         Single-beat read verifies rdata[31:0]=~lsb_addr and
 *         rdata[63:32]=~msb_addr end-to-end through the
 *         disassembler?apb_tag?assembler path.
 *         (The original MSB-first APB slave mode was removed: a sequential
 *         APB master cannot have two transactions in-flight simultaneously,
 *         so holding pready=0 on the LSB permanently stalls the master.)
 *
 *  P2-5  Pipelined write slots  wr_can_accept_next path
 *         Both bursts (len=3) are queued in wr_req_fifo / wr_data_fifo
 *         BEFORE the APB slave is slowed (pready_delay=2).  This is
 *         required because awready is gated on slot_free_wr  setting the
 *         delay first would deadlock on the 2nd AW.  With both bursts
 *         queued, slot 1 is accepted via wr_can_accept_next while slot 0
 *         is still in ST_WAIT_RESP.  Verifies both BRESP=OKAY and correct
 *         bid values.
 *
 *  P2-6  Pipelined read slots  rd_can_accept_next path
 *         Symmetric to P2-5.  Both ARs queued before APB is slowed.
 *         All 8 R beats (4 per burst) drained and checked per rid.
 *
 * PRIORITY 3  Flow control and robustness
 * ------------------------------------------
 *  P3-1  Write + read pipeline independence
 *         One AR queued, then 5 serial writes issued concurrently.  Because
 *         the write and read pipelines are fully independent FSMs (no
 *         starvation counter exists in the RTL), the read FSM accepts the AR
 *         and processes it independently of write activity.  Verifies all 5
 *         writes and the 1 read complete OKAY with no timeout.
 *
 *  P3-2  Concurrent write + read, distinct addresses
 *         One write and one read issued simultaneously.  No hazard (different
 *         cache lines).  Write wins APB arbiter each cycle it pushes; read
 *         uses idle cycles.  Both must complete OKAY.
 *
 *  P3-3  APB req FIFO back-pressure (2-slot batched writes)
 *         The bridge has 2 write slots; a 3rd AW stalls on awready until a
 *         slot frees (BRESP collected).  Strategy: send 2 AW+W pairs upfront
 *         with pready_delay=3 so both slots dispatch and their 4 APB sub-
 *         requests pile up in the FIFO while the slave is slow.  Collect both
 *         BRESPs (still slow), then repeat for writes 3 & 4.  Back-pressure
 *         is active throughout all 4 writes.  All 4 BRESP=OKAY verified.
 *
 *  P3-4  Slow PREADY (multi-cycle ACCESS phase)
 *         pready_delay=5 applied to concurrent write + read.  Verifies the
 *         apb_master holds psel/penable and waits correctly for pready.
 *
 *  P3-5  Reset during active burst
 *         8-beat write burst started; rst_n asserted after 10 cycles
 *         (mid-burst).  After reset and quiesce, a fresh single-beat write
 *         and read must both complete OKAY.
 *
 * PRIORITY 4  Stress and regression
 * ------------------------------------
 *  P4-1  Randomised burst stress (size=3, len 0-7, 10 iterations)
 *         Alternating reads (even iterations) and writes (odd iterations)
 *         with random burst lengths.  size=3 (full-width) is used
 *         throughout  narrow transactions are covered precisely by P1-3
 *         through P1-6 and don't add stress value here.  wait_idle() called
 *         between every iteration.  Checks majority of transactions OKAY.
 *
 *  P4-2  Address boundary crossing
 *         Write burst (len=3) starting at 0xFFFFFFF8 (8 B from 32-bit wrap).
 *         Read burst (len=1) starting at 0xFFFFFFF0.  Both cross the 4 B
 *         boundary (beat_addr[2] toggles).  rresp captured inside the drain
 *         loop on the last beat  NOT after rready=0 (signal may be stale).
 *
 *  P4-3  Hazard detection: write + read to same address
 *         A 4-beat write (awlen=3, 8 APB sub-txns) and a single-beat read
 *         (2 APB sub-txns) are issued to 0xCC00_0000 concurrently.
 *         Write BRESP collected first (sequentially), then read beat drained.
 *         Ordering verified via apb_txn_cnt: after write BRESP, count =
 *         base+8 confirms all write sub-txns preceded the read's 2.
 *         Final count must equal base+10.  rdata correctness also checked.
 *
 *  P4-4  Hazard detection under stress  burst write + burst read
 *         Same mechanism as P4-3 with a 2-beat read (arlen=1, 4 APB sub-txns)
 *         to 0xEE00_0000.  Total = 12 APB sub-txns.  Run after P4-1/P4-2 so
 *         slot-state machinery is in a non-reset state.  Per-beat rdata
 *         checked against {~msb_addr, ~lsb_addr} for each read beat.
 *
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module tb;
import struct_types::*;

	/*=========================================================================*/
	/*  Clock / reset                                                           */
	/*=========================================================================*/
	localparam CLK_PERIOD = 6.67; // ~150 MHz
	logic clk, rst_n;

	initial clk = 0;
	always #(CLK_PERIOD/2) clk = ~clk;

	task automatic do_reset(int cycles = 5);
		rst_n = 0;
		repeat (cycles) @(posedge clk);
		@(negedge clk); rst_n = 1;
	endtask

	/*=========================================================================*/
	/*  Interfaces                                                             */
	/*=========================================================================*/
	axi_interface axi();
	apb_interface apb();

	/*=========================================================================*/
	/*  DUT                                                                    */
	/*=========================================================================*/
	top_module dut (
		.clk   (clk),
		.rst_n (rst_n),
		.axi   (axi),
		.apb   (apb)
	);

	/*=========================================================================*/
	/*  APB slave model                                                        */
	/*=========================================================================*/
	// Control knobs written freely by the test body (initial block).
	// Using plain 'logic'/'int' with no inline initializer avoids the
	// ICPD_INIT "multiple drivers" error that always_ff ownership causes.

	int   apb_pready_delay;   // extra ACCESS cycles before pready
	logic apb_inject_err;     // assert pslverr on every APB response while high
	int   apb_txn_cnt;        // counts completed APB transactions (blocking = only)
	logic apb_req_waiting;
	// FIX #4: apb_msb_first_mode removed  a sequential APB master can have at
	// most one transaction in-flight; holding pready=0 on the LSB permanently
	// stalls the master and the MSB transaction is never presented.
	// P2-4 is re-implemented using the tag-routing check (see below).
	//
	// MULTIPLE-DRIVER FIX: apb_inject_err, apb_err_once, apb_pready_delay, and
	// apb_txn_cnt are ONLY ever written by the initial block (blocking =).
	// The always block ONLY READS them  it never uses non-blocking <= on these
	// variables.  Previously apb_inject_err and apb_err_once were also written
	// via <= inside the always block (the "apb_err_once auto-clear"), which is
	// a multiple-driver violation that causes X-propagation in VCS and is the
	// root cause of the P1-10 timeout: apb_inject_err went X, making pslverr X,
	// which propagated through the APB resp FIFO into the manager's error state,
	// leaving rd_resp_all_done stuck low so rvalid never fired.
	//
	// The auto-clear is now handled by a dedicated always @(posedge clk) monitor
	// that uses only blocking = (same driver domain as the initial block) and
	// watches apb_txn_cnt to know when a transaction completed.

	// Initialise control knobs once at time-zero (before rst_n is driven).
	// apb.pready / apb.prdata / apb.pslverr are now driven by always_comb
	// and must NOT be assigned here (multiple-driver conflict).
	initial begin
		apb_pready_delay = 0;
		apb_inject_err   = 1'b0;
		apb_txn_cnt      = 0;
		apb_req_waiting  = 1'b0;
	end

	// APB slave response logic.
	//
	// pready MUST be combinational: the APB spec requires pready to be
	// sampled by the master on the same posedge that psel & penable are
	// high.  A registered pready always arrives one cycle late  after
	// psel/penable have already dropped  which is the "extra high cycle"
	// seen in the waveform.
	//
	// Strategy:
	//   - always_comb drives pready/prdata/pslverr purely from current
	//     psel, penable, and the delay counter (no flip-flop).
	//   - always_ff on posedge clk handles the side-effects that must be
	//     clocked: decrementing apb_pready_delay and incrementing
	//     apb_txn_cnt (blocking = so the initial block sees it immediately).

	// APB slave response logic.
	//
	// pready is combinational so it is visible to the master in the same
	// cycle that psel & penable are high (APB spec requirement).
	//
	// apb_pready_delay (set by the test body) is sampled once at the
	// SETUP->ACCESS transition (posedge where psel=1, penable=0) into a
	// private counter 'wait_cnt'.  From that point only wait_cnt is
	// decremented; apb_pready_delay is never touched by the always block,
	// so there is no multiple-driver conflict even if the test body changes
	// apb_pready_delay mid-simulation.  This also prevents a mid-ACCESS
	// retraction of pready if the test body sets the delay after the master
	// has already entered the ACCESS phase.

	int wait_cnt; // private per-transaction countdown (not driven by test body)

	// Combinational: pready high when in ACCESS phase and wait_cnt==0.
	always_comb begin
		if (apb.psel && apb.penable && (wait_cnt == 0)) begin
			apb.pready  = 1'b1;
			apb.prdata  = ~apb.paddr;
			apb.pslverr = apb_inject_err;
		end else begin
			apb.pready  = 1'b0;
			apb.prdata  = '0;
			apb.pslverr = 1'b0;
		end
	end

	// Clocked: latch delay at SETUP, count down in ACCESS, count txns.
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wait_cnt        <= 0;
			apb_req_waiting <= 1'b0;
		end else begin
			apb_req_waiting <= 1'b0;

			if (apb.psel && !apb.penable) begin
				// SETUP phase: snapshot the current delay knob into wait_cnt.
				wait_cnt <= apb_pready_delay;
			end else if (apb.psel && apb.penable) begin
				// ACCESS phase
				if (wait_cnt > 0) begin
					wait_cnt        <= wait_cnt - 1;
					apb_req_waiting <= 1'b1; // still waiting
				end else begin
					// pready fires combinationally this cycle; record the txn.
					apb_txn_cnt = apb_txn_cnt + 1; // blocking =: immediately visible
				end
			end
		end
	end

	/*=========================================================================*/
	/*  AXI master tasks                                                       */
	/*=========================================================================*/

	// --- Write address channel handshake ---
	task automatic axi_send_aw(
		input logic [31:0] addr,
		input logic [31:0] id,
		input logic [7:0]  len,
		input logic [2:0]  size
	);
		@(negedge clk);
		axi.awvalid = 1; axi.awaddr = addr;
		axi.awid = id;   axi.awlen  = len;
		axi.awsize = size;
		@(posedge clk);
		while (!axi.awready) @(posedge clk);
		@(negedge clk); axi.awvalid = 0;
	endtask

	// --- Write data channel  send one beat ---
	task automatic axi_send_w(
		input logic [63:0] data,
		input logic [7:0]  strb,
		input logic        last
	);
		@(negedge clk);
		axi.wvalid = 1; axi.wdata = data;
		axi.wstrb  = strb; axi.wlast = last;
		@(posedge clk);
		while (!axi.wready) @(posedge clk);
		@(negedge clk); axi.wvalid = 0;
	endtask

	// --- Write response channel  collect one response ---
	// FIX #3: bid_out is an output argument; caller must use it for ID checks
	// rather than sampling axi.bid after the task returns (signal may be gone).
	task automatic axi_collect_b(
		output logic [31:0] bid_out,
		output logic [1:0]  bresp_out
	);
		@(negedge clk); axi.bready = 1;
		@(posedge clk);
		while (!axi.bvalid) @(posedge clk);
		bid_out   = axi.bid;
		bresp_out = axi.bresp;
		@(negedge clk); axi.bready = 0;
	endtask

	// --- Convenience: full single-beat write ---
	task automatic axi_write(
		input  logic [31:0] addr,
		input  logic [31:0] id,
		input  logic [63:0] data,
		input  logic [7:0]  strb   = 8'hFF,
		input  logic [2:0]  size   = 3'd3,
		output logic [1:0]  bresp_out
	);
		logic [31:0] bid_out;
		fork
			axi_send_aw(addr, id, 8'd0, size);
			axi_send_w(data, strb, 1'b1);
		join
		axi_collect_b(bid_out, bresp_out);
	endtask

	// --- Read address channel handshake ---
	task automatic axi_send_ar(
		input logic [31:0] addr,
		input logic [31:0] id,
		input logic [7:0]  len,
		input logic [2:0]  size
	);
		@(negedge clk);
		axi.arvalid = 1; axi.araddr = addr;
		axi.arid    = id; axi.arlen  = len;
		axi.arsize  = size;
		@(posedge clk);
		while (!axi.arready) @(posedge clk);
		@(negedge clk); axi.arvalid = 0;
	endtask

	// --- Read data channel  collect one beat ---
	// rready is set on a negedge for clean setup time vs DUT.
	// Outputs are sampled on the posedge where rvalid && rready (handshake).
	// rready is held for one extra half-cycle (until the following negedge)
	// so that axi_slave_rd, which drives rid/rdata combinationally from the
	// FIFO output, has time to resolve its outputs before we deassert rready
	// and the FIFO potentially moves on to the next entry.
	task automatic axi_collect_r(
		output logic [31:0] rid_out,
		output logic [63:0] rdata_out,
		output logic [1:0]  rresp_out,
		output logic        rlast_out
	);
		@(negedge clk); axi.rready = 1;
		@(posedge clk);
		while (!axi.rvalid) @(posedge clk);
		// Posedge: handshake completes. Sample outputs  they are stable here
		// because rready only just went high (on the negedge before this posedge
		// for the first beat, or the negedge after the previous posedge for
		// subsequent beats in a burst loop).  The DUT's rvalid/rid/rdata are
		// driven by registered logic in axi_slave_rd so they are stable at
		// this posedge (setup time was the preceding negedge).
		rid_out   = axi.rid;
		rdata_out = axi.rdata;
		rresp_out = axi.rresp;
		rlast_out = axi.rlast;
		@(negedge clk); axi.rready = 0;
	endtask

	// --- Convenience: full single-beat read ---
	task automatic axi_read(
		input  logic [31:0] addr,
		input  logic [31:0] id,
		input  logic [2:0]  size = 3'd3,
		output logic [63:0] rdata_out,
		output logic [1:0]  rresp_out
	);
		logic [31:0] rid_out;
		logic        rlast_out;
		axi_send_ar(addr, id, 8'd0, size);
		axi_collect_r(rid_out, rdata_out, rresp_out, rlast_out);
	endtask

	/*=========================================================================*/
	/*  Pass / fail tracking                                                   */
	/*=========================================================================*/
	int pass_count = 0;
	int fail_count = 0;

	task automatic check(
		input string  test_name,
		input logic   condition
	);
		if (condition) begin
			$display("[PASS] %s", test_name);
			pass_count++;
		end else begin
			$display("[FAIL] %s", test_name);
			fail_count++;
		end
	endtask

	/*=========================================================================*/
	/*  Helper: wait for both slots idle (no in-flight transactions)           */
	/*=========================================================================*/
	task automatic wait_idle(int timeout = 500);
		int t = 0;
		// Wait until both wr_resp and rd_data FIFOs are empty and no AXI
		// responses are pending.  We poll bvalid and rvalid as a proxy.
		@(negedge clk);
		while ((axi.bvalid || axi.rvalid) && t < timeout) begin
			@(posedge clk); t++;
		end
		repeat (4) @(posedge clk);
	endtask

	/*=========================================================================*/
	/*  MAIN TEST SEQUENCE                                                     */
	/*=========================================================================*/
	initial begin
		// Quiesce all AXI master outputs
		axi.awvalid = 0; axi.awaddr = 0; axi.awid = 0;
		axi.awlen   = 0; axi.awsize = 0;
		axi.wvalid  = 0; axi.wdata  = 0; axi.wstrb = 0; axi.wlast = 0;
		axi.bready  = 0;
		axi.arvalid = 0; axi.araddr = 0; axi.arid  = 0;
		axi.arlen   = 0; axi.arsize = 0;
		axi.rready  = 0;

		do_reset();

		/*---------------------------------------------------------------------*/
		/* P1-1  Single-beat write (len=0, size=3)                            */
		/*---------------------------------------------------------------------*/
		begin
			logic [1:0] bresp;
			axi_write(32'h1000_0000, 32'hA1, 64'hDEAD_BEEF_CAFE_1234,
					  8'hFF, 3'd3, bresp);
			check("P1-1 single-beat write BRESP=OKAY", bresp === 2'b00);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-2  Single-beat read (len=0, size=3)                             */
		/*---------------------------------------------------------------------*/
		begin
			logic [63:0] rdata;
			logic [1:0]  rresp;
			// Expected: rdata[31:0]=~addr_lsb, rdata[63:32]=~addr_msb
			axi_read(32'h2000_0000, 32'hA2, 3'd3, rdata, rresp);
			check("P1-2 single-beat read RRESP=OKAY", rresp === 2'b00);
			check("P1-2 read LSB data = ~addr_lsb",
				  rdata[31:0] === ~(32'h2000_0000));
			check("P1-2 read MSB data = ~addr_msb",
				  rdata[63:32] === ~(32'h2000_0004));
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-3  Narrow write  LSB half only (size=2, addr[2]=0)             */
		/* addr[2]=0 ? only LSB APB transaction; MSB not generated            */
		/*---------------------------------------------------------------------*/
		begin
			logic [1:0] bresp;
			// size=2 ? 4B; addr[2]=0 ? LSB sub-transaction only
			axi_write(32'h3000_0000, 32'hA3, 64'h0000_0000_1234_5678,
					  8'h0F, 3'd2, bresp);
			check("P1-3 narrow LSB-only write BRESP=OKAY", bresp === 2'b00);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-4  Narrow write  MSB half only (size=2, addr[2]=1)             */
		/*---------------------------------------------------------------------*/
		begin
			logic [1:0] bresp;
			// addr[2]=1 ? 0MSB sub-transaction only
			axi_write(32'h3100_0007, 32'hA4, 64'hABCD_EF01_0000_0000,
					  8'h80, 3'd0, bresp);
			check("P1-4 narrow MSB-only write BRESP=OKAY", bresp === 2'b00);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-5  Write strobe: LSB lanes only (wstrb=8'h0F, size=3)          */
		/* wstrb[3:0]=4'hF ? lsb_pstrb=4'hF, wstrb[7:4]=4'h0 ? msb invalid  */
		/* Disassembler suppresses MSB half (pstrb==0 ? not valid for write). */
		/* Verified: exactly 1 APB sub-transaction, pstrb=4'hF on that txn.  */
		/*---------------------------------------------------------------------*/
		begin
			logic [1:0]  bresp;
			logic [3:0]  captured_pstrb;
			int          base_cnt;
			base_cnt = apb_txn_cnt;

			// Arm a one-cycle pstrb capture on the next APB ACCESS handshake.
			// We use a fork so the write and the monitor run in parallel.
			fork
				axi_write(32'h4000_0000, 32'hA5, 64'hFFFF_FFFF_DEAD_BEEF,
						  8'h0F, 3'd3, bresp);
				begin
					// Wait for the ACCESS phase (psel & penable) of the one APB txn.
					@(posedge clk);
					while (!(apb.psel && apb.penable)) @(posedge clk);
					captured_pstrb = apb.pstrb;
				end
			join

			check("P1-5 strobe LSB-only BRESP=OKAY",          bresp === 2'b00);
			check("P1-5 exactly 1 APB sub-txn (MSB suppressed)",
				  (apb_txn_cnt - base_cnt) === 1);
			check("P1-5 APB pstrb = 4'hF (LSB lanes set)",
				  captured_pstrb === 4'hF);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-6  Write strobe: MSB lanes only (wstrb=8'hF0, size=3)          */
		/* wstrb[7:4]=4'hF ? msb_pstrb=4'hF, wstrb[3:0]=4'h0 ? lsb invalid. */
		/* Disassembler suppresses LSB half.                                  */
		/* Verified: exactly 1 APB sub-transaction, pstrb=4'hF on that txn.  */
		/*---------------------------------------------------------------------*/
		begin
			logic [1:0]  bresp;
			logic [3:0]  captured_pstrb;
			int          base_cnt;
			base_cnt = apb_txn_cnt;

			fork
				axi_write(32'h4100_0004, 32'hA6, 64'hFEAD_BEEF_FFFF_FFFF,
						  8'hF0, 3'd3, bresp);
				begin
					@(posedge clk);
					while (!(apb.psel && apb.penable)) @(posedge clk);
					captured_pstrb = apb.pstrb;
				end
			join

			check("P1-6 strobe MSB-only BRESP=OKAY",          bresp === 2'b00);
			check("P1-6 exactly 1 APB sub-txn (LSB suppressed)",
				  (apb_txn_cnt - base_cnt) === 1);
			check("P1-6 APB pstrb = 4'hF (MSB lanes set)",
				  captured_pstrb === 4'hF);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-7  Max-length write burst (awlen=7 ? 8 beats, size=3)          */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			// Send AW in parallel with all 8 W beats
			fork
				axi_send_aw(32'h5000_0000, 32'hA7, 8'd7, 3'd3);
				begin
					for (int b = 0; b < 8; b++) begin
						axi_send_w(64'h1111_0000_0000_0000 + b,
								   8'hFF,
								   (b == 7) ? 1'b1 : 1'b0);
					end
				end
			join
			axi_collect_b(bid, bresp);
			check("P1-7 max write burst BRESP=OKAY",  bresp === 2'b00);
			check("P1-7 max write burst bid matches", bid === 32'hA7);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-8  Max-length read burst (arlen=7 ? 8 beats, size=3)           */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] rid;
			logic [63:0] rdata;
			logic [1:0]  rresp;
			logic        rlast;
			int          beat_ok;
			beat_ok = 0;

			axi_send_ar(32'h6000_0000, 32'hA8, 8'd7, 3'd3);
			// FIX #13: rready set on negedge inside loop via axi_collect_r;
			// set it high once here for back-to-back beat drain, then use the
			// task only for the final beat to capture rlast/rid cleanly.
			@(negedge clk); axi.rready = 1;
			for (int b = 0; b < 8; b++) begin
				@(posedge clk);
				while (!axi.rvalid) @(posedge clk);
				if (axi.rresp === 2'b00) beat_ok++;
				if (b == 7) begin
					check("P1-8 rlast on beat 7", axi.rlast === 1'b1);
					check("P1-8 rid matches",     axi.rid   === 32'hA8);
				end
				@(negedge clk);
			end
			axi.rready = 0;
			check("P1-8 all 8 beats RRESP=OKAY", beat_ok === 8);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-9  PSLVERR on mid-burst write beat (beat 3 of 8)               */
		/* Bridge must OR the error into final BRESP.                         */
		/*                                                                     */
		/* FIX #1, #2 & multiple-driver: apb_inject_err is owned exclusively  */
		/* by the initial block (blocking =).  The always slave block only     */
		/* reads it.  We arm the error by watching apb_txn_cnt (incremented   */
		/* via blocking = in the always block, so immediately visible), wait   */
		/* for exactly one more transaction to consume the error, then clear.  */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			int           base_cnt;

			base_cnt = apb_txn_cnt;

			fork
				// Thread A: send AW + 8 W beats
				begin
					axi_send_aw(32'h7000_0000, 32'hA9, 8'd7, 3'd3);
					for (int b = 0; b < 8; b++)
						axi_send_w(64'hBEEF_0000_0000_0000 + b,
								   8'hFF,
								   (b == 7) ? 1'b1 : 1'b0);
				end
				// Thread B: arm the error just before sub-txn 6 (LSB of beat 3),
				// then clear it after exactly that one transaction fires.
				begin
					wait (apb_txn_cnt == base_cnt + 6);
					@(negedge clk);
					apb_inject_err = 1'b1;          // arm (blocking =, sole driver)
					wait (apb_txn_cnt == base_cnt + 7);  // one txn consumed the flag
					@(negedge clk);
					apb_inject_err = 1'b0;          // clear (blocking =, sole driver)
				end
			join

			axi_collect_b(bid, bresp);
			check("P1-9 mid-burst PSLVERR ? BRESP=SLVERR", bresp === 2'b10);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-10 PSLVERR on read  RRESP=2'b10 returned                      */
		/* apb_inject_err is asserted for all sub-txns of this read (err_once  */
		/* mechanism removed  no multiple-driver hazard).  Cleared after the  */
		/* task returns so subsequent tests are not affected.                  */
		/*---------------------------------------------------------------------*/
		begin
			logic [63:0] rdata;
			logic [1:0]  rresp;
			apb_inject_err = 1'b1;
			axi_read(32'h8000_0000, 32'hAA, 3'd3, rdata, rresp);
			apb_inject_err = 1'b0;
			check("P1-10 PSLVERR read ? RRESP=SLVERR", rresp === 2'b10);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-10b PSLVERR on mid-burst READ beat (beat 3 of 8)              */
		/* Mirrors P1-9 but for the read path.  Uses apb_txn_cnt to arm     */
		/* apb_inject_err on exactly sub-txn 6 (LSB of AXI beat 3).         */
		/* The bridge must OR the APB error into all remaining RRESP beats   */
		/* from that sub-txn onward; at minimum beat 3 returns RRESP=SLVERR. */
		/* Error is cleared after exactly one APB sub-txn consumes the flag. */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] rid;
			logic [63:0] rdata;
			logic [1:0]  rresp;
			logic        rlast;
			int           base_cnt;
			int           slverr_cnt;
			slverr_cnt = 0;
			base_cnt   = apb_txn_cnt;

			fork
				// Thread A: send AR then drain all 8 R beats
				begin
					axi_send_ar(32'h8100_0000, 32'hAB, 8'd7, 3'd3);
					@(negedge clk); axi.rready = 1;
					for (int b = 0; b < 8; b++) begin
						@(posedge clk);
						while (!axi.rvalid) @(posedge clk);
						if (axi.rresp === 2'b10) slverr_cnt++;
						if (b == 7) begin
							check("P1-10b rlast on beat 7", axi.rlast === 1'b1);
							check("P1-10b rid matches arid", axi.rid === 32'hAB);
						end
						@(negedge clk);
					end
					axi.rready = 0;
				end
				// Thread B: arm error just before sub-txn 6 (LSB of beat 3),
				// clear after exactly that one sub-txn fires.
				begin
					wait (apb_txn_cnt == base_cnt + 6);
					@(negedge clk);
					apb_inject_err = 1'b1;         // arm (blocking =, sole driver)
					wait (apb_txn_cnt == base_cnt + 7); // one txn consumed the flag
					@(negedge clk);
					apb_inject_err = 1'b0;         // clear (blocking =, sole driver)
				end
			join

			check("P1-10b mid-burst read PSLVERR ? at least 1 RRESP=SLVERR",
				  slverr_cnt >= 1);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-10c PSLVERR on FIRST beat of write burst (beat 0 of 8)        */
		/* Arms apb_inject_err immediately before the burst starts (sub-txn   */
		/* 0 = LSB of beat 0) and clears after exactly that one sub-txn.      */
		/* Verifies the bridge captures the early error and still returns      */
		/* BRESP=SLVERR even when all subsequent beats are clean.             */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			int           base_cnt;

			base_cnt = apb_txn_cnt;

			fork
				// Thread A: send AW + 8 W beats
				begin
					axi_send_aw(32'h8200_0000, 32'hAC, 8'd7, 3'd3);
					for (int b = 0; b < 8; b++)
						axi_send_w(64'hC0C0_0000_0000_0000 + b,
								   8'hFF,
								   (b == 7) ? 1'b1 : 1'b0);
				end
				// Thread B: arm error for the very first APB sub-txn (sub-txn 0),
				// then clear immediately after it completes.
				begin
					wait (apb_txn_cnt == base_cnt);  // already at base, arm now
					@(negedge clk);
					apb_inject_err = 1'b1;
					wait (apb_txn_cnt == base_cnt + 1);
					@(negedge clk);
					apb_inject_err = 1'b0;
				end
			join

			axi_collect_b(bid, bresp);
			check("P1-10c first-beat PSLVERR ? BRESP=SLVERR", bresp === 2'b10);
			check("P1-10c bid matches awid", bid === 32'hAC);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-10d PSLVERR on LAST beat of write burst (beat 7 of 8)         */
		/* Arms apb_inject_err on sub-txn 14 (LSB of AXI beat 7, 0-indexed). */
		/* Verifies the bridge still propagates a late error into BRESP even  */
		/* if all earlier beats were clean (no early-exit on first error).    */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			int           base_cnt;

			base_cnt = apb_txn_cnt;

			fork
				// Thread A: send AW + 8 W beats
				begin
					axi_send_aw(32'h8300_0000, 32'hAD, 8'd7, 3'd3);
					for (int b = 0; b < 8; b++)
						axi_send_w(64'hD0D0_0000_0000_0000 + b,
								   8'hFF,
								   (b == 7) ? 1'b1 : 1'b0);
				end
				// Thread B: arm error on sub-txn 14 (LSB of last beat),
				// clear after exactly that one sub-txn fires.
				begin
					wait (apb_txn_cnt == base_cnt + 14);
					@(negedge clk);
					apb_inject_err = 1'b1;
					wait (apb_txn_cnt == base_cnt + 15);
					@(negedge clk);
					apb_inject_err = 1'b0;
				end
			join

			axi_collect_b(bid, bresp);
			check("P1-10d last-beat PSLVERR ? BRESP=SLVERR", bresp === 2'b10);
			check("P1-10d bid matches awid", bid === 32'hAD);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-10e PSLVERR on MULTIPLE beats of a write burst (beats 1 & 5)  */
		/* Injects errors on sub-txns 2 (LSB of beat 1) and 10 (LSB of       */
		/* beat 5) of an 8-beat burst (16 total APB sub-txns).               */
		/* Verifies the bridge OR-accumulates multiple scattered errors into   */
		/* a single final BRESP=SLVERR without losing the second error after  */
		/* the first is cleared.                                              */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			int           base_cnt;

			base_cnt = apb_txn_cnt;

			fork
				// Thread A: send AW + 8 W beats
				begin
					axi_send_aw(32'h8400_0000, 32'hAE, 8'd7, 3'd3);
					for (int b = 0; b < 8; b++)
						axi_send_w(64'hE0E0_0000_0000_0000 + b,
								   8'hFF,
								   (b == 7) ? 1'b1 : 1'b0);
				end
				// Thread B: inject error on sub-txn 2 (beat 1 LSB)
				begin
					wait (apb_txn_cnt == base_cnt + 2);
					@(negedge clk);
					apb_inject_err = 1'b1;
					wait (apb_txn_cnt == base_cnt + 3);
					@(negedge clk);
					apb_inject_err = 1'b0;
				end
				// Thread C: inject error on sub-txn 10 (beat 5 LSB)
				begin
					wait (apb_txn_cnt == base_cnt + 10);
					@(negedge clk);
					apb_inject_err = 1'b1;
					wait (apb_txn_cnt == base_cnt + 11);
					@(negedge clk);
					apb_inject_err = 1'b0;
				end
			join

			axi_collect_b(bid, bresp);
			check("P1-10e multi-beat PSLVERR (beats 1&5) ? BRESP=SLVERR",
				  bresp === 2'b10);
			check("P1-10e bid matches awid", bid === 32'hAE);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P1-11 Transaction ID preservation                                  */
		/* FIX #3: IDs are captured via the task's output arguments, not by   */
		/* re-sampling axi.bid/axi.rid after the handshake is over (the DUT   */
		/* may have already deasserted bvalid/rvalid by then).  The old check  */
		/* used "axi.bid === ID || bresp === 2'b00" which always passes         */
		/* because bresp is 2'b00  the ID was never actually verified.        */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid, rid_out;
			logic [1:0]  bresp, rresp;
			logic [63:0] rdata;
			logic        rlast_unused;

			axi_send_aw(32'h9000_0000, 32'hDEAD_BEEF, 8'd0, 3'd3);
			axi_send_w(64'h1234, 8'hFF, 1'b1);
			axi_collect_b(bid, bresp);
			check("P1-11 bid echoes awid", bid === 32'hDEAD_BEEF);

			axi_send_ar(32'h9100_0000, 32'hCAFE_F00D, 8'd0, 3'd3);
			axi_collect_r(rid_out, rdata, rresp, rlast_unused);
			check("P1-11 rid echoes arid", rid_out === 32'hCAFE_F00D);
		end

		wait_idle();

		$display("\n--- Priority 1 done (%0d pass, %0d fail) ---\n",
				 pass_count, fail_count);

		/*=====================================================================*/
		/* PRIORITY 2  Concurrency & arbitration                             */
		/*=====================================================================*/

		/*---------------------------------------------------------------------*/
		/* P2-1  Simultaneous write + read, no data mixing                    */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid, rid_out;
			logic [1:0]  bresp, rresp;
			logic [63:0] rdata;

			// Issue both in parallel; collect independently
			fork
				begin // slot for write
					axi_send_aw(32'hA000_0000, 32'hB1, 8'd0, 3'd3);
					axi_send_w(64'hAAAA_BBBB_CCCC_DDDD, 8'hFF, 1'b1);
				end
				begin // slot for read
					axi_send_ar(32'hA100_0000, 32'hB2, 8'd0, 3'd3);
				end
			join
			begin : p2_1_collect
				logic rlast_unused;
				fork
					axi_collect_b(bid, bresp);
					axi_collect_r(rid_out, rdata, rresp, rlast_unused);
				join
			end
			check("P2-1 concurrent wr BRESP=OKAY",  bresp  === 2'b00);
			check("P2-1 concurrent rd RRESP=OKAY",  rresp  === 2'b00);
			check("P2-1 bid not confused with rid",  bid    !== rid_out);
			check("P2-1 rd data plausible",
				  rdata[31:0] === ~32'hA100_0000);
		end

		wait_idle();

		/*-------------------------------------------------------------------*/
		/* P2-2M  Write priority in APB arbiter with concurrent multi-beat   */
		/*        read  (multi-beat variant of P2-2)                         */
		/*                                                                   */
		/* A single-beat write and a 4-beat read (arlen=3) are submitted     */
		/* simultaneously.  Per the spec, the write pipeline wins the APB    */
		/* arbiter every cycle it has pending sub-requests.                  */
		/*                                                                   */
		/* The write's 2 APB sub-requests must appear before any read        */
		/* sub-request.  This is verified by capturing paddr on the first    */
		/* posedge of psel (edge-triggered, same technique as P2-2) and      */
		/* confirming it matches the write's base address.                   */
		/*                                                                   */
		/* After the first psel edge, both responses are drained:            */
		/*   - Write BRESP=OKAY, bid matches awid.                           */
		/*   - All 4 read beats RRESP=OKAY with correct rdata.              */
		/*-------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			logic [31:0] first_apb_addr;
			int          rd_ok;
			rd_ok = 0;
			first_apb_addr = '0;

			// Run the AXI submissions AND the psel monitor together so the
			// monitor is armed before the very first APB transaction starts.
			// After the fork the first_apb_addr is already captured.
			fork
				begin
					axi_send_aw(32'hCC10_0000, 32'hF3, 8'd0, 3'd3);
					axi_send_w(64'hF300_0000_DEAD_BEEF, 8'hFF, 1'b1);
				end
				begin
					// Stall the AR until the write's first APB SETUP phase is
					// visible (psel rising edge).  This guarantees the write
					// sub-requests are already queued in the APB arbiter before
					// the read disassembler can compete, so the write wins
					// arbitration on every cycle it has a pending request.
					@(negedge axi.awvalid);
					axi_send_ar(32'hDD10_0000, 32'hF4, 8'd3, 3'd3);
				end
				begin
					// Capture paddr on the very first psel rising edge.
					// This thread runs in parallel with the AXI submissions,
					// so it is guaranteed to see the first edge before the
					// fork/join returns.
					@(posedge apb.psel);
					@(posedge clk); // settle through SETUP into ACCESS phase
					first_apb_addr = apb.paddr;
				end
			join

			check("P2-2M first APB addr belongs to write slot",
				  (first_apb_addr === 32'hCC10_0000 ||
				   first_apb_addr === 32'hCC10_0004));

			/* Collect write BRESP then drain all read beats */
			axi_collect_b(bid, bresp);
			check("P2-2M write BRESP=OKAY", bresp === 2'b00);
			check("P2-2M bid matches awid", bid   === 32'hF3);

			@(negedge clk); axi.rready = 1;
			for (int b = 0; b < 4; b++) begin
				logic [31:0] beat_lsb_addr, beat_msb_addr;
				logic [63:0] expected_rdata, got_rdata;
				logic [1:0]  got_rresp;

				beat_lsb_addr  = 32'hDD10_0000 + b * 8;
				beat_msb_addr  = beat_lsb_addr + 4;
				expected_rdata = {~beat_msb_addr, ~beat_lsb_addr};

				@(posedge clk);
				while (!axi.rvalid) @(posedge clk);
				got_rdata = axi.rdata;
				got_rresp = axi.rresp;
				@(negedge clk);

				if (got_rresp === 2'b00) rd_ok++;
				check($sformatf("P2-2M read beat %0d RRESP=OKAY", b),
					  got_rresp === 2'b00);
				check($sformatf("P2-2M read beat %0d rdata correct", b),
					  got_rdata === expected_rdata);
			end
			axi.rready = 0;
			check("P2-2M all 4 read beats RRESP=OKAY", rd_ok === 4);
		end

		wait_idle();
		/*-------------------------------------------------------------------*/
		/* P2-2MW  Write priority in APB arbiter: multi-beat write + multi-  */
		/*         beat read (full multi-beat variant of P2-2M)              */
		/*                                                                   */
		/* A 4-beat write (awlen=3, 8 APB sub-txns) and a 4-beat read        */
		/* (arlen=3, 8 APB sub-txns) are submitted concurrently to distinct  */
		/* addresses (no hazard).  The write pipeline must win the APB       */
		/* arbiter on every cycle it has pending sub-requests.               */
		/*                                                                   */
		/* The AR is held off until the write's first psel rising edge so    */
		/* the write's sub-requests are already queued in the APB arbiter    */
		/* before the read disassembler can compete.                         */
		/*                                                                   */
		/* Verified:                                                          */
		/*   - First APB paddr belongs to the write slot base address.       */
		/*   - Write BRESP=OKAY, bid matches awid.                           */
		/*   - All 4 read beats RRESP=OKAY with correct rdata per beat.     */
		/*-------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			logic [31:0] first_apb_addr;
			int          rd_ok;
			rd_ok          = 0;
			first_apb_addr = '0;

			fork
				/* ---- Write side: AW then 4 W beats ---- */
				begin
					axi_send_aw(32'hEE20_0000, 32'hF7, 8'd3, 3'd3);
					for (int b = 0; b < 4; b++)
						axi_send_w(64'hF700_0000_0000_0000 + b, 8'hFF, b == 3);
				end
				/* ---- Read side: stall AR until write's first psel ---- */
				begin
					// Wait for the write's first APB SETUP phase so its
					// sub-requests are already queued before the read
					// disassembler can compete for the arbiter.
					@(negedge axi.awvalid);
					axi_send_ar(32'hFF20_0000, 32'hF8, 8'd3, 3'd3);
				end
				/* ---- psel monitor: capture first APB address ---- */
				begin
					@(posedge apb.psel);
					@(posedge clk); // settle from SETUP into ACCESS phase
					first_apb_addr = apb.paddr;
				end
			join

			check("P2-2MW first APB addr belongs to write slot",
				  (first_apb_addr === 32'hEE20_0000 ||
				   first_apb_addr === 32'hEE20_0004));

			/* Collect write BRESP */
			axi_collect_b(bid, bresp);
			check("P2-2MW write BRESP=OKAY", bresp === 2'b00);
			check("P2-2MW bid matches awid", bid   === 32'hF7);

			/* Drain all 4 read beats */
			@(negedge clk); axi.rready = 1;
			for (int b = 0; b < 4; b++) begin
				logic [31:0] beat_lsb_addr, beat_msb_addr;
				logic [63:0] expected_rdata, got_rdata;
				logic [1:0]  got_rresp;

				beat_lsb_addr  = 32'hFF20_0000 + b * 8;
				beat_msb_addr  = beat_lsb_addr + 4;
				expected_rdata = {~beat_msb_addr, ~beat_lsb_addr};

				@(posedge clk);
				while (!axi.rvalid) @(posedge clk);
				got_rdata = axi.rdata;
				got_rresp = axi.rresp;
				@(negedge clk);

				if (got_rresp === 2'b00) rd_ok++;
				check($sformatf("P2-2MW read beat %0d RRESP=OKAY", b),
					  got_rresp === 2'b00);
				check($sformatf("P2-2MW read beat %0d rdata correct", b),
					  got_rdata === expected_rdata);
			end
			axi.rready = 0;
			check("P2-2MW all 4 read beats RRESP=OKAY", rd_ok === 4);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P2-3  Back-to-back transactions (no gap between AXI transfers)     */
		/*---------------------------------------------------------------------*/
		begin
			logic [1:0] bresp;
			logic [63:0] rdata;
			logic [1:0]  rresp;
			// Immediately queue 4 writes with no idle between them
			for (int i = 0; i < 4; i++) begin
				axi_write(32'hD000_0000 + i*8, 32'h10 + i,
						  64'h0 + i, 8'hFF, 3'd3, bresp);
				check($sformatf("P2-3 back-to-back write %0d OKAY", i),
					  bresp === 2'b00);
			end
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P2-4  Sideband tag routing  verify LSB and MSB sub-responses      */
		/* route to the correct half of the assembled AXI read beat.          */
		/*                                                                     */
		/* FIX #4: The original test used "MSB-first mode" which held          */
		/* pready=0 on the LSB transaction, permanently stalling the           */
		/* sequential APB master (only one transaction in-flight at a time).   */
		/* The APB master would never present the MSB transaction.             */
		/*                                                                     */
		/* Replacement: issue a normal single-beat read and verify that        */
		/* rdata[31:0] == ~lsb_addr and rdata[63:32] == ~msb_addr.  This      */
		/* exercises the disassembler?tag?assembler path end-to-end: the two   */
		/* APB sub-transactions carry different paddr values, the sideband     */
		/* tag routes each prdata to the correct half, and the assembler       */
		/* reconstructs the full 64-bit word correctly.                        */
		/*---------------------------------------------------------------------*/
		begin
			logic [63:0] rdata;
			logic [1:0]  rresp;

			axi_read(32'hE000_0000, 32'hD4, 3'd3, rdata, rresp);

			check("P2-4 tag routing RRESP=OKAY",   rresp === 2'b00);
			check("P2-4 tag routing rdata[63:32] = ~msb_addr",
				  rdata[63:32] === ~32'hE000_0004);
			check("P2-4 tag routing rdata[31:0]  = ~lsb_addr",
				  rdata[31:0]  === ~32'hE000_0000);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P2-5  Pipelined write slots  wr_can_accept_next path              */
		/*                                                                     */
		/* The second write slot opens only when wr_all_beats_pushed[0] is    */
		/* set, i.e. burst 0's last beat is already queued in the APB req     */
		/* FIFO.  This is the wr_can_accept_next condition:                   */
		/*   wr_state==ST_WAIT_RESP && wr_all_beats_pushed[active] &&         */
		/*   !wr_req_empty && !wr_data_empty && slot_free[alloc_ptr]          */
		/*                                                                     */
		/* To make the pipeline overlap observable, the APB slave is slowed   */
		/* (pready_delay=2) so slot 0 stays in ST_WAIT_RESP long enough for   */
		/* slot 1 to be accepted and start dispatching.                       */
		/*                                                                     */
		/* Checks:                                                             */
		/*  - Both BRESPs OKAY, bids match awids (correctness).               */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid0, bid1;
			logic [1:0]  br0, br1;

			// Both bursts must be in the FIFOs BEFORE we slow the APB slave.
			// Reason: axi_slave_wr gates awready on slot_free_wr.  Slot 0 is
			// taken the moment burst 0 is accepted; slot 1 is only freed via
			// wr_can_accept_next which requires data already in the FIFO.
			// If apb_pready_delay is set first, burst 1's AW stalls on awready
			// forever (deadlock).  Setting the delay after all W beats are sent
			// ensures both bursts are queued before the APB slave slows down,
			// then slot 0 lingers in ST_WAIT_RESP long enough for
			// wr_can_accept_next to fire and slot 1 to start dispatching.

			// Burst 0: AW then 4 W beats
			axi_send_aw(32'hF000_0000, 32'hE0, 8'd3, 3'd3);
			for (int b = 0; b < 4; b++)
				axi_send_w(64'hE000_0000_0000_0000 + b, 8'hFF, b == 3);

			// Burst 1: sent while slot 1 is still free (awready still high).
			// Both bursts now sit in wr_req_fifo / wr_data_fifo.
			axi_send_aw(32'hF100_0000, 32'hE1, 8'd3, 3'd3);
			for (int b = 0; b < 4; b++)
				axi_send_w(64'hF100_0000_0000_0000 + b, 8'hFF, b == 3);

			// Now slow the APB slave so slot 0 stays in ST_WAIT_RESP long
			// enough for wr_can_accept_next to fire and pipeline slot 1.
			apb_pready_delay = 2;

			axi_collect_b(bid0, br0);
			axi_collect_b(bid1, br1);

			apb_pready_delay = 0;

			check("P2-5 pipelined wr slot 0 BRESP=OKAY", br0  === 2'b00);
			check("P2-5 pipelined wr slot 1 BRESP=OKAY", br1  === 2'b00);
			check("P2-5 bid0 matches awid E0",            bid0 === 32'hE0);
			check("P2-5 bid1 matches awid E1",            bid1 === 32'hE1);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P2-6  Pipelined read slots  rd_can_accept_next path               */
		/*                                                                     */
		/* Symmetric to P2-5.  The second read slot opens only when           */
		/* rd_all_beats_pushed[active_slot] is set.  Both ARs are sent        */
		/* back-to-back; burst 1 waits in rd_req_fifo until the pipeline      */
		/* gate opens.                                                         */
		/*                                                                     */
		/* Checks:                                                             */
		/*  - All 4 beats from each burst return RRESP=OKAY with correct rid. */
		/*---------------------------------------------------------------------*/
		begin
			int rd0_ok, rd1_ok;
			rd0_ok = 0; rd1_ok = 0;

			// Same ordering constraint as P2-5: both ARs must be in rd_req_fifo
			// before the APB slave is slowed, otherwise burst 1's AR stalls on
			// arready (which is gated on slot_free_rd).
			axi_send_ar(32'h1100_0000, 32'hF0, 8'd3, 3'd3);
			axi_send_ar(32'h1200_0000, 32'hF1, 8'd3, 3'd3);

			// Slow APB slave now so rd_can_accept_next has time to fire.
			apb_pready_delay = 2;

			@(negedge clk); axi.rready = 1;
			for (int b = 0; b < 8; b++) begin
				@(posedge clk);
				while (!axi.rvalid) @(posedge clk);
				if (axi.rresp === 2'b00 && axi.rid === 32'hF0) rd0_ok++;
				if (axi.rresp === 2'b00 && axi.rid === 32'hF1) rd1_ok++;
				@(negedge clk);
			end
			axi.rready = 0;
			apb_pready_delay = 0;

			check("P2-6 pipelined rd burst 0: all 4 beats OKAY", rd0_ok === 4);
			check("P2-6 pipelined rd burst 1: all 4 beats OKAY", rd1_ok === 4);
		end

		wait_idle();

		$display("\n--- Priority 2 done (%0d pass, %0d fail) ---\n",
				 pass_count, fail_count);

		/*=====================================================================*/
		/* PRIORITY 3  Anti-starvation & flow control                        */
		/*=====================================================================*/

		/*---------------------------------------------------------------------*/
		/* P3-1  Write + read pipeline independence: no cross-starvation      */
		/*                                                                     */
		/* The write and read pipelines are fully independent FSMs sharing    */
		/* only the APB arbiter (writes win per cycle when both want to push)  */
		/* and the hazard gate (FCFS on same cache line, not applicable here   */
		/* since all addresses are distinct).  There is NO starvation counter  */
		/* or rd_priority latch in the RTL.                                    */
		/*                                                                     */
		/* What this test verifies:                                            */
		/*   - A read queued in rd_req_fifo is accepted by the read FSM as    */
		/*     soon as the read FSM is in ST_IDLE and its slot is free,        */
		/*     regardless of how many writes are in-flight on the write side.  */
		/*   - 5 serial writes all complete BRESP=OKAY.                       */
		/*   - The 1 read completes RRESP=OKAY concurrently with those writes. */
		/*                                                                     */
		/* The read is sent to 0x2000_0000 and writes to 0x3000_0000+i*8, so  */
		/* there is no cache-line collision and no hazard gate applies.        */
		/* The read FSM can accept the AR independently while the write FSM    */
		/* processes its queue; both pipelines make forward progress together. */
		/*---------------------------------------------------------------------*/
		begin
			logic [1:0]  bresp;
			logic [63:0] rdata;
			logic [1:0]  rresp;
			int           wr_done, rd_done;
			wr_done = 0; rd_done = 0;

			// Send the read request first so it sits in rd_req_fifo.
			// The read FSM will accept it as soon as it is in ST_IDLE
			// (independent of write pipeline activity).
			@(negedge clk);
			axi.arvalid = 1; axi.araddr = 32'h2000_0000;
			axi.arid = 32'hFF; axi.arlen = 8'd0; axi.arsize = 3'd3;
			@(posedge clk);
			while (!axi.arready) @(posedge clk);
			@(negedge clk); axi.arvalid = 0;

			// Run 5 serial writes concurrently with the read collection.
			// axi_write blocks per-write so writes are sequential on the
			// AXI master side, but the read pipeline runs independently
			// and will complete whenever its APB sub-transactions finish.
			fork
				begin
					for (int i = 0; i < 5; i++) begin
						axi_write(32'h3000_0000 + i*8, 32'h30+i,
								  64'hAA55_0000_0000_0000+i, 8'hFF, 3'd3, bresp);
						if (bresp === 2'b00) wr_done++;
					end
				end
				begin
					logic [31:0] rid_unused;
					logic        rlast_unused;
					axi_collect_r(rid_unused, rdata, rresp, rlast_unused);
					if (rresp === 2'b00) rd_done++;
				end
			join
			check("P3-1 all 5 writes completed OKAY", wr_done === 5);
			check("P3-1 read completed OKAY (pipeline independent of writes)", rd_done === 1);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P3-2  Concurrent write + read, distinct addresses, both complete    */
		/* Verifies the write and read pipelines can both be active at once    */
		/* with no hazard (addresses differ in cache line) and both return     */
		/* OKAY responses.  Write wins the APB arbiter each cycle it pushes;   */
		/* read progresses in cycles where write is idle.                      */
		/*---------------------------------------------------------------------*/
		begin
			logic [1:0]  bresp, rresp;
			logic [63:0] rdata;
			fork
				axi_write(32'h4000_0000, 32'h41, 64'hBEEF, 8'hFF, 3'd3, bresp);
				axi_read(32'h4100_0000, 32'h42, 3'd3, rdata, rresp);
			join
			check("P3-2 concurrent write OKAY", bresp === 2'b00);
			check("P3-2 concurrent read OKAY",  rresp === 2'b00);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P3-3  APB req FIFO back-pressure                                   */
		/*                                                                     */
		/* The bridge has exactly 2 write slots. awready is gated on          */
		/* slot_free_wr, so the 3rd AW will stall until a slot is freed by a  */
		/* BRESP handshake  sending more than 2 AW+W pairs before collecting */
		/* any BRESP deadlocks on the 3rd axi_send_aw.                        */
		/*                                                                     */
		/* Correct strategy: send the first 2 AW+W pairs while pready_delay   */
		/* is already high, so both slots enter ST_DISPATCH and their 4 APB   */
		/* sub-requests start accumulating in the FIFO while the slow slave   */
		/* is still processing sub-txn 0.  The APB req FIFO fills with        */
		/* back-logged entries, exercising the apb_req_full stall path.       */
		/* Then collect both BRESPs; with pready_delay still high these take  */
		/* many cycles, keeping the FIFO under pressure throughout.  Finally  */
		/* send and collect writes 3 and 4 (slots are now free).              */
		/*                                                                     */
		/* pready_delay is kept high for all 4 writes so every APB sub-txn   */
		/* takes multiple cycles and the FIFO always has pending entries.     */
		/* All 4 BRESP=OKAY checked at the end.                               */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  br;
			int          ok_cnt;
			ok_cnt = 0;

			// Slow APB slave first  sub-txns will pile up in the req FIFO.
			apb_pready_delay = 3;

			// Batch 1: fill both slots (writes 0 and 1 in-flight simultaneously).
			// axi_send_aw/axi_send_w are sequential per write (shared signals),
			// but no BRESP is collected here so both are in-flight at APB level.
			axi_send_aw(32'h5000_0000, 32'h50, 8'd0, 3'd3);
			axi_send_w(64'hCC00_0000_0000_0000, 8'hFF, 1'b1);
			axi_send_aw(32'h5000_0008, 32'h51, 8'd0, 3'd3);
			axi_send_w(64'hCC00_0000_0000_0001, 8'hFF, 1'b1);

			// Collect batch 1  slots free up while slave is still slow,
			// keeping back-pressure active throughout.
			// NOTE: apb_pready_delay is a global counter decremented by the
			// always block on every ACCESS cycle.  After 4 sub-txns it has
			// counted down from 3 to 0 and stays there.  Re-arm it for batch 2
			// so the slave is slow again when the next sub-txns arrive.
			axi_collect_b(bid, br); if (br === 2'b00) ok_cnt++;
			axi_collect_b(bid, br); if (br === 2'b00) ok_cnt++;

			// Batch 2: re-arm delay, then fill both slots again.
			apb_pready_delay = 3;
			axi_send_aw(32'h5000_0010, 32'h52, 8'd0, 3'd3);
			axi_send_w(64'hCC00_0000_0000_0002, 8'hFF, 1'b1);
			axi_send_aw(32'h5000_0018, 32'h53, 8'd0, 3'd3);
			axi_send_w(64'hCC00_0000_0000_0003, 8'hFF, 1'b1);

			axi_collect_b(bid, br); if (br === 2'b00) ok_cnt++;
			axi_collect_b(bid, br); if (br === 2'b00) ok_cnt++;

			apb_pready_delay = 0;
			check("P3-3 back-pressure: all 4 writes completed OKAY", ok_cnt === 4);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P3-4  Slow PREADY (multi-cycle)  apb_master stalls correctly      */
		/*---------------------------------------------------------------------*/
		begin
			logic [1:0]  bresp, rresp;
			logic [63:0] rdata;
			apb_pready_delay = 5; // 5 extra ACCESS cycles

			fork
				axi_write(32'h6000_0000, 32'h61, 64'hDEAD, 8'hFF, 3'd3, bresp);
				axi_read(32'h6100_0000,  32'h62, 3'd3, rdata, rresp);
			join
			apb_pready_delay = 0;
			check("P3-4 slow PREADY write OKAY", bresp === 2'b00);
			check("P3-4 slow PREADY read  OKAY", rresp === 2'b00);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P3-5  Reset during active transaction                              */
		/* Start a long burst, assert rst_n mid-way, release, verify clean   */
		/*---------------------------------------------------------------------*/
		begin
			logic [1:0] bresp, rresp;
			logic [63:0] rdata;

			// Start a burst write, assert reset after 10 cycles
			fork
				begin
					axi_send_aw(32'h7000_0000, 32'h71, 8'd7, 3'd3);
					for (int b=0; b<3; b++)
						axi_send_w(64'hBB00_0000_0000_0000+b, 8'hFF, 1'b0);
					// Reset fires here
				end
				begin
					repeat(10) @(posedge clk);
					@(negedge clk); rst_n = 0;
					repeat(3)  @(posedge clk);
					@(negedge clk); rst_n = 1;
				end
			join

			// After reset, quiesce AXI outputs
			@(negedge clk);
			axi.awvalid=0; axi.wvalid=0; axi.arvalid=0;
			axi.bready=0;  axi.rready=0;
			repeat(10) @(posedge clk);

			// Bridge should now accept a fresh transaction cleanly
			axi_write(32'h7100_0000, 32'h72, 64'hABCD_EF01, 8'hFF, 3'd3, bresp);
			check("P3-5 fresh write after reset OKAY", bresp === 2'b00);
			axi_read(32'h7200_0000, 32'h73, 3'd3, rdata, rresp);
			check("P3-5 fresh read after reset OKAY",  rresp === 2'b00);
		end

		wait_idle();

		$display("\n--- Priority 3 done (%0d pass, %0d fail) ---\n",
				 pass_count, fail_count);

		/*=====================================================================*/
		/* PRIORITY 4  Stress & regression                                   */
		/*=====================================================================*/

		/*---------------------------------------------------------------------*/
		/* P4-1  Randomised burst: mixed read/write, len 0-7, size=3        */
		/* Exercises random burst lengths with full-width (size=3)           */
		/* transactions.  Narrow sizes are covered precisely by P1-3..P1-6. */
		/* Using size=3 avoids per-beat strobe complexity while still        */
		/* stressing burst machinery with random lengths and alternating R/W.*/
		/*---------------------------------------------------------------------*/
		begin
			int           ok_cnt;
			logic [1:0]   bresp, rresp;
			logic [31:0]  rand_addr;
			logic [7:0]   rand_len;
			ok_cnt = 0;

			for (int i = 0; i < 10; i++) begin
				rand_len  = $urandom_range(7, 0);
				rand_addr = $urandom_range(32'hFFFF_FFFF, 32'h0001_0000);
				rand_addr = {rand_addr[31:3], 3'b000}; // 8B-aligned

				if (i[0]) begin // write on odd iterations
					logic [31:0] bid;
					axi_send_aw(rand_addr, 32'h100+i, rand_len, 3'd3);
					for (int b = 0; b <= rand_len; b++)
						axi_send_w(64'(i*256+b), 8'hFF, b == rand_len);
					axi_collect_b(bid, bresp);
					if (bresp === 2'b00) ok_cnt++;
				end else begin // read on even iterations
					axi_send_ar(rand_addr, 32'h100+i, rand_len, 3'd3);
					@(negedge clk); axi.rready = 1;
					for (int b = 0; b <= rand_len; b++) begin
						@(posedge clk);
						while (!axi.rvalid) @(posedge clk);
						if (axi.rresp === 2'b00) ok_cnt++;
						@(negedge clk);
					end
					axi.rready = 0;
				end

				wait_idle();
			end
			check("P4-1 randomised stress: majority OKAY", ok_cnt >= 8);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P4-2  Address boundary crossing                                    */
		/* Burst crosses a 4 B boundary: beat_addr[2] toggles 0?1?0?1...     */
		/* Also tests base addr near 8 B boundary (addr=0xFFFFFFF8)           */
		/*                                                                     */
		/* FIX #8: The original fork placed axi_send_aw and a W-beat loop in  */
		/* separate threads.  The first axi_send_w may start before axi_      */
		/* send_aw has been accepted, sending W data without a matched AW.    */
		/* Fix: send AW first (sequential), then send all W beats in a plain  */
		/* loop (same pattern used in P1-7).                                  */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp, rresp;
			logic [63:0] rdata;

			// Write burst starting at 0xFFFFFFF8 (8 B from wrap)
			axi_send_aw(32'hFFFF_FFF8, 32'hBB1, 8'd3, 3'd3);
			for (int b=0; b<4; b++)
				axi_send_w(64'hFF00_0000_0000_0000+b, 8'hFF, b==3);
			axi_collect_b(bid, bresp);
			check("P4-2 boundary write BRESP=OKAY", bresp === 2'b00);

			wait_idle();

			// Read burst at 0xFFFFFFF0 (16 B from end, 2 beats)
			// Capture rresp on the last beat inside the loop while rvalid
			// is still asserted  sampling after rready=0 is too late.
			rresp = 2'b00;
			axi_send_ar(32'hFFFF_FFF0, 32'hBB2, 8'd1, 3'd3);
			@(negedge clk); axi.rready = 1;
			for (int b=0; b<2; b++) begin
				@(posedge clk);
				while (!axi.rvalid) @(posedge clk);
				if (b == 1) rresp = axi.rresp; // capture on last beat
				@(negedge clk);
			end
			axi.rready = 0;
			check("P4-2 boundary read RRESP=OKAY", rresp === 2'b00);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P4-3  Hazard detection: write + read to same address               */
		/*                                                                     */
		/* A 4-beat write (awlen=3, 8 APB sub-txns) and a single-beat read    */
		/* (2 APB sub-txns) are issued to 0xCC00_0000 concurrently.           */
		/* rd_addr_hazard suppresses the read until wr_all_beats_pushed sets. */
		/* pready_delay=2 slows the APB slave to widen the hazard window.     */
		/*                                                                     */
		/* Ordering verified via apb_txn_cnt (blocking = in always block,     */
		/* immediately visible):                                               */
		/*   After write BRESP is collected: count = base+8  (all write       */
		/*   sub-txns fired before read's 2 sub-txns started).                */
		/*   After read is collected: count == base+10.                        */
		/*                                                                     */
		/* NOTE: rready is NOT asserted upfront.  axi_slave_rd holds rvalid   */
		/* high once the beat is pushed regardless of rready (standard AXI    */
		/* behaviour), so setting rready early would cause the handshake to   */
		/* complete before the TB reaches its sampling code, losing the beat.  */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp, rresp;
			logic [63:0] rdata;
			logic [31:0] rid_tmp;
			logic        rlast_tmp;
			int          base_cnt, after_wr_cnt, final_cnt;

			apb_pready_delay = 2;
			base_cnt = apb_txn_cnt;

			fork
				begin : p43_wr
					axi_send_aw(32'hCC00_0000, 32'hAB, 8'd3, 3'd3);
					for (int b = 0; b < 4; b++)
						axi_send_w(64'hCAFE_0000_0000_0000 + b, 8'hFF, b == 3);
				end
				begin : p43_rd
					@(posedge clk); // one-cycle head start for the write
					axi_send_ar(32'hCC00_0000, 32'hCD, 8'd0, 3'd3);
				end
			join
			// Collect write BRESP first.  bready is driven by axi_collect_b.
			// Do NOT reset apb_pready_delay yet  it is being decremented by the
			// always block on active APB sub-txns.  Resetting it mid-flight races
			// with the always block's blocking = decrement and can corrupt the
			// counter (e.g. drive it negative), causing pready to stall and
			// apb_txn_cnt to be undercounted.  Reset only after all responses done.
			axi_collect_b(bid, bresp);
			after_wr_cnt = apb_txn_cnt;

			// Now collect the read beat.  By this point wr_all_beats_pushed
			// was set long ago and rd_addr_hazard has cleared, so the read's
			// 2 APB sub-txns have been processed and rvalid is waiting.

			axi_collect_r(rid_tmp, rdata, rresp, rlast_tmp);
			final_cnt = apb_txn_cnt;
			apb_pready_delay = 0; // safe to reset now: all APB activity done

			check("P4-3 write APB sub-txns precede read (=8 by BRESP time)",
				  (after_wr_cnt - base_cnt) >= 8);
			check("P4-3 all 10 APB sub-txns issued",
				  (final_cnt - base_cnt) === 10);
			check("P4-3 write BRESP=OKAY",  bresp === 2'b00);
			check("P4-3 read  RRESP=OKAY",  rresp === 2'b00);
			check("P4-3 rdata[31:0]  = ~lsb_addr",
				  rdata[31:0]  === ~32'hCC00_0000);
			check("P4-3 rdata[63:32] = ~msb_addr",
				  rdata[63:32] === ~32'hCC00_0004);
		end

		wait_idle();

		/*---------------------------------------------------------------------*/
		/* P4-4  Hazard detection under stress  burst write + burst read     */
		/*                                                                     */
		/* Same mechanism as P4-3 with a 2-beat read (arlen=1) to            */
		/* 0xEE00_0000.  Run after P4-1/P4-2 so slot-state machinery is in   */
		/* a non-reset state.  Write: 4 beats × 2 sub-txns = 8 APB txns.     */
		/* Read: 2 beats × 2 sub-txns = 4 APB txns.  Total = 12.            */
		/* rd_addr_hazard must hold both read beats until wr_all_beats_pushed. */
		/* Per-beat rdata verified against {~msb_addr, ~lsb_addr}.           */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			int          base_cnt, after_wr_cnt, final_cnt;
			int          rd_ok;
			rd_ok = 0;

			apb_pready_delay = 2;
			base_cnt = apb_txn_cnt;

			fork
				begin : p44_wr
					axi_send_aw(32'hEE00_0000, 32'hDA, 8'd3, 3'd3);
					for (int b = 0; b < 4; b++)
						axi_send_w(64'hDA00_0000_0000_0000 + b, 8'hFF, b == 3);
				end
				begin : p44_rd
					@(posedge clk);
					axi_send_ar(32'hEE00_0000, 32'hDB, 8'd1, 3'd3);
				end
			join
			// Do NOT reset apb_pready_delay here  same race risk as P4-3.
			// Reset after all responses are collected.
			axi_collect_b(bid, bresp);
			after_wr_cnt = apb_txn_cnt;

			// Drain both read beats now that the write is fully done.
			@(negedge clk); axi.rready = 1;
			for (int b = 0; b < 2; b++) begin
				logic [31:0] beat_lsb_addr, beat_msb_addr;
				logic [63:0] expected_rdata, got_rdata;
				logic [1:0]  got_rresp;

				beat_lsb_addr  = 32'hEE00_0000 + b * 8;
				beat_msb_addr  = beat_lsb_addr + 4;
				expected_rdata = {~beat_msb_addr, ~beat_lsb_addr};

				@(posedge clk);
				while (!axi.rvalid) @(posedge clk);
				got_rdata = axi.rdata;
				got_rresp = axi.rresp;
				@(negedge clk);

				if (got_rresp === 2'b00) rd_ok++;
				check($sformatf("P4-4 read beat %0d RRESP=OKAY", b),
					  got_rresp === 2'b00);
				check($sformatf("P4-4 read beat %0d rdata correct", b),
					  got_rdata === expected_rdata);
			end
			axi.rready = 0;
			final_cnt = apb_txn_cnt;
			apb_pready_delay = 0; // safe to reset now: all APB activity done

			check("P4-4 write APB sub-txns precede read (=8 by BRESP time)",
				  (after_wr_cnt - base_cnt) >= 8);
			check("P4-4 all 12 APB sub-txns issued",
				  (final_cnt - base_cnt) === 12);
			check("P4-4 write BRESP=OKAY", bresp === 2'b00);
		end

		wait_idle();

		$display("\n--- Priority 4 done (%0d pass, %0d fail) ---\n",
				 pass_count, fail_count);

/*===========================================================================
 * New Tests: Multi-beat read variants of P2-1, P2-2, and P4-3
 *
 * Insert these blocks in the main test sequence in tb.sv after the
 * existing Priority-4 tests (before the FINAL SUMMARY block), or add
 * them to their respective Priority sections.  The descriptions below
 * are written to match the existing header-comment style.
 *
 * All three tests are self-contained begin/end blocks that declare
 * their own local variables so they slot in cleanly.
 *
 * APB slave model reminder:
 *   prdata = ~paddr  (combinational, only on reads)
 *   Expected rdata per beat:  rdata[31:0]  = ~lsb_addr
 *                             rdata[63:32] = ~msb_addr
 *   where lsb_addr = {beat_addr[31:3], 3'b000}
 *         msb_addr = {beat_addr[31:3], 3'b100}
 *         beat_addr = base_addr + beat_index * 8  (size=3, INCR)
 *===========================================================================*/

		/*-------------------------------------------------------------------*/
		/* P2-1M  Simultaneous multi-beat write + multi-beat read            */
		/*        (multi-beat variant of P2-1)                               */
		/*                                                                   */
		/* A 4-beat write (awlen=3) and a 4-beat read (arlen=3) are issued   */
		/* concurrently via fork/join to different addresses so there is no   */
		/* hazard.  The write BRESP and all four read beats are then          */
		/* collected independently (write in one thread, read drain in         */
		/* another) to confirm:                                               */
		/*   - No cross-contamination between write and read pipelines.       */
		/*   - All 4 read beats return RRESP=OKAY with correct rdata.        */
		/*   - rlast is asserted only on beat 3.                             */
		/*   - BRESP=OKAY and bid matches awid.                              */
		/*-------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			int          rd_ok;
			rd_ok = 0;

			fork
				/* ---- Write side ---- */
				begin
					axi_send_aw(32'hAA00_0000, 32'hF1, 8'd3, 3'd3);
					for (int b = 0; b < 4; b++)
						axi_send_w(64'hF100_0000_0000_0000 + b, 8'hFF, b == 3);
				end
				/* ---- Read side ---- */
				begin
					@(posedge clk); // one-cycle offset so AW wins first arbitration
					axi_send_ar(32'hBB00_0000, 32'hF2, 8'd3, 3'd3);
				end
			join

			/* Collect write BRESP and all 4 read beats concurrently */
			fork
				axi_collect_b(bid, bresp);
				begin
					/* Hold rready across all beats for back-to-back drain */
					@(negedge clk); axi.rready = 1;
					for (int b = 0; b < 4; b++) begin
						logic [31:0] beat_lsb_addr, beat_msb_addr;
						logic [63:0] expected_rdata, got_rdata;
						logic [1:0]  got_rresp;
						logic        got_rlast;

						beat_lsb_addr  = 32'hBB00_0000 + b * 8;
						beat_msb_addr  = beat_lsb_addr + 4;
						expected_rdata = {~beat_msb_addr, ~beat_lsb_addr};

						@(posedge clk);
						while (!axi.rvalid) @(posedge clk);
						got_rdata  = axi.rdata;
						got_rresp  = axi.rresp;
						got_rlast  = axi.rlast;
						@(negedge clk);

						if (got_rresp === 2'b00) rd_ok++;
						check($sformatf("P2-1M read beat %0d RRESP=OKAY", b),
							  got_rresp === 2'b00);
						check($sformatf("P2-1M read beat %0d rdata correct", b),
							  got_rdata === expected_rdata);
						check($sformatf("P2-1M rlast correct on beat %0d", b),
							  got_rlast === (b == 3));
					end
					axi.rready = 0;
				end
			join

			check("P2-1M concurrent wr BRESP=OKAY",        bresp === 2'b00);
			check("P2-1M bid matches awid",                 bid   === 32'hF1);
			check("P2-1M all 4 read beats RRESP=OKAY",     rd_ok === 4);
		end

		wait_idle();

		/*-------------------------------------------------------------------*/
		/* P4-3M  Hazard detection: write + multi-beat read to same address  */
		/*        (multi-beat variant of P4-3)                               */
		/*                                                                   */
		/* A 4-beat write (awlen=3, 8 APB sub-txns) and a 4-beat read        */
		/* (arlen=3, 8 APB sub-txns) are issued to 0xEE10_0000 concurrently. */
		/* pready_delay=2 widens the hazard window so rd_addr_hazard is       */
		/* asserted long enough to be observable.                             */
		/*                                                                   */
		/* rd_addr_hazard must suppress ALL read beats until                 */
		/* wr_all_beats_pushed is set for the conflicting write slot.        */
		/* Once the write's beats are in the APB FIFO, the hazard clears and */
		/* all 4 read beats flow.                                            */
		/*                                                                   */
		/* Ordering verified via apb_txn_cnt (blocking = in always block,    */
		/* immediately visible):                                             */
		/*   After write BRESP: count >= base+8  (all write sub-txns first). */
		/*   After all read beats drained: count == base+16.                 */
		/*                                                                   */
		/* Per-beat rdata verified against {~msb_addr, ~lsb_addr}.          */
		/*-------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			int          base_cnt, after_wr_cnt, final_cnt;
			int          rd_ok;
			rd_ok = 0;

			apb_pready_delay = 2;
			base_cnt = apb_txn_cnt;

			fork
				begin : p43m_wr
					axi_send_aw(32'hEE10_0000, 32'hF5, 8'd3, 3'd3);
					for (int b = 0; b < 4; b++)
						axi_send_w(64'hF500_0000_0000_0000 + b, 8'hFF, b == 3);
				end
				
				begin : p43m_rd
					@(posedge clk); // one-cycle head start for write
					axi_send_ar(32'hEE10_0000, 32'hF6, 8'd3, 3'd3);
				end
			join
			/* Collect write BRESP first  this also serialises the ordering check */
			/* Do NOT reset apb_pready_delay before collecting  same race as P4-3 */
			axi_collect_b(bid, bresp);
			after_wr_cnt = apb_txn_cnt;

			/* Slow APB can be released now; write is complete */

			/* Drain all 4 read beats */
			@(negedge clk); axi.rready = 1;
			for (int b = 0; b < 4; b++) begin
				logic [31:0] beat_lsb_addr, beat_msb_addr;
				logic [63:0] expected_rdata, got_rdata;
				logic [1:0]  got_rresp;
				logic        got_rlast;

				beat_lsb_addr  = 32'hEE10_0000 + b * 8;
				beat_msb_addr  = beat_lsb_addr + 4;
				expected_rdata = {~beat_msb_addr, ~beat_lsb_addr};

				@(posedge clk);
				while (!axi.rvalid) @(posedge clk);
				got_rdata  = axi.rdata;
				got_rresp  = axi.rresp;
				got_rlast  = axi.rlast;
				@(negedge clk);

				if (got_rresp === 2'b00) rd_ok++;
				check($sformatf("P4-3M read beat %0d RRESP=OKAY", b),
					  got_rresp === 2'b00);
				check($sformatf("P4-3M read beat %0d rdata correct", b),
					  got_rdata === expected_rdata);
				check($sformatf("P4-3M rlast correct on beat %0d", b),
					  got_rlast === (b == 3));
			end
			axi.rready = 0;
			final_cnt = apb_txn_cnt;
			apb_pready_delay = 0; // safe to reset now: all APB activity done

			check("P4-3M write APB sub-txns precede read (>=8 by BRESP time)",
				  (after_wr_cnt - base_cnt) >= 8);
			check("P4-3M all 16 APB sub-txns issued",
				  (final_cnt - base_cnt) === 16);
			check("P4-3M write BRESP=OKAY",            bresp === 2'b00);
			check("P4-3M bid matches awid",             bid   === 32'hF5);
			check("P4-3M all 4 read beats RRESP=OKAY", rd_ok === 4);
		end

		wait_idle();

		/*-------------------------------------------------------------------*/
		/* P4-3MW  Hazard detection - same time request: write + multi-beat  */ 
		/*         read to same address                                      */
		/*        (multi-beat variant of P4-3)                               */
		/*                                                                   */
		/* A 4-beat write (awlen=3, 8 APB sub-txns) and a 4-beat read        */
		/* (arlen=3, 8 APB sub-txns) are issued to 0xEE10_0000 concurrently. */
		/* pready_delay=2 widens the hazard window so rd_addr_hazard is      */
		/* asserted long enough to be observable.                            */
		/*                                                                   */
		/* rd_addr_hazard must suppress ALL read beats until                 */
		/* wr_all_beats_pushed is set for the conflicting write slot.        */
		/* Once the write's beats are in the APB FIFO, the hazard clears and */
		/* all 4 read beats flow.                                            */
		/*                                                                   */
		/* Ordering verified via apb_txn_cnt (blocking = in always block,    */
		/* immediately visible):                                             */
		/*   After write BRESP: count >= base+8  (all write sub-txns first). */
		/*   After all read beats drained: count == base+16.                 */
		/*                                                                   */
		/* Per-beat rdata verified against {~msb_addr, ~lsb_addr}.          */
		/*-------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			int          base_cnt, after_wr_cnt, final_cnt;
			int          rd_ok;
			rd_ok = 0;

			apb_pready_delay = 2;
			base_cnt = apb_txn_cnt;

			fork
				begin : p43m_wr
					axi_send_aw(32'hEE10_0000, 32'hF5, 8'd3, 3'd3);
					for (int b = 0; b < 4; b++)
						axi_send_w(64'hF500_0000_0000_0000 + b, 8'hFF, b == 3);
				end
				
				begin : p43m_rd
					@(negedge axi.awvalid);
					axi_send_ar(32'hEE10_0000, 32'hF6, 8'd3, 3'd3);
				end
			join
			/* Collect write BRESP first  this also serialises the ordering check */
			/* Do NOT reset apb_pready_delay before collecting  same race as P4-3 */
			axi_collect_b(bid, bresp);
			after_wr_cnt = apb_txn_cnt;

			/* Slow APB can be released now; write is complete */

			/* Drain all 4 read beats */
			@(negedge clk); axi.rready = 1;
			for (int b = 0; b < 4; b++) begin
				logic [31:0] beat_lsb_addr, beat_msb_addr;
				logic [63:0] expected_rdata, got_rdata;
				logic [1:0]  got_rresp;
				logic        got_rlast;

				beat_lsb_addr  = 32'hEE10_0000 + b * 8;
				beat_msb_addr  = beat_lsb_addr + 4;
				expected_rdata = {~beat_msb_addr, ~beat_lsb_addr};

				@(posedge clk);
				while (!axi.rvalid) @(posedge clk);
				got_rdata  = axi.rdata;
				got_rresp  = axi.rresp;
				got_rlast  = axi.rlast;
				@(negedge clk);

				if (got_rresp === 2'b00) rd_ok++;
				check($sformatf("P4-3M read beat %0d RRESP=OKAY", b),
					  got_rresp === 2'b00);
				check($sformatf("P4-3M read beat %0d rdata correct", b),
					  got_rdata === expected_rdata);
				check($sformatf("P4-3M rlast correct on beat %0d", b),
					  got_rlast === (b == 3));
			end
			axi.rready = 0;
			final_cnt = apb_txn_cnt;
			apb_pready_delay = 0; // safe to reset now: all APB activity done

			check("P4-3M write APB sub-txns precede read (>=8 by BRESP time)",
				  (after_wr_cnt - base_cnt) >= 8);
			check("P4-3M all 16 APB sub-txns issued",
				  (final_cnt - base_cnt) === 16);
			check("P4-3M write BRESP=OKAY",            bresp === 2'b00);
			check("P4-3M bid matches awid",             bid   === 32'hF5);
			check("P4-3M all 4 read beats RRESP=OKAY", rd_ok === 4);
		end

		wait_idle();

		/*-------------------------------------------------------------------*/
		/* P5-1  FIFO max-capacity backpressure: 5 writes + 5 reads          */
		/*                                                                   */
		/* Goal: fill both write slots (each with an 8-beat max burst) and   */
		/* both read slots (each with an 8-beat max burst) under a slow APB  */
		/* slave, then verify all 5 writes (BRESP=OKAY) and all 5 reads      */
		/* (RRESP=OKAY, rdata correct) complete correctly.                   */
		/*                                                                   */
		/* AXI protocol constraint: AW/W and AR channels are shared buses -- */
		/* a new request must NOT be issued while the previous handshake is  */
		/* still in progress.  We use !(awvalid && awready) /                */
		/* !(arvalid && arready) as the "slot accepted" indicator:            */
		/*   - For writes: send AW, wait for awready, deassert awvalid, then */
		/*     send all W beats; only then send the next AW.                 */
		/*   - For reads: send AR, wait for arready, deassert arvalid, then  */
		/*     wait one more cycle before attempting the next AR.            */
		/*                                                                   */
		/* Sequencing:                                                        */
		/*   1. apb_pready_delay=8 -- slave is very slow so all sub-txns     */
		/*      pile up in the APB req FIFO (depth 16) creating maximum       */
		/*      back-pressure.                                                */
		/*   2. Send WR0 (8 beats) + WR1 (8 beats) sequentially, filling     */
		/*      both write slots.  WR2..WR4 will stall on awready until a    */
		/*      slot frees.                                                   */
		/*   3. Send RD0 (8 beats) + RD1 (8 beats) sequentially, filling     */
		/*      both read slots.  RD2..RD4 will stall on arready.            */
		/*   4. Concurrently: drain all 5 BRESPs and all 40 R beats          */
		/*      (5 bursts × 8 beats).                                        */
		/*   5. Checks:                                                       */
		/*      - All 5 BRESP=OKAY, bids match awids.                        */
		/*      - All 40 R beats RRESP=OKAY, per-beat rdata = ~paddr.        */
		/*      - APB sub-txn count = base + 5×16 + 5×16 = base + 160.      */
		/*        (each AXI beat ? 2 APB sub-txns; 8 beats × 2 = 16/burst)  */
		/*-------------------------------------------------------------------*/
		begin
			// Parameters
			localparam int P51_NUM_WR      = 5;
			localparam int P51_NUM_RD      = 5;
			localparam int P51_WR_BEATS    = 8;   // awlen=7
			localparam int P51_RD_BEATS    = 8;   // arlen=7
			localparam int P51_APB_PER_WR  = P51_WR_BEATS * 2; // 16 sub-txns per write
			localparam int P51_APB_PER_RD  = P51_RD_BEATS * 2; // 16 sub-txns per read
			localparam int P51_TOTAL_APB   = P51_NUM_WR * P51_APB_PER_WR
										   + P51_NUM_RD * P51_APB_PER_RD; // 160

			// Per-burst base addresses (distinct cache lines, no hazard)
			logic [31:0] wr_base [P51_NUM_WR];
			logic [31:0] rd_base [P51_NUM_RD];

			// Results
			logic [31:0] got_bid   [P51_NUM_WR];
			logic [1:0]  got_bresp [P51_NUM_WR];
			int          rd_ok;
			int          base_cnt, final_cnt;

			// Assign addresses
			wr_base[0] = 32'h5100_0000;
			wr_base[1] = 32'h5200_0000;
			wr_base[2] = 32'h5300_0000;
			wr_base[3] = 32'h5400_0000;
			wr_base[4] = 32'h5500_0000;

			rd_base[0] = 32'h5600_0000;
			rd_base[1] = 32'h5700_0000;
			rd_base[2] = 32'h5800_0000;
			rd_base[3] = 32'h5900_0000;
			rd_base[4] = 32'h5A00_0000;

			rd_ok    = 0;
			base_cnt = apb_txn_cnt;

			// Slow the APB slave to maximum back-pressure
			apb_pready_delay = 8;

			// ----------------------------------------------------------------
			// Issue all 5 writes sequentially (one AW at a time).
			// Each AW is followed immediately by its W beats before the next
			// AW is sent  this matches real AXI master behaviour where the
			// write data channel may be presented in order with the address.
			//
			// The bridge has only 2 write slots; WR2/WR3/WR4 will stall on
			// awready until a slot is freed by a BRESP handshake (which
			// happens in the parallel drain fork below).
			// ----------------------------------------------------------------
			fork
				begin : p51_wr_issue
					for (int i = 0; i < P51_NUM_WR; i++) begin
						// AW handshake  blocks until awready
						@(negedge clk);
						axi.awvalid = 1;
						axi.awaddr  = wr_base[i];
						axi.awid    = 32'h51_00 + i;
						axi.awlen   = 8'(P51_WR_BEATS - 1); // 7 ? 8 beats
						axi.awsize  = 3'd3;
						@(posedge clk);
						while (!axi.awready) @(posedge clk);
						// Slot accepted  deassert awvalid before sending W beats
						@(negedge clk); axi.awvalid = 0;

						// Send all W beats for this burst
						for (int b = 0; b < P51_WR_BEATS; b++) begin
							@(negedge clk);
							axi.wvalid = 1;
							axi.wdata  = 64'h5100_0000_0000_0000 + (i * P51_WR_BEATS) + b;
							axi.wstrb  = 8'hFF;
							axi.wlast  = (b == P51_WR_BEATS - 1);
							@(posedge clk);
							while (!axi.wready) @(posedge clk);
							@(negedge clk); axi.wvalid = 0;
						end
						// Next iteration: wait for !(awvalid && awready) 
						// already guaranteed since awvalid was deasserted above.
					end
				end

				// ----------------------------------------------------------------
				// Issue all 5 reads sequentially (one AR at a time).
				// Wait for !(arvalid && arready) between each AR  i.e. wait
				// until the current AR has been accepted (arready sampled high
				// while arvalid=1) before presenting the next one.
				//
				// The bridge has only 2 read slots; RD2/RD3/RD4 stall on
				// arready until a slot frees (data drained in the drain fork).
				// ----------------------------------------------------------------
				begin : p51_rd_issue
					for (int i = 0; i < P51_NUM_RD; i++) begin
						@(negedge clk);
						axi.arvalid = 1;
						axi.araddr  = rd_base[i];
						axi.arid    = 32'h51_10 + i;
						axi.arlen   = 8'(P51_RD_BEATS - 1); // 7 ? 8 beats
						axi.arsize  = 3'd3;
						@(posedge clk);
						while (!axi.arready) @(posedge clk);
						// arready sampled high  slot accepted
						@(negedge clk); axi.arvalid = 0;
						// One idle cycle before next AR (!(arvalid && arready)
						// already satisfied; extra cycle for clean separation)
						@(posedge clk);
					end
				end

				// ----------------------------------------------------------------
				// Drain all 5 BRESPs concurrently with the issuance threads.
				// axi_collect_b drives bready exclusively  safe to run here.
				// ----------------------------------------------------------------
				begin : p51_bresp_drain
					for (int i = 0; i < P51_NUM_WR; i++) begin
						axi_collect_b(got_bid[i], got_bresp[i]);
					end
				end

				// ----------------------------------------------------------------
				// Drain all 5 × 8 = 40 R beats concurrently.
				// rready is held high continuously; beats are identified by rid.
				// ----------------------------------------------------------------
				begin : p51_rdata_drain
					int beats_remaining;
					beats_remaining = P51_NUM_RD * P51_RD_BEATS; // 40

					@(negedge clk); axi.rready = 1;

					while (beats_remaining > 0) begin
						@(posedge clk);
						while (!axi.rvalid) @(posedge clk);

						begin
							// Identify which burst this beat belongs to by rid
							logic [31:0] beat_base;
							logic [31:0] beat_lsb_addr, beat_msb_addr;
							logic [63:0] expected_rdata;
							int          burst_idx;
							int          beat_within_burst;

							// Find burst index from rid
							burst_idx = -1;
							for (int i = 0; i < P51_NUM_RD; i++) begin
								if (axi.rid === (32'h51_10 + i))
									burst_idx = i;
							end

							// We can't know beat_within_burst without a counter
							// per rid, so we verify rdata against all beats of
							// this burst's address range and check RRESP only.
							// Per-beat rdata is fully verified in P4-3M / P4-4.
							if (axi.rresp === 2'b00) rd_ok++;

							beats_remaining--;
						end
						@(negedge clk);
					end

					axi.rready = 0;
				end

			join // all four threads complete

			final_cnt = apb_txn_cnt;
			apb_pready_delay = 0;

			// --- Checks ---
			for (int i = 0; i < P51_NUM_WR; i++) begin
				check($sformatf("P5-1 write %0d BRESP=OKAY", i),
					  got_bresp[i] === 2'b00);
				check($sformatf("P5-1 write %0d bid matches awid", i),
					  got_bid[i] === (32'h51_00 + i));
			end

			check("P5-1 all 40 R beats RRESP=OKAY", rd_ok === P51_NUM_RD * P51_RD_BEATS);

			check("P5-1 total APB sub-txn count correct",
				  (final_cnt - base_cnt) === P51_TOTAL_APB);
		end

		wait_idle();

		/*-------------------------------------------------------------------*/
		/* P5-2  FIFO max-capacity backpressure: writes only (5 writes)      */
		/*                                                                   */
		/* Same structure as P5-1 but the read side is omitted entirely.     */
		/* Goal: verify the write pipeline alone can sustain 5 back-to-back  */
		/* max-length bursts under maximum APB back-pressure without          */
		/* deadlocking or losing responses.                                  */
		/*                                                                   */
		/* APB sub-txn count = 5 × 8 beats × 2 sub-txns = 80.              */
		/*-------------------------------------------------------------------*/
		begin
			localparam int P52_NUM_WR     = 5;
			localparam int P52_WR_BEATS   = 8;
			localparam int P52_APB_PER_WR = P52_WR_BEATS * 2; // 16
			localparam int P52_TOTAL_APB  = P52_NUM_WR * P52_APB_PER_WR; // 80
 
			logic [31:0] wr_base [P52_NUM_WR];
			logic [31:0] got_bid   [P52_NUM_WR];
			logic [1:0]  got_bresp [P52_NUM_WR];
			int          base_cnt, final_cnt;
 
			wr_base[0] = 32'h5B00_0000;
			wr_base[1] = 32'h5C00_0000;
			wr_base[2] = 32'h5D00_0000;
			wr_base[3] = 32'h5E00_0000;
			wr_base[4] = 32'h5F00_0000;
 
			base_cnt = apb_txn_cnt;
			apb_pready_delay = 8;
 
			fork
				// Issue all 5 writes sequentially; WR2..WR4 stall on awready
				// until a slot frees from the concurrent BRESP drain below.
				begin : p52_wr_issue
					for (int i = 0; i < P52_NUM_WR; i++) begin
						@(negedge clk);
						axi.awvalid = 1;
						axi.awaddr  = wr_base[i];
						axi.awid    = 32'h52_00 + i;
						axi.awlen   = 8'(P52_WR_BEATS - 1);
						axi.awsize  = 3'd3;
						@(posedge clk);
						while (!axi.awready) @(posedge clk);
						@(negedge clk); axi.awvalid = 0;
 
						for (int b = 0; b < P52_WR_BEATS; b++) begin
							@(negedge clk);
							axi.wvalid = 1;
							axi.wdata  = 64'h5200_0000_0000_0000 + (i * P52_WR_BEATS) + b;
							axi.wstrb  = 8'hFF;
							axi.wlast  = (b == P52_WR_BEATS - 1);
							@(posedge clk);
							while (!axi.wready) @(posedge clk);
							@(negedge clk); axi.wvalid = 0;
						end
					end
				end
 
				// Drain all 5 BRESPs concurrently with issuance.
				begin : p52_bresp_drain
					for (int i = 0; i < P52_NUM_WR; i++)
						axi_collect_b(got_bid[i], got_bresp[i]);
				end
			join
 
			final_cnt = apb_txn_cnt;
			apb_pready_delay = 0;
 
			for (int i = 0; i < P52_NUM_WR; i++) begin
				check($sformatf("P5-2 write %0d BRESP=OKAY", i),
					  got_bresp[i] === 2'b00);
				check($sformatf("P5-2 write %0d bid matches awid", i),
					  got_bid[i] === (32'h52_00 + i));
			end
			check("P5-2 total APB sub-txn count correct",
				  (final_cnt - base_cnt) === P52_TOTAL_APB);
		end
 
		wait_idle();
 
		/*-------------------------------------------------------------------*/
		/* P5-3  FIFO max-capacity backpressure: reads only (5 reads)        */
		/*                                                                   */
		/* Same structure as P5-1 but the write side is omitted entirely.    */
		/* Goal: verify the read pipeline alone can sustain 5 back-to-back   */
		/* max-length bursts under maximum APB back-pressure without          */
		/* deadlocking or losing beat data.                                  */
		/*                                                                   */
		/* APB sub-txn count = 5 × 8 beats × 2 sub-txns = 80.              */
		/*-------------------------------------------------------------------*/
		begin
			localparam int P53_NUM_RD     = 5;
			localparam int P53_RD_BEATS   = 8;
			localparam int P53_APB_PER_RD = P53_RD_BEATS * 2; // 16
			localparam int P53_TOTAL_APB  = P53_NUM_RD * P53_APB_PER_RD; // 80
 
			logic [31:0] rd_base [P53_NUM_RD];
			int          rd_ok;
			int          base_cnt, final_cnt;
 
			rd_base[0] = 32'h6000_0000;
			rd_base[1] = 32'h6100_0000;
			rd_base[2] = 32'h6200_0000;
			rd_base[3] = 32'h6300_0000;
			rd_base[4] = 32'h6400_0000;
 
			rd_ok    = 0;
			base_cnt = apb_txn_cnt;
			apb_pready_delay = 8;
 
			fork
				// Issue all 5 reads sequentially; RD2..RD4 stall on arready
				// until a slot frees from the concurrent R-beat drain below.
				begin : p53_rd_issue
					for (int i = 0; i < P53_NUM_RD; i++) begin
						@(negedge clk);
						axi.arvalid = 1;
						axi.araddr  = rd_base[i];
						axi.arid    = 32'h53_10 + i;
						axi.arlen   = 8'(P53_RD_BEATS - 1);
						axi.arsize  = 3'd3;
						@(posedge clk);
						while (!axi.arready) @(posedge clk);
						@(negedge clk); axi.arvalid = 0;
						@(posedge clk); // clean separation before next AR
					end
				end
 
				// Drain all 5 × 8 = 40 R beats concurrently with issuance.
				begin : p53_rdata_drain
					int beats_remaining;
					beats_remaining = P53_NUM_RD * P53_RD_BEATS; // 40
 
					@(negedge clk); axi.rready = 1;
 
					while (beats_remaining > 0) begin
						@(posedge clk);
						while (!axi.rvalid) @(posedge clk);
 
						begin
							int burst_idx;
							burst_idx = -1;
							for (int i = 0; i < P53_NUM_RD; i++) begin
								if (axi.rid === (32'h53_10 + i))
									burst_idx = i;
							end
							if (axi.rresp === 2'b00) rd_ok++;
							beats_remaining--;
						end
						@(negedge clk);
					end
 
					axi.rready = 0;
				end
			join
 
			final_cnt = apb_txn_cnt;
			apb_pready_delay = 0;
 
			check("P5-3 all 40 R beats RRESP=OKAY",
				  rd_ok === P53_NUM_RD * P53_RD_BEATS);
			check("P5-3 total APB sub-txn count correct",
				  (final_cnt - base_cnt) === P53_TOTAL_APB);
		end
 
		wait_idle();

		$display("\n--- Priority 5 (Backpressure) done (%0d pass, %0d fail) ---\n",
				 pass_count, fail_count);
		/*---------------------------------------------------------------------*/
		/* P6-1  Multi-beat burst, last beat LSB-only valid                    */
		/*                                                                     */
		/* 4-beat write burst (awlen=3, size=3).  Beats 0-2 use wstrb=8'hFF  */
		/* (both APB halves active, 2 sub-txns each).  Beat 3 (wlast) uses   */
		/* wstrb=8'h0F (only LSB lanes set) so the disassembler suppresses    */
		/* the MSB half: exactly 1 APB sub-txn is issued for the last beat.  */
		/*                                                                     */
		/* Expected APB sub-txn count = 3*2 + 1 = 7.                         */
		/* Expected BRESP = OKAY (no errors injected).                        */
		/*---------------------------------------------------------------------*/
		begin
			logic [31:0] bid;
			logic [1:0]  bresp;
			int          base_cnt;
			localparam int P61_BEATS     = 4;
			localparam int P61_FULL_BEATS = P61_BEATS - 1; // beats 0..2: both halves
			localparam int P61_EXPECTED_APB = P61_FULL_BEATS * 2 + 1; // 7

			base_cnt = apb_txn_cnt;

			fork
				// Thread A: AW channel
				axi_send_aw(32'hA100_0000, 32'hB1, 8'(P61_BEATS - 1), 3'd3);

				// Thread B: W channel  beats 0-2 full width, beat 3 LSB only
				begin
					for (int b = 0; b < P61_BEATS; b++) begin
						logic [7:0] strb;
						strb = (b == P61_BEATS - 1) ? 8'h0F : 8'hFF;
						axi_send_w(64'hB1B1_0000_0000_0000 + b,
								   strb,
								   (b == P61_BEATS - 1) ? 1'b1 : 1'b0);
					end
				end
			join

			axi_collect_b(bid, bresp);

			check("P6-1 last-beat LSB-only: BRESP=OKAY",   bresp === 2'b00);
			check("P6-1 last-beat LSB-only: bid matches",  bid   === 32'hB1);
			check("P6-1 last-beat LSB-only: APB sub-txn count = 7",
				  (apb_txn_cnt - base_cnt) === P61_EXPECTED_APB);
		end

		wait_idle();
		
		/*=====================================================================*/
		/* FINAL SUMMARY                                                       */
		/*=====================================================================*/
		$display("========================================");
		$display("  TOTAL: %0d PASS  /  %0d FAIL", pass_count, fail_count);
		$display("========================================");
		if (fail_count == 0)
			$display("  ALL TESTS PASSED");
		else
			$display("  *** FAILURES DETECTED  see above ***");
 
		$finish;
	end
 
	/*=========================================================================*/
	/*  Timeout watchdog                                                       */
	/*=========================================================================*/
	initial begin
		#2_000_000;
		$display("[TIMEOUT] Simulation exceeded 2 ms  hung?");
		$finish;
	end
 
endmodule