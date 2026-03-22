/*------------------------------------------------------------------------------
 * File          : tb.sv
 * Project       : RTL
 * Description   : Burst-focused testbench for the AXI-to-APB bridge.
 *
 * DUT TOPOLOGY
 * ============
 *   axi_master_bfm  ->  top_module  ->  apb_slave_bfm
 *
 *   top_module contains all FIFOs, manager, axi_slave_wr/rd, apb_master.
 *   The testbench drives the AXI master interface and responds on the APB
 *   slave interface, then checks the AXI responses.
 *
 * TEST LIST
 * =========
 *   TEST 1  - Single-beat write  (size=3, 1 beat, OKAY)
 *   TEST 2  - 4-beat write burst (size=3, awlen=3, OKAY)
 *   TEST 3  - Single-beat read   (size=3, 1 beat, OKAY)
 *   TEST 4  - 4-beat read burst  (size=3, arlen=3, OKAY)
 *   TEST 5  - Write SLVERR propagation (one beat returns pslverr=1)
 *   TEST 6  - Read  SLVERR propagation (one half returns pslverr=1)
 *   TEST 7  - Narrow write (size=2, single half per beat)
 *   TEST 8  - Concurrent write + read bursts in both slots simultaneously
 *   TEST 9  - Back-to-back write bursts (no idle gap between them)
 *   TEST 10 - Maximum-length write burst (awlen=7, 8 beats)
 *
 * APB SLAVE MODEL
 * ===============
 *   Responds to every APB request with pready after PREADY_DELAY cycles.
 *   For reads it returns a deterministic rdata = ~paddr (easy to predict).
 *   For SLVERR tests a configurable address range triggers pslverr=1.
 *
 * CHECKING STRATEGY
 * =================
 *   - Every AXI write response (B channel): check bid matches awid, check
 *     bresp is OKAY or SLVERR as expected.
 *   - Every AXI read data beat (R channel): check rid matches arid, check
 *     rdata reconstructed correctly from the two APB halves, check rresp,
 *     check rlast only on the final beat.
 *   - After each test: call check_no_overflow() which verifies no FIFO
 *     overflow or underflow $display messages were emitted (via a simple
 *     flag set by a UVM-style string-intercept task -- see below).
 *   - A global error counter drives the final pass/fail verdict.
 *
 * KNOWN LIMITATIONS
 * =================
 *   - Does not test outstanding-transaction interleaving beyond TEST 8.
 *   - APB slave always grants pready within PREADY_DELAY; no back-pressure
 *     on APB side (that is tested implicitly by the FIFO full/empty logic).
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module tb;

    import struct_types::*;

    /*=========================================================================*/
    /*  Parameters                                                             */
    /*=========================================================================*/

    localparam CLK_PERIOD    = 10;   // ns
    localparam PREADY_DELAY  = 1;    // APB slave pready latency (cycles after penable)
    localparam TIMEOUT_LIMIT = 5000; // max cycles before a wait loop aborts

    // Colour codes for $display readability
    localparam string GRN = "\033[32m";
    localparam string RED = "\033[31m";
    localparam string YEL = "\033[33m";
    localparam string RST = "\033[0m";

    /*=========================================================================*/
    /*  Clock and reset                                                        */
    /*=========================================================================*/

    logic clk  = 0;
    logic rst_n = 0;

    always #(CLK_PERIOD/2) clk = ~clk;

    initial begin
        rst_n = 0;
        repeat(4) @(posedge clk);
        @(negedge clk);
        rst_n = 1;
    end

    /*=========================================================================*/
    /*  Interface instances                                                    */
    /*=========================================================================*/

    axi_interface #(
        .ADDR_WIDTH  (32),
        .DATA_WIDTH  (64),
        .ID_WIDTH    (32),
        .RRESP_WIDTH (2),
        .BRESP_WIDTH (2),
        .MAX_SIZE    (3),
        .MAX_LEN     (8)
    ) axi_if ();

    apb_interface apb_if ();

    /*=========================================================================*/
    /*  DUT                                                                    */
    /*=========================================================================*/

    // Connect bare interface handles -- the modport directions are enforced
    // by top_module's own port declarations (axi_interface.slave, apb_interface.master).
    // Specifying a modport here as well causes a type mismatch in VCS.
    top_module dut (
        .clk   (clk),
        .rst_n (rst_n),
        .axi   (axi_if),
        .apb   (apb_if)
    );

    /*=========================================================================*/
    /*  Global scoreboard                                                      */
    /*=========================================================================*/

    int  error_count = 0;
    int  test_num    = 0;
    string test_name = "";

    // Reference counters for shared ready signals.
    // bready/rready stay asserted as long as at least one task needs them.
    // Each task increments on entry, decrements on exit.
    int  bready_users = 0;
    int  rready_users = 0;

    // SLVERR trigger: APB slave asserts pslverr when paddr matches this value.
    // Set by individual tests; 0 = no error injection.
    logic [31:0] slverr_addr = 0;
    logic        slverr_en   = 0;

    /*=========================================================================*/
    /*  APB slave BFM                                                         */
    /*=========================================================================*/
    // Responds to every APB transaction with pready after PREADY_DELAY cycles.
    // rdata = ~paddr (deterministic, predictable by the checker).
    // pslverr asserted when slverr_en=1 and paddr==slverr_addr.
    //
    // TIMING DISCIPLINE
    // -----------------
    // All outputs are driven with BLOCKING assigns after @(negedge clk).
    // This guarantees the DUT's posedge always_ff blocks see fully settled
    // values and eliminates the NBA ordering race that occurs when both the
    // BFM and apb_master drive interface signals in the same posedge timestep.
    //
    // We latch paddr at the setup-phase negedge so the slverr check always
    // uses the address that was stable when the transaction started.

    logic [31:0] apb_bfm_paddr_lat = '0;

    initial begin
        apb_if.pready  = 0;
        apb_if.prdata  = 0;
        apb_if.pslverr = 0;

        forever begin
            // Sample at negedge: apb_master's posedge-FF outputs are stable here.
            @(negedge clk);
            if (apb_if.psel && !apb_if.penable) begin
                // Setup phase: latch address now while it is stable.
                apb_bfm_paddr_lat = apb_if.paddr;

                // Wait PREADY_DELAY full cycles inside access phase.
                repeat(PREADY_DELAY) @(negedge clk);

                // Drive with blocking assigns: DUT sees these at the next posedge.
                apb_if.pready  = 1;
                apb_if.prdata  = ~apb_bfm_paddr_lat;
                apb_if.pslverr = slverr_en & (apb_bfm_paddr_lat == slverr_addr);

                // Hold for one full cycle, then release.
                @(negedge clk);
                apb_if.pready  = 0;
                apb_if.prdata  = 0;
                apb_if.pslverr = 0;
            end
        end
    end

    /*=========================================================================*/
    /*  AXI master BFM tasks                                                  */
    /*=========================================================================*/

    // -----------------------------------------------------------------------
    // axi_write_burst
    //   Drives one full AXI write burst: AW handshake, then all W beats,
    //   then collects the B response.
    //   wdata_arr  : array of DATA_WIDTH values, one per beat
    //   wstrb_arr  : array of WSTRB_WIDTH values, one per beat
    //   exp_bresp  : expected B-channel response code
    // -----------------------------------------------------------------------
    task automatic axi_write_burst(
        input  logic [31:0] awid,
        input  logic [31:0] awaddr,
        input  logic [7:0]  awlen,
        input  logic [2:0]  awsize,
        input  logic [63:0] wdata_arr [],
        input  logic [7:0]  wstrb_arr [],
        input  logic [1:0]  exp_bresp,
        input  string       label
    );
        int beats = int'(awlen) + 1;
        logic [31:0] got_bid;
        logic [1:0]  got_bresp;
        int          timeout;

        // -- AW channel --
        @(negedge clk);
        axi_if.awvalid = 1;
        axi_if.awid    = awid;
        axi_if.awaddr  = awaddr;
        axi_if.awlen   = awlen;
        axi_if.awsize  = awsize;
        timeout = 0;
        do begin
            @(posedge clk);
            timeout++;
            if (timeout > TIMEOUT_LIMIT) begin
                $display("%s[FAIL]%s %s: AW handshake timeout", RED, RST, label);
                error_count++;
                return;
            end
        end while (!axi_if.awready);
        @(negedge clk);
        axi_if.awvalid = 0;

        // -- W channel (all beats) --
        for (int b = 0; b < beats; b++) begin
            @(negedge clk);
            axi_if.wvalid = 1;
            axi_if.wdata  = wdata_arr[b];
            axi_if.wstrb  = wstrb_arr[b];
            axi_if.wlast  = (b == beats-1);
            timeout = 0;
            do begin
                @(posedge clk);
                timeout++;
                if (timeout > TIMEOUT_LIMIT) begin
                    $display("%s[FAIL]%s %s: W beat %0d handshake timeout", RED, RST, label, b);
                    error_count++;
                    return;
                end
            end while (!axi_if.wready);
        end
        @(negedge clk);
        axi_if.wvalid = 0;
        axi_if.wlast  = 0;

        // -- B channel --
        // Use a reference counter so bready stays high while ANY concurrent
        // task needs a B response.  Each task filters on its own bid.
        @(negedge clk);
        bready_users++;
        axi_if.bready = 1;
        timeout = 0;
        do begin
            @(posedge clk);
            timeout++;
            if (timeout > TIMEOUT_LIMIT) begin
                $display("%s[FAIL]%s %s: B response timeout", RED, RST, label);
                error_count++;
                bready_users--;
                if (bready_users == 0) axi_if.bready = 0;
                return;
            end
        end while (!(axi_if.bvalid && axi_if.bid == awid));
        got_bid   = axi_if.bid;
        got_bresp = axi_if.bresp;
        @(negedge clk);
        bready_users--;
        if (bready_users == 0) axi_if.bready = 0;

        // -- Check --
        if (got_bid !== awid) begin
            $display("%s[FAIL]%s %s: bid=%0h expected %0h", RED, RST, label, got_bid, awid);
            error_count++;
        end
        if (got_bresp !== exp_bresp) begin
            $display("%s[FAIL]%s %s: bresp=%02b expected %02b", RED, RST, label, got_bresp, exp_bresp);
            error_count++;
        end else begin
            $display("%s[PASS]%s %s: bid=%0h bresp=%02b", GRN, RST, label, got_bid, got_bresp);
        end
    endtask

    // -----------------------------------------------------------------------
    // axi_read_burst
    //   Drives one full AXI read burst: AR handshake, then collects all
    //   R beats. Checks rid, rlast, rresp and rdata reconstruction.
    //
    //   Expected rdata per beat: the APB slave returns prdata = ~paddr.
    //   For size=3 (full 64-bit): LSB half uses lsb_addr, MSB uses msb_addr.
    //     rdata[31:0]  = ~lsb_addr  (lower APB word)
    //     rdata[63:32] = ~msb_addr  (upper APB word)
    //   For size<3 (narrow): only one half active; the other is 0.
    //   The task computes expected_rdata from first principles.
    // -----------------------------------------------------------------------
    task automatic axi_read_burst(
        input  logic [31:0] arid,
        input  logic [31:0] araddr,
        input  logic [7:0]  arlen,
        input  logic [2:0]  arsize,
        input  logic [1:0]  exp_rresp [],   // one entry per beat
        input  string       label
    );
        int beats = int'(arlen) + 1;
        logic [63:0] got_rdata;
        logic [1:0]  got_rresp;
        logic [31:0] got_rid;
        logic        got_rlast;
        int          timeout;

        logic [31:0] lsb_addr, msb_addr, beat_addr;
        logic [63:0] exp_rdata;
        logic        full_width;
        logic        all_ok;

        // -- AR channel --
        @(negedge clk);
        axi_if.arvalid = 1;
        axi_if.arid    = arid;
        axi_if.araddr  = araddr;
        axi_if.arlen   = arlen;
        axi_if.arsize  = arsize;
        timeout = 0;
        do begin
            @(posedge clk);
            timeout++;
            if (timeout > TIMEOUT_LIMIT) begin
                $display("%s[FAIL]%s %s: AR handshake timeout", RED, RST, label);
                error_count++;
                return;
            end
        end while (!axi_if.arready);
        @(negedge clk);
        axi_if.arvalid = 0;

        // -- R channel (all beats) --
        // Use a reference counter so rready stays high while ANY concurrent
        // task needs R beats.  Each task filters on its own rid.
        @(negedge clk);
        rready_users++;
        axi_if.rready = 1;
        all_ok = 1;

        for (int b = 0; b < beats; b++) begin
            timeout = 0;
            do begin
                @(posedge clk);
                timeout++;
                if (timeout > TIMEOUT_LIMIT) begin
                    $display("%s[FAIL]%s %s: R beat %0d timeout", RED, RST, label, b);
                    error_count++;
                    axi_if.rready = 0;
                    return;
                end
            end while (!(axi_if.rvalid && axi_if.rid == arid));

            got_rid   = axi_if.rid;
            got_rdata = axi_if.rdata;
            got_rresp = axi_if.rresp;
            got_rlast = axi_if.rlast;

            // Compute expected rdata from beat address (mirror disassembler logic)
            beat_addr  = araddr + (32'(b) << arsize);
            full_width = (arsize == 3'd3);
            lsb_addr   = {beat_addr[31:3], 3'b000};
            msb_addr   = {beat_addr[31:3], 3'b100};

            // APB slave: prdata = ~paddr (32-bit)
            // assembler: rdata[31:0]=lsb_prdata, rdata[63:32]=msb_prdata
            // For narrow reads only one half is valid; the other is 0.
            if (full_width) begin
                exp_rdata = {~msb_addr, ~lsb_addr};
            end else begin
                if (beat_addr[2] == 0)
                    exp_rdata = {32'h0, ~lsb_addr};
                else
                    exp_rdata = {~msb_addr, 32'h0};
            end

            // -- Checks --
            if (got_rid !== arid) begin
                $display("%s[FAIL]%s %s beat %0d: rid=%0h expected %0h",
                    RED, RST, label, b, got_rid, arid);
                error_count++; all_ok = 0;
            end
            if (got_rresp !== exp_rresp[b]) begin
                $display("%s[FAIL]%s %s beat %0d: rresp=%02b expected %02b",
                    RED, RST, label, b, got_rresp, exp_rresp[b]);
                error_count++; all_ok = 0;
            end
            // Only check rdata when OKAY is expected for this beat
            if (exp_rresp[b] == 2'b00 && got_rdata !== exp_rdata) begin
                $display("%s[FAIL]%s %s beat %0d: rdata=%016h expected %016h",
                    RED, RST, label, b, got_rdata, exp_rdata);
                error_count++; all_ok = 0;
            end
            if (got_rlast !== (b == beats-1)) begin
                $display("%s[FAIL]%s %s beat %0d: rlast=%0b expected %0b",
                    RED, RST, label, b, got_rlast, (b == beats-1));
                error_count++; all_ok = 0;
            end

            @(negedge clk);
        end
        rready_users--;
        if (rready_users == 0) axi_if.rready = 0;

        if (all_ok)
            $display("%s[PASS]%s %s: all %0d beats OK  rid=%0h",
                GRN, RST, label, beats, got_rid);
    endtask

    /*=========================================================================*/
    /*  Idle-driver: keep AXI master signals quiet between transactions       */
    /*=========================================================================*/

    initial begin
        axi_if.awvalid = 0; axi_if.awid = 0; axi_if.awaddr = 0;
        axi_if.awlen   = 0; axi_if.awsize = 0;
        axi_if.wvalid  = 0; axi_if.wdata = 0; axi_if.wstrb = 0; axi_if.wlast = 0;
        axi_if.bready  = 0;
        axi_if.arvalid = 0; axi_if.arid = 0; axi_if.araddr = 0;
        axi_if.arlen   = 0; axi_if.arsize = 0;
        axi_if.rready  = 0;
    end

    /*=========================================================================*/
    /*  SLVERR path probe                                                      */
    /*  Traces every stage of the pslverr->bresp pipeline each posedge.       */
    /*  Only prints when something interesting is non-zero so the log stays   */
    /*  readable.  Disable by setting SLVERR_PROBE=0.                         */
    /*=========================================================================*/

    localparam SLVERR_PROBE = 1;

    generate if (SLVERR_PROBE) begin : gen_probe
        always @(posedge clk) begin
            // 1. APB interface outputs from BFM
            if (apb_if.psel && apb_if.penable && apb_if.pready)
                $display("[%0t] APB_CAPTURE  paddr=%08h pwrite=%0b pslverr=%0b prdata=%08h",
                    $time,
                    apb_if.paddr, apb_if.pwrite,
                    apb_if.pslverr, apb_if.prdata);

            // 2. APB response FIFO push (inside top_module - access via hierarchical ref)
            if (!dut.apb_resp_push_n)
                $display("[%0t] APB_RESP_PUSH paddr=%08h pslverr=%0b",
                    $time,
                    dut.apb_resp_fifo_in.paddr,
                    dut.apb_resp_fifo_in.pslverr);

            // 3. APB response FIFO pop + tag (manager consuming the response)
            if (!dut.apb_resp_pop_n)
                $display("[%0t] APB_RESP_POP  pslverr=%0b  tag={slot=%0b is_msb=%0b}",
                    $time,
                    dut.apb_resp_fifo_out.pslverr,
                    dut.apb_tag_fifo_out.slot,
                    dut.apb_tag_fifo_out.is_msb);

            // 4. Manager slot response accumulation
            for (int s = 0; s < 2; s++) begin
                if (dut.u_manager.slot_lsb_valid[s] || dut.u_manager.slot_msb_valid[s])
                    $display("[%0t] SLOT[%0d]_RESP  lsb_valid=%0b lsb_pslverr=%0b  msb_valid=%0b msb_pslverr=%0b  burst_err=%0b",
                        $time, s,
                        dut.u_manager.slot_lsb_valid[s],
                        dut.u_manager.slot_lsb_pslverr[s],
                        dut.u_manager.slot_msb_valid[s],
                        dut.u_manager.slot_msb_pslverr[s],
                        dut.u_manager.slot_wr_burst_err[s]);
            end

            // 5. Register file write enable (assembled response going into rf)
            if (dut.u_manager.rf_enable_wr)
                $display("[%0t] RF_ENABLE_WR   bresp=%02b  burst_err_at_push=%0b",
                    $time,
                    dut.u_manager.rf_axi_wr_resp_in.bresp,
                    dut.u_manager.slot_wr_burst_err[dut.u_manager.asm_slot]);

            // 6. AXI write response FIFO push
            if (!dut.wr_resp_push_n)
                $display("[%0t] WR_RESP_PUSH   bid=%0h bresp=%02b",
                    $time,
                    dut.wr_resp_fifo_in.bid,
                    dut.wr_resp_fifo_in.bresp);
        end
    end endgenerate

    /*=========================================================================*/
    /*  Write-write collision probe  (enhanced)                              */
    /*=========================================================================*/

    always @(posedge clk) begin
        if (rst_n) begin
            if (dut.u_manager.wr_collision &&
                !$past(dut.u_manager.wr_collision))
                $display("[%0t] COLLISION ASSERTED  inc=[%08h..%08h] CL[%0d..%0d]  slot0=[%08h..%08h] CL[%0d..%0d]  slot0_state=%0s",
                    $time,
                    dut.u_manager.wr_req_fifo_out.awaddr,
                    dut.u_manager.wr_req_fifo_out.awaddr
                        + ((dut.u_manager.wr_req_fifo_out.awlen+1) << dut.u_manager.wr_req_fifo_out.awsize) - 1,
                    dut.u_manager.inc_cl_start,
                    dut.u_manager.inc_cl_end,
                    dut.u_manager.slot_wr_req[0].awaddr,
                    dut.u_manager.slot_wr_req[0].awaddr
                        + ((dut.u_manager.slot_wr_req[0].awlen+1) << dut.u_manager.slot_wr_req[0].awsize) - 1,
                    dut.u_manager.slot_cl_start[0],
                    dut.u_manager.slot_cl_end[0],
                    dut.u_manager.slot_state[0].name());
            if (!dut.u_manager.wr_collision &&
                $past(dut.u_manager.wr_collision))
                $display("[%0t] COLLISION CLEARED  wr_req_empty=%0b  wr_data_empty=%0b  slot0_state=%0s  slot0_is_wr=%0b  slot0_cl=[%0d..%0d]  inc_cl=[%0d..%0d]",
                    $time,
                    dut.wr_req_empty,
                    dut.wr_data_empty,
                    dut.u_manager.slot_state[0].name(),
                    dut.u_manager.slot_is_wr[0],
                    dut.u_manager.slot_cl_start[0],
                    dut.u_manager.slot_cl_end[0],
                    dut.u_manager.inc_cl_start,
                    dut.u_manager.inc_cl_end);

            // Every cycle during T12: dump key signals
            if (test_num == 12)
                $display("[%0t] T12_STATE  slot0=%0s slot1=%0s  wr_req_empty=%0b wr_data_empty=%0b  wr_collision=%0b  inc_cl=[%0d..%0d]  s0cl=[%0d..%0d]",
                    $time,
                    dut.u_manager.slot_state[0].name(),
                    dut.u_manager.slot_state[1].name(),
                    dut.wr_req_empty,
                    dut.wr_data_empty,
                    dut.u_manager.wr_collision,
                    dut.u_manager.inc_cl_start,
                    dut.u_manager.inc_cl_end,
                    dut.u_manager.slot_cl_start[0],
                    dut.u_manager.slot_cl_end[0]);
        end
    end

    /*=========================================================================*/
    /*  Anti-starvation probe                                                 */
    /*  Prints whenever rd_priority asserts or the counter moves.            */
    /*=========================================================================*/

    always @(posedge clk) begin
        if (rst_n) begin
            if (dut.u_manager.rd_priority &&
                !$past(dut.u_manager.rd_priority))
                $display("[%0t] STARVATION  rd_priority ASSERTED  (cnt reached %0d)",
                    $time, dut.u_manager.WR_STARVE_THRESH);
            if (!dut.u_manager.rd_priority &&
                $past(dut.u_manager.rd_priority))
                $display("[%0t] STARVATION  rd_priority CLEARED (read accepted)",
                    $time);
        end
    end

    /*=========================================================================*/
    /*  Helper: wait for DUT to settle after reset                            */
    /*=========================================================================*/

    task automatic wait_reset_done;
        @(posedge rst_n);
        repeat(2) @(posedge clk);
    endtask

    /*=========================================================================*/
    /*  Helper: small inter-test gap                                          */
    /*=========================================================================*/

    task automatic inter_test_gap;
        repeat(8) @(posedge clk);
    endtask

    /*=========================================================================*/
    /*  TEST STIMULUS                                                          */
    /*=========================================================================*/

    initial begin
        automatic logic [63:0] wdata_arr[];
        automatic logic [7:0]  wstrb_arr[];

        wait_reset_done();

        // =================================================================
        // TEST 1: Single-beat write, size=3 (full 64-bit), OKAY
        // =================================================================
        test_num  = 1;
        test_name = "T1 single-beat write OKAY";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);

        wdata_arr = new[1]; wstrb_arr = new[1];
        wdata_arr[0] = 64'hDEAD_BEEF_CAFE_F00D;
        wstrb_arr[0] = 8'hFF;

        axi_write_burst(
            .awid     (32'h1),
            .awaddr   (32'h0000_1000),
            .awlen    (8'h00),       // 1 beat
            .awsize   (3'd3),        // 8 bytes
            .wdata_arr(wdata_arr),
            .wstrb_arr(wstrb_arr),
            .exp_bresp(2'b00),
            .label    (test_name)
        );

        inter_test_gap();

        // =================================================================
        // TEST 2: 4-beat write burst, size=3, OKAY
        // =================================================================
        test_num  = 2;
        test_name = "T2 4-beat write burst OKAY";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);

        wdata_arr = new[4]; wstrb_arr = new[4];
        for (int i = 0; i < 4; i++) begin
            wdata_arr[i] = {32'hA000_0000 + i, 32'hB000_0000 + i};
            wstrb_arr[i] = 8'hFF;
        end

        axi_write_burst(
            .awid     (32'h2),
            .awaddr   (32'h0000_2000),
            .awlen    (8'h03),       // 4 beats
            .awsize   (3'd3),
            .wdata_arr(wdata_arr),
            .wstrb_arr(wstrb_arr),
            .exp_bresp(2'b00),
            .label    (test_name)
        );

        inter_test_gap();

        // =================================================================
        // TEST 3: Single-beat read, size=3, OKAY
        // =================================================================
        test_num  = 3;
        test_name = "T3 single-beat read OKAY";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);

        axi_read_burst(
            .arid     (32'h3),
            .araddr   (32'h0000_3000),
            .arlen    (8'h00),       // 1 beat
            .arsize   (3'd3),
            .exp_rresp('{2'b00}),
            .label    (test_name)
        );

        inter_test_gap();

        // =================================================================
        // TEST 4: 4-beat read burst, size=3, OKAY
        // =================================================================
        test_num  = 4;
        test_name = "T4 4-beat read burst OKAY";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);

        axi_read_burst(
            .arid     (32'h4),
            .araddr   (32'h0000_4000),
            .arlen    (8'h03),       // 4 beats
            .arsize   (3'd3),
            .exp_rresp('{2'b00, 2'b00, 2'b00, 2'b00}),
            .label    (test_name)
        );

        inter_test_gap();

        // =================================================================
        // TEST 5: Write burst with SLVERR on beat 1 (of 3)
        //         Expected: bresp=SLVERR (2'b10), bid correct
        // =================================================================
        test_num  = 5;
        test_name = "T5 write burst SLVERR propagation";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);

        // Beat 1 MSB address = 0x0000_500C; inject error there
        slverr_addr = 32'h0000_500C;
        slverr_en   = 1;

        wdata_arr = new[3]; wstrb_arr = new[3];
        for (int i = 0; i < 3; i++) begin
            wdata_arr[i] = 64'hFACE_CAFE_0000_0000 + i;
            wstrb_arr[i] = 8'hFF;
        end

        axi_write_burst(
            .awid     (32'h5),
            .awaddr   (32'h0000_5000),
            .awlen    (8'h02),       // 3 beats
            .awsize   (3'd3),
            .wdata_arr(wdata_arr),
            .wstrb_arr(wstrb_arr),
            .exp_bresp(2'b10),       // SLVERR
            .label    (test_name)
        );

        slverr_en = 0; slverr_addr = 0;
        inter_test_gap();

        // =================================================================
        // TEST 6: Read burst with SLVERR on beat 0 LSB
        // =================================================================
        test_num  = 6;
        test_name = "T6 read burst SLVERR propagation";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);

        // Beat 0 LSB address = 0x0000_6000 -> matches slverr_addr -> rresp=SLVERR
        // Beat 1 address    = 0x0000_6008 -> no match              -> rresp=OKAY
        slverr_addr = 32'h0000_6000;
        slverr_en   = 1;
        axi_read_burst(
            .arid     (32'h6),
            .araddr   (32'h0000_6000),
            .arlen    (8'h01),       // 2 beats
            .arsize   (3'd3),
            .exp_rresp('{2'b10, 2'b00}),   // beat 0: SLVERR, beat 1: OKAY
            .label    (test_name)
        );

        slverr_en = 0; slverr_addr = 0;
        inter_test_gap();

        // =================================================================
        // TEST 7: Narrow write burst, size=2 (4 bytes per beat)
        //         Only one APB transaction per beat (address selects half).
        //         Beat 0: addr=0x0000_7000 -> bit[2]=0 -> LSB only
        //         Beat 1: addr=0x0000_7004 -> bit[2]=1 -> MSB only
        // =================================================================
        test_num  = 7;
        test_name = "T7 narrow write burst size=2";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);

        wdata_arr = new[2]; wstrb_arr = new[2];
        // size=2: lower 4 bytes of wdata are used; wstrb[3:0] active
        wdata_arr[0] = 64'h0000_0000_1111_2222;
        wstrb_arr[0] = 8'h0F;  // lower half only
        wdata_arr[1] = 64'h0000_0000_3333_4444;
        wstrb_arr[1] = 8'h0F;

        axi_write_burst(
            .awid     (32'h7),
            .awaddr   (32'h0000_7000),
            .awlen    (8'h01),       // 2 beats
            .awsize   (3'd2),        // 4 bytes per beat (narrow)
            .wdata_arr(wdata_arr),
            .wstrb_arr(wstrb_arr),
            .exp_bresp(2'b00),
            .label    (test_name)
        );

        inter_test_gap();

        // =================================================================
        // TEST 8: Concurrent write + read in both slots
        //   Slot 0: 4-beat write  (started first)
        //   Slot 1: 4-beat read   (started one cycle later, while slot 0 active)
        // =================================================================
        test_num  = 8;
        test_name = "T8 concurrent write+read dual slot";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);

        fork
            begin
                // Slot 0: write burst
                automatic logic [63:0] wd[]; automatic logic [7:0] ws[];
                wd = new[4]; ws = new[4];
                for (int i = 0; i < 4; i++) begin
                    wd[i] = 64'hCAFE_0000_0000_0000 + (64'(i) << 32);
                    ws[i] = 8'hFF;
                end
                axi_write_burst(
                    .awid     (32'hA0),
                    .awaddr   (32'h0000_A000),
                    .awlen    (8'h03),
                    .awsize   (3'd3),
                    .wdata_arr(wd),
                    .wstrb_arr(ws),
                    .exp_bresp(2'b00),
                    .label    ("T8-WR")
                );
            end
            begin
                // Slot 1: read burst, starts 3 cycles later to avoid AW/AR
                // contention on the same clock edge
                repeat(3) @(posedge clk);
                axi_read_burst(
                    .arid     (32'hB0),
                    .araddr   (32'h0000_B000),
                    .arlen    (8'h03),
                    .arsize   (3'd3),
                    .exp_rresp('{2'b00, 2'b00, 2'b00, 2'b00}),
                    .label    ("T8-RD")
                );
            end
        join

        inter_test_gap();

        // =================================================================
        // TEST 9: Back-to-back write bursts (no gap)
        //   Two 2-beat write bursts submitted consecutively; verifies that
        //   after the first burst completes the slot returns to IDLE and
        //   the second burst is accepted.
        // =================================================================
        test_num  = 9;
        test_name = "T9 back-to-back write bursts";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);

        begin
            automatic logic [63:0] wd1[], wd2[];
            automatic logic [7:0]  ws1[], ws2[];
            wd1 = new[2]; ws1 = new[2];
            wd2 = new[2]; ws2 = new[2];
            wd1[0] = 64'h1111_1111_1111_1111; ws1[0] = 8'hFF;
            wd1[1] = 64'h2222_2222_2222_2222; ws1[1] = 8'hFF;
            wd2[0] = 64'h3333_3333_3333_3333; ws2[0] = 8'hFF;
            wd2[1] = 64'h4444_4444_4444_4444; ws2[1] = 8'hFF;

            axi_write_burst(32'hC1, 32'h0000_C100, 8'h01, 3'd3, wd1, ws1, 2'b00, "T9-WR1");
            // No inter_test_gap -- submit immediately
            axi_write_burst(32'hC2, 32'h0000_C200, 8'h01, 3'd3, wd2, ws2, 2'b00, "T9-WR2");
        end

        inter_test_gap();

        // =================================================================
        // TEST 10: Maximum-length write burst (awlen=7, 8 beats)
        // =================================================================
        test_num  = 10;
        test_name = "T10 max-length write burst (8 beats)";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);

        wdata_arr = new[8]; wstrb_arr = new[8];
        for (int i = 0; i < 8; i++) begin
            wdata_arr[i] = {32'hFEED_0000 + i, 32'hDEAD_0000 + i};
            wstrb_arr[i] = 8'hFF;
        end

        axi_write_burst(
            .awid     (32'hD0),
            .awaddr   (32'h0000_D000),
            .awlen    (8'h07),       // 8 beats
            .awsize   (3'd3),
            .wdata_arr(wdata_arr),
            .wstrb_arr(wstrb_arr),
            .exp_bresp(2'b00),
            .label    (test_name)
        );

        inter_test_gap();

        // =================================================================
        // TEST 11: Anti-starvation stress test
        //   Fire 8 consecutive write bursts while keeping a read request
        //   continuously pending in the AR FIFO.  With WR_STARVE_THRESH=4
        //   the read must be serviced before all 8 writes complete.
        //   We check: (a) the read completes at all, (b) rd_priority was
        //   observed asserting at least once (via the $display probe), and
        //   (c) all write responses come back OKAY.
        //
        //   The write bursts are launched in a fork alongside a single read
        //   burst so they run concurrently.  The read task will timeout if
        //   it is never granted a slot.
        // =================================================================
        test_num  = 11;
        test_name = "T11 anti-starvation stress";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);

        fork
            // -- 6 back-to-back write bursts (each 2 beats) --
            begin
                for (int w = 0; w < 6; w++) begin
                    automatic logic [63:0] wd[];
                    automatic logic [7:0]  ws[];
                    automatic logic [31:0] wid   = 32'hE000 + w;
                    automatic logic [31:0] waddr = 32'hE000_0000 + (w << 4);
                    automatic string       wlabel;
                    wd = new[2]; ws = new[2];
                    wd[0] = 64'hAAAA_BBBB_0000_0000 + w;  ws[0] = 8'hFF;
                    wd[1] = 64'hCCCC_DDDD_0000_0000 + w;  ws[1] = 8'hFF;
                    wlabel = $sformatf("T11-WR%0d", w);
                    axi_write_burst(wid, waddr, 8'h01, 3'd3, wd, ws,
                                    2'b00, wlabel);
                end
            end
            // -- Single read burst launched immediately --
            // The AXI slave accepts the AR as soon as arready is high;
            // the slot may not be granted until rd_priority fires.
            begin
                automatic logic [1:0] rresp_arr[];
                rresp_arr = new[2];
                rresp_arr[0] = 2'b00; rresp_arr[1] = 2'b00;
                axi_read_burst(32'hF000, 32'hF000_0000, 8'h01, 3'd3,
                               rresp_arr, "T11-RD");
            end
        join

        inter_test_gap();

        // =================================================================
        // TEST 12: Write-write cache-line collision
        //   Two write bursts to the same cache line (64-byte = 8 AXI beats).
        //   Both start with awaddr in the same 64-byte region.
        //   Expected: they run serially -- B response for the first arrives
        //   before the second even gets a slot (verified by checking that
        //   the second write's APB transactions only start after the first
        //   B response is sent).
        //
        // TEST 13: Write-write no collision (different cache lines)
        //   Two write bursts to different 64-byte cache lines.
        //   Expected: they interleave freely on APB, both complete.
        //
        // TEST 14: Write-read same cache line -- must NOT stall
        //   A write and a read to the same cache line run concurrently.
        //   Expected: no stall, both complete (collision only blocks WaW).
        // =================================================================

        // ---- T12: WaW collision ----
        // Submit sequentially from the TB -- the collision is exercised at
        // the manager level since both AW+W requests land in the FIFOs with
        // no gap and the manager must hold WR2 back via wr_collision.
        // Forking is avoided because both tasks would drive the shared
        // W-channel signals (wvalid/wdata/wstrb/wlast) simultaneously.
        test_num  = 12;
        test_name = "T12 WaW cache-line collision (serial)";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);
        begin
            automatic logic [63:0] wd1[], wd2[];
            automatic logic [7:0]  ws1[], ws2[];
            wd1 = new[2]; ws1 = new[2];
            wd2 = new[2]; ws2 = new[2];
            for (int i = 0; i < 2; i++) begin
                wd1[i] = 64'h1111_0000_0000_0000 + i; ws1[i] = 8'hFF;
                wd2[i] = 64'h2222_0000_0000_0000 + i; ws2[i] = 8'hFF;
            end
            axi_write_burst(32'h10, 32'h1000_0000, 8'h01, 3'd3,
                            wd1, ws1, 2'b00, "T12-WR1");
            axi_write_burst(32'h11, 32'h1000_0008, 8'h01, 3'd3,
                            wd2, ws2, 2'b00, "T12-WR2");
        end

        inter_test_gap();

        // ---- T13: WaW no collision (different cache lines) ----
        // Same approach: sequential submission.  With different CLs the
        // manager assigns slot 1 immediately (no collision), so WR2 proceeds
        // to APB in parallel with WR1 -- verified by interleaved APB_CAPTURE.
        test_num  = 13;
        test_name = "T13 WaW different cache lines (concurrent)";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);
        begin
            automatic logic [63:0] wd1[], wd2[];
            automatic logic [7:0]  ws1[], ws2[];
            wd1 = new[2]; ws1 = new[2];
            wd2 = new[2]; ws2 = new[2];
            for (int i = 0; i < 2; i++) begin
                wd1[i] = 64'h3333_0000_0000_0000 + i; ws1[i] = 8'hFF;
                wd2[i] = 64'h4444_0000_0000_0000 + i; ws2[i] = 8'hFF;
            end
            axi_write_burst(32'h20, 32'h2000_0000, 8'h01, 3'd3,
                            wd1, ws1, 2'b00, "T13-WR1");
            axi_write_burst(32'h21, 32'h2000_0040, 8'h01, 3'd3,
                            wd2, ws2, 2'b00, "T13-WR2");
        end

        inter_test_gap();

        // ---- T14: Write + read same cache line (no stall) ----
        test_num  = 14;
        test_name = "T14 WaR same cache line (no stall)";
        $display("\n%s--- %s ---%s", YEL, test_name, RST);
        begin
            automatic logic [63:0] wd[];
            automatic logic [7:0]  ws[];
            automatic logic [1:0]  rresp[];
            wd = new[2]; ws = new[2]; rresp = new[2];
            for (int i = 0; i < 2; i++) begin
                wd[i] = 64'h5555_0000_0000_0000 + i; ws[i] = 8'hFF;
                rresp[i] = 2'b00;
            end
            fork
                axi_write_burst(32'h30, 32'h3000_0000, 8'h01, 3'd3,
                                wd, ws, 2'b00, "T14-WR");
                begin
                    repeat(2) @(posedge clk);
                    // Read to same cache line -- must not be blocked
                    axi_read_burst(32'h31, 32'h3000_0008, 8'h01, 3'd3,
                                   rresp, "T14-RD");
                end
            join
        end

        inter_test_gap();

        // =================================================================
        // Final verdict
        // =================================================================
        repeat(4) @(posedge clk);
        $display("\n========================================");
        if (error_count == 0)
            $display("%s ALL TESTS PASSED %s", GRN, RST);
        else
            $display("%s %0d ERRORS DETECTED %s", RED, error_count, RST);
        $display("========================================\n");

        $finish;
    end

    /*=========================================================================*/
    /*  Watchdog: abort simulation if it hangs completely                     */
    /*=========================================================================*/

    initial begin
        #(CLK_PERIOD * 100_000);
        $display("%s[ABORT]%s Global watchdog timeout -- simulation hung", RED, RST);
        $finish;
    end

    /*=========================================================================*/
    /*  Optional waveform dump                                                */
    /*=========================================================================*/

    initial begin
        $dumpfile("tb.vcd");
        $dumpvars(0, tb);
    end

endmodule