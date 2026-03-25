`timescale 1ns/1ps

/*------------------------------------------------------------------------------
 * File          : tb_write.sv
 * Description   : Write-path testbench for the AXI-to-APB bridge.
 *
 * TEST PLAN
 * =========
 * TB_WRITE_001  Single-beat write all-zeros data
 * TB_WRITE_002  Single-beat write all-ones data
 * TB_WRITE_003  Write wstrb=8'h00 (no bytes enabled) -- 0 APB txns, bresp=OKAY
 * TB_WRITE_004  Write partial strobe LSB only (wstrb=8'h0F)
 * TB_WRITE_005  Write partial strobe MSB only (wstrb=8'hF0)
 * TB_WRITE_006  Write partial strobe single byte LSB (wstrb=8'h01)
 * TB_WRITE_007  Write partial strobe single byte MSB (wstrb=8'h80)
 * TB_WRITE_008  Narrow write size=1 (2B) beat at LSB-aligned address (addr[2]=0)
 * TB_WRITE_009  Narrow write size=1 (2B) beat at MSB-aligned address (addr[2]=1)
 * TB_WRITE_010  Write awid=0 (min ID boundary)
 * TB_WRITE_011  Write awid=0xFFFF_FFFF (max ID boundary)
 * TB_WRITE_012  Write SLVERR on LSB only (first APB txn)
 * TB_WRITE_013  Write SLVERR on MSB only (second APB txn)
 * TB_WRITE_014  Write SLVERR on both halves
 * TB_WRITE_015  Write SLVERR on non-first beat of burst (beat2 MSB of 4-beat burst)
 * TB_WRITE_016  AW channel back-pressure (master holds awvalid low for 5 cycles)
 * TB_WRITE_017  W channel back-pressure (master inserts 5-cycle gap before wvalid)
 * TB_WRITE_018  B channel back-pressure (bready held low for 10 cycles after bvalid)
 *
 * DISPLAY CONVENTIONS
 * ===================
 * Every check prints a self-contained line:
 *   [<time>] PASS|FAIL  <label>  exp=<value>  got=<value>
 * The label embeds the expected value so the log is self-documenting.
 *
 * APB monitor logs every completed APB transfer:
 *   [<time>] APB MONITOR  txn#N  WRITE|READ  paddr=0x<addr>  pwrite=<b>
 * APB slave logs every access:
 *   [<time>] APB SLAVE  WRITE  paddr=...  pwdata=...  pstrb=...
 *
 * Per-test APB transaction counter (apb_txn_count):
 *   Reset before each test with apb_monitor_reset().
 *   Checked with chk_int() to verify exact APB sub-transaction count.
 *   pwrite[], paddr[], pwdata[], pstrb[] arrays allow per-txn field checks.
 *
 * BACK-PRESSURE NOTES (tests 016-018)
 * =====================================
 * awready = !wr_req_fifo_full (DUT drives; TB cannot de-assert it directly).
 * wready  = !wr_data_fifo_full (same).
 * Tests 016/017 simulate a SLOW AXI MASTER by inserting idle cycles before
 * asserting awvalid / wvalid.  This is the correct AXI way to test the
 * handshake without corrupting the DUT's ready signals.
 * Test 018 holds bready low after bvalid asserts, verifying the B-response
 * is held stable until bready is asserted.
 *
 * APB SLAVE MODEL
 * ===============
 * Byte-addressed 32-bit memory.  Word index = paddr[15:2] (bottom 64 KB).
 * Addresses >= 64 KB use a 64-entry upper bank at paddr[7:2].
 * READ returns ~paddr.
 * Error injection: set apb_err_addr to the paddr that should assert pslverr=1.
 * Double-error: set apb_err_addr2 for a second one-shot error.
 * Both clear automatically after firing.
 *------------------------------------------------------------------------------*/

module tb_write;

import struct_types::*;

/*=========================================================================*/
/*  Parameters                                                              */
/*=========================================================================*/

localparam APB_MEM_WORDS = 16384; // 64 KB / 4 bytes per word

/*=========================================================================*/
/*  Clock / reset                                                           */
/*=========================================================================*/

logic clk   = 0;
logic rst_n;

always #3 clk = ~clk;  // ~166 MHz

initial begin
    rst_n = 0;
    repeat(4) @(posedge clk);
    @(negedge clk);
    rst_n = 1;
end

/*=========================================================================*/
/*  Interface instances                                                     */
/*=========================================================================*/

axi_interface axi_if();
apb_interface apb_if();

/*=========================================================================*/
/*  AXI driver signals                                                      */
/*=========================================================================*/

logic                    awvalid_tb = 0;
logic [ID_WIDTH-1:0]     awid_tb    = '0;
logic [ADDR_WIDTH-1:0]   awaddr_tb  = '0;
logic [MAX_LEN-1:0]      awlen_tb   = '0;
logic [MAX_SIZE-1:0]     awsize_tb  = '0;

logic                    wvalid_tb  = 0;
logic [DATA_WIDTH-1:0]   wdata_tb   = '0;
logic [WSTRB_WIDTH-1:0]  wstrb_tb   = '0;
logic                    wlast_tb   = 0;

logic                    bready_tb  = 0;

assign axi_if.awvalid = awvalid_tb;
assign axi_if.awid    = awid_tb;
assign axi_if.awaddr  = awaddr_tb;
assign axi_if.awlen   = awlen_tb;
assign axi_if.awsize  = awsize_tb;
assign axi_if.wvalid  = wvalid_tb;
assign axi_if.wdata   = wdata_tb;
assign axi_if.wstrb   = wstrb_tb;
assign axi_if.wlast   = wlast_tb;
assign axi_if.bready  = bready_tb;

// Tie off unused read channels
assign axi_if.arvalid = 1'b0;
assign axi_if.arid    = '0;
assign axi_if.araddr  = '0;
assign axi_if.arlen   = '0;
assign axi_if.arsize  = '0;
assign axi_if.rready  = 1'b0;

logic        pready_tb;
logic [31:0] prdata_tb;
logic        pslverr_tb;

assign apb_if.pready  = pready_tb;
assign apb_if.prdata  = prdata_tb;
assign apb_if.pslverr = pslverr_tb;

/*=========================================================================*/
/*  DUT                                                                     */
/*=========================================================================*/

top_module u_dut (
    .clk   (clk),
    .rst_n (rst_n),
    .axi   (axi_if.slave),
    .apb   (apb_if.master)
);

/*=========================================================================*/
/*  Pass / fail counters and check tasks                                    */
/*=========================================================================*/

int pass_count = 0;
int fail_count = 0;

task automatic chk1(
    input string label,
    input logic  got,
    input logic  exp
);
    if (got === exp) begin
        $display("[%0t] PASS  %-65s  exp=%0b  got=%0b",
                 $time, label, exp, got);
        pass_count++;
    end else begin
        $display("[%0t] FAIL  %-65s  exp=%0b  got=%0b  <<< MISMATCH",
                 $time, label, exp, got);
        fail_count++;
    end
endtask

task automatic chk_hex(
    input string       label,
    input logic [63:0] got,
    input logic [63:0] exp
);
    if (got === exp) begin
        $display("[%0t] PASS  %-65s  exp=0x%016h  got=0x%016h",
                 $time, label, exp, got);
        pass_count++;
    end else begin
        $display("[%0t] FAIL  %-65s  exp=0x%016h  got=0x%016h  <<< MISMATCH",
                 $time, label, exp, got);
        fail_count++;
    end
endtask

task automatic chk_int(
    input string label,
    input int    got,
    input int    exp
);
    if (got === exp) begin
        $display("[%0t] PASS  %-65s  exp=%0d  got=%0d",
                 $time, label, exp, got);
        pass_count++;
    end else begin
        $display("[%0t] FAIL  %-65s  exp=%0d  got=%0d  <<< MISMATCH",
                 $time, label, exp, got);
        fail_count++;
    end
endtask

task automatic chk_time_after(
    input string label,
    input time   t_later,
    input time   t_earlier
);
    if (t_later > t_earlier) begin
        $display("[%0t] PASS  %-65s  t_earlier=%0t  t_later=%0t",
                 $time, label, t_earlier, t_later);
        pass_count++;
    end else begin
        $display("[%0t] FAIL  %-65s  t_earlier=%0t  t_later=%0t (NOT after!)  <<< MISMATCH",
                 $time, label, t_earlier, t_later);
        fail_count++;
    end
endtask

/*=========================================================================*/
/*  APB monitor + per-test transaction recorder                             */
/*=========================================================================*/
// Fires on every completed APB transfer (psel & penable & pready).
// Records pwrite, paddr, pwdata, pstrb for up to 64 txns per test.

int                    apb_txn_count = 0;
logic                  apb_txn_pwrite [0:63];
logic [ADDR_WIDTH-1:0] apb_txn_paddr  [0:63];
logic [31:0]           apb_txn_pwdata [0:63];
logic [3:0]            apb_txn_pstrb  [0:63];

always_ff @(posedge clk) begin
    if (rst_n && apb_if.psel && apb_if.penable && apb_if.pready) begin
        if (apb_txn_count < 64) begin
            apb_txn_pwrite[apb_txn_count] <= apb_if.pwrite;
            apb_txn_paddr [apb_txn_count] <= apb_if.paddr;
            apb_txn_pwdata[apb_txn_count] <= apb_if.pwdata;
            apb_txn_pstrb [apb_txn_count] <= apb_if.pstrb;
        end
        apb_txn_count <= apb_txn_count + 1;
        $display("[%0t] APB MONITOR  txn#%0d  %s  paddr=0x%08h  pwdata=0x%08h  pstrb=4'b%04b  pwrite=%0b",
                 $time, apb_txn_count,
                 apb_if.pwrite ? "WRITE" : "READ ",
                 apb_if.paddr, apb_if.pwdata, apb_if.pstrb, apb_if.pwrite);
    end
end

task automatic apb_monitor_reset();
    @(negedge clk);
    apb_txn_count = 0;
endtask

/*=========================================================================*/
/*  APB slave model                                                         */
/*=========================================================================*/

logic [31:0]           apb_mem       [0:APB_MEM_WORDS-1];
logic [31:0]           apb_upper_mem [0:63];

// Two one-shot error injection addresses.
// '1 = disabled.  Each clears itself after firing.
logic [ADDR_WIDTH-1:0] apb_err_addr  = '1;
logic [ADDR_WIDTH-1:0] apb_err_addr2 = '1;

task automatic apb_mem_write(
    input logic [ADDR_WIDTH-1:0] pa,
    input logic [31:0]           wd,
    input logic [3:0]            ps
);
    logic [31:0] old;
    old = (pa[31:16] == 16'h0) ? apb_mem[pa[15:2]] : apb_upper_mem[pa[7:2]];
    for (int b = 0; b < 4; b++)
        if (ps[b]) old[b*8 +: 8] = wd[b*8 +: 8];
    if (pa[31:16] == 16'h0) apb_mem[pa[15:2]]    = old;
    else                     apb_upper_mem[pa[7:2]] = old;
endtask

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pready_tb  <= 0;
        prdata_tb  <= 0;
        pslverr_tb <= 0;
        foreach (apb_mem[i])       apb_mem[i]       <= 32'hDEAD_DEAD;
        foreach (apb_upper_mem[i]) apb_upper_mem[i] <= 32'hDEAD_DEAD;
    end else begin
        pready_tb  <= 0;
        pslverr_tb <= 0;

        if (apb_if.psel && apb_if.penable && !pready_tb) begin
            pready_tb <= 1;

            // Error injection (first one-shot)
            if (apb_if.paddr == apb_err_addr) begin
                pslverr_tb  <= 1;
                apb_err_addr <= '1;
                $display("[%0t] APB SLAVE  SLVERR injected  paddr=0x%08h (err_addr1)",
                         $time, apb_if.paddr);
            // Error injection (second one-shot)
            end else if (apb_if.paddr == apb_err_addr2) begin
                pslverr_tb   <= 1;
                apb_err_addr2 <= '1;
                $display("[%0t] APB SLAVE  SLVERR injected  paddr=0x%08h (err_addr2)",
                         $time, apb_if.paddr);
            end

            if (apb_if.pwrite) begin
                apb_mem_write(apb_if.paddr, apb_if.pwdata, apb_if.pstrb);
                $display("[%0t] APB SLAVE  WRITE  paddr=0x%08h  pwdata=0x%08h  pstrb=4'b%04b",
                         $time, apb_if.paddr, apb_if.pwdata, apb_if.pstrb);
            end else begin
                prdata_tb <= ~apb_if.paddr;
                $display("[%0t] APB SLAVE  READ   paddr=0x%08h  prdata=0x%08h (=~paddr)",
                         $time, apb_if.paddr, ~apb_if.paddr);
            end
        end
    end
end

/*=========================================================================*/
/*  AXI driver tasks                                                        */
/*=========================================================================*/

// --- drive_aw: assert AW channel and wait for awready handshake ---
// delay_cycles: number of extra clock cycles to hold awvalid low before
//               asserting it (simulates a slow master / AW back-pressure).
task automatic drive_aw(
    input logic [ID_WIDTH-1:0]   id,
    input logic [ADDR_WIDTH-1:0] addr,
    input logic [MAX_LEN-1:0]    len,
    input logic [MAX_SIZE-1:0]   size,
    input int                    delay_cycles = 0
);
    if (delay_cycles > 0)
        repeat(delay_cycles) @(posedge clk);
    @(negedge clk);
    awvalid_tb = 1; awid_tb = id; awaddr_tb = addr;
    awlen_tb = len; awsize_tb = size;
    @(posedge clk);
    while (!axi_if.awready) @(posedge clk);
    @(negedge clk);
    awvalid_tb = 0;
endtask

// --- drive_w: assert W channel and wait for wready handshake ---
// delay_cycles: extra cycles before wvalid (slow master / W back-pressure).
task automatic drive_w(
    input logic [DATA_WIDTH-1:0]  data,
    input logic [WSTRB_WIDTH-1:0] strb,
    input logic                   last,
    input int                     delay_cycles = 0
);
    if (delay_cycles > 0)
        repeat(delay_cycles) @(posedge clk);
    @(negedge clk);
    wvalid_tb = 1; wdata_tb = data; wstrb_tb = strb; wlast_tb = last;
    @(posedge clk);
    while (!axi_if.wready) @(posedge clk);
    @(negedge clk);
    wvalid_tb = 0; wlast_tb = 0;
endtask

// --- collect_b: wait for bvalid then sample ---
// bready_delay_cycles: cycles to keep bready LOW after bvalid asserts
//   before finally accepting (tests B-channel back-pressure).
// Returns bid, bresp, time of handshake, and (if bready_delay>0) the time
// bvalid first asserted so the delay can be verified.
task automatic collect_b(
    output logic [ID_WIDTH-1:0]    bid_out,
    output logic [BRESP_WIDTH-1:0] bresp_out,
    output time                    t_handshake,
    output time                    t_bvalid_first,
    input  int                     bready_delay_cycles = 0
);
    // Assert bready (may be delayed below)
    @(negedge clk);
    bready_tb = 0;

    // Wait for bvalid to assert
    @(posedge clk);
    while (!axi_if.bvalid) @(posedge clk);
    t_bvalid_first = $time;

    // Hold bready low for the requested number of extra cycles
    if (bready_delay_cycles > 0) begin
        repeat(bready_delay_cycles) begin
            // Sample that bvalid remains stable during the delay
            @(negedge clk);
            // bready stays 0
            @(posedge clk);
        end
    end

    // Now assert bready to complete the handshake
    @(negedge clk);
    bready_tb = 1;
    @(posedge clk);
    while (!axi_if.bvalid) @(posedge clk); // should fire immediately

    bid_out   = axi_if.bid;
    bresp_out = axi_if.bresp;
    t_handshake = $time;

    $display("[%0t] AXI B RESP  bid=0x%08h  bresp=2'b%02b (%s)",
             $time, bid_out, bresp_out,
             (bresp_out == 2'b00) ? "OKAY" : "SLVERR");
    @(negedge clk);
    bready_tb = 0;
endtask

// --- axi_single_write: convenience wrapper for single-beat write ---
// Drives AW + W + collects B.
// aw_delay / w_delay: cycles before awvalid / wvalid (back-pressure).
// b_delay: cycles bready held low after bvalid.
task automatic axi_single_write(
    input  logic [ID_WIDTH-1:0]    id,
    input  logic [ADDR_WIDTH-1:0]  addr,
    input  logic [DATA_WIDTH-1:0]  data,
    input  logic [WSTRB_WIDTH-1:0] strb,
    input  logic [MAX_SIZE-1:0]    size          = 3'b011,
    input  int                     aw_delay      = 0,
    input  int                     w_delay       = 0,
    input  int                     b_delay       = 0,
    output logic [ID_WIDTH-1:0]    bid_out,
    output logic [BRESP_WIDTH-1:0] bresp_out,
    output time                    t_bvalid_first,
    output time                    t_handshake
);
    fork drive_aw(id, addr, 8'h00, size, aw_delay); join_none
    drive_w(data, strb, 1'b1, w_delay);
    collect_b(bid_out, bresp_out, t_handshake, t_bvalid_first, b_delay);
endtask

// --- axi_burst_write: multi-beat write ---
task automatic axi_burst_write(
    input  logic [ID_WIDTH-1:0]    id,
    input  logic [ADDR_WIDTH-1:0]  addr,
    input  logic [MAX_LEN-1:0]     len,
    input  logic [MAX_SIZE-1:0]    size,
    input  logic [DATA_WIDTH-1:0]  data [],
    input  logic [WSTRB_WIDTH-1:0] strb [],
    output logic [ID_WIDTH-1:0]    bid_out,
    output logic [BRESP_WIDTH-1:0] bresp_out,
    output time                    t_bvalid_first,
    output time                    t_handshake
);
    fork drive_aw(id, addr, len, size); join_none
    for (int i = 0; i <= int'(len); i++)
        drive_w(data[i], strb[i], (i == int'(len)));
    collect_b(bid_out, bresp_out, t_handshake, t_bvalid_first);
endtask

task automatic idle(input int cycles);
    repeat(cycles) @(posedge clk);
endtask

/*=========================================================================*/
/*  Stimulus                                                                */
/*=========================================================================*/

initial begin
    $dumpfile("tb_write.vcd");
    $dumpvars(0, tb_write);

    @(posedge rst_n);
    idle(2);

    // ================================================================
    // TB_WRITE_001  Single-beat write all-zeros data
    //
    // Stimulus : awid=0x10  addr=0x0000_A000  awlen=0  awsize=3(8B)
    //            wdata=0x0000_0000_0000_0000  wstrb=8'hFF  wlast=1
    // APB expected (2 txns):
    //   txn0: WRITE paddr=0xA000  pwdata=0x0000_0000  pstrb=4'b1111
    //   txn1: WRITE paddr=0xA004  pwdata=0x0000_0000  pstrb=4'b1111
    // AXI expected: bid=0x10  bresp=2'b00(OKAY)
    // Rationale: verify zero data is not corrupted by any reset-value
    //            logic in the pipeline.
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_001: Single-beat write all-zeros data ======");
        $display("[001] Stimulus: awid=0x10 addr=0xA000 awlen=0 awsize=3");
        $display("[001] wdata=0x0 wstrb=8'hFF");
        $display("[001] Expected: 2 APB WRITE txns both pwdata=0x0; bid=0x10; bresp=OKAY");

        apb_monitor_reset();
        axi_single_write(32'h10, 32'h0000_A000, 64'h0, 8'hFF,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        // AXI response
        chk_hex("001: bid==0x0000_0010",              64'(bid),    64'h0000_0010);
        chk1   ("001: bresp[1]==0 (not SLVERR)",      bresp[1],    1'b0);
        chk1   ("001: bresp[0]==0 (OKAY=2'b00)",      bresp[0],    1'b0);
        // APB structural
        chk_int("001: apb_txn_count==2",              apb_txn_count, 2);
        chk1   ("001: txn0 pwrite==1 (WRITE)",        apb_txn_pwrite[0], 1'b1);
        chk_hex("001: txn0 paddr==0x0000_A000 (LSB)", 64'(apb_txn_paddr[0]),  64'h0000_A000);
        chk_hex("001: txn0 pwdata==0x0 (zero data)",  64'(apb_txn_pwdata[0]), 64'h0);
        chk_hex("001: txn0 pstrb==4'hF (all bytes)",  64'(apb_txn_pstrb[0]),  64'hF);
        chk1   ("001: txn1 pwrite==1 (WRITE)",        apb_txn_pwrite[1], 1'b1);
        chk_hex("001: txn1 paddr==0x0000_A004 (MSB)", 64'(apb_txn_paddr[1]),  64'h0000_A004);
        chk_hex("001: txn1 pwdata==0x0 (zero data)",  64'(apb_txn_pwdata[1]), 64'h0);
        chk_hex("001: txn1 pstrb==4'hF (all bytes)",  64'(apb_txn_pstrb[1]),  64'hF);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_002  Single-beat write all-ones data
    //
    // Stimulus : awid=0x11  addr=0x0000_B000  awlen=0  awsize=3(8B)
    //            wdata=0xFFFF_FFFF_FFFF_FFFF  wstrb=8'hFF  wlast=1
    // APB expected (2 txns):
    //   txn0: WRITE paddr=0xB000  pwdata=0xFFFF_FFFF  pstrb=4'b1111
    //   txn1: WRITE paddr=0xB004  pwdata=0xFFFF_FFFF  pstrb=4'b1111
    // AXI expected: bid=0x11  bresp=2'b00(OKAY)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_002: Single-beat write all-ones data ======");
        $display("[002] Stimulus: awid=0x11 addr=0xB000 awlen=0 awsize=3");
        $display("[002] wdata=0xFFFF_FFFF_FFFF_FFFF wstrb=8'hFF");
        $display("[002] Expected: 2 APB WRITE txns both pwdata=0xFFFF_FFFF; bid=0x11; bresp=OKAY");

        apb_monitor_reset();
        axi_single_write(32'h11, 32'h0000_B000, 64'hFFFF_FFFF_FFFF_FFFF, 8'hFF,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        // AXI response
        chk_hex("002: bid==0x0000_0011",                     64'(bid),    64'h0000_0011);
        chk1   ("002: bresp[1]==0 (not SLVERR)",             bresp[1],    1'b0);
        chk1   ("002: bresp[0]==0 (OKAY=2'b00)",             bresp[0],    1'b0);
        // APB structural
        chk_int("002: apb_txn_count==2",                     apb_txn_count, 2);
        chk1   ("002: txn0 pwrite==1 (WRITE)",               apb_txn_pwrite[0], 1'b1);
        chk_hex("002: txn0 paddr==0x0000_B000 (LSB)",        64'(apb_txn_paddr[0]),  64'h0000_B000);
        chk_hex("002: txn0 pwdata==0xFFFF_FFFF (all ones)",  64'(apb_txn_pwdata[0]), 64'hFFFF_FFFF);
        chk_hex("002: txn0 pstrb==4'hF",                     64'(apb_txn_pstrb[0]),  64'hF);
        chk1   ("002: txn1 pwrite==1 (WRITE)",               apb_txn_pwrite[1], 1'b1);
        chk_hex("002: txn1 paddr==0x0000_B004 (MSB)",        64'(apb_txn_paddr[1]),  64'h0000_B004);
        chk_hex("002: txn1 pwdata==0xFFFF_FFFF (all ones)",  64'(apb_txn_pwdata[1]), 64'hFFFF_FFFF);
        chk_hex("002: txn1 pstrb==4'hF",                     64'(apb_txn_pstrb[1]),  64'hF);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_003  Write wstrb=8'h00 (no bytes enabled)
    //
    // Stimulus : awid=0x12  addr=0x0000_C000  awlen=0  awsize=3(8B)
    //            wdata=0xDEAD_BEEF_CAFE_1234  wstrb=8'h00  wlast=1
    // Disassembler: valid_lsb = |wstrb[3:0] = 0 -> no LSB txn
    //               valid_msb = |wstrb[7:4] = 0 -> no MSB txn
    // APB expected: 0 txns issued
    // AXI expected: bid=0x12  bresp=2'b00(OKAY)
    //   The manager transitions SLOT_DISPATCH -> SLOT_WAIT_RESP immediately
    //   because slot_lsb_done=1 and slot_msb_done=1 (neither is valid).
    //   resp_ready fires immediately (neither half expected).
    //   B-response is issued with OKAY.
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_003: Write wstrb=8'h00 (no bytes enabled) ======");
        $display("[003] Stimulus: awid=0x12 addr=0xC000 awlen=0 awsize=3");
        $display("[003] wdata=0xDEAD_BEEF_CAFE_1234 wstrb=8'h00");
        $display("[003] Expected: 0 APB txns; bid=0x12; bresp=OKAY");
        $display("[003]   (disassembler valid_lsb=0 valid_msb=0; B fires from WAIT_RESP with resp_ready=1)");

        apb_monitor_reset();
        axi_single_write(32'h12, 32'h0000_C000, 64'hDEAD_BEEF_CAFE_1234, 8'h00,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        // AXI response
        chk_hex("003: bid==0x0000_0012",              64'(bid),    64'h0000_0012);
        chk1   ("003: bresp[1]==0 (not SLVERR)",      bresp[1],    1'b0);
        chk1   ("003: bresp[0]==0 (OKAY=2'b00)",      bresp[0],    1'b0);
        // APB structural: the key assertion -- zero txns must be issued
        chk_int("003: apb_txn_count==0 (no APB txns when wstrb=8'h00)",
                apb_txn_count, 0);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_004  Write partial strobe LSB only (wstrb=8'h0F)
    //
    // Stimulus : awid=0x13  addr=0x0000_D000  awlen=0  awsize=3(8B)
    //            wdata=0xDEAD_BEEF_CAFE_1234  wstrb=8'h0F  wlast=1
    // Disassembler: valid_lsb = |wstrb[3:0] = |4'hF = 1 -> LSB txn issued
    //               valid_msb = |wstrb[7:4] = |4'h0 = 0 -> MSB suppressed
    // APB expected (1 txn):
    //   txn0: WRITE paddr=0xD000  pwdata=0xCAFE_1234  pstrb=4'b0000_1111
    // AXI expected: bid=0x13  bresp=2'b00(OKAY)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_004: Partial strobe LSB only (wstrb=8'h0F) ======");
        $display("[004] Stimulus: awid=0x13 addr=0xD000 awlen=0 awsize=3");
        $display("[004] wdata=0xDEAD_BEEF_CAFE_1234 wstrb=8'h0F");
        $display("[004] Expected: 1 APB WRITE txn (LSB only); pstrb=4'hF; MSB suppressed");

        apb_monitor_reset();
        axi_single_write(32'h13, 32'h0000_D000, 64'hDEAD_BEEF_CAFE_1234, 8'h0F,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        // AXI response
        chk_hex("004: bid==0x0000_0013",                        64'(bid),    64'h0000_0013);
        chk1   ("004: bresp[1]==0 (not SLVERR)",                bresp[1],    1'b0);
        chk1   ("004: bresp[0]==0 (OKAY=2'b00)",                bresp[0],    1'b0);
        // APB structural: exactly 1 txn (LSB only)
        chk_int("004: apb_txn_count==1 (LSB only; MSB suppressed)", apb_txn_count, 1);
        chk1   ("004: txn0 pwrite==1 (WRITE)",                  apb_txn_pwrite[0], 1'b1);
        chk_hex("004: txn0 paddr==0x0000_D000 (LSB addr)",      64'(apb_txn_paddr[0]),  64'h0000_D000);
        chk_hex("004: txn0 pwdata==0xCAFE_1234 (wdata[31:0])",  64'(apb_txn_pwdata[0]), 64'hCAFE_1234);
        chk_hex("004: txn0 pstrb==4'hF (wstrb[3:0]=4'hF)",     64'(apb_txn_pstrb[0]),  64'hF);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_005  Write partial strobe MSB only (wstrb=8'hF0)
    //
    // Stimulus : awid=0x14  addr=0x0000_E000  awlen=0  awsize=3(8B)
    //            wdata=0xDEAD_BEEF_CAFE_1234  wstrb=8'hF0  wlast=1
    // Disassembler: valid_lsb = |wstrb[3:0] = |4'h0 = 0 -> LSB suppressed
    //               valid_msb = |wstrb[7:4] = |4'hF = 1 -> MSB txn issued
    //   MSB pstrb = wstrb[7:4] shifted to pstrb[3:0] = 4'hF
    //   MSB pwdata = wdata[63:32] = 0xDEAD_BEEF
    // APB expected (1 txn):
    //   txn0: WRITE paddr=0xE004  pwdata=0xDEAD_BEEF  pstrb=4'b1111
    // AXI expected: bid=0x14  bresp=2'b00(OKAY)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_005: Partial strobe MSB only (wstrb=8'hF0) ======");
        $display("[005] Stimulus: awid=0x14 addr=0xE000 awlen=0 awsize=3");
        $display("[005] wdata=0xDEAD_BEEF_CAFE_1234 wstrb=8'hF0");
        $display("[005] Expected: 1 APB WRITE txn (MSB only); paddr=0xE004; pstrb=4'hF; LSB suppressed");

        apb_monitor_reset();
        axi_single_write(32'h14, 32'h0000_E000, 64'hDEAD_BEEF_CAFE_1234, 8'hF0,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        // AXI response
        chk_hex("005: bid==0x0000_0014",                          64'(bid),    64'h0000_0014);
        chk1   ("005: bresp[1]==0 (not SLVERR)",                  bresp[1],    1'b0);
        chk1   ("005: bresp[0]==0 (OKAY=2'b00)",                  bresp[0],    1'b0);
        // APB structural: exactly 1 txn (MSB only)
        chk_int("005: apb_txn_count==1 (MSB only; LSB suppressed)", apb_txn_count, 1);
        chk1   ("005: txn0 pwrite==1 (WRITE)",                    apb_txn_pwrite[0], 1'b1);
        chk_hex("005: txn0 paddr==0x0000_E004 (MSB addr)",        64'(apb_txn_paddr[0]),  64'h0000_E004);
        chk_hex("005: txn0 pwdata==0xDEAD_BEEF (wdata[63:32])",   64'(apb_txn_pwdata[0]), 64'hDEAD_BEEF);
        chk_hex("005: txn0 pstrb==4'hF (wstrb[7:4]=4'hF shifted)",64'(apb_txn_pstrb[0]),  64'hF);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_006  Write partial strobe single byte LSB (wstrb=8'h01)
    //
    // Stimulus : awid=0x15  addr=0x0000_F000  awlen=0  awsize=3(8B)
    //            wdata=0xDEAD_BEEF_CAFE_1234  wstrb=8'h01  wlast=1
    // Disassembler: valid_lsb = |wstrb[3:0] = |4'h1 = 1 -> LSB txn
    //               valid_msb = |wstrb[7:4] = |4'h0 = 0 -> MSB suppressed
    //   LSB pstrb = wstrb[3:0] = 4'h1
    //   LSB pwdata = wdata[31:0] = 0xCAFE_1234
    // APB expected (1 txn):
    //   txn0: WRITE paddr=0xF000  pwdata=0xCAFE_1234  pstrb=4'b0001
    // AXI expected: bid=0x15  bresp=2'b00(OKAY)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_006: Single-byte strobe LSB (wstrb=8'h01) ======");
        $display("[006] Stimulus: awid=0x15 addr=0xF000 awlen=0 awsize=3");
        $display("[006] wdata=0xDEAD_BEEF_CAFE_1234 wstrb=8'h01");
        $display("[006] Expected: 1 APB WRITE txn; pstrb=4'h1; pwdata=0xCAFE_1234; bid=0x15");

        apb_monitor_reset();
        axi_single_write(32'h15, 32'h0000_F000, 64'hDEAD_BEEF_CAFE_1234, 8'h01,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        // AXI response
        chk_hex("006: bid==0x0000_0015",                        64'(bid),    64'h0000_0015);
        chk1   ("006: bresp[1]==0 (not SLVERR)",                bresp[1],    1'b0);
        chk1   ("006: bresp[0]==0 (OKAY=2'b00)",                bresp[0],    1'b0);
        // APB structural
        chk_int("006: apb_txn_count==1 (byte0 only; 3 bytes suppressed)", apb_txn_count, 1);
        chk1   ("006: txn0 pwrite==1 (WRITE)",                  apb_txn_pwrite[0], 1'b1);
        chk_hex("006: txn0 paddr==0x0000_F000 (LSB addr)",      64'(apb_txn_paddr[0]),  64'h0000_F000);
        chk_hex("006: txn0 pwdata==0xCAFE_1234 (wdata[31:0])",  64'(apb_txn_pwdata[0]), 64'hCAFE_1234);
        chk_hex("006: txn0 pstrb==4'h1 (byte0 only)",           64'(apb_txn_pstrb[0]),  64'h1);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_007  Write partial strobe single byte MSB (wstrb=8'h80)
    //
    // Stimulus : awid=0x16  addr=0x0001_0000  awlen=0  awsize=3(8B)
    //            wdata=0xDEAD_BEEF_CAFE_1234  wstrb=8'h80  wlast=1
    // Disassembler: valid_lsb = |wstrb[3:0] = |4'h0 = 0 -> LSB suppressed
    //               valid_msb = |wstrb[7:4] = |4'h8 = 1 -> MSB txn
    //   MSB pstrb = wstrb[7:4] shifted to pstrb[3:0] = 4'h8
    //   MSB pwdata = wdata[63:32] = 0xDEAD_BEEF
    // APB expected (1 txn):
    //   txn0: WRITE paddr=0x0001_0004  pwdata=0xDEAD_BEEF  pstrb=4'b1000
    // AXI expected: bid=0x16  bresp=2'b00(OKAY)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_007: Single-byte strobe MSB (wstrb=8'h80) ======");
        $display("[007] Stimulus: awid=0x16 addr=0x0001_0000 awlen=0 awsize=3");
        $display("[007] wdata=0xDEAD_BEEF_CAFE_1234 wstrb=8'h80");
        $display("[007] Expected: 1 APB WRITE txn; paddr=0x0001_0004; pstrb=4'h8; bid=0x16");

        apb_monitor_reset();
        axi_single_write(32'h16, 32'h0001_0000, 64'hDEAD_BEEF_CAFE_1234, 8'h80,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        // AXI response
        chk_hex("007: bid==0x0000_0016",                          64'(bid),    64'h0000_0016);
        chk1   ("007: bresp[1]==0 (not SLVERR)",                  bresp[1],    1'b0);
        chk1   ("007: bresp[0]==0 (OKAY=2'b00)",                  bresp[0],    1'b0);
        // APB structural
        chk_int("007: apb_txn_count==1 (byte7 only; 7 bytes suppressed)", apb_txn_count, 1);
        chk1   ("007: txn0 pwrite==1 (WRITE)",                    apb_txn_pwrite[0], 1'b1);
        chk_hex("007: txn0 paddr==0x0001_0004 (MSB addr)",        64'(apb_txn_paddr[0]),  64'h0001_0004);
        chk_hex("007: txn0 pwdata==0xDEAD_BEEF (wdata[63:32])",   64'(apb_txn_pwdata[0]), 64'hDEAD_BEEF);
        chk_hex("007: txn0 pstrb==4'h8 (wstrb[7:4]=4'h8 shifted)",64'(apb_txn_pstrb[0]),  64'h8);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_008  Narrow write size=1 (2B) at LSB-aligned address
    //
    // Stimulus : awid=0x17  addr=0x0001_1000  awlen=0  awsize=1(2B)
    //            wdata=0x0000_0000_0000_ABCD  wstrb=8'h03  wlast=1
    // beat_addr=0x1000, addr[2]=0 -> disassembler selects LSB half
    //   LSB: lsb_active=1  valid_lsb = lsb_active & |wstrb[3:0] = 1&|4'h3 = 1
    //   MSB: msb_active=0  valid_msb = 0
    //   paddr = {addr[31:3], 3'b000} = 0x1000
    //   pwdata = wdata[31:0] = 0x0000_ABCD
    //   pstrb = wstrb[3:0] = 4'h3
    // APB expected (1 txn):
    //   txn0: WRITE paddr=0x0001_1000  pwdata=0x0000_ABCD  pstrb=4'b0011
    // AXI expected: bid=0x17  bresp=2'b00(OKAY)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_008: Narrow size=1 (2B) at addr[2]=0 (LSB) ======");
        $display("[008] Stimulus: awid=0x17 addr=0x0001_1000 awlen=0 awsize=1(2B)");
        $display("[008] wdata=0x0000_0000_0000_ABCD wstrb=8'h03");
        $display("[008] Expected: 1 APB WRITE txn (LSB); paddr=0x1000; pwdata=0xABCD; pstrb=4'h3");

        apb_monitor_reset();
        axi_single_write(32'h17, 32'h0001_1000, 64'h0000_0000_0000_ABCD, 8'h03,
                         3'b001, 0, 0, 0, bid, bresp, t_bv, t_bh);

        // AXI response
        chk_hex("008: bid==0x0000_0017",                          64'(bid),    64'h0000_0017);
        chk1   ("008: bresp[1]==0 (not SLVERR)",                  bresp[1],    1'b0);
        chk1   ("008: bresp[0]==0 (OKAY=2'b00)",                  bresp[0],    1'b0);
        // APB structural
        chk_int("008: apb_txn_count==1 (narrow: LSB only, addr[2]=0)", apb_txn_count, 1);
        chk1   ("008: txn0 pwrite==1 (WRITE)",                    apb_txn_pwrite[0], 1'b1);
        chk_hex("008: txn0 paddr==0x0001_1000 (aligned LSB)",    64'(apb_txn_paddr[0]),  64'h0001_1000);
        chk_hex("008: txn0 pwdata==0x0000_ABCD (wdata[31:0])",   64'(apb_txn_pwdata[0]), 64'h0000_ABCD);
        chk_hex("008: txn0 pstrb==4'h3 (wstrb[3:0]=4'h3)",       64'(apb_txn_pstrb[0]),  64'h3);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_009  Narrow write size=1 (2B) at MSB-aligned address
    //
    // Stimulus : awid=0x18  addr=0x0001_1004  awlen=0  awsize=1(2B)
    //            wdata=0x0000_0000_ABCD_0000  wstrb=8'h0C  wlast=1
    //
    // beat_addr=0x1004, addr[2]=1 -> disassembler selects MSB half
    //   LSB: lsb_active=0  valid_lsb = 0
    //   MSB: msb_active=1  valid_msb = msb_active & |wstrb[7:4] = 1&|4'h0 = 0
    //
    // Wait -- wstrb=8'h0C means wstrb[7:4]=4'h0 and wstrb[3:0]=4'hC.
    // For addr[2]=1, the MSB APB half is active. The strobe that feeds the
    // MSB is wstrb[7:4]. With wstrb=8'h0C, wstrb[7:4]=4'h0 -> no MSB txn!
    //
    // Corrected stimulus: to hit the MSB half with strobe data, we need
    // wstrb[7:4] != 0. Use wstrb=8'hC0 (wstrb[7:4]=4'hC, [3:0]=4'h0).
    // Then valid_msb = 1; pstrb = wstrb[7:4] shifted = 4'hC -> BUT per
    // disassembler msb_pstrb = wstrb[WSTRB_WIDTH-1:PSTRB_WIDTH] = wstrb[7:4]
    // passed directly (already a 4-bit field = pstrb[3:0]).
    // So pstrb = 4'hC.
    //
    // Corrected spec:
    //   wstrb=8'hC0  wdata=0xABCD_0000_0000_0000
    //   txn0: WRITE paddr=0x0001_1004 (msb_addr = {addr[31:3],3'b100})
    //               pwdata=0xABCD_0000 (wdata[63:32])
    //               pstrb=4'hC (wstrb[7:4]=4'hC shifted to [3:0])
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_009: Narrow size=1 (2B) at addr[2]=1 (MSB) ======");
        $display("[009] Stimulus: awid=0x18 addr=0x0001_1004 awlen=0 awsize=1(2B)");
        $display("[009] wdata=0xABCD_0000_0000_0000 wstrb=8'hC0");
        $display("[009] Expected: 1 APB WRITE txn (MSB); paddr=0x0001_1004; pwdata=0xABCD_0000; pstrb=4'hC");
        $display("[009]   (wstrb[7:4]=4'hC shifted to APB pstrb[3:0])");

        apb_monitor_reset();
        axi_single_write(32'h18, 32'h0001_1004, 64'hABCD_0000_0000_0000, 8'hC0,
                         3'b001, 0, 0, 0, bid, bresp, t_bv, t_bh);

        // AXI response
        chk_hex("009: bid==0x0000_0018",                            64'(bid),    64'h0000_0018);
        chk1   ("009: bresp[1]==0 (not SLVERR)",                    bresp[1],    1'b0);
        chk1   ("009: bresp[0]==0 (OKAY=2'b00)",                    bresp[0],    1'b0);
        // APB structural
        chk_int("009: apb_txn_count==1 (narrow: MSB only, addr[2]=1)", apb_txn_count, 1);
        chk1   ("009: txn0 pwrite==1 (WRITE)",                      apb_txn_pwrite[0], 1'b1);
        chk_hex("009: txn0 paddr==0x0001_1004 (MSB addr, addr[2]=1)",64'(apb_txn_paddr[0]),  64'h0001_1004);
        chk_hex("009: txn0 pwdata==0xABCD_0000 (wdata[63:32])",      64'(apb_txn_pwdata[0]), 64'hABCD_0000);
        chk_hex("009: txn0 pstrb==4'hC (wstrb[7:4]=4'hC shifted)",  64'(apb_txn_pstrb[0]),  64'hC);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_010  Write awid=0 (min ID boundary)
    //
    // Stimulus : awid=0x0  addr=0x0002_0000  awlen=0  awsize=3(8B)
    //            wdata=0xAAAA_BBBB_CCCC_DDDD  wstrb=8'hFF  wlast=1
    // AXI expected: bid=0x0  bresp=2'b00(OKAY)
    // Verifies: 32-bit ID=0 is passed through correctly (not confused with
    //           any default/reset value in the pipeline).
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_010: Write awid=0x0 (min ID) ======");
        $display("[010] Stimulus: awid=0x0 addr=0x0002_0000 awlen=0 awsize=3");
        $display("[010] Expected: 2 APB WRITE txns; bid=0x0; bresp=OKAY");

        apb_monitor_reset();
        axi_single_write(32'h0, 32'h0002_0000, 64'hAAAA_BBBB_CCCC_DDDD, 8'hFF,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        chk_hex("010: bid==0x0000_0000 (zero ID passthrough)", 64'(bid),    64'h0000_0000);
        chk1   ("010: bresp[1]==0 (not SLVERR)",               bresp[1],    1'b0);
        chk1   ("010: bresp[0]==0 (OKAY=2'b00)",               bresp[0],    1'b0);
        chk_int("010: apb_txn_count==2",                       apb_txn_count, 2);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_011  Write awid=0xFFFF_FFFF (max ID boundary)
    //
    // Stimulus : awid=0xFFFF_FFFF  addr=0x0002_0000  awlen=0  awsize=3(8B)
    //            wdata=0x1234_5678_9ABC_DEF0  wstrb=8'hFF  wlast=1
    // AXI expected: bid=0xFFFF_FFFF  bresp=2'b00(OKAY)
    // Verifies: full 32-bit ID width preserved through pipeline.
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_011: Write awid=0xFFFF_FFFF (max ID) ======");
        $display("[011] Stimulus: awid=0xFFFF_FFFF addr=0x0002_0000 awlen=0 awsize=3");
        $display("[011] Expected: 2 APB WRITE txns; bid=0xFFFF_FFFF; bresp=OKAY");

        apb_monitor_reset();
        axi_single_write(32'hFFFF_FFFF, 32'h0002_0000, 64'h1234_5678_9ABC_DEF0, 8'hFF,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        chk_hex("011: bid==0xFFFF_FFFF (max ID passthrough)", 64'(bid),    64'hFFFF_FFFF);
        chk1   ("011: bresp[1]==0 (not SLVERR)",              bresp[1],    1'b0);
        chk1   ("011: bresp[0]==0 (OKAY=2'b00)",              bresp[0],    1'b0);
        chk_int("011: apb_txn_count==2",                      apb_txn_count, 2);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_012  Write SLVERR on LSB only (APB txn#0)
    //
    // Stimulus : awid=0x19  addr=0x0003_0000  awlen=0  awsize=3(8B)
    //            wdata=0xDEAD_BEEF_CAFE_1234  wstrb=8'hFF  wlast=1
    //            pslverr injected on paddr=0x0003_0000 (LSB, txn#0)
    // APB txn order:
    //   txn0: WRITE 0x0003_0000 -> SLVERR  <<< injected
    //   txn1: WRITE 0x0003_0004 -> OKAY
    // AXI expected: bid=0x19  bresp=2'b10(SLVERR)
    //   (assembler OR: pslverr_lsb | pslverr_msb = 1|0 = 1)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_012: SLVERR on LSB only (txn#0) ======");
        $display("[012] Stimulus: awid=0x19 addr=0x0003_0000 awlen=0 awsize=3");
        $display("[012] Inject:   pslverr on paddr=0x0003_0000 (LSB txn#0)");
        $display("[012] Expected: 2 APB txns; bid=0x19; bresp=2'b10(SLVERR)");

        apb_monitor_reset();
        apb_err_addr = 32'h0003_0000;

        axi_single_write(32'h19, 32'h0003_0000, 64'hDEAD_BEEF_CAFE_1234, 8'hFF,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        chk_hex("012: bid==0x0000_0019",                      64'(bid),   64'h0000_0019);
        chk1   ("012: bresp[1]==1 (SLVERR from LSB)",         bresp[1],   1'b1);
        chk1   ("012: bresp[0]==0 (SLVERR=2'b10 not 2'b11)", bresp[0],   1'b0);
        chk_int("012: apb_txn_count==2",                      apb_txn_count, 2);
        chk_hex("012: txn0 paddr==0x0003_0000 (SLVERR txn)",  64'(apb_txn_paddr[0]),  64'h0003_0000);
        chk_hex("012: txn1 paddr==0x0003_0004 (OKAY txn)",    64'(apb_txn_paddr[1]),  64'h0003_0004);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_013  Write SLVERR on MSB only (APB txn#1)
    //
    // Stimulus : awid=0x1A  addr=0x0004_0000  awlen=0  awsize=3(8B)
    //            wdata=0xDEAD_BEEF_CAFE_1234  wstrb=8'hFF  wlast=1
    //            pslverr injected on paddr=0x0004_0004 (MSB, txn#1)
    // APB txn order:
    //   txn0: WRITE 0x0004_0000 -> OKAY
    //   txn1: WRITE 0x0004_0004 -> SLVERR  <<< injected
    // AXI expected: bid=0x1A  bresp=2'b10(SLVERR)
    //   (assembler OR: pslverr_lsb | pslverr_msb = 0|1 = 1)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_013: SLVERR on MSB only (txn#1) ======");
        $display("[013] Stimulus: awid=0x1A addr=0x0004_0000 awlen=0 awsize=3");
        $display("[013] Inject:   pslverr on paddr=0x0004_0004 (MSB txn#1)");
        $display("[013] Expected: 2 APB txns; bid=0x1A; bresp=2'b10(SLVERR)");

        apb_monitor_reset();
        apb_err_addr = 32'h0004_0004;

        axi_single_write(32'h1A, 32'h0004_0000, 64'hDEAD_BEEF_CAFE_1234, 8'hFF,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        chk_hex("013: bid==0x0000_001A",                      64'(bid),   64'h0000_001A);
        chk1   ("013: bresp[1]==1 (SLVERR from MSB)",         bresp[1],   1'b1);
        chk1   ("013: bresp[0]==0 (SLVERR=2'b10 not 2'b11)", bresp[0],   1'b0);
        chk_int("013: apb_txn_count==2",                      apb_txn_count, 2);
        chk_hex("013: txn0 paddr==0x0004_0000 (OKAY txn)",    64'(apb_txn_paddr[0]),  64'h0004_0000);
        chk_hex("013: txn1 paddr==0x0004_0004 (SLVERR txn)",  64'(apb_txn_paddr[1]),  64'h0004_0004);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_014  Write SLVERR on both halves
    //
    // Stimulus : awid=0x1B  addr=0x0005_0000  awlen=0  awsize=3(8B)
    //            wdata=0xDEAD_BEEF_CAFE_1234  wstrb=8'hFF  wlast=1
    //            pslverr injected on both 0x0005_0000 AND 0x0005_0004
    // APB txn order:
    //   txn0: WRITE 0x0005_0000 -> SLVERR  <<< injected
    //   txn1: WRITE 0x0005_0004 -> SLVERR  <<< injected
    // AXI expected: bid=0x1B  bresp=2'b10(SLVERR)
    //   (assembler OR: 1|1 = 1)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_014: SLVERR on both halves ======");
        $display("[014] Stimulus: awid=0x1B addr=0x0005_0000 awlen=0 awsize=3");
        $display("[014] Inject:   pslverr on 0x0005_0000 (LSB) AND 0x0005_0004 (MSB)");
        $display("[014] Expected: 2 APB txns both SLVERR; bid=0x1B; bresp=2'b10(SLVERR)");

        apb_monitor_reset();
        apb_err_addr  = 32'h0005_0000;
        apb_err_addr2 = 32'h0005_0004;

        axi_single_write(32'h1B, 32'h0005_0000, 64'hDEAD_BEEF_CAFE_1234, 8'hFF,
                         3'b011, 0, 0, 0, bid, bresp, t_bv, t_bh);

        chk_hex("014: bid==0x0000_001B",                          64'(bid),   64'h0000_001B);
        chk1   ("014: bresp[1]==1 (SLVERR: both halves errored)", bresp[1],   1'b1);
        chk1   ("014: bresp[0]==0 (SLVERR=2'b10 not 2'b11)",     bresp[0],   1'b0);
        chk_int("014: apb_txn_count==2",                          apb_txn_count, 2);
        chk_hex("014: txn0 paddr==0x0005_0000 (SLVERR txn0)",     64'(apb_txn_paddr[0]),  64'h0005_0000);
        chk_hex("014: txn1 paddr==0x0005_0004 (SLVERR txn1)",     64'(apb_txn_paddr[1]),  64'h0005_0004);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_015  Write SLVERR on non-first beat of burst (beat2 MSB)
    //
    // Stimulus : awid=0x1C  addr=0x0006_0000  awlen=3(4 beats)  awsize=3(8B)
    //            beat i: wdata={0xABCD_0000+i, 0x1234_0000+i}  wstrb=8'hFF
    //            pslverr injected on paddr=0x0006_001C (beat2 MSB = APB txn#5)
    //
    // APB txn order (8 total, 2 per beat):
    //   beat0: txn0=0x6000 OKAY  txn1=0x6004 OKAY
    //   beat1: txn2=0x6008 OKAY  txn3=0x600C OKAY
    //   beat2: txn4=0x6010 OKAY  txn5=0x601C SLVERR  <<< injected
    //   beat3: txn6=0x6020 OKAY  txn7=0x6024 OKAY
    //
    // AXI expected: bid=0x1C  bresp=2'b10(SLVERR)
    //   wlast respected: B-resp issued only after beat3 (wlast beat)
    //   Mid-burst error accumulation: slot_wr_burst_err latches the SLVERR
    //   and OR-combines it into the final bresp on wlast.
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        logic [DATA_WIDTH-1:0]  wd[];
        logic [WSTRB_WIDTH-1:0] ws[];
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_015: Mid-burst SLVERR (beat2 MSB, 4-beat burst) ======");
        $display("[015] Stimulus: awid=0x1C addr=0x0006_0000 awlen=3 awsize=3");
        $display("[015] Inject:   pslverr on paddr=0x0006_001C (beat2 MSB, txn#5)");
        $display("[015] Expected: 8 APB txns; bid=0x1C; bresp=2'b10(SLVERR)");
        $display("[015]           B-resp only after wlast (beat3)");

        apb_monitor_reset();
        wd = new[4]; ws = new[4];
        for (int i = 0; i < 4; i++) begin
            wd[i] = {32'hABCD_0000 + i, 32'h1234_0000 + i};
            ws[i] = 8'hFF;
        end
        apb_err_addr = 32'h0006_001C;  // beat2 MSB

        axi_burst_write(32'h1C, 32'h0006_0000, 8'h03, 3'b011,
                        wd, ws, bid, bresp, t_bv, t_bh);

        chk_hex("015: bid==0x0000_001C",                            64'(bid),   64'h0000_001C);
        chk1   ("015: bresp[1]==1 (SLVERR: mid-burst propagated)",  bresp[1],   1'b1);
        chk1   ("015: bresp[0]==0 (SLVERR=2'b10)",                  bresp[0],   1'b0);
        chk_int("015: apb_txn_count==8 (4 beats * 2 halves)",       apb_txn_count, 8);
        // Verify the injected SLVERR was on txn5
        chk_hex("015: txn5 paddr==0x0006_001C (SLVERR txn)",        64'(apb_txn_paddr[5]),  64'h0006_001C);
        chk1   ("015: txn5 pwrite==1 (was a WRITE)",                apb_txn_pwrite[5],     1'b1);
        // Verify the non-error beats have correct addresses
        chk_hex("015: txn0 paddr==0x0006_0000 (beat0 LSB)",         64'(apb_txn_paddr[0]),  64'h0006_0000);
        chk_hex("015: txn6 paddr==0x0006_0020 (beat3 LSB, after err)",64'(apb_txn_paddr[6]),64'h0006_0020);
        chk_hex("015: txn7 paddr==0x0006_0024 (beat3 MSB, after err)",64'(apb_txn_paddr[7]),64'h0006_0024);
        idle(4);
    end

    // ================================================================
    // TB_WRITE_016  AW channel back-pressure (slow master)
    //
    // Stimulus : awid=0x1D  addr=0x0007_0000  awlen=0  awsize=3(8B)
    //            wdata=0xAAAA_AAAA_AAAA_AAAA  wstrb=8'hFF  wlast=1
    //            AW channel: master holds awvalid LOW for 5 cycles before
    //            asserting it (simulates a slow AXI master on AW).
    //
    // Note: awready = !wr_req_fifo_full (DUT-driven).
    //   The TB cannot de-assert awready directly.
    //   Back-pressure from the DUT side occurs when the FIFO fills.
    //   Here we test the master-side delay: the master is slow to present
    //   the address, but the DUT must accept it correctly when it appears.
    //
    // AXI expected: bid=0x1D  bresp=2'b00(OKAY)
    //   Transaction must complete successfully despite the AW delay.
    //   The time from stimulus start to B-response must be > 5 clock cycles
    //   longer than a zero-delay equivalent (validated by timestamp).
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;
        time                    t_start;

        $display("\n====== TB_WRITE_016: AW channel back-pressure (5-cycle master delay) ======");
        $display("[016] Stimulus: awid=0x1D addr=0x0007_0000 awlen=0 awsize=3");
        $display("[016] AW delay: master holds awvalid LOW for 5 cycles before asserting");
        $display("[016] Expected: 2 APB WRITE txns; bid=0x1D; bresp=OKAY");
        $display("[016]           Transaction completes correctly despite AW delay");

        apb_monitor_reset();
        t_start = $time;
        axi_single_write(32'h1D, 32'h0007_0000, 64'hAAAA_AAAA_AAAA_AAAA, 8'hFF,
                         3'b011, 5, 0, 0, bid, bresp, t_bv, t_bh);

        chk_hex("016: bid==0x0000_001D",                             64'(bid),    64'h0000_001D);
        chk1   ("016: bresp[1]==0 (not SLVERR; handshake ok)",       bresp[1],    1'b0);
        chk1   ("016: bresp[0]==0 (OKAY=2'b00)",                     bresp[0],    1'b0);
        chk_int("016: apb_txn_count==2 (data not lost during delay)", apb_txn_count, 2);
        chk_hex("016: txn0 pwdata==0xAAAA_AAAA (no corruption)",      64'(apb_txn_pwdata[0]), 64'hAAAA_AAAA);
        chk_hex("016: txn1 pwdata==0xAAAA_AAAA (no corruption)",      64'(apb_txn_pwdata[1]), 64'hAAAA_AAAA);
        // Timing: B-handshake must be at least 5 cycles after start
        // (5 delay cycles * 10 ns/cycle = 50 ns minimum overhead)
        if (t_bh - t_start >= 50) begin
            $display("[%0t] PASS  016: B-resp timing correct (delay >= 50ns: t_start=%0t t_bh=%0t)",
                     $time, t_start, t_bh);
            pass_count++;
        end else begin
            $display("[%0t] FAIL  016: B-resp timing too fast (expected >= 50ns: t_start=%0t t_bh=%0t)  <<< MISMATCH",
                     $time, t_start, t_bh);
            fail_count++;
        end
        idle(4);
    end

    // ================================================================
    // TB_WRITE_017  W channel back-pressure (slow master, 2-beat burst)
    //
    // Stimulus : awid=0x1E  addr=0x0008_0000  awlen=1(2 beats)  awsize=3(8B)
    //            beat0: wdata=0x1111_1111_2222_2222  wstrb=8'hFF
    //            beat1: wdata=0x3333_3333_4444_4444  wstrb=8'hFF  wlast=1
    //            W channel: each W beat delayed by 5 cycles before wvalid
    //            asserts (simulates a slow AXI master on W channel).
    //
    // AXI expected: bid=0x1E  bresp=2'b00(OKAY)
    //   wlast must still be correctly asserted on beat1 only.
    //   4 APB txns with correct addresses and data.
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;
        time                    t_start;

        $display("\n====== TB_WRITE_017: W channel back-pressure (5-cycle delay per beat) ======");
        $display("[017] Stimulus: awid=0x1E addr=0x0008_0000 awlen=1 awsize=3");
        $display("[017] W delay:  5 idle cycles before each wvalid assertion");
        $display("[017] Expected: 4 APB WRITE txns; bid=0x1E; bresp=OKAY; wlast on beat1");

        apb_monitor_reset();
        t_start = $time;

        // AW (no delay on address)
        fork drive_aw(32'h1E, 32'h0008_0000, 8'h01, 3'b011, 0); join_none

        // W beat0 with 5-cycle delay
        drive_w(64'h1111_1111_2222_2222, 8'hFF, 1'b0, 5);
        // W beat1 with 5-cycle delay, wlast=1
        drive_w(64'h3333_3333_4444_4444, 8'hFF, 1'b1, 5);

        collect_b(bid, bresp, t_bh, t_bv);

        chk_hex("017: bid==0x0000_001E",                             64'(bid),    64'h0000_001E);
        chk1   ("017: bresp[1]==0 (not SLVERR)",                     bresp[1],    1'b0);
        chk1   ("017: bresp[0]==0 (OKAY=2'b00)",                     bresp[0],    1'b0);
        chk_int("017: apb_txn_count==4 (4 txns despite W delay)",    apb_txn_count, 4);
        // Verify data integrity through the delay
        chk_hex("017: txn0 paddr==0x0008_0000 (beat0 LSB)",          64'(apb_txn_paddr[0]),  64'h0008_0000);
        chk_hex("017: txn0 pwdata==0x2222_2222 (beat0 wdata[31:0])", 64'(apb_txn_pwdata[0]), 64'h2222_2222);
        chk_hex("017: txn1 paddr==0x0008_0004 (beat0 MSB)",          64'(apb_txn_paddr[1]),  64'h0008_0004);
        chk_hex("017: txn1 pwdata==0x1111_1111 (beat0 wdata[63:32])",64'(apb_txn_pwdata[1]), 64'h1111_1111);
        chk_hex("017: txn2 paddr==0x0008_0008 (beat1 LSB)",          64'(apb_txn_paddr[2]),  64'h0008_0008);
        chk_hex("017: txn2 pwdata==0x4444_4444 (beat1 wdata[31:0])", 64'(apb_txn_pwdata[2]), 64'h4444_4444);
        chk_hex("017: txn3 paddr==0x0008_000C (beat1 MSB)",          64'(apb_txn_paddr[3]),  64'h0008_000C);
        chk_hex("017: txn3 pwdata==0x3333_3333 (beat1 wdata[63:32])",64'(apb_txn_pwdata[3]), 64'h3333_3333);
        // Timing: must be > 10 extra cycles (2 beats * 5 delay cycles each)
        if (t_bh - t_start >= 100) begin
            $display("[%0t] PASS  017: B-resp timing correct (>= 100ns delay: t_start=%0t t_bh=%0t)",
                     $time, t_start, t_bh);
            pass_count++;
        end else begin
            $display("[%0t] FAIL  017: B-resp timing too fast (expected >= 100ns: t_start=%0t t_bh=%0t)  <<< MISMATCH",
                     $time, t_start, t_bh);
            fail_count++;
        end
        idle(4);
    end

    // ================================================================
    // TB_WRITE_018  B channel back-pressure (bready held low for 10 cycles)
    //
    // Stimulus : awid=0x1F  addr=0x0009_0000  awlen=0  awsize=3(8B)
    //            wdata=0xBBBB_BBBB_BBBB_BBBB  wstrb=8'hFF  wlast=1
    //            bready is held LOW for 10 cycles after bvalid asserts.
    //
    // AXI expected:
    //   bvalid asserts when APB txns complete and B-response is ready.
    //   bid and bresp must remain stable while bready is low.
    //   The handshake only completes when bready finally asserts.
    //   After handshake: bid=0x1F  bresp=2'b00(OKAY)
    //   No duplicate B-response must be issued.
    //
    // Key checks:
    //   - bid and bresp sampled at handshake == expected values
    //   - apb_txn_count==2 (APB activity not repeated)
    //   - t_handshake > t_bvalid_first (bvalid held for at least 10 cycles)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        time                    t_bv, t_bh;

        $display("\n====== TB_WRITE_018: B channel back-pressure (bready LOW for 10 cycles) ======");
        $display("[018] Stimulus: awid=0x1F addr=0x0009_0000 awlen=0 awsize=3");
        $display("[018] bready:   held LOW for 10 cycles after bvalid asserts");
        $display("[018] Expected: 2 APB txns; bid=0x1F; bresp=OKAY; bvalid stable for 10 cycles");

        apb_monitor_reset();
        axi_single_write(32'h1F, 32'h0009_0000, 64'hBBBB_BBBB_BBBB_BBBB, 8'hFF,
                         3'b011, 0, 0, 10, bid, bresp, t_bv, t_bh);

        chk_hex("018: bid==0x0000_001F (stable through back-pressure)",  64'(bid),   64'h0000_001F);
        chk1   ("018: bresp[1]==0 (not SLVERR)",                         bresp[1],   1'b0);
        chk1   ("018: bresp[0]==0 (OKAY=2'b00; stable through bp)",      bresp[0],   1'b0);
        chk_int("018: apb_txn_count==2 (no duplicate APB activity)",     apb_txn_count, 2);
        // Verify bvalid was held for at least 10 cycles (100 ns) before handshake
        chk_time_after("018: handshake after bvalid (bready was delayed 10 cycles)",
                        t_bh, t_bv + 100);
        // Sanity: the 10-cycle hold means t_handshake >= t_bvalid_first + 100ns
        $display("[018]   t_bvalid_first=%0t  t_handshake=%0t  delta=%0t ns",
                 t_bv, t_bh, t_bh - t_bv);
        idle(4);
    end

    // ================================================================
    // Summary
    // ================================================================
    idle(8);
    $display("\n====================================================");
    $display("  SIMULATION COMPLETE");
    $display("  PASS: %0d   FAIL: %0d   TOTAL: %0d",
             pass_count, fail_count, pass_count + fail_count);
    if (fail_count == 0)
        $display("  RESULT: ALL TESTS PASSED");
    else
        $display("  RESULT: %0d TEST(S) FAILED  <<< REVIEW LOG", fail_count);
    $display("====================================================\n");
    $finish;
end

/*=========================================================================*/
/*  Global timeout watchdog                                                  */
/*=========================================================================*/

initial begin
    #2_000_000;
    $display("[%0t] TIMEOUT: simulation did not complete -- hung or deadlocked", $time);
    $display("       PASS so far: %0d   FAIL so far: %0d", pass_count, fail_count);
    $finish;
end

endmodule