`timescale 1ns/1ps

/*------------------------------------------------------------------------------
 * File          : tb_burst.sv
 * Description   : Burst testbench for the AXI-to-APB bridge.
 *
 * TEST PLAN
 * =========
 * TB_BURST_001  Single-beat write OKAY (regression duplicate of tb_basic)
 * TB_BURST_002  4-beat write burst OKAY
 * TB_BURST_003  Single-beat read OKAY (assembler correctness)
 * TB_BURST_004  4-beat read burst OKAY
 * TB_BURST_005  Write burst SLVERR propagation (beat 1 MSB)
 * TB_BURST_006  Read burst SLVERR propagation (beat 0 LSB)
 * TB_BURST_007  Narrow write size=2 (4B per beat, 2 beats)
 * TB_BURST_008  Concurrent write + read (both outstanding slots used)
 * TB_BURST_009  Back-to-back write bursts with no idle gap
 * TB_BURST_010  Maximum-length write burst (awlen=7, 8 beats)
 * TB_BURST_011  Anti-starvation stress (6 writes concurrent with 1 read)
 * TB_BURST_012  Write-write same cache line (serial collision)
 * TB_BURST_013  Write-write different cache lines (concurrent)
 * TB_BURST_014  WaR same cache line - serialised by war_collision (updated)
 *
 * DISPLAY CONVENTIONS
 * ===================
 * Every check prints a self-contained line:
 *   [<time>] PASS|FAIL  <label>  exp=<value>  got=<value>
 * The label always embeds the expected value so the log is readable without
 * cross-referencing the source, e.g.:
 *   [150] PASS  002: txn0 paddr==0x00002000  exp=0x0000000000002000  got=...
 *
 * Raw bus events are logged by:
 *   collect_b / collect_r  -- AXI handshake lines
 *   APB slave always_ff    -- APB SLAVE WRITE/READ per transaction
 *   APB monitor always_ff  -- APB MONITOR txn# line, pwrite, paddr
 *
 * Per-test APB transaction counter (apb_txn_count):
 *   Reset before each test with apb_monitor_reset().
 *   Checked with chk_int() to verify the exact number of APB sub-txns.
 *   The monitor also records pwrite[] and paddr[] so address-stride and
 *   ordering checks can be performed after the fact.
 *
 * APB SLAVE MODEL
 * ===============
 * Byte-addressed 32-bit memory.  Word index = paddr[15:2] (bottom 64 KB).
 * For addresses >= 64 KB (tests 011-014), a 64-entry upper bank indexed by
 * paddr[7:2] is used.
 * READ returns ~paddr -- a unique, address-derived value that makes
 * assembler correctness trivially verifiable without pre-loading memory.
 * Error injection: set apb_err_addr to the exact paddr that should fire
 * pslverr=1.  One-shot: auto-clears after firing once.
 *------------------------------------------------------------------------------*/

module tb_burst;

import struct_types::*;

/*=========================================================================*/
/*  Parameters                                                              */
/*=========================================================================*/

localparam APB_MEM_WORDS = 16384; // 64 KB / 4 bytes per word

/*=========================================================================*/
/*  Clock / reset                                                           */
/*=========================================================================*/

logic clk;
logic rst_n;

always #3 clk = ~clk;  // ~166 MHz

/*=========================================================================*/
/*  Interface instances                                                     */
/*=========================================================================*/

axi_interface axi_if();
apb_interface apb_if();

/*=========================================================================*/
/*  AXI driver signals                                                      */
/*=========================================================================*/

logic                    awvalid_tb;
logic [ID_WIDTH-1:0]     awid_tb;
logic [ADDR_WIDTH-1:0]   awaddr_tb;
logic [MAX_LEN-1:0]      awlen_tb;
logic [MAX_SIZE-1:0]     awsize_tb;

logic                    wvalid_tb;
logic [DATA_WIDTH-1:0]   wdata_tb;
logic [WSTRB_WIDTH-1:0]  wstrb_tb;
logic                    wlast_tb;

logic                    bready_tb;

logic                    arvalid_tb;
logic [ID_WIDTH-1:0]     arid_tb;
logic [ADDR_WIDTH-1:0]   araddr_tb;
logic [MAX_LEN-1:0]      arlen_tb;
logic [MAX_SIZE-1:0]     arsize_tb;

logic                    rready_tb;

/*=========================================================================*/
/*  APB slave model (variables declared for use in initial block)          */
/*=========================================================================*/

logic [31:0]           apb_mem       [0:APB_MEM_WORDS-1];
logic [31:0]           apb_upper_mem [0:63];
logic [ADDR_WIDTH-1:0] apb_err_addr;  // '1 = no error injected

initial begin
    clk = 0;
    rst_n = 0;
    // Initialize AXI driver signals
    awvalid_tb = 0;
    awid_tb    = '0;
    awaddr_tb  = '0;
    awlen_tb   = '0;
    awsize_tb  = '0;
    wvalid_tb  = 0;
    wdata_tb   = '0;
    wstrb_tb   = '0;
    wlast_tb   = 0;
    bready_tb  = 0;
    arvalid_tb = 0;
    arid_tb    = '0;
    araddr_tb  = '0;
    arlen_tb   = '0;
    arsize_tb  = '0;
    rready_tb  = 0;
    apb_err_addr = '1;
    
    repeat(4) @(posedge clk);
    @(negedge clk);
    rst_n = 1;
end

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
assign axi_if.arvalid = arvalid_tb;
assign axi_if.arid    = arid_tb;
assign axi_if.araddr  = araddr_tb;
assign axi_if.arlen   = arlen_tb;
assign axi_if.arsize  = arsize_tb;
assign axi_if.rready  = rready_tb;

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

int pass_count;
int fail_count;

// Helper task to log results and update counters consistently
task automatic log_result(
    input string label,
    input logic  is_pass
);
    if (is_pass) begin
        $display("[%0t] PASS  %s", $time, label);
        pass_count++;
    end else begin
        $display("[%0t] FAIL  %s  <<< MISMATCH", $time, label);
        fail_count++;
    end
endtask

// 1-bit logic check.
// Label should embed the expected value so the log line is self-contained,
// e.g. "001: bresp[1]==0(OKAY)" makes the pass/fail readable without source.
task automatic chk1(
    input string label,
    input logic  got,
    input logic  exp
);
    if (got === exp) begin
        $display("[%0t] PASS  %-60s  exp=%0b  got=%0b",
                 $time, label, exp, got);
        pass_count++;
    end else begin
        $display("[%0t] FAIL  %-60s  exp=%0b  got=%0b  <<< MISMATCH",
                 $time, label, exp, got);
        fail_count++;
    end
endtask

// 64-bit hex check.  Uses full fixed-width formatting (016h) so leading
// zeros are preserved and 32-bit IDs/addresses are unambiguous.
task automatic chk_hex(
    input string       label,
    input logic [63:0] got,
    input logic [63:0] exp
);
    if (got === exp) begin
        $display("[%0t] PASS  %-60s  exp=0x%016h  got=0x%016h",
                 $time, label, exp, got);
        pass_count++;
    end else begin
        $display("[%0t] FAIL  %-60s  exp=0x%016h  got=0x%016h  <<< MISMATCH",
                 $time, label, exp, got);
        fail_count++;
    end
endtask

// Integer check (APB transaction counts, beat indices, etc.).
task automatic chk_int(
    input string label,
    input int    got,
    input int    exp
);
    if (got === exp) begin
        $display("[%0t] PASS  %-60s  exp=%0d  got=%0d",
                 $time, label, exp, got);
        pass_count++;
    end else begin
        $display("[%0t] FAIL  %-60s  exp=%0d  got=%0d  <<< MISMATCH",
                 $time, label, exp, got);
        fail_count++;
    end
endtask

// Time-ordering check: assert t_later > t_earlier (strict).
task automatic chk_time_after(
    input string label,
    input time   t_later,
    input time   t_earlier
);
    if (t_later > t_earlier) begin
        $display("[%0t] PASS  %-60s  t_earlier=%0t  t_later=%0t",
                 $time, label, t_earlier, t_later);
        pass_count++;
    end else begin
        $display("[%0t] FAIL  %-60s  t_earlier=%0t  t_later=%0t (NOT after!)  <<< MISMATCH",
                 $time, label, t_earlier, t_later);
        fail_count++;
    end
endtask

/*=========================================================================*/
/*  APB monitor + per-test transaction recorder                             */
/*=========================================================================*/
// Fires on every completed APB transfer (psel & penable & pready).
// apb_txn_count increments each time; reset via apb_monitor_reset().
// pwrite and paddr of each txn are stored so callers can check
// ordering, address stride, and direction (READ vs WRITE) after the burst.

logic [31:0]           apb_txn_count;
logic                  apb_txn_pwrite [0:63];
logic [ADDR_WIDTH-1:0] apb_txn_paddr  [0:63];
logic                  apb_monitor_do_reset;  // Reset control signal

always_ff @(posedge clk) begin
    if (apb_monitor_do_reset) begin
        apb_txn_count <= 0;
    end else if (rst_n && apb_if.psel && apb_if.penable && apb_if.pready) begin
        if (apb_txn_count < 64) begin
            apb_txn_pwrite[apb_txn_count] <= apb_if.pwrite;
            apb_txn_paddr [apb_txn_count] <= apb_if.paddr;
        end
        apb_txn_count <= apb_txn_count + 1;
        $display("[%0t] APB MONITOR  txn#%0d  %s  paddr=0x%08h  pwrite=%0b",
                 $time, apb_txn_count,
                 apb_if.pwrite ? "WRITE" : "READ ",
                 apb_if.paddr, apb_if.pwrite);
    end
end

task automatic apb_monitor_reset();
    @(negedge clk);
    apb_monitor_do_reset = 1;
    @(posedge clk);
    apb_monitor_do_reset = 0;
endtask

/*=========================================================================*/
/*  APB slave model                                                         */
/*=========================================================================*/

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

            // Error injection (one-shot)
            if (apb_if.paddr == apb_err_addr) begin
                pslverr_tb  <= 1;
                $display("[%0t] APB SLAVE  SLVERR injected on paddr=0x%08h",
                         $time, apb_if.paddr);
            end

            if (apb_if.pwrite) begin
                // Perform memory write in always_ff to maintain single-procedure assignment
                begin
                    logic [31:0] old;
                    old = (apb_if.paddr[31:16] == 16'h0) ? apb_mem[apb_if.paddr[15:2]] : apb_upper_mem[apb_if.paddr[7:2]];
                    for (int b = 0; b < 4; b++)
                        if (apb_if.pstrb[b]) old[b*8 +: 8] = apb_if.pwdata[b*8 +: 8];
                    if (apb_if.paddr[31:16] == 16'h0) apb_mem[apb_if.paddr[15:2]]    = old;
                    else                     apb_upper_mem[apb_if.paddr[7:2]] = old;
                end
                $display("[%0t] APB SLAVE  WRITE  paddr=0x%08h  pwdata=0x%08h  pstrb=4'b%04b",
                         $time, apb_if.paddr, apb_if.pwdata, apb_if.pstrb);
            end else begin
                // Read returns ~paddr: unique per address, easy to verify
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

task automatic drive_aw(
    input logic [ID_WIDTH-1:0]   id,
    input logic [ADDR_WIDTH-1:0] addr,
    input logic [MAX_LEN-1:0]    len,
    input logic [MAX_SIZE-1:0]   size
);
    @(negedge clk);
    awvalid_tb = 1; awid_tb = id; awaddr_tb = addr;
    awlen_tb = len; awsize_tb = size;
    @(posedge clk);
    while (!axi_if.awready) @(posedge clk);
    @(negedge clk);
    awvalid_tb = 0;
endtask

task automatic drive_w(
    input logic [DATA_WIDTH-1:0]  data,
    input logic [WSTRB_WIDTH-1:0] strb,
    input logic                   last
);
    @(negedge clk);
    wvalid_tb = 1; wdata_tb = data; wstrb_tb = strb; wlast_tb = last;
    @(posedge clk);
    while (!axi_if.wready) @(posedge clk);
    @(negedge clk);
    wvalid_tb = 0; wlast_tb = 0;
endtask

// Returns bid, bresp, and the $time of the handshake.
// Prints a self-contained log line with all field values.
task automatic collect_b(
    output logic [ID_WIDTH-1:0]    bid_out,
    output logic [BRESP_WIDTH-1:0] bresp_out,
    output time                    t_out
);
    @(negedge clk);
    bready_tb = 1;
    @(posedge clk);
    while (!axi_if.bvalid) @(posedge clk);
    bid_out   = axi_if.bid;
    bresp_out = axi_if.bresp;
    t_out     = $time;
    $display("[%0t] AXI B RESP  bid=0x%08h  bresp=2'b%02b (%s)",
             $time, bid_out, bresp_out,
             (bresp_out == 2'b00) ? "OKAY" : "SLVERR");
    @(negedge clk);
    bready_tb = 0;
endtask

task automatic drive_ar(
    input logic [ID_WIDTH-1:0]   id,
    input logic [ADDR_WIDTH-1:0] addr,
    input logic [MAX_LEN-1:0]    len,
    input logic [MAX_SIZE-1:0]   size
);
    @(negedge clk);
    arvalid_tb = 1; arid_tb = id; araddr_tb = addr;
    arlen_tb = len; arsize_tb = size;
    @(posedge clk);
    while (!axi_if.arready) @(posedge clk);
    @(negedge clk);
    arvalid_tb = 0;
endtask

// Returns all R fields and the $time of the handshake.
// Prints a self-contained log line with all field values.
task automatic collect_r(
    output logic [ID_WIDTH-1:0]    rid_out,
    output logic [DATA_WIDTH-1:0]  rdata_out,
    output logic [RRESP_WIDTH-1:0] rresp_out,
    output logic                   rlast_out,
    output time                    t_out
);
    @(negedge clk);
    rready_tb = 1;
    @(posedge clk);
    while (!axi_if.rvalid) @(posedge clk);
    rid_out   = axi_if.rid;
    rdata_out = axi_if.rdata;
    rresp_out = axi_if.rresp;
    rlast_out = axi_if.rlast;
    t_out     = $time;
    $display("[%0t] AXI R DATA  rid=0x%08h  rdata=0x%016h  rresp=2'b%02b (%s)  rlast=%0b",
             $time, rid_out, rdata_out, rresp_out,
             (rresp_out == 2'b00) ? "OKAY" : "SLVERR", rlast_out);
    @(negedge clk);
    rready_tb = 0;
endtask

task automatic axi_write_burst(
    input  logic [ID_WIDTH-1:0]    id,
    input  logic [ADDR_WIDTH-1:0]  addr,
    input  logic [MAX_LEN-1:0]     len,
    input  logic [MAX_SIZE-1:0]    size,
    input  logic [DATA_WIDTH-1:0]  data [],
    input  logic [WSTRB_WIDTH-1:0] strb [],
    output logic [ID_WIDTH-1:0]    bid_out,
    output logic [BRESP_WIDTH-1:0] bresp_out,
    output time                    t_bresp
);
    fork drive_aw(id, addr, len, size); join_none
    for (int i = 0; i <= int'(len); i++)
        drive_w(data[i], strb[i], (i == int'(len)));
    collect_b(bid_out, bresp_out, t_bresp);
endtask

task automatic axi_read_burst(
    input  logic [ID_WIDTH-1:0]    id,
    input  logic [ADDR_WIDTH-1:0]  addr,
    input  logic [MAX_LEN-1:0]     len,
    input  logic [MAX_SIZE-1:0]    size,
    output logic [ID_WIDTH-1:0]    rid_out   [],
    output logic [DATA_WIDTH-1:0]  rdata_out [],
    output logic [RRESP_WIDTH-1:0] rresp_out [],
    output logic                   rlast_out [],
    output time                    t_first_r
);
    int  beats;
    time t_tmp;
    beats = int'(len) + 1;
    rid_out   = new [beats];
    rdata_out = new [beats];
    rresp_out = new [beats];
    rlast_out = new [beats];
    drive_ar(id, addr, len, size);
    for (int i = 0; i < beats; i++) begin
        collect_r(rid_out[i], rdata_out[i], rresp_out[i], rlast_out[i], t_tmp);
        if (i == 0) t_first_r = t_tmp;
    end
endtask

task automatic idle(input int cycles);
    repeat(cycles) @(posedge clk);
endtask

/*=========================================================================*/
/*  Stimulus                                                                */
/*=========================================================================*/

initial begin
    $dumpfile("tb_burst.vcd");
    $dumpvars(0, tb_burst);

    @(posedge rst_n);
    idle(2);

    // ================================================================
    // TB_BURST_001  Single-beat write OKAY
    //
    // Stimulus : awid=0x1  addr=0x0000_1000  awlen=0  awsize=3(8B)
    //            wdata=0xDEAD_BEEF_CAFE_1234  wstrb=8'hFF  wlast=1
    // APB expected (2 txns, in order):
    //   txn0: WRITE paddr=0x0000_1000  pwdata=0xCAFE_1234  pstrb=4'b1111
    //   txn1: WRITE paddr=0x0000_1004  pwdata=0xDEAD_BEEF  pstrb=4'b1111
    // AXI expected: bid=0x0000_0001  bresp=2'b00(OKAY)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        logic [DATA_WIDTH-1:0]  wd[];
        logic [WSTRB_WIDTH-1:0] ws[];
        time                    t_bresp;

        $display("\n====== TB_BURST_001: Single-beat write OKAY ======");
        $display("[001] Stimulus: awid=0x1 addr=0x0000_1000 awlen=0 awsize=3");
        $display("[001] Expected: 2 APB WRITE txns; bid=0x1; bresp=2'b00(OKAY)");

        apb_monitor_reset();
        wd = new[1]; ws = new[1];
        wd[0] = 64'hDEAD_BEEF_CAFE_1234;
        ws[0] = 8'hFF;

        axi_write_burst(32'h1, 32'h0000_1000, 8'h00, 3'b011,
                        wd, ws, bid, bresp, t_bresp);

        // --- AXI response checks ---
        chk_hex("001: bid==0x0000_0001",             64'(bid),    64'h0000_0001);
        chk1   ("001: bresp[1]==0 (not SLVERR)",     bresp[1],    1'b0);
        chk1   ("001: bresp[0]==0 (OKAY=2'b00)",     bresp[0],    1'b0);
        // --- APB structural checks ---
        chk_int("001: apb_txn_count==2 (LSB+MSB)",   apb_txn_count, 2);
        chk1   ("001: txn0 pwrite==1 (WRITE)",       apb_txn_pwrite[0], 1'b1);
        chk_hex("001: txn0 paddr==0x0000_1000 (LSB)",64'(apb_txn_paddr[0]), 64'h0000_1000);
        chk1   ("001: txn1 pwrite==1 (WRITE)",       apb_txn_pwrite[1], 1'b1);
        chk_hex("001: txn1 paddr==0x0000_1004 (MSB)",64'(apb_txn_paddr[1]), 64'h0000_1004);
        idle(4);
    end

    // ================================================================
    // TB_BURST_002  4-beat write burst OKAY
    //
    // Stimulus : awid=0x2  addr=0x0000_2000  awlen=3  awsize=3(8B)
    //            beat i: wdata={0xAAAA_000i+1, 0xBBBB_000i+1}  wstrb=8'hFF
    // APB expected (8 txns, 2 per beat, sequential addresses):
    //   txn0: WRITE 0x2000   txn1: WRITE 0x2004
    //   txn2: WRITE 0x2008   txn3: WRITE 0x200C
    //   txn4: WRITE 0x2010   txn5: WRITE 0x2014
    //   txn6: WRITE 0x2018   txn7: WRITE 0x201C
    // AXI expected: bid=0x0000_0002  bresp=2'b00(OKAY)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        logic [DATA_WIDTH-1:0]  wd[];
        logic [WSTRB_WIDTH-1:0] ws[];
        time                    t_bresp;

        $display("\n====== TB_BURST_002: 4-beat write burst OKAY ======");
        $display("[002] Stimulus: awid=0x2 addr=0x2000 awlen=3 awsize=3");
        $display("[002] Expected: 8 APB WRITE txns at 0x2000..0x201C stride 4; bid=0x2; bresp=OKAY");

        apb_monitor_reset();
        wd = new[4]; ws = new[4];
        for (int i = 0; i < 4; i++) begin
            wd[i] = {32'hAAAA_0000 + i + 1, 32'hBBBB_0000 + i + 1};
            ws[i] = 8'hFF;
        end

        axi_write_burst(32'h2, 32'h0000_2000, 8'h03, 3'b011,
                        wd, ws, bid, bresp, t_bresp);

        // --- AXI response checks ---
        chk_hex("002: bid==0x0000_0002",         64'(bid),   64'h0000_0002);
        chk1   ("002: bresp[1]==0 (not SLVERR)", bresp[1],   1'b0);
        chk1   ("002: bresp[0]==0 (OKAY=2'b00)", bresp[0],   1'b0);
        // --- APB structural checks ---
        chk_int("002: apb_txn_count==8 (2 per beat * 4 beats)", apb_txn_count, 8);
        for (int i = 0; i < 8; i++) begin
            logic [ADDR_WIDTH-1:0] exp_pa;
            exp_pa = 32'h0000_2000 + (i * 4);
            chk1   ($sformatf("002: txn%0d pwrite==1 (WRITE)", i),
                    apb_txn_pwrite[i], 1'b1);
            chk_hex($sformatf("002: txn%0d paddr==0x%08h", i, exp_pa),
                    64'(apb_txn_paddr[i]), 64'(exp_pa));
        end
        idle(4);
    end

    // ================================================================
    // TB_BURST_003  Single-beat read OKAY
    //
    // Stimulus : arid=0x3  addr=0x0000_3000  arlen=0  arsize=3(8B)
    // APB expected (2 txns):
    //   txn0: READ paddr=0x3000 -> slave prdata=~0x3000 = 0xFFFF_CFFF
    //   txn1: READ paddr=0x3004 -> slave prdata=~0x3004 = 0xFFFF_CFFB
    // AXI expected:
    //   rid=0x3  rdata[63:32]=0xFFFF_CFFB  rdata[31:0]=0xFFFF_CFFF
    //   rresp=2'b00(OKAY)  rlast=1
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    rid[];
        logic [DATA_WIDTH-1:0]  rdata[];
        logic [RRESP_WIDTH-1:0] rresp[];
        logic                   rlast[];
        time                    t_first_r;

        $display("\n====== TB_BURST_003: Single-beat read OKAY ======");
        $display("[003] Stimulus: arid=0x3 addr=0x3000 arlen=0 arsize=3");
        $display("[003] Expected: 2 APB READ txns; rdata={~0x3004,~0x3000}; rresp=OKAY; rlast=1");

        apb_monitor_reset();
        axi_read_burst(32'h3, 32'h0000_3000, 8'h00, 3'b011,
                       rid, rdata, rresp, rlast, t_first_r);

        // --- AXI response checks ---
        chk_hex("003: rid==0x0000_0003",                   64'(rid[0]),         64'h0000_0003);
        chk_hex("003: rdata[63:32]==~0x3004(=0xFFFF_CFFB)",64'(rdata[0][63:32]),64'(~32'h0000_3004));
        chk_hex("003: rdata[31:0]==~0x3000(=0xFFFF_CFFF)", 64'(rdata[0][31:0]), 64'(~32'h0000_3000));
        chk1   ("003: rresp[1]==0 (not SLVERR)",           rresp[0][1],         1'b0);
        chk1   ("003: rresp[0]==0 (OKAY=2'b00)",           rresp[0][0],         1'b0);
        chk1   ("003: rlast==1 (single beat must assert)", rlast[0],            1'b1);
        // --- APB structural checks ---
        chk_int("003: apb_txn_count==2 (LSB+MSB)",         apb_txn_count, 2);
        chk1   ("003: txn0 pwrite==0 (READ)",              apb_txn_pwrite[0], 1'b0);
        chk_hex("003: txn0 paddr==0x0000_3000 (LSB)",      64'(apb_txn_paddr[0]), 64'h0000_3000);
        chk1   ("003: txn1 pwrite==0 (READ)",              apb_txn_pwrite[1], 1'b0);
        chk_hex("003: txn1 paddr==0x0000_3004 (MSB)",      64'(apb_txn_paddr[1]), 64'h0000_3004);
        idle(4);
    end

    // ================================================================
    // TB_BURST_004  4-beat read burst OKAY
    //
    // Stimulus : arid=0x4  addr=0x0000_4000  arlen=3  arsize=3(8B)
    // For beat i: beat_addr = 0x4000 + i*8
    //   APB txn 2i  : READ paddr=beat_addr,   prdata=~beat_addr
    //   APB txn 2i+1: READ paddr=beat_addr+4, prdata=~(beat_addr+4)
    //   AXI rdata[i] = { ~(beat_addr+4)[31:0], ~beat_addr[31:0] }
    // AXI expected: rid=0x4 stable; rresp=OKAY all beats; rlast=1 on beat3 only
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    rid[];
        logic [DATA_WIDTH-1:0]  rdata[];
        logic [RRESP_WIDTH-1:0] rresp[];
        logic                   rlast[];
        time                    t_first_r;

        $display("\n====== TB_BURST_004: 4-beat read burst OKAY ======");
        $display("[004] Stimulus: arid=0x4 addr=0x4000 arlen=3 arsize=3");
        $display("[004] Expected: 8 APB READ txns; rdata[i]={~(addr+4),~addr}; rlast on beat3 only");

        apb_monitor_reset();
        axi_read_burst(32'h4, 32'h0000_4000, 8'h03, 3'b011,
                       rid, rdata, rresp, rlast, t_first_r);

        // --- AXI response checks per beat ---
        for (int i = 0; i < 4; i++) begin
            logic [ADDR_WIDTH-1:0] ba;
            ba = 32'h0000_4000 + (i * 8);
            chk_hex($sformatf("004: beat%0d rid==0x0000_0004", i),
                    64'(rid[i]), 64'h0000_0004);
            chk_hex($sformatf("004: beat%0d rdata[63:32]==~0x%08h", i, ba+4),
                    64'(rdata[i][63:32]), 64'(~(ba+4)));
            chk_hex($sformatf("004: beat%0d rdata[31:0]==~0x%08h", i, ba),
                    64'(rdata[i][31:0]),  64'(~ba));
            chk1   ($sformatf("004: beat%0d rresp[1]==0 (OKAY)", i),
                    rresp[i][1], 1'b0);
            chk1   ($sformatf("004: beat%0d rresp[0]==0", i),
                    rresp[i][0], 1'b0);
            chk1   ($sformatf("004: beat%0d rlast==%0b", i, (i==3)),
                    rlast[i], logic'(i==3));
        end
        // --- APB structural checks ---
        chk_int("004: apb_txn_count==8 (2 per beat * 4 beats)", apb_txn_count, 8);
        for (int i = 0; i < 8; i++) begin
            logic [ADDR_WIDTH-1:0] exp_pa;
            exp_pa = 32'h0000_4000 + (i * 4);
            chk1   ($sformatf("004: txn%0d pwrite==0 (READ)", i),
                    apb_txn_pwrite[i], 1'b0);
            chk_hex($sformatf("004: txn%0d paddr==0x%08h", i, exp_pa),
                    64'(apb_txn_paddr[i]), 64'(exp_pa));
        end
        idle(4);
    end

    // ================================================================
    // TB_BURST_005  Write burst SLVERR propagation (beat1 MSB)
    //
    // Stimulus : awid=0x5  addr=0x0000_5000  awlen=2(3 beats)  awsize=3(8B)
    //            wstrb=8'hFF each beat
    //            pslverr injected on paddr=0x500C (beat1 MSB = APB txn#3)
    // APB txn order (6 total):
    //   txn0: WRITE 0x5000 OKAY   txn1: WRITE 0x5004 OKAY
    //   txn2: WRITE 0x5008 OKAY   txn3: WRITE 0x500C SLVERR  <<< injected
    //   txn4: WRITE 0x5010 OKAY   txn5: WRITE 0x5014 OKAY
    // AXI expected: bid=0x5  bresp=2'b10(SLVERR)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        logic [DATA_WIDTH-1:0]  wd[];
        logic [WSTRB_WIDTH-1:0] ws[];
        time                    t_bresp;

        $display("\n====== TB_BURST_005: Write burst SLVERR propagation ======");
        $display("[005] Stimulus: awid=0x5 addr=0x5000 awlen=2 awsize=3");
        $display("[005] Inject:   pslverr on paddr=0x500C (beat1 MSB, txn#3)");
        $display("[005] Expected: 6 APB WRITE txns; bid=0x5; bresp=2'b10(SLVERR)");

        apb_monitor_reset();
        wd = new[3]; ws = new[3];
        for (int i = 0; i < 3; i++) begin
            wd[i] = 64'hFFFF_0000_EEEE_0000 + i;
            ws[i] = 8'hFF;
        end
        apb_err_addr = 32'h0000_500C;

        axi_write_burst(32'h5, 32'h0000_5000, 8'h02, 3'b011,
                        wd, ws, bid, bresp, t_bresp);

        // --- AXI response checks ---
        chk_hex("005: bid==0x0000_0005",                      64'(bid),   64'h0000_0005);
        chk1   ("005: bresp[1]==1 (SLVERR bit set)",          bresp[1],   1'b1);
        chk1   ("005: bresp[0]==0 (SLVERR=2'b10 not 2'b11)", bresp[0],   1'b0);
        // --- APB structural checks ---
        chk_int("005: apb_txn_count==6 (3 beats * 2 halves)",apb_txn_count, 6);
        // Confirm the SLVERR was on txn3 (index 3, zero-based)
        chk_hex("005: txn3 paddr==0x0000_500C (SLVERR txn)", 64'(apb_txn_paddr[3]), 64'h0000_500C);
        chk1   ("005: txn3 pwrite==1 (was a WRITE)",         apb_txn_pwrite[3],     1'b1);
        apb_err_addr = '1;  // Clear error injection
        idle(4);
    end

    // ================================================================
    // TB_BURST_006  Read burst SLVERR propagation (beat0 LSB)
    //
    // Stimulus : arid=0x6  addr=0x0000_6000  arlen=1(2 beats)  arsize=3(8B)
    //            pslverr injected on paddr=0x6000 (beat0 LSB = APB txn#0)
    // APB txn order (4 total):
    //   txn0: READ 0x6000 SLVERR  <<< injected
    //   txn1: READ 0x6004 OKAY
    //   txn2: READ 0x6008 OKAY
    //   txn3: READ 0x600C OKAY
    // AXI expected:
    //   beat0: rid=0x6  rresp=2'b10(SLVERR)  rlast=0
    //   beat1: rid=0x6  rresp=2'b00(OKAY)    rlast=1
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    rid[];
        logic [DATA_WIDTH-1:0]  rdata[];
        logic [RRESP_WIDTH-1:0] rresp[];
        logic                   rlast[];
        time                    t_first_r;

        $display("\n====== TB_BURST_006: Read burst SLVERR propagation ======");
        $display("[006] Stimulus: arid=0x6 addr=0x6000 arlen=1 arsize=3");
        $display("[006] Inject:   pslverr on paddr=0x6000 (beat0 LSB, txn#0)");
        $display("[006] Expected: beat0 rresp=SLVERR rlast=0; beat1 rresp=OKAY rlast=1");

        apb_monitor_reset();
        apb_err_addr = 32'h0000_6000;

        axi_read_burst(32'h6, 32'h0000_6000, 8'h01, 3'b011,
                       rid, rdata, rresp, rlast, t_first_r);

        // --- AXI response checks beat0 ---
        chk_hex("006: beat0 rid==0x0000_0006",                 64'(rid[0]),   64'h0000_0006);
        chk1   ("006: beat0 rresp[1]==1 (SLVERR bit set)",    rresp[0][1],   1'b1);
        chk1   ("006: beat0 rresp[0]==0 (SLVERR=2'b10)",      rresp[0][0],   1'b0);
        chk1   ("006: beat0 rlast==0 (not last beat)",        rlast[0],      1'b0);
        // --- AXI response checks beat1 ---
        chk_hex("006: beat1 rid==0x0000_0006",                 64'(rid[1]),   64'h0000_0006);
        chk1   ("006: beat1 rresp[1]==0 (not SLVERR)",        rresp[1][1],   1'b0);
        chk1   ("006: beat1 rresp[0]==0 (OKAY=2'b00)",        rresp[1][0],   1'b0);
        chk1   ("006: beat1 rlast==1 (last beat of burst)",   rlast[1],      1'b1);
        // --- APB structural checks ---
        chk_int("006: apb_txn_count==4 (2 beats * 2 halves)", apb_txn_count, 4);
        chk1   ("006: txn0 pwrite==0 (READ)",                 apb_txn_pwrite[0], 1'b0);
        chk_hex("006: txn0 paddr==0x0000_6000 (SLVERR txn)", 64'(apb_txn_paddr[0]), 64'h0000_6000);
        apb_err_addr = '1;  // Clear error injection
        idle(4);
    end

    // ================================================================
    // TB_BURST_007  Narrow write size=2 (4B per beat, 2 beats)
    //
    // Stimulus : awid=0x7  addr=0x0000_7000  awlen=1  awsize=2(4B)
    //   beat0: beat_addr=0x7000  addr[2]=0 -> LSB half only
    //     wdata=0xCCCC_CCCC_1111_2222  wstrb=8'h0F
    //     -> APB txn0: WRITE paddr=0x7000 pwdata=0x1111_2222 pstrb=4'b1111
    //   beat1: beat_addr=0x7004  addr[2]=1 -> MSB half only
    //     wdata=0xDDDD_DDDD_3333_4444  wstrb=8'hF0
    //     -> APB txn1: WRITE paddr=0x7004 pwdata=0xDDDD_DDDD pstrb=4'b1111
    // AXI expected: bid=0x7  bresp=2'b00(OKAY)
    // KEY check: apb_txn_count==2 (1 per beat, NOT 2) -- narrow select working
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        logic [DATA_WIDTH-1:0]  wd[];
        logic [WSTRB_WIDTH-1:0] ws[];
        time                    t_bresp;

        $display("\n====== TB_BURST_007: Narrow write size=2 (4B/beat) ======");
        $display("[007] Stimulus: awid=0x7 addr=0x7000 awlen=1 awsize=2 (narrow 4B)");
        $display("[007] Expected: 2 APB WRITE txns (1/beat); bid=0x7; bresp=OKAY");
        $display("[007]   txn0: paddr=0x7000 (beat0 addr[2]=0 -> LSB selected)");
        $display("[007]   txn1: paddr=0x7004 (beat1 addr[2]=1 -> MSB selected)");

        apb_monitor_reset();
        wd = new[2]; ws = new[2];
        wd[0] = 64'hCCCC_CCCC_1111_2222;  ws[0] = 8'h0F;  // addr[2]=0 -> LSB
        wd[1] = 64'hDDDD_DDDD_3333_4444;  ws[1] = 8'hF0;  // addr[2]=1 -> MSB

        axi_write_burst(32'h7, 32'h0000_7000, 8'h01, 3'b010,
                        wd, ws, bid, bresp, t_bresp);

        // --- AXI response checks ---
        chk_hex("007: bid==0x0000_0007",                               64'(bid),   64'h0000_0007);
        chk1   ("007: bresp[1]==0 (not SLVERR)",                       bresp[1],   1'b0);
        chk1   ("007: bresp[0]==0 (OKAY=2'b00)",                       bresp[0],   1'b0);
        // --- APB structural checks (narrow-select correctness) ---
        chk_int("007: apb_txn_count==2 (narrow: 1 per beat, not 2)",  apb_txn_count, 2);
        chk1   ("007: txn0 pwrite==1 (WRITE)",                         apb_txn_pwrite[0], 1'b1);
        chk_hex("007: txn0 paddr==0x0000_7000 (beat0 LSB, addr[2]=0)",64'(apb_txn_paddr[0]), 64'h0000_7000);
        chk1   ("007: txn1 pwrite==1 (WRITE)",                         apb_txn_pwrite[1], 1'b1);
        chk_hex("007: txn1 paddr==0x0000_7004 (beat1 MSB, addr[2]=1)",64'(apb_txn_paddr[1]), 64'h0000_7004);
        idle(4);
    end

    // ================================================================
    // TB_BURST_008  Concurrent write + read (both outstanding slots used)
    //
    // Stimulus : WR awid=0x8  addr=0x0000_8000  awlen=1  awsize=3
    //            RD arid=0x8  addr=0x0000_8100  arlen=1  arsize=3
    //            Addresses are in different 64-byte CLs -> no WaR collision
    //            AR launched concurrently while W beats are being driven
    // APB expected: 4 WR txns + 4 RD txns = 8 total (interleaved)
    // AXI expected:
    //   WR: bid=0x8  bresp=2'b00(OKAY)
    //   RD beat0: rid=0x8  rresp=2'b00(OKAY)  rlast=0
    //   RD beat1: rid=0x8  rresp=2'b00(OKAY)  rlast=1
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        logic [ID_WIDTH-1:0]    rid[];
        logic [DATA_WIDTH-1:0]  rdata[];
        logic [RRESP_WIDTH-1:0] rresp[];
        logic                   rlast[];
        time                    t_bresp, t_first_r, t_tmp;

        $display("\n====== TB_BURST_008: Concurrent write + read ======");
        $display("[008] Stimulus: WR awid=0x8 addr=0x8000 awlen=1; RD arid=0x8 addr=0x8100 arlen=1");
        $display("[008] Expected: 8 APB txns (4 WR + 4 RD); WR bresp=OKAY; RD all rresp=OKAY; rlast on beat1");

        apb_monitor_reset();

        fork drive_aw(32'h8, 32'h0000_8000, 8'h01, 3'b011); join_none
        fork drive_ar(32'h8, 32'h0000_8100, 8'h01, 3'b011); join_none

        drive_w(64'hAAAA_AAAA_1111_1111, 8'hFF, 1'b0);
        drive_w(64'hBBBB_BBBB_2222_2222, 8'hFF, 1'b1);

        collect_b(bid, bresp, t_bresp);

        rid = new[2]; rdata = new[2]; rresp = new[2]; rlast = new[2];
        collect_r(rid[0], rdata[0], rresp[0], rlast[0], t_first_r);
        collect_r(rid[1], rdata[1], rresp[1], rlast[1], t_tmp);

        // --- AXI WR response checks ---
        chk_hex("008: WR bid==0x0000_0008",                 64'(bid),    64'h0000_0008);
        chk1   ("008: WR bresp[1]==0 (not SLVERR)",         bresp[1],    1'b0);
        chk1   ("008: WR bresp[0]==0 (OKAY=2'b00)",         bresp[0],    1'b0);
        // --- AXI RD response checks ---
        chk_hex("008: RD beat0 rid==0x0000_0008",           64'(rid[0]), 64'h0000_0008);
        chk1   ("008: RD beat0 rresp[1]==0 (OKAY)",         rresp[0][1], 1'b0);
        chk1   ("008: RD beat0 rresp[0]==0",                rresp[0][0], 1'b0);
        chk1   ("008: RD beat0 rlast==0 (not last)",        rlast[0],    1'b0);
        chk_hex("008: RD beat1 rid==0x0000_0008",           64'(rid[1]), 64'h0000_0008);
        chk1   ("008: RD beat1 rresp[1]==0 (OKAY)",         rresp[1][1], 1'b0);
        chk1   ("008: RD beat1 rresp[0]==0",                rresp[1][0], 1'b0);
        chk1   ("008: RD beat1 rlast==1 (last beat)",       rlast[1],    1'b1);
        // --- APB structural checks ---
        chk_int("008: apb_txn_count==8 (4 WR + 4 RD)", apb_txn_count, 8);
        idle(4);
    end

    // ================================================================
    // TB_BURST_009  Back-to-back write bursts (no idle gap)
    //
    // Stimulus : WR1 awid=0xC1  addr=0x0000_C100  awlen=1  awsize=3
    //            WR2 awid=0xC2  addr=0x0000_C200  awlen=1  awsize=3
    //            WR2 AW issued right after WR1 W beats (pipeline, no idle)
    //            Addresses in different CLs -> no wr_collision, both slots used
    // APB expected: 8 txns (4 WR1 + 4 WR2)
    // AXI expected:
    //   WR1: bid=0xC1  bresp=2'b00(OKAY)
    //   WR2: bid=0xC2  bresp=2'b00(OKAY)
    //   NOTE: Responses may arrive in any order per AXI spec (out-of-order allowed)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid_resp[2];
        logic [BRESP_WIDTH-1:0] bresp_resp[2];
        time                    t_resp[2];

        $display("\n====== TB_BURST_009: Back-to-back write bursts ======");
        $display("[009] Stimulus: WR1 awid=0xC1 addr=0xC100 awlen=1; WR2 awid=0xC2 addr=0xC200 awlen=1");
        $display("[009] Expected: 8 APB WRITE txns; WR1 bid=0xC1 bresp=OKAY; WR2 bid=0xC2 bresp=OKAY");
        $display("[009]           Responses may arrive out-of-order (matched by bid)");

        apb_monitor_reset();

        fork drive_aw(32'hC1, 32'h0000_C100, 8'h01, 3'b011); join_none
        drive_w(64'hF0F0_F0F0_1234_5678, 8'hFF, 1'b0);
        drive_w(64'hF0F0_F0F0_8765_4321, 8'hFF, 1'b1);

        fork drive_aw(32'hC2, 32'h0000_C200, 8'h01, 3'b011); join_none
        drive_w(64'hA5A5_A5A5_DEAD_BEEF, 8'hFF, 1'b0);
        drive_w(64'hA5A5_A5A5_CAFE_F00D, 8'hFF, 1'b1);

        collect_b(bid_resp[0], bresp_resp[0], t_resp[0]);
        collect_b(bid_resp[1], bresp_resp[1], t_resp[1]);

        // --- AXI response checks (responses may arrive out-of-order) ---
        // Find which response corresponds to WR1 (bid=0xC1) and WR2 (bid=0xC2)
        begin
            int wr1_idx, wr2_idx;
            wr1_idx = -1;
            wr2_idx = -1;
            for (int i = 0; i < 2; i++) begin
                if (bid_resp[i] == 32'h0000_00C1) wr1_idx = i;
                if (bid_resp[i] == 32'h0000_00C2) wr2_idx = i;
            end
            if (wr1_idx >= 0) begin
                chk_hex("009: WR1 bid==0x0000_00C1",             64'(bid_resp[wr1_idx]),   64'h0000_00C1);
                chk1   ("009: WR1 bresp[1]==0 (not SLVERR)",     bresp_resp[wr1_idx][1],   1'b0);
                chk1   ("009: WR1 bresp[0]==0 (OKAY=2'b00)",     bresp_resp[wr1_idx][0],   1'b0);
            end else begin
                log_result("009: WR1 response (bid=0xC1) received", 1'b0);
            end
            if (wr2_idx >= 0) begin
                chk_hex("009: WR2 bid==0x0000_00C2",             64'(bid_resp[wr2_idx]),   64'h0000_00C2);
                chk1   ("009: WR2 bresp[1]==0 (not SLVERR)",     bresp_resp[wr2_idx][1],   1'b0);
                chk1   ("009: WR2 bresp[0]==0 (OKAY=2'b00)",     bresp_resp[wr2_idx][0],   1'b0);
            end else begin
                log_result("009: WR2 response (bid=0xC2) received", 1'b0);
            end
        end
        // --- APB structural checks ---
        chk_int("009: apb_txn_count==8 (4 WR1 + 4 WR2)", apb_txn_count, 8);
        idle(4);
    end

    // ================================================================
    // TB_BURST_010  Maximum-length write burst (awlen=7, 8 beats)
    //
    // Stimulus : awid=0xD0  addr=0x0000_D000  awlen=7(8 beats)  awsize=3(8B)
    //            beat i: wdata={0xFEED_0000+i, 0xDEAD_0000+i}  wstrb=8'hFF
    // APB expected (16 txns, 2 per beat):
    //   txn 2i  : WRITE paddr = 0xD000 + i*8        (beat i LSB)
    //   txn 2i+1: WRITE paddr = 0xD000 + i*8 + 4    (beat i MSB)
    //   i.e. paddr sequence: 0xD000,D004,D008,D00C,...,D038,D03C
    // AXI expected: bid=0xD0  bresp=2'b00(OKAY)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        logic [DATA_WIDTH-1:0]  wd[];
        logic [WSTRB_WIDTH-1:0] ws[];
        time                    t_bresp;

        $display("\n====== TB_BURST_010: Maximum-length write burst (8 beats) ======");
        $display("[010] Stimulus: awid=0xD0 addr=0xD000 awlen=7 awsize=3");
        $display("[010] Expected: 16 APB WRITE txns at 4B stride; bid=0xD0; bresp=OKAY");

        apb_monitor_reset();
        wd = new[8]; ws = new[8];
        for (int i = 0; i < 8; i++) begin
            wd[i] = {32'hFEED_0000 + i, 32'hDEAD_0000 + i};
            ws[i] = 8'hFF;
        end

        axi_write_burst(32'hD0, 32'h0000_D000, 8'h07, 3'b011,
                        wd, ws, bid, bresp, t_bresp);

        // --- AXI response checks ---
        chk_hex("010: bid==0x0000_00D0",          64'(bid),   64'h0000_00D0);
        chk1   ("010: bresp[1]==0 (not SLVERR)",  bresp[1],   1'b0);
        chk1   ("010: bresp[0]==0 (OKAY=2'b00)",  bresp[0],   1'b0);
        // --- APB structural checks: 16 txns, correct address stride ---
        chk_int("010: apb_txn_count==16 (2 per beat * 8 beats)", apb_txn_count, 16);
        for (int i = 0; i < 16; i++) begin
            logic [ADDR_WIDTH-1:0] exp_pa;
            exp_pa = 32'h0000_D000 + (i * 4);
            chk1   ($sformatf("010: txn%02d pwrite==1 (WRITE)", i),
                    apb_txn_pwrite[i], 1'b1);
            chk_hex($sformatf("010: txn%02d paddr==0x%08h (stride 4B)", i, exp_pa),
                    64'(apb_txn_paddr[i]), 64'(exp_pa));
        end
        idle(4);
    end

    // ================================================================
    // TB_BURST_011  Anti-starvation stress (6 writes concurrent with 1 read)
    //
    // Stimulus : 6x WR  awid=0xE000+i  addr=0xE000_0000+i*32  awlen=1  awsize=3
    //            1x RD  arid=0xF000     addr=0xF000_0000        arlen=1  arsize=3
    //            All issued concurrently; W beats driven sequentially.
    //            WR_STARVE_THRESH=4: after 4 writes accepted with RD pending,
    //            rd_priority latches and next idle slot is forced to RD.
    // APB expected: 12 WR txns + 4 RD txns = 16 total
    // AXI expected:
    //   WR i: bid=0xE000+i  bresp=2'b00(OKAY)  for i=0..5
    //   RD beat0: rid=0xF000  rresp=2'b00(OKAY)  rlast=0
    //   RD beat1: rid=0xF000  rresp=2'b00(OKAY)  rlast=1
    //   Observable proxy for anti-starvation: RD completes (does not timeout)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid   [6];
        logic [BRESP_WIDTH-1:0] bresp [6];
        logic [ID_WIDTH-1:0]    rid[];
        logic [DATA_WIDTH-1:0]  rdata[];
        logic [RRESP_WIDTH-1:0] rresp[];
        logic                   rlast[];
        time                    t_bresp[6], t_first_r, t_tmp;

        $display("\n====== TB_BURST_011: Anti-starvation stress ======");
        $display("[011] Stimulus: 6 WR bursts + 1 RD burst all concurrent");
        $display("[011] Expected: 16 APB txns (12 WR + 4 RD); all bresp=OKAY; RD completes");

        apb_monitor_reset();

        fork
            for (int i = 0; i < 6; i++) begin
                automatic int ii = i;
                fork
                    drive_aw(32'hE000 + ii, 32'hE000_0000 + ii*32, 8'h01, 3'b011);
                join_none
            end
        join_none

        fork drive_ar(32'hF000, 32'hF000_0000, 8'h01, 3'b011); join_none

        for (int i = 0; i < 6; i++) begin
            drive_w(64'hAAAA_0000_0000_0000 + i, 8'hFF, 1'b0);
            drive_w(64'hBBBB_0000_0000_0000 + i, 8'hFF, 1'b1);
        end

        for (int i = 0; i < 6; i++)
            collect_b(bid[i], bresp[i], t_bresp[i]);

        rid = new[2]; rdata = new[2]; rresp = new[2]; rlast = new[2];
        collect_r(rid[0], rdata[0], rresp[0], rlast[0], t_first_r);
        collect_r(rid[1], rdata[1], rresp[1], rlast[1], t_tmp);

        // --- AXI WR response checks ---
        for (int i = 0; i < 6; i++) begin
            chk_hex($sformatf("011: WR%0d bid==0x%08h", i, 32'hE000+i),
                    64'(bid[i]), 64'(32'hE000+i));
            chk1   ($sformatf("011: WR%0d bresp[1]==0 (not SLVERR)", i),
                    bresp[i][1], 1'b0);
            chk1   ($sformatf("011: WR%0d bresp[0]==0 (OKAY=2'b00)", i),
                    bresp[i][0], 1'b0);
        end
        // --- AXI RD response checks ---
        chk_hex("011: RD beat0 rid==0x0000_F000",             64'(rid[0]),   64'h0000_F000);
        chk1   ("011: RD beat0 rresp[1]==0 (not SLVERR)",    rresp[0][1],   1'b0);
        chk1   ("011: RD beat0 rresp[0]==0 (OKAY=2'b00)",    rresp[0][0],   1'b0);
        chk1   ("011: RD beat0 rlast==0 (not last)",         rlast[0],      1'b0);
        chk_hex("011: RD beat1 rid==0x0000_F000",             64'(rid[1]),   64'h0000_F000);
        chk1   ("011: RD beat1 rresp[1]==0 (not SLVERR)",    rresp[1][1],   1'b0);
        chk1   ("011: RD beat1 rresp[0]==0 (OKAY=2'b00)",    rresp[1][0],   1'b0);
        chk1   ("011: RD beat1 rlast==1 (last beat of burst)",rlast[1],      1'b1);
        // --- APB structural checks ---
        chk_int("011: apb_txn_count==16 (12 WR + 4 RD, anti-starvation)", apb_txn_count, 16);
        idle(4);
    end

    // ================================================================
    // TB_BURST_012  Write-write same cache line (serial wr_collision)
    //
    // Stimulus : WR1 awid=0x10  addr=0x1000_0000  awlen=1  awsize=3
    //            WR2 awid=0x11  addr=0x1000_0008  awlen=1  awsize=3
    //            Both in the same 64B CL (0x1000_0000..0x1000_003F)
    //            -> wr_collision: WR2 held in SLOT_IDLE until WR1 B-resp
    // APB expected: 4 WR1 txns complete; THEN 4 WR2 txns start (8 total)
    // AXI expected:
    //   WR1: bid=0x10  bresp=2'b00(OKAY)
    //   WR2: bid=0x11  bresp=2'b00(OKAY)
    //   Serialisation: t(WR2 B-resp) > t(WR1 B-resp)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid1, bid2;
        logic [BRESP_WIDTH-1:0] bresp1, bresp2;
        time                    t_b1, t_b2;

        $display("\n====== TB_BURST_012: Write-write same CL (serial wr_collision) ======");
        $display("[012] Stimulus: WR1 awid=0x10 addr=0x1000_0000; WR2 awid=0x11 addr=0x1000_0008");
        $display("[012] Same 64B CL -> wr_collision serializes WR2 after WR1");
        $display("[012] Expected: WR1 bid=0x10 bresp=OKAY; WR2 bid=0x11 bresp=OKAY");
        $display("[012]           t(WR2 B-resp) > t(WR1 B-resp)");

        apb_monitor_reset();

        fork drive_aw(32'h10, 32'h1000_0000, 8'h01, 3'b011); join_none
        idle(1);
        fork drive_aw(32'h11, 32'h1000_0008, 8'h01, 3'b011); join_none

        drive_w(64'h1111_1111_2222_2222, 8'hFF, 1'b0);
        drive_w(64'h3333_3333_4444_4444, 8'hFF, 1'b1);
        drive_w(64'h5555_5555_6666_6666, 8'hFF, 1'b0);
        drive_w(64'h7777_7777_8888_8888, 8'hFF, 1'b1);

        collect_b(bid1, bresp1, t_b1);
        collect_b(bid2, bresp2, t_b2);

        // --- AXI response checks ---
        chk_hex("012: WR1 bid==0x0000_0010",             64'(bid1),   64'h0000_0010);
        chk1   ("012: WR1 bresp[1]==0 (not SLVERR)",     bresp1[1],   1'b0);
        chk1   ("012: WR1 bresp[0]==0 (OKAY=2'b00)",     bresp1[0],   1'b0);
        chk_hex("012: WR2 bid==0x0000_0011",             64'(bid2),   64'h0000_0011);
        chk1   ("012: WR2 bresp[1]==0 (not SLVERR)",     bresp2[1],   1'b0);
        chk1   ("012: WR2 bresp[0]==0 (OKAY=2'b00)",     bresp2[0],   1'b0);
        chk_time_after("012: WR2 B-resp after WR1 B-resp (wr_collision serialized)",
                        t_b2, t_b1);
        // --- APB structural checks ---
        chk_int("012: apb_txn_count==8 (4 WR1 then 4 WR2)", apb_txn_count, 8);
        // First 4 txns must be WR1 addresses, last 4 must be WR2 addresses
        // Both in same 64B CL: mask with 0xFFFFFFC0 (64-byte boundary)
        for (int i = 0; i < 4; i++)
            chk_hex($sformatf("012: txn%0d in WR1 range (0x1000_0000..0x0000_000C)", i),
                    64'(apb_txn_paddr[i] & 32'hFFFFFFC0), 64'h1000_0000);
        for (int i = 4; i < 8; i++)
            chk_hex($sformatf("012: txn%0d in WR2 range (0x1000_0008..0x1000_0014)", i),
                    64'(apb_txn_paddr[i] & 32'hFFFFFFC0), 64'h1000_0000);
        idle(4);
    end

    // ================================================================
    // TB_BURST_013  Write-write different cache lines (concurrent)
    //
    // Stimulus : WR1 awid=0x20  addr=0x2000_0000  awlen=1  awsize=3
    //            WR2 awid=0x21  addr=0x2000_0040  awlen=1  awsize=3
    //            0x2000_0000 in CL N; 0x2000_0040 in CL N+1 (64B boundary)
    //            -> NO wr_collision; both slots occupied simultaneously
    // APB expected: 4 WR1 + 4 WR2 = 8 txns, interleaved on APB
    // AXI expected:
    //   WR1: bid=0x20  bresp=2'b00(OKAY)
    //   WR2: bid=0x21  bresp=2'b00(OKAY)
    // KEY check (concurrency): at least one WR2 APB txn appears in the
    //   monitor log before the last WR1 APB txn (interleave verification)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid1, bid2;
        logic [BRESP_WIDTH-1:0] bresp1, bresp2;
        time                    t_b1, t_b2;

        $display("\n====== TB_BURST_013: Write-write different CL (concurrent) ======");
        $display("[013] Stimulus: WR1 awid=0x20 addr=0x2000_0000; WR2 awid=0x21 addr=0x2000_0040");
        $display("[013] Different 64B CLs -> no wr_collision; concurrent slots");
        $display("[013] Expected: WR1 bid=0x20 bresp=OKAY; WR2 bid=0x21 bresp=OKAY");
        $display("[013]           APB txns from WR1 and WR2 interleaved");

        apb_monitor_reset();

        fork drive_aw(32'h20, 32'h2000_0000, 8'h01, 3'b011); join_none
        idle(1);
        fork drive_aw(32'h21, 32'h2000_0040, 8'h01, 3'b011); join_none

        drive_w(64'hABCD_EF01_2345_6789, 8'hFF, 1'b0);
        drive_w(64'hFEDC_BA98_7654_3210, 8'hFF, 1'b1);
        drive_w(64'h0F0F_0F0F_F0F0_F0F0, 8'hFF, 1'b0);
        drive_w(64'h1234_5678_8765_4321, 8'hFF, 1'b1);

        collect_b(bid1, bresp1, t_b1);
        collect_b(bid2, bresp2, t_b2);

        // --- AXI response checks ---
        chk_hex("013: WR1 bid==0x0000_0020",             64'(bid1),   64'h0000_0020);
        chk1   ("013: WR1 bresp[1]==0 (not SLVERR)",     bresp1[1],   1'b0);
        chk1   ("013: WR1 bresp[0]==0 (OKAY=2'b00)",     bresp1[0],   1'b0);
        chk_hex("013: WR2 bid==0x0000_0021",             64'(bid2),   64'h0000_0021);
        chk1   ("013: WR2 bresp[1]==0 (not SLVERR)",     bresp2[1],   1'b0);
        chk1   ("013: WR2 bresp[0]==0 (OKAY=2'b00)",     bresp2[0],   1'b0);
        // --- APB structural checks ---
        chk_int("013: apb_txn_count==8 (4 WR1 + 4 WR2)", apb_txn_count, 8);
        // Interleave check: scan monitor log for a WR2 txn appearing before
        // the last WR1 txn.  WR1 paddr in [0x2000_0000..0x2000_000C];
        //                   WR2 paddr in [0x2000_0040..0x2000_004C].
        begin
            int last_wr1_idx;
            int first_wr2_idx;
            last_wr1_idx  = -1;
            first_wr2_idx = -1;
            for (int i = 0; i < apb_txn_count && i < 64; i++) begin
                if (apb_txn_paddr[i] >= 32'h2000_0000 &&
                    apb_txn_paddr[i] <= 32'h2000_000C)
                    last_wr1_idx = i;
                if (apb_txn_paddr[i] >= 32'h2000_0040 && first_wr2_idx < 0)
                    first_wr2_idx = i;
            end
            if (first_wr2_idx >= 0 && last_wr1_idx >= 0 &&
                first_wr2_idx < last_wr1_idx) begin
                log_result($sformatf("013: APB txns interleaved (first WR2=txn#%0d before last WR1=txn#%0d)",
                                     first_wr2_idx, last_wr1_idx), 1'b1);
            end else begin
                log_result($sformatf("013: APB txns interleaved (first WR2=txn#%0d, last WR1=txn#%0d)",
                                     first_wr2_idx, last_wr1_idx), 1'b0);
            end
        end
        idle(4);
    end

    // ================================================================
    // TB_BURST_014  WaR same cache line - serialised by war_collision
    //
    // Stimulus : WR awid=0x30  addr=0x3000_0000  awlen=1  awsize=3
    //            RD arid=0x31  addr=0x3000_0000  arlen=1  arsize=3
    //            Same 64B CL -> war_collision holds RD in SLOT_IDLE
    //            until WR B-resp is issued, then RD proceeds.
    //            (BEHAVIOUR CHANGED from old TB which expected no stall)
    // APB expected: 4 WR txns (paddr 0x3000_0000..000C) all complete;
    //              THEN 4 RD txns (paddr 0x3000_0000..000C) start.
    //              txn0-3: pwrite==1; txn4-7: pwrite==0
    // AXI expected:
    //   WR: bid=0x30  bresp=2'b00(OKAY)
    //   RD beat0: rid=0x31  rresp=2'b00(OKAY)  rlast=0
    //   RD beat1: rid=0x31  rresp=2'b00(OKAY)  rlast=1
    //   KEY: t(first RD R-beat) > t(WR B-resp)
    // ================================================================
    begin
        logic [ID_WIDTH-1:0]    bid;
        logic [BRESP_WIDTH-1:0] bresp;
        logic [ID_WIDTH-1:0]    rid[];
        logic [DATA_WIDTH-1:0]  rdata[];
        logic [RRESP_WIDTH-1:0] rresp[];
        logic                   rlast[];
        time                    t_bresp, t_first_r, t_tmp;

        $display("\n====== TB_BURST_014: WaR same CL - war_collision serialized ======");
        $display("[014] Stimulus: WR awid=0x30 addr=0x3000_0000 awlen=1; RD arid=0x31 addr=0x3000_0000 arlen=1");
        $display("[014] Same 64B CL -> war_collision: RD held until WR B-resp");
        $display("[014] Expected: WR bid=0x30 bresp=OKAY; RD beat0 rid=0x31 rresp=OKAY rlast=0");
        $display("[014]           RD beat1 rid=0x31 rresp=OKAY rlast=1");
        $display("[014]           t(first RD R-beat) > t(WR B-resp)  [serialization]");
        $display("[014]           APB txn0-3 are WR, txn4-7 are RD (no interleave)");

        apb_monitor_reset();

        fork drive_aw(32'h30, 32'h3000_0000, 8'h01, 3'b011); join_none
        // RD AR issued concurrently; war_collision should hold it
        fork drive_ar(32'h31, 32'h3000_0000, 8'h01, 3'b011); join_none

        drive_w(64'hCAFE_BABE_1234_5678, 8'hFF, 1'b0);
        drive_w(64'hDEAD_BEEF_8765_4321, 8'hFF, 1'b1);

        collect_b(bid, bresp, t_bresp);

        rid = new[2]; rdata = new[2]; rresp = new[2]; rlast = new[2];
        collect_r(rid[0], rdata[0], rresp[0], rlast[0], t_first_r);
        collect_r(rid[1], rdata[1], rresp[1], rlast[1], t_tmp);

        // --- AXI WR response checks ---
        chk_hex("014: WR bid==0x0000_0030",                  64'(bid),    64'h0000_0030);
        chk1   ("014: WR bresp[1]==0 (not SLVERR)",          bresp[1],    1'b0);
        chk1   ("014: WR bresp[0]==0 (OKAY=2'b00)",          bresp[0],    1'b0);
        // --- AXI RD response checks ---
        chk_hex("014: RD beat0 rid==0x0000_0031",             64'(rid[0]), 64'h0000_0031);
        chk1   ("014: RD beat0 rresp[1]==0 (not SLVERR)",    rresp[0][1], 1'b0);
        chk1   ("014: RD beat0 rresp[0]==0 (OKAY=2'b00)",    rresp[0][0], 1'b0);
        chk1   ("014: RD beat0 rlast==0 (not last)",         rlast[0],    1'b0);
        chk_hex("014: RD beat1 rid==0x0000_0031",             64'(rid[1]), 64'h0000_0031);
        chk1   ("014: RD beat1 rresp[1]==0 (not SLVERR)",    rresp[1][1], 1'b0);
        chk1   ("014: RD beat1 rresp[0]==0 (OKAY=2'b00)",    rresp[1][0], 1'b0);
        chk1   ("014: RD beat1 rlast==1 (last beat of burst)",rlast[1],    1'b1);
        // --- APB structural checks (strict ordering: WR first, then RD) ---
        chk_int("014: apb_txn_count==8 (4 WR then 4 RD)", apb_txn_count, 8);
        for (int i = 0; i < 4; i++)
            chk1($sformatf("014: txn%0d pwrite==1 (WR completes before RD)", i),
                 apb_txn_pwrite[i], 1'b1);
        for (int i = 4; i < 8; i++)
            chk1($sformatf("014: txn%0d pwrite==0 (RD starts only after WR done)", i),
                 apb_txn_pwrite[i], 1'b0);
        // --- KEY serialisation timing check ---
        chk_time_after("014: first RD R-beat after WR B-resp (war_collision serialized)",
                        t_first_r, t_bresp);
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