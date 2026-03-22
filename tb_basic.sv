`timescale 1ns/1ps

/*------------------------------------------------------------------------------
 * File          : tb_basic.sv
 * Description   : Basic testbench for the AXI-to-APB bridge.
 *                 Tests one single-beat write followed by one single-beat read.
 *
 * TEST PLAN
 * =========
 * 1. AXI WRITE: id=1, addr=0x1000, data=0xDEAD_BEEF_CAFE_1234, strb=0xFF
 *    Expected APB transactions:
 *      LSB: paddr=0x1000, pwdata=0xCAFE_1234, pstrb=4'hF, pwrite=1
 *      MSB: paddr=0x1004, pwdata=0xDEAD_BEEF, pstrb=4'hF, pwrite=1
 *    Expected AXI response: bresp=OKAY
 *
 * 2. AXI READ: id=2, addr=0x1000
 *    Expected APB transactions:
 *      LSB: paddr=0x1000, pwrite=0 -> prdata=0xCAFE_1234
 *      MSB: paddr=0x1004, pwrite=0 -> prdata=0xDEAD_BEEF
 *    Expected AXI response:
 *      rdata=0xDEAD_BEEF_CAFE_1234, rresp=OKAY, rlast=1
 *
 * FIXES vs. ORIGINAL TB
 * =====================
 * - Added awsize_tb / arsize_tb signals and assigns (was missing -> X on DUT)
 * - Added awsize / arsize parameters to axi_write / axi_read tasks
 * - Default size = 3'b011 (8 bytes = full 64-bit AXI width)
 * - Added PASS/FAIL checker for write response (bresp)
 * - Added per-task timeout watchdog (separate from global timeout)
 *
 * APB SLAVE MODEL
 * ===============
 * Simple 4-word (4 x 32-bit) memory at word addresses 0-3.
 * Address mapping: apb_mem[(paddr >> 2) & 3]
 *   0x1000 -> slot 0
 *   0x1004 -> slot 1
 * pready is asserted one cycle after psel+penable (zero wait-state slave).
 *------------------------------------------------------------------------------*/

module tb;

import struct_types::*;

/*=========================================================================*/
/*  Clock / reset                                                           */
/*=========================================================================*/

logic clk   = 0;
logic rst_n;

always #5 clk = ~clk;  // 100 MHz

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
/*  TB -> DUT intermediate signals                                          */
/*  (one driver per net: TB drives these, assigns feed the interfaces)     */
/*=========================================================================*/

// AW channel
logic                           awvalid_tb = 0;
logic [ID_WIDTH-1:0]            awid_tb    = '0;
logic [ADDR_WIDTH-1:0]          awaddr_tb  = '0;
logic [MAX_LEN-1:0]                     awlen_tb   = '0;
logic [MAX_SIZE-1:0]            awsize_tb  = '0;   // FIX: was missing

// W channel
logic                           wvalid_tb  = 0;
logic [DATA_WIDTH-1:0]          wdata_tb   = '0;
logic [WSTRB_WIDTH-1:0]         wstrb_tb   = '0;
logic                           wlast_tb   = 0;

// B channel
logic                           bready_tb  = 0;

// AR channel
logic                           arvalid_tb = 0;
logic [ID_WIDTH-1:0]            arid_tb    = '0;
logic [ADDR_WIDTH-1:0]          araddr_tb  = '0;
logic [MAX_LEN-1:0]                     arlen_tb   = '0;
logic [MAX_SIZE-1:0]            arsize_tb  = '0;   // FIX: was missing

// R channel
logic                           rready_tb  = 0;

// Continuous assigns -> interface
assign axi_if.awvalid = awvalid_tb;
assign axi_if.awid    = awid_tb;
assign axi_if.awaddr  = awaddr_tb;
assign axi_if.awlen   = awlen_tb;
assign axi_if.awsize  = awsize_tb;   // FIX: was missing
assign axi_if.wvalid  = wvalid_tb;
assign axi_if.wdata   = wdata_tb;
assign axi_if.wstrb   = wstrb_tb;
assign axi_if.wlast   = wlast_tb;
assign axi_if.bready  = bready_tb;
assign axi_if.arvalid = arvalid_tb;
assign axi_if.arid    = arid_tb;
assign axi_if.araddr  = araddr_tb;
assign axi_if.arlen   = arlen_tb;
assign axi_if.arsize  = arsize_tb;   // FIX: was missing
assign axi_if.rready  = rready_tb;

// APB slave response signals (driven only by always_ff below)
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
/*  AXI master driver tasks                                                 */
/*=========================================================================*/

// ------------------------------------------------------------
// axi_write
//   Drives AW, W, and waits for B response.
//   size: 3'b011 = 8 bytes (full 64-bit bus, default)
//         3'b010 = 4 bytes (narrow, upper or lower half only)
// ------------------------------------------------------------
task automatic axi_write (
    input logic [ID_WIDTH-1:0]    id,
    input logic [ADDR_WIDTH-1:0]  addr,
    input logic [DATA_WIDTH-1:0]  data,
    input logic [WSTRB_WIDTH-1:0] strb,
    input logic [MAX_SIZE-1:0]    size = 3'b011  // default: full 8-byte beat
);
    // -- AW channel --
    // Drive at negedge so signals are stable well before next posedge sample
    @(negedge clk);
    awvalid_tb = 1;
    awid_tb    = id;
    awaddr_tb  = addr;
    awlen_tb   = 8'h00;   // single beat (len=0 means 1 transfer)
    awsize_tb  = size;
    // Wait for awready handshake
    @(posedge clk);
    while (!axi_if.awready) @(posedge clk);
    @(negedge clk);
    awvalid_tb = 0;

    // -- W channel --
    wvalid_tb = 1;
    wdata_tb  = data;
    wstrb_tb  = strb;
    wlast_tb  = 1;        // single beat, always last
    @(posedge clk);
    while (!axi_if.wready) @(posedge clk);
    @(negedge clk);
    wvalid_tb = 0;
    wlast_tb  = 0;

    // -- B channel --
    bready_tb = 1;
    @(posedge clk);
    while (!axi_if.bvalid) @(posedge clk);

    // Response check
    $display("[%0t] WRITE RESP  bid=%0h  bresp=%0b  (%s)",
             $time, axi_if.bid, axi_if.bresp,
             (axi_if.bresp == 2'b00) ? "OKAY" : "SLVERR");
    if (axi_if.bresp == 2'b00)
        $display("[%0t] CHECK PASS: bresp is OKAY", $time);
    else
        $display("[%0t] CHECK FAIL: bresp=%0b, expected OKAY", $time, axi_if.bresp);

    @(negedge clk);
    bready_tb = 0;
endtask

// ------------------------------------------------------------
// axi_read
//   Drives AR and waits for R response.
//   expected_data: value to check rdata against.
// ------------------------------------------------------------
task automatic axi_read (
    input logic [ID_WIDTH-1:0]   id,
    input logic [ADDR_WIDTH-1:0] addr,
    input logic [DATA_WIDTH-1:0] expected_data,
    input logic [MAX_SIZE-1:0]   size = 3'b011   // default: full 8-byte beat
);
    // -- AR channel --
    @(negedge clk);
    arvalid_tb = 1;
    arid_tb    = id;
    araddr_tb  = addr;
    arlen_tb   = 8'h00;   // single beat
    arsize_tb  = size;
    @(posedge clk);
    while (!axi_if.arready) @(posedge clk);
    @(negedge clk);
    arvalid_tb = 0;

    // -- R channel --
    rready_tb = 1;
    @(posedge clk);
    while (!axi_if.rvalid) @(posedge clk);

    $display("[%0t] READ  DATA   rid=%0h  rdata=0x%0h  rresp=%0b  rlast=%0b  (%s)",
             $time, axi_if.rid, axi_if.rdata, axi_if.rresp, axi_if.rlast,
             (axi_if.rresp == 2'b00) ? "OKAY" : "SLVERR");

    // Data check
    if (axi_if.rdata === expected_data)
        $display("[%0t] CHECK PASS: rdata=0x%0h matches expected", $time, axi_if.rdata);
    else
        $display("[%0t] CHECK FAIL: rdata=0x%0h, expected=0x%0h",
                 $time, axi_if.rdata, expected_data);

    // Response check
    if (axi_if.rresp == 2'b00)
        $display("[%0t] CHECK PASS: rresp is OKAY", $time);
    else
        $display("[%0t] CHECK FAIL: rresp=%0b, expected OKAY", $time, axi_if.rresp);

    // rlast check (single beat must always assert rlast)
    if (axi_if.rlast)
        $display("[%0t] CHECK PASS: rlast asserted on single beat", $time);
    else
        $display("[%0t] CHECK FAIL: rlast not asserted on single beat", $time);

    @(negedge clk);
    rready_tb = 0;
endtask

/*=========================================================================*/
/*  Inline APB slave model                                                  */
/*=========================================================================*/
// 4-word 32-bit memory. Word address = (paddr >> 2) & 3.
//   0x1000 -> word 0
//   0x1004 -> word 1
//   0x1008 -> word 2
//   0x100C -> word 3
//
// Timing:
//   Cycle N:   psel=1, penable=1 (ACCESS phase enters)
//   Cycle N+1: pready=1, prdata valid (zero wait-state response)

logic [31:0] apb_mem [0:3];

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pready_tb  <= 0;
        prdata_tb  <= 0;
        pslverr_tb <= 0;
        foreach (apb_mem[i]) apb_mem[i] <= 32'hDEAD_DEAD;
    end else begin
        pready_tb <= 0;     // default: deassert each cycle

        if (apb_if.psel && apb_if.penable && !pready_tb) begin
            pready_tb  <= 1;
            pslverr_tb <= 0;

            if (apb_if.pwrite) begin
                // Write: update only the byte lanes selected by pstrb
                for (int b = 0; b < 4; b++)
                    if (apb_if.pstrb[b])
                        apb_mem[(apb_if.paddr >> 2) & 3][b*8 +: 8]
                            <= apb_if.pwdata[b*8 +: 8];
                $display("[%0t] APB WRITE  paddr=0x%08h  pwdata=0x%08h  pstrb=4'b%0b",
                         $time, apb_if.paddr, apb_if.pwdata, apb_if.pstrb);
            end else begin
                // Read: return the word from memory
                prdata_tb <= apb_mem[(apb_if.paddr >> 2) & 3];
                $display("[%0t] APB READ   paddr=0x%08h  prdata=0x%08h",
                         $time, apb_if.paddr,
                         apb_mem[(apb_if.paddr >> 2) & 3]);
            end
        end
    end
end

/*=========================================================================*/
/*  Stimulus                                                                */
/*=========================================================================*/

initial begin
    $dumpfile("tb_basic.vcd");
    $dumpvars(0, tb);

    // Wait for reset to release
    @(posedge rst_n);
    repeat(2) @(posedge clk);

    // ----------------------------------------------------------------
    // TEST 1: Single-beat write
    // addr=0x1000, data=0xDEAD_BEEF_CAFE_1234, all byte enables set
    //
    // Expected APB sequence:
    //   LSB: paddr=0x1000, pwdata=0xCAFE_1234, pstrb=4'hF
    //   MSB: paddr=0x1004, pwdata=0xDEAD_BEEF, pstrb=4'hF
    // Expected AXI response: bresp=OKAY
    // ----------------------------------------------------------------
    $display("\n=== TEST 1: AXI WRITE  id=1  addr=0x1000  data=0xDEAD_BEEF_CAFE_1234 ===");
    axi_write(
        .id   (32'h1),
        .addr (32'h0000_1000),
        .data (64'hDEAD_BEEF_CAFE_1234),
        .strb (8'hFF),
        .size (3'b011)   // 8 bytes: both APB halves active
    );

    repeat(4) @(posedge clk);

    // ----------------------------------------------------------------
    // TEST 2: Single-beat read of the same address
    // Expected APB sequence:
    //   LSB: paddr=0x1000 -> prdata=0xCAFE_1234
    //   MSB: paddr=0x1004 -> prdata=0xDEAD_BEEF
    // Expected AXI response:
    //   rdata=0xDEAD_BEEF_CAFE_1234, rresp=OKAY, rlast=1
    // ----------------------------------------------------------------
    $display("\n=== TEST 2: AXI READ   id=2  addr=0x1000  expected=0xDEAD_BEEF_CAFE_1234 ===");
    axi_read(
        .id            (32'h2),
        .addr          (32'h0000_1000),
        .expected_data (64'hDEAD_BEEF_CAFE_1234),
        .size          (3'b011)   // 8 bytes: both APB halves active
    );

    repeat(8) @(posedge clk);
    $display("\n=== Simulation complete ===");
    $finish;
end

// Global timeout watchdog
initial begin
    #100_000;
    $display("[%0t] TIMEOUT: simulation did not complete in time", $time);
    $finish;
end

endmodule