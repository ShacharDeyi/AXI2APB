/*------------------------------------------------------------------------------
 * File          : register_file.sv
 * Project       : RTL
 * Description   : 4-slot transaction register file for the AXI-to-APB bridge.
 *
 * OVERVIEW
 * ========
 * Provides 2 write slots and 2 read slots.  Each slot owns one AXI transaction
 * from the moment its request header is stored (allocation) to the moment the
 * assembled AXI response is drained back to the slave (done).
 *
 * SLOT LIFECYCLE
 * ==============
 *   1. ALLOCATION   manager asserts alloc_wr / alloc_rd, supplies the AXI
 *                   request header and the slot index (alloc_slot_wr/rd).
 *                   The slot transitions from FREE → ACTIVE.
 *                   The request header is stored in a DW03_reg_s_pl instance
 *                   so the manager can read it back every cycle to compute
 *                   beat addresses.
 *
 *   2. RESPONSE     As APB sub-responses arrive the manager writes an
 * ACCUMULATION      assembled axi_wr_resp / axi_rd_data into the slot via
 *                   resp_wr_wr / resp_wr_rd.  For reads every beat produces
 *                   a new rd_data beat; for writes only the final beat
 *                   triggers a write (accumulated bresp).
 *
 *   3. COMPLETION   manager asserts resp_valid_set_wr / _rd (with the slot
 *                   index) to flag the slot as response-ready.  The slot
 *                   transitions ACTIVE → READY.
 *
 *   4. DRAIN        The AXI slave side reads wr_resp_out / rd_data_out
 *                   (always the read-pointer slot) and asserts done_wr /
 *                   done_rd.  The slot transitions READY → FREE and the
 *                   read pointer advances.
 *
 * POINTER SCHEME
 * ==============
 * Independent write (alloc) and read (drain) pointers per direction,
 * toggling between 0 and 1.  This is the standard 2-entry FIFO scheme.
 * The alloc pointer lives in the MANAGER (it knows when to allocate);
 * this module receives the explicit slot index on every alloc/resp write.
 * The read pointer lives here and advances on done_wr / done_rd.
 *
 * STORAGE
 * =======
 * Request headers and response data are stored in DW03_reg_s_pl instances
 * (synchronous FF with active-low reset and enable).  Per-slot control
 * bits (valid, resp_ready) are plain always_ff registers.
 *
 * PROTOCOL CONTRACT (manager must obey)
 * ======================================
 *   - alloc_wr/rd must not be asserted when the target slot is not free
 *     (slot_free_wr/rd[slot] == 0).
 *   - resp_valid_set_wr/rd must only be asserted for an already-active slot.
 *   - done_wr/rd must only be asserted when resp_ready_wr/rd is high.
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

    // Allocation: manager pops wr_req_fifo and immediately stores the header.
    input  logic                        alloc_wr,           // 1 = allocate this cycle
    input  logic                        alloc_slot_wr,      // which slot (0 or 1)
    input  struct_types::axi_wr_req     alloc_req_wr,       // header to store

    // Header readback: manager reads these every cycle for beat-addr computation.
    output struct_types::axi_wr_req     req_out_wr  [0:1],

    // Response write: manager writes assembled bresp as beats complete.
    input  logic                        resp_wr_en,         // 1 = write this cycle
    input  logic                        resp_slot_wr,       // which slot
    input  struct_types::axi_wr_resp    resp_in_wr,         // assembled response

    // Completion: manager signals the slot's response is final.
    input  logic                        resp_valid_set_wr,  // 1 = mark slot done
    input  logic                        resp_valid_slot_wr, // which slot

    // Drain: AXI slave has consumed the response; advance read pointer.
    input  logic                        done_wr,

    // Outputs to AXI slave (always from read-pointer slot).
    output struct_types::axi_wr_resp    resp_out_wr,
    output logic [ID_WIDTH-1:0]         resp_id_out_wr,     // bid for the response
    output logic                        resp_ready_wr,      // read slot is response-ready

    // Free flags: manager checks before allocating.
    output logic                        slot_free_wr [0:1],

    /*--------------------------------------------------------------------*/
    /* Read direction — 2 slots (symmetric)                               */
    /*--------------------------------------------------------------------*/

    input  logic                        alloc_rd,
    input  logic                        alloc_slot_rd,
    input  struct_types::axi_rd_req     alloc_req_rd,

    output struct_types::axi_rd_req     req_out_rd  [0:1],

    input  logic                        resp_rd_en,
    input  logic                        resp_slot_rd,
    input  struct_types::axi_rd_data    resp_in_rd,

    input  logic                        resp_valid_set_rd,
    input  logic                        resp_valid_slot_rd,

    input  logic                        done_rd,

    output struct_types::axi_rd_data    resp_out_rd,
    output logic [ID_WIDTH-1:0]         resp_id_out_rd,
    output logic                        resp_ready_rd,

    output logic                        slot_free_rd [0:1]
);

/*=========================================================================*/
/*  Write direction                                                         */
/*=========================================================================*/

    // Read pointer (drain side).  Alloc pointer lives in the manager.
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
            wr_slot_valid[0]   <= 1'b0; wr_resp_ready_q[0] <= 1'b0;
            wr_slot_valid[1]   <= 1'b0; wr_resp_ready_q[1] <= 1'b0;
        end else begin
            // Slot 0
            if (alloc_wr && !alloc_slot_wr)
                wr_slot_valid[0] <= 1'b1;
            else if (done_wr && !wr_read_ptr)
                wr_slot_valid[0] <= 1'b0;

            if (resp_valid_set_wr && !resp_valid_slot_wr)
                wr_resp_ready_q[0] <= 1'b1;
            else if (done_wr && !wr_read_ptr)
                wr_resp_ready_q[0] <= 1'b0;

            // Slot 1
            if (alloc_wr && alloc_slot_wr)
                wr_slot_valid[1] <= 1'b1;
            else if (done_wr && wr_read_ptr)
                wr_slot_valid[1] <= 1'b0;

            if (resp_valid_set_wr && resp_valid_slot_wr)
                wr_resp_ready_q[1] <= 1'b1;
            else if (done_wr && wr_read_ptr)
                wr_resp_ready_q[1] <= 1'b0;
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

    // Outputs — always driven from the read-pointer slot.
    assign req_out_wr    = wr_req_q;                       // both slots visible
    assign resp_out_wr   = wr_resp_q    [wr_read_ptr];
    assign resp_id_out_wr= wr_req_q     [wr_read_ptr].awid;
    assign resp_ready_wr = wr_resp_ready_q [wr_read_ptr];

    assign slot_free_wr[0] = !wr_slot_valid[0];
    assign slot_free_wr[1] = !wr_slot_valid[1];

    // Simulation assertions.
    // synthesis translate_off
    always_ff @(posedge clk) begin
        if (alloc_wr && wr_slot_valid[alloc_slot_wr])
            $display("[%0t] RF WARNING: alloc_wr into already-valid wr slot %0b",
                     $time, alloc_slot_wr);
        if (done_wr && !wr_resp_ready_q[wr_read_ptr])
            $display("[%0t] RF WARNING: done_wr on slot %0b which has no ready response",
                     $time, wr_read_ptr);
    end
    // synthesis translate_on

/*=========================================================================*/
/*  Read direction (symmetric)                                              */
/*=========================================================================*/

    logic rd_read_ptr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) rd_read_ptr <= 1'b0;
        else if (done_rd) rd_read_ptr <= ~rd_read_ptr;
    end

    logic rd_slot_valid    [0:1];
    logic rd_resp_ready_q  [0:1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_slot_valid[0]   <= 1'b0; rd_resp_ready_q[0] <= 1'b0;
            rd_slot_valid[1]   <= 1'b0; rd_resp_ready_q[1] <= 1'b0;
        end else begin
            // Slot 0
            if (alloc_rd && !alloc_slot_rd)
                rd_slot_valid[0] <= 1'b1;
            else if (done_rd && !rd_read_ptr)
                rd_slot_valid[0] <= 1'b0;

            if (resp_valid_set_rd && !resp_valid_slot_rd)
                rd_resp_ready_q[0] <= 1'b1;
            else if (done_rd && !rd_read_ptr)
                rd_resp_ready_q[0] <= 1'b0;

            // Slot 1
            if (alloc_rd && alloc_slot_rd)
                rd_slot_valid[1] <= 1'b1;
            else if (done_rd && rd_read_ptr)
                rd_slot_valid[1] <= 1'b0;

            if (resp_valid_set_rd && resp_valid_slot_rd)
                rd_resp_ready_q[1] <= 1'b1;
            else if (done_rd && rd_read_ptr)
                rd_resp_ready_q[1] <= 1'b0;
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

    struct_types::axi_rd_data rd_resp_q [0:1];

    DW03_reg_s_pl #(
        .width       ($bits(axi_rd_data)),
        .reset_value (0)
    ) u_rd_resp0 (
        .clk     (clk),
        .reset_N (rst_n),
        .enable  (resp_rd_en & ~resp_slot_rd),
        .d       (resp_in_rd),
        .q       (rd_resp_q[0])
    );

    DW03_reg_s_pl #(
        .width       ($bits(axi_rd_data)),
        .reset_value (0)
    ) u_rd_resp1 (
        .clk     (clk),
        .reset_N (rst_n),
        .enable  (resp_rd_en & resp_slot_rd),
        .d       (resp_in_rd),
        .q       (rd_resp_q[1])
    );

    assign req_out_rd     = rd_req_q;
    assign resp_out_rd    = rd_resp_q    [rd_read_ptr];
    assign resp_id_out_rd = rd_req_q     [rd_read_ptr].arid;
    assign resp_ready_rd  = rd_resp_ready_q [rd_read_ptr];

    assign slot_free_rd[0] = !rd_slot_valid[0];
    assign slot_free_rd[1] = !rd_slot_valid[1];

    // synthesis translate_off
    always_ff @(posedge clk) begin
        if (alloc_rd && rd_slot_valid[alloc_slot_rd])
            $display("[%0t] RF WARNING: alloc_rd into already-valid rd slot %0b",
                     $time, alloc_slot_rd);
        // NOTE: done_rd does NOT require resp_ready_rd to be set.
        // Read responses bypass the register file entirely (pushed directly to
        // rd_data_fifo); resp_valid_set_rd is never asserted, so rd_resp_ready_q
        // is always 0 for reads.  The correct precondition for done_rd is that
        // the slot is actually allocated (valid), not that it has a ready response.
        if (done_rd && !rd_slot_valid[rd_read_ptr])
            $display("[%0t] RF WARNING: done_rd on slot %0b which is not allocated",
                     $time, rd_read_ptr);
    end
    // synthesis translate_on

endmodule