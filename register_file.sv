/*------------------------------------------------------------------------------
 * File          : register_file.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Mar 3, 2026
 * Description   :
 *   Two-slot register file for AXI response storage.
 *
 *   Each direction (write response, read data) has two physical storage slots
 *   and two independent pointers:
 *
 *     write_ptr : points to the next slot to WRITE into.
 *                 Advances (toggles) when enable_wr fires.
 *     read_ptr  : points to the next slot to READ/OUTPUT.
 *                 Advances (toggles) when done_wr fires.
 *
 *   This is the standard 2-entry FIFO pointer scheme.  Keeping them separate
 *   is the key correctness fix over the original design, which used a single
 *   pointer for both and cleared the wrong slot.
 *
 *   FIXES vs. ORIGINAL
 *   ===================
 *   1. Separate write_ptr / read_ptr per direction (was one shared pointer).
 *      Output mux and valid clear now use read_ptr; write uses write_ptr.
 *      Previously done_wr cleared the write slot instead of the read slot.
 *
 *   2. Simultaneous enable + done priority:
 *      enable_wr takes priority over done_wr for the same physical slot.
 *      Previously the clear (done) silently won, dropping the new response.
 *
 *   3. Guard against enable_wr into a slot that is still valid:
 *      Manager must check rf_wr_valid before asserting enable_wr.
 *      An assertion fires in simulation if this contract is violated.
 *
 *   4. Data storage uses DW03_reg_s_pl (DesignWare registered FF with enable).
 *      reset_value=0 so all slots initialise to 0 on reset_N assertion.
 *      Two instances per direction (one per physical slot).
 *
 *   5. rf_wr_id_out / rf_rd_id_out retained (unused by manager today but
 *      will be needed for out-of-order response handling in future).
 *
 *   PROTOCOL CONTRACT (manager must obey)
 *   ======================================
 *     enable_wr must not be asserted when rf_wr_valid is already high
 *     (both slots full).  Same for enable_rd / rf_rd_valid.
 *     The manager currently satisfies this because it only has 2 transaction
 *     slots and each slot generates at most one response per direction.
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module register_file
import struct_types::*;
(
    input  logic   clk,
    input  logic   rst_n,

    /*-------- Write response direction --------*/
    input  logic                        enable_wr,          // write resp_in into next slot
    input  logic                        done_wr,            // response consumed; advance read ptr
    input  logic [ID_WIDTH-1:0]         rf_txn_id_wr,       // ID tag written alongside data
    input  struct_types::axi_wr_resp    rf_axi_wr_resp_in,
    output struct_types::axi_wr_resp    rf_axi_wr_resp_out, // current read-slot output
    output logic [ID_WIDTH-1:0]         rf_wr_id_out,       // ID of the current read slot
    output logic                        rf_wr_valid,        // read slot holds unread data

    /*-------- Read data direction --------*/
    input  logic                        enable_rd,
    input  logic                        done_rd,
    input  logic [ID_WIDTH-1:0]         rf_txn_id_rd,
    input  struct_types::axi_rd_data    rf_axi_rd_data_in,
    output struct_types::axi_rd_data    rf_axi_rd_data_out,
    output logic [ID_WIDTH-1:0]         rf_rd_id_out,
    output logic                        rf_rd_valid
);

/*=========================================================================*/
/*  Write response slots                                                    */
/*=========================================================================*/

    // write_ptr: which physical slot receives the next enable_wr
    // read_ptr:  which physical slot is currently presented on the outputs
    // Both toggle independently.
    logic wr_write_ptr, wr_read_ptr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_write_ptr <= 1'b0;
            wr_read_ptr  <= 1'b0;
        end else begin
            if (enable_wr) wr_write_ptr <= ~wr_write_ptr;
            if (done_wr)   wr_read_ptr  <= ~wr_read_ptr;
        end
    end

    // Per-slot valid bits and ID tags
    logic [ID_WIDTH-1:0] wr_id        [0:1];
    logic                wr_slot_valid[0:1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_id[0] <= '0; wr_slot_valid[0] <= 1'b0;
            wr_id[1] <= '0; wr_slot_valid[1] <= 1'b0;
        end else begin
            // Slot 0
            if (enable_wr && !wr_write_ptr) begin
                // Write into slot 0: set valid and tag
                // enable_wr takes priority over done_wr for the same slot
                wr_id[0]         <= rf_txn_id_wr;
                wr_slot_valid[0] <= 1'b1;
            end else if (done_wr && !wr_read_ptr) begin
                // Consume from slot 0: clear valid
                wr_slot_valid[0] <= 1'b0;
            end

            // Slot 1
            if (enable_wr && wr_write_ptr) begin
                wr_id[1]         <= rf_txn_id_wr;
                wr_slot_valid[1] <= 1'b1;
            end else if (done_wr && wr_read_ptr) begin
                wr_slot_valid[1] <= 1'b0;
            end
        end
    end

    // Data storage via DW03_reg_s_pl (synchronous FF with active-low reset and enable)
    struct_types::axi_wr_resp wr_resp_q [0:1];

    DW03_reg_s_pl #(
        .width       ($bits(axi_wr_resp)),
        .reset_value (0)
    ) u_reg_wr_resp0 (
        .clk     (clk),
        .reset_N (rst_n),
        .enable  (enable_wr & ~wr_write_ptr),
        .d       (rf_axi_wr_resp_in),
        .q       (wr_resp_q[0])
    );

    DW03_reg_s_pl #(
        .width       ($bits(axi_wr_resp)),
        .reset_value (0)
    ) u_reg_wr_resp1 (
        .clk     (clk),
        .reset_N (rst_n),
        .enable  (enable_wr & wr_write_ptr),
        .d       (rf_axi_wr_resp_in),
        .q       (wr_resp_q[1])
    );

    // Outputs: always driven from the read pointer slot
    assign rf_axi_wr_resp_out = wr_resp_q    [wr_read_ptr];
    assign rf_wr_id_out       = wr_id        [wr_read_ptr];
    assign rf_wr_valid        = wr_slot_valid[wr_read_ptr];

    // Simulation assertion: manager must not write into a full register file
    // synthesis translate_off
    always_ff @(posedge clk) begin
        if (enable_wr && wr_slot_valid[wr_write_ptr])
            $display("[%0t] REGISTER_FILE WARNING: enable_wr into already-valid wr slot %0d",
                     $time, wr_write_ptr);
    end
    // synthesis translate_on

/*=========================================================================*/
/*  Read data slots                                                         */
/*=========================================================================*/

    logic rd_write_ptr, rd_read_ptr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_write_ptr <= 1'b0;
            rd_read_ptr  <= 1'b0;
        end else begin
            if (enable_rd) rd_write_ptr <= ~rd_write_ptr;
            if (done_rd)   rd_read_ptr  <= ~rd_read_ptr;
        end
    end

    logic [ID_WIDTH-1:0] rd_id        [0:1];
    logic                rd_slot_valid[0:1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_id[0] <= '0; rd_slot_valid[0] <= 1'b0;
            rd_id[1] <= '0; rd_slot_valid[1] <= 1'b0;
        end else begin
            // Slot 0
            if (enable_rd && !rd_write_ptr) begin
                rd_id[0]         <= rf_txn_id_rd;
                rd_slot_valid[0] <= 1'b1;
            end else if (done_rd && !rd_read_ptr) begin
                rd_slot_valid[0] <= 1'b0;
            end

            // Slot 1
            if (enable_rd && rd_write_ptr) begin
                rd_id[1]         <= rf_txn_id_rd;
                rd_slot_valid[1] <= 1'b1;
            end else if (done_rd && rd_read_ptr) begin
                rd_slot_valid[1] <= 1'b0;
            end
        end
    end

    struct_types::axi_rd_data rd_data_q [0:1];

    DW03_reg_s_pl #(
        .width       ($bits(axi_rd_data)),
        .reset_value (0)
    ) u_reg_rd_data0 (
        .clk     (clk),
        .reset_N (rst_n),
        .enable  (enable_rd & ~rd_write_ptr),
        .d       (rf_axi_rd_data_in),
        .q       (rd_data_q[0])
    );

    DW03_reg_s_pl #(
        .width       ($bits(axi_rd_data)),
        .reset_value (0)
    ) u_reg_rd_data1 (
        .clk     (clk),
        .reset_N (rst_n),
        .enable  (enable_rd & rd_write_ptr),
        .d       (rf_axi_rd_data_in),
        .q       (rd_data_q[1])
    );

    assign rf_axi_rd_data_out = rd_data_q    [rd_read_ptr];
    assign rf_rd_id_out       = rd_id        [rd_read_ptr];
    assign rf_rd_valid        = rd_slot_valid[rd_read_ptr];

    // synthesis translate_off
    always_ff @(posedge clk) begin
        if (enable_rd && rd_slot_valid[rd_write_ptr])
            $display("[%0t] REGISTER_FILE WARNING: enable_rd into already-valid rd slot %0d",
                     $time, rd_write_ptr);
    end
    // synthesis translate_on

endmodule