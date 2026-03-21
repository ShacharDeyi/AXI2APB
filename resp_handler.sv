/*------------------------------------------------------------------------------
 * File          : resp_handler.sv
 * Project       : RTL
 * Author        : epsdso
 * Description   : Buffers assembled AXI responses from manager slots and
 *                 pushes them to the AXI write-response and read-data FIFOs.
 *
 * RESPONSIBILITY
 * ==============
 * The manager collects APB responses and assembles them back into AXI
 * responses per slot. This module takes those assembled responses,
 * buffers them in a 2-slot register_file, and pushes them downstream
 * to the AXI response FIFOs when space is available.
 *
 * The manager does NOT need to know about register_file or FIFO push
 * timing -- it simply asserts slot_resp[s].valid for one cycle when a
 * response is ready and waits for slot_consumed[s] before proceeding.
 *
 * INTERFACE TO MANAGER
 * ====================
 * Manager drives slot_resp[s] with valid=1 for exactly one cycle when
 * slot s has a completed response. This module returns slot_consumed[s]=1
 * the same cycle to acknowledge. The manager can then move the slot back
 * to SLOT_IDLE.
 *
 * PIPELINE
 * ========
 *   manager slot[s] resp_ready
 *          |
 *          v  (slot_resp[s].valid pulse)
 *   resp_handler -> register_file (latch into next free slot)
 *          |
 *          v  (rf_wr_valid or rf_rd_valid)
 *   AXI wr_resp_fifo / rd_data_fifo push
 *
 * NOTE: register_file has 2 slots per direction. This allows both manager
 * slots to have pending responses simultaneously without blocking each other.
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module resp_handler
import struct_types::*;
#(
    parameter NUM_SLOTS = 2
)
(
    input  logic    clk,
    input  logic    rst_n,

    /*------------------------------------------------------------------*/
    /* From manager: one response descriptor per slot                   */
    /*------------------------------------------------------------------*/
    // valid      : asserted for exactly 1 cycle when response is ready
    // is_wr      : 1=write response, 0=read data beat
    // wr_resp    : assembled B-channel response (valid when is_wr=1)
    // rd_data    : assembled R-channel beat     (valid when is_wr=0)
    // txn_id     : awid or arid, for register_file ID tagging
    input  logic                        slot_valid  [0:NUM_SLOTS-1],
    input  logic                        slot_is_wr  [0:NUM_SLOTS-1],
    input  struct_types::axi_wr_resp    slot_wr_resp[0:NUM_SLOTS-1],
    input  struct_types::axi_rd_data    slot_rd_data[0:NUM_SLOTS-1],
    input  logic [ID_WIDTH-1:0]         slot_txn_id [0:NUM_SLOTS-1],

    // Consumed: asserted same cycle as valid, tells manager slot is free
    output logic                        slot_consumed[0:NUM_SLOTS-1],

    /*------------------------------------------------------------------*/
    /* To top_module: AXI write response FIFO                           */
    /*------------------------------------------------------------------*/
    output logic                        wr_resp_push_n,
    output struct_types::axi_wr_resp    wr_resp_fifo_in,
    input  logic                        wr_resp_full,

    /*------------------------------------------------------------------*/
    /* To top_module: AXI read data FIFO                                */
    /*------------------------------------------------------------------*/
    output logic                        rd_data_push_n,
    output struct_types::axi_rd_data    rd_data_fifo_in,
    input  logic                        rd_data_full
);

    /*=====================================================================*/
    /*  register_file wires                                                */
    /*=====================================================================*/

    logic                       rf_enable_wr, rf_done_wr, rf_wr_valid;
    logic [ID_WIDTH-1:0]        rf_txn_id_wr, rf_wr_id_out;
    struct_types::axi_wr_resp   rf_axi_wr_resp_in, rf_axi_wr_resp_out;

    logic                       rf_enable_rd, rf_done_rd, rf_rd_valid;
    logic [ID_WIDTH-1:0]        rf_txn_id_rd, rf_rd_id_out;
    struct_types::axi_rd_data   rf_axi_rd_data_in, rf_axi_rd_data_out;

    /*=====================================================================*/
    /*  register_file instantiation                                        */
    /*=====================================================================*/

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
    /*  Response ingestion: manager slot -> register_file                  */
    /*=====================================================================*/
    // Arbitrate between two slots. Slot 0 has priority.
    // Only one write and one read can be ingested per cycle (register_file
    // has separate write and read channels so a write and read CAN both
    // be ingested in the same cycle).

    always_comb begin
        rf_enable_wr      = 1'b0;
        rf_txn_id_wr      = '0;
        rf_axi_wr_resp_in = '0;
        rf_enable_rd      = 1'b0;
        rf_txn_id_rd      = '0;
        rf_axi_rd_data_in = '0;

        // Write response ingestion: first valid write slot wins
        for (int s = 0; s < NUM_SLOTS; s++) begin
            if (!rf_enable_wr && slot_valid[s] && slot_is_wr[s]) begin
                rf_enable_wr      = 1'b1;
                rf_txn_id_wr      = slot_txn_id[s];
                rf_axi_wr_resp_in = slot_wr_resp[s];
            end
        end

        // Read data ingestion: first valid read slot wins
        for (int s = 0; s < NUM_SLOTS; s++) begin
            if (!rf_enable_rd && slot_valid[s] && !slot_is_wr[s]) begin
                rf_enable_rd      = 1'b1;
                rf_txn_id_rd      = slot_txn_id[s];
                rf_axi_rd_data_in = slot_rd_data[s];
            end
        end
    end

    // Consumed: acknowledge to manager the same cycle valid is seen.
    // FIX: Only the slot that actually WON arbitration this cycle gets
    // consumed=1.  Previously, if slot 0 and slot 1 both had valid write
    // responses, rf_enable_wr=1 caused consumed=1 for BOTH slots, but the
    // register_file only has room for one entry.  Slot 1's response was
    // silently dropped and the manager never retried it.
    // Now: consumed[s]=1 only when slot s is the first valid winner for
    // its direction -- matching exactly which slot set rf_enable_wr/rd.
    always_comb begin
        for (int s = 0; s < NUM_SLOTS; s++) begin
            slot_consumed[s] = 1'b0;
        end
        // Write direction: consumed for the first valid write slot only
        for (int s = 0; s < NUM_SLOTS; s++) begin
            if (slot_valid[s] && slot_is_wr[s] && rf_enable_wr) begin
                // rf_enable_wr was set by the first valid write slot in the
                // ingestion loop above; check it is THIS slot by re-running
                // the priority: consumed only if no earlier slot already won.
                begin : wr_winner
                    logic already_won;
                    already_won = 1'b0;
                    for (int t = 0; t < s; t++) begin
                        if (slot_valid[t] && slot_is_wr[t])
                            already_won = 1'b1;
                    end
                    if (!already_won) slot_consumed[s] = 1'b1;
                end
                break;
            end
        end
        // Read direction: consumed for the first valid read slot only
        for (int s = 0; s < NUM_SLOTS; s++) begin
            if (slot_valid[s] && !slot_is_wr[s] && rf_enable_rd) begin
                begin : rd_winner
                    logic already_won;
                    already_won = 1'b0;
                    for (int t = 0; t < s; t++) begin
                        if (slot_valid[t] && !slot_is_wr[t])
                            already_won = 1'b1;
                    end
                    if (!already_won) slot_consumed[s] = 1'b1;
                end
                break;
            end
        end
    end

    /*=====================================================================*/
    /*  FIFO push: register_file -> AXI response FIFOs                    */
    /*=====================================================================*/
    // Push whenever the register_file output slot is valid and FIFO has room.
    // rf_done pulses when the push succeeds, advancing the register_file
    // read pointer.

    assign wr_resp_push_n  = !(rf_wr_valid && !wr_resp_full);
    assign wr_resp_fifo_in = rf_axi_wr_resp_out;
    assign rf_done_wr      = !wr_resp_push_n;

    assign rd_data_push_n  = !(rf_rd_valid && !rd_data_full);
    assign rd_data_fifo_in = rf_axi_rd_data_out;
    assign rf_done_rd      = !rd_data_push_n;

endmodule