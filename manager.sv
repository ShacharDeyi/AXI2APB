/*------------------------------------------------------------------------------
 * File          : manager.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Feb 27, 2026
 * Description   : 2-outstanding AXI-to-APB bridge manager.
 *
 * ============================================================================
 * OVERVIEW
 * ============================================================================
 *
 * The manager coordinates the full data path between the AXI FIFOs
 * (populated by axi_slave_wr/rd) and the APB FIFOs (consumed by apb_master).
 * It supports two simultaneously in-flight transactions via two independent
 * transaction slots (slot 0 and slot 1).
 *
 * Internal sub-modules instantiated here:
 *   - disassembler     : splits 64-bit AXI beat into raw 32-bit LSB/MSB fields
 *   - apb_req_builder  : packs raw fields into apb_struct for the APB req FIFO
 *   - assembler        : merges two 32-bit APB responses into 64-bit AXI fields
 *   - axi_resp_builder : packs raw fields into axi_rd_data / axi_wr_resp structs
 *   - register_file    : 2-slot ping-pong buffer for assembled AXI responses
 *
 * ============================================================================
 * HOW EACH REQUEST IS TRACKED
 * ============================================================================
 *
 * When a new AXI transaction arrives it is assigned to whichever slot
 * (0 or 1) is currently SLOT_IDLE. Slot 0 has priority.
 *
 * The slot then independently manages the full lifecycle of that transaction:
 *
 *   1. SLOT_DISPATCH
 *      - Pops wr_req or rd_req FIFO ONCE (latched into slot_wr/rd_req[s]).
 *        These hold awid/awaddr/awlen/awsize (or their AR equivalents)
 *        and remain stable for the entire burst.
 *      - Pops wr_data FIFO every beat (latched into slot_wr_data[s]).
 *      - Computes beat_addr = base_addr + beat_index << size (combinational).
 *      - Drives disassembler + apb_req_builder to produce APB request(s).
 *      - Pushes up to 2 APB requests (LSB first, MSB second) to apb_req_queue.
 *      - Pushes matching sideband tags {slot=s, is_msb=0/1} to apb_tag_queue
 *        in lockstep. This is the key tracking mechanism: every APB request
 *        carries a label saying "this response belongs to slot s, half LSB/MSB".
 *      - Tracks slot_lsb_sent[s] and slot_msb_sent[s] so a FIFO-stall in
 *        one cycle doesn't cause a double-push in the next.
 *      - Latches slot_expected_lsb/msb[s] so SLOT_WAIT_RESP knows how many
 *        responses to collect.
 *      - Moves to SLOT_WAIT_RESP once all valid halves are pushed.
 *
 *   2. SLOT_WAIT_RESP
 *      - The response collector (always_comb block) reads the sideband tag
 *        FIFO head each cycle. When it pops:
 *          tag.slot   -> which slot this response belongs to
 *          tag.is_msb -> which half (LSB or MSB)
 *        Writes prdata/pslverr into slot_lsb/msb_prdata/pslverr[tag.slot].
 *        Sets slot_lsb/msb_valid[tag.slot].
 *      - resp_ready[s] fires when all expected_lsb/msb flags are satisfied.
 *      - On resp_ready:
 *          Drives assembler (merges prdata, computes rresp/bresp).
 *          Drives axi_resp_builder (adds transaction_id, is_last).
 *          Writes assembled response into register_file.
 *          For writes: accumulates pslverr into slot_wr_burst_err[s].
 *                      On wlast -> push B response, return to SLOT_IDLE.
 *                      Else     -> clear per-beat state, increment beat_index,
 *                                  return to SLOT_DISPATCH for next beat.
 *          For reads:  push R data every beat.
 *                      On last beat (beat_index==arlen) -> SLOT_IDLE.
 *                      Else -> increment beat_index, back to SLOT_DISPATCH.
 *
 * ============================================================================
 * APB REQUEST ARBITRATION
 * ============================================================================
 *
 * Both slots may want to push APB requests simultaneously.
 * Only one push is allowed per cycle (both apb_req_queue and apb_tag_queue
 * must stay in sync -- they are always pushed together).
 *
 * Fixed priority: slot 0 wins over slot 1.
 * The losing slot waits one cycle -- negligible given APB's 2+ cycle latency.
 *
 * ============================================================================
 * APB RESPONSE ROUTING (sideband tag FIFO)
 * ============================================================================
 *
 * APB is strictly sequential: responses arrive in exactly the same order
 * as requests. The sideband tag FIFO mirrors this ordering.
 *
 * Each cycle, if both apb_resp_queue and apb_tag_queue are non-empty:
 *   - Read tag = apb_tag_fifo_out  (head of sideband FIFO, no pop yet)
 *   - If slot[tag.slot] is in SLOT_WAIT_RESP: pop both FIFOs
 *   - Route response to slot[tag.slot].lsb or .msb based on tag.is_msb
 *
 * ============================================================================
 * REGISTER FILE
 * ============================================================================
 *
 * register_file provides 2 write-response slots and 2 read-data slots,
 * ping-ponging between them (slot 0 / slot 1 alternate on each enable).
 *
 * Manager -> register_file:
 *   enable_wr / rf_txn_id_wr / rf_axi_wr_resp_in  : on write burst complete
 *   enable_rd / rf_txn_id_rd / rf_axi_rd_data_in  : on each read beat
 *
 * register_file -> AXI response FIFOs (via assign in this module):
 *   rf_wr_valid / rf_axi_wr_resp_out -> wr_resp FIFO push
 *   rf_rd_valid / rf_axi_rd_data_out -> rd_data FIFO push
 *   rf_done_wr / rf_done_rd          : pulsed when the push succeeds
 *
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

module manager
import struct_types::*;
#(
    parameter NUM_SLOTS = 2     // number of in-flight transaction slots
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
    /*  Combinational per-slot derived signals                            */
    /*=====================================================================*/

    logic                     resp_ready          [0:NUM_SLOTS-1];
    logic [ADDR_WIDTH-1:0]    slot_beat_addr      [0:NUM_SLOTS-1];
    logic [MAX_SIZE-1:0]      slot_beat_size      [0:NUM_SLOTS-1];
    logic [ADDR_WIDTH-1:0]    slot_base_addr      [0:NUM_SLOTS-1];
    logic                     slot_is_last        [0:NUM_SLOTS-1];

    /*=====================================================================*/
    /*  Disassembler / apb_req_builder shared wires                      */
    /*  Muxed: only the arbitration-winning slot drives these             */
    /*=====================================================================*/

    logic                     dis_slot;
    logic                     dis_is_wr;
    logic [ADDR_WIDTH-1:0]    dis_beat_addr;
    logic [MAX_SIZE-1:0]      dis_beat_size;
    logic [DATA_WIDTH-1:0]    dis_wdata;
    logic [WSTRB_WIDTH-1:0]   dis_wstrb;

    logic [ADDR_WIDTH-1:0]    dis_lsb_addr,  dis_msb_addr;
    logic [PDATA_WIDTH-1:0]   dis_lsb_wdata, dis_msb_wdata;
    logic [PSTRB_WIDTH-1:0]   dis_lsb_pstrb, dis_msb_pstrb;
    logic                     dis_valid_lsb,  dis_valid_msb;

    struct_types::apb_struct  builder_apb_lsb, builder_apb_msb;

    /*=====================================================================*/
    /*  Assembler / axi_resp_builder shared wires                        */
    /*  Muxed: only the resp_ready-winning slot drives these              */
    /*=====================================================================*/

    logic                     asm_slot;
    logic [DATA_WIDTH-1:0]    asm_rdata;
    logic [RRESP_WIDTH-1:0]   asm_rresp;
    logic [BRESP_WIDTH-1:0]   asm_bresp;

    struct_types::axi_rd_data resp_rd_data;
    struct_types::axi_wr_resp resp_wr_resp;

    /*=====================================================================*/
    /*  Register file wires                                                */
    /*=====================================================================*/

    logic                       rf_enable_wr, rf_done_wr, rf_wr_valid;
    logic [ID_WIDTH-1:0]        rf_txn_id_wr, rf_wr_id_out;
    struct_types::axi_wr_resp   rf_axi_wr_resp_in, rf_axi_wr_resp_out;

    logic                       rf_enable_rd, rf_done_rd, rf_rd_valid;
    logic [ID_WIDTH-1:0]        rf_txn_id_rd, rf_rd_id_out;
    struct_types::axi_rd_data   rf_axi_rd_data_in, rf_axi_rd_data_out;

    /*=====================================================================*/
    /*  Pop want signals (one per slot, arbitrated below)                 */
    /*=====================================================================*/

    logic slot_wants_wr_req_pop  [0:NUM_SLOTS-1];
    logic slot_wants_rd_req_pop  [0:NUM_SLOTS-1];
    logic slot_wants_wr_data_pop [0:NUM_SLOTS-1];

    // Locals moved from inside procedural blocks (SV does not allow logic
    // declarations inside always_ff case statements or always_comb)
    logic s0_dispatching;                       // used in dis_slot mux
    logic s0_takes_wr        [0:NUM_SLOTS-1];  // used in SLOT_IDLE per-slot
    logic s0_takes_rd        [0:NUM_SLOTS-1];  // used in SLOT_IDLE per-slot
    logic slot_lsb_done      [0:NUM_SLOTS-1];  // used in SLOT_DISPATCH per-slot
    logic slot_msb_done      [0:NUM_SLOTS-1];  // used in SLOT_DISPATCH per-slot
    logic slot_req_latched   [0:NUM_SLOTS-1];  // used in SLOT_DISPATCH per-slot

    /*=====================================================================*/
    /*  Sub-module: disassembler                                          */
    /*=====================================================================*/

    disassembler u_disassembler (
        .is_wr      (dis_is_wr),
        .beat_addr  (dis_beat_addr),
        .beat_size  (dis_beat_size),
        .wdata      (dis_wdata),
        .wstrb      (dis_wstrb),
        .lsb_addr   (dis_lsb_addr),
        .lsb_wdata  (dis_lsb_wdata),
        .lsb_pstrb  (dis_lsb_pstrb),
        .valid_lsb  (dis_valid_lsb),
        .msb_addr   (dis_msb_addr),
        .msb_wdata  (dis_msb_wdata),
        .msb_pstrb  (dis_msb_pstrb),
        .valid_msb  (dis_valid_msb)
    );

    /*=====================================================================*/
    /*  Sub-module: apb_req_builder                                       */
    /*=====================================================================*/

    apb_req_builder u_apb_req_builder (
        .is_wr      (dis_is_wr),
        .lsb_addr   (dis_lsb_addr),
        .lsb_wdata  (dis_lsb_wdata),
        .lsb_pstrb  (dis_lsb_pstrb),
        .msb_addr   (dis_msb_addr),
        .msb_wdata  (dis_msb_wdata),
        .msb_pstrb  (dis_msb_pstrb),
        .apb_lsb    (builder_apb_lsb),
        .apb_msb    (builder_apb_msb)
    );

    /*=====================================================================*/
    /*  Sub-module: assembler                                              */
    /*=====================================================================*/

    assembler u_assembler (
        .valid_lsb   (slot_expected_lsb [asm_slot]),
        .valid_msb   (slot_expected_msb [asm_slot]),
        .lsb_prdata  (slot_lsb_prdata   [asm_slot]),
        .lsb_pslverr (slot_lsb_pslverr  [asm_slot]),
        .msb_prdata  (slot_msb_prdata   [asm_slot]),
        .msb_pslverr (slot_msb_pslverr  [asm_slot]),
        .rdata       (asm_rdata),
        .rresp       (asm_rresp),
        .bresp       (asm_bresp)
    );

    /*=====================================================================*/
    /*  Sub-module: axi_resp_builder                                      */
    /*=====================================================================*/

    axi_resp_builder u_axi_resp_builder (
        .transaction_id (slot_is_wr[asm_slot]
                            ? slot_wr_req[asm_slot].awid
                            : slot_rd_req[asm_slot].arid),
        .is_last        (slot_is_last[asm_slot]),
        .rdata          (asm_rdata),
        .rresp          (asm_rresp),
        // For writes: if any prior beat had a slave error, override to SLVERR
        .bresp          (slot_is_wr[asm_slot]
                            ? (slot_wr_burst_err[asm_slot] ? 2'b10 : asm_bresp)
                            : asm_bresp),
        .rd_data        (resp_rd_data),
        .wr_resp        (resp_wr_resp)
    );

    /*=====================================================================*/
    /*  Sub-module: register_file                                         */
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
    /*  Beat address computation (combinational, per slot)                */
    /*=====================================================================*/

    generate
        for (genvar gs = 0; gs < NUM_SLOTS; gs++) begin : gen_beat_addr
            always_comb begin
                slot_beat_size[gs] = slot_is_wr[gs]
                                   ? slot_wr_req[gs].awsize
                                   : slot_rd_req[gs].arsize;
                slot_base_addr[gs] = slot_is_wr[gs]
                                   ? slot_wr_req[gs].awaddr
                                   : slot_rd_req[gs].araddr;

                // beat_offset = beat_index << size (wired shift, no multiplier)
                slot_beat_addr[gs] = slot_base_addr[gs]
                                   + (ADDR_WIDTH'(slot_beat_index[gs])
                                      << slot_beat_size[gs]);

                // is_last: true on the final beat of the burst
                slot_is_last[gs] = slot_is_wr[gs]
                                 ? slot_wr_data[gs].wlast
                                 : (slot_beat_index[gs] == slot_rd_req[gs].arlen);

                // resp_ready: all expected APB responses have arrived
                resp_ready[gs] = (!slot_expected_lsb[gs] || slot_lsb_valid[gs]) &&
                                 (!slot_expected_msb[gs] || slot_msb_valid[gs]);
            end
        end
    endgenerate

    /*=====================================================================*/
    /*  Disassembler input mux (APB push arbitration winner)              */
    /*=====================================================================*/
    // dis_slot is the slot that wins the APB push arbitration this cycle.
    // It drives the disassembler; its builder outputs are the ones pushed.
    // Slot 0 wins if it is dispatching and has not yet sent all halves.

    always_comb begin
        // Check if slot 0 still has halves to push this beat
        // Slot 0 is dispatching if it is in SLOT_DISPATCH with its req
        // latched and has not yet finished sending all halves this beat.
        // No reference to dis_valid_lsb/msb here -- that would be circular
        // since dis_slot feeds the disassembler which produces those signals.
        s0_dispatching = (slot_state[0] == SLOT_DISPATCH) &&
                         (slot_wr_req_latched[0] || slot_rd_req_latched[0]) &&
                         (!slot_lsb_sent[0] || !slot_msb_sent[0]);

        dis_slot = s0_dispatching ? 1'b0 : 1'b1;

        dis_is_wr     = slot_is_wr   [dis_slot];
        dis_beat_addr = slot_beat_addr[dis_slot];
        dis_beat_size = slot_beat_size[dis_slot];
        dis_wdata     = slot_wr_data  [dis_slot].wdata;
        dis_wstrb     = slot_wr_data  [dis_slot].wstrb;
    end

    /*=====================================================================*/
    /*  Assembler input mux (resp_ready arbitration)                      */
    /*=====================================================================*/
    // Slot 0 takes priority if both are resp_ready simultaneously.

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

                // Pop wr_req / rd_req in SLOT_IDLE the same cycle the slot
                // accepts a transaction. This ensures the req is latched by
                // the time we enter SLOT_DISPATCH, so dis_slot arbitration
                // and the APB push gate work correctly from cycle 1 of dispatch.
                if (slot_state[gp] == SLOT_IDLE) begin
                    // For writes: pop wr_req AND wr_data together so both are
                    // latched before the first cycle of SLOT_DISPATCH.
                    if (!wr_req_empty && !wr_data_empty && !s0_takes_wr[gp]) begin
                        slot_wants_wr_req_pop [gp] = 1'b1;
                        slot_wants_wr_data_pop[gp] = 1'b1;
                    end else if ((wr_req_empty || wr_data_empty) && !rd_req_empty && !s0_takes_rd[gp])
                        slot_wants_rd_req_pop[gp] = 1'b1;
                end

                if (slot_state[gp] == SLOT_DISPATCH) begin
                    if (slot_is_wr[gp]) begin
                        // Pop wr_data for subsequent beats (beat 1 onwards,
                        // when returning from WAIT_RESP to DISPATCH)
                        if (!wr_data_empty && slot_wr_req_latched[gp])
                            slot_wants_wr_data_pop[gp] = 1'b1;
                    end
                end
            end
        end
    endgenerate

    /*=====================================================================*/
    /*  FIFO pop arbitration (slot 0 wins for shared FIFOs)               */
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
    /*  APB request push + sideband tag push (arbitrated, lockstep)       */
    /*=====================================================================*/
    // Gate: both apb_req_full and apb_tag_full must be clear.
    // Slot 0 wins. apb_tag always mirrors apb_req push exactly.

    always_comb begin
        apb_req_push_n  = 1'b1;
        apb_req_fifo_in = '0;
        apb_tag_push_n  = 1'b1;
        apb_tag_fifo_in = '0;

        if (!apb_req_full && !apb_tag_full) begin
            for (int s = 0; s < NUM_SLOTS; s++) begin
                // Only consider slots where disassembler is currently pointing
                if (apb_req_push_n &&
                    slot_state[s] == SLOT_DISPATCH &&
                    (slot_wr_req_latched[s] || slot_rd_req_latched[s]) &&
                    dis_slot == logic'(s)) begin

                    if (!slot_lsb_sent[s] && dis_valid_lsb) begin
                        apb_req_push_n  = 1'b0;
                        apb_req_fifo_in = builder_apb_lsb;
                        apb_tag_push_n  = 1'b0;
                        apb_tag_fifo_in = '{slot: logic'(s), is_msb: 1'b0};
                    end else if ((slot_lsb_sent[s] || !dis_valid_lsb) &&
                                 !slot_msb_sent[s] && dis_valid_msb) begin
                        apb_req_push_n  = 1'b0;
                        apb_req_fifo_in = builder_apb_msb;
                        apb_tag_push_n  = 1'b0;
                        apb_tag_fifo_in = '{slot: logic'(s), is_msb: 1'b1};
                    end
                end
            end
        end
    end

    /*=====================================================================*/
    /*  APB response pop + sideband tag pop (lockstep, response routing)  */
    /*=====================================================================*/
    // Pop both FIFOs together when the destination slot is SLOT_WAIT_RESP.
    // The actual routing into slot registers happens in the always_ff below.

    always_comb begin
        apb_resp_pop_n = 1'b1;
        apb_tag_pop_n  = 1'b1;

        if (!apb_resp_empty && !apb_tag_empty) begin
            // Destination slot is encoded in the tag head (no pop yet -- peek)
            if (slot_state[apb_tag_fifo_out.slot] == SLOT_WAIT_RESP) begin
                apb_resp_pop_n = 1'b0;
                apb_tag_pop_n  = 1'b0;
            end
        end
    end

    /*=====================================================================*/
    /*  Register file push (write into rf when beat/burst completes)      */
    /*=====================================================================*/

    always_comb begin
        rf_enable_wr      = 1'b0;
        rf_txn_id_wr      = '0;
        rf_axi_wr_resp_in = '0;
        rf_enable_rd      = 1'b0;
        rf_txn_id_rd      = '0;
        rf_axi_rd_data_in = '0;

        if (slot_state[asm_slot] == SLOT_WAIT_RESP && resp_ready[asm_slot]) begin
            if (slot_is_wr[asm_slot] && slot_wr_data[asm_slot].wlast) begin
                rf_enable_wr      = 1'b1;
                rf_txn_id_wr      = slot_wr_req[asm_slot].awid;
                rf_axi_wr_resp_in = resp_wr_resp;
            end else if (!slot_is_wr[asm_slot]) begin
                rf_enable_rd      = 1'b1;
                rf_txn_id_rd      = slot_rd_req[asm_slot].arid;
                rf_axi_rd_data_in = resp_rd_data;
            end
        end
    end

    // Push register_file outputs to AXI response FIFOs
    assign wr_resp_push_n  = !(rf_wr_valid && !wr_resp_full);
    assign wr_resp_fifo_in = rf_axi_wr_resp_out;
    assign rf_done_wr      = !wr_resp_push_n;

    assign rd_data_push_n  = !(rf_rd_valid && !rd_data_full);
    assign rd_data_fifo_in = rf_axi_rd_data_out;
    assign rf_done_rd      = !rd_data_push_n;

    /*=====================================================================*/
    /*  Per-slot sequential state machine                                  */
    /*=====================================================================*/

    /*=====================================================================*/
    /*  Combinational intermediates for per-slot FSM                      */
    /*  (cannot be declared inside always_ff -- moved here)               */
    /*=====================================================================*/

    always_comb begin
        for (int i = 0; i < NUM_SLOTS; i++) begin
            // Is slot 0 about to take a write this cycle?
            // (Only meaningful when computing slot 1's behaviour)
            s0_takes_wr[i] = (i == 1) &&
                              (slot_state[0] == SLOT_IDLE) &&
                              !wr_req_empty && !wr_data_empty;

            // Is slot 0 about to take a read this cycle?
            s0_takes_rd[i] = (i == 1) &&
                              (slot_state[0] == SLOT_IDLE) &&
                              (wr_req_empty || wr_data_empty) &&
                              !rd_req_empty;

            // Which req FIFO has been latched for this slot?
            slot_req_latched[i] = slot_is_wr[i]
                                 ? slot_wr_req_latched[i]
                                 : slot_rd_req_latched[i];

            // Is the LSB half done dispatching this beat?
            slot_lsb_done[i] = !dis_valid_lsb || slot_lsb_sent[i] ||
                                (!apb_req_push_n && dis_slot == logic'(i) &&
                                 !slot_lsb_sent[i] && dis_valid_lsb);

            // Is the MSB half done dispatching this beat?
            slot_msb_done[i] = !dis_valid_msb || slot_msb_sent[i] ||
                                (!apb_req_push_n && dis_slot == logic'(i) &&
                                 (slot_lsb_sent[i] || !dis_valid_lsb) &&
                                 !slot_msb_sent[i] && dis_valid_msb);
        end
    end

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
                        // Accept a new transaction if one is available.
                        // Slot 0 has priority: slot 1 only accepts a
                        // transaction that slot 0 is not also taking this cycle.
                        //
                        // Write priority: writes accepted before reads.
                        // Slot 1 can shadow slot 0 only if slot 0 took a
                        // write and a read is also waiting (or vice versa),
                        // avoiding races on the shared req FIFOs.
                        //
                        // The req FIFO (wr_req or rd_req) is popped HERE in
                        // SLOT_IDLE so that by the first cycle of SLOT_DISPATCH
                        // the req is already latched and the disassembler has
                        // stable inputs to compute valid_lsb/msb and beat_addr.

                            if (!wr_req_empty && !wr_data_empty && !s0_takes_wr[gi]) begin
                                slot_state          [gi] <= SLOT_DISPATCH;
                                slot_is_wr          [gi] <= 1'b1;
                                // Latch wr_req and wr_data[beat0] unconditionally --
                                // we KNOW we're popping them (the condition above
                                // is identical to the pop-wants condition for this slot).
                                // Avoids relying on always_comb outputs inside always_ff.
                                slot_wr_req        [gi] <= wr_req_fifo_out;
                                slot_wr_req_latched[gi] <= 1'b1;
                                slot_wr_data       [gi] <= wr_data_fifo_out;
                            end else if ((wr_req_empty || wr_data_empty) &&
                                         !rd_req_empty && !s0_takes_rd[gi]) begin
                                slot_state          [gi] <= SLOT_DISPATCH;
                                slot_is_wr          [gi] <= 1'b0;
                                // Latch rd_req unconditionally -- same reasoning.
                                slot_rd_req        [gi] <= rd_req_fifo_out;
                                slot_rd_req_latched[gi] <= 1'b1;
                            end
                        end

                        /*---------------------------------------------------*/
                        SLOT_DISPATCH: begin
                        /*---------------------------------------------------*/

                            // Latch wr_data every beat when this slot wins
                            if (!wr_data_pop_n && slot_wants_wr_data_pop[gi])
                                slot_wr_data[gi] <= wr_data_fifo_out;

                            // Track which halves have been pushed (dis_slot==gi
                            // means the disassembler outputs belong to this slot)
                            if (!apb_req_push_n && dis_slot == logic'(gi)) begin
                                if (!slot_lsb_sent[gi] && dis_valid_lsb) begin
                                    slot_lsb_sent    [gi] <= 1'b1;
                                    slot_expected_lsb[gi] <= 1'b1;
                                end else if ((slot_lsb_sent[gi] || !dis_valid_lsb) &&
                                             !slot_msb_sent[gi] && dis_valid_msb) begin
                                    slot_msb_sent    [gi] <= 1'b1;
                                    slot_expected_msb[gi] <= 1'b1;
                                end
                            end
                            // Halves that are not valid are never expected
                            if (!dis_valid_lsb) slot_expected_lsb[gi] <= 1'b0;
                            if (!dis_valid_msb) slot_expected_msb[gi] <= 1'b0;

                            // Transition to SLOT_WAIT_RESP when all valid halves sent.
                            // A half is "done" if: it was not valid, or it was
                            // already sent, or it is being sent right now.
                            if (slot_req_latched[gi] && slot_lsb_done[gi] && slot_msb_done[gi])
                                slot_state[gi] <= SLOT_WAIT_RESP;
                        end

                        /*---------------------------------------------------*/
                        SLOT_WAIT_RESP: begin
                        /*---------------------------------------------------*/

                            // Route incoming APB response to this slot
                            // (response collector pops when tag.slot == gi)
                            if (!apb_resp_pop_n &&
                                apb_tag_fifo_out.slot == logic'(gi)) begin
                                if (!apb_tag_fifo_out.is_msb) begin
                                    // LSB response
                                    slot_lsb_prdata [gi] <= apb_resp_fifo_out.prdata;
                                    slot_lsb_pslverr[gi] <= apb_resp_fifo_out.pslverr;
                                    slot_lsb_valid  [gi] <= 1'b1;
                                end else begin
                                    // MSB response
                                    slot_msb_prdata [gi] <= apb_resp_fifo_out.prdata;
                                    slot_msb_pslverr[gi] <= apb_resp_fifo_out.pslverr;
                                    slot_msb_valid  [gi] <= 1'b1;
                                end
                            end

                            // When all expected responses are in, advance state
                            // asm_slot == gi ensures this slot is currently
                            // feeding the assembler (not the other slot)
                            if (resp_ready[gi] && asm_slot == logic'(gi)) begin

                                // Clear per-beat response flags
                                slot_lsb_valid   [gi] <= 1'b0;
                                slot_msb_valid   [gi] <= 1'b0;
                                slot_lsb_sent    [gi] <= 1'b0;
                                slot_msb_sent    [gi] <= 1'b0;
                                slot_expected_lsb[gi] <= 1'b0;
                                slot_expected_msb[gi] <= 1'b0;

                                if (slot_is_wr[gi]) begin
                                    // Accumulate write burst error across beats.
                                    // On wlast clear it (B response already sent).
                                    slot_wr_burst_err[gi] <=
                                        slot_wr_data[gi].wlast ? 1'b0
                                        : slot_wr_burst_err[gi] | asm_bresp[1];

                                    if (slot_wr_data[gi].wlast) begin
                                        // Write burst complete
                                        slot_state          [gi] <= SLOT_IDLE;
                                        slot_wr_req_latched [gi] <= 1'b0;
                                        slot_beat_index     [gi] <= '0;
                                        slot_wr_burst_err   [gi] <= 1'b0;
                                    end else begin
                                        // Next beat
                                        slot_state      [gi] <= SLOT_DISPATCH;
                                        slot_beat_index [gi] <= slot_beat_index[gi] + 1'b1;
                                    end

                                end else begin
                                    // Read
                                    if (slot_beat_index[gi] == slot_rd_req[gi].arlen) begin
                                        // Read burst complete
                                        slot_state          [gi] <= SLOT_IDLE;
                                        slot_rd_req_latched [gi] <= 1'b0;
                                        slot_beat_index     [gi] <= '0;
                                    end else begin
                                        // Next beat
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

endmodule