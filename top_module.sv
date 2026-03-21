/*------------------------------------------------------------------------------
 * File          : top_module.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Jul 5, 2025
 * Description   : AXI-to-APB bridge top level.
 *
 * RESPONSIBILITY
 * ==============
 * top_module owns:
 *   - All inter-module FIFOs (data highways between modules)
 *   - Module instantiation and port wiring
 *
 * top_module does NOT own:
 *   - Any logic (no FSMs, no counters, no data manipulation)
 *   - Modules that serve only one other module (e.g. register_file,
 *     disassembler, assembler -- those live inside manager)
 *------------------------------------------------------------------------------*/

module top_module
import struct_types::*;
(
    input logic             clk,
    input logic             rst_n,
    axi_interface.slave     axi,    // External AXI bus (slave port)
    apb_interface.master    apb     // External APB bus (master port)
);

    /*=========================================================================*/
    /*  AXI write channel signals                                              */
    /*=========================================================================*/

    // AW channel (write request)
    logic       wr_req_push_n,  wr_req_pop_n,  wr_req_full,  wr_req_empty;
    axi_wr_req  wr_req_fifo_in, wr_req_fifo_out;

    // W channel (write data)
    logic       wr_data_push_n, wr_data_pop_n, wr_data_full, wr_data_empty;
    axi_wr_data wr_data_fifo_in, wr_data_fifo_out;

    // B channel (write response)
    logic       wr_resp_push_n, wr_resp_pop_n, wr_resp_full, wr_resp_empty;
    axi_wr_resp wr_resp_fifo_in, wr_resp_fifo_out;

    /*=========================================================================*/
    /*  AXI read channel signals                                               */
    /*=========================================================================*/

    // AR channel (read request)
    logic       rd_req_push_n,  rd_req_pop_n,  rd_req_full,  rd_req_empty;
    axi_rd_req  rd_req_fifo_in, rd_req_fifo_out;

    // R channel (read data)
    logic       rd_data_push_n, rd_data_pop_n, rd_data_full, rd_data_empty;
    axi_rd_data rd_data_fifo_in, rd_data_fifo_out;

    /*=========================================================================*/
    /*  APB request, response, and sideband tag signals                        */
    /*=========================================================================*/

    logic       apb_req_push_n,  apb_req_pop_n,  apb_req_full,  apb_req_empty;
    apb_struct  apb_req_fifo_in, apb_req_fifo_out;

    logic       apb_resp_push_n, apb_resp_pop_n, apb_resp_full, apb_resp_empty;
    apb_struct  apb_resp_fifo_in, apb_resp_fifo_out;

    // Sideband tag: pushed with every APB req, popped with every APB resp
    logic       apb_tag_push_n, apb_tag_pop_n, apb_tag_full, apb_tag_empty;
    apb_tag_t   apb_tag_fifo_in, apb_tag_fifo_out;

    /*=========================================================================*/
    /*  AXI write slave                                                        */
    /*=========================================================================*/

    axi_slave_wr u_axi_slave_wr (
        .clk                (clk),
        .rst_n              (rst_n),
        .axi                (axi),
        .req_fifo_push_n    (wr_req_push_n),
        .req_fifo_data_out  (wr_req_fifo_in),
        .req_fifo_full      (wr_req_full),
        .data_fifo_push_n   (wr_data_push_n),
        .data_fifo_data_out (wr_data_fifo_in),
        .data_fifo_full     (wr_data_full),
        .resp_fifo_pop_n    (wr_resp_pop_n),
        .resp_fifo_data_out (wr_resp_fifo_out),
        .resp_fifo_empty    (wr_resp_empty)
    );

    /*=========================================================================*/
    /*  AXI write channel FIFOs                                                */
    /*=========================================================================*/

    DW_fifo_s1_sf #(
        .width ($bits(axi_wr_req)),
        .depth (16)
    ) axi_wr_req_queue (
        .clk        (clk),
        .rst_n      (rst_n),
        .push_req_n (wr_req_push_n),
        .data_in    (wr_req_fifo_in),
        .full       (wr_req_full),
        .data_out   (wr_req_fifo_out),
        .empty      (wr_req_empty),
        .pop_req_n  (wr_req_pop_n)
    );

    DW_fifo_s1_sf #(
        .width ($bits(axi_wr_data)),
        .depth (16)
    ) axi_wr_data_queue (
        .clk        (clk),
        .rst_n      (rst_n),
        .push_req_n (wr_data_push_n),
        .data_in    (wr_data_fifo_in),
        .full       (wr_data_full),
        .data_out   (wr_data_fifo_out),
        .empty      (wr_data_empty),
        .pop_req_n  (wr_data_pop_n)
    );

    DW_fifo_s1_sf #(
        .width ($bits(axi_wr_resp)),
        .depth (16)
    ) axi_wr_resp_queue (
        .clk        (clk),
        .rst_n      (rst_n),
        .push_req_n (wr_resp_push_n),
        .data_in    (wr_resp_fifo_in),
        .full       (wr_resp_full),
        .data_out   (wr_resp_fifo_out),
        .empty      (wr_resp_empty),
        .pop_req_n  (wr_resp_pop_n)
    );

    /*=========================================================================*/
    /*  AXI read slave                                                         */
    /*=========================================================================*/

    axi_slave_rd u_axi_slave_rd (
        .clk                (clk),
        .rst_n              (rst_n),
        .axi                (axi),
        .req_fifo_push_n    (rd_req_push_n),
        .req_fifo_data_out  (rd_req_fifo_in),
        .req_fifo_full      (rd_req_full),
        .data_fifo_pop_n    (rd_data_pop_n),
        .data_fifo_data_in  (rd_data_fifo_out),
        .data_fifo_empty    (rd_data_empty)
    );

    /*=========================================================================*/
    /*  AXI read channel FIFOs                                                 */
    /*=========================================================================*/

    DW_fifo_s1_sf #(
        .width ($bits(axi_rd_req)),
        .depth (16)
    ) axi_rd_req_queue (
        .clk        (clk),
        .rst_n      (rst_n),
        .push_req_n (rd_req_push_n),
        .data_in    (rd_req_fifo_in),
        .full       (rd_req_full),
        .data_out   (rd_req_fifo_out),
        .empty      (rd_req_empty),
        .pop_req_n  (rd_req_pop_n)
    );

    DW_fifo_s1_sf #(
        .width ($bits(axi_rd_data)),
        .depth (16)
    ) axi_rd_data_queue (
        .clk        (clk),
        .rst_n      (rst_n),
        .push_req_n (rd_data_push_n),
        .data_in    (rd_data_fifo_in),
        .full       (rd_data_full),
        .data_out   (rd_data_fifo_out),
        .empty      (rd_data_empty),
        .pop_req_n  (rd_data_pop_n)
    );

    /*=========================================================================*/
    /*  APB request FIFO + sideband tag FIFO                                   */
    /*=========================================================================*/
    // apb_req_queue  : carries full APB request struct to apb_master
    // apb_tag_queue  : carries 2-bit {slot, is_msb} routing tag in parallel
    // Both pushed together by manager; tag popped together with apb_resp
    // so manager always knows which slot/half each response belongs to.

    DW_fifo_s1_sf #(
        .width ($bits(apb_struct)),
        .depth (16)
    ) apb_req_queue (
        .clk        (clk),
        .rst_n      (rst_n),
        .push_req_n (apb_req_push_n),
        .data_in    (apb_req_fifo_in),
        .full       (apb_req_full),
        .data_out   (apb_req_fifo_out),
        .empty      (apb_req_empty),
        .pop_req_n  (apb_req_pop_n)
    );

    DW_fifo_s1_sf #(
        .width ($bits(apb_tag_t)),      // 2 bits: {slot, is_msb}
        .depth (16)                     // must match apb_req_queue depth
    ) apb_tag_queue (
        .clk        (clk),
        .rst_n      (rst_n),
        .push_req_n (apb_tag_push_n),
        .data_in    (apb_tag_fifo_in),
        .full       (apb_tag_full),
        .data_out   (apb_tag_fifo_out),
        .empty      (apb_tag_empty),
        .pop_req_n  (apb_tag_pop_n)
    );

    /*=========================================================================*/
    /*  APB master + response FIFO                                             */
    /*=========================================================================*/

    apb_master u_apb_master (
        .clk                (clk),
        .rst_n              (rst_n),
        .apb                (apb),
        .req_fifo_pop_n     (apb_req_pop_n),
        .req_fifo_data_out  (apb_req_fifo_out),
        .req_fifo_empty     (apb_req_empty),
        .resp_fifo_push_n   (apb_resp_push_n),
        .resp_fifo_data_in  (apb_resp_fifo_in),
        .resp_fifo_full     (apb_resp_full)
    );

    DW_fifo_s1_sf #(
        .width ($bits(apb_struct)),
        .depth (16)
    ) apb_resp_queue (
        .clk        (clk),
        .rst_n      (rst_n),
        .push_req_n (apb_resp_push_n),
        .data_in    (apb_resp_fifo_in),
        .full       (apb_resp_full),
        .data_out   (apb_resp_fifo_out),
        .empty      (apb_resp_empty),
        .pop_req_n  (apb_resp_pop_n)
    );

    /*=========================================================================*/
    /*  Manager                                                                */
    /*=========================================================================*/
    // Owns FSM, beat tracking, and all internal sub-modules:
    //   disassembler, apb_req_builder, assembler, axi_resp_builder,
    //   and register_file (2-slot response buffer for 2-outstanding support).

    manager u_manager (
        .clk                (clk),
        .rst_n              (rst_n),

        // AXI read request
        .rd_req_pop_n       (rd_req_pop_n),
        .rd_req_empty       (rd_req_empty),
        .rd_req_fifo_out    (rd_req_fifo_out),

        // AXI write request
        .wr_req_pop_n       (wr_req_pop_n),
        .wr_req_empty       (wr_req_empty),
        .wr_req_fifo_out    (wr_req_fifo_out),

        // AXI write data
        .wr_data_pop_n      (wr_data_pop_n),
        .wr_data_empty      (wr_data_empty),
        .wr_data_fifo_out   (wr_data_fifo_out),

        // APB request + sideband tag (pushed together)
        .apb_req_push_n     (apb_req_push_n),
        .apb_req_fifo_in    (apb_req_fifo_in),
        .apb_req_full       (apb_req_full),
        .apb_tag_push_n     (apb_tag_push_n),
        .apb_tag_fifo_in    (apb_tag_fifo_in),
        .apb_tag_full       (apb_tag_full),

        // APB response + sideband tag (popped together)
        .apb_resp_pop_n     (apb_resp_pop_n),
        .apb_resp_empty     (apb_resp_empty),
        .apb_resp_fifo_out  (apb_resp_fifo_out),
        .apb_tag_pop_n      (apb_tag_pop_n),
        .apb_tag_empty      (apb_tag_empty),
        .apb_tag_fifo_out   (apb_tag_fifo_out),

        // AXI read data
        .rd_data_push_n     (rd_data_push_n),
        .rd_data_fifo_in    (rd_data_fifo_in),
        .rd_data_full       (rd_data_full),

        // AXI write response
        .wr_resp_push_n     (wr_resp_push_n),
        .wr_resp_fifo_in    (wr_resp_fifo_in),
        .wr_resp_full       (wr_resp_full)
    );

endmodule