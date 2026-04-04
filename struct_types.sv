/*------------------------------------------------------------------------------
 * File          : struct_types.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Dec 18, 2025
 * Description   : Shared parameters and packed struct definitions for the
 *                 AXI-to-APB bridge.
 *
 *   All modules import this package to get consistent type and width
 *   definitions without duplicating constants.
 *
 *   AXI parameters: ADDR_WIDTH, DATA_WIDTH (64-bit), ID_WIDTH, RESP_WIDTH,
 *                   WSTRB_WIDTH, MAX_SIZE, MAX_LEN.
 *   APB parameters: PADDR_WIDTH, PDATA_WIDTH (32-bit), PSTRB_WIDTH.
 *
 *   Structs for FIFO payloads:
 *     axi_wr_req  — AW channel (burst descriptor)
 *     axi_wr_data — W  channel (data + strobe + last)
 *     axi_wr_resp — B  channel (id + response code)
 *     axi_rd_req  — AR channel (burst descriptor)
 *     axi_rd_data — R  channel (id + data + response + last)
 *     apb_struct  — full APB transaction (request fields + response fields)
 *     apb_tag_t   — 3-bit sideband routing tag {is_wr, slot_idx, is_msb}
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

package struct_types;

    parameter FIFO_DEPTH = 2;
    /*=========================================================================*/
    /*  AXI parameters                                                         */
    /*=========================================================================*/
    parameter ADDR_WIDTH  = 32;     // Address width (bits) — same on both AXI and APB
    parameter DATA_WIDTH  = 64;     // AXI data width (bits) — twice the APB width
    parameter ID_WIDTH    = 32;     // AXI transaction ID width (bits)
    parameter RESP_WIDTH = 2;       // AXI response code width (OKAY=2'b00, SLVERR=2'b10)
    parameter WSTRB_WIDTH = 8;      // AXI write strobe width (one bit per byte lane)
    parameter MAX_SIZE    = 3;      // AXI size field width (encodes 1-8 bytes per beat)
    parameter MAX_LEN     = 8;      // AXI len field width (encodes burst length 1-256)

    /*=========================================================================*/
    /*  APB parameters                                                         */
    /*=========================================================================*/
    parameter PADDR_WIDTH = 32;     // APB address width (bits)
    parameter PDATA_WIDTH = 32;     // APB data width (bits) — half the AXI width
    parameter PSTRB_WIDTH = 4;      // APB strobe width (one bit per byte lane)

    /*=========================================================================*/
    /*  AXI write channel structs                                              */
    /*=========================================================================*/

    // AW channel payload: burst descriptor, one entry per AXI write transaction.
    typedef struct packed {
        logic [ID_WIDTH-1:0]    awid;   // Transaction ID (echoed in bid)
        logic [ADDR_WIDTH-1:0]  awaddr; // Burst base address
        logic [MAX_LEN-1:0]     awlen;  // Number of beats minus 1
        logic [MAX_SIZE-1:0]    awsize; // Bytes per beat, encoded as log2
    } axi_wr_req;

    // W channel payload: write data, one entry per beat.
    typedef struct packed {
        logic [DATA_WIDTH-1:0]  wdata;  // 64-bit write data
        logic [WSTRB_WIDTH-1:0] wstrb;  // 8-bit byte enables
        logic                   wlast;  // Asserted on the last beat of the burst
    } axi_wr_data;

    // B channel payload: write response, one entry per burst (sent after wlast).
    typedef struct packed {
        logic [ID_WIDTH-1:0]    bid;    // Echoes awid
        logic [RESP_WIDTH-1:0] bresp;  // OKAY=2'b00, SLVERR=2'b10
    } axi_wr_resp;

    /*=========================================================================*/
    /*  AXI read channel structs                                               */
    /*=========================================================================*/

    // AR channel payload: burst descriptor, one entry per AXI read transaction.
    typedef struct packed {
        logic [ID_WIDTH-1:0]    arid;   // Transaction ID (echoed in rid)
        logic [ADDR_WIDTH-1:0]  araddr; // Burst base address
        logic [MAX_LEN-1:0]     arlen;  // Number of beats minus 1
        logic [MAX_SIZE-1:0]    arsize; // Bytes per beat, encoded as log2
    } axi_rd_req;

    // R channel payload: read data, one entry per beat.
    typedef struct packed {
        logic [ID_WIDTH-1:0]    rid;    // Echoes arid
        logic [DATA_WIDTH-1:0]  rdata;  // 64-bit read data
        logic [RESP_WIDTH-1:0] rresp;  // OKAY=2'b00, SLVERR=2'b10
        logic                   rlast;  // Asserted on the last beat of the burst
    } axi_rd_data;

    /*=========================================================================*/
    /*  APB struct                                                             */
    /*=========================================================================*/
    // Single struct carrying all APB signals through both the request and
    // response FIFOs.  On the request path, the response fields (pready,
    // prdata, pslverr) are zeroed by apb_req_builder; apb_master fills them
    // in when the slave responds and pushes the completed struct to the
    // response FIFO.

    typedef struct packed {
        logic [PADDR_WIDTH-1:0] paddr;   // Transaction address
        logic                   pwrite;  // Direction: 1=write, 0=read
        logic [PDATA_WIDTH-1:0] pwdata;  // Write data (valid when pwrite=1)
        logic [PSTRB_WIDTH-1:0] pstrb;   // Byte enables (valid when pwrite=1)
        logic                   pready;  // Slave ready flag (response field)
        logic [PDATA_WIDTH-1:0] prdata;  // Read data from slave (response field)
        logic                   pslverr; // Slave error flag (response field)
    } apb_struct;

    /*=========================================================================*/
    /*  Sideband tag struct                                                    */
    /*=========================================================================*/
    // One tag entry is pushed into the sideband FIFO alongside every APB
    // request, and popped when the corresponding APB response arrives.
    // The manager uses the tag to route each response to the correct
    // transaction slot and half (LSB or MSB) without embedding routing
    // metadata inside the APB struct itself.
    //
    // This works because APB is strictly ordered: responses arrive in
    // exactly the same order as requests.

    typedef struct packed {
        logic is_wr;     // 1 = write pipeline, 0 = read pipeline
        logic slot_idx;  // Which of the 2 register-file slots for that direction
        logic is_msb;    // 0 = LSB half, 1 = MSB half of the 64→32 split
    } apb_tag_t;

endpackage