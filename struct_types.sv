/*------------------------------------------------------------------------------
 * File          : struct_types.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Dec 18, 2025
 * Description   : Shared types and parameters for the AXI-to-APB bridge.
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

package struct_types;

    parameter FIFO_DEPTH = 16;
    /*=========================================================================*/
    /*  AXI parameters                                                         */
    /*=========================================================================*/
    parameter ADDR_WIDTH  = 32;     // AXI address width (bits)
    parameter DATA_WIDTH  = 64;     // AXI data width (bits) -- twice APB width
    parameter ID_WIDTH    = 32;     // AXI transaction ID width (bits)
    parameter RRESP_WIDTH = 2;      // AXI read response code width
    parameter BRESP_WIDTH = 2;      // AXI write response code width
    parameter WSTRB_WIDTH = 8;      // AXI write strobe width (DATA_WIDTH/8)
    parameter MAX_SIZE    = 3;      // AXI size field width (encodes 1-8 bytes)
    parameter MAX_LEN     = 8;      // AXI len field width (encodes burst length)

    /*=========================================================================*/
    /*  APB parameters                                                         */
    /*=========================================================================*/
    parameter PADDR_WIDTH = 32;     // APB address width (bits)
    parameter PDATA_WIDTH = 32;     // APB data width (bits) -- half AXI width
    parameter PSTRB_WIDTH = 4;      // APB strobe width (PDATA_WIDTH/8)

    /*=========================================================================*/
    /*  AXI write channel structs                                              */
    /*=========================================================================*/

    // AW channel: write address / burst descriptor (one per burst)
    typedef struct packed {
        logic [ID_WIDTH-1:0]    awid;   // transaction ID
        logic [ADDR_WIDTH-1:0]  awaddr; // burst base address
        logic [MAX_LEN-1:0]     awlen;  // number of beats minus 1
        logic [MAX_SIZE-1:0]    awsize; // bytes per beat (encoded as log2)
    } axi_wr_req;

    // W channel: write data (one entry per beat)
    typedef struct packed {
        logic [DATA_WIDTH-1:0]  wdata;  // 64-bit write data
        logic [WSTRB_WIDTH-1:0] wstrb;  // 8-bit byte enables
        logic                   wlast;  // last beat of burst
    } axi_wr_data;

    // B (write response) channel: write response (one per burst, sent after wlast)
    typedef struct packed {
        logic [ID_WIDTH-1:0]    bid;    // echoes awid
        logic [BRESP_WIDTH-1:0] bresp;  // OKAY=2'b00, SLVERR=2'b10
    } axi_wr_resp;

    /*=========================================================================*/
    /*  AXI read channel structs                                               */
    /*=========================================================================*/

    // AR channel: read address / burst descriptor (one per burst)
    typedef struct packed {
        logic [ID_WIDTH-1:0]    arid;   // transaction ID
        logic [ADDR_WIDTH-1:0]  araddr; // burst base address
        logic [MAX_LEN-1:0]     arlen;  // number of beats minus 1
        logic [MAX_SIZE-1:0]    arsize; // bytes per beat (encoded as log2)
    } axi_rd_req;

    // R channel: read data (one entry per beat)
    typedef struct packed {
        logic [ID_WIDTH-1:0]    rid;    // echoes arid
        logic [DATA_WIDTH-1:0]  rdata;  // 64-bit read data
        logic [RRESP_WIDTH-1:0] rresp;  // OKAY=2'b00, SLVERR=2'b10
        logic                   rlast;  // last beat of burst
    } axi_rd_data;

    /*=========================================================================*/
    /*  APB struct                                                             */
    /*=========================================================================*/
    // Carries all APB signals through the request and response FIFOs.
    // Response fields (pready, prdata, pslverr) are zeroed on the request
    // path and filled in by apb_master when the slave responds.

    typedef struct packed {
        logic [PADDR_WIDTH-1:0] paddr;   // transaction address
        logic                   pwrite;  // 1=write, 0=read
        logic [PDATA_WIDTH-1:0] pwdata;  // write data (valid when pwrite=1)
        logic [PSTRB_WIDTH-1:0] pstrb;   // byte enables (valid when pwrite=1)
        logic                   pready;  // slave ready (response field)
        logic [PDATA_WIDTH-1:0] prdata;  // read data (response field)
        logic                   pslverr; // slave error (response field)
    } apb_struct;

    /*=========================================================================*/
    /*  Sideband tag struct                                                    */
    /*=========================================================================*/
    // One entry pushed into the sideband FIFO for every APB request pushed.
    // Popped when the APB response arrives to route it to the correct
    // transaction slot and half (LSB or MSB) in the manager.
    //
    // This keeps apb_struct clean -- no transaction metadata leaks into
    // the APB pipeline -- and exploits the fact that APB is strictly
    // ordered (responses arrive in the same order as requests).

    typedef struct packed {
        logic is_wr;     // 1 = write pipeline
        logic slot_idx;  // which of the 2 slots for that direction
        logic is_msb;    // LSB or MSB half
    } apb_tag_t;

endpackage