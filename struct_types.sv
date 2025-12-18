/*------------------------------------------------------------------------------
 * File          : struct_types.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Dec 18, 2025
 * Description   :
 *------------------------------------------------------------------------------*/

package struct_types;

	parameter ADDR_WIDTH    = 32,
	parameter DATA_WIDTH    = 64,
	parameter ID_WIDTH      = 32,
	parameter RRESP_WIDTH   = 2,
	parameter BRESP_WIDTH   = 2,
	
	parameter PADDR_WIDTH = 32;
	parameter PDATA_WIDTH = 32; //should be 32, might need 64 for compatibility while testing
	parameter PSTRB_WIDTH = 4; 
			
typedef struct packed {
	logic [ID_WIDTH-1:0]       awid; /*Transaction identifier for the write channels*/
	logic [ADDR_WIDTH-1:0]     awaddr; /*Transaction address*/
	logic 					   awlen; /*Defines the number of data transfers in a transaction*/
} axi_wr_req;

typedef struct packed {
	logic [DATA_WIDTH-1:0]     	wdata; /*DATA_WIDTH*/
	logic [DATA_WIDTH/8-1:0]	wstrb; /*PSTRB indicates which byte lanes to update during a write transfer*/
	logic                      	wlast; /*Last write data*/
} axi_wr_data;

typedef struct packed {
	logic [ID_WIDTH-1:0]       	bid; /*Transaction identifier for the write channels*/
	logic [BRESP_WIDTH-1:0]    	bresp; /*Write response*/
} axi_wr_resp;

typedef struct packed {
	logic [ID_WIDTH-1:0]       	arid; /*Transaction identifier for the write channels*/
	logic [ADDR_WIDTH-1:0]     	araddr; /*Transaction address*/
	logic						arlen; /*Defines the number of data transfers in a transaction*/
} axi_rd_req;

typedef struct packed {
	logic [ID_WIDTH-1:0]       rid;
	logic [DATA_WIDTH-1:0]     rdata;
	logic [RRESP_WIDTH-1:0]    rresp;
	logic                      rlast;
} axi_rd_data;

typedef struct packed {
	logic [PADDR_WIDTH-1:0] 	paddr; 		/*TRANSACTION ADDRESS*/
	logic 						pwrite; 	/*PWRITE INDICATES AN APB WRITE ACCESS WHEN HIGH AND AN APB READ ACCESS WHEN LOW.*/
	logic [PDATA_WIDTH-1:0] 	pwdata; 	/*The PWDATA write data bus is driven by the APB bridge Requester during write cycles when PWRITE is HIGH. */
	logic [PSTRB_WIDTH-1:0] 	pstrb;		/*PSTRB indicates which byte lanes to update during a write transfer*/
	logic 						pready;
	logic [PDATA_WIDTH-1:0] 	prdata; 	/*The PRDATA read data bus is driven by the selected Completer during read cycles when PWRITE is LOW.*/
	logic 						pslverr; 	/*PSLVERR is an optional signal that can be asserted HIGH by the Completer to indicate an error condition on an APB transfer.*/ 
} apb;

endpackage