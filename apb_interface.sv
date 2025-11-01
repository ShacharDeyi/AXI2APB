/*------------------------------------------------------------------------------
 * File          : apb_interface.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Jul 5, 2025
 * Description   :
 *------------------------------------------------------------------------------*/

// APB4 interfcae
interface apb_interface();	
//APB interface
	parameter PADDR_WIDTH = 32;
	parameter PDATA_WIDTH = 32;
	parameter PSTRB_WIDTH = 4; 
	parameter PUSER_WIDTH = 32; //size of request
	
	wire pready;
	wire [PDATA_WIDTH-1:0] 	prdata; 	/*The PRDATA read data bus is driven by the selected Completer during read cycles when PWRITE is LOW.*/
	wire 					pslverr; 	/*PSLVERR is an optional signal that can be asserted HIGH by the Completer to indicate an error condition on an APB transfer.*/ 
	wire [PADDR_WIDTH-1:0] 	paddr; 		/*TRANSACTION ADDRESS*/
	wire [2:0] 				pprot;
	wire 					penable; 	/*PENABLE indicates the second and subsequent cycles of an APB transfer.*/
	wire 					pwrite; 	/*PWRITE INDICATES AN APB WRITE ACCESS WHEN HIGH AND AN APB READ ACCESS WHEN LOW.*/
	wire [PDATA_WIDTH-1:0] 	pwdata; 	/*The PWDATA write data bus is driven by the APB bridge Requester during write cycles when PWRITE is HIGH. */
	wire 					psel; 		/*The Requester generates a PSELx signal for each Completer. PSELx indicates that the Completer is selected and that a data transfer is required.*/
	wire [PSTRB_WIDTH-1:0] 	pstrb;		/*PSTRB indicates which byte lanes to update during a write transfer. There is one write strobe for each 8 bits of the write data bus. PSTRB[n] corresponds to PWDATA[(8n + 7):(8n)]. PSTRB must not be active during a read transfer.*/
	wire [PUSER_WIDTH-1:0] 	puser; 	

	modport slave (
		
		input paddr,
		input pprot,
		input penable,
		input psel,
		input pwdata,
		input pwrite,
		input pstrb,
		input puser,

		output pready,
		output prdata,
		output pslverr
	);
	
	modport master (
		output paddr,
		output pprot,
		output penable,
		output psel, 
		output pwdata,
		output pwrite,
		output pstrb,
		output puser,

		input pready, 
		input prdata, 
		input pslverr
	);
			
endinterface: apb_interface