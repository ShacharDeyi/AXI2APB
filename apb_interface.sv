/*------------------------------------------------------------------------------
 * File          : apb_interface.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Jul 5, 2025
 * Description   :
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

// APB4 interfcae
interface apb_interface();	
//APB interface
	parameter PADDR_WIDTH = 32;
	parameter PDATA_WIDTH = 32; //should be 32, might need 64 for compatibility while testing
	parameter PSTRB_WIDTH = 4; 
	
	
	logic [PADDR_WIDTH-1:0] 	paddr; 		/*TRANSACTION ADDRESS*/
	logic 						psel; 		/*The Requester generates a PSELx signal for each Completer. PSELx indicates that the Completer is selected and that a data transfer is required.*/
	logic 						penable; 	/*PENABLE indicates the second and subsequent cycles of an APB transfer.*/
	logic [PDATA_WIDTH-1:0] 	pwdata; 	/*The PWDATA write data bus is driven by the APB bridge Requester during write cycles when PWRITE is HIGH. */
	logic 						pwrite; 	/*PWRITE INDICATES AN APB WRITE ACCESS WHEN HIGH AND AN APB READ ACCESS WHEN LOW.*/
	logic [PSTRB_WIDTH-1:0] 	pstrb;		/*PSTRB indicates which byte lanes to update during a write transfer*/
	logic 						pready;
	logic [PDATA_WIDTH-1:0] 	prdata; 	/*The PRDATA read data bus is driven by the selected Completer during read cycles when PWRITE is LOW.*/
	logic 						pslverr; 	/*PSLVERR is an optional signal that can be asserted HIGH by the Completer to indicate an error condition on an APB transfer.*/ 
	
	modport slave (
		
		input paddr,
		input psel,
		input penable,
		input pwdata,
		input pwrite,
		input pstrb,

		output pready,
		output prdata,
		output pslverr
	);
	
	modport master (
		output paddr,
		output psel, 
		output penable,
		output pwdata,
		output pwrite,
		output pstrb,

		input pready, 
		input prdata, 
		input pslverr
	);
			
endinterface: apb_interface