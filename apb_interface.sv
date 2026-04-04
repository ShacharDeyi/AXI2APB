/*------------------------------------------------------------------------------
 * File          : apb_interface.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Jul 5, 2025
 * Description   : APB4 bus interface definition.
 *
 *   Declares all APB signals and exposes two modports:
 *     - master : used by apb_master (drives requests, reads responses)
 *     - slave  : used by the APB slave / testbench model
 *
 *   Using a SystemVerilog interface enforces correct signal directions
 *   at compile time and simplifies port lists throughout the design.
 *------------------------------------------------------------------------------*/
`timescale 1ns/1ps

interface apb_interface();
//APB interface
	parameter PADDR_WIDTH = 32;
	parameter PDATA_WIDTH = 32;
	parameter PSTRB_WIDTH = PDATA_WIDTH/8; 
	
	
	logic [PADDR_WIDTH-1:0] 	paddr; 		// Transaction address
	logic 						psel; 		// Slave select; asserted by the master to indicate a transfer is required
	logic 						penable; 	// Asserted on the second (and any subsequent wait) cycle of a transfer
	logic [PDATA_WIDTH-1:0] 	pwdata; 	// Write data, driven by the master during write cycles (pwrite=1)
	logic 						pwrite; 	// Direction: 1=write, 0=read
	logic [PSTRB_WIDTH-1:0] 	pstrb;		// Byte enables: which byte lanes to update on a write
	logic 						pready;     // Slave ready; when low, slave inserts wait states
	logic [PDATA_WIDTH-1:0] 	prdata; 	// Read data, driven by the slave during read cycles (pwrite=0)
	logic 						pslverr; 	// Slave error; when high, indicates the transfer ended with an error
	
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