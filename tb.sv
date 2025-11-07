/*------------------------------------------------------------------------------
 * File          : tb.sv
 * Project       : RTL
 * Author        : epsdso
 * Creation date : Nov 3, 2025
 * Description   : Testbench for AXI to APB bridge verification
 *------------------------------------------------------------------------------*/
module tb;
    // Clock and reset
    logic clk;
    logic reset;

    // Instantiate interfaces with correct modports
    axi_interface axi_if();  // Base interface for testbench (master)
    apb_interface apb_if();  // Base interface for testbench (slave)

    // DUT instantiation
    top_module uut (
        .clk(clk),
        .reset(reset),
        .axi(axi_if.slave),  // Connect DUT to slave modport
        .apb(apb_if.master)  // Connect DUT to master modport
    );

    // Clock generation
    initial clk = 0;
    always #5 clk = ~clk;

    // Test stimulus task for AXI write transaction
    task automatic axi_write(input logic [31:0] addr, input logic [63:0] data);
        // Write Address Channel
        axi_if.awvalid = 1'b1;
        axi_if.awaddr = addr;
        axi_if.awid = 32'h1;
        @(posedge clk);
        while (!axi_if.awready) @(posedge clk);
        axi_if.awvalid = 1'b0;

        // Write Data Channel
        axi_if.wvalid = 1'b1;
        axi_if.wdata = data;
        axi_if.wlast = 1'b1;
        @(posedge clk);
        while (!axi_if.wready) @(posedge clk);
        axi_if.wvalid = 1'b0;
        axi_if.wlast = 1'b0;

        // Write Response Channel
        axi_if.bready = 1'b1;
        @(posedge clk);
        while (!axi_if.bvalid) @(posedge clk);
        axi_if.bready = 1'b0;
    endtask

    // Test stimulus task for AXI read transaction
    task automatic axi_read(input logic [31:0] addr);
        // Read Address Channel
        axi_if.arvalid = 1'b1;
        axi_if.araddr = addr;
        axi_if.arid = 32'h2;
        @(posedge clk);
        while (!axi_if.arready) @(posedge clk);
        axi_if.arvalid = 1'b0;

        // Read Data Channel
        axi_if.rready = 1'b1;
        @(posedge clk);
        while (!axi_if.rvalid) @(posedge clk);
        axi_if.rready = 1'b0;
    endtask

    // Test sequence
    initial begin
        // Initialize AXI signals
        axi_if.awvalid = 0;
        axi_if.awaddr = 0;
        axi_if.awid = 0;
        axi_if.wvalid = 0;
        axi_if.wdata = 0;
        axi_if.wlast = 0;
        axi_if.bready = 0;
        axi_if.arvalid = 0;
        axi_if.araddr = 0;
        axi_if.arid = 0;
        axi_if.rready = 0;

        // Reset sequence
        reset = 1;
        repeat(5) @(posedge clk);
        reset = 0;
        repeat(5) @(posedge clk);

        // Test Case 1: Write Transaction
        $display("Starting Write Transaction Test");
        axi_write(32'h1000_0000, 64'hDEAD_BEEF_1234_5678);
        repeat(10) @(posedge clk);

        // Test Case 2: Read Transaction
        $display("Starting Read Transaction Test");
        axi_read(32'h1000_0000);
        repeat(10) @(posedge clk);

        // Add more test cases here as needed

        repeat(20) @(posedge clk);
        $display("Test completed successfully");
        $finish;
    end

    // Assertions for protocol checking
    property axi_write_resp_after_data;
        @(posedge clk) disable iff (reset)
        axi_if.wlast && axi_if.wvalid && axi_if.wready |-> ##[1:$] axi_if.bvalid;
    endproperty

    property axi_read_data_after_addr;
        @(posedge clk) disable iff (reset)
        axi_if.arvalid && axi_if.arready |-> ##[1:$] axi_if.rvalid;
    endproperty

    assert property(axi_write_resp_after_data)
        else $error("Write response not received after write data");
    assert property(axi_read_data_after_addr)
        else $error("Read data not received after address");

    // Coverage
    covergroup axi_cg @(posedge clk);
        write_addr: coverpoint axi_if.awaddr[31:0] {
            bins low = {[0:32'h7FFF_FFFF]};
            bins high = {[32'h8000_0000:32'hFFFF_FFFF]};
        }
        read_addr: coverpoint axi_if.araddr[31:0] {
            bins low = {[0:32'h7FFF_FFFF]};
            bins high = {[32'h8000_0000:32'hFFFF_FFFF]};
        }
    endgroup

    axi_cg cg = new();

endmodule : tb