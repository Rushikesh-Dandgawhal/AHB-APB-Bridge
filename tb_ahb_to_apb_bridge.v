// =============================================================================
//  Testbench: AHB to APB Bridge
//  Tests: single write, single read, burst write, burst read, back-to-back
// =============================================================================
`ifdef SIMULATION
 
module tb_ahb_to_apb_bridge;
 
    parameter DW = 32;
    parameter AW = 32;
    parameter NS = 3;
 
    // Clock and reset
    reg         HCLK, HRESETn;
    // AHB
    reg         HSEL;
    reg [AW-1:0] HADDR;
    reg         HWRITE;
    reg [1:0]   HTRANS;
    reg [2:0]   HSIZE, HBURST, HPROT;
    reg [DW-1:0] HWDATA;
    reg         HREADYIN;
    wire [DW-1:0] HRDATA;
    wire        HREADYOUT;
    wire [1:0]  HRESP;
    // APB
    wire [AW-1:0] PADDR;
    wire          PWRITE;
    wire [NS-1:0] PSEL;
    wire          PENABLE;
    wire [DW-1:0] PWDATA;
    reg  [DW-1:0] PRDATA;
    reg           PREADY;
    reg           PSLVERR;
 
    // DUT
    ahb_to_apb_bridge #(.DATA_WIDTH(DW), .ADDR_WIDTH(AW), .NUM_SLAVES(NS)) dut (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .HSEL(HSEL), .HADDR(HADDR), .HWRITE(HWRITE), .HTRANS(HTRANS),
        .HSIZE(HSIZE), .HBURST(HBURST), .HPROT(HPROT), .HWDATA(HWDATA),
        .HREADYIN(HREADYIN),
        .HRDATA(HRDATA), .HREADYOUT(HREADYOUT), .HRESP(HRESP),
        .PADDR(PADDR), .PWRITE(PWRITE), .PSEL(PSEL), .PENABLE(PENABLE),
        .PWDATA(PWDATA), .PRDATA(PRDATA), .PREADY(PREADY), .PSLVERR(PSLVERR)
    );
 
    // Clock generation: 10ns period
    initial HCLK = 0;
    always #5 HCLK = ~HCLK;
 
    // APB slave model: respond after 1 cycle (PREADY always 1)
    initial begin
        PREADY  = 1'b1;
        PSLVERR = 1'b0;
        PRDATA  = 32'hDEAD_BEEF;
    end
 
    // Task: AHB single write
    task ahb_write;
        input [AW-1:0] addr;
        input [DW-1:0] data;
        begin
            @(posedge HCLK); #1;
            // Address phase
            HSEL    = 1'b1;
            HADDR   = addr;
            HWRITE  = 1'b1;
            HTRANS  = 2'b10;  // NONSEQ
            HREADYIN= 1'b1;
            @(posedge HCLK); #1;
            // Data phase
            HWDATA  = data;
            HTRANS  = 2'b00;  // IDLE
            HSEL    = 1'b0;
            // Wait for HREADYOUT
            while (!HREADYOUT) @(posedge HCLK);
            $display("[%0t] WRITE addr=%h data=%h -> PADDR=%h PWDATA=%h",
                     $time, addr, data, PADDR, PWDATA);
        end
    endtask
 
    // Task: AHB single read
    task ahb_read;
        input [AW-1:0] addr;
        begin
            @(posedge HCLK); #1;
            HSEL    = 1'b1;
            HADDR   = addr;
            HWRITE  = 1'b0;
            HTRANS  = 2'b10;
            HREADYIN= 1'b1;
            @(posedge HCLK); #1;
            HTRANS  = 2'b00;
            HSEL    = 1'b0;
            while (!HREADYOUT) @(posedge HCLK);
            $display("[%0t] READ  addr=%h -> HRDATA=%h", $time, addr, HRDATA);
        end
    endtask
 
    initial begin
        // Apply reset
        HRESETn  = 0;
        HSEL     = 0; HADDR = 0; HWRITE = 0;
        HTRANS   = 0; HSIZE = 3'b010; HBURST = 0; HPROT = 0;
        HWDATA   = 0; HREADYIN = 1;
        repeat(3) @(posedge HCLK);
        HRESETn = 1;
 
        // Test 1: Single write
        $display("--- Test 1: Single Write ---");
        ahb_write(32'h4000_0000, 32'hA5A5_A5A5);
 
        // Test 2: Single read
        $display("--- Test 2: Single Read ---");
        ahb_read(32'h4000_0004);
 
        // Test 3: Burst write (4 beats)
        $display("--- Test 3: Burst Write (4 beats) ---");
        @(posedge HCLK); #1;
        HSEL = 1; HADDR = 32'h4000_0010; HWRITE = 1;
        HTRANS = 2'b10; HREADYIN = 1;
        @(posedge HCLK); #1; HWDATA = 32'h1111_1111; HADDR = 32'h4000_0014; HTRANS = 2'b11;
        @(posedge HCLK); #1; HWDATA = 32'h2222_2222; HADDR = 32'h4000_0018; HTRANS = 2'b11;
        @(posedge HCLK); #1; HWDATA = 32'h3333_3333; HADDR = 32'h4000_001C; HTRANS = 2'b11;
        @(posedge HCLK); #1; HWDATA = 32'h4444_4444; HTRANS = 2'b00; HSEL = 0;
        repeat(8) @(posedge HCLK);
        $display("Burst write done");
 
        // Test 4: Burst read (4 beats)
        $display("--- Test 4: Burst Read (4 beats) ---");
        @(posedge HCLK); #1;
        HSEL = 1; HADDR = 32'h4000_0020; HWRITE = 0;
        HTRANS = 2'b10; HREADYIN = 1;
        @(posedge HCLK); #1; HADDR = 32'h4000_0024; HTRANS = 2'b11;
        @(posedge HCLK); #1; HADDR = 32'h4000_0028; HTRANS = 2'b11;
        @(posedge HCLK); #1; HADDR = 32'h4000_002C; HTRANS = 2'b11;
        @(posedge HCLK); #1; HTRANS = 2'b00; HSEL = 0;
        repeat(12) @(posedge HCLK);
        $display("Burst read done");
 
        $display("--- All tests complete ---");
        $finish;
    end
 
    // Waveform dump
    initial begin
        $dumpfile("ahb_apb_bridge.vcd");
        $dumpvars(0, tb_ahb_to_apb_bridge);
    end
 
endmodule
 
`endif // SIMULATION