// =============================================================================
//  AHB-Lite to APB Bridge
//  AMBA 2.0 Compliant
//
//  Architecture:
//    - AHB Slave Interface  : captures address/control in address phase
//    - APB Controller FSM   : 8-state machine drives PSEL/PENABLE/PWRITE
//    - Supports: single read, single write, burst read, burst write,
//                back-to-back (pipelined) transfers
//
//  Timing model:
//    AHB address phase  -> latch HADDR, HWRITE, HSEL, HTRANS
//    AHB data phase     -> HWDATA valid (one cycle after address)
//    APB setup phase    -> PSEL=1, PENABLE=0  (one HCLK cycle)
//    APB enable phase   -> PSEL=1, PENABLE=1  (one HCLK cycle minimum)
//
//  FSM States:
//    ST_IDLE     : idle, waiting for valid AHB transfer
//    ST_WWAIT    : write address latched, waiting for HWDATA
//    ST_WRITE    : APB write setup phase (no next transfer pending)
//    ST_WRITEP   : APB write setup phase (next transfer pending)
//    ST_WENABLE  : APB write enable phase (no next transfer pending)
//    ST_WENABLEP : APB write enable phase (next transfer pending)
//    ST_READ     : APB read setup phase
//    ST_RENABLE  : APB read enable phase (HRDATA captured)
// =============================================================================
 
module ahb_to_apb_bridge #(
    parameter DATA_WIDTH  = 32,   // AHB/APB data width
    parameter ADDR_WIDTH  = 32,   // AHB/APB address width
    parameter NUM_SLAVES  = 3     // Number of APB slaves (PSEL width)
)(
    // -----------------------------------------------------------------------
    // Global signals
    // -----------------------------------------------------------------------
    input  wire                   HCLK,        // AHB clock
    input  wire                   HRESETn,     // AHB active-low reset
 
    // -----------------------------------------------------------------------
    // AHB Slave Interface (from AHB Master / Interconnect)
    // -----------------------------------------------------------------------
    input  wire                   HSEL,        // Slave select (from decoder)
    input  wire [ADDR_WIDTH-1:0]  HADDR,       // Transfer address
    input  wire                   HWRITE,      // 1=Write, 0=Read
    input  wire [1:0]             HTRANS,      // Transfer type (IDLE/BUSY/NONSEQ/SEQ)
    input  wire [2:0]             HSIZE,       // Transfer size (byte/half/word)
    input  wire [2:0]             HBURST,      // Burst type
    input  wire [3:0]             HPROT,       // Protection control
    input  wire [DATA_WIDTH-1:0]  HWDATA,      // Write data
    input  wire                   HREADYIN,    // Previous slave ready (bus ready in)
 
    output wire [DATA_WIDTH-1:0]  HRDATA,      // Read data back to master
    output wire                   HREADYOUT,   // This slave ready (stall when 0)
    output wire [1:0]             HRESP,       // Transfer response (OKAY/ERROR)
 
    // -----------------------------------------------------------------------
    // APB Master Interface (to APB Slaves)
    // -----------------------------------------------------------------------
    output wire [ADDR_WIDTH-1:0]  PADDR,       // APB address
    output wire                   PWRITE,      // 1=Write, 0=Read
    output wire [NUM_SLAVES-1:0]  PSEL,        // Per-slave select
    output wire                   PENABLE,     // APB enable (2nd cycle)
    output wire [DATA_WIDTH-1:0]  PWDATA,      // APB write data
 
    input  wire [DATA_WIDTH-1:0]  PRDATA,      // APB read data (muxed from slaves)
    input  wire                   PREADY,      // APB slave ready (optional; tie 1 if unused)
    input  wire                   PSLVERR      // APB slave error (optional; tie 0 if unused)
);
 
    // -----------------------------------------------------------------------
    // HTRANS encoding (AMBA spec)
    // -----------------------------------------------------------------------
    localparam HTRANS_IDLE   = 2'b00;
    localparam HTRANS_BUSY   = 2'b01;
    localparam HTRANS_NONSEQ = 2'b10;
    localparam HTRANS_SEQ    = 2'b11;
 
    // -----------------------------------------------------------------------
    // FSM state encoding (one-hot for reliability in FPGA/ASIC)
    // -----------------------------------------------------------------------
    localparam [7:0] ST_IDLE     = 8'b00000001;
    localparam [7:0] ST_WWAIT    = 8'b00000010;
    localparam [7:0] ST_READ     = 8'b00000100;
    localparam [7:0] ST_WRITE    = 8'b00001000;
    localparam [7:0] ST_WRITEP   = 8'b00010000;
    localparam [7:0] ST_RENABLE  = 8'b00100000;
    localparam [7:0] ST_WENABLE  = 8'b01000000;
    localparam [7:0] ST_WENABLEP = 8'b10000000;
 
    // -----------------------------------------------------------------------
    // Internal registers
    // -----------------------------------------------------------------------
    reg [7:0]             state, next_state;
 
    // AHB address-phase latch registers (captured when HREADYIN=1)
    reg [ADDR_WIDTH-1:0]  addr_reg;     // latched HADDR
    reg                   write_reg;    // latched HWRITE
    reg [NUM_SLAVES-1:0]  psel_reg;     // latched slave select (decoded from HADDR)
 
    // APB output registers
    reg [ADDR_WIDTH-1:0]  paddr_r;
    reg                   pwrite_r;
    reg [NUM_SLAVES-1:0]  psel_r;
    reg                   penable_r;
    reg [DATA_WIDTH-1:0]  pwdata_r;
    reg [DATA_WIDTH-1:0]  hrdata_r;
    reg                   hreadyout_r;
    reg [1:0]             hresp_r;
 
    // -----------------------------------------------------------------------
    // valid: an active AHB transfer is being presented to this slave
    //   - HSEL must be asserted
    //   - HTRANS must be NONSEQ or SEQ (not IDLE/BUSY)
    //   - HREADYIN must be 1 (previous slave completed)
    // -----------------------------------------------------------------------
    wire valid = HSEL & HREADYIN & ((HTRANS == HTRANS_NONSEQ) | (HTRANS == HTRANS_SEQ));
 
    // -----------------------------------------------------------------------
    // APB Slave Address Decode
    //   Each slave occupies a 4KB region for this example.
    //   Modify boundaries to match your memory map.
    //   HADDR[ADDR_WIDTH-1:12] selects the slave.
    // -----------------------------------------------------------------------
    function [NUM_SLAVES-1:0] decode_psel;
        input [ADDR_WIDTH-1:0] addr;
        integer i;
        begin
            decode_psel = {NUM_SLAVES{1'b0}};
            // Example: Slave 0 @ 0x4000_0000, Slave 1 @ 0x4000_1000, etc.
            // Modify these base addresses per your SoC address map
            for (i = 0; i < NUM_SLAVES; i = i + 1) begin
                if (addr[ADDR_WIDTH-1:12] == (32'h4000_0000 >> 12) + i)
                    decode_psel[i] = 1'b1;
            end
        end
    endfunction
 
    // -----------------------------------------------------------------------
    // AHB Address Phase Latch
    //   Capture address, write, and decoded PSEL when HREADYIN=1
    //   (i.e., when the address phase is valid on AHB)
    // -----------------------------------------------------------------------
    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            addr_reg  <= {ADDR_WIDTH{1'b0}};
            write_reg <= 1'b0;
            psel_reg  <= {NUM_SLAVES{1'b0}};
        end else if (HREADYIN & HSEL) begin
            // Latch in address phase; HWDATA comes one cycle later
            addr_reg  <= HADDR;
            write_reg <= HWRITE;
            psel_reg  <= decode_psel(HADDR);
        end
    end
 
    // -----------------------------------------------------------------------
    // FSM: Sequential (state register)
    // -----------------------------------------------------------------------
    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn)
            state <= ST_IDLE;
        else
            state <= next_state;
    end
 
    // -----------------------------------------------------------------------
    // FSM: Combinational (next state logic)
    // -----------------------------------------------------------------------
    always @(*) begin
        next_state = state;  // default: stay
        case (state)
 
            // -----------------------------------------------------------------
            // IDLE: No APB transaction. Wait for valid AHB transfer.
            // -----------------------------------------------------------------
            ST_IDLE: begin
                if (valid & ~HWRITE)
                    next_state = ST_READ;       // read: no data wait needed
                else if (valid & HWRITE)
                    next_state = ST_WWAIT;      // write: must wait for HWDATA
                // else stay IDLE
            end
 
            // -----------------------------------------------------------------
            // WWAIT: Write address latched, waiting for HWDATA to be valid.
            //   HWDATA appears on AHB bus in the cycle AFTER HADDR.
            //   If another transfer is already queued (valid), go to WRITEP.
            // -----------------------------------------------------------------
            ST_WWAIT: begin
                if (~valid)
                    next_state = ST_WRITE;      // no new transfer pending
                else
                    next_state = ST_WRITEP;     // new transfer already on bus
            end
 
            // -----------------------------------------------------------------
            // READ: APB read setup phase. PSEL=1, PENABLE=0.
            //   Always advance to RENABLE on next clock.
            // -----------------------------------------------------------------
            ST_READ: begin
                next_state = ST_RENABLE;
            end
 
            // -----------------------------------------------------------------
            // WRITE: APB write setup, no pending transfer.
            //   Wait for PREADY (if peripheral needs more time).
            // -----------------------------------------------------------------
            ST_WRITE: begin
                if (PREADY)
                    next_state = ST_WENABLE;    // advance to enable phase
                // else stay ST_WRITE (peripheral is stalling) -- optional
                // For simplicity (PREADY always 1), goes straight to WENABLE
            end
 
            // -----------------------------------------------------------------
            // WRITEP: APB write setup, next transfer is pending.
            // -----------------------------------------------------------------
            ST_WRITEP: begin
                if (PREADY)
                    next_state = ST_WENABLEP;
            end
 
            // -----------------------------------------------------------------
            // RENABLE: APB read enable. PENABLE=1, capture PRDATA.
            //   Transition based on what's queued next.
            // -----------------------------------------------------------------
            ST_RENABLE: begin
                if (PREADY) begin
                    if (~valid)
                        next_state = ST_IDLE;
                    else if (valid & ~write_reg)
                        next_state = ST_READ;   // back-to-back read
                    else
                        next_state = ST_WWAIT;  // back-to-back write
                end
            end
 
            // -----------------------------------------------------------------
            // WENABLE: APB write enable, no pending transfer.
            //   Release bus when PREADY, return to IDLE.
            // -----------------------------------------------------------------
            ST_WENABLE: begin
                if (PREADY) begin
                    if (~valid)
                        next_state = ST_IDLE;
                    else if (valid & ~write_reg)
                        next_state = ST_READ;
                    else
                        next_state = ST_WWAIT;
                end
            end
 
            // -----------------------------------------------------------------
            // WENABLEP: APB write enable, next transfer pending.
            //   Immediately start next APB cycle without returning to IDLE.
            // -----------------------------------------------------------------
            ST_WENABLEP: begin
                if (PREADY) begin
                    if (write_reg)
                        next_state = ST_WRITEP; // next is also a write
                    else
                        next_state = ST_READ;   // next is a read
                end
            end
 
            default: next_state = ST_IDLE;
        endcase
    end
 
    // -----------------------------------------------------------------------
    // FSM: Output logic (registered APB outputs and HREADYOUT)
    // -----------------------------------------------------------------------
    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            paddr_r     <= {ADDR_WIDTH{1'b0}};
            pwrite_r    <= 1'b0;
            psel_r      <= {NUM_SLAVES{1'b0}};
            penable_r   <= 1'b0;
            pwdata_r    <= {DATA_WIDTH{1'b0}};
            hrdata_r    <= {DATA_WIDTH{1'b0}};
            hreadyout_r <= 1'b1;   // after reset: AHB bus is free
            hresp_r     <= 2'b00;
        end else begin
            case (next_state)
 
                // -------------------------------------------------------------
                // Going to IDLE: deassert all APB signals, release AHB
                // -------------------------------------------------------------
                ST_IDLE: begin
                    psel_r      <= {NUM_SLAVES{1'b0}};
                    penable_r   <= 1'b0;
                    hreadyout_r <= 1'b1;
                    hresp_r     <= 2'b00;
                end
 
                // -------------------------------------------------------------
                // Going to WWAIT: stall AHB, wait for HWDATA
                // -------------------------------------------------------------
                ST_WWAIT: begin
                    psel_r      <= {NUM_SLAVES{1'b0}};
                    penable_r   <= 1'b0;
                    hreadyout_r <= 1'b0;  // stall AHB master
                    // Latch address/write from the just-captured registers
                    paddr_r     <= addr_reg;
                    pwrite_r    <= write_reg;
                end
 
                // -------------------------------------------------------------
                // Going to READ: drive APB setup, stall AHB
                // -------------------------------------------------------------
                ST_READ: begin
                    psel_r      <= psel_reg;
                    paddr_r     <= addr_reg;
                    pwrite_r    <= 1'b0;
                    penable_r   <= 1'b0;
                    hreadyout_r <= 1'b0;  // stall AHB until read completes
                end
 
                // -------------------------------------------------------------
                // Going to WRITE: HWDATA is now valid, drive APB setup
                // -------------------------------------------------------------
                ST_WRITE: begin
                    psel_r      <= psel_reg;
                    paddr_r     <= addr_reg;
                    pwrite_r    <= 1'b1;
                    pwdata_r    <= HWDATA;    // capture HWDATA (now valid)
                    penable_r   <= 1'b0;
                    hreadyout_r <= 1'b0;
                end
 
                // -------------------------------------------------------------
                // Going to WRITEP: same as WRITE, next transfer pending
                // -------------------------------------------------------------
                ST_WRITEP: begin
                    psel_r      <= psel_reg;
                    paddr_r     <= addr_reg;
                    pwrite_r    <= 1'b1;
                    pwdata_r    <= HWDATA;
                    penable_r   <= 1'b0;
                    hreadyout_r <= 1'b0;
                end
 
                // -------------------------------------------------------------
                // Going to RENABLE: assert PENABLE, read data from APB
                // -------------------------------------------------------------
                ST_RENABLE: begin
                    psel_r      <= psel_r;   // hold PSEL
                    penable_r   <= 1'b1;     // assert PENABLE (enable phase)
                    hreadyout_r <= 1'b0;     // still stalling AHB
                end
 
                // -------------------------------------------------------------
                // Going to WENABLE: assert PENABLE, complete write
                // -------------------------------------------------------------
                ST_WENABLE: begin
                    penable_r   <= 1'b1;
                    hreadyout_r <= 1'b1;  // release AHB: write is completing
                    hresp_r     <= PSLVERR ? 2'b01 : 2'b00;
                end
 
                // -------------------------------------------------------------
                // Going to WENABLEP: assert PENABLE, next transfer pending
                // -------------------------------------------------------------
                ST_WENABLEP: begin
                    penable_r   <= 1'b1;
                    hreadyout_r <= 1'b1;  // release AHB master
                    hresp_r     <= PSLVERR ? 2'b01 : 2'b00;
                end
 
                default: begin
                    psel_r      <= {NUM_SLAVES{1'b0}};
                    penable_r   <= 1'b0;
                    hreadyout_r <= 1'b1;
                end
            endcase
 
            // Capture PRDATA when in RENABLE enable phase and PREADY
            if ((state == ST_RENABLE) && PREADY) begin
                hrdata_r    <= PRDATA;
                hreadyout_r <= 1'b1;  // read complete: release AHB
                hresp_r     <= PSLVERR ? 2'b01 : 2'b00;
            end
        end
    end
 
    // -----------------------------------------------------------------------
    // Output assignments
    // -----------------------------------------------------------------------
    assign PADDR      = paddr_r;
    assign PWRITE     = pwrite_r;
    assign PSEL       = psel_r;
    assign PENABLE    = penable_r;
    assign PWDATA     = pwdata_r;
    assign HRDATA     = hrdata_r;
    assign HREADYOUT  = hreadyout_r;
    assign HRESP      = hresp_r;
 
endmodule
