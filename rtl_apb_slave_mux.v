// =============================================================================
//  APB Slave Mux
//  Selects PRDATA from the correct slave based on PSEL
//  Place at top level between bridge and peripherals
// =============================================================================
module apb_slave_mux #(
    parameter DATA_WIDTH = 32,
    parameter NUM_SLAVES = 3
)(
    input  wire [NUM_SLAVES-1:0]              PSEL,
    input  wire [NUM_SLAVES*DATA_WIDTH-1:0]   PRDATA_ALL,  // packed: {slave2, slave1, slave0}
    input  wire [NUM_SLAVES-1:0]              PREADY_ALL,
    input  wire [NUM_SLAVES-1:0]              PSLVERR_ALL,
    output reg  [DATA_WIDTH-1:0]              PRDATA,
    output reg                                PREADY,
    output reg                                PSLVERR
);
    integer i;
    always @(*) begin
        PRDATA  = {DATA_WIDTH{1'b0}};
        PREADY  = 1'b1;
        PSLVERR = 1'b0;
        for (i = 0; i < NUM_SLAVES; i = i + 1) begin
            if (PSEL[i]) begin
                PRDATA  = PRDATA_ALL[i*DATA_WIDTH +: DATA_WIDTH];
                PREADY  = PREADY_ALL[i];
                PSLVERR = PSLVERR_ALL[i];
            end
        end
    end
endmodule
