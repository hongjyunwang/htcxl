// Basic SRAM module
// Each SRAM holds one way across all sets

module sram  #(
    parameter int DATA_W = 64, // 1 + TAG_W + NUM_NODES + NUM_NODES, set by wrapper
    parameter int DEPTH = 8 // depth = number of sets
)(
    input logic clk_i,

    // Write data input
    input logic ce_i,
    input logic we_i,
    input logic [$clog2(DEPTH)-1:0] idx_i,
    input logic [DATA_W-1:0] wdata_i,
    
    // Read data output 
    output logic [DATA_W-1:0] rdata_o
);

    logic [DATA_W-1:0] mem [0:DEPTH-1];

    // Initialize
    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1)
            mem[i] = '0;
    end

    // rw takes one cycle each
    always_ff @(posedge clk_i) begin
        if (ce_i) begin
            if (we_i)
                mem[idx_i] <= wdata_i;
            else
                rdata_o <= mem[idx_i];
        end
    end

endmodule
