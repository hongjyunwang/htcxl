// Simple dual-port (1R1W) synchronous SRAM — native FPGA block-RAM shape.
// Independent write port (we/waddr/wdata) and read port (re/raddr/rdata),
// both clocked, so a read and a write may occur in the SAME cycle to
// independent addresses. Registered read => 1-cycle latency.
//
// Read-during-write to the same address in the same cycle returns OLD data
// (read-first), from the NBA ordering below. cxl_table does not depend on
// this (see the correctness note there).
module sram #(
    parameter int DATA_W = 64,
    parameter int DEPTH  = 8
)(
    input  logic                     clk_i,
    // write port
    input  logic                     we_i,
    input  logic [$clog2(DEPTH)-1:0] waddr_i,
    input  logic [DATA_W-1:0]        wdata_i,
    // read port
    input  logic                     re_i,
    input  logic [$clog2(DEPTH)-1:0] raddr_i,
    output logic [DATA_W-1:0]        rdata_o
);
    logic [DATA_W-1:0] mem [DEPTH];

    // Zero-init to preserve "reads as invalid until written" (matches your
    // current behavior; also how FPGA BRAM powers up). Drop if your original
    // sram used a reset instead.
    initial begin
        for (int i = 0; i < DEPTH; i++) mem[i] = '0;
        rdata_o = '0;
    end

    always_ff @(posedge clk_i) if (we_i) mem[waddr_i] <= wdata_i; // write port
    always_ff @(posedge clk_i) if (re_i) rdata_o <= mem[raddr_i]; // read port
endmodule