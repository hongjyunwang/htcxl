module top #(
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,
    parameter int NUM_NODES = 4,
    parameter int RELEASE_SET_DEPTH = 16,
    parameter int CXL_TABLE_DEPTH = 32,

    parameter int MEM_DEPTH = 1024, // stub memory size (words)
    parameter int MEM_LATENCY = 1 // stub read latency in cycles (>=1)
)(
    input logic clk_i,
    input logic rst_i,

    input logic [NUM_NODES-1:0] req_valid_i,
    input logic [1:0] tx_signal_i,
    input logic [ADDR_W-1:0] load_addr_i,
    input logic [RELEASE_SET_DEPTH-1:0] release_valid_i,
    input logic [RELEASE_SET_DEPTH-1:0] release_is_write_i,
    input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_i,
    input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_i,

    output logic [NUM_NODES-1:0] req_ready_o,
    output logic [NUM_NODES-1:0] resp_valid_o,
    output logic [1:0] comp_signal_o,
    output logic [DATA_W-1:0] load_data_o
);

    // Internal Wires
    logic mem_req_valid;
    logic mem_we;
    logic [ADDR_W-1:0] mem_addr;
    logic [DATA_W-1:0] mem_wdata;
    logic [DATA_W-1:0] mem_rdata;
    logic mem_rvalid;

    // DUT
    cxl_controller #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .NUM_NODES(NUM_NODES),
        .RELEASE_SET_DEPTH(RELEASE_SET_DEPTH),
        .CXL_TABLE_DEPTH(CXL_TABLE_DEPTH)
    ) dut (
        .clk_i(clk_i),
        .rst_i(rst_i),

        .req_valid_i(req_valid_i),
        .tx_signal_i(tx_signal_i),
        .load_addr_i(load_addr_i),
        .release_valid_i(release_valid_i),
        .release_is_write_i(release_is_write_i),
        .release_addr_i(release_addr_i),
        .release_data_i(release_data_i),

        .mem_rvalid_i(mem_rvalid),
        .mem_rdata_i(mem_rdata),

        .req_ready_o(req_ready_o),
        .resp_valid_o(resp_valid_o),
        .comp_signal_o(comp_signal_o),
        .load_data_o(load_data_o),

        .mem_req_valid_o(mem_req_valid),
        .mem_we_o(mem_we),
        .mem_addr_o(mem_addr),
        .mem_wdata_o(mem_wdata)
    );

    // memory pool
    mem_pool_stub #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .MEM_DEPTH(MEM_DEPTH),
        .MEM_LATENCY(MEM_LATENCY)
    ) mem (
        .clk_i(clk_i),
        .rst_i(rst_i),
        .mem_req_valid_i(mem_req_valid),
        .mem_we_i(mem_we),
        .mem_addr_i(mem_addr),
        .mem_wdata_i(mem_wdata),
        .mem_rdata_o(mem_rdata),
        .mem_rvalid_o(mem_rvalid)
    );

endmodule