module top #(
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,
    parameter int NUM_NODES = 4,
    parameter int RELEASE_SET_DEPTH = 16,
    parameter int CXL_TABLE_DEPTH = 32,
    parameter int BUF_DEPTH = 16, // wr_buf FIFO depth
    parameter int MEM_DEPTH = 1024, // stub memory size (words)
    parameter int MEM_LATENCY = 1 // stub read latency in cycles (>=1)
)(
    input logic clk_i,
    input logic rst_i,

    input logic [NUM_NODES-1:0] req_valid_i,
    input logic [1:0] tx_signal_i,
    input logic [ADDR_W-1:0] load_addr_i,
    input logic [RELEASE_SET_DEPTH-1:0] release_is_write_i,
    input logic [RELEASE_SET_DEPTH-1:0] release_is_read_i,
    input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_i,
    input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_i,

    output logic [NUM_NODES-1:0] req_ready_o,

    // Controller-driven completion (TX_ABORT only)
    output logic [NUM_NODES-1:0] ctrl_resp_valid_o,
    output logic [1:0] ctrl_comp_signal_o,

    // wr_buf-driven completion (LOAD only)
    output logic buf_resp_valid_o,
    output logic [NUM_NODES-1:0] buf_resp_worker_o,
    output logic [DATA_W-1:0] buf_resp_rdata_o
);

    // ---- Controller -> Arbiter ----
    logic ctrl_req_valid;
    logic [1:0] ctrl_req_type;
    logic [ADDR_W-1:0] ctrl_addr; // load address (LOAD)
    logic [RELEASE_SET_DEPTH-1:0] ctrl_rel_is_write;
    logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] ctrl_rel_addr;
    logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] ctrl_rel_data;
    logic [NUM_NODES-1:0] ctrl_worker;

    // ---- Arbiter -> Controller ----
    logic arb_busy;
    logic arb_done; // no consumer yet (controller has no done_i)

    // ---- Arbiter -> wr_buf (push side) ----
    logic push_valid;
    logic push_we;
    logic [ADDR_W-1:0] push_addr;
    logic [DATA_W-1:0] push_wdata;
    logic [NUM_NODES-1:0] push_worker;
    logic [1:0] push_req_type;
    logic buf_push_ready;   // wr_buf -> arbiter

    // ---- wr_buf <-> memory pool ----
    logic pop_valid;
    logic pop_we;
    logic [ADDR_W-1:0] pop_addr;
    logic [DATA_W-1:0] pop_wdata;
    logic [NUM_NODES-1:0] pop_worker;
    logic [1:0] pop_req_type;
    logic mem_ready;
    logic [DATA_W-1:0] mem_rdata;

    // ================ Controller ================
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
        .release_is_write_i(release_is_write_i),
        .release_is_read_i(release_is_read_i),
        .release_addr_i(release_addr_i),
        .release_data_i(release_data_i),

        .buffer_full_i(arb_busy),   // downstream job slot busy (was !buf_push_ready)

        .req_ready_o(req_ready_o),
        .resp_valid_o(ctrl_resp_valid_o),
        .comp_signal_o(ctrl_comp_signal_o),

        .mem_req_valid_o(ctrl_req_valid),
        .mem_we_o(),                // unused: arbiter derives we from req_type
        .mem_addr_o(ctrl_addr),
        .release_is_write_o(ctrl_rel_is_write),
        .release_addr_o(ctrl_rel_addr),
        .release_data_o(ctrl_rel_data),
        .mem_worker_o(ctrl_worker),
        .mem_req_type_o(ctrl_req_type)
    );

    // ================ Write Arbiter (serializer between controller and FIFO) ================
    wr_arbiter #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .NUM_NODES(NUM_NODES),
        .RELEASE_SET_DEPTH(RELEASE_SET_DEPTH),
        .DEPTH(BUF_DEPTH) // unused in body, passed for tidiness
    ) arb_inst (
        .clk_i(clk_i),
        .rst_i(rst_i),

        // From controller
        .req_valid_i(ctrl_req_valid),
        .req_type_i(ctrl_req_type),
        .load_addr_i(ctrl_addr),
        .release_addr_i(ctrl_rel_addr),
        .release_is_write_i(ctrl_rel_is_write),
        .release_data_i(ctrl_rel_data),
        .worker_i(ctrl_worker),

        // To controller
        .busy_o(arb_busy),
        .done_o(arb_done),

        // From FIFO
        .push_ready_i(buf_push_ready),

        // To FIFO
        .push_valid_o(push_valid),
        .push_we_o(push_we),
        .push_addr_o(push_addr),
        .push_wdata_o(push_wdata),
        .push_worker_o(push_worker),
        .push_req_type_o(push_req_type)
    );

    // ================ Write Buffer (FIFO between arbiter and memory) ================
    wr_buf #(
        .NUM_NODES(NUM_NODES),
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .DEPTH(BUF_DEPTH)
    ) buf_inst (
        .clk_i(clk_i),
        .rst_i(rst_i),

        // Push side <- arbiter
        .push_valid_i(push_valid),
        .push_we_i(push_we),
        .push_addr_i(push_addr),
        .push_wdata_i(push_wdata),
        .push_worker_i(push_worker),
        .push_req_type_i(push_req_type),
        .push_ready_o(buf_push_ready),

        // Pop side -> memory pool
        .pop_valid_o(pop_valid),
        .pop_we_o(pop_we),
        .pop_addr_o(pop_addr),
        .pop_wdata_o(pop_wdata),
        .pop_worker_o(pop_worker),
        .pop_req_type_o(pop_req_type),

        // From memory pool
        .mem_ready_i(mem_ready),
        .mem_rdata_i(mem_rdata),

        // To nodes
        .resp_valid_o(buf_resp_valid_o),
        .resp_worker_o(buf_resp_worker_o),
        .resp_rdata_o(buf_resp_rdata_o)
    );

    // ================ Memory Pool Stub ================
    mem_pool_stub #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .MEM_DEPTH(MEM_DEPTH),
        .MEM_LATENCY(MEM_LATENCY)
    ) mem (
        .clk_i(clk_i),
        .rst_i(rst_i),
        .mem_req_valid_i(pop_valid),
        .mem_we_i(pop_we),
        .mem_addr_i(pop_addr),
        .mem_wdata_i(pop_wdata),
        .mem_rdata_o(mem_rdata),
        .mem_rvalid_o(mem_ready)
    );

endmodule