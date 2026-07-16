`timescale 1ns/1ps

module cxl_controller #(
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,

    parameter int NUM_NODES = 4,
    parameter int RELEASE_SET_DEPTH = 16, // max 16 variables per transaction
    parameter int CXL_TABLE_DEPTH = 32 // max 32 entires in cxl table
)(
    input logic clk_i,
    input logic rst_i,

    // Input nodes
    input logic [NUM_NODES-1:0] req_valid_i, // handshake signal specific for each node, also encodes the requesting node's id
    input logic [1:0] tx_signal_i, // 00: CMD_LOAD, 01: CMD_TX_ABORT, 10: CMD_TX_COMMIT
    input logic [ADDR_W-1:0] load_addr_i, // Used for CMD_LOAD

    // Release set inputs
    input logic [RELEASE_SET_DEPTH-1:0] release_is_write_i, // release set write mark
    input logic [RELEASE_SET_DEPTH-1:0] release_is_read_i, // release set read mark
    input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_i, // release set address
    input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_i, // release set data

    // Inputs from CXL memory pool (buffer)
    input logic buffer_full_i,

    // Outputs to hosts/nodes
    output logic [NUM_NODES-1:0] req_ready_o, // tell the host that CXL controller is ready to accept a new request from specifc nodes
    output logic [NUM_NODES-1:0] resp_valid_o, // handshake to signal completed request (only Tx_abort uses this)
    output logic [1:0] comp_signal_o, // type of request completed (only Tx_abort or abort in tx_commit uses this)

    // Outputs to CXL memory pool (engine)
    output logic mem_req_valid_o,
    output logic mem_we_o,
    output logic [ADDR_W-1:0] mem_addr_o,
    // release set stuff
    output logic [RELEASE_SET_DEPTH-1:0] release_is_write_o, // release set write mark
    output logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_o, // release set address
    output logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_o, // release set data
    output logic [NUM_NODES-1:0] mem_worker_o,
    output logic [1:0] mem_req_type_o
);
    localparam int NUM_WORKERS = NUM_NODES;
    localparam int CXL_IDX_W = $clog2(CXL_TABLE_DEPTH);

    // Command encoding
    typedef enum logic [1:0] {
        CMD_LOAD = 2'b00,
        CMD_TX_ABORT  = 2'b01,
        CMD_TX_COMMIT = 2'b10
    } cxl_cmd_t;
    cxl_cmd_t cmd;
    assign cmd = cxl_cmd_t'(tx_signal_i);

    // Completion signal encoding
    parameter logic [1:0] COMP_NONE = 2'b00;
    parameter logic [1:0] COMP_LOAD = 2'b01;
    parameter logic [1:0] COMP_ABORT = 2'b10;
    parameter logic [1:0] COMP_COMMIT = 2'b11;

    // ================ Pipeline Registers ================
    // IDLE -> MOD
    typedef struct packed {
        logic valid;
        logic [NUM_NODES-1:0] worker;
        logic [1:0] request_type;
        logic [ADDR_W-1:0] load_addr;
        logic [RELEASE_SET_DEPTH-1:0] release_is_write;
        logic [RELEASE_SET_DEPTH-1:0] release_is_read;
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data;
    } idle_mod_t;
    // MOD -> REQ
    typedef struct packed {
        logic valid;
        logic [NUM_NODES-1:0] worker;
        logic [1:0] request_type;
        logic [ADDR_W-1:0] load_addr;
        logic [RELEASE_SET_DEPTH-1:0] release_is_write;
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data;
        logic any_conflict;
    } mod_req_t;
    idle_mod_t idle_mod_q, idle_mod_d;
    mod_req_t mod_req_q, mod_req_d;

    // ================ Building CXL Table ================
    logic cxl_hit;
    logic [NUM_NODES-1:0] cxl_checkout;
    logic cxl_busy;
    logic cxl_req_compl;
    logic any_conflict;

    cxl_table #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .NUM_NODES(NUM_NODES),
        .RELEASE_SET_DEPTH(RELEASE_SET_DEPTH),
        .CXL_TABLE_DEPTH(CXL_TABLE_DEPTH)
    ) u_cxl_table (
        .clk_i(clk_i),
        .rst_i(rst_i),
        .controller_valid_i(idle_mod_q.valid && !cxl_busy), // valid cxl update request
        .req_type_i(idle_mod_q.request_type),
        .req_node_i(idle_mod_q.worker),
        .addr_i(idle_mod_q.load_addr),
        .release_is_write_i(idle_mod_q.release_is_write),
        .release_is_read_i(idle_mod_q.release_is_read),
        .release_addr_i(idle_mod_q.release_addr),
        .release_data_i(idle_mod_q.release_data),

        .hit_o(cxl_hit),
        .check_out_o(cxl_checkout),
        .busy_o(cxl_busy),
        .req_compl_o(cxl_req_compl),
        .any_conflict_o(any_conflict)
    );

    // Set next register state
    always_comb begin
        // Default
        idle_mod_d = idle_mod_q;
        mod_req_d = mod_req_q;

        // IDLE -> MOD
        idle_mod_d.valid = (req_valid_i != '0);
        idle_mod_d.worker = req_valid_i;
        idle_mod_d.request_type = tx_signal_i;
        idle_mod_d.load_addr = load_addr_i;
        idle_mod_d.release_is_write = release_is_write_i;
        idle_mod_d.release_is_read = release_is_read_i;
        idle_mod_d.release_addr = release_addr_i;
        idle_mod_d.release_data = release_data_i;

        // MOD -> REQ
        mod_req_d.valid = idle_mod_q.valid;
        mod_req_d.worker = idle_mod_q.worker;
        mod_req_d.release_is_write = idle_mod_q.release_is_write;
        mod_req_d.release_addr = idle_mod_q.release_addr;
        mod_req_d.release_data = idle_mod_q.release_data;
        mod_req_d.request_type = idle_mod_q.request_type;
        mod_req_d.load_addr = idle_mod_q.load_addr;
        mod_req_d.any_conflict = any_conflict; // latch onto conflict detection from cxl table
    end

    // ================ REQ Stage ================
    // Push memory request into wr_buf — fire and forget, no waiting on completion
    logic req_stall; // global stall signal, now purely buffer-driven
    always_comb begin
        // defaults
        mem_req_valid_o = '0;
        mem_we_o = '0;
        mem_addr_o = '0;
        mem_worker_o = '0;
        mem_req_type_o = '0;
        resp_valid_o = '0;
        comp_signal_o = '0;

        release_is_write_o = '0;
        release_addr_o = '0;
        release_data_o = '0;

        mem_worker_o = '0;
        mem_req_type_o = '0;

        unique case (mod_req_q.request_type)
            CMD_LOAD: begin
                mem_req_valid_o = mod_req_q.valid;
                mem_we_o = '0;
                mem_addr_o = mod_req_q.load_addr;
                release_is_write_o = '0;
                release_addr_o = '0;
                release_data_o = '0;
                mem_worker_o = mod_req_q.worker;
                mem_req_type_o = mod_req_q.request_type;
                // LOAD completion no longer goes through this pipeline
                // wr_buf signals resp_valid_o/resp_worker_o/resp_rdata_o directly to nodes
            end

            CMD_TX_ABORT : begin
                // Completes within the controller itself (no memory request)
                resp_valid_o = mod_req_q.valid ? mod_req_q.worker : '0;
                comp_signal_o = COMP_ABORT;
            end

            CMD_TX_COMMIT: begin
            if (mod_req_q.any_conflict) begin
                // conflict -> abort, suppress the write entirely
                resp_valid_o = mod_req_q.valid ? mod_req_q.worker : '0;
                comp_signal_o = COMP_ABORT;
                mem_req_valid_o = 1'b0; // suppress write
            end else begin
                // clean commit -> ship the write set
                mem_req_valid_o = mod_req_q.valid;
                mem_we_o = 1'b1;
                mem_addr_o = '0;
                release_is_write_o = mod_req_q.release_is_write;
                release_addr_o = mod_req_q.release_addr;
                release_data_o = mod_req_q.release_data;
                mem_worker_o = mod_req_q.worker;
                mem_req_type_o = mod_req_q.request_type;
                // optional: signal COMP_COMMIT here if nodes consume commit acks
            end
        end

            default: begin
                // Placeholder
            end
        endcase

        // Stall purely on the buffer being full or cxl table processing
        req_stall = (mod_req_q.valid && mod_req_q.request_type == CMD_LOAD && buffer_full_i) || cxl_busy;

    end

    // Sequential logic
    logic ctrl_ready_q;
    assign req_ready_o = {NUM_NODES{ctrl_ready_q}};
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            ctrl_ready_q <= 1'b1;
            idle_mod_q <= '0;
            mod_req_q <= '0;
        end else begin
            // Accept new request only when truly idle (no request in flight)
            // This also advances the pipeline into MOD stage
            if (!idle_mod_q.valid && idle_mod_d.valid) begin
                idle_mod_q <= idle_mod_d;
                ctrl_ready_q <= 1'b0; // close the gate immediately
            end

            // When cxl_table completes, advance to REQ stage and re-open gate
            if (cxl_req_compl) begin
                mod_req_q <= mod_req_d; // mod_req_q only becomes populated one cycle after cxl_req_compl fired
                idle_mod_q <= '0; // reset idle_mod registers
                ctrl_ready_q <= 1'b1; // ready to accept next request
            end

            // Clear mod_req_q one cycle after it fires
            if (mod_req_q.valid)
                mod_req_q <= '0;
        end
    end

endmodule
