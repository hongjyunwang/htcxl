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

    // Release set inputs:
    input logic [RELEASE_SET_DEPTH-1:0] release_valid_i, // release set valid mark
    input logic [RELEASE_SET_DEPTH-1:0] release_is_write_i, // release set write mark
    input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_i, // release set address
    input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_i, // release set data

    // Inputs from CXL memory pool
    input logic mem_rvalid_i,
    input logic [DATA_W-1:0] mem_rdata_i,

    // Outputs to hosts/nodes
    output logic [NUM_NODES-1:0] req_ready_o, // tell the host that CXL controller is ready to accept a new request from specifc nodes
    output logic [NUM_NODES-1:0] resp_valid_o, // handshake to signal completed request
    output logic [1:0] comp_signal_o,
    output logic [DATA_W-1:0] load_data_o,

    // Outputs to CXL memory pool
    output logic mem_req_valid_o,
    output logic mem_we_o,
    output logic [ADDR_W-1:0] mem_addr_o,
    output logic [DATA_W-1:0] mem_wdata_o
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

    // ================ Building CXL Table ================
    logic cxl_table_valid_q [CXL_TABLE_DEPTH];
    logic [ADDR_W-1:0] cxl_table_addr_q [CXL_TABLE_DEPTH];
    logic [NUM_NODES-1:0] cxl_table_checkout_vec_q [CXL_TABLE_DEPTH];
    logic [NUM_NODES-1:0] cxl_table_in_progress_vec_q [CXL_TABLE_DEPTH];
    logic cxl_table_locked_q [CXL_TABLE_DEPTH];

    logic cxl_table_valid_d [CXL_TABLE_DEPTH];
    logic [ADDR_W-1:0] cxl_table_addr_d [CXL_TABLE_DEPTH];
    logic [NUM_NODES-1:0] cxl_table_checkout_vec_d [CXL_TABLE_DEPTH];
    logic [NUM_NODES-1:0] cxl_table_in_progress_vec_d [CXL_TABLE_DEPTH];
    logic cxl_table_locked_d [CXL_TABLE_DEPTH];

    // ================ Pipeline Registers ================
    // IDLE -> MOD
    typedef struct packed {
        logic valid;
        logic [NUM_NODES-1:0] worker;
        logic [1:0] request_type;
        logic [ADDR_W-1:0] load_addr;
        logic [RELEASE_SET_DEPTH-1:0] release_valid;
        logic [RELEASE_SET_DEPTH-1:0] release_is_write;
        logic [RELEASE_SET_DEPTH][ADDR_W-1:0] release_addr;
        logic [RELEASE_SET_DEPTH][DATA_W-1:0] release_data;

        logic [CXL_TABLE_DEPTH-1:0] cxl_release_mask;
        logic cxl_upd_alloc;
        logic [CXL_IDX_W-1:0] cxl_upd_idx;
    } idle_mod_t;
    // MOD -> REQ
    typedef struct packed {
        logic valid;
        logic [NUM_NODES-1:0] worker;
        logic [1:0] request_type;
        logic [ADDR_W-1:0] load_addr;
    } mod_req_t;
    // REQ -> RESP
    typedef struct packed {
        logic valid;
        logic [NUM_NODES-1:0] worker;
        logic [1:0] request_type;
        logic [DATA_W-1:0] resp_data;
    } req_resp_t;
    // q is present, d is next state
    idle_mod_t idle_mod_q, idle_mod_d;
    mod_req_t mod_req_q, mod_req_d;
    req_resp_t req_resp_q, req_resp_d;

    // Set next register state
    always_comb begin
        // Default
        idle_mod_d = idle_mod_q;
        mod_req_d = mod_req_q;
        req_resp_d = req_resp_q;

        // IDLE -> MOD
        idle_mod_d.valid = (req_valid_i != '0);
        idle_mod_d.worker = req_valid_i;
        idle_mod_d.request_type = tx_signal_i;
        idle_mod_d.load_addr = load_addr_i;
        idle_mod_d.release_valid = release_valid_i;
        idle_mod_d.release_is_write = release_is_write_i;
        idle_mod_d.release_addr = release_addr_i;
        idle_mod_d.release_data = release_data_i;

        // MOD -> REQ
        mod_req_d.valid = idle_mod_q.valid;
        mod_req_d.worker = idle_mod_q.worker;
        mod_req_d.request_type = idle_mod_q.request_type;
        mod_req_d.load_addr = idle_mod_q.load_addr;

        // REQ -> RESP
        req_resp_d.valid = mem_rvalid_i && mod_req_q.valid;
        req_resp_d.worker = mod_req_q.worker;
        req_resp_d.request_type = mod_req_q.request_type;
        req_resp_d.resp_data = mem_rdata_i;
    end

    // ================ MOD Stage ================
    // Combinational CAM CXL Table lookup
    wire [RELEASE_SET_DEPTH-1:0] local_release_valid;
    assign local_release_valid = idle_mod_q.release_valid;
    wire [RELEASE_SET_DEPTH-1:0] local_release_addr;
    assign local_release_addr = idle_mod_q.release_valid;
    always_comb begin
        // Default: hold current values
        for (int k = 0; k < CXL_TABLE_DEPTH; k++) begin
            cxl_table_valid_d[k] = cxl_table_valid_q[k];
            cxl_table_addr_d[k] = cxl_table_addr_q[k];
            cxl_table_checkout_vec_d[k] = cxl_table_checkout_vec_q[k];
            cxl_table_in_progress_vec_d[k] = cxl_table_in_progress_vec_q[k];
            cxl_table_locked_d[k] = cxl_table_locked_q[k];
        end

        if (idle_mod_q.valid) begin
            unique case (idle_mod_q.request_type)
                CMD_LOAD: begin
                    // scan cxl table for hit
                    for (int i = 0; i < CXL_TABLE_DEPTH; i++) begin
                        if (cxl_table_valid_q[i] && (cxl_table_addr_q[i] == idle_mod_q.load_addr)) begin
                            // Existing entry: OR in this worker's checkout bit, clear its in_progress bit
                            cxl_table_checkout_vec_d[i] = cxl_table_checkout_vec_q[i] | idle_mod_q.worker;
                            cxl_table_in_progress_vec_d[i] = cxl_table_in_progress_vec_q[i] & ~idle_mod_q.worker;
                        end
                    end
                    // logically no hit, so scan cxl table for first free entry
                    for (int i = CXL_TABLE_DEPTH-1; i >= 0; i--) begin
                        logic alloc_done;
                        alloc_done = 1'b0;
                        if (!alloc_done && !cxl_table_valid_q[i]) begin
                            // New entry allocation
                            cxl_table_valid_d[i] = 1'b1;
                            cxl_table_addr_d[i] = idle_mod_q.load_addr;
                            cxl_table_checkout_vec_d[i] = idle_mod_q.worker;
                            cxl_table_in_progress_vec_d[i] = '0;
                            cxl_table_locked_d[i] = 1'b0;
                            alloc_done = 1'b1;;
                        end
                    end
                    // Need CXL Table full detection
                end

                CMD_TX_ABORT : begin
                    for (int i = 0; i < RELEASE_SET_DEPTH; i++) begin
                        if (local_release_valid[i]) begin
                            for (int j = 0; j < CXL_TABLE_DEPTH; j++) begin
                                if (cxl_table_valid_q[j] && cxl_table_addr_q[j] == local_release_addr[i]) begin
                                    cxl_table_checkout_vec_d[j] = cxl_table_checkout_vec_q[j] & ~idle_mod_q.worker;
                                    // If removing node makes entry invalid, mark invalid
                                    if ((cxl_table_checkout_vec_q[j] & ~idle_mod_q.worker) == '0) begin
                                        cxl_table_valid_d[j] = 1'b0;
                                    end
                                end
                            end
                        end
                    end
                end

                CMD_TX_COMMIT: begin
                    // Placeholder
                end

                default: begin
                    // Placeholder
                end
            endcase
        end 
    end

    // ================ REQ Stage ================
    // Send memory request response
    logic mem_req_sent_q;
    always_comb begin
        // defaults
        mem_req_valid_o = '0;
        mem_we_o = '0;
        mem_addr_o = '0;
        mem_wdata_o = '0;
        unique case (mod_req_q.request_type)
            CMD_LOAD: begin
                mem_req_valid_o = mod_req_q.valid && !mem_req_sent_q;
                mem_we_o = '0;
                mem_addr_o = mod_req_q.load_addr;
                mem_wdata_o = '0;
            end

            CMD_TX_ABORT : begin
                // No action
            end

            CMD_TX_COMMIT: begin
                // Placeholder
            end

            default: begin
                // Placeholder
            end
        endcase
    end

    // ================ RESP Stage ================
    // Drive node response ports 
    always_comb begin
        // default
        resp_valid_o = '0;
        comp_signal_o = '0;
        load_data_o = '0;
        unique case (req_resp_q.request_type)
            CMD_LOAD: begin
                resp_valid_o = req_resp_q.valid ? req_resp_q.worker : '0;
                comp_signal_o = req_resp_q.request_type;
                load_data_o = req_resp_q.resp_data;
            end

            CMD_TX_ABORT : begin
                resp_valid_o = req_resp_q.valid ? req_resp_q.worker : '0;
                comp_signal_o = req_resp_q.request_type;
            end

            CMD_TX_COMMIT: begin
                // Placeholder
            end

            default: begin
                // Placeholder
            end
        endcase
    end

    // Sequential logic
    logic ctrl_ready_q;
    assign req_ready_o = {NUM_NODES{ctrl_ready_q}};
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            ctrl_ready_q <= 1'b1;
            mem_req_sent_q <= 1'b0;
            idle_mod_q <= '0;
            mod_req_q <= '0;
            req_resp_q <= '0;
            for (int k = 0; k < CXL_TABLE_DEPTH; k++) begin
                cxl_table_valid_q[k] <= 1'b0;
                cxl_table_addr_q[k] <= '0;
                cxl_table_checkout_vec_q[k] <= '0;
                cxl_table_in_progress_vec_q[k] <= '0;
                cxl_table_locked_q[k] <= 1'b0;
            end
        end else begin
            ctrl_ready_q <= mem_rvalid_i || !mod_req_q.valid;
            if (mem_rvalid_i || !mod_req_q.valid) begin
                // pipeline is free to advance
                mem_req_sent_q <= 1'b0;
                idle_mod_q <= idle_mod_d;
                mod_req_q <= mod_req_d;
                req_resp_q <= req_resp_d;
                for (int k = 0; k < CXL_TABLE_DEPTH; k++) begin
                    cxl_table_valid_q[k] <= cxl_table_valid_d[k];
                    cxl_table_addr_q[k] <= cxl_table_addr_d[k];
                    cxl_table_checkout_vec_q[k] <= cxl_table_checkout_vec_d[k];
                    cxl_table_in_progress_vec_q[k] <= cxl_table_in_progress_vec_d[k];
                    cxl_table_locked_q[k] <= cxl_table_locked_d[k];
                end
            end else begin
                // stalled, mod_req_q holds its value; mark that we've now sent
                // the request for it (so we don't re-assert mem_req_valid_o)
                mem_req_sent_q <= 1'b1;
            end
        end
    end

    // Debug: CXL Controller State
    always @(negedge clk_i) begin
        if (!rst_i) begin
            if (idle_mod_q.valid)
                $display("[%0t] CXL CONTROLLER [IDLE→MOD] node=%b cmd=%0d addr=%0d", $time, idle_mod_q.worker, idle_mod_q.request_type, idle_mod_q.load_addr);

            if (mod_req_q.valid)
                $display("[%0t] CXL CONTROLLER [MOD→REQ]  node=%b addr=%0d", $time, mod_req_q.worker, mod_req_q.load_addr);

            if (req_resp_q.valid && req_resp_q.worker != 0)
                $display("[%0t] CXL CONTROLLER [REQ→RESP] node=%b data=%h", $time, req_resp_q.worker, req_resp_q.resp_data);
        end
    end

    // always @(negedge clk_i) begin
    //     if (!rst_i) begin
    //         $display("[%0t] DEBUG: idle_lkp_q.valid=%b local_hit=%b local_has_free=%b gate=%b mem_rvalid_i=%b mod_req_q.valid=%b",
    //             $time,
    //             idle_lkp_q.valid,
    //             local_hit,
    //             local_has_free,
    //             (mem_rvalid_i || !mod_req_q.valid),
    //             mem_rvalid_i,
    //             mod_req_q.valid);

    //         $display("[%0t] CXL TABLE STATE:", $time);
    //         for (int i = 0; i < CXL_TABLE_DEPTH; i++) begin
    //             if (cxl_table_valid_q[i]) begin
    //                 $display("  entry[%0d]: addr=%0d checkout=%b in_progress=%b locked=%b",
    //                     i,
    //                     cxl_table_addr_q[i],
    //                     cxl_table_checkout_vec_q[i],
    //                     cxl_table_in_progress_vec_q[i],
    //                     cxl_table_locked_q[i]);
    //             end
    //         end
    //     end
    // end

    // always @(negedge clk_i) begin
    //     if (!rst_i) begin
    //         $display("[%0t] DEBUG: gate=%b | mem_rvalid_i=%b mem_rdata_i=%h | mod_req_q.valid=%b mod_req_q.worker=%b mod_req_q.addr=%0d | ctrl_ready_q=%b",
    //             $time,
    //             (mem_rvalid_i || !mod_req_q.valid),
    //             mem_rvalid_i,
    //             mem_rdata_i,
    //             mod_req_q.valid,
    //             mod_req_q.worker,
    //             mod_req_q.load_addr,
    //             ctrl_ready_q);
    //     end
    // end

endmodule

