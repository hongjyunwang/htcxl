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
    input logic req_valid_i, // handshake signal
    input logic [1:0] tx_signal_i, // 00: CMD_LOAD, 01: CMD_TX_ABORT, 10: CMD_TX_COMMIT
    input logic [NUM_NODES-1:0] node_id_i, // one hot bit encoding node id
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
    output logic req_ready_o, // tell the host that CXL controller is ready to accept a new request
    output logic resp_valid_o,
    output logic [1:0] comp_signal_o,
    output logic [DATA_W-1:0] load_data_o,

    // Outputs to CXL memory pool
    output logic mem_req_valid_o,
    output logic mem_we_o,
    output logic [ADDR_W-1:0] mem_addr_o,
    output logic [DATA_W-1:0] mem_wdata_o
);

    parameter int NUM_WORKERS = 2;

    // Command encoding
    typedef enum logic [1:0] {
        CMD_LOAD = 2'b00,
        CMD_TX_ABORT  = 2'b01,
        CMD_TX_COMMIT = 2'b10
    } cxl_cmd_t;
    cxl_cmd_t cmd;
    assign cmd = cxl_cmd_t'(tx_signal_i);

    // Completion signal encoding
    typedef enum logic [1:0] {
        COMP_NONE = 2'b00,
        COMP_LOAD_DONE = 2'b01,
        COMP_ABORT = 2'b10,
        COMP_COMMIT = 2'b11
    } cxl_comp_t;

    // ================ Building Release Set ================
    typedef struct packed {
        logic valid;
        logic is_write;
        logic [ADDR_W-1:0] addr;
        logic [DATA_W-1:0] data;
    } release_entry_t;
    release_entry_t [RELEASE_SET_DEPTH-1:0] release_set; // PACKED, index-addressable
    // Fill in the set from the input
    always_comb begin
        release_entry_t rs_tmp;
        for (int i = 0; i < RELEASE_SET_DEPTH; i++) begin
            rs_tmp.valid = release_valid_i[i];
            rs_tmp.is_write = release_is_write_i[i];
            rs_tmp.addr = release_addr_i[i];
            rs_tmp.data = release_data_i[i];
            release_set[i] = rs_tmp;       // whole-element write
        end
    end

    // ================ Building CXL Table ================
    logic cxl_table_valid [CXL_TABLE_DEPTH];
    logic [ADDR_W-1:0] cxl_table_addr [CXL_TABLE_DEPTH];
    logic [NUM_NODES-1:0] cxl_table_checkout_vec [CXL_TABLE_DEPTH];
    logic [NUM_NODES-1:0] cxl_table_in_progress_vec [CXL_TABLE_DEPTH];
    logic cxl_table_locked [CXL_TABLE_DEPTH];


    // ================ Per-Worker CAM Lookup ================
    // Each worker independently queries the shared CXL Table.
    // Reads are non-conflicting (purely combinational); only writes (in always_ff) need arbitration via the locked field.
    localparam int CXL_IDX_W = $clog2(CXL_TABLE_DEPTH);
    localparam int WORKER_IDX_W = $clog2(NUM_WORKERS);

    logic [ADDR_W-1:0] cam_query_addr [NUM_WORKERS];
    logic [CXL_TABLE_DEPTH-1:0] cam_match_vec [NUM_WORKERS];
    logic [CXL_TABLE_DEPTH-1:0] cam_free_vec [NUM_WORKERS];
    logic cam_hit [NUM_WORKERS];
    logic cam_has_free [NUM_WORKERS];
    logic [CXL_IDX_W-1:0] cam_hit_idx [NUM_WORKERS]; // index in cxl table
    logic [CXL_IDX_W-1:0] cam_free_idx [NUM_WORKERS]; // index in cxl table

    // // Combinational CAM lookup
    // always_comb begin
    //     for (int w = 0; w < NUM_WORKERS; w++) begin
    //         // Defaults
    //         cam_hit_idx[w] = '0;
    //         cam_free_idx[w] = '0;

    //         // Parallel comparators
    //         for (int i = 0; i < CXL_TABLE_DEPTH; i++) begin
    //             cam_match_vec[w][i] = cxl_table_valid[i] && (cxl_table_addr[i] == cam_query_addr[w]);
    //             cam_free_vec[w][i]  = !cxl_table_valid[i];
    //         end

    //         cam_hit[w] = |cam_match_vec[w];
    //         cam_has_free[w] = |cam_free_vec[w];

    //         // Priority encoder for hit_idx
    //         for (int i = 0; i < CXL_TABLE_DEPTH; i++) begin
    //             if (cam_match_vec[w][i]) cam_hit_idx[w] = i; // extract bits
    //         end

    //         // Priority encoder for free_idx
    //         for (int i = CXL_TABLE_DEPTH-1; i >= 0; i--) begin
    //             if (cam_free_vec[w][i]) cam_free_idx[w] = i; // extract bits
    //         end
    //     end
    // end

    // ================ Concurrent Workers ================
    typedef enum logic [2:0] {
        W_IDLE,
        W_CXL_QUERY,

        W_MEM_REQ,
        W_MEM_WAIT,
        W_RESPOND,

        W_DONE
    } worker_state_t;
    // These four arrays make up the workers (index-addressable)
    worker_state_t worker_state [NUM_WORKERS];
    worker_state_t worker_next_state [NUM_WORKERS];
    cxl_cmd_t worker_cmd [NUM_WORKERS];
    logic [NUM_NODES-1:0] worker_node_id [NUM_WORKERS];
    logic [ADDR_W-1:0] worker_load_addr [NUM_WORKERS];
    release_entry_t worker_release_set [NUM_WORKERS][RELEASE_SET_DEPTH];

    // Mark idle workers and dispatch worker
    logic [NUM_WORKERS-1:0] worker_idle;
    logic dispatch_valid;
    logic [$clog2(NUM_WORKERS)-1:0] dispatch_idx; // marks which worker was dispatched
    always_comb begin

        // Mark idle workers
        for (int i = 0; i < NUM_WORKERS; i++) begin
            worker_idle[i] = (worker_state[i] == W_IDLE);
        end

        // mark first available worker as available (dispatched)
        dispatch_valid = 1'b0;
        dispatch_idx = '0;
        for (int i = 0; i < NUM_WORKERS; i++) begin
            if (!dispatch_valid && worker_idle[i]) begin
                dispatch_valid = 1'b1;
                dispatch_idx = i;
            end
        end
    end
    assign req_ready_o = dispatch_valid;

    // Drive CAM query inputs from each worker's state
    logic cxl_upd_en [NUM_WORKERS];
    logic [$clog2(CXL_TABLE_DEPTH)-1:0] cxl_upd_idx [NUM_WORKERS];
    logic cxl_upd_alloc [NUM_WORKERS]; // 1 = new slot, 0 = existing slot

    // internal arrays for cam lookup
    logic local_hit [NUM_WORKERS];
    logic local_has_free [NUM_WORKERS];
    logic [CXL_IDX_W-1:0] local_hit_idx [NUM_WORKERS];
    logic [CXL_IDX_W-1:0] local_free_idx [NUM_WORKERS];
    always_comb begin
        for (int w = 0; w < NUM_WORKERS; w++) begin
            // Defaults
            cam_query_addr[w] = '0;
            cxl_upd_en[w] = 1'b0;
            cxl_upd_alloc[w] = 1'b0;
            cxl_upd_idx[w] = '0;
            worker_next_state[w] = worker_state[w]; // default to remaining in same state

            local_hit[w] = 1'b0;
            local_has_free[w] = 1'b0;
            local_hit_idx[w] = '0;
            local_free_idx[w] = '0;

            case (worker_state[w])

                W_IDLE: begin
                    // $display("[%0t] CXL_CONTROLLER: worker[%0d] in W_IDLE state", $time, w);
                    // Stay idle until dispatched by sequential logic
                    if (req_valid_i && req_ready_o && (w == dispatch_idx)) begin
                        worker_next_state[w] = W_CXL_QUERY;
                    end else begin
                        worker_next_state[w] = W_IDLE;
                    end
                end

                W_CXL_QUERY: begin
                    // $display("[%0t] CXL_CONTROLLER: worker[%0d] in W_CXL_QUERY state", $time, w);
                    unique case (worker_cmd[w])
                        CMD_LOAD: begin
                            // scan cxl table for hit
                            for (int i = 0; i < CXL_TABLE_DEPTH; i++) begin
                                if (cxl_table_valid[i] && (cxl_table_addr[i] == worker_load_addr[w])) begin
                                    local_hit[w] = 1'b1;
                                    local_hit_idx[w] = i;
                                end
                            end
                            // scan cxl table for free entries
                            for (int i = CXL_TABLE_DEPTH-1; i >= 0; i--) begin
                                if (!cxl_table_valid[i]) begin
                                    local_has_free[w] = 1'b1;
                                    local_free_idx[w] = i;
                                end
                            end
                            
                            if (local_hit[w]) begin
                                cxl_upd_en[w] = 1'b1;
                                cxl_upd_alloc[w] = 1'b0;
                                cxl_upd_idx[w] = local_hit_idx[w];
                                worker_next_state[w] = W_MEM_REQ;
                            end else if (local_has_free[w]) begin
                                cxl_upd_en[w] = 1'b1;
                                cxl_upd_alloc[w] = 1'b1;
                                cxl_upd_idx[w] = local_free_idx[w];
                                worker_next_state[w] = W_MEM_REQ;
                            end else begin
                                worker_next_state[w] = W_CXL_QUERY;
                            end
                        end

                        CMD_TX_ABORT, CMD_TX_COMMIT: begin
                            worker_next_state[w] = W_DONE; // placeholder
                        end

                        default: begin
                            worker_next_state[w] = W_IDLE;
                        end
                    endcase
                end

                W_MEM_REQ: begin
                    // Placeholder for now
                    // $display("[%0t] CXL_CONTROLLER: worker[%0d] in W_MEM_REQ state", $time, w);
                    worker_next_state[w] = W_MEM_REQ;
                end

                W_MEM_WAIT: begin
                    worker_next_state[w] = W_MEM_WAIT;
                end

                W_RESPOND: begin
                    worker_next_state[w] = W_RESPOND;
                end

                W_DONE: begin
                    worker_next_state[w] = W_IDLE;
                end

                default: begin
                    worker_next_state[w] = W_IDLE;
                end

            endcase
        end
    end


    // Sequential Logic
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            for (int i = 0; i < NUM_WORKERS; i++) begin
                worker_state[i] <= W_IDLE;
                worker_cmd[i] <= CMD_LOAD;
                worker_node_id[i] <= '0;
                worker_load_addr[i] <= '0;
            end
            for (int e = 0; e < CXL_TABLE_DEPTH; e++) begin
                cxl_table_valid[e] <= 1'b0;
                cxl_table_addr[e] <= '0;
                cxl_table_checkout_vec[e] <= '0;
                cxl_table_in_progress_vec[e] <= '0;
                cxl_table_locked[e] <= 1'b0;
            end
        end else begin
            // Assign work to the dispatched worker
            if (req_valid_i && req_ready_o) begin
                worker_cmd[dispatch_idx] <= cmd;
                worker_node_id[dispatch_idx] <= node_id_i;
                worker_load_addr[dispatch_idx] <= load_addr_i;
                for (int i = 0; i < RELEASE_SET_DEPTH; i++) begin
                    worker_release_set[dispatch_idx][i] <= release_set[i];
                end
            end

            for (int w = 0; w < NUM_WORKERS; w++) begin
                // cxl table update
                if (cxl_upd_en[w]) begin
                    if (cxl_upd_alloc[w]) begin
                        cxl_table_valid[cxl_upd_idx[w]] <= 1'b1;
                        cxl_table_addr[cxl_upd_idx[w]] <= worker_load_addr[w];
                        cxl_table_checkout_vec[cxl_upd_idx[w]] <= worker_node_id[w];
                        cxl_table_in_progress_vec[cxl_upd_idx[w]] <= '0;
                        cxl_table_locked[cxl_upd_idx[w]] <= 1'b0;
                    end else begin
                        cxl_table_checkout_vec[cxl_upd_idx[w]] <=
                            cxl_table_checkout_vec[cxl_upd_idx[w]] | worker_node_id[w];
                    end
                end
                
                // advance state machine
                if (worker_state[w] !== worker_next_state[w]) begin
                    $display("[%0t] STATE CHANGE worker[%0d]: %0d -> %0d", $time, w, worker_state[w], worker_next_state[w]);
                end
                worker_state[w] <= worker_next_state[w];
            end
        end
    end

endmodule