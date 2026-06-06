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
    output logic [NUM_NODES-1:0] req_ready_o, // tell the host that CXL controller is ready to accept a new request
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
    parameter logic [1:0] COMP_LOAD_DONE = 2'b01;
    parameter logic [1:0] COMP_ABORT = 2'b10;
    parameter logic [1:0] COMP_COMMIT = 2'b11;

    // ================ Building CXL Table ================
    // not bus
    logic cxl_table_valid [CXL_TABLE_DEPTH];
    logic [ADDR_W-1:0] cxl_table_addr [CXL_TABLE_DEPTH];
    logic [NUM_NODES-1:0] cxl_table_checkout_vec [CXL_TABLE_DEPTH];
    logic [NUM_NODES-1:0] cxl_table_in_progress_vec [CXL_TABLE_DEPTH];
    logic cxl_table_locked [CXL_TABLE_DEPTH];

    // ================ Per-Worker CAM Lookup ================
    // Each worker independently queries the shared CXL Table.
    // Reads are non-conflicting (purely combinational); only writes (in always_ff) need arbitration via the locked field.
    localparam int CXL_IDX_W = $clog2(CXL_TABLE_DEPTH);

    // state of each worker
    typedef enum logic [2:0] {
        W_IDLE,
        W_CXL_QUERY,
        W_MEM_REQ,
        W_MEM_WAIT,
        W_RESPOND,

        W_ABORT_CXL_UPDATE
    } worker_state_t;

    // These four arrays make up the workers (index-addressable)
    worker_state_t worker_state [NUM_WORKERS];
    worker_state_t worker_next_state [NUM_WORKERS];
    cxl_cmd_t worker_cmd [NUM_WORKERS];
    logic [NUM_NODES-1:0] worker_node_id [NUM_WORKERS];
    logic [ADDR_W-1:0] worker_load_addr [NUM_WORKERS];
    // Release set latches
    logic [RELEASE_SET_DEPTH-1:0] worker_release_valid [NUM_WORKERS];
    logic [RELEASE_SET_DEPTH-1:0] worker_release_is_write [NUM_WORKERS];
    logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] worker_release_addr [NUM_WORKERS];
    logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] worker_release_data [NUM_WORKERS];

    // Accept request from a node when its assigned worker is idle
    genvar i;
    generate
        for (i = 0; i < NUM_NODES; i++) begin : gen_req_ready
            assign req_ready_o[i] = (worker_state[i] == W_IDLE);
        end
    endgenerate

    // Convert req_valid_i to the index needed to access the requesting node's corresponding worker
    logic [$clog2(NUM_NODES)-1:0] active_worker_idx;
    always_comb begin
        active_worker_idx = '0;
        for (int i = 0; i < NUM_NODES; i++) begin
            if (req_valid_i[i]) begin
                active_worker_idx = ($clog2(NUM_NODES))'(i);
            end
        end
    end

    // Drive CAM query inputs from each worker's state
    logic cxl_upd_en [NUM_WORKERS]; // signal cxl table update
    logic [$clog2(CXL_TABLE_DEPTH)-1:0] cxl_upd_idx [NUM_WORKERS]; // CXL Table entry(s) index to update
    logic cxl_upd_alloc [NUM_WORKERS]; // whether to allocate new table entry

    // Registered versions of cxl_upd_idx and cxl_upd_alloc, latched on W_CXL_QUERY -> W_MEM_REQ transition
    logic [CXL_IDX_W-1:0] worker_cxl_upd_idx [NUM_WORKERS];
    logic worker_cxl_upd_alloc [NUM_WORKERS];

    // internal arrays for cam lookup for individual address (LOAD)
    logic local_hit [NUM_WORKERS];
    logic local_has_free [NUM_WORKERS];
    logic [CXL_IDX_W-1:0] local_hit_idx [NUM_WORKERS];
    logic [CXL_IDX_W-1:0] local_free_idx [NUM_WORKERS];

    // latch address from memory pool to respond to node
    logic [DATA_W-1:0] worker_resp_data [NUM_WORKERS];

    // Flags for output arbiter
    // logic found_mem;
    // logic found_resp;
    // logic node_resp_served [NUM_WORKERS];
    // logic mem_resp_served [NUM_WORKERS];
    // logic any_mem_wait;

    // registers for release operation
   logic [CXL_TABLE_DEPTH-1:0] local_release_mask [NUM_WORKERS]; // marks cxl table entries to free
   logic [CXL_TABLE_DEPTH-1:0] worker_release_mask [NUM_WORKERS]; // latch for worker across states
   logic [CXL_IDX_W-1:0] worker_release_ptr [NUM_WORKERS]; // tracker to track which cxl table entries have been freed

    // Combinational Logic
    always_comb begin
        resp_valid_o = '0;
        comp_signal_o = '0;
        load_data_o = '0;
        mem_req_valid_o = 0;
        mem_we_o = 0;
        mem_addr_o = '0;
        
        for (int w = 0; w < NUM_WORKERS; w++) begin
            // Defaults
            cxl_upd_en[w] = 1'b0;
            cxl_upd_alloc[w] = 1'b0;
            cxl_upd_idx[w] = '0;
            worker_next_state[w] = worker_state[w]; // default to remaining in same state
            local_hit[w] = 1'b0;
            local_has_free[w] = 1'b0;
            local_hit_idx[w] = '0;
            local_free_idx[w] = '0;
            local_release_mask[w] = '0;

            case (worker_state[w])

                W_IDLE: begin
                    // $display("[%0t] CXL_CONTROLLER: worker[%0d] in W_IDLE state", $time, w);
                    // Stay idle until dispatched by sequential logic
                    if (req_valid_i[w] && req_ready_o[w]) begin
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
                            
                            // if address found in CXL Table
                            if (local_hit[w]) begin
                                cxl_upd_alloc[w] = 1'b0;
                                cxl_upd_idx[w] = local_hit_idx[w];
                                worker_next_state[w] = W_MEM_REQ;
                            // if not found, assign free entry and initiate load
                            end else if (local_has_free[w]) begin
                                cxl_upd_alloc[w] = 1'b1;
                                cxl_upd_idx[w] = local_free_idx[w];
                                worker_next_state[w] = W_MEM_REQ;
                            // stall if table full (for now)
                            end else begin
                                worker_next_state[w] = W_CXL_QUERY;
                            end
                        end

                        CMD_TX_ABORT : begin
                            // It iterates over every address in the node's Release_Set
                            // For each address, it deletes the node's ID from the CXL Table entry's CheckOut Vector
                            // It returns a CXL_ABORT response back to the node

                            // Identify CXL Table entries to update
                            local_release_mask[w] = '0;
                            for (int i = 0; i < RELEASE_SET_DEPTH; i++) begin
                                if (worker_release_valid[w][i]) begin
                                    for (int j = 0; j < CXL_TABLE_DEPTH; j++) begin
                                        if (cxl_table_valid[j] && cxl_table_addr[j] == worker_release_addr[w][i]) begin
                                            local_release_mask[w][j] = 1'b1;
                                        end
                                    end
                                end
                            end
                            worker_next_state[w] = W_ABORT_CXL_UPDATE;
                        end

                        CMD_TX_COMMIT: begin
                            worker_next_state[w] = W_IDLE; // placeholder
                        end

                        default: begin
                            worker_next_state[w] = W_IDLE;
                        end
                    endcase
                end

                W_MEM_REQ: begin
                    mem_req_valid_o = 1;
                    mem_we_o = 0;
                    mem_addr_o = worker_load_addr[w];
                    worker_next_state[w] = W_MEM_WAIT;
                end

                W_MEM_WAIT: begin
                    if (mem_rvalid_i) begin
                        cxl_upd_en[w] = 1'b1;
                        worker_next_state[w] = W_RESPOND;
                    end else begin
                        worker_next_state[w] = W_MEM_WAIT;
                    end
                end

                W_RESPOND: begin
                    resp_valid_o[w] = 1;  // only assert the bit for this worker's node
                    worker_next_state[w] = W_IDLE;
                    case (worker_cmd[w])
                        CMD_LOAD: begin
                            comp_signal_o = COMP_LOAD_DONE;
                            load_data_o = worker_resp_data[w];
                        end
                        CMD_TX_ABORT: begin
                            comp_signal_o = COMP_ABORT;
                            load_data_o = '0;
                        end
                        default: begin
                            comp_signal_o = COMP_NONE;
                            load_data_o = '0;
                        end
                    endcase
                end

                W_ABORT_CXL_UPDATE: begin
                    // state when a worker is updating the cxl table from the release set
                    // find next set bit in mask from current pointer
                    worker_next_state[w] = W_RESPOND; // assume done
                    // only transition to W_ABORT_CXL_UPDATE if there are entries to release
                    for (int j = 0; j < CXL_TABLE_DEPTH; j++) begin
                        if (worker_release_mask[w][j] && j >= worker_release_ptr[w]) begin
                            worker_next_state[w] = W_ABORT_CXL_UPDATE; // still more to do
                        end
                    end
                end

                default: begin
                    worker_next_state[w] = W_IDLE;
                end

            endcase
        end

        // Output arbitration

        // // check if any worker is already in W_MEM_WAIT
        // any_mem_wait = 1'b0;
        // for (int w = 0; w < NUM_WORKERS; w++) begin
        //     if (worker_state[w] == W_MEM_WAIT) begin
        //         any_mem_wait = 1'b1;
        //     end
        // end

        // // pick first worker in W_MEM_REQ, only if no worker is already waiting
        // for (int w = 0; w < NUM_WORKERS; w++) begin
        //     // only send request if there is no one else waiting for a request
        //     if (!found_mem && !any_mem_wait && worker_state[w] == W_MEM_REQ) begin
        //         $display("[%0t] CXL_CONTROLLER: arbiter serving worker[%0d] mem_req, setting mem_resp_served=1", $time, w);
        //         mem_req_valid_o = 1;
        //         mem_we_o = 0;
        //         mem_addr_o = worker_load_addr[w];
        //         found_mem = 1'b1;
        //         mem_resp_served[w] = 1'b1;
        //     end
        // end

        // // pick first worker in W_RESPOND
        // for (int w = 0; w < NUM_WORKERS; w++) begin
        //     if (!found_resp && worker_state[w] == W_RESPOND) begin
        //         $display("[%0t] CXL_CONTROLLER: arbiter serving worker[%0d] response, setting node_resp_served=1", $time, w);
        //         resp_valid_o = 1;
        //         found_resp = 1'b1;
        //         node_resp_served[w] = 1'b1;
        //         case (worker_cmd[w])
        //             CMD_LOAD: begin // completed load
        //                 comp_signal_o = COMP_LOAD_DONE;
        //                 load_data_o = worker_resp_data[w];
        //             end
        //             CMD_TX_ABORT: begin // completed tx_abort
        //                 comp_signal_o = COMP_ABORT;
        //                 load_data_o = '0;
        //             end
        //             default: begin
        //                 comp_signal_o = COMP_NONE;
        //                 load_data_o = '0;
        //             end
        //         endcase
        //     end
        // end

    end


    // Sequential Logic
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            for (int i = 0; i < NUM_WORKERS; i++) begin
                worker_state[i] <= W_IDLE;
                worker_cmd[i] <= CMD_LOAD;
                worker_node_id[i] <= '0;
                worker_load_addr[i] <= '0;
                worker_cxl_upd_idx[i] <= '0;
                worker_cxl_upd_alloc[i] <= '0;

                worker_release_valid[i] <= '0;
                worker_release_is_write[i] <= '0;
                worker_release_addr[i] <= '0;
                worker_release_data[i] <= '0;
                worker_release_ptr[i] <= '0;
                worker_release_mask[i] <= '0;
            end
            for (int e = 0; e < CXL_TABLE_DEPTH; e++) begin
                cxl_table_valid[e] <= 1'b0;
                cxl_table_addr[e] <= '0;
                cxl_table_checkout_vec[e] <= '0;
                cxl_table_in_progress_vec[e] <= '0;
                cxl_table_locked[e] <= 1'b0;
            end
        end else begin
            
            // Latch worker fields with request information from nodes to the active worker
            // Worker can begin work when there is a request and the worker is idle
            if (req_valid_i[active_worker_idx] && req_ready_o[active_worker_idx]) begin
                worker_cmd[active_worker_idx] <= cmd;
                worker_load_addr[active_worker_idx] <= load_addr_i;
                worker_node_id[active_worker_idx] <= req_valid_i;
                worker_release_valid[active_worker_idx] <= release_valid_i;
                worker_release_is_write[active_worker_idx] <= release_is_write_i;
                worker_release_addr[active_worker_idx] <= release_addr_i;
                worker_release_data[active_worker_idx] <= release_data_i;
            end

            for (int w = 0; w < NUM_WORKERS; w++) begin
                // latches for W_CXL_QUERY -> W_MEM_REQ transition
                if (worker_state[w] == W_CXL_QUERY && worker_next_state[w] == W_MEM_REQ) begin
                    worker_cxl_upd_idx[w] <= cxl_upd_idx[w];
                    worker_cxl_upd_alloc[w] <= cxl_upd_alloc[w];
                    cxl_table_in_progress_vec[cxl_upd_idx[w]] <= cxl_table_in_progress_vec[cxl_upd_idx[w]] | worker_node_id[w];
                end

                // latches for W_CXL_QUERY -> W_ABORT_CXL_UPDATE transition
                if (worker_state[w] == W_CXL_QUERY && worker_next_state[w] == W_ABORT_CXL_UPDATE) begin
                    worker_release_mask[w] <= local_release_mask[w];
                    worker_release_ptr[w] <= '0;
                end

                // cxl table update
                if (cxl_upd_en[w]) begin
                    // LOAD case
                    if(worker_cmd[w] == CMD_LOAD) begin
                        // if address was not in cxl table
                        if (worker_cxl_upd_alloc[w]) begin
                            cxl_table_valid[worker_cxl_upd_idx[w]] <= 1'b1;
                            cxl_table_addr[worker_cxl_upd_idx[w]] <= worker_load_addr[w];
                            cxl_table_checkout_vec[worker_cxl_upd_idx[w]] <= worker_node_id[w];
                            cxl_table_in_progress_vec[worker_cxl_upd_idx[w]] <= '0;
                            cxl_table_locked[worker_cxl_upd_idx[w]] <= 1'b0;
                            $display("[%0t] CXL_TABLE ALLOC: worker[%0d] allocated new entry[%0d] for addr=0x%0h node_id=%0b", 
                                $time, w, worker_cxl_upd_idx[w], worker_load_addr[w], worker_node_id[w]);
                        // if address was already in cxl table
                        end else begin
                            cxl_table_checkout_vec[worker_cxl_upd_idx[w]] <= cxl_table_checkout_vec[worker_cxl_upd_idx[w]] | worker_node_id[w];
                            cxl_table_in_progress_vec[worker_cxl_upd_idx[w]] <= cxl_table_in_progress_vec[worker_cxl_upd_idx[w]] & ~worker_node_id[w];
                            $display("[%0t] CXL_TABLE HIT: worker[%0d] added node_id=%0b to existing entry[%0d] for addr=0x%0h checkout_vec=%0b", 
                                $time, w, worker_node_id[w], worker_cxl_upd_idx[w], worker_load_addr[w], cxl_table_checkout_vec[worker_cxl_upd_idx[w]]);
                        end
                    end
                end
                
                // Delete node from CXL_Table[address] 
                if (worker_state[w] == W_ABORT_CXL_UPDATE) begin
                    // find and clear the current entry
                    for (int j = 0; j < CXL_TABLE_DEPTH; j++) begin
                        if (worker_release_mask[w][j] && j >= worker_release_ptr[w]) begin
                            cxl_table_checkout_vec[j] <= cxl_table_checkout_vec[j] & ~worker_node_id[w];
                            // invalidate entry if no nodes are using it
                            if ((cxl_table_checkout_vec[j] & ~worker_node_id[w]) == '0) begin
                                cxl_table_valid[j] <= 1'b0;
                            end
                            worker_release_ptr[w] <= j + 1; // advance past this entry
                            break;
                        end
                    end
                end

                // latch data sent from memory pool
                if (mem_rvalid_i && worker_state[w] == W_MEM_WAIT) begin
                    worker_resp_data[w] <= mem_rdata_i;
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

