module cxl_controller #(
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,

    parameter int NUM_NODES = 4,
    parameter int RELEASE_SET_DEPTH = 16, // max 16 variables per transaction
    parameter int CXL_TABLE_DEPTH = 32, // max 32 entires in cxl table
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
    release_entry_t release_set [RELEASE_SET_DEPTH];
    // Fill in the set from the input
    always_comb begin
        for (int i = 0; i < RELEASE_SET_DEPTH; i++) begin
            release_set[i].valid = release_valid_i[i];
            release_set[i].is_write = release_is_write_i[i];
            release_set[i].addr = release_addr_i[i];
            release_set[i].data = release_data_i[i];
        end
    end

    // ================ Building CXL Table ================
    typedef struct packed {
        logic valid; // indicate whether the entry is in use
        logic [ADDR_W-1:0] addr; // indicate cxl address of the entry
        logic [NUM_NODES-1:0] checkout_vec; // store the checked-out bit for each entry
        logic [NUM_NODES-1:0] in_progress_vec; // store the in-progress bit for each entry
        logic locked; // lock for each entry
    } cxl_table_entry_t;
    cxl_table_entry_t cxl_table [CXL_TABLE_DEPTH];

    // ================ Concurrent Workers ================
    parameter int NUM_WORKERS = 2;
    typedef enum logic [1:0] {
        W_IDLE,
        W_BUSY,
        W_DONE
    } worker_state_t;
    // These four arrays make up the workers (index-addressable)
    worker_state_t worker_state [NUM_WORKERS];
    cxl_cmd_t worker_cmd [NUM_WORKERS];
    logic [NUM_NODES-1:0] worker_node_id [NUM_WORKERS];
    logic [ADDR_W-1:0] worker_load_addr [NUM_WORKERS];
    release_entry_t worker_release_set [NUM_WORKERS][RELEASE_SET_DEPTH];

    // Mark idle workers
    logic [NUM_WORKERS-1:0] worker_idle; 
    always_comb begin
        for (int i = 0; i < NUM_WORKERS; i++) begin
            worker_idle[i] = (worker_state[i] == W_IDLE);
        end
    end
    // Dispatch worker
    logic dispatch_valid;
    logic [$clog2(NUM_WORKERS)-1:0] dispatch_idx; // marks which worker was dispatched
    always_comb begin
        dispatch_valid = 1'b0;
        dispatch_idx = '0;
        // mark first available worker as available
        for (int i = 0; i < NUM_WORKERS; w++) begin
            if (!dispatch_valid && worker_idle[i]) begin
                dispatch_valid = 1'b1;
                dispatch_idx = i[$clog2(NUM_WORKERS)-1:0];
            end
        end
    end
    assign req_ready_o = dispatch_valid;

    // Combinational Logic
    always_comb begin
        for (int i = 0; i < NUM_WORKERS; i++) begin
            if (worker_state[i] == W_BUSY) begin
                case (worker_cmd[i])

                    CMD_LOAD: begin
                        // 
                    end

                    CMD_TX_ABORT: begin
                        // 
                    end

                    CMD_TX_COMMIT: begin
                        // 
                    end

                    default:

                endcase
            end
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
        end else begin
            // Assign work to dispatched worker
            if (req_valid_i && req_ready_o) begin
                worker_state[dispatch_idx] <= W_BUSY;
                worker_cmd[dispatch_idx] <= cmd;
                worker_node_id[dispatch_idx] <= node_id_i;
                worker_load_addr[dispatch_idx] <= load_addr_i;
                for (int i = 0; i < RELEASE_SET_DEPTH; i++) begin
                    worker_release_set[dispatch_idx][i] <= release_set[i];
                end
            end

        end
    end

endmodule