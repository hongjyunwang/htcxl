module cxl_controller #(
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,

    parameter int NUM_NODES = 4,
    parameter int RELEASE_SET_DEPTH = 16, // max 16 variables per transaction
    parameter int CXL_TABLE_DEPTH = 32, // max 32 entires in cxl table

)(
    input logic clk_i,
    input logic rst_i,

    // Inputs from hosts/nodes
    input logic req_valid_i,
    input logic [1:0] tx_signal_i,
    input logic [NUM_NODES-1:0] node_id_i,
    input logic [ADDR_W-1:0] load_addr_i, // Used for CMD_LOAD
    input logic [RELEASE_SET_DEPTH-1:0] release_valid_i, // release set valid mark
    input logic [RELEASE_SET_DEPTH-1:0] release_is_write_i, // release set write mark
    input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_i, // release set address
    input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_i, // release set data
    
    // Inputs from CXL memory pool
    input logic [DATA_W-1:0] mem_rdata_i,

    // Outputs to hosts/nodes
    output logic req_ready_o,
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

    // Building Release Set 
    typedef struct packed {
        logic valid;
        logic is_write;
        logic [ADDR_W-1:0] addr;
        logic [DATA_W-1:0] data;
    } release_entry_t;
    release_entry_t release_set [RELEASE_SET_DEPTH];
    always_comb begin
        for (int i = 0; i < RELEASE_SET_DEPTH; i++) begin
            release_set[i].valid = release_valid_i[i];
            release_set[i].is_write = release_is_write_i[i];
            release_set[i].addr = release_addr_i[i];
            release_set[i].data = release_data_i[i];
        end
    end

    // Building CXL Table 
    typedef struct packed {
        logic valid;
        logic [ADDR_W-1:0] addr;
        logic [NUM_NODES-1:0] checkout_vec;
        logic [NUM_NODES-1:0] in_progress_vec;
    } cxl_table_entry_t; 
    cxl_table_entry_t cxl_table [CXL_TABLE_DEPTH];

    // FSM 
    always_comb begin
        req_ready_o = 1'b1;
        resp_valid_o = 1'b0;
        comp_signal_o = COMP_NONE;
        load_data_o = '0;
        mem_req_valid_o = 1'b0;
        mem_we_o = 1'b0;
        mem_addr_o = '0;
        mem_wdata_o = '0;

        case (cmd)
            CMD_LOAD: begin
                // handle load
            end

            CMD_TX_ABORT: begin
                // handle abort
            end

            CMD_TX_COMMIT: begin
                // handle commit
            end

            default: begin
                // default behavior
            end
        endcase
    end

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            for (int i = 0; i < CXL_TABLE_DEPTH; i++) begin
                cxl_table[i].valid <= 1'b0;
                cxl_table[i].addr <= '0;
                cxl_table[i].checkout_vec <= '0;
                cxl_table[i].in_progress_vec <= '0;
            end
        end else begin
            
            
            
        end
    end

endmodule