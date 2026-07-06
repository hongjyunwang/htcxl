module wr_buf #(
    parameter int NUM_NODES = 4,
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,
    parameter int DEPTH = 32
)(
    input logic clk_i,
    input logic rst_i,

    // Input from release_engine
    // Push
    input logic push_valid_i, // handshake
    input logic push_we_i,
    input logic [ADDR_W-1:0] push_addr_i,
    input logic [DATA_W-1:0] push_wdata_i,
    input logic [NUM_NODES-1:0] push_worker_i,
    input logic [1:0] push_req_type_i,

    // Output to cxl controller
    output logic push_ready_o, // FIFO not full

    // Input from the memory pool
    input logic mem_ready_i,
    input logic [DATA_W-1:0] mem_rdata_i,

    // Output to memory pool
    // Pop
    output logic pop_valid_o,
    output logic pop_we_o,
    output logic [ADDR_W-1:0] pop_addr_o,
    output logic [DATA_W-1:0] pop_wdata_o,
    output logic [NUM_NODES-1:0] pop_worker_o,
    output logic [1:0] pop_req_type_o,

    // Output to the nodes 
    output logic resp_valid_o, // handshake to nodes, signal completion
    output logic [NUM_NODES-1:0] resp_worker_o, // which node gets the memory response
    output logic [DATA_W-1:0] resp_rdata_o
);

    localparam int PTR_W = $clog2(DEPTH);

    typedef struct packed {
        logic we;
        logic [ADDR_W-1:0] addr;
        logic [DATA_W-1:0] wdata;
        logic [NUM_NODES-1:0] worker;
        logic [1:0] req_type;
    } entry_t;

    // Simple FIFO buffer
    entry_t mem [DEPTH];
    logic [PTR_W-1:0] wr_ptr;
    logic [PTR_W-1:0] rd_ptr;
    logic [PTR_W:0] count;

    logic full;
    logic empty;
    assign full = (count == DEPTH[PTR_W:0]);
    assign empty = (count == '0);

    // Registers to store in-flight memory requests
    logic req_in_flight;
    logic [NUM_NODES-1:0] inflight_worker;

    // combinational output wires
    assign push_ready_o = !full;
    assign pop_valid_o = !empty && !req_in_flight;

    entry_t entry_to_read;
    entry_t entry_to_write;
    assign entry_to_read = mem[rd_ptr];
    assign pop_we_o = entry_to_read.we;
    assign pop_addr_o = entry_to_read.addr;
    assign pop_wdata_o = entry_to_read.wdata;
    assign pop_worker_o = entry_to_read.worker;
    assign pop_req_type_o = entry_to_read.req_type;
    always_comb begin
        entry_to_write.we = push_we_i;
        entry_to_write.addr = push_addr_i;
        entry_to_write.wdata = push_wdata_i;
        entry_to_write.worker = push_worker_i;
        entry_to_write.req_type = push_req_type_i;
    end
    assign resp_rdata_o = mem_rdata_i;

   
    // Sequential Logic
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
            count <= '0;
            req_in_flight <= 1'b0;
            inflight_worker <= '0;
            resp_valid_o <= 1'b0;
            resp_worker_o <= '0;
        end else begin
            // Default: clear response pulse every cycle
            resp_valid_o <= 1'b0;

            // Memory response returned, relay to node
            if (req_in_flight && mem_ready_i) begin
                req_in_flight <= 1'b0;
                resp_valid_o <= 1'b1; // one-cycle pulse
                resp_worker_o <= inflight_worker; // tell caller which node this data belongs to
            end

            // Push
            if (!full && push_valid_i) begin
                mem[wr_ptr] <= entry_to_write;
                wr_ptr <= (wr_ptr == PTR_W'(DEPTH-1)) ? '0 : wr_ptr + 1'b1;
            end

            // Pop 
            // check no request is in flight
            if (!empty && !req_in_flight) begin
                inflight_worker <= entry_to_read.worker;
                req_in_flight <= entry_to_read.we ? 1'b0 : 1'b1; // writes: no in-flight wait
                rd_ptr <= (rd_ptr == PTR_W'(DEPTH-1)) ? '0 : rd_ptr + 1'b1;
            end

            // Count
            case ({push_valid_i && !full, !empty && !req_in_flight})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
