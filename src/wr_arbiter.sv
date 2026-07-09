// Handle pushing release set entries into write buffer
// serializer / burst engine: it accepts one variable-length job from the controller 
// (1 entry for a LOAD, N entries for a COMMIT) and meters it into the single-port FIFO. 
// The controller stops running the streaming loop itself and goes back to being a clean 
// fixed-latency pipeline

module wr_arbiter #(
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,

    parameter int NUM_NODES = 4,
    parameter int RELEASE_SET_DEPTH = 16,
    parameter int DEPTH = 32
)(

    input logic clk_i,
    input logic rst_i, 

    // Input from the cxl controller
    input logic req_valid_i, // one cycle accept pulse
    input logic [1:0] req_type_i,
    input logic [DATA_W-1:0] load_addr_i,
    input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_i,
    input logic [RELEASE_SET_DEPTH-1:0] release_is_write_i,
    input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_i,
    input logic [NUM_NODES-1:0] worker_i,

    // Output to the cxl controller
    output logic busy_o,
    output logic done_o,

    // Input from the memory buffer
    input logic push_ready_i,

    // Output to the memory buffer
    output logic push_valid_o, // handshake
    output logic push_we_o, // should always be 1
    output logic [ADDR_W-1:0] push_addr_o,
    output logic [DATA_W-1:0] push_wdata_o,
    output logic [NUM_NODES-1:0] push_worker_o,
    output logic [1:0] push_req_type_o
);

    localparam int PTR_W = $clog2(RELEASE_SET_DEPTH);
    localparam logic [1:0] CMD_TX_COMMIT = 2'b10;
    localparam logic [1:0] CMD_LOAD = 2'b00;

    // States
    logic busy_q; // currently pushing read/write requests into the memory buffer
    logic [PTR_W-1:0] release_ptr_q;


    // ================ Latched job payload ================
    // captured at accept, drives outputs during drain
    logic [1:0] req_type_q;
    logic [DATA_W-1:0] load_addr_q;
    logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_q;
    logic [RELEASE_SET_DEPTH-1:0] release_is_write_q;
    logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_q;
    logic [NUM_NODES-1:0] worker_q;

    // ================ Release Set Pointer Manipulation ================
    // Find first write index
    logic [PTR_W-1:0] first_write_idx;
    logic first_write_found;
    always_comb begin
        first_write_idx = '0;
        first_write_found = 1'b0; // flag
        if(req_type_i == CMD_TX_COMMIT) begin
            // go through the release set to find first write index
            for (int i = 0; i < RELEASE_SET_DEPTH; i++) begin
                if (!first_write_found && release_is_write_i[i]) begin
                    first_write_idx = PTR_W'(i);
                    first_write_found = 1'b1;
                end
            end
        end
    end

    // Release set pointer advance (strictly finds the single next pointer)
    logic [PTR_W-1:0] next_write_idx;
    logic has_next_write;
    always_comb begin
        next_write_idx = release_ptr_q;
        has_next_write = 1'b0;
        for (int i = 0; i < RELEASE_SET_DEPTH; i++) begin
            if (!has_next_write && (i > release_ptr_q) && release_is_write_q[i]) begin
                next_write_idx = PTR_W'(i);
                has_next_write = 1'b1;
            end
        end
    end
    
    // ================ Combinational output drive ================
    always_comb begin
        push_valid_o = busy_q;
        push_we_o = (req_type_q == CMD_TX_COMMIT) ? 1'b1 : 1'b0;
        push_addr_o = (req_type_q == CMD_TX_COMMIT) ? release_addr_q[release_ptr_q] : load_addr_q;
        push_wdata_o = release_data_q[release_ptr_q];
        push_worker_o = worker_q;
        push_req_type_o = req_type_q;
    end
    assign busy_o = busy_q;

    // ================ Sequencer ================
    logic accept_beat; // working when the buffer downstream can accept a push
    assign accept_beat = push_valid_o && push_ready_i;
    assign done_o = (accept_beat && req_type_q == CMD_TX_COMMIT && !has_next_write) || // last commit write
                    (accept_beat && req_type_q == CMD_LOAD); // single load beat
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            busy_q        <= 1'b0;
            release_ptr_q <= '0;
        end else begin
            if (!busy_q) begin
                if (req_valid_i) begin
                    if (req_type_i == CMD_TX_COMMIT && first_write_found) begin
                        busy_q <= 1'b1;
                        release_ptr_q <= first_write_idx;
                        // latch payload
                        req_type_q <= req_type_i;
                        load_addr_q <= load_addr_i;
                        release_addr_q <= release_addr_i;
                        release_is_write_q <= release_is_write_i;
                        release_data_q <= release_data_i;
                        worker_q <= worker_i;
                    end else if (req_type_i == CMD_LOAD) begin
                        busy_q <= 1'b1;
                        release_ptr_q <= '0;
                        // latch payload
                        req_type_q <= req_type_i;
                        load_addr_q <= load_addr_i;
                        release_addr_q <= release_addr_i;
                        release_is_write_q <= release_is_write_i;
                        release_data_q <= release_data_i;
                        worker_q <= worker_i;
                    end
                end
            end else begin
                if (accept_beat) begin
                    if (req_type_q == CMD_LOAD)
                        busy_q <= 1'b0; // one-beat load done
                    else if (has_next_write)
                        release_ptr_q <= next_write_idx;
                    else
                        busy_q <= 1'b0; // last commit write done
                end
            end
        end
    end

endmodule