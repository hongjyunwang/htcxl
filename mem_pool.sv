module mem_pool_stub #(
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,
    parameter int MEM_DEPTH = 1024,
    parameter int MEM_LATENCY = 1
)(
    input logic clk_i,
    input logic rst_i,

    input logic mem_req_valid_i,
    input logic mem_we_i,
    input logic [ADDR_W-1:0] mem_addr_i,
    input logic [DATA_W-1:0] mem_wdata_i,

    output logic [DATA_W-1:0] mem_rdata_o,
    output logic mem_rvalid_o
);

    // Backing store (testbench preloads this via hierarchical reference)
    logic [DATA_W-1:0] mem [MEM_DEPTH];

    // Map a byte/word address into the backing store
    // converts address to index in mem
    function automatic int unsigned widx(input logic [ADDR_W-1:0] a);
        widx = a % MEM_DEPTH;
    endfunction

    // Read-response pipeline: MEM_LATENCY stages
    // a bit string that increments itself to mimic memory latency
    logic rd_pending [MEM_LATENCY];
    logic [DATA_W-1:0] rd_data [MEM_LATENCY];

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            for (int i = 0; i < MEM_LATENCY; i++) begin
                rd_pending[i] <= 1'b0;
                rd_data[i] <= '0;
            end
        end else begin
            // Writes apply immediately at this edge
            if (mem_req_valid_i && mem_we_i) begin
                mem[widx(mem_addr_i)] <= mem_wdata_i;
            end

            // Stage 0: capture a fresh read
            rd_pending[0] <= (mem_req_valid_i && !mem_we_i);
            rd_data[0] <= mem[widx(mem_addr_i)];

            // Shift the pipeline toward the output
            for (int i = 1; i < MEM_LATENCY; i++) begin
                rd_pending[i] <= rd_pending[i-1];
                rd_data[i] <= rd_data[i-1];
            end
        end
    end

    // Combinational output from the final stage → exactly MEM_LATENCY-cycle latency
    assign mem_rvalid_o = rd_pending[MEM_LATENCY-1];
    assign mem_rdata_o  = rd_data[MEM_LATENCY-1];

endmodule