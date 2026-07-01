// wrapper around sram

module cxl_table#(
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,

    parameter int NUM_NODES = 4,
    parameter int RELEASE_SET_DEPTH = 16, // max 16 variables per transaction
    parameter int CXL_TABLE_DEPTH = 16 // max 32 entires in cxl table
)(
    input logic clk_i,
    input logic rst_i,

    // inputs from cxl controller
    input logic controller_valid_i,

    input logic [1:0] req_type_i, // 00: CMD_LOAD, 01: CMD_TX_ABORT, 10: CMD_TX_COMMIT
    input logic [NUM_NODES-1:0] req_node_i,
    input logic [ADDR_W-1:0] addr_i, // single address for LOAD
    input logic [RELEASE_SET_DEPTH-1:0] release_valid_i,
    input logic [RELEASE_SET_DEPTH-1:0] release_is_write_i,
    input logic [RELEASE_SET_DEPTH-1:0] release_is_read_i,
    input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_i,
    input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_i,

    // outputs to cxl controller
    output logic hit_o,
    output logic [NUM_NODES-1:0] check_out_o,
    output logic [NUM_NODES-1:0] in_progress_o,
    output logic busy_o, 
    output logic req_compl_o, // completion signal of ENTIRE request, not one memory fetch
    output logic any_conflict_o // used for tx_commit
);

// ================ Parameters and SRAM Instantiation ================
// Defining CXL Table parameters
// 16 entries, 2 ways
localparam int NUM_WAYS = 2;
localparam int NUM_SETS = CXL_TABLE_DEPTH / NUM_WAYS;
localparam int SET_IDX_W = $clog2(NUM_SETS);
localparam int TAG_W = ADDR_W - SET_IDX_W;
localparam int SRAM_W = 1 + TAG_W + 2*NUM_NODES;
localparam int SEQ_PTR_W = $clog2(RELEASE_SET_DEPTH);

localparam logic [1:0] CMD_LOAD = 2'b00;
localparam logic [1:0] CMD_TX_ABORT  = 2'b01;
localparam logic [1:0] CMD_TX_COMMIT = 2'b10;

logic ce_way0, ce_way1;
logic we_way0, we_way1;
logic [SET_IDX_W-1:0] set_index;
logic [SRAM_W-1:0] wdata_way0, wdata_way1;
logic [SRAM_W-1:0] rdata_way0, rdata_way1;

sram #(
    .DATA_W(SRAM_W),
    .DEPTH(NUM_SETS)
) sram_way0 (
    .clk_i(clk_i),
    .ce_i(ce_way0),
    .we_i(we_way0),
    .idx_i(set_index),
    .wdata_i(wdata_way0),
    .rdata_o(rdata_way0)
);

sram #(
    .DATA_W(SRAM_W),
    .DEPTH(NUM_SETS)
) sram_way1 (
    .clk_i(clk_i),
    .ce_i(ce_way1),
    .we_i(we_way1),
    .idx_i(set_index),
    .wdata_i(wdata_way1),
    .rdata_o(rdata_way1)
);

// ================ Address Parsing ================
// For a given 64-bit CXL address:
// addr[SET_IDX_W-1 : 0] --> set index
// addr[ADDR_W-1 : SET_IDX_W] --> tag of specific entry (way)

function automatic logic [SET_IDX_W-1:0] get_set_index(input logic [ADDR_W-1:0] addr);
    return addr[SET_IDX_W-1:0];
endfunction

function automatic logic [TAG_W-1:0] get_tag(input logic [ADDR_W-1:0] addr);
    return addr[ADDR_W-1:SET_IDX_W];
endfunction

// Unpack a raw SRAM word into its fields
task automatic unpack_entry(
    input  logic [SRAM_W-1:0] word,
    output logic valid,
    output logic [TAG_W-1:0] tag,
    output logic [NUM_NODES-1:0] checkout_vec,
    output logic [NUM_NODES-1:0] inprog_vec
);
    inprog_vec = word[NUM_NODES-1 : 0];
    checkout_vec = word[2*NUM_NODES-1 : NUM_NODES];
    tag = word[2*NUM_NODES + TAG_W - 1 : 2*NUM_NODES];
    valid = word[SRAM_W-1];
endtask

function automatic logic [SRAM_W-1:0] pack_entry(
    input logic valid,
    input logic [TAG_W-1:0] tag,
    input logic [NUM_NODES-1:0] checkout_vec,
    input logic [NUM_NODES-1:0] inprog_vec
);
    return {valid, tag, checkout_vec, inprog_vec};
endfunction

// ================ FSM ================
// State machine:
//   SEQ_IDLE  : waiting for controller_valid_i
//   SEQ_READ  : assert ce to both SRAMs, present set_index (1 cycle)
//   SEQ_MOD   : SRAM data has arrived; tag compare, compute new vectors (1 cycle)
//   SEQ_WRITE : write modified word back to winning way (1 cycle)
//   SEQ_DRAIN : no more entries to feed, wait for pipeline to drain
//   SEQ_DONE  : pulse req_compl_o, return to IDLE
//
// No hazard detection needed: release sets are guaranteed to have no duplicate
// addresses, and we do not pipeline multiple LOADs. Each entry in flight always
// touches a distinct set index from all others currently in the pipeline.

typedef enum logic [2:0] {
    SEQ_IDLE = 3'd0,
    SEQ_READ = 3'd1,
    SEQ_MOD = 3'd2,
    SEQ_WRITE = 3'd3,
    SEQ_DRAIN = 3'd4,
    SEQ_DONE = 3'd5
} seq_state_t;
seq_state_t seq_state_q, seq_state_d;

// Current entry pointer into the release set
logic [SEQ_PTR_W-1:0] seq_ptr_q, seq_ptr_d;

// Latched request — held stable for entire multi-cycle operation
logic [1:0] lat_req_type_q;
logic [NUM_NODES-1:0] lat_req_node_q;
logic [ADDR_W-1:0] lat_load_addr_q;
logic [RELEASE_SET_DEPTH-1:0] lat_rel_valid_q;
logic [RELEASE_SET_DEPTH-1:0] lat_rel_is_write_q;
logic [RELEASE_SET_DEPTH-1:0] lat_rel_is_read_q;
logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] lat_rel_addr_q;
logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] lat_rel_data_q;

logic any_conflict_q;


// ================ Combinational Helpers ================

// Active address being processed this iteration + parsing
// For LOAD it is lat_load_addr_q; for ABORT/COMMIT it is lat_rel_addr_q[seq_ptr_q]
logic [ADDR_W-1:0] cur_addr;
always_comb begin
    // LOAD -> use single address
    if (lat_req_type_q == CMD_LOAD)
        cur_addr = lat_load_addr_q;
    else
        // Tx_ABORT or Tx_COMMIT -> use release set entry
        cur_addr = lat_rel_addr_q[seq_ptr_q];
end
logic [SET_IDX_W-1:0] cur_set_index;
logic [TAG_W-1:0] cur_tag;
always_comb begin
    cur_set_index = get_set_index(cur_addr);
    cur_tag = get_tag(cur_addr);
end

// SRAM read results — valid one cycle after SEQ_READ
logic r_valid0, r_valid1;
logic [TAG_W-1:0] r_tag0, r_tag1;
logic [NUM_NODES-1:0] r_checkout0, r_checkout1;
logic [NUM_NODES-1:0] r_inprog0, r_inprog1;
// Remember that rdata_way0 and rdata_way1 are data returned from sram
always_comb begin
    unpack_entry(rdata_way0, r_valid0, r_tag0, r_checkout0, r_inprog0);
    unpack_entry(rdata_way1, r_valid1, r_tag1, r_checkout1, r_inprog1);
end

// Deciding which way to allocate to when allocating new table entry
// LRU state — one bit per set (0=way0 MRU, 1=way1 MRU)
logic lru_q [NUM_SETS];
// Way to write on a miss (prefer invalid way, then LRU)
logic alloc_way;
always_comb begin
    if (!r_valid0) alloc_way = 1'b0; // way0 is free
    else if (!r_valid1) alloc_way = 1'b1; // way1 is free
    else alloc_way = lru_q[cur_set_index]; // both valid, evict LRU
end

// ================ Pipeline Stage Registers ================
// READ -> MOD
typedef struct packed {
    logic valid;
    logic [SET_IDX_W-1:0] set_index;
    logic [TAG_W-1:0] tag;
    logic [1:0] req_type;
    logic [NUM_NODES-1:0] req_node;
    logic is_write; // only meaningful for CMD_TX_COMMIT
    logic is_read; // only meaningful for CMD_TX_COMMIT
} read_mod_t;
// MOD -> WRITE
typedef struct packed {
    logic valid;
    logic [SET_IDX_W-1:0] set_index;
    logic [TAG_W-1:0] tag;
    logic [1:0] req_type;
    logic [NUM_NODES-1:0] req_node;
    logic hit;
    logic hit_way;
    logic alloc_way;
    logic [NUM_NODES-1:0] new_checkout;
    logic [NUM_NODES-1:0] new_inprog;
    logic new_valid;
} mod_write_t;

read_mod_t read_mod_q, read_mod_d;
mod_write_t mod_write_q, mod_write_d;

// Local storage for MOD stage
logic mod_hit0, mod_hit1, mod_any_hit;
logic mod_hit_way;
logic [NUM_NODES-1:0] mod_hit_checkout, mod_hit_inprog;
logic mod_alloc_way;
logic [SRAM_W-1:0] write_packed_word;
logic write_way;
logic entry_is_write_conflict; // for Tx_Commit conflict detection

assign any_conflict_o = any_conflict_q;

// ================ Sequencer FSM ================
always_comb begin
    // defaults
    seq_state_d = seq_state_q;
    seq_ptr_d = seq_ptr_q;
    read_mod_d = read_mod_q;
    mod_write_d = mod_write_q;

    ce_way0 = 1'b0; ce_way1 = 1'b0;
    we_way0 = 1'b0; we_way1 = 1'b0;
    set_index = cur_set_index;
    wdata_way0 = '0; wdata_way1 = '0;

    busy_o = 1'b1;
    req_compl_o = 1'b0;
    hit_o = 1'b0;
    check_out_o = '0;
    in_progress_o = '0;

    entry_is_write_conflict = '0;

    // MOD stage combinational logic for LOAD
    // indicators parsed from SRAM read results
    mod_hit0 = r_valid0 && (r_tag0 == read_mod_q.tag); // way 0 hit indicator
    mod_hit1 = r_valid1 && (r_tag1 == read_mod_q.tag); // way 1 hit indicator
    mod_any_hit = mod_hit0 || mod_hit1;
    mod_hit_way = mod_hit1;
    mod_hit_checkout = mod_hit1 ? r_checkout1 : r_checkout0;
    mod_hit_inprog = mod_hit1 ? r_inprog1 : r_inprog0;
    mod_alloc_way = (!r_valid0) ? 1'b0 : (!r_valid1) ? 1'b1 : lru_q[read_mod_q.set_index];

    // WRITE stage combinational logic
    write_way = mod_write_q.hit ? mod_write_q.hit_way : mod_write_q.alloc_way;
    write_packed_word = pack_entry(
        mod_write_q.new_valid,
        mod_write_q.tag,
        mod_write_q.new_checkout,
        mod_write_q.new_inprog
    );

    // ---- Sequencer FSM ----
    unique case (seq_state_q)

        SEQ_IDLE: begin
            busy_o = 1'b0;
            read_mod_d.valid = 1'b0;
            mod_write_d.valid = 1'b0;
            if (controller_valid_i) begin
                seq_state_d = SEQ_READ;
                seq_ptr_d = '0;
            end
        end

        // ================ READ Stage ================
        SEQ_READ: begin
            // Issue SRAM read for current entry — no hazard check needed
            ce_way0 = 1'b1; // read
            ce_way1 = 1'b1; // read
            we_way0 = 1'b0; // not write
            we_way1 = 1'b0; // not write
            set_index = cur_set_index;

            // Fill READ->MOD register
            read_mod_d.valid = 1'b1; // move to MOD
            read_mod_d.set_index = cur_set_index;
            read_mod_d.tag = cur_tag;
            read_mod_d.req_type = lat_req_type_q;
            read_mod_d.req_node = lat_req_node_q;
            read_mod_d.is_write = lat_rel_is_write_q[seq_ptr_q];
            read_mod_d.is_read = lat_rel_is_read_q[seq_ptr_q];

            // Advance pointer to next valid entry for next cycle
            begin : find_next
                logic found_next;
                found_next = 1'b0;
                seq_ptr_d = seq_ptr_q;
                for (int i = seq_ptr_q + 1; i < RELEASE_SET_DEPTH; i++) begin
                    case (lat_req_type_q)
                        // Tx_Abort, just check valid bit
                        01: begin
                            if (!found_next && lat_rel_valid_q[i]) begin
                                seq_ptr_d = SEQ_PTR_W'(i);
                                found_next = 1'b1;
                            end
                        end
                        // Tx_Commit, check valid bit and .write or .read bit
                        default: begin
                            if (!found_next && lat_rel_valid_q[i] && (lat_rel_is_write_q[i] || lat_rel_is_read_q[i])) begin
                                seq_ptr_d = SEQ_PTR_W'(i);
                                found_next = 1'b1;
                            end
                        end
                    endcase
                end
                // If no next entry, move to drain remaining pipeline stages
                if (!found_next)
                    seq_state_d = SEQ_DRAIN;
            end
        end

        SEQ_DRAIN: begin
            // No more entries to feed. Wait for MOD and WRITE to finish.
            read_mod_d.valid = 1'b0; // no new reads
            if (!mod_write_q.valid && !read_mod_q.valid)
                seq_state_d = SEQ_DONE;
        end

        SEQ_DONE: begin
            req_compl_o = 1'b1;
            busy_o = 1'b0;
            seq_state_d = SEQ_IDLE;
        end

        default: seq_state_d = SEQ_IDLE;
    endcase

    // ================ MOD stage ================
    if (read_mod_q.valid) begin
        mod_write_d.valid = 1'b1;
        mod_write_d.set_index = read_mod_q.set_index;
        mod_write_d.tag = read_mod_q.tag;
        mod_write_d.req_type = read_mod_q.req_type;
        mod_write_d.req_node = read_mod_q.req_node;
        mod_write_d.hit = mod_any_hit;
        mod_write_d.hit_way = mod_hit_way;
        mod_write_d.alloc_way = mod_alloc_way;

        // Compute new vectors based on command
        unique case (read_mod_q.req_type)
            CMD_LOAD: begin
                mod_write_d.new_checkout = mod_any_hit ?
                    (mod_hit_checkout | read_mod_q.req_node) :
                    read_mod_q.req_node;
                mod_write_d.new_inprog = mod_hit_inprog & ~read_mod_q.req_node;
                mod_write_d.new_valid = 1'b1;
            end
            CMD_TX_ABORT: begin
                mod_write_d.new_checkout = mod_hit_checkout & ~read_mod_q.req_node;
                mod_write_d.new_inprog = mod_hit_inprog & ~read_mod_q.req_node;
                mod_write_d.new_valid = ((mod_hit_checkout & ~read_mod_q.req_node) != '0);
            end
            CMD_TX_COMMIT: begin
                // If a .write entry
                if(read_mod_q.is_write) begin
                    // should assert that there is always a hit
                    // if conflict
                    if((mod_hit_checkout & ~read_mod_q.req_node) != '0) begin
                        // indicate CXL_ABORT
                        entry_is_write_conflict = 1;
                    end
                    // Delete hostid from cxl table entry
                    mod_write_d.new_checkout = mod_hit_checkout & ~read_mod_q.req_node;
                    mod_write_d.new_valid = ((mod_hit_checkout & ~read_mod_q.req_node) != '0);
                end
                // .read entry
                else begin
                    // Delete hostid from cxl table entry
                    mod_write_d.new_checkout = mod_hit_checkout & ~read_mod_q.req_node;
                    mod_write_d.new_valid = ((mod_hit_checkout & ~read_mod_q.req_node) != '0);
                end
            end
            default: begin
                mod_write_d.new_checkout = mod_hit_checkout;
                mod_write_d.new_inprog = mod_hit_inprog;
                mod_write_d.new_valid = mod_any_hit;
            end
        endcase

        // Drive hit outputs for LOAD
        hit_o = mod_any_hit;
        check_out_o = mod_hit_checkout;
        in_progress_o = mod_hit_inprog;
    end else begin
        // Nothing more to do for this request
        mod_write_d.valid = 1'b0;
    end

    // ================ WRITE stage ================
    if (mod_write_q.valid) begin
        set_index = mod_write_q.set_index;
        if (write_way == 1'b0) begin
            ce_way0 = 1'b1; // write not read
            we_way0 = 1'b1;
            wdata_way0 = write_packed_word;
        end else begin
            ce_way1 = 1'b1;
            we_way1 = 1'b1;
            wdata_way1 = write_packed_word;
        end
    end
end

// ================ Sequential Logic ================
always_ff @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        seq_state_q <= SEQ_IDLE;
        seq_ptr_q <= '0;
        read_mod_q <= '0;
        mod_write_q <= '0;
        lat_req_type_q <= '0;
        lat_req_node_q <= '0;
        lat_load_addr_q <= '0;
        lat_rel_valid_q <= '0;
        lat_rel_is_write_q <= '0;
        lat_rel_is_read_q <= '0;
        lat_rel_addr_q <= '0;
        lat_rel_data_q <= '0;
        any_conflict_q <= '0;

        for (int i = 0; i < NUM_SETS; i++) lru_q[i] <= 1'b0;
    end else begin
        seq_state_q <= seq_state_d;
        seq_ptr_q <= seq_ptr_d;
        read_mod_q <= read_mod_d;
        mod_write_q <= mod_write_d;

        if (seq_state_q == SEQ_IDLE && controller_valid_i) begin
            lat_req_type_q <= req_type_i;
            lat_req_node_q <= req_node_i;
            lat_load_addr_q <= addr_i;
            lat_rel_valid_q <= release_valid_i;
            lat_rel_is_write_q <= release_is_write_i;
            lat_rel_is_read_q <= release_is_read_i;
            lat_rel_addr_q <= release_addr_i;
            lat_rel_data_q <= release_data_i;
            any_conflict_q  <= 1'b0;
        end

        // Set and keep any_conflict_q as 1 whenever a conflict is detected
        if (read_mod_q.valid && read_mod_q.req_type == CMD_TX_COMMIT && entry_is_write_conflict)
            any_conflict_q <= any_conflict_q | 1'b1;
        
        // Update LRU on any MOD stage hit
        if (read_mod_q.valid && mod_any_hit)
            lru_q[read_mod_q.set_index] <= mod_hit_way;
    end
end

always @(posedge clk_i) begin
    if (!rst_i) begin
        if (controller_valid_i)
            $display("[%0t] CXL TABLE: request received (cmd=%0d node=%0b addr=%0h)",
                $time, req_type_i, req_node_i, addr_i);
        if (req_compl_o)
            $display("[%0t] CXL TABLE: request complete", $time);
    end
end

always @(posedge clk_i) begin
    if (!rst_i) begin
        if (seq_state_q == SEQ_READ)
            $display("[%0t] CXL TABLE [SEQ_READ]: ptr=%0d cur_addr=%0h cur_set=%0d cur_tag=%0h",
                $time, seq_ptr_q, cur_addr, cur_set_index, cur_tag);
        if (read_mod_q.valid)
            $display("[%0t] CXL TABLE [MOD]:      cmd=%0d node=%0b tag=%0h set=%0d hit=%0b checkout=%0b inprog=%0b",
                $time, read_mod_q.req_type, read_mod_q.req_node,
                read_mod_q.tag, read_mod_q.set_index,
                mod_any_hit, mod_hit_checkout, mod_hit_inprog);
        if (mod_write_q.valid)
            $display("[%0t] CXL TABLE [WRITE]:    way=%0b set=%0d valid=%0b tag=%0h new_checkout=%0b new_inprog=%0b",
                $time, write_way, mod_write_q.set_index,
                mod_write_q.new_valid, mod_write_q.tag,
                mod_write_q.new_checkout, mod_write_q.new_inprog);
        if (seq_state_q == SEQ_DRAIN)
            $display("[%0t] CXL TABLE [DRAIN]:    read_mod_valid=%0b mod_write_valid=%0b",
                $time, read_mod_q.valid, mod_write_q.valid);
        if (req_compl_o)
            $display("[%0t] CXL TABLE [DONE]",  $time);
    end
end

endmodule