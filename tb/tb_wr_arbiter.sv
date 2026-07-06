`timescale 1ns/1ps

module tb_wr_arbiter;

    localparam int DATA_W = 64;
    localparam int ADDR_W = 64;
    localparam int NUM_NODES = 4;
    localparam int RELEASE_SET_DEPTH = 16;
    localparam int DEPTH = 32;
    localparam logic [1:0] CMD_TX_COMMIT = 2'b10;
    localparam logic [1:0] CMD_LOAD = 2'b00;

    logic clk_i;
    logic rst_i;

    logic req_valid_i;
    logic [1:0] req_type_i;
    logic [DATA_W-1:0] load_addr_i;
    logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_i;
    logic [RELEASE_SET_DEPTH-1:0] release_is_write_i;
    logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_i;
    logic [NUM_NODES-1:0] worker_i;

    logic busy_o;
    logic done_o;

    logic push_ready_i;

    logic push_valid_o;
    logic push_we_o;
    logic [ADDR_W-1:0] push_addr_o;
    logic [DATA_W-1:0] push_wdata_o;
    logic [NUM_NODES-1:0] push_worker_o;
    logic [1:0] push_req_type_o;

    // ================================ DUT ================================
    wr_arbiter #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .NUM_NODES(NUM_NODES),
        .RELEASE_SET_DEPTH(RELEASE_SET_DEPTH),
        .DEPTH(DEPTH)
    ) dut (
        .clk_i(clk_i),
        .rst_i(rst_i),

        // Input from the cxl controller
        .req_valid_i(req_valid_i),
        .req_type_i(req_type_i),
        .load_addr_i(load_addr_i),
        .release_addr_i(release_addr_i),
        .release_is_write_i(release_is_write_i),
        .release_data_i(release_data_i),
        .worker_i(worker_i),

        // Output to the cxl controller
        .busy_o(busy_o),
        .done_o(done_o),

        // Input from the memory buffer
        .push_ready_i(push_ready_i),

        // Output to the memory buffer
        .push_valid_o(push_valid_o),
        .push_we_o(push_we_o),
        .push_addr_o(push_addr_o),
        .push_wdata_o(push_wdata_o),
        .push_worker_o(push_worker_o),
        .push_req_type_o(push_req_type_o)
    );
    localparam int CLK_PERIOD = 10;

    // Generate clock
    initial clk_i = 0;
    always #(CLK_PERIOD/2) clk_i = ~clk_i;

    // Reset function
    task automatic do_reset();
        rst_i = 1;
        repeat(2) @(posedge clk_i);
        #1; // slight skew so we're sampling after the edge
        rst_i = 0;
        @(posedge clk_i); #1;
    endtask

    // ================================ Tasks ================================
    // Idle defaults (drive before each test)
    task automatic idle_inputs();
        req_valid_i = 1'b0;
        req_type_i = CMD_LOAD; // benign
        load_addr_i = '0;
        release_addr_i = '0;
        release_is_write_i = '0;
        release_data_i = '0;
        worker_i = '0;
    endtask

    // push load
    task automatic push_load(
        input logic [DATA_W-1:0] load_addr,
        input logic [NUM_NODES-1:0] worker
    );
        // Wait for space
        while (!push_ready_i) begin
            @(posedge clk_i); #1;
        end
        #1;
        req_valid_i = 1;
        req_type_i = CMD_LOAD;
        load_addr_i = load_addr;
        worker_i = worker;

        @(posedge clk_i); #1;
        req_valid_i = 0;

    endtask

    // push commit
    task automatic push_commit(
        input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] addr,
        input logic [RELEASE_SET_DEPTH-1:0] is_write,
        input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] data,
        input logic [NUM_NODES-1:0] worker
    );
        // Wait for space
        while (!push_ready_i) begin
            @(posedge clk_i); #1;
        end

        #1;

        req_valid_i = 1;
        req_type_i = CMD_TX_COMMIT;
        release_addr_i = addr;
        release_is_write_i = is_write;
        release_data_i = data;
        worker_i = worker;

        @(posedge clk_i); #1;
        req_valid_i = 0;
    endtask

    // Spin until done_o pulses; fail loudly if it never comes
    task automatic wait_done(
        input int timeout_cycles
    );
        int count;
        for (count = 0; count < timeout_cycles; count++) begin
            @(posedge clk_i); 
            // #1; // sample after DUT nonblocking assignments settle

            if (done_o) begin
                return;
            end
        end
        $fatal(1, "done_o wait exceeded timeout_cycles=%0d", timeout_cycles);
    endtask

    // Static override of the downstream ready line (for startup)
    task automatic set_ready(
        input logic val
    );
        push_ready_i = val;
    endtask

    // Randomly toggles push_ready_i to create mid-burst bubbles
    task automatic drive_ready_random(
        input int seed,
        input int stall_pct // 0..100 chance of deasserting ready per cycle
    );
        int r;
        int roll;
        r = seed; // seed a local state for $random / $urandom
        forever begin
            @(posedge clk_i); #1;
            // roll 0..99, stall if below the threshold
            roll = {$random(r)} % 100; 
            if (roll < stall_pct)
                push_ready_i = 1'b0;
            else
                push_ready_i = 1'b1;
        end
    endtask

    // Fill the parallel release-set arrays with recognizable patterns
    task automatic make_commit(
        input int num_entries,
        input logic [RELEASE_SET_DEPTH-1:0] write_mask,
        output logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] addr,
        output logic [RELEASE_SET_DEPTH-1:0] is_write,
        output logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] data
    );
        int i;

        addr = '0;
        is_write = '0;
        data = '0;
        
        for(i = 0; i < num_entries; i++) begin
            addr[i] = ADDR_W'('hA000 + i); // [A000, A001, A002, A003, ...]
            data[i] = DATA_W'('hD000 + i); // [D000, D001, D002, D003, ...]
            is_write[i] = write_mask[i];
        end
    endtask

    // ================================ Scoreboard ================================
    // Check a single accepted beat against expected values.
    // Increments pass/fail counters (declared as TB-level ints).
    task automatic expect_beat(
        input logic exp_we,
        input logic [ADDR_W-1:0] exp_addr,
        input logic [DATA_W-1:0] exp_wdata,
        input logic [NUM_NODES-1:0] exp_worker,
        input logic [1:0] exp_type,
        input logic check_wdata // 0 for load because no write
    );
        // same flag as accept_beat in the engine
        if (push_valid_o && push_ready_i) begin
            if (exp_we !== push_we_o) begin
                $fatal(1, "release engine -> buffer pushed write enable incorrect: got %b exp %b @ %0t",
                       push_we_o, exp_we, $time);
            end
            if (exp_addr !== push_addr_o) begin
                $fatal(1, "release engine -> buffer pushed address incorrect: got %h exp %h @ %0t",
                       push_addr_o, exp_addr, $time);
            end
            if (check_wdata && (exp_wdata !== push_wdata_o)) begin
                $fatal(1, "release engine -> buffer pushed data incorrect: got %h exp %h @ %0t",
                       push_wdata_o, exp_wdata, $time);
            end
            if (exp_worker !== push_worker_o) begin
                $fatal(1, "release engine -> buffer pushed worker incorrect: got %b exp %b @ %0t",
                       push_worker_o, exp_worker, $time);
            end
            if (exp_type !== push_req_type_o) begin
                $fatal(1, "release engine -> buffer pushed request type incorrect: got %b exp %b @ %0t",
                       push_req_type_o, exp_type, $time);
            end
        end
    endtask

    logic                  exp_we_q     [$];
    logic [ADDR_W-1:0]     exp_addr_q   [$];
    logic [DATA_W-1:0]     exp_wdata_q  [$];
    logic [NUM_NODES-1:0]  exp_worker_q [$];
    logic [1:0]            exp_type_q   [$];
    logic                  exp_cw_q     [$];

    task automatic enq_beat(
        input logic exp_we,
        input logic [ADDR_W-1:0] exp_addr,
        input logic [DATA_W-1:0] exp_wdata,
        input logic [NUM_NODES-1:0] exp_worker,
        input logic [1:0] exp_type,
        input logic check_wdata
    );
        exp_we_q.push_back(exp_we);
        exp_addr_q.push_back(exp_addr);
        exp_wdata_q.push_back(exp_wdata);
        exp_worker_q.push_back(exp_worker);
        exp_type_q.push_back(exp_type);
        exp_cw_q.push_back(check_wdata);
    endtask

    int beats_seen;
    int pass_cnt;
    int fail_cnt;

    // Make sure every thing pushed into the buffer is correct
    task automatic push_monitor();
        forever begin
            @(posedge clk_i);
            if (push_valid_o && push_ready_i) begin // accepted beat
                beats_seen++;
                if (exp_we_q.size() == 0) begin
                    $fatal(1, "push_monitor: unexpected beat with empty expected queue @ %0t (addr=%h)",
                           $time, push_addr_o);
                end
                expect_beat(exp_we_q.pop_front(),
                        exp_addr_q.pop_front(),
                        exp_wdata_q.pop_front(),
                        exp_worker_q.pop_front(),
                        exp_type_q.pop_front(),
                        exp_cw_q.pop_front());
            end
        end
    endtask

    // ================================ Reporting ================================
    // Print pass/fail totals; $finish with nonzero on failure
    task automatic report();
        $display("==================================================");
        $display(" wr_arbiter TB summary");
        $display("   beats observed : %0d", beats_seen);
        $display("   pass_cnt       : %0d", pass_cnt);
        $display("   fail_cnt       : %0d", fail_cnt);
        $display("   leftover exp_q : %0d", exp_we_q.size());
        $display("==================================================");

        if (fail_cnt != 0 || exp_we_q.size() != 0) begin
            $display(" RESULT: FAIL");
            $fatal(1, "wr_arbiter TB failed");
        end else begin
            $display(" RESULT: PASS");
            $finish;
        end
    endtask


    // ================ Main Testing Sequence ================
    initial begin
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] c_addr;
        logic [RELEASE_SET_DEPTH-1:0] c_isw;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] c_data;
        logic [NUM_NODES-1:0] wrk;
        int i;
        int prev_beats;

        // ---- waveform dump ----
        $dumpfile("tb_wr_arbiter.vcd");
        $dumpvars(0, tb_wr_arbiter);

        // ---- init ----
        pass_cnt = 0;
        fail_cnt = 0;
        beats_seen = 0;
        idle_inputs();
        push_ready_i = 1'b0;
        do_reset();
        set_ready(1'b1); // baseline: buffer always accepts

        // ---- launch the scoreboard monitor ----
        fork
            push_monitor();
        join_none

        // ======================================================
        // Test 1: single LOAD  (one beat, we=0, type=LOAD)
        // ======================================================
        $display("[T1] single load");
        // stuff to push
        wrk = 4'b0010; // node 1
        enq_beat(1'b0, 64'h0000_0000_0000_1111, '0, wrk, CMD_LOAD, 1'b0);
        // push
        push_load(64'h0000_0000_0000_1111, wrk);
        wait_done(50);
        idle_inputs();
        repeat(3) @(posedge clk_i);

        // ======================================================
        // Test 2: COMMIT, dense (all writes)  8 entries, all write
        // ======================================================
        $display("[T2] commit all-writes");
        wrk = 4'b0001; // node 0
        make_commit(8, 16'h00FF, c_addr, c_isw, c_data);
        for (i = 0; i < RELEASE_SET_DEPTH; i++) begin
            if (c_isw[i]) begin
                enq_beat(1'b1, c_addr[i], c_data[i], wrk, CMD_TX_COMMIT, 1'b1);
            end
        end
        push_commit(c_addr, c_isw, c_data, wrk);
        wait_done(200);
        idle_inputs();
        repeat(3) @(posedge clk_i);

        // ======================================================
        // Test 3: COMMIT, sparse (writes interleaved with reads)
        //   6 entries, writes at indices 1,3,5 -> mask 0x2A
        //   exercises first_write_idx skip + next_write_idx skipping reads
        // ======================================================
        $display("[T3] commit sparse-writes");
        wrk = 4'b0100; // node 2
        make_commit(6, 16'h002A, c_addr, c_isw, c_data);
        for (i = 0; i < RELEASE_SET_DEPTH; i++) begin
            if (c_isw[i]) begin
                enq_beat(1'b1, c_addr[i], c_data[i], wrk, CMD_TX_COMMIT, 1'b1);
            end
        end
        push_commit(c_addr, c_isw, c_data, wrk);
        wait_done(200);
        idle_inputs();
        repeat(3) @(posedge clk_i);

        // ======================================================
        // Test 4: read-only COMMIT (no writes)
        //   Engine must NOT go busy and must push ZERO beats.
        //   NOTE: done_o never asserts here (busy_q never set), so we do
        //         NOT call wait_done -- we time a fixed window and check
        //         that nothing was pushed. See flag below.
        // ======================================================
        $display("[T4] commit read-only");
        wrk = 4'b1000;               // node 3
        prev_beats = beats_seen;
        make_commit(4, 16'h0000, c_addr, c_isw, c_data);   // all reads
        push_commit(c_addr, c_isw, c_data, wrk);
        repeat(8) @(posedge clk_i);
        if (beats_seen != prev_beats)
            $fatal(1, "[T4] read-only commit pushed %0d unexpected beat(s)",
                   beats_seen - prev_beats);
        if (busy_o)
            $fatal(1, "[T4] busy_o asserted on read-only commit");
        idle_inputs();
        repeat(3) @(posedge clk_i);

        // ======================================================
        // Test 5: COMMIT under mid-burst backpressure
        //   Launch a dense commit, then stall push_ready_i for a few
        //   cycles (simulate full buffer). Engine must hold release_ptr_q and re-drive the
        //   same beat -- no beat dropped or duplicated.
        // ======================================================
        $display("[T5] commit with backpressure");
        wrk = 4'b0001;
        make_commit(8, 16'h00FF, c_addr, c_isw, c_data);
        for (i = 0; i < RELEASE_SET_DEPTH; i++) begin
            if (c_isw[i]) begin
                enq_beat(1'b1, c_addr[i], c_data[i], wrk, CMD_TX_COMMIT, 1'b1);
            end
        end
        push_commit(c_addr, c_isw, c_data, wrk); // accepted at ready=1
        set_ready(1'b0); // jam the buffer
        repeat(4) @(posedge clk_i); #1;
        set_ready(1'b1); // release

        wait_done(200);
        idle_inputs();
        repeat(3) @(posedge clk_i);

        // ======================================================
        // Test 6: back-to-back LOAD then COMMIT (pipeline/ptr reset)
        // ======================================================
        $display("[T6] load then commit, back to back");
        wrk = 4'b0010;
        enq_beat(1'b0, 64'h0000_0000_0000_2222, '0, wrk, CMD_LOAD, 1'b0);
        push_load(64'h0000_0000_0000_2222, wrk);
        wait_done(50);
        @(posedge clk_i); // let load busy_q fall before next job
        idle_inputs();

        // no long gap -- immediately issue a commit
        wrk = 4'b0100;
        make_commit(4, 16'h000F, c_addr, c_isw, c_data);
        for (i = 0; i < RELEASE_SET_DEPTH; i++) begin
            if (c_isw[i])
                enq_beat(1'b1, c_addr[i], c_data[i], wrk, CMD_TX_COMMIT, 1'b1);
        end
        push_commit(c_addr, c_isw, c_data, wrk);
        wait_done(200);
        @(posedge clk_i); // settle before final idle
        idle_inputs();

        // ---- drain & report ----
        repeat(10) @(posedge clk_i);
        report();
    end

endmodule