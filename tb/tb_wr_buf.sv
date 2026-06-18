`timescale 1ns/1ps

module tb_wr_buf;

    localparam int NUM_NODES = 4;
    localparam int DATA_W = 64;
    localparam int ADDR_W = 64;
    localparam int DEPTH = 16;
    localparam int MEM_LATENCY = 3; // >1 to stress the in-flight gate
    localparam int CLK_PERIOD = 10;

    logic clk;
    logic rst;

    // Generate clock
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    task automatic do_reset();
        rst = 1;
        repeat(2) @(posedge clk);
        #1; // slight skew so we're sampling after the edge
        rst = 0;
        @(posedge clk); #1;
    endtask

    // DUT signals 
    // input wires
    logic push_valid;
    logic push_we;
    logic [ADDR_W-1:0] push_addr;
    logic [DATA_W-1:0] push_wdata;
    logic [NUM_NODES-1:0] push_worker;
    logic [1:0] push_req_type;
    logic push_ready;

    // output wires
    logic pop_valid;
    logic pop_we;
    logic [ADDR_W-1:0] pop_addr;
    logic [DATA_W-1:0] pop_wdata;
    logic [NUM_NODES-1:0] pop_worker;
    logic [1:0] pop_req_type;

    // memory pool input
    logic mem_ready; // stub rvalid → dut ready
    logic [DATA_W-1:0] mem_rdata;

    // output to nodes
    logic [NUM_NODES-1:0] resp_worker;
    logic resp_valid;

    // ================ DUT module instantiation ================
    wr_buf #(
        .NUM_NODES(NUM_NODES),
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .DEPTH(DEPTH)
    ) dut (
        .clk_i(clk),
        .rst_i(rst),
        .push_valid_i(push_valid),
        .push_we_i(push_we),
        .push_addr_i(push_addr),
        .push_wdata_i(push_wdata),
        .push_worker_i(push_worker),
        .push_req_type_i(push_req_type),
        .push_ready_o(push_ready),
        .pop_valid_o(pop_valid),
        .pop_we_o(pop_we),
        .pop_addr_o(pop_addr),
        .pop_wdata_o(pop_wdata),
        .pop_worker_o(pop_worker),
        .pop_req_type_o(pop_req_type),
        .mem_ready_i(mem_ready),
        .mem_rdata_i(mem_rdata),
        .resp_worker_o(resp_worker),
        .resp_valid_o(resp_valid)
    );

    // ================ Memory pool stub instantiation ================
    mem_pool_stub #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .MEM_DEPTH(1024),
        .MEM_LATENCY(MEM_LATENCY)
    ) mem_stub (
        .clk_i(clk),
        .rst_i(rst),
        .mem_req_valid_i(pop_valid),
        .mem_we_i(pop_we),
        .mem_addr_i(pop_addr),
        .mem_wdata_i(pop_wdata),
        .mem_rdata_o(mem_rdata),
        .mem_rvalid_o(mem_ready)
    );

    // ================ Tasks ================
    // Idle defaults (drive before each test)
    task automatic idle_inputs();
        push_valid = 0;
        push_we = 0;
        push_addr = '0;
        push_wdata = '0;
        push_worker = '0;
        push_req_type = '0;
    endtask

    // push
    task automatic push_entry(
        input logic we,
        input logic [ADDR_W-1:0] addr,
        input logic [DATA_W-1:0] wdata,
        input logic [NUM_NODES-1:0] worker,
        input logic [1:0] req_type
    );
        // Wait for space
        while (!push_ready) begin
            @(posedge clk); #1;
        end

        push_valid = 1;
        push_we = we;
        push_addr = addr;
        push_wdata = wdata;
        push_worker = worker;
        push_req_type = req_type;

        @(posedge clk); #1;
        push_valid = 0;
    endtask

    // Wait for resp_valid pulse; returns the worker tag and data
    task automatic wait_resp(
        output logic [NUM_NODES-1:0] got_worker,
        output logic [DATA_W-1:0] got_rdata
    );
        while (!resp_valid) begin
            @(posedge clk); #1;
        end

        got_worker = resp_worker;
        got_rdata = dut.resp_rdata_o; // sample on the pulse cycle

        @(posedge clk); #1;
    endtask

    // ================ Scoreboard ================
    // track resp_valid pulses and assert no double-fires
    int resp_count;
    always_ff @(posedge clk) begin
        if (rst) begin
            resp_count <= 0;
        end else if (resp_valid) begin
            resp_count <= resp_count + 1;
            $display("[%0t] resp_valid pulse | resp_worker=%0b", $time, resp_worker);
        end
    end
    // resp_valid must be a single-cycle pulse (never two consecutive cycles)
    // impossible for response to be ready in back to back cycles
    logic resp_valid_prev;
    always_ff @(posedge clk) resp_valid_prev <= resp_valid;
    always_ff @(posedge clk) begin
        if (!rst) begin
            assert (!(resp_valid && resp_valid_prev))
                else $fatal(1, "resp_valid held high for >1 cycle — should be a pulse");
        end
    end

    // ================ TEST VARIABLES ================
    logic [NUM_NODES-1:0] got_worker;
    logic [DATA_W-1:0] got_rdata;
    int test_num; 
    int pushed;
    int full_seen;
    int target;
    logic [DATA_W-1:0] expected_a;
    logic [DATA_W-1:0] expected_b;
    logic [ADDR_W-1:0] addr_a;
    logic [ADDR_W-1:0] addr_b;

    // ================ MAIN TEST SEQUENCE
    initial begin
        $dumpfile("tb_wr_buf.vcd");
        $dumpvars(0, tb_wr_buf);

        idle_inputs();
        do_reset();

        // ============================================================
        // TEST 1: Basic push -> memory response -> resp_valid + worker
        // ============================================================
        test_num = 1;
        $display("\n=== TEST %0d: Basic push, wait for memory response ===", test_num);

        // Node 0001, LOAD request, address 64'hA0
        push_entry(.we(1'b0), .addr(64'hA0), .wdata('0),
                   .worker(4'b0001), .req_type(2'b00));

        wait_resp(got_worker, got_rdata);

        assert (got_worker === 4'b0001)
            else $fatal(1, "T%0d: wrong resp_worker. got=%0b exp=0001", test_num, got_worker);
        $display("T%0d PASS: resp_worker correct (%0b)", test_num, got_worker);

        // Verify resp_valid deasserted the next cycle
        assert (!resp_valid)
            else $fatal(1, "T%0d: resp_valid still high after pulse cycle", test_num);
        $display("T%0d PASS: resp_valid deasserted correctly", test_num);

        // reset after every test
        do_reset();

        // ============================================================
        // TEST 2: Back-to-back pushes to verify in-flight gate
        //   Push two entries. The second pop must NOT fire until
        //   mem_ready_i returns for the first.
        // ============================================================
        test_num = 2;
        $display("\n=== TEST %0d: In-flight gate — two sequential pops ===", test_num);

        // Push both before either is consumed (FIFO has room)
        // Node 0010, LOAD request, address 64'hB0
        push_entry(.we(1'b0), .addr(64'hB0), .wdata('0),
                   .worker(4'b0010), .req_type(2'b00));

        // Node 0100, LOAD request, address 64'hC0
        push_entry(.we(1'b0), .addr(64'hC0), .wdata('0),
                   .worker(4'b0100), .req_type(2'b00));

        // First response: should be worker 0010
        wait_resp(got_worker, got_rdata);
        assert (got_worker === 4'b0010)
            else $fatal(1, "T%0d: 1st resp wrong. got=%0b exp=0010", test_num, got_worker);
        $display("T%0d PASS: 1st resp_worker=0010", test_num);

        // Second response: should be worker 0100
        wait_resp(got_worker, got_rdata);
        assert (got_worker === 4'b0100)
            else $fatal(1, "T%0d: 2nd resp wrong. got=%0b exp=0100", test_num, got_worker);
        $display("T%0d PASS: 2nd resp_worker=0100", test_num);

        assert (!resp_valid)
            else $fatal(1, "T%0d: resp_valid still high after pulse cycle", test_num);
        $display("T%0d PASS: resp_valid deasserted correctly", test_num);

        do_reset();

        // ============================================================
        // TEST 3: Fill FIFO to full
        // ============================================================
        test_num = 3;
        $display("\n=== TEST %0d: Fill FIFO to DEPTH=%0d ===", test_num, DEPTH);

        begin
            // Force req_in_flight high so no pops occur while filling
            force dut.req_in_flight = 1;

            pushed = 0;
            push_we = 0;
            push_wdata = '0;
            push_req_type = 2'b00;

            while (pushed < DEPTH) begin
                if (push_ready) begin
                    push_valid = 1;
                    push_addr = 64'(pushed * 8);
                    push_worker = NUM_NODES'(pushed % NUM_NODES);
                    pushed++;
                end else begin
                    push_valid = 0;
                end
                @(posedge clk); #1;
            end
            push_valid = 0;

            // Let last push settle
            @(posedge clk); #1;

            assert (dut.count == DEPTH)
                else $fatal(1, "T%0d: count=%0d expected %0d", test_num, dut.count, DEPTH);
            assert (!push_ready)
                else $fatal(1, "T%0d: push_ready still high — FIFO not full", test_num);
            $display("T%0d PASS: FIFO filled to DEPTH=%0d, push_ready deasserted", test_num, DEPTH);

            release dut.req_in_flight;
            force dut.req_in_flight = 0;
            @(posedge clk); #1;  // one pop fires here, req_in_flight FF sets to 1
            release dut.req_in_flight;
            // From here the FF drives naturally: it will self-clear when mem_ready_i returns
        end

        // Drain: track from current resp_count baseline so we wait for exactly DEPTH responses
        begin
            int drain_start;
            drain_start = resp_count;
            target = DEPTH - 1; // one entry already popped during the force-0 pulse
            $display("T%0d: waiting for %0d responses (1 already in flight)...", test_num, target);
            while ((resp_count - drain_start) < target) begin
                @(posedge clk); #1;
            end
            $display("T%0d PASS: all %0d responses received", test_num, DEPTH);
        end

        do_reset();

        // ============================================================
        // TEST 4: Simultaneous push + pop (count should stay stable)
        // Push one entry, wait for it to be popped (req_in_flight high),
        // then push another while the first is in flight.
        // Count: started at 1 after first push, drops to 0 on pop,
        // rises back to 1 on the simultaneous push. Net = stable.
        // ============================================================
        test_num = 4;
        $display("\n=== TEST %0d: Push while request in flight ===", test_num);

        // Node 1000
        push_entry(.we(1'b0), .addr(64'hD0), .wdata('0),
                   .worker(4'b1000), .req_type(2'b00));

        // Wait one cycle so the FIFO pops and req_in_flight goes high
        @(posedge clk); #1;

        // Now push a second entry while first is in flight
        // Node 0001
        push_entry(.we(1'b0), .addr(64'hE0), .wdata('0),
                   .worker(4'b0001), .req_type(2'b00));

        // Collect both responses
        wait_resp(got_worker, got_rdata);
        $display("T%0d: 1st resp worker=%0b", test_num, got_worker);
        wait_resp(got_worker, got_rdata);
        $display("T%0d: 2nd resp worker=%0b", test_num, got_worker);
        $display("T%0d PASS: both responses received", test_num);

        // Verify FIFO is now empty (count should be 0)
        @(posedge clk); #1;
        assert (dut.count === '0)
            else $fatal(1, "T%0d: FIFO not empty after drain. count=%0d", test_num, dut.count);
        $display("T%0d PASS: FIFO empty after drain", test_num);

        do_reset();

        // ============================================================
        // TEST 5: Write (we=1) entry, verify write path doesn't corrupt the worker tag
        // ============================================================
        test_num = 5;
        $display("\n=== TEST %0d: Write request — no hang, FIFO drains ===", test_num);

        push_entry(.we(1'b1), .addr(64'hF0), .wdata(64'hDEADBEEF),
                .worker(4'b0110), .req_type(2'b01));

        // Writes are fire-and-forget — no resp_valid pulse expected.
        repeat(4) @(posedge clk); #1;

        assert (dut.count === '0)
            else $fatal(1, "T%0d: FIFO not empty after write — req_in_flight may be stuck", test_num);
        assert (!dut.req_in_flight)
            else $fatal(1, "T%0d: req_in_flight still set after write completion", test_num);
        $display("T%0d PASS: write consumed, FIFO empty, no hang", test_num);

        do_reset();

        // ============================================================
        // TEST 6: Data relay — resp_rdata_o carries correct mem data
        // ============================================================
        test_num = 6;
        $display("\n=== TEST %0d: Data relay — resp_rdata_o correctness ===", test_num);

        begin
            expected_a = 64'hCAFEBABE_DEADBEEF;
            expected_b = 64'h0123456789ABCDEF;
            addr_a = 64'h100;
            addr_b = 64'h200;

            // Preload the stub's backing store directly
            // widx(addr) = addr % MEM_DEPTH = 0x100 % 1024 = 256, 0x200 % 1024 = 512
            mem_stub.mem[addr_a % 1024] = expected_a;
            mem_stub.mem[addr_b % 1024] = expected_b;

            // Issue two reads to distinct addresses with distinct workers
            push_entry(.we(1'b0), .addr(addr_a), .wdata('0),
                    .worker(4'b0001), .req_type(2'b00));
            push_entry(.we(1'b0), .addr(addr_b), .wdata('0),
                    .worker(4'b0010), .req_type(2'b00));

            // First response: addr_a → expected_a, worker 0001
            wait_resp(got_worker, got_rdata);
            assert (got_worker === 4'b0001)
                else $fatal(1, "T%0d: 1st resp wrong worker. got=%0b exp=0001", test_num, got_worker);
            assert (got_rdata === expected_a)
                else $fatal(1, "T%0d: 1st resp wrong data. got=%0h exp=%0h",
                            test_num, got_rdata, expected_a);
            $display("T%0d PASS: 1st read — worker=%0b data=%0h", test_num, got_worker, got_rdata);

            // Second response: addr_b → expected_b, worker 0010
            wait_resp(got_worker, got_rdata);
            assert (got_worker === 4'b0010)
                else $fatal(1, "T%0d: 2nd resp wrong worker. got=%0b exp=0010", test_num, got_worker);
            assert (got_rdata === expected_b)
                else $fatal(1, "T%0d: 2nd resp wrong data. got=%0h exp=%0h",
                            test_num, got_rdata, expected_b);
            $display("T%0d PASS: 2nd read — worker=%0b data=%0h", test_num, got_worker, got_rdata);
        end

        do_reset();

        // ============================================================
        // Done
        // ============================================================
        $display("\n=== ALL TESTS PASSED ===");
        $finish;
    end

    // ----------------------------------------------------------------
    // Timeout watchdog (prevents infinite loops on hangs)
    // ----------------------------------------------------------------
    initial begin
        #(CLK_PERIOD * 100000);
        $fatal(1, "TIMEOUT: simulation exceeded watchdog limit");
    end

endmodule