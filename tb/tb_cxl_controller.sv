`timescale 1ns/1ps

module tb_cxl_controller;

    localparam int DATA_W = 64;
    localparam int ADDR_W = 64;
    localparam int NUM_NODES = 4;
    localparam int RELEASE_SET_DEPTH = 16;
    localparam int CXL_TABLE_DEPTH = 32;
    localparam int BUF_DEPTH = 16;
    localparam int MEM_DEPTH = 1024;
    localparam int MEM_LATENCY = 1;

    // Command encodings
    localparam logic [1:0] CMD_LOAD = 2'b00;
    localparam logic [1:0] CMD_TX_ABORT  = 2'b01;
    localparam logic [1:0] CMD_TX_COMMIT = 2'b10;

    logic clk, rst;

    logic [NUM_NODES-1:0] req_valid;
    logic [1:0] tx_signal;
    logic [ADDR_W-1:0] load_addr;
    logic [RELEASE_SET_DEPTH-1:0] release_is_write;
    logic [RELEASE_SET_DEPTH-1:0] release_is_read;
    logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr;
    logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data;

    logic [NUM_NODES-1:0] req_ready;

    // Controller-driven completion (TX_ABORT / TX_COMMIT)
    logic [NUM_NODES-1:0] ctrl_resp_valid;
    logic [1:0] ctrl_comp_signal;

    // wr_buf-driven completion (LOAD only)
    logic buf_resp_valid;
    logic [NUM_NODES-1:0] buf_resp_worker;
    logic [DATA_W-1:0] buf_resp_rdata;

    // Self-check bookkeeping
    integer errors = 0;

    // DUT
    top #(
        .DATA_W(DATA_W), 
        .ADDR_W(ADDR_W), 
        .NUM_NODES(NUM_NODES),
        .RELEASE_SET_DEPTH(RELEASE_SET_DEPTH), 
        .CXL_TABLE_DEPTH(CXL_TABLE_DEPTH),
        .BUF_DEPTH(BUF_DEPTH),
        .MEM_DEPTH(MEM_DEPTH), 
        .MEM_LATENCY(MEM_LATENCY)
    ) top_inst (
        .clk_i(clk),
        .rst_i(rst),
        .req_valid_i(req_valid),
        .tx_signal_i(tx_signal),
        .load_addr_i(load_addr),
        .release_is_write_i(release_is_write),
        .release_is_read_i(release_is_read),
        .release_addr_i(release_addr), 
        .release_data_i(release_data),
        .req_ready_o(req_ready),
        .ctrl_resp_valid_o(ctrl_resp_valid),
        .ctrl_comp_signal_o(ctrl_comp_signal),
        .buf_resp_valid_o(buf_resp_valid),
        .buf_resp_worker_o(buf_resp_worker),
        .buf_resp_rdata_o(buf_resp_rdata)
    );

    // Clock generation and watchdog
    initial begin
        clk = 1'b0;
        $display("[0] TB: Clock Instantiated");
        #6000;   // widened from 2000: more stimulus + inter-test settle waits
        $display("[%0t] TB: TIMEOUT", $time);
        $finish;
    end
    always #5 clk = ~clk; // 10ns clock period

    // Passive response monitor. Encodings of ctrl_comp_signal for commit vs abort
    // are not asserted here (RTL-dependent); memory readback is the authoritative
    // protocol check. This just gives visibility into responses and load returns.
    always @(posedge clk) begin
        if (!rst) begin
            if (ctrl_resp_valid != '0)
                $display("[%0t] TB: MON ctrl_resp_valid=%b comp_signal=%0d",
                         $time, ctrl_resp_valid, ctrl_comp_signal);
            if (buf_resp_valid)
                $display("[%0t] TB: MON buf_resp worker=%b rdata=%h",
                         $time, buf_resp_worker, buf_resp_rdata);
        end
    end

    // Memory readback check. Success commits must land the write; failed commits
    // must leave the preload value untouched.
    task automatic expect_mem(input logic [ADDR_W-1:0] a, input logic [DATA_W-1:0] exp);
        if (top_inst.mem.mem[a] === exp)
            $display("[%0t] TB: PASS mem[%0d] = %h", $time, a, top_inst.mem.mem[a]);
        else begin
            $display("[%0t] TB: FAIL mem[%0d] = %h (expected %h)",
                     $time, a, top_inst.mem.mem[a], exp);
            errors = errors + 1;
        end
    endtask

    // Drive a cxl request (LOAD, Tx_Commit, or Tx_Abort) and complete the valid/ready handshake.
    // Watches the correct response path based on which command was issued:
    //   CMD_LOAD       -> buf_resp_valid / buf_resp_worker / buf_resp_rdata (from wr_buf)
    //   CMD_TX_ABORT   -> ctrl_resp_valid / ctrl_comp_signal (from controller)
    task automatic issue_request(
        input logic [1:0] cmd,
        input logic [NUM_NODES-1:0] node,
        input logic [ADDR_W-1:0] addr
    );
        @(negedge clk);
        while ((req_ready & node) == '0) @(negedge clk);  // wait until this node is ready

        req_valid = node;
        tx_signal = cmd;
        load_addr = addr;
        $display("[%0t] TB: request issued (cmd=%0d node=%b addr=%0d)",
                $time, cmd, node, addr);

        @(posedge clk); // controller accepts here (req_valid & req_ready)
        @(negedge clk);
        req_valid = '0;
    endtask

    initial begin
        // Init all inputs
        req_valid = '0; 
        tx_signal = 0; 
        load_addr = 0;
        release_is_write = 0; 
        release_is_read = 0;
        release_addr = 0; 
        release_data = 0;

        // Reset
        rst = 1'b1;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // Preload memory — the hierarchy goes through wr_buf, so the memory stub
        // is at top_inst.mem (path unchanged).
        top_inst.mem.mem[5]  = 64'hDEAD_BEEF_0000_0001;
        top_inst.mem.mem[10] = 64'hDEAD_BEEF_0000_0002;
        top_inst.mem.mem[15] = 64'hDEAD_BEEF_0000_0003;
        top_inst.mem.mem[20] = 64'hDEAD_BEEF_0000_0004; // write target, success (write+read)
        top_inst.mem.mem[25] = 64'hDEAD_BEEF_0000_0005; // write target, fail (write-only)
        top_inst.mem.mem[30] = 64'hDEAD_BEEF_0000_0006; // write target, fail (write+read)
        top_inst.mem.mem[35] = 64'hDEAD_BEEF_0000_0007; // read target, fail (write+read)
        $display("[%0t] TB: preloaded mem[5]  = %h", $time, top_inst.mem.mem[5]);
        $display("[%0t] TB: preloaded mem[10] = %h", $time, top_inst.mem.mem[10]);
        $display("[%0t] TB: preloaded mem[15] = %h", $time, top_inst.mem.mem[15]);
        $display("[%0t] TB: preloaded mem[20] = %h", $time, top_inst.mem.mem[20]);
        $display("[%0t] TB: preloaded mem[25] = %h", $time, top_inst.mem.mem[25]);
        $display("[%0t] TB: preloaded mem[30] = %h", $time, top_inst.mem.mem[30]);
        $display("[%0t] TB: preloaded mem[35] = %h", $time, top_inst.mem.mem[35]);

        issue_request(CMD_LOAD, 4'b0001, 64'd5);
        issue_request(CMD_LOAD, 4'b0001, 64'd10);

        // Then abort with the two reads in the release set
        begin
            release_is_read = '0;

            release_is_read[0] = 1'b1;
            release_addr[0] = 64'd5;
            // release_is_read[1] = 1'b1;
            // release_addr[1] = 64'd10;

            release_data = '0;

            issue_request(CMD_TX_ABORT, 4'b0001, 64'd0);

            release_is_read  = '0;
            release_addr = '0;
            release_data = '0;
        end

        // ------------------------------------------------------------------
        // Successful Commit with only .write
        // addr 5 and 10 were cleared by the abort above -> no checkouts ->
        // no conflict -> writes applied.
        // ------------------------------------------------------------------
        begin
            release_is_write = '0;

            release_is_write[0] = 1'b1;
            release_addr[0] = 64'd5;
            release_data[0] = 64'hAAAA_BBBB_CCCC_DDDD;

            release_is_write[1] = 1'b1;
            release_addr[1] = 64'd10;
            release_data[1] = 64'hEEEE_EEEE_EEEE_EEEE;

            issue_request(CMD_TX_COMMIT, 4'b0001, 64'd0);

            release_is_write = '0;
            release_addr = '0;
            release_data = '0;
        end
        repeat (20) @(posedge clk); #1;
        expect_mem(64'd5,  64'hAAAA_BBBB_CCCC_DDDD);
        expect_mem(64'd10, 64'hEEEE_EEEE_EEEE_EEEE);

        // ------------------------------------------------------------------
        // Successful Commit with .write and .read variables
        // node0 loads addr15 (its read); write addr20 is write-only and held by
        // nobody -> no conflict -> commit succeeds. Read release clears node0
        // from addr15.
        // ------------------------------------------------------------------
        begin
            // read variable must be checked out by this node first
            issue_request(CMD_LOAD, 4'b0001, 64'd15);

            release_is_write = '0;
            release_is_read  = '0;
            release_addr     = '0;
            release_data     = '0;

            release_is_write[0] = 1'b1;              // write: addr 20
            release_addr[0]     = 64'd20;
            release_data[0]     = 64'h1111_2222_3333_4444;

            release_is_read[1]  = 1'b1;              // read: addr 15
            release_addr[1]     = 64'd15;

            issue_request(CMD_TX_COMMIT, 4'b0001, 64'd0);

            release_is_write = '0;
            release_is_read  = '0;
            release_addr     = '0;
            release_data     = '0;
        end
        repeat (20) @(posedge clk); #1;
        expect_mem(64'd20, 64'h1111_2222_3333_4444);

        // ------------------------------------------------------------------
        // Failed Commit with only .write
        // node1 loads addr25 (foreign checkout); node0 commits a write to addr25
        // -> conflict -> abort. mem[25] must remain the preload value.
        // ------------------------------------------------------------------
        begin
            // create the conflict: another node holds the write variable
            issue_request(CMD_LOAD, 4'b0010, 64'd25);
 
            release_is_write = '0;
            release_is_read  = '0;
            release_addr     = '0;
            release_data     = '0;
 
            release_is_write[0] = 1'b1;
            release_addr[0] = 64'd25;
            release_data[0] = 64'h5555_6666_7777_8888;
 
            issue_request(CMD_TX_COMMIT, 4'b0001, 64'd0);
 
            release_is_write = '0;
            release_addr     = '0;
            release_data     = '0;
        end
        repeat (20) @(posedge clk);
        expect_mem(64'd25, 64'hDEAD_BEEF_0000_0005); // unchanged -> abort honored

        // ------------------------------------------------------------------
        // Failed Commit with .write and .read variables
        // node0 loads addr35 (its read); node1 loads addr30 (foreign checkout on
        // the write); node0 commits write addr30 + read addr35 -> conflict on
        // addr30 -> abort. mem[30] must remain the preload value.
        // ------------------------------------------------------------------
        begin
            issue_request(CMD_LOAD, 4'b0001, 64'd35); // node0's read
            issue_request(CMD_LOAD, 4'b0010, 64'd30); // node1's conflicting checkout

            release_is_write = '0;
            release_is_read  = '0;
            release_addr     = '0;
            release_data     = '0;

            release_is_write[0] = 1'b1;              // write: addr 30 (conflicted)
            release_addr[0]     = 64'd30;
            release_data[0]     = 64'h9999_AAAA_BBBB_CCCC;

            release_is_read[1]  = 1'b1;              // read: addr 35
            release_addr[1]     = 64'd35;

            issue_request(CMD_TX_COMMIT, 4'b0001, 64'd0);

            release_is_write = '0;
            release_is_read  = '0;
            release_addr     = '0;
            release_data     = '0;
        end
        repeat (20) @(posedge clk); #1;
        expect_mem(64'd30, 64'hDEAD_BEEF_0000_0006); // unchanged -> abort honored

        top_inst.mem.mem[8]  = 64'hCAFE_0000_0000_0008; // E0 read way, set 8  (clean)
        top_inst.mem.mem[12] = 64'hCAFE_0000_0000_000C; // E1 read way, set 12 (filler)
        top_inst.mem.mem[16] = 64'hDEAD_BEEF_0000_0016; // E2 write target, set 16 -- must NOT change

        // ==================================================================
        // 3-ENTRY PORT-COLLISION PROBE (deep overlap: SEQ_READ(E2) || WRITE(E0))
        //   E0: read  addr 8  -> set 8   (node0's own checkout; CLEAN set)
        //   E1: read  addr 12 -> set 12  (filler, keeps pipeline 3-deep)
        //   E2: WRITE addr 16 -> set 16  (node1 holds it -> REAL conflict)
        //
        // CORRECT arbitration: E2 reads set 16, sees node1's checkout,
        //   conflict -> whole commit aborts -> mem[16] stays DEAD_BEEF...0016.
        // COLLISION: E2's SEQ_READ is hijacked to set 8 (E0's write-back),
        //   sees the clean set, misses the conflict -> false commit ->
        //   mem[16] := 1616...  AND the [MOD] line for E2 prints the WRONG set.
        // ==================================================================
        begin
            // node0 loads its two read variables (legit checkouts to release)
            issue_request(CMD_LOAD, 4'b0001, 64'd8);
            issue_request(CMD_LOAD, 4'b0001, 64'd12);
            // node1 loads the write variable -> foreign checkout in set 16
            issue_request(CMD_LOAD, 4'b0010, 64'd16);

            release_is_write = '0;
            release_is_read  = '0;
            release_addr = '0;
            release_data = '0;

            release_is_read[0]  = 1'b1; // E0: read, set 8  (clean)
            release_addr[0] = 64'd8;

            release_is_read[1]  = 1'b1; // E1: read, set 12 (filler)
            release_addr[1] = 64'd12;

            release_is_write[2] = 1'b1; // E2: WRITE, set 16 (conflicted)
            release_addr[2] = 64'd16;
            release_data[2] = 64'h1616_1616_1616_1616;

            issue_request(CMD_TX_COMMIT, 4'b0001, 64'd0);

            release_is_write = '0;
            release_is_read  = '0;
            release_addr     = '0;
            release_data     = '0;
        end
        repeat (20) @(posedge clk); #1;
        expect_mem(64'd16, 64'hDEAD_BEEF_0000_0016); // FAIL here == false commit == collision

        // Let it run so the state trace prints
        repeat (12) @(posedge clk);

        if (errors == 0) $display("[%0t] TB: ALL CHECKS PASSED", $time);
        else             $display("[%0t] TB: %0d CHECK(S) FAILED", $time, errors);

        $display("[%0t] TB: done", $time);
        $finish;
    end

    // Waveforms
    initial begin
        $dumpfile("tb_cxl_controller.vcd");
        $dumpvars(0, tb_cxl_controller);
    end

endmodule