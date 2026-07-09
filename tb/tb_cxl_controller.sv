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

    // Controller-driven completion (TX_ABORT only)
    logic [NUM_NODES-1:0] ctrl_resp_valid;
    logic [1:0] ctrl_comp_signal;

    // wr_buf-driven completion (LOAD only)
    logic buf_resp_valid;
    logic [NUM_NODES-1:0] buf_resp_worker;
    logic [DATA_W-1:0] buf_resp_rdata;

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
        #2000;
        $display("[%0t] TB: TIMEOUT", $time);
        $finish;
    end
    always #5 clk = ~clk; // 10ns clock period

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

        // Preload memory at address 5 — note the hierarchy now goes through
        // wr_buf, so the memory stub is at top_inst.mem (unchanged path)
        top_inst.mem.mem[5] = 64'hDEAD_BEEF_0000_0001;
        top_inst.mem.mem[10] = 64'hDEAD_BEEF_0000_0002;
        top_inst.mem.mem[15] = 64'hDEAD_BEEF_0000_0003;
        $display("[%0t] TB: preloaded mem[5] = %h", $time, top_inst.mem.mem[5]);
        $display("[%0t] TB: preloaded mem[10] = %h", $time, top_inst.mem.mem[10]);
        $display("[%0t] TB: preloaded mem[15] = %h", $time, top_inst.mem.mem[15]);

        issue_request(CMD_LOAD, 4'b0001, 64'd5);
        issue_request(CMD_LOAD, 4'b0001, 64'd10);

        // Then abort with all three in the release set
        // begin
        //     release_is_read = '0;

        //     release_is_read[0] = 1'b1;
        //     release_addr[0] = 64'd5;
        //     release_is_read[1] = 1'b1;
        //     release_addr[1] = 64'd10;

        //     release_data = '0;

        //     issue_request(CMD_TX_ABORT, 4'b0001, 64'd0);

        //     release_is_read  = '0;
        //     release_addr = '0;
        //     release_data = '0;
        // end

        // Successful Commit with only .write
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

        // Successful Commit with .write and .read variables


        // Failed Commit with only .write


        // Failed Commit with .write and .read variables



        // Let it run so the state trace prints
        repeat (12) @(posedge clk);

        $display("[%0t] TB: done", $time);
        $finish;
    end

    // Waveforms
    initial begin
        $dumpfile("tb_cxl_controller.vcd");
        $dumpvars(0, tb_cxl_controller);
    end

endmodule