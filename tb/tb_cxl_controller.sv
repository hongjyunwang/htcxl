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
    logic [RELEASE_SET_DEPTH-1:0] release_valid;
    logic [RELEASE_SET_DEPTH-1:0] release_is_write;
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
        .release_valid_i(release_valid), 
        .release_is_write_i(release_is_write),
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
        while ((req_ready & node) == '0) @(negedge clk);  // also check own bit on req_ready

        req_valid = node;
        tx_signal = cmd;
        load_addr = addr;
        $display("[%0t] TB: request issued (cmd=%0d node=%b addr=%0d)", $time, cmd, node, addr);

        @(posedge clk);
        @(negedge clk);
        req_valid = '0;

        if (cmd == CMD_LOAD) begin
            // LOAD completion comes from wr_buf — resp_worker is a one-hot tag,
            // not gated per-node like the old resp_valid vector was
            while (!(buf_resp_valid && (buf_resp_worker == node))) @(negedge clk);
            $display("[%0t] TB: LOAD response received (node=%b data=%0h)",
                      $time, node, buf_resp_rdata);
        end else if (cmd == CMD_TX_ABORT) begin
            while (!(ctrl_resp_valid & node)) @(negedge clk);
            $display("[%0t] TB: TX_ABORT response received (node=%b comp=%0d)",
                      $time, node, ctrl_comp_signal);
        end else begin
            // CMD_TX_COMMIT — placeholder, no completion path defined yet
            $display("[%0t] TB: TX_COMMIT issued, no completion tracking implemented", $time);
        end
    endtask

    initial begin
        // Init all inputs
        req_valid = '0; 
        tx_signal = 0; 
        load_addr = 0;
        release_valid = 0; 
        release_is_write = 0; 
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

        // fork
        //     issue_request(CMD_LOAD, 4'b0001, 64'd5); // node 1, cycle N
        //     begin
        //         @(negedge clk); // wait one cycle behind node 1
        //         issue_request(CMD_LOAD, 4'b0010, 64'd5); // node 2, cycle N+1
        //     end
        // join

        // TX_ABORT test — node 0001 aborts with address 5 in its release set
        issue_request(CMD_LOAD, 4'b0001, 64'd5);
        issue_request(CMD_LOAD, 4'b0001, 64'd10);
        issue_request(CMD_LOAD, 4'b0001, 64'd15);

        // Then abort with all three in the release set
        begin
            release_valid = '0;
            release_valid[0] = 1;
            release_valid[1] = 1;
            release_valid[2] = 1;
            release_is_write = '0;
            release_addr[0] = 64'd5;
            release_addr[1] = 64'd10;
            release_addr[2] = 64'd15;
            release_data = '0;

            issue_request(CMD_TX_ABORT, 4'b0001, 64'd0);

            release_valid = '0;
            release_is_write = '0;
            release_addr = '0;
            release_data = '0;
        end

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
    
    // always @(negedge clk) begin
    //     if (!rst) begin
    //         if (top_inst.dut.dbg_mod_req_valid)
    //             $display("[%0t] CTRL REQ stage: cmd=%0d node=%b addr=%0d",
    //                 $time, top_inst.dut.dbg_mod_req_cmd,
    //                 top_inst.dut.dbg_mod_req_worker,
    //                 top_inst.dut.dbg_mod_req_addr);

    //         if (top_inst.dut.mem_req_valid_o)
    //             $display("[%0t] MEM REQ: we=%0b addr=%0d worker=%b",
    //                 $time, top_inst.dut.mem_we_o,
    //                 top_inst.dut.mem_addr_o,
    //                 top_inst.dut.mem_worker_o);

    //         if (buf_resp_valid)
    //             $display("[%0t] TB RESP: worker=%b rdata=%0h",
    //                 $time, buf_resp_worker, buf_resp_rdata);
    //     end
    // end

endmodule