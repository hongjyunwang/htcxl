`timescale 1ns/1ps

module tb_cxl_controller;

    localparam int DATA_W = 64;
    localparam int ADDR_W = 64;
    localparam int NUM_NODES = 4;
    localparam int RELEASE_SET_DEPTH = 16;
    localparam int CXL_TABLE_DEPTH = 32;
    localparam int MEM_DEPTH = 1024;
    localparam int MEM_LATENCY = 1;

    // Command encodings
    localparam logic [1:0] CMD_LOAD = 2'b00;
    localparam logic [1:0] CMD_TX_ABORT  = 2'b01;
    localparam logic [1:0] CMD_TX_COMMIT = 2'b10;

    logic clk, rst;

    logic req_valid;
    logic [1:0] tx_signal;
    logic [NUM_NODES-1:0] node_id;
    logic [ADDR_W-1:0] load_addr;
    logic [RELEASE_SET_DEPTH-1:0] release_valid;
    logic [RELEASE_SET_DEPTH-1:0] release_is_write;
    logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr;
    logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data;

    logic req_ready;
    logic resp_valid;
    logic [1:0] comp_signal;
    logic [DATA_W-1:0] load_data;

    top #(
        .DATA_W(DATA_W), 
        .ADDR_W(ADDR_W), 
        .NUM_NODES(NUM_NODES),
        .RELEASE_SET_DEPTH(RELEASE_SET_DEPTH), 
        .CXL_TABLE_DEPTH(CXL_TABLE_DEPTH),
        .MEM_DEPTH(MEM_DEPTH), 
        .MEM_LATENCY(MEM_LATENCY)
    ) top_inst (
        .clk_i(clk),
        .rst_i(rst),
        .req_valid_i(req_valid), 
        .tx_signal_i(tx_signal),
        .node_id_i(node_id), 
        .load_addr_i(load_addr),
        .release_valid_i(release_valid), 
        .release_is_write_i(release_is_write),
        .release_addr_i(release_addr), 
        .release_data_i(release_data),

        .req_ready_o(req_ready), 
        .resp_valid_o(resp_valid),
        .comp_signal_o(comp_signal), 
        .load_data_o(load_data)
    );

    // 10ns clock
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // Watchdog so a stalled FSM doesn't hang the sim forever
    initial begin
        #2000;
        $display("[%0t] *** TIMEOUT — FSM likely stalled (expected for now in W_MEM_REQ) ***", $time);
        $finish;
    end

    // Drive a cxl request (LOAD, Tx_Commit, or Tx_Abort) and complete the valid/ready handshake
    task automatic issue_request(
        input logic [1:0] cmd,
        input logic [NUM_NODES-1:0] node,
        input logic [ADDR_W-1:0] addr
    );

        $display("[%0t] TB: Begin attempting request (cmd=%0d node=%b addr=%0d)", $time, cmd, node, addr);

        req_valid = 1'b1;
        tx_signal = cmd;
        node_id = node;
        load_addr = addr;

        // Wait at negedge (signals stable) until the controller is ready
        @(negedge clk);
        while (!req_ready) @(negedge clk);

        // req_ready is high; the next posedge is where the controller dispatches
        @(posedge clk);
        req_valid = 1'b0;

        $display("[%0t] TB: request accepted (cmd=%0d node=%b addr=%0d)", $time, cmd, node, addr);
    endtask

    initial begin
        // Init all inputs
        req_valid = 0; tx_signal = 0; node_id = 0; load_addr = 0;
        release_valid = 0; release_is_write = 0; release_addr = 0; release_data = 0;

        // Reset
        rst = 1'b1;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // Preload memory at address 5
        top_inst.mem.mem[5] = 64'hDEAD_BEEF_0000_0001;
        $display("[%0t] TB: preloaded mem[5] = %h", $time, top_inst.mem.mem[5]);

        // Issue LOAD: node 1 (one-hot 0001), address 5
        issue_request(CMD_LOAD, 4'b0001, 64'd5);

        // Let it run so the state trace prints
        repeat (12) @(posedge clk);

        // Check the part that currently works: CXL table entry 0
        // (empty table -> free_idx priority encoder picks index 0)
        // $display("[%0t] TB: checking CXL table entry 0...", $time);
        // if (top_inst.dut.cxl_table[0].valid === 1'b1   &&
        //     top_inst.dut.cxl_table[0].addr === 64'd5  &&
        //     top_inst.dut.cxl_table[0].checkout_vec === 4'b0001) begin
        //     $display("[%0t] TB: PASS — entry 0 valid, addr=5, checkout=0001", $time);
        // end else begin
        //     $display("[%0t] TB: FAIL — entry 0: valid=%b addr=%0d checkout=%b",
        //              $time,
        //              top_inst.dut.cxl_table[0].valid,
        //              top_inst.dut.cxl_table[0].addr,
        //              top_inst.dut.cxl_table[0].checkout_vec);
        // end

        // $display("[%0t] TB: done", $time);
        // $finish;
    end

    // Waveforms
    initial begin
        $dumpfile("tb_cxl_controller.vcd");
        $dumpvars(0, tb_cxl_controller);
    end

endmodule