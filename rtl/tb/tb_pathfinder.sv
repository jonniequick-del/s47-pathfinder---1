// SPDX-License-Identifier: Apache-2.0
// Testbench for S247-PATHFINDER-1 Top Module

`timescale 1ns / 1ps

module tb_pathfinder;

    reg         clk;
    reg         rst_n;
    reg         wbs_stb_i, wbs_cyc_i, wbs_we_i;
    reg  [3:0]  wbs_sel_i;
    reg  [31:0] wbs_dat_i, wbs_adr_i;
    wire        wbs_ack_o;
    wire [31:0] wbs_dat_o;
    reg  [63:0] la_data_in;
    wire [63:0] la_data_out;
    reg  [63:0] la_oenb;
    reg  [15:0] io_in;
    wire [15:0] io_out;
    wire [15:0] io_oeb;

    s247_pathfinder_top #(.NUM_CORES(8)) dut (
        .clk(clk), .rst_n(rst_n),
        .wb_clk_i(clk), .wb_rst_i(~rst_n),
        .wbs_stb_i(wbs_stb_i), .wbs_cyc_i(wbs_cyc_i),
        .wbs_we_i(wbs_we_i), .wbs_sel_i(wbs_sel_i),
        .wbs_dat_i(wbs_dat_i), .wbs_adr_i(wbs_adr_i),
        .wbs_ack_o(wbs_ack_o), .wbs_dat_o(wbs_dat_o),
        .la_data_in(la_data_in), .la_data_out(la_data_out),
        .la_oenb(la_oenb),
        .io_in(io_in), .io_out(io_out), .io_oeb(io_oeb)
    );

    initial clk = 0;
    always #10 clk = ~clk;

    task wb_write(input [31:0] addr, input [31:0] data);
        begin
            @(posedge clk);
            wbs_stb_i <= 1; wbs_cyc_i <= 1; wbs_we_i <= 1;
            wbs_sel_i <= 4'hF; wbs_adr_i <= addr; wbs_dat_i <= data;
            @(posedge clk); wait(wbs_ack_o); @(posedge clk);
            wbs_stb_i <= 0; wbs_cyc_i <= 0; wbs_we_i <= 0;
        end
    endtask

    task wb_read(input [31:0] addr, output [31:0] data);
        begin
            @(posedge clk);
            wbs_stb_i <= 1; wbs_cyc_i <= 1; wbs_we_i <= 0;
            wbs_sel_i <= 4'hF; wbs_adr_i <= addr;
            @(posedge clk); wait(wbs_ack_o);
            data = wbs_dat_o; @(posedge clk);
            wbs_stb_i <= 0; wbs_cyc_i <= 0;
        end
    endtask

    reg [31:0] read_data;

    initial begin
        $dumpfile("tb_pathfinder.vcd");
        $dumpvars(0, tb_pathfinder);

        rst_n = 0; wbs_stb_i = 0; wbs_cyc_i = 0;
        wbs_we_i = 0; wbs_sel_i = 0; wbs_dat_i = 0;
        wbs_adr_i = 0; la_data_in = 64'h0;
        la_oenb = 64'h0; io_in = 16'h0;

        #100; rst_n = 1; #20;

        $display("========================================");
        $display(" S247-PATHFINDER-1 — Testbench");
        $display("========================================");

        $display("[TEST 1] Writing GPS coordinates...");
        wb_write(32'h0000_000C, 32'h0033_8555);
        wb_write(32'h0000_0010, 32'hFFFF_DF48);

        $display("[TEST 2] Setting geofence...");
        wb_write(32'h0000_0014, 32'h0033_8555);
        wb_write(32'h0000_0018, 32'hFFFF_DF48);
        wb_write(32'h0000_001C, 32'h0000_024E);

        $display("[TEST 3] Enabling all cores...");
        wb_write(32'h0000_0008, 32'h0000_00FF);
        wb_write(32'h0000_0004, 32'h0000_0001);

        #500;

        $display("[TEST 4] Reading status register...");
        wb_read(32'h0000_0000, read_data);
        $display("  Status: 0x%08h", read_data);
        $display("  Emergency halt: %b", read_data[0]);

        $display("[TEST 5] Simulating geofence breach...");
        wb_write(32'h0000_000C, 32'h0050_0000);
        wb_write(32'h0000_0004, 32'h0000_0001);

        #500;

        wb_read(32'h0000_0000, read_data);
        $display("  Status after breach: 0x%08h", read_data);
        $display("  Emergency halt: %b", read_data[0]);

        $display("========================================");
        $display(" All tests complete.");
        $display("========================================");

        #100; $finish;
    end

endmodule
