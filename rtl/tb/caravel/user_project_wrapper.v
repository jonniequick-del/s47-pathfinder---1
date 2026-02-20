// SPDX-License-Identifier: Apache-2.0
// Caravel User Project Wrapper — S247-PATHFINDER-1
// Bridges Caravel SoC harness to our custom GPU top module

`default_nettype none

module user_project_wrapper #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vdda1,
    inout vdda2,
    inout vssa1,
    inout vssa2,
    inout vccd1,
    inout vccd2,
    inout vssd1,
    inout vssd2,
`endif
    // Wishbone Slave
    input  wb_clk_i,
    input  wb_rst_i,
    input  wbs_stb_i,
    input  wbs_cyc_i,
    input  wbs_we_i,
    input  [3:0]  wbs_sel_i,
    input  [31:0] wbs_dat_i,
    input  [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // GPIOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (unused)
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (unused)
    input  user_clock2,

    // IRQ
    output [2:0] user_irq
);

    // Tie off unused outputs
    assign la_data_out[127:64] = 64'b0;
    assign io_out[`MPRJ_IO_PADS-1:16] = {(`MPRJ_IO_PADS-16){1'b0}};
    assign io_oeb[`MPRJ_IO_PADS-1:16] = {(`MPRJ_IO_PADS-16){1'b1}};
    assign user_irq = 3'b0;

    // Instantiate S247-PATHFINDER-1
    s247_pathfinder_top #(
        .NUM_CORES(8)
    ) pathfinder (
        .clk        (wb_clk_i),
        .rst_n      (~wb_rst_i),
        .wb_clk_i   (wb_clk_i),
        .wb_rst_i   (wb_rst_i),
        .wbs_stb_i  (wbs_stb_i),
        .wbs_cyc_i  (wbs_cyc_i),
        .wbs_we_i   (wbs_we_i),
        .wbs_sel_i  (wbs_sel_i),
        .wbs_dat_i  (wbs_dat_i),
        .wbs_adr_i  (wbs_adr_i),
        .wbs_ack_o  (wbs_ack_o),
        .wbs_dat_o  (wbs_dat_o),
        .la_data_in (la_data_in[63:0]),
        .la_data_out(la_data_out[63:0]),
        .la_oenb    (la_oenb[63:0]),
        .io_in      (io_in[15:0]),
        .io_out     (io_out[15:0]),
        .io_oeb     (io_oeb[15:0])
    );

endmodule

`default_nettype wire
