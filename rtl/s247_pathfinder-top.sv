// SPDX-License-Identifier: Apache-2.0
// S247-PATHFINDER-1 — Top-Level Module
// Sovereign GPU for drone navigation (8 compute cores, Q16.16 fixed-point)
// Target: Efabless/Google Open MPW (GF180MCU), 50 MHz, 4mm² die

`default_nettype none

module s247_pathfinder_top #(
    parameter NUM_CORES = 8,
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 12,
    parameter FRAC_BITS = 16  // Q16.16 fixed-point
)(
    input  wire        clk,
    input  wire        rst_n,

    // Wishbone slave interface (Caravel interconnect)
    input  wire        wb_clk_i,
    input  wire        wb_rst_i,
    input  wire        wbs_stb_i,
    input  wire        wbs_cyc_i,
    input  wire        wbs_we_i,
    input  wire [3:0]  wbs_sel_i,
    input  wire [31:0] wbs_dat_i,
    input  wire [31:0] wbs_adr_i,
    output reg         wbs_ack_o,
    output reg  [31:0] wbs_dat_o,

    // Logic analyzer probes
    input  wire [63:0] la_data_in,
    output wire [63:0] la_data_out,
    input  wire [63:0] la_oenb,

    // GPIO
    input  wire [15:0] io_in,
    output wire [15:0] io_out,
    output wire [15:0] io_oeb
);

    // =========================================================================
    // Internal signals
    // =========================================================================
    reg  [DATA_WIDTH-1:0] instruction_mem [0:255];
    reg  [DATA_WIDTH-1:0] data_mem [0:255];

    // Core control
    reg  [NUM_CORES-1:0] core_enable;
    reg  [NUM_CORES-1:0] core_done;
    reg  [NUM_CORES-1:0] core_halt;
    wire [DATA_WIDTH-1:0] core_result [0:NUM_CORES-1];

    // Global control registers (Wishbone-mapped)
    reg [31:0] ctrl_status;     // 0x00: status register
    reg [31:0] ctrl_command;    // 0x04: command register
    reg [31:0] ctrl_core_mask;  // 0x08: which cores to activate
    reg [31:0] gps_lat;         // 0x0C: GPS latitude  (Q16.16)
    reg [31:0] gps_lon;         // 0x10: GPS longitude (Q16.16)
    reg [31:0] geofence_lat;    // 0x14: geofence center lat
    reg [31:0] geofence_lon;    // 0x18: geofence center lon
    reg [31:0] geofence_radius; // 0x1C: geofence radius

    // Emergency halt — active if ANY core asserts halt
    wire emergency_halt = |core_halt;

    // =========================================================================
    // Wishbone slave — register read/write
    // =========================================================================
    wire wb_valid = wbs_stb_i && wbs_cyc_i;
    wire [7:0] wb_reg_addr = wbs_adr_i[9:2]; // word-aligned

    always @(posedge wb_clk_i) begin
        if (wb_rst_i) begin
            wbs_ack_o      <= 1'b0;
            wbs_dat_o      <= 32'h0;
            ctrl_status    <= 32'h0;
            ctrl_command   <= 32'h0;
            ctrl_core_mask <= 32'hFF; // all 8 cores enabled
            gps_lat        <= 32'h0;
            gps_lon        <= 32'h0;
            geofence_lat   <= 32'h0;
            geofence_lon   <= 32'h0;
            geofence_radius<= 32'h0;
        end else begin
            wbs_ack_o <= 1'b0;
            if (wb_valid && !wbs_ack_o) begin
                wbs_ack_o <= 1'b1;
                if (wbs_we_i) begin
                    // Write
                    case (wb_reg_addr)
                        8'h00: ctrl_status     <= wbs_dat_i;
                        8'h01: ctrl_command    <= wbs_dat_i;
                        8'h02: ctrl_core_mask  <= wbs_dat_i;
                        8'h03: gps_lat         <= wbs_dat_i;
                        8'h04: gps_lon         <= wbs_dat_i;
                        8'h05: geofence_lat    <= wbs_dat_i;
                        8'h06: geofence_lon    <= wbs_dat_i;
                        8'h07: geofence_radius <= wbs_dat_i;
                        default: ; // instruction/data mem writes could go here
                    endcase
                end else begin
                    // Read
                    case (wb_reg_addr)
                        8'h00: wbs_dat_o <= {core_halt, core_done, core_enable, emergency_halt, 7'h0};
                        8'h01: wbs_dat_o <= ctrl_command;
                        8'h02: wbs_dat_o <= ctrl_core_mask;
                        8'h03: wbs_dat_o <= gps_lat;
                        8'h04: wbs_dat_o <= gps_lon;
                        8'h05: wbs_dat_o <= geofence_lat;
                        8'h06: wbs_dat_o <= geofence_lon;
                        8'h07: wbs_dat_o <= geofence_radius;
                        default: wbs_dat_o <= 32'hDEADBEEF;
                    endcase
                end
            end
        end
    end

    // =========================================================================
    // Compute core instantiation (8 cores)
    // =========================================================================
    genvar i;
    generate
        for (i = 0; i < NUM_CORES; i = i + 1) begin : gen_core
            s247_compute_core #(
                .CORE_ID(i),
                .DATA_WIDTH(DATA_WIDTH),
                .FRAC_BITS(FRAC_BITS)
            ) u_core (
                .clk        (clk),
                .rst_n      (rst_n & ~emergency_halt),
                .enable     (core_enable[i]),
                .gps_lat    (gps_lat),
                .gps_lon    (gps_lon),
                .fence_lat  (geofence_lat),
                .fence_lon  (geofence_lon),
                .fence_rad  (geofence_radius),
                .result     (core_result[i]),
                .done       (core_done[i]),
                .halt       (core_halt[i])
            );
        end
    endgenerate

    // =========================================================================
    // Core enable logic (from command register)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            core_enable <= {NUM_CORES{1'b0}};
        end else if (emergency_halt) begin
            core_enable <= {NUM_CORES{1'b0}}; // freeze all on halt
        end else begin
            core_enable <= ctrl_core_mask[NUM_CORES-1:0] & {NUM_CORES{ctrl_command[0]}};
        end
    end

    // =========================================================================
    // Logic analyzer outputs
    // =========================================================================
    assign la_data_out = {
        core_halt,          // [63:56]
        core_done,          // [55:48]
        core_enable,        // [47:40]
        emergency_halt,     // [39]
        7'b0,               // [38:32]
        gps_lat[31:16],     // [31:16] integer part of lat
        gps_lon[31:16]      // [15:0]  integer part of lon
    };

    // =========================================================================
    // GPIO — directly expose core status
    // =========================================================================
    assign io_out = {core_halt, core_done};
    assign io_oeb = 16'h0000; // all outputs

endmodule
