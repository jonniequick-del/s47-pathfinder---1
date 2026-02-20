// SPDX-License-Identifier: Apache-2.0
// S247 Compute Core — Single execution unit
// Opcodes: PATHEVAL, GEOCHECK, AESENC, HALT
// Q16.16 fixed-point arithmetic, 64-bit MUL results

`default_nettype none

module s247_compute_core #(
    parameter CORE_ID    = 0,
    parameter DATA_WIDTH = 32,
    parameter FRAC_BITS  = 16
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  enable,

    // GPS / geofence inputs (Q16.16)
    input  wire [DATA_WIDTH-1:0] gps_lat,
    input  wire [DATA_WIDTH-1:0] gps_lon,
    input  wire [DATA_WIDTH-1:0] fence_lat,
    input  wire [DATA_WIDTH-1:0] fence_lon,
    input  wire [DATA_WIDTH-1:0] fence_rad,

    // Outputs
    output reg  [DATA_WIDTH-1:0] result,
    output reg                   done,
    output reg                   halt
);

    localparam [3:0] OP_NOP      = 4'h0;
    localparam [3:0] OP_PATHEVAL = 4'h1;
    localparam [3:0] OP_GEOCHECK = 4'h2;
    localparam [3:0] OP_AESENC   = 4'h3;
    localparam [3:0] OP_HALT     = 4'hF;

    reg [3:0]              opcode;
    reg [DATA_WIDTH-1:0]   reg_file [0:7];
    reg [7:0]              pc;
    reg [2:0]              state;

    localparam S_IDLE    = 3'd0;
    localparam S_FETCH   = 3'd1;
    localparam S_DECODE  = 3'd2;
    localparam S_EXECUTE = 3'd3;
    localparam S_DONE    = 3'd4;
    localparam S_HALTED  = 3'd5;

    reg signed [63:0] mul_result;
    reg signed [DATA_WIDTH-1:0] delta_lat;
    reg signed [DATA_WIDTH-1:0] delta_lon;
    reg signed [63:0]           dist_sq;

    integer j;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc       <= 8'h0;
            opcode   <= OP_NOP;
            state    <= S_IDLE;
            result   <= 32'h0;
            done     <= 1'b0;
            halt     <= 1'b0;
            mul_result <= 64'h0;
            delta_lat  <= 32'h0;
            delta_lon  <= 32'h0;
            dist_sq    <= 64'h0;
            for (j = 0; j < 8; j = j + 1)
                reg_file[j] <= 32'h0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 1'b0;
                    halt <= 1'b0;
                    if (enable) state <= S_FETCH;
                end

                S_FETCH: begin
                    opcode <= OP_GEOCHECK;
                    state  <= S_DECODE;
                end

                S_DECODE: begin
                    case (opcode)
                        OP_PATHEVAL: begin
                            reg_file[0] <= gps_lat;
                            reg_file[1] <= gps_lon;
                        end
                        OP_GEOCHECK: begin
                            delta_lat <= $signed(gps_lat) - $signed(fence_lat);
                            delta_lon <= $signed(gps_lon) - $signed(fence_lon);
                        end
                        OP_AESENC: begin
                            reg_file[2] <= gps_lat ^ gps_lon;
                        end
                        default: ;
                    endcase
                    state <= S_EXECUTE;
                end

                S_EXECUTE: begin
                    case (opcode)
                        OP_PATHEVAL: begin
                            result <= reg_file[0] + reg_file[1];
                            state  <= S_DONE;
                        end
                        OP_GEOCHECK: begin
                            dist_sq <= ($signed(delta_lat) * $signed(delta_lat)) +
                                       ($signed(delta_lon) * $signed(delta_lon));
                            mul_result <= $signed(fence_rad) * $signed(fence_rad);
                            if (dist_sq <= mul_result)
                                result <= 32'h0001_0000;
                            else begin
                                result <= 32'h0000_0000;
                                halt   <= 1'b1;
                            end
                            state <= S_DONE;
                        end
                        OP_AESENC: begin
                            result <= reg_file[2] ^ 32'h5A5A_5A5A;
                            state  <= S_DONE;
                        end
                        OP_HALT: begin
                            halt  <= 1'b1;
                            state <= S_HALTED;
                        end
                        default: begin
                            result <= 32'h0;
                            state  <= S_DONE;
                        end
                    endcase
                end

                S_DONE: begin
                    done  <= 1'b1;
                    pc    <= pc + 8'h1;
                    state <= S_IDLE;
                end

                S_HALTED: begin
                    halt <= 1'b1;
                    done <= 1'b0;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
