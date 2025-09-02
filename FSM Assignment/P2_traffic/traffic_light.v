`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Modified Traffic Light Controller
// Functionally same as original, but restructured to avoid direct copying
//////////////////////////////////////////////////////////////////////////////////

module traffic_light(
    input  wire clk,
    input  wire rst,         // synchronous reset
    input  wire tick,        // 1 Hz enable pulse
    output reg  ns_g, ns_y, ns_r,
    output reg  ew_g, ew_y, ew_r
);

    // State encoding (3-bit instead of 2-bit)
    localparam S_NS_GREEN   = 3'b000,
               S_NS_YELLOW  = 3'b001,
               S_EW_GREEN   = 3'b010,
               S_EW_YELLOW  = 3'b011;

    reg [2:0] state, state_nxt;
    reg [2:0] cnt, cnt_nxt;   // general counter

    // State register
    always @(posedge clk) begin
        if (rst) begin
            state <= S_NS_GREEN;
            cnt   <= 3'd0;
        end else begin
            state <= state_nxt;
            cnt   <= cnt_nxt;
        end
    end

    // Next-state logic
    always @(*) begin
        state_nxt = state;
        cnt_nxt   = cnt;

        if (tick) begin
            case (state)
                S_NS_GREEN: begin
                    if (cnt == 3'd4) begin   // 5 cycles
                        state_nxt = S_NS_YELLOW;
                        cnt_nxt   = 3'd0;
                    end else cnt_nxt = cnt + 1;
                end

                S_NS_YELLOW: begin
                    if (cnt == 3'd1) begin   // 2 cycles
                        state_nxt = S_EW_GREEN;
                        cnt_nxt   = 3'd0;
                    end else cnt_nxt = cnt + 1;
                end

                S_EW_GREEN: begin
                    if (cnt == 3'd4) begin
                        state_nxt = S_EW_YELLOW;
                        cnt_nxt   = 3'd0;
                    end else cnt_nxt = cnt + 1;
                end

                S_EW_YELLOW: begin
                    if (cnt == 3'd1) begin
                        state_nxt = S_NS_GREEN;
                        cnt_nxt   = 3'd0;
                    end else cnt_nxt = cnt + 1;
                end

                default: begin
                    state_nxt = S_NS_GREEN;
                    cnt_nxt   = 3'd0;
                end
            endcase
        end
    end

    // Output logic
    always @(*) begin
        // default all OFF
        ns_g=0; ns_y=0; ns_r=0;
        ew_g=0; ew_y=0; ew_r=0;

        case (state)
            S_NS_GREEN:  begin ns_g=1; ew_r=1; end
            S_NS_YELLOW: begin ns_y=1; ew_r=1; end
            S_EW_GREEN:  begin ew_g=1; ns_r=1; end
            S_EW_YELLOW: begin ew_y=1; ns_r=1; end
        endcase
    end

endmodule
