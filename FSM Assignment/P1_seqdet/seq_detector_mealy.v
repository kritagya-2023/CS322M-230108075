`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Sequence Detector (Mealy FSM)
// Pattern: 1101 (with overlap)
// Verilog-2005 compatible (no typedef/enum)
//////////////////////////////////////////////////////////////////////////////////

module seq_detector_mealy(
    input  wire clk,
    input  wire rst_n,   // active-low reset
    input  wire din,
    output wire dout
);

    // state encoding using parameters
    parameter S0 = 2'b00;  // no match
    parameter S1 = 2'b01;  // seen '1'
    parameter S2 = 2'b10;  // seen "11"
    parameter S3 = 2'b11;  // seen "110"

    reg [1:0] cur_state, nxt_state;

    // state register
    always @(posedge clk) begin
        if (!rst_n) 
            cur_state <= S0;
        else        
            cur_state <= nxt_state;
    end

    // next state logic
    always @(*) begin
        case (cur_state)
            S0:   nxt_state = (din) ? S1 : S0;
            S1:   nxt_state = (din) ? S2 : S0;
            S2:   nxt_state = (din) ? S2 : S3;
            S3:   nxt_state = (din) ? S1 : S0;
            default: nxt_state = S0;
        endcase
    end

    // output logic (Mealy)
    assign dout = (cur_state == S3) & din;

endmodule
