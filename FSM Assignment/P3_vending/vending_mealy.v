`timescale 1ns/1ps

module vending_mealy(
    input wire clk,
    input wire rst,          // synchronous active-high reset
    input wire [1:0] coin,   // 01=5, 10=10, 00=idle, 11=invalid
    output reg dispense,     // 1-cycle pulse
    output reg chg5          // 1-cycle pulse when returning 5
);

    // State encoding (total inserted so far)
    localparam S0  = 2'b00;
    localparam S5  = 2'b01;
    localparam S10 = 2'b10;
    localparam S15 = 2'b11;

    reg [1:0] state, next_state;

    // Next-state and output logic (Mealy)
    always @(*) begin
        // defaults
        next_state = state;
        dispense   = 0;
        chg5       = 0;

        case (state)
            S0: begin
                if (coin == 2'b01) next_state = S5;        // +5
                else if (coin == 2'b10) next_state = S10;  // +10
            end

            S5: begin
                if (coin == 2'b01) next_state = S10;       // +5
                else if (coin == 2'b10) next_state = S15;  // +10
            end

            S10: begin
                if (coin == 2'b01) next_state = S15;       // +5
                else if (coin == 2'b10) begin
                    // total = 20 → dispense
                    dispense   = 1;
                    next_state = S0;
                end
            end

            S15: begin
                if (coin == 2'b01) begin
                    // total = 20 → dispense
                    dispense   = 1;
                    next_state = S0;
                end
                else if (coin == 2'b10) begin
                    // total = 25 → dispense + change
                    dispense   = 1;
                    chg5       = 1;
                    next_state = S0;
                end
            end
        endcase
    end

    // State register
    always @(posedge clk) begin
        if (rst) state <= S0;
        else     state <= next_state;
    end

endmodule
