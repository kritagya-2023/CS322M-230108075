`timescale 1ns / 1ps
module tb_comparator_4bit;
    reg [3:0] A, B;
    wire equal;

    comparator_4bit uut (
        .A(A),
        .B(B),
        .equal(equal)
    );

    initial begin
        $dumpfile("dump2.vcd");
        $dumpvars(0, tb_comparator_4bit);

        // Test cases
        A = 4'b0000; B = 4'b0000; #10;
        A = 4'b1010; B = 4'b1010; #10;
        A = 4'b1100; B = 4'b1001; #10;
        A = 4'b1111; B = 4'b1111; #10;
        A = 4'b0011; B = 4'b0110; #10;

        $finish;
    end
endmodule
