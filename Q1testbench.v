module tb_comparator_1bit;
    reg A, B;
    wire o1, o2, o3;

    comparator_1bit uut (
        .A(A), .B(B),
        .o1(o1), .o2(o2), .o3(o3)
    );

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_comparator_1bit);
        $display("A B | o1 o2 o3");
        A = 0; B = 0; #10; $display("%b %b |  %b  %b  %b", A, B, o1, o2, o3);
        A = 0; B = 1; #10; $display("%b %b |  %b  %b  %b", A, B, o1, o2, o3);
        A = 1; B = 0; #10; $display("%b %b |  %b  %b  %b", A, B, o1, o2, o3);
        A = 1; B = 1; #10; $display("%b %b |  %b  %b  %b", A, B, o1, o2, o3);
        $finish;
    end
endmodule