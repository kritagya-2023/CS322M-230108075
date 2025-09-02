`timescale 1ns/1ps

module tb_vending_mealy;

    reg clk, rst;
    reg [1:0] coin;
    wire dispense, chg5;

    // DUT instance
    vending_mealy dut(
        .clk(clk),
        .rst(rst),
        .coin(coin),
        .dispense(dispense),
        .chg5(chg5)
    );

    // Clock generation (10ns period = 100MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Stimulus
    initial begin
        // Dumpfile for GTKWave
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_vending_mealy);

        // Reset
        rst = 1; coin = 2'b00;
        #12 rst = 0;

        // --- Test cases ---
        // Case 1: 10 + 10 = 20 → dispense
        put10();
        put10();

        // Case 2: 5 + 5 + 10 = 20 → dispense
        put5();
        put5();
        put10();

        // Case 3: 10 + 5 + 10 = 25 → dispense + change
        put10();
        put5();
        put10();

        // Finish
        #50 $finish;
    end

    // Tasks to insert coins
    task put5;
        begin
            @(negedge clk);
            coin = 2'b01;
            @(negedge clk);
            coin = 2'b00; // idle
        end
    endtask

    task put10;
        begin
            @(negedge clk);
            coin = 2'b10;
            @(negedge clk);
            coin = 2'b00; // idle
        end
    endtask

endmodule
