`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Testbench: Sequence Detector (Mealy FSM)
// Pattern: 1101 with overlap
//////////////////////////////////////////////////////////////////////////////////

module tb_seq_detector_mealy;

    reg clk_tb;
    reg rst_n_tb;
    reg din_tb;
    wire dout_tb;

    // DUT instance
    seq_detector_mealy uut (
        .clk  (clk_tb),
        .rst_n(rst_n_tb),
        .din  (din_tb),
        .dout (dout_tb)
    );

    // Clock generation (20ns period, i.e. 50 MHz)
    initial clk_tb = 0;
    always #10 clk_tb = ~clk_tb;

    // Stimulus
    initial begin
        // VCD dump for GTKWave
        $dumpfile("tb_seq_detector_mealy.vcd");
        $dumpvars(0, tb_seq_detector_mealy);

        // Apply reset (active-low)
        rst_n_tb = 0;
        din_tb   = 0;
        repeat(2) @(posedge clk_tb);
        rst_n_tb = 1;

        // --- Sequence #1: 1101 (expect one detection) ---
        drive_bits(4, 4'b1101);

        // --- Sequence #2: 1101101 (expect overlap detections) ---
        drive_bits(7, 7'b1101101);

        // --- Sequence #3: 01101101 (expect multiple hits) ---
        drive_bits(8, 8'b01101101);

        // --- Sequence #4: 111101 (expect pulse near end) ---
        drive_bits(6, 6'b111101);

        // End simulation
        #50;
        $display("\n[TB] Testbench completed.\n");
        $finish;
    end

    // Task to feed a sequence bit-by-bit (Verilog-2005 friendly)
    task drive_bits;
        input integer n;        // number of bits
        input [31:0] seq;       // sequence (max 32 bits)
        integer i;
        begin
            for (i = n-1; i >= 0; i = i-1) begin
                din_tb = seq[i];
                @(posedge clk_tb);
            end
        end
    endtask

endmodule
