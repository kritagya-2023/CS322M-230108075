`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// Testbench: Traffic Light Controller
// Description: Stimulus and monitoring for traffic light FSM
//////////////////////////////////////////////////////////////////////////////////

module tb_traffic_light;

    // Inputs
    reg clk;
    reg rst;
    reg tick;

    // Outputs
    wire ns_g, ns_y, ns_r;
    wire ew_g, ew_y, ew_r;

    // DUT instantiation
    traffic_light uut (
        .clk(clk), 
        .rst(rst), 
        .tick(tick),
        .ns_g(ns_g), .ns_y(ns_y), .ns_r(ns_r),
        .ew_g(ew_g), .ew_y(ew_y), .ew_r(ew_r)
    );

    // Clock: 10ns period
    initial clk = 0;
    always #5 clk = ~clk;

    // Tick pulse generator (1-cycle pulse every 6 clock cycles here instead of 5)
    integer counter;
    always @(posedge clk) begin
        if (rst) begin
            counter <= 0;
            tick    <= 0;
        end else begin
            counter <= counter + 1;
            tick    <= (counter % 6 == 0); 
        end
    end

    // Stimulus
    initial begin
        $dumpfile("traffic_light_tb.vcd");
        $dumpvars(0, tb_traffic_light);

        // Apply reset
        rst = 1; #25; 
        rst = 0;

        // Run long enough for multiple cycles
        #2500;
        $finish;
    end

    // Monitor only when tick happens
    always @(posedge clk) begin
        if (tick) begin
            $display("[%0t ns] NS => G:%b Y:%b R:%b || EW => G:%b Y:%b R:%b",
                      $time, ns_g, ns_y, ns_r, ew_g, ew_y, ew_r);
        end
    end

endmodule
