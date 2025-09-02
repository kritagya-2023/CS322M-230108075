`timescale 1ns/1ps
module tb_link_top;

  reg clk = 0;
  reg rst = 1;
  wire done;

  link_top dut(.clk(clk), .rst(rst), .done(done));

  always #5 clk = ~clk; // 100 MHz

  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0, tb_link_top);

    repeat (4) @(posedge clk);
    rst <= 0;

    wait (done == 1'b1);
    @(posedge clk);
    repeat (5) @(posedge clk);

    $display("DONE observed at %0t", $time);
    $finish;
  end

  wire req   = dut.u_master.req;
  wire ack   = dut.u_slave.ack;
  wire [7:0] data = dut.u_master.data;
  wire [7:0] last = dut.u_slave.last_byte;

  always @(posedge clk) begin
    if (!rst)
      $strobe("t=%0t req=%b ack=%b data=%h last=%h done=%b",
              $time, req, ack, data, last, done);
  end
endmodule
