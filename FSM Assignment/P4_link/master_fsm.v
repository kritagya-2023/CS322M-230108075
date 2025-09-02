// master_fsm.v — 4-byte burst master with 4-phase req/ack
module master_fsm(
  input  wire       clk,
  input  wire       rst,     // synchronous, active-high
  input  wire       ack,
  output reg        req,
  output reg [7:0]  data,
  output reg        done     // 1-cycle pulse after 4th transfer
);

  // state encoding
  parameter S_IDLE       = 3'd0,
            S_DRIVE      = 3'd1,
            S_WAIT_ACK   = 3'd2,
            S_DROP_REQ   = 3'd3,
            S_WAIT_ACK0  = 3'd4,
            S_DONE_PULSE = 3'd5;

  reg [2:0] state, nstate;
  reg [1:0] idx;         // 0..3 (four bytes)
  reg [7:0] rom [0:3];

  // simple pattern A0..A3
  initial begin
    rom[0] = 8'hA0;
    rom[1] = 8'hA1;
    rom[2] = 8'hA2;
    rom[3] = 8'hA3;
  end

  // sequential
  always @(posedge clk) begin
    if (rst) begin
      state <= S_IDLE;
      idx   <= 2'd0;
      data  <= 8'h00;
      req   <= 1'b0;
      done  <= 1'b0;
    end else begin
      state <= nstate;
      if (nstate != S_DONE_PULSE) done <= 1'b0;

      case (state)
        S_IDLE: begin
          req <= 1'b0;
        end
        S_DRIVE: begin
          data <= rom[idx];
          req  <= 1'b1;
        end
        S_WAIT_ACK: begin
          req  <= 1'b1;
        end
        S_DROP_REQ: begin
          req  <= 1'b0;
        end
        S_WAIT_ACK0: begin
          req  <= 1'b0;
        end
        S_DONE_PULSE: begin
          done <= 1'b1;
          req  <= 1'b0;
        end
      endcase
    end
  end

  // next-state
  always @(*) begin
    nstate = state;
    case (state)
      S_IDLE:       nstate = S_DRIVE;
      S_DRIVE:      nstate = S_WAIT_ACK;
      S_WAIT_ACK:   if (ack) nstate = S_DROP_REQ;
      S_DROP_REQ:   nstate = S_WAIT_ACK0;
      S_WAIT_ACK0:  if (!ack)
                      if (idx == 2'd3) nstate = S_DONE_PULSE;
                      else             nstate = S_DRIVE;
      S_DONE_PULSE: nstate = S_IDLE;
    endcase
  end

  // index update
  always @(posedge clk) begin
    if (rst) begin
      idx <= 2'd0;
    end else if (state == S_WAIT_ACK0 && !ack) begin
      if (idx == 2'd3) idx <= 2'd0;
      else             idx <= idx + 2'd1;
    end
  end

endmodule
