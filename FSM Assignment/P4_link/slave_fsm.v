// slave_fsm.v — latches data, asserts ack for >=2 cycles, drops after req falls
module slave_fsm(
  input  wire       clk,
  input  wire       rst,
  input  wire       req,
  input  wire [7:0] data_in,
  output reg        ack,
  output reg [7:0]  last_byte
);

  // states
  parameter W_WAIT_REQ = 2'd0,
            W_ASSERT   = 2'd1,
            W_HOLD     = 2'd2,
            W_DROP     = 2'd3;

  reg [1:0] state, nstate;
  reg [1:0] hold_cnt;

  // sequential
  always @(posedge clk) begin
    if (rst) begin
      state     <= W_WAIT_REQ;
      hold_cnt  <= 2'd0;
      ack       <= 1'b0;
      last_byte <= 8'h00;
    end else begin
      state <= nstate;

      case (state)
        W_WAIT_REQ: begin
          ack <= 1'b0;
          if (req) begin
            last_byte <= data_in;
            ack       <= 1'b1;
            hold_cnt  <= 2'd0;
          end
        end
        W_ASSERT: begin
          ack <= 1'b1;
          hold_cnt <= hold_cnt + 1;
        end
        W_HOLD: begin
          ack <= 1'b1;
        end
        W_DROP: begin
          ack <= 1'b0;
        end
      endcase
    end
  end

  // next-state
  always @(*) begin
    nstate = state;
    case (state)
      W_WAIT_REQ: if (req) nstate = W_ASSERT;
      W_ASSERT:   if (hold_cnt == 2'd1) nstate = (req ? W_HOLD : W_DROP);
      W_HOLD:     if (!req) nstate = W_DROP;
      W_DROP:     nstate = W_WAIT_REQ;
    endcase
  end

endmodule
