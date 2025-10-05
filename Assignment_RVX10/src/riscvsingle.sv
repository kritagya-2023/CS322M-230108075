// riscvsingle.sv

// RISC-V single-cycle processor with RV-CustomX extension
// Based on Section 7.6 of Digital Design & Computer Architecture
// This version implements a custom instruction set extension.

// Implements a subset of the base integer instructions plus RV-CustomX:
//   lw, sw
//   add, sub, and, or, slt,
//   addi, andi, ori, slti
//   beq
//   jal
//   RV-CustomX: ANDN, ORN, XNOR, MIN, MAX, ROL, ROR, ABS

// little-endian memory

module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite);

  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
        if(DataAdr === 100 & WriteData === 25) begin
          $display("Simulation succeeded");
          $stop;
        end else if (DataAdr !== 96) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end
endmodule

module top(input  logic        clk, reset,
           output logic [31:0] WriteData, DataAdr,
           output logic        MemWrite);

  logic [31:0] PC, Instr, ReadData;

  // instantiate processor and memories
  riscvsingle rvsingle(clk, reset, PC, Instr, MemWrite, DataAdr,
                       WriteData, ReadData);
  imem imem(PC, Instr);
  dmem dmem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

module riscvsingle(input  logic        clk, reset,
                   output logic [31:0] PC,
                   input  logic [31:0] Instr,
                   output logic        MemWrite,
                   output logic [31:0] ALUResult, WriteData,
                   input  logic [31:0] ReadData);

  logic        ALUSrc, RegWrite, Jump, Zero, PCSrc;
  logic [1:0]  ResultSrc, ImmSrc;
  logic [3:0]  ALUControl; // Widened to 4 bits for custom instructions

  controller c(Instr, Zero,
               ResultSrc, MemWrite, PCSrc,
               ALUSrc, RegWrite, Jump,
               ImmSrc, ALUControl);
  datapath dp(clk, reset, ResultSrc, PCSrc,
              ALUSrc, RegWrite,
              ImmSrc, ALUControl,
              Zero, PC, Instr,
              ALUResult, WriteData, ReadData);
endmodule

module controller(input  logic [31:0] Instr,
                  input  logic        Zero,
                  output logic [1:0] ResultSrc,
                  output logic        MemWrite,
                  output logic        PCSrc, ALUSrc,
                  output logic        RegWrite, Jump,
                  output logic [1:0] ImmSrc,
                  output logic [3:0] ALUControl); // Widened

  logic [1:0] ALUOp;
  logic       Branch;
  
  // Deconstruct instruction fields
  logic [6:0] op = Instr[6:0];
  logic [2:0] funct3 = Instr[14:12];
  logic [6:0] funct7 = Instr[31:25];

  maindec md(op, ResultSrc, MemWrite, Branch,
              ALUSrc, RegWrite, Jump, ImmSrc, ALUOp);
  aludec  ad(op[5], funct3, funct7, ALUOp, ALUControl);

  assign PCSrc = (Branch & Zero) | Jump;
endmodule

module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic        MemWrite,
               output logic        Branch, ALUSrc,
               output logic        RegWrite, Jump,
               output logic [1:0] ImmSrc,
               output logic [1:0] ALUOp);

  logic [10:0] controls;

  // Main decoder control signals
  //      RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
  assign {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = controls;

  always_comb
    case(op)
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
      7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
      7'b0101011: controls = 11'b1_xx_0_0_00_0_11_0; // RV-CustomX instructions
      default:    controls = 11'bx_xx_x_x_xx_x_xx_x; // non-implemented
    endcase
endmodule

module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic [6:0] funct7,
              input  logic [1:0] ALUOp,
              output logic [3:0] ALUControl); // Widened

  always_comb
    case(ALUOp)
      2'b00:  ALUControl = 4'b0000; // lw/sw: addition
      2'b01:  ALUControl = 4'b0001; // beq: subtraction
      2'b10:  // R-Type or I-Type
        case(funct3)
          3'b000: if (funct7[5] & opb5) ALUControl = 4'b0001; // sub
                  else                   ALUControl = 4'b0000; // add, addi
          3'b010: ALUControl = 4'b0101; // slt, slti
          3'b110: ALUControl = 4'b0011; // or, ori
          3'b111: ALUControl = 4'b0010; // and, andi
          default: ALUControl = 4'bxxxx;
        endcase
      2'b11:  // RV-CustomX instructions. Unique decoding scheme.
        // All custom ops use funct7 = 7'b0100000
        if (funct7 == 7'b0100000)
          case(funct3)
            3'b000: ALUControl = 4'b1000; // ANDN
            3'b001: ALUControl = 4'b1001; // ORN
            3'b010: ALUControl = 4'b1010; // XNOR
            3'b011: ALUControl = 4'b1011; // ROL
            3'b100: ALUControl = 4'b1100; // ROR
            3'b101: ALUControl = 4'b1101; // MIN (signed)
            3'b110: ALUControl = 4'b1110; // MAX (signed)
            3'b111: ALUControl = 4'b1111; // ABS
            default: ALUControl = 4'bxxxx;
          endcase
        else ALUControl = 4'bxxxx; // Undefined custom op
      default: ALUControl = 4'bxxxx;
    endcase
endmodule

module datapath(input  logic        clk, reset,
                input  logic [1:0]  ResultSrc,
                input  logic        PCSrc, ALUSrc,
                input  logic        RegWrite,
                input  logic [1:0]  ImmSrc,
                input  logic [3:0]  ALUControl, // Widened
                output logic        Zero,
                output logic [31:0] PC,
                input  logic [31:0] Instr,
                output logic [31:0] ALUResult, WriteData,
                input  logic [31:0] ReadData);

  logic [31:0] PCNext, PCPlus4, PCTarget;
  logic [31:0] ImmExt;
  logic [31:0] SrcA, SrcB;
  logic [31:0] Result;

  // PC logic
  flopr #(32) pcreg(clk, reset, PCNext, PC);
  adder       pcadd4(PC, 32'd4, PCPlus4);
  adder       pcaddbranch(PC, ImmExt, PCTarget);
  mux2 #(32)  pcmux(PCPlus4, PCTarget, PCSrc, PCNext);

  // register file logic
  regfile     rf(clk, RegWrite, Instr[19:15], Instr[24:20],
                  Instr[11:7], Result, SrcA, WriteData);
  extend      ext(Instr, ImmSrc, ImmExt); // Pass full instruction to extend

  // ALU logic
  mux2 #(32)  srcbmux(WriteData, ImmExt, ALUSrc, SrcB);
  alu         alu(SrcA, SrcB, ALUControl, ALUResult, Zero);
  mux3 #(32)  resultmux(ALUResult, ReadData, PCPlus4, ResultSrc, Result);
endmodule

module regfile(input  logic       clk,
               input  logic       we3,
               input  logic [4:0] a1, a2, a3,
               input  logic [31:0] wd3,
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three-ported register file
  always_ff @(posedge clk)
    if (we3 && (a3 != 5'b0)) rf[a3] <= wd3; // Prevent writes to x0

  assign rd1 = (a1 != 5'b0) ? rf[a1] : 32'b0;
  assign rd2 = (a2 != 5'b0) ? rf[a2] : 32'b0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);
  assign y = a + b;
endmodule

module extend(input  logic [31:0] instr, // Changed to take full instruction
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);

  always_comb
    case(immsrc)
      // I-type
      2'b00:  immext = {{20{instr[31]}}, instr[31:20]};
      // S-type
      2'b01:  immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
      // B-type
      2'b10:  immext = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
      // J-type
      2'b11:  immext = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
      default: immext = 32'bx;
    endcase
endmodule


module flopr #(parameter WIDTH = 8)
             (input  logic             clk, reset,
              input  logic [WIDTH-1:0] d,
              output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
            (input  logic [WIDTH-1:0] d0, d1,
             input  logic             s,
             output logic [WIDTH-1:0] y);
  assign y = s ? d1 : d0;
endmodule

module mux3 #(parameter WIDTH = 8)
            (input  logic [WIDTH-1:0] d0, d1, d2,
             input  logic [1:0]       s,
             output logic [WIDTH-1:0] y);
  assign y = s[1] ? d2 : (s[0] ? d1 : d0);
endmodule

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
    $readmemh("riscvtest.txt",RAM); // Using original test file

  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module dmem(input  logic       clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [3:0]  alucontrol, // Widened
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] adder_out;
  logic [31:0] b_inverted;
  logic        is_sub_slt;
  logic        overflow;
  logic signed [31:0] signed_a, signed_b;
  
  assign signed_a = a;
  assign signed_b = b;

  // adder logic for add, sub, slt
  assign b_inverted = alucontrol[0] ? ~b : b;
  assign adder_out = a + b_inverted + alucontrol[0];
  assign is_sub_slt = (alucontrol == 4'b0001) | (alucontrol == 4'b0101); // SUB or SLT
  assign overflow = (a[31] == b_inverted[31]) && (adder_out[31] != a[31]);

  always_comb
    case (alucontrol)
      4'b0000:  result = adder_out;         // ADD
      4'b0001:  result = adder_out;         // SUB
      4'b0010:  result = a & b;             // AND
      4'b0011:  result = a | b;             // OR
      4'b0101:  result = {31'b0, (overflow ^ adder_out[31])}; // SLT
      
      // --- RV-CustomX Operations ---
      4'b1000:  result = a & ~b;            // ANDN
      4'b1001:  result = a | ~b;            // ORN
      4'b1010:  result = ~(a ^ b);          // XNOR
      4'b1011:  result = (a << b[4:0]) | (a >> (32 - b[4:0])); // ROL
      4'b1100:  result = (a >> b[4:0]) | (a << (32 - b[4:0])); // ROR
      4'b1101:  result = (signed_a < signed_b) ? a : b; // MIN (signed)
      4'b1110:  result = (signed_a > signed_b) ? a : b; // MAX (signed)
      4'b1111:  result = a[31] ? -a : a;      // ABS (unique implementation)
      default:  result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
endmodule