RISC-V Single-Cycle Processor with RVX10 Extensions

This project implements a single-cycle RISC-V processor extended with 10 custom RVX10 instructions while keeping the hardware modifications minimal.

Thought Process & Design Approach

The primary goal was to add new instructions without large hardware changes.

1. ALU Control Width

Noted that 15 total instructions require 4 bits for ALU control.

Change: ALUControl[2:0] was extended to ALUControl[3:0].

Files updated include the main processor, controller, and ALU decoder modules.

The ALU control width was adjusted across all relevant modules to support the extra instructions.

2. ALU Update

The ALU was extended according to the assignment’s pseudo-code while preserving the base design.

The modification allows the new instructions to be executed without affecting the existing operations.

3. funct7_2b Signal

Introduced a new signal funct7_2b since the first two bits of funct7 determine the new R-type opcode 0x0B.

This signal was added to both the controller and the ALU decoder.

The addition allows the processor to differentiate between standard and RVX10 instructions efficiently.

4. Main Decoder

The main decoder was updated to recognize a new R-type instruction following the existing R-type structure.

This ensures that instruction decoding remains consistent and modular while supporting the extension.

5. ALU Decoder

Logic was written for ALUOp = 2'b11 to handle all RVX10 instructions.

The ALU decoder now correctly interprets the new opcode and maps it to the appropriate ALU operation.

6. Register (x0) Protection

To prevent accidental writes to register x0, a simple condition was added:

if (we & a3 != 0) rf[a3] <= wd;


This ensures the zero register remains constant at 0, preserving RISC-V architectural requirements.
