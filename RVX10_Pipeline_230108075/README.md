# RVX10-P: Five-Stage Pipelined RISC-V Processor

## Overview

**RVX10-P** is a five-stage pipelined processor implementing the **RISC-V RV32I** instruction set, extended with ten additional arithmetic and logic operations under the **RVX10** custom extension.  
This processor was developed as part of the **Digital Logic and Computer Architecture** course under **Dr. Satyajit Das, IIT Guwahati**.

The design correctly executes all base RV32I and RVX10 instructions, manages both data and control hazards using **forwarding**, **stalls**, and **flush mechanisms**, and integrates **performance counters** to measure the average **Cycles Per Instruction (CPI)**.

---

## Pipeline Architecture

The processor adopts the classic **five-stage pipeline** organization:

| Stage | Name | Function |
| :----: | :---- | :-------- |
| **IF** | Instruction Fetch | Retrieves instructions from instruction memory (IMEM) |
| **ID** | Instruction Decode / Register Read | Decodes opcodes and accesses source registers |
| **EX** | Execute | Performs ALU computations and evaluates branch conditions |
| **MEM** | Memory Access | Reads from or writes to data memory (DMEM) |
| **WB** | Write Back | Writes computed results back to the register file |

---

## Core Components

### 🔹 Datapath
Implements the complete five-stage pipeline with **valid-bit tracking**.  
Incorporates **forwarding logic**, **stall and flush control**, and supports both **RV32I** and **RVX10** ALU operations.

---

### 🔹 Control Unit
Responsible for generating all control signals for the ALU, memory access, and register file.  
It consists of:
- **maindec** – decodes instruction opcodes  
- **aludec** – interprets function fields for ALU operation selection  

---

### 🔹 Forwarding Unit
Detects data dependencies between the **EX**, **MEM**, and **WB** stages and forwards the required operands to the ALU, minimizing stalls.

<img width="724" height="478" alt="image" src="https://github.com/user-attachments/assets/7448b04e-6698-43f1-bf27-fbf017754272" />

---

### 🔹 Hazard Detection Unit
Identifies **load-use** and **branch hazards**, inserting stalls or flushes when necessary to maintain correct execution order.

<img width="790" height="363" alt="image" src="https://github.com/user-attachments/assets/c37ef675-8e36-4d20-b595-f467321b2c09" />

---

### 🔹 Performance Counters
Tracks both the **total cycle count** and **number of executed instructions**, allowing computation of the processor’s **CPI** (Cycles Per Instruction).

<img width="808" height="505" alt="image" src="https://github.com/user-attachments/assets/313a6b37-2be9-44b2-a08c-49cf342373ff" />

---

## Supported Instruction Sets

### Base RV32I Instructions
`add`, `sub`, `and`, `or`, `slt`, `addi`, `lw`, `sw`, `beq`, `jal`

### RVX10 Custom ALU Instructions
<img width="952" height="376" alt="image" src="https://github.com/user-attachments/assets/acd590b8-e0ac-4e08-8ca3-7d4a39f5f4bc" />

---

## Block Diagram: Fully Pipelined Processor with Hazard Handling

<img width="1084" height="690" alt="image" src="https://github.com/user-attachments/assets/da449398-c181-483e-b0a5-491d1364909c" />

---

## Running the Simulation

```bash
# Compile (with SystemVerilog-2012 support)
iverilog -g2012 -o cpu_tb riscvpipeline.sv.sv

# Execute simulation
vvp cpu_tb

##  Acknowledgment
Dr. Satyajit Das
Assistant Professor
Department of Computer Science and Engineering
Indian Institute of Technology, Guwahati
