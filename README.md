# RISC-V-Assembler
designed and implemented a from-scratch assembler for the RISC-V RV32I ISA using C++. implemented support for all R, I, S, B, U, and J-type instructions, along with special cases like ecall, ebreak, and fence.

we designed and implemented a from-scratch assembler for the RISC-V RV32I ISA using C++. The assembler works in two passes: the first pass parses assembly code, builds a symbol table for labels, and tracks instruction addresses; the second pass encodes each instruction into its 32-bit machine code representation. We implemented support for all R, I, S, B, U, and J-type instructions, along with special cases like ecall, ebreak, and fence. To ensure correctness, immediate field range checks were added so that invalid or out-of-range constants generate clear error messages rather than silent failures.
We also extended the assembler with pseudo-instruction expansion (e.g., nop, mv, li, j, jr, ret), which translates user-friendly shortcuts into actual machine instructions. Additionally, we implemented directives like .word, .byte, .ascii, and .asciz, which allow embedding constants and strings into memory. This makes the assembler more practical for real program development.
Finally, we tested the assembler against provided instruction memory and register dump files to verify correctness. The project not only deepened our understanding of the RISC-V instruction encoding and assembly process but also provided hands-on experience in building a working assembler that bridges human-readable assembly with executable machine code.


Features:
·  Two-Pass Assembler Design
Pass 1: Parses lines, strips comments, records labels in a symbol table, and assigns instruction addresses.
Pass 2: Encodes instructions into 32-bit machine code using the symbol table.
·  Full RV32I Instruction Support
Implements R, I, S, B, U, and J types with correct encoding logic.
Handles load/store formats (imm(rs1)), branch offsets, and alignment checks.
·  Immediate Field Range Checking
Validates signed/unsigned immediates (12-bit for I/S, 13-bit for B, 21-bit for J).
Prevents out-of-range constants and misaligned branch/jump offsets.
·  Pseudo-Instruction Expansion
Supports common pseudo-instructions (nop, mv, li, j, jr, ret).
Expands these into equivalent real instructions (addi, jal, jalr, lui+addi).
·  Assembler Directives
.word – embed 32-bit constants.
.byte – embed 8-bit constants.
.ascii / .asciz – embed ASCII strings (with/without null termination).
·  Error Handling
Clear error messages for invalid registers, unknown mnemonics, syntax errors, and out-of-range immediates.
·  Readable Hex Output
Outputs each instruction as 8-digit uppercase hex, suitable for direct loading into memory.
Data directives produce correct-sized hex output.

Explanation:
Our assembler is built as a compact two-pass program for the RV32I instruction set. In the first pass, it scans the assembly source, removes comments, parses labels, and records their addresses into a symbol table. In the second pass, it uses this symbol table to correctly resolve labels and generate 32-bit machine code instructions. The assembler supports all core RISC-V instruction formats (R, I, S, B, U, J) and enforces strict range checking on immediate values so that errors are caught early. To make the assembler more practical, we extended it with pseudo-instruction expansion such as nop, mv, li, j, jr, and ret, which are internally translated into their real instruction equivalents. We also added support for useful assembler directives like .word, .byte, .ascii, and .asciz, which allow embedding data and strings directly into memory. The program is designed with careful error handling, ensuring that invalid registers, syntax errors, or out-of-range immediates are reported clearly. Finally, the output is produced as clean 8-digit hexadecimal instruction words, which can be directly used in a simulator or hardware implementation.

