// riscv_sim.cpp
// Minimal educational RV32I simulator (RV32I base ISA only) to pair with the assembler.
// - Loads 32-bit instructions from a text .hex file (one 8-hex-digit word per line, optional 0x prefix).
// - Places instructions in memory starting at PC=0, little-endian.
// - Executes sequentially with branches/jumps; halts on ECALL/EBREAK or when PC leaves code range.
// - Supports R/I/S/B/U/J, loads/stores (byte/half/word), JAL/JALR, AUIPC/LUI, ALU-immediates, ALU-R.
// - Writes register dump to regfile.hex (32 lines, 8 hex digits uppercase).
// - Optional CLI args: --max-steps=N, --init-sp=0xVALUE (set x2/sp), --dump-mem=memdump.bin
//
// Build: g++ -std=c++17 -O2 -Wall -Wextra -o riscv_sim riscv_sim.cpp
// Run:   ./riscv_sim program.hex [--max-steps=1000000] [--init-sp=0x2000] [--dump-mem=mem.bin]
//
// Note: This is a simple, zero-privilege simulator (no CSRs, no exceptions, no MMU).
//
#include <bits/stdc++.h>
using namespace std;

static inline uint32_t sext32(uint32_t v, int bits){
    uint32_t m = 1u << (bits-1);
    return (v ^ m) - m;
}
static inline int32_t sext(int32_t v, int bits){
    int32_t m = 1 << (bits-1);
    return (v ^ m) - m;
}

struct CPU {
    uint32_t regs[32]{}; // x0..x31
    uint32_t pc = 0;
};

struct Mem {
    vector<uint8_t> bytes;
    explicit Mem(size_t size): bytes(size, 0) {}
    size_t size() const { return bytes.size(); }
    uint8_t  lb(uint32_t addr) const { return bytes[addr]; }
    uint16_t lh_raw(uint32_t addr) const { return bytes[addr] | (uint16_t(bytes[addr+1])<<8); }
    uint32_t lw_raw(uint32_t addr) const { return uint32_t(bytes[addr]) | (uint32_t(bytes[addr+1])<<8) | (uint32_t(bytes[addr+2])<<16) | (uint32_t(bytes[addr+3])<<24); }
    void sb(uint32_t addr, uint8_t v){ bytes[addr] = v; }
    void sh(uint32_t addr, uint16_t v){ bytes[addr] = v & 0xFF; bytes[addr+1] = (v>>8) & 0xFF; }
    void sw(uint32_t addr, uint32_t v){ bytes[addr] = v & 0xFF; bytes[addr+1] = (v>>8) & 0xFF; bytes[addr+2] = (v>>16) & 0xFF; bytes[addr+3] = (v>>24) & 0xFF; }
};

static bool read_hex_program(const string& path, vector<uint32_t>& prog){
    ifstream fin(path);
    if(!fin) return false;
    string line;
    while(getline(fin, line)){
        // strip spaces
        line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
        if(line.empty()) continue;
        if(line.rfind("0x",0)==0 || line.rfind("0X",0)==0) line = line.substr(2);
        if(line.size()<1) continue;
        // take up to 8 hex chars (ignore comments if any)
        size_t n = 0;
        while(n<line.size() && isxdigit((unsigned char)line[n])) ++n;
        string word = line.substr(0, min<size_t>(8, n));
        if(word.empty()) continue;
        uint32_t val = 0;
        std::stringstream ss; ss << std::hex << word; ss >> val;
        prog.push_back(val);
    }
    return true;
}

static uint32_t get_bits(uint32_t x, int hi, int lo){
    return (x >> lo) & ((1u << (hi-lo+1)) - 1u);
}

int main(int argc, char** argv){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    if(argc<2){
        cerr << "Usage: " << argv[0] << " program.hex [--max-steps=N] [--init-sp=0xVAL] [--dump-mem=mem.bin]\n";
        return 1;
    }
    string hexfile = argv[1];
    uint64_t max_steps = 1000000;
    uint32_t init_sp = 0; // default 0; can be set via flag
    string dump_mem_path;

    for(int i=2;i<argc;++i){
        string a = argv[i];
        if(a.rfind("--max-steps=",0)==0){
            max_steps = strtoull(a.c_str()+12, nullptr, 0);
        } else if(a.rfind("--init-sp=",0)==0){
            init_sp = strtoul(a.c_str()+10, nullptr, 0);
        } else if(a.rfind("--dump-mem=",0)==0){
            dump_mem_path = a.substr(11);
        } else {
            cerr << "Unknown arg: " << a << "\n";
            return 1;
        }
    }

    vector<uint32_t> prog;
    if(!read_hex_program(hexfile, prog)){
        cerr << "Cannot read " << hexfile << "\n";
        return 1;
    }

    // Allocate memory: code + some extra stack/heap space (e.g., 1MB)
    const size_t MEM_SIZE = 1<<20; // 1 MiB
    Mem mem(MEM_SIZE);

    // Load program words at addr 0, little-endian
    for(size_t i=0;i<prog.size();++i){
        uint32_t w = prog[i];
        mem.sw(uint32_t(i*4), w);
    }

    CPU cpu;
    memset(cpu.regs, 0, sizeof(cpu.regs));
    cpu.pc = 0;
    cpu.regs[2] = init_sp; // sp

    auto x = [&](int i)->uint32_t& { return cpu.regs[i]; };

    auto read32 = [&](uint32_t addr)->uint32_t { return mem.lw_raw(addr); };
    auto read16 = [&](uint32_t addr)->uint16_t { return mem.lh_raw(addr); };
    auto read8  = [&](uint32_t addr)->uint8_t  { return mem.lb(addr); };

    auto write8  = [&](uint32_t addr, uint8_t v){ mem.sb(addr, v); };
    auto write16 = [&](uint32_t addr, uint16_t v){ mem.sh(addr, v); };
    auto write32 = [&](uint32_t addr, uint32_t v){ mem.sw(addr, v); };

    uint64_t steps = 0;
    bool halted = false;

    while(!halted && steps < max_steps){
        if(cpu.pc >= prog.size()*4){
            // PC fell off the loaded program -> halt
            break;
        }
        uint32_t insn = read32(cpu.pc);
        uint32_t opcode = insn & 0x7F;
        uint32_t rd  = get_bits(insn, 11, 7);
        uint32_t funct3 = get_bits(insn, 14, 12);
        uint32_t rs1 = get_bits(insn, 19, 15);
        uint32_t rs2 = get_bits(insn, 24, 20);
        uint32_t funct7 = get_bits(insn, 31, 25);

        uint32_t next_pc = cpu.pc + 4;

        auto imm_i = (int32_t)sext32(get_bits(insn,31,20), 12);
        auto imm_u = insn & 0xFFFFF000u;
        auto imm_s = (int32_t)sext32( (get_bits(insn,31,25)<<5) | get_bits(insn,11,7), 12);
        auto imm_b = (int32_t)sext32( (get_bits(insn,31,31)<<12) | (get_bits(insn,7,7)<<11) | (get_bits(insn,30,25)<<5) | (get_bits(insn,11,8)<<1), 13);
        auto imm_j = (int32_t)sext32( (get_bits(insn,31,31)<<20) | (get_bits(insn,19,12)<<12) | (get_bits(insn,20,20)<<11) | (get_bits(insn,30,21)<<1), 21);

        switch(opcode){
            case 0x33: { // R-type
                switch(funct3){
                    case 0x0: x(rd) = (funct7==0x20) ? (x(rs1) - x(rs2)) : (x(rs1) + x(rs2)); break; // sub/add
                    case 0x1: x(rd) = (x(rs1) << (x(rs2)&31)); break;
                    case 0x2: x(rd) = (int32_t)x(rs1) < (int32_t)x(rs2); break;
                    case 0x3: x(rd) = (uint32_t)x(rs1) < (uint32_t)x(rs2); break;
                    case 0x4: x(rd) = x(rs1) ^ x(rs2); break;
                    case 0x5: x(rd) = (funct7==0x20) ? ( (int32_t)x(rs1) >> (x(rs2)&31) )
                                                     : ( x(rs1) >> (x(rs2)&31) ); break;
                    case 0x6: x(rd) = x(rs1) | x(rs2); break;
                    case 0x7: x(rd) = x(rs1) & x(rs2); break;
                    default: /*unknown*/ break;
                }
                break;
            }
            case 0x13: { // I-type ALU & shifts
                switch(funct3){
                    case 0x0: x(rd) = x(rs1) + imm_i; break; // addi
                    case 0x2: x(rd) = (int32_t)x(rs1) < imm_i; break; // slti
                    case 0x3: x(rd) = (uint32_t)x(rs1) < (uint32_t)imm_i; break; // sltiu
                    case 0x4: x(rd) = x(rs1) ^ imm_i; break; // xori
                    case 0x6: x(rd) = x(rs1) | imm_i; break; // ori
                    case 0x7: x(rd) = x(rs1) & imm_i; break; // andi
                    case 0x1: { // slli
                        uint32_t shamt = get_bits(insn,24,20);
                        x(rd) = x(rs1) << (shamt & 31);
                        break;
                    }
                    case 0x5: { // srli/srai
                        uint32_t shamt = get_bits(insn,24,20);
                        if(funct7==0x20) x(rd) = (int32_t)x(rs1) >> (shamt & 31);
                        else             x(rd) = x(rs1) >> (shamt & 31);
                        break;
                    }
                }
                break;
            }
            case 0x03: { // Loads
                uint32_t addr = x(rs1) + imm_i;
                switch(funct3){
                    case 0x0: x(rd) = (int32_t)(int8_t)read8(addr); break;   // lb
                    case 0x1: x(rd) = (int32_t)(int16_t)read16(addr); break; // lh
                    case 0x2: x(rd) = read32(addr); break;                   // lw
                    case 0x4: x(rd) = (uint32_t)read8(addr); break;          // lbu
                    case 0x5: x(rd) = (uint32_t)read16(addr); break;         // lhu
                }
                break;
            }
            case 0x23: { // Stores
                uint32_t addr = x(rs1) + imm_s;
                switch(funct3){
                    case 0x0: write8(addr,  x(rs2) & 0xFF); break;   // sb
                    case 0x1: write16(addr, x(rs2) & 0xFFFF); break; // sh
                    case 0x2: write32(addr, x(rs2)); break;          // sw
                }
                break;
            }
            case 0x63: { // Branches
                bool take=false;
                switch(funct3){
                    case 0x0: take = (x(rs1)==x(rs2)); break; // beq
                    case 0x1: take = (x(rs1)!=x(rs2)); break; // bne
                    case 0x4: take = ((int32_t)x(rs1) <  (int32_t)x(rs2)); break; // blt
                    case 0x5: take = ((int32_t)x(rs1) >= (int32_t)x(rs2)); break; // bge
                    case 0x6: take = ((uint32_t)x(rs1) <  (uint32_t)x(rs2)); break; // bltu
                    case 0x7: take = ((uint32_t)x(rs1) >= (uint32_t)x(rs2)); break; // bgeu
                }
                if(take) next_pc = cpu.pc + imm_b;
                break;
            }
            case 0x37: { // LUI
                x(rd) = imm_u;
                break;
            }
            case 0x17: { // AUIPC
                x(rd) = cpu.pc + imm_u;
                break;
            }
            case 0x6F: { // JAL
                x(rd) = next_pc;
                next_pc = cpu.pc + imm_j;
                break;
            }
            case 0x67: { // JALR
                x(rd) = next_pc;
                next_pc = (x(rs1) + imm_i) & ~1u;
                break;
            }
            case 0x73: { // SYSTEM (ecall/ebreak)
                uint32_t imm = get_bits(insn,31,20);
                if(imm==0x000 || imm==0x001){
                    halted = true; // ECALL/EBREAK -> halt
                }
                break;
            }
            default:
                // Unknown opcode; halt for safety
                halted = true;
                break;
        }

        cpu.pc = next_pc;
        x(0) = 0; // enforce x0=0
        ++steps;
    }

    // Write regfile.hex
    ofstream rf("regfile.hex");
    if(!rf){ cerr << "Cannot write regfile.hex\n"; return 1; }
    rf << std::uppercase << std::hex << setfill('0');
    for(int i=0;i<32;++i){
        rf << setw(8) << cpu.regs[i] << "\n";
    }
    rf.close();

    // Optional memory dump
    if(!dump_mem_path.empty()){
        ofstream dm(dump_mem_path, ios::binary);
        if(dm){
            dm.write(reinterpret_cast<const char*>(mem.bytes.data()), mem.bytes.size());
        }
    }

    // Also print a tiny summary to stdout
    cout << "HALT after " << steps << " steps. PC=0x" << std::uppercase << std::hex << setw(8) << cpu.pc << "\n";
    return 0;
}
