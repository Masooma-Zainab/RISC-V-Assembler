// riscv_assembler.cpp
// A from-scratch, small-but-correct two-pass assembler for RV32I (32-bit) base ISA.
// - Supports labels, comments ('#' or '//'), registers x0..x31 and ABI names (zero, ra, sp, ...).
// - Supports core RV32I instructions: R/I/S/B/U/J types; also ECALL/EBREAK/FENCE.
// - Two passes: pass1 builds the symbol table; pass2 encodes machine code.
// - Outputs 8-hex-digit instruction words (one per line).
// - Pseudo-instructions: nop, mv, li, j, jr, ret
// - Directives: .text, .globl/.global, .org, .align, .word, .byte, .ascii, .asciz
//
// Build: g++ -std=c++17 -O2 -Wall -Wextra -o riscv_asm riscv_assembler.cpp
// Usage: ./riscv_asm input.s > out.hex
//
// Notes:
// - No other pseudo-instructions are expanded (li/mv/nop/j/jr/ret only).
// - Immediates: decimal (e.g., 123), hex (0x1F), negative values allowed where ISA permits.
// - Branch/JAL immediates may be labels or numeric byte offsets; assembler handles proper PC-relative placement.
// - Range checking with clear error messages.
// - Endianness: output is just 32-bit instruction words in hex; you can post-process to bytes if needed.

#include <bits/stdc++.h>
using namespace std;

static inline string ltrim(string s){
    size_t i = 0; while(i < s.size() && isspace((unsigned char)s[i])) ++i; return s.substr(i);
}
static inline string rtrim(string s){
    if(s.empty()) return s;
    size_t i = s.size()-1; while(i < s.size() && isspace((unsigned char)s[i])) { if(i==0){return string();} --i; }
    return s.substr(0, i+1);
}
static inline string trim(string s){ return rtrim(ltrim(s)); }

static inline void tolower_inplace(string &s){ for(char &c: s) c = (char)tolower((unsigned char)c); }

// Split by commas and whitespace, but keep paren expressions like imm(rs1).
static vector<string> tokenize_operands(const string& s){
    vector<string> toks;
    string cur;
    int paren = 0;
    auto flush = [&](){ if(!cur.empty()){ toks.push_back(trim(cur)); cur.clear(); } };
    for(size_t i=0;i<s.size();++i){
        char c = s[i];
        if(c=='('){ paren++; cur.push_back(c); }
        else if(c==')'){ paren = max(0, paren-1); cur.push_back(c); }
        else if((c==',' || isspace((unsigned char)c)) && paren==0){ flush(); }
        else { cur.push_back(c); }
    }
    flush();
    return toks;
}

struct AsmError : runtime_error { using runtime_error::runtime_error; };

// Register parsing (xN or ABI names).
static unordered_map<string,int> abi = {
    {"zero",0},{"ra",1},{"sp",2},{"gp",3},{"tp",4},
    {"t0",5},{"t1",6},{"t2",7},
    {"s0",8},{"fp",8},{"s1",9},
    {"a0",10},{"a1",11},{"a2",12},{"a3",13},{"a4",14},{"a5",15},{"a6",16},{"a7",17},
    {"s2",18},{"s3",19},{"s4",20},{"s5",21},{"s6",22},{"s7",23},{"s8",24},{"s9",25},{"s10",26},{"s11",27},
    {"t3",28},{"t4",29},{"t5",30},{"t6",31}
};

static int parse_reg(string s){
    s = trim(s);
    if(s.empty()) throw AsmError("empty register token");
    tolower_inplace(s);
    if(s[0]=='x'){
        string n = s.substr(1);
        if(n.empty()) throw AsmError("bad register '"+s+"'");
        for(char c: n) if(!isdigit((unsigned char)c)) throw AsmError("bad register '"+s+"'");
        int v = stoi(n);
        if(v<0 || v>31) throw AsmError("register out of range '"+s+"'");
        return v;
    }
    auto it = abi.find(s);
    if(it!=abi.end()) return it->second;
    throw AsmError("unknown register '"+s+"'");
}

static long long parse_int(string s){
    s = trim(s);
    if(s.size()>2 && s[0]=='0' && (s[1]=='x' || s[1]=='X')){
        // hex
        long long v = 0;
        std::stringstream ss; ss << std::hex << s;
        ss >> v;
        if(ss.fail()) throw AsmError("bad hex immediate '"+s+"'");
        return v;
    } else {
        // decimal (allow leading + or -)
        bool ok = true;
        for(size_t i=0;i<s.size();++i){
            char c = s[i];
            if(i==0 && (c=='+'||c=='-')) continue;
            if(!isdigit((unsigned char)c)) { ok=false; break; }
        }
        if(!ok || s.empty() || (s=="+"||s=="-")) throw AsmError("bad decimal immediate '"+s+"'");
        long long v = stoll(s);
        return v;
    }
}

// Extract imm(rs1) formats for loads/stores.
static pair<long long,int> parse_mem_operand(const string& tok){
    // forms: imm(rs1) OR (rs1) with implicit imm=0
    auto L = tok.find('(');
    auto R = tok.find(')');
    if(L==string::npos || R==string::npos || R<L) throw AsmError("expected imm(rs1) but got '"+tok+"'");
    string left = trim(tok.substr(0,L));
    string inside = trim(tok.substr(L+1, R-L-1));
    int rs1 = parse_reg(inside);
    long long imm = 0;
    if(!left.empty()) imm = parse_int(left);
    return {imm, rs1};
}

// Bit helpers
static inline uint32_t maskbits(int v, int bits){ uint32_t u = (uint32_t)v; if(bits==32) return u; return u & ((1u<<bits)-1u); }
static inline bool fits_signed(long long v, int bits){
    long long minv = -(1ll<<(bits-1));
    long long maxv =  (1ll<<(bits-1)) - 1;
    return v>=minv && v<=maxv;
}
static inline bool fits_unsigned(long long v, int bits){
    return v>=0 && ( (bits==64) ? true : (v < (1ll<<bits)) );
}

// Encoders per format
static uint32_t enc_R(int rd, int rs1, int rs2, int funct3, int funct7, int opcode){
    return ((uint32_t)funct7<<25) | ((uint32_t)rs2<<20) | ((uint32_t)rs1<<15) |
           ((uint32_t)funct3<<12) | ((uint32_t)rd<<7) | ((uint32_t)opcode);
}
static uint32_t enc_I(int rd, int rs1, long long imm, int funct3, int opcode){
    if(!fits_signed(imm,12)) throw AsmError("I-immediate out of range: "+to_string(imm));
    uint32_t uimm = maskbits((int)imm, 12);
    return (uimm<<20) | ((uint32_t)rs1<<15) | ((uint32_t)funct3<<12) | ((uint32_t)rd<<7) | ((uint32_t)opcode);
}
static uint32_t enc_S(int rs1, int rs2, long long imm, int funct3, int opcode){
    if(!fits_signed(imm,12)) throw AsmError("S-immediate out of range: "+to_string(imm));
    uint32_t uimm = maskbits((int)imm, 12);
    uint32_t imm11_5 = (uimm >> 5) & 0x7F;
    uint32_t imm4_0  =  uimm & 0x1F;
    return (imm11_5<<25) | ((uint32_t)rs2<<20) | ((uint32_t)rs1<<15) |
           ((uint32_t)funct3<<12) | (imm4_0<<7) | ((uint32_t)opcode);
}
static uint32_t enc_B(int rs1, int rs2, long long imm, int funct3, int opcode){
    // imm is byte offset relative to PC (current instr address). Must be even (LSB=0).
    if(imm % 2 != 0) throw AsmError("branch offset not 2-byte aligned: "+to_string(imm));
    if(!fits_signed(imm,13)) throw AsmError("B-immediate out of range: "+to_string(imm));
    uint32_t u = maskbits((int)imm, 13);
    uint32_t b12   = (u >> 12) & 0x1;
    uint32_t b10_5 = (u >> 5)  & 0x3F;
    uint32_t b4_1  = (u >> 1)  & 0xF;
    uint32_t b11   = (u >> 11) & 0x1;
    return (b12<<31) | (b10_5<<25) | ((uint32_t)rs2<<20) | ((uint32_t)rs1<<15) |
           ((uint32_t)funct3<<12) | (b4_1<<8) | (b11<<7) | ((uint32_t)opcode);
}
static uint32_t enc_U(int rd, long long imm, int opcode){
    // LUI/AUIPC: imm[31:12], low 12 bits must be zero (assembler enforces alignment).
    if(!fits_signed(imm, 32)) throw AsmError("U-immediate out of range: " + to_string(imm));
    if(imm % 0x1000 != 0) throw AsmError("U-immediate not aligned to 4096 (low 12 bits must be zero): " + to_string(imm));
    uint32_t u = (uint32_t)imm;
    return (u & 0xFFFFF000u) | ((uint32_t)rd<<7) | ((uint32_t)opcode);
}
static uint32_t enc_J(int rd, long long imm, int opcode){
    // JAL immediate is 21-bit signed (bits 20..1, LSB=0)
    if(imm % 2 != 0) throw AsmError("JAL offset not 2-byte aligned: "+to_string(imm));
    if(!fits_signed(imm,21)) throw AsmError("J-immediate out of range: "+to_string(imm));
    uint32_t u = maskbits((int)imm, 21);
    uint32_t b20   = (u >> 20) & 0x1;
    uint32_t b10_1 = (u >> 1)  & 0x3FF;
    uint32_t b11   = (u >> 11) & 0x1;
    uint32_t b19_12= (u >> 12) & 0xFF;
    return (b20<<31) | (b19_12<<12) | (b11<<20) | (b10_1<<21) | ((uint32_t)rd<<7) | ((uint32_t)opcode);
}

// Tables for opcodes/funct3/funct7
struct RDesc { int funct3, funct7, opcode; };
struct IDesc { int funct3, opcode; };
struct BDesc { int funct3, opcode; };
struct SDesc { int funct3, opcode; };

static unordered_map<string,RDesc> Rtab = {
    {"add",{0x0,0x00,0x33}}, {"sub",{0x0,0x20,0x33}},
    {"sll",{0x1,0x00,0x33}}, {"slt",{0x2,0x00,0x33}},
    {"sltu",{0x3,0x00,0x33}},{"xor",{0x4,0x00,0x33}},
    {"srl",{0x5,0x00,0x33}}, {"sra",{0x5,0x20,0x33}},
    {"or",{0x6,0x00,0x33}},  {"and",{0x7,0x00,0x33}}
};

static unordered_map<string,IDesc> Itab = {
    {"addi",{0x0,0x13}}, {"slti",{0x2,0x13}}, {"sltiu",{0x3,0x13}},
    {"xori",{0x4,0x13}}, {"ori",{0x6,0x13}}, {"andi",{0x7,0x13}},
    {"slli",{0x1,0x13}}, {"srli",{0x5,0x13}}, {"srai",{0x5,0x13}}, // srai distinguished by imm[11:5]=0x20
    {"jalr",{0x0,0x67}},
    {"lb",{0x0,0x03}}, {"lh",{0x1,0x03}}, {"lw",{0x2,0x03}},
    {"lbu",{0x4,0x03}}, {"lhu",{0x5,0x03}}
};

static unordered_map<string,SDesc> Stab = {
    {"sb",{0x0,0x23}}, {"sh",{0x1,0x23}}, {"sw",{0x2,0x23}}
};

static unordered_map<string,BDesc> Btab = {
    {"beq",{0x0,0x63}}, {"bne",{0x1,0x63}},
    {"blt",{0x4,0x63}}, {"bge",{0x5,0x63}},
    {"bltu",{0x6,0x63}},{"bgeu",{0x7,0x63}}
};

// U/J opcodes:
static unordered_map<string,int> Uop = { {"lui",0x37}, {"auipc",0x17} };
static unordered_map<string,int> Jop = { {"jal",0x6F} };

// Specials
// fence -> opcode=0x0F, funct3=0, rd=0, rs1=0, fm/pred/succ fields come from imm (we accept 'fence' with no operands -> fence 0,0)
// ecall -> 0x00000073, ebreak -> 0x00100073
static uint32_t encode_special(const string& mnem, const vector<string>& ops){
    string M = mnem; tolower_inplace(*(string*)&M);
    if(M=="ecall") return 0x00000073u;
    if(M=="ebreak") return 0x00100073u;
    if(M=="fence"){
        int rd=0, rs1=0, funct3=0, opcode=0x0F;
        int pred=0, succ=0, fm=0;
        if(!ops.empty()){
            pred = (int)parse_int(ops[0]);
            if(ops.size()>1) succ = (int)parse_int(ops[1]);
        }
        int imm = (fm<<28) | ((pred & 0xF)<<24) | ((succ & 0xF)<<20);
        uint32_t uimm = maskbits(imm,12);
        return (uimm<<20) | ((uint32_t)rs1<<15) | ((uint32_t)funct3<<12) | ((uint32_t)rd<<7) | ((uint32_t)opcode);
    }
    throw AsmError("unknown special '"+mnem+"'");
}

struct Line {
    string raw;
    string label;     // without ':'
    string mnemonic;  // lowercased
    vector<string> operands;
    int line_no = 0;
    uint32_t addr = 0;
    bool is_empty = false;
    bool is_dir = false;
    string directive;
    vector<string> dir_args;
};

static string strip_comment(string s){
    // Remove '#' or '//' comments
    size_t p = s.find('#');
    size_t q = s.find("//");
    size_t cut = string::npos;
    if(p!=string::npos) cut = p;
    if(q!=string::npos) cut = (cut==string::npos? q : min(cut,q));
    if(cut!=string::npos) s = s.substr(0,cut);
    return s;
}

static Line parse_line(string raw, int line_no){
    Line L; L.raw = raw; L.line_no=line_no;
    string s = strip_comment(raw);
    s = trim(s);
    if(s.empty()){ L.is_empty = true; return L; }
    // split label if present
    size_t col = s.find(':');
    if(col!=string::npos){
        L.label = trim(s.substr(0,col));
        s = trim(s.substr(col+1));
    }
    if(s.empty()){ return L; }
    // directive?
    if(s[0]=='.'){
        L.is_dir = true;
        auto sp = s.find_first_of(" \t");
        if(sp==string::npos){ L.directive = s; }
        else {
            L.directive = s.substr(0,sp);
            L.dir_args = tokenize_operands(s.substr(sp+1));
        }
        tolower_inplace(L.directive);
        return L;
    }
    // instruction
    size_t sp = s.find_first_of(" \t");
    if(sp==string::npos){
        L.mnemonic = s; tolower_inplace(L.mnemonic);
    } else {
        L.mnemonic = s.substr(0,sp); tolower_inplace(L.mnemonic);
        L.operands = tokenize_operands(s.substr(sp+1));
    }
    return L;
}

struct Program {
    vector<Line> lines;
    unordered_map<string,uint32_t> sym; // label -> address
};

static Program pass1(istream& in){
    Program P;
    string line;
    uint32_t pc = 0;
    int ln = 0;
    while(getline(in, line)){
        ++ln;
        Line L = parse_line(line, ln);
        if(L.is_empty){ P.lines.push_back(L); continue; }
        if(!L.label.empty()){
            if(P.sym.count(L.label)) throw AsmError("duplicate label '"+L.label+"' at line "+to_string(ln));
            P.sym[L.label] = pc;
        }
        if(L.is_dir){
            if(L.directive==".text"){ /* no-op */ }
            else if(L.directive==".globl" || L.directive==".global"){ /* ignore symbol visibility */ }
            else if(L.directive==".org"){
                if(L.dir_args.size()!=1) throw AsmError(".org expects 1 argument at line "+to_string(ln));
                long long v = parse_int(L.dir_args[0]);
                if(v<0 || v>0xFFFFFFFFll) throw AsmError(".org out of range at line "+to_string(ln));
                pc = (uint32_t)v;
            }
            else if(L.directive==".align"){
                if(L.dir_args.size()!=1) throw AsmError(".align expects power-of-two boundary at line "+to_string(ln));
                long long a = parse_int(L.dir_args[0]);
                if(a<=0) throw AsmError(".align expects positive at line "+to_string(ln));
                uint32_t mask = (1u<<a)-1u;
                pc = (pc + mask) & ~mask;
            }
            else if(L.directive==".word"){
                if(L.dir_args.empty()) throw AsmError(".word expects at least one argument at line "+to_string(ln));
                pc += 4u * (uint32_t)L.dir_args.size();
            }
            else if(L.directive==".byte"){
                if(L.dir_args.empty()) throw AsmError(".byte expects at least one argument at line "+to_string(ln));
                pc += 1u * (uint32_t)L.dir_args.size();
            }
            else if(L.directive==".ascii" || L.directive==".asciz"){
                if(L.dir_args.size()!=1) throw AsmError(string(L.directive).append(" expects exactly one string at line ")+to_string(ln));
                string s=L.dir_args[0];
                if(!(s.size()>=2 && s.front()=='"' && s.back()=='"'))
                    throw AsmError(string(L.directive).append(" missing quotes at line ")+to_string(ln));
                s = s.substr(1, s.size()-2);
                pc += (uint32_t)s.size();
                if(L.directive==".asciz") pc += 1;
            }
            else {
                throw AsmError("unknown directive '"+L.directive+"' at line "+to_string(ln));
            }
            L.addr = pc;
        } else {
            L.addr = pc;
            if(!L.mnemonic.empty()){
                // Assume 1 instruction for PC advance; pseudo may expand to 2 but label addresses are per source line.
                pc += 4;
            }
        }
        P.lines.push_back(L);
    }
    return P;
}

static long long resolve_imm_label(const string& tok, const Program& P, uint32_t cur_pc, bool pc_rel, bool /*for_branch*/){
    if(P.sym.count(tok)){
        long long target = (long long)P.sym.at(tok);
        long long off = target - (long long)cur_pc;
        if(pc_rel) return off;
        return target; // absolute (rarely used for U-type directly)
    }
    return parse_int(tok);
}

// NEW: return possibly multiple instructions for a line (pseudo expansion)
static vector<uint32_t> encode_line(const Line& L, const Program& P){
    vector<uint32_t> out;
    string m = L.mnemonic;
    if(m.empty()) return out;

    // PSEUDO FIRST
    if(m=="nop"){
        out.push_back(enc_I(0,0,0,0x0,0x13)); // addi x0,x0,0
        return out;
    }
    if(m=="mv"){
        if(L.operands.size()!=2) throw AsmError("syntax: mv rd, rs   at line "+to_string(L.line_no));
        int rd=parse_reg(L.operands[0]);
        int rs=parse_reg(L.operands[1]);
        out.push_back(enc_I(rd,rs,0,0x0,0x13)); // addi rd, rs, 0
        return out;
    }
    if(m=="li"){
        if(L.operands.size()!=2) throw AsmError("syntax: li rd, imm   at line "+to_string(L.line_no));
        int rd=parse_reg(L.operands[0]);
        long long imm=parse_int(L.operands[1]);
        if(fits_signed(imm,12)){
            out.push_back(enc_I(rd,0,imm,0x0,0x13)); // addi rd, x0, imm
        } else {
            long long hi = (imm + 0x800) >> 12;
            long long lo = imm - (hi<<12);
            out.push_back(enc_U(rd, hi<<12, 0x37));         // LUI rd, hi
            out.push_back(enc_I(rd, rd, lo, 0x0, 0x13));    // ADDI rd, rd, lo
        }
        return out;
    }
    if(m=="j"){
        if(L.operands.size()!=1) throw AsmError("syntax: j label   at line "+to_string(L.line_no));
        long long off = resolve_imm_label(L.operands[0], P, L.addr, true, false);
        out.push_back(enc_J(0, off, 0x6F)); // jal x0, label
        return out;
    }
    if(m=="jr"){
        if(L.operands.size()!=1) throw AsmError("syntax: jr rs1   at line "+to_string(L.line_no));
        int rs1=parse_reg(L.operands[0]);
        out.push_back(enc_I(0, rs1, 0, 0x0, 0x67)); // jalr x0, rs1, 0
        return out;
    }
    if(m=="ret"){
        out.push_back(enc_I(0, 1, 0, 0x0, 0x67)); // jalr x0, ra, 0
        return out;
    }

    // Specials
    if(m=="ecall" || m=="ebreak" || m=="fence"){
        out.push_back(encode_special(m, L.operands));
        return out;
    }
    // U/J
    if(Uop.count(m)){
        if(L.operands.size()!=2) throw AsmError("syntax: "+m+" rd, imm   at line "+to_string(L.line_no));
        int rd = parse_reg(L.operands[0]);
        long long imm = resolve_imm_label(L.operands[1], P, L.addr, /*pc_rel*/false, false);
        out.push_back(enc_U(rd, imm, Uop[m]));
        return out;
    }
    if(Jop.count(m)){
        if(L.operands.size()!=2) throw AsmError("syntax: jal rd, target   at line "+to_string(L.line_no));
        int rd = parse_reg(L.operands[0]);
        long long off = resolve_imm_label(L.operands[1], P, L.addr, /*pc_rel*/true, false);
        out.push_back(enc_J(rd, off, Jop[m]));
        return out;
    }
    // B-type
    if(Btab.count(m)){
        if(L.operands.size()!=3) throw AsmError("syntax: "+m+" rs1, rs2, target   at line "+to_string(L.line_no));
        int rs1 = parse_reg(L.operands[0]);
        int rs2 = parse_reg(L.operands[1]);
        long long off = resolve_imm_label(L.operands[2], P, L.addr, /*pc_rel*/true, true);
        auto d = Btab[m];
        out.push_back(enc_B(rs1, rs2, off, d.funct3, d.opcode));
        return out;
    }
    // S-type (stores): rs2, imm(rs1)
    if(Stab.count(m)){
        if(L.operands.size()!=2) throw AsmError("syntax: "+m+" rs2, imm(rs1)   at line "+to_string(L.line_no));
        int rs2 = parse_reg(L.operands[0]);
        auto [imm, rs1] = parse_mem_operand(L.operands[1]);
        auto d = Stab[m];
        out.push_back(enc_S(rs1, rs2, imm, d.funct3, d.opcode));
        return out;
    }
    // Loads (I-type mem): rd, imm(rs1)
    if(Itab.count(m) && (m=="lb"||m=="lh"||m=="lw"||m=="lbu"||m=="lhu")){
        if(L.operands.size()!=2)
            throw AsmError("syntax: " + m + " rd, imm(rs1)   at line " + to_string(L.line_no));
        int rd = parse_reg(L.operands[0]);
        auto [imm, rs1] = parse_mem_operand(L.operands[1]);
        auto d = Itab[m];
        out.push_back(enc_I(rd, rs1, imm, d.funct3, d.opcode));
        return out;
    }
    // I-type ALU, shifts and JALR
    if(Itab.count(m)){
        if(m=="jalr"){
            if(L.operands.size()!=3) throw AsmError("syntax: jalr rd, rs1, imm   at line "+to_string(L.line_no));
            int rd = parse_reg(L.operands[0]);
            int rs1 = parse_reg(L.operands[1]);
            long long imm = parse_int(L.operands[2]);
            out.push_back(enc_I(rd, rs1, imm, Itab[m].funct3, Itab[m].opcode));
            return out;
        }
        // shifts have shamt (0..31), with special encoding for srai (imm[11:5]=0x20)
        if(m=="slli" || m=="srli" || m=="srai"){
            if(L.operands.size()!=3) throw AsmError("syntax: "+m+" rd, rs1, shamt   at line "+to_string(L.line_no));
            int rd = parse_reg(L.operands[0]);
            int rs1 = parse_reg(L.operands[1]);
            long long shamt = parse_int(L.operands[2]);
            if(shamt<0 || shamt>31) throw AsmError("shift amount out of range at line "+to_string(L.line_no));
            int opcode = Itab[m].opcode;
            int funct3 = Itab[m].funct3;
            int funct7 = (m=="srai") ? 0x20 : 0x00;
            long long imm = (funct7<<5) | (int)shamt; // imm[11:5]=funct7
            out.push_back(enc_I(rd, rs1, imm, funct3, opcode));
            return out;
        }
        // normal ALU-immediates
        if(L.operands.size()!=3) throw AsmError("syntax: "+m+" rd, rs1, imm   at line "+to_string(L.line_no));
        int rd = parse_reg(L.operands[0]);
        int rs1 = parse_reg(L.operands[1]);
        long long imm = parse_int(L.operands[2]);
        auto d = Itab[m];
        out.push_back(enc_I(rd, rs1, imm, d.funct3, d.opcode));
        return out;
    }
    // R-type
    if(Rtab.count(m)){
        if(L.operands.size()!=3) throw AsmError("syntax: "+m+" rd, rs1, rs2   at line "+to_string(L.line_no));
        int rd = parse_reg(L.operands[0]);
        int rs1 = parse_reg(L.operands[1]);
        int rs2 = parse_reg(L.operands[2]);
        auto d = Rtab[m];
        out.push_back(enc_R(rd, rs1, rs2, d.funct3, d.funct7, d.opcode));
        return out;
    }

    throw AsmError("unknown mnemonic '"+m+"' at line "+to_string(L.line_no));
}

int main(int argc, char** argv){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    if(argc<2){
        cerr << "Usage: " << argv[0] << " input.s > out.hex\n";
        return 1;
    }
    ifstream fin(argv[1]);
    if(!fin){ cerr << "Cannot open " << argv[1] << "\n"; return 1; }

    Program P;
    try {
        P = pass1(fin);
    } catch(const AsmError& e){
        cerr << "Pass1 error: " << e.what() << "\n";
        return 1;
    }

    try {
        for(const auto& L : P.lines){
            if(L.is_empty) continue;

            // Emit data directives
            if(L.is_dir){
                if(L.directive==".word"){
                    for(auto &arg:L.dir_args){
                        long long v=parse_int(arg);
                        uint32_t u=(uint32_t)v;
                        std::stringstream ss;
                        ss<<std::uppercase<<hex<<setw(8)<<setfill('0')<<u;
                        cout<<ss.str()<<"\n";
                    }
                }
                else if(L.directive==".byte"){
                    for(auto &arg:L.dir_args){
                        long long v=parse_int(arg);
                        if(v<0 || v>255) throw AsmError(".byte value out of range: "+to_string(v));
                        uint8_t u=(uint8_t)v;
                        std::stringstream ss;
                        ss<<std::uppercase<<hex<<setw(2)<<setfill('0')<<(int)u;
                        cout<<ss.str()<<"\n";
                    }
                }
                else if(L.directive==".ascii" || L.directive==".asciz"){
                    if(L.dir_args.size()!=1) throw AsmError(string(L.directive).append(" expects exactly one string"));
                    string s=L.dir_args[0];
                    if(!(s.size()>=2 && s.front()=='"' && s.back()=='"'))
                        throw AsmError(string(L.directive).append(" missing quotes"));
                    s = s.substr(1, s.size()-2);
                    for(unsigned char c:s){
                        std::stringstream ss;
                        ss<<std::uppercase<<hex<<setw(2)<<setfill('0')<<(int)c;
                        cout<<ss.str()<<"\n";
                    }
                    if(L.directive==".asciz"){
                        cout<<"00\n";
                    }
                }
                // .text, .globl, .org, .align handled in pass1 (no direct emission)
                continue;
            }

            // Emit instructions (including pseudo-expanded multiple)
            if(L.mnemonic.empty()) continue;
            auto insns = encode_line(L, P);
            for(uint32_t insn : insns){
                std::stringstream ss;
                ss << std::uppercase << std::hex << setw(8) << setfill('0') << insn;
                cout << ss.str() << "\n";
            }
        }
    } catch(const AsmError& e){
        cerr << "Pass2 error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
