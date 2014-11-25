#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <vector>
#include "consts.h"
#include "options.h"
#include "ils.h"
#include "qkfpu.h"
using namespace std;

static const char instnames[INSTRUCTION_NAME_MAX][8] = {
  "sll", "srl", "sra", "sllv", "srlv", "srav", "jr", "jalr", "addu", "subu",
  "and", "or", "xor", "nor", "slt", "sltu", "j", "jal", "beq", "bne",
  "addiu", "slti", "sltiu", "andi", "ori", "xori", "lui", "lw", "sw",
  "bc1f", "bc1t", "mtc1", "mfc1", "add.s", "sub.s", "mul.s", "div.s",
  "sqrt.s", "mov.s", "cvt.s.w", "cvt.w.s", "c.eq.s", "c.olt.s", "c.ole.s"
};

static const char regnames[32][5] = {
  "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
  "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
  "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
};
static const char fregnames[32][4] = {
  "f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7",
  "f8", "f9", "f10", "f11", "f12", "f13", "f14", "f15",
  "f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
  "f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31"
};

static uint32_t ram[1<<20];
static const int rs232c_recv_count = 2;
static int rs232c_recv_status;
static uint32_t rs232c_recv_data;
static const int rs232c_send_count = 2;
static int rs232c_send_status;

static void rs232c_prereceive() {
  unsigned char ch;
  size_t readsize = fread(&ch,1,1,stdin);
  if(readsize<1) {
    if(feof(stdin)) {
      rs232c_recv_status = -1;
      return;
    } else {
      fprintf(stderr, "error: reading from input\n");
      exit(1);
    }
  }
  rs232c_recv_status = rs232c_recv_count-1;
  rs232c_recv_data = ch;
}

static int64_t instruction_count_all;
static int64_t instruction_counts[INSTRUCTION_NAME_MAX];
static int64_t branch_counts[1<<15];
static int ils_run() {
  instruction_count_all = 0;
  fill(instruction_counts, instruction_counts+INSTRUCTION_NAME_MAX, 0);
  fill(branch_counts, branch_counts+(1<<15), 0);
  int pc = 0;
  uint32_t reg[32], freg[32];
  bool cc0 = false;
  std::fill(reg, reg+32, 0);
  std::fill(freg, freg+32, 0);
  rs232c_prereceive();
  for(;;) {
    if(pc < 0 || pc >= (1<<15)) {
      fprintf(stderr, "error: program counter 0x%08x is out of range\n",
          pc*4);
      exit(1);
    }
    uint32_t pword = ram[pc];
    int opcode = pword>>26;
    int rs = (pword>>21)&31;
    int rt = (pword>>16)&31;
    int rd = (pword>>11)&31;
    int sa = (pword>> 6)&31;
    int funct = pword&63;
    int fmt = rs;
    int ft = rt;
    int fs = rd;
    int fd = sa;
    uint32_t uimm16 = (uint16_t)pword;
    uint32_t simm16 = (int16_t)pword;
    int jt = (pc>>26<<26)|(pword&((1U<<26)-1));
    bool is_branch = false;
    bool branch_success = false;
    int branch_target = -1;
    int set_reg = 0;
    uint32_t set_reg_val = 0;
    int set_freg = -1;
    uint32_t set_freg_val = 0;
    switch(opcode) {
      case OPCODE_SPECIAL:
        switch(funct) {
          case FUNCT_SLL:
            set_reg = rd;
            set_reg_val = reg[rt] << sa;
            ++instruction_counts[INSTRUCTION_NAME_SLL];
            break;
          case FUNCT_SRL:
            set_reg = rd;
            set_reg_val = reg[rt] >> sa;
            ++instruction_counts[INSTRUCTION_NAME_SRL];
            break;
          case FUNCT_SRA:
            set_reg = rd;
            set_reg_val = (int32_t)reg[rt] >> sa;
            ++instruction_counts[INSTRUCTION_NAME_SRA];
            break;
          case FUNCT_SLLV:
            set_reg = rd;
            set_reg_val = reg[rt] << (reg[rs]&31);
            ++instruction_counts[INSTRUCTION_NAME_SLLV];
            break;
          case FUNCT_SRLV:
            set_reg = rd;
            set_reg_val = reg[rt] >> (reg[rs]&31);
            ++instruction_counts[INSTRUCTION_NAME_SRLV];
            break;
          case FUNCT_SRAV:
            set_reg = rd;
            set_reg_val = (int32_t)reg[rt] >> (reg[rs]&31);
            ++instruction_counts[INSTRUCTION_NAME_SRAV];
            break;
          case FUNCT_JR:
            if(reg[rs]&3) {
              fprintf(stderr, "error: JR: unaligned jump: 0x%08x\n", reg[rs]);
              exit(1);
            }
            is_branch = true;
            branch_success = true;
            branch_target = reg[rs]>>2;
            ++instruction_counts[INSTRUCTION_NAME_JR];
            break;
          case FUNCT_ADDU:
            set_reg = rd;
            set_reg_val = reg[rs] + reg[rt];
            ++instruction_counts[INSTRUCTION_NAME_ADDU];
            break;
          case FUNCT_SUBU:
            set_reg = rd;
            set_reg_val = reg[rs] - reg[rt];
            ++instruction_counts[INSTRUCTION_NAME_SUBU];
            break;
          case FUNCT_AND:
            set_reg = rd;
            set_reg_val = reg[rs] & reg[rt];
            ++instruction_counts[INSTRUCTION_NAME_AND];
            break;
          case FUNCT_OR:
            set_reg = rd;
            set_reg_val = reg[rs] | reg[rt];
            ++instruction_counts[INSTRUCTION_NAME_OR];
            break;
          case FUNCT_XOR:
            set_reg = rd;
            set_reg_val = reg[rs] ^ reg[rt];
            ++instruction_counts[INSTRUCTION_NAME_XOR];
            break;
          case FUNCT_NOR:
            set_reg = rd;
            set_reg_val = ~(reg[rs] | reg[rt]);
            ++instruction_counts[INSTRUCTION_NAME_NOR];
            break;
          case FUNCT_SLT:
            set_reg = rd;
            set_reg_val = ((int32_t)reg[rs] < (int32_t)reg[rt]);
            ++instruction_counts[INSTRUCTION_NAME_SLT];
            break;
          case FUNCT_SLTU:
            set_reg = rd;
            set_reg_val = (reg[rs] < reg[rt]);
            ++instruction_counts[INSTRUCTION_NAME_SLTU];
            break;
          default:
            fprintf(stderr, "error: SPECIAL: unknown funct: %d\n", funct);
            fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
                pc*4, pword);
            exit(1);
        }
        break;
      case OPCODE_J:
        is_branch = true;
        branch_success = true;
        branch_target = jt;
        ++instruction_counts[INSTRUCTION_NAME_J];
        break;
      case OPCODE_JAL:
        is_branch = true;
        branch_success = true;
        branch_target = jt;
        set_reg = REG_RA;
        set_reg_val = (uint32_t)(pc + 1) * 4;
        ++instruction_counts[INSTRUCTION_NAME_JAL];
        break;
      case OPCODE_BEQ:
        is_branch = true;
        branch_success = (reg[rs] == reg[rt]);
        branch_target = pc+1+simm16;
        ++instruction_counts[INSTRUCTION_NAME_BEQ];
        break;
      case OPCODE_BNE:
        is_branch = true;
        branch_success = (reg[rs] != reg[rt]);
        branch_target = pc+1+simm16;
        ++instruction_counts[INSTRUCTION_NAME_BNE];
        break;
      case OPCODE_ADDIU:
        set_reg = rt;
        set_reg_val = reg[rs] + simm16;
        ++instruction_counts[INSTRUCTION_NAME_ADDIU];
        break;
      case OPCODE_SLTI:
        set_reg = rt;
        set_reg_val = ((int32_t)reg[rs] < (int32_t)simm16);
        ++instruction_counts[INSTRUCTION_NAME_SLTI];
        break;
      case OPCODE_SLTIU:
        set_reg = rt;
        set_reg_val = (reg[rs] < simm16);
        ++instruction_counts[INSTRUCTION_NAME_SLTIU];
        break;
      case OPCODE_ANDI:
        set_reg = rt;
        set_reg_val = reg[rs] & uimm16;
        ++instruction_counts[INSTRUCTION_NAME_ANDI];
        break;
      case OPCODE_ORI:
        set_reg = rt;
        set_reg_val = reg[rs] | uimm16;
        ++instruction_counts[INSTRUCTION_NAME_ORI];
        break;
      case OPCODE_XORI:
        set_reg = rt;
        set_reg_val = reg[rs] ^ uimm16;
        ++instruction_counts[INSTRUCTION_NAME_XORI];
        break;
      case OPCODE_LUI:
        set_reg = rt;
        set_reg_val = (uint32_t)uimm16 << 16;
        ++instruction_counts[INSTRUCTION_NAME_LUI];
        break;
      case OPCODE_COP1:
        switch(fmt) {
          case COP1_FMT_BRANCH:
            if(ft == 0) {
              is_branch = true;
              branch_success = !cc0;
              branch_target = pc+1+simm16;
              ++instruction_counts[INSTRUCTION_NAME_FP_BC1F];
            } else if(ft == 1) {
              is_branch = true;
              branch_success = cc0;
              branch_target = pc+1+simm16;
              ++instruction_counts[INSTRUCTION_NAME_FP_BC1T];
            } else {
              fprintf(stderr, "error: BC1x: unknown condition: %d\n", ft);
              fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
                  pc*4, pword);
              exit(1);
            }
            break;
          case COP1_FMT_MFC1:
            set_reg = rt;
            set_reg_val = freg[fs];
            ++instruction_counts[INSTRUCTION_NAME_FP_MFC1];
            break;
          case COP1_FMT_MTC1:
            set_freg = fs;
            set_freg_val = reg[rt];
            ++instruction_counts[INSTRUCTION_NAME_FP_MTC1];
            break;
          case COP1_FMT_S:
            switch(funct) {
              case COP1_FUNCT_ADD:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val = native_fadd(freg[fs], freg[ft]);
                } else {
                  set_freg_val = fadd(freg[fs], freg[ft]);
                }
                ++instruction_counts[INSTRUCTION_NAME_FP_ADD_S];
                break;
              case COP1_FUNCT_SUB:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val = native_fsub(freg[fs], freg[ft]);
                } else {
                  set_freg_val = fsub(freg[fs], freg[ft]);
                }
                ++instruction_counts[INSTRUCTION_NAME_FP_SUB_S];
                break;
              case COP1_FUNCT_MUL:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val = native_fmul(freg[fs], freg[ft]);
                } else {
                  set_freg_val = fmul(freg[fs], freg[ft]);
                }
                ++instruction_counts[INSTRUCTION_NAME_FP_MUL_S];
                break;
              case COP1_FUNCT_DIV:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val = native_fdiv(freg[fs], freg[ft]);
                } else {
                  set_freg_val = fdiv(freg[fs], freg[ft]);
                }
                ++instruction_counts[INSTRUCTION_NAME_FP_DIV_S];
                break;
              case COP1_FUNCT_SQRT:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val = native_fsqrt(freg[fs]);
                } else {
                  set_freg_val = fsqrt(freg[fs]);
                }
                ++instruction_counts[INSTRUCTION_NAME_FP_SQRT_S];
                break;
              case COP1_FUNCT_MOV:
                set_freg = fd;
                set_freg_val = freg[fs];
                ++instruction_counts[INSTRUCTION_NAME_FP_MOV_S];
                break;
              case COP1_FUNCT_CVT_W:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val = native_ftoi(freg[fs]);
                } else {
                  set_freg_val = ftoi(freg[fs]);
                }
                ++instruction_counts[INSTRUCTION_NAME_FP_CVT_W_S];
                break;
              case COP1_FUNCT_C_EQ:
                if(use_native_fp) {
                  cc0 = native_feq(freg[fs], freg[ft]);
                } else {
                  cc0 = feq(freg[fs], freg[ft]);
                }
                ++instruction_counts[INSTRUCTION_NAME_FP_C_EQ_S];
                break;
              case COP1_FUNCT_C_OLT:
                if(use_native_fp) {
                  cc0 = native_flt(freg[fs], freg[ft]);
                } else {
                  cc0 = flt(freg[fs], freg[ft]);
                }
                ++instruction_counts[INSTRUCTION_NAME_FP_C_OLT_S];
                break;
              case COP1_FUNCT_C_OLE:
                if(use_native_fp) {
                  cc0 = native_fle(freg[fs], freg[ft]);
                } else {
                  cc0 = fle(freg[fs], freg[ft]);
                }
                ++instruction_counts[INSTRUCTION_NAME_FP_C_OLE_S];
                break;
              default:
                fprintf(stderr, "error: COP1.S: unknown funct: %d\n", funct);
                fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
                    pc*4, pword);
                exit(1);
            }
            break;
          case COP1_FMT_W:
            switch(funct) {
              case COP1_FUNCT_CVT_S:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val = native_itof(freg[fs]);
                } else {
                  set_freg_val = itof(freg[fs]);
                }
                ++instruction_counts[INSTRUCTION_NAME_FP_CVT_S_W];
                break;
              default:
                fprintf(stderr, "error: COP1.W: unknown funct: %d\n", funct);
                fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
                    pc*4, pword);
                exit(1);
            }
            break;
          default:
            fprintf(stderr, "error: COP1: unknown fmt: %d\n", fmt);
            fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
                pc*4, pword);
            exit(1);
        }
        break;
      case OPCODE_LW: {
        set_reg = rt;
        uint32_t addr = reg[rs] + simm16;
        if(addr&3) {
          fprintf(stderr, "error: LW: unaligned access: 0x%08x\n", addr);
          exit(1);
        }
        if(addr <= (1U<<22)) {
          set_reg_val = ram[addr>>2];
        } else if(addr == 0xFFFF0000U) {
          if(rs232c_recv_status > 0) {
            --rs232c_recv_status;
            set_reg_val = 0;
          } else {
            set_reg_val = 1;
          }
        } else if(addr == 0xFFFF0004U) {
          if(rs232c_recv_status < 0) {
            fprintf(stderr, "LW: End of File reached. Halt.\n");
            return 0;
          } else if(rs232c_recv_status > 0) {
            fprintf(stderr, "error: LW: tried to read unready data\n");
            exit(1);
          }
          set_reg_val = rs232c_recv_data;
          rs232c_prereceive();
        } else if(addr == 0xFFFF0008U) {
          if(rs232c_send_status > 0) {
            --rs232c_send_status;
            set_reg_val = 0;
          } else {
            set_reg_val = 1;
          }
        } else {
          fprintf(stderr, "error: LW: out of range: 0x%08x\n", addr);
          exit(1);
        }
        ++instruction_counts[INSTRUCTION_NAME_LW];
        break;
      }
      case OPCODE_SW: {
        uint32_t addr = reg[rs] + simm16;
        if(addr&3) {
          fprintf(stderr, "error: SW: unaligned access: 0x%08x\n", addr);
          exit(1);
        }
        if(show_commit_log) {
          fprintf(stderr, "pc=0x%08x: Memory[0x%08x] <- 0x%08x\n",
              pc*4, addr, reg[rt]);
        }
        if(addr <= (1U<<22)) {
          ram[addr>>2] = reg[rt];
        } else if(addr == 0xFFFF000CU) {
          if(rs232c_send_status > 0) {
            fprintf(stderr, "error: SW: tried to send to unready port\n");
            exit(1);
          }
          unsigned char ch = reg[rt];
          fwrite(&ch,1,1,stdout);
          rs232c_send_status = rs232c_send_count-1;
        } else {
          fprintf(stderr, "error: LW: out of range: 0x%08x\n", addr);
          exit(1);
        }
        ++instruction_counts[INSTRUCTION_NAME_SW];
        break;
      }
      default:
        fprintf(stderr, "error: unknown opcode: %d\n", opcode);
        fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
            pc*4, pword);
        exit(1);
    }
    if(set_reg) {
      reg[set_reg] = set_reg_val;
      if(show_commit_log) {
        fprintf(stderr, "pc=0x%08x: $%s <- 0x%08x\n",
            pc*4, regnames[set_reg], set_reg_val);
      }
    }
    if(set_freg != -1) {
      freg[set_freg] = set_freg_val;
      if(show_commit_log) {
        fprintf(stderr, "pc=0x%08x: $%s <- 0x%08x\n",
            pc*4, fregnames[set_freg], set_freg_val);
      }
    }
    if(is_branch && show_commit_log) {
      if(branch_success) {
        fprintf(stderr, "pc=0x%08x: branch taken, 0x%08x\n",
            pc*4, branch_target*4);
      } else {
        fprintf(stderr, "pc=0x%08x: branch not taken\n", pc*4);
      }
    }
    if(branch_success) {
      pc = branch_target;
      ++branch_counts[pc];
    } else {
      pc = pc + 1;
    }
    ++instruction_count_all;
  }
}

void ils_main() {
  std::fill(ram,ram+(1<<20),0x55555555U);
  int load_pc = 0;
  for(;;) {
    unsigned char chs[4];
    size_t readsize = fread(chs,1,4,stdin);
    if(readsize<4) {
      fprintf(stderr, "input error during loading program\n");
      exit(1);
    }
    uint32_t load_pword = (chs[0]<<24)|(chs[1]<<16)|(chs[2]<<8)|chs[3];
    if(load_pword == (uint32_t)-1) break;
    ram[load_pc++] = load_pword;
  }
  for(int i = 0; i < 32; ++i) ram[load_pc++] = 0U;
  int retval = ils_run();
  if(show_statistics) {
    fprintf(stderr, "\n");
    {
      fprintf(stderr, "instruction count by types:\n");
      vector<pair<int64_t,int>> v;
      for(int i = 0; i < INSTRUCTION_NAME_MAX; ++i) {
        if(instruction_counts[i]) {
          v.emplace_back(instruction_counts[i],i);
        }
      }
      sort(v.begin(), v.end());
      reverse(v.begin(), v.end());
      for(pair<int64_t,int> ci : v) {
        fprintf(stderr, "%10s : %12lld\n", instnames[ci.second],
            (long long int)ci.first);
      }
      fprintf(stderr, "---------------------------\n");
      fprintf(stderr, "%10s : %12lld\n", "SUM",
          (long long int)instruction_count_all);
    }
    fprintf(stderr, "\n\n");
    fprintf(stderr, "successful branch count by targets:\n");
    {
      vector<pair<int64_t,int>> v;
      for(int i = 0; i < (1<<15); ++i) {
        if(branch_counts[i]) {
          v.emplace_back(branch_counts[i], i);
        }
      }
      sort(v.begin(), v.end());
      reverse(v.begin(), v.end());
      for(pair<int64_t,int> ci : v) {
        fprintf(stderr, "0x%08x : %12lld\n", ci.second*4,
            (long long int)ci.first);
      }
    }
    fprintf(stderr, "\n");
  }
  exit(retval);
}
