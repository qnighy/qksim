#include <cstdlib>
#include <cstdint>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <algorithm>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "consts.h"
#include "options.h"
#include "jit.h"
using namespace std;

static string regnames[32] = {
  "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
  "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
  "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
};
static string fregnames[32] = {
  "f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7",
  "f8", "f9", "f10", "f11", "f12", "f13", "f14", "f15",
  "f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
  "f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31"
};

inline string use_regnames(int r) {
  if(r) return regnames[r];
  return "0U";
}

inline string dec_repr(uint32_t i) {
  ostringstream o;
  o << i;
  return o.str();
}

inline string hex_repr(uint32_t i) {
  ostringstream o;
  o << "0x" << hex << setw(8) << setfill('0') << i;
  return o.str();
}

void jit_main() {
  ostringstream prologue;
  ostringstream body;
  ostringstream epilogue;
  prologue << "#include <stdio.h>" << endl;
  prologue << "#include <stdlib.h>" << endl;
  prologue << "#include <stdint.h>" << endl;
  prologue << "#include \"qkfpu.h\"" << endl;
  prologue << "" << endl;
  prologue << "uint32_t ram[1<<20];" << endl;
  prologue << "" << endl;
  prologue << "inline uint32_t load_word(uint32_t addr) {" << endl;
  prologue << "  if(addr & 0x80000000) {" << endl;
  prologue << "    if(addr == 0xFFFF0000U) {" << endl;
  prologue << "      return 1;" << endl;
  prologue << "    }" << endl;
  prologue << "    if(addr == 0xFFFF0004U) {" << endl;
  prologue << "      unsigned char ch;" << endl;
  prologue << "      size_t readsize = fread(&ch,1,1,stdin);" << endl;
  prologue << "      if(readsize<1) exit(0);" << endl;
  prologue << "      return ch;" << endl;
  prologue << "    }" << endl;
  prologue << "    if(addr == 0xFFFF0008U) {" << endl;
  prologue << "      return 1;" << endl;
  prologue << "    }" << endl;
  prologue << "  } else {" << endl;
  prologue << "    return ram[addr>>2];" << endl;
  prologue << "  }" << endl;
  prologue << "}" << endl;
  prologue << "" << endl;
  prologue << "inline void store_word(uint32_t addr, uint32_t val) {" << endl;
  prologue << "  if(addr & 0x80000000) {" << endl;
  prologue << "    if(addr == 0xFFFF000CU) {" << endl;
  prologue << "      unsigned char ch = val;" << endl;
  prologue << "      fwrite(&ch,1,1,stdout);" << endl;
  prologue << "    }" << endl;
  prologue << "  } else {" << endl;
  prologue << "    ram[addr>>2] = val;" << endl;
  prologue << "  }" << endl;
  prologue << "}" << endl;
  prologue << "" << endl;
  prologue << "int main() {" << endl;
  for(int i = 1; i < 32; ++i) {
    prologue << "  uint32_t " << regnames[i] << " = 0U;" << endl;
  }
  for(int i = 0; i < 32; ++i) {
    prologue << "  uint32_t " << fregnames[i] << " = 0U;" << endl;
  }
  prologue << "  int cc0 = 0;" << endl;
  epilogue << "  return 0;" << endl;
  epilogue << "}" << endl;

  vector<uint32_t> ram(1<<20);
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
  {
    ofstream intmp("tmp-qksim-input.dat");
    for(;;) {
      unsigned char ch;
      size_t readsize = fread(&ch,1,1,stdin);
      if(readsize<1) {
        if(feof(stdin)) {
          break;
        } else {
          fprintf(stderr, "input error\n");
          exit(1);
        }
      }
      intmp << ch;
    }
  }

  prologue << "  static const void *labels[" << dec_repr(load_pc) << "] = {"
    << endl;
  for(int pc = 0; pc < load_pc; ++pc) {
    prologue << "    && L" << hex_repr(pc*4);
    if(pc < load_pc-1) prologue << ",";
    prologue << endl;
  }
  prologue << "  };" << endl;

  for(int pc = 0; pc < load_pc; ++pc) {
    body << "L" << hex_repr(pc*4) << ":" << endl;
    // body << "  fprintf(stderr, \"pc = " << hex_repr(pc*4) << "\\n\");" << endl;
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
    string branch_cond;
    int branch_target = -1;
    bool jump_success = false;
    int jump_target = -1;
    int jump_target_reg = -1;
    int set_reg = 0;
    string set_reg_val;
    int set_freg = -1;
    string set_freg_val;
    string set_cc0_val;
    switch(opcode) {
      case OPCODE_SPECIAL:
        switch(funct) {
          case FUNCT_SLL:
            set_reg = rd;
            set_reg_val = use_regnames(rt) + " << " + dec_repr(sa);
            break;
          case FUNCT_SRL:
            set_reg = rd;
            set_reg_val = use_regnames(rt) + " >> " + dec_repr(sa);
            break;
          case FUNCT_SRA:
            set_reg = rd;
            set_reg_val =
              "(int32_t)" + use_regnames(rt) + " >> " + dec_repr(sa);
            break;
          case FUNCT_SLLV:
            set_reg = rd;
            set_reg_val =
              use_regnames(rt) + " << (" + use_regnames(rs) + "&31)";
            break;
          case FUNCT_SRLV:
            set_reg = rd;
            set_reg_val =
              use_regnames(rt) + " >> (" + use_regnames(rs) + "&31)";
            break;
          case FUNCT_SRAV:
            set_reg = rd;
            set_reg_val =
              "(int32_t)" + use_regnames(rt) +
              " >> (" + use_regnames(rs) + "&31)";
            break;
          case FUNCT_JR:
            jump_success = true;
            jump_target_reg = rs;
            break;
          case FUNCT_ADDU:
            set_reg = rd;
            set_reg_val = use_regnames(rs) + " + " + use_regnames(rt);
            break;
          case FUNCT_SUBU:
            set_reg = rd;
            set_reg_val = use_regnames(rs) + " - " + use_regnames(rt);
            break;
          case FUNCT_AND:
            set_reg = rd;
            set_reg_val = use_regnames(rs) + " & " + use_regnames(rt);
            break;
          case FUNCT_OR:
            set_reg = rd;
            set_reg_val = use_regnames(rs) + " | " + use_regnames(rt);
            break;
          case FUNCT_XOR:
            set_reg = rd;
            set_reg_val = use_regnames(rs) + " ^ " + use_regnames(rt);
            break;
          case FUNCT_NOR:
            set_reg = rd;
            set_reg_val =
              "~(" + use_regnames(rs) + " | " + use_regnames(rt) + ")";
            break;
          case FUNCT_SLT:
            set_reg = rd;
            set_reg_val =
              "((int32_t)" + use_regnames(rs) + " < (int32_t)" +
              use_regnames(rt) + ")";
            break;
          case FUNCT_SLTU:
            set_reg = rd;
            set_reg_val =
              "(" + use_regnames(rs) + " < " + use_regnames(rt) + ")";
            break;
          default:
            fprintf(stderr, "error: SPECIAL: unknown funct: %d\n", funct);
            fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
                pc*4, pword);
            exit(1);
        }
        break;
      case OPCODE_J:
        jump_success = true;
        jump_target = jt;
        break;
      case OPCODE_JAL:
        jump_success = true;
        jump_target = jt;
        set_reg = REG_RA;
        set_reg_val = hex_repr((pc+1)*4);
        break;
      case OPCODE_BEQ:
        branch_cond = use_regnames(rs) + " == " + use_regnames(rt);
        branch_target = pc+1+simm16;
        break;
      case OPCODE_BNE:
        branch_cond = use_regnames(rs) + " != " + use_regnames(rt);
        branch_target = pc+1+simm16;
        break;
      case OPCODE_ADDIU:
        set_reg = rt;
        set_reg_val = use_regnames(rs) + " + " + hex_repr(simm16);
        break;
      case OPCODE_SLTI:
        set_reg = rt;
        set_reg_val =
          "((int32_t)" + use_regnames(rs) + " < (int32_t)" +
          hex_repr(simm16) + ")";
        break;
      case OPCODE_SLTIU:
        set_reg = rt;
        set_reg_val =
          "(" + use_regnames(rs) + " < " + hex_repr(simm16) + ")";
        break;
      case OPCODE_ANDI:
        set_reg = rt;
        set_reg_val = use_regnames(rs) + " & " + hex_repr(uimm16);
        break;
      case OPCODE_ORI:
        set_reg = rt;
        set_reg_val = use_regnames(rs) + " | " + hex_repr(uimm16);
        break;
      case OPCODE_XORI:
        set_reg = rt;
        set_reg_val = use_regnames(rs) + " ^ " + hex_repr(uimm16);
        break;
      case OPCODE_LUI:
        set_reg = rt;
        set_reg_val = hex_repr(uimm16<<16);
        break;
      case OPCODE_COP1:
        switch(fmt) {
          case COP1_FMT_BRANCH:
            if(ft == 0) {
              branch_cond = "!cc0";
              branch_target = pc+1+simm16;
            } else if(ft == 1) {
              branch_cond = "cc0";
              branch_target = pc+1+simm16;
            } else {
              fprintf(stderr, "error: BC1x: unknown condition: %d\n", ft);
              fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
                  pc*4, pword);
              exit(1);
            }
            break;
          case COP1_FMT_MFC1:
            set_reg = rt;
            set_reg_val = fregnames[fs];
            break;
          case COP1_FMT_MTC1:
            set_freg = fs;
            set_freg_val = use_regnames(rt);
            break;
          case COP1_FMT_S:
            switch(funct) {
              case COP1_FUNCT_ADD:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val =
                    "native_fadd(" + fregnames[fs] + ", "
                    + fregnames[ft] + ")";
                } else {
                  set_freg_val =
                    "fadd(" + fregnames[fs] + ", " + fregnames[ft] + ")";
                }
                break;
              case COP1_FUNCT_SUB:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val =
                    "native_fsub(" + fregnames[fs] + ", "
                    + fregnames[ft] + ")";
                } else {
                  set_freg_val =
                    "fsub(" + fregnames[fs] + ", " + fregnames[ft] + ")";
                }
                break;
              case COP1_FUNCT_MUL:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val =
                    "native_fmul(" + fregnames[fs] + ", "
                    + fregnames[ft] + ")";
                } else {
                  set_freg_val =
                    "fmul(" + fregnames[fs] + ", " + fregnames[ft] + ")";
                }
                break;
              case COP1_FUNCT_DIV:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val =
                    "native_fdiv(" + fregnames[fs] + ", "
                    + fregnames[ft] + ")";
                } else {
                  set_freg_val =
                    "fdiv(" + fregnames[fs] + ", " + fregnames[ft] + ")";
                }
                break;
              case COP1_FUNCT_SQRT:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val = "native_fsqrt(" + fregnames[fs] + ")";
                } else {
                  set_freg_val = "fsqrt(" + fregnames[fs] + ")";
                }
                break;
              case COP1_FUNCT_MOV:
                set_freg = fd;
                set_freg_val = fregnames[fs];
                break;
              case COP1_FUNCT_CVT_W:
                set_freg = fd;
                if(use_native_fp) {
                  set_freg_val = "native_ftoi(" + fregnames[fs] + ")";
                } else {
                  set_freg_val = "ftoi(" + fregnames[fs] + ")";
                }
                break;
              case COP1_FUNCT_C_EQ:
                if(use_native_fp) {
                  set_cc0_val = "native_feq(" + fregnames[fs] + ", "
                    + fregnames[ft] + ")";
                } else {
                  set_cc0_val = "feq(" + fregnames[fs] + ", "
                    + fregnames[ft] + ")";
                }
                break;
              case COP1_FUNCT_C_OLT:
                if(use_native_fp) {
                  set_cc0_val = "native_flt(" + fregnames[fs] + ", "
                    + fregnames[ft] + ")";
                } else {
                  set_cc0_val = "flt(" + fregnames[fs] + ", "
                    + fregnames[ft] + ")";
                }
                break;
              case COP1_FUNCT_C_OLE:
                if(use_native_fp) {
                  set_cc0_val = "native_fle(" + fregnames[fs] + ", "
                    + fregnames[ft] + ")";
                } else {
                  set_cc0_val = "fle(" + fregnames[fs] + ", "
                    + fregnames[ft] + ")";
                }
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
                  set_freg_val = "native_itof(" + fregnames[fs] + ")";
                } else {
                  set_freg_val = "itof(" + fregnames[fs] + ")";
                }
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
      case OPCODE_LW:
        set_reg = rt;
        set_reg_val =
          "load_word(" + use_regnames(rs) + " + " + hex_repr(simm16) + ")";
        break;
      case OPCODE_SW:
        body <<
          "  store_word(" + use_regnames(rs) + " + " + hex_repr(simm16) +
          ", " + use_regnames(rt) + ");" << endl;
        break;
      default:
        fprintf(stderr, "error: unknown opcode: %d\n", opcode);
        fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
            pc*4, pword);
        exit(1);
    }
    if(set_reg) {
      body << "  " << regnames[set_reg] << " = " << set_reg_val << ";" << endl;
    }
    if(set_freg != -1) {
      body << "  " << fregnames[set_freg] << " = "
        << set_freg_val << ";" << endl;
    }
    if(set_cc0_val != "") {
      body << "  cc0 = "
        << set_cc0_val << ";" << endl;
    }
    if(branch_cond != "") {
      body << "  if(" << branch_cond << ") goto L" <<
        hex_repr(branch_target*4) << ";" << endl;
    }
    if(jump_success) {
      if(jump_target_reg == -1) {
        body << "  goto L" << hex_repr(jump_target*4) << ";" << endl;
      } else {
        body << "  goto *labels[" << use_regnames(jump_target_reg) <<
          ">>2];" << endl;
      }
    }
  }

  {
    ofstream srcfile("tmp-qksim-compiled.c");
    srcfile << prologue.str() << body.str() << epilogue.str();
  }

  ostringstream command;
  command << "gcc -std=c99 -O2 -Wall -Wextra -g ";
  command << "-o " << "tmp-qksim-compiled" << " ";
  command << "tmp-qksim-compiled.c ";
  command << "fpu/C/fadd.o ";
  command << "fpu/C/fcmp.o ";
  command << "fpu/C/fdiv.o ";
  command << "fpu/C/ffloor.o ";
  command << "fpu/C/finv.o ";
  command << "fpu/C/float.o ";
  command << "fpu/C/fmul.o ";
  command << "fpu/C/fsqrt.o ";
  command << "fpu/C/ftoi.o ";
  command << "fpu/C/itof.o ";
  command << "native_fpu.o ";
  command << "-lm ";
  cerr << command.str() << endl;
  int retval = system(command.str().c_str());
  if(retval < 0) {
    fprintf(stderr, "error: compiler invocation failed\n");
    exit(1);
  } else if(retval > 0) {
    fprintf(stderr, "error: compiler failed\n");
    exit(1);
  }
  int inputfd = open("tmp-qksim-input.dat", O_RDONLY);
  dup2(inputfd, 0);
  printf("hoge\n");
  execlp("./tmp-qksim-compiled", "./tmp-qksim-compiled", NULL);
}
