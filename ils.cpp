#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include "consts.h"
#include "ils.h"

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

static void ils_run() {
  int pc = 0;
  uint32_t reg[32];
  std::fill(reg, reg+32, 0);
  rs232c_prereceive();
  for(;;) {
    if(pc < 0 || pc >= (1<<20)) {
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
    uint32_t uimm16 = (uint16_t)pword;
    uint32_t simm16 = (int16_t)pword;
    int jt = (pc>>26<<26)|(pword&((1U<<26)-1));
    bool branch_success = false;
    int branch_target = -1;
    int set_reg = 0;
    uint32_t set_reg_val = 0;
    switch(opcode) {
      case OPCODE_SPECIAL:
        switch(funct) {
          case FUNCT_SLL:
            set_reg = rd;
            set_reg_val = reg[rt] << sa;
            break;
          case FUNCT_SRL:
            set_reg = rd;
            set_reg_val = reg[rt] >> sa;
            break;
          case FUNCT_SRA:
            set_reg = rd;
            set_reg_val = (int32_t)reg[rt] >> sa;
            break;
          case FUNCT_SLLV:
            set_reg = rd;
            set_reg_val = reg[rt] << (reg[rs]&31);
            break;
          case FUNCT_SRLV:
            set_reg = rd;
            set_reg_val = reg[rt] >> (reg[rs]&31);
            break;
          case FUNCT_SRAV:
            set_reg = rd;
            set_reg_val = (int32_t)reg[rt] >> (reg[rs]&31);
            break;
          case FUNCT_JR:
            if(reg[rs]&3) {
              fprintf(stderr, "error: JR: unaligned jump: 0x%08x\n", reg[rs]);
              exit(1);
            }
            branch_success = true;
            branch_target = reg[rs]>>2;
            break;
          case FUNCT_ADDU:
            set_reg = rd;
            set_reg_val = reg[rs] + reg[rt];
            break;
          case FUNCT_SUBU:
            set_reg = rd;
            set_reg_val = reg[rs] - reg[rt];
            break;
          case FUNCT_AND:
            set_reg = rd;
            set_reg_val = reg[rs] & reg[rt];
            break;
          case FUNCT_OR:
            set_reg = rd;
            set_reg_val = reg[rs] | reg[rt];
            break;
          case FUNCT_XOR:
            set_reg = rd;
            set_reg_val = reg[rs] ^ reg[rt];
            break;
          case FUNCT_NOR:
            set_reg = rd;
            set_reg_val = ~(reg[rs] | reg[rt]);
            break;
          case FUNCT_SLT:
            set_reg = rd;
            set_reg_val = ((int32_t)reg[rs] < (int32_t)reg[rt]);
            break;
          case FUNCT_SLTU:
            set_reg = rd;
            set_reg_val = (reg[rs] < reg[rt]);
            break;
          default:
            fprintf(stderr, "error: SPECIAL: unknown funct: %d\n", funct);
            fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
                pc*4, pword);
            exit(1);
        }
        break;
      case OPCODE_J:
        branch_success = true;
        branch_target = jt;
        break;
      case OPCODE_JAL:
        branch_success = true;
        branch_target = jt;
        set_reg = REG_RA;
        set_reg_val = (uint32_t)(pc + 1) * 4;
        break;
      case OPCODE_BEQ:
        branch_success = (reg[rs] == reg[rt]);
        branch_target = pc+1+simm16;
        break;
      case OPCODE_BNE:
        branch_success = (reg[rs] != reg[rt]);
        branch_target = pc+1+simm16;
        break;
      case OPCODE_ADDIU:
        set_reg = rt;
        set_reg_val = reg[rs] + simm16;
        break;
      case OPCODE_SLTI:
        set_reg = rt;
        set_reg_val = ((int32_t)reg[rs] < (int32_t)simm16);
        break;
      case OPCODE_SLTIU:
        set_reg = rt;
        set_reg_val = (reg[rs] < simm16);
        break;
      case OPCODE_ANDI:
        set_reg = rt;
        set_reg_val = reg[rs] & uimm16;
        break;
      case OPCODE_ORI:
        set_reg = rt;
        set_reg_val = reg[rs] | uimm16;
        break;
      case OPCODE_XORI:
        set_reg = rt;
        set_reg_val = reg[rs] ^ uimm16;
        break;
      case OPCODE_LUI:
        set_reg = rt;
        set_reg_val = (uint32_t)uimm16 << 16;
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
            exit(0);
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
        break;
      }
      case OPCODE_SW: {
        uint32_t addr = reg[rs] + simm16;
        if(addr&3) {
          fprintf(stderr, "error: SW: unaligned access: 0x%08x\n", addr);
          exit(1);
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
    }
    if(branch_success) {
      pc = branch_target;
    } else {
      pc = pc + 1;
    }
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
  ils_run();
}
