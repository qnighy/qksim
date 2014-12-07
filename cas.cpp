#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include "consts.h"
#include "options.h"
#include "cas.h"
#include "qkfpu.h"
using namespace std;

const double clk = 66.666e6;
const double baudrate = 460800.0;
const int num_databits = 8;
const double stopbit = 1.0;

const int clk_per_byte = clk / (baudrate / (num_databits+stopbit+1));

static int recv_queue[1024];
static int recv_queue_top;
static int recv_queue_bottom;
static int recv_count;
static bool recv_eof;
static int send_queue_top;
static int send_queue_bottom;
static int send_count;

inline uint32_t rs_recv_status() {
  if(recv_queue_top == recv_queue_bottom && recv_eof) {
    fprintf(stderr, "LW: End of File reached. Halt.\n");
    exit(0);
  }
  return recv_queue_top != recv_queue_bottom;
}
inline uint32_t rs_recv_data() {
  uint32_t ret = recv_queue[recv_queue_top];
  recv_queue_top++;
  recv_queue_top &= 1023;
  return ret;
}
inline uint32_t rs_send_status() {
  return send_queue_top != ((send_queue_bottom+1)&1023);
}
inline void rs_send_data(uint32_t dat) {
  unsigned char ch = dat;
  fwrite(&ch,1,1,stdout);
  send_queue_bottom++;
  send_queue_bottom &= 1023;
}

inline void rs_init() {
  recv_queue_top = 0;
  recv_queue_bottom = 0;
  recv_count = clk_per_byte;
  recv_eof = false;
  send_queue_top = 0;
  send_queue_bottom = 0;
  send_count = 0;
}

inline void rs_cycle() {
  if(recv_count > 0) {
    --recv_count;
  } else if(!recv_eof) {
    unsigned char ch;
    size_t readsize = fread(&ch,1,1,stdin);
    if(readsize<1) {
      if(feof(stdin)) {
        recv_eof = true;
        return;
      } else {
        fprintf(stderr, "error: reading from input\n");
        exit(1);
      }
    }
    if(((recv_queue_bottom+1)&1023) == recv_queue_top) {
      fprintf(stderr, "error: RS-232C receive queue overflow\n");
      exit(1);
    }
    recv_queue[recv_queue_bottom] = ch;
    recv_queue_bottom++;
    recv_queue_bottom &= 1023;
    recv_count = clk_per_byte;
  }
  if(send_count > 0) {
    --send_count;
  } else if(send_queue_bottom != send_queue_top) {
    send_queue_top++;
    send_queue_top &= 1023;
    send_count = clk_per_byte;
  }
}

#define ALU_OP_ADD  000
#define ALU_OP_ADDU 001
#define ALU_OP_SUB  002
#define ALU_OP_SUBU 003
#define ALU_OP_AND  004
#define ALU_OP_OR   005
#define ALU_OP_XOR  006
#define ALU_OP_NOR  007
#define ALU_OP_SLT  012
#define ALU_OP_SLTU 013
#define ALU_OP_SLL  014
#define ALU_OP_SRL  016
#define ALU_OP_SRA  017

const int TAG_LENGTH = 5;
const int NUM_TAGS = (1<<TAG_LENGTH);
const int REG_LENGTH = 7;
const int NUM_REGS = (1<<REG_LENGTH);
const int CDB_SIZE = 3;

struct value_tag {
  bool available;
  uint32_t value;
  int tag;
  void set_value(uint32_t value) {
    this->available = true;
    this->value = value;
  }
  void set_tag(int tag) {
    this->available = false;
    this->tag = tag;
  }
  void rollback() {
    this->available = true;
  }
};
inline value_tag from_value(uint32_t value) {
  value_tag ret;
  ret.set_value(value);
  return ret;
}
inline value_tag from_tag(int tag) {
  value_tag ret;
  ret.set_tag(tag);
  return ret;
}
inline value_tag cdb_unavailable_val() {
  value_tag ret;
  ret.available = false;
  return ret;
}
inline value_tag cdb_available_val(uint32_t value, int tag) {
  value_tag ret;
  ret.available = true;
  ret.value = value;
  ret.tag = tag;
  return ret;
}

struct rob_val {
  bool busy;
  bool decode_success;
  bool isstore;
  int set_reg;
  value_tag val;
  value_tag branch_target;
  uint32_t predicted_branch;
};

static value_tag cdb[CDB_SIZE];

inline void reset_cdb() {
  for(int i = 0; i < CDB_SIZE; ++i) {
    cdb[i] = cdb_unavailable_val();
  }
}

inline value_tag snoop(value_tag vt) {
  if(vt.available) return vt;
  for(int i = 0; i < CDB_SIZE; ++i) {
    if(cdb[i].available && vt.tag == cdb[i].tag) {
      return from_value(cdb[i].value);
    }
  }
  return vt;
}

template<int num_operands>
struct rs_entry {
  bool busy;
  int tag;
  int opcode;
  value_tag operands[num_operands];
  bool issuable() {
    bool ret = busy;
    for(int i = 0; i < num_operands; ++i) {
      ret = ret && operands[i].available;
    }
    return ret;
  }
};

template<int latency, int num_entries, int num_operands>
struct reservation_station {
  rs_entry<num_operands> entries[num_entries];
  value_tag calculation_pipeline[latency+1];
  function<uint32_t (const rs_entry<num_operands>&)> fun;
  bool dispatchable() {
    return !entries[num_entries-1].busy;
  }
  void do_issue() {
    for(int i = 0; i < num_entries; ++i) {
      for(int j = 0; j < num_operands; ++j) {
        entries[i].operands[j] = snoop(entries[i].operands[j]);
      }
    }
    for(int i = 0; i < latency; ++i) {
      calculation_pipeline[i] = calculation_pipeline[i+1];
    }
    value_tag issue = cdb_unavailable_val();
    {
      int i;
      for(i = 0; i < num_entries; ++i) {
        if(entries[i].issuable()) {
          issue = cdb_available_val(fun(entries[i]), entries[i].tag);
          break;
        }
      }
      for(; i < num_entries; ++i) {
        if(i+1 < num_entries) {
          entries[i] = entries[i+1];
        } else {
          entries[i].busy = false;
        }
      }
    }
    calculation_pipeline[latency] = issue;
  }
  void dispatch(const rs_entry<num_operands> &d) {
    for(int i = 0; i < num_entries; ++i) {
      if(!entries[i].busy) {
        entries[i] = d;
        return;
      }
    }
  }
  void reset() {
    for(int i = 0; i < num_entries; ++i) {
      entries[i].busy = false;
    }
    for(int i = 0; i <= latency; ++i) {
      calculation_pipeline[i].available = false;
    }
  }
};

static uint32_t ram[1<<20];

inline uint32_t read_ram(uint32_t address) {
  if(address&3) {
    fprintf(stderr, "error: invalid address alignment\n");
    exit(1);
  }
  if((address>>2) < (1U<<20)) {
    return ram[address>>2];
  }
  if(address == 0xFFFF0000U) return rs_recv_status();
  if(address == 0xFFFF0004U) return rs_recv_data();
  if(address == 0xFFFF0008U) return rs_send_status();
  fprintf(stderr, "error: read address out-of-bounds: 0x%08x\n", address);
  exit(1);
}
inline void write_ram(uint32_t address, uint32_t data) {
  if(address&3) {
    fprintf(stderr, "error: invalid address alignment\n");
    exit(1);
  }
  if((address>>2) < (1U<<20)) {
    ram[address>>2] = data; return;
  }
  if(address == 0xFFFF000CU) {
    rs_send_data(data);
    return;
  }
  fprintf(stderr, "error: write address out-of-bounds: 0x%08x\n", address);
  exit(1);
}

struct ls_entry1 {
  bool busy;
  int tag;
  bool isstore;
  value_tag base;
  uint32_t offset;
};
struct ls_entry2 {
  bool busy;
  int tag;
  bool isstore;
  uint32_t address;
};

template<int num_entries1, int num_entries2>
struct load_store_buffer {
  static const int latency = 3;
  ls_entry1 entries1[num_entries1];
  ls_entry2 entries2[num_entries2];
  value_tag calculation_pipeline[latency+1];
  bool store_committable(int rob_top, bool rob_top_committable) {
    for(int i = 0; i < num_entries2; ++i) {
      if(entries2[i].busy && entries2[i].isstore &&
         rob_top_committable && entries2[i].tag == rob_top) {
        return true;
      }
    }
    for(int i = 0; i < num_entries2; ++i) {
      fprintf(stderr, "%d: %d, %d, %d, %d\n", i,
          entries2[i].busy, entries2[i].isstore,
          entries2[i].tag, rob_top);
    }
    return false;
  }
  bool dispatchable() {
    return !entries1[num_entries1-1].busy;
  }
  void do_issue(int rob_top, bool rob_top_committable, uint32_t data) {
    for(int i = 0; i < num_entries1; ++i) {
      entries1[i].base = snoop(entries1[i].base);
    }
    for(int i = 0; i < latency; ++i) {
      calculation_pipeline[i] = calculation_pipeline[i+1];
    }
    ls_entry2 issue1;
    issue1.busy = false;
    if(!entries2[num_entries2-1].busy &&
       entries1[0].busy && entries1[0].base.available) {
      issue1.busy = true;
      issue1.tag = entries1[0].tag;
      issue1.isstore = entries1[0].isstore;
      issue1.address = entries1[0].base.value + entries1[0].offset;
      for(int i = 1; i < num_entries1; ++i) {
        entries1[i-1] = entries1[i];
      }
      entries1[num_entries1-1].busy = false;
    }
    value_tag issue2 = cdb_unavailable_val();
    {
      int i;
      for(i = 0; i < num_entries2; ++i) {
        bool issuable =
          entries2[i].busy &&
          (!(entries2[i].isstore ||
             (entries2[i].address&0xFFFF0000U) == 0xFFFF0000U) ||
           ((!entries2[i].isstore || rob_top_committable) &&
            entries2[i].tag == rob_top));
        for(int j = 0; j < i; ++j) {
          issuable = issuable && entries2[i].address != entries2[j].address;
        }
        if(issuable) {
          if(entries2[i].isstore) {
            write_ram(entries2[i].address, data);
          } else {
            issue2 = cdb_available_val(read_ram(entries2[i].address),
                                       entries2[i].tag);
          }
          break;
        }
      }
      for(; i < num_entries2; ++i) {
        if(i+1 < num_entries2) {
          entries2[i] = entries2[i+1];
        } else {
          entries2[i].busy = false;
        }
      }
    }
    calculation_pipeline[latency] = issue2;
    for(int i = 0; i < num_entries2; ++i) {
      if(!entries2[i].busy) {
        entries2[i] = issue1;
        return;
      }
    }
  }
  void dispatch(const ls_entry1 &d) {
    for(int i = 0; i < num_entries1; ++i) {
      if(!entries1[i].busy) {
        entries1[i] = d;
        return;
      }
    }
  }
  void reset() {
    for(int i = 0; i < num_entries1; ++i) {
      entries1[i].busy = false;
    }
    for(int i = 0; i < num_entries2; ++i) {
      entries2[i].busy = false;
    }
  }
};

static const char regnames[128][7] = {
  "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
  "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
  "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra",
  "f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7",
  "f8", "f9", "f10", "f11", "f12", "f13", "f14", "f15",
  "f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
  "f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31",
  "cc0", "reg65", "reg66", "reg67", "reg68", "reg69", "reg70", "reg71",
  "reg72", "reg73", "reg74", "reg75", "reg76", "reg77", "reg78", "reg79",
  "reg80", "reg81", "reg82", "reg83", "reg84", "reg85", "reg86", "reg87",
  "reg88", "reg89", "reg90", "reg91", "reg92", "reg93", "reg94", "reg95",
  "reg96", "reg97", "reg98", "reg99", "reg100", "reg101", "reg102", "reg103",
  "reg104", "reg105", "reg106", "reg107",
  "reg108", "reg109", "reg110", "reg111",
  "reg112", "reg113", "reg114", "reg115",
  "reg116", "reg117", "reg118", "reg119",
  "reg120", "reg121", "reg122", "reg123",
  "reg124", "reg125", "reg126", "reg127"
};

static rob_val rob[NUM_TAGS];
static int rob_top;
static int rob_bottom;

value_tag reg[NUM_REGS];

inline value_tag get_reg(int r) {
  value_tag rv = reg[r];
  if(rv.available) return rv;
  if(rob[rv.tag].val.available) return rob[rv.tag].val;
  return snoop(rv);
}

static void cas_run() {
  int pc = 0;
  uint32_t fetched_instruction = 0x55555555U;
  uint32_t decoded_instruction = 0x55555555U;
  int fetched_instruction_pc = -1;
  int decoded_instruction_pc = -1;
  uint32_t fetched_instruction_predicted_branch = 0x55555555U;
  uint32_t decoded_instruction_predicted_branch = 0x55555555U;
  bool fetched_instruction_available = false;
  bool decoded_instruction_available = false;

  reset_cdb();
  for(int i = 0; i < NUM_REGS; ++i) {
    reg[i] = from_value(0);
  }

  rs_init();

  load_store_buffer<2, 2> lsbuffer;
  reservation_station<1, 2, 4> brancher;
  reservation_station<1, 2, 2> alu;
  brancher.fun = [](const rs_entry<4> &e) -> uint32_t {
    if((e.opcode&1) ^ (e.operands[0].value == e.operands[1].value)) {
      return e.operands[3].value;
    } else {
      return e.operands[2].value;
    }
  };
  alu.fun = [](const rs_entry<2> &e) -> uint32_t {
    switch(e.opcode) {
      case ALU_OP_ADDU:
        return e.operands[0].value + e.operands[1].value;
      case ALU_OP_SUBU:
        return e.operands[0].value - e.operands[1].value;
      case ALU_OP_AND:
        return e.operands[0].value & e.operands[1].value;
      case ALU_OP_OR:
        return e.operands[0].value | e.operands[1].value;
      case ALU_OP_XOR:
        return e.operands[0].value ^ e.operands[1].value;
      case ALU_OP_NOR:
        return ~(e.operands[0].value | e.operands[1].value);
      case ALU_OP_SLT:
        return (int32_t)e.operands[0].value < (int32_t)e.operands[1].value;
      case ALU_OP_SLTU:
        return e.operands[0].value < e.operands[1].value;
      case ALU_OP_SLL:
        return e.operands[1].value << (e.operands[0].value&31);
      case ALU_OP_SRL:
        return e.operands[1].value >> (e.operands[0].value&31);
      case ALU_OP_SRA:
        return (int32_t)e.operands[1].value >> (e.operands[0].value&31);
      default:
        fprintf(stderr, "error: unknown ALU opcode %d\n", e.opcode);
        exit(1);
        return 0;
    }
  };
  lsbuffer.reset();
  brancher.reset();
  alu.reset();

  rob_top = 0;
  rob_bottom = 0;

  for(;;) {
    rs_cycle();
    int last_rob_top = rob_top;
    uint32_t last_rob_val = rob[rob_top].val.value;
    bool refetch = false;
    int refetch_address = -1;
    if(rob[rob_top].busy && !rob[rob_top].decode_success) {
      fprintf(stderr, "error: tried to commit undecoded instruction\n");
      exit(1);
    }
    bool rob_top_committable =
      rob[rob_top].busy &&
      rob[rob_top].val.available &&
      rob[rob_top].branch_target.available;
    if(rob_top_committable &&
       (!rob[rob_top].isstore ||
        lsbuffer.store_committable(rob_top, rob_top_committable)) ) {
      if(rob[rob_top].set_reg) {
        if(show_commit_log) {
          fprintf(stderr, "$%s <- 0x%08x\n",
              regnames[rob[rob_top].set_reg], rob[rob_top].val.value);
        }
        reg[rob[rob_top].set_reg].value = rob[rob_top].val.value;
        reg[rob[rob_top].set_reg].available =
          reg[rob[rob_top].set_reg].tag == rob_top;
      }
      if(rob[rob_top].branch_target.value != rob[rob_top].predicted_branch) {
        if(show_commit_log) {
          fprintf(stderr, "refetch\n");
        }
        refetch = true;
        if(rob[rob_top].branch_target.value&3) {
          fprintf(stderr, "error: unaligned branch target 0x%08x\n",
              rob[rob_top].branch_target.value);
        }
        refetch_address = rob[rob_top].branch_target.value>>2;
      }
      rob[rob_top].busy = false;
      rob_top++;
      rob_top &= NUM_TAGS-1;
    } else {
      // TODO: commit stall
      // fprintf(stderr, "%d, %d, %d, %d, %d, %d\n",
      //    rob[rob_top].busy,
      //    rob[rob_top].val.available,
      //    rob[rob_top].branch_target.available,
      //    (!rob[rob_top].isstore || false),
      //    rob_top, rob_bottom);
    }
    for(int i = 0; i < NUM_TAGS; ++i) {
      if(rob[i].busy) {
        rob[i].val = snoop(rob[i].val);
        // if(!rob[i].branch_target.available) {
        //   fprintf(stderr, "hoge\n");
        // }
        rob[i].branch_target = snoop(rob[i].branch_target);
      }
    }
    if(refetch) {
      for(int i = 0; i < NUM_TAGS; ++i) {
        rob[i].busy = false;
      }
      rob_top = 0;
      rob_bottom = 0;
      lsbuffer.reset();
      brancher.reset();
      alu.reset();
      for(int i = 0; i < NUM_REGS; ++i) {
        reg[i].rollback();
      }
    }
    bool decode_stall = false;
    if(refetch) {
    } else if(decoded_instruction_available) {
      // dispatch
      uint32_t pword = decoded_instruction;
      int opcode = pword>>26;
      int rs = (pword>>21)&31;
      int rt = (pword>>16)&31;
      int rd = (pword>>11)&31;
      int sa = (pword>> 6)&31;
      int funct = pword&63;
      int fmt = rs;
      int ft = rt|0x20;
      int fs = rd|0x20;
      int fd = sa|0x20;
      uint32_t uimm16 = (uint16_t)pword;
      uint32_t simm16 = (int16_t)pword;
      int jt = (pc>>26<<26)|(pword&((1U<<26)-1));
      rob_val dispatch_rob;
      dispatch_rob.busy = true;
      dispatch_rob.decode_success = true;
      dispatch_rob.isstore = false;
      dispatch_rob.val = from_tag(rob_bottom);
      dispatch_rob.branch_target =
        from_value((uint32_t)(decoded_instruction_pc+1)*4);
      dispatch_rob.predicted_branch = decoded_instruction_predicted_branch;
      dispatch_rob.set_reg = 0;
      bool do_dispatch = !rob[rob_bottom].busy;
      ls_entry1 dispatch_lsbuffer;
      rs_entry<2> dispatch_alu;
      rs_entry<4> dispatch_brancher;
      dispatch_lsbuffer.busy = false;
      dispatch_lsbuffer.tag = rob_bottom;
      dispatch_alu.busy = false;
      dispatch_alu.tag = rob_bottom;
      dispatch_brancher.busy = false;
      dispatch_brancher.tag = rob_bottom;
      switch(opcode) {
        case OPCODE_SPECIAL:
          switch(funct) {
            case FUNCT_SLL:
            case FUNCT_SRL:
            case FUNCT_SRA:
            case FUNCT_SLLV:
            case FUNCT_SRLV:
            case FUNCT_SRAV:
            case FUNCT_ADDU:
            case FUNCT_SUBU:
            case FUNCT_AND:
            case FUNCT_OR:
            case FUNCT_XOR:
            case FUNCT_NOR:
            case FUNCT_SLT:
            case FUNCT_SLTU:
              dispatch_rob.set_reg = rd;
              do_dispatch = do_dispatch && alu.dispatchable();
              if(do_dispatch) {
                dispatch_alu.busy = true;
                if(funct == FUNCT_ADDU) {
                  dispatch_alu.opcode = ALU_OP_ADDU;
                } else if(funct == FUNCT_SUBU) {
                  dispatch_alu.opcode = ALU_OP_SUBU;
                } else if(funct == FUNCT_AND) {
                  dispatch_alu.opcode = ALU_OP_AND;
                } else if(funct == FUNCT_OR) {
                  dispatch_alu.opcode = ALU_OP_OR;
                } else if(funct == FUNCT_XOR) {
                  dispatch_alu.opcode = ALU_OP_XOR;
                } else if(funct == FUNCT_NOR) {
                  dispatch_alu.opcode = ALU_OP_NOR;
                } else if(funct == FUNCT_SLT) {
                  dispatch_alu.opcode = ALU_OP_SLT;
                } else if(funct == FUNCT_SLTU) {
                  dispatch_alu.opcode = ALU_OP_SLTU;
                } else if(funct == FUNCT_SLL || funct == FUNCT_SLLV) {
                  dispatch_alu.opcode = ALU_OP_SLL;
                } else if(funct == FUNCT_SRL || funct == FUNCT_SRLV) {
                  dispatch_alu.opcode = ALU_OP_SRL;
                } else if(funct == FUNCT_SRA || funct == FUNCT_SRAV) {
                  dispatch_alu.opcode = ALU_OP_SRA;
                }
                if(funct == FUNCT_SLL || funct == FUNCT_SRL ||
                   funct == FUNCT_SRA) {
                  dispatch_alu.operands[0] = from_value(sa);
                } else {
                  dispatch_alu.operands[0] = get_reg(rs);
                }
                dispatch_alu.operands[1] = get_reg(rt);
              }
              break;
            case FUNCT_JR:
            case FUNCT_JALR:
              dispatch_rob.val =
                from_value((uint32_t)(decoded_instruction_pc+1)*4);
              dispatch_rob.branch_target = get_reg(rs);
              if(funct == FUNCT_JALR) {
                dispatch_rob.set_reg = rd;
              }
              break;
            default:
              fprintf(stderr,
                  "decode error: unknown SPECIAL funct: %d\n", funct);
              fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
                  decoded_instruction_pc*4, pword);
              dispatch_rob.decode_success = false;
          }
          break;
        case OPCODE_J:
        case OPCODE_JAL:
          dispatch_rob.val =
            from_value((uint32_t)(decoded_instruction_pc+1)*4);
          dispatch_rob.branch_target = from_value(jt*4);
          if(opcode == OPCODE_JAL) {
            dispatch_rob.set_reg = REG_RA;
          }
          break;
        case OPCODE_BEQ:
        case OPCODE_BNE:
          do_dispatch = do_dispatch && brancher.dispatchable();
          dispatch_rob.branch_target = from_tag(rob_bottom);
          if(do_dispatch) {
            dispatch_brancher.busy = true;
            if(opcode == OPCODE_BEQ) {
              dispatch_brancher.opcode = 0;
            } else {
              dispatch_brancher.opcode = 1;
            }
            dispatch_brancher.operands[0] = get_reg(rs);
            dispatch_brancher.operands[1] = get_reg(rt);
            dispatch_brancher.operands[2] =
              from_value((uint32_t)(decoded_instruction_pc+1)*4);
            dispatch_brancher.operands[3] =
              from_value((uint32_t)(decoded_instruction_pc+1+simm16)*4);
          }
          break;
        case OPCODE_ADDIU:
        case OPCODE_SLTI:
        case OPCODE_SLTIU:
        case OPCODE_ANDI:
        case OPCODE_ORI:
        case OPCODE_XORI:
        case OPCODE_LUI:
          dispatch_rob.set_reg = rt;
          do_dispatch = do_dispatch && alu.dispatchable();
          if(do_dispatch) {
            dispatch_alu.busy = true;
            if(opcode == OPCODE_ADDIU) {
              dispatch_alu.opcode = ALU_OP_ADDU;
            } else if(opcode == OPCODE_SLTI) {
              dispatch_alu.opcode = ALU_OP_SLT;
            } else if(opcode == OPCODE_SLTIU) {
              dispatch_alu.opcode = ALU_OP_SLTU;
            } else if(opcode == OPCODE_ANDI) {
              dispatch_alu.opcode = ALU_OP_AND;
            } else if(opcode == OPCODE_ORI) {
              dispatch_alu.opcode = ALU_OP_OR;
            } else if(opcode == OPCODE_XORI) {
              dispatch_alu.opcode = ALU_OP_XOR;
            } else {
              dispatch_alu.opcode = ALU_OP_SLL;
            }
            if(opcode == OPCODE_LUI) {
              dispatch_alu.operands[0] = from_value(16);
            } else {
              dispatch_alu.operands[0] = get_reg(rs);
            }
            if(opcode == OPCODE_ANDI || opcode == OPCODE_ORI ||
               opcode == OPCODE_XORI || opcode == OPCODE_LUI) {
              dispatch_alu.operands[1] = from_value(uimm16);
            } else {
              dispatch_alu.operands[1] = from_value(simm16);
            }
          }
          break;
        case OPCODE_LW:
          dispatch_rob.set_reg = rt;
          do_dispatch = do_dispatch && lsbuffer.dispatchable();
          if(do_dispatch) {
            dispatch_lsbuffer.busy = true;
            dispatch_lsbuffer.isstore = false;
            dispatch_lsbuffer.base = get_reg(rs);
            dispatch_lsbuffer.offset = simm16;
          }
          break;
        case OPCODE_SW:
          dispatch_rob.isstore = true;
          dispatch_rob.val = get_reg(rt);
          do_dispatch = do_dispatch && lsbuffer.dispatchable();
          if(do_dispatch) {
            dispatch_lsbuffer.busy = true;
            dispatch_lsbuffer.isstore = true;
            dispatch_lsbuffer.base = get_reg(rs);
            dispatch_lsbuffer.offset = simm16;
          }
          break;
        default:
          fprintf(stderr,
              "decode error: unknown opcode: %d\n", opcode);
          fprintf(stderr, "pc = 0x%08x, pword = 0x%08x\n",
              decoded_instruction_pc*4, pword);
          dispatch_rob.decode_success = false;
      }
      if(do_dispatch) {
        // fprintf(stderr, "dispatch: pc=0x%08x, opcode=%d\n", decoded_instruction_pc, opcode);
        if(dispatch_rob.set_reg) {
          reg[dispatch_rob.set_reg].available = 0;
          reg[dispatch_rob.set_reg].tag = rob_bottom;
        }
        rob[rob_bottom++] = dispatch_rob;
        rob_bottom &= NUM_TAGS-1;
      } else {
        decode_stall = true;
      }
      if(dispatch_lsbuffer.busy) {
        lsbuffer.dispatch(dispatch_lsbuffer);
      }
      if(dispatch_alu.busy) {
        alu.dispatch(dispatch_alu);
      }
      if(dispatch_brancher.busy) {
        brancher.dispatch(dispatch_brancher);
      }
    } else if(decoded_instruction_available) {
      decode_stall = true;
    }
    bool fetch_stall = false;
    if(refetch) {
      decoded_instruction_available = false;
    } else if(!decode_stall && fetched_instruction_available) {
      // decode
      // do nothing on simulation, decode when dispatching
      decoded_instruction = fetched_instruction;
      decoded_instruction_pc = fetched_instruction_pc;
      decoded_instruction_predicted_branch =
        fetched_instruction_predicted_branch;
      decoded_instruction_available = true;
    } else if(fetched_instruction_available) {
      fetch_stall = true;
    }
    if(refetch) {
      fetched_instruction_available = false;
      pc = refetch_address;
    } else if(!fetch_stall) {
      // fetch
      if(pc < 0 || pc >= (1<<15)) {
        fprintf(stderr, "error: program counter 0x%08x is out of range\n",
            pc*4);
        exit(1);
      }
      fetched_instruction = ram[pc];
      fetched_instruction_pc = pc;
      fetched_instruction_predicted_branch = (pc+1)*4;
      fetched_instruction_available = true;
      ++pc;
    }
    lsbuffer.do_issue(last_rob_top, rob_top_committable,
                      last_rob_val);
    brancher.do_issue();
    alu.do_issue();
    cdb[0] = lsbuffer.calculation_pipeline[0];
    cdb[1] = brancher.calculation_pipeline[0];
    cdb[2] = alu.calculation_pipeline[0];
    // fprintf(stderr, "rob_top=%d, rob_bottom=%d\n", rob_top, rob_bottom);
  }
}

void cas_main() {
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
  cas_run();
}
