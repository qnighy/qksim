#ifndef CONSTS_H_
#define CONSTS_H_

#define REG_ZERO 0
#define REG_RA 31

#define OPCODE_SPECIAL 000
#define OPCODE_J       002
#define OPCODE_JAL     003
#define OPCODE_BEQ     004
#define OPCODE_BNE     005
#define OPCODE_ADDIU   011
#define OPCODE_SLTI    012
#define OPCODE_SLTIU   013
#define OPCODE_ANDI    014
#define OPCODE_ORI     015
#define OPCODE_XORI    016
#define OPCODE_LUI     017
#define OPCODE_COP1    021
#define OPCODE_LW      043
#define OPCODE_SW      053

#define FUNCT_SLL  000
#define FUNCT_SRL  002
#define FUNCT_SRA  003
#define FUNCT_SLLV 004
#define FUNCT_SRLV 006
#define FUNCT_SRAV 007
#define FUNCT_JR   010
#define FUNCT_JALR 011
#define FUNCT_ADDU 041
#define FUNCT_SUBU 043
#define FUNCT_AND  044
#define FUNCT_OR   045
#define FUNCT_XOR  046
#define FUNCT_NOR  047
#define FUNCT_SLT  052
#define FUNCT_SLTU 053

#endif /* CONSTS_H_ */
