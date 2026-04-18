`ifndef COMMON_VH
`define COMMON_VH

// COMMON DEFINES

`define RV_OPCODE_LUI    7'b0110111
`define RV_OPCODE_AUIPC  7'b0010111
`define RV_OPCODE_JAL    7'b1101111
`define RV_OPCODE_JALR   7'b1100111
`define RV_OPCODE_BRANCH 7'b1100011
`define RV_OPCODE_LOAD   7'b0000011
`define RV_OPCODE_STORE  7'b0100011
`define RV_OPCODE_IMM    7'b0010011
`define RV_OPCODE_REG    7'b0110011

`define RV_FUNCT3_SRLI_SRAI 3'b101

// {funct7[5], funct3}
`define ALU_OP_WIDTH 4

`define ALU_OP_ADD  4'b0000
`define ALU_OP_SUB  4'b1000
`define ALU_OP_SLL  4'b0001
`define ALU_OP_SRL  4'b0101
`define ALU_OP_SRA  4'b1101
`define ALU_OP_SLT  4'b0010
`define ALU_OP_SLTU 4'b0011
`define ALU_OP_XOR  4'b0100
`define ALU_OP_OR   4'b0110
`define ALU_OP_AND  4'b0111

// funct3
`define CMP_OP_WIDTH 3

`define CMP_OP_BEQ  3'b000
`define CMP_OP_BNE  3'b001
`define CMP_OP_BLT  3'b100
`define CMP_OP_BGE  3'b101
`define CMP_OP_BLTU 3'b110
`define CMP_OP_BGEU 3'b111


`define ALU_SEL1_WIDTH 1

`define ALU_SEL1_IMMU 1'b0
`define ALU_SEL1_REG1 1'b1


`define ALU_SEL2_WIDTH 2

`define ALU_SEL2_REG2 2'b00
`define ALU_SEL2_IMMI 2'b01
`define ALU_SEL2_IMMS 2'b10
`define ALU_SEL2_PC   2'b11


`define WRB_SEL_WIDTH 2

`define WRB_SEL_IMMU   2'b00
`define WRB_SEL_ALURES 2'b01
`define WRB_SEL_LSUDAT 2'b10
`define WRB_SEL_PCINC  2'b11


`define LSU_SIZE_WIDTH 3

`define LSU_SIZE_BYTE  3'b000
`define LSU_SIZE_HALF  3'b001
`define LSU_SIZE_WORD  3'b010
`define LSU_SIZE_BYTEU 3'b100
`define LSU_SIZE_HALFU 3'b101


`define LSU_ADDR_IMM_WIDTH 1

`define LSU_ADDR_IMM_I 1'b0
`define LSU_ADDR_IMM_S 1'b1


`define PC_NEXT_WIDTH 2

`define PC_NEXT_JAL  2'b00
`define PC_NEXT_BRAN 2'b01
`define PC_NEXT_JALR 2'b11

`endif
