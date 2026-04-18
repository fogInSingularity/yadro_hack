`include "common.vh"

module core #(
    parameter [31:0] PC_INIT_VAL = 32'h0
) (
    input  wire        clk,
    input  wire        rst_n,

    output wire [29:0] o_imem_addr,
    input  wire [31:0] i_imem_data,

    output wire [29:0] o_dmem_addr,
    output wire [31:0] o_dmem_data,
    input  wire [31:0] i_dmem_data,
    output wire        o_dmem_wren,
    output wire [3:0]  o_dmem_mask
);

wire [6:0]  opcode_s0;
wire [2:0]  funct3_s0;
wire [6:0]  funct7_s0;
wire [4:0]  rs1_s0;
wire [4:0]  rs2_s0;
wire [4:0]  rd_s0;
reg  [4:0]  rd_s1;
wire [31:0] imm_i_s0;
wire [31:0] imm_s_s0;
wire [31:0] imm_b_s0;
wire [31:0] imm_u_s0;
reg  [31:0] imm_u_s1;
wire [31:0] imm_j_s0;

wire [`ALU_SEL1_WIDTH-1:0]     alu_sel1_s0;
wire [`ALU_SEL2_WIDTH-1:0]     alu_sel2_s0;
wire [`WRB_SEL_WIDTH-1:0]      wrb_sel_s0;
reg  [`WRB_SEL_WIDTH-1:0]      wrb_sel_s1;
wire [`ALU_OP_WIDTH-1:0]       alu_op_s0;
wire [`CMP_OP_WIDTH-1:0]       cmp_op_s0;
wire [`PC_NEXT_WIDTH-1:0]      pc_next_sel_s0;
wire                           instr_branch_s0;
wire                           instr_jump_s0;
wire                           rf_wren_s0;
reg                            rf_wren_s1;
wire                           lsu_wren_s0;
wire [`LSU_SIZE_WIDTH-1:0]     lsu_size_s0;
reg  [31:0]                    lsu_addr_s0;
wire [`LSU_ADDR_IMM_WIDTH-1:0] lsu_addr_imm_s0;
reg  [31:0]                    lsu_addr_imm_dat_s0;


wire [31:0] src_reg1_s0, src_reg2_s0;
reg [31:0] alu_a_s0, alu_b_s0, alu_res_s0, alu_res_s1;
reg [31:0] wrb_data_s1, lsu_data_out_s1;

wire cmp_res_s0;
wire branch_taken_s0;

reg [29:0]  pc;
reg [29:0]  pc_next_s0;
wire [29:0] pc_inc_s0;
reg [29:0]  pc_inc_s1;

reg reset_done;

// -----

decoder decoder_inst (
    .i_inst   (i_imem_data),

    .o_opcode (opcode_s0),
    .o_funct3 (funct3_s0),
    .o_funct7 (funct7_s0),
    .o_rs1    (rs1_s0),
    .o_rs2    (rs2_s0),
    .o_rd     (rd_s0),
    .o_imm_i  (imm_i_s0),
    .o_imm_s  (imm_s_s0),
    .o_imm_b  (imm_b_s0),
    .o_imm_u  (imm_u_s0),
    .o_imm_j  (imm_j_s0)
);

ctrl_unit ctrl_unit_inst (
    .i_opcode       (opcode_s0),
    .i_funct3       (funct3_s0),
    .i_funct7       (funct7_s0),

    .o_alu_sel1     (alu_sel1_s0),
    .o_alu_sel2     (alu_sel2_s0),
    .o_wrb_sel      (wrb_sel_s0),
    .o_alu_op       (alu_op_s0),
    .o_cmp_op       (cmp_op_s0),
    .o_pc_next_sel  (pc_next_sel_s0),
    .o_branch       (instr_branch_s0),
    .o_jump         (instr_jump_s0),
    .o_rf_wren      (rf_wren_s0),
    .o_lsu_wren     (lsu_wren_s0),
    .o_lsu_size     (lsu_size_s0),
    .o_lsu_addr_imm (lsu_addr_imm_s0)
);

// -----

always @(*) begin
    case (alu_sel1_s0)
        `ALU_SEL1_IMMU: alu_a_s0 = imm_u_s0;
        `ALU_SEL1_REG1: alu_a_s0 = src_reg1_s0;
        default:        alu_a_s0 = 32'bx;
    endcase

    case (alu_sel2_s0)
        `ALU_SEL2_REG2: alu_b_s0 = src_reg2_s0;
        `ALU_SEL2_IMMI: alu_b_s0 = imm_i_s0;
        `ALU_SEL2_IMMS: alu_b_s0 = imm_s_s0;
        `ALU_SEL2_PC:   alu_b_s0 = {pc, 2'b0};
        default:        alu_b_s0 = 32'bx;
    endcase
end

alu #(.DATA_WIDTH(32)) alu_inst (
    .i_op   (alu_op_s0),
    .i_a    (alu_a_s0),
    .i_b    (alu_b_s0),
    .o_res  (alu_res_s0)
);

// -----

cmp #(.DATA_WIDTH(32)) cmp_inst (
    .i_op    (cmp_op_s0),
    .i_a     (src_reg1_s0),
    .i_b     (src_reg2_s0),
    .o_taken (cmp_res_s0)
);

assign branch_taken_s0 = instr_jump_s0 | (instr_branch_s0 & cmp_res_s0);

// -----

rf_2r1w_byp #(.DATA_WIDTH(32), .REG_NUM(32)) rf_2r1w_inst (
    .clk        (clk),

    .i_rd1_addr (rs1_s0),
    .o_rd1_data (src_reg1_s0),

    .i_rd2_addr (rs2_s0),
    .o_rd2_data (src_reg2_s0),

    .i_wr_addr  (rd_s1),
    .i_wr_data  (wrb_data_s1),
    .i_wr_en    (rf_wren_s1)
);

// -----

wire lsu_wren_clean;
assign lsu_wren_clean = lsu_wren_s0 & reset_done;

always @(*) begin
    case (lsu_addr_imm_s0)
        `LSU_ADDR_IMM_I: lsu_addr_imm_dat_s0 = imm_i_s0;
        `LSU_ADDR_IMM_S: lsu_addr_imm_dat_s0 = imm_s_s0;
        default:         lsu_addr_imm_dat_s0 = 32'bx;
    endcase
    lsu_addr_s0 = lsu_addr_imm_dat_s0 + src_reg1_s0;
end

lsu lsu_inst(
    .clk         (clk),

    .i_core_addr (lsu_addr_s0),
    .i_core_data (src_reg2_s0),
    .o_core_data (lsu_data_out_s1),
    .i_wren      (lsu_wren_clean),
    .i_size      (lsu_size_s0),

    .o_mem_addr  (o_dmem_addr),
    .o_mem_data  (o_dmem_data),
    .i_mem_data  (i_dmem_data),
    .o_wren      (o_dmem_wren),
    .o_mask      (o_dmem_mask)
);

// -----

always @(posedge clk) begin
    rd_s1      <= rd_s0;
    rf_wren_s1 <= rf_wren_s0;

    imm_u_s1   <= imm_u_s0;
    alu_res_s1 <= alu_res_s0;
    pc_inc_s1  <= pc_inc_s0;

    wrb_sel_s1 <= wrb_sel_s0;
end

// -----

always @(*) begin
    case (wrb_sel_s1)
        `WRB_SEL_IMMU:   wrb_data_s1 = imm_u_s1;
        `WRB_SEL_ALURES: wrb_data_s1 = alu_res_s1;
        `WRB_SEL_LSUDAT: wrb_data_s1 = lsu_data_out_s1;
        `WRB_SEL_PCINC:  wrb_data_s1 = {pc_inc_s1, 2'b0};
        default:         wrb_data_s1 = 32'bx;
    endcase
end

// -----

pc_adder pc_adder_inst (
    .i_pc           (pc),
    .i_imm_i        (imm_i_s0),
    .i_imm_j        (imm_j_s0),
    .i_imm_b        (imm_b_s0),
    .i_rs1          (src_reg1_s0),
    .i_pc_next_sel  (pc_next_sel_s0),
    .i_branch_taken (branch_taken_s0),
    .o_pc_inc       (pc_inc_s0),
    .o_pc_next      (pc_next_s0)
);

assign o_imem_addr = (reset_done) ? pc_next_s0 : pc;

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        pc <= PC_INIT_VAL[31:2];
    end else if (reset_done) begin
        pc <= pc_next_s0;
    end
end

// -----

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        reset_done <= 1'b0;
    end else begin
        reset_done <= 1'b1;
    end
end

endmodule
