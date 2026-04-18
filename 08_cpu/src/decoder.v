module decoder (
    input  wire [31:0] i_inst,

    output wire [6:0]  o_opcode,
    output wire [2:0]  o_funct3,
    output wire [6:0]  o_funct7,
    output wire [4:0]  o_rs1,
    output wire [4:0]  o_rs2,
    output wire [4:0]  o_rd, 
    output wire [31:0] o_imm_i, 
    output wire [31:0] o_imm_s,
    output wire [31:0] o_imm_b,
    output wire [31:0] o_imm_u,
    output wire [31:0] o_imm_j
);

wire [31:0] i;
assign i = i_inst;

assign o_opcode = i[6:0];
assign o_rd     = i[11:7];
assign o_rs1    = i[19:15];
assign o_rs2    = i[24:20];
assign o_funct3 = i[14:12];
assign o_funct7 = i[31:25];

assign o_imm_i = {{21{i[31]}}, i[30:25], i[24:21], i[20]};
assign o_imm_s = {{21{i[31]}}, i[30:25], i[11:8], i[7]};
assign o_imm_b = {{20{i[31]}}, i[7], i[30:25], i[11:8], 1'b0};
assign o_imm_u = {i[31:12], {12{1'b0}}};
assign o_imm_j = {{12{i[31]}}, i[19:12], i[20], i[30:21], 1'b0};

endmodule
