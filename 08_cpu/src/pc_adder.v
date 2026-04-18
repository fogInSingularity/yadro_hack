`include "common.vh" 

module pc_adder (
    input  wire [29:0]               i_pc,
    input  wire [31:0]               i_imm_i,
    input  wire [31:0]               i_imm_j,
    input  wire [31:0]               i_imm_b,
    input  wire [31:0]               i_rs1,

    input  wire [`PC_NEXT_WIDTH-1:0] i_pc_next_sel,
    input  wire                      i_branch_taken,

    output wire [29:0]               o_pc_inc,
    output wire [29:0]               o_pc_next
);

reg [31:0] add_a, add_b, add_res;
wire [31:0] pc_ext;

assign o_pc_inc = i_pc + 1'b1;
assign pc_ext = {i_pc, 2'b0};

always @(*) begin
    case (i_pc_next_sel)
        `PC_NEXT_JAL:  begin add_a = pc_ext; add_b = i_imm_j; end
        `PC_NEXT_BRAN: begin add_a = pc_ext; add_b = i_imm_b; end
        `PC_NEXT_JALR: begin add_a = i_rs1;  add_b = i_imm_i; end
        default:       begin add_a = 32'bx;  add_b = 32'bx;   end
    endcase

    add_res = add_a + add_b;
end

assign o_pc_next = i_branch_taken ? add_res[31:2] : o_pc_inc;

endmodule
