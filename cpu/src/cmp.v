`include "common.vh"

module cmp #(
    parameter DATA_WIDTH = 32
) (
    input  wire [`CMP_OP_WIDTH-1:0] i_op,
    input  wire [DATA_WIDTH-1:0]    i_a,
    input  wire [DATA_WIDTH-1:0]    i_b,

    output reg                      o_taken
);

always @(*) begin
    case (i_op)
        `CMP_OP_BEQ:  o_taken = (i_a == i_b);
        `CMP_OP_BNE:  o_taken = (i_a != i_b);
        `CMP_OP_BLT:  o_taken = ($signed(i_a) <  $signed(i_b));
        `CMP_OP_BGE:  o_taken = ($signed(i_a) >= $signed(i_b));
        `CMP_OP_BLTU: o_taken = ($unsigned(i_a) <  $unsigned(i_b));
        `CMP_OP_BGEU: o_taken = ($unsigned(i_a) >= $unsigned(i_b));
    default:
        o_taken = 1'bx;
    endcase
end

endmodule
