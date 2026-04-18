module top #(
    parameter [15:0] NUM_TO_DISPLAY = 16'h19AB,
    parameter CNT_WIDTH = 16
) (
    input  wire clk,
    input  wire rst_n,

    output wire [3:0] o_digs,
    output wire [7:0] o_segs
);

hex_display #(
    .CNT_WIDTH(CNT_WIDTH)
) hex_display (
    .clk        (clk),
    .rst_n      (rst_n),
    .i_data     (NUM_TO_DISPLAY),
    .o_anodes   (o_digs),
    .o_segments (o_segs)
);


endmodule
