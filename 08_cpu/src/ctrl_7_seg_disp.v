module ctrl_7_seg_disp #(
    parameter CNT_WIDTH = 14
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [15:0] i_data,
    input  wire        i_wren,
    input  wire [1:0]  i_mask,

    output wire        o_stcp,
    output wire        o_shcp,
    output wire        o_ds,
    output wire        o_oe
);

wire  [3:0] anodes;
wire  [7:0] segments;

wire shift_done;

reg [15:0] data;

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        data <= 16'b0;
    end else if (i_wren) begin
        if (i_mask[0]) data[7:0]  <= i_data[7:0];
        if (i_mask[1]) data[15:8] <= i_data[15:8]; 
    end
end

hex_display #(
    .CNT_WIDTH(CNT_WIDTH)
) hex_display (
    .clk        (clk),
    .rst_n      (rst_n),
    .i_data     (data),
    .i_rdy      (shift_done),
    .o_anodes   (anodes),
    .o_segments (segments)
);

ctrl_74hc595 ctrl(
    .clk    (clk                ),
    .rst_n  (rst_n              ),
    .i_data ({segments, anodes} ),
    .o_stcp (o_stcp             ),
    .o_shcp (o_shcp             ),
    .o_ds   (o_ds               ),
    .o_oe   (o_oe               ),
    .o_done (shift_done         )
);

endmodule
