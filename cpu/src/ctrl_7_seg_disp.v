module ctrl_7_seg_disp #(
    parameter CNT_WIDTH = 14
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [15:0] i_data,
    input  wire        i_wren,
    input  wire [1:0]  i_mask,

    output wire [3:0]  o_digs,
    output wire [7:0]  o_segs
);

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
    .o_anodes   (o_digs),
    .o_segments (o_segs)
);

endmodule
