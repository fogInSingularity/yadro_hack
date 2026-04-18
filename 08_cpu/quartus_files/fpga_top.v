module fpga_top(
    input  wire CLK,
    input  wire RSTN,

    output wire STCP,
    output wire SHCP,
    output wire DS,
    output wire OE
);

wire clk_pll;
reg rst_n, RSTN_d;

always @(posedge clk_pll) begin
    rst_n  <= RSTN_d;
    RSTN_d <= RSTN;
end

pll pll(
    .inclk0	 (CLK),
    .c0      (clk_pll)
);

system_top #(
    .DISP_7SEG_CNT_WIDTH(14)
) system_top_inst (
    .clk              (clk_pll),
    .rst_n            (rst_n),
    .o_7seg_disp_stcp (STCP),
    .o_7seg_disp_shcp (SHCP),
    .o_7seg_disp_ds   (DS),
    .o_7seg_disp_oe   (OE)
);

endmodule
