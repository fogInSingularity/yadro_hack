module fpga_top(
    input  wire CLK,
    input  wire RESET,
    
    output wire [3:0] DIGS,
    output wire [7:0] SEGS
);

reg rst, RST_d;
wire rst_n;

always @(posedge CLK) begin
    rst <= RST_d;
    RST_d <= RESET;
end

assign rst_n = rst;

top top_inst (
    .clk     (CLK),
    .rst_n   (rst_n),
    .o_digs  (DIGS),
    .o_segs  (SEGS)
);

endmodule
