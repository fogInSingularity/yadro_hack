module fpga_top(
    input  wire CLK,
    input  wire RESET,
    
    output wire [3:0] DIGS,
    output wire [7:0] SEGS//,

    // inout wire SDA,
    // inout wire SCL
);

reg rst, RST_d;
wire rst_n;

always @(posedge CLK) begin
    rst <= RST_d;
    RST_d <= RESET;
end

assign rst_n = rst;

system_top system_top_inst (
    .clk     (CLK),
    .rst_n   (rst_n),
    // .sda_io  (SDA),
    // .scl_io  (SCL),
    .digs    (DIGS),
    .segs    (SEGS)
);

endmodule
