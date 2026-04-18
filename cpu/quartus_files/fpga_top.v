module fpga_top(
    input  wire CLK,
    input  wire RSTN,
    
    output wire [3:0] DIGS,
    output wire [7:0] SEGS//,

    // inout wire SDA,
    // inout wire SCL
);

reg rst_n, RSTN_d;

always @(posedge CLK) begin
    rst_n <= RSTN_d;
    RSTN_d <= RSTN;
end

system_top system_top_inst (
    .clk              (CLK),
    .rst_n            (rst_n),
    // .sda_io           (SDA),
    // .scl_io           (SCL),
    .o_disp_7seg_digs (DIGS),
    .o_disp_7seg_segs (SEGS)
);

endmodule
