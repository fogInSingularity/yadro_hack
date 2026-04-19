module fpga_top(
    input  wire CLK,
    input  wire RSTN,
    
    output wire [3:0] DIGS,
    output wire [7:0] SEGS,

    input  wire BTN1,
    input  wire BTN2,

    inout wire SDA,
    inout wire SCL
);

reg rst_n, RSTN_d;

always @(posedge CLK) begin
    rst_n <= RSTN_d;
    RSTN_d <= RSTN;
end

reg btn1, BTN1_d;

always @(posedge CLK) begin
    btn1 <= BTN1_d;
    BTN1_d <= BTN1;
end

reg btn2, BTN2_d;

always @(posedge CLK) begin
    btn2 <= BTN2_d;
    BTN2_d <= BTN2;
end


system_top system_top_inst (
    .clk              (CLK),
    .rst_n            (rst_n),
    .sda_io           (SDA),
    .scl_io           (SCL),
    .i_btn1           (btn1),
    .i_btn2           (btn2),
    .o_disp_7seg_digs (DIGS),
    .o_disp_7seg_segs (SEGS)
);

endmodule
