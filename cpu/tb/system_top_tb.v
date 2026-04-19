`timescale 1ns/1ps

module system_top_tb;

reg clk;
reg rst_n;

wire SDA, SCL;
wire [3:0] DIGS;
wire [7:0] SEGS;
wire BTN1, BTN2;

assign BTN1 = 0;
assign BTN2 = 0;
 
initial begin
    clk = 1'b0;
    forever #1 clk = ~clk;
end

system_top #(
    .DISP_7SEG_CNT_WIDTH (4)
) system_top_inst (
    .clk              (clk),
    .rst_n            (rst_n),
    .sda_io           (SDA),
    .scl_io           (SCL),
    .i_btn1           (BTN1),
    .i_btn2           (BTN2),
    .o_disp_7seg_digs (DIGS),
    .o_disp_7seg_segs (SEGS)
);

initial begin
    if($test$plusargs("RAND_SEED"))
        $display("NOTE: RAND_SEED is not used directly in this tb.");

    $dumpvars;

    @(negedge clk);
    rst_n = 1'b0;

    @(negedge clk);
    rst_n = 1'b1;

    repeat (10000) @(posedge clk);

    $finish;
end

endmodule
