`timescale 1ns/1ps

module system_top_tb;

reg clk = 1'b0;
reg rst_n = 1'b1;

wire STCP;
wire SHCP;
wire DS;
wire OE;

always #1 clk <= ~clk;

system_top #(
    .DISP_7SEG_CNT_WIDTH (4)
) system_top_inst (
    .clk              (clk),
    .rst_n            (rst_n),
    .o_7seg_disp_stcp (STCP),
    .o_7seg_disp_shcp (SHCP),
    .o_7seg_disp_ds   (DS),
    .o_7seg_disp_oe   (OE)
);

initial begin
    if($test$plusargs("RAND_SEED"))
        $display("NOTE: RAND_SEED is not used directly in this tb.");

    $dumpvars;

    @(negedge clk);
    rst_n = 1'b0;

    @(negedge clk);
    rst_n = 1'b1;

    repeat (1000) @(posedge clk);

    $finish;
end

endmodule
