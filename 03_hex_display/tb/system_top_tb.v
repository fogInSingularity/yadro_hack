`timescale 1ns/1ps

module system_top_tb;

reg clk = 1'b0;
reg rst_n = 1'b1;

wire [3:0] digs;
wire [7:0] segs;

always #1 clk <= ~clk;

top #(
    .NUM_TO_DISPLAY (16'hDEAD),
    .CNT_WIDTH      (10)
) top_inst (
    .clk     (clk),
    .rst_n   (rst_n),
    .o_digs  (digs),
    .o_segs  (segs)
);
initial begin
    if($test$plusargs("RAND_SEED"))
        $display("NOTE: RAND_SEED is not used directly in this tb.");
    
    $dumpvars;

    @(negedge clk);
    rst_n = 1'b0;
    @(negedge clk);
    rst_n = 1'b1;

    repeat(20000) @(posedge clk);

    $finish;
end

endmodule
