
`timescale 1ns/1ps

module system_top_tb;

reg clk = 1'b0;
reg rst_n = 1'b1;

always #1 clk <= ~clk;

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
