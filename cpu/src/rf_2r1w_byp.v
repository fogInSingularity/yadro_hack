module rf_2r1w_byp #(
    parameter DATA_WIDTH  = 32,
    parameter REG_NUM     = 32,
    
    parameter ADDR_WIDTH = $clog2(REG_NUM)
) (
    input  wire                   clk,

    input  wire  [ADDR_WIDTH-1:0] i_rd1_addr,
    output reg   [DATA_WIDTH-1:0] o_rd1_data,

    input  wire  [ADDR_WIDTH-1:0] i_rd2_addr,
    output reg   [DATA_WIDTH-1:0] o_rd2_data,

    input  wire  [ADDR_WIDTH-1:0] i_wr_addr,
    input  wire  [DATA_WIDTH-1:0] i_wr_data,
    input  wire                   i_wr_en
);

reg [DATA_WIDTH-1:0] r[REG_NUM];
wire bypass1, bypass2;

assign bypass1 = i_wr_en & (i_rd1_addr == i_wr_addr);
assign bypass2 = i_wr_en & (i_rd2_addr == i_wr_addr);

assign o_rd1_data = (i_rd1_addr != 0) ? (bypass1 ? i_wr_data : r[i_rd1_addr]) : 32'h0;
assign o_rd2_data = (i_rd2_addr != 0) ? (bypass2 ? i_wr_data : r[i_rd2_addr]) : 32'h0;

always @(posedge clk) begin
    if (i_wr_en) begin
        r[i_wr_addr] <= i_wr_data;
    end
end

endmodule
