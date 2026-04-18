module hex_display #(
  parameter CNT_WIDTH = 14
)(
  input  wire         clk,
  input  wire         rst_n,
  input  wire [15:0]  i_data,
  output wire [3:0]   o_anodes,
  output wire [7:0]   o_segments
);

reg [CNT_WIDTH-1:0] cnt;
reg [1:0]           pos;
reg [7:0]           segments_reg_n;

wire [3:0] digit = i_data[{pos, 2'b0} +: 4];

always @(posedge clk, negedge rst_n) begin
  if (!rst_n) begin
    cnt <= {CNT_WIDTH{1'b0}};
    pos <= 2'd0;
  end else begin
    if (~(&cnt)) begin
      cnt <= cnt + 1'b1;
    end else if (&cnt) begin
      cnt <= {CNT_WIDTH{1'b0}};
      pos <= pos + 1'b1;
    end
  end
end

assign o_anodes = ~(4'b1 << pos);
assign o_segments = ~segments_reg_n;

always @(*) begin
  case (digit)
    4'h0: segments_reg_n = 8'b11111100;
    4'h1: segments_reg_n = 8'b01100000;
    4'h2: segments_reg_n = 8'b11011010;
    4'h3: segments_reg_n = 8'b11110010;
    4'h4: segments_reg_n = 8'b01100110;
    4'h5: segments_reg_n = 8'b10110110;
    4'h6: segments_reg_n = 8'b10111110;
    4'h7: segments_reg_n = 8'b11100000;
    4'h8: segments_reg_n = 8'b11111110;
    4'h9: segments_reg_n = 8'b11110110;
    4'hA: segments_reg_n = 8'b11101110;
    4'hB: segments_reg_n = 8'b00111110;
    4'hC: segments_reg_n = 8'b10011100;
    4'hD: segments_reg_n = 8'b01111010;
    4'hE: segments_reg_n = 8'b10011110;
    4'hF: segments_reg_n = 8'b10001110;
  endcase
end

endmodule
