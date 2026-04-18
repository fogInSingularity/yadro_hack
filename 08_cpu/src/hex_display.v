module hex_display #(
  parameter CNT_WIDTH = 14
)(
  input  wire         clk,
  input  wire         rst_n,
  input  wire [15:0]  i_data,
  input  wire         i_rdy,
  output wire [3:0]   o_anodes,
  output reg  [7:0]   o_segments
);

reg [CNT_WIDTH-1:0] cnt;
reg [1:0]           pos;

reg got_rdy;

wire [3:0] digit = i_data[{pos, 2'b0} +: 4];

always @(posedge clk, negedge rst_n) begin
  if (!rst_n) begin
    cnt <= {CNT_WIDTH{1'b0}};
    pos <= 2'd0;
  end else begin
    if (~(&cnt)) begin
      cnt <= cnt + 1'b1;
    end else if (&cnt & got_rdy) begin
      cnt <= {CNT_WIDTH{1'b0}};
      pos <= pos + 1'b1;
    end
  end
end

always @(posedge clk, negedge rst_n) begin
  if (!rst_n) begin
    got_rdy <= 1'b0;
  end else begin
    if (i_rdy) begin
      got_rdy <= 1'b1;
    end else if (&cnt & got_rdy) begin
      got_rdy <= 1'b0;
    end
  end
end

assign o_anodes = ~(4'b1 << pos);

always @(*) begin
  case (digit)
    4'h0: o_segments = 8'b11111100;
    4'h1: o_segments = 8'b01100000;
    4'h2: o_segments = 8'b11011010;
    4'h3: o_segments = 8'b11110010;
    4'h4: o_segments = 8'b01100110;
    4'h5: o_segments = 8'b10110110;
    4'h6: o_segments = 8'b10111110;
    4'h7: o_segments = 8'b11100000;
    4'h8: o_segments = 8'b11111110;
    4'h9: o_segments = 8'b11110110;
    4'hA: o_segments = 8'b11101110;
    4'hB: o_segments = 8'b00111110;
    4'hC: o_segments = 8'b10011100;
    4'hD: o_segments = 8'b01111010;
    4'hE: o_segments = 8'b10011110;
    4'hF: o_segments = 8'b10001110;
  endcase
end

endmodule
