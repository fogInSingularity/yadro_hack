module system_top (
    input wire clk,
    input wire rst_n,

    inout wire sda_io,
    inout wire scl_io
);

wire scl_pad_i;
wire scl_pad_o;
wire scl_padoen;
wire sda_pad_i;
wire sda_pad_o;
wire sda_padoen;

wire [15:0] distance;

// Tri-state buffers (open-drain)
assign scl_io    = scl_padoen ? 1'bz : scl_pad_o;
assign scl_pad_i = scl_io;

assign sda_io    = sda_padoen ? 1'bz : sda_pad_o;
assign sda_pad_i = sda_io;

// Constant low output (open-drain drives 0 when enabled)
assign scl_pad_o = 1'b0;
assign sda_pad_o = 1'b0;

sensor_ctrl sensor_ctrl_inst (
    .clk(clk),
    .rst_n(rst_n),

    .scl_i(scl_pad_i),
    .scl_oen_o(scl_padoen),
    .sda_i(sda_pad_i),
    .sda_oen_o(sda_padoen),

    .o_distance(distance)
);

endmodule
