module system_top (
    input wire clk,
    input wire rst_n,

    output wire [3:0] digs,
    output wire [7:0] segs//,

    // inout wire sda_io,
    // inout wire scl_io
);

wire [15:0] distance;
reg  [15:0] distance_r;

// wire scl_pad_i;
// wire scl_pad_o;
// wire scl_padoen;
// wire sda_pad_i;
// wire sda_pad_o;
// wire sda_padoen;

// Tri-state buffers (open-drain)
// assign scl_io    = scl_padoen ? 1'bz : 1'b0;
// assign scl_pad_i = scl_io;

// assign sda_io    = sda_padoen ? 1'bz : 1'b0;
// assign sda_pad_i = sda_io;

// reg [1:0] sda_pad_i_sync;
// always @(posedge clk) sda_pad_i_sync <= {sda_pad_i_sync[0], sda_pad_i};
// wire sda_pad_i_s = sda_pad_i_sync[1];

// reg [1:0] scl_pad_i_sync;
// always @(posedge clk) scl_pad_i_sync <= {scl_pad_i_sync[0], scl_pad_i};
// wire scl_pad_i_s = scl_pad_i_sync[1];

// sensor_ctrl sensor_ctrl_inst (
//     .clk(clk),
//     .rst_n(rst_n),

//     .xshut_o(),

//     .scl_i(scl_pad_i_s),
//     .scl_oen_o(scl_padoen),
//     .sda_i(sda_pad_i_s),
//     .sda_oen_o(sda_padoen),

//     .o_distance(distance)
// );

always @(posedge clk) begin
    distance_r <= 16'h0;
end

hex_display hex_display (
    .clk        (clk),    
    .rst_n      (rst_n),    
    .i_data     (distance_r),    
    .o_anodes   (digs),        
    .o_segments (segs)        
);

endmodule
