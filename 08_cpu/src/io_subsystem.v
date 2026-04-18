module io_subsystem #(
    parameter DISP_7SEG_CNT_WIDTH = 0,
    parameter [29:0] DISP_7SEG_ADDR = 30'h0000 
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [29:0] i_mmio_addr,
    input  wire [31:0] i_mmio_data,
    input  wire        i_mmio_wren,
    input  wire [3:0]  i_mmio_mask,
    output wire [31:0] o_mmio_data,
    output wire        o_7seg_disp_stcp,
    output wire        o_7seg_disp_shcp,
    output wire        o_7seg_disp_ds,
    output wire        o_7seg_disp_oe 
);

// there would be an i/o xbar here if there were more than one i/o device

wire [15:0] disp_7seg_data;
wire        disp_7seg_wren;
wire [1:0]  disp_7seg_mask;

assign o_mmio_data = 32'b0;

assign disp_7seg_data = (i_mmio_addr == DISP_7SEG_ADDR) ? i_mmio_data[15:0] : 16'h0;
assign disp_7seg_wren = (i_mmio_addr == DISP_7SEG_ADDR) ? i_mmio_wren       : 1'b0;
assign disp_7seg_mask = (i_mmio_addr == DISP_7SEG_ADDR) ? i_mmio_mask[1:0]  : 2'b0;

ctrl_7_seg_disp #(
    .CNT_WIDTH (DISP_7SEG_CNT_WIDTH)
) ctrl_7_seg_disp_inst (
    .clk    (clk),
    .rst_n  (rst_n),
    .i_data (disp_7seg_data),
    .i_wren (disp_7seg_wren),
    .i_mask (disp_7seg_mask),
    .o_stcp (o_7seg_disp_stcp),
    .o_shcp (o_7seg_disp_shcp),
    .o_ds   (o_7seg_disp_ds),
    .o_oe   (o_7seg_disp_oe)
);

endmodule
