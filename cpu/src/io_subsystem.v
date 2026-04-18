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
    output wire [3:0]  o_disp_7seg_digs,
    output wire [7:0]  o_disp_7seg_segs
);

// ---------------------------------------
// ------------- DISP 7 SEG --------------
// ---------------------------------------


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
    .o_digs (o_disp_7seg_digs),
    .o_segs (o_disp_7seg_segs)
);

// ---------------------------------------
// ------------- I2C MASTER -------------- 
// ---------------------------------------



endmodule
