`include "config.vh"

module system_top #(
    parameter DISP_7SEG_CNT_WIDTH = 14
) (
    input  wire clk,
    input  wire rst_n,

    output wire o_7seg_disp_stcp,
    output wire o_7seg_disp_shcp,
    output wire o_7seg_disp_ds,
    output wire o_7seg_disp_oe
);

wire [29:0] core2imem_addr;
wire [31:0] imem2core_data;

wire [29:0] core2xbar_addr;
wire [31:0] core2xbar_data;
wire        core2xbar_wren;
wire  [3:0] core2xbar_mask;
wire [31:0] xbar2core_data;

wire [29:0] xbar2dmem_addr;
wire [31:0] xbar2dmem_data;
wire        xbar2dmem_wren;
wire  [3:0] xbar2dmem_mask;
wire [31:0] dmem2xbar_data;

wire [29:0] xbar2mmio_addr;
wire [31:0] xbar2mmio_data;
wire  [3:0] xbar2mmio_mask;
wire        xbar2mmio_wren;
wire [31:0] mmio2xbar_data;


imem imem (
    .clk            (clk),
    .rst_n          (rst_n),
    .i_addr         (core2imem_addr[`IMEM_ADDR_WIDTH-1:0]),
    .o_data         (imem2core_data)
);

dmem dmem (
     .clk           (clk),
     .i_addr        (xbar2dmem_addr[`DMEM_ADDR_WIDTH-1:0]),
     .i_data        (xbar2dmem_data),
     .i_we          (xbar2dmem_wren),
     .i_mask        (xbar2dmem_mask),
     .o_data        (dmem2xbar_data)
);

io_subsystem #(
    .DISP_7SEG_CNT_WIDTH (DISP_7SEG_CNT_WIDTH),
    .DISP_7SEG_ADDR      (`DISP_7SEG_ADDR)
) io_subsystem (
    .clk                 (clk),
    .rst_n               (rst_n),
    .i_mmio_addr         (xbar2mmio_addr),
    .i_mmio_data         (xbar2mmio_data),
    .i_mmio_wren         (xbar2mmio_wren),
    .i_mmio_mask         (xbar2mmio_mask),
    .o_mmio_data         (mmio2xbar_data),
    .o_7seg_disp_stcp    (o_7seg_disp_stcp),
    .o_7seg_disp_shcp    (o_7seg_disp_shcp),
    .o_7seg_disp_ds      (o_7seg_disp_ds),
    .o_7seg_disp_oe      (o_7seg_disp_oe)
);

mem_xbar #(
    .DMEM_START    (`XBAR_DMEM_START),
    .DMEM_LIMIT    (`XBAR_DMEM_LIMIT),
    .MMIO_START    (`XBAR_MMIO_START),
    .MMIO_LIMIT    (`XBAR_MMIO_LIMIT)
) mem_xbar (
    .clk           (clk),

    .i_addr        (core2xbar_addr),
    .i_data        (core2xbar_data),
    .i_wren        (core2xbar_wren),
    .i_mask        (core2xbar_mask),
    .o_data        (xbar2core_data),

    .o_dmem_addr   (xbar2dmem_addr),
    .o_dmem_data   (xbar2dmem_data),
    .o_dmem_wren   (xbar2dmem_wren),
    .o_dmem_mask   (xbar2dmem_mask),
    .i_dmem_data   (dmem2xbar_data),

    .o_mmio_addr   (xbar2mmio_addr),
    .o_mmio_data   (xbar2mmio_data),
    .o_mmio_wren   (xbar2mmio_wren),
    .o_mmio_mask   (xbar2mmio_mask),
    .i_mmio_data   (mmio2xbar_data)
);

core #(
    .PC_INIT_VAL   (`PC_INIT_VAL)
) core (
    .clk           (clk),
    .rst_n         (rst_n),
    .i_imem_data   (imem2core_data),
    .o_imem_addr   (core2imem_addr),
    .o_dmem_addr   (core2xbar_addr),
    .o_dmem_data   (core2xbar_data),
    .o_dmem_wren   (core2xbar_wren),
    .o_dmem_mask   (core2xbar_mask),
    .i_dmem_data   (xbar2core_data)
);

endmodule
