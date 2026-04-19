module io_subsystem #(
    parameter DISP_7SEG_CNT_WIDTH   = 0,
    parameter [29:0] DISP_7SEG_ADDR = 30'h0000,
    parameter I2C_PRESCALE          = 16'd124 // 50 MHz → 100 kHz
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [29:0] i_mmio_addr,
    input  wire [31:0] i_mmio_data,
    input  wire        i_mmio_wren,
    input  wire [3:0]  i_mmio_mask,
    output reg  [31:0] o_mmio_data,

    output wire [3:0]  o_disp_7seg_digs,
    output wire [7:0]  o_disp_7seg_segs,

    input  wire        i_btn1,
    input  wire        i_btn2,

    input  wire        i_scl,
    output wire        o_scl_oen,
    input  wire        i_sda,
    output wire        o_sda_oen
);

reg [31:0] mmio_data_q;
reg [31:0] mmio_data; // not a reg actually

assign o_mmio_data = mmio_data_q;

always @(*) begin
    case ({i_mmio_addr, 2'b0, i_mmio_mask})
        {32'h30, 4'b0001}: mmio_data = {24'b0, i2c_o_dout};
        {32'h34, 4'b0001}: mmio_data = {31'b0, i2c_o_rx_ack};
        {32'h38, 4'b0001}: mmio_data = {31'b0, i2c_o_ready};
        {32'h3C, 4'b0001}: mmio_data = {31'b0, i2c_o_arb_lost};
        {32'h44, 4'b0001}: mmio_data = {31'b0, i2c_o_busy};
        {32'h00, 4'b0001}: mmio_data = {31'b0, btn1_r};
        {32'h04, 4'b0001}: mmio_data = {31'b0, btn2_r};
        default:           mmio_data = 32'bx;
    endcase
end

// ---------------------------------------
// ------------- DISP 7 SEG --------------
// ---------------------------------------

wire [15:0] disp_7seg_data;
wire        disp_7seg_wren;
wire [1:0]  disp_7seg_mask;


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
// ---------------- BTNS -----------------
// ---------------------------------------

reg btn1_r, btn2_r;

always @(posedge clk) begin
    btn1_r <= i_btn1;
    btn2_r <= i_btn2;
end

// ---------------------------------------
// ------------- I2C MASTER --------------
// ---------------------------------------

reg        i2c_i_cmd_valid, i2c_i_cmd_valid_r;
reg  [2:0] i2c_i_cmd, i2c_i_cmd_r;
reg  [7:0] i2c_i_din, i2c_i_din_r;
wire [7:0] i2c_o_dout;
wire       i2c_o_rx_ack;
wire       i2c_o_ready;
wire       i2c_o_arb_lost;
reg        i2c_i_arb_lost_clear, i2c_i_arb_lost_clear_r;
wire       i2c_o_busy;

reg i2c_cmd_valid_wren;
reg i2c_cmd_wren;
reg i2c_din_wren;
reg i2c_arb_lost_clear_wren;


always @(posedge clk) begin
    mmio_data_q <= mmio_data;
end

always @(*) begin
    i2c_i_cmd_valid         = '0;
    i2c_i_cmd               = '0;
    i2c_i_din               = '0;
    i2c_i_arb_lost_clear    = '0;
    i2c_cmd_valid_wren      = 1'b0;    
    i2c_cmd_wren            = 1'b0;
    i2c_din_wren            = 1'b0;
    i2c_arb_lost_clear_wren = 1'b0;        

    casez ({i_mmio_addr, 2'b0, i_mmio_mask, i_mmio_wren})
        {32'h24, 4'b???1, 1'b1}: begin 
            i2c_i_cmd_valid    = i_mmio_data[0];
            i2c_cmd_valid_wren = 1'b1; 
        end
        {32'h28, 4'b???1, 1'b1}: begin 
            i2c_i_cmd    = i_mmio_data[2:0];
            i2c_cmd_wren = 1'b1; 
        end
        {32'h2C, 4'b???1, 1'b1}: begin 
            i2c_i_din    = i_mmio_data[7:0];
            i2c_din_wren = 1'b1; 
        end
        {32'h40, 4'b???1, 1'b1}: begin 
            i2c_i_arb_lost_clear    = i_mmio_data[0];
            i2c_arb_lost_clear_wren = 1'b1; 
        end
        default: begin
            i2c_i_cmd_valid         = '0;
            i2c_i_cmd               = '0;
            i2c_i_din               = '0;
            i2c_i_arb_lost_clear    = '0;
            i2c_cmd_valid_wren      = 1'b0;
            i2c_cmd_wren            = 1'b0;
            i2c_din_wren            = 1'b0;
            i2c_arb_lost_clear_wren = 1'b0;
        end
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        i2c_i_cmd_valid_r      <= '0;
        i2c_i_cmd_r            <= '0;
        i2c_i_din_r            <= '0;
        i2c_i_arb_lost_clear_r <= '0;
    end else begin
        if (i2c_cmd_valid_wren)      i2c_i_cmd_valid_r      <= i2c_i_cmd_valid;
        if (i2c_cmd_wren)            i2c_i_cmd_r            <= i2c_i_cmd;
        if (i2c_din_wren)            i2c_i_din_r            <= i2c_i_din;
        if (i2c_arb_lost_clear_wren) i2c_i_arb_lost_clear_r <= i2c_i_arb_lost_clear;
    end
end

i2c_master_core i2c_master_core_inst (
    .clk_i              (clk),
    .rstn_i             (rst_n),
    .ena_i              (i2c_core_ena_r),

    // Command interface
    .cmd_valid_i        (i2c_i_cmd_valid_r),
    .cmd_i              (i2c_i_cmd_r),
    .din_i              (i2c_i_din_r),
    .dout_o             (i2c_o_dout),
    .rx_ack_o           (i2c_o_rx_ack),
    .ready_o            (i2c_o_ready),
    .arb_lost_o         (i2c_o_arb_lost),
    .arb_lost_clear_i   (i2c_i_arb_lost_clear_r),
    .busy_o             (i2c_o_busy),

    // I2C pad interface — directly to tri-state buffers
    .scl_i              (i_scl),
    .scl_oen_o          (o_scl_oen),
    .sda_i              (i_sda),
    .sda_oen_o          (o_sda_oen)
);

reg [15:0] i2c_prescale_cnt_r;
reg        i2c_core_ena_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        i2c_prescale_cnt_r <= 16'd0;
        i2c_core_ena_r     <= 1'b0;
    end else if (i2c_prescale_cnt_r == 16'd0) begin
        i2c_prescale_cnt_r <= I2C_PRESCALE;
        i2c_core_ena_r     <= 1'b1;
    end else begin
        i2c_prescale_cnt_r <= i2c_prescale_cnt_r - 16'd1;
        i2c_core_ena_r     <= 1'b0;
    end
end

endmodule
