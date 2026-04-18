module #(
    parameter DATAW = 16
    parameter DEFAULT_PRESCALE   = 16'd124   // 100 MHz → 100 kHz
) sensor_ctrl (
    input wire clk,
    input wire rst_n,

    output xshut_o,

    input wire scl_i,
    output reg scl_oen_o,
    input wire sda_i,
    output reg sda_oen_o,

    output wire [DATAW - 1 : 0] o_distance
);

wire       cmd_valid;
wire [2:0] cmd;
wire [7:0] din;
wire       ready;
wire [7:0] dout;
wire       rx_ack;    
wire       arb_lost;
wire       arb_lost_clear;
wire       busy;

reg [15:0] prescale_r;       // PRESCALE
reg [15:0] prescale_cnt_r;
reg        core_ena_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        prescale_r <= DEFAULT_PRESCALE;
        prescale_cnt_r <= 16'd0;
        core_ena_r     <= 1'b0;
    end else if (!ctrl_en_r) begin
        prescale_cnt_r <= prescale_r;
        core_ena_r     <= 1'b0;
    end else if (prescale_cnt_r == 16'd0) begin
        prescale_cnt_r <= prescale_r;
        core_ena_r     <= 1'b1;
    end else begin
        prescale_cnt_r <= prescale_cnt_r - 16'd1;
        core_ena_r     <= 1'b0;
    end
end

i2c_master_core i2c_master_core_inst (
    .clk_i(clk),
    .rstn_i(rst_n),
    .ena_i(core_ena_r),

    // Command interface (active when ready_o == 1)
    cmd_valid_i(cmd_valid),
    cmd_i(cmd),            
    din_i(din),            

    dout_o(dout),          
    rx_ack_o(rx_ack),      
    ready_o(ready),     

    // Status
    .arb_lost_o(arb_lost),     
    .arb_lost_clear_i(arb_lost_clear),
    .busy_o(busy),        

    // I2C pad interface — directly to tri-state buffers
    scl_i(scl_i),           
    scl_oen_o(scl_oen_o),       
    sda_i(sda_i),           
    sda_oen_o(sda_oen_o)        
);

fsm_ctrl fsm_ctrl_inst (
    .clk(clk),
    .rst_n(rst_n),

    .o_distance(o_distance),
    .o_sensor_xshut(xshut_o),

    // Интерфейс подключения к i2c_master_core
    .o_cmd_valid(cmd_valid),
    .o_cmd(cmd),
    .o_din(din),
    .i_ready(ready),
    .i_dout(dout),
    .i_rx_ack(rx_ack),
    .i_arb_lost(arb_lost),
    .o_arb_lost_clear(arb_lost_clear),
    .i_busy(busy)
);

endmodule