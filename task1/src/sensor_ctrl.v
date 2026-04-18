module #(
    DATAW = 16
) sensor_ctrl (
    input wire clk,
    input wire rst_n,

    input ena_i,
    output xshut_o,

    input wire scl_i,
    output reg scl_oen_o,
    input wire sda_i,
    output reg sda_oen_o,

    output wire [DATAW - 1 : 0] o_distance
);

output wire       cmd_valid;
output wire [2:0] cmd;
output wire [7:0] din;
input  wire       ready;
input  wire [7:0] dout;
input  wire       rx_ack;    
input  wire       arb_lost;
output wire       arb_lost_clear;
input  wire       busy;

i2c_master_core i2c_master_core_inst (
    .clk_i(clk),
    .rstn_i(rst_n),
    .ena_i(ena_i),

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