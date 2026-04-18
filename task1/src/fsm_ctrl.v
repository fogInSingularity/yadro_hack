`timescale 1ns / 1ps

module fsm_ctrl #(
    parameter CLK_FREQ = 50_000_000 // Базовая частота для расчёта таймеров
)(
    input  wire        clk,
    input  wire        rst_n,

    // Пользовательский интерфейс данных
    output reg  [15:0] o_distance,
    output reg         o_distance_valid,
    output reg         o_sensor_xshut,    // Управление пином XSHUT сенсора

    // Интерфейс подключения к i2c_master_core
    output reg         o_cmd_valid,
    output reg  [2:0]  o_cmd,
    output reg  [7:0]  o_din,
    input  wire        i_ready,
    input  wire [7:0]  i_dout,
    input  wire        i_rx_ack,          // Можно использовать для дебага (NACK)
    input  wire        i_arb_lost,
    output reg         o_arb_lost_clear,
    input  wire        i_busy
);

    // =========================================================================
    // Константы I2C Команд
    // =========================================================================
    localparam CMD_START   = 3'd1;
    localparam CMD_WRITE   = 3'd2;
    localparam CMD_READ    = 3'd3;
    localparam CMD_STOP    = 3'd4;
    localparam CMD_RESTART = 3'd5;

    // Расчёт задержек:
    localparam [31:0] MS_1_2 = (CLK_FREQ / 10000) * 12; // 1.2 мс
    localparam [31:0] MS_1_0 = (CLK_FREQ / 1000);       // 1.0 мс

    // =========================================================================
    // Внутренний автомат: I2C Транзакции
    // =========================================================================
    localparam OP_WR8  = 2'd0;
    localparam OP_RD8  = 2'd1;
    localparam OP_RD16 = 2'd2;

    localparam I_IDLE        = 4'd0;
    localparam I_START1      = 4'd1;
    localparam I_WR_ADDR1    = 4'd2;
    localparam I_WR_REG      = 4'd3;
    localparam I_WR_DATA     = 4'd4;
    localparam I_RESTART     = 4'd5;
    localparam I_WR_ADDR2    = 4'd6;
    localparam I_RD_DATA8    = 4'd7;
    localparam I_RD_DATA16_H = 4'd8;
    localparam I_RD_DATA16_L = 4'd9;
    localparam I_STOP        = 4'd10;
    localparam I_DONE        = 4'd11;

    reg [3:0]  i_state;
    reg        tx_req;
    reg        tx_ack;
    reg        tx_done;
    reg[1:0]  tx_op;
    reg [7:0]  tx_reg;
    reg [7:0]  tx_wdata;
    reg [15:0] tx_rdata;

    // Вспомогательный handshake для I2C Master Core
    reg        do_cmd;
    reg [2:0]  next_cmd;
    reg [7:0]  next_din;
    reg        cmd_done;
    reg [1:0]  hs_state;

    // =========================================================================
    // Внешний автомат: Секвенсор
    // =========================================================================
    localparam O_ST_RESET = 3'd0;
    localparam O_ST_INIT  = 3'd1;
    localparam O_ST_DELAY = 3'd2;
    localparam O_ST_EXEC  = 3'd3;
    localparam O_ST_WAIT  = 3'd4;

    reg [2:0]  o_state;
    reg[4:0]  seq_step;
    reg [31:0] timer;
    reg[7:0]  stop_variable;

    // Детектор потери арбитража для аварийного перезапуска
    reg arb_lost_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) arb_lost_d <= 1'b0;
        else arb_lost_d <= i_arb_lost;
    end
    wire arb_lost_edge = i_arb_lost && !arb_lost_d;

    // Комбинаторная логика для шагов
    reg [1:0] tx_op_c;
    reg [7:0] tx_reg_c;
    reg [7:0] tx_wdata_c;

    always @(*) begin
        tx_op_c    = OP_WR8;
        tx_reg_c   = 8'h00;
        tx_wdata_c = 8'h00;
        case (seq_step)
            // Identity
            1:  begin tx_op_c = OP_RD8; tx_reg_c = 8'hC0; end
            2:  begin tx_op_c = OP_RD8; tx_reg_c = 8'hC1; end
            3:  begin tx_op_c = OP_RD8; tx_reg_c = 8'hC2; end
            // ST Magic init blob
            4:  begin tx_op_c = OP_WR8; tx_reg_c = 8'h88; tx_wdata_c = 8'h00; end
            5:  begin tx_op_c = OP_WR8; tx_reg_c = 8'h80; tx_wdata_c = 8'h01; end
            6:  begin tx_op_c = OP_WR8; tx_reg_c = 8'hFF; tx_wdata_c = 8'h01; end
            7:  begin tx_op_c = OP_WR8; tx_reg_c = 8'h00; tx_wdata_c = 8'h00; end
            8:  begin tx_op_c = OP_RD8; tx_reg_c = 8'h91; end // Читаем stop_variable
            9:  begin tx_op_c = OP_WR8; tx_reg_c = 8'h00; tx_wdata_c = 8'h01; end
            10: begin tx_op_c = OP_WR8; tx_reg_c = 8'hFF; tx_wdata_c = 8'h00; end
            11: begin tx_op_c = OP_WR8; tx_reg_c = 8'h80; tx_wdata_c = 8'h00; end
            // Disable Interrupt
            12: begin tx_op_c = OP_WR8; tx_reg_c = 8'h0A; tx_wdata_c = 8'h04; end
            13: begin tx_op_c = OP_WR8; tx_reg_c = 8'h84; tx_wdata_c = 8'h00; end
            // SINGLE SHOT LOOP -> Arm Measurement
            14: begin tx_op_c = OP_WR8; tx_reg_c = 8'h80; tx_wdata_c = 8'h01; end
            15: begin tx_op_c = OP_WR8; tx_reg_c = 8'hFF; tx_wdata_c = 8'h01; end
            16: begin tx_op_c = OP_WR8; tx_reg_c = 8'h00; tx_wdata_c = 8'h00; end
            17: begin tx_op_c = OP_WR8; tx_reg_c = 8'h91; tx_wdata_c = stop_variable; end
            18: begin tx_op_c = OP_WR8; tx_reg_c = 8'h00; tx_wdata_c = 8'h01; end
            19: begin tx_op_c = OP_WR8; tx_reg_c = 8'hFF; tx_wdata_c = 8'h00; end
            20: begin tx_op_c = OP_WR8; tx_reg_c = 8'h80; tx_wdata_c = 8'h00; end
            // Trigger
            21: begin tx_op_c = OP_WR8; tx_reg_c = 8'h00; tx_wdata_c = 8'h01; end
            // Poll Ready
            22: begin tx_op_c = OP_RD8; tx_reg_c = 8'h13; end
            // 23 — это программная задержка (обрабатывается отдельно)
            // Read Distance
            24: begin tx_op_c = OP_RD16; tx_reg_c = 8'h1E; end
            // Clear Interrupt
            25: begin tx_op_c = OP_WR8; tx_reg_c = 8'h0B; tx_wdata_c = 8'h01; end
            default: ;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Глобальный сброс
            o_distance       <= 16'd0;
            o_distance_valid <= 1'b0;
            o_sensor_xshut   <= 1'b0;
            
            o_cmd_valid      <= 1'b0;
            o_cmd            <= 3'd0;
            o_din            <= 8'd0;
            o_arb_lost_clear <= 1'b0;

            hs_state <= 2'd0;
            cmd_done <= 1'b0;
            do_cmd   <= 1'b0;
            next_cmd <= 3'd0;
            next_din <= 8'd0;

            i_state <= I_IDLE;
            tx_req  <= 1'b0;
            tx_ack  <= 1'b0;
            tx_done <= 1'b0;
            
            o_state  <= O_ST_RESET;
            seq_step <= 5'd0;
            timer    <= 32'd0;

        end else if (arb_lost_edge) begin
            // При потере арбитража полностью перегружаем FSM
            o_arb_lost_clear <= 1'b1;
            o_state          <= O_ST_RESET;
            i_state          <= I_IDLE;
            tx_req           <= 1'b0;
            do_cmd           <= 1'b0;
            o_cmd_valid      <= 1'b0;
            hs_state         <= 2'd0;
            cmd_done         <= 1'b0;
        end else begin
            // Сброс импульсов
            o_distance_valid <= 1'b0;
            o_arb_lost_clear <= 1'b0;
            cmd_done         <= 1'b0;

            // -----------------------------------------------------------------
            // Handshake Автомат — Обращение к ядру I2C
            // -----------------------------------------------------------------
            case (hs_state)
                2'd0: begin
                    if (do_cmd && i_ready) begin
                        o_cmd_valid <= 1'b1;
                        o_cmd       <= next_cmd;
                        o_din       <= next_din;
                        hs_state    <= 2'd1;
                    end
                end
                2'd1: begin
                    if (!i_ready) begin
                        o_cmd_valid <= 1'b0;
                        hs_state    <= 2'd2;
                    end
                end
                2'd2: begin
                    if (i_ready) begin
                        cmd_done <= 1'b1;
                        hs_state <= 2'd3;
                    end
                end
                2'd3: begin
                    // Ожидаем, пока внутренний автомат не снимет запрос перед следующим
                    if (!do_cmd) hs_state <= 2'd0;
                end
            endcase

            // -----------------------------------------------------------------
            // Внутренний Автомат — Последовательности транзакций
            // -----------------------------------------------------------------
            case (i_state)
                I_IDLE: begin
                    tx_ack  <= 1'b0;
                    tx_done <= 1'b0;
                    if (tx_req) begin
                        tx_ack  <= 1'b1;
                        i_state <= I_START1;
                    end
                end
                I_START1: begin
                    tx_ack <= 1'b0;
                    do_cmd <= 1'b1; next_cmd <= CMD_START;
                    if (cmd_done) begin do_cmd <= 1'b0; i_state <= I_WR_ADDR1; end
                end
                I_WR_ADDR1: begin
                    do_cmd <= 1'b1; next_cmd <= CMD_WRITE; next_din <= 8'h52; // 0x29 + Write
                    if (cmd_done) begin do_cmd <= 1'b0; i_state <= I_WR_REG; end
                end
                I_WR_REG: begin
                    do_cmd <= 1'b1; next_cmd <= CMD_WRITE; next_din <= tx_reg;
                    if (cmd_done) begin
                        do_cmd <= 1'b0;
                        if (tx_op == OP_WR8) i_state <= I_WR_DATA;
                        else i_state <= I_RESTART;
                    end
                end
                I_WR_DATA: begin
                    do_cmd <= 1'b1; next_cmd <= CMD_WRITE; next_din <= tx_wdata;
                    if (cmd_done) begin do_cmd <= 1'b0; i_state <= I_STOP; end
                end
                I_RESTART: begin
                    do_cmd <= 1'b1; next_cmd <= CMD_RESTART;
                    if (cmd_done) begin do_cmd <= 1'b0; i_state <= I_WR_ADDR2; end
                end
                I_WR_ADDR2: begin
                    do_cmd <= 1'b1; next_cmd <= CMD_WRITE; next_din <= 8'h53; // 0x29 + Read
                    if (cmd_done) begin
                        do_cmd <= 1'b0;
                        if (tx_op == OP_RD8) i_state <= I_RD_DATA8;
                        else i_state <= I_RD_DATA16_H;
                    end
                end
                I_RD_DATA8: begin
                    do_cmd <= 1'b1; next_cmd <= CMD_READ; next_din <= 8'h01; // NACK
                    if (cmd_done) begin do_cmd <= 1'b0; tx_rdata[7:0] <= i_dout; i_state <= I_STOP; end
                end
                I_RD_DATA16_H: begin
                    do_cmd <= 1'b1; next_cmd <= CMD_READ; next_din <= 8'h00; // ACK
                    if (cmd_done) begin do_cmd <= 1'b0; tx_rdata[15:8] <= i_dout; i_state <= I_RD_DATA16_L; end
                end
                I_RD_DATA16_L: begin
                    do_cmd <= 1'b1; next_cmd <= CMD_READ; next_din <= 8'h01; // NACK
                    if (cmd_done) begin do_cmd <= 1'b0; tx_rdata[7:0] <= i_dout; i_state <= I_STOP; end
                end
                I_STOP: begin
                    do_cmd <= 1'b1; next_cmd <= CMD_STOP;
                    if (cmd_done) begin do_cmd <= 1'b0; i_state <= I_DONE; end
                end
                I_DONE: begin
                    tx_done <= 1'b1;
                    i_state <= I_IDLE;
                end
            endcase

            // -----------------------------------------------------------------
            // Внешний Автомат — Секвенсор VL53L5CX
            // -----------------------------------------------------------------
            case (o_state)
                O_ST_RESET: begin
                    o_sensor_xshut <= 1'b0; // Удерживаем сенсор в Standby
                    timer          <= MS_1_0;
                    seq_step       <= 5'd0;
                    o_state        <= O_ST_DELAY;
                end
                O_ST_INIT: begin
                    o_sensor_xshut <= 1'b1; // Отпускаем Standby
                    timer          <= MS_1_2; // Ждем 1.2мс (tBOOT)
                    seq_step       <= 5'd1;
                    o_state        <= O_ST_DELAY;
                end
                O_ST_DELAY: begin
                    if (timer != 0) begin
                        timer <= timer - 1;
                    end else begin
                        if (seq_step == 0) o_state <= O_ST_INIT;
                        else if (seq_step == 22) o_state <= O_ST_EXEC; // Возврат к чтению 0x13
                        else o_state <= O_ST_EXEC;
                    end
                end
                O_ST_EXEC: begin
                    if (seq_step == 23) begin
                        // Специальный шаг: задержка поллинга (~1мс)
                        timer    <= MS_1_0;
                        o_state  <= O_ST_DELAY;
                        seq_step <= 5'd22;
                    end else begin
                        tx_op    <= tx_op_c;
                        tx_reg   <= tx_reg_c;
                        tx_wdata <= tx_wdata_c;
                        tx_req   <= 1'b1;
                        
                        if (tx_ack) begin
                            tx_req  <= 1'b0;
                            o_state <= O_ST_WAIT;
                        end
                    end
                end
                O_ST_WAIT: begin
                    if (tx_done) begin
                        o_state <= O_ST_EXEC; // По умолчанию продолжаем
                        
                        case (seq_step)
                            1: if (tx_rdata[7:0] == 8'hEE) seq_step <= 5'd2; else seq_step <= 5'd1; // Retry
                            2: if (tx_rdata[7:0] == 8'hAA) seq_step <= 5'd3; else seq_step <= 5'd1;
                            3: if (tx_rdata[7:0] == 8'h10) seq_step <= 5'd4; else seq_step <= 5'd1;
                            
                            // Сохранение stop_variable
                            8: begin 
                                stop_variable <= tx_rdata[7:0]; 
                                seq_step      <= seq_step + 1;
                            end
                            
                            // Проверка готовности
                            22: begin
                                if ((tx_rdata[7:0] & 8'h07) != 0) seq_step <= 5'd24; // Готов -> к чтению
                                else seq_step <= 5'd23; // Иначе -> к задержке 1 мс
                            end
                            
                            // Выгрузка данных и генерация валидного импульса
                            24: begin
                                o_distance       <= tx_rdata;
                                o_distance_valid <= 1'b1;
                                seq_step         <= 5'd25;
                            end
                            
                            // Закольцовка после Clear Interrupt
                            25: seq_step <= 5'd14; // Возврат к Arm Measurement (регистр 0x80)
                            
                            // Все остальные обычные записи просто идут дальше
                            default: seq_step <= seq_step + 1;
                        endcase
                    end
                end
            endcase
        end
    end

endmodule