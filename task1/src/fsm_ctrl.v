module fsm_ctrl #(
    parameter CLK_FREQ = 50_000_000 // Базовая частота для расчёта таймеров
)(
    input  wire        clk,
    input  wire        rst_n,

    // Пользовательский интерфейс данных
    output reg  [15:0] o_distance,
    output reg         o_distance_valid,
    output reg         o_sensor_xshut,

    // Интерфейс подключения к i2c_master_core
    output reg         o_cmd_valid,
    output reg  [2:0]  o_cmd,
    output reg  [7:0]  o_din,
    input  wire        i_ready,
    input  wire [7:0]  i_dout,
    input  wire        i_rx_ack,
    input  wire        i_arb_lost,
    output reg         o_arb_lost_clear,
    input  wire        i_busy
);

    // -------------------------------------------------------------------------
    // I2C command codes expected by the documented i2c_master_core
    // -------------------------------------------------------------------------
    localparam [2:0] CMD_NOP     = 3'd0;
    localparam [2:0] CMD_START   = 3'd1;
    localparam [2:0] CMD_WRITE   = 3'd2;
    localparam [2:0] CMD_READ    = 3'd3;
    localparam [2:0] CMD_STOP    = 3'd4;
    localparam [2:0] CMD_RESTART = 3'd5;

    // VL53L0X default 7-bit I2C address = 0x29
    localparam [7:0] VL53_ADDR_W = 8'h52;
    localparam [7:0] VL53_ADDR_R = 8'h53;

    // Register addresses used here
    localparam [7:0] REG_SYSRANGE_START                       = 8'h00;
    localparam [7:0] REG_SYSTEM_SEQUENCE_CONFIG               = 8'h01;
    localparam [7:0] REG_SYSTEM_INTERRUPT_CONFIG_GPIO         = 8'h0A;
    localparam [7:0] REG_SYSTEM_INTERRUPT_CLEAR               = 8'h0B;
    localparam [7:0] REG_RESULT_INTERRUPT_STATUS              = 8'h13;
    localparam [7:0] REG_RESULT_RANGE_MM                      = 8'h1E; // RESULT_RANGE_STATUS + 10
    localparam [7:0] REG_FINAL_RANGE_MIN_COUNT_RATE           = 8'h44;
    localparam [7:0] REG_MSRC_CONFIG_CONTROL                  = 8'h60;
    localparam [7:0] REG_FINAL_RANGE_TIMEOUT_MACROP_HI        = 8'h71;
    localparam [7:0] REG_GPIO_HV_MUX_ACTIVE_HIGH              = 8'h84;
    localparam [7:0] REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV     = 8'h89;
    localparam [7:0] REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0     = 8'hB0;
    localparam [7:0] REG_GLOBAL_CONFIG_REF_EN_START_SELECT    = 8'hB6;
    localparam [7:0] REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD  = 8'h4E;
    localparam [7:0] REG_DYNAMIC_SPAD_REF_EN_START_OFFSET     = 8'h4F;

    // Delays after toggling XSHUT. 10 ms is conservative for this sensor.
    localparam integer XSHUT_LOW_CYCLES  = (CLK_FREQ / 1000) * 10;
    localparam integer XSHUT_HIGH_CYCLES = (CLK_FREQ / 1000) * 10;

    // -------------------------------------------------------------------------
    // Transaction engine opcodes: one opcode = one complete register transaction
    // -------------------------------------------------------------------------
    localparam [3:0] OP_NONE   = 4'd0;
    localparam [3:0] OP_WRITE8 = 4'd1;
    localparam [3:0] OP_WRITE16= 4'd2;
    localparam [3:0] OP_WRITE32= 4'd3;
    localparam [3:0] OP_WRITE6 = 4'd4;
    localparam [3:0] OP_READ8  = 4'd5;
    localparam [3:0] OP_READ16 = 4'd6;
    localparam [3:0] OP_READ6  = 4'd7;

    localparam [4:0] TR_IDLE       = 5'd0;
    localparam [4:0] TR_START      = 5'd1;
    localparam [4:0] TR_ADDR_W     = 5'd2;
    localparam [4:0] TR_REG        = 5'd3;
    localparam [4:0] TR_WDATA      = 5'd4;
    localparam [4:0] TR_WDATA_ADV  = 5'd5;
    localparam [4:0] TR_RESTART    = 5'd6;
    localparam [4:0] TR_ADDR_R     = 5'd7;
    localparam [4:0] TR_RDATA      = 5'd8;
    localparam [4:0] TR_RDATA_ADV  = 5'd9;
    localparam [4:0] TR_STOP       = 5'd10;
    localparam [4:0] TR_FINISH     = 5'd11;
    localparam [4:0] TR_CMD_ISSUE  = 5'd12;
    localparam [4:0] TR_CMD_WAIT   = 5'd13;
    localparam [4:0] TR_CMD_ACCEPT = 5'd14;

    reg        tr_busy;
    reg        tr_done;
    reg        tr_error;
    reg [3:0]  tr_kind;
    reg [4:0]  tr_state;
    reg [4:0]  tr_after_cmd;
    reg [7:0]  tr_reg_addr;
    reg [31:0] tr_wdata;
    reg [2:0]  tr_index;
    reg [2:0]  tr_wr_len;
    reg [2:0]  tr_rd_len;
    reg [2:0]  core_cmd_latched;
    reg [7:0]  core_din_latched;
    reg [7:0]  last_dout;

    reg [7:0] read_buf [0:5];
    reg [7:0] ref_spad_map [0:5];

    wire tr_is_write = (tr_kind == OP_WRITE8)  ||
                       (tr_kind == OP_WRITE16) ||
                       (tr_kind == OP_WRITE32) ||
                       (tr_kind == OP_WRITE6);

    reg [7:0] wr_byte;
    always @* begin
        wr_byte = 8'h00;
        case (tr_kind)
            OP_WRITE8: begin
                wr_byte = tr_wdata[7:0];
            end

            OP_WRITE16: begin
                case (tr_index)
                    3'd0: wr_byte = tr_wdata[15:8];
                    default: wr_byte = tr_wdata[7:0];
                endcase
            end

            OP_WRITE32: begin
                case (tr_index)
                    3'd0: wr_byte = tr_wdata[31:24];
                    3'd1: wr_byte = tr_wdata[23:16];
                    3'd2: wr_byte = tr_wdata[15:8];
                    default: wr_byte = tr_wdata[7:0];
                endcase
            end

            OP_WRITE6: begin
                case (tr_index)
                    3'd0: wr_byte = ref_spad_map[0];
                    3'd1: wr_byte = ref_spad_map[1];
                    3'd2: wr_byte = ref_spad_map[2];
                    3'd3: wr_byte = ref_spad_map[3];
                    3'd4: wr_byte = ref_spad_map[4];
                    default: wr_byte = ref_spad_map[5];
                endcase
            end

            default: begin
                wr_byte = 8'h00;
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // High-level VL53L0X sequencer
    // -------------------------------------------------------------------------
    localparam [7:0] H_RESET_LOW       = 8'd0;
    localparam [7:0] H_RESET_HIGH      = 8'd1;
    localparam [7:0] H_WAIT_BUS_IDLE   = 8'd2;
    localparam [7:0] H_INIT_STEP       = 8'd3;
    localparam [7:0] H_WAIT_OP         = 8'd4;
    localparam [7:0] H_SPAD_PREP       = 8'd5;
    localparam [7:0] H_SPAD_LOOP       = 8'd6;
    localparam [7:0] H_TUNING_LOOP     = 8'd7;
    localparam [7:0] H_POLL_STATUS     = 8'd8;
    localparam [7:0] H_POLL_RANGE      = 8'd9;
    localparam [7:0] H_POLL_CLEAR      = 8'd10;

    localparam [3:0] POST_ADV_INIT_STEP = 4'd0;
    localparam [3:0] POST_SAVE_TMP      = 4'd1;
    localparam [3:0] POST_SAVE_STOPVAR  = 4'd2;
    localparam [3:0] POST_WAIT_NONZERO  = 4'd3;
    localparam [3:0] POST_SAVE_SPAD     = 4'd4;
    localparam [3:0] POST_COPY_REF_MAP  = 4'd5;
    localparam [3:0] POST_WAIT_IRQ      = 4'd6;
    localparam [3:0] POST_TUNING_ADV    = 4'd7;
    localparam [3:0] POST_POLL_STATUS   = 4'd8;
    localparam [3:0] POST_POLL_RANGE    = 4'd9;
    localparam [3:0] POST_POLL_CLEAR    = 4'd10;

    reg [7:0]  h_state;
    reg [7:0]  init_step;
    reg [3:0]  post_action;
    reg [31:0] delay_cnt;
    reg [7:0]  tmp_byte;
    reg [7:0]  stop_variable;
    reg [7:0]  tuning_index;

    reg [6:0]  spad_count;
    reg        spad_type_is_aperture;
    reg [5:0]  first_spad_to_enable;
    reg [5:0]  spad_index;
    reg [6:0]  spads_enabled;

    integer ri;

    // -------------------------------------------------------------------------
    // Tuning ROM: DefaultTuningSettings from the common VL53L0X API sequence
    // -------------------------------------------------------------------------
    localparam [7:0] TUNING_LEN = 8'd80;

    function [7:0] tuning_reg;
        input [7:0] idx;
        begin
            case (idx)
                8'd0:  tuning_reg = 8'hFF;  8'd1:  tuning_reg = 8'h00;
                8'd2:  tuning_reg = 8'hFF;  8'd3:  tuning_reg = 8'h09;
                8'd4:  tuning_reg = 8'h10;  8'd5:  tuning_reg = 8'h11;
                8'd6:  tuning_reg = 8'h24;  8'd7:  tuning_reg = 8'h25;
                8'd8:  tuning_reg = 8'h75;  8'd9:  tuning_reg = 8'hFF;
                8'd10: tuning_reg = 8'h4E;  8'd11: tuning_reg = 8'h48;
                8'd12: tuning_reg = 8'h30;  8'd13: tuning_reg = 8'hFF;
                8'd14: tuning_reg = 8'h30;  8'd15: tuning_reg = 8'h54;
                8'd16: tuning_reg = 8'h31;  8'd17: tuning_reg = 8'h32;
                8'd18: tuning_reg = 8'h40;  8'd19: tuning_reg = 8'h46;
                8'd20: tuning_reg = 8'h60;  8'd21: tuning_reg = 8'h27;
                8'd22: tuning_reg = 8'h50;  8'd23: tuning_reg = 8'h51;
                8'd24: tuning_reg = 8'h52;  8'd25: tuning_reg = 8'h56;
                8'd26: tuning_reg = 8'h57;  8'd27: tuning_reg = 8'h61;
                8'd28: tuning_reg = 8'h62;  8'd29: tuning_reg = 8'h64;
                8'd30: tuning_reg = 8'h65;  8'd31: tuning_reg = 8'h66;
                8'd32: tuning_reg = 8'hFF;  8'd33: tuning_reg = 8'h22;
                8'd34: tuning_reg = 8'h47;  8'd35: tuning_reg = 8'h49;
                8'd36: tuning_reg = 8'h4A;  8'd37: tuning_reg = 8'hFF;
                8'd38: tuning_reg = 8'h7A;  8'd39: tuning_reg = 8'h7B;
                8'd40: tuning_reg = 8'h78;  8'd41: tuning_reg = 8'hFF;
                8'd42: tuning_reg = 8'h23;  8'd43: tuning_reg = 8'h42;
                8'd44: tuning_reg = 8'h44;  8'd45: tuning_reg = 8'h45;
                8'd46: tuning_reg = 8'h46;  8'd47: tuning_reg = 8'h40;
                8'd48: tuning_reg = 8'h0E;  8'd49: tuning_reg = 8'h20;
                8'd50: tuning_reg = 8'h43;  8'd51: tuning_reg = 8'hFF;
                8'd52: tuning_reg = 8'h34;  8'd53: tuning_reg = 8'h35;
                8'd54: tuning_reg = 8'hFF;  8'd55: tuning_reg = 8'h31;
                8'd56: tuning_reg = 8'h4B;  8'd57: tuning_reg = 8'h4C;
                8'd58: tuning_reg = 8'h4D;  8'd59: tuning_reg = 8'hFF;
                8'd60: tuning_reg = 8'h44;  8'd61: tuning_reg = 8'h45;
                8'd62: tuning_reg = 8'h47;  8'd63: tuning_reg = 8'h48;
                8'd64: tuning_reg = 8'h67;  8'd65: tuning_reg = 8'h70;
                8'd66: tuning_reg = 8'h71;  8'd67: tuning_reg = 8'h72;
                8'd68: tuning_reg = 8'h76;  8'd69: tuning_reg = 8'h77;
                8'd70: tuning_reg = 8'hFF;  8'd71: tuning_reg = 8'h0D;
                8'd72: tuning_reg = 8'hFF;  8'd73: tuning_reg = 8'h80;
                8'd74: tuning_reg = 8'h01;  8'd75: tuning_reg = 8'hFF;
                8'd76: tuning_reg = 8'h8E;  8'd77: tuning_reg = 8'h00;
                8'd78: tuning_reg = 8'hFF;  8'd79: tuning_reg = 8'h80;
                default: tuning_reg = 8'h00;
            endcase
        end
    endfunction

    function [7:0] tuning_val;
        input [7:0] idx;
        begin
            case (idx)
                8'd0:  tuning_val = 8'h01;  8'd1:  tuning_val = 8'h00;
                8'd2:  tuning_val = 8'h00;  8'd3:  tuning_val = 8'h00;
                8'd4:  tuning_val = 8'h00;  8'd5:  tuning_val = 8'h00;
                8'd6:  tuning_val = 8'h01;  8'd7:  tuning_val = 8'hFF;
                8'd8:  tuning_val = 8'h00;  8'd9:  tuning_val = 8'h01;
                8'd10: tuning_val = 8'h2C;  8'd11: tuning_val = 8'h00;
                8'd12: tuning_val = 8'h20;  8'd13: tuning_val = 8'h00;
                8'd14: tuning_val = 8'h09;  8'd15: tuning_val = 8'h00;
                8'd16: tuning_val = 8'h04;  8'd17: tuning_val = 8'h03;
                8'd18: tuning_val = 8'h83;  8'd19: tuning_val = 8'h25;
                8'd20: tuning_val = 8'h00;  8'd21: tuning_val = 8'h00;
                8'd22: tuning_val = 8'h06;  8'd23: tuning_val = 8'h00;
                8'd24: tuning_val = 8'h96;  8'd25: tuning_val = 8'h08;
                8'd26: tuning_val = 8'h30;  8'd27: tuning_val = 8'h00;
                8'd28: tuning_val = 8'h00;  8'd29: tuning_val = 8'h00;
                8'd30: tuning_val = 8'h00;  8'd31: tuning_val = 8'hA0;
                8'd32: tuning_val = 8'h01;  8'd33: tuning_val = 8'h32;
                8'd34: tuning_val = 8'h14;  8'd35: tuning_val = 8'hFF;
                8'd36: tuning_val = 8'h00;  8'd37: tuning_val = 8'h00;
                8'd38: tuning_val = 8'h0A;  8'd39: tuning_val = 8'h00;
                8'd40: tuning_val = 8'h21;  8'd41: tuning_val = 8'h01;
                8'd42: tuning_val = 8'h34;  8'd43: tuning_val = 8'h00;
                8'd44: tuning_val = 8'hFF;  8'd45: tuning_val = 8'h26;
                8'd46: tuning_val = 8'h05;  8'd47: tuning_val = 8'h40;
                8'd48: tuning_val = 8'h06;  8'd49: tuning_val = 8'h1A;
                8'd50: tuning_val = 8'h40;  8'd51: tuning_val = 8'h00;
                8'd52: tuning_val = 8'h03;  8'd53: tuning_val = 8'h44;
                8'd54: tuning_val = 8'h01;  8'd55: tuning_val = 8'h04;
                8'd56: tuning_val = 8'h09;  8'd57: tuning_val = 8'h05;
                8'd58: tuning_val = 8'h04;  8'd59: tuning_val = 8'h00;
                8'd60: tuning_val = 8'h00;  8'd61: tuning_val = 8'h20;
                8'd62: tuning_val = 8'h08;  8'd63: tuning_val = 8'h28;
                8'd64: tuning_val = 8'h00;  8'd65: tuning_val = 8'h04;
                8'd66: tuning_val = 8'h01;  8'd67: tuning_val = 8'hFE;
                8'd68: tuning_val = 8'h00;  8'd69: tuning_val = 8'h00;
                8'd70: tuning_val = 8'h01;  8'd71: tuning_val = 8'h01;
                8'd72: tuning_val = 8'h00;  8'd73: tuning_val = 8'h01;
                8'd74: tuning_val = 8'hF8;  8'd75: tuning_val = 8'h01;
                8'd76: tuning_val = 8'h01;  8'd77: tuning_val = 8'h01;
                8'd78: tuning_val = 8'h00;  8'd79: tuning_val = 8'h00;
                default: tuning_val = 8'h00;
            endcase
        end
    endfunction

    // -------------------------------------------------------------------------
    // Helper tasks. These are syntactic shorthand; synthesis expands them.
    // -------------------------------------------------------------------------
    task send_core_cmd;
        input [2:0] cmd;
        input [7:0] din;
        input [4:0] after_state;
        begin
            core_cmd_latched <= cmd;
            core_din_latched <= din;
            tr_after_cmd     <= after_state;
            tr_state         <= TR_CMD_ISSUE;
        end
    endtask

    task start_transaction;
        input [3:0] kind;
        input [7:0] reg_addr;
        input [31:0] wdata;
        input [3:0] post;
        begin
            tr_busy     <= 1'b1;
            tr_done     <= 1'b0;
            tr_error    <= 1'b0;
            tr_kind     <= kind;
            tr_reg_addr <= reg_addr;
            tr_wdata    <= wdata;
            tr_index    <= 3'd0;
            tr_state    <= TR_START;
            post_action <= post;
            h_state     <= H_WAIT_OP;

            case (kind)
                OP_WRITE8:  tr_wr_len <= 3'd1;
                OP_WRITE16: tr_wr_len <= 3'd2;
                OP_WRITE32: tr_wr_len <= 3'd4;
                OP_WRITE6:  tr_wr_len <= 3'd6;
                default:    tr_wr_len <= 3'd0;
            endcase

            case (kind)
                OP_READ8:  tr_rd_len <= 3'd1;
                OP_READ16: tr_rd_len <= 3'd2;
                OP_READ6:  tr_rd_len <= 3'd6;
                default:   tr_rd_len <= 3'd0;
            endcase
        end
    endtask

    task start_write8;
        input [7:0] reg_addr;
        input [7:0] data;
        input [3:0] post;
        begin
            start_transaction(OP_WRITE8, reg_addr, {24'h000000, data}, post);
        end
    endtask

    task start_write16;
        input [7:0] reg_addr;
        input [15:0] data;
        input [3:0] post;
        begin
            start_transaction(OP_WRITE16, reg_addr, {16'h0000, data}, post);
        end
    endtask

    task start_read8;
        input [7:0] reg_addr;
        input [3:0] post;
        begin
            start_transaction(OP_READ8, reg_addr, 32'h00000000, post);
        end
    endtask

    task start_read16;
        input [7:0] reg_addr;
        input [3:0] post;
        begin
            start_transaction(OP_READ16, reg_addr, 32'h00000000, post);
        end
    endtask

    task start_read6;
        input [7:0] reg_addr;
        input [3:0] post;
        begin
            start_transaction(OP_READ6, reg_addr, 32'h00000000, post);
        end
    endtask

    task start_write6;
        input [7:0] reg_addr;
        input [3:0] post;
        begin
            start_transaction(OP_WRITE6, reg_addr, 32'h00000000, post);
        end
    endtask

    task recover_from_error;
        begin
            o_cmd_valid       <= 1'b0;
            o_cmd             <= CMD_NOP;
            o_din             <= 8'h00;
            tr_busy           <= 1'b0;
            tr_done           <= 1'b0;
            tr_error          <= 1'b0;
            tr_state          <= TR_IDLE;
            h_state           <= H_RESET_LOW;
            init_step         <= 8'd0;
            tuning_index      <= 8'd0;
            o_sensor_xshut    <= 1'b0;
            delay_cnt         <= XSHUT_LOW_CYCLES;
        end
    endtask

    // -------------------------------------------------------------------------
    // Main sequential logic
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_distance          <= 16'h0000;
            o_distance_valid    <= 1'b0;
            o_sensor_xshut      <= 1'b0;

            o_cmd_valid         <= 1'b0;
            o_cmd               <= CMD_NOP;
            o_din               <= 8'h00;
            o_arb_lost_clear    <= 1'b0;

            tr_busy             <= 1'b0;
            tr_done             <= 1'b0;
            tr_error            <= 1'b0;
            tr_kind             <= OP_NONE;
            tr_state            <= TR_IDLE;
            tr_after_cmd        <= TR_IDLE;
            tr_reg_addr         <= 8'h00;
            tr_wdata            <= 32'h00000000;
            tr_index            <= 3'd0;
            tr_wr_len           <= 3'd0;
            tr_rd_len           <= 3'd0;
            core_cmd_latched    <= CMD_NOP;
            core_din_latched    <= 8'h00;
            last_dout           <= 8'h00;

            h_state             <= H_RESET_LOW;
            init_step           <= 8'd0;
            post_action         <= POST_ADV_INIT_STEP;
            delay_cnt           <= XSHUT_LOW_CYCLES;
            tmp_byte            <= 8'h00;
            stop_variable       <= 8'h00;
            tuning_index        <= 8'd0;

            spad_count          <= 7'd0;
            spad_type_is_aperture <= 1'b0;
            first_spad_to_enable  <= 6'd0;
            spad_index          <= 6'd0;
            spads_enabled       <= 7'd0;

            for (ri = 0; ri < 6; ri = ri + 1) begin
                read_buf[ri]     <= 8'h00;
                ref_spad_map[ri] <= 8'h00;
            end
        end else begin
            // default pulse outputs
            o_distance_valid <= 1'b0;
            o_arb_lost_clear <= 1'b0;
            tr_done          <= 1'b0;

            if (i_arb_lost) begin
                // The core uses a sticky arb_lost flag. Clear and restart.
                o_arb_lost_clear <= 1'b1;
                recover_from_error();
            end else begin
                // -------------------------------------------------------------
                // Low-level transaction engine
                // -------------------------------------------------------------
                if (tr_busy) begin
                    case (tr_state)
                        TR_IDLE: begin
                            // no-op
                        end

                        TR_START: begin
                            send_core_cmd(CMD_START, 8'h00, TR_ADDR_W);
                        end

                        TR_ADDR_W: begin
                            send_core_cmd(CMD_WRITE, VL53_ADDR_W, TR_REG);
                        end

                        TR_REG: begin
                            if (tr_error) begin
                                send_core_cmd(CMD_STOP, 8'h00, TR_FINISH);
                            end else begin
                                send_core_cmd(CMD_WRITE, tr_reg_addr,
                                              (tr_is_write ? TR_WDATA : TR_RESTART));
                            end
                        end

                        TR_WDATA: begin
                            if (tr_error) begin
                                send_core_cmd(CMD_STOP, 8'h00, TR_FINISH);
                            end else begin
                                send_core_cmd(CMD_WRITE, wr_byte, TR_WDATA_ADV);
                            end
                        end

                        TR_WDATA_ADV: begin
                            if (tr_error) begin
                                send_core_cmd(CMD_STOP, 8'h00, TR_FINISH);
                            end else if (tr_index == (tr_wr_len - 3'd1)) begin
                                send_core_cmd(CMD_STOP, 8'h00, TR_FINISH);
                            end else begin
                                tr_index <= tr_index + 3'd1;
                                tr_state <= TR_WDATA;
                            end
                        end

                        TR_RESTART: begin
                            if (tr_error) begin
                                send_core_cmd(CMD_STOP, 8'h00, TR_FINISH);
                            end else begin
                                send_core_cmd(CMD_RESTART, 8'h00, TR_ADDR_R);
                            end
                        end

                        TR_ADDR_R: begin
                            if (tr_error) begin
                                send_core_cmd(CMD_STOP, 8'h00, TR_FINISH);
                            end else begin
                                send_core_cmd(CMD_WRITE, VL53_ADDR_R, TR_RDATA);
                            end
                        end

                        TR_RDATA: begin
                            if (tr_error) begin
                                send_core_cmd(CMD_STOP, 8'h00, TR_FINISH);
                            end else begin
                                // din[0] = 0 => ACK, 1 => NACK. NACK only last byte.
                                send_core_cmd(CMD_READ,
                                              (tr_index == (tr_rd_len - 3'd1)) ? 8'h01 : 8'h00,
                                              TR_RDATA_ADV);
                            end
                        end

                        TR_RDATA_ADV: begin
                            case (tr_index)
                                3'd0: read_buf[0] <= last_dout;
                                3'd1: read_buf[1] <= last_dout;
                                3'd2: read_buf[2] <= last_dout;
                                3'd3: read_buf[3] <= last_dout;
                                3'd4: read_buf[4] <= last_dout;
                                default: read_buf[5] <= last_dout;
                            endcase

                            if (tr_index == (tr_rd_len - 3'd1)) begin
                                send_core_cmd(CMD_STOP, 8'h00, TR_FINISH);
                            end else begin
                                tr_index <= tr_index + 3'd1;
                                tr_state <= TR_RDATA;
                            end
                        end

                        TR_STOP: begin
                            send_core_cmd(CMD_STOP, 8'h00, TR_FINISH);
                        end

                        TR_FINISH: begin
                            tr_busy  <= 1'b0;
                            tr_done  <= 1'b1;
                            tr_state <= TR_IDLE;
                        end

                        TR_CMD_ISSUE: begin
                            // Do not present a new command until the core reports ready.
                            o_cmd_valid <= 1'b0;
                            o_cmd       <= core_cmd_latched;
                            o_din       <= core_din_latched;
                            if (i_ready) begin
                                tr_state <= TR_CMD_ACCEPT;
                            end
                        end

                        TR_CMD_ACCEPT: begin
                            // Hold cmd_valid until the I2C core accepts it and drops ready.
                            o_cmd_valid <= 1'b1;
                            o_cmd       <= core_cmd_latched;
                            o_din       <= core_din_latched;
                            if (!i_ready) begin
                                o_cmd_valid <= 1'b0;
                                tr_state    <= TR_CMD_WAIT;
                            end
                        end

                        TR_CMD_WAIT: begin
                            o_cmd_valid <= 1'b0;

                            if (i_ready) begin
                                last_dout <= i_dout;
                                if ((core_cmd_latched == CMD_WRITE) && i_rx_ack) begin
                                    tr_error <= 1'b1;
                                end
                                tr_state <= tr_after_cmd;
                            end
                        end

                        default: begin
                            tr_state <= TR_IDLE;
                            tr_busy  <= 1'b0;
                            tr_error <= 1'b1;
                        end
                    endcase
                end else begin
                    o_cmd_valid <= 1'b0;
                end

                // -------------------------------------------------------------
                // High-level VL53L0X FSM
                // -------------------------------------------------------------
                case (h_state)
                    H_RESET_LOW: begin
                        o_sensor_xshut <= 1'b0;
                        if (delay_cnt != 32'd0) begin
                            delay_cnt <= delay_cnt - 32'd1;
                        end else begin
                            o_sensor_xshut <= 1'b1;
                            delay_cnt <= XSHUT_HIGH_CYCLES;
                            h_state <= H_RESET_HIGH;
                        end
                    end

                    H_RESET_HIGH: begin
                        o_sensor_xshut <= 1'b1;
                        if (delay_cnt != 32'd0) begin
                            delay_cnt <= delay_cnt - 32'd1;
                        end else begin
                            h_state <= H_WAIT_BUS_IDLE;
                        end
                    end

                    H_WAIT_BUS_IDLE: begin
                        if (!i_busy && !tr_busy) begin
                            init_step <= 8'd0;
                            h_state <= H_INIT_STEP;
                        end
                    end

                    H_INIT_STEP: begin
                        if (!tr_busy) begin
                            case (init_step)
                                // 2V8 mode: reg 0x89 |= 0x01
                                8'd0:  start_read8 (REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, POST_SAVE_TMP);
                                8'd1:  start_write8(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, tmp_byte | 8'h01, POST_ADV_INIT_STEP);

                                // I2C standard mode
                                8'd2:  start_write8(8'h88, 8'h00, POST_ADV_INIT_STEP);

                                // Read stop_variable
                                8'd3:  start_write8(8'h80, 8'h01, POST_ADV_INIT_STEP);
                                8'd4:  start_write8(8'hFF, 8'h01, POST_ADV_INIT_STEP);
                                8'd5:  start_write8(8'h00, 8'h00, POST_ADV_INIT_STEP);
                                8'd6:  start_read8 (8'h91, POST_SAVE_STOPVAR);
                                8'd7:  start_write8(8'h00, 8'h01, POST_ADV_INIT_STEP);
                                8'd8:  start_write8(8'hFF, 8'h00, POST_ADV_INIT_STEP);
                                8'd9:  start_write8(8'h80, 8'h00, POST_ADV_INIT_STEP);

                                // Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks
                                8'd10: start_read8 (REG_MSRC_CONFIG_CONTROL, POST_SAVE_TMP);
                                8'd11: start_write8(REG_MSRC_CONFIG_CONTROL, tmp_byte | 8'h12, POST_ADV_INIT_STEP);

                                // Final range signal rate limit = 0.25 MCPS = 0x0020 in Q9.7
                                8'd12: start_write16(REG_FINAL_RANGE_MIN_COUNT_RATE, 16'h0020, POST_ADV_INIT_STEP);
                                8'd13: start_write8 (REG_SYSTEM_SEQUENCE_CONFIG, 8'hFF, POST_ADV_INIT_STEP);

                                // getSpadInfo()
                                8'd14: start_write8(8'h80, 8'h01, POST_ADV_INIT_STEP);
                                8'd15: start_write8(8'hFF, 8'h01, POST_ADV_INIT_STEP);
                                8'd16: start_write8(8'h00, 8'h00, POST_ADV_INIT_STEP);
                                8'd17: start_write8(8'hFF, 8'h06, POST_ADV_INIT_STEP);
                                8'd18: start_read8 (8'h83, POST_SAVE_TMP);
                                8'd19: start_write8(8'h83, tmp_byte | 8'h04, POST_ADV_INIT_STEP);
                                8'd20: start_write8(8'hFF, 8'h07, POST_ADV_INIT_STEP);
                                8'd21: start_write8(8'h81, 8'h01, POST_ADV_INIT_STEP);
                                8'd22: start_write8(8'h80, 8'h01, POST_ADV_INIT_STEP);
                                8'd23: start_write8(8'h94, 8'h6B, POST_ADV_INIT_STEP);
                                8'd24: start_write8(8'h83, 8'h00, POST_ADV_INIT_STEP);
                                8'd25: start_read8 (8'h83, POST_WAIT_NONZERO);
                                8'd26: start_write8(8'h83, 8'h01, POST_ADV_INIT_STEP);
                                8'd27: start_read8 (8'h92, POST_SAVE_SPAD);
                                8'd28: start_write8(8'h81, 8'h00, POST_ADV_INIT_STEP);
                                8'd29: start_write8(8'hFF, 8'h06, POST_ADV_INIT_STEP);
                                8'd30: start_read8 (8'h83, POST_SAVE_TMP);
                                8'd31: start_write8(8'h83, tmp_byte & 8'hFB, POST_ADV_INIT_STEP);
                                8'd32: start_write8(8'hFF, 8'h01, POST_ADV_INIT_STEP);
                                8'd33: start_write8(8'h00, 8'h01, POST_ADV_INIT_STEP);
                                8'd34: start_write8(8'hFF, 8'h00, POST_ADV_INIT_STEP);
                                8'd35: start_write8(8'h80, 8'h00, POST_ADV_INIT_STEP);

                                // Read RefGoodSpadMap and configure reference SPADs
                                8'd36: start_read6(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, POST_COPY_REF_MAP);
                                8'd37: start_write8(8'hFF, 8'h01, POST_ADV_INIT_STEP);
                                8'd38: start_write8(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 8'h00, POST_ADV_INIT_STEP);
                                8'd39: start_write8(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 8'h2C, POST_ADV_INIT_STEP);
                                8'd40: start_write8(8'hFF, 8'h00, POST_ADV_INIT_STEP);
                                8'd41: start_write8(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 8'hB4, POST_ADV_INIT_STEP);
                                8'd42: begin
                                    h_state <= H_SPAD_PREP;
                                end
                                8'd43: start_write6(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, POST_ADV_INIT_STEP);

                                // Default tuning settings ROM
                                8'd44: begin
                                    tuning_index <= 8'd0;
                                    h_state <= H_TUNING_LOOP;
                                end

                                // GPIO interrupt: new sample ready, active low
                                8'd45: start_write8(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 8'h04, POST_ADV_INIT_STEP);
                                8'd46: start_read8 (REG_GPIO_HV_MUX_ACTIVE_HIGH, POST_SAVE_TMP);
                                8'd47: start_write8(REG_GPIO_HV_MUX_ACTIVE_HIGH, tmp_byte & 8'hEF, POST_ADV_INIT_STEP);
                                8'd48: start_write8(REG_SYSTEM_INTERRUPT_CLEAR, 8'h01, POST_ADV_INIT_STEP);

                                // Disable MSRC and TCC by default and re-apply default timing budget.
                                // 0x0290 is the encoded final-range timeout produced by the C code path.
                                8'd49: start_write8 (REG_SYSTEM_SEQUENCE_CONFIG, 8'hE8, POST_ADV_INIT_STEP);
                                8'd50: start_write16(REG_FINAL_RANGE_TIMEOUT_MACROP_HI, 16'h0290, POST_ADV_INIT_STEP);

                                // Reference calibration: VHV
                                8'd51: start_write8(REG_SYSTEM_SEQUENCE_CONFIG, 8'h01, POST_ADV_INIT_STEP);
                                8'd52: start_write8(REG_SYSRANGE_START, 8'h41, POST_ADV_INIT_STEP);
                                8'd53: start_read8 (REG_RESULT_INTERRUPT_STATUS, POST_WAIT_IRQ);
                                8'd54: start_write8(REG_SYSTEM_INTERRUPT_CLEAR, 8'h01, POST_ADV_INIT_STEP);
                                8'd55: start_write8(REG_SYSRANGE_START, 8'h00, POST_ADV_INIT_STEP);

                                // Reference calibration: phase
                                8'd56: start_write8(REG_SYSTEM_SEQUENCE_CONFIG, 8'h02, POST_ADV_INIT_STEP);
                                8'd57: start_write8(REG_SYSRANGE_START, 8'h01, POST_ADV_INIT_STEP);
                                8'd58: start_read8 (REG_RESULT_INTERRUPT_STATUS, POST_WAIT_IRQ);
                                8'd59: start_write8(REG_SYSTEM_INTERRUPT_CLEAR, 8'h01, POST_ADV_INIT_STEP);
                                8'd60: start_write8(REG_SYSRANGE_START, 8'h00, POST_ADV_INIT_STEP);
                                8'd61: start_write8(REG_SYSTEM_SEQUENCE_CONFIG, 8'hE8, POST_ADV_INIT_STEP);

                                // Start continuous back-to-back ranging
                                8'd62: start_write8(8'h80, 8'h01, POST_ADV_INIT_STEP);
                                8'd63: start_write8(8'hFF, 8'h01, POST_ADV_INIT_STEP);
                                8'd64: start_write8(8'h00, 8'h00, POST_ADV_INIT_STEP);
                                8'd65: start_write8(8'h91, stop_variable, POST_ADV_INIT_STEP);
                                8'd66: start_write8(8'h00, 8'h01, POST_ADV_INIT_STEP);
                                8'd67: start_write8(8'hFF, 8'h00, POST_ADV_INIT_STEP);
                                8'd68: start_write8(8'h80, 8'h00, POST_ADV_INIT_STEP);
                                8'd69: start_write8(REG_SYSRANGE_START, 8'h02, POST_ADV_INIT_STEP);

                                default: begin
                                    h_state <= H_POLL_STATUS;
                                end
                            endcase
                        end
                    end

                    H_WAIT_OP: begin
                        if (tr_done) begin
                            if (tr_error) begin
                                recover_from_error();
                            end else begin
                                case (post_action)
                                    POST_ADV_INIT_STEP: begin
                                        init_step <= init_step + 8'd1;
                                        h_state <= H_INIT_STEP;
                                    end

                                    POST_SAVE_TMP: begin
                                        tmp_byte <= read_buf[0];
                                        init_step <= init_step + 8'd1;
                                        h_state <= H_INIT_STEP;
                                    end

                                    POST_SAVE_STOPVAR: begin
                                        stop_variable <= read_buf[0];
                                        init_step <= init_step + 8'd1;
                                        h_state <= H_INIT_STEP;
                                    end

                                    POST_WAIT_NONZERO: begin
                                        if (read_buf[0] != 8'h00) begin
                                            init_step <= init_step + 8'd1;
                                        end
                                        h_state <= H_INIT_STEP;
                                    end

                                    POST_SAVE_SPAD: begin
                                        spad_count <= read_buf[0][6:0];
                                        spad_type_is_aperture <= read_buf[0][7];
                                        init_step <= init_step + 8'd1;
                                        h_state <= H_INIT_STEP;
                                    end

                                    POST_COPY_REF_MAP: begin
                                        ref_spad_map[0] <= read_buf[0];
                                        ref_spad_map[1] <= read_buf[1];
                                        ref_spad_map[2] <= read_buf[2];
                                        ref_spad_map[3] <= read_buf[3];
                                        ref_spad_map[4] <= read_buf[4];
                                        ref_spad_map[5] <= read_buf[5];
                                        init_step <= init_step + 8'd1;
                                        h_state <= H_INIT_STEP;
                                    end

                                    POST_WAIT_IRQ: begin
                                        if ((read_buf[0] & 8'h07) != 8'h00) begin
                                            init_step <= init_step + 8'd1;
                                        end
                                        h_state <= H_INIT_STEP;
                                    end

                                    POST_TUNING_ADV: begin
                                        tuning_index <= tuning_index + 8'd1;
                                        h_state <= H_TUNING_LOOP;
                                    end

                                    POST_POLL_STATUS: begin
                                        if ((read_buf[0] & 8'h07) != 8'h00) begin
                                            h_state <= H_POLL_RANGE;
                                        end else begin
                                            h_state <= H_POLL_STATUS;
                                        end
                                    end

                                    POST_POLL_RANGE: begin
                                        o_distance <= {read_buf[0], read_buf[1]};
                                        o_distance_valid <= 1'b1;
                                        h_state <= H_POLL_CLEAR;
                                    end

                                    POST_POLL_CLEAR: begin
                                        h_state <= H_POLL_STATUS;
                                    end

                                    default: begin
                                        init_step <= init_step + 8'd1;
                                        h_state <= H_INIT_STEP;
                                    end
                                endcase
                            end
                        end
                    end

                    H_SPAD_PREP: begin
                        first_spad_to_enable <= spad_type_is_aperture ? 6'd12 : 6'd0;
                        spad_index <= 6'd0;
                        spads_enabled <= 7'd0;
                        h_state <= H_SPAD_LOOP;
                    end

                    H_SPAD_LOOP: begin
                        if (spad_index < 6'd48) begin
                            if ((spad_index < first_spad_to_enable) ||
                                (spads_enabled == spad_count)) begin
                                case (spad_index[5:3])
                                    3'd0: ref_spad_map[0] <= ref_spad_map[0] & ~(8'h01 << spad_index[2:0]);
                                    3'd1: ref_spad_map[1] <= ref_spad_map[1] & ~(8'h01 << spad_index[2:0]);
                                    3'd2: ref_spad_map[2] <= ref_spad_map[2] & ~(8'h01 << spad_index[2:0]);
                                    3'd3: ref_spad_map[3] <= ref_spad_map[3] & ~(8'h01 << spad_index[2:0]);
                                    3'd4: ref_spad_map[4] <= ref_spad_map[4] & ~(8'h01 << spad_index[2:0]);
                                    default: ref_spad_map[5] <= ref_spad_map[5] & ~(8'h01 << spad_index[2:0]);
                                endcase
                            end else begin
                                case (spad_index[5:3])
                                    3'd0: if (ref_spad_map[0][spad_index[2:0]]) spads_enabled <= spads_enabled + 7'd1;
                                    3'd1: if (ref_spad_map[1][spad_index[2:0]]) spads_enabled <= spads_enabled + 7'd1;
                                    3'd2: if (ref_spad_map[2][spad_index[2:0]]) spads_enabled <= spads_enabled + 7'd1;
                                    3'd3: if (ref_spad_map[3][spad_index[2:0]]) spads_enabled <= spads_enabled + 7'd1;
                                    3'd4: if (ref_spad_map[4][spad_index[2:0]]) spads_enabled <= spads_enabled + 7'd1;
                                    default: if (ref_spad_map[5][spad_index[2:0]]) spads_enabled <= spads_enabled + 7'd1;
                                endcase
                            end
                            spad_index <= spad_index + 6'd1;
                        end else begin
                            init_step <= 8'd43;
                            h_state <= H_INIT_STEP;
                        end
                    end

                    H_TUNING_LOOP: begin
                        if (!tr_busy) begin
                            if (tuning_index < TUNING_LEN) begin
                                start_write8(tuning_reg(tuning_index), tuning_val(tuning_index), POST_TUNING_ADV);
                            end else begin
                                init_step <= 8'd45;
                                h_state <= H_INIT_STEP;
                            end
                        end
                    end

                    H_POLL_STATUS: begin
                        if (!tr_busy) begin
                            start_read8(REG_RESULT_INTERRUPT_STATUS, POST_POLL_STATUS);
                        end
                    end

                    H_POLL_RANGE: begin
                        if (!tr_busy) begin
                            start_read16(REG_RESULT_RANGE_MM, POST_POLL_RANGE);
                        end
                    end

                    H_POLL_CLEAR: begin
                        if (!tr_busy) begin
                            start_write8(REG_SYSTEM_INTERRUPT_CLEAR, 8'h01, POST_POLL_CLEAR);
                        end
                    end

                    default: begin
                        recover_from_error();
                    end
                endcase
            end
        end
    end

endmodule

