`include "common.vh"

//NOTE only naturally aligned STs & LDs are supported. OTHERWISE UB
module lsu (
    input  wire                       clk, 

    input  wire [31:0]                i_core_addr,
    input  wire [31:0]                i_core_data,
    output reg  [31:0]                o_core_data,
    input  wire                       i_wren,
    input  wire [`LSU_SIZE_WIDTH-1:0] i_size, 

    output wire [29:0]                o_mem_addr,
    output reg  [31:0]                o_mem_data,
    input  wire [31:0]                i_mem_data,
    output wire                       o_wren,
    output reg  [3:0]                 o_mask
);

wire [1:0] in_addr_off_b_s0, in_addr_off_h_s0; 
wire [4:0] in_word_off_b_s0, in_word_off_h_s0;

reg  [4:0]  in_word_off_b_s1, in_word_off_h_s1;
wire [7:0]  from_mem_byte_s1;
wire [15:0] from_mem_half_s1;
wire [31:0] from_mem_word_s1;

reg [`LSU_SIZE_WIDTH-1:0] size_s1;

assign in_addr_off_b_s0 = i_core_addr[1:0];
assign in_addr_off_h_s0 = {i_core_addr[1], 1'b0};

assign in_word_off_b_s0 = {in_addr_off_b_s0, 3'b0};
assign in_word_off_h_s0 = {in_addr_off_h_s0, 3'b0};

assign o_wren     = i_wren;
assign o_mem_addr = i_core_addr[31:2];

always @(*) begin
    o_mask      = 4'bx;
    o_mem_data  = 32'bx;
    case (i_size)
        `LSU_SIZE_BYTE:  
        begin 
            o_mask = 4'b0001 << in_addr_off_b_s0;
            o_mem_data[in_word_off_b_s0 +: 8] = i_core_data[7:0];
        end  
        `LSU_SIZE_BYTEU: 
        begin 
            o_mask = 4'b0001 << in_addr_off_b_s0;
            o_mem_data[in_word_off_b_s0 +: 8] = i_core_data[7:0];
        end 
        `LSU_SIZE_HALF: 
        begin 
            o_mask = 4'b0011 << in_addr_off_h_s0;
            o_mem_data[in_word_off_h_s0 +: 16] = i_core_data[15:0];
        end
        `LSU_SIZE_HALFU: 
        begin 
            o_mask = 4'b0011 << in_addr_off_h_s0;
            o_mem_data[in_word_off_h_s0 +: 16] = i_core_data[15:0];
        end
        `LSU_SIZE_WORD: 
        begin 
            o_mask = 4'b1111;
            o_mem_data = i_core_data;
        end
        default: 
        begin
            o_mask      = 4'bx;
            o_mem_data  = 32'bx;
        end
    endcase
end

always @(posedge clk) begin
    size_s1          <= i_size;
    in_word_off_b_s1 <= in_word_off_b_s0; 
    in_word_off_h_s1 <= in_word_off_h_s0; 
end

assign from_mem_byte_s1 = i_mem_data[in_word_off_b_s1 +: 8];
assign from_mem_half_s1 = i_mem_data[in_word_off_h_s1 +: 16];
assign from_mem_word_s1 = i_mem_data;

always @(*) begin
    o_core_data = 32'bx;
    case (size_s1)
        `LSU_SIZE_BYTE:  o_core_data = { {24{from_mem_byte_s1[7]}}, from_mem_byte_s1[7:0] };        
        `LSU_SIZE_BYTEU: o_core_data = { {24{1'b0}}, from_mem_byte_s1[7:0] };
        `LSU_SIZE_HALF:  o_core_data = { {16{from_mem_half_s1[15]}}, from_mem_half_s1[15:0] };
        `LSU_SIZE_HALFU: o_core_data = { {16{1'b0}}, from_mem_half_s1[15:0] };
        `LSU_SIZE_WORD:  o_core_data = from_mem_word_s1;
        default:         o_core_data = 32'bx;
    endcase
end

endmodule
