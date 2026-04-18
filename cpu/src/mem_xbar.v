module mem_xbar #(
    parameter [29:0] DMEM_START = 30'h0,
    parameter [29:0] DMEM_LIMIT = 30'h0,
    parameter [29:0] MMIO_START = 30'h0,
    parameter [29:0] MMIO_LIMIT = 30'h0
) (
    input wire         clk,

    input  wire [29:0] i_addr,
    input  wire [31:0] i_data,
    input  wire        i_wren,
    input  wire [3:0]  i_mask,
    output wire [31:0] o_data,

    output wire [29:0] o_dmem_addr,
    output wire [31:0] o_dmem_data,
    output wire        o_dmem_wren,
    output wire [3:0]  o_dmem_mask,
    input  wire [31:0] i_dmem_data,

    output wire [29:0] o_mmio_addr,
    output wire [31:0] o_mmio_data,
    output wire        o_mmio_wren,
    output wire [3:0]  o_mmio_mask,
    input  wire [31:0] i_mmio_data
);

wire dmem, mmio;
reg dmem_q, mmio_q;

/* verilator lint_off UNSIGNED */
assign dmem = (DMEM_START <= i_addr && i_addr <= DMEM_LIMIT);
assign mmio = (MMIO_START <= i_addr && i_addr <= MMIO_LIMIT);
/* verilator lint_on UNSIGNED */

assign o_dmem_addr = dmem ? i_addr : 30'bx;
assign o_dmem_data = dmem ? i_data : 32'bx;
assign o_dmem_mask = dmem ? i_mask :  4'bx;
assign o_dmem_wren = dmem ? i_wren :  1'b0;

assign o_mmio_addr = mmio ? i_addr : 30'bx;
assign o_mmio_data = mmio ? i_data : 32'bx;
assign o_mmio_mask = mmio ? i_mask :  4'bx;
assign o_mmio_wren = mmio ? i_wren :  1'b0;

assign o_data = (dmem_q ? i_dmem_data :
                (mmio_q ? i_mmio_data :
                                      32'bx));

always @(posedge clk) begin
    dmem_q <= dmem;
    mmio_q <= mmio;
end

endmodule
