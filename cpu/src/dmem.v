`include "config.vh"

module dmem(
    input  wire clk,

    input  wire                        i_we,
    input  wire [3:0]                  i_mask,
    input  wire [`DMEM_ADDR_WIDTH-1:0] i_addr,
    input  wire [31:0]                 i_data,
    output wire [31:0]                 o_data
);

`ifdef __ICARUS__
/* verilator lint_off LATCH */

reg  [31:0] mem [0:(2**`DMEM_ADDR_WIDTH)-1];
reg   [`DMEM_ADDR_WIDTH-1:0] addr;
reg   [3:0] mask;
reg         we;
reg [31:0] data;

always @(posedge clk) begin
    addr <= i_addr;
    we   <= i_we;
    mask <= i_mask;
    data <= i_data;
end

always @(addr, we, mask, data) begin
    if (we) begin
        if (mask[0])
            mem[addr][7:0]   = data[7:0];
        if (mask[1])
            mem[addr][15:8]  = data[15:8];
        if (mask[2])
            mem[addr][23:16] = data[23:16];
        if (mask[3])
            mem[addr][31:24] = data[31:24];
    end
end

assign o_data = mem[addr];

/* verilator lint_on LATCH */
`else
 
ram1rw32x1024 ram(
    .address    (i_addr ),
    .byteena    (i_mask ),
    .clock      (clk    ),
    .data       (i_data ),
    .wren       (i_we   ),
    .q          (o_data )
);

`endif

endmodule


