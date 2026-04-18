`include "config.vh"

module imem(
    input  wire                        clk,  
    input  wire                        rst_n,

    input  wire [`IMEM_ADDR_WIDTH-1:0] i_addr,
    output wire [31:0]                 o_data
);

wire rst = ~rst_n;

`ifdef __ICARUS__

reg  [31:0] mem [0:(2**`IMEM_ADDR_WIDTH)-1];
reg   [`IMEM_ADDR_WIDTH-1:0] addr_d;

initial begin
   $readmemh(`IMEM_FILE_TXT, mem);
end

always @(posedge clk or posedge rst) begin
    if (rst)
        addr_d <= `IMEM_ADDR_WIDTH'b0;
    else
        addr_d <= i_addr;
end

assign o_data = mem[addr_d];

`else

imem1r32x4096 #(
    .INIT_FILE_MIF(`IMEM_FILE_MIF)
)
imem(
    .aclr           (rst    ),
    .address        (i_addr ),
    .clock          (clk    ),
    .q              (o_data )
);

`endif

endmodule
