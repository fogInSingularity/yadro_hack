`ifndef CONFIG_VH
`define CONFIG_VH

// SYSTEM CONFIG
`ifdef __ICARUS__
`define IMEM_FILE_TXT   "out_prog/fibc.txt"
`else // Quartus
`define IMEM_FILE_MIF   "../out_prog/fib_fpga.mif"
`endif

// MUST BE COMPLIANT WITH ACTUAL VALUES IN imem.v & dmem.v
`define IMEM_ADDR_WIDTH 8
`define DMEM_ADDR_WIDTH 8

`define XBAR_MMIO_START 30'h0000
`define XBAR_MMIO_LIMIT 30'h03FF
`define XBAR_DMEM_START 30'h0400
`define XBAR_DMEM_LIMIT 30'h04FF //DMEM_ADDR_WIDTH=8 => 256 words of DMEM
`define DISP_7SEG_ADDR  30'h0008 //0x20 if full 32-bit address
`define PC_INIT_VAL     32'h1000

`endif
