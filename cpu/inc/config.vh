`ifndef CONFIG_VH
`define CONFIG_VH

// SYSTEM CONFIG
`ifdef __ICARUS__
`define IMEM_FILE_TXT   "out_prog/display.txt"
`else // Quartus
`define IMEM_FILE_MIF   "../out_prog/display.mif"
`endif

// MUST BE COMPLIANT WITH ACTUAL VALUES IN imem.v & dmem.v
`define IMEM_ADDR_WIDTH 12
`define DMEM_ADDR_WIDTH 10

`define XBAR_MMIO_START 30'h0000
`define XBAR_MMIO_LIMIT 30'h001F
`define XBAR_DMEM_START 30'h0020
`define XBAR_DMEM_LIMIT 30'h0420 //DMEM_ADDR_WIDTH=10 => 1024 words of DMEM
`define DISP_7SEG_ADDR  30'h0008 //0x20 if full 32-bit address
`define PC_INIT_VAL     32'h10000

`endif
