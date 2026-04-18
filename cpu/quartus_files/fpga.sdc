create_clock -period "50.0 MHz" [get_ports CLK]

derive_pll_clocks
derive_clock_uncertainty

set_false_path -from * -to [get_ports {SEGS[*]}]
set_false_path -from * -to [get_ports {DIGS[*]}]
set_false_path -from RSTN -to [all_clocks]

set_false_path -from SDA -to [all_clocks]
set_false_path -from SCL -to [all_clocks]

set_false_path -from [all_clocks] -to SDA
set_false_path -from [all_clocks] -to SCL
