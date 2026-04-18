create_clock -period "50.0 MHz" [get_ports CLK]

derive_clock_uncertainty

set_false_path -from * -to [get_ports {SEGS[*]}]
set_false_path -from * -to [get_ports {DIGS[*]}]
set_false_path -from RESET -to [all_clocks]