# Timing constraint for i2c_master (lab 3, variant 14)
# System clock: 50 MHz -> period 20.000 ns
create_clock -period 20.000 -name clk [get_ports clk]
