add_file src/gowin_rpll/gowin_rpll.v
add_file src/psram_controller.v
add_file src/psram_picomem_test_top.v
add_file src/psram_picomem_v2.v
add_file src/uart_tx.v
add_file src/tang9k.cst
add_file src/tang9k.sdc
set_device -name GW1NR-9C GW1NR-LV9QN88PC6/I5
set_option -top_module memory_test
set_option -bit_format bin
set_option -bit_security 0
run all
