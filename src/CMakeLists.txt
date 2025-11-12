add_executable(picow_ds4
	main.c
	bt_hid.c
)

pico_generate_pio_header(picow_ds4 ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

pico_enable_stdio_usb(picow_ds4 1)

target_include_directories(picow_ds4 PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(picow_ds4 PRIVATE
	hardware_gpio
	hardware_pio
	hardware_pwm
	m
	pico_cyw43_arch_none
	pico_multicore
	pico_stdlib
  pico_btstack_ble
  pico_btstack_classic
  pico_btstack_cyw43
)

pico_add_extra_outputs(picow_ds4)
