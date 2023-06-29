set(CMAKE_CXX_STANDARD 17)

add_executable(mcp9808_basic
    main.c
)
include(./library/import.cmake)

add_library( AHRS 
	MahonyAHRS.c
)
target_link_libraries(mcp9808_basic
    PUBLIC
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap1
    VL53L1X_pico_api
    pico_stdlib
    hardware_pwm
    i2c_dma
    common
    PUBLIC
    AHRS
)

pico_enable_stdio_usb(mcp9808_basic 1)
pico_enable_stdio_uart(mcp9808_basic 0)

pico_add_extra_outputs(mcp9808_basic)

