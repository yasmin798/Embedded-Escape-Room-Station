cmake_minimum_required(VERSION 3.12)

# Set SDK path
set(PICO_SDK_PATH "C:/VSARM/sdk/pico/pico-sdk")

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(station6 C CXX ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME}
    main.c
    servo.c
    buzzer.c
    buttons.c
    keypad.c
    lcd_i2c.c
    ultrasonic_sensor.c
    state_machine.c
    pulley.c
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_i2c
    hardware_gpio
    hardware_timer
    hardware_pwm
    hardware_uart
    m
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME})
