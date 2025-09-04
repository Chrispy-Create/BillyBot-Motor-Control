cmake_minimum_required(VERSION 3.12)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Tell the SDK which board you’re targeting
set(PICO_BOARD pico_w)

project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK (pulls in the pico_cyw43_driver automatically)
pico_sdk_init()

# on SDKs missing pico_set_cyw43_country(), define directly:
add_compile_definitions(PICO_CYW43_COUNTRY=\"US\")
# This is needed to turn on the LED for the Pico W 

link_directories(${CMAKE_CURRENT_LIST_DIR}/libmicroros)

add_executable(pico_micro_ros_example
  main.c
  motor_driver.c
  pico_uart_transport.c
)

target_include_directories(pico_micro_ros_example PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/libmicroros/include
  ${CMAKE_CURRENT_LIST_DIR}
  # the Pico SDK’s CYW43 headers are already on your include path once you link below
)

target_link_libraries(pico_micro_ros_example
  pico_stdlib
  pico_multicore
  pico_cyw43_arch        # Pico-W’s Wi-Fi/LED driver
  pico_cyw43_arch_none   # LED‐only driver, no LWIP dependency
  microros
  hardware_gpio
  hardware_pwm
  hardware_uart
  hardware_irq
)

# Optimize
add_compile_options(-ffunction-sections -fdata-sections)

# USB stdout/stderr on, UART stdio off
pico_enable_stdio_usb(pico_micro_ros_example 1)
pico_enable_stdio_uart(pico_micro_ros_example 0)

add_compile_definitions(
  PICO_UART_ENABLE_CRLF_SUPPORT=0
  PICO_STDIO_ENABLE_CRLF_SUPPORT=0
  PICO_STDIO_DEFAULT_CRLF=0
)

# Build .uf2, .hex, etc.
pico_add_extra_outputs(pico_micro_ros_example)
