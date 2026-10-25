cmake_minimum_required(VERSION 3.22)

# Servo test for the Blackpill: a command prompt on the USB-C port for the
# ST3215 bus servo on PA2. No CubeMX or FreeRTOS: the HAL and ST's USB
# device library (in lib/, copied from usb-uart-bridge) do the USB, and
# USART2 is set up at register level.
#     cmake --preset Debug && cmake --build --preset Debug

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME servo-test)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
enable_language(C ASM)

set(APP_SOURCES
    Src/main.c
    Src/servo.c
    Src/servo_uart.c
    Src/stm32f4xx_it.c
    Src/usbd_cdc_if.c
)

add_executable(${CMAKE_PROJECT_NAME}
    startup_stm32f411xe.s
    Src/system_stm32f4xx.c
    ${APP_SOURCES}

    # USB device glue
    Src/usb_device.c
    Src/usbd_conf.c
    Src/usbd_desc.c

    # ST USB device library: core and CDC class
    lib/usb/Src/usbd_core.c
    lib/usb/Src/usbd_ctlreq.c
    lib/usb/Src/usbd_ioreq.c
    lib/usb/Src/usbd_cdc.c

    # STM32F4 HAL: only what the clock setup and USB need
    lib/hal/Src/stm32f4xx_hal.c
    lib/hal/Src/stm32f4xx_hal_cortex.c
    lib/hal/Src/stm32f4xx_hal_rcc.c
    lib/hal/Src/stm32f4xx_hal_rcc_ex.c
    lib/hal/Src/stm32f4xx_hal_gpio.c
    lib/hal/Src/stm32f4xx_hal_pwr.c
    lib/hal/Src/stm32f4xx_hal_pwr_ex.c
    lib/hal/Src/stm32f4xx_hal_flash.c
    lib/hal/Src/stm32f4xx_hal_flash_ex.c
    lib/hal/Src/stm32f4xx_hal_pcd.c
    lib/hal/Src/stm32f4xx_hal_pcd_ex.c
    lib/hal/Src/stm32f4xx_ll_usb.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Inc
    lib/cmsis
    lib/hal/Inc
    lib/usb/Inc
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32F411xE
    $<$<CONFIG:Debug>:DEBUG>
)

target_link_libraries(${CMAKE_PROJECT_NAME} ${TOOLCHAIN_LINK_LIBRARIES})

# Extra warnings for our own code only.
set_source_files_properties(${APP_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
