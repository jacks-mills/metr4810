cmake_minimum_required(VERSION 3.22)

# UART terminal test for the PCB (STM32F401VCT6). Standalone: no CubeMX, no HAL.
# Build with the presets, like robot2:
#     cmake --preset Debug && cmake --build --preset Debug

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME uart-test)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME}
    startup_stm32f401xc.s
    Src/system_stm32f4xx.c
    Src/main.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE cmsis)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE STM32F401xC)
set_source_files_properties(Src/main.c PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
