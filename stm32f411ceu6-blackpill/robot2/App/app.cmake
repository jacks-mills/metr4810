# Application code: everything in App/ (CubeMX never touches this folder).
#
# Pulled into the build by one line in the root CMakeLists.txt, after
# add_subdirectory(cmake/stm32cubemx):
#
#     include(App/app.cmake)
#
# Add new .c files to the list below.

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/Src/app.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/command.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/console.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/exti.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/fault.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/log.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/robot_state.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/sysinfo.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/task_control.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/task_terminal.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/term.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/term_io_usb.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/terminal.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/ui.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/Inc
)

# Extra warnings for our own code only (not the HAL or FreeRTOS).
file(GLOB APP_SOURCES ${CMAKE_CURRENT_LIST_DIR}/Src/*.c)
set_source_files_properties(${APP_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
