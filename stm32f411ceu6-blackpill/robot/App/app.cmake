# Application code: everything in App/ (CubeMX never touches this folder).
#
# Pulled into the build by this line in the root CMakeLists.txt, after
# add_subdirectory(cmake/stm32cubemx):
#
#     include(App/app.cmake)
#
# Add new .c files to the list below.

# Simulated arm, drill and servo, for a bare board:
#     cmake --preset Debug -DROBOT_SIMULATED=ON     (OFF to go back)
# Kept in the build folder until changed.
option(ROBOT_SIMULATED "Simulate the arm, drill and servo (no hardware needed)" OFF)
if(ROBOT_SIMULATED)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE HW_SIMULATED=1)
endif()
message(STATUS "robot (Blackpill), simulated: ${ROBOT_SIMULATED}")

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # start-up, shared state, queues
    ${CMAKE_CURRENT_LIST_DIR}/Src/app.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/command.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/log.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/robot_state.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/exti.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/fault.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/sysinfo.c

    # control task
    ${CMAKE_CURRENT_LIST_DIR}/Src/task_control.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/control.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/drill.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/status_led_blackpill.c

    # drive task
    ${CMAKE_CURRENT_LIST_DIR}/Src/task_drive.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/drive.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/drive_hw.c

    # rotate task
    ${CMAKE_CURRENT_LIST_DIR}/Src/task_rotate.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/rotate.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/servo.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/servo_uart.c

    # simulated hardware (compiled only when HW_SIMULATED is 1)
    ${CMAKE_CURRENT_LIST_DIR}/Src/sim.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/servo_sim.c

    # terminal task
    ${CMAKE_CURRENT_LIST_DIR}/Src/task_terminal.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/terminal.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/console.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/ui.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/term.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/term_io_usb.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/Inc
)

# Extra warnings for our own code only (not the HAL or FreeRTOS).
file(GLOB APP_SOURCES ${CMAKE_CURRENT_LIST_DIR}/Src/*.c)
set_source_files_properties(${APP_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
