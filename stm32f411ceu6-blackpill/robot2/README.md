# robot2

FreeRTOS firmware for the boring robot (METR4810 Group 7), developed on the
STM32F411CEU6 Blackpill before moving to the STM32F401VCT6 PCB. The agreed
design is in the metr4810 project (`claude/robot2-design.md`).

**Stage 1: terminal UI.** The arm and base plate are simulated by a stand-in
control task; nothing but the LED is driven.

## First build

1. Open `robot2.ioc` in STM32CubeMX and click **GENERATE CODE**. That adds
   `Drivers/`, `Middlewares/`, the rest of `Core/` and `USB_DEVICE/`, and the
   CMake files. `main.c`, `freertos.c` and `usbd_cdc_if.c` are already here
   with their USER CODE sections filled in; CubeMX keeps those.
2. In the root `CMakeLists.txt` (CubeMX creates it once and then leaves it
   alone), add this line after `add_subdirectory(cmake/stm32cubemx)`:

   ```cmake
   include(App/app.cmake)
   ```

3. Build:

   ```sh
   cmake --preset Debug
   cmake --build --preset Debug
   ```

4. Flash:

   ```sh
   STM32_Programmer_CLI -c port=SWD -w build/Debug/robot2.elf -v -rst
   ```

5. Plug in the Blackpill's USB-C and open the terminal (any baud rate):

   ```sh
   picocom /dev/ttyACM0        # quit with Ctrl-A Ctrl-X
   ```

In vim: `:set makeprg=cmake\ --build\ --preset\ Debug`, then `:make`. For
clangd, `ln -s build/Debug/compile_commands.json .`

## Using the terminal

Type `help` for the commands. Esc stops everything immediately, Up/Down recall
earlier commands, Ctrl-L redraws (do that after resizing the window).
`debug on` adds servo, heap and stack figures to the panel.

## Layout

| Path | What |
|---|---|
| `App/` | All our code. CubeMX never touches it. New `.c` files go in `App/app.cmake` |
| `App/Src/task_control.c` | Control task (stage 1: stand-in with simulation) |
| `App/Src/task_terminal.c`, `terminal.c`, `ui.c` | Terminal task: input, screen layout, drawing |
| `App/Src/console.c` | Command language (text to `struct Command`) |
| `App/Src/command.c`, `log.c` | Queues: commands to control, log lines to the terminal |
| `App/Src/term_io_usb.c` | USB CDC transport (swap for USART1 on the PCB) |
| `App/Inc/robot_state.h` | Shared state; each field has one writer |
| `App/Inc/config.h` | Rotation limit, default speeds |
| `robot2.ioc` | CubeMX configuration |
