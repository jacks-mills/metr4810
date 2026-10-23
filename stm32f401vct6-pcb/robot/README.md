# robot on the PCB (STM32F401VCT6)

FreeRTOS firmware for the boring robot (METR4810 Group 7), for the team PCB.
The agreed design is in the metr4810 project (`claude/robot-design.md`).

This is a self-contained project. It started as a copy of the Blackpill
version (`../../stm32f411ceu6-blackpill/robot`), and apart from these files
the two `App/` folders are identical:

| | Blackpill | PCB (here) |
|---|---|---|
| Clock | 25 MHz crystal, 96 MHz | no crystal: HSI 16 MHz, PLL to 84 MHz |
| Terminal | USB CDC, `term_io_usb.c` | USART1 115200 8N1 on J7, `term_io_uart.c` |
| LEDs | PC13, `status_led_blackpill.c` | red, orange, green, start, `status_led_pcb.c` |
| `App/app.cmake`, `App/Inc/config.h` | its own | its own |

So a fix to the shared logic needs copying across by hand. To see what
differs: `diff -r App ../../stm32f411ceu6-blackpill/robot/App`.

## First time: generate the code

1. Open `robot.ioc` in STM32CubeMX. Check the pinout and the clock tree
   (HCLK 84 MHz, APB1 42 MHz), then click **GENERATE CODE**.
2. CubeMX keeps the USER CODE sections already in `Core/Src/main.c`,
   `freertos.c` and `stm32f4xx_it.c`: check that `main.c` still calls
   `app_init_early()` in USER CODE 2 and `app_init_rtos()` in RTOS_QUEUES.
   It writes the root `CMakeLists.txt` only when there isn't one, so ours
   should survive too. If it doesn't, put `include(App/app.cmake)` back after
   `add_subdirectory(cmake/stm32cubemx)`.

Leave **PA2, PA9, PA10, USART1, USART2 and DMA** unassigned in CubeMX. The
servo link (USART2 on PA2) and the terminal (USART1 on PA9/PA10, with DMA2)
are set up in code. Don't turn on the USART1 interrupt in CubeMX either: our
`USART1_IRQHandler` is in `App/Src/term_io_uart.c`.

After any later change to `robot.ioc`, generate again; code in USER CODE
sections and in `App/` is kept.

## Build and flash

```sh
cmake --preset Debug                  # once, or after deleting build/
cmake --build --preset Debug
STM32_Programmer_CLI -c port=SWD -w build/Debug/robot.elf -v -rst
# or: openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
#             -c "program build/Debug/robot.elf verify reset exit"
picocom -b 115200 /dev/ttyUSB0        # quit with Ctrl-A Ctrl-X
```

## Trying it without hardware

Configure with `cmake --preset Debug -DROBOT_SIMULATED=ON` (`OFF` for the
real thing; the setting stays in `build/Debug` until changed, and the
configure output shows `robot (PCB), simulated: ...`). The arm, drill and
servo are then simulated: the arm takes 4 s to travel at 100 %, the limit
switches "close" at the ends, and a virtual ST3215 answers on the servo link.
`#define HW_SIMULATED 1` in `App/Inc/config.h` does the same.

## Terminal (J7)

Use a **3.3 V** USB-serial adapter: adapter RX to PA9 (our TX), adapter TX to
PA10 (our RX), and GND. 115200 baud, 8N1, no flow control.

A serial line can't tell when a terminal opens, so the screen is drawn once
at start-up. If you open the terminal later, press **Ctrl-L** to redraw (that
also reads the window size). Closing the terminal doesn't stop manual moves
on this board, as it does over USB on the Blackpill. Esc still stops
everything.

Received bytes go through DMA, so keys aren't lost while the servo link has
interrupts masked.

Without an adapter, the start button and the LEDs are enough to run the
sequence.

## Pins

| Function | Pin | Schematic net | CubeMX label | Notes |
|---|---|---|---|---|
| Arm drive PWM | PA7 (TIM3 CH2) | RIGHT_PWM | DRIVE_PWM | DRI0002 E, 20 kHz |
| Arm drive direction | PE1 | RIGHT_DIR | DRIVE_DIR | DRI0002 M |
| Drill PWM | PB0 (TIM3 CH3) | DRILL_PWM | DRILL_PWM | FIT0441, inverted (polarity Low) |
| Drill direction | PE2 | DRILL_DIR | DRILL_DIR | high = forward |
| EXT_MAX | PC2 (EXTI2) | EXT_MAX | EXT_MAX | switch to 3.3 V, pull-down, both edges |
| EXT_MIN | PC3 (EXTI3) | EXT_MIN | EXT_MIN | switch to 3.3 V, pull-down, both edges |
| Start button | PC4 (EXTI4) | START_BTN | START_BUTTON | to 3.3 V, pull-down, rising edge |
| Servo | PA2 (USART2, single wire) | SERVO_SIG | (set in code) | see below |
| Terminal | PA9 TX, PA10 RX (USART1) | J7 | (set in code) | RX has the internal pull-up |
| LEDs | PD0, PD1, PD2, PD3 | red, orange, green, start | LED_RED, LED_ORANGE, LED_GREEN, LED_START | lit when high (see below) |
| SWD | PA13, PA14 | | | |

Not used: LEFT_PWM (PA6), LEFT_DIR (PE0), the FG inputs (PA0, PA1),
LEFT_MIN/RIGHT_MIN (PC0, PC1), I2C1, SPI2.

- **Servo:** the ST3215's signal wire goes to PA2, with a 1–2.2 kΩ pull-up to
  3.3 V. The servo needs its own 12 V supply, with GND shared with the board.
- **Limit switches** wired to GND instead: set `LIMIT_ACTIVE_HIGH` to 0 in
  `config.h` and change their pull to pull-up in CubeMX.

## LEDs

| LED | Meaning |
|---|---|
| green | on: sequence running. Short blink every 2 s: idle |
| orange | on: paused, manual control |
| red | on: stopped. Flashing: fault |
| start | on: the start button will start or continue the sequence |
| all four flashing fast | `fault_halt()` (stack overflow, heap full, hard fault): the debugger shows `faultReason` |

## Before the first real run

1. **LED polarity:** `status_led_pcb.c` assumes each LED lights when its pin
   is high. If they're the other way round, change `LED_ON` there.
2. **Switch wiring:** check the limit switches and start button against the
   schematic (see Pins).
3. **Motors during reset:** while the F401 is held in reset or being flashed,
   PA7 and PB0 float. Check that the arm (DRI0002 E) and the drill don't move
   then. The FIT0441 runs flat out on a low PWM input. If either moves, add
   a pull-down on RIGHT_PWM or a pull-up (e.g. 10 kΩ to 3.3 V) on DRILL_PWM.
4. **Directions:** with the arm clear, `drive extend` should move the arm out.
   If it goes in, flip `DRIVE_DIR_EXTEND` in `config.h`. Likewise
   `SERVO_REVERSED` if `rotate right` turns the plate left.
5. **Travel time:** time a full extend at your drive speed and set
   `DRIVE_TIMEOUT_MS` to about 1.5 times that (it's 30 s for now).
6. **Servo centre and limits** (saved in the servo, so once per servo; skip
   if it was already done on the Blackpill):
   ```
   debug on
   servo free              (torque off)
   ...turn the plate to straight ahead by hand...
   servo centre            (this is now 0.0 deg)
   servo hold
   servo limits -2.5 2.5
   debug off
   ```
   Until the servo is centred, rotation is refused if the plate reads more
   than 1 deg outside the +/-2.5 deg range.

The HSI is within about 1 % at room temperature, which is fine for 115200
baud and the servo's 1 Mbaud.

## Using it

Type `help` for the commands. Esc stops everything, Up/Down recall commands,
Ctrl-L redraws. `start` (or the start button) runs or continues the
sequence: retract to EXT_MIN, drill on, extend to EXT_MAX, retract to
EXT_MIN, drill off. `pause` gives you manual control; `start` then carries on
from the same step.

While the sequence runs only `pause`, `stop` and `debug` are accepted. With
safety on (the default), the base only rotates with the arm stopped at
EXT_MIN, and the arm won't extend while the base is rotating.

`debug on` adds the servo's voltage, temperature, load and limits, the free
heap, and each task's unused stack in words (the unit CubeMX uses).

## Layout

| Path | What |
|---|---|
| `App/` | All our code. CubeMX never touches it. New `.c` files go in `App/app.cmake` |
| `App/Inc/config.h` | Pins' polarities, timeouts, speeds, servo settings, `HW_SIMULATED` |
| `App/Src/control.c` | Modes, command rules, safety interlock, the sequence |
| `App/Src/drive.c`, `drive_hw.c` | Arm drive: run to a limit switch, soft start, timeout; pins |
| `App/Src/drill.c` | Drill PWM (no task: control calls it) |
| `App/Src/rotate.c`, `servo.c`, `servo_uart.c` | Base rotation: goals and polling; ST3215 protocol; USART2 link |
| `App/Src/sim.c`, `servo_sim.c` | Simulated arm, drill and servo (`HW_SIMULATED`) |
| `App/Src/terminal.c`, `ui.c`, `console.c` | Terminal: keys, screen, command language |
| `App/Src/term_io_uart.c` | Terminal transport: USART1 with DMA |
| `App/Src/status_led_pcb.c` | The four status LEDs |
| `App/Src/task_*.c` | The four task loops (control, drive, rotate, terminal) |
| `App/Inc/robot_state.h` | Shared state; each field has one writer |
| `robot.ioc` | CubeMX configuration |
| `Core/`, `Drivers/`, `Middlewares/`, `cmake/` | Generated by CubeMX (our code only in USER CODE sections) |
