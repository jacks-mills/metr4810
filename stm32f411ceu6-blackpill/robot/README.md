# robot

FreeRTOS firmware for the boring robot (METR4810 Group 7), developed on the
STM32F411CEU6 Blackpill before moving to the STM32F401VCT6 PCB. The agreed
design is in the metr4810 project (`claude/robot-design.md`).

The PCB version is a separate project with its own copy of `App/`:
`../../stm32f401vct6-pcb/robot`. Apart from the board files (terminal
transport, status LEDs), `config.h` and `app.cmake`, the two `App/` folders
are identical, so a fix to the shared logic needs copying across. To see what
differs:

```sh
diff -r App ../../stm32f401vct6-pcb/robot/App
```

## Build and flash

After any change to `robot.ioc`, open it in STM32CubeMX and click
**GENERATE CODE** (code in USER CODE sections and in `App/` is kept). Then:

```sh
cmake --preset Debug                  # once, or after deleting build/
cmake --build --preset Debug
STM32_Programmer_CLI -c port=SWD -w build/Debug/robot.elf -v -rst
picocom /dev/ttyACM0                  # Blackpill USB-C; quit with Ctrl-A Ctrl-X
```

The root `CMakeLists.txt` must contain `include(App/app.cmake)` after
`add_subdirectory(cmake/stm32cubemx)`. CubeMX creates that file once and
leaves it alone afterwards.

## Trying it without hardware

Configure with `cmake --preset Debug -DROBOT_SIMULATED=ON` (`OFF` for the
real thing; the setting stays in `build/Debug` until changed). The arm, drill
and servo are then simulated: the arm takes 4 s to travel at 100 %, the limit
switches "close" at the ends, and a virtual ST3215 answers on the servo link.

`#define HW_SIMULATED 1` in `App/Inc/config.h` does the same.

## Blackpill wiring

| Signal | Blackpill | PCB (F401) | Notes |
|---|---|---|---|
| DRIVE_PWM | PA7 (TIM3 CH2) | PA7 (RIGHT_PWM) | DRI0002 E input, 20 kHz |
| DRIVE_DIR | PB1 | PE1 (RIGHT_DIR) | DRI0002 M input |
| DRILL_PWM | PB0 (TIM3 CH3) | PB0 | FIT0441 PWM, inverted (polarity Low) |
| DRILL_DIR | PB10 | PE2 | high = forward |
| EXT_MAX | PB12 | PC2 | switch to 3.3 V, pull-down |
| EXT_MIN | PB13 | PC3 | switch to 3.3 V, pull-down |
| SERVO | PA2 (USART2, single wire) | PA2 (SERVO_SIG) | see below |
| START_BUTTON | PA3 | PC4 | to 3.3 V, pull-down |
| LED | PC13 | PD0–PD3 (red, orange, green, start) | see the PCB README |

- **Servo:** the ST3215's signal wire goes to PA2, with a 1–2.2 kΩ pull-up to
  3.3 V. The servo needs its own 12 V supply, with GND shared with the
  Blackpill. USART2 is set up in `servo_uart.c`, not CubeMX, so leave PA2
  unassigned there.
- **Limit switches** wired to GND instead: set `LIMIT_ACTIVE_HIGH` to 0 in
  `config.h` and change their pull to pull-up in CubeMX.
- **Drill PWM:** the FIT0441 runs flat out on a low PWM input. While the chip
  is in reset or being flashed the pin floats, so check what the drill does
  then. If it spins, add a pull-up (e.g. 10 kΩ to 3.3 V) on DRILL_PWM.

## Before the first real run

1. **Directions:** with the arm clear, `drive extend` should move the arm out.
   If it goes in, flip `DRIVE_DIR_EXTEND` in `config.h`. Likewise
   `SERVO_REVERSED` if `rotate right` turns the plate left.
2. **Travel time:** time a full extend at your drive speed, then set the
   timeout to about 1.5 times that from the terminal: `drive timeout 20`
   (seconds, 1 to 600, one decimal allowed). It covers both extend and
   retract, shows on the Arm row, and can be changed while the arm moves
   (counted from the start of that move), but not while the sequence runs.
   It goes back to `DRIVE_TIMEOUT_MS` in `config.h` (30 s) at every reset,
   so put your number there too once you're happy with it.
3. **Servo centre and limits** (saved in the servo, so once per servo):
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

## Using it

Type `help` for the commands. Esc stops everything, Up/Down recall commands,
Ctrl-L redraws. `start` runs (or continues) the sequence: retract to EXT_MIN,
drill on, extend to EXT_MAX, retract to EXT_MIN, drill off. `pause` gives you
manual control; `start` then carries on from the same step.

**Pulsing the arm:** `pulse extend 200` or `pulse retract 200` runs the arm
at full power (the DRI0002's E input held high, no PWM) for that many
milliseconds (1 to 10000), then stops. It's for moving the arm in steps while
the PWM speed control doesn't work with the driver: press Up then Enter to
repeat. The length is exact to the millisecond, the drive speed is ignored,
and the limit switches and Esc still stop it. A pulse in the same direction
while one is running carries on from now with no gap; reversing waits
150 ms first. Not accepted while the sequence runs.

While the sequence runs only `pause`, `stop` and `debug` are accepted. With
safety on (the default), the base only rotates with the arm stopped at
EXT_MIN, and the arm won't extend while the base is rotating.

`debug on` adds the servo's voltage, temperature, load and limits, the free
heap, and each task's unused stack in words (the unit CubeMX uses).

LED: short blink every 2 s = idle, on = auto, 1 Hz = manual, 2 Hz = stopped,
fast flicker = fault. A very fast steady flash with nothing on the terminal is
`fault_halt()` (stack overflow, heap full or hard fault): the debugger shows
the reason in `faultReason`.

## Layout

| Path | What |
|---|---|
| `App/` | All our code. CubeMX never touches it. New `.c` files go in `App/app.cmake` |
| `App/Inc/config.h` | Pins' polarities, timeouts, speeds, servo settings, `HW_SIMULATED` |
| `App/Src/control.c` | Modes, command rules, safety interlock, the sequence |
| `App/Src/drive.c`, `drive_hw.c` | Arm drive: run to a limit switch, soft start, timeout, timed pulses; pins |
| `App/Src/drill.c` | Drill PWM (no task: control calls it) |
| `App/Src/rotate.c`, `servo.c`, `servo_uart.c` | Base rotation: goals and polling; ST3215 protocol; USART2 link |
| `App/Src/sim.c`, `servo_sim.c` | Simulated arm, drill and servo (`HW_SIMULATED`) |
| `App/Src/terminal.c`, `ui.c`, `console.c` | Terminal: keys, screen, command language |
| `App/Src/task_*.c` | The four task loops (control, drive, rotate, terminal) |
| `App/Src/term_io_usb.c` | Terminal transport: USB CDC |
| `App/Src/status_led_blackpill.c` | Status LED on PC13 |
| `App/Inc/robot_state.h` | Shared state; each field has one writer |
| `robot.ioc` | CubeMX configuration |
