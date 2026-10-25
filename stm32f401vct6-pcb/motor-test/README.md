# motor-test (PCB)

A serial command prompt for the arm motor on J1, with status LEDs on J5.
It's standalone like `uart-test`: register level, no HAL, no CubeMX and no
FreeRTOS, all in one `Src/main.c`. It runs from the internal 16 MHz
oscillator (HSI), so no crystal or PLL setup is needed.

## Commands

Type a command and press Enter. Commands aren't case-sensitive.

| Command | What it does |
|---|---|
| `dir 0`, `dir 1`, `dir toggle` | Sets the direction pin M (PE0) low or high |
| `drive on`, `drive off` | Turns E (PA6) on at 100 % (held high), or off |
| `pulse <ms>` | Turns E on for `<ms>` milliseconds (1 to 60000), then off |
| `stop` | Turns the drive off (same as `drive off`) |
| `status` | Shows pins, limit switches, uptime, reset cause and serial error counts |
| `help` | Lists the commands |
| **Esc** or **Ctrl-C** | Turns the drive off at once, without Enter. Arrow keys send Esc too, so they also stop it |

The settings line is printed after every command, and again whenever
something changes on its own (a pulse ends, a limit switch changes):

```
> dir 1
  dir 1 | drive off | E low, M high | limits: max open, min open
> pulse 500
  dir 1 | drive pulse 500 ms, 500 ms left (starting) | E low, M high | limits: max open, min open
>
pulse done
  dir 1 | drive off | E low, M high | limits: max open, min open
>
```

`E` and `M` are read back from the pins themselves. If a pin reads back at
the other level from the one it's set to, the line adds a warning such as
`!! E (PA6) set high but reads low: shorted or overloaded?`.

- **Pulse timing:** starting from stopped, E goes on at the next 1 ms tick
  and off exactly `<ms>` ticks later. The timing runs in the SysTick
  interrupt, so printing doesn't affect it. HSI is within 1 % at room
  temperature, so the pulse length is too. If E is already on, it stays on
  with no gap and goes off `<ms>` later, to within 1 ms.
- **Changing direction while the motor runs:** E goes off first, M changes,
  and E comes back on about 100 ms later (`REVERSE_PAUSE_MS`). A pulse
  doesn't count down during that pause.
- **Limit switches** (EXT_MAX PC2, EXT_MIN PC3) are only shown. They don't
  stop the motor.

## Wiring

| What | Where | Pin | Notes |
|---|---|---|---|
| DRI0002 E (enable) | J1 pin 3, LEFT_PWM | PA6 | high = motor on |
| DRI0002 M (direction) | J1 pin 4, LEFT_DIR | PE0 | low at reset |
| Heartbeat LED | J5 pin 3, SPI_CS | PB12 | 1 Hz blink |
| Drive LED | J5 pin 4, SPI2_SCK | PB13 | lit while E is on |
| Direction LED | J5 pin 5, SPI2_MISO | PB14 | lit while M is high |
| Serial LED | J5 pin 6, SPI2_MOSI | PB15 | flickers on each byte received, solid for 2 s after a line error |
| Serial TX | J7 | PA9 | to the Blackpill's PA10 through 1 kΩ |
| Serial RX | J7 | PA10 | from the Blackpill's PA9 through 1 kΩ (internal pull-up) |
| Limit switches | | PC2, PC3 | pressed = high (switch to 3.3 V, internal pull-down) |

**LEDs on J5:** wire each one as pin → 330 Ω to 1 kΩ resistor → LED anode,
LED cathode → GND (J5 pin 2 or 8). Leave J5 pins 1 and 7 alone. The J5
pin-to-MCU mapping (PB12 to PB15) was read from the layout and hasn't been
checked against the schematic. At reset, each LED lights in turn for
0.25 s, in the order heartbeat, drive, direction, serial. If the order is
wrong, swap the entries in the `leds[]` table at the top of `Src/main.c`.

The same four signals also go to the board LED pins PD0 to PD3 (green
heartbeat, orange drive, start LED direction, red serial). Those LEDs go
through J4, which isn't fitted, so nothing lights there for now.

**Serial (J7):** use the Blackpill running `usb-uart-bridge` (see its
README). Connect GND first. Use 1 kΩ in each data line, and don't connect any
power pins between the boards. **Take the test LED off J7 first**, because
PA9 is now the serial TX.

The other motor pins (RIGHT_PWM PA7, RIGHT_DIR PE1, DRILL_PWM PB0, DRILL_DIR
PE2) aren't touched. They stay floating inputs, the same as during reset.

## Build, flash, run

```sh
cd stm32f401vct6-pcb/motor-test
cmake --preset Debug                  # once, or after deleting build/
cmake --build --preset Debug
STM32_Programmer_CLI -c port=SWD -w build/Debug/motor-test.elf -v -rst
picocom -b 115200 /dev/ttyACM0        # the Blackpill bridge; quit with Ctrl-A Ctrl-X
```

**Always give `-b 115200`.** The bridge uses whatever baud rate the terminal
asks for, and picocom's default is 9600. On Windows use PuTTY → Serial → the
Blackpill's `COMx`, speed 115200.

The start-up banner is printed once at reset. The bridge keeps it until a
terminal opens the port, so you see it even if you open picocom afterwards.
Press Enter for a fresh prompt, or type `status`.

**Turn the motor supply off while flashing.** PA6 floats while the chip is
in reset or being flashed. If the arm moves during reset, add a 10 kΩ
pull-down from E (PA6) to GND.

## Reliability

- **Motor off at start-up:** E is set low before it becomes an output, and
  it's the first thing the program does.
- **Stop always gets through:** Esc and Ctrl-C are handled in the receive
  interrupt, so they work even if the main loop is busy.
- **The link drops, the motor stops:** any serial error while the drive is
  on turns it off and prints `STOP: serial line error`. That covers framing
  errors, noise, overrun and the receive buffer filling up. Unplugging or
  powering off the Blackpill pulls the RX line low, which shows up as a
  framing error.
- **Watchdog:** if the main loop stops for about 1 s (0.7 to 1.9 s,
  depending on the LSI oscillator), the chip resets with the motor off. The
  watchdog pauses while a debugger has the core halted.
- **Faults:** a HardFault turns E off, prints `*** HardFault: motor off,
  resetting ***` and resets.
- **Reset cause** is printed at start-up and in `status`: power-on,
  watchdog, brown-out, reset pin, or software (after flashing or a fault).
  An unexpected `watchdog` or `brown-out` points at a hang or a supply dip.

## Settings (top of `Src/main.c`)

| Setting | Default | |
|---|---|---|
| `PULSE_MAX_MS` | 60000 | longest pulse allowed |
| `REVERSE_PAUSE_MS` | 100 | E off this long when the direction changes while running |
| `DIR_AT_RESET` | 0 | M level at reset |
| `E_ON_LEVEL` | 1 | E level that runs the motor (DRI0002: high) |
| `LED_ACTIVE_HIGH` | 1 | 0 if the LEDs go from 3.3 V to the pin instead |
| `LIMIT_ACTIVE_HIGH` | 1 | 0 if the switches go to GND (with pull-ups) |
| `WATCHDOG_MS` | 1000 | watchdog timeout, nominal |

## If it doesn't work

- **No LEDs at all after reset:** check the J5 wiring and LED polarity. If
  the LEDs on J5 do nothing but SWD flashing works, the J5 pins may not be
  PB12 to PB15. Check the schematic.
- **Heartbeat blinks but nothing on the terminal:** check the baud rate
  (`-b 115200`), that TX and RX are crossed (each PA9 goes to the other
  board's PA10), and the shared GND. Do the bridge's loopback check (see its
  README).
- **Text arrives but typing does nothing:** the Blackpill's PA9 isn't
  reaching the PCB's PA10. The serial LED (J5 pin 6) should flicker as you
  type. If it doesn't, no bytes are getting through.
- **Serial LED stays on and `status` shows framing errors:** the baud rate is
  wrong, or the RX line is noisy or disconnected.
- **`drive on` and E reads high, but the motor doesn't move:** the firmware
  side is working. Measure E and M at the DRI0002 itself (E ≈ 3.3 V), then
  check the driver's logic supply and the 12 V motor supply. The DRI0002
  needs its logic power (VD 6.5 to 12 V, or its supply jumper set) as well
  as the motor supply.
- **The arm goes the wrong way:** that's just which M level means extend.
  Note which `dir` value extends.

## Testing without hardware

QEMU's F405 board runs it, but QEMU doesn't model the F4 GPIO or RCC, so
`E`/`M` always read `low` and the reset cause shows `unknown`. Its SysTick
also runs faster than real time.

```sh
qemu-system-arm -M netduinoplus2 -nographic -serial mon:stdio -kernel build/Debug/motor-test.elf
```

Quit with Ctrl-A then X. With `-d unimp -D log.txt`, every write to E shows
up as a `GPIOA ... offset 0x018` line: value `0x40` = on, `0x400000` = off,
one per millisecond tick. The 25 Oct checks used this to confirm that
`pulse 1`, `pulse 50`, `pulse 777` and `pulse 3000` held E on for exactly
that many ticks, in both Debug and Release.
