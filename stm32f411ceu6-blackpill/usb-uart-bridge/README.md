# usb-uart-bridge (Blackpill)

Turns the Blackpill into a 3.3 V USB-serial adapter. Its USB-C port shows up
on the PC as a serial port (`/dev/ttyACM*` or `COMx`), and everything is
passed through to USART1 on PA9 (TX) and PA10 (RX). It's for reaching the
PCB's terminal on J7 without the PL2303 adapter, whose TX is 5 V.

The baud rate, parity and stop bits follow the terminal program, like any
USB-serial adapter. Until the program sets them it's 115200 8N1.

Standalone: the HAL and ST's USB device library (copied into `lib/`, the
same versions robot uses) do the USB; USART1 and its DMA are set up at
register level. No CubeMX, no FreeRTOS.

## Wiring to the PCB (J7)

| Blackpill | | PCB J7 |
|---|---|---|
| GND | | GND (connect this first) |
| PA9 (TX) | 1 kΩ | PA10 (RX) |
| PA10 (RX) | 1 kΩ | PA9 (TX) |

- **Don't connect 3.3 V or 5 V between the boards.** The Blackpill runs from
  its USB-C, the PCB from its own supply.
- **The 1 kΩ resistors** limit the current when one board is powered and the
  other isn't: a TX pin driving an unpowered chip's RX pin powers it through
  its protection diodes. As a habit, turn the PCB on before plugging the
  Blackpill in, and unplug the Blackpill before turning the PCB off.
- **Take the test LED off J7** first.
- Same pins and settings as uart-test and the PCB robot terminal.

## Build, flash, run

```sh
cmake --preset Debug                  # once, or after deleting build/
cmake --build --preset Debug
STM32_Programmer_CLI -c port=SWD -w build/Debug/usb-uart-bridge.elf -v -rst
picocom -b 115200 /dev/ttyACM0        # quit with Ctrl-A Ctrl-X
```

**Always give the baud rate** (`-b 115200`, or Speed 115200 in PuTTY). The
bridge uses whatever the terminal asks for, and picocom's default is 9600,
which turns everything from the PCB into garbage.

Without an ST-LINK, flash over USB instead: hold BOOT0, press and release
NRST, release BOOT0, then
`STM32_Programmer_CLI -c port=USB1 -w build/Debug/usb-uart-bridge.elf -v`
and press NRST. To use the Blackpill as the robot again, flash robot.

The robot firmware on the Blackpill also appears as `/dev/ttyACM*`. The
bridge calls itself "USB-UART bridge", so `ls /dev/serial/by-id/` tells them
apart. On Windows, PuTTY → Serial → the new `COMx` (Device Manager → Ports);
Windows 10 and 11 need no driver.

## Check it on its own first

Connect PA9 straight to PA10 on the Blackpill (loopback), open the terminal,
and type: every key should come back. That proves the USB side, the UART and
the DMA before the PCB is involved. Try a big paste too; nothing should go
missing.

## The PCB robot through the bridge

The robot draws its screen once at start-up, and the bridge throws away
anything that arrives while no terminal has the port open. Open the terminal,
then press **Ctrl-L** to redraw the screen.

## LED (PC13)

| LED | Meaning |
|---|---|
| Blinks at 1 Hz | No USB connection to the PC (or not enumerated yet) |
| Short blip every second | Connected, but no terminal has the port open |
| On | Terminal open; flickers while data moves |
| Very fast flicker | Error: the crystal didn't start, or a fault |

## How it behaves

- **PC to target:** the USB link holds the PC off when the UART can't keep
  up, so even a big paste arrives complete.
- **Target to PC:** 4 KB of buffer. If the PC stops reading for long enough
  (0.35 s at 115200) the newest bytes are dropped. While no terminal has the
  port open (DTR low), everything from the target is thrown away, so
  opening the port never replays old output.
- **Formats:** 1500 to 6000000 baud; 8N1, 8E1, 8O1, 7E1 or 7O1; 1, 1.5 or 2
  stop bits. Anything else is ignored and the previous format kept.
- **Debugger:** `bridgeStats` counts dropped bytes and rejected formats;
  `uartErrors` counts line errors. Rising `framingErrors` almost always
  means the two ends have different baud rates.

## If it doesn't work

- **No `/dev/ttyACM*` and the LED blinks at 1 Hz:** the PC hasn't
  enumerated it. Try another cable (some USB-C cables are charge-only);
  `dmesg | tail` shows what happened.
- **Garbage characters:** the terminal isn't at 115200 (see above).
- **Nothing from the PCB:** check that TX and RX are crossed (each PA9 goes
  to the other board's PA10), that GND is shared, and that the PCB is powered
  and running. Try the loopback check above.
- **Characters you type do nothing on the PCB:** the bridge's PA9 isn't
  reaching the PCB's PA10, or the PCB's RX pin is damaged. The loopback
  check tells which.
- **Odd `AT` commands reach the target just after plugging in:** that's
  ModemManager on Linux probing the new port. Stop it with
  `sudo systemctl stop ModemManager`.

## Testing the bridge logic on a PC

`test/bridge_test.c` runs `Src/bridge.c` against a fake USB port and UART:
order and completeness both ways, holding the PC off, nothing sent while the
port is closed, and format changes only between bytes.

```sh
cd test
gcc -std=c11 -Wall -Wextra -I../Inc -o bridge_test bridge_test.c ../Src/bridge.c && ./bridge_test
```

## Layout

| Path | What |
|---|---|
| `Src/main.c` | Clock (25 MHz crystal, 96 MHz, 48 MHz USB), LED, main loop |
| `Src/bridge.c` | The two FIFOs, flow control, format changes, LED patterns |
| `Src/uart.c` | USART1 with DMA, register level |
| `Src/usbd_cdc_if.c` | USB serial port callbacks, connected to the bridge |
| `Src/usb_device.c`, `usbd_conf.c`, `usbd_desc.c` | USB device setup, from robot's CubeMX `USB_DEVICE` (only the product name changed) |
| `Inc/stm32f4xx_hal_conf.h` | robot's, minus the unused modules; SysTick at the highest priority |
| `lib/` | CMSIS, the HAL files needed, ST's USB device library (core and CDC class) |
| `test/bridge_test.c` | PC test for `bridge.c` |
