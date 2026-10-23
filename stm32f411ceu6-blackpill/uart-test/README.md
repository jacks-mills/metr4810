# uart-test (Blackpill)

Checks the 3.3 V USB-serial adapter and terminal program before they're used
for the robot2 terminal on the PCB. Same port and settings as the PCB:
USART1, PA9 TX / PA10 RX, 115200 8N1. Standalone: register level, no HAL,
no CubeMX, no FreeRTOS.

| Command | |
|---|---|
| `led on`, `led off`, `led toggle` | blue LED on PC13 |
| `led status` | says whether the LED is on |
| `echo <text>` | sends back everything after `echo ` |
| `help` | list the commands |

Commands aren't case-sensitive; Enter (CR, LF or CR LF) ends a line.
Characters are echoed as you type (picocom doesn't show what you type) and
Backspace works. Control bytes show as `<hex>`, so an arrow key appears as
`<1B>[A`.

Clock is the reset default, HSI 16 MHz, so it works without the crystal.

## Wiring

| Adapter | Blackpill |
|---|---|
| TX | PA10 (RX) |
| RX | PA9 (TX) |
| GND | GND |

Power the Blackpill from its USB-C as usual and leave the adapter's power pin
unconnected. On the PCB the same adapter goes to J7 (also PA9/PA10).

## Build, flash, run

```sh
cmake --preset Debug                  # once, or after deleting build/
cmake --build --preset Debug
STM32_Programmer_CLI -c port=SWD -w build/Debug/uart-test.elf -v -rst
picocom -b 115200 /dev/ttyUSB0        # quit with Ctrl-A Ctrl-X
```

The banner is printed once at reset, so press the Blackpill's NRST button
after opening the terminal if you missed it.

## If it doesn't work

- Nothing at all: TX and RX swapped (most common), no shared GND, or wrong
  `/dev/tty*` (`dmesg | tail` after plugging the adapter in).
- Garbage characters: baud rate mismatch (should be 115200).
- Text arrives but typed characters aren't echoed: adapter TX isn't reaching
  PA10.
