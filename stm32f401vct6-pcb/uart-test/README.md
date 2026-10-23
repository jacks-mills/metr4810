# uart-test (PCB)

Checks the 3.3 V USB-serial adapter on J7 and the four LEDs before running
robot2 on the PCB. Same port and settings as the robot2 terminal: USART1,
PA9 TX / PA10 RX, 115200 8N1. Standalone: register level, no HAL, no CubeMX,
no FreeRTOS. The Blackpill version is `../../stm32f411ceu6-blackpill/uart-test`.

| Command | |
|---|---|
| `led on`, `led off`, `led toggle`, `led status` | all four LEDs |
| `led <colour> on` / `off` / `toggle` / `status` | one LED: `red` (PD0), `orange` (PD1), `green` (PD2), `start` (PD3) |
| `echo <text>` | sends back everything after `echo ` |
| `help` | list the commands |

Commands aren't case-sensitive; Enter (CR, LF or CR LF) ends a line.
Characters are echoed as you type (picocom doesn't show what you type) and
Backspace works. Control bytes show as `<hex>`, so an arrow key appears as
`<1B>[A`.

Clock is the reset default, HSI 16 MHz (the PCB has no crystal).

## LED polarity

The LEDs are assumed lit when the pin is high, like robot2's
`status_led_pcb.c`. If `led on` turns them off, set `LED_ACTIVE_HIGH` to 0 in
`Src/main.c`, and change `LED_ON` in robot2's `status_led_pcb.c` to
`GPIO_PIN_RESET` too.

## Wiring (J7)

| Adapter | PCB |
|---|---|
| TX | PA10 (RX) |
| RX | PA9 (TX) |
| GND | GND |

Leave the adapter's power pin unconnected unless the board has no other 3.3 V
supply.

## Build, flash, run

```sh
cmake --preset Debug                  # once, or after deleting build/
cmake --build --preset Debug
STM32_Programmer_CLI -c port=SWD -w build/Debug/uart-test.elf -v -rst
picocom -b 115200 /dev/ttyUSB0        # quit with Ctrl-A Ctrl-X
```

The banner is printed once at reset; type `help` if you missed it.

In QEMU (no F401 model, but the F405 one runs it; LEDs aren't shown, but
`led status` tracks them):

```sh
qemu-system-arm -M netduinoplus2 -nographic -serial mon:stdio -kernel build/Debug/uart-test.elf
```

Quit with Ctrl-A then X.

## If it doesn't work

- Nothing at all: TX and RX swapped (most common), no shared GND, or wrong
  `/dev/tty*` (`dmesg | tail` after plugging the adapter in).
- Garbage characters: baud rate mismatch (should be 115200).
- Text arrives but typed characters aren't echoed: adapter TX isn't reaching
  PA10.
