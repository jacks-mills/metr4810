# cutter-ramp (PCB)

Runs the cutting head on the LEFT motor header through a fixed speed ramp
from power-up, with no serial link needed, and fades the LED on J7 with it.
Standalone like `uart-test`: register level, no HAL, no CubeMX, no FreeRTOS.

## What it does

1. 5 s: motor stopped, LED flashing. **Stand clear.**
2. 10 s: ramps up from stopped to full speed.
3. 2 s at full speed.
4. 10 s: ramps down to stopped.
5. 2 s stopped, then back to 2, for ever.

The LED fades with the motor: off when stopped, brightest at full speed.

## Pins

| Function | Pin | Net | Notes |
|---|---|---|---|
| Cutting head PWM | PA6 (TIM3 CH1) | LEFT_PWM | 20 kHz, inverted: high = stopped |
| Cutting head direction | PE0 | LEFT_DIR | high = forward |
| LED | PA9 (TIM1 CH2) | J7 | 1 kHz, with a series resistor |
| Arm drive PWM | PA7 | RIGHT_PWM | held low (stopped) |
| Drill PWM | PB0 | DRILL_PWM | held high (stopped) |

## Settings (top of `Src/main.c`)

| Setting | Default | |
|---|---|---|
| `START_DELAY_MS` | 5000 | stopped, LED flashing, after reset |
| `RAMP_UP_MS`, `RAMP_DOWN_MS` | 10000 | length of each ramp |
| `HOLD_TOP_MS`, `HOLD_STOPPED_MS` | 2000 | pauses at the top and bottom |
| `MOTOR_TOP_PERMILLE` | 1000 | top of the ramp (1000 = full speed) |
| `MOTOR_PWM_INVERTED` | 1 | 1: pin high = stopped (FIT0441). 0: pin high = run |
| `MOTOR_DIR_LEVEL` | 1 | flip if the cutting head turns the wrong way |
| `LED_MAX_PERMILLE` | 1000 | LED duty at full brightness (1000 = fully on) |
| `LED_ACTIVE_HIGH` | 1 | 0 if the LED goes from 3.3 V to PA9 instead of PA9 to GND |

## Build, flash

```sh
cd ~/uni/metr4810/repo/stm32f401vct6-pcb/cutter-ramp
rm -rf build
cmake --preset Debug
cmake --build --preset Debug
STM32_Programmer_CLI -c port=SWD -w build/Debug/cutter-ramp.elf -v -rst
```

## Before the first run

1. **Check the PWM polarity with the motor's 12 V off.** The code assumes
   the cutting head is a FIT0441 like the drill (input high = stopped).
   After flashing, measure LEFT_PWM (PA6) to GND with a multimeter: for the
   first 5 s it should read about 3.3 V, then fall towards 0 V as the ramp
   goes up (a multimeter shows the average). If the motor on that header
   runs when its input is high instead, set `MOTOR_PWM_INVERTED` to 0.
2. **Turn the motor's 12 V off before flashing.** PA6 floats while the F401
   is in reset or being flashed, and the FIT0441 runs flat out on a low
   input. If the cutting head spins during reset, add a 10 kΩ pull-up from
   LEFT_PWM to 3.3 V.

Pressing reset starts the sequence again from step 1.
