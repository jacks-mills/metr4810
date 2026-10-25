# motor-test (PCB, STM32F401VCT6)

Serial command prompt for the motor on J1, plus a PWM test, the ST3215 servo and status LEDs.

## Install (once)

```sh
# Ubuntu / Debian
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi cmake ninja-build openocd picocom
# Fedora
sudo dnf install arm-none-eabi-gcc-cs arm-none-eabi-gcc-cs-c++ arm-none-eabi-newlib cmake ninja-build openocd picocom
# Arch
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib cmake ninja openocd picocom

sudo usermod -aG dialout $USER    # serial port access (Arch: uucp), then log out and back in
```

## Build

```sh
cd stm32f401vct6-pcb/motor-test
cmake --preset Release
cmake --build --preset Release
```

## Flash

ST-Link to J8: pin 2 SWCLK, pin 3 SWDIO, pin 5 GND, pin 4 3.3 V.

```sh
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/Release/motor-test.elf verify reset exit"
```

## Run

The console goes through a Blackpill running `usb-uart-bridge`. Build and flash it the same way from
`stm32f411ceu6-blackpill/usb-uart-bridge` (file `build/Release/usb-uart-bridge.elf`).

| Blackpill | PCB |
|---|---|
| PA9 | J7 pin 6 |
| PA10 | J7 pin 5 |
| GND | J7 pin 8 |

Motor driver: E to J1 pin 5, M to J1 pin 4, GND to J1 pin 2.

```sh
picocom -b 115200 /dev/ttyACM0    # quit with Ctrl-A Ctrl-X
```

```
> help                                         list all commands
> dir 1                                        direction
> pulse 500                                    motor on for 500 ms
> auto pulse 200 wait 300 timeout 10s start    automatic back-and-forth
> stop                                         or press Esc
```
