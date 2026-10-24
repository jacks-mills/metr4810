# METR4810

Firmware for the METR4810 Group 7 boring robot. It was developed first on an
STM32F411CEU6 Blackpill, then ported to our STM32F401VCT6 PCB.

```
stm32f411ceu6-blackpill/    Blackpill (STM32F411CEU6, 25 MHz crystal, USB-C)
    blinky/                 LED blink test
    robot/                  robot firmware, terminal over USB
    uart-test/              USB-serial adapter test
    usb-uart-bridge/        the Blackpill as a 3.3 V USB-serial adapter for the PCB
stm32f401vct6-pcb/          our PCB (STM32F401VCT6, no crystal, no USB)
    blinky/                 LED blink test
    robot/                  robot firmware, terminal over USART1 (J7)
    uart-test/              USB-serial adapter and LED test
```

## Projects

### robot (both boards)

The robot firmware, written with FreeRTOS. It runs the drilling sequence:
home the arm, spin up the drill, extend to the far limit switch, back out
still drilling, then stop the drill. It drives the arm motor (DRI0002), the
drill (FIT0441 BLDC) and the base-rotation servo (ST3215, half-duplex UART).
You control it from a terminal UI with commands such as `start`, `pause`,
`stop` and manual `drive` / `rotate` / `drill`.

- **Blackpill:** terminal over the USB-C port (USB CDC), status LED on PC13.
- **PCB:** terminal over USART1 on J7 through a 3.3 V USB-serial adapter
  (or a Blackpill running usb-uart-bridge); four status LEDs on PD0–PD3.

Our own code lives in `App/`. The two `App/` folders are identical apart from the board files,
so a fix to shared logic has to be copied across by hand. Configure with
`-DROBOT_SIMULATED=ON` to simulate the arm, drill and servo on a bare board.
Each project's README has the pinout, wiring and details.

### uart-test (both boards)

A small standalone program for checking a USB-serial adapter and terminal
before using them with robot. It uses USART1 (PA9 TX, PA10 RX) at 115200 8N1,
the same as the robot terminal on the PCB. It's written at register level
with no HAL, CubeMX or FreeRTOS. Commands: `led on`, `led off`, `led toggle`,
`led status`, `echo <text>`, `help`. On the PCB, `led red on` and so on
picks one of the four LEDs.

### usb-uart-bridge (Blackpill)

Turns the Blackpill into a 3.3 V USB-serial adapter, for reaching the PCB's
terminal on J7 without a separate adapter. The USB-C port appears on the PC
as a serial port, and everything goes through to USART1 (PA9 TX, PA10 RX),
which is wired to J7 with 1 kΩ in each line. The baud rate follows the
terminal program, so always give it (`picocom -b 115200`). It uses the HAL
and ST's USB library (its own copy in `lib/`) for the USB, and register-level
USART1 with DMA. Its README has the wiring, LED meanings and a loopback check.

### blinky (both boards)

The first "is the board alive" tests. Both run from the internal 16 MHz
oscillator (HSI), so they work even if the crystal or clock setup is wrong.

- **Blackpill:** flashes the PC13 LED, 333 ms on and 333 ms off.
- **PCB:** toggles the green LED (PD2) every second.

### How the projects are set up

Every project builds the same way, with CMake and Ninja (see
[Build](#build)). robot and blinky are STM32CubeMX projects: the `.ioc` file
holds the pin, clock and peripheral setup, and CubeMX generates `Core/`,
`Drivers/` and the files in `cmake/` from it. uart-test and usb-uart-bridge
are hand-written and have no `.ioc`.

## Software

| Tool | What it's for | Needed for |
|---|---|---|
| [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html) | ST's command-line toolset: Arm GCC (`arm-none-eabi-gcc`), CMake, Ninja, `STM32_Programmer_CLI` and the ST-LINK drivers, all in one install | building and flashing everything |
| [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) | edits `.ioc` files and regenerates the HAL/start-up code | only when changing pins, clocks or peripherals in robot or blinky |
| Serial terminal | talking to robot and uart-test | picocom (Linux), PuTTY (Windows) |
| Git | getting the repo | |
| QEMU (optional) | running uart-test without hardware | |

ST's downloads need a free myST account. You'll also need an **ST-LINK** (V2
or V3) to flash over SWD, and for the PCB a **3.3 V** USB-serial adapter or
the Blackpill running usb-uart-bridge.

### Linux (Ubuntu / Debian)

1. **Basic tools:**

   ```sh
   sudo apt update
   sudo apt install git picocom
   sudo apt install qemu-system-arm        # optional
   ```

2. **STM32CubeCLT:** download the *Debian Linux installer* from the
   STM32CubeCLT page, unzip it and run the bundle. It installs to
   `/opt/st/stm32cubeclt_<version>` along with the ST-LINK udev rules:

   ```sh
   unzip en.st-stm32cubeclt_*.zip
   sudo sh ./st-stm32cubeclt_*_amd64.deb_bundle.sh
   ```

   Log out and back in so the new `PATH` takes effect, then check:

   ```sh
   arm-none-eabi-gcc --version
   cmake --version
   ninja --version
   STM32_Programmer_CLI --version
   ```

   On Fedora use the `rpm_bundle.sh` file instead. Instead of CubeCLT you can
   use the distribution's packages (`sudo apt install gcc-arm-none-eabi
   libnewlib-arm-none-eabi cmake ninja-build`) plus a separate STM32CubeProgrammer
   install for `STM32_Programmer_CLI`.

3. **Serial port access:** let your user open `/dev/ttyUSB*` and `/dev/ttyACM*`,
   then log out and back in:

   ```sh
   sudo usermod --append --groups dialout $USER
   ```

4. **STM32CubeMX** (only if you'll change a `.ioc`): download the Linux zip,
   unzip it and run the `SetupSTM32CubeMX-*` installer inside.

### Windows 10 / 11

Run these in PowerShell.

1. **Basic tools:**

   ```powershell
   winget install --id Git.Git --exact
   winget install --id PuTTY.PuTTY --exact
   winget install --id SoftwareFreedomConservancy.QEMU --exact   # optional; installs to C:\Program Files\qemu (not on PATH)
   ```

2. **STM32CubeCLT:** download the *Windows installer*, unzip it and run the
   `.exe` as administrator. Keep the **default install location**, because
   the installer only adds the tools to `PATH` for the default location. In
   *Choose Components*, tick the **ST-LINK driver**. Open a new PowerShell
   window afterwards and check:

   ```powershell
   arm-none-eabi-gcc --version
   cmake --version
   ninja --version
   STM32_Programmer_CLI --version
   ```

3. **USB-serial adapter driver:** FTDI adapters usually work straight away.
   CH340 adapters need the WCH driver and CP210x adapters the Silicon Labs
   driver. Plug the adapter in and look under **Device Manager → Ports (COM &
   LPT)** for its `COMx` number.

4. **STM32CubeMX** (only if you'll change a `.ioc`): download and run the
   Windows installer.

## Build

Every project builds the same way. From the project folder:

```sh
cd stm32f401vct6-pcb/robot            # or any other project
cmake --preset Debug                  # once, or after deleting build/
cmake --build --preset Debug          # after every change
```

The result is `build/Debug/<project>.elf` (`robot.elf`, `uart-test.elf`,
`blinky.elf`, `usb-uart-bridge.elf`). Use `Release` instead of `Debug` for an optimised build in
`build/Release/`.

- **Simulated robot:** `cmake --preset Debug -DROBOT_SIMULATED=ON` (`OFF` to
  go back). The setting is kept in `build/Debug` until you change it.
- **Clean build:** delete the `build` folder and configure again
  (`rm --recursive --force build` on Linux, `Remove-Item -Recurse -Force build` in
  PowerShell). Also do this after moving or renaming a project folder,
  because the CMake cache stores the full path.
- **After editing a `.ioc`:** open it in STM32CubeMX and click **GENERATE
  CODE**, then build as usual. Code in `App/` and inside `USER CODE` blocks is
  kept.

## Flash

Connect the ST-LINK to the board's SWD pins: **SWDIO, SWCLK, GND**, and
**3.3 V** if the board isn't powered some other way. NRST is optional. Then,
from the project folder:

```sh
STM32_Programmer_CLI --connect port=SWD --write build/Debug/robot.elf --verify -rst
```

`--write` writes the file, `--verify` reads it back to check it and `-rst`
resets the chip so the new program starts (`-rst` has no long form). Change
the `.elf` name for other projects. It's the same command on Windows and
Linux, and `STM32_Programmer_CLI --help` lists every option.

If it can't connect, check the wiring and power first. If the running
program has disabled the SWD pins or is stuck, wire NRST as well and connect
under reset: `--connect port=SWD mode=UR`.

## Serial terminal

All the terminals are 115200 baud, 8N1, no flow control.

| Project | Connection | Linux | Windows |
|---|---|---|---|
| Blackpill robot | USB-C cable | `picocom /dev/ttyACM0` | PuTTY, Serial, the board's `COMx` |
| PCB robot, both uart-tests | 3.3 V USB-serial adapter: adapter TX → PA10, adapter RX → PA9, GND → GND | `picocom --baud 115200 /dev/ttyUSB0` | PuTTY, Serial, the adapter's `COMx`, speed 115200 |
| PCB robot through usb-uart-bridge | Blackpill USB-C; Blackpill PA9 → 1 kΩ → PCB PA10, PCB PA9 → 1 kΩ → Blackpill PA10, GND → GND | `picocom --baud 115200 /dev/ttyACM0` (the baud rate is required) | PuTTY, Serial, the Blackpill's `COMx`, speed 115200 |

Quit picocom with **Ctrl-A** then **Ctrl-X**. `dmesg | tail` (Linux) or Device
Manager (Windows) shows which port appeared when you plugged in. robot's UI
uses ANSI escape codes, which both picocom and PuTTY handle.

## Without hardware (optional)

QEMU has no F411 or F401 model, but its F405 board runs uart-test unchanged.
LEDs aren't shown; `led status` still tracks them.

```sh
cd stm32f411ceu6-blackpill/uart-test
cmake --preset Debug && cmake --build --preset Debug
qemu-system-arm -machine netduinoplus2 -nographic -serial mon:stdio -kernel build/Debug/uart-test.elf
```

Quit with **Ctrl-A** then **X**. robot doesn't run in QEMU because it needs
USB, timers and the PLL. Use `-DROBOT_SIMULATED=ON` on a real board instead.
