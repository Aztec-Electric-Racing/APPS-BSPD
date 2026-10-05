# APPS-BSPD

Firmware for AER's accelerator pedal position sensor (APPS) board. It reads two pedal sensors on an STM32F446RCT6, checks each reading against saved limits, and sets two DAC outputs. USB commands let you read the sensor values and save calibration limits.

## Board and signals

| Signal | MCU pin | Use |
| --- | --- | --- |
| APPS1 | PC4 | ADC1 input, channel 14 |
| APPS2 | PC5 | ADC1 input, channel 15 |
| DAC1 | PA4 | Analog output |
| DAC2 | PA5 | Analog output |
| USB | USB FS | USB CDC serial connection |

The project is configured for an 8 MHz crystal and a 30 MHz system clock. The APPS code assumes the input divider passes 72% of the sensor voltage. Check the schematic and measure the board before relying on these values.

## Build and flash

You need CMake 3.20 or newer, Ninja, and the GNU Arm Embedded toolchain. To build from the repository folder:

```sh
cmake -S . -B build -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

This creates `build/APPS-BSPD.elf`, `build/APPS-BSPD.hex`, and `build/APPS-BSPD.bin`.

To flash with an ST-Link, install OpenOCD, connect the board, and run:

```sh
cmake --build build --target flash
```

You can also open the project in STM32CubeIDE and use `APPS_BSPD Firmware Debug.launch`.

## USB commands

Open the board's USB CDC port in a terminal and send a command followed by Enter. The baud-rate setting does not affect USB. Only one program can use the port at a time.

| Command | Result |
| --- | --- |
| `HELP` | Lists the commands. |
| `PING` | Replies `PONG`. |
| `READ` | Prints raw ADC counts for both sensors. |
| `STATUS` | Prints sensor readings, saved limits, fault state, and calculated DAC values. |
| `GETSAVED` | Prints calibration values saved in flash. |
| `SAVE LOWER` | Saves the current readings as the lower limits. |
| `SAVE UPPER` | Saves the current readings as the upper limits. |
| `HEARTBEAT ON` | Prints status once per second. |
| `HEARTBEAT OFF` | Stops periodic status messages. |

For calibration, put the pedal at its released position and send `SAVE LOWER`. Move it to full travel and send `SAVE UPPER`. Use `GETSAVED` to check the stored values. A save erases a 128 KiB flash sector and can pause the firmware briefly.

## TODO

### Firmware

- [ ] Confirm the 10% sensor-difference tolerance against the applicable rules and vehicle design. It is currently a code constant, not a verified requirement.
- [ ] Add sensor-to-sensor mismatch detection and torque shutdown when the mismatch lasts longer than 100 ms. The current code only checks each sensor against its own saved range.
- [ ] Confirm how open, shorted, stuck, invalid, and lost sensor signals should be detected. Add any missing checks and define the fault response.
- [ ] Review the calibration storage approach and flash wear. The code comment notes that saving currently erases a flash sector; determine whether EEPROM emulation is suitable for expected calibration writes.
- [ ] Confirm the DAC calculation and whether output scaling must account for any external circuit or divider. The code comment calls this out as unresolved.

### Bench and vehicle checks

- [ ] Compare ADC counts and reported voltages with measured APPS voltages, including the input divider.
- [ ] Measure both DAC outputs and compare them with the reported DAC commands across the pedal range.
- [ ] Record each sensor's readings at released pedal (0%) and full travel (100%); confirm the pedal returns to zero and the mechanical stop prevents overtravel.
- [ ] Test unplugged, shorted, stuck, out-of-range, and mismatched sensor inputs. Record whether torque is removed and how long each fault takes to respond.
- [ ] Confirm the two sensors are electrically separate and have different transfer functions.
- [ ] Confirm which controller handles brake-plus-throttle plausibility. The current firmware has no brake input or brake/APPS check. The vehicle behavior must cut torque when brake is pressed with APPS above 25%, and keep it cut until APPS is below 5%.
- [ ] Verify the physical torque/shutdown response; USB messages alone do not prove that the outputs or shutdown circuit work.

### Vehicle-level ownership

- [ ] Keep BOTS and BSPD shutdown functions independent of programmable firmware. This repository must not delay or bypass them.
- [ ] Confirm this board's signals and behavior are represented in the system wiring documentation and any CAN/DAQ handoff that applies.
- [ ] Track the other system requirements—BMS, precharge, indicators, shutdown circuit, charging, and Ready to Drive—in their owning repositories and system documents. They are not implemented by this APPS firmware.

## Code locations

- `Core/Src/main.c` — sensor checks, DAC calculation, calibration, and USB commands.
- `Core/Src/ee.c` and `Core/Src/ee_config.h` — calibration storage in flash.
- `USB_DEVICE/` and `Middlewares/` — USB CDC support.
- `CMakeLists.txt` and `cmake/` — build and flash setup.
- `APPS_BSPD Firmware.ioc` — STM32CubeMX project configuration.
