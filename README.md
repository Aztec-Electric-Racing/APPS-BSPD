

# 🔌 USB-Fix and Flash Conversion (WORK IN PROGRESS)

> *Madam Team Lead (Zaina) if i got anything wrong, fix everything pls.*

## ℹ️ Overview:

### What is APPS?

APPS, in Formula SAE is the *Accelerator Pedal Position Sensor*. The program reads two pedal sensors (APPS1/APPS2) and checks if they are within safety parameters. This allows the DAC (Digital-to-Analog Converter) to output voltages into computed values.  

> *Work In Progress*
## 🛠️ Hardware

MCU (STM32F446RCT6, 8 MHz crystal), pin table (PC4/PC5 = APPS inputs, PA4/PA5 = DAC outputs, PB14/PB15 = USB)

> *double check with zaina*

> *There were also some problems with the [ST-Link/V2](https://www.st.com/resource/en/user_manual/um1075-stlinkv2-incircuit-debuggerprogrammer-for-stm8-and-stm32-stmicroelectronics.pdf)*

## 🚀 Build & Flash

Open in [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html), use `APPS_BSPD Firmware Debug.launch`, connect the ST-Link.
### Terminal Settings & Commands

- Any baud works, but we specifically used `115200`. Only one program can hold COM3 at a time.

> *Errr.... I actually don't remember the baud but I think this is what we used*
## 👾 The Problem

We were having a few issues when attempting to flash the repo into the board, specifically with the `USBCDC`.
- The repo itself did not compile (6 errors), had several bugs that was fixed.
	- Not defined anywhere:
		- `CDC_SendString`
		- `sensAcceptibleDiff`
		- `cmd`
- The terminal on PuTTY was outputting `RX` in COM3. Every command outputted `RX` and nothing else. The `PING` / `PONG` logic was never observed on the terminal.
	- This was fixed. The firmware on the board was not built from the current repo.
## 🔧 Fixes (Yay)

### Compile errors (fixed)

- **Wrong EEPROM function names:** `EE_Init`/`EE_Read`/`EE_Write` don't exist. The library uses lowercase `ee_init`/`ee_read`/`ee_write`.
- **`sensAcceptibleDiff` was never defined:** it's now `SENS_ACCEPTABLE_DIFF = 0.10` (*ASSUMED 10% FSAE LIMIT*)
	- ^^^ !!! ***Confirm with Zaina because this is assumed*** !!! ^^^
- **`cmd` was never defined, and `CDC_SendString` didn't exist:** both are now implemented.
- Missing `#include`s were added. `uint32_t` values are now printed portably using `PRIu32` from `<inttypes.h>`.

### Found by Code Review:

- The DAC outputs were never started.
- The ADC only took one reading.
- `SAVE UPPER` / `SAVE LOWER` always stored 0.
- Calibration was reset on every boot.


## 💾 Flash Calibration Storage Conversion

The `flash-conversion` branch replaces the original erase-before-every-save EEPROM behavior with append-only Flash calibration records.

### Storage Layout

- Application: Flash sectors 0–3, limited to 64 KB
- Storage sector A: sector 4 at `0x08010000`, 64 KB
- Storage sector B: sector 5 at `0x08020000`, 128 KB

The linker prevents application code from overlapping either storage sector.

### Record Format

Each calibration save creates a fixed 40-byte record containing:

- Record identifier
- Format version
- Sequence number
- Four APPS calibration values
- Reserved field
- CRC-32
- Commit marker

The commit marker is written last. Incomplete or interrupted writes are therefore rejected during startup.

### Storage Behavior

- Normal saves append without erasing the active sector.
- Both sectors are scanned at startup.
- The newest valid committed record is loaded.
- Corrupted, incomplete, unsupported, and invalid records are ignored.
- Active RAM calibration is updated only after a successful Flash commit.
- Valid legacy sector-5 calibration can be migrated into the new record format.
- When the active sector fills, the alternate sector is erased and verified.
- The previous calibration and requested calibration are committed in the alternate sector before the old sector is erased.
- A failed old-sector cleanup is reported separately from a failed calibration save.

### Current Status

- Debug build passes.
- Release build passes.
- Hardware bench testing is pending.
- The final `main.c` baseline must be confirmed before merging.
- This branch should not be merged until review and bench testing are complete.

## ✅ USB-Fix Baseline Verification:

The board now prints over the USB as soon as the port is opened. It was flashed and tested over COM3:

| Sent           | Reply from Board                                                                 |
| -------------- | -------------------------------------------------------------------------------- |
| `PING`     | `PONG`                                                                       |
| `READ`     | `APPS1=70, APPS2=72`                                                         |
| `GETSAVED` | `Saved Upper1=2749, Saved Upper2=1914, Saved Lower1=1914, Saved Lower2=1079` |

> No sensors were connected.

```
=== AER APPS-BSPD firmware, built Sep 30 2026 20:17:04 ===
USB serial OK. Type HELP for commands.
[USB OK] up 12s | APPS1=68 (54mV) APPS2=75 (60mV) | FAULT | DAC1=0 (0mV) DAC2=0 (0mV)
[USB OK] up 13s | APPS1=74 (59mV) APPS2=71 (57mV) | FAULT | DAC1=0 (0mV) DAC2=0 (0mV)
```
> These results verify the earlier USB fix. They do not constitute hardware verification of the Flash-storage conversion.

## 📝 TODO

> From Jack Varagas

- Need to double check ADC and DAC conversions as well as account for the voltage divider on ADC inputs
