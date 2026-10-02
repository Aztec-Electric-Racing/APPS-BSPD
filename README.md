

# 🔌USB-Fix (WORK IN PROGRESS)

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
- **Missing pieces:**  The missing `#include`s was added the `printf` format warnings (`%u` → `%lu` for `uint32_t`) was fixed

### Found by Code Review:

- The EEPROM wrote to the code's own flash address.
- The DAC outputs were never started.
- The ADC only took one reading.
- `SAVE UPPER` / `SAVE LOWER` always stored 0.
- Calibration was reset on every boot.
### Verification:

The board now prints over the USB as soon as the port is opened. It was flashed and tested over COM3:

| Sent           | Reply from Board                                                                 |
| -------------- | -------------------------------------------------------------------------------- |
| ```PING```     | ```PONG```                                                                       |
| ```READ```     | ```APPS1=70, APPS2=72```                                                         |
| ```GETSAVED``` | ```Saved Upper1=2749, Saved Upper2=1914, Saved Lower1=1914, Saved Lower2=1079``` |

> No sensors were connected.

```
=== AER APPS-BSPD firmware, built Sep 30 2026 20:17:04 ===
USB serial OK. Type HELP for commands.
[USB OK] up 12s | APPS1=68 (54mV) APPS2=75 (60mV) | FAULT | DAC1=0 (0mV) DAC2=0 (0mV)
[USB OK] up 13s | APPS1=74 (59mV) APPS2=71 (57mV) | FAULT | DAC1=0 (0mV) DAC2=0 (0mV)
```
## 📝 TODO

> From Jack Varagas

- Need to double check ADC and DAC conversions as well as account for the voltage divider on ADC inputs
