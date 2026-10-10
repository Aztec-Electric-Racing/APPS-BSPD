# Changes of note from usb-fix branch & the nucleo board benchtest branch


> [All Changes In Main](main-changes.txt)

## Overview:

`new-bench-test` branch introduces **brake monitoring**, **additional APPS fault detection**, and a **throttle/brake inhibit system**.

### Why?

The old board had hardware checks that performed calculations physically. The new nucleo board lacks these functions so all operations now need to be performed through our software.The purpose is essentially the same, we are just adding new code to compensate for the lack of hardware implementation of checks.

### Major Changes found by Code Review:

| Priority | Change | Significance |
|------|----------|-----------------|
|<span style="color:red">**HIGH**</span>|Throttle/brake inhibit system | New safety output control
|<span style="color:red">**HIGH**</span>| APPS mismatch detection | Detects disagreement between pedal sensors |
|<span style="color:red">**HIGH**</span>|ADC voltage-divider compensation | Changes how sensor readings are interpreted |
|Medium| THird ADC Channel | Adds brake sensor monitoring |
|Medium          |USART2 Communication        | Adds UART console alongside USB             |
|Low | Expanded diagnostics | More detailed status reporting |

### Throttle/brake inhibit system

New function called `UpdateThrottleBrakeLatch`.

- Determines whether the system should activate the inhibit output
- Did not exist in main.

> **Logic:**
> 
> - Check APPS sensor for validity and fault status
> - Invalid/APPS fault -> Latch Inhibit
> - Sensors Valid -> Check brake and Throttle
> 
> **Brake + Throttle >= 25%:**
> Activate Inhibit Latch
> 
> Clear latch when permitted and throttle <= 5%. Must have valid sensors and no APPS fault.

### USART2 Communication Added
- Previously, console communication was handeld through USB CDC.
- New utilizes `USART2`.
