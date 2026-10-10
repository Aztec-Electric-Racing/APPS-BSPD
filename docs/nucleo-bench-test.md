# Nucleo-F446RE APPS and brake-latch bench test

This procedure exercises the **current bench firmware** on a Nucleo-F446RE using two adjustable DC voltage signals for APPS1/APPS2, a switch or jumper for the brake input, and a meter or logic probe on PB1. It is a low-voltage logic test. It does not connect to an inverter, contactor, shutdown circuit, or vehicle, and it does not prove that motor torque is removed.

The Nucleo wiring below is a bench harness. It is not a claim that the custom APPS PCB uses the same pin assignments.

## What this test is intended to show

The firmware samples two analog inputs and converts each sample into a calibrated pedal percentage. It also reads a digital brake input. When brake is pressed while its calculated throttle is at least 25%, the firmware latches an **inhibit request** on PB1. PB1 stays low after the brake is released until both sensor readings are valid and the calculated throttle is at or below 5%.

PB1 is only a logic output during this test. Low means “inhibit requested”; high means “inhibit released.” A meter, LED circuit, or scope can show the signal changing, but none of those makes the motor stop. The DAC outputs are a separate pair of outputs and are not forced to zero by the brake latch. The DAC commands go to zero for an APPS out-of-range fault.

## Bench connections

| Function | Nucleo pin | Bench connection |
| --- | --- | --- |
| APPS1 signal | PC4 / ADC1_IN14, CN10 pin 34 | Adjustable 0–3.3 V DC signal source 1 |
| APPS2 signal | PC5 / ADC1_IN15, CN10 pin 6 | Separate adjustable 0–3.3 V DC signal source 2 |
| Brake input | PB0, CN7 pin 34 | Switch/jumper to Nucleo GND means pressed; open means released |
| Inhibit output | PB1, CN10 pin 24 | Meter/logic probe/scope input only; low = inhibit |
| Ground reference | Nucleo GND | Common ground for both signal sources, brake switch, and measuring equipment |
| DAC1 / DAC2 | PA4 / PA5 | Optional voltage measurements; leave disconnected from other equipment unless measuring with a high-impedance meter/scope |

**Signal-source wiring:** connect each source output to its APPS input and connect the source return/ground to Nucleo GND. Both sources must share the Nucleo ground reference. Before connecting, set each source to 0 V and verify its output with a meter. Keep both APPS pins between 0 and 3.3 V; never apply 5 V. Use current-limited, low-voltage bench outputs. Do not connect a non-isolated supply in a way that creates an unintended earth-ground path; ask the lab lead if unsure.

The brake switch is active-low: PB0 has an internal pull-up, so an open switch reads released and connecting PB0 to GND reads pressed. Do not apply an external voltage to PB0 for this procedure.

The Nucleo USB cable plugs into the **ST-LINK USB connector**. The terminal appears through the ST-LINK virtual COM port, using USART2 at 115200 baud, 8 data bits, no parity, and 1 stop bit (115200 8N1). This is different from the custom PCB's USB CDC connection. Send each command followed by Enter.

During this procedure, leave all vehicle-side wiring disconnected. In particular, do not connect PB1 to a motor controller, inverter, contactor, or vehicle shutdown circuit. PA5 is connected to the Nucleo user LED LD2, which can load the DAC2 output; account for that if measuring PA5.

## How to choose the two APPS voltages

The firmware’s `STATUS` report shows `Throttle: n.n%`. For the two channels to represent the **same pedal position**, set each channel to the matching percentage of its own calibrated voltage span. The two voltages may be different because the sensors may have different transfer functions.

First choose and record a low endpoint and a high endpoint for **each** channel:

| Channel | Released endpoint | Full-travel endpoint |
| --- | --- | --- |
| APPS1 / PC4 | `V1_low` | `V1_high` |
| APPS2 / PC5 | `V2_low` | `V2_high` |

Then calculate the input for a requested percentage `P`:

```text
APPS1 voltage = V1_low + (P / 100) × (V1_high - V1_low)
APPS2 voltage = V2_low + (P / 100) × (V2_high - V2_low)
```

Example: if APPS1 is calibrated from 0.60 V to 2.20 V, its 25% point is 1.00 V. If APPS2 is calibrated from 0.90 V to 2.10 V, its 25% point is 1.20 V. Enter those separate voltages to represent both sensors reporting 25% travel.

The formula uses measured voltages at the Nucleo pins. The project has a 72% divider-compensation setting, but because calibration and measurements use the same channel path, the normalized percentage is based on the endpoints you actually saved. Confirm the source voltage at the pins with a meter; do not rely only on the bench supply display.

For this bench setup, choose a high endpoint below approximately **2.37 V** (for example, 2.0 V) and leave room above it for an out-of-range test. This firmware divides the ADC reading by 0.72 and caps the compensated result at 4095; with a 3.3 V ADC reference, readings above about 2.37 V hit that cap. Voltages up to 3.3 V are electrically within the pin's signal range, but the compensated software value will saturate, making higher over-range tests indistinguishable.

## Setup and calibration

1. Connect the Nucleo to the host computer through the ST-LINK USB connector. Connect both signal-source grounds to Nucleo GND, both source outputs to PC4/PC5, and your meter/probe ground to Nucleo GND. Leave the brake switch open (released).
2. Set both source outputs to 0 V before enabling them. Then set them to the chosen, safe released-pedal voltages `V1_low` and `V2_low`, each within 0–3.3 V. Keep each high calibration endpoint below approximately 2.37 V as described above.
3. Open the ST-LINK virtual COM port at 115200 8N1. Send `PING` and confirm `PONG`; send `HELP` to confirm the console is responding.
4. With both inputs still at their released endpoints, send `SAVE LOWER`.
5. Set each source to its chosen full-travel endpoint (`V1_high`, `V2_high`) and send `SAVE UPPER`.
6. Send `GETSAVED` and record all four saved values. Send `STATUS`; check that APPS is `OK` and throttle is approximately 100% at the upper endpoints.
7. Return both inputs to their low endpoints. Send `STATUS`. The startup inhibit latch should now clear because valid throttle is at or below 5%; confirm `Inhibit: HIGH` and measure PB1 high.

`SAVE LOWER` and `SAVE UPPER` write calibration to flash. Do not send them repeatedly during the functional cases. Select repeatable endpoints with enough room below the 2.37 V compensation limit and enough room above the upper endpoint for over-range tests. If a save says the calibration is invalid, the lower value for that sensor is not below its upper value; correct the source settings and recalibrate.

## Functional cases: exact inputs and expected results

For each percentage row, use the interpolation formula above to set **both** sources to that channel's corresponding voltage. After changing the voltages or brake switch, wait briefly for the continuous ADC/DMA sampling and then send `STATUS`. Record both the reported state and the measured PB1 voltage. The STATUS text reports calculated DAC commands, not measured DAC-pin voltages.

| Case | Set the inputs | Expected result from this firmware |
| --- | --- | --- |
| Establish clear state | APPS1 = 0%; APPS2 = 0%; brake open | APPS `OK`, throttle about 0.0%, latch clears, `Inhibit: HIGH`, PB1 high |
| Normal released | Keep both at 0%; brake open | Same as above; this is the normal released-pedal state |
| Brake below trip point | Set both to 24%; connect PB0 to GND; send `STATUS` | Brake `PRESSED`; no new trip; if latch was already clear, `Inhibit: HIGH`, PB1 high |
| Brake at the implemented trip boundary | Set both to about 25%; connect PB0 to GND. Adjust slightly until `STATUS` reports at least 25.0% | Current code trips when its integer throttle value reaches **250 permille** or higher: latch sets, `Inhibit: LOW`, PB1 low. ADC and integer rounding may require slightly more than a dial-calculated 25.0% |
| Brake above trip point | Set both to 30%; connect PB0 to GND | Latch sets, `Inhibit: LOW`, PB1 low. DAC commands may remain nonzero because the APPS readings are still valid |
| Brake released, throttle still high | After a trip, open PB0 but keep both inputs at 30% | Brake reports `released`; latch remains set; PB1 remains low |
| Latch hold above clear point | Keep brake open; set both to 6%, then 10% | Latch remains set and PB1 stays low because throttle is above 5% |
| Clear latch | Keep brake open; set both inputs to 4% | Current code clears at **5.0% or below** (`<= 50` permille); PB1 returns high |
| APPS below calibrated range | From a cleared state, set either one channel just below its saved lower endpoint; keep the other channel in range | APPS `FAULT`; DAC commands become zero; latch sets; PB1 low |
| APPS above calibrated range | From a cleared state, set either one channel just above its saved upper endpoint, without exceeding 3.3 V | APPS `FAULT`; DAC commands become zero; latch sets; PB1 low |
| Recover from APPS range fault | Return both inputs inside their saved ranges but leave throttle above 5% | APPS returns to `OK`, but latch remains set and PB1 remains low |
| Clear after APPS range fault | With both inputs valid, set both to 4% | Latch clears and PB1 returns high |
| Brake at low throttle | From a cleared state, set both to 10%; press and release brake | No brake/APPS trip because throttle is below 25%; PB1 stays high |

### Important threshold boundary to document

The current code uses `throttlePermille >= 250` to trip and `throttlePermille <= 50` to clear. That means it trips at exactly 25% and clears at exactly 5%. The rule wording supplied for this project says “more than 25%” and “less than 5%.” The exact equality cases therefore need a team decision against the official applicable rulebook. The table above documents what the **current code** does; it is not a ruling that the boundary implementation is compliant.

## Redundant-sensor mismatch demonstration (known gap, not a pass case)

This is a useful test to expose what is not implemented yet:

1. Start from a cleared state: valid readings at or below 5%, brake released, PB1 high.
2. Keep the brake released.
3. Set APPS1 to 10% of its own saved range and APPS2 to 60% of its own saved range. Both voltages must remain within their saved limits.
4. Send `STATUS` and record the result.

Expected from **current code**: the firmware chooses the higher normalized reading for `Throttle`, so it should report about 60%. It does not perform a channel-to-channel mismatch comparison, so it will not set APPS `FAULT` or latch PB1 just because those readings differ. With brake released and an already-clear latch, PB1 remains high. This is a documented gap relative to the intended APPS plausibility behavior; do not record this as a passing safety test.

If you press the brake during this mismatch example, the 60% higher channel will trigger the separate brake/APPS latch. That would test brake plausibility, not prove APPS mismatch detection.

## What part of the code each test exercises

| Code | What it does | Bench observation |
| --- | --- | --- |
| `HAL_ADC_ConvCpltCallback()` in `Core/Src/main.c` | Copies the latest DMA ADC samples into `apps1` and `apps2` | Changing PC4/PC5 changes `READ`/`STATUS` ADC values |
| `AdcToSensorCounts()` in `Core/Src/main.c` | Applies the configured 72% divider compensation | Converts raw ADC readings into the values compared with saved calibration |
| `CalibrationIsValid()` and `SAVE LOWER/UPPER` in `Core/Src/main.c` | Checks and stores the calibration endpoints | `GETSAVED` shows the endpoints used by the percentage formula |
| Main loop sensor check in `Core/Src/main.c` | Checks each channel against its own saved lower/upper range | Inputs outside either range show APPS `FAULT` and zero DAC commands |
| `GetThrottlePermille()` in `Core/Src/main.c` | Converts each valid sensor to 0–1000 permille and selects the higher value | `STATUS` throttle follows the larger normalized channel |
| `UpdateThrottleBrakeLatch()` in `Core/Src/main.c` | Reads active-low PB0, sets/clears the latch, and writes active-low PB1 | Brake+throttle trip, latch hold, and low-throttle clear cases |
| `MX_GPIO_Init()` in `Core/Src/main.c` | Configures PB0 as pulled-up input and PB1 as output, initially low | Explains why open PB0 means released and low PB1 means inhibit |
| `SendStatusLine()` in `Core/Src/main.c` | Formats APPS, brake, throttle, inhibit, and DAC-command text | The `STATUS` response used to record each case |

## How to interpret the important STATUS fields

- `APPS: OK` means both readings are inside their own saved calibration ranges. It does **not** mean the two sensor readings agree with each other.
- `APPS: FAULT` means at least one reading is outside its own range.
- `Throttle: n.n%` is the higher of the two channels after each is normalized to its own saved endpoints.
- `Brake: PRESSED` means PB0 is electrically low (connected to GND).
- `Inhibit: LOW` means PB1 is being driven low by the bench firmware. It does not mean a motor was stopped.
- `DAC commands` are values requested by software. Measure PA4/PA5 separately if you need actual pin voltages. The brake latch itself does not zero these DAC values.

## Recording results

For each run, record:

| Record | Value |
| --- | --- |
| Firmware revision / build identifier | |
| Saved APPS1 lower / upper values | |
| Saved APPS2 lower / upper values | |
| APPS1 input voltage at PC4 | |
| APPS2 input voltage at PC5 | |
| Brake switch state | |
| `STATUS` throttle / APPS / inhibit result | |
| Measured PB1 voltage | |
| Optional measured PA4 / PA5 voltages | |
| Notes or unexpected behavior | |

Repeat trip and clear cases several times. ADC quantization and source accuracy can move readings slightly around a threshold, so record the measured voltages and reported throttle rather than assuming the supply dial is exact.

## What this bench test does not prove

- It does not prove live APPS1/APPS2 mismatch detection or the required 100 ms response. The mismatch case above intentionally demonstrates the current gap.
- It does not prove open-wire, short-wire, stuck-sensor, or brake-sensor failure detection. A fixed voltage source does not automatically simulate those faults; each fault needs a defined, safe wiring test.
- It does not prove response timing. The main loop also services serial commands and status output; timing must be measured under worst-case communication activity.
- It does not prove PB1 is wired to the custom PCB's intended pin or that a vehicle controller acts on PB1.
- It does not prove motor torque shutdown, shutdown-circuit operation, BOTS, or BSPD operation.
- It does not validate the vehicle pedal's springs, positive stop, or 0% return-to-zero behavior.

The custom PCB uses its own schematic and may have different signal pins, voltage scaling, output circuitry, and USB wiring. Verify those against that board's schematic before transferring any Nucleo pin mapping or bench conclusion to the vehicle.

## Pinout reference

![Nucleo-F446RE bench pinout](image.png)
