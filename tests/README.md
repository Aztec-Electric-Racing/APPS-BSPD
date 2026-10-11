# APPS-BSPD Software Testing

## Testing Architecture:

This project utilizes the **Unity C Testing Framework** for automated unit testing and CMake for building the test executables.

## Prerequisites:

- CMake 3.20 or newer
- Native C Compiler
  - _MinGW-w64 recommended_
- [Unity C Testing Framework](https://github.com/ThrowTheSwitch/Unity)
  - _Included in the test directory._

## Current Test Coverage:

### ADC Conversion:

#### Conversion Formula:

$$
\text{SensorCounts} = \min\left(4095,\ \operatorname{round}\left(\frac{\text{ADCCounts} \times 100}{72}\right)\right)
$$

**Source:** [Core/Src/apps_conversion.c](Core/Src/apps_conversion.c)

**Test file:** [tests/unit/test_adc_conversion.c](tests/unit/test_adc_conversion.c)

**Function:** `ADCtoSensorCounts()`

The ADC conversion tests verify the software's voltage divider compensation calculation, rounding behavior, and output saturaiton.

The current implementation assumes a 72% input voltage divider ratio and a maximum sensor count of `4095`.

|Test|Purpose|
|----------------|--------------|
|test_adc_zero|Verify zero-input behavior.|
|test_adc_scaling|Verify nominal conversion scaling.|
|test_adc_midrange|Verify intermediate conversion results.|
|test_adc_clamping|Verify maximum output saturation.|
|test_adc_near_saturation|Verify behavior near the saturation boundary.|
|test_adc_above_saturation|Verify output saturation for larger inputs.|
|test_adc_rounding|Verify rounding behavior.|

### Important Limitations Note:

The current testing infrastructure does not verify any physical sensor accuracy or calibration. It currently **only** executes software on a development computer and does not verify:

- Physical sensor accuracy or calibration
- Actual voltage-divider component values
- STM32 ADC peripheral  behavior
- Real-time performance on the microcontroller
- Electrical safety outputs or shutdown circuitry

## Notes:

Disagreement detection is not implemented. 