# Program Overview Notes

## Purpose

The purpose of this program is to take in readings from APPS and provides a check for if they are within acceptable values (safe limits). If they are invalid and within unacceptable values, provide a failsafe.

## Differences from original usb-fix:

> [See notes on usb-fix changes](./usb-fix-changes.md)

## Main Loop Logic

**Execution Flow:**

```c
// TAKES IN TWO SENSOR READINGS AND SAVING COPIES
    uint16_t rawS1 = apps1;
    uint16_t rawS2 = apps2;
// COPIES READINGS INTO VARIABLES
    checkedApps1 = rawS1;
    checkedApps2 = rawS2;
 // PASSES EACH RAW ADC READINGS THRU ADC TO SENSOR COUNTS
    uint16_t s1 = AdcToSensorCounts(rawS1);
    uint16_t s2 = AdcToSensorCounts(rawS2);
```

### AdcToSensorCounts Method:
- Takes in raw ADC reading and scales to compensate for the input voltage divider.
  - _Essentially, ADC is measured after the voltage divider. This method gives the equivalent reading before the divider._
- Nothing is allowed to go above ```ADC_MAX_COUNTS```

> This method prevents the result from exceeding the ADC range. It clamps the value to a fixed ```4095```

### sensorsValid variable:

- A T/F variable checking for if both sensors are currently within acceptable readings.

```c
ee.sens1Lower <= s1       // s1 isn't too low
s1 <= ee.sens1Upper       // s1 isn't too high

ee.sens2Lower <= s2       // s2 isn't too low
s2 <= ee.sens2Upper       // s2 isn't too high
```

### if else sensorsValid variable check:

##### Both Sensors are Valid:

```c 
throttlePermille = GetThrottlePermille(s1, s2)
```

> GetThrottlePermille takes two APPS sensor readings and converts them into a single throttle position from 0-1000, ```1000``` being  throttle.
- Calculates the throttle position using the two sensor readings. ```Permille = "Per Thousand"```
- Finds the absolute difference between `sens1Lower` & `sens2Lower` _(Lower calibrated values of two sensors)_
- Calculates the acceptable toldwerance, how much sensor disagreement is considered acceptable through `SENS_ACCEPTABLE_DIFF` *(Currently sitting at 10%)*
- Then calculates two DAC outputs through `ClampDac`
- Clears the APPS fault (setting to `0`)
  - Because sensors passed validation: There is no APPS fault, hence 0.

##### Either Sensor is Invalid:

- Sets throttlePermille to `0`
- Sets DAC1,2 to 0
- Sets APPS fault to 1 (on)

### After if else:

#### UpdateThrottleBrakeLatch(sensorsValid):

**UpdateThrottleBrakeLatch:**
- This is a safety latch between the accelerator pedal and the brake pedal.
- If the throttle is too high while the brake is pressed, or if the APPS sensors become invalid, this method provides a safety latch. It stays active until the throttle comes back down to a safe value.

```c 
HAL_DAC_SetValue(
    &hdac,            // Which DAC peripheral
    DAC_CHANNEL_1,    // Which DAC channel
    DAC_ALIGN_12B_R,  // 12-bit, right-aligned value
    dac1Out           // Value to output
);
```
- Sends values calculated from `dac1Out` & `dac2Out` and sends them to the STM32's DAC outputs.
  - Writes the calculated 12-bit DAC values to DAC channels `1` & `2`, producing corresponding analog voltages on `PA4` and `PA5`

### USB/Serial Console Communication (If Else)



> End of main loop.