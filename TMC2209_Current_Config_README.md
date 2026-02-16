# TMC2209 Physics-Based Current Configuration

## Overview
This implementation provides accurate TMC2209 stepper driver current configuration based on the actual physics of the chip, rather than using simplified library methods.

## Formula
```
CS = round(32 × I_rms × R_SENSE × √2 / VREF - 1)
```

Where:
- **CS**: Current scale register value (0-31)
- **I_rms**: Desired RMS phase current in Amps
- **R_SENSE**: Sense resistor value in Ohms
- **VREF**: Internal reference voltage (0.325V for TMC2209)

## Configuration Parameters

### Units
- `target_irms_mA`: RMS current in **milliamps** (e.g., 800)
- `rsense_ohm`: Sense resistor in **Ohms** (e.g., 0.100)
- `vref_volts`: Reference voltage in **Volts** (default: 0.325)

### Assumptions
- **Internal current reference**: Uses chip's internal 0.325V reference (not external VREF pin)
- **VSENSE = 0**: High sensitivity mode (0.325V reference path)
- **IHOLD = IRUN / 2**: Recommended for power savings when motor is stationary

## Usage

### Basic Configuration
```cpp
// Configure motor with 800mA, 16 microsteps, SGTHRS=100, R_SENSE=0.1Ω
xMotor->configureTMC(800, 16, 100, 0.100f);
xMotor->initTMC(false);
```

### Expected Output
```
[TMC2209 X] R_SENSE=0.100Ω, VREF=0.325V, IRUN=10, IHOLD=5, I_rms_req=0.800A, I_rms_set=0.790A
```

### Advanced Usage
```cpp
// Get detailed configuration info
auto cfg = xMotor->configureTMC2209Current(800.0f, 0.100f);
if (cfg.clamped) {
    Serial.println("WARNING: Current setting was clamped!");
}
Serial.printf("Actual current: %.3f A\n", cfg.irms_achieved_A);
```

## Test Cases

All test cases use R_SENSE = 0.100Ω:

| Target (mA) | Expected CS | Actual Current (A) |
|-------------|-------------|--------------------|
| 400         | 6           | 0.456              |
| 800         | 10          | 0.790              |
| 1200        | 18          | 1.239              |
| 1600        | 25          | 1.696              |
| 50          | 0 (clamped) | 0.065              |
| 3000        | 31 (clamped)| 2.088              |

## Validation

To enable compile-time tests, add `-DTMC2209_CURRENT_TESTS` to your build flags and call:
```cpp
#ifdef TMC2209_CURRENT_TESTS
testTMC2209CurrentCalculations();
#endif
```

## Why This Matters

The TMCStepper library's `rms_current()` method uses a simplified calculation that may not account for your actual R_SENSE value. This implementation:

1. **Uses your actual R_SENSE value** from hardware
2. **Shows achieved current** so you know exactly what you're getting
3. **Warns when limits are hit** (CS clamped to 0 or 31)
4. **Sets IHOLD properly** for power savings

## Hardware Notes

### R_SENSE Selection
- Typical values: 0.100Ω, 0.110Ω, 0.120Ω
- Check your board schematic or measure resistance between SENSE pins
- Lower R_SENSE → Higher max current capability
- Higher R_SENSE → Better current accuracy

### VREF Modes
- **Internal (default)**: 0.325V, used when `I_scale_analog = 0`
- **External**: Can use external VREF pin if `I_scale_analog = 1`
- This implementation assumes **internal mode**

### Current Limits
With R_SENSE = 0.100Ω and VREF = 0.325V:
- **Minimum**: ~65 mA (CS = 0)
- **Maximum**: ~2088 mA (CS = 31)

## References
- TMC2209 Datasheet Section 9: Current Settings
- Trinamic Application Note AN001: Choosing R_SENSE

