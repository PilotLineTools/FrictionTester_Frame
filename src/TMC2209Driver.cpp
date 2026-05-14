#include "TMC2209Driver.h"

// Datasheet: MRES 0 = 256 native; MRES 1..8 = 128,64,32,16,8,4,2,1 (microsteps per step = 2^MRES interpretation)
uint16_t TMC2209Driver::clampMicrostepsToValid(uint16_t value)
{
    const uint16_t valid[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };
    const int n = sizeof(valid) / sizeof(valid[0]);
    if (value <= valid[0])
        return valid[0];
    if (value >= valid[n - 1])
        return valid[n - 1];
    for (int i = 0; i < n - 1; i++)
        if (value <= (valid[i] + valid[i + 1]) / 2)
            return valid[i];
    return valid[n - 1];
}

TMC2209Driver::TMC2209Driver(TMC2209Stepper *stepper, const char *name)
    : _stepper(stepper), _name(name), _rmsCurrent(800), _microsteps(16), _sgthrs(10), _rsense(0.100f)
{
}

void TMC2209Driver::configure(uint16_t rmsCurrent, uint16_t microsteps, uint8_t sgthrs, float rsense, const TMC2209Config &config)
{
    _rmsCurrent = rmsCurrent;
    _microsteps = microsteps;
    _sgthrs = sgthrs;
    _rsense = rsense;
    _config = config;
}

void TMC2209Driver::init(bool use_velocity_mode)
{
    if (_stepper == nullptr)
        return;

    _stepper->begin();
    USBSerial.printf("[%s] Initializing TMC2209\n", _name);

    uint8_t initial_ifcnt = _stepper->IFCNT();
    USBSerial.print("Initial IFCNT: ");
    USBSerial.println(initial_ifcnt);

    // Configure SpreadCycle/StealthChop mode based on config
    // Do this AFTER begin() to ensure it sticks
    delay(10); // Let driver settle

    // Use the library's built-in method to set en_spreadcycle
    _stepper->en_spreadCycle(_config.en_spreadcycle);

    if (_config.en_spreadcycle)
    {
        USBSerial.printf("[%s] GCONF: SpreadCycle mode enabled\n", _name);
    }
    else
    {
        USBSerial.printf("[%s] GCONF: StealthChop mode enabled\n", _name);
    }

    // Verify the write
    delay(10);
    bool spreadcycle_set = _stepper->en_spreadCycle();
    USBSerial.printf("[%s] GCONF readback: en_spreadcycle=%d (expected %d)\n",
                     _name, spreadcycle_set, _config.en_spreadcycle);

    // Use physics-based current calculation for ALL motors to ensure accurate IRUN/IHOLD
    configureCurrent((float)_rmsCurrent, _rsense);

    _stepper->TPOWERDOWN(0x14);

    CHOPCONF_t chopconf{0};
    chopconf.toff = _config.toff;
    chopconf.tbl = _config.tbl;
    // MRES: 0 = 256 native; 1..8 = 128,64,32,16,8,4,2,1
    chopconf.mres = (_microsteps >= 256) ? 0 : (8 - __builtin_ctz((unsigned)_microsteps));
    chopconf.intpol = 1;
    chopconf.vsense = 0;
    _stepper->CHOPCONF(chopconf.sr);
    _stepper->TPWMTHRS(_config.tpwmthrs);
    _stepper->TCOOLTHRS(_config.tcoolthrs);
    _stepper->SGTHRS(_sgthrs);

    // delay(100);

    // uint32_t drv_status = _stepper->DRV_STATUS();
    // uint8_t cs_actual = drv_status & 0x1F;
    // USBSerial.printf("[%s] CS_ACTUAL: %d\n", _name, cs_actual);
}

void TMC2209Driver::setMicrosteps(uint16_t microsteps)
{
    if (microsteps == 0)
        microsteps = 1;
    _microsteps = microsteps;

    if (_stepper == nullptr)
        return;

    CHOPCONF_t chopconf{0};
    chopconf.toff = _config.toff;
    chopconf.tbl = _config.tbl;
    chopconf.mres = (_microsteps >= 256) ? 0 : (8 - __builtin_ctz((unsigned)_microsteps));
    chopconf.intpol = 1;
    chopconf.vsense = 0;
    _stepper->CHOPCONF(chopconf.sr);
    // USBSerial.printf("[%s] Microsteps set to %u (mres=%d)\n", _name, (unsigned)_microsteps, chopconf.mres);
}

uint16_t TMC2209Driver::getMicrosteps() const
{
    if (_stepper == nullptr)
        return _microsteps;
    uint8_t mres = _stepper->mres();
    if (mres > 8)
        mres = 8;
    // MRES 0 = 256; MRES 1..8 = 128,64,32,16,8,4,2,1
    return (mres == 0) ? 256 : (uint16_t)(1 << (8 - mres));
}

void TMC2209Driver::setRmsCurrent(uint16_t rmsCurrent)
{
    _rmsCurrent = rmsCurrent;
    if (_stepper)
        configureCurrent((float)_rmsCurrent, _rsense);
    USBSerial.printf("[%s] RMS current set to %u mA\n", _name, _rmsCurrent);
}

/**
 * Configure TMC2209 current settings based on physics
 *
 * Formula: CS = round(32 * I_rms * R_SENSE * sqrt(2) / VREF - 1)
 *
 * Assumptions:
 * - Internal current reference (VREF = 0.325V typical for TMC2209)
 * - VSENSE = 0 (high sensitivity mode, 0.325V reference)
 * - IHOLD = IRUN / 2 (recommended for power savings)
 *
 * @param target_irms_mA Target RMS phase current in milliamps
 * @param rsense_ohm Sense resistor value in Ohms (e.g., 0.100)
 * @param vref_volts Internal reference voltage (default 0.325V)
 * @return TMC2209CurrentConfig struct with computed values
 */
TMC2209CurrentConfig TMC2209Driver::configureCurrent(float target_irms_mA, float rsense_ohm, float vref_volts)
{
    TMC2209CurrentConfig cfg;
    cfg.rsense_ohm = rsense_ohm;
    cfg.vref_used_V = vref_volts;
    cfg.clamped = false;

    // Validation
    if (rsense_ohm <= 0.0f)
    {
        USBSerial.println("[TMC2209] ERROR: R_SENSE must be > 0");
        cfg.cs_irun = 0;
        cfg.cs_ihold = 0;
        cfg.irms_requested_A = 0.0f;
        cfg.irms_achieved_A = 0.0f;
        cfg.clamped = true;
        return cfg;
    }

    if (target_irms_mA <= 0.0f)
    {
        USBSerial.println("[TMC2209] WARNING: Target current <= 0");
    }

    // Convert mA to A
    float irms_A = target_irms_mA / 1000.0f;
    cfg.irms_requested_A = irms_A;

    // Calculate current scale (CS)
    // CS = 32 * I_rms * R_SENSE * sqrt(2) / VREF - 1
    float cs_float = 32.0f * irms_A * rsense_ohm * 1.41421356f / vref_volts - 1.0f;
    int cs_raw = (int)roundf(cs_float);

    // Clamp to valid range [0, 31]
    if (cs_raw < 0)
    {
        cfg.cs_irun = 0;
        cfg.clamped = true;
        USBSerial.printf("[TMC2209] WARNING: CS clamped to 0 (requested %.1f mA too low)\n", target_irms_mA);
    }
    else if (cs_raw > 31)
    {
        cfg.cs_irun = 31;
        cfg.clamped = true;
        USBSerial.printf("[TMC2209] WARNING: CS clamped to 31 (requested %.1f mA too high)\n", target_irms_mA);
    }
    else
    {
        cfg.cs_irun = (uint8_t)cs_raw;
    }

    // IHOLD = IRUN / 2
    cfg.cs_ihold = cfg.cs_irun / 2;

    // Calculate achieved RMS current
    // I_rms_achieved = ((CS + 1) / 32) * (VREF / (R_SENSE * sqrt(2)))
    cfg.irms_achieved_A = ((cfg.cs_irun + 1.0f) / 32.0f) * (vref_volts / (rsense_ohm * 1.41421356f));

    // Program the driver if attached
    if (_stepper != nullptr)
    {
        // Use internal current reference (not external VREF pin)
        _stepper->I_scale_analog(false);

        // Set IRUN and IHOLD
        _stepper->irun(cfg.cs_irun);
        _stepper->ihold(cfg.cs_ihold);
        _stepper->iholddelay(10); // Reasonable delay before reducing to hold current

        USBSerial.printf("[TMC2209 %s] R_SENSE=%.3fOhm, VREF=%.3fV, IRUN=%d, IHOLD=%d, I_rms_req=%.3fA, I_rms_set=%.3fA%s\n",
                         _name, rsense_ohm, vref_volts, cfg.cs_irun, cfg.cs_ihold,
                         cfg.irms_requested_A, cfg.irms_achieved_A,
                         cfg.clamped ? " [CLAMPED]" : "");
    }

    return cfg;
}

void TMC2209Driver::setVactual(int32_t vactualRegisterValue)
{
    if (_stepper == nullptr)
        return;
    // VACTUAL = desired_µsteps_per_second × (2²⁴ / fCLK); sign = direction, 0 = STEP/DIR
    _stepper->VACTUAL(vactualRegisterValue);
}

// Set Rpm Actual
void TMC2209Driver::setRpmActual(float rpm)
{
    if (_stepper == nullptr)
        return;
    
    // Calculate VACTUAL from RPM using configured microsteps.
    // 200 full steps/rev for a typical 1.8° motor.
    float ustepsPerRev = 200.0f * (float)_microsteps;
    float ustepsPerSec = (fabsf(rpm) / 60.0f) * ustepsPerRev;
    float vactualScale = (float)(1UL << _scale_exp) / (float)_fclk_hz;
    int32_t vactual = (int32_t)(ustepsPerSec * vactualScale + 0.5f);

    // Apply direction
    if (rpm < 0.0f)
        vactual = -vactual;

    _stepper->VACTUAL(vactual);
    //USBSerial.printf("[%s] RPM set to %.1f -> VACTUAL=%d (usteps/s=%.1f)\n", _name, rpm, vactual, ustepsPerSec);

} 

uint16_t TMC2209Driver::getSGResult()
{
    if (_stepper)
        return _stepper->SG_RESULT();
    return 0;
}

uint8_t TMC2209Driver::getSGTHRS() const
{
    return _sgthrs;
}

uint32_t TMC2209Driver::getTstep() const
{
    if (_stepper)
        return _stepper->TSTEP();
    return 0;
}

#ifdef TMC2209_CURRENT_TESTS
// Compile-time test cases (enable with -DTMC2209_CURRENT_TESTS)
void testTMC2209CurrentCalculations()
{
    // Create a null stepper for testing (won't actually program hardware)
    TMC2209Driver testDriver(nullptr, "TEST");

    USBSerial.println("\n=== TMC2209 Current Configuration Tests ===");

    // Test 1: 400 mA with 0.100Ω → expect CS≈6
    auto cfg1 = testDriver.configureCurrent(400.0f, 0.100f);
    USBSerial.printf("Test 1 (400mA): CS=%d, I_set=%.3fA (expected CS≈6, I≈0.456A)\n",
                     cfg1.cs_irun, cfg1.irms_achieved_A);

    // Test 2: 800 mA with 0.100Ω → expect CS≈10
    auto cfg2 = testDriver.configureCurrent(800.0f, 0.100f);
    USBSerial.printf("Test 2 (800mA): CS=%d, I_set=%.3fA (expected CS≈10, I≈0.790A)\n",
                     cfg2.cs_irun, cfg2.irms_achieved_A);

    // Test 3: 1200 mA with 0.100Ω → expect CS≈18
    auto cfg3 = testDriver.configureCurrent(1200.0f, 0.100f);
    USBSerial.printf("Test 3 (1200mA): CS=%d, I_set=%.3fA (expected CS≈18, I≈1.239A)\n",
                     cfg3.cs_irun, cfg3.irms_achieved_A);

    // Test 4: 1600 mA with 0.100Ω → expect CS≈25
    auto cfg4 = testDriver.configureCurrent(1600.0f, 0.100f);
    USBSerial.printf("Test 4 (1600mA): CS=%d, I_set=%.3fA (expected CS≈25, I≈1.696A)\n",
                     cfg4.cs_irun, cfg4.irms_achieved_A);

    // Test 5: Lower clamp (50 mA)
    auto cfg5 = testDriver.configureCurrent(50.0f, 0.100f);
    USBSerial.printf("Test 5 (50mA - lower clamp): CS=%d, clamped=%d\n",
                     cfg5.cs_irun, cfg5.clamped);

    // Test 6: Upper clamp (3000 mA)
    auto cfg6 = testDriver.configureCurrent(3000.0f, 0.100f);
    USBSerial.printf("Test 6 (3000mA - upper clamp): CS=%d, clamped=%d\n",
                     cfg6.cs_irun, cfg6.clamped);

    USBSerial.println("=== Tests Complete ===\n");
}
#endif
