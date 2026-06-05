// ============================================================================
// ActuatorControl.h — Pan/Tilt Actuator Hardware Abstraction
// Cursr-V Antenna Tracker
//
// Pan  (Azimuth):   NEMA17 stepper via TB6600 driver + AS5048A encoder
//                   Driven by AccelStepper with PID speed control
// Tilt (Elevation): DS51150 servo via ESP32-S3 MCPWM at 300 Hz
//
// All angle I/O is in DEGREES.
// ============================================================================
#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <AccelStepper.h>
#include <SPI.h>
#include "driver/mcpwm.h"
#include "config.h"

// --------------------------------------------------------------------------
//  PID Gains for Pan Speed Controller
// --------------------------------------------------------------------------
struct PanPID {
    float Kp;
    float Ki;
    float Kd;
};

// --------------------------------------------------------------------------
//  Actuators Class
// --------------------------------------------------------------------------
class Actuators {
public:
    Actuators();

    /// Initialise all hardware.
    void begin();

    // ---- Pan (Azimuth) -----
    void  setPanNorthOffset(double northHeadingDeg);
    float readEncoderDeg();
    float getPanAngleDeg();
    // Direction-corrected, North-referenced azimuth from an already-read raw
    // count (no SPI access) — lets telemetry reuse its single per-tick read.
    float panAngleFromRaw(uint16_t raw) const;
    void  setTargetAzimuth(float azDeg);
    void  updatePan();
    void  updatePanWithPosition(float currentAzDeg);
    void  runPan();

    // ---- Telemetry / HIL helpers ----
    float getStepperPositionDeg();
    float getPidOutput() const;
    float getStepperSpeed();
    float getTargetAzimuth() const;

    // ---- Raw debug accessors (values straight off the actuators) ----
    uint16_t readEncoderRaw();        // AS5048A 14-bit angle counts [0,16383]
    long     getStepCount();          // absolute stepper step count
    uint32_t getTiltPulseUs() const;  // last commanded servo pulse width (µs)

    // ---- Tilt (Elevation) ----
    void  setTiltAngle(float elevDeg);

    // ---- PID Tuning ----
    void  setPanPID(float Kp, float Ki, float Kd);

private:
    AccelStepper _stepper;
    float _panOffset;
    PanPID _pid;
    float  _targetAzDeg;
    float  _integralError;
    float  _prevError;
    float  _lastPidOutput;
    float  _lastCommandedSPS;
    uint32_t _lastTiltPulseUs;
    unsigned long _lastPidUs;

    // AS5048A helpers
    uint16_t _spiTransfer16(uint16_t cmd);
    uint16_t _buildReadCommand(uint16_t address);
    static uint8_t _evenParity(uint16_t value);
    static float _angleDiffDeg(float target, float current);
    // Raw 14-bit count → direction-corrected degrees [0,360) (no North offset).
    static float _dirCorrectedDeg(uint16_t raw);

    // Shared PID core. Both updatePan() and updatePanWithPosition() compute
    // their own positional error, then delegate the deadband + anti-windup +
    // slew-limited PID math here. Owns the dt calculation from _lastPidUs.
    void _runPid(float error);
};

#endif // ACTUATOR_CONTROL_H