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
    void  setTargetAzimuth(float azDeg);
    void  updatePan();
    void  updatePanWithPosition(float currentAzDeg);
    void  runPan();

    // ---- Telemetry / HIL helpers ----
    float getStepperPositionDeg();
    float getPidOutput() const;
    float getStepperSpeed();
    float getTargetAzimuth() const;

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
    unsigned long _lastPidUs;

    // AS5048A helpers
    uint16_t _spiTransfer16(uint16_t cmd);
    uint16_t _buildReadCommand(uint16_t address);
    static uint8_t _evenParity(uint16_t value);
    static float _angleDiffDeg(float target, float current);
};

#endif // ACTUATOR_CONTROL_H
