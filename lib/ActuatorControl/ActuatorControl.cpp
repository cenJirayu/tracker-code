// ============================================================================
// ActuatorControl.cpp — Pan/Tilt Actuator Implementation
// Cursr-V Antenna Tracker
//
// Uses the legacy driver/mcpwm.h API (ESP-IDF v4.x bundled with Arduino-ESP32)
// ============================================================================
#include "ActuatorControl.h"
#include <Arduino.h>

// MCPWM unit and timer used for the tilt servo
static constexpr mcpwm_unit_t  SERVO_MCPWM_UNIT  = MCPWM_UNIT_0;
static constexpr mcpwm_timer_t SERVO_MCPWM_TIMER = MCPWM_TIMER_0;

// ============================================================================
Actuators::Actuators()
#ifdef USE_ULN2003
    // 28BYJ-48: Half-step 4-wire mode
    // AccelStepper pin order: IN1, IN3, IN2, IN4 for correct phase sequence
    : _stepper(AccelStepper::HALF4WIRE, PIN_IN1, PIN_IN3, PIN_IN2, PIN_IN4)
#else
    // NEMA17 + TB6600: Step/Dir driver mode
    : _stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR)
#endif
    , _panOffset(0.0f)
    , _pid{PAN_PID_KP, PAN_PID_KI, PAN_PID_KD}
    , _targetAzDeg(0.0f)
    , _integralError(0.0f)
    , _prevError(0.0f)
    , _lastPidOutput(0.0f)
    , _lastCommandedSPS(0.0f)
    , _lastPidUs(0)
{}

// ============================================================================
void Actuators::begin() {
    // --- SPI for AS5048A encoder ---
    pinMode(PIN_ENC_CS, OUTPUT);
    digitalWrite(PIN_ENC_CS, HIGH);
    SPI.begin();

    // Flush SPI pipeline (AS5048A needs two transactions; first read is stale)
    _spiTransfer16(_buildReadCommand(AS5048A_CMD_ANGLE));
    delayMicroseconds(10);
    _spiTransfer16(0x0000);

    // --- TB6600 Stepper Driver ---
    pinMode(PIN_ENABLE, OUTPUT);
    digitalWrite(PIN_ENABLE, LOW);   // active-low enable on most TB6600
    _stepper.setMaxSpeed(PAN_MAX_SPEED);
    _stepper.setAcceleration(PAN_MAX_ACCEL);
    _stepper.setMinPulseWidth(5);

    // --- MCPWM Tilt Servo (300 Hz) — Legacy API ---
    mcpwm_gpio_init(SERVO_MCPWM_UNIT, MCPWM0A, PIN_TILT_SERVO);

    mcpwm_config_t pwmCfg;
    pwmCfg.frequency    = SERVO_FREQ_HZ;        // 300 Hz
    pwmCfg.cmpr_a       = 0.0f;                 // initial duty %
    pwmCfg.cmpr_b       = 0.0f;
    pwmCfg.duty_mode    = MCPWM_DUTY_MODE_0;    // active high
    pwmCfg.counter_mode = MCPWM_UP_COUNTER;
    mcpwm_init(SERVO_MCPWM_UNIT, SERVO_MCPWM_TIMER, &pwmCfg);

    // Set initial position to horizon (0°)
    mcpwm_set_duty_in_us(SERVO_MCPWM_UNIT, SERVO_MCPWM_TIMER,
                          MCPWM_GEN_A, SERVO_MIN_PULSE_US);

    _lastPidUs = micros();
}

// ============================================================================
//  Pan North Offset
// ============================================================================
void Actuators::setPanNorthOffset(double northHeadingDeg) {
    float encoderDeg = readEncoderDeg();
    _panOffset = (float)northHeadingDeg - encoderDeg;
    while (_panOffset > 180.0f)  _panOffset -= 360.0f;
    while (_panOffset <= -180.0f) _panOffset += 360.0f;
}

// ============================================================================
//  AS5048A Encoder Read
// ============================================================================
float Actuators::readEncoderDeg() {
    _spiTransfer16(_buildReadCommand(AS5048A_CMD_ANGLE));
    delayMicroseconds(1);
    uint16_t raw = _spiTransfer16(0x0000);
    uint16_t angle14 = raw & 0x3FFF;
    return (float)angle14 * (360.0f / 16384.0f);
}

uint16_t Actuators::_spiTransfer16(uint16_t cmd) {
    SPI.beginTransaction(SPISettings(AS5048A_SPI_SPEED, MSBFIRST, SPI_MODE1));
    digitalWrite(PIN_ENC_CS, LOW);
    uint16_t result = SPI.transfer16(cmd);
    digitalWrite(PIN_ENC_CS, HIGH);
    SPI.endTransaction();
    return result;
}

uint16_t Actuators::_buildReadCommand(uint16_t address) {
    uint16_t cmd = (1 << 14) | (address & 0x3FFF);   // R/W = 1 (read)
    cmd |= ((uint16_t)_evenParity(cmd) << 15);        // even parity in MSB
    return cmd;
}

uint8_t Actuators::_evenParity(uint16_t value) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (value & (1 << i)) count++;
    }
    return (count & 1);
}

// ============================================================================
//  getPanAngleDeg — True-North-referenced output angle
// ============================================================================
float Actuators::getPanAngleDeg() {
    float raw = readEncoderDeg();
    float angle = raw + _panOffset;
    while (angle < 0.0f)    angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

// ============================================================================
//  Pan target and PID update
// ============================================================================
void Actuators::setTargetAzimuth(float azDeg) {
    _targetAzDeg = azDeg;
}

void Actuators::setPanPID(float Kp, float Ki, float Kd) {
    _pid.Kp = Kp;
    _pid.Ki = Ki;
    _pid.Kd = Kd;
}

void Actuators::updatePan() {
    unsigned long nowUs = micros();
    float dt = (float)(nowUs - _lastPidUs) * 1e-6f;
    _lastPidUs = nowUs;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;

    float currentAz = getPanAngleDeg();
    float error = _angleDiffDeg(_targetAzDeg, currentAz);

    // Deadband: hold still inside the window so quantization can't ratchet
    // single-step ticks at rest.
    if (fabsf(error) < PAN_DEADBAND_DEG) {
        _integralError    = 0.0f;
        _prevError        = error;
        _lastPidOutput    = 0.0f;
        _lastCommandedSPS = 0.0f;
        _stepper.setSpeed(0.0f);
        return;
    }

    // PID with anti-windup
    _integralError += error * dt;
    if (_integralError > PAN_INTEGRAL_LIMIT)  _integralError = PAN_INTEGRAL_LIMIT;
    if (_integralError < -PAN_INTEGRAL_LIMIT) _integralError = -PAN_INTEGRAL_LIMIT;

    float derivative = (error - _prevError) / dt;
    _prevError = error;

    float pidOutput = _pid.Kp * error
                    + _pid.Ki * _integralError
                    + _pid.Kd * derivative;

    _lastPidOutput = pidOutput;

    // Convert PID output (deg/s) → stepper speed (steps/s)
    float speedSPS = pidOutput * STEPS_PER_DEGREE;
    if (speedSPS > PAN_MAX_SPEED)  speedSPS = PAN_MAX_SPEED;
    if (speedSPS < -PAN_MAX_SPEED) speedSPS = -PAN_MAX_SPEED;

    // Slew limit: cap |Δspeed| per tick to PAN_MAX_ACCEL × dt so the motor
    // is never asked to jump faster than it can physically accelerate.
    float maxDelta = PAN_MAX_ACCEL * dt;
    float delta    = speedSPS - _lastCommandedSPS;
    if (delta >  maxDelta) speedSPS = _lastCommandedSPS + maxDelta;
    if (delta < -maxDelta) speedSPS = _lastCommandedSPS - maxDelta;
    _lastCommandedSPS = speedSPS;

    _stepper.setSpeed(speedSPS);
}

void Actuators::runPan() {
    _stepper.runSpeed();
}

// ============================================================================
//  Shortest-path angular difference (handles 0/360 wraparound)
// ============================================================================
float Actuators::_angleDiffDeg(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// ============================================================================
//  Tilt Servo — 300 Hz via legacy MCPWM
// ============================================================================
void Actuators::setTiltAngle(float elevDeg) {
    // Strict mechanical clamp — prevents self-destruction
    if (elevDeg < TILT_MIN_DEG) elevDeg = TILT_MIN_DEG;
    if (elevDeg > TILT_MAX_DEG) elevDeg = TILT_MAX_DEG;

    // Map physical elevation [0°..135°] to servo pulse width.
    // The servo has 270° range mapped to [500µs .. 2500µs].
    // 2:1 belt reduction: phys 0° → servo 0°, phys 135° → servo 270°.
    uint32_t pulseUs = (uint32_t)map(
        (long)(elevDeg * 100.0f),
        (long)(TILT_MIN_DEG * 100.0f),
        (long)(TILT_MAX_DEG * 100.0f),
        (long)SERVO_MIN_PULSE_US,
        (long)SERVO_MAX_PULSE_US
    );

    mcpwm_set_duty_in_us(SERVO_MCPWM_UNIT, SERVO_MCPWM_TIMER,
                          MCPWM_GEN_A, pulseUs);
}

// ============================================================================
//  HIL Helper Methods — Simulated Encoder & Telemetry
// ============================================================================
void Actuators::updatePanWithPosition(float currentAzDeg) {
    unsigned long nowUs = micros();
    float dt = (float)(nowUs - _lastPidUs) * 1e-6f;
    _lastPidUs = nowUs;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;

    float error = _angleDiffDeg(_targetAzDeg, currentAzDeg);

    // Deadband: hold still inside the window so quantization can't ratchet
    // single-step ticks at rest.
    if (fabsf(error) < PAN_DEADBAND_DEG) {
        _integralError    = 0.0f;
        _prevError        = error;
        _lastPidOutput    = 0.0f;
        _lastCommandedSPS = 0.0f;
        _stepper.setSpeed(0.0f);
        return;
    }

    // PID with anti-windup
    _integralError += error * dt;
    if (_integralError > PAN_INTEGRAL_LIMIT)  _integralError = PAN_INTEGRAL_LIMIT;
    if (_integralError < -PAN_INTEGRAL_LIMIT) _integralError = -PAN_INTEGRAL_LIMIT;

    float derivative = (error - _prevError) / dt;
    _prevError = error;

    float pidOutput = _pid.Kp * error
                    + _pid.Ki * _integralError
                    + _pid.Kd * derivative;

    _lastPidOutput = pidOutput;

    // Convert PID output (deg/s) → stepper speed (steps/s)
    float speedSPS = pidOutput * STEPS_PER_DEGREE;
    if (speedSPS > PAN_MAX_SPEED)  speedSPS = PAN_MAX_SPEED;
    if (speedSPS < -PAN_MAX_SPEED) speedSPS = -PAN_MAX_SPEED;

    // Slew limit: cap |Δspeed| per tick to PAN_MAX_ACCEL × dt so the motor
    // is never asked to jump faster than it can physically accelerate.
    float maxDelta = PAN_MAX_ACCEL * dt;
    float delta    = speedSPS - _lastCommandedSPS;
    if (delta >  maxDelta) speedSPS = _lastCommandedSPS + maxDelta;
    if (delta < -maxDelta) speedSPS = _lastCommandedSPS - maxDelta;
    _lastCommandedSPS = speedSPS;

    _stepper.setSpeed(speedSPS);
}

float Actuators::getStepperPositionDeg() {
    float deg = (float)_stepper.currentPosition() / STEPS_PER_DEGREE;
    // Normalize to [0, 360)
    while (deg < 0.0f)    deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

float Actuators::getPidOutput() const {
    return _lastPidOutput;
}

float Actuators::getStepperSpeed() {
    return _stepper.speed();
}

float Actuators::getTargetAzimuth() const {
    return _targetAzDeg;
}
