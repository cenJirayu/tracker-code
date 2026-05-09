// ============================================================================
// config.h — Central Hardware & Tuning Configuration
// Cursr-V Antenna Tracker
//
// Edit this file to match your wiring, launch-site, and tuning parameters.
// No other source files need to be modified for routine configuration changes.
// ============================================================================
#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

// ============================================================================
//  MOTOR DRIVER SELECTION
// ============================================================================
// Uncomment ONE of the following to select your pan-axis motor driver:
//   USE_TB6600   — NEMA17 stepper + TB6600 driver (STEP/DIR/EN)
//   USE_ULN2003  — 28BYJ-48 stepper + ULN2003 driver (4-wire)

#define USE_TB6600
//#define USE_ULN2003

// Uncomment ONE of the following to select your tilt servo:
//   USE_DS51150  — DS51150 heavy-duty servo (300 Hz MCPWM, 270° range, 2:1 belt)
//   USE_SG90     — SG90 micro servo (50 Hz standard PWM, 180° range, direct)

#define USE_DS51150
//#define USE_SG90

// ============================================================================
//  PIN ASSIGNMENTS
// ============================================================================

#ifdef USE_ULN2003
// --- ULN2003 Driver + 28BYJ-48 Stepper (Pan Axis) ---
// Pin order: IN1, IN2, IN3, IN4 on the ULN2003 board
static constexpr int PIN_IN1 = 16;
static constexpr int PIN_IN2 = 17;
static constexpr int PIN_IN3 = 18;
static constexpr int PIN_IN4 = 15;
#else
// --- TB6600 Stepper Driver (Pan Axis) ---
static constexpr int PIN_STEP       = 16;
static constexpr int PIN_DIR        = 17;
static constexpr int PIN_ENABLE     = 18;
#endif

// --- AS5048A Magnetic Encoder (SPI) ---
static constexpr int PIN_ENC_CS     = 5;
// SPI bus uses default ESP32-S3 pins: MOSI=11, MISO=13, SCK=12

// --- DS51150 Tilt Servo (MCPWM) ---
static constexpr int PIN_TILT_SERVO = 4;

// --- UART from Heltec LoRa Receiver ---
static constexpr int  UART_RX_PIN   = 44;
static constexpr int  UART_TX_PIN   = 43;
static constexpr long UART_BAUD     = 115200;

// --- I2C Bus (GPS + Magnetometer) ---
static constexpr int I2C_SDA_PIN    = 8;
static constexpr int I2C_SCL_PIN    = 9;

// ============================================================================
//  SITE-SPECIFIC CONSTANTS
// ============================================================================

/// Local magnetic declination in degrees (east-positive).
/// Look up your launch site at:
///   https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
static constexpr double MAGNETIC_DECLINATION_DEG = 0.5;

// ============================================================================
//  MECHANICAL CONSTANTS
// ============================================================================

#ifdef USE_ULN2003
/// 28BYJ-48 with internal 1:63.68 gear ratio
/// Half-step mode: 64 half-steps/motor-rev × 63.68 gear ratio ≈ 4076 steps/rev
static constexpr float PAN_GEAR_RATIO      = 1.0f;   // no external gearing
static constexpr float MOTOR_STEPS_PER_REV = 4076.0f; // half-step with internal gear
#else
/// Pan axis gear reduction ratio (motor:output).
/// 2.0 means 2 motor revolutions = 1 output revolution.
static constexpr float PAN_GEAR_RATIO      = 2.0f;

/// Steps per motor revolution (full-steps × microstepping).
/// Adjust to match your TB6600 DIP-switch microstepping setting.
/// Example: 200 full-steps × 16 microsteps = 3200
static constexpr float MOTOR_STEPS_PER_REV = 200.0f * 16.0f;
#endif

/// Derived: steps per degree at the output shaft (after gearing).
static constexpr float STEPS_PER_DEGREE    = (MOTOR_STEPS_PER_REV * PAN_GEAR_RATIO) / 360.0f;

#ifdef USE_SG90
/// SG90: direct drive, 0-180° range
static constexpr float TILT_MIN_DEG = 0.0f;
static constexpr float TILT_MAX_DEG = 180.0f;
#else
/// DS51150: 2:1 belt reduction on a 270° servo → 135° usable sweep.
static constexpr float TILT_MIN_DEG = 0.0f;
static constexpr float TILT_MAX_DEG = 135.0f;
#endif

// ============================================================================
//  TILT SERVO TIMING
// ============================================================================

#ifdef USE_SG90
/// SG90: standard 50 Hz PWM, 500-2400µs pulse range for 0°-180°
static constexpr uint32_t SERVO_FREQ_HZ      = 50;
static constexpr uint32_t SERVO_MIN_PULSE_US  = 500;    // 0°
static constexpr uint32_t SERVO_MAX_PULSE_US  = 2400;   // 180°
#else
/// DS51150: 300 Hz high-frequency PWM, 500-2500µs for 0°-270°
static constexpr uint32_t SERVO_FREQ_HZ      = 300;
static constexpr uint32_t SERVO_MIN_PULSE_US  = 500;    // 0°
static constexpr uint32_t SERVO_MAX_PULSE_US  = 2500;   // 270°
#endif

// ============================================================================
//  AS5048A ENCODER
// ============================================================================

static constexpr uint16_t AS5048A_CMD_ANGLE  = 0x3FFF;  // Angle register address
static constexpr uint32_t AS5048A_SPI_SPEED  = 1000000; // 1 MHz SPI clock

// ============================================================================
//  PAN PID TUNING
// ============================================================================

#ifdef USE_ULN2003
/// PID gains tuned for 28BYJ-48 (slower, lower inertia)
static constexpr float PAN_PID_KP = 2.0f;
static constexpr float PAN_PID_KI = 0.01f;
static constexpr float PAN_PID_KD = 0.3f;

/// Maximum stepper speed (steps/sec) — 28BYJ-48 stalls above ~800 sps
static constexpr float PAN_MAX_SPEED       = 800.0f;

/// Maximum stepper acceleration (steps/sec²)
static constexpr float PAN_MAX_ACCEL       = 400.0f;
#else
/// Default PID gains — tune on actual hardware.
static constexpr float PAN_PID_KP = 4.0f;
static constexpr float PAN_PID_KI = 0.02f;
static constexpr float PAN_PID_KD = 0.5f;

/// Maximum stepper speed (steps/sec)
static constexpr float PAN_MAX_SPEED       = 8000.0f;

/// Maximum stepper acceleration (steps/sec²)
static constexpr float PAN_MAX_ACCEL       = 4000.0f;
#endif

/// PID integral anti-windup limit (degrees)
static constexpr float PAN_INTEGRAL_LIMIT  = 50.0f;

// ============================================================================
//  ALPHA-BETA FILTER TUNING
// ============================================================================

/// Position smoothing gain (0 < α ≤ 1). Higher = more responsive to noise.
static constexpr float AB_ALPHA = 0.85f;

/// Velocity smoothing gain (0 < β ≤ 1). Higher = faster velocity adaptation.
static constexpr float AB_BETA  = 0.005f;

/// Expected time step between telemetry samples (seconds).
/// 0.01 = 100 Hz, 0.02 = 50 Hz
static constexpr float AB_DT    = 0.01f;

// ============================================================================
//  CONTROL LOOP TIMING
// ============================================================================

/// PID update interval in microseconds (5000 µs = 200 Hz)
static constexpr unsigned long PID_INTERVAL_US  = 5000;

/// Serial debug output interval in milliseconds
static constexpr unsigned long DEBUG_INTERVAL_MS = 500;

// ============================================================================
//  HIL TEST DEFAULTS
// ============================================================================

/// Default base station position (Bangkok) — used when GNSS is not available
static constexpr double HIL_DEFAULT_BASE_LAT = 13.7563;
static constexpr double HIL_DEFAULT_BASE_LON = 100.5018;
static constexpr double HIL_DEFAULT_BASE_ALT = 10.0;

/// Telemetry report rate (milliseconds)
static constexpr unsigned long TELEMETRY_INTERVAL_MS = 100;  // 10 Hz

/// Sweep test speed (degrees per second)
static constexpr float SWEEP_SPEED_DEG_PER_SEC = 30.0f;

#endif // CONFIG_H
