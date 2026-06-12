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
//  PIN ASSIGNMENTS
// ============================================================================

static constexpr int PIN_STEP       = 16; //Pulse
static constexpr int PIN_DIR        = 15;
static constexpr int PIN_ENABLE     = 7;

// --- AS5048A Magnetic Encoder (SPI) ---
static constexpr int PIN_ENC_CS     = 10;
// Explicit SPI bus pins (do NOT rely on board variant defaults — the
// esp32s3usbotg variant does not necessarily map SPI to these GPIOs).
static constexpr int PIN_ENC_SCK    = 12;
static constexpr int PIN_ENC_MISO   = 13;
static constexpr int PIN_ENC_MOSI   = 11;
// Encoder rotation sense. The AS5048A counts increase opposite to the desired
// clockwise-positive azimuth on this rig, so the interpreted angle is mirrored.
// Raw counts in telemetry are left untouched. Flip to false if a future rig
// wires the encoder the other way.
static constexpr bool ENC_DIR_INVERT = true;

// --- DS51150 Tilt Servo (MCPWM) ---
static constexpr int PIN_TILT_SERVO = 21;

// --- UART from Heltec LoRa Receiver ---
static constexpr int  UART_RX_PIN   = 44;
static constexpr int  UART_TX_PIN   = 43;
static constexpr long UART_BAUD     = 115200;

// --- I2C Bus (GPS) ---
static constexpr int I2C_SDA_PIN    = 5;
static constexpr int I2C_SCL_PIN    = 4;

// ============================================================================
//  MECHANICAL CONSTANTS
// ============================================================================

/// Pan axis gear reduction ratio (motor:output).
/// 2.0 means 2 motor revolutions = 1 output revolution.
static constexpr float PAN_GEAR_RATIO      = 2.0f;

/// Steps per motor revolution (full-steps × microstepping).
/// Adjust to match your TB6600 DIP-switch microstepping setting.
/// Example: 200 full-steps × 16 microsteps = 3200
static constexpr float MOTOR_STEPS_PER_REV = 200.0f * 16.0f;

/// Derived: steps per degree at the output shaft (after gearing).
static constexpr float STEPS_PER_DEGREE    = (MOTOR_STEPS_PER_REV * PAN_GEAR_RATIO) / 360.0f;


/// DS51150: 2:1 belt reduction on a 270° servo → 135° usable sweep.
static constexpr float TILT_MIN_DEG = 0.0f;
static constexpr float TILT_MAX_DEG = 135.0f;

// ============================================================================
//  TILT SERVO TIMING
// ============================================================================

/// DS51150: 300 Hz high-frequency PWM, 500-2500µs for 0°-270°
static constexpr uint32_t SERVO_FREQ_HZ      = 300;
static constexpr uint32_t SERVO_MIN_PULSE_US  = 500;    // 0°
static constexpr uint32_t SERVO_MAX_PULSE_US  = 2500;   // 270°

// ============================================================================
//  AS5048A ENCODER
// ============================================================================

static constexpr uint16_t AS5048A_CMD_ANGLE  = 0x3FFF;  // Angle register address
static constexpr uint32_t AS5048A_SPI_SPEED  = 1000000; // 1 MHz SPI clock

// ============================================================================
//  PAN PID TUNING
// ============================================================================

/// Default PID gains — tune on actual hardware.
static constexpr float PAN_PID_KP = 4.0f;
static constexpr float PAN_PID_KI = 0.02f;
static constexpr float PAN_PID_KD = 0.5f;

/// Maximum stepper speed (steps/sec)
static constexpr float PAN_MAX_SPEED       = 2000.0f;

/// Maximum stepper acceleration (steps/sec²)
static constexpr float PAN_MAX_ACCEL       = 4000.0f;

/// PID integral anti-windup limit (degrees)
static constexpr float PAN_INTEGRAL_LIMIT  = 50.0f;

/// Deadband around target (degrees). Inside this window the PID holds the
/// motor still and zeroes its integral, so quantization in the simulated
/// encoder doesn't ratchet single-step ticks at rest.
static constexpr float PAN_DEADBAND_DEG    = 0.2f;

// ============================================================================
//  ALPHA-BETA FILTER TUNING
// ============================================================================

/// Position smoothing gain (0 < α ≤ 1). Higher = more responsive to noise.
static constexpr float AB_ALPHA = 0.85f;

/// Velocity smoothing gain (0 < β ≤ 1). Higher = faster velocity adaptation.
static constexpr float AB_BETA  = 0.005f;

// ============================================================================
//  FLIGHT-REPLAY TRACK STREAM (mission.html → `track` command)
// ============================================================================

/// Cadence (Hz) at which the panel streams `track` position samples. The three
/// per-axis alpha-beta filters use 1/TRACK_STREAM_HZ as their dt, so this MUST
/// match the panel's playback rate or the velocity term extrapolates wrong.
static constexpr float TRACK_STREAM_HZ = 15.0f;

/// Idle gap (ms) after which the next `track` sample re-seeds the filters — i.e.
/// the start of a fresh replay resets the smoothing state.
static constexpr unsigned long TRACK_IDLE_RESET_MS = 500;

// ============================================================================
//  CONTROL LOOP TIMING
// ============================================================================

/// PID update interval in microseconds (5000 µs = 200 Hz)
static constexpr unsigned long PID_INTERVAL_US  = 5000;

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

// ============================================================================
//  MISSION MODE (src/mission) — UART uplink + launch detection
// ============================================================================

/// Expected GroundLinkPacket rate from the ground Heltec. The mission
/// alpha-beta filters use 1/LINK_RATE_HZ as their dt.
static constexpr float    LINK_RATE_HZ          = 10.0f;

/// No valid packet for this long while tracking → SIGNAL_LOST (hold pointing).
static constexpr uint32_t LINK_TIMEOUT_MS       = 2000;

/// Launch is declared when, for LAUNCH_DETECT_SAMPLES consecutive packets,
/// altitude exceeds the pad by LAUNCH_ALT_DELTA_M or climb rate exceeds
/// LAUNCH_CLIMB_MS.
static constexpr float    LAUNCH_ALT_DELTA_M    = 20.0f;
static constexpr float    LAUNCH_CLIMB_MS       = 8.0f;
static constexpr uint8_t  LAUNCH_DETECT_SAMPLES = 3;

/// Packets that must agree (within PAD_DRIFT_MAX_M of the running average)
/// before the pad position is considered locked.
static constexpr uint16_t PAD_LOCK_MIN_PACKETS  = 20;
static constexpr float    PAD_DRIFT_MAX_M       = 30.0f;

/// After pad lock the rig slews to the pad, then de-energises the motors once
/// this settle window has elapsed (low-power ARMED standby).
static constexpr uint32_t ARM_SETTLE_MS         = 5000;

// ============================================================================
//  GNSS PRECISE-FIX THRESHOLDS (background base acquisition in loop)
// ============================================================================

/// Minimum satellites-in-view required to accept a fix as "precise".
static constexpr uint8_t  GNSS_PRECISE_MIN_SATS  = 6;

/// Maximum horizontal accuracy (metres) required to accept the fix.
static constexpr float    GNSS_PRECISE_MAX_HACC_M = 5.0f;

#endif // CONFIG_H
