// ============================================================================
// main.cpp — HIL (Hardware-In-the-Loop) Actuator Test Harness
// Cursr-V Antenna Tracker
//
// Accepts JSON commands over USB Serial to drive actuators.
// Sensors (AS5048A, Magnetometer, GNSS) are optional and toggleable.
// Designed to work with the companion Web UI (hil_panel/index.html).
//
// Protocol:
//   IN  → {"cmd":"direct","az":45.0,"el":30.0}   (newline-terminated JSON)
//   OUT ← {"t":"tel","az_t":45.0,"el_t":30.0,...} (10 Hz telemetry)
// ============================================================================
#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "ActuatorControl.h"
#include "Navigation.h"

// ============================================================================
//  Global Objects
// ============================================================================
Actuators actuators;

// ============================================================================
//  Sensor Availability Flags (toggled via Web UI at runtime)
// ============================================================================
bool hasEncoder      = false;   // AS5048A magnetic encoder
bool hasMagnetometer = false;   // MMC5983MA magnetometer
bool hasGNSS         = false;   // u-blox GNSS module

// ============================================================================
//  Base Station Position (manual entry or GNSS)
// ============================================================================
double baseLat = HIL_DEFAULT_BASE_LAT;
double baseLon = HIL_DEFAULT_BASE_LON;
double baseAlt = HIL_DEFAULT_BASE_ALT;

// ============================================================================
//  Target State
// ============================================================================
float targetAz = 0.0f;
float targetEl = 0.0f;

// ============================================================================
//  Sweep State Machine
// ============================================================================
enum SweepState : uint8_t {
    SWEEP_NONE,
    SWEEP_AZ_FWD,      // 0 → 360
    SWEEP_EL_FWD,      // 0 → 135
    SWEEP_EL_REV        // 135 → 0
};

SweepState sweepState   = SWEEP_NONE;
float      sweepAngle   = 0.0f;
unsigned long sweepLastUs = 0;

// ============================================================================
//  Timing
// ============================================================================
unsigned long lastTelemetryMs = 0;
unsigned long lastPidUs       = 0;

// ============================================================================
//  Serial Input Buffer
// ============================================================================
static String serialBuffer;

// ============================================================================
//  Forward Declarations
// ============================================================================
void parseSerialJSON();
void processCommand(JsonDocument& doc);
void updateSweep();
void updatePanAxis();
void sendTelemetry();
void sendAck(const char* cmd, JsonDocument* extra = nullptr);
void sendError(const char* msg);

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait for USB CDC */ }

    // Initialize actuators (servo + stepper + encoder SPI)
    // Encoder SPI init is harmless even when hardware is absent
    actuators.begin();

    // Reserve buffer for serial input
    serialBuffer.reserve(256);

    // ---- Startup Self-Test ----
    // Briefly move both actuators to confirm hardware is working
    Serial.println("HIL: Running startup self-test...");

    // Test servo: move to 45°, wait, return to 0°
    actuators.setTiltAngle(45.0f);
    delay(500);
    actuators.setTiltAngle(0.0f);
    delay(300);

    // Test stepper: rotate a small amount forward, then back
    // Directly step a few pulses to verify motor responds
    actuators.setTargetAzimuth(15.0f);  // target 15°
    for (int i = 0; i < 500; i++) {
        actuators.updatePanWithPosition(0.0f);  // PID thinks we're at 0°
        actuators.runPan();
        delayMicroseconds(500);
    }
    // Return to 0
    actuators.setTargetAzimuth(0.0f);
    for (int i = 0; i < 500; i++) {
        float pos = actuators.getStepperPositionDeg();
        actuators.updatePanWithPosition(pos);
        actuators.runPan();
        delayMicroseconds(500);
    }

    Serial.println("HIL: Self-test complete.");

    // Ready message
    JsonDocument readyDoc;
    readyDoc["t"]       = "ready";
    readyDoc["version"] = "1.0.0";
    readyDoc["board"]   = "ESP32-S3";
#ifdef USE_ULN2003
    readyDoc["motor"]   = "28BYJ-48/ULN2003";
#else
    readyDoc["motor"]   = "NEMA17/TB6600";
#endif
#ifdef USE_SG90
    readyDoc["servo"]   = "SG90/50Hz";
#else
    readyDoc["servo"]   = "DS51150/300Hz";
#endif
    readyDoc["enc"]     = hasEncoder;
    readyDoc["mag"]     = hasMagnetometer;
    readyDoc["gps"]     = hasGNSS;
    serializeJson(readyDoc, Serial);
    Serial.println();

    lastPidUs = micros();
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
    // 1. Parse incoming JSON commands (non-blocking)
    parseSerialJSON();

    // 2. Update sweep animation if active
    updateSweep();

    // 3. PID update at 200 Hz (rate-limited)
    unsigned long nowUs = micros();
    if (nowUs - lastPidUs >= PID_INTERVAL_US) {
        lastPidUs = nowUs;
        updatePanAxis();
    }

    // 4. Stepper pulse generation (non-blocking, call every loop)
    actuators.runPan();

    // 5. Send telemetry at 10 Hz
    unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryMs = nowMs;
        sendTelemetry();
    }
}

// ============================================================================
//  Serial JSON Parser (non-blocking, newline-delimited)
// ============================================================================
void parseSerialJSON() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                JsonDocument doc;
                DeserializationError err = deserializeJson(doc, serialBuffer);
                if (err) {
                    sendError(err.c_str());
                } else {
                    processCommand(doc);
                }
                serialBuffer = "";
            }
        } else {
            if (serialBuffer.length() < 256) {
                serialBuffer += c;
            }
        }
    }
}

// ============================================================================
//  Command Processor
// ============================================================================
void processCommand(JsonDocument& doc) {
    const char* cmd = doc["cmd"];
    if (!cmd) {
        sendError("missing cmd field");
        return;
    }

    // ---- set_sensors: toggle sensor availability ----
    if (strcmp(cmd, "set_sensors") == 0) {
        if (doc["enc"].is<bool>()) hasEncoder      = doc["enc"];
        if (doc["mag"].is<bool>()) hasMagnetometer = doc["mag"];
        if (doc["gps"].is<bool>()) hasGNSS         = doc["gps"];

        JsonDocument ack;
        ack["enc"] = hasEncoder;
        ack["mag"] = hasMagnetometer;
        ack["gps"] = hasGNSS;
        sendAck("set_sensors", &ack);
    }
    // ---- direct: set Az/El directly ----
    else if (strcmp(cmd, "direct") == 0) {
        if (!doc["az"].isNull()) {
            targetAz = doc["az"].as<float>();
            actuators.setTargetAzimuth(targetAz);
        }
        if (!doc["el"].isNull()) {
            targetEl = doc["el"].as<float>();
            actuators.setTiltAngle(targetEl);
        }
        
        sweepState = SWEEP_NONE; // cancel any sweep

        JsonDocument ack;
        ack["az"] = targetAz;
        ack["el"] = targetEl;
        sendAck("direct", &ack);
    }
    // ---- inject: WGS84 coordinates → Az/El via Navigation ----
    else if (strcmp(cmd, "inject") == 0) {
        if (doc["lat"].isNull() || doc["lon"].isNull() || doc["alt"].isNull()) {
            sendError("inject: missing lat/lon/alt");
            return;
        }
        double tgtLat = doc["lat"].as<double>();
        double tgtLon = doc["lon"].as<double>();
        double tgtAlt = doc["alt"].as<double>();

        PointingAngles pa = computePointing(baseLat, baseLon, baseAlt,
                                             tgtLat,  tgtLon,  tgtAlt);
        targetAz = (float)pa.azimuth;
        targetEl = (float)pa.elevation;
        if (targetEl < TILT_MIN_DEG) targetEl = TILT_MIN_DEG;
        if (targetEl > TILT_MAX_DEG) targetEl = TILT_MAX_DEG;

        actuators.setTargetAzimuth(targetAz);
        actuators.setTiltAngle(targetEl);
        sweepState = SWEEP_NONE;

        JsonDocument ack;
        ack["az"] = targetAz;
        ack["el"] = targetEl;
        sendAck("inject", &ack);
    }
    // ---- set_base: update base station coordinates ----
    else if (strcmp(cmd, "set_base") == 0) {
        if (doc["lat"].isNull() || doc["lon"].isNull() || doc["alt"].isNull()) {
            sendError("set_base: missing lat/lon/alt");
            return;
        }
        baseLat = doc["lat"].as<double>();
        baseLon = doc["lon"].as<double>();
        baseAlt = doc["alt"].as<double>();
        sendAck("set_base");
    }
    // ---- home: return to Az=0, El=0 ----
    else if (strcmp(cmd, "home") == 0) {
        targetAz = 0.0f;
        targetEl = 0.0f;
        actuators.setTargetAzimuth(0.0f);
        actuators.setTiltAngle(0.0f);
        sweepState = SWEEP_NONE;
        sendAck("home");
    }
    // ---- set_pid: update PID gains ----
    else if (strcmp(cmd, "set_pid") == 0) {
        if (doc["kp"].isNull() || doc["ki"].isNull() || doc["kd"].isNull()) {
            sendError("set_pid: missing kp/ki/kd");
            return;
        }
        float kp = doc["kp"].as<float>();
        float ki = doc["ki"].as<float>();
        float kd = doc["kd"].as<float>();
        if (kp < 0.0f || ki < 0.0f || kd < 0.0f) {
            sendError("set_pid: gains must be >= 0");
            return;
        }
        actuators.setPanPID(kp, ki, kd);

        JsonDocument ack;
        ack["kp"] = kp;
        ack["ki"] = ki;
        ack["kd"] = kd;
        sendAck("set_pid", &ack);
    }
    // ---- sweep_el: sweep elevation 0→135→0 ----
    else if (strcmp(cmd, "sweep_el") == 0) {
        sweepState  = SWEEP_EL_FWD;
        sweepAngle  = 0.0f;
        sweepLastUs = micros();
        sendAck("sweep_el");
    }
    // ---- sweep_az: sweep azimuth 0→360 ----
    else if (strcmp(cmd, "sweep_az") == 0) {
        sweepState  = SWEEP_AZ_FWD;
        sweepAngle  = 0.0f;
        sweepLastUs = micros();
        sendAck("sweep_az");
    }
    // ---- stop: halt all motion ----
    else if (strcmp(cmd, "stop") == 0) {
        sweepState = SWEEP_NONE;
        // Keep current position — just stop updating targets
        sendAck("stop");
    }
    else {
        sendError("unknown command");
    }
}

// ============================================================================
//  Sweep State Machine
// ============================================================================
void updateSweep() {
    if (sweepState == SWEEP_NONE) return;

    unsigned long nowUs = micros();
    float dt = (float)(nowUs - sweepLastUs) * 1e-6f;
    sweepLastUs = nowUs;
    if (dt <= 0.0f || dt > 0.5f) return;

    float step = SWEEP_SPEED_DEG_PER_SEC * dt;

    switch (sweepState) {
        case SWEEP_AZ_FWD:
            sweepAngle += step;
            if (sweepAngle >= 360.0f) {
                sweepAngle = 0.0f;
                sweepState = SWEEP_NONE;
            }
            targetAz = sweepAngle;
            actuators.setTargetAzimuth(targetAz);
            break;

        case SWEEP_EL_FWD:
            sweepAngle += step;
            if (sweepAngle >= TILT_MAX_DEG) {
                sweepAngle = TILT_MAX_DEG;
                sweepState = SWEEP_EL_REV;
            }
            targetEl = sweepAngle;
            actuators.setTiltAngle(targetEl);
            break;

        case SWEEP_EL_REV:
            sweepAngle -= step;
            if (sweepAngle <= TILT_MIN_DEG) {
                sweepAngle = TILT_MIN_DEG;
                sweepState = SWEEP_NONE;
            }
            targetEl = sweepAngle;
            actuators.setTiltAngle(targetEl);
            break;

        default:
            break;
    }
}

// ============================================================================
//  Pan Axis Update — choose real encoder or simulated
// ============================================================================
void updatePanAxis() {
    actuators.setTargetAzimuth(targetAz);

    if (hasEncoder) {
        // Use real AS5048A encoder via Actuators class
        actuators.updatePan();
    } else {
        // Simulate encoder from accumulated stepper steps
        float simPosition = actuators.getStepperPositionDeg();
        actuators.updatePanWithPosition(simPosition);
    }
}

// ============================================================================
//  Telemetry Output (JSON, 10 Hz)
// ============================================================================
void sendTelemetry() {
    float currentAz = hasEncoder
        ? actuators.getPanAngleDeg()
        : actuators.getStepperPositionDeg();

    JsonDocument doc;
    doc["t"]     = "tel";
    doc["az_t"]  = serialized(String(targetAz, 1));
    doc["el_t"]  = serialized(String(targetEl, 1));
    doc["az_c"]  = serialized(String(currentAz, 1));
    doc["el_c"]  = serialized(String(targetEl, 1));  // servo is open-loop
    doc["pid"]   = serialized(String(actuators.getPidOutput(), 2));
    doc["spd"]   = serialized(String(actuators.getStepperSpeed(), 1));
    doc["up"]    = millis();
    doc["enc"]   = hasEncoder;
    doc["mag"]   = hasMagnetometer;
    doc["gps"]   = hasGNSS;

    const char* sweepStr = "none";
    switch (sweepState) {
        case SWEEP_AZ_FWD: sweepStr = "az";  break;
        case SWEEP_EL_FWD:
        case SWEEP_EL_REV: sweepStr = "el";  break;
        default: break;
    }
    doc["sweep"] = sweepStr;

    serializeJson(doc, Serial);
    Serial.println();
}

// ============================================================================
//  ACK / Error Helpers
// ============================================================================
void sendAck(const char* cmd, JsonDocument* extra) {
    JsonDocument doc;
    doc["t"]   = "ack";
    doc["cmd"] = cmd;
    if (extra) {
        // Merge extra fields
        for (JsonPair kv : extra->as<JsonObject>()) {
            doc[kv.key()] = kv.value();
        }
    }
    serializeJson(doc, Serial);
    Serial.println();
}

void sendError(const char* msg) {
    JsonDocument doc;
    doc["t"]   = "err";
    doc["msg"] = msg;
    serializeJson(doc, Serial);
    Serial.println();
}