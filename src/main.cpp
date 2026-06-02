// ============================================================================
// main.cpp — HIL (Hardware-In-the-Loop) Actuator Test Harness
// Cursr-V Antenna Tracker
//
// Accepts JSON commands over USB Serial to drive the pan/tilt actuators.
// Sensors (AS5048A encoder, u-blox GNSS) are optional and toggleable at
// runtime via set_sensors. Telemetry reports each sensor's raw wire value so
// the companion panels can show raw + interpreted data side by side.
//
// Protocol is documented in hil_panel/PROTOCOL.md.
//   IN  → {"cmd":"direct","az":45.0,"el":30.0}   (newline-terminated JSON)
//   OUT ← {"t":"tel","az_t":45.0,"el_t":30.0,...} (10 Hz telemetry)
// ============================================================================
#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "ActuatorControl.h"
#include "Navigation.h"
#include "GNSSManager.h"

// ============================================================================
//  Global Objects
// ============================================================================
Actuators   actuators;
GNSSManager gnss;

// ============================================================================
//  Sensor Availability Flags (toggled via Web UI at runtime)
// ============================================================================
bool hasEncoder = false;   // AS5048A magnetic encoder is the pan feedback source
bool hasGNSS    = false;   // u-blox GNSS drives the base station

// True when the GNSS hardware responded on begin(); independent of the runtime
// toggle (hasGNSS).
bool gnssHardwareOk = false;

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

// Last injected target coordinates — echoed in telemetry for pipeline display.
double lastTgtLat = 0.0;
double lastTgtLon = 0.0;
double lastTgtAlt = 0.0;
bool   hasLastInject = false;

// ============================================================================
//  Sweep State Machine
// ============================================================================
enum SweepState : uint8_t {
    SWEEP_NONE,
    SWEEP_AZ_FWD,   // 0 → 360
    SWEEP_EL_FWD,   // 0 → 135
    SWEEP_EL_REV    // 135 → 0
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

    // Initialize actuators (servo + stepper + encoder SPI).
    // Encoder SPI init is harmless even when hardware is absent.
    actuators.begin();

    // ---- GNSS (u-blox SAM-M10Q, I2C) ----
    gnssHardwareOk = gnss.begin();
    if (gnssHardwareOk) {
        Serial.println("HIL: GNSS OK — waiting for precise fix...");
        uint32_t gnssStart   = millis();
        uint32_t gnssLastLog = 0;
        while (millis() - gnssStart < GNSS_FIX_TIMEOUT_MS) {
            gnss.update();
            if (gnss.isValid()
                    && gnss.getSatsUsed() >= GNSS_PRECISE_MIN_SATS
                    && gnss.getHorizAccM() <= GNSS_PRECISE_MAX_HACC_M) {
                baseLat  = gnss.getLat();
                baseLon  = gnss.getLon();
                baseAlt  = gnss.getAlt();
                hasGNSS  = true;
                Serial.print("HIL: Base set from GNSS. Sats=");
                Serial.print(gnss.getSatsUsed());
                Serial.print(" hAcc=");
                Serial.print(gnss.getHorizAccM(), 1);
                Serial.println("m");
                break;
            }
            uint32_t now = millis();
            if (now - gnssLastLog >= 5000) {
                gnssLastLog = now;
                Serial.print("HIL: GNSS searching — fix=");
                Serial.print(gnss.getFixType());
                Serial.print(" sats=");
                Serial.println(gnss.getSatsUsed());
            }
            delay(100);
        }
        if (!hasGNSS) {
            Serial.println("HIL: GNSS timeout — using default base station.");
        }
    } else {
        Serial.println("HIL: GNSS not found on I2C.");
    }

    // Reserve buffer for serial input
    serialBuffer.reserve(256);

    // ---- Startup Self-Test ----
    Serial.println("HIL: Running startup self-test...");

    // Test servo: move to 45°, wait, return to 0°
    actuators.setTiltAngle(45.0f);
    delay(500);
    actuators.setTiltAngle(0.0f);
    delay(300);

    // Test stepper: rotate forward 15°, then return
    actuators.setTargetAzimuth(15.0f);
    for (int i = 0; i < 500; i++) {
        actuators.updatePanWithPosition(0.0f);
        actuators.runPan();
        delayMicroseconds(500);
    }
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
    readyDoc["t"]      = "ready";
    readyDoc["ver"]    = "2.0.0";
    readyDoc["board"]  = "ESP32-S3";
    readyDoc["motor"]  = "NEMA17/TB6600";
    readyDoc["servo"]  = "DS51150/300Hz";
    readyDoc["enc"]    = hasEncoder;
    readyDoc["gps"]    = hasGNSS;
    readyDoc["gps_hw"] = gnssHardwareOk;
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

    // 2. Poll GNSS and update base station when enabled
    if (gnssHardwareOk) {
        gnss.update();
        if (hasGNSS && gnss.isValid()) {
            baseLat = gnss.getLat();
            baseLon = gnss.getLon();
            baseAlt = gnss.getAlt();
        }
    }

    // 3. Update sweep animation if active
    updateSweep();

    // 4. PID update at 200 Hz (rate-limited)
    unsigned long nowUs = micros();
    if (nowUs - lastPidUs >= PID_INTERVAL_US) {
        lastPidUs = nowUs;
        updatePanAxis();
    }

    // 5. Stepper pulse generation (non-blocking, call every loop)
    actuators.runPan();

    // 6. Send telemetry at 10 Hz
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
        if (doc["enc"].is<bool>()) hasEncoder = doc["enc"];
        if (doc["gps"].is<bool>()) {
            bool newGNSS = doc["gps"];
            if (newGNSS && !hasGNSS && gnssHardwareOk && gnss.isValid()) {
                baseLat = gnss.getLat();
                baseLon = gnss.getLon();
                baseAlt = gnss.getAlt();
            }
            hasGNSS = newGNSS;
        }

        JsonDocument ack;
        ack["enc"] = hasEncoder;
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
        sweepState = SWEEP_NONE;

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

        // Track last injected target for pipeline telemetry
        lastTgtLat   = tgtLat;
        lastTgtLon   = tgtLon;
        lastTgtAlt   = tgtAlt;
        hasLastInject = true;

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
            if (sweepAngle >= 360.0f) { sweepAngle = 0.0f; sweepState = SWEEP_NONE; }
            targetAz = sweepAngle;
            actuators.setTargetAzimuth(targetAz);
            break;

        case SWEEP_EL_FWD:
            sweepAngle += step;
            if (sweepAngle >= TILT_MAX_DEG) { sweepAngle = TILT_MAX_DEG; sweepState = SWEEP_EL_REV; }
            targetEl = sweepAngle;
            actuators.setTiltAngle(targetEl);
            break;

        case SWEEP_EL_REV:
            sweepAngle -= step;
            if (sweepAngle <= TILT_MIN_DEG) { sweepAngle = TILT_MIN_DEG; sweepState = SWEEP_NONE; }
            targetEl = sweepAngle;
            actuators.setTiltAngle(targetEl);
            break;

        default: break;
    }
}

// ============================================================================
//  Pan Axis Update — choose real encoder or simulated
// ============================================================================
void updatePanAxis() {
    actuators.setTargetAzimuth(targetAz);

    if (hasEncoder) {
        actuators.updatePan();
    } else {
        float simPos = actuators.getStepperPositionDeg();
        actuators.updatePanWithPosition(simPos);
    }
}

// ============================================================================
//  Telemetry Output (JSON, 10 Hz)
//
//  Emits raw + interpreted values per sensor (see hil_panel/PROTOCOL.md).
// ============================================================================
void sendTelemetry() {
    // One encoder read per tick; az_c follows the encoder when it is the
    // active source, else the simulated stepper position. Keeping az_c tied
    // to the same read means enc_deg and az_c never disagree.
    uint16_t encRaw  = actuators.readEncoderRaw();
    float    encDeg  = encRaw * (360.0f / 16384.0f);
    float    stepPos = actuators.getStepperPositionDeg();
    float    currentAz = hasEncoder ? encDeg : stepPos;

    const char* sweepStr = "none";
    switch (sweepState) {
        case SWEEP_AZ_FWD: sweepStr = "az"; break;
        case SWEEP_EL_FWD:
        case SWEEP_EL_REV: sweepStr = "el"; break;
        default: break;
    }

    JsonDocument doc;
    doc["t"]  = "tel";
    doc["up"] = millis();

    // Pointing
    doc["az_t"]  = serialized(String(targetAz,  1));
    doc["az_c"]  = serialized(String(currentAz, 1));
    doc["el_t"]  = serialized(String(targetEl,  1));
    doc["el_c"]  = serialized(String(targetEl,  1));  // servo is open-loop
    doc["sweep"] = sweepStr;

    // Pan stepper — raw control signal + interpreted position
    doc["pid"]  = serialized(String(actuators.getPidOutput(),    2));
    doc["sps"]  = serialized(String(actuators.getStepperSpeed(), 1));
    doc["step"] = actuators.getStepCount();
    doc["pos"]  = serialized(String(stepPos, 1));

    // Encoder (AS5048A) — raw counts + interpreted degrees
    doc["enc"]     = hasEncoder;
    doc["enc_raw"] = encRaw;
    doc["enc_deg"] = serialized(String(encDeg, 1));

    // Tilt servo — raw commanded pulse width (interpreted angle == el_c)
    doc["srv_us"] = actuators.getTiltPulseUs();

    // GNSS — raw wire integers (the panel interprets fix label / deg / metres)
    doc["gps"] = hasGNSS;
    if (gnssHardwareOk) {
        doc["gnss_fix"]     = gnss.getFixType();
        doc["gnss_sats"]    = gnss.getSatsUsed();
        doc["gnss_lat_e7"]  = gnss.getLatRaw();
        doc["gnss_lon_e7"]  = gnss.getLonRaw();
        doc["gnss_alt_mm"]  = gnss.getAltRawMM();
        doc["gnss_hacc_mm"] = gnss.getHaccRawMM();
    }

    // Base station position and source
    doc["base_lat"] = serialized(String(baseLat, 6));
    doc["base_lon"] = serialized(String(baseLon, 6));
    doc["base_alt"] = serialized(String((float)baseAlt, 1));
    doc["base_src"] = (hasGNSS && gnssHardwareOk) ? "gnss" : "manual";

    // Last injected target — enables pipeline display on the dashboard
    if (hasLastInject) {
        doc["tgt_lat"] = serialized(String(lastTgtLat, 6));
        doc["tgt_lon"] = serialized(String(lastTgtLon, 6));
        doc["tgt_alt"] = serialized(String((float)lastTgtAlt, 1));
    }

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
