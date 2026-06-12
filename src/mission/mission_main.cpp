// ============================================================================
// mission_main.cpp — Standalone Mission Firmware
// Cursr-V Antenna Tracker
//
// Runs the real mission with no laptop attached. Rocket position arrives as
// 28-byte GroundLinkPacket frames over hardware UART (Serial1, pins 44/43)
// from the ground Heltec. The tracker:
//
//   INIT → WAIT_LINK → PAD_LOCK → ARMED (low-power standby) → TRACKING
//                                                         ↕ SIGNAL_LOST
//
//   - PAD_LOCK averages incoming fixes into a pad position, slews to it once,
//     then de-energises the stepper (zero holding current) and relaxes the
//     servo. The AS5048A encoder is absolute, so nothing is lost.
//   - Launch is detected from the packets themselves (altitude above pad or
//     sustained climb rate) and tracking starts with no operator action.
//   - TRACKING runs the same proven pipeline as the HIL `track` command:
//     WGS84 → ECEF → ENU (vs base) → alpha-beta filter → Az/El → PID + servo.
//
// USB-CDC stays a pure monitor port: the 10 Hz JSON telemetry stream is the
// same schema as the HIL harness (plus mission fields), and inbound commands
// are limited to set_base / set_pid before tracking — everything else is
// rejected, so plugging in a laptop can never disturb a flight.
// ============================================================================
#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "ActuatorControl.h"
#include "Navigation.h"
#include "GNSSManager.h"
#include "SignalFilter.h"
#include "TelemetryPacket.h"

// ============================================================================
//  Mission State Machine
// ============================================================================
enum MissionState : uint8_t {
    M_WAIT_LINK,    // no valid uplink packets yet
    M_PAD_LOCK,     // averaging pad position / waiting for base fix
    M_ARMED,        // pad locked, motors de-energised, watching for launch
    M_TRACKING,     // live flight tracking
    M_SIGNAL_LOST   // tracking, but uplink stale — hold last pointing
};

const char* missionStateName(MissionState s) {
    switch (s) {
        case M_WAIT_LINK:   return "wait_link";
        case M_PAD_LOCK:    return "pad_lock";
        case M_ARMED:       return "armed";
        case M_TRACKING:    return "tracking";
        case M_SIGNAL_LOST: return "signal_lost";
    }
    return "?";
}

// ============================================================================
//  Global Objects
// ============================================================================
Actuators   actuators;
GNSSManager gnss;
PacketStreamParser<GroundLinkPacket> linkParser;

// ============================================================================
//  Sensor / Base State
// ============================================================================
bool hasEncoder     = false;  // AS5048A detected at boot → pan feedback source
bool gnssHardwareOk = false;
bool gnssAutoBaseDone = false;  // one-shot precise-fix adoption (set_base overrides)
bool baseValid      = false;    // a real base exists (GNSS fix or set_base)
bool baseFromGnss   = false;    // base came from the GNSS fix (vs set_base)

double baseLat = HIL_DEFAULT_BASE_LAT;
double baseLon = HIL_DEFAULT_BASE_LON;
double baseAlt = HIL_DEFAULT_BASE_ALT;

// ============================================================================
//  Mission / Link State
// ============================================================================
MissionState mstate = M_WAIT_LINK;
bool motorsLive = false;        // stepper energised + servo PWM running

// Last uplink packet (raw)
GroundLinkPacket lastPkt = {};
unsigned long lastPktMs  = 0;   // millis() of last valid frame
bool          everLinked = false;

// Pad position (EMA of fixes while the rocket sits still)
double   padLat = 0.0, padLon = 0.0;
float    padAlt = 0.0f;
uint16_t padPackets = 0;
unsigned long armedAtMs = 0;    // when ARMED was entered (settle window)

// Launch detection
float   climbRate = 0.0f;       // EMA'd vertical speed (m/s)
float   prevPktAlt = 0.0f;
uint8_t launchHits = 0;         // consecutive packets satisfying launch condition

// Tracking pipeline — same shape as the HIL `track` path
AlphaBetaFilter trackFilterE(AB_ALPHA, AB_BETA, 1.0f / LINK_RATE_HZ);
AlphaBetaFilter trackFilterN(AB_ALPHA, AB_BETA, 1.0f / LINK_RATE_HZ);
AlphaBetaFilter trackFilterU(AB_ALPHA, AB_BETA, 1.0f / LINK_RATE_HZ);
float trkERaw = 0, trkNRaw = 0, trkURaw = 0;
float trkE = 0, trkN = 0, trkU = 0;

// Pointing
float targetAz = 0.0f;
float targetEl = 0.0f;
double lastTgtLat = 0.0, lastTgtLon = 0.0, lastTgtAlt = 0.0;
bool   hasTarget = false;

// ============================================================================
//  Timing / Serial
// ============================================================================
unsigned long lastTelemetryMs = 0;
unsigned long lastPidUs       = 0;
static String usbBuffer;

// ============================================================================
//  Forward Declarations
// ============================================================================
void pollUplink();
void onPacket(const GroundLinkPacket& p);
void trackPacket(const GroundLinkPacket& p);
void setMotorsLive(bool live);
Vec3d enuFromGeodetic(double lat, double lon, double alt);
void pointAtTarget(double lat, double lon, double alt);
void adoptGnssBase();
void resetTrackFilters();
void updatePanAxis();
bool detectEncoder();
void parseUsbJSON();
void processUsbCommand(JsonDocument& doc);
void sendTelemetry();
void sendAck(const char* cmd, JsonDocument* extra = nullptr);
void sendError(const char* msg);

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait for USB CDC (optional host) */ }

    actuators.begin();

    // Uplink UART from the ground Heltec.
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // GNSS for the base station — non-blocking acquisition in loop().
    gnssHardwareOk = gnss.begin();

    usbBuffer.reserve(256);

    // ---- Capture boot heading as North (operator aligns rig, then powers on)
    for (int i = 0; i < 5; ++i) { actuators.readEncoderRaw(); delayMicroseconds(500); }
    actuators.setPanNorthOffset(0.0);

    // ---- Encoder presence: a wired AS5048A never reads all-zeros/all-ones
    hasEncoder = detectEncoder();

    // ---- Startup self-test: prove both axes move before the mission
    actuators.setTiltAngle(45.0f);
    delay(500);
    actuators.setTiltAngle(0.0f);
    delay(300);
    actuators.setTargetAzimuth(15.0f);
    for (int i = 0; i < 500; i++) {
        actuators.updatePanWithPosition(actuators.getStepperPositionDeg());
        actuators.runPan();
        delayMicroseconds(500);
    }
    actuators.setTargetAzimuth(0.0f);
    for (int i = 0; i < 500; i++) {
        actuators.updatePanWithPosition(actuators.getStepperPositionDeg());
        actuators.runPan();
        delayMicroseconds(500);
    }

    // Idle in low power until there is something to point at.
    setMotorsLive(false);

    JsonDocument readyDoc;
    readyDoc["t"]      = "ready";
    readyDoc["ver"]    = "2.0.0";
    readyDoc["mode"]   = "mission";
    readyDoc["board"]  = "ESP32-S3";
    readyDoc["motor"]  = "NEMA17/TB6600";
    readyDoc["servo"]  = "DS51150/300Hz";
    readyDoc["enc"]    = hasEncoder;
    readyDoc["gps_hw"] = gnssHardwareOk;
    serializeJson(readyDoc, Serial);
    Serial.println();

    lastPidUs = micros();
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
    // 1. Uplink packets (the mission data source)
    pollUplink();

    // 2. USB commands (monitor port; pre-flight config only)
    parseUsbJSON();

    // 3. GNSS base station (same auto-adopt pattern as the HIL firmware)
    if (gnssHardwareOk) {
        gnss.update();
        if (!gnssAutoBaseDone
                && gnss.isValid()
                && gnss.getSatsUsed() >= GNSS_PRECISE_MIN_SATS
                && gnss.getHorizAccM() <= GNSS_PRECISE_MAX_HACC_M) {
            adoptGnssBase();
            baseValid = true;
            baseFromGnss = true;
            gnssAutoBaseDone = true;
        }
    }

    // 4. State housekeeping driven by time (packets drive the rest)
    unsigned long nowMs = millis();
    unsigned long age = nowMs - lastPktMs;

    if (mstate == M_PAD_LOCK && age > LINK_TIMEOUT_MS) {
        // Link died before lock completed — start over.
        mstate = M_WAIT_LINK;
        padPackets = 0;
    }
    if (mstate == M_ARMED && motorsLive && nowMs - armedAtMs >= ARM_SETTLE_MS) {
        // Pad slew settled — drop into low-power standby.
        setMotorsLive(false);
    }
    if (mstate == M_TRACKING && age > LINK_TIMEOUT_MS) {
        mstate = M_SIGNAL_LOST;   // hold pointing, motors stay live
    }

    // 5. Pan PID at 200 Hz — only when the stepper is energised
    unsigned long nowUs = micros();
    if (nowUs - lastPidUs >= PID_INTERVAL_US) {
        lastPidUs = nowUs;
        if (motorsLive) updatePanAxis();
    }

    // 6. Stepper pulse generation (non-blocking, every loop)
    actuators.runPan();

    // 7. Telemetry at 10 Hz (emitted whether or not a USB host is attached)
    if (nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryMs = nowMs;
        sendTelemetry();
    }
}

// ============================================================================
//  Uplink — drain Serial1 through the frame parser
// ============================================================================
void pollUplink() {
    GroundLinkPacket pkt;
    while (Serial1.available()) {
        if (linkParser.feed((uint8_t)Serial1.read(), pkt)) {
            onPacket(pkt);
        }
    }
}

// ============================================================================
//  Packet handler — advances the mission state machine
// ============================================================================
void onPacket(const GroundLinkPacket& p) {
    unsigned long nowMs = millis();

    // Climb rate from successive packets (EMA over the raw difference).
    float dtPkt = (nowMs - lastPktMs) * 1e-3f;
    if (everLinked && dtPkt > 0.01f && dtPkt < 2.0f) {
        float v = (p.alt - prevPktAlt) / dtPkt;
        climbRate = 0.6f * climbRate + 0.4f * v;
    } else {
        climbRate = 0.0f;
    }
    prevPktAlt = p.alt;
    lastPkt    = p;
    lastPktMs  = nowMs;
    everLinked = true;

    switch (mstate) {
        case M_WAIT_LINK:
            padLat = p.lat; padLon = p.lon; padAlt = p.alt;
            padPackets = 1;
            mstate = M_PAD_LOCK;
            break;

        case M_PAD_LOCK: {
            // Distance of this fix from the running pad average (flat earth).
            Vec3d enuPad = enuFromGeodetic(padLat, padLon, padAlt);
            Vec3d enuFix = enuFromGeodetic(p.lat, p.lon, p.alt);
            float de = (float)(enuFix.x - enuPad.x);
            float dn = (float)(enuFix.y - enuPad.y);
            float dist = sqrtf(de * de + dn * dn);

            if (dist <= PAD_DRIFT_MAX_M) {
                padLat = 0.9 * padLat + 0.1 * p.lat;
                padLon = 0.9 * padLon + 0.1 * p.lon;
                padAlt = 0.9f * padAlt + 0.1f * p.alt;
                padPackets++;
            } else {
                // Rocket moved (transport to pad, GPS jump) — restart the average.
                padLat = p.lat; padLon = p.lon; padAlt = p.alt;
                padPackets = 1;
            }

            if (padPackets >= PAD_LOCK_MIN_PACKETS && baseValid) {
                // Lock complete: slew once to the pad, then ARM.
                setMotorsLive(true);
                pointAtTarget(padLat, padLon, padAlt);
                launchHits = 0;
                armedAtMs  = millis();
                mstate     = M_ARMED;
            }
            break;
        }

        case M_ARMED: {
            bool launch = (p.alt - padAlt > LAUNCH_ALT_DELTA_M)
                       || (climbRate > LAUNCH_CLIMB_MS);
            launchHits = launch ? (uint8_t)(launchHits + 1) : 0;

            if (launchHits >= LAUNCH_DETECT_SAMPLES) {
                setMotorsLive(true);
                resetTrackFilters();
                mstate = M_TRACKING;
                trackPacket(p);
            }
            break;
        }

        case M_SIGNAL_LOST:
            mstate = M_TRACKING;   // link is back — fall through to track
            // fallthrough
        case M_TRACKING:
            trackPacket(p);
            break;
    }
}

// ============================================================================
//  Tracking pipeline — identical shape to the HIL `track` command:
//  WGS84 packet → precise ECEF→ENU (vs base) → per-axis alpha-beta filter →
//  ENU→WGS84 → computePointing → pan PID + tilt servo.
// ============================================================================
void trackPacket(const GroundLinkPacket& p) {
    Vec3d enu = enuFromGeodetic(p.lat, p.lon, p.alt);
    trkERaw = (float)enu.x;
    trkNRaw = (float)enu.y;
    trkURaw = (float)enu.z;
    trkE = trackFilterE.update(trkERaw);
    trkN = trackFilterN.update(trkNRaw);
    trkU = trackFilterU.update(trkURaw);

    Vec3d g = enuToGeodetic(trkE, trkN, trkU, baseLat, baseLon, baseAlt);
    pointAtTarget(g.x, g.y, g.z);
}

// ============================================================================
//  Helpers
// ============================================================================

// Local ENU of a WGS84 point relative to the base station (precise transform).
Vec3d enuFromGeodetic(double lat, double lon, double alt) {
    Vec3d b = geodeticToECEF(baseLat, baseLon, baseAlt);
    Vec3d t = geodeticToECEF(lat, lon, alt);
    Vec3d d = { t.x - b.x, t.y - b.y, t.z - b.z };
    return ecefToENU(d, baseLat, baseLon);
}

// Same contract as the HIL helper: compute Az/El, clamp tilt, drive both
// axes, record the target for telemetry.
void pointAtTarget(double lat, double lon, double alt) {
    PointingAngles pa = computePointing(baseLat, baseLon, baseAlt,
                                        lat, lon, alt);
    targetAz = (float)pa.azimuth;
    targetEl = constrain((float)pa.elevation, TILT_MIN_DEG, TILT_MAX_DEG);

    actuators.setTargetAzimuth(targetAz);
    actuators.setTiltAngle(targetEl);

    lastTgtLat = lat;
    lastTgtLon = lon;
    lastTgtAlt = alt;
    hasTarget  = true;
}

void adoptGnssBase() {
    baseLat = gnss.getLat();
    baseLon = gnss.getLon();
    baseAlt = gnss.getAlt();
}

void resetTrackFilters() {
    trackFilterE.reset();
    trackFilterN.reset();
    trackFilterU.reset();
}

void setMotorsLive(bool live) {
    if (live == motorsLive) return;
    motorsLive = live;
    actuators.setPanEnabled(live);
    actuators.setTiltActive(live);
}

void updatePanAxis() {
    actuators.setTargetAzimuth(targetAz);
    if (hasEncoder) {
        actuators.updatePan();
    } else {
        actuators.updatePanWithPosition(actuators.getStepperPositionDeg());
    }
}

// A wired AS5048A returns a varying mid-range count; a floating/absent MISO
// reads all zeros (or all ones). Sample a few times and reject the stuck rails.
bool detectEncoder() {
    for (int i = 0; i < 5; ++i) {
        uint16_t raw = actuators.readEncoderRaw();
        if (raw != 0x0000 && raw != 0x3FFF) return true;
        delayMicroseconds(500);
    }
    return false;
}

// ============================================================================
//  USB Command Port — monitor-first, pre-flight config only
// ============================================================================
void parseUsbJSON() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (usbBuffer.length() > 0) {
                JsonDocument doc;
                DeserializationError err = deserializeJson(doc, usbBuffer);
                if (err) {
                    sendError(err.c_str());
                } else {
                    processUsbCommand(doc);
                }
                usbBuffer = "";
            }
        } else if (usbBuffer.length() < 256) {
            usbBuffer += c;
        }
    }
}

void processUsbCommand(JsonDocument& doc) {
    const char* cmd = doc["cmd"];
    if (!cmd) {
        sendError("missing cmd field");
        return;
    }

    // During flight nothing may interfere.
    bool inFlight = (mstate == M_TRACKING || mstate == M_SIGNAL_LOST);

    if (strcmp(cmd, "set_base") == 0 && !inFlight) {
        if (doc["lat"].isNull() || doc["lon"].isNull() || doc["alt"].isNull()) {
            sendError("set_base: missing lat/lon/alt");
            return;
        }
        baseLat = doc["lat"].as<double>();
        baseLon = doc["lon"].as<double>();
        baseAlt = doc["alt"].as<double>();
        baseValid = true;
        baseFromGnss = false;
        gnssAutoBaseDone = true;   // manual base overrides GNSS adoption
        sendAck("set_base");
    }
    else if (strcmp(cmd, "set_pid") == 0 && !inFlight) {
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
        sendAck("set_pid");
    }
    else {
        sendError("mission mode: read-only");
    }
}

// ============================================================================
//  Telemetry — HIL `tel` schema plus mission fields, 10 Hz
// ============================================================================
void sendTelemetry() {
    uint16_t encRaw  = actuators.readEncoderRaw();
    float    encDeg  = actuators.panAngleFromRaw(encRaw);
    float    stepPos = actuators.getStepperPositionDeg();
    float    currentAz = hasEncoder ? encDeg : stepPos;

    JsonDocument doc;
    doc["t"]    = "tel";
    doc["up"]   = millis();
    doc["mode"] = "mission";

    // Mission state + uplink quality
    doc["mstate"] = missionStateName(mstate);
    doc["mot"]    = motorsLive;
    doc["rx_ok"]  = linkParser.okCount;
    doc["rx_bad"] = linkParser.badCount;
    doc["age_ms"] = everLinked ? (millis() - lastPktMs) : -1;
    if (everLinked) {
        doc["seq"]  = lastPkt.seq;
        doc["rssi"] = lastPkt.rssi_dbm;
        doc["snr"]  = serialized(String(lastPkt.snr_db_x4 / 4.0f, 1));
        doc["rkt_lat"] = serialized(String(lastPkt.lat, 6));
        doc["rkt_lon"] = serialized(String(lastPkt.lon, 6));
        doc["rkt_alt"] = serialized(String(lastPkt.alt, 1));
        doc["clb"]     = serialized(String(climbRate, 1));
    }
    if (padPackets > 0) {
        doc["pad_lat"] = serialized(String(padLat, 6));
        doc["pad_lon"] = serialized(String(padLon, 6));
        doc["pad_alt"] = serialized(String(padAlt, 1));
        doc["pad_n"]   = padPackets;
    }

    // Pointing (same fields as HIL)
    doc["az_t"] = serialized(String(targetAz,  1));
    doc["az_c"] = serialized(String(currentAz, 1));
    doc["el_t"] = serialized(String(targetEl,  1));
    doc["el_c"] = serialized(String(targetEl,  1));   // servo is open-loop

    // Pan stepper — raw control signal + interpreted position
    doc["pid"]  = serialized(String(actuators.getPidOutput(),    2));
    doc["sps"]  = serialized(String(actuators.getStepperSpeed(), 1));
    doc["step"] = actuators.getStepCount();
    doc["pos"]  = serialized(String(stepPos, 1));

    // Encoder
    doc["enc"]     = hasEncoder;
    doc["enc_raw"] = encRaw;
    doc["enc_deg"] = serialized(String(encDeg, 1));

    // Tilt servo
    doc["srv_us"] = actuators.getTiltPulseUs();

    // GNSS raw wire values (panel interprets)
    doc["gps"] = baseFromGnss;
    if (gnssHardwareOk) {
        doc["gnss_fix"]     = gnss.getFixType();
        doc["gnss_sats"]    = gnss.getSatsUsed();
        doc["gnss_lat_e7"]  = gnss.getLatRaw();
        doc["gnss_lon_e7"]  = gnss.getLonRaw();
        doc["gnss_alt_mm"]  = gnss.getAltRawMM();
        doc["gnss_hacc_mm"] = gnss.getHaccRawMM();
    }

    // Base station
    doc["base_lat"] = serialized(String(baseLat, 6));
    doc["base_lon"] = serialized(String(baseLon, 6));
    doc["base_alt"] = serialized(String((float)baseAlt, 1));
    doc["base_src"] = baseValid ? (baseFromGnss ? "gnss" : "manual") : "default";

    // Current pointing target
    if (hasTarget) {
        doc["tgt_lat"] = serialized(String(lastTgtLat, 6));
        doc["tgt_lon"] = serialized(String(lastTgtLon, 6));
        doc["tgt_alt"] = serialized(String((float)lastTgtAlt, 1));
    }

    // Filter view (same fields the HIL replay uses)
    bool tracking = (mstate == M_TRACKING || mstate == M_SIGNAL_LOST);
    doc["trk"] = tracking;
    if (tracking) {
        doc["trk_e_raw"] = serialized(String(trkERaw, 1));
        doc["trk_n_raw"] = serialized(String(trkNRaw, 1));
        doc["trk_u_raw"] = serialized(String(trkURaw, 1));
        doc["trk_e"]     = serialized(String(trkE, 1));
        doc["trk_n"]     = serialized(String(trkN, 1));
        doc["trk_u"]     = serialized(String(trkU, 1));
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
