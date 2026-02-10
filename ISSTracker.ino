#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

// ---------------------- MATH STRUCTS ----------------------
struct Vec3 { double x, y, z; };

// ---------------------- WIFI / OTA ----------------------
static const char* WIFI_SSID = "reese-wlan";
static const char* WIFI_PASS = "kjt94v28";

static const char* AP_SSID = "ISS-Tracker";
static const char* AP_PASS = "iss-tracker";

static const char* OTA_HOSTNAME = "iss-tracker";
static const char* OTA_PASSWORD = "F9jb76q2";

// ---------------------- PINS ----------------------
const int IN1 = 25;
const int IN2 = 26;
const int IN3 = 27;
const int IN4 = 14;

const int HALL_PIN = 33;
// Your observed behavior: raw=0 behaves "active" (index latch), raw=1 behaves "inactive" (reset latch)
const bool HALL_ACTIVE_LOW = true;

const int SERVO_PIN = 32;

// ---------------------- MECH / TUNING ----------------------
const long  STEPS_PER_REV = 8192;
const float STEPS_PER_DEG = (float)STEPS_PER_REV / 360.0f;

// Flip if your axis is mirrored
const bool AZ_INVERT = true;

// From your logs: reset edge happens ~1992 steps after index (when index=0)
const long RESET_OFFSET_STEPS = 1992;

// Drift threshold: if we hit a hall landmark and we're off by more than this many steps,
// stop tracking and re-home.
const long HALL_DRIFT_THRESHOLD_STEPS = 150;

// homing
const float SEEK_SPEED   = 350.0f;
const float SEEK_ACCEL   = 600.0f;
const float VERIFY_SPEED = 180.0f;
const float VERIFY_ACCEL = 300.0f;

// tracking/jog motion
const float MOVE_SPEED = 380.0f;
const float MOVE_ACCEL = 650.0f;

const long  JOG_STEP_DELTA = 200;

// Retarget gating: don't accept a new target until we're close to current target
// Tune: smaller = "finish moves before retarget", larger = "more willing to retarget"
const long RETARGET_THRESHOLD_STEPS = 250;

// Servo
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;


// ---------------------- OLED (SSD1306 128x64 I2C) ----------------------
static const int OLED_W = 128;
static const int OLED_H = 64;
static const uint8_t OLED_ADDR = 0x3C;   // common; some modules use 0x3D
static const int OLED_RESET_PIN = -1;    // not used with I2C modules

Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, OLED_RESET_PIN);
bool oledOk = false;
unsigned long lastOledMs = 0;

void setupOLED();
void updateOLED();
// ---------------------- OBSERVER LOCATION ----------------------
// Defaults (Bakersfield-ish). These are persisted in NVS once changed via UI.
static const double OBS_LAT_DEFAULT_DEG = 35.3733;
static const double OBS_LON_DEFAULT_DEG = -119.0187;
static const double OBS_ALT_DEFAULT_M   = 120.0;

// Runtime (mutable) observer location used by ISS tracking math.
double obsLatDeg = OBS_LAT_DEFAULT_DEG;
double obsLonDeg = OBS_LON_DEFAULT_DEG;
double obsAltM    = OBS_ALT_DEFAULT_M;

// ---------------------- ISS API ----------------------
static const char* ISS_URL = "https://api.wheretheiss.at/v1/satellites/25544";

// ---------------------- NETWORK TIMEOUTS ----------------------
const uint32_t ISS_HTTP_TIMEOUT_MS = 500;  // your preference

// Dynamic polling based on elevation (hysteresis)
const uint32_t ISS_POLL_SLOW_MS = 2000;
const uint32_t ISS_POLL_FAST_MS = 500;

const double FAST_ENTER_EL_DEG = 20.0; // enter FAST when el >= 20
const double FAST_EXIT_EL_DEG  = 15.0; // exit FAST when el <= 15

// ---------------------- OBJECTS ----------------------
WebServer server(80);
Preferences prefs;

AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);
Servo elevServo;

// ---------------------- STATE ----------------------
enum HomeState { ST_IDLE, ST_SEEK_RESET, ST_SEEK_INDEX, ST_VERIFY, ST_DONE, ST_FAIL };
volatile HomeState homeState = ST_IDLE;

unsigned long phaseStartMs = 0;
unsigned long phaseTimeoutMs = 0;

volatile long northOffsetSteps = 0;
volatile bool isHomed = false;
volatile bool hasNorthOffset = false;

volatile int servoAngle = 90;       // UI-friendly
int servoZeroDeg = 90;              // servo angle that corresponds to 0° elevation (level)
volatile int servoPulseUs = 1500;   // actual commanded

// Hall transition tracking
volatile int lastHallRaw = HIGH;

// Tracking control
volatile bool trackingEnabled = false;
volatile bool manualOverride = false; // set true on any manual action; Start Tracking clears it

// Re-home request (triggered by drift checks)
volatile bool rehomeRequested = false;
volatile bool resumeTrackingAfterRehome = false;
volatile bool autoStartAfterHoming = false;

// Background task shared data
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

volatile double issLatDeg = 0, issLonDeg = 0, issAltM = 0;
volatile double targetAzDeg = 0, targetElDeg = 0;
volatile bool haveTarget = false;

volatile bool havePendingTarget = false;
volatile double pendingAzDeg = 0;
volatile double pendingElDeg = 0;

volatile int lastIssHttpCode = 0;
volatile unsigned long lastIssOkMs = 0;
String lastIssErr = "";

volatile bool fastMode = false;      // hysteresis state
volatile uint32_t taskPollMs = 0;    // for UI debugging

TaskHandle_t issTaskHandle = nullptr;

// ---------------------- HELPERS ----------------------
int hallRaw() { return digitalRead(HALL_PIN); }

bool hallActive() {
  int v = hallRaw();
  return HALL_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void setPhaseTimeoutForRevs(float stepsPerSec, float revs, unsigned long fudgeMs = 5000) {
  if (stepsPerSec < 1.0f) stepsPerSec = 1.0f;
  if (revs < 0.1f) revs = 0.1f;
  float seconds = (STEPS_PER_REV * revs) / stepsPerSec;
  phaseStartMs = millis();
  phaseTimeoutMs = (unsigned long)(seconds * 1000.0f) + fudgeMs;
}

bool phaseTimedOut() { return (millis() - phaseStartMs) > phaseTimeoutMs; }

void setServoAngleClamped(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  servoAngle = angle;

  // Keep manual servo control consistent with tracking (use configured pulse range)
  double elNorm = ((double)servoAngle) / 180.0;
  if (elNorm < 0.0) elNorm = 0.0;
  if (elNorm > 1.0) elNorm = 1.0;
  int pulseUs = SERVO_MIN_US + (int)((SERVO_MAX_US - SERVO_MIN_US) * elNorm);
  servoPulseUs = pulseUs;
  elevServo.writeMicroseconds(pulseUs);

  prefs.putInt("servoAngle", servoAngle);
}


String wifiModeString() {
  if (WiFi.getMode() == WIFI_AP) return "AP";
  if (WiFi.getMode() == WIFI_STA) return "STA";
  return "UNKNOWN";
}

String ipString() {
  if (WiFi.getMode() == WIFI_AP) return WiFi.softAPIP().toString();
  return WiFi.localIP().toString();
}


// Minimal JSON string escaper (enough for /api/status payload)
static String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    const char c = in[i];
    switch (c) {
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n':  out += "\\n";  break;
      case '\r':  out += "\\r";  break;
      case '\t':  out += "\\t";  break;
      default:
        if ((uint8_t)c >= 0x20) out += c;
        break;
    }
  }
  return out;
}

// Normalize degrees to [0,360)
double wrap360(double deg) {
  while (deg < 0) deg += 360.0;
  while (deg >= 360.0) deg -= 360.0;
  return deg;
}

// Choose shortest rotation to reach desired azimuth (north-referenced degrees)
long chooseShortestTargetStepsFromAzDeg(double desiredAzDeg) {
  desiredAzDeg = wrap360(desiredAzDeg);

  long currentSteps = stepper.currentPosition();

  long desiredRelSteps = (long)llround((AZ_INVERT ? -desiredAzDeg : desiredAzDeg) * STEPS_PER_DEG);
  long currentRelSteps = currentSteps - (long)northOffsetSteps;

  long curMod = currentRelSteps % STEPS_PER_REV;
  if (curMod < 0) curMod += STEPS_PER_REV;

  long desMod = desiredRelSteps % STEPS_PER_REV;
  if (desMod < 0) desMod += STEPS_PER_REV;

  long delta = desMod - curMod;

  if (delta > (STEPS_PER_REV / 2))  delta -= STEPS_PER_REV;
  if (delta < -(STEPS_PER_REV / 2)) delta += STEPS_PER_REV;

  return currentSteps + delta;
}

// ---------------------- HALL LANDMARK TRACKING (NO SNAP DURING TRACKING) ----------------------
static inline long modSteps(long x) {
  long m = x % STEPS_PER_REV;
  if (m < 0) m += STEPS_PER_REV;
  return m;
}

static inline long wrapSignedSteps(long x) {
  // wrap to (-STEPS_PER_REV/2 .. +STEPS_PER_REV/2]
  if (x >  (STEPS_PER_REV / 2)) x -= STEPS_PER_REV;
  if (x < -(STEPS_PER_REV / 2)) x += STEPS_PER_REV;
  return x;
}

void updateHallLandmarkSnap() {
  // NOTE: despite the name, this function no longer "snaps" the stepper origin.
  // With a latching hall sensor, snapping during tracking causes reference-frame jumps.
  // We only use hall transitions to detect drift and trigger a re-home if needed.
  int h = hallRaw();
  if (h == lastHallRaw) return;

  int from = lastHallRaw;
  int to   = h;
  lastHallRaw = h;

  // Only consider drift checks while actively tracking (not manual, not homing)
  if (!trackingEnabled || manualOverride) return;
  if (!isHomed || !hasNorthOffset) return;
  if (homeState != ST_IDLE && homeState != ST_DONE && homeState != ST_FAIL) return;

  // Expected landmark positions are defined relative to HOME (red magnet) where raw becomes LOW (active=true).
  // When raw transitions to LOW -> HOME landmark (expected mod position ~0).
  // When raw transitions to HIGH -> RESET landmark (expected mod position ~RESET_OFFSET_STEPS).
  long expected = (to == LOW) ? 0 : RESET_OFFSET_STEPS;

  long pos = stepper.currentPosition();
  long posMod = modSteps(pos);
  long expMod = modSteps(expected);

  long err = wrapSignedSteps(posMod - expMod);

  if (labs(err) > HALL_DRIFT_THRESHOLD_STEPS) {
    // Flag a re-home. The main loop will stop tracking and run the homing routine.
    rehomeRequested = true;
    resumeTrackingAfterRehome = true;
  }
}


// ---------------------- HOMING (LATCHING HALL, CCW ONLY) ----------------------
// Confirmed hardware mapping (per Robert):
// - Red magnet sets hall raw LOW  -> hallActive()==true  (HOME landmark)
// - Blue magnet sets hall raw HIGH -> hallActive()==false (RESET landmark), ~90deg from HOME
//
// Homing strategy (CCW only, no reverse/backoff):
// - If we boot in HOME state (hallActive==true), first seek RESET (hallActive==false), then seek HOME again.
// - If we boot in RESET state (hallActive==false), seek HOME directly.
// We only call setCurrentPosition(0) when we hit HOME (hallActive==true).

void startHoming() {
  // Caller decides whether to auto-start tracking after homing via autoStartAfterHoming / resumeTrackingAfterRehome.
  homeState = ST_SEEK_RESET;
  isHomed = false;
  prefs.putBool("isHomed", false);

  stepper.setMaxSpeed(SEEK_SPEED);
  stepper.setAcceleration(SEEK_ACCEL);
  stepper.setSpeed(SEEK_SPEED); // CCW (per existing build wiring)

  lastHallRaw = hallRaw();

  // If we're already in RESET state (hallActive==false), skip directly to seeking HOME.
  if (!hallActive()) {
    homeState = ST_SEEK_INDEX;
    setPhaseTimeoutForRevs(SEEK_SPEED, 1.1f, 5000);
  } else {
    // We are in HOME band, so seek RESET first (<=90deg), then HOME.
    homeState = ST_SEEK_RESET;
    setPhaseTimeoutForRevs(SEEK_SPEED, 0.5f, 5000);
  }

  trackingEnabled = false;
  manualOverride = true;

  portENTER_CRITICAL(&dataMux);
  havePendingTarget = false;
  portEXIT_CRITICAL(&dataMux);
}

void finishHoming(bool success) {
  stepper.setSpeed(0);
  stepper.stop();

  if (!success) {
    homeState = ST_FAIL;
    isHomed = false;
    prefs.putBool("isHomed", false);
    trackingEnabled = false;
    manualOverride = true;
  } else {
    homeState = ST_DONE;
    isHomed = true;
    prefs.putBool("isHomed", true);

    // Auto-resume logic:
    // - If we triggered a re-home while tracking, resume automatically (if north offset exists).
    // - On boot, autoStartAfterHoming may be set true to auto-start once homed.
    bool shouldResume = (resumeTrackingAfterRehome || autoStartAfterHoming) && hasNorthOffset;

    if (shouldResume) {
      manualOverride = false;
      trackingEnabled = true;

      portENTER_CRITICAL(&dataMux);
      havePendingTarget = false;
      haveTarget = false;
      fastMode = false;
      taskPollMs = ISS_POLL_SLOW_MS;
      portEXIT_CRITICAL(&dataMux);
    } else {
      trackingEnabled = false;
      manualOverride = true;
    }
  }

  // Clear one-shot flags
  rehomeRequested = false;
  resumeTrackingAfterRehome = false;
  autoStartAfterHoming = false;
}

void updateHoming() {
  if (homeState == ST_IDLE || homeState == ST_DONE || homeState == ST_FAIL) return;

  if (phaseTimedOut()) {
    finishHoming(false);
    return;
  }

  stepper.runSpeed();

  // track hall transitions
  int h = hallRaw();
  if (h != lastHallRaw) lastHallRaw = h;

  switch (homeState) {
    case ST_SEEK_RESET:
      // seek blue marker => hallActive()==false (raw HIGH)
      if (!hallActive()) {
        homeState = ST_SEEK_INDEX;
        setPhaseTimeoutForRevs(SEEK_SPEED, 1.1f, 5000);
      }
      break;

    case ST_SEEK_INDEX:
      // seek red HOME marker => hallActive()==true (raw LOW)
      if (hallActive()) {
        // define absolute zero at HOME
        stepper.setCurrentPosition(0);
        finishHoming(true);
      }
      break;

    default:
      break;
  }
}

// ---------------------- ISS MATH (ECEF -> ENU -> Az/El) ---------------------- (ECEF -> ENU -> Az/El) ----------------------
static inline double deg2rad(double d) { return d * 0.017453292519943295; }
static inline double rad2deg(double r) { return r * 57.29577951308232; }

const double EARTH_RADIUS_M = 6371000.0;

Vec3 llhToEcef(double latDeg, double lonDeg, double altM) {
  double lat = deg2rad(latDeg);
  double lon = deg2rad(lonDeg);
  double r = EARTH_RADIUS_M + altM;

  Vec3 v;
  v.x = r * cos(lat) * cos(lon);
  v.y = r * cos(lat) * sin(lon);
  v.z = r * sin(lat);
  return v;
}

Vec3 ecefToEnuDelta(const Vec3& d, double obsLatDeg, double obsLonDeg) {
  double lat = deg2rad(obsLatDeg);
  double lon = deg2rad(obsLonDeg);

  double sinLat = sin(lat), cosLat = cos(lat);
  double sinLon = sin(lon), cosLon = cos(lon);

  Vec3 enu;
  enu.x = -sinLon * d.x + cosLon * d.y;                                  // East
  enu.y = -sinLat * cosLon * d.x - sinLat * sinLon * d.y + cosLat * d.z; // North
  enu.z =  cosLat * cosLon * d.x + cosLat * sinLon * d.y + sinLat * d.z; // Up
  return enu;
}

void computeAzElDeg(double obsLat, double obsLon, double obsAlt,
                    double tgtLat, double tgtLon, double tgtAlt,
                    double &azDegOut, double &elDegOut) {
  Vec3 obs = llhToEcef(obsLat, obsLon, obsAlt);
  Vec3 tgt = llhToEcef(tgtLat, tgtLon, tgtAlt);

  Vec3 d{ tgt.x - obs.x, tgt.y - obs.y, tgt.z - obs.z };
  Vec3 enu = ecefToEnuDelta(d, obsLat, obsLon);

  double east  = enu.x;
  double north = enu.y;
  double up    = enu.z;

  double az = atan2(east, north);
  double horiz = sqrt(east*east + north*north);
  double el = atan2(up, horiz);

  azDegOut = wrap360(rad2deg(az));
  elDegOut = rad2deg(el);
}

// ---------------------- ISS FETCH ----------------------
bool extractJsonNumber(const String& body, const char* key, double &outVal) {
  String pattern = String("\"") + key + "\":";
  int idx = body.indexOf(pattern);
  if (idx < 0) return false;
  idx += pattern.length();

  while (idx < (int)body.length() && (body[idx] == ' ')) idx++;

  int end = idx;
  while (end < (int)body.length()) {
    char c = body[end];
    if (c == ',' || c == '}' || c == '\n' || c == '\r') break;
    end++;
  }
  String num = body.substring(idx, end);
  num.trim();
  outVal = num.toDouble();
  return true;
}

bool fetchIssOnce(double &lat, double &lon, double &altM_out, int &httpCode, String &err) {
  err = "";
  httpCode = 0;

  if (WiFi.getMode() != WIFI_STA || WiFi.status() != WL_CONNECTED) {
    err = "WiFi not connected";
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(ISS_HTTP_TIMEOUT_MS);

  HTTPClient http;
  http.setTimeout(ISS_HTTP_TIMEOUT_MS);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, ISS_URL)) {
    err = "http.begin() failed";
    return false;
  }

  int code = http.GET();
  httpCode = code;

  if (code <= 0) {
    err = String("GET failed: ") + http.errorToString(code);
    http.end();
    return false;
  }
  if (code != 200) {
    err = String("HTTP ") + code;
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();

  double aKm;
  if (!extractJsonNumber(body, "latitude", lat))  { err = "parse latitude"; return false; }
  if (!extractJsonNumber(body, "longitude", lon)) { err = "parse longitude"; return false; }
  if (!extractJsonNumber(body, "altitude", aKm))  { err = "parse altitude"; return false; }

  altM_out = aKm * 1000.0;
  return true;
}

// ---------------------- ISS BACKGROUND TASK ----------------------
void issTask(void* pv) {
  uint32_t pollMs = ISS_POLL_SLOW_MS;
  fastMode = false;
  taskPollMs = pollMs;

  for (;;) {
    // If we aren't tracking or user is manually controlling, idle
    if (!trackingEnabled || manualOverride) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // If homing is running, don't fetch
    if (homeState != ST_IDLE && homeState != ST_DONE && homeState != ST_FAIL) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    double lat = 0, lon = 0, altM = 0;
    int code = 0;
    String err;

    bool ok = fetchIssOnce(lat, lon, altM, code, err);

    portENTER_CRITICAL(&dataMux);
    lastIssHttpCode = code;
    lastIssErr = err;

    if (ok) {
      issLatDeg = lat;
      issLonDeg = lon;
      issAltM   = altM;

      double az, el;
      computeAzElDeg(obsLatDeg, obsLonDeg, obsAltM,
                     lat, lon, altM,
                     az, el);

      targetAzDeg = az;
      targetElDeg = el;
      haveTarget = true;

      // Queue latest target for main loop to commit when stepper is ready
      pendingAzDeg = az;
      pendingElDeg = el;
      havePendingTarget = true;

      lastIssOkMs = millis();

      // Elevation hysteresis
      if (!fastMode) {
        if (el >= FAST_ENTER_EL_DEG) fastMode = true;
      } else {
        if (el <= FAST_EXIT_EL_DEG) fastMode = false;
      }

      pollMs = fastMode ? ISS_POLL_FAST_MS : ISS_POLL_SLOW_MS;
    }

    taskPollMs = pollMs;
    portEXIT_CRITICAL(&dataMux);

    vTaskDelay(pdMS_TO_TICKS(pollMs));
  }
}

// ---------------------- TRACKING MOTION (MAIN LOOP) ----------------------
void updateTrackingMotion() {
  if (!trackingEnabled || manualOverride) return;

  updateHallLandmarkSnap();

  bool commit = false;
  double az = 0, el = 0;

  portENTER_CRITICAL(&dataMux);
  bool pending = havePendingTarget;
  if (pending) {
    // Only commit a new target when we're close to finishing the current one
    if (labs(stepper.distanceToGo()) <= RETARGET_THRESHOLD_STEPS || stepper.distanceToGo() == 0) {
      az = pendingAzDeg;
      el = pendingElDeg;
      havePendingTarget = false;
      commit = true;
    }
  }
  portEXIT_CRITICAL(&dataMux);

  if (commit) {
    // Azimuth
    long targetSteps = chooseShortestTargetStepsFromAzDeg(az);
    stepper.setMaxSpeed(MOVE_SPEED);
    stepper.setAcceleration(MOVE_ACCEL);
    stepper.moveTo(targetSteps);

    // Elevation mapping: astro elevation (-90..+90) -> servo (0..180)
    // Calibrated so that servoZeroDeg corresponds to 0° elevation (level)
    double elServoDeg = (double)servoZeroDeg + el;
    if (elServoDeg < 0.0) elServoDeg = 0.0;
    if (elServoDeg > 180.0) elServoDeg = 180.0;
    double elNorm = elServoDeg / 180.0;
    if (elNorm < 0.0) elNorm = 0.0;
    if (elNorm > 1.0) elNorm = 1.0;

    int pulseUs = SERVO_MIN_US + (int)((SERVO_MAX_US - SERVO_MIN_US) * elNorm);
    servoPulseUs = pulseUs;
    elevServo.writeMicroseconds(pulseUs);

    servoAngle = (int)lround(elServoDeg);
  }

  stepper.run();
}

// ---------------------- WIFI / OTA ----------------------
void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) delay(200);

  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
}

void setupOTA() {
  if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
    MDNS.begin(OTA_HOSTNAME);
  }
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    trackingEnabled = false;
    manualOverride = true;
    stepper.stop();
    stepper.setSpeed(0);
  });

  ArduinoOTA.begin();
}

// ---------------------- OLED (SSD1306 128x64 I2C) ----------------------
void setupOLED() {
  // Default ESP32 I2C pins: SDA=21, SCL=22
  Wire.begin();

  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOk) return;

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("ISS Tracker");
  oled.println("OLED online");
  oled.display();
  lastOledMs = 0;
}

static const char* uiStateLabel() {
  if (homeState != ST_IDLE && homeState != ST_DONE && homeState != ST_FAIL) return "HOMING";
  if (trackingEnabled && !manualOverride) return "TRACK";
  if (manualOverride) return "MANUAL";
  return "IDLE";
}

void updateOLED() {
  if (!oledOk) return;

  unsigned long now = millis();
  if (now - lastOledMs < 500) return;
  lastOledMs = now;

  // Snapshot shared data
  double issLat = 0, issLon = 0, tAz = 0, tEl = 0;
  unsigned long ageMs = 0;
  int httpCode = 0;
  portENTER_CRITICAL(&dataMux);
  issLat = issLatDeg;
  issLon = issLonDeg;
  tAz = targetAzDeg;
  tEl = targetElDeg;
  httpCode = lastIssHttpCode;
  ageMs = (lastIssOkMs == 0) ? 0 : (millis() - lastIssOkMs);
  portEXIT_CRITICAL(&dataMux);

  String ipStr = (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED)
                   ? WiFi.localIP().toString()
                   : WiFi.softAPIP().toString();

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // Line 1: IP
  oled.setCursor(0, 0);
  oled.print("IP ");
  oled.print(ipStr);

  // Line 2: RSSI + state
  oled.setCursor(0, 10);
  oled.print("RSSI ");
  oled.print(WiFi.RSSI());
  oled.print("  ");
  oled.print(uiStateLabel());

  // Line 3: readiness / hints
  oled.setCursor(0, 20);
  if (!isHomed) {
    oled.print("Need: HOME");
  } else if (!hasNorthOffset) {
    oled.print("Need: SET NORTH");
  } else {
    oled.print("Az ");
    oled.print(tAz, 0);
    oled.print(" El ");
    oled.print(tEl, 0);
  }

  // Line 4: ISS lat/lon (rounded)
  oled.setCursor(0, 30);
  oled.print("ISS ");
  oled.print(issLat, 1);
  oled.print(",");
  oled.print(issLon, 1);

  // Line 5: data age / HTTP
  oled.setCursor(0, 40);
  oled.print("Age ");
  oled.print((unsigned long)(ageMs / 1000));
  oled.print("s  H ");
  oled.print(httpCode);

  // Line 6: pass stub (we'll fill later)
  oled.setCursor(0, 50);
  oled.print("Next: TBD");

  oled.display();
}


// ---------------------- WEB UI ----------------------
String htmlPage() {
  String s;
  s.reserve(12000);

  s += R"HTML(
<!doctype html><html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ISS Tracker Bench UI</title>
<style>
  :root {
    --bg: #0b0f14;
    --card: #121a24;
    --border: #2a3442;
    --text: #e6eaf0;
    --muted: #aab4c0;
    --btn: #1b2736;
    --btnBorder: #2b3a4c;
    --btnHover: #233246;
    --accent: #6d5efc;
    --danger: #d0636c;
    --input: #0f1620;
  }

  body { font-family: system-ui, sans-serif; margin: 16px; background: var(--bg); color: var(--text); }
  h2 { text-align: center; margin: 10px 0 16px; letter-spacing: 0.2px; }
  .wrap { max-width: 980px; margin: 0 auto; }

  .card { background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 14px; margin-bottom: 12px; box-shadow: 0 2px 10px rgba(0,0,0,0.25); }
  button { padding: 12px 14px; margin: 6px; border-radius: 12px; border: 1px solid var(--btnBorder); background: var(--btn); color: var(--text); cursor: pointer; }
  button:hover { background: var(--btnHover); }
  button:active { transform: translateY(1px); }
  button.primary { border-color: rgba(109,94,252,0.75); }
  button.danger { border-color: rgba(208,99,108,0.75); }

  .row { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; }
  .label { min-width: 140px; font-weight: 600; color: var(--muted); }
  code { background: rgba(255,255,255,0.06); padding: 2px 6px; border-radius: 6px; }

  .controlsTop { display: flex; gap: 18px; align-items: center; flex-wrap: wrap; justify-content: center; }
  .dpad { display: flex; flex-direction: column; align-items: center; gap: 8px; }
  .dpadMid { display: flex; align-items: center; gap: 8px; }
  .dpadBtn { width: 58px; height: 48px; font-size: 20px; padding: 0; }
  .dpadCenter { width: 58px; height: 48px; }

  .sideBtns { display: flex; flex-direction: column; gap: 10px; align-items: stretch; }
  .sideBtns button { min-width: 160px; margin: 0; }

  .statusGrid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }

  .text { width: 140px; padding: 10px 10px; border-radius: 12px; border: 1px solid var(--btnBorder); background: var(--input); color: var(--text); }
  .text.obs { width: 112px; padding: 8px 10px; margin: 0; }
  .obsLbl { color: var(--muted); font-weight: 650; font-size: 13px; margin-right: 4px; }
  .observerInline { width: 100%; margin-top: 8px; }
  .observerRow { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; justify-content: center; }
  button.small { padding: 10px 12px; margin: 0; border-radius: 12px; }


  .hint { color: var(--muted); font-size: 13px; margin-top: 8px; }

  @media (max-width: 720px) {
    .statusGrid { grid-template-columns: 1fr; }
    .sideBtns { flex-direction: row; flex-wrap: wrap; justify-content: center; }
    .sideBtns button { min-width: 140px; }
  }
</style>
</head>
<body>
<div class="wrap">
<h2>ISS Tracker Bench UI</h2>

<div class="card">
  <div class="controlsTop">
    <div class="dpad">
      <button class="dpadBtn" onclick="nudgeServo(+5)" title="Up">▲</button>
      <div class="dpadMid">
        <button class="dpadBtn" onclick="api('/api/jog?dir=-1')" title="Left">◀</button>
        <div class="dpadCenter"></div>
        <button class="dpadBtn" onclick="api('/api/jog?dir=+1')" title="Right">▶</button>
      </div>
      <button class="dpadBtn" onclick="nudgeServo(-5)" title="Down">▼</button>
    </div>

    <div class="sideBtns">
      <button onclick="api('/api/home')">Home</button>
      <button onclick="api('/api/set_north')">Set North</button>
      <button onclick="setLevel()">Set Level (0°)</button>
      <button class="primary" onclick="api('/api/track/start')">Start Tracking</button>
      <button class="danger" onclick="api('/api/track/stop')">Stop Tracking</button>
    </div>

  </div>

  <div class="observerInline">
    <div class="observerRow">
      <span class="obsLbl">Lat</span><input class="text obs" id="obsLat" placeholder="35.3733">
      <span class="obsLbl">Lon</span><input class="text obs" id="obsLon" placeholder="-119.0187">
      <span class="obsLbl">Alt</span><input class="text obs" id="obsAlt" placeholder="120">
      <button class="small" onclick="saveObserver()">Save</button>
    </div>
    <div class="hint">Saving observer location stops tracking and switches to manual mode.</div>
  </div>
</div>




<div class="card">
  <h3>Status</h3>
  <div class="statusGrid">
    <div id="statusL">Loading...</div>
    <div id="statusR"></div>
  </div>
</div>




<script>
async function api(url){ try { await fetch(url); } catch(e) {} }
let curServoAngle = 90;

async function nudgeServo(delta){
  let v = parseInt(curServoAngle || "0", 10) + delta;
  if (v < 0) v = 0;
  if (v > 180) v = 180;
  curServoAngle = v;
  await fetch('/api/servo?angle=' + encodeURIComponent(v));
}

let obsFilled = false;
async function saveObserver(){
  const lat = document.getElementById('obsLat').value;
  const lon = document.getElementById('obsLon').value;
  const alt = document.getElementById('obsAlt').value;
  await fetch(`/api/observer/set?lat=${encodeURIComponent(lat)}&lon=${encodeURIComponent(lon)}&alt=${encodeURIComponent(alt)}`);
  // Refresh status so the UI reflects new values immediately
  await poll();
}

async function setLevel(){
  // Save current servo position as 0° elevation reference
  await fetch('/api/servo/zero');
  await poll();
}

function renderStatus(j){

  const left = [];
  const right = [];

  left.push(`<b>WiFi:</b> ${j.wifiMode} ${j.ip} (RSSI ${j.rssi})`);
  left.push(`<b>Tracking:</b> ${j.tracking}`);
  left.push(`<b>Ready:</b> ${j.ready} ${(!j.ready ? '(Home + Set North required)' : '')}`);
  left.push(`<b>ManualOverride:</b> ${j.manualOverride}`);
  left.push(`<b>Homed:</b> ${j.homed} &nbsp; <b>HomeState:</b> ${j.homeState}`);
  left.push(`<b>Hall:</b> raw=${j.hallRaw} active=${j.hallActive}`);
  left.push(`<b>Az deg (north-ref):</b> ${Number(j.azDeg).toFixed(2)}&deg;`);
  left.push(`<b>Az steps:</b> ${j.azSteps}`);
  left.push(`<b>North offset steps:</b> ${j.northOffset}`);
  left.push(`<b>Servo:</b> ${j.servoAngle}&deg; (pulse ${j.servoPulse}us)`);
  left.push(`<b>Level ref:</b> ${j.servoZeroDeg}&deg;`);

  right.push(`<b>Observer:</b> ${Number(j.obsLat).toFixed(4)}, ${Number(j.obsLon).toFixed(4)} alt ${Number(j.obsAltM).toFixed(0)}m`);
  right.push(`<b>ISS:</b> ${Number(j.issLat).toFixed(3)}, ${Number(j.issLon).toFixed(3)} alt ${Number(j.issAltM).toFixed(0)}m`);
  right.push(`<b>Target Az/El:</b> ${Number(j.tAz).toFixed(2)}&deg; / ${Number(j.tEl).toFixed(2)}&deg;`);
  right.push(`<b>Poll:</b> ${j.pollMs}ms &nbsp; <b>FAST:</b> ${j.fastMode}`);
  right.push(`<b>ISS HTTP:</b> ${j.issHttp} &nbsp; <b>Age:</b> ${Math.round(j.issAgeMs/1000)}s`);
  right.push(`<b>Uptime:</b> ${Math.round(j.uptimeMs/1000)}s`);

  document.getElementById('statusL').innerHTML = left.join('<br>');
  document.getElementById('statusR').innerHTML = right.join('<br>');
  curServoAngle = j.servoAngle;


  if (!obsFilled) {
    const elLat = document.getElementById('obsLat');
    const elLon = document.getElementById('obsLon');
    const elAlt = document.getElementById('obsAlt');
    if (elLat && elLon && elAlt) {
      elLat.value = j.obsLat;
      elLon.value = j.obsLon;
      elAlt.value = j.obsAltM;
      obsFilled = true;
    }
  }
}

async function poll(){
  try{
    const r = await fetch('/api/status');
    const j = await r.json();
    renderStatus(j);
  }catch(e){
    document.getElementById('statusL').innerText = 'Status fetch failed';
  }
}
async function pollLoop(){
  await poll();
  setTimeout(pollLoop, 600);
}
pollLoop();
</script>
</body></html>
)HTML";

  return s;
}

// ---------------------- API ----------------------
void sendOk(const char* msg="ok") { server.send(200, "text/plain", msg); }
void handleRoot() { server.send(200, "text/html", htmlPage()); }

void handleStatus() {
  long steps = stepper.currentPosition();
  double azDeg = wrap360(((double)(steps - (long)northOffsetSteps)) / STEPS_PER_DEG);

  double lat, lon, altM, tAz, tEl;
  bool have;
  int httpc;
  unsigned long okMs;
  String err;
  bool fm;
  uint32_t pm;

  portENTER_CRITICAL(&dataMux);
  lat = issLatDeg; lon = issLonDeg; altM = issAltM;
  tAz = targetAzDeg; tEl = targetElDeg;
  have = haveTarget;
  httpc = lastIssHttpCode;
  okMs = lastIssOkMs;
  err = lastIssErr;
  fm = fastMode;
  pm = taskPollMs;
  portEXIT_CRITICAL(&dataMux);

  String json = "{";
  json += "\"wifiMode\":\"" + wifiModeString() + "\",";
  json += "\"ip\":\"" + ipString() + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"otaHost\":\"" + String(OTA_HOSTNAME) + "\",";
  json += "\"tracking\":" + String(trackingEnabled ? "true" : "false") + ",";
  json += "\"manualOverride\":" + String(manualOverride ? "true" : "false") + ",";
  json += "\"homed\":" + String(isHomed ? "true" : "false") + ",";
  json += "\"hasNorthOffset\":" + String(hasNorthOffset ? "true" : "false") + ",";
  json += "\"ready\":" + String((isHomed && hasNorthOffset) ? "true" : "false") + ",";
  json += "\"homeState\":" + String((int)homeState) + ",";
  json += "\"hallRaw\":" + String(hallRaw()) + ",";
  json += "\"hallActive\":" + String(hallActive() ? "true" : "false") + ",";
  json += "\"azSteps\":" + String(steps) + ",";
  json += "\"azDeg\":" + String(azDeg, 4) + ",";
  json += "\"obsLat\":" + String(obsLatDeg, 6) + ",";
  json += "\"obsLon\":" + String(obsLonDeg, 6) + ",";
  json += "\"obsAltM\":" + String(obsAltM, 1) + ",";
  json += "\"northOffset\":" + String((long)northOffsetSteps) + ",";
  json += "\"servoAngle\":" + String((int)servoAngle) + ",";
  json += "\"servoZeroDeg\":" + String((int)servoZeroDeg) + ",";
  json += "\"servoPulse\":" + String((int)servoPulseUs) + ",";
  json += "\"issLat\":" + String(lat, 6) + ",";
  json += "\"issLon\":" + String(lon, 6) + ",";
  json += "\"issAltM\":" + String(altM, 1) + ",";
  json += "\"tAz\":" + String(have ? tAz : 0.0, 4) + ",";
  json += "\"tEl\":" + String(have ? tEl : 0.0, 4) + ",";
  json += "\"fastMode\":" + String(fm ? "true" : "false") + ",";
  json += "\"pollMs\":" + String(pm) + ",";
  json += "\"issHttp\":" + String(httpc) + ",";
  json += "\"issErr\":\"" + err + "\",";
  json += "\"issAgeMs\":" + String(okMs == 0 ? 999999999UL : (millis() - okMs)) + ",";
  json += "\"uptimeMs\":" + String(millis());
  json += "}";
  
  server.send(200, "application/json", json);
}


void handleObserverGet() {
  String json = "{";
  json += "\"lat\":" + String(obsLatDeg, 6) + ",";
  json += "\"lon\":" + String(obsLonDeg, 6) + ",";
  json += "\"altM\":" + String(obsAltM, 1);
  json += "}";
  server.send(200, "application/json", json);
}

static bool parseDoubleArg(const String& name, double& out) {
  if (!server.hasArg(name)) return false;
  out = server.arg(name).toDouble();
  return true;
}

void handleObserverSet() {
  trackingEnabled = false;
  manualOverride = true;

  double lat, lon, alt;
  if (!parseDoubleArg("lat", lat) || !parseDoubleArg("lon", lon) || !parseDoubleArg("alt", alt)) {
    server.send(400, "text/plain", "missing lat/lon/alt");
    return;
  }

  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 || alt < -500.0 || alt > 10000.0) {
    server.send(400, "text/plain", "invalid range");
    return;
  }

  obsLatDeg = lat;
  obsLonDeg = lon;
  obsAltM   = alt;

  prefs.putDouble("obsLat", obsLatDeg);
  prefs.putDouble("obsLon", obsLonDeg);
  prefs.putDouble("obsAlt", obsAltM);

  sendOk("observer saved");
}


void handleHome() {
  trackingEnabled = false;
  manualOverride = true;

  // Manual Home should NOT auto-start tracking afterwards.
  rehomeRequested = false;
  resumeTrackingAfterRehome = false;
  autoStartAfterHoming = false;

  startHoming();
  sendOk("homing started");
}

void handleSetNorth() {
  trackingEnabled = false;
  manualOverride = true;
  northOffsetSteps = stepper.currentPosition();
  prefs.putLong("northOffset", (long)northOffsetSteps);
  hasNorthOffset = true;
  prefs.putBool("hasNorthOffset", true);
  sendOk("north offset saved");
}

void handleServo() {
  trackingEnabled = false;
  manualOverride = true;
  if (!server.hasArg("angle")) { server.send(400, "text/plain", "missing angle"); return; }
  int a = server.arg("angle").toInt();
  setServoAngleClamped(a);
  sendOk();
}
void handleServoZero() {
  trackingEnabled = false;
  manualOverride = true;
  servoZeroDeg = servoAngle;
  prefs.putInt("servoZeroDeg", servoZeroDeg);
  sendOk("level saved");
}


void handleJog() {
  trackingEnabled = false;
  manualOverride = true;

  if (!server.hasArg("dir")) { server.send(400, "text/plain", "missing dir"); return; }
  if (homeState != ST_IDLE && homeState != ST_DONE && homeState != ST_FAIL) {
    server.send(409, "text/plain", "busy homing");
    return;
  }

  int dir = server.arg("dir").toInt();
  if (dir == 0) dir = 1;

  stepper.setMaxSpeed(MOVE_SPEED);
  stepper.setAcceleration(MOVE_ACCEL);

  long target = stepper.currentPosition() + (dir > 0 ? JOG_STEP_DELTA : -JOG_STEP_DELTA);
  stepper.moveTo(target);

  sendOk("jogging");
}

void handleStop() {
  trackingEnabled = false;
  manualOverride = true;
  stepper.stop();
  sendOk("stopping");
}

void handleTrackStart() {
  if (homeState != ST_IDLE && homeState != ST_DONE && homeState != ST_FAIL) {
    server.send(409, "text/plain", "busy homing");
    return;
  }

  if (!isHomed) {
    server.send(409, "text/plain", "not homed - press Home first");
    return;
  }

  if (!hasNorthOffset) {
    server.send(409, "text/plain", "north not set - jog to North and press Set North");
    return;
  }

  manualOverride = false;
  trackingEnabled = true;

  portENTER_CRITICAL(&dataMux);
  havePendingTarget = false;
  haveTarget = false;
  fastMode = false;
  taskPollMs = ISS_POLL_SLOW_MS;
  portEXIT_CRITICAL(&dataMux);

  sendOk("tracking enabled");
}

void handleTrackStop() {
  trackingEnabled = false;
  manualOverride = true;
  stepper.stop();
  sendOk("tracking disabled");
}

// ---------------------- SETUP / LOOP ----------------------

// Forward declarations
void handleObserverGet();
void handleObserverSet();

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(HALL_PIN, INPUT_PULLUP);
  lastHallRaw = hallRaw();

  prefs.begin("isst", false);
  northOffsetSteps = prefs.getLong("northOffset", 0);
  hasNorthOffset = prefs.getBool("hasNorthOffset", false);
  isHomed = false; // always re-home on boot (latching hall has no absolute at power-up)
  prefs.putBool("isHomed", false);
  servoAngle = prefs.getInt("servoAngle", 90);
  servoZeroDeg = prefs.getInt("servoZeroDeg", 90); // default level at 90°


  // Load observer location (persisted if changed via UI)
  obsLatDeg = prefs.getDouble("obsLat", OBS_LAT_DEFAULT_DEG);
  obsLonDeg = prefs.getDouble("obsLon", OBS_LON_DEFAULT_DEG);
  obsAltM   = prefs.getDouble("obsAlt", OBS_ALT_DEFAULT_M);

  stepper.setMaxSpeed(MOVE_SPEED);
  stepper.setAcceleration(MOVE_ACCEL);
  stepper.setCurrentPosition(0);

  elevServo.setPeriodHertz(50);
  elevServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  elevServo.write((int)servoAngle);

  startWiFi();
  setupOTA();
  setupOLED();

  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/observer", handleObserverGet);
  server.on("/api/observer/set", handleObserverSet);
  server.on("/api/home", handleHome);
  server.on("/api/set_north", handleSetNorth);
  server.on("/api/servo", handleServo);
  server.on("/api/servo/zero", handleServoZero);
  server.on("/api/jog", handleJog);
  server.on("/api/stop", handleStop);
  server.on("/api/track/start", handleTrackStart);
  server.on("/api/track/stop", handleTrackStop);

  server.begin();

  // Always home on boot. Optionally auto-start tracking after homing if North was previously set.
  trackingEnabled = false;
  manualOverride = true;

  rehomeRequested = false;
  resumeTrackingAfterRehome = false;
  autoStartAfterHoming = hasNorthOffset;

  startHoming();


  // Start ISS background task (Core 0 tends to keep WiFi happy; Core 1 runs loop/UI)
  xTaskCreatePinnedToCore(
    issTask,
    "issTask",
    8192,
    nullptr,
    1,
    &issTaskHandle,
    0
  );
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  updateOLED();

  // If we detected drift while tracking, stop and re-home.
  if (rehomeRequested && (homeState == ST_IDLE || homeState == ST_DONE || homeState == ST_FAIL)) {
    trackingEnabled = false;
    manualOverride = true;
    autoStartAfterHoming = false; // resume logic is handled by resumeTrackingAfterRehome
    startHoming();
  }

  // Homing mode
  if (homeState != ST_IDLE && homeState != ST_DONE && homeState != ST_FAIL) {
    updateHoming();
    updateOLED();
    return;
  }

  // Tracking motion consumer (non-blocking)
  if (trackingEnabled && !manualOverride) {
    updateTrackingMotion();
  }

  // Always allow stepper to progress (jogs, etc.)
  stepper.run();
}
