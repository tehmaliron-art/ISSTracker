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
static const uint8_t OLED_ADDR_PRIMARY  = 0x3C;   // 7-bit addr; silkscreen may show 0x78 (8-bit write addr)
static const uint8_t OLED_ADDR_FALLBACK = 0x3D;   // alternate 7-bit addr
static const int OLED_RESET_PIN = -1;    // not used with I2C modules

Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, OLED_RESET_PIN);
bool oledOk = false;
unsigned long lastOledMs = 0;

void setupOLED();
void updateOLED();
int i2cScanAndReport(bool &found3C, bool &found3D);
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

// N2YO "visual passes" API (ISS NORAD id 25544). Used for next overhead pass estimates.
static const char* N2YO_BASE = "https://api.n2yo.com/rest/v1/satellite";
static const char* N2YO_API_KEY = "B2A452-X29HFK-647XCY-5NME";

// ---------------------- SATELLITE SELECTION (Stage 1: UI + status only) ----------------------
// Default to ISS. Stage 2 will use this for live pointing.
static uint32_t trackedNoradId = 25544;


// Tracking motion mode
static bool smoothMotion = false; // if true, allow continuous retargeting (Stage 2)
static const char* satNameFor(uint32_t noradId) {
  switch (noradId) {
    case 25544: return "ISS";
    case 20580: return "Hubble";
    case 48274: return "Tiangong";
    case 25338: return "NOAA 15";
    case 28654: return "NOAA 18";
    case 33591: return "NOAA 19";
    case 25994: return "Terra";
    case 27424: return "Aqua";
    case 39084: return "Landsat 8";
    case 40697: return "Sentinel-2A";
    default:    return "Custom";
  }
}


// ---------------------- NETWORK TIMEOUTS ----------------------
const uint32_t ISS_HTTP_TIMEOUT_MS = 500;  // your preference
const uint32_t N2YO_HTTP_TIMEOUT_MS = 2500; // only fetched on boot / after pass

// ---------------------- UNIX TIME (from ISS timestamp + millis) ----------------------
static bool timeSynced = false;
static uint32_t timeBaseEpoch = 0;
static uint32_t timeBaseMillis = 0;

uint32_t nowEpoch() {
  if (!timeSynced) return 0;
  return timeBaseEpoch + (uint32_t)((millis() - timeBaseMillis) / 1000);
}

// ---------------------- NEXT PASS (N2YO) ----------------------
static bool haveNextPass = false;
static uint32_t nextPassMaxUTC = 0;
static double nextPassMaxEl = 0.0;
static int nextPassMaxAz = -1; // degrees, 0-359; -1 unknown
static unsigned long lastN2yoFetchMs = 0;


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

// Convert azimuth degrees to a short compass direction (8-way)
const char* azToCompass8(int azDegInt) {
  if (azDegInt < 0) return "";
  azDegInt %= 360;
  if (azDegInt < 0) azDegInt += 360;
  static const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
  int idx = (azDegInt + 22) / 45; // center bins
  if (idx < 0) idx = 0;
  if (idx > 7) idx = 7;
  return dirs[idx];
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

  // Use ISS API timestamp as a lightweight Unix time source.
  double ts = 0;
  if (extractJsonNumber(body, "timestamp", ts) && ts > 0) {
    timeBaseEpoch = (uint32_t)ts;
    timeBaseMillis = millis();
    timeSynced = true;
  }

  altM_out = aKm * 1000.0;
  return true;
}


bool fetchNextPassN2YO(uint32_t &maxUTCOut, double &maxElOut, int &maxAzOut, int &httpCode, String &err) {
  err = "";
  httpCode = 0;

  if (WiFi.getMode() != WIFI_STA || WiFi.status() != WL_CONNECTED) {
    err = "WiFi not connected";
    return false;
  }

  // Build URL:
  // https://api.n2yo.com/rest/v1/satellite/visualpasses/{id}/{lat}/{lon}/{alt_m}/{days}/{min_visibility}/&apiKey={key}
  String url = String(N2YO_BASE) + "/visualpasses/" + String(trackedNoradId) + "/"+
               String(obsLatDeg, 6) + "/" + String(obsLonDeg, 6) + "/" +
               String((int)lround(obsAltM)) + "/2/0/&apiKey=" + N2YO_API_KEY;

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(N2YO_HTTP_TIMEOUT_MS);

  HTTPClient http;
  http.setTimeout(N2YO_HTTP_TIMEOUT_MS);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, url)) {
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

  double maxUTC = 0;
  double maxEl = 0;
  double maxAz = 0;
  if (!extractJsonNumber(body, "maxUTC", maxUTC) || maxUTC <= 0) { err = "parse maxUTC"; return false; }
  if (!extractJsonNumber(body, "maxEl",  maxEl))                 { err = "parse maxEl";  return false; }

  // maxAz is optional; if absent, we just won't show a direction hint.
  int azInt = -1;
  if (extractJsonNumber(body, "maxAz", maxAz)) {
    azInt = (int)lround(maxAz);
    azInt %= 360;
    if (azInt < 0) azInt += 360;
  }

  maxUTCOut = (uint32_t)maxUTC;
  maxElOut = maxEl;
  maxAzOut = azInt;
  return true;
}


// ---------------------- N2YO POSITIONS (Stage 2: live az/el) ----------------------
struct PosSample {
  uint32_t ts;
  float az;
  float el;
  float satLat;
  float satLon;
  float satAltM;
};

static const uint32_t TRACK_POLL_MS = 5000;
static const uint8_t  TRACK_SECONDS = 6;

static PosSample posBuf[TRACK_SECONDS];
volatile uint8_t posCount = 0;
volatile uint8_t posEmitIdx = 0;
volatile uint32_t posBaseMs = 0;
volatile uint32_t lastPosFetchMs = 0;

static bool parseJsonNumberAt(const String &body, const char *key, int start, double &out, int &nextPos) {
  String needle = String("\"") + key + "\":";
  int k = body.indexOf(needle, start);
  if (k < 0) return false;
  int vStart = k + needle.length();
  const char *p = body.c_str() + vStart;
  char *endp = nullptr;
  out = strtod(p, &endp);
  if (endp == p) return false;
  nextPos = (int)(endp - body.c_str());
  return true;
}

struct PosSample; // forward decl for pointer params

static bool fetchPositionsN2YO(uint32_t noradId, uint8_t seconds,
                              PosSample *out, uint8_t &outCount,
                              int &httpCode, String &err) {
  err = "";
  httpCode = 0;
  outCount = 0;

  if (WiFi.getMode() != WIFI_STA || WiFi.status() != WL_CONNECTED) {
    err = "WiFi not connected";
    return false;
  }
  if (seconds < 1) seconds = 1;
  if (seconds > 60) seconds = 60;

  // https://api.n2yo.com/rest/v1/satellite/positions/{id}/{lat}/{lon}/{alt_m}/{seconds}/&apiKey={key}
  String url = String(N2YO_BASE) + "/positions/" + String(noradId) + "/" +
               String(obsLatDeg, 6) + "/" + String(obsLonDeg, 6) + "/" +
               String((int)lround(obsAltM)) + "/" + String((int)seconds) +
               "/&apiKey=" + N2YO_API_KEY;

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(N2YO_HTTP_TIMEOUT_MS);

  HTTPClient http;
  http.setTimeout(N2YO_HTTP_TIMEOUT_MS);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, url)) {
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

  int arr = body.indexOf("\"positions\"");
  if (arr < 0) { err = "parse positions[]"; return false; }
  int p = body.indexOf('[', arr);
  if (p < 0) { err = "parse positions["; return false; }

  uint8_t count = 0;
  int cur = p + 1;
  while (count < seconds) {
    int obj = body.indexOf('{', cur);
    if (obj < 0) break;
    int objEnd = body.indexOf('}', obj);
    if (objEnd < 0) break;

    double ts=0, az=0, el=0, slat=0, slon=0, saltKm=0;
    int np=0;

    if (!parseJsonNumberAt(body, "timestamp", obj, ts, np)) { cur = objEnd + 1; continue; }
    if (!parseJsonNumberAt(body, "azimuth",   obj, az, np)) { cur = objEnd + 1; continue; }
    if (!parseJsonNumberAt(body, "elevation", obj, el, np)) { cur = objEnd + 1; continue; }

    // optional sat position values (nice for status display)
    parseJsonNumberAt(body, "satlatitude",  obj, slat, np);
    parseJsonNumberAt(body, "satlongitude", obj, slon, np);
    parseJsonNumberAt(body, "sataltitude",  obj, saltKm, np); // km

    out[count].ts = (uint32_t)ts;
    out[count].az = (float)az;
    out[count].el = (float)el;
    out[count].satLat = (float)slat;
    out[count].satLon = (float)slon;
    out[count].satAltM = (float)(saltKm * 1000.0);

    count++;
    cur = objEnd + 1;
  }

  outCount = count;

  // Use N2YO timestamp as a lightweight Unix time source
  if (outCount > 0 && out[0].ts > 0) {
    timeBaseEpoch = out[0].ts;
    timeBaseMillis = millis();
    timeSynced = true;
  }

  if (outCount == 0) {
    err = "no position samples";
    return false;
  }
  return true;
}

void ensureNextPass(bool forceFetch) {
  // Avoid hammering the API if WiFi just came up or time is not synced yet.
  if (WiFi.getMode() != WIFI_STA || WiFi.status() != WL_CONNECTED) return;

  if (!forceFetch && haveNextPass && timeSynced) {
    int32_t dt = (int32_t)nextPassMaxUTC - (int32_t)nowEpoch();
    if (dt > 0) return; // still in the future
  }

  // Throttle retries in case of temporary failures.
  if (millis() - lastN2yoFetchMs < 60000) return;

  uint32_t maxUTC = 0;
  double maxEl = 0;
  int maxAz = -1;
  int code = 0;
  String err;

  bool ok = fetchNextPassN2YO(maxUTC, maxEl, maxAz, code, err);
  lastN2yoFetchMs = millis();

  if (ok) {
    haveNextPass = true;
    nextPassMaxUTC = maxUTC;
    nextPassMaxEl = maxEl;
    nextPassMaxAz = maxAz;
    Serial.printf("[PASS] Next overhead maxUTC=%lu (in %ld s), maxEl=%.1f\n",
                  (unsigned long)nextPassMaxUTC,
                  timeSynced ? (long)((int32_t)nextPassMaxUTC - (int32_t)nowEpoch()) : -1L,
                  nextPassMaxEl);
  } else {
    Serial.printf("[PASS] N2YO fetch failed (HTTP %d): %s\n", code, err.c_str());
  }
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

    
    int code = 0;
    String err;

    // Fetch a batch of az/el samples every TRACK_POLL_MS
    uint32_t nowMs = millis();
    bool needFetch = false;

    portENTER_CRITICAL(&dataMux);
    uint32_t lastFetch = lastPosFetchMs;
    uint8_t  cnt = posCount;
    portEXIT_CRITICAL(&dataMux);

    if (cnt == 0) needFetch = true;
    if (nowMs - lastFetch >= TRACK_POLL_MS) needFetch = true;

    if (needFetch) {
      PosSample tmp[TRACK_SECONDS];
      uint8_t outCount = 0;
      bool ok = fetchPositionsN2YO(trackedNoradId, TRACK_SECONDS, tmp, outCount, code, err);

      portENTER_CRITICAL(&dataMux);
      lastIssHttpCode = code;
      lastIssErr = err;

      if (ok) {
        for (uint8_t i = 0; i < outCount && i < TRACK_SECONDS; i++) posBuf[i] = tmp[i];
        posCount = outCount;
        posEmitIdx = 0;
        posBaseMs = millis();
        lastPosFetchMs = posBaseMs;

        // Update status fields when available
        issLatDeg = tmp[0].satLat;
        issLonDeg = tmp[0].satLon;
        issAltM   = tmp[0].satAltM;

        lastIssOkMs = millis();
      }
      taskPollMs = TRACK_POLL_MS;
      fastMode = false;
      portEXIT_CRITICAL(&dataMux);
    }

    // Emit one target per second from the current buffer (indexes 0..4). Index 5 is lookahead.
    static uint32_t nextEmitMs = 0;
    if (nextEmitMs == 0) nextEmitMs = millis();

    if (millis() >= nextEmitMs) {
      uint8_t emitIdx;
      uint8_t haveCnt;
      PosSample sample;

      portENTER_CRITICAL(&dataMux);
      emitIdx = posEmitIdx;
      haveCnt = posCount;
      if (haveCnt > emitIdx && emitIdx < TRACK_SECONDS) sample = posBuf[emitIdx];
      portEXIT_CRITICAL(&dataMux);

      if (haveCnt >= 5 && emitIdx < 5) {
        double az = sample.az;
        double el = sample.el;

        portENTER_CRITICAL(&dataMux);
        targetAzDeg = az;
        targetElDeg = el;
        haveTarget = true;

        pendingAzDeg = az;
        pendingElDeg = el;
        havePendingTarget = true;

        posEmitIdx = emitIdx + 1;
        portEXIT_CRITICAL(&dataMux);

        nextEmitMs += 1000;
      } else {
        nextEmitMs = millis() + 200;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));

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
    if (smoothMotion || labs(stepper.distanceToGo()) <= RETARGET_THRESHOLD_STEPS || stepper.distanceToGo() == 0) {
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

int i2cScanAndReport(bool &found3C, bool &found3D) {
  found3C = false;
  found3D = false;

  Serial.println();
  Serial.println("[I2C] Scanning bus...");

  int found = 0;
  for (uint8_t addr = 1; addr < 0x7F; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.printf("[I2C] Device found at 0x%02X\n", addr);
      found++;
      if (addr == 0x3C) found3C = true;
      if (addr == 0x3D) found3D = true;
    } else if (err == 4) {
      Serial.printf("[I2C] Unknown error at 0x%02X\n", addr);
    }
  }

  if (found == 0) {
    Serial.println("[I2C] No devices found. Check wiring (SDA=21, SCL=22), power, and pull-ups.");
  } else {
    Serial.printf("[I2C] Scan complete: %d device(s)\n", found);
  }

  return found;
}

void setupOLED() {
  // Default ESP32 I2C pins: SDA=21, SCL=22
  Wire.begin(21, 22);

  bool found3C = false, found3D = false;
  (void)i2cScanAndReport(found3C, found3D);

  // Try the common 7-bit SSD1306 addresses. Many modules silkscreen the 8-bit write address (0x78),
  // which corresponds to 7-bit 0x3C.
  const uint8_t addrsToTry[] = { OLED_ADDR_PRIMARY, OLED_ADDR_FALLBACK };
  oledOk = false;

  for (uint8_t i = 0; i < (sizeof(addrsToTry) / sizeof(addrsToTry[0])); i++) {
    uint8_t addr = addrsToTry[i];
    Serial.printf("[OLED] Trying SSD1306 init at 0x%02X...\n", addr);
    if (oled.begin(SSD1306_SWITCHCAPVCC, addr)) {
      oledOk = true;
      Serial.println("[OLED] SSD1306 init OK.");
      break;
    }
    Serial.println("[OLED] SSD1306 init FAILED.");
  }

  if (!oledOk) {
    Serial.println("[OLED] OLED init failed.");
    if (found3C || found3D) {
      Serial.println("[OLED] I2C device responded at an OLED address, but SSD1306 init failed.");
      Serial.println("[OLED] Device seen at 0x3C/0x3D but SSD1306 init failed; may be SH1106 (try Adafruit_SH110X).");
    } else {
      Serial.println("[OLED] No I2C device responded at 0x3C/0x3D. Re-check wiring, power, and I2C pull-ups.");
    }
    return;
  }

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
  // Line 6: next overhead (from N2YO) countdown to maxUTC
  if (haveNextPass && timeSynced) {
    int32_t dt = (int32_t)nextPassMaxUTC - (int32_t)nowEpoch();
    if (dt < 0) dt = 0;
    char buf[16];
    if (dt >= 3600) {
      int h = dt / 3600;
      int m = (dt % 3600) / 60;
      snprintf(buf, sizeof(buf), "Next %dh%02dm", h, m);
    } else {
      int m = dt / 60;
      int s = dt % 60;
      snprintf(buf, sizeof(buf), "Next %dm%02ds", m, s);
    }
    oled.print(buf);
    oled.print(" P");
    oled.print((int)lround(nextPassMaxEl));
    if (nextPassMaxAz >= 0) {
      oled.print(" A");
      oled.print(nextPassMaxAz);
      oled.print(azToCompass8(nextPassMaxAz));
    }
  } else {
    oled.print("Next TBD");
  }

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
  <h3>Satellite</h3>
  <div class="row">
    <label class="lbl">Preset</label>
    <select id="satPreset" class="text" style="min-width:240px" onchange="satPresetChanged()">
      <option value="25544">ISS (SPACE STATION)</option>
      <option value="20580">Hubble (HST)</option>
      <option value="48274">Tiangong (CSS Tianhe-1)</option>
      <option value="25338">NOAA 15</option>
      <option value="28654">NOAA 18</option>
      <option value="33591">NOAA 19</option>
      <option value="25994">Terra</option>
      <option value="27424">Aqua</option>
      <option value="39084">Landsat 8</option>
      <option value="40697">Sentinel-2A</option>
      <option value="custom">Custom…</option>
    </select>

    <label class="lbl">NORAD</label>
    <input id="satNorad" class="text" style="max-width:140px" placeholder="25544">
    <button class="small" onclick="setSat()">Set</button>
    <label class="lbl" style="margin-left:14px">Smooth</label>
    <input type="checkbox" id="smoothMotion" onclick="setSmooth()">
  </div>
  <div class="hint">Stage 1: store the selected NORAD ID; Stage 2 will use it for tracking.</div>
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

async function loadSat(){
  try{
    const r = await fetch('/api/sat');
    const j = await r.json();
    const id = String(j.id || '');
    const preset = document.getElementById('satPreset');
    const norad = document.getElementById('satNorad');
    if (norad) norad.value = id;
    if (preset){
      let found = false;
      for (const opt of preset.options){
        if (opt.value === id){ preset.value = id; found = true; break; }
      }
      if (!found) preset.value = 'custom';
    }
  }catch(e){}

async function loadSmooth(){
  try{
    const r = await fetch('/api/smooth');
    const j = await r.json();
    const cb = document.getElementById('smoothMotion');
    if (cb) cb.checked = !!j.smooth;
  }catch(e){}
}

async function setSmooth(){
  const cb = document.getElementById('smoothMotion');
  const on = (cb && cb.checked) ? 1 : 0;
  await fetch('/api/smooth/set?on=' + on);
  await poll();
}

}

function satPresetChanged(){
  const preset = document.getElementById('satPreset');
  const norad = document.getElementById('satNorad');
  if (!preset || !norad) return;
  if (preset.value && preset.value !== 'custom') norad.value = preset.value;
}

async function setSat(){
  const norad = document.getElementById('satNorad');
  if (!norad) return;
  const id = (norad.value || '').trim();
  if (!/^\d+$/.test(id)) { alert('Enter a numeric NORAD ID'); return; }
  await fetch('/api/sat/set?id=' + encodeURIComponent(id));
  await loadSat();
loadSmooth();
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

  function fmtDt(sec){
    if (sec === undefined || sec === null || sec < 0) return 'TBD';
    sec = Math.floor(sec);
    const h = Math.floor(sec/3600);
    const m = Math.floor((sec%3600)/60);
    const s = sec%60;
    if (h > 0) return `${h}h ${m}m`;
    return `${m}m ${s}s`;
  }

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
  right.push(`<b>Satellite:</b> ${j.selSatName} (NORAD ${j.selNoradId})`);
  right.push(`<b>ISS:</b> ${Number(j.issLat).toFixed(3)}, ${Number(j.issLon).toFixed(3)} alt ${Number(j.issAltM).toFixed(0)}m`);
  right.push(`<b>Target Az/El:</b> ${Number(j.tAz).toFixed(2)}&deg; / ${Number(j.tEl).toFixed(2)}&deg;`);

  if (j.haveNextPass && j.timeSynced) {
    let line = `<b>Next pass (max):</b> in ${fmtDt(j.nextPassDt)} (P${Math.round(j.nextPassMaxEl)}&deg;`;
    if (j.nextPassMaxAz >= 0) {
      const comp = (j.nextPassAzComp || '').trim();
      line += `, Az ${Math.round(j.nextPassMaxAz)}&deg; ${comp}`;
    }
    line += `)`;
    right.push(line);
  } else {
    right.push(`<b>Next pass (max):</b> TBD`);
  }

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
loadSat();
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

  bool ts;
  uint32_t nowE;
  bool hnp;
  uint32_t pMaxUTC;
  double pMaxEl;
  int pMaxAz;

  portENTER_CRITICAL(&dataMux);
  uint32_t selId;

  lat = issLatDeg; lon = issLonDeg; altM = issAltM;
  tAz = targetAzDeg; tEl = targetElDeg;
  have = haveTarget;
  httpc = lastIssHttpCode;
  okMs = lastIssOkMs;
  err = lastIssErr;
  fm = fastMode;
  pm = taskPollMs;
  ts = timeSynced;
  nowE = ts ? nowEpoch() : 0;
  hnp = haveNextPass;
  pMaxUTC = nextPassMaxUTC;
  pMaxEl = nextPassMaxEl;
  pMaxAz = nextPassMaxAz;
  selId = trackedNoradId;
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
  json += "\"selNoradId\":" + String(selId) + ",";
  json += "\"selSatName\":\"" + String(satNameFor(selId)) + "\",";
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
  json += "\"timeSynced\":" + String(ts ? "true" : "false") + ",";
  json += "\"nowEpoch\":" + String(nowE) + ",";
  json += "\"haveNextPass\":" + String(hnp ? "true" : "false") + ",";
  json += "\"nextPassMaxUTC\":" + String(pMaxUTC) + ",";
  json += "\"nextPassMaxEl\":" + String(pMaxEl, 1) + ",";
  json += "\"nextPassMaxAz\":" + String(pMaxAz) + ",";
  int32_t pdt = (hnp && ts) ? ((int32_t)pMaxUTC - (int32_t)nowE) : -1;
  if (pdt < 0) pdt = -1;
  json += "\"nextPassDt\":" + String(pdt) + ",";
  json += "\"nextPassAzComp\":\"" + String(azToCompass8(pMaxAz)) + "\","; 
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


// Satellite selection (Stage 1)
void handleSatGet() {
  String json = "{";
  json += "\"id\":" + String(trackedNoradId) + ",";
  json += "\"name\":\"" + jsonEscape(String(satNameFor(trackedNoradId))) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSatSet() {
  if (!server.hasArg("id")) {
    server.send(400, "text/plain", "missing id");
    return;
  }
  long id = server.arg("id").toInt();
  if (id < 1 || id > 99999) {
    server.send(400, "text/plain", "invalid id");
    return;
  }
  trackedNoradId = (int)id;
  prefs.putInt("trackedNoradId", trackedNoradId);
  sendOk("satellite set");
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
void handleSatGet();
void handleSatSet();


// -------------------------
// Smooth motion API handlers (Stage 2)
// -------------------------
void handleSmoothGet() {
  String json = String("{\"on\":") + (smoothMotion ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void handleSmoothSet() {
  // expects on=0|1
  bool on = false;
  if (server.hasArg("on")) {
    String v = server.arg("on");
    v.toLowerCase();
    on = (v == "1" || v == "true" || v == "on" || v == "yes");
  }
  smoothMotion = on;
  prefs.putBool("smoothMotion", smoothMotion);
  handleSmoothGet();
}
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
  // Load selected satellite (Stage 1: UI + status only)
  trackedNoradId = prefs.getUInt("trackedNoradId", 25544);
  smoothMotion = prefs.getBool("smoothMotion", false);


  stepper.setMaxSpeed(MOVE_SPEED);
  stepper.setAcceleration(MOVE_ACCEL);
  stepper.setCurrentPosition(0);

  elevServo.setPeriodHertz(50);
  elevServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  elevServo.write((int)servoAngle);

  startWiFi();
  // On boot, grab a timestamp (for countdown) and the next overhead pass estimate.
  if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
    double lat=0, lon=0, altM=0;
    int code=0;
    String err;
    // Use a single N2YO positions call to seed time (timestamp) and status
    PosSample tmp[TRACK_SECONDS];
    uint8_t outCount=0;
    fetchPositionsN2YO(trackedNoradId, 1, tmp, outCount, code, err);
    ensureNextPass(true);
  }

  setupOTA();
  setupOLED();

  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/observer", handleObserverGet);
  server.on("/api/observer/set", handleObserverSet);
  server.on("/api/sat", handleSatGet);
  server.on("/api/sat/set", handleSatSet);
  server.on("/api/smooth", handleSmoothGet);
  server.on("/api/smooth/set", handleSmoothSet);
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

  // Refresh next-pass prediction only after the current one has happened.
  static unsigned long lastPassCheckMs = 0;
  if (millis() - lastPassCheckMs > 5000) {
    lastPassCheckMs = millis();
    if (!haveNextPass) {
      ensureNextPass(true);
    } else if (timeSynced && (int32_t)(nowEpoch() - nextPassMaxUTC) >= 0) {
      ensureNextPass(true);
    }
  }

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
