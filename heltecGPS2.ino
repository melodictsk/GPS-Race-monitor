#include <heltec.h>

#include "WiFi.h"
#include "HT_st7735.h"
HT_st7735 st7735;
extern SPIClass st7735_spi;
#include <MicroNMEA.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

char nmeaBuffer[400];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

String device_name = "WiFiBTGPS";

// ---- Service A: Nordic UART (NUS) -- for BLE terminals / raw NMEA passthrough ----
#define BLE_UART_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UART_RX_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // write from client -> ESP
#define BLE_UART_TX_CHAR_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // notify ESP -> client

// ---- Service B: RaceChrono DIY BLE GPS (service 0x1FF8) ----
// Only GPS characteristics (0x0003 main, 0x0004 time) are implemented.
// CAN-Bus 0x0001/0x0002 are NOT exposed — the user must configure the device
// in RaceChrono with only the "GPS" checkbox enabled.
#define BLE_RC_SERVICE_UUID    "00001ff8-0000-1000-8000-00805f9b34fb"
#define BLE_RC_GPS_MAIN_UUID   "00000003-0000-1000-8000-00805f9b34fb"
#define BLE_RC_GPS_TIME_UUID   "00000004-0000-1000-8000-00805f9b34fb"

BLEServer*         bleServer     = nullptr;
BLECharacteristic* bleTxChar     = nullptr;  // NUS TX
BLECharacteristic* bleRxChar     = nullptr;  // NUS RX
BLECharacteristic* bleRcMainChar = nullptr;  // RaceChrono 0x0003
BLECharacteristic* bleRcTimeChar = nullptr;  // RaceChrono 0x0004
bool               bleConnected  = false;
bool               bleWasConnected = false;

// Sync bits (3-bit counter, shared between 0x0003 and 0x0004, increments every
// time the time-packet value changes).
uint8_t  rcSyncBits = 0;
// Previous encoded time packet (24-bit, held in a uint32_t) — used to decide
// when to bump sync bits and re-notify 0x0004.
uint32_t rcPrevTimePacket = 0xFFFFFFFF;

// Buffer for data received from BLE client -> forwarded to Serial1
#define BLE_RX_BUF_SIZE 1024
uint8_t  bleRxBuf[BLE_RX_BUF_SIZE];
volatile size_t bleRxHead = 0;  // written by callback (from BLE task)
volatile size_t bleRxTail = 0;  // read by loop
#define VGNSS_CTRL 3
// How many clients should be able to telnet to this ESP32
#define MAX_SRV_CLIENTS 2
const char *ssid = "WiFiBTGPS";
const char *password = "87654321";
uint8_t i;
NetworkServer server(23);
NetworkClient serverClients[MAX_SRV_CLIENTS];

// Variables for tracking min/max speed on segments
float minSpeedSegment = 0.0;
float maxSpeedSegment = 0.0;
float lastSpeed = -1.0;
float trendStartTime = 0.0;
int currentTrend = 0;
float lastReportTime = 0.0;
float trendStartSpeed = 0;

#define BUTTON_PIN 0
#define FINISH_LINE_HALF_WIDTH 5.0  // 5 meters each side

// --- Lap trace recording ---
// At 10 Hz GPS, a 5-minute lap = 3000 points.
// Each point: lat(4) + lon(4) + timeCenti(2) + speed(1) = 11 bytes
// 3000 * 11 = 33000 bytes per buffer, x2 buffers = 66 KB — fits in ESP32 RAM.
#define MAX_LAP_POINTS 3000

struct __attribute__((packed)) LapPoint {
  float    lat;          // degrees (float precision ~1m is enough for matching)
  float    lon;          // degrees
  uint16_t timeCenti;    // centiseconds since lap start (0..65535 = 0..655.35 sec)
  uint8_t  speedKmh;     // km/h, 0..255
};
                   
// Two trace buffers:
//   bestLap    — the best completed lap trace
//   currentLap — trace being recorded now
LapPoint bestLap[MAX_LAP_POINTS];
int      bestLapCount = 0;

LapPoint currentLap[MAX_LAP_POINTS];
int      currentLapCount = 0;

// Start/finish line definition
bool   lapTimerActive = false;
double finishLat1, finishLon1;
double finishLat2, finishLon2;
double finishCenterLat, finishCenterLon;

// Lap timing state
float  lapStartTime = 0.0;
int    lapCount = 0;
float  lastCrossingTime = 0.0;
#define CROSSING_DEBOUNCE_SEC 5.0

// Button debounce
unsigned long lastButtonPress = 0;
#define BUTTON_DEBOUNCE_MS 500

// Previous GPS position and time for crossing detection & interpolation
double prevLat = 0.0;
double prevLon = 0.0;
float  prevGPSTime = 0.0;
bool   hasPrevPosition = false;

// Lap results
float bestLapTime = 0.0;
float lastLapTime = 0.0;

// Duplicate fix filter — global so it can be reset on lap start
uint32_t lastFixTime = 0xFFFFFFFF;

// True after the first valid GPS fix — guards display from drawing garbage
bool gpsEverValid = false;

// Violet/purple color (RGB565)
#ifndef ST7735_PURPLE
#define ST7735_PURPLE 0xA81F
#endif
// Gray color (RGB565) — dark gray, visible on black but clearly dim vs white
#ifndef ST7735_GRAY
#define ST7735_GRAY 0x39E7
#endif
// Cyan color (RGB565) — light blue for BLE indicator
#ifndef ST7735_CYAN
#define ST7735_CYAN 0x07FF
#endif
// Post-crossing summary: 5-second window after crossing the finish line
float postCrossingEndTime = 0.0;  // GPS time when the window ends (0 = window inactive)
float postCrossingOldBest = 0.0;  // best lap time BEFORE the current lap
float postCrossingLapTime = 0.0;  // time of the lap that just finished
bool  postCrossingWasBest = false; // did the current lap beat the old record?

// Live delta comparison state
float  deltaTime = 0.0;      // current time delta vs best lap (+ = slower)
int    deltaSpeed = 0;        // current speed delta vs best lap (+ = faster)
bool   deltaValid = false;    // do we have a valid comparison?
int    bestLapSearchIdx = 0;  // last matched index in bestLap (monotonic search)

// ==================== RTOS ====================
// Mutex protecting the MicroNMEA object and all variables derived from it
SemaphoreHandle_t nmeaMutex = nullptr;
// Mutex protecting the SPI bus / st7735 display — must be held for every draw call
SemaphoreHandle_t spiMutex  = nullptr;

// Ring buffer: GPS task -> BLE TX task (NMEA passthrough)
// Large enough for a few seconds of 115200 baud NMEA bursts
#define BLE_TX_BUF_SIZE 4096
static uint8_t  bleTxBuf[BLE_TX_BUF_SIZE];
static volatile size_t bleTxHead = 0;  // written by GPS task
static volatile size_t bleTxTail = 0;  // read by BLE task

void bleTxEnqueue(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    size_t next = (bleTxHead + 1) % BLE_TX_BUF_SIZE;
    if (next == bleTxTail) break;  // full — drop
    bleTxBuf[bleTxHead] = data[i];
    bleTxHead = next;
  }
}

// FreeRTOS queue: GPS task -> WiFi task (по одному байту, потокобезопасно)
#define WIFI_TX_BUF_SIZE 4096
static QueueHandle_t wifiTxQueue = nullptr;

void wifiTxEnqueue(const uint8_t* data, size_t len) {
  if (wifiTxQueue == nullptr) return;
  for (size_t i = 0; i < len; i++) {
    xQueueSend(wifiTxQueue, &data[i], 0);
  }
}

// Task handles
TaskHandle_t taskGPS         = nullptr;
TaskHandle_t taskWiFi        = nullptr;
TaskHandle_t taskBLE         = nullptr;
TaskHandle_t taskDisplay     = nullptr;


float getCurrentGPSTime() {
  float timeInSeconds = (nmea.getHour() * 3600.0) + 
                        (nmea.getMinute() * 60.0) + 
                        nmea.getSecond() + 
                        (nmea.getHundredths() / 100.0);
  
  static float lastGPSTime = 0;
  if (timeInSeconds < lastGPSTime && lastGPSTime > 0) {
    timeInSeconds += 86400.0;
  }
  lastGPSTime = timeInSeconds;
  return timeInSeconds;
}

double degToRad(double deg) {
  return deg * PI / 180.0;
}

double crossProduct2D(double ax, double ay, double bx, double by) {
  return ax * by - ay * bx;
}

double gpsDistance(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371000.0;
  double dLat = degToRad(lat2 - lat1);
  double dLon = degToRad(lon2 - lon1);
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(degToRad(lat1)) * cos(degToRad(lat2)) *
             sin(dLon / 2.0) * sin(dLon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

// Fast approximate distance in meters using flat-earth (good for < 1km)
float fastDistMeters(float lat1, float lon1, float lat2, float lon2) {
  float dLat = (lat2 - lat1) * 111320.0f;
  float dLon = (lon2 - lon1) * 111320.0f * cosf(lat1 * PI / 180.0f);
  return sqrtf(dLat * dLat + dLon * dLon);
}

// Format time as M:SS.xx
String formatLapTime(float seconds) {
  if (seconds < 0) seconds = -seconds;
  int mins = (int)(seconds / 60.0);
  float secs = seconds - (mins * 60.0);
  char buf[16];
  if (secs < 10.0)
    snprintf(buf, sizeof(buf), "%d:0%.2f", mins, (double)secs);
  else
    snprintf(buf, sizeof(buf), "%d:%.2f", mins, (double)secs);
  return String(buf);
}

// Format delta time as +/-S.xx (short for display)
String formatDeltaTime(float seconds) {
  char sign = (seconds >= 0) ? '+' : '-';
  float absVal = fabs(seconds);
  char buf[12];
  if (absVal < 10.0)
    snprintf(buf, sizeof(buf), "%c%.2f", sign, (double)absVal);
  else if (absVal < 100.0)
    snprintf(buf, sizeof(buf), "%c%.1f", sign, (double)absVal);
  else
    snprintf(buf, sizeof(buf), "%c%.0f", sign, (double)absVal);
  return String(buf);
}

// ==================== LAPTIMER FUNCTIONS ====================

void setFinishLine() {
  if (!nmea.isValid()) {
    return;
  }
  
  finishCenterLat = nmea.getLatitude() / 1.0e6;
  finishCenterLon = nmea.getLongitude() / 1.0e6;
  
  long courseMilli = nmea.getCourse();
  double courseDeg = courseMilli / 1000.0;
  double courseRad = degToRad(courseDeg);
  
  double perpRad = courseRad + PI / 2.0;
  double dLatPerMeter = 1.0 / 111320.0;
  double dLonPerMeter = 1.0 / (111320.0 * cos(degToRad(finishCenterLat)));
  
  double offsetNorth = FINISH_LINE_HALF_WIDTH * cos(perpRad);
  double offsetEast  = FINISH_LINE_HALF_WIDTH * sin(perpRad);
  
  finishLat1 = finishCenterLat + offsetNorth * dLatPerMeter;
  finishLon1 = finishCenterLon + offsetEast  * dLonPerMeter;
  finishLat2 = finishCenterLat - offsetNorth * dLatPerMeter;
  finishLon2 = finishCenterLon - offsetEast  * dLonPerMeter;
  
  lapTimerActive = true;
  lapCount = 0;
  lapStartTime = 0.0;
  bestLapTime = 0.0;
  lastLapTime = 0.0;
  lastCrossingTime = 0.0;
  hasPrevPosition = false;
  prevGPSTime = 0.0;
  currentLapCount = 0;
  bestLapCount = 0;
  deltaValid = false;
  bestLapSearchIdx = 0;
  postCrossingEndTime = 0.0;
}

// Record one point of the current lap trace.
void recordCurrentLapPoint() {
  if (!lapTimerActive || lapStartTime <= 0.0) return;
  if (currentLapCount >= MAX_LAP_POINTS) return;  // buffer full

  // Timestamp of the current NMEA fix (centiseconds of the day, unsigned 32-bit)
  uint32_t fixTime = (uint32_t)nmea.getHour()       * 360000UL
                   + (uint32_t)nmea.getMinute()     *   6000UL
                   + (uint32_t)nmea.getSecond()     *    100UL
                   + (uint32_t)nmea.getHundredths();

  // Duplicate filter: skip if the timestamp did not change
  if (fixTime == lastFixTime) return;
  lastFixTime = fixTime;

  float curTime = getCurrentGPSTime();
  float elapsed = curTime - lapStartTime;

  // Convert to centiseconds, clamp to uint16_t range (max 655.35 sec)
  uint32_t centi = (uint32_t)(elapsed * 100.0f);
  if (centi > 65535) centi = 65535;

  uint8_t spd = (uint8_t)constrain((int)(nmea.getSpeed() * 1.852 / 1000), 0, 255);

  LapPoint &p = currentLap[currentLapCount];
  p.lat       = (float)(nmea.getLatitude()  / 1.0e6);
  p.lon       = (float)(nmea.getLongitude() / 1.0e6);
  p.timeCenti = (uint16_t)centi;
  p.speedKmh  = (uint8_t)spd;

  currentLapCount++;
}

// Find the closest point on the best lap trace to the current position
// and compute time/speed deltas.
// Uses monotonic forward search from bestLapSearchIdx to avoid O(n) each call.
void updateDeltaComparison() {
  if (bestLapCount < 5 || lapStartTime <= 0.0 || currentLapCount < 2) {
    deltaValid = false;
    return;
  }
  
  float curLat = currentLap[currentLapCount - 1].lat;
  float curLon = currentLap[currentLapCount - 1].lon;
  float curElapsed = currentLap[currentLapCount - 1].timeCenti / 100.0f;
  int   curSpeed = currentLap[currentLapCount - 1].speedKmh;
  
  // Search around last matched index.
  // Lookback 50 pts handles being faster than best lap (we're geographically
  // ahead, so the matching bestLap index is behind the time-equivalent index).
  // Forward 150 pts handles being slower or GPS rate mismatches.
  // Total window 200 pts = 20 sec at 10 Hz — covers any realistic delta.
  int searchStart = bestLapSearchIdx - 50;
  if (searchStart < 0) searchStart = 0;
  int searchEnd = bestLapSearchIdx + 150;
  if (searchEnd > bestLapCount) searchEnd = bestLapCount;
  
  float bestDist = 1e9f;
  int bestIdx = bestLapSearchIdx;
  
  for (int i = searchStart; i < searchEnd; i++) {
    float d = fastDistMeters(curLat, curLon, bestLap[i].lat, bestLap[i].lon);
    if (d < bestDist) {
      bestDist = d;
      bestIdx = i;
    }
  }
  
  // If the closest point is too far (>50m), expand search to full trace
  if (bestDist > 50.0f) {
    for (int i = 0; i < bestLapCount; i++) {
      float d = fastDistMeters(curLat, curLon, bestLap[i].lat, bestLap[i].lon);
      if (d < bestDist) {
        bestDist = d;
        bestIdx = i;
      }
    }
  }
  
  if (bestDist > 100.0f) {
    deltaValid = false;
    return;
  }
  
  bestLapSearchIdx = bestIdx;
  
  // Interpolate between bestIdx and the next point for better precision
  float bestRefTime;
  float bestRefSpeed;
  
  if (bestIdx < bestLapCount - 1) {
    // Project current position onto segment bestIdx -> bestIdx+1
    float ax = bestLap[bestIdx].lon;
    float ay = bestLap[bestIdx].lat;
    float bx = bestLap[bestIdx + 1].lon;
    float by = bestLap[bestIdx + 1].lat;
    
    float abx = bx - ax;
    float aby = by - ay;
    float apx = curLon - ax;
    float apy = curLat - ay;
    
    float ab2 = abx * abx + aby * aby;
    float frac = 0.0f;
    if (ab2 > 1e-14f) {
      frac = (apx * abx + apy * aby) / ab2;
      if (frac < 0.0f) frac = 0.0f;
      if (frac > 1.0f) frac = 1.0f;
    }
    
    // Decode centiseconds to seconds for interpolation
    float t0 = bestLap[bestIdx].timeCenti / 100.0f;
    float t1 = bestLap[bestIdx + 1].timeCenti / 100.0f;
    float s0 = (float)bestLap[bestIdx].speedKmh;
    float s1 = (float)bestLap[bestIdx + 1].speedKmh;
    
    bestRefTime  = t0 + frac * (t1 - t0);
    bestRefSpeed = s0 + frac * (s1 - s0);
  } else {
    bestRefTime  = bestLap[bestIdx].timeCenti / 100.0f;
    bestRefSpeed = (float)bestLap[bestIdx].speedKmh;
  }
  
  // Delta: positive = current lap is SLOWER (behind), negative = FASTER (ahead)
  deltaTime = curElapsed - bestRefTime;
  
  // Speed delta: positive = current is FASTER, negative = current is SLOWER
  deltaSpeed = (int)(curSpeed - bestRefSpeed);
  
  deltaValid = true;
}

// Check finish line crossing with interpolated time
void checkLineCrossing() {
  if (!lapTimerActive || !nmea.isValid()) return;
  
  double curLat = nmea.getLatitude() / 1.0e6;
  double curLon = nmea.getLongitude() / 1.0e6;
  float  curTime = getCurrentGPSTime();
  
  if (!hasPrevPosition) {
    prevLat = curLat;
    prevLon = curLon;
    prevGPSTime = curTime;
    hasPrevPosition = true;
    return;
  }
  
  double moved = gpsDistance(prevLat, prevLon, curLat, curLon);
  if (moved < 0.5) {
    prevGPSTime = curTime;
    return;
  }
  
  double px = curLon - prevLon;
  double py = curLat - prevLat;
  double qx = finishLon2 - finishLon1;
  double qy = finishLat2 - finishLat1;
  
  double denom = crossProduct2D(px, py, qx, qy);
  
  if (fabs(denom) < 1e-15) {
    prevLat = curLat;
    prevLon = curLon;
    prevGPSTime = curTime;
    return;
  }
  
  double sx = finishLon1 - prevLon;
  double sy = finishLat1 - prevLat;
  
  double t = crossProduct2D(sx, sy, qx, qy) / denom;
  double u = crossProduct2D(sx, sy, px, py) / denom;
  
  if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
    float crossingTime = prevGPSTime + (float)t * (curTime - prevGPSTime);
    
    if (lastCrossingTime > 0 && (crossingTime - lastCrossingTime) < CROSSING_DEBOUNCE_SEC) {
      prevLat = curLat;
      prevLon = curLon;
      prevGPSTime = curTime;
      return;
    }
    
    if (lapStartTime > 0.0) {
      // ---- Completed a lap ----
      float lapTime = crossingTime - lapStartTime;
      lapCount++;
      lastLapTime = lapTime;
      
      // Capture the old best BEFORE it can be overwritten
      float oldBest = bestLapTime;
      
      bool isBest = false;
      if (bestLapTime <= 0.0 || lapTime < bestLapTime) {
        bestLapTime = lapTime;
        isBest = true;
        
        // Copy current trace to best lap buffer
        bestLapCount = currentLapCount;
        memcpy(bestLap, currentLap, currentLapCount * sizeof(LapPoint));
      }
      
      // Activate the 5-second post-finish window
      postCrossingOldBest = oldBest;
      postCrossingLapTime = lapTime;
      postCrossingWasBest = isBest;
      postCrossingEndTime = crossingTime + 5.0;
      
      float diffToBest = lapTime - bestLapTime;
     }
    
    // Start new lap
    lapStartTime = crossingTime;
    lastCrossingTime = crossingTime;
    currentLapCount = 0;
    deltaValid = false;
    bestLapSearchIdx = 0;
    lastFixTime = 0xFFFFFFFF;  // reset the duplicate filter
  }
  
  prevLat = curLat;
  prevLon = curLon;
  prevGPSTime = curTime;
}

void handleButton() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    unsigned long now = millis();
    if (now - lastButtonPress > BUTTON_DEBOUNCE_MS) {
      lastButtonPress = now;
      setFinishLine();
    }
  }
}

// ==================== DISPLAY FUNCTIONS ====================

// Bottom row (y=62) — three states:
//   0 = timer disabled        -> hint "BTN=SET LINE"
//   1 = timer active, normal  -> best lap time (PURPLE) + live deltas
//   2 = 5 sec after finish    -> old best | current lap time | diff
//
// Colors in state 2:
//   record beaten:  old best = YELLOW, current = PURPLE
//   not beaten:     old best = PURPLE, current = YELLOW
//   diff:           GREEN if faster than old best, RED if slower
//
// State 2 layout (Font_11x18, 11px per char):
//   x=  0..54  : old best     "SSS:T" (5 chars)
//   x= 57..111 : current lap  "SSS:T" (5 chars)
//   x=114..149 : diff         "+-S.T" (4 chars)

void updateDisplayBottom() {
  static int   lastState          = -1;
  static float displayedBestLap   = -2.0;
  static float displayedOldBest   = -2.0;
  static float displayedCurLap    = -2.0;
  static float displayedDiff      = -1000.0;
  static float displayedDeltaTime = -999.0;
  static int   displayedDeltaSpeed = -999;
  static bool  displayedDeltaValid = false;

  // Determine current state
  int state;
  if (!lapTimerActive) {
    state = 0;
  } else if (postCrossingEndTime > 0.0) {
    float now = getCurrentGPSTime();
    if (now < postCrossingEndTime) {
      state = 2;
    } else {
      postCrossingEndTime = 0.0;  // window expired
      state = 1;
    }
  } else {
    state = 1;
  }

  // On state change — clear the bottom row and reset cache
  if (state != lastState) {
    st7735.st7735_fill_rectangle(0, 62, 160, 18, ST7735_BLACK);

    displayedBestLap    = -2.0;
    displayedOldBest    = -2.0;
    displayedCurLap     = -2.0;
    displayedDiff       = -1000.0;
    displayedDeltaTime  = -999.0;
    displayedDeltaSpeed = -999;
    displayedDeltaValid = false;
    lastState = state;
  }

  // -- State 0: timer disabled ------------------------------------------------
  if (state == 0) {
    st7735.st7735_write_str(16, 62, "BTN=SET LINE", Font_11x18, ST7735_YELLOW);
    return;
  }

  // -- State 2: 5-second window after crossing the finish line ---------------
  if (state == 2) {
    // Old best (x=0)
    if (postCrossingOldBest != displayedOldBest) {
      uint16_t c = postCrossingWasBest ? ST7735_YELLOW : ST7735_PURPLE;
      char buf[8];
      if (postCrossingOldBest <= 0.0f) {
        snprintf(buf, sizeof(buf), "---:-");
      } else {
        int secs   = (int)postCrossingOldBest % 1000;
        int tenths = (int)(postCrossingOldBest * 10.0f) % 10;
        snprintf(buf, sizeof(buf), "%3d:%1d", secs, tenths);
      }
      st7735.st7735_write_str(0, 62, String(buf), Font_11x18, c);
      displayedOldBest = postCrossingOldBest;
    }

    // Current lap (x=57)
    if (postCrossingLapTime != displayedCurLap) {
      uint16_t c = postCrossingWasBest ? ST7735_PURPLE : ST7735_YELLOW;
      char buf[8];
      int secs   = (int)postCrossingLapTime % 1000;
      int tenths = (int)(postCrossingLapTime * 10.0f) % 10;
      snprintf(buf, sizeof(buf), "%3d:%1d", secs, tenths);
      st7735.st7735_write_str(57, 62, String(buf), Font_11x18, c);
      displayedCurLap = postCrossingLapTime;
    }

    // Diff (x=114) — only if there was a previous best
    float diffToShow;
    if (postCrossingOldBest > 0.0f) {
      diffToShow = postCrossingLapTime - postCrossingOldBest;
    } else {
      diffToShow = -1000.0f;  // marker: no diff available
    }
    if (diffToShow != displayedDiff) {
      if (diffToShow <= -999.0f) {
        st7735.st7735_write_str(114, 62, "----", Font_11x18, ST7735_WHITE);
      } else {
        float absdiff = fabsf(diffToShow);
        if (absdiff > 9.9f) absdiff = 9.9f;
        int d_sec   = (int)absdiff;
        int d_tenth = (int)(absdiff * 10.0f) % 10;
        char sign   = (diffToShow < 0.0f) ? '-' : '+';
        char buf[6];
        snprintf(buf, sizeof(buf), "%c%d.%d", sign, d_sec, d_tenth);
        uint16_t c = (diffToShow < 0.0f) ? ST7735_GREEN : ST7735_RED;
        st7735.st7735_write_str(114, 62, String(buf), Font_11x18, c);
      }
      displayedDiff = diffToShow;
    }
    return;
  }

  // -- State 1: normal (timer active, no post-window) ------------------------
  // Block 1: best lap time (PURPLE)
  if (bestLapTime != displayedBestLap) {
    char bestBuf[8];
    if (bestLapTime <= 0.0f) {
      snprintf(bestBuf, sizeof(bestBuf), "---:-");
    } else {
      int secs   = (int)bestLapTime % 1000;
      int tenths = (int)(bestLapTime * 10.0f) % 10;
      snprintf(bestBuf, sizeof(bestBuf), "%3d:%1d", secs, tenths);
    }
    st7735.st7735_write_str(0, 62, String(bestBuf), Font_11x18, ST7735_PURPLE);
    displayedBestLap = bestLapTime;
  }

  // Blocks 2-3: live deltas
  if (!deltaValid || bestLapCount == 0) {
    if (displayedDeltaValid) {
      st7735.st7735_write_str(59, 62, "        ", Font_11x18, ST7735_BLACK);
      displayedDeltaValid  = false;
      displayedDeltaTime   = -999.0;
      displayedDeltaSpeed  = -999;
    }
    return;
  }

  float dtRounded = roundf(deltaTime * 10.0f) / 10.0f;
  int   dsRounded = deltaSpeed;

  if (fabsf(dtRounded - displayedDeltaTime) < 0.05f &&
      dsRounded == displayedDeltaSpeed &&
      displayedDeltaValid) {
    return;  // no visible changes
  }

  // Block 2: speed delta "+-NN"
  {
    int ds = dsRounded;
    if (ds >  99) ds =  99;
    if (ds < -99) ds = -99;
    char spdBuf[5];
    snprintf(spdBuf, sizeof(spdBuf), "%+3d", ds);
    uint16_t speedColor = (ds >= 0) ? ST7735_GREEN : ST7735_RED;
    st7735.st7735_write_str(60, 62, String(spdBuf), Font_11x18, speedColor);
  }

  // Block 3: time delta "+-S,T"
  {
    float absdt = fabsf(dtRounded);
    if (absdt > 9.9f) absdt = 9.9f;
    int dt_sec   = (int)absdt;
    int dt_tenth = (int)(absdt * 10.0f) % 10;
    char sign    = (deltaTime <= 0.0f) ? '-' : '+';
    char timeBuf[6];
    snprintf(timeBuf, sizeof(timeBuf), "%c%d,%d", sign, dt_sec, dt_tenth);
    uint16_t timeColor = (deltaTime <= 0.0f) ? ST7735_GREEN : ST7735_RED;
    st7735.st7735_write_str(108, 62, String(timeBuf), Font_11x18, timeColor);
  }

  displayedDeltaTime  = dtRounded;
  displayedDeltaSpeed = dsRounded;
  displayedDeltaValid = true;
}

void updateDisplayBat() {
  int percent = (analogReadMilliVolts(1) * 5 - 3300) / 8;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  uint16_t color;
  if (percent >= 50)      color = ST7735_WHITE;
  else if (percent >= 20) color = ST7735_YELLOW;
  else                    color = ST7735_RED;
  st7735.st7735_write_str(66, 0, "B:" + String(percent) + "% ", Font_7x10, color);
}

void updateWirelessStatus() {
  bool wifiActive = (WiFi.softAPgetStationNum() > 0);
  if (!wifiActive) {
    for (uint8_t k = 0; k < MAX_SRV_CLIENTS; k++) {
      if (serverClients[k] && serverClients[k].connected()) {
        wifiActive = true;
        break;
      }
    }
  }

  uint16_t cw = wifiActive  ? ST7735_GREEN : ST7735_GRAY;
  uint16_t cb = bleConnected ? ST7735_CYAN  : ST7735_GRAY;
  st7735.st7735_write_str(112, 0, "W", Font_7x10, cw);
  st7735.st7735_write_str(119, 0, "B", Font_7x10, cb);
}

void updateDisplayTime() {
  char buf[9];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
           nmea.getHour(), nmea.getMinute(), nmea.getSecond());
  st7735.st7735_write_str(3, 0, String(buf), Font_7x10, ST7735_WHITE);
}

void updateDisplaySats() {
  if (!gpsEverValid) {
    st7735.st7735_write_str(131, 0, "S:00", Font_7x10, ST7735_RED);
    return;
  }
  char buf[5];
  snprintf(buf, sizeof(buf), "S:%02d", nmea.getNumSatellites());
  uint16_t color = nmea.isValid() ? ST7735_GREEN : ST7735_RED;
  st7735.st7735_write_str(131, 0, String(buf), Font_7x10, color);
}

void updateDisplaySpeed() {
  int spd = constrain((int)(nmea.getSpeed() * 1.852 / 1000), 0, 999);
  char buf[5];
  snprintf(buf, sizeof(buf), "%3d", spd);
  uint16_t color = nmea.isValid() ? ST7735_WHITE : ST7735_RED;
  st7735.st7735_write_str2(55, 11, String(buf), Font_16x26, color);
}

void updateDisplayMinMax() {
  auto fmt = [](int v) -> String {
    if (v < 10)  return "  " + String(v, DEC);
    if (v < 100) return " "  + String(v, DEC);
    return String(v, DEC);
  };
  st7735.st7735_write_str(4, 12, fmt((int)maxSpeedSegment), Font_16x26, ST7735_WHITE);
  st7735.st7735_write_str(4, 38, fmt((int)minSpeedSegment), Font_16x26, ST7735_WHITE);
}

void processSerialData() {
    uint8_t i;
    // data received from BLE client -> forward to Serial1 (mirrors Serial -> Serial1 below)
    drainBleRxToSerial1();

    if (Serial1.available()) {  //1
      size_t len = Serial1.available();  //1
      uint8_t rbuf[len];
      Serial1.readBytes(rbuf, len);
      if (Serial) Serial.write(rbuf, len);

      // queue for BLE TX task — does not block GPS task
      bleTxEnqueue(rbuf, len);
      // queue for WiFi TX task — does not block GPS task
      wifiTxEnqueue(rbuf, len);
      
      for (i = 0; i < len; i++) {
        nmea.process(rbuf[i]);
      }
    }
    if (Serial && Serial.available()) {
      size_t len = Serial.available();
      uint8_t sbuf[len];
      Serial.readBytes(sbuf, len);
      Serial1.write(sbuf, len);
    }
}

void updateSpeedTracking() {
  int speedKmh = nmea.getSpeed() * 1.852 / 1000;
  if (speedKmh < 0) speedKmh = 0;
  
  float currentSpeed = (float)speedKmh;
  float currentGPSTime = getCurrentGPSTime();
  
  if (lastSpeed < 0) {
    lastSpeed = currentSpeed;
    minSpeedSegment = currentSpeed;
    maxSpeedSegment = currentSpeed;
    trendStartTime = currentGPSTime;
    trendStartSpeed = currentSpeed;
    lastReportTime = currentGPSTime;
    currentTrend = 0;
    return;
  }
  
  float speedDiff = currentSpeed - lastSpeed;
  int newTrend = 0;
  float threshold = 0.5;
  
  if (speedDiff > threshold) {
    newTrend = 1;
  } else if (speedDiff < -threshold) {
    newTrend = -1;
  } else {
    newTrend = currentTrend;
  }
  
  if (newTrend != currentTrend) {
    currentTrend = newTrend;
    trendStartTime = currentGPSTime;
    trendStartSpeed = currentSpeed;
    minSpeedSegment = currentSpeed;
    maxSpeedSegment = currentSpeed;
    lastReportTime = currentGPSTime;
  } else {
    if (currentSpeed > maxSpeedSegment) maxSpeedSegment = currentSpeed;
    if (currentSpeed < minSpeedSegment) minSpeedSegment = currentSpeed;
  }
  
  lastSpeed = currentSpeed;
}

void handleTelnetClients() {
  if (server.hasClient()) {
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (!serverClients[i] || !serverClients[i].connected()) {
        if (serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.accept();
        break;
      }
    }
    if (i >= MAX_SRV_CLIENTS) server.accept().stop();
  }

  // Read from clients -> Serial1 (non-blocking)
  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && serverClients[i].connected()) {
      while (serverClients[i].available()) {
        size_t len = serverClients[i].available();
        uint8_t sbuf[len];
        serverClients[i].readBytes(sbuf, len);
        Serial1.write(sbuf, len);
      }
    } else {
      if (serverClients[i]) serverClients[i].stop();
    }
  }
}

// ==================== BLE ====================

class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected = true;
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
  }
};

class BleRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* chr) override {
    String v = chr->getValue();
    size_t n = v.length();
    if (n == 0) return;

    // push into ring buffer; loop will drain it and forward to Serial1
    for (size_t i = 0; i < n; i++) {
      size_t next = (bleRxHead + 1) % BLE_RX_BUF_SIZE;
      if (next == bleRxTail) {
        // overflow — drop the oldest byte
        bleRxTail = (bleRxTail + 1) % BLE_RX_BUF_SIZE;
      }
      bleRxBuf[bleRxHead] = (uint8_t)v[i];
      bleRxHead = next;
    }
  }
};

void initBLE() {
  BLEDevice::init(device_name.c_str());
  // Request a larger MTU up-front. RaceChrono still uses 20-byte payloads per
  // spec, but a bigger MTU helps NUS throughput for NMEA streaming.
  BLEDevice::setMTU(247);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BleServerCallbacks());

  // ---- Service A: Nordic UART (NMEA passthrough) ----
  BLEService* nus = bleServer->createService(BLE_UART_SERVICE_UUID);

  bleTxChar = nus->createCharacteristic(
      BLE_UART_TX_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  bleTxChar->addDescriptor(new BLE2902());

  bleRxChar = nus->createCharacteristic(
      BLE_UART_RX_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  bleRxChar->setCallbacks(new BleRxCallbacks());

  nus->start();

  // ---- Service B: RaceChrono DIY (GPS only) ----
  BLEService* rc = bleServer->createService(BLE_RC_SERVICE_UUID);

  bleRcMainChar = rc->createCharacteristic(
      BLE_RC_GPS_MAIN_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  bleRcMainChar->addDescriptor(new BLE2902());

  bleRcTimeChar = rc->createCharacteristic(
      BLE_RC_GPS_TIME_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  bleRcTimeChar->addDescriptor(new BLE2902());

  // Prime both characteristics with "invalid" values so the first READ from the
  // app returns something sane before the first GPS fix arrives.
  {
    uint8_t empty[20];
    memset(empty, 0xFF, sizeof(empty));
    bleRcMainChar->setValue(empty, 20);
    uint8_t emptyTime[3] = {0xFF, 0xFF, 0xFF};
    bleRcTimeChar->setValue(emptyTime, 3);
  }

  rc->start();

  // Advertise BOTH service UUIDs so clients can find either NUS or RaceChrono.
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_UART_SERVICE_UUID);
  adv->addServiceUUID(BLE_RC_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

// Send a buffer to the BLE client in chunks (default MTU payload ~20 bytes;
// after MTU negotiation it can be larger — 180 is a safe upper bound).
void bleNotifyBytes(const uint8_t* data, size_t len) {
  if (!bleConnected || bleTxChar == nullptr || len == 0) return;
  const size_t CHUNK = 180;
  size_t off = 0;
  while (off < len) {
    size_t n = (len - off > CHUNK) ? CHUNK : (len - off);
    bleTxChar->setValue((uint8_t*)(data + off), n);
    bleTxChar->notify();
    off += n;
    // small pause so we don't overflow the BLE stack queue
    delay(3);
  }
}

// Drain whatever accumulated in the BLE RX ring buffer into Serial1
// (same direction as Serial -> Serial1 in processSerialData).
void drainBleRxToSerial1() {
  while (bleRxTail != bleRxHead) {
    uint8_t b = bleRxBuf[bleRxTail];
    bleRxTail = (bleRxTail + 1) % BLE_RX_BUF_SIZE;
    Serial1.write(b);
  }
}

// ==================== RaceChrono DIY BLE encoder ====================
//
// Encodes the current NMEA fix into the two RaceChrono characteristics
// (0x0003 main, 0x0004 time) per the spec at
// https://github.com/aollin/racechrono-ble-diy-device
//
// All multi-byte values are big-endian and unsigned unless noted.

// Pack a 24-bit value (sync<<21 | lower21) into dst[0..2], big-endian.
static void rcPack24BE(uint8_t* dst, uint32_t v24) {
  dst[0] = (uint8_t)((v24 >> 16) & 0xFF);
  dst[1] = (uint8_t)((v24 >>  8) & 0xFF);
  dst[2] = (uint8_t)( v24        & 0xFF);
}

static void rcPack16BE(uint8_t* dst, uint16_t v) {
  dst[0] = (uint8_t)((v >> 8) & 0xFF);
  dst[1] = (uint8_t)( v       & 0xFF);
}

static void rcPack32BE(uint8_t* dst, uint32_t v) {
  dst[0] = (uint8_t)((v >> 24) & 0xFF);
  dst[1] = (uint8_t)((v >> 16) & 0xFF);
  dst[2] = (uint8_t)((v >>  8) & 0xFF);
  dst[3] = (uint8_t)( v        & 0xFF);
}

// Build and notify the two RaceChrono characteristics with the latest NMEA data.
// Called on every loop iteration — internally dedups by NMEA fix timestamp
// so we don't re-notify the same fix.
void updateRaceChronoCharacteristics() {
  if (!bleConnected) return;
  if (bleRcMainChar == nullptr || bleRcTimeChar == nullptr) return;
  if (!nmea.isValid()) return;

  // Dedup: skip if NMEA fix timestamp has not advanced since last call.
  static uint32_t rcLastFixTime = 0xFFFFFFFF;
  uint32_t fixTime = (uint32_t)nmea.getHour()       * 360000UL
                   + (uint32_t)nmea.getMinute()     *   6000UL
                   + (uint32_t)nmea.getSecond()     *    100UL
                   + (uint32_t)nmea.getHundredths();
  if (fixTime == rcLastFixTime) return;
  rcLastFixTime = fixTime;

  // ---- 0x0004: time packet (year/month/day/hour) -------------------------
  // Check whether the hour or the date has changed. If yes — bump sync bits
  // and re-notify 0x0004. 0x0003 is notified every time either way.
  uint32_t timePkt21 = 0x1FFFFFu; // 21 bits set (invalid placeholder)
  {
    int y = nmea.getYear();
    int m = nmea.getMonth();
    int d = nmea.getDay();
    int h = nmea.getHour();
    if (y >= 2000 && m >= 1 && d >= 1) {
      timePkt21 = (uint32_t)((y - 2000) * 8928
                           + (m - 1)    * 744
                           + (d - 1)    * 24
                           +  h) & 0x1FFFFFu;
    }
  }

  if (timePkt21 != rcPrevTimePacket) {
    rcSyncBits = (rcSyncBits + 1) & 0x07;
    rcPrevTimePacket = timePkt21;

    uint32_t v24 = ((uint32_t)rcSyncBits << 21) | timePkt21;
    uint8_t timeBuf[3];
    rcPack24BE(timeBuf, v24);
    bleRcTimeChar->setValue(timeBuf, 3);
    bleRcTimeChar->notify();
  }

  // ---- 0x0003: main GPS packet (20 bytes) --------------------------------
  uint8_t buf[20];

  // bytes 0..2: sync bits (3) + time-from-hour-start (21 bits)
  //   tfh = minute*30000 + second*500 + ms/2
  uint32_t tfh;
  {
    int mm  = nmea.getMinute();
    int ss  = nmea.getSecond();
    int hnd = nmea.getHundredths();  // 0..99 (10 ms units)
    int ms  = hnd * 10;              // milliseconds within the second
    tfh = (uint32_t)(mm * 30000 + ss * 500 + ms / 2) & 0x1FFFFFu;
  }
  uint32_t hdr24 = ((uint32_t)rcSyncBits << 21) | tfh;
  rcPack24BE(&buf[0], hdr24);

  // byte 3: fix quality (2 bits) + locked satellites (6 bits, 0x3F = invalid)
  uint8_t fixQ = 1;  // valid fix
  uint8_t sats = 0x3F;
  {
    int n = nmea.getNumSatellites();
    if (n >= 0 && n < 0x3F) sats = (uint8_t)n;
  }
  buf[3] = (uint8_t)(((fixQ & 0x03) << 6) | (sats & 0x3F));

  // bytes 4..7: latitude, degrees * 1e7, signed int32 BE, 0x7FFFFFFF = invalid
  // bytes 8..11: longitude, same
  // MicroNMEA returns lat/lon in millionths of a degree (int32, deg*1e6).
  // RaceChrono expects deg*1e7, so multiply by 10.
  uint32_t latEnc;
  uint32_t lonEnc;
  {
    int64_t lat1e7 = (int64_t)nmea.getLatitude()  * 10;
    int64_t lon1e7 = (int64_t)nmea.getLongitude() * 10;
    latEnc = (uint32_t)(int32_t)lat1e7;
    lonEnc = (uint32_t)(int32_t)lon1e7;
  }
  rcPack32BE(&buf[4],  latEnc);
  rcPack32BE(&buf[8],  lonEnc);

  // bytes 12..13: altitude
  //   primary:   ((m + 500) * 10) & 0x7FFF      (0.1 m resolution, -500..+6053.5)
  //   fallback:  ((m + 500)      ) & 0x7FFF | 0x8000 (1 m resolution, wider range)
  uint16_t altEnc = 0xFFFF;
  {
    long altMm;
    if (nmea.getAltitude(altMm)) {
      float altM = altMm / 1000.0f;
      float shifted = altM + 500.0f;
      if (shifted >= 0.0f && shifted <= 6053.5f) {
        uint32_t v = (uint32_t)lroundf(shifted * 10.0f) & 0x7FFFu;
        altEnc = (uint16_t)v;
      } else {
        int32_t vi = (int32_t)lroundf(shifted);
        if (vi < 0) vi = 0;
        altEnc = (uint16_t)((vi & 0x7FFF) | 0x8000);
      }
    }
  }
  rcPack16BE(&buf[12], altEnc);

  // bytes 14..15: speed
  //   primary:   (km/h * 100) & 0x7FFF       (0.01 km/h, 0..655.35)
  //   fallback:  (km/h * 10)  & 0x7FFF | 0x8000
  // MicroNMEA getSpeed() returns knots * 1000. Convert to km/h.
  uint16_t spdEnc = 0xFFFF;
  {
    long spdMilliKnots = nmea.getSpeed();
    if (spdMilliKnots >= 0) {
      float kmh = (spdMilliKnots / 1000.0f) * 1.852f;
      if (kmh <= 655.35f) {
        uint32_t v = (uint32_t)lroundf(kmh * 100.0f) & 0x7FFFu;
        spdEnc = (uint16_t)v;
      } else {
        uint32_t v = (uint32_t)lroundf(kmh * 10.0f) & 0x7FFFu;
        spdEnc = (uint16_t)(v | 0x8000u);
      }
    }
  }
  rcPack16BE(&buf[14], spdEnc);

  // bytes 16..17: bearing (degrees * 100), 0xFFFF = invalid
  // MicroNMEA getCourse() returns degrees * 1000.
  uint16_t brgEnc = 0xFFFF;
  {
    long crsMilliDeg = nmea.getCourse();
    if (crsMilliDeg >= 0) {
      uint32_t v = (uint32_t)((crsMilliDeg + 5) / 10) & 0xFFFFu;  // *100
      if (v == 0xFFFF) v = 0xFFFE; // avoid collision with invalid marker
      brgEnc = (uint16_t)v;
    }
  }
  rcPack16BE(&buf[16], brgEnc);

  // bytes 18..19: HDOP, VDOP (dop * 10). MicroNMEA does not expose these —
  // send invalid markers.
  buf[18] = 0xFF;
  buf[19] = 0xFF;

  bleRcMainChar->setValue(buf, 20);
  bleRcMainChar->notify();
}

// Reconnection handling — after a disconnect we must restart advertising
void handleBleReconnect() {
  if (!bleConnected && bleWasConnected) {
    delay(200); // give the stack time to tear down the connection
    BLEDevice::startAdvertising();
    bleWasConnected = false;
  }
  if (bleConnected && !bleWasConnected) {
    bleWasConnected = true;
  }
}

void setup() {
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  st7735.st7735_init();
  st7735_spi.setFrequency(27000000);
  st7735.st7735_fill_screen(ST7735_BLACK);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 33, 34);
  Serial1.setRxBufferSize(1024);

  analogReadResolution(12);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  // Create mutex before starting tasks
  nmeaMutex    = xSemaphoreCreateMutex();
  spiMutex     = xSemaphoreCreateMutex();
  wifiTxQueue  = xQueueCreate(WIFI_TX_BUF_SIZE, sizeof(uint8_t));

  // WiFi + telnet server (must happen in setup, not in a task)
  WiFi.softAP(ssid, password);
  server.begin();
  server.setNoDelay(true);
  st7735.st7735_write_str(0, 0, (String)"WiFiBTGPS     pass: 87654321 IP:192.168.4.1Port:23");
  delay(2500);
  st7735.st7735_fill_screen(ST7735_BLACK);

  initBLE();

  // ---- Spawn RTOS tasks ----
  // GPS task: Core 1, highest priority — must drain Serial1 before HW buffer overflows
  xTaskCreatePinnedToCore(taskGPSFn,     "GPS",     4096, nullptr, 4, &taskGPS,     1);
  // WiFi/Telnet task: Core 0, medium priority
  xTaskCreatePinnedToCore(taskWiFiFn,    "WiFi",    4096, nullptr, 2, &taskWiFi,    0);
  // BLE reconnect task: Core 0, low priority (BLE stack itself has its own internal tasks)
  xTaskCreatePinnedToCore(taskBLEFn,     "BLE",     3072, nullptr, 1, &taskBLE,     0);
  // Display task: Core 0, low priority, strict 10 Hz via vTaskDelayUntil
  xTaskCreatePinnedToCore(taskDisplayFn, "Display", 4096, nullptr, 1, &taskDisplay, 0);
}

// Arduino loop() is itself a FreeRTOS task — leave it suspended.
void loop() {
  vTaskDelay(portMAX_DELAY);
}

// ==================== RTOS TASKS ====================

// GPS task: reads Serial1 -> NMEA, forwards to BLE/WiFi, lap logic
void taskGPSFn(void* pv) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(50);  // 20 Hz
  while (true) {
    // Forward Serial1 bytes, feed MicroNMEA, notify BLE/telnet
    processSerialData();

    // Button (debounced, cheap)
    handleButton();

    // GPS-derived logic — brief mutex hold
    if (xSemaphoreTake(nmeaMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      if (nmea.isValid()) {
        if (!gpsEverValid) gpsEverValid = true;
        updateSpeedTracking();
        checkLineCrossing();
        recordCurrentLapPoint();
        updateDeltaComparison();
        updateRaceChronoCharacteristics();
      }
      xSemaphoreGive(nmeaMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// WiFi/Telnet task: 10 Hz
void taskWiFiFn(void* pv) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(100);

  while (true) {
    if (server.hasClient()) {
      for (uint8_t i = 0; i < MAX_SRV_CLIENTS; i++) {
        if (!serverClients[i] || !serverClients[i].connected()) {
          if (serverClients[i]) serverClients[i].stop();
          serverClients[i] = server.accept();
          serverClients[i].setTimeout(2000);  // 2 sec timeout
          xQueueReset(wifiTxQueue);
          break;
        }
      }
    }

    // Wifi clients -> Serial1
    for (uint8_t i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (serverClients[i] && serverClients[i].connected()) {
        while (serverClients[i].available()) {
          size_t len = serverClients[i].available();
          uint8_t sbuf[len];
          serverClients[i].readBytes(sbuf, len);
          Serial1.write(sbuf, len);
        }
      } else {
        if (serverClients[i]) serverClients[i].stop();
      }
    }

    uint8_t chunk[256];
    size_t n = 0;
    while (n < sizeof(chunk) && xQueueReceive(wifiTxQueue, &chunk[n], 0) == pdTRUE) n++;
    if (n > 0) {
      for (uint8_t i = 0; i < MAX_SRV_CLIENTS; i++) {
        if (serverClients[i] && serverClients[i].connected()) {
          serverClients[i].write(chunk, n);  // таймаут защитит от зависания
        }
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// BLE task: drains NMEA TX queue + handles reconnect
void taskBLEFn(void* pv) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(100);  // 10 Hz

  while (true) {
    // Drain the NMEA passthrough queue into BLE notify chunks
    while (bleTxTail != bleTxHead) {
      // Gather up to CHUNK bytes contiguously
      const size_t CHUNK = 180;
      uint8_t chunk[CHUNK];
      size_t n = 0;
      while (n < CHUNK && bleTxTail != bleTxHead) {
        chunk[n++] = bleTxBuf[bleTxTail];
        bleTxTail = (bleTxTail + 1) % BLE_TX_BUF_SIZE;
      }
      bleNotifyBytes(chunk, n);
    }

    handleBleReconnect();
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// Display task: exactly 10 Hz using vTaskDelayUntil
// Single display task: 10 Hz fast updates, 1 Hz slow updates (every 10th tick)
void taskDisplayFn(void* pv) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(100);  // 10 Hz
  uint8_t slowTick = 0;

  while (true) {
    if (xSemaphoreTake(nmeaMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // 10 Hz
        updateDisplaySpeed();
        updateDisplayMinMax();
        updateDisplayBottom();
        // 1 Hz
        if (++slowTick >= 10) {
          slowTick = 0;
          updateDisplayBat();
          updateDisplayTime();
          updateDisplaySats();
          updateWirelessStatus();
        }
        xSemaphoreGive(spiMutex);
      }
      xSemaphoreGive(nmeaMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}