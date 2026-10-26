/*
 * RS232 FWD GPS v0.9.9 MADRID FINAL - ESP32-S3 (UM980 HAS) + BNO085 + Dynatest FWD
 *
 * Hardware:
 *   - GNSS (UM980):    RX=44, TX=43, BAUD=115200
 *   - Dynatest Output: RX=18, TX=17, BAUD=38400
 *   - IMU (BNO085):    SDA=8, SCL=9 (Wire1), I2C 400kHz, addr 0x4B
 *   - LED_RED:         Pin 4  (GNSS/HAS state)
 *   - LED_GREEN:       Pin 5  (Movement/Lock state)
 *
 * State Machine:
 *   Movement: MOVING -> AVERAGING (speed<0.20m/s, 2s) -> LOCKED (15s avg) -> MOVING (>1.0m dist o speed>0.30m/s)
 *   PPP/HAS:  SIN_PPP -> PPP_CONVERGING -> PPP_ESTABLE
 *
 * Antena->Piston offset: 0.55m, bearing = yaw + 270° (antena a la derecha, piston a la izquierda)
 * Declinación magnética Madrid: 1.0° (yaw_geografico = yaw_magnetico - declinacion)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#define GNSS_RX         44
#define GNSS_TX         43
#define GNSS_BAUD       115200
#define DYNATEST_RX     18
#define DYNATEST_TX     17
#define DYNATEST_BAUD   38400

#define LED_RED         4
#define LED_GREEN       5

#define I2C_SDA         8
#define I2C_SCL         9
#define I2C_FREQ        400000

#define OFFSET_M              0.55
#define DECLINATION_OFFSET    1.0
#define EARTH_RADIUS          6378137.0

#define STOP_THRESHOLD_MS      2000
#define SPEED_FRESHNESS_MS     2000
#define GGA_FRESHNESS_MS       2000
#define IMU_RETRY_MS           5000
#define IMU_LOG_INTERVAL_MS    5000
#define AVERAGING_WINDOW_MS    15000
#define OUTPUT_PERIOD_MS       100
#define NEW_LOCATION_DIST      1.0
#define MAX_SAMPLES            160

#define SPEED_ENTER_STOP       0.20
#define SPEED_EXIT_STOP        0.30

enum MovementState { MOVING = 0, AVERAGING = 1, LOCKED = 2 };
enum PPPState       { SIN_PPP = 0, PPP_CONVERGING = 1, PPP_ESTABLE = 2 };

MovementState movementState = MOVING;
PPPState pppState = SIN_PPP;

uint32_t lastStopCheckMs = 0;
uint32_t averagingStartMs = 0;
uint32_t lastOutputMs = 0;

bool gnssValid = false;
double currentLat = 0.0;
double currentLon = 0.0;
double currentAlt = 0.0;
double currentSpeedMS = 0.0;
double currentCourseOverGround = 0.0;

int satCount = 0;
char utcTime[12] = "000000.00";

uint32_t lastSpeedUpdateMs = 0;
uint32_t lastGgaMs = 0;
uint32_t lastSampledGgaMs = 0;

double latBuffer[MAX_SAMPLES];
double lonBuffer[MAX_SAMPLES];
double altBuffer[MAX_SAMPLES];
double yawBuffer[MAX_SAMPLES];

int sampleCount = 0;
int yawSampleCount = 0;

bool lockedValid = false;
double lockedLat = 0.0;
double lockedLon = 0.0;
double lockedAlt = 0.0;
double lockedYaw = NAN;

double lastLockedLat = 0.0;
double lastLockedLon = 0.0;

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

double currentYaw = NAN;
uint32_t lastBnoMs = 0;
uint32_t lastImuRetryMs = 0;
uint32_t lastImuLogMs = 0;
bool bnoAvailable = false;

static char gnssLineBuf[220];
static int gnssLineIdx = 0;

// ============================================================================
// MATEMÁTICAS
// ============================================================================

double toRadians(double deg) { return deg * M_PI / 180.0; }
double toDegrees(double rad) { return rad * 180.0 / M_PI; }

double normalizeAngle(double ang) {
  while (ang < 0.0) ang += 360.0;
  while (ang >= 360.0) ang -= 360.0;
  return ang;
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dlat = toRadians(lat2 - lat1);
  double dlon = toRadians(lon2 - lon1);
  double a = sin(dlat/2.0) * sin(dlat/2.0) +
             cos(toRadians(lat1)) * cos(toRadians(lat2)) *
             sin(dlon/2.0) * sin(dlon/2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return EARTH_RADIUS * c;
}

double mean_circular_deg(double* buf, int cnt) {
  if (cnt <= 0) return NAN;
  double sin_sum = 0.0, cos_sum = 0.0;
  for (int i = 0; i < cnt; i++) {
    double rad = toRadians(buf[i]);
    sin_sum += sin(rad);
    cos_sum += cos(rad);
  }
  double mean_rad = atan2(sin_sum / cnt, cos_sum / cnt);
  return normalizeAngle(toDegrees(mean_rad));
}

double trimmed_mean(double* buf, int cnt) {
  if (cnt <= 0) return NAN;

  static double sortBuf[MAX_SAMPLES];
  memcpy(sortBuf, buf, cnt * sizeof(double));

  for (int i = 0; i < cnt - 1; i++) {
    for (int j = 0; j < cnt - i - 1; j++) {
      if (sortBuf[j] > sortBuf[j+1]) {
        double tmp = sortBuf[j];
        sortBuf[j] = sortBuf[j+1];
        sortBuf[j+1] = tmp;
      }
    }
  }

  int trimCnt = (int)ceil(cnt * 0.05);
  int start, end;
  if ((trimCnt * 2) >= cnt) {
    start = 0;
    end = cnt;
  } else {
    start = trimCnt;
    end = cnt - trimCnt;
  }

  double sum = 0.0;
  int validCnt = 0;
  for (int i = start; i < end; i++) {
    if (!isnan(sortBuf[i])) {
      sum += sortBuf[i];
      validCnt++;
    }
  }

  return (validCnt > 0) ? sum / validCnt : NAN;
}

// ============================================================================
// PARSING NMEA
// ============================================================================

void parseGGA(const char* line) {
  const char* asterisk = strchr(line, '*');
  if (!asterisk) return;

  unsigned char calcChecksum = 0;
  for (const char* p = line + 1; p < asterisk; p++) calcChecksum ^= *p;

  unsigned char rxChecksum = 0;
  sscanf(asterisk + 1, "%02hhx", &rxChecksum);
  if (calcChecksum != rxChecksum) {
    Serial.println("[GNSS] GGA checksum error");
    return;
  }

  char* fields[15];
  char lineCopy[220];
  strncpy(lineCopy, line, sizeof(lineCopy) - 1);
  lineCopy[sizeof(lineCopy) - 1] = '\0';

  int fieldCount = 0;
  fields[0] = lineCopy;
  for (char* p = lineCopy; *p; p++) {
    if (*p == ',') {
      *p = '\0';
      fields[++fieldCount] = p + 1;
      if (fieldCount >= 14) break;
    }
  }

  if (fieldCount < 9) return;

  int fixQ = atoi(fields[6]);
  if (fixQ < 1) return;

  const char* latStr = fields[2];
  const char* latHem = fields[3];
  if (strlen(latStr) < 7) return;

  int latDeg = (latStr[0] - '0') * 10 + (latStr[1] - '0');
  double latMin = strtod(latStr + 2, NULL);
  double lat = latDeg + latMin / 60.0;
  if (latHem[0] == 'S') lat = -lat;

  const char* lonStr = fields[4];
  const char* lonHem = fields[5];
  if (strlen(lonStr) < 8) return;

  int lonDeg = (lonStr[0] - '0') * 100 + (lonStr[1] - '0') * 10 + (lonStr[2] - '0');
  double lonMin = strtod(lonStr + 3, NULL);
  double lon = lonDeg + lonMin / 60.0;
  if (lonHem[0] == 'W') lon = -lon;

  double alt = strtod(fields[9], NULL);

  strncpy(utcTime, fields[1], sizeof(utcTime) - 1);
  utcTime[sizeof(utcTime) - 1] = '\0';
  satCount = atoi(fields[7]);

  currentLat = lat;
  currentLon = lon;
  currentAlt = alt;
  gnssValid = true;
  lastGgaMs = millis();

  Serial.printf("[GNSS] GGA: lat=%.6f, lon=%.6f, alt=%.1f, fixQ=%d, sat=%d\n",
                lat, lon, alt, fixQ, satCount);
}

void parseRMC(const char* line) {
  const char* asterisk = strchr(line, '*');
  if (!asterisk) return;

  unsigned char calcChecksum = 0;
  for (const char* p = line + 1; p < asterisk; p++) calcChecksum ^= *p;

  unsigned char rxChecksum = 0;
  sscanf(asterisk + 1, "%02hhx", &rxChecksum);
  if (calcChecksum != rxChecksum) return;

  char* fields[13];
  char lineCopy[220];
  strncpy(lineCopy, line, sizeof(lineCopy) - 1);
  lineCopy[sizeof(lineCopy) - 1] = '\0';

  int fieldCount = 0;
  fields[0] = lineCopy;
  for (char* p = lineCopy; *p; p++) {
    if (*p == ',') {
      *p = '\0';
      fields[++fieldCount] = p + 1;
      if (fieldCount >= 12) break;
    }
  }

  if (fieldCount < 8) return;
  if (fields[2][0] != 'A') return;

  double speedKnots = strtod(fields[7], NULL);
  currentSpeedMS = speedKnots * 0.51444;
  currentCourseOverGround = strtod(fields[8], NULL);
  lastSpeedUpdateMs = millis();

  Serial.printf("[GNSS] RMC: speed=%.2f m/s, COG=%.1f°\n",
                currentSpeedMS, currentCourseOverGround);
}

void parsePPPNAV(const char* line) {
  if (strstr(line, "HAS") || strstr(line, "PPP_ESTABLE")) {
    pppState = PPP_ESTABLE;
    Serial.println("[HAS] State: ESTABLE");
  } else if (strstr(line, "PPP_CONVERGING")) {
    pppState = PPP_CONVERGING;
    Serial.println("[HAS] State: CONVERGING");
  } else {
    pppState = SIN_PPP;
    Serial.println("[HAS] State: SIN_PPP");
  }
}

void readGNSSSerial() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '$' || c == '#') {
      gnssLineIdx = 0;
      gnssLineBuf[gnssLineIdx++] = c;
    } else if (gnssLineIdx > 0) {
      if (c == '\n') {
        gnssLineBuf[gnssLineIdx] = '\0';

        if (strstr(gnssLineBuf, "$GPGGA")) {
          parseGGA(gnssLineBuf);
        } else if (strstr(gnssLineBuf, "$GPRMC")) {
          parseRMC(gnssLineBuf);
        } else if (strstr(gnssLineBuf, "#PPPNAVA") || strstr(gnssLineBuf, "HAS")) {
          parsePPPNAV(gnssLineBuf);
        }

        gnssLineIdx = 0;
      } else if (gnssLineIdx < (int)sizeof(gnssLineBuf) - 1) {
        gnssLineBuf[gnssLineIdx++] = c;
      } else {
        gnssLineIdx = 0;
      }
    }
  }
}

// ============================================================================
// IMU (BNO085)
// ============================================================================

bool initBNO085() {
  if (!bno08x.begin_I2C(0x4B, &Wire1)) {
    Serial.println("[IMU] BNO085 init failed!");
    return false;
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("[IMU] Could not enable rotation vector");
    return false;
  }
  bnoAvailable = true;
  lastBnoMs = millis();
  Serial.println("[IMU] BNO085 initialized");
  return true;
}

double readYaw() {
  uint32_t nowMs = millis();

  if (!bnoAvailable) {
    if (nowMs - lastImuRetryMs >= IMU_RETRY_MS) {
      lastImuRetryMs = nowMs;
      Serial.println("[IMU] Retry init...");
      bnoAvailable = initBNO085();
    }
    return NAN;
  }

  if (nowMs - lastBnoMs > IMU_RETRY_MS) {
    if (nowMs - lastImuLogMs >= IMU_LOG_INTERVAL_MS) {
      lastImuLogMs = nowMs;
      Serial.println("[IMU] Timeout - yaw=NAN");
    }
    currentYaw = NAN;
    return NAN;
  }

  if (bno08x.wasReset()) {
    Serial.println("[IMU] Reset detected - reinitializing...");
    bnoAvailable = initBNO085();
    if (!bnoAvailable) return NAN;
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float i = sensorValue.un.rotationVector.i;
      float j = sensorValue.un.rotationVector.j;
      float k = sensorValue.un.rotationVector.k;
      float real = sensorValue.un.rotationVector.real;

      double yaw_rad = atan2(2.0 * (real * k + i * j),
                             1.0 - 2.0 * (j * j + k * k));
      double yaw_mag = toDegrees(yaw_rad);
      double yaw_geo = normalizeAngle(yaw_mag - DECLINATION_OFFSET);

      currentYaw = yaw_geo;
      lastBnoMs = nowMs;
      return currentYaw;
    }
  }

  return currentYaw;
}

// ============================================================================
// LEDS
// ============================================================================

void updateLED() {
  uint32_t nowMs = millis();

  static uint32_t startupMs = 0;
  if (startupMs == 0) startupMs = nowMs;

  if (nowMs - startupMs < 1200) {
    uint32_t cycle = (nowMs - startupMs) % 400;
    bool on = (cycle < 200);
    digitalWrite(LED_RED, on ? HIGH : LOW);
    digitalWrite(LED_GREEN, on ? HIGH : LOW);
    return;
  }

  if (!gnssValid) {
    digitalWrite(LED_RED, LOW);
  } else if (pppState == PPP_CONVERGING) {
    uint32_t cycle = nowMs % 400;
    digitalWrite(LED_RED, (cycle < 200) ? HIGH : LOW);
  } else {
    digitalWrite(LED_RED, HIGH);
  }

  if (movementState == MOVING) {
    digitalWrite(LED_GREEN, LOW);
  } else if (movementState == AVERAGING) {
    uint32_t cycle = nowMs % 1000;
    digitalWrite(LED_GREEN, (cycle < 500) ? HIGH : LOW);
  } else if (movementState == LOCKED && lockedValid) {
    digitalWrite(LED_GREEN, HIGH);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }
}

// ============================================================================
// MÁQUINA DE ESTADOS
// ============================================================================

int getOutputFixQ() {
  if (!gnssValid) return 0;
  if (movementState == LOCKED && lockedValid) return 4;
  if (pppState == PPP_ESTABLE) return 4;
  if (pppState == PPP_CONVERGING) return 2;
  return 1;
}

void updateMovementState() {
  uint32_t nowMs = millis();

  bool speedValid = (nowMs - lastSpeedUpdateMs) <= SPEED_FRESHNESS_MS;
  if (!speedValid) currentSpeedMS = 0.0;

  bool ggaValid = (nowMs - lastGgaMs) <= GGA_FRESHNESS_MS;
  if (!ggaValid) gnssValid = false;

  switch (movementState) {
    case MOVING:
      if (speedValid && currentSpeedMS < SPEED_ENTER_STOP) {
        if (lastStopCheckMs == 0) lastStopCheckMs = nowMs;
      } else if (speedValid && currentSpeedMS >= SPEED_EXIT_STOP) {
        lastStopCheckMs = 0;
      }

      if (lastStopCheckMs > 0 && (nowMs - lastStopCheckMs) >= STOP_THRESHOLD_MS) {
        averagingStartMs = nowMs;
        sampleCount = 0;
        yawSampleCount = 0;
        memset(latBuffer, 0, sizeof(latBuffer));
        memset(lonBuffer, 0, sizeof(lonBuffer));
        memset(altBuffer, 0, sizeof(altBuffer));
        memset(yawBuffer, 0, sizeof(yawBuffer));
        lastSampledGgaMs = 0;

        movementState = AVERAGING;
        lockedValid = false;
        Serial.println("[STATE] MOVING -> AVERAGING");
      }
      break;

    case AVERAGING:
      if (lastGgaMs != lastSampledGgaMs && sampleCount < MAX_SAMPLES) {
        latBuffer[sampleCount] = currentLat;
        lonBuffer[sampleCount] = currentLon;
        altBuffer[sampleCount] = currentAlt;
        sampleCount++;
        lastSampledGgaMs = lastGgaMs;

        if (!isnan(currentYaw) && yawSampleCount < MAX_SAMPLES) {
          yawBuffer[yawSampleCount++] = currentYaw;
        }

        Serial.printf("[SAMPLE] %d/%d - %.1fs remaining\n",
                      sampleCount, MAX_SAMPLES,
                      (AVERAGING_WINDOW_MS - (nowMs - averagingStartMs)) / 1000.0);
      }

      if (speedValid && currentSpeedMS > SPEED_EXIT_STOP) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;
        sampleCount = 0;
        Serial.println("[STATE] AVERAGING -> MOVING (velocidad aumentó)");
        break;
      }

      if ((nowMs - averagingStartMs) >= AVERAGING_WINDOW_MS && sampleCount > 0) {
        lockedLat = trimmed_mean(latBuffer, sampleCount);
        lockedLon = trimmed_mean(lonBuffer, sampleCount);
        lockedAlt = trimmed_mean(altBuffer, sampleCount);
        lockedYaw = mean_circular_deg(yawBuffer, yawSampleCount);

        if (isnan(lockedLat) || isnan(lockedLon)) {
          movementState = MOVING;
          lockedValid = false;
          sampleCount = 0;
          Serial.println("[STATE] AVERAGING failed - invalid trimmed means");
          break;
        }

        if (!isnan(lockedYaw)) {
          double correctionBearing = normalizeAngle(lockedYaw + 270.0);
          double offsetRad = OFFSET_M / EARTH_RADIUS;
          double bearingRad = toRadians(correctionBearing);

          double latOffset = offsetRad * cos(bearingRad);
          double lonOffset = offsetRad * sin(bearingRad) / cos(toRadians(lockedLat));

          lockedLat += latOffset;
          lockedLon += lonOffset;

          Serial.printf("[LOCKED] offset aplicado, bearing=%.1f°\n", correctionBearing);
        } else {
          Serial.println("[LOCKED] yaw NAN - offset NO aplicado (posición GNSS pura)");
        }

        lastLockedLat = lockedLat;
        lastLockedLon = lockedLon;

        movementState = LOCKED;
        lockedValid = true;
        lastStopCheckMs = 0;

        Serial.printf("[STATE] AVERAGING -> LOCKED lat=%.6f lon=%.6f yaw=%.1f\n",
                      lockedLat, lockedLon, lockedYaw);
      }
      break;

    case LOCKED:
      if (speedValid && currentSpeedMS > SPEED_EXIT_STOP) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;
        sampleCount = 0;
        Serial.println("[STATE] LOCKED -> MOVING (velocidad alta)");
        break;
      }

      if (gnssValid) {
        double dist = haversine(currentLat, currentLon, lastLockedLat, lastLockedLon);
        if (dist > NEW_LOCATION_DIST) {
          movementState = MOVING;
          lockedValid = false;
          lastStopCheckMs = 0;
          sampleCount = 0;
          Serial.printf("[STATE] LOCKED -> MOVING (dist %.2fm)\n", dist);
          break;
        }
      }
      break;
  }
}

void transmitDynatestOutput() {
  uint32_t nowMs = millis();
  if (nowMs - lastOutputMs < OUTPUT_PERIOD_MS) return;
  lastOutputMs = nowMs;

  int fixQ = getOutputFixQ();
  if (!gnssValid || fixQ < 1) return;

  double outLat, outLon, outAlt, outYaw;

  if (movementState == LOCKED && lockedValid) {
    outLat = lockedLat;
    outLon = lockedLon;
    outAlt = lockedAlt;
    outYaw = lockedYaw;

    Serial.printf("[TX-LOCKED] fixQ=%d pos=%.6f,%.6f yaw=%.1f\n",
                  fixQ, outLat, outLon, outYaw);
  } else {
    outLat = currentLat;
    outLon = currentLon;
    outAlt = currentAlt;
    outYaw = currentYaw;

    if (!isnan(outYaw)) {
      double correctionBearing = normalizeAngle(outYaw + 270.0);
      double offsetRad = OFFSET_M / EARTH_RADIUS;
      double bearingRad = toRadians(correctionBearing);

      double latOffset = offsetRad * cos(bearingRad);
      double lonOffset = offsetRad * sin(bearingRad) / cos(toRadians(outLat));

      outLat += latOffset;
      outLon += lonOffset;

      Serial.printf("[TX-INST] fixQ=%d pos=%.6f,%.6f yaw=%.1f bearing=%.1f\n",
                    fixQ, outLat, outLon, outYaw, correctionBearing);
    } else {
      Serial.printf("[TX-INST] fixQ=%d pos=%.6f,%.6f yaw=NAN (sin offset)\n",
                    fixQ, outLat, outLon);
    }
  }

  char gga[120];
  snprintf(gga, sizeof(gga),
           "$GPGGA,%s,%02d%07.4f,%c,%03d%07.4f,%c,%d,%02d,1.0,%.1f,M,0.0,M,,",
           utcTime,
           (int)fabs(outLat), (fabs(outLat) - (int)fabs(outLat)) * 60.0, (outLat >= 0) ? 'N' : 'S',
           (int)fabs(outLon), (fabs(outLon) - (int)fabs(outLon)) * 60.0, (outLon >= 0) ? 'E' : 'W',
           fixQ, satCount, outAlt);

  unsigned char checksum = 0;
  for (const char* p = gga + 1; *p; p++) checksum ^= *p;

  char output[140];
  snprintf(output, sizeof(output), "%s*%02X\r\n", gga, checksum);

  Serial2.write((const uint8_t*)output, strlen(output));
}

// ============================================================================
// SETUP Y LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[SETUP] RS232 FWD GPS v0.9.9 MADRID FINAL");

  Serial1.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
  Serial.println("[SETUP] GNSS UART initialized");

  Serial2.begin(DYNATEST_BAUD, SERIAL_8N1, DYNATEST_RX, DYNATEST_TX);
  Serial.println("[SETUP] Dynatest Output UART initialized");

  Wire1.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  if (!initBNO085()) {
    Serial.println("[SETUP] BNO085 init failed - will retry in loop");
  }

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  Serial.println("[SETUP] Ready!");
}

void loop() {
  readGNSSSerial();
  readYaw();
  updateMovementState();
  updateLED();
  transmitDynatestOutput();
}
