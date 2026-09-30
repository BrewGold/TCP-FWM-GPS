/*
  RS232-FMW-GPS - Rev.3.0
  Arduino UNO R4 WiFi

  Cambios principales respecto a 2.2:
  - BLE sustituye al AP WiFi y al terminal TCP de diagnóstico/control.
  - Ethernet W5500 sigue siendo el canal FWM principal.
  - COM2 D2 @ 38400 sigue siendo la salida de emergencia/supervisión.

  LEDS:
    OUT5: +5V / D5 / GND
    OUT6: +5V / D6 / GND

  Prueba de arranque:
    D5 y D6 parpadean durante 5 segundos

  SENSORES REQUERIDOS:
    1. GNSS: UM980 por COM3 (D0/D1) a 115200 bps
       - GGA a 10 Hz
       - RMC a 10 Hz
       - PUBX,00 a 10 Hz (para detectar HAS)
       - HAS (Galileo High Accuracy Service) habilitado

    2. IMU/MAGNETÓMETRO: BNO085 por I2C (SDA/SCL)
       - Proporciona yaw para corrección de offset antena-pistón
       - Sin BNO085: corrección desactivada

    3. ETHERNET: Shield W5500 con IP 192.168.1.122 puerto 15919

    4. BLE: UNO R4 WiFi nativo (diagnóstico + control)

  CONFIGURACIÓN UM980 (enviar por u-center):
    UNLOG COM3
    CONFIG COM3 115200
    GNGGA COM3 0.1
    GNRMC COM3 0.1
    CONFIG NMEA PUBX ENABLE
    ENABLE HAS
    SAVECONFIG

  SALIDA FWD:
    Puerto D2 a 38400 bps (soft-serial, TX only)
    GGA NMEA cada 100 ms (10 Hz)
    Sin RMC, sin otros mensajes

  BLE (Rev.3.0):
    Nombre anunciado: FWD-GPS
    Servicio: 19B10000-E8F2-537E-4F6C-D104768A1214

    Comandos:
      freq <1-10>        - Cambia frecuencia de salida GGA (Hz)
      speed_stop <0-1>   - Umbral parada (m/s)
      speed_move <0-1>   - Umbral movimiento (m/s)
      offset <0-2>       - Offset antena (m)
      decl <-180-180>    - Declinación magnética (°)
      avg <5-60>         - Ventana promedio (s)
      diag <1-60>        - Intervalo de diagnóstico BLE (s)
      status             - Muestra estado actual
      help               - Lista de comandos

  NOTA IMPORTANTE:
    SoftwareSerial.h NO es compatible con el core Renesas RA4M1
    del Arduino UNO R4 WiFi. Se implementa COM2 como bit-banging TX-only,
    compensado con micros() para evitar drift.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Ethernet.h>
#include <ArduinoBLE.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

// ============================================================================
// CONFIGURACIÓN
// ============================================================================

#define GNSS_BAUD                 115200
#define COM2_BAUD                 38400
#define COM2_TX_PIN               2

#define SD_CS_PIN                 4
#define ETHERNET_CS_PIN           10

#define LED1_PIN                  5
#define LED2_PIN                  6

#define BNO085_ADDRESS            0x4B
#define I2C_CLOCK_HZ              100000

#define STARTUP_LED_TEST_MS       5000
#define STARTUP_LED_PERIOD_MS     400

// Ethernet
byte ethernetMac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ethernetTargetIP(192, 168, 1, 122);
const uint16_t ethernetTargetPort = 15919;

EthernetClient ethernetClient;
bool ethernetReady = false;
uint32_t lastEthernetCheckMs = 0;

// BLE
const char BLE_DEVICE_NAME[] = "FWD-GPS";
const char BLE_SERVICE_UUID[] = "19B10000-E8F2-537E-4F6C-D104768A1214";
const char BLE_COMMAND_UUID[] = "19B10001-E8F2-537E-4F6C-D104768A1214";
const char BLE_RESPONSE_UUID[] = "19B10002-E8F2-537E-4F6C-D104768A1214";
const char BLE_DIAGNOSTIC_UUID[] = "19B10003-E8F2-537E-4F6C-D104768A1214";
const int BLE_PAYLOAD_MAX = 240;

BLEService diagnosticService(BLE_SERVICE_UUID);
BLEStringCharacteristic commandCharacteristic(BLE_COMMAND_UUID, BLEWrite, BLE_PAYLOAD_MAX);
BLEStringCharacteristic responseCharacteristic(BLE_RESPONSE_UUID, BLERead | BLENotify, BLE_PAYLOAD_MAX);
BLEStringCharacteristic diagnosticCharacteristic(BLE_DIAGNOSTIC_UUID, BLERead | BLENotify, BLE_PAYLOAD_MAX);
bool bleReady = false;
bool bleWasConnected = false;

// Navegación
#define OFFSET_M_DEFAULT           0.55
#define DECLINATION_DEG_DEFAULT    1.0
#define EARTH_RADIUS_M             6378137.0

#define STOP_CONFIRMATION_MS       2000
#define SPEED_FRESHNESS_MS         2000
#define GGA_FRESHNESS_MS           2000
#define IMU_RETRY_MS               5000
#define IMU_TIMEOUT_MS             5000

#define OUTPUT_PERIOD_MS_DEFAULT   100
#define AVERAGING_WINDOW_MS_DEFAULT 15000
#define RELOCK_DISTANCE_M          1.0
#define MAX_SAMPLES                160

#define SPEED_ENTER_STOP_MS_DEFAULT 0.20
#define SPEED_EXIT_STOP_MS_DEFAULT  0.30

// ============================================================================
// VARIABLES CONFIGURABLES
// ============================================================================

uint16_t OUTPUT_PERIOD_MS_VAR = OUTPUT_PERIOD_MS_DEFAULT;
uint16_t AVERAGING_WINDOW_MS_VAR = AVERAGING_WINDOW_MS_DEFAULT;
double SPEED_ENTER_STOP_MS_VAR = SPEED_ENTER_STOP_MS_DEFAULT;
double SPEED_EXIT_STOP_MS_VAR = SPEED_EXIT_STOP_MS_DEFAULT;
double OFFSET_M_VAR = OFFSET_M_DEFAULT;
double DECLINATION_DEG_VAR = DECLINATION_DEG_DEFAULT;

// DIAGNÓSTICO BLE independiente del periodo de salida GGA
uint32_t diagnosticPeriodMs = 5000;
uint32_t lastDiagSendMs = 0;

// ============================================================================
// COM2 - UART por software (solo TX) - Renesas RA4M1
// ============================================================================

static uint32_t com2BitDurationUs = 0;

void softSerialInit() {
  com2BitDurationUs = (uint32_t)(1000000UL / COM2_BAUD);

  pinMode(COM2_TX_PIN, OUTPUT);
  digitalWrite(COM2_TX_PIN, HIGH);
}

static inline void waitUntil(uint32_t targetUs) {
  while ((int32_t)(micros() - targetUs) < 0) {
    // espera activa
  }
}

void softSerialWriteByte(uint8_t value) {
  noInterrupts();

  uint32_t nextBitTime = micros();

  // Start bit
  digitalWrite(COM2_TX_PIN, LOW);
  nextBitTime += com2BitDurationUs;
  waitUntil(nextBitTime);

  // 8 bits
  for (uint8_t bitIndex = 0; bitIndex < 8; bitIndex++) {
    digitalWrite(COM2_TX_PIN, (value & 0x01) ? HIGH : LOW);
    value >>= 1;
    nextBitTime += com2BitDurationUs;
    waitUntil(nextBitTime);
  }

  // Stop bit
  digitalWrite(COM2_TX_PIN, HIGH);
  nextBitTime += com2BitDurationUs;
  waitUntil(nextBitTime);

  interrupts();
}

void softSerialWriteString(const char* text, size_t length) {
  if (text == nullptr) {
    return;
  }

  for (size_t i = 0; i < length; i++) {
    softSerialWriteByte((uint8_t)text[i]);
  }
}

// ============================================================================
// OBJETOS
// ============================================================================

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// ============================================================================
// ESTADOS
// ============================================================================

enum MovementState {
  MOVING = 0,
  AVERAGING = 1,
  LOCKED = 2
};

enum SolutionType {
  SOL_UNKNOWN = 0,
  SOL_GPS = 1,
  SOL_DGPS = 2,
  SOL_RTK = 3,
  SOL_HAS = 4
};

MovementState movementState = MOVING;
SolutionType solutionType = SOL_UNKNOWN;
bool hasActive = false;

// ============================================================================
// VARIABLES GNSS
// ============================================================================

bool gnssValid = false;

double currentLat = 0.0;
double currentLon = 0.0;
double currentAlt = 0.0;
double currentSpeedMS = 0.0;
double currentCourse = 0.0;

int satelliteCount = 0;

char utcTime[16] = "000000.00";
char hdopText[16] = "1.0";

uint32_t lastGgaMs = 0;
uint32_t lastRmcMs = 0;
uint32_t lastPubxMs = 0;

// ============================================================================
// VARIABLES BNO085
// ============================================================================

bool bnoAvailable = false;
double currentYaw = NAN;

uint32_t lastBnoDataMs = 0;
uint32_t lastBnoRetryMs = 0;
uint32_t lastBnoMessageMs = 0;

// ============================================================================
// VARIABLES DE MOVIMIENTO
// ============================================================================

uint32_t lastStopCheckMs = 0;
uint32_t averagingStartMs = 0;
uint32_t lastSampledGgaMs = 0;
uint32_t lastOutputMs = 0;

double latitudeSamples[MAX_SAMPLES];
double longitudeSamples[MAX_SAMPLES];
double altitudeSamples[MAX_SAMPLES];
double yawSamples[MAX_SAMPLES];

int sampleCount = 0;
int yawSampleCount = 0;

double rawLockedLat = 0.0;
double rawLockedLon = 0.0;

bool lockedValid = false;
double lockedLat = 0.0;
double lockedLon = 0.0;
double lockedAlt = 0.0;
double lockedYaw = NAN;

// ============================================================================
// BUFFER GNSS
// ============================================================================

char gnssLine[240];
int gnssLineIndex = 0;

// ============================================================================
// DIAGNÓSTICO
// ============================================================================

void debugPrintf(const char* format, ...) {
  char buffer[240];
  va_list arguments;
  va_start(arguments, format);
  vsnprintf(buffer, sizeof(buffer), format, arguments);
  va_end(arguments);

  Serial.print(buffer);
}

// ============================================================================
// MATEMÁTICAS
// ============================================================================

double degreesToRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

double radiansToDegrees(double radians) {
  return radians * 180.0 / M_PI;
}

double normalizeAngle(double angle) {
  while (angle < 0.0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  double deltaLat = degreesToRadians(lat2 - lat1);
  double deltaLon = degreesToRadians(lon2 - lon1);

  double a =
    sin(deltaLat / 2.0) * sin(deltaLat / 2.0) +
    cos(degreesToRadians(lat1)) *
    cos(degreesToRadians(lat2)) *
    sin(deltaLon / 2.0) *
    sin(deltaLon / 2.0);

  if (a > 1.0) a = 1.0;

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return EARTH_RADIUS_M * c;
}

double trimmedMean(double* values, int count) {
  if (count <= 0 || count > MAX_SAMPLES) return NAN;

  static double sortedValues[MAX_SAMPLES];
  memcpy(sortedValues, values, count * sizeof(double));

  for (int i = 0; i < count - 1; i++) {
    for (int j = 0; j < count - i - 1; j++) {
      if (sortedValues[j] > sortedValues[j + 1]) {
        double temp = sortedValues[j];
        sortedValues[j] = sortedValues[j + 1];
        sortedValues[j + 1] = temp;
      }
    }
  }

  int trimCount = (int)ceil(count * 0.05);
  int first = 0;
  int last = count;

  if (trimCount * 2 < count) {
    first = trimCount;
    last = count - trimCount;
  }

  double sum = 0.0;
  int validCount = 0;

  for (int i = first; i < last; i++) {
    if (!isnan(sortedValues[i])) {
      sum += sortedValues[i];
      validCount++;
    }
  }

  if (validCount == 0) return NAN;
  return sum / validCount;
}

double circularMeanDegrees(double* values, int count) {
  if (count <= 0) return NAN;

  double sineSum = 0.0;
  double cosineSum = 0.0;

  for (int i = 0; i < count; i++) {
    double radians = degreesToRadians(values[i]);
    sineSum += sin(radians);
    cosineSum += cos(radians);
  }

  double meanRadians = atan2(sineSum / count, cosineSum / count);
  return normalizeAngle(radiansToDegrees(meanRadians));
}

bool parseDoubleField(const char* text, double* value) {
  if (text == nullptr || value == nullptr || text[0] == '\0') return false;

  char* endPointer = nullptr;
  double parsedValue = strtod(text, &endPointer);

  if (endPointer == text || *endPointer != '\0' || !isfinite(parsedValue)) return false;

  *value = parsedValue;
  return true;
}

// ============================================================================
// CHECKSUM NMEA
// ============================================================================

uint8_t calculateNmeaChecksum(const char* sentence) {
  uint8_t checksum = 0;
  if (sentence == nullptr) return checksum;

  const char* pointer = sentence;

  while (*pointer != '\0') {
    if (*pointer == '$' || *pointer == '#') {
      pointer++;
      continue;
    }
    if (*pointer == '*') break;

    checksum ^= (uint8_t)(*pointer);
    pointer++;
  }

  return checksum;
}

bool validateNmeaChecksum(const char* sentence) {
  if (sentence == nullptr) return false;

  const char* asterisk = strchr(sentence, '*');
  if (asterisk == nullptr || strlen(asterisk + 1) < 2) return false;

  char checksumText[3];
  checksumText[0] = asterisk[1];
  checksumText[1] = asterisk[2];
  checksumText[2] = '\0';

  char* endPointer = nullptr;
  unsigned long receivedChecksum = strtoul(checksumText, &endPointer, 16);

  if (endPointer == checksumText || *endPointer != '\0' || receivedChecksum > 0xFF) return false;

  uint8_t calculatedChecksum = calculateNmeaChecksum(sentence);
  return calculatedChecksum == (uint8_t)receivedChecksum;
}

// ============================================================================
// CONVERSIÓN NMEA
// ============================================================================

bool parseLatitude(const char* latitudeField, const char* hemisphereField, double* latitude) {
  if (latitudeField == nullptr || hemisphereField == nullptr || latitude == nullptr || strlen(latitudeField) < 7) {
    return false;
  }

  int degrees = (latitudeField[0] - '0') * 10 + (latitudeField[1] - '0');
  double minutes = strtod(latitudeField + 2, nullptr);

  if (!isfinite(minutes)) return false;

  double result = degrees + minutes / 60.0;
  if (hemisphereField[0] == 'S') result = -result;

  *latitude = result;
  return true;
}

bool parseLongitude(const char* longitudeField, const char* hemisphereField, double* longitude) {
  if (longitudeField == nullptr || hemisphereField == nullptr || longitude == nullptr || strlen(longitudeField) < 8) {
    return false;
  }

  int degrees = (longitudeField[0] - '0') * 100 + (longitudeField[1] - '0') * 10 + (longitudeField[2] - '0');
  double minutes = strtod(longitudeField + 3, nullptr);

  if (!isfinite(minutes)) return false;

  double result = degrees + minutes / 60.0;
  if (hemisphereField[0] == 'W') result = -result;

  *longitude = result;
  return true;
}

// ============================================================================
// PARSER GGA
// ============================================================================

bool parseGGA(const char* line) {
  if (line == nullptr) return false;
  if (!validateNmeaChecksum(line)) {
    Serial.println("[GNSS] Error de checksum GGA");
    return false;
  }

  char copy[240];
  strncpy(copy, line, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char* fields[16];
  int fieldCount = 0;

  fields[fieldCount++] = copy;

  for (char* pointer = copy; *pointer != '\0'; pointer++) {
    if (*pointer == ',') {
      *pointer = '\0';
      if (fieldCount < 16) fields[fieldCount++] = pointer + 1;
    }
  }

  if (fieldCount < 10) return false;

  int fixQuality = atoi(fields[6]);
  if (fixQuality < 1) {
    gnssValid = false;
    return false;
  }

  double latitude = 0.0;
  double longitude = 0.0;

  if (!parseLatitude(fields[2], fields[3], &latitude)) return false;
  if (!parseLongitude(fields[4], fields[5], &longitude)) return false;

  double altitude = strtod(fields[9], nullptr);
  if (!isfinite(altitude)) return false;

  strncpy(utcTime, fields[1], sizeof(utcTime) - 1);
  utcTime[sizeof(utcTime) - 1] = '\0';

  satelliteCount = atoi(fields[7]);

  double hdop = 1.0;
  if (!parseDoubleField(fields[8], &hdop) || hdop < 0.0) hdop = 1.0;

  snprintf(hdopText, sizeof(hdopText), "%.1f", hdop);

  currentLat = latitude;
  currentLon = longitude;
  currentAlt = altitude;

  gnssValid = true;
  lastGgaMs = millis();

  debugPrintf("[GNSS] GGA lat=%.6f lon=%.6f alt=%.1f HDOP=%s SAT=%d\n",
              currentLat, currentLon, currentAlt, hdopText, satelliteCount);

  return true;
}

// ============================================================================
// PARSER RMC
// ============================================================================

bool parseRMC(const char* line) {
  if (line == nullptr) return false;
  if (!validateNmeaChecksum(line)) {
    Serial.println("[GNSS] Error de checksum RMC");
    return false;
  }

  char copy[240];
  strncpy(copy, line, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char* fields[14];
  int fieldCount = 0;

  fields[fieldCount++] = copy;

  for (char* pointer = copy; *pointer != '\0'; pointer++) {
    if (*pointer == ',') {
      *pointer = '\0';
      if (fieldCount < 14) fields[fieldCount++] = pointer + 1;
    }
  }

  if (fieldCount < 9) return false;
  if (fields[2][0] != 'A') return false;

  double speedKnots = strtod(fields[7], nullptr);
  double course = strtod(fields[8], nullptr);

  if (!isfinite(speedKnots)) return false;

  currentSpeedMS = speedKnots * 0.51444;
  currentCourse = isfinite(course) ? course : 0.0;

  lastRmcMs = millis();

  debugPrintf("[GNSS] RMC speed=%.2f m/s COG=%.1f\n", currentSpeedMS, currentCourse);
  return true;
}

// ============================================================================
// PARSER PUBX,00 - Detectar HAS activo
// ============================================================================

bool parsePUBX00(const char* line) {
  if (line == nullptr) return false;
  if (strncmp(line, "$PUBX,00", 8) != 0) return false;
  if (!validateNmeaChecksum(line)) {
    Serial.println("[GNSS] Error de checksum PUBX,00");
    return false;
  }

  char copy[240];
  strncpy(copy, line, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char* fields[30];
  int fieldCount = 0;

  fields[fieldCount++] = copy;

  for (char* pointer = copy; *pointer != '\0'; pointer++) {
    if (*pointer == ',') {
      *pointer = '\0';
      if (fieldCount < 30) fields[fieldCount++] = pointer + 1;
    }
  }

  if (fieldCount < 27) return false;

  const char* navStat = fields[26];

  if (strstr(navStat, "HP") != nullptr || strstr(navStat, "HAS") != nullptr) {
    hasActive = true;
    solutionType = SOL_HAS;
    debugPrintf("[GNSS] HAS ACTIVO - Precisión mejorada\n");
  } else if (strstr(navStat, "R3") != nullptr || strstr(navStat, "RTK") != nullptr) {
    hasActive = false;
    solutionType = SOL_RTK;
    debugPrintf("[GNSS] RTK detectado\n");
  } else if (strstr(navStat, "G3") != nullptr || strstr(navStat, "G2") != nullptr) {
    hasActive = false;
    solutionType = SOL_GPS;
    debugPrintf("[GNSS] GPS+Galileo estándar\n");
  } else {
    hasActive = false;
    solutionType = SOL_UNKNOWN;
  }

  lastPubxMs = millis();
  return true;
}

// ============================================================================
// RECEPCIÓN GNSS
// ============================================================================

void processGnssLine(const char* line) {
  if (line == nullptr || line[0] == '\0') return;

  if (strncmp(line, "$GPGGA", 6) == 0 ||
      strncmp(line, "$GNGGA", 6) == 0 ||
      strncmp(line, "$GCGGA", 6) == 0) {
    parseGGA(line);
  } else if (strncmp(line, "$GPRMC", 6) == 0 ||
             strncmp(line, "$GNRMC", 6) == 0 ||
             strncmp(line, "$GCRMC", 6) == 0) {
    parseRMC(line);
  } else if (strncmp(line, "$PUBX,00", 8) == 0) {
    parsePUBX00(line);
  }
}

void readGNSS() {
  while (Serial1.available() > 0) {
    char character = (char)Serial1.read();

    if (character == '$' || character == '#') {
      gnssLineIndex = 0;
      gnssLine[gnssLineIndex++] = character;
      continue;
    }

    if (gnssLineIndex <= 0) continue;

    if (character == '\r') continue;

    if (character == '\n') {
      gnssLine[gnssLineIndex] = '\0';
      processGnssLine(gnssLine);
      gnssLineIndex = 0;
      continue;
    }

    if (gnssLineIndex < (int)sizeof(gnssLine) - 1) {
      gnssLine[gnssLineIndex++] = character;
    } else {
      Serial.println("[GNSS] Línea demasiado larga");
      gnssLineIndex = 0;
    }
  }
}

// ============================================================================
// BNO085
// ============================================================================

bool initializeBNO085() {
  if (!bno08x.begin_I2C(BNO085_ADDRESS, &Wire)) {
    Serial.println("[IMU] No se pudo inicializar BNO085");
    bnoAvailable = false;
    return false;
  }

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("[IMU] No se pudo activar rotation vector");
    bnoAvailable = false;
    return false;
  }

  bnoAvailable = true;
  lastBnoDataMs = millis();
  Serial.println("[IMU] BNO085 inicializado");
  return true;
}

double readYaw() {
  uint32_t now = millis();

  if (!bnoAvailable) {
    if (now - lastBnoRetryMs >= IMU_RETRY_MS) {
      lastBnoRetryMs = now;
      Serial.println("[IMU] Reintentando BNO085");
      initializeBNO085();
    }
    return NAN;
  }

  if (bno08x.wasReset()) {
    Serial.println("[IMU] Reset del BNO085 detectado");
    bnoAvailable = false;
    currentYaw = NAN;
    initializeBNO085();
    return currentYaw;
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float i = sensorValue.un.rotationVector.i;
      float j = sensorValue.un.rotationVector.j;
      float k = sensorValue.un.rotationVector.k;
      float real = sensorValue.un.rotationVector.real;

      double yawRadians = atan2(2.0 * (real * k + i * j),
                                1.0 - 2.0 * (j * j + k * k));
      double magneticYaw = radiansToDegrees(yawRadians);

      currentYaw = normalizeAngle(magneticYaw - DECLINATION_DEG_VAR);
      lastBnoDataMs = now;
      return currentYaw;
    }
  }

  if (now - lastBnoDataMs >= IMU_TIMEOUT_MS) {
    if (now - lastBnoMessageMs >= IMU_RETRY_MS) {
      lastBnoMessageMs = now;
      Serial.println("[IMU] Timeout de yaw");
    }
    currentYaw = NAN;
  }

  return currentYaw;
}

// ============================================================================
// OFFSET ANTENA-PISTÓN
// ============================================================================

void applyAntennaOffset(double rawLat, double rawLon, double yaw,
                       double* correctedLat, double* correctedLon) {
  *correctedLat = rawLat;
  *correctedLon = rawLon;

  if (isnan(yaw)) return;

  double bearing = normalizeAngle(yaw + 270.0);
  double bearingRadians = degreesToRadians(bearing);

  double offsetRadians = OFFSET_M_VAR / EARTH_RADIUS_M;

  double latitudeOffset = offsetRadians * cos(bearingRadians);

  double latitudeCosine = cos(degreesToRadians(rawLat));
  if (fabs(latitudeCosine) < 0.000001) return;

  double longitudeOffset = offsetRadians * sin(bearingRadians) / latitudeCosine;

  *correctedLat = rawLat + latitudeOffset;
  *correctedLon = rawLon + longitudeOffset;
}

// ============================================================================
// PRUEBA DE ARRANQUE DE LEDS
// ============================================================================

void startupLedTest() {
  uint32_t startMs = millis();
  uint32_t lastPrintMs = 0;

  Serial.println("[LED] Prueba de arranque durante 5 segundos");

  while (millis() - startMs < STARTUP_LED_TEST_MS) {
    uint32_t now = millis();
    uint32_t elapsedMs = now - startMs;

    bool state = ((elapsedMs % STARTUP_LED_PERIOD_MS) < (STARTUP_LED_PERIOD_MS / 2));

    digitalWrite(LED1_PIN, state ? HIGH : LOW);
    digitalWrite(LED2_PIN, state ? HIGH : LOW);

    if (now - lastPrintMs >= 500) {
      lastPrintMs = now;
      debugPrintf("[LED] Startup: %lu ms - %s\n", (unsigned long)elapsedMs, state ? "ON" : "OFF");
    }

    delay(10);
  }

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  Serial.println("[LED] Fin de prueba de arranque");
}

// ============================================================================
// ESTADO NORMAL DE LEDS
// ============================================================================

void updateLeds() {
  uint32_t now = millis();

  if (!gnssValid) {
    digitalWrite(LED1_PIN, LOW);
  } else if (!bnoAvailable) {
    digitalWrite(LED1_PIN, ((now % 200) < 100) ? HIGH : LOW);
  } else if (!hasActive) {
    digitalWrite(LED1_PIN, ((now % 600) < 300) ? HIGH : LOW);
  } else if (movementState == LOCKED && lockedValid) {
    digitalWrite(LED1_PIN, HIGH);
  } else {
    digitalWrite(LED1_PIN, ((now % 1000) < 500) ? HIGH : LOW);
  }

  if (movementState == MOVING) {
    digitalWrite(LED2_PIN, LOW);
  } else if (movementState == AVERAGING) {
    digitalWrite(LED2_PIN, ((now % 400) < 200) ? HIGH : LOW);
  } else if (movementState == LOCKED && lockedValid) {
    digitalWrite(LED2_PIN, HIGH);
  } else {
    digitalWrite(LED2_PIN, LOW);
  }
}

// ============================================================================
// MÁQUINA DE ESTADOS
// ============================================================================

void clearAverageBuffers() {
  sampleCount = 0;
  yawSampleCount = 0;
  lastSampledGgaMs = 0;

  memset(latitudeSamples, 0, sizeof(latitudeSamples));
  memset(longitudeSamples, 0, sizeof(longitudeSamples));
  memset(altitudeSamples, 0, sizeof(altitudeSamples));
  memset(yawSamples, 0, sizeof(yawSamples));
}

int getOutputFixQuality() {
  if (!gnssValid) return 0;
  if (movementState == LOCKED && lockedValid) return 4;
  if (hasActive) return 2;
  return 1;
}

void updateMovementState() {
  uint32_t now = millis();

  bool speedFresh = (now - lastRmcMs) <= SPEED_FRESHNESS_MS;
  bool ggaFresh = (now - lastGgaMs) <= GGA_FRESHNESS_MS;

  if (!speedFresh) currentSpeedMS = 0.0;
  if (!ggaFresh) {
    gnssValid = false;
    lockedValid = false;
  }

  switch (movementState) {
    case MOVING: {
      if (speedFresh && currentSpeedMS < SPEED_ENTER_STOP_MS_VAR) {
        if (lastStopCheckMs == 0) lastStopCheckMs = now;
      } else if (speedFresh && currentSpeedMS >= SPEED_EXIT_STOP_MS_VAR) {
        lastStopCheckMs = 0;
      }

      if (lastStopCheckMs > 0 && now - lastStopCheckMs >= STOP_CONFIRMATION_MS) {
        movementState = AVERAGING;
        averagingStartMs = now;
        lockedValid = false;
        clearAverageBuffers();
        Serial.println("[STATE] MOVING -> AVERAGING");
      }
      break;
    }

    case AVERAGING: {
      if (gnssValid && lastGgaMs != lastSampledGgaMs && sampleCount < MAX_SAMPLES) {
        latitudeSamples[sampleCount] = currentLat;
        longitudeSamples[sampleCount] = currentLon;
        altitudeSamples[sampleCount] = currentAlt;
        sampleCount++;
        lastSampledGgaMs = lastGgaMs;

        if (bnoAvailable && !isnan(currentYaw) && yawSampleCount < MAX_SAMPLES) {
          yawSamples[yawSampleCount++] = currentYaw;
        }
      }

      if (speedFresh && currentSpeedMS > SPEED_EXIT_STOP_MS_VAR) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;
        clearAverageBuffers();
        Serial.println("[STATE] AVERAGING -> MOVING");
        break;
      }

      if (now - averagingStartMs >= AVERAGING_WINDOW_MS_VAR && sampleCount > 0) {
        double averageRawLat = trimmedMean(latitudeSamples, sampleCount);
        double averageRawLon = trimmedMean(longitudeSamples, sampleCount);
        double averageAlt = trimmedMean(altitudeSamples, sampleCount);
        double averageYaw = circularMeanDegrees(yawSamples, yawSampleCount);

        if (isnan(averageRawLat) || isnan(averageRawLon) || isnan(averageAlt)) {
          movementState = MOVING;
          lockedValid = false;
          clearAverageBuffers();
          Serial.println("[STATE] Error en promedio");
          break;
        }

        rawLockedLat = averageRawLat;
        rawLockedLon = averageRawLon;

        lockedAlt = averageAlt;
        lockedYaw = averageYaw;

        applyAntennaOffset(rawLockedLat, rawLockedLon, lockedYaw, &lockedLat, &lockedLon);

        lockedValid = true;
        movementState = LOCKED;
        lastStopCheckMs = 0;

        debugPrintf("[STATE] AVERAGING -> LOCKED lat=%.6f lon=%.6f yaw=%.1f\n",
                    lockedLat, lockedLon, lockedYaw);
      }
      break;
    }

    case LOCKED: {
      if (speedFresh && currentSpeedMS > SPEED_EXIT_STOP_MS_VAR) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;
        clearAverageBuffers();
        Serial.println("[STATE] LOCKED -> MOVING por velocidad");
        break;
      }

      if (gnssValid) {
        double distance = haversineMeters(currentLat, currentLon, rawLockedLat, rawLockedLon);

        if (distance > RELOCK_DISTANCE_M) {
          movementState = MOVING;
          lockedValid = false;
          lastStopCheckMs = 0;
          clearAverageBuffers();
          debugPrintf("[STATE] LOCKED -> MOVING dist=%.2f m\n", distance);
        }
      }
      break;
    }
  }
}

// ============================================================================
// FORMATO NMEA
// ============================================================================

void formatLatitude(double latitude, char* output, size_t outputSize, char* hemisphere) {
  double absoluteValue = fabs(latitude);
  int degrees = (int)absoluteValue;
  double minutes = (absoluteValue - degrees) * 60.0;

  *hemisphere = latitude >= 0.0 ? 'N' : 'S';

  char minutesText[24];
  dtostrf(minutes, 0, 4, minutesText);

  char* firstCharacter = minutesText;
  while (*firstCharacter == ' ') firstCharacter++;

  snprintf(output, outputSize, "%02d%s", degrees, firstCharacter);
}

void formatLongitude(double longitude, char* output, size_t outputSize, char* hemisphere) {
  double absoluteValue = fabs(longitude);
  int degrees = (int)absoluteValue;
  double minutes = (absoluteValue - degrees) * 60.0;

  *hemisphere = longitude >= 0.0 ? 'E' : 'W';

  char minutesText[24];
  dtostrf(minutes, 0, 4, minutesText);

  char* firstCharacter = minutesText;
  while (*firstCharacter == ' ') firstCharacter++;

  snprintf(output, outputSize, "%03d%s", degrees, firstCharacter);
}

void createGCGGA(char* output, size_t outputSize, double latitude, double longitude,
                 double altitude, int fixQuality) {
  char latitudeText[24];
  char longitudeText[24];
  char altitudeText[24];
  char latitudeHemisphere;
  char longitudeHemisphere;

  formatLatitude(latitude, latitudeText, sizeof(latitudeText), &latitudeHemisphere);
  formatLongitude(longitude, longitudeText, sizeof(longitudeText), &longitudeHemisphere);

  dtostrf(altitude, 0, 1, altitudeText);

  char* firstAltitudeCharacter = altitudeText;
  while (*firstAltitudeCharacter == ' ') firstAltitudeCharacter++;

  snprintf(output, outputSize,
           "$GCGGA,%s,%s,%c,%s,%c,%d,%02d,%s,%s,M,0.0,M,,",
           utcTime,
           latitudeText,
           latitudeHemisphere,
           longitudeText,
           longitudeHemisphere,
           fixQuality,
           satelliteCount,
           hdopText,
           firstAltitudeCharacter);
}

void addNmeaChecksumAndCrlf(char* sentence, size_t sentenceSize) {
  if (sentence == nullptr) return;

  uint8_t checksum = calculateNmeaChecksum(sentence);
  size_t length = strlen(sentence);

  if (length + 7 >= sentenceSize) return;

  snprintf(sentence + length, sentenceSize - length, "*%02X\r\n", checksum);
}

// ============================================================================
// ETHERNET
// ============================================================================

bool initializeEthernet() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  Ethernet.init(ETHERNET_CS_PIN);
  Serial.println("[ETH] Inicializando Ethernet");

  int dhcpResult = Ethernet.begin(ethernetMac);
  if (dhcpResult == 0) {
    Serial.println("[ETH] DHCP no disponible");
    ethernetReady = false;
    return false;
  }

  delay(1000);
  Serial.print("[ETH] IP local: ");
  Serial.println(Ethernet.localIP());
  ethernetReady = true;
  return true;
}

void maintainEthernet() {
  uint32_t now = millis();

  if (now - lastEthernetCheckMs < 5000) return;
  lastEthernetCheckMs = now;

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    ethernetReady = false;
    Serial.println("[ETH] Shield no detectado");
    return;
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    ethernetReady = false;
    Serial.println("[ETH] Cable Ethernet desconectado");
    return;
  }

  ethernetReady = true;
}

void transmitEthernet(const char* sentence) {
  if (!ethernetReady || sentence == nullptr) return;

  if (!ethernetClient.connected()) {
    ethernetClient.stop();
    if (!ethernetClient.connect(ethernetTargetIP, ethernetTargetPort)) {
      return;
    }
    Serial.println("[ETH] Conexión TCP establecida");
  }

  ethernetClient.write((const uint8_t*)sentence, strlen(sentence));
}

// ============================================================================
// BLE - Rev.3.0
// ============================================================================

bool initializeBLE() {
  Serial.println("[BLE] Inicializando");

  if (!BLE.begin()) {
    Serial.println("[BLE] No disponible");
    return false;
  }

  BLE.setLocalName(BLE_DEVICE_NAME);
  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setAdvertisedService(diagnosticService);
  diagnosticService.addCharacteristic(commandCharacteristic);
  diagnosticService.addCharacteristic(responseCharacteristic);
  diagnosticService.addCharacteristic(diagnosticCharacteristic);
  BLE.addService(diagnosticService);

  responseCharacteristic.writeValue("READY");
  diagnosticCharacteristic.writeValue("GNSS=FAIL;LED1=OFF;LED2=OFF");
  BLE.advertise();

  Serial.println("[BLE] Anunciando como FWD-GPS");
  return true;
}

void sendBleResponse(const char* text) {
  if (text == nullptr) return;

  Serial.print("[BLE RESP] ");
  Serial.println(text);

  if (bleReady && BLE.connected()) {
    responseCharacteristic.writeValue(text);
  }
}

void handleBLE() {
  if (!bleReady) return;

  BLE.poll();

  bool connected = BLE.connected();
  if (connected && !bleWasConnected) {
    Serial.println("[BLE] Central conectado");
  } else if (!connected && bleWasConnected) {
    Serial.println("[BLE] Central desconectado; reanudando anuncio");
    BLE.advertise();
  }
  bleWasConnected = connected;

  if (!connected || !commandCharacteristic.written()) return;

  String command = commandCharacteristic.value();
  command.trim();
  if (command.length() == 0) return;

  char commandBuffer[BLE_PAYLOAD_MAX + 1];
  command.toCharArray(commandBuffer, sizeof(commandBuffer));
  parseCommand(commandBuffer);
}

void parseCommand(const char* line) {
  if (line == nullptr || line[0] == '\0') return;

  char command[64];
  char arg[64];

  int parsed = sscanf(line, "%63s %63s", command, arg);

  if (parsed < 1) return;

  for (int i = 0; command[i]; i++) {
    command[i] = tolower(command[i]);
  }

  // freq: cambia salida GGA/FWD
  if (strcmp(command, "freq") == 0 && parsed >= 2) {
    int freq = atoi(arg);

    if (freq < 1 || freq > 10) {
      sendBleResponse("ERROR: freq debe ser 1-10 Hz");
      return;
    }

    OUTPUT_PERIOD_MS_VAR = 1000 / freq;

    char msg[128];
    snprintf(msg, sizeof(msg), "OK: salida GGA cambiada a %d Hz (%d ms)", freq, OUTPUT_PERIOD_MS_VAR);
    sendBleResponse(msg);
  }

  // speed_stop
  else if (strcmp(command, "speed_stop") == 0 && parsed >= 2) {
    double speed = strtod(arg, nullptr);

    if (speed < 0.0 || speed > 1.0) {
      sendBleResponse("ERROR: speed_stop debe ser 0.0-1.0 m/s");
      return;
    }

    SPEED_ENTER_STOP_MS_VAR = speed;

    char msg[128];
    snprintf(msg, sizeof(msg), "OK: Umbral parada = %.2f m/s", speed);
    sendBleResponse(msg);
  }

  // speed_move
  else if (strcmp(command, "speed_move") == 0 && parsed >= 2) {
    double speed = strtod(arg, nullptr);

    if (speed < 0.0 || speed > 1.0) {
      sendBleResponse("ERROR: speed_move debe ser 0.0-1.0 m/s");
      return;
    }

    SPEED_EXIT_STOP_MS_VAR = speed;

    char msg[128];
    snprintf(msg, sizeof(msg), "OK: Umbral movimiento = %.2f m/s", speed);
    sendBleResponse(msg);
  }

  // offset
  else if (strcmp(command, "offset") == 0 && parsed >= 2) {
    double offset = strtod(arg, nullptr);

    if (offset < 0.0 || offset > 2.0) {
      sendBleResponse("ERROR: offset debe ser 0.0-2.0 m");
      return;
    }

    OFFSET_M_VAR = offset;

    char msg[128];
    snprintf(msg, sizeof(msg), "OK: Offset antena = %.2f m", offset);
    sendBleResponse(msg);
  }

  // decl
  else if (strcmp(command, "decl") == 0 && parsed >= 2) {
    double decl = strtod(arg, nullptr);

    if (decl < -180.0 || decl > 180.0) {
      sendBleResponse("ERROR: decl debe ser -180 a +180 grados");
      return;
    }

    DECLINATION_DEG_VAR = decl;

    char msg[128];
    snprintf(msg, sizeof(msg), "OK: Declinacion = %.1f grados", decl);
    sendBleResponse(msg);
  }

  // avg
  else if (strcmp(command, "avg") == 0 && parsed >= 2) {
    int window = atoi(arg);

    if (window < 5 || window > 60) {
      sendBleResponse("ERROR: avg debe ser 5-60 segundos");
      return;
    }

    AVERAGING_WINDOW_MS_VAR = window * 1000;

    char msg[128];
    snprintf(msg, sizeof(msg), "OK: Ventana promedio = %d segundos", window);
    sendBleResponse(msg);
  }

  // diag: cambia intervalo de diagnóstico BLE
  else if (strcmp(command, "diag") == 0 && parsed >= 2) {
    int seconds = atoi(arg);

    if (seconds < 1 || seconds > 60) {
      sendBleResponse("ERROR: diag debe ser 1-60 segundos");
      return;
    }

    diagnosticPeriodMs = (uint32_t)seconds * 1000UL;

    char msg[128];
    snprintf(msg, sizeof(msg), "OK: diagnostico cada %d segundos", seconds);
    sendBleResponse(msg);
  }

  // status
  else if (strcmp(command, "status") == 0) {
    char status[BLE_PAYLOAD_MAX + 1];

    snprintf(status, sizeof(status),
             "FREQ=%d;PERIOD=%d;DIAG=%lu;STOP=%.2f;MOVE=%.2f;"
             "OFFSET=%.2f;DECL=%.1f;AVG=%.0f;GNSS=%s;IMU=%s;"
             "HAS=%s;STATE=%s;LOCK=%s",
             1000 / OUTPUT_PERIOD_MS_VAR,
             OUTPUT_PERIOD_MS_VAR,
             diagnosticPeriodMs / 1000UL,
             SPEED_ENTER_STOP_MS_VAR,
             SPEED_EXIT_STOP_MS_VAR,
             OFFSET_M_VAR,
             DECLINATION_DEG_VAR,
             AVERAGING_WINDOW_MS_VAR / 1000.0,
             gnssValid ? "OK" : "FAIL",
             bnoAvailable ? "OK" : "FAIL",
             hasActive ? "ON" : "OFF",
             movementState == MOVING ? "MOVING" :
             movementState == AVERAGING ? "AVERAGING" : "LOCKED",
             lockedValid ? "YES" : "NO");

    sendBleResponse(status);
  }

  // help
  else if (strcmp(command, "help") == 0) {
    const char* help =
      "freq <1-10>;speed_stop <0-1>;speed_move <0-1>;"
      "offset <0-2>;decl <-180-180>;avg <5-60>;diag <1-60>;"
      "status;help";

    sendBleResponse(help);
  }

  else {
    sendBleResponse("ERROR: Comando desconocido. Escribe 'help'");
  }
}

// ============================================================================
// DIAGNÓSTICO BLE INDEPENDIENTE
// ============================================================================

void sendDiagnosticBLE() {
  uint32_t now = millis();

  if (now - lastDiagSendMs < diagnosticPeriodMs) return;
  lastDiagSendMs = now;

  if (!bleReady || !BLE.connected()) return;

  const char* movementText =
    movementState == MOVING ? "MOVING" :
    movementState == AVERAGING ? "AVERAGING" :
    "LOCKED";

  const char* solutionText =
    solutionType == SOL_HAS ? "HAS" :
    solutionType == SOL_RTK ? "RTK" :
    solutionType == SOL_GPS ? "GPS" :
    solutionType == SOL_DGPS ? "DGPS" :
    "UNKNOWN";

  const char* led1Text =
    !gnssValid ? "OFF" :
    !bnoAvailable ? "BLINK_RED" :
    !hasActive ? "BLINK_YELLOW" :
    movementState == LOCKED && lockedValid ? "ON/GREEN" :
    "BLINK_GREEN";

  const char* led2Text =
    movementState == MOVING ? "OFF" :
    movementState == AVERAGING ? "BLINK_YELLOW" :
    movementState == LOCKED && lockedValid ? "ON/GREEN" :
    "OFF";

  char yawText[16];
  if (isnan(currentYaw)) {
    strcpy(yawText, "N/A");
  } else {
    snprintf(yawText, sizeof(yawText), "%.1f", currentYaw);
  }

  char line[BLE_PAYLOAD_MAX + 1];

  int diagnosticLength = snprintf(line, sizeof(line),
           "LED1=%s;LED2=%s;GNSS=%s;FIX=%d;SATS=%d;"
           "HDOP=%s;LAT=%.6f;LON=%.6f;ALT=%.1f;"
           "SPEED=%.2f;COURSE=%.1f;SOLUTION=%s;HAS=%s;IMU=%s;"
           "YAW=%s;STATE=%s;LOCK=%s;N=%d;HZ=%d;CFG=%lu/%.2f/%.1f",
           led1Text,
           led2Text,
           gnssValid ? "OK" : "FAIL",
           getOutputFixQuality(),
           satelliteCount,
           hdopText,
           currentLat,
           currentLon,
           currentAlt,
           currentSpeedMS,
           currentCourse,
           solutionText,
           hasActive ? "ON" : "OFF",
           bnoAvailable ? "OK" : "FAIL",
           yawText,
           movementText,
           lockedValid ? "YES" : "NO",
           sampleCount,
           1000 / OUTPUT_PERIOD_MS_VAR,
           (unsigned long)(AVERAGING_WINDOW_MS_VAR / 1000UL),
           OFFSET_M_VAR,
           DECLINATION_DEG_VAR);

  if (diagnosticLength < 0 || diagnosticLength > BLE_PAYLOAD_MAX) {
    snprintf(line, sizeof(line),
             "LED1=%s;LED2=%s;GNSS=%s;FIX=%d;SATS=%d;"
             "HAS=%s;IMU=%s;STATE=%s;LOCK=%s",
             led1Text,
             led2Text,
             gnssValid ? "OK" : "FAIL",
             getOutputFixQuality(),
             satelliteCount,
             hasActive ? "ON" : "OFF",
             bnoAvailable ? "OK" : "FAIL",
             movementText,
             lockedValid ? "YES" : "NO");
  }

  diagnosticCharacteristic.writeValue(line);
  Serial.print("[BLE DIAG] ");
  Serial.println(line);
}

// ============================================================================
// TRANSMISIÓN GCGGA
// ============================================================================

void transmitGCGGA() {
  uint32_t now = millis();

  if (now - lastOutputMs < OUTPUT_PERIOD_MS_VAR) return;
  lastOutputMs = now;

  int fixQuality = getOutputFixQuality();

  if (!gnssValid || fixQuality < 1) return;

  double outputLat = currentLat;
  double outputLon = currentLon;
  double outputAlt = currentAlt;

  if (movementState == LOCKED && lockedValid) {
    outputLat = lockedLat;
    outputLon = lockedLon;
    outputAlt = lockedAlt;
  } else {
    if (bnoAvailable && !isnan(currentYaw)) {
      applyAntennaOffset(currentLat, currentLon, currentYaw, &outputLat, &outputLon);
    }
  }

  char sentence[240];

  createGCGGA(sentence, sizeof(sentence), outputLat, outputLon, outputAlt, fixQuality);
  addNmeaChecksumAndCrlf(sentence, sizeof(sentence));

  size_t sentenceLength = strlen(sentence);

  // COM2: salida serie por software (solo TX)
  softSerialWriteString(sentence, sentenceLength);

  transmitEthernet(sentence);

  Serial.print("[TX] ");
  Serial.print(sentence);
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("RS232-FMW-GPS Rev.3.0");
  Serial.println("Arduino UNO R4 WiFi");
  Serial.println("==========================================");
  Serial.println("GNSS:     Serial1 D0/D1 @ 115200 (UM980 COM3)");
  Serial.println("COM2:     Emergencia/supervision D2 @ 38400 (soft-UART TX-only)");
  Serial.println("Ethernet: Primario FWM, Shield 2 / W5500");
  Serial.println("Destino:  192.168.1.122:15919");
  Serial.println("BNO085:   TWI/I2C @ 100 kHz");
  Serial.println("LED1:     D5 / OUT5 (GNSS+IMU+HAS)");
  Serial.println("LED2:     D6 / OUT6 (Movimiento)");
  Serial.println("SD:       D4 desactivada");
  Serial.println("==========================================");
  Serial.println("BLE (Rev.3.0):");
  Serial.println("  Nombre: FWD-GPS");
  Serial.println("  Cmds: freq, offset, decl, speed_*, avg, diag, status, help");
  Serial.println("==========================================");

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  // Prueba visible de 5 segundos
  startupLedTest();

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  Serial1.begin(GNSS_BAUD);
  softSerialInit();

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  if (!initializeBNO085()) {
    Serial.println("[SETUP] BNO085 no disponible; se reintentará");
  }

  if (!initializeEthernet()) {
    Serial.println("[SETUP] Ethernet no disponible");
  }

  bleReady = initializeBLE();

  Serial.println("[SETUP] Sistema preparado");
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  maintainEthernet();

  readGNSS();
  readYaw();
  updateMovementState();
  updateLeds();
  transmitGCGGA();

  handleBLE();
  sendDiagnosticBLE();
}