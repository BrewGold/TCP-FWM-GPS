/*
  RS232-FMW-GPS - Rev.2.1
  Arduino UNO R4 WiFi

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
       - Sin BNO085: corrección desactivada (LED1 parpadea muy rápido)
    
    3. ETHERNET: Shield W5500 con IP 192.168.1.122 puerto 15919

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
    Sin RMC, sin otros mensajes (como requiere manual FWD)

  LEDs Rev.2:
    LED1 (D5) - GNSS + IMU + HAS:
      - OFF = Sin GNSS
      - Parpadeo MUY rápido (200ms) = GNSS OK pero SIN magnetómetro
      - Parpadeo MEDIO (600ms) = GNSS + IMU pero SIN HAS
      - Parpadeo LENTO (1000ms) = GNSS + IMU + HAS, esperando bloqueo
      - ON = GNSS + IMU + HAS + Posición bloqueada ✅
    
    LED2 (D6) - Movimiento:
      - OFF = En movimiento
      - Parpadeo RÁPIDO (400ms) = Promediando posición
      - ON = Posición bloqueada

  NOTA IMPORTANTE (Rev.1 - fix):
    SoftwareSerial.h NO es compatible con el core Renesas RA4M1
    del Arduino UNO R4 WiFi. Se implementa COM2 como bit-banging TX-only,
    compensado con temporización absoluta basada en micros() para evitar drift.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Ethernet.h>
#include <WiFiS3.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

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

// Prueba visible al arrancar
#define STARTUP_LED_TEST_MS       5000
#define STARTUP_LED_PERIOD_MS     400

// Ethernet
byte ethernetMac[] = {
  0xDE,
  0xAD,
  0xBE,
  0xEF,
  0xFE,
  0xED
};

IPAddress ethernetTargetIP(192, 168, 1, 122);
const uint16_t ethernetTargetPort = 15919;

EthernetClient ethernetClient;
bool ethernetReady = false;
uint32_t lastEthernetCheckMs = 0;

// Navegación
#define OFFSET_M                   0.55
#define DECLINATION_DEG            1.0
#define EARTH_RADIUS_M             6378137.0

#define STOP_CONFIRMATION_MS       2000
#define SPEED_FRESHNESS_MS         2000
#define GGA_FRESHNESS_MS           2000
#define IMU_RETRY_MS               5000
#define IMU_TIMEOUT_MS             5000

#define OUTPUT_PERIOD_MS           100
#define AVERAGING_WINDOW_MS        15000
#define RELOCK_DISTANCE_M          1.0
#define MAX_SAMPLES                160

#define SPEED_ENTER_STOP_MS        0.20
#define SPEED_EXIT_STOP_MS         0.30

// WiFi AP + TCP diagnóstico
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "FWD-GPS-Diag"
#endif

#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD "12345678"
#endif

#ifndef WIFI_TCP_PORT
#define WIFI_TCP_PORT 15920
#endif

const char wifiSsid[] = WIFI_AP_SSID;
const char wifiPassword[] = WIFI_AP_PASSWORD;
const uint16_t wifiTcpPort = WIFI_TCP_PORT;

WiFiServer diagTcpServer(wifiTcpPort);
WiFiClient diagTcpClient;

// ============================================================================
// COM2 - UART POR SOFTWARE (SOLO TX) - Compensado para RA4M1
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

  // 8 bits de datos, LSB primero
  for (uint8_t bitIndex = 0; bitIndex < 8; bitIndex++) {
    digitalWrite(
      COM2_TX_PIN,
      (value & 0x01) ? HIGH : LOW
    );

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

// Variables dinámicas por TCP (RAM, sin EEPROM)
uint32_t OUTPUT_PERIOD_MS_VAR = OUTPUT_PERIOD_MS;
uint32_t AVERAGING_WINDOW_MS_VAR = AVERAGING_WINDOW_MS;
double SPEED_ENTER_STOP_MS_VAR = SPEED_ENTER_STOP_MS;
double SPEED_EXIT_STOP_MS_VAR = SPEED_EXIT_STOP_MS;
double OFFSET_M_VAR = OFFSET_M;
double DECLINATION_DEG_VAR = DECLINATION_DEG;

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

char tcpLine[120];
int tcpLineIndex = 0;
uint32_t lastTcpDiagMs = 0;
bool wifiApReady = false;
uint32_t lastWiFiRetryMs = 0;
bool diagTcpServerStarted = false;

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

void tcpPrintf(const char* format, ...) {
  char buffer[240];

  va_list arguments;
  va_start(arguments, format);
  vsnprintf(buffer, sizeof(buffer), format, arguments);
  va_end(arguments);

  if (diagTcpClient && diagTcpClient.connected()) {
    diagTcpClient.print(buffer);
  }
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
  while (angle < 0.0) {
    angle += 360.0;
  }

  while (angle >= 360.0) {
    angle -= 360.0;
  }

  return angle;
}

double haversineMeters(
  double lat1,
  double lon1,
  double lat2,
  double lon2
) {
  double deltaLat = degreesToRadians(lat2 - lat1);
  double deltaLon = degreesToRadians(lon2 - lon1);

  double a =
    sin(deltaLat / 2.0) * sin(deltaLat / 2.0) +
    cos(degreesToRadians(lat1)) *
    cos(degreesToRadians(lat2)) *
    sin(deltaLon / 2.0) *
    sin(deltaLon / 2.0);

  if (a > 1.0) {
    a = 1.0;
  }

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return EARTH_RADIUS_M * c;
}

double trimmedMean(double* values, int count) {
  if (count <= 0 || count > MAX_SAMPLES) {
    return NAN;
  }

  static double sortedValues[MAX_SAMPLES];

  memcpy(
    sortedValues,
    values,
    count * sizeof(double)
  );

  for (int i = 0; i < count - 1; i++) {
    for (int j = 0; j < count - i - 1; j++) {
      if (sortedValues[j] > sortedValues[j + 1]) {
        double temporary = sortedValues[j];
        sortedValues[j] = sortedValues[j + 1];
        sortedValues[j + 1] = temporary;
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

  if (validCount == 0) {
    return NAN;
  }

  return sum / validCount;
}

double circularMeanDegrees(double* values, int count) {
  if (count <= 0) {
    return NAN;
  }

  double sineSum = 0.0;
  double cosineSum = 0.0;

  for (int i = 0; i < count; i++) {
    double radians = degreesToRadians(values[i]);

    sineSum += sin(radians);
    cosineSum += cos(radians);
  }

  double meanRadians = atan2(
    sineSum / count,
    cosineSum / count
  );

  return normalizeAngle(
    radiansToDegrees(meanRadians)
  );
}

bool parseDoubleField(
  const char* text,
  double* value
) {
  if (text == nullptr ||
      value == nullptr ||
      text[0] == '\0') {
    return false;
  }

  char* endPointer = nullptr;
  double parsedValue = strtod(text, &endPointer);

  if (endPointer == text ||
      *endPointer != '\0' ||
      !isfinite(parsedValue)) {
    return false;
  }

  *value = parsedValue;

  return true;
}

// ============================================================================
// CHECKSUM NMEA
// ============================================================================

uint8_t calculateNmeaChecksum(const char* sentence) {
  uint8_t checksum = 0;

  if (sentence == nullptr) {
    return checksum;
  }

  const char* pointer = sentence;

  while (*pointer != '\0') {
    if (*pointer == '$' || *pointer == '#') {
      pointer++;
      continue;
    }

    if (*pointer == '*') {
      break;
    }

    checksum ^= (uint8_t)(*pointer);
    pointer++;
  }

  return checksum;
}

bool validateNmeaChecksum(const char* sentence) {
  if (sentence == nullptr) {
    return false;
  }

  const char* asterisk = strchr(sentence, '*');

  if (asterisk == nullptr ||
      strlen(asterisk + 1) < 2) {
    return false;
  }

  char checksumText[3];

  checksumText[0] = asterisk[1];
  checksumText[1] = asterisk[2];
  checksumText[2] = '\0';

  char* endPointer = nullptr;

  unsigned long receivedChecksum = strtoul(
    checksumText,
    &endPointer,
    16
  );

  if (endPointer == checksumText ||
      *endPointer != '\0' ||
      receivedChecksum > 0xFF) {
    return false;
  }

  uint8_t calculatedChecksum =
    calculateNmeaChecksum(sentence);

  return calculatedChecksum ==
         (uint8_t)receivedChecksum;
}

// ============================================================================
// CONVERSIÓN NMEA
// ============================================================================

bool parseLatitude(
  const char* latitudeField,
  const char* hemisphereField,
  double* latitude
) {
  if (latitudeField == nullptr ||
      hemisphereField == nullptr ||
      latitude == nullptr ||
      strlen(latitudeField) < 7) {
    return false;
  }

  int degrees =
    (latitudeField[0] - '0') * 10 +
    (latitudeField[1] - '0');

  double minutes = strtod(
    latitudeField + 2,
    nullptr
  );

  if (!isfinite(minutes)) {
    return false;
  }

  double result =
    degrees + minutes / 60.0;

  if (hemisphereField[0] == 'S') {
    result = -result;
  }

  *latitude = result;

  return true;
}

bool parseLongitude(
  const char* longitudeField,
  const char* hemisphereField,
  double* longitude
) {
  if (longitudeField == nullptr ||
      hemisphereField == nullptr ||
      longitude == nullptr ||
      strlen(longitudeField) < 8) {
    return false;
  }

  int degrees =
    (longitudeField[0] - '0') * 100 +
    (longitudeField[1] - '0') * 10 +
    (longitudeField[2] - '0');

  double minutes = strtod(
    longitudeField + 3,
    nullptr
  );

  if (!isfinite(minutes)) {
    return false;
  }

  double result =
    degrees + minutes / 60.0;

  if (hemisphereField[0] == 'W') {
    result = -result;
  }

  *longitude = result;

  return true;
}

// ============================================================================
// PARSER GGA
// ============================================================================

bool parseGGA(const char* line) {
  if (line == nullptr) {
    return false;
  }

  if (!validateNmeaChecksum(line)) {
    Serial.println("[GNSS] Error de checksum GGA");
    return false;
  }

  char copy[240];

  strncpy(
    copy,
    line,
    sizeof(copy) - 1
  );

  copy[sizeof(copy) - 1] = '\0';

  char* fields[16];
  int fieldCount = 0;

  fields[fieldCount++] = copy;

  for (
    char* pointer = copy;
    *pointer != '\0';
    pointer++
  ) {
    if (*pointer == ',') {
      *pointer = '\0';

      if (fieldCount < 16) {
        fields[fieldCount++] = pointer + 1;
      }
    }
  }

  if (fieldCount < 10) {
    return false;
  }

  int fixQuality = atoi(fields[6]);

  if (fixQuality < 1) {
    gnssValid = false;
    return false;
  }

  double latitude = 0.0;
  double longitude = 0.0;

  if (!parseLatitude(
        fields[2],
        fields[3],
        &latitude
      )) {
    return false;
  }

  if (!parseLongitude(
        fields[4],
        fields[5],
        &longitude
      )) {
    return false;
  }

  double altitude = strtod(
    fields[9],
    nullptr
  );

  if (!isfinite(altitude)) {
    return false;
  }

  strncpy(
    utcTime,
    fields[1],
    sizeof(utcTime) - 1
  );

  utcTime[sizeof(utcTime) - 1] = '\0';

  satelliteCount = atoi(fields[7]);

  double hdop = 1.0;

  if (!parseDoubleField(
        fields[8],
        &hdop
      ) || hdop < 0.0) {
    hdop = 1.0;
  }

  snprintf(
    hdopText,
    sizeof(hdopText),
    "%.1f",
    hdop
  );

  currentLat = latitude;
  currentLon = longitude;
  currentAlt = altitude;

  gnssValid = true;
  lastGgaMs = millis();

  debugPrintf(
    "[GNSS] GGA lat=%.6f lon=%.6f alt=%.1f HDOP=%s SAT=%d\n",
    currentLat,
    currentLon,
    currentAlt,
    hdopText,
    satelliteCount
  );

  return true;
}

// ============================================================================
// PARSER RMC
// ============================================================================

bool parseRMC(const char* line) {
  if (line == nullptr) {
    return false;
  }

  if (!validateNmeaChecksum(line)) {
    Serial.println("[GNSS] Error de checksum RMC");
    return false;
  }

  char copy[240];

  strncpy(
    copy,
    line,
    sizeof(copy) - 1
  );

  copy[sizeof(copy) - 1] = '\0';

  char* fields[14];
  int fieldCount = 0;

  fields[fieldCount++] = copy;

  for (
    char* pointer = copy;
    *pointer != '\0';
    pointer++
  ) {
    if (*pointer == ',') {
      *pointer = '\0';

      if (fieldCount < 14) {
        fields[fieldCount++] = pointer + 1;
      }
    }
  }

  if (fieldCount < 9) {
    return false;
  }

  if (fields[2][0] != 'A') {
    return false;
  }

  double speedKnots = strtod(
    fields[7],
    nullptr
  );

  double course = strtod(
    fields[8],
    nullptr
  );

  if (!isfinite(speedKnots)) {
    return false;
  }

  currentSpeedMS =
    speedKnots * 0.51444;

  currentCourse =
    isfinite(course) ? course : 0.0;

  lastRmcMs = millis();

  debugPrintf(
    "[GNSS] RMC speed=%.2f m/s COG=%.1f\n",
    currentSpeedMS,
    currentCourse
  );

  return true;
}

// ============================================================================
// PARSER PUBX,00 - Detectar HAS activo
// ============================================================================

bool parsePUBX00(const char* line) {
  if (line == nullptr) {
    return false;
  }

  if (strncmp(line, "$PUBX,00", 8) != 0) {
    return false;
  }

  if (!validateNmeaChecksum(line)) {
    Serial.println("[GNSS] Error de checksum PUBX,00");
    return false;
  }

  char copy[240];

  strncpy(
    copy,
    line,
    sizeof(copy) - 1
  );

  copy[sizeof(copy) - 1] = '\0';

  char* fields[30];
  int fieldCount = 0;

  fields[fieldCount++] = copy;

  for (
    char* pointer = copy;
    *pointer != '\0';
    pointer++
  ) {
    if (*pointer == ',') {
      *pointer = '\0';

      if (fieldCount < 30) {
        fields[fieldCount++] = pointer + 1;
      }
    }
  }

  // Campo 27 (índice 26) usado por el UM980 en la trama:
  // $PUBX,00,hhmmss.ss,lat,N,lon,E,alt,navStat,...*CS
  // donde navStat reporta modo de solución (ej. G3, R3, HP/HAS).
  // Este índice sigue el formato de salida actual del receptor UM980.
  if (fieldCount < 27) {
    return false;
  }

  const char* navStat = fields[26];

  // Detectar HAS
  if (strstr(navStat, "HP") != nullptr ||
      strstr(navStat, "HAS") != nullptr) {
    hasActive = true;
    solutionType = SOL_HAS;
    debugPrintf("[GNSS] HAS ACTIVO - Precisión mejorada\n");
  }
  else if (strstr(navStat, "R3") != nullptr ||
           strstr(navStat, "RTK") != nullptr) {
    hasActive = false;
    solutionType = SOL_RTK;
    debugPrintf("[GNSS] RTK detectado\n");
  }
  else if (strstr(navStat, "G3") != nullptr ||
           strstr(navStat, "G2") != nullptr) {
    hasActive = false;
    solutionType = SOL_GPS;
    debugPrintf("[GNSS] GPS+Galileo estándar\n");
  }
  else {
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
  if (line == nullptr ||
      line[0] == '\0') {
    return;
  }

  if (strncmp(line, "$GPGGA", 6) == 0 ||
      strncmp(line, "$GNGGA", 6) == 0 ||
      strncmp(line, "$GCGGA", 6) == 0) {
    parseGGA(line);
  }
  else if (strncmp(line, "$GPRMC", 6) == 0 ||
           strncmp(line, "$GNRMC", 6) == 0 ||
           strncmp(line, "$GCRMC", 6) == 0) {
    parseRMC(line);
  }
  else if (strncmp(line, "$PUBX,00", 8) == 0) {
    parsePUBX00(line);
  }
}

void readGNSS() {
  while (Serial1.available() > 0) {
    char character =
      (char)Serial1.read();

    if (character == '$' ||
        character == '#') {
      gnssLineIndex = 0;
      gnssLine[gnssLineIndex++] = character;
      continue;
    }

    if (gnssLineIndex <= 0) {
      continue;
    }

    if (character == '\r') {
      continue;
    }

    if (character == '\n') {
      gnssLine[gnssLineIndex] = '\0';

      processGnssLine(gnssLine);

      gnssLineIndex = 0;
      continue;
    }

    if (gnssLineIndex <
        (int)sizeof(gnssLine) - 1) {
      gnssLine[gnssLineIndex++] = character;
    }
    else {
      Serial.println("[GNSS] Línea demasiado larga");
      gnssLineIndex = 0;
    }
  }
}

// ============================================================================
// BNO085
// ============================================================================

bool initializeBNO085() {
  if (!bno08x.begin_I2C(
        BNO085_ADDRESS,
        &Wire
      )) {
    Serial.println("[IMU] No se pudo inicializar BNO085");
    bnoAvailable = false;
    return false;
  }

  if (!bno08x.enableReport(
        SH2_ROTATION_VECTOR,
        10000
      )) {
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
    if (sensorValue.sensorId ==
        SH2_ROTATION_VECTOR) {
      float i =
        sensorValue.un.rotationVector.i;

      float j =
        sensorValue.un.rotationVector.j;

      float k =
        sensorValue.un.rotationVector.k;

      float real =
        sensorValue.un.rotationVector.real;

      double yawRadians = atan2(
        2.0 * (real * k + i * j),
        1.0 - 2.0 * (j * j + k * k)
      );

      double magneticYaw =
        radiansToDegrees(yawRadians);

      currentYaw =
        normalizeAngle(
          magneticYaw - DECLINATION_DEG_VAR
        );

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

void applyAntennaOffset(
  double rawLat,
  double rawLon,
  double yaw,
  double* correctedLat,
  double* correctedLon
) {
  *correctedLat = rawLat;
  *correctedLon = rawLon;

  if (isnan(yaw)) {
    return;
  }

  double bearing =
    normalizeAngle(yaw + 90.0);

  double bearingRadians =
    degreesToRadians(bearing);

  double offsetRadians =
    OFFSET_M_VAR / EARTH_RADIUS_M;

  double latitudeOffsetRadians =
    offsetRadians * cos(bearingRadians);

  double latitudeCosine =
    cos(degreesToRadians(rawLat));

  if (fabs(latitudeCosine) < 0.000001) {
    return;
  }

  double longitudeOffsetRadians =
    offsetRadians *
    sin(bearingRadians) /
    latitudeCosine;

  double latitudeOffsetDegrees =
    radiansToDegrees(latitudeOffsetRadians);

  double longitudeOffsetDegrees =
    radiansToDegrees(longitudeOffsetRadians);

  *correctedLat =
    rawLat + latitudeOffsetDegrees;

  *correctedLon =
    rawLon + longitudeOffsetDegrees;
}

// ============================================================================
// PRUEBA DE ARRANQUE DE LEDS
// ============================================================================

void startupLedTest() {
  uint32_t startMs = millis();
  uint32_t lastPrintMs = 0;

  Serial.println(
    "[LED] Prueba de arranque durante 5 segundos"
  );

  while (millis() - startMs < STARTUP_LED_TEST_MS) {
    uint32_t now = millis();
    uint32_t elapsedMs = now - startMs;

    bool state =
      ((elapsedMs % STARTUP_LED_PERIOD_MS) <
       (STARTUP_LED_PERIOD_MS / 2));

    digitalWrite(
      LED1_PIN,
      state ? HIGH : LOW
    );

    digitalWrite(
      LED2_PIN,
      state ? HIGH : LOW
    );

    if (now - lastPrintMs >= 500) {
      lastPrintMs = now;

      debugPrintf(
        "[LED] Startup: %lu ms - %s\n",
        (unsigned long)elapsedMs,
        state ? "ON" : "OFF"
      );
    }

    delay(10);
  }

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  Serial.println("[LED] Fin de prueba de arranque");
}

// ============================================================================
// ESTADO NORMAL DE LEDS - Rev.2
// ============================================================================

void updateLeds() {
  uint32_t now = millis();

  // LED1: GNSS + IMU + HAS
  if (!gnssValid) {
    // OFF = Sin GNSS
    digitalWrite(LED1_PIN, LOW);
  }
  else if (!bnoAvailable) {
    // Parpadeo MUY rápido (200ms) = GNSS OK pero SIN magnetómetro
    digitalWrite(
      LED1_PIN,
      ((now % 200) < 100) ? HIGH : LOW
    );
  }
  else if (!hasActive) {
    // Parpadeo MEDIO (600ms) = GNSS + IMU pero SIN HAS
    digitalWrite(
      LED1_PIN,
      ((now % 600) < 300) ? HIGH : LOW
    );
  }
  else if (movementState == LOCKED && lockedValid) {
    // ON = GNSS + IMU + HAS + Posición bloqueada ✅
    digitalWrite(LED1_PIN, HIGH);
  }
  else {
    // Parpadeo LENTO (1000ms) = GNSS + IMU + HAS, esperando bloqueo
    digitalWrite(
      LED1_PIN,
      ((now % 1000) < 500) ? HIGH : LOW
    );
  }

  // LED2: Movimiento
  if (movementState == MOVING) {
    // OFF = En movimiento
    digitalWrite(LED2_PIN, LOW);
  }
  else if (movementState == AVERAGING) {
    // Parpadeo RÁPIDO (400ms) = Promediando
    digitalWrite(
      LED2_PIN,
      ((now % 400) < 200) ? HIGH : LOW
    );
  }
  else if (movementState == LOCKED && lockedValid) {
    // ON = Posición bloqueada
    digitalWrite(LED2_PIN, HIGH);
  }
  else {
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

  memset(
    latitudeSamples,
    0,
    sizeof(latitudeSamples)
  );

  memset(
    longitudeSamples,
    0,
    sizeof(longitudeSamples)
  );

  memset(
    altitudeSamples,
    0,
    sizeof(altitudeSamples)
  );

  memset(
    yawSamples,
    0,
    sizeof(yawSamples)
  );
}

int getOutputFixQuality() {
  if (!gnssValid &&
      !(movementState == LOCKED && lockedValid)) {
    return 0;
  }

  if (hasActive) {
    return 2;
  }

  return 1;
}

void updateMovementState() {
  uint32_t now = millis();

  bool speedFresh =
    (now - lastRmcMs) <= SPEED_FRESHNESS_MS;

  bool ggaFresh =
    (now - lastGgaMs) <= GGA_FRESHNESS_MS;

  if (!speedFresh) {
    currentSpeedMS = 0.0;
  }

  bool speedForStop = speedFresh ||
                      currentSpeedMS <= 0.0001;

  if (!ggaFresh) {
    gnssValid = false;
    if (movementState != LOCKED) {
      lockedValid = false;
    }
  }

  switch (movementState) {
    case MOVING: {
      if (speedForStop &&
          currentSpeedMS < SPEED_ENTER_STOP_MS_VAR) {
        if (lastStopCheckMs == 0) {
          lastStopCheckMs = now;
        }
      }
      else if (speedFresh &&
               currentSpeedMS >= SPEED_EXIT_STOP_MS_VAR) {
        lastStopCheckMs = 0;
      }

      if (lastStopCheckMs > 0 &&
          now - lastStopCheckMs >=
          STOP_CONFIRMATION_MS) {
        movementState = AVERAGING;
        averagingStartMs = now;
        lockedValid = false;

        clearAverageBuffers();

        Serial.println(
          "[STATE] MOVING -> AVERAGING"
        );
      }

      break;
    }

    case AVERAGING: {
      if (gnssValid &&
          lastGgaMs != lastSampledGgaMs &&
          sampleCount < MAX_SAMPLES) {
        latitudeSamples[sampleCount] = currentLat;
        longitudeSamples[sampleCount] = currentLon;
        altitudeSamples[sampleCount] = currentAlt;

        sampleCount++;
        lastSampledGgaMs = lastGgaMs;

        // Solo guardar yaw si BNO disponible
        if (bnoAvailable && !isnan(currentYaw) &&
            yawSampleCount < MAX_SAMPLES) {
          yawSamples[yawSampleCount++] =
            currentYaw;
        }
      }

      if (speedFresh &&
          currentSpeedMS > SPEED_EXIT_STOP_MS_VAR) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;

        clearAverageBuffers();

        Serial.println(
          "[STATE] AVERAGING -> MOVING"
        );

        break;
      }

      if (now - averagingStartMs >=
          AVERAGING_WINDOW_MS_VAR &&
          sampleCount > 0) {
        double averageRawLat =
          trimmedMean(
            latitudeSamples,
            sampleCount
          );

        double averageRawLon =
          trimmedMean(
            longitudeSamples,
            sampleCount
          );

        double averageAlt =
          trimmedMean(
            altitudeSamples,
            sampleCount
          );

        double averageYaw =
          circularMeanDegrees(
            yawSamples,
            yawSampleCount
          );

        if (isnan(averageRawLat) ||
            isnan(averageRawLon) ||
            isnan(averageAlt)) {
          movementState = MOVING;
          lockedValid = false;

          clearAverageBuffers();

          Serial.println(
            "[STATE] Error en promedio"
          );

          break;
        }

        rawLockedLat = averageRawLat;
        rawLockedLon = averageRawLon;

        lockedAlt = averageAlt;
        lockedYaw = averageYaw;

        applyAntennaOffset(
          rawLockedLat,
          rawLockedLon,
          lockedYaw,
          &lockedLat,
          &lockedLon
        );

        lockedValid = true;
        movementState = LOCKED;
        lastStopCheckMs = 0;

        debugPrintf(
          "[STATE] AVERAGING -> LOCKED "
          "lat=%.6f lon=%.6f yaw=%.1f\n",
          lockedLat,
          lockedLon,
          lockedYaw
        );
      }

      break;
    }

    case LOCKED: {
      if (speedFresh &&
          currentSpeedMS > SPEED_EXIT_STOP_MS_VAR) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;

        clearAverageBuffers();

        Serial.println(
          "[STATE] LOCKED -> MOVING por velocidad"
        );

        break;
      }

      if (gnssValid) {
        double distance =
          haversineMeters(
            currentLat,
            currentLon,
            rawLockedLat,
            rawLockedLon
          );

        if (distance > RELOCK_DISTANCE_M) {
          movementState = MOVING;
          lockedValid = false;
          lastStopCheckMs = 0;

          clearAverageBuffers();

          debugPrintf(
            "[STATE] LOCKED -> MOVING "
            "dist=%.2f m\n",
            distance
          );
        }
      }

      break;
    }
  }
}

// ============================================================================
// FORMATO NMEA
// ============================================================================

void formatLatitude(
  double latitude,
  char* output,
  size_t outputSize,
  char* hemisphere
) {
  double absoluteValue = fabs(latitude);
  int degrees = (int)absoluteValue;

  double minutes =
    (absoluteValue - degrees) * 60.0;

  *hemisphere =
    latitude >= 0.0 ? 'N' : 'S';

  int minuteInt = (int)minutes;
  int minuteFrac = (int)round(
    (minutes - minuteInt) * 10000.0
  );

  if (minuteFrac >= 10000) {
    minuteFrac -= 10000;
    minuteInt++;
  }

  if (minuteInt >= 60) {
    minuteInt -= 60;
    degrees++;
  }

  snprintf(
    output,
    outputSize,
    "%02d%02d.%04d",
    degrees,
    minuteInt,
    minuteFrac
  );
}

void formatLongitude(
  double longitude,
  char* output,
  size_t outputSize,
  char* hemisphere
) {
  double absoluteValue = fabs(longitude);
  int degrees = (int)absoluteValue;

  double minutes =
    (absoluteValue - degrees) * 60.0;

  *hemisphere =
    longitude >= 0.0 ? 'E' : 'W';

  int minuteInt = (int)minutes;
  int minuteFrac = (int)round(
    (minutes - minuteInt) * 10000.0
  );

  if (minuteFrac >= 10000) {
    minuteFrac -= 10000;
    minuteInt++;
  }

  if (minuteInt >= 60) {
    minuteInt -= 60;
    degrees++;
  }

  snprintf(
    output,
    outputSize,
    "%03d%02d.%04d",
    degrees,
    minuteInt,
    minuteFrac
  );
}

void createGCGGA(
  char* output,
  size_t outputSize,
  double latitude,
  double longitude,
  double altitude,
  int fixQuality
) {
  char latitudeText[24];
  char longitudeText[24];
  char altitudeText[24];

  char latitudeHemisphere;
  char longitudeHemisphere;

  formatLatitude(
    latitude,
    latitudeText,
    sizeof(latitudeText),
    &latitudeHemisphere
  );

  formatLongitude(
    longitude,
    longitudeText,
    sizeof(longitudeText),
    &longitudeHemisphere
  );

  dtostrf(
    altitude,
    0,
    1,
    altitudeText
  );

  char* firstAltitudeCharacter =
    altitudeText;

  while (*firstAltitudeCharacter == ' ') {
    firstAltitudeCharacter++;
  }

  snprintf(
    output,
    outputSize,
    "$GCGGA,%s,%s,%c,%s,%c,%d,%02d,%s,%s,M,0.0,M,,",
    utcTime,
    latitudeText,
    latitudeHemisphere,
    longitudeText,
    longitudeHemisphere,
    fixQuality,
    satelliteCount,
    hdopText,
    firstAltitudeCharacter
  );
}

void addNmeaChecksumAndCrlf(
  char* sentence,
  size_t sentenceSize
) {
  if (sentence == nullptr) {
    return;
  }

  uint8_t checksum =
    calculateNmeaChecksum(sentence);

  size_t length = strlen(sentence);

  if (length + 7 >= sentenceSize) {
    return;
  }

  snprintf(
    sentence + length,
    sentenceSize - length,
    "*%02X\r\n",
    checksum
  );
}

// ============================================================================
// ETHERNET
// ============================================================================

bool initializeEthernet() {
  pinMode(
    SD_CS_PIN,
    OUTPUT
  );

  digitalWrite(
    SD_CS_PIN,
    HIGH
  );

  Ethernet.init(
    ETHERNET_CS_PIN
  );

  Serial.println(
    "[ETH] Inicializando Ethernet"
  );

  int dhcpResult =
    Ethernet.begin(ethernetMac);

  if (dhcpResult == 0) {
    Serial.println(
      "[ETH] DHCP no disponible"
    );

    ethernetReady = false;
    return false;
  }

  delay(1000);

  Serial.print(
    "[ETH] IP local: "
  );

  Serial.println(
    Ethernet.localIP()
  );

  ethernetReady = true;

  return true;
}

void maintainEthernet() {
  uint32_t now = millis();

  if (now - lastEthernetCheckMs < 5000) {
    return;
  }

  lastEthernetCheckMs = now;

  if (Ethernet.hardwareStatus() ==
      EthernetNoHardware) {
    ethernetReady = false;

    Serial.println(
      "[ETH] Shield no detectado"
    );

    return;
  }

  if (Ethernet.linkStatus() ==
      LinkOFF) {
    ethernetReady = false;

    Serial.println(
      "[ETH] Cable Ethernet desconectado"
    );

    return;
  }

  ethernetReady = true;
}

void transmitEthernet(
  const char* sentence
) {
  if (!ethernetReady ||
      sentence == nullptr) {
    return;
  }

  if (!ethernetClient.connected()) {
    ethernetClient.stop();

    if (!ethernetClient.connect(
          ethernetTargetIP,
          ethernetTargetPort
        )) {
      return;
    }

    Serial.println(
      "[ETH] Conexión TCP establecida"
    );
  }

  ethernetClient.write(
    (const uint8_t*)sentence,
    strlen(sentence)
  );
}

// ============================================================================
// WIFI AP + TERMINAL TCP INTERACTIVO
// ============================================================================

bool initializeWiFiAp() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("[WIFI] Módulo WiFi no detectado");
    return false;
  }

  Serial.println("[WIFI] Iniciando AP de diagnóstico");

  int status = WiFi.beginAP(
    wifiSsid,
    wifiPassword
  );

  if (status != WL_AP_LISTENING &&
      WiFi.status() != WL_AP_LISTENING) {
    Serial.println("[WIFI] AP en arranque (no bloqueante)");
    wifiApReady = false;
    return false;
  }

  wifiApReady = true;
  if (!diagTcpServerStarted) {
    diagTcpServer.begin();
    diagTcpServerStarted = true;
  }

  Serial.print("[WIFI] AP activo: ");
  Serial.print(wifiSsid);
  Serial.print(" @ ");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(wifiTcpPort);

  return true;
}

void maintainWiFiAp() {
  if (WiFi.status() == WL_NO_MODULE) {
    wifiApReady = false;
    return;
  }

  if (wifiApReady) {
    if (WiFi.status() != WL_AP_LISTENING) {
      wifiApReady = false;
      if (diagTcpClient) {
        diagTcpClient.stop();
      }
      Serial.println("[WIFI] AP detenido, reintentando");
    }

    return;
  }

  if (WiFi.status() == WL_AP_LISTENING) {
    wifiApReady = true;
    if (!diagTcpServerStarted) {
      diagTcpServer.begin();
      diagTcpServerStarted = true;
    }

    Serial.print("[WIFI] AP activo: ");
    Serial.print(wifiSsid);
    Serial.print(" @ ");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(wifiTcpPort);
    return;
  }

  uint32_t now = millis();

  if (now - lastWiFiRetryMs < 5000) {
    return;
  }

  lastWiFiRetryMs = now;

  int status = WiFi.beginAP(
    wifiSsid,
    wifiPassword
  );

  if (status == WL_AP_LISTENING ||
      WiFi.status() == WL_AP_LISTENING) {
    wifiApReady = true;
    if (!diagTcpServerStarted) {
      diagTcpServer.begin();
      diagTcpServerStarted = true;
    }

    Serial.print("[WIFI] AP activo: ");
    Serial.print(wifiSsid);
    Serial.print(" @ ");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(wifiTcpPort);
  }
}

void sendTcpHelp() {
  tcpPrintf("=== COMANDOS DISPONIBLES ===\r\n");
  tcpPrintf("freq <1-10>        - Frecuencia en Hz\r\n");
  tcpPrintf("speed_stop <0-1>   - Umbral parada (m/s)\r\n");
  tcpPrintf("speed_move <0-1>   - Umbral movimiento (m/s)\r\n");
  tcpPrintf("offset <0-2>       - Offset antena (m)\r\n");
  tcpPrintf("decl <-180-180>    - Declinación magnética (°)\r\n");
  tcpPrintf("avg <5-60>         - Ventana promedio (s)\r\n");
  tcpPrintf("status             - Mostrar estado actual\r\n");
  tcpPrintf("help               - Este mensaje\r\n");
  tcpPrintf("============================\r\n");
}

void sendTcpStatus() {
  const char* stateText = "MOVING";

  if (movementState == AVERAGING) {
    stateText = "AVERAGING";
  }
  else if (movementState == LOCKED) {
    stateText = "LOCKED";
  }

  tcpPrintf("=== ESTADO ACTUAL ===\r\n");
  tcpPrintf(
    "Frecuencia: %lu Hz (%lu ms)\r\n",
    (unsigned long)(1000UL / OUTPUT_PERIOD_MS_VAR),
    (unsigned long)OUTPUT_PERIOD_MS_VAR
  );
  tcpPrintf(
    "Umbral parada: %.2f m/s\r\n",
    SPEED_ENTER_STOP_MS_VAR
  );
  tcpPrintf(
    "Umbral movimiento: %.2f m/s\r\n",
    SPEED_EXIT_STOP_MS_VAR
  );
  tcpPrintf(
    "Offset antena: %.2f m\r\n",
    OFFSET_M_VAR
  );
  tcpPrintf(
    "Declinación: %.1f°\r\n",
    DECLINATION_DEG_VAR
  );
  tcpPrintf(
    "Ventana promedio: %.1f s\r\n",
    (double)AVERAGING_WINDOW_MS_VAR / 1000.0
  );
  tcpPrintf(
    "Estado GNSS: %s\r\n",
    gnssValid ? "OK" : "NO"
  );
  tcpPrintf(
    "Estado IMU: %s\r\n",
    bnoAvailable ? "OK" : "NO"
  );
  tcpPrintf(
    "HAS activo: %s\r\n",
    hasActive ? "SI" : "NO"
  );
  tcpPrintf("Movimiento: %s\r\n", stateText);
  tcpPrintf(
    "Bloqueado: %s\r\n",
    lockedValid ? "SI" : "NO"
  );
  tcpPrintf("===================\r\n");
}

void processTcpCommand(char* line) {
  if (line == nullptr) {
    return;
  }

  while (*line == ' ' || *line == '\t') {
    line++;
  }

  if (*line == '\0') {
    return;
  }

  char* commandEnd = line;
  char* argument = nullptr;

  while (*commandEnd != '\0' &&
         *commandEnd != ' ' &&
         *commandEnd != '\t') {
    commandEnd++;
  }

  if (*commandEnd != '\0') {
    *commandEnd = '\0';
    argument = commandEnd + 1;

    while (*argument == ' ' || *argument == '\t') {
      argument++;
    }
  }
  else {
    argument = commandEnd;
  }

  if (strcmp(line, "help") == 0) {
    sendTcpHelp();
    return;
  }

  if (strcmp(line, "status") == 0) {
    sendTcpStatus();
    return;
  }

  if (*argument == '\0') {
    tcpPrintf("ERROR: Falta valor\r\n");
    return;
  }

  if (strcmp(line, "freq") == 0) {
    long hz = strtol(argument, nullptr, 10);

    if (hz < 1 || hz > 10) {
      tcpPrintf("ERROR: freq fuera de rango (1-10)\r\n");
      return;
    }

    OUTPUT_PERIOD_MS_VAR = (uint32_t)(1000UL / (uint32_t)hz);
    tcpPrintf(
      "OK: Frecuencia cambiada a %ld Hz (%lu ms)\r\n",
      hz,
      (unsigned long)OUTPUT_PERIOD_MS_VAR
    );
    return;
  }

  double value = 0.0;

  if (!parseDoubleField(argument, &value)) {
    tcpPrintf("ERROR: Valor inválido\r\n");
    return;
  }

  if (strcmp(line, "speed_stop") == 0) {
    if (value < 0.0 || value > 1.0) {
      tcpPrintf("ERROR: speed_stop fuera de rango (0-1)\r\n");
      return;
    }

    if (value >= SPEED_EXIT_STOP_MS_VAR) {
      tcpPrintf("ERROR: speed_stop debe ser menor que speed_move\r\n");
      return;
    }

    SPEED_ENTER_STOP_MS_VAR = value;
    tcpPrintf("OK: Umbral parada = %.2f m/s\r\n", value);
    return;
  }

  if (strcmp(line, "speed_move") == 0) {
    if (value < 0.0 || value > 1.0) {
      tcpPrintf("ERROR: speed_move fuera de rango (0-1)\r\n");
      return;
    }

    if (value <= SPEED_ENTER_STOP_MS_VAR) {
      tcpPrintf("ERROR: speed_move debe ser mayor que speed_stop\r\n");
      return;
    }

    SPEED_EXIT_STOP_MS_VAR = value;
    tcpPrintf("OK: Umbral movimiento = %.2f m/s\r\n", value);
    return;
  }

  if (strcmp(line, "offset") == 0) {
    if (value < 0.0 || value > 2.0) {
      tcpPrintf("ERROR: offset fuera de rango (0-2)\r\n");
      return;
    }

    OFFSET_M_VAR = value;
    tcpPrintf("OK: Offset antena = %.2f m\r\n", value);
    return;
  }

  if (strcmp(line, "decl") == 0) {
    if (value < -180.0 || value > 180.0) {
      tcpPrintf("ERROR: decl fuera de rango (-180 a 180)\r\n");
      return;
    }

    DECLINATION_DEG_VAR = value;
    tcpPrintf("OK: Declinación = %.1f°\r\n", value);
    return;
  }

  if (strcmp(line, "avg") == 0) {
    if (value < 5.0 || value > 60.0) {
      tcpPrintf("ERROR: avg fuera de rango (5-60)\r\n");
      return;
    }

    AVERAGING_WINDOW_MS_VAR = (uint32_t)(value * 1000.0);
    tcpPrintf("OK: Ventana promedio = %.1f s\r\n", value);
    return;
  }

  tcpPrintf("ERROR: Comando no reconocido\r\n");
}

void handleTcpServer() {
  if (!wifiApReady) {
    return;
  }

  WiFiClient newClient = diagTcpServer.available();

  if (newClient) {
    if (diagTcpClient &&
        diagTcpClient.connected()) {
      newClient.print("ERROR: Ya existe un cliente conectado\r\n");
      newClient.stop();
      return;
    }

    if (diagTcpClient) {
      diagTcpClient.stop();
    }

    diagTcpClient = newClient;
    tcpLineIndex = 0;

    tcpPrintf("Cliente conectado\r\n");
    sendTcpHelp();
  }

  if (!diagTcpClient || !diagTcpClient.connected()) {
    return;
  }

  while (diagTcpClient.available() > 0) {
    char character = (char)diagTcpClient.read();

    if (character == '\r') {
      continue;
    }

    if (character == '\n') {
      tcpLine[tcpLineIndex] = '\0';
      processTcpCommand(tcpLine);
      tcpLineIndex = 0;
      continue;
    }

    if (tcpLineIndex < (int)sizeof(tcpLine) - 1) {
      tcpLine[tcpLineIndex++] = character;
    }
    else {
      tcpLineIndex = 0;
      tcpPrintf("ERROR: Línea demasiado larga\r\n");
    }
  }
}

void sendTcpDiagnostic() {
  if (!diagTcpClient || !diagTcpClient.connected()) {
    return;
  }

  uint32_t now = millis();

  if (now - lastTcpDiagMs < OUTPUT_PERIOD_MS_VAR) {
    return;
  }

  lastTcpDiagMs = now;

  char line[240];

  snprintf(
    line,
    sizeof(line),
    "[%05lu] GNSS:%s LAT:%.6f LON:%.6f ALT:%.1f SAT:%d HDOP:%s SPEED:%.2f STATE:%d HAS:%d IMU:%d LOCK:%d\r\n",
    (unsigned long)((now / 1000UL) % 100000UL),
    gnssValid ? "OK" : "NO",
    currentLat,
    currentLon,
    currentAlt,
    satelliteCount,
    hdopText,
    currentSpeedMS,
    (int)movementState,
    hasActive ? 1 : 0,
    bnoAvailable ? 1 : 0,
    lockedValid ? 1 : 0
  );

  diagTcpClient.print(line);
}

// ============================================================================
// TRANSMISIÓN GCGGA
// ============================================================================

void transmitGCGGA() {
  uint32_t now = millis();

  if (now - lastOutputMs <
      OUTPUT_PERIOD_MS_VAR) {
    return;
  }

  lastOutputMs = now;

  int fixQuality =
    getOutputFixQuality();

  bool lockedOutput =
    (movementState == LOCKED &&
     lockedValid);

  if (!gnssValid && !lockedOutput) {
    return;
  }

  if (fixQuality < 1) {
    return;
  }

  double outputLat = currentLat;
  double outputLon = currentLon;
  double outputAlt = currentAlt;

  if (movementState == LOCKED &&
      lockedValid) {
    outputLat = lockedLat;
    outputLon = lockedLon;
    outputAlt = lockedAlt;
  }
  else {
    // Solo aplicar offset si tenemos yaw del magnetómetro
    if (bnoAvailable && !isnan(currentYaw)) {
      applyAntennaOffset(
        currentLat,
        currentLon,
        currentYaw,
        &outputLat,
        &outputLon
      );
    }
  }

  char sentence[240];

  createGCGGA(
    sentence,
    sizeof(sentence),
    outputLat,
    outputLon,
    outputAlt,
    fixQuality
  );

  addNmeaChecksumAndCrlf(
    sentence,
    sizeof(sentence)
  );

  size_t sentenceLength = strlen(sentence);

  // COM2: salida serie por software (solo TX, temporización compensada)
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
  Serial.println(
    "=========================================="
  );
  Serial.println(
    "RS232-FMW-GPS Rev.2.1"
  );
  Serial.println(
    "Arduino UNO R4 WiFi"
  );
  Serial.println(
    "=========================================="
  );
  Serial.println(
    "GNSS:     Serial1 D0/D1 @ 115200 (UM980 COM3)"
  );
  Serial.println(
    "COM2:     D2 @ 38400 (soft-UART TX-only, compensado)"
  );
  Serial.println(
    "Ethernet: Shield 2 / W5500"
  );
  Serial.println(
    "Destino:  192.168.1.122:15919"
  );
  Serial.println(
    "WiFi AP:  FWD-GPS-Diag / 12345678"
  );
  Serial.println(
    "TCP Diag: 192.168.4.1:15920"
  );
  Serial.println(
    "BNO085:   TWI/I2C @ 100 kHz"
  );
  Serial.println(
    "LED1:     D5 / OUT5 (GNSS+IMU+HAS)"
  );
  Serial.println(
    "LED2:     D6 / OUT6 (Movimiento)"
  );
  Serial.println(
    "SD:       D4 desactivada"
  );
  Serial.println(
    "=========================================="
  );
  Serial.println(
    "CONFIGURACIÓN UM980 (u-center):"
  );
  Serial.println(
    "  UNLOG COM3"
  );
  Serial.println(
    "  CONFIG COM3 115200"
  );
  Serial.println(
    "  GNGGA COM3 0.1"
  );
  Serial.println(
    "  GNRMC COM3 0.1"
  );
  Serial.println(
    "  CONFIG NMEA PUBX ENABLE"
  );
  Serial.println(
    "  ENABLE HAS"
  );
  Serial.println(
    "  SAVECONFIG"
  );
  Serial.println(
    "=========================================="
  );

  // Configurar LEDs antes de inicializar cualquier periférico
  pinMode(
    LED1_PIN,
    OUTPUT
  );

  pinMode(
    LED2_PIN,
    OUTPUT
  );

  digitalWrite(
    LED1_PIN,
    LOW
  );

  digitalWrite(
    LED2_PIN,
    LOW
  );

  // Prueba visible de 5 segundos
  startupLedTest();

  // Desactivar micro-SD
  pinMode(
    SD_CS_PIN,
    OUTPUT
  );

  digitalWrite(
    SD_CS_PIN,
    HIGH
  );

  // GNSS
  Serial1.begin(
    GNSS_BAUD
  );

  // COM2 (UART por software, solo TX, compensado)
  softSerialInit();

  // Bus TWI/I2C
  Wire.begin();

  Wire.setClock(
    I2C_CLOCK_HZ
  );

  if (!initializeBNO085()) {
    Serial.println(
      "[SETUP] BNO085 no disponible; se reintentará"
    );
  }

  if (!initializeEthernet()) {
    Serial.println(
      "[SETUP] Ethernet no disponible"
    );
  }

  if (!initializeWiFiAp()) {
    Serial.println(
      "[SETUP] WiFi AP pendiente (reintento en loop)"
    );
  }

  Serial.println(
    "[SETUP] Sistema preparado"
  );
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  maintainWiFiAp();

  maintainEthernet();

  readGNSS();

  readYaw();

  updateMovementState();

  updateLeds();

  transmitGCGGA();

  handleTcpServer();
  sendTcpDiagnostic();
}