/*
  RS232-FMW-GPS - Rev.2.7 BLE
  Arduino UNO R4 WiFi

  FUNCIONES:
  - GNSS UM980 conectado a Serial1, D0/D1, 115200 baudios.
  - COM2 TX por D2 mediante UART software directa, 38400 baudios.
  - Salida NMEA $GCGGA a 10 Hz.
  - ICM-20948 por I2C.
  - Corrección de posición mediante yaw y offset de antena.
  - Promedio y bloqueo de posición con el vehículo parado.
  - Ethernet W5500.
  - Diagnóstico Bluetooth Low Energy.
  - WiFi Server eliminado.
  - Precisión GNGST visible en BLE.
  - Validación FIX_IN para LOCKED.
  - Magnetómetro calibrado en fábrica (EEPROM).

  BLE:
  - Nombre: FWD-GPS-Diag
  - Servicio compatible con Nordic UART Service.
  - Activar notificaciones en la característica TX.
  - Escribir comandos en la característica RX.

  COMANDOS BLE:
    status
    imu
    com2
    help
    magcal status
    yawoff <-180..180>

  LIBRERÍAS:
  - SparkFun ICM-20948 Arduino Library
  - ArduinoBLE
  - Ethernet
  - EEPROM
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "ICM_20948.h"
#include <Ethernet.h>
#include <ArduinoBLE.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

// ============================================================================
// CONFIGURACIÓN GENERAL
// ============================================================================

#define GNSS_BAUD                   115200
#define COM2_BAUD                   38400
#define COM2_TX_PIN                 2

#define SD_CS_PIN                   4
#define ETHERNET_CS_PIN             10

#define LED1_PIN                    5
#define LED2_PIN                    6

#define ICM_ADDRESS_PRIMARY         0x69
#define ICM_ADDRESS_SECONDARY       0x68
#define I2C_CLOCK_HZ                100000

#define STARTUP_LED_TEST_MS         5000
#define STARTUP_LED_PERIOD_MS       400

#define YAW_GYRO_WEIGHT             0.98
#define ACC_FILTER_ALPHA            0.20
#define MAG_FILTER_ALPHA            0.20
#define GYRO_MAX_DT_S               0.5

#define EEPROM_MAGCAL_ADDR          0
#define EEPROM_MAGIC                0x49434D33UL

#define OFFSET_M_DEFAULT            0.55
#define DECLINATION_DEG_DEFAULT     -3.5
#define EARTH_RADIUS_M              6378137.0

#define STOP_CONFIRMATION_MS        2000
#define SPEED_FRESHNESS_MS          2000
#define GGA_FRESHNESS_MS            2000
#define IMU_RETRY_MS                5000
#define IMU_TIMEOUT_MS              5000

#define OUTPUT_PERIOD_MS_DEFAULT    100
#define AVERAGING_WINDOW_MS_DEFAULT 15000
#define RELOCK_DISTANCE_M           1.0
#define MAX_SAMPLES                 160

#define SPEED_ENTER_STOP_MS_DEFAULT 0.20
#define SPEED_EXIT_STOP_MS_DEFAULT  0.30

// ============================================================================
// CONFIGURACIÓN BLE
// ============================================================================

#define BLE_DEVICE_NAME             "FWD-GPS-Diag"
#define BLE_DIAGNOSTIC_PERIOD_MS     5000
#define BLE_TX_CHUNK_SIZE            20
#define BLE_COMMAND_BUFFER_SIZE      96

/*
  UUID Nordic UART Service:

  Servicio:
    6E400001-B5A3-F393-E0A9-E50E24DCCA9E

  RX: teléfono -> Arduino
    6E400002-B5A3-F393-E0A9-E50E24DCCA9E

  TX: Arduino -> teléfono
    6E400003-B5A3-F393-E0A9-E50E24DCCA9E
*/

BLEService diagnosticBLEService(
  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
);

BLECharacteristic bleRxCharacteristic(
  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
  BLEWrite | BLEWriteWithoutResponse,
  64
);

BLECharacteristic bleTxCharacteristic(
  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
  BLERead | BLENotify,
  244
);

// ============================================================================
// PROTOTIPOS
// ============================================================================

void debugPrintf(const char* format, ...);

void softSerialInit();
void softSerialWriteByte(uint8_t value);
void softSerialWriteString(const char* text, size_t length);

bool initializeBLE();
void maintainBLE();
void readBLECommands();
void processBLECommand(const char* command);
void sendBLEText(const char* text);
void sendBLEStatus();
void sendBLEIMU();
void sendBLECOM2();
void sendBLEHelp();
void sendBLEPeriodicDiagnostic();

void loadMagCalibration();
void saveMagCalibration();
void startMagCalibration();
bool stopMagCalibration();
void resetMagCalibration();

bool initializeICM20948();
double readYaw();

void readGNSS();
void updateMovementState();
void updateLeds();
void transmitGCGGA();
void clearAverageBuffers();

// ============================================================================
// ETHERNET
// ============================================================================

byte ethernetMac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ethernetTargetIP(192, 168, 1, 122);
const uint16_t ethernetTargetPort = 15919;

EthernetClient ethernetClient;

bool ethernetReady = false;
uint32_t lastEthernetCheckMs = 0;

// ============================================================================
// VARIABLES CONFIGURABLES
// ============================================================================

uint16_t OUTPUT_PERIOD_MS_VAR = OUTPUT_PERIOD_MS_DEFAULT;
uint16_t AVERAGING_WINDOW_MS_VAR = AVERAGING_WINDOW_MS_DEFAULT;

double SPEED_ENTER_STOP_MS_VAR = SPEED_ENTER_STOP_MS_DEFAULT;
double SPEED_EXIT_STOP_MS_VAR = SPEED_EXIT_STOP_MS_DEFAULT;
double OFFSET_M_VAR = OFFSET_M_DEFAULT;
double DECLINATION_DEG_VAR = DECLINATION_DEG_DEFAULT;
double YAW_MOUNT_OFFSET_DEG_VAR = 0.0;

// ============================================================================
// ESTADÍSTICAS COM2
// ============================================================================

uint32_t com2BytesSent = 0;
uint32_t com2FramesSent = 0;
uint32_t com2LastFrameMs = 0;

// ============================================================================
// ESTADO BLE
// ============================================================================

BLEDevice bleCentral;

bool bleReady = false;
bool bleConnected = false;

uint32_t lastBleDiagnosticMs = 0;

char bleCommandBuffer[BLE_COMMAND_BUFFER_SIZE];
uint8_t bleCommandIndex = 0;

// ============================================================================
// COM2 - UART SOFTWARE DIRECTA, SIN BUFFER
// ============================================================================

static uint32_t com2BitDurationUs = 0;

static inline void waitUntil(uint32_t targetUs) {
  while ((int32_t)(micros() - targetUs) < 0) {
    // Espera activa.
  }
}

void softSerialInit() {
  com2BitDurationUs =
    (uint32_t)((1000000UL + (COM2_BAUD / 2)) / COM2_BAUD);

  pinMode(COM2_TX_PIN, OUTPUT);

  // UART en reposo permanece en HIGH.
  digitalWrite(COM2_TX_PIN, HIGH);

  debugPrintf(
    "[COM2] Inicializado D%d a %d bps, bit=%lu us\n",
    COM2_TX_PIN,
    COM2_BAUD,
    (unsigned long)com2BitDurationUs
  );
}

void softSerialWriteByte(uint8_t value) {
  noInterrupts();

  uint32_t nextBitTime = micros();

  // Bit de inicio.
  digitalWrite(COM2_TX_PIN, LOW);
  nextBitTime += com2BitDurationUs;
  waitUntil(nextBitTime);

  // 8 bits de datos, LSB primero.
  for (uint8_t bitIndex = 0; bitIndex < 8; bitIndex++) {
    if (value & 0x01) {
      digitalWrite(COM2_TX_PIN, HIGH);
    } else {
      digitalWrite(COM2_TX_PIN, LOW);
    }

    value >>= 1;

    nextBitTime += com2BitDurationUs;
    waitUntil(nextBitTime);
  }

  // Bit de parada.
  digitalWrite(COM2_TX_PIN, HIGH);
  nextBitTime += com2BitDurationUs;
  waitUntil(nextBitTime);

  interrupts();

  com2BytesSent++;
}

void softSerialWriteString(const char* text, size_t length) {
  if (text == nullptr || length == 0) {
    return;
  }

  for (size_t i = 0; i < length; i++) {
    softSerialWriteByte((uint8_t)text[i]);
  }
}

// ============================================================================
// OBJETO ICM-20948
// ============================================================================

ICM_20948_I2C icm;

// ============================================================================
// ESTADOS DE MOVIMIENTO Y SOLUCIÓN
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
int inputFixQuality = 0;

char utcTime[16] = "000000.00";
char hdopText[16] = "1.0";

uint32_t lastGgaMs = 0;
uint32_t lastRmcMs = 0;
uint32_t lastPubxMs = 0;

// NUEVO: Precisión GNGST
double gnsHorizErr = NAN;
double gnsVertErr = NAN;
uint32_t lastGnstMs = 0;

// ============================================================================
// VARIABLES ICM-20948
// ============================================================================

bool imuAvailable = false;
uint8_t imuAddress = 0;

double currentYaw = NAN;

uint32_t lastImuDataMs = 0;
uint32_t lastImuRetryMs = 0;
uint32_t lastImuMessageMs = 0;
uint32_t lastImuMicros = 0;

double accFX = 0.0;
double accFY = 0.0;
double accFZ = 0.0;

double magFX = 0.0;
double magFY = 0.0;
double magFZ = 0.0;

double gyroZdps = 0.0;
double imuTempC = 0.0;
double imuRoll = 0.0;
double imuPitch = 0.0;
double magHeading = NAN;
double fusedHeading = NAN;

bool imuFilterSeeded = false;

// ============================================================================
// CALIBRACIÓN DEL MAGNETÓMETRO
// ============================================================================

struct MagCalibration {
  uint32_t magic;
  float offsetX;
  float offsetY;
  float offsetZ;
  float yawMountOffset;
};

MagCalibration magCal = {
  EEPROM_MAGIC,
  0.0f,
  0.0f,
  0.0f,
  0.0f
};

bool magCalValid = false;
bool magCalRunning = false;

double magMinX = 0.0;
double magMaxX = 0.0;
double magMinY = 0.0;
double magMaxY = 0.0;
double magMinZ = 0.0;
double magMaxZ = 0.0;

uint32_t magCalSamples = 0;

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

// Últimas coordenadas realmente enviadas por COM2.
double lastOutputLat = 0.0;
double lastOutputLon = 0.0;
double lastOutputAlt = 0.0;
int lastOutputFixQuality = 0;

// ============================================================================
// BUFFER GNSS
// ============================================================================

char gnssLine[240];
int gnssLineIndex = 0;

// ============================================================================
// DIAGNÓSTICO USB
// ============================================================================

void debugPrintf(const char* format, ...) {
  char buffer[320];

  va_list arguments;
  va_start(arguments, format);
  vsnprintf(buffer, sizeof(buffer), format, arguments);
  va_end(arguments);

  Serial.print(buffer);
}

// ============================================================================
// TEXTOS DE ESTADO
// ============================================================================

const char* movementStateText() {
  switch (movementState) {
    case MOVING:
      return "MOVING";

    case AVERAGING:
      return "AVERAGING";

    case LOCKED:
      return "LOCKED";

    default:
      return "UNKNOWN";
  }
}

const char* solutionTypeText() {
  switch (solutionType) {
    case SOL_GPS:
      return "GPS";

    case SOL_DGPS:
      return "DGPS";

    case SOL_RTK:
      return "RTK";

    case SOL_HAS:
      return "HAS";

    default:
      return "UNKNOWN";
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

double wrap180(double angle) {
  while (angle > 180.0) {
    angle -= 360.0;
  }

  while (angle <= -180.0) {
    angle += 360.0;
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

  return normalizeAngle(radiansToDegrees(meanRadians));
}

bool parseDoubleField(const char* text, double* value) {
  if (
    text == nullptr ||
    value == nullptr ||
    text[0] == '\0'
  ) {
    return false;
  }

  char* endPointer = nullptr;
  double parsedValue = strtod(text, &endPointer);

  if (
    endPointer == text ||
    *endPointer != '\0' ||
    !isfinite(parsedValue)
  ) {
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

  if (*pointer == '$' || *pointer == '#') {
    pointer++;
  }

  while (*pointer != '\0' && *pointer != '*') {
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

  if (
    asterisk == nullptr ||
    strlen(asterisk + 1) < 2
  ) {
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

  if (
    endPointer == checksumText ||
    *endPointer != '\0' ||
    receivedChecksum > 0xFF
  ) {
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
  if (
    latitudeField == nullptr ||
    hemisphereField == nullptr ||
    latitude == nullptr ||
    strlen(latitudeField) < 7
  ) {
    return false;
  }

  int degrees =
    (latitudeField[0] - '0') * 10 +
    (latitudeField[1] - '0');

  double minutes =
    strtod(latitudeField + 2, nullptr);

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
  if (
    longitudeField == nullptr ||
    hemisphereField == nullptr ||
    longitude == nullptr ||
    strlen(longitudeField) < 8
  ) {
    return false;
  }

  int degrees =
    (longitudeField[0] - '0') * 100 +
    (longitudeField[1] - '0') * 10 +
    (longitudeField[2] - '0');

  double minutes =
    strtod(longitudeField + 3, nullptr);

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
    Serial.println("[GNSS] Error checksum GGA");
    return false;
  }

  char copy[240];

  strncpy(copy, line, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char* fields[20];
  int fieldCount = 0;

  fields[fieldCount++] = copy;

  for (char* pointer = copy; *pointer != '\0'; pointer++) {
    if (*pointer == ',') {
      *pointer = '\0';

      if (fieldCount < 20) {
        fields[fieldCount++] = pointer + 1;
      }
    }
  }

  if (fieldCount < 10) {
    return false;
  }

  int fixQuality = atoi(fields[6]);
  inputFixQuality = fixQuality;

  if (fixQuality < 1) {
    gnssValid = false;
    return false;
  }

  double latitude = 0.0;
  double longitude = 0.0;

  if (!parseLatitude(fields[2], fields[3], &latitude)) {
    return false;
  }

  if (!parseLongitude(fields[4], fields[5], &longitude)) {
    return false;
  }

  double altitude = strtod(fields[9], nullptr);

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

  if (
    !parseDoubleField(fields[8], &hdop) ||
    hdop < 0.0
  ) {
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

  /*
    Algunos receptores reflejan el tipo de solución directamente
    en el campo de calidad GGA.

    No se fuerza HAS aquí, porque el significado exacto del valor
    depende de la configuración del receptor.
  */

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
    Serial.println("[GNSS] Error checksum RMC");
    return false;
  }

  char copy[240];

  strncpy(copy, line, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char* fields[20];
  int fieldCount = 0;

  fields[fieldCount++] = copy;

  for (char* pointer = copy; *pointer != '\0'; pointer++) {
    if (*pointer == ',') {
      *pointer = '\0';

      if (fieldCount < 20) {
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

  double speedKnots = strtod(fields[7], nullptr);
  double course = strtod(fields[8], nullptr);

  if (!isfinite(speedKnots)) {
    return false;
  }

  currentSpeedMS = speedKnots * 0.51444;

  if (isfinite(course)) {
    currentCourse = course;
  } else {
    currentCourse = 0.0;
  }

  lastRmcMs = millis();

  return true;
}

// ============================================================================
// PARSER PUBX,00
// ============================================================================

bool parsePUBX00(const char* line) {
  if (line == nullptr) {
    return false;
  }

  if (strncmp(line, "$PUBX,00", 8) != 0) {
    return false;
  }

  if (!validateNmeaChecksum(line)) {
    Serial.println("[GNSS] Error checksum PUBX,00");
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

      if (fieldCount < 30) {
        fields[fieldCount++] = pointer + 1;
      }
    }
  }

  if (fieldCount < 7) {
    return false;
  }

  double latitude = 0.0;
  double longitude = 0.0;

  if (!parseLatitude(fields[3], fields[4], &latitude)) {
    return false;
  }

  if (!parseLongitude(fields[5], fields[6], &longitude)) {
    return false;
  }

  currentLat = latitude;
  currentLon = longitude;

  lastPubxMs = millis();

  return true;
}

// ============================================================================
// PARSER GNGST (NUEVO - Precisión)
// ============================================================================

bool parseGNST(const char* line) {
  if (line == nullptr) {
    return false;
  }

  if (!validateNmeaChecksum(line)) {
    return false;
  }

  char copy[240];

  strncpy(copy, line, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char* fields[15];
  int fieldCount = 0;

  fields[fieldCount++] = copy;

  for (char* pointer = copy; *pointer != '\0'; pointer++) {
    if (*pointer == ',') {
      *pointer = '\0';

      if (fieldCount < 15) {
        fields[fieldCount++] = pointer + 1;
      }
    }
  }

  if (fieldCount < 8) {
    return false;
  }

  // Índices: $GNGST,hhmmss.ss,RMS,std_lat,std_lon,std_alt,...
  double stdLat = strtod(fields[3], nullptr);
  double stdLon = strtod(fields[4], nullptr);
  double stdAlt = strtod(fields[5], nullptr);

  if (isfinite(stdLat) && isfinite(stdLon) && isfinite(stdAlt)) {
    gnsHorizErr = sqrt(stdLat * stdLat + stdLon * stdLon);
    gnsVertErr = stdAlt;
    lastGnstMs = millis();
    return true;
  }

  return false;
}

// ============================================================================
// readGNSS - CON PARSEO DE GNGST
// ============================================================================

void readGNSS() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      gnssLine[gnssLineIndex] = '\0';

      if (strncmp(gnssLine, "$GNGGA", 6) == 0) {
        parseGGA(gnssLine);
      } else if (strncmp(gnssLine, "$GNRMC", 6) == 0) {
        parseRMC(gnssLine);
      } else if (strncmp(gnssLine, "$PUBX,00", 8) == 0) {
        parsePUBX00(gnssLine);
      } else if (strncmp(gnssLine, "$GNGST", 6) == 0) {
        parseGNST(gnssLine);
      }

      gnssLineIndex = 0;
    } else if (c == '\r') {
      // Ignorar
    } else if (gnssLineIndex < sizeof(gnssLine) - 1) {
      gnssLine[gnssLineIndex++] = c;
    }
  }
}

// ============================================================================
// clearAverageBuffers
// ============================================================================

void clearAverageBuffers() {
  sampleCount = 0;
  yawSampleCount = 0;
  memset(latitudeSamples, 0, sizeof(latitudeSamples));
  memset(longitudeSamples, 0, sizeof(longitudeSamples));
  memset(altitudeSamples, 0, sizeof(altitudeSamples));
  memset(yawSamples, 0, sizeof(yawSamples));
}

// ============================================================================
// updateMovementState - CON VALIDACIÓN FIX_IN
// ============================================================================

void updateMovementState() {
  uint32_t now = millis();

  bool speedFresh = (now - lastRmcMs) <= SPEED_FRESHNESS_MS;
  bool ggaFresh = (now - lastGgaMs) <= GGA_FRESHNESS_MS;

  if (!speedFresh) {
    currentSpeedMS = 0.0;
  }

  if (!ggaFresh) {
    gnssValid = false;
    lockedValid = false;
  }

  switch (movementState) {
    case MOVING: {
      if (speedFresh && currentSpeedMS < SPEED_ENTER_STOP_MS_VAR) {
        if (lastStopCheckMs == 0) {
          lastStopCheckMs = now;
        }
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
      }

      if (imuAvailable && !isnan(currentYaw)) {
        if (yawSampleCount < MAX_SAMPLES) {
          yawSamples[yawSampleCount] = currentYaw;
          yawSampleCount++;
        }
      }

      uint32_t elapsedMs = now - averagingStartMs;

      // VALIDACIÓN FIX_IN PARA LOCKED
      if (sampleCount > 0 && elapsedMs >= AVERAGING_WINDOW_MS_VAR && (inputFixQuality >= 4 || hasActive)) {
        double avgLat = trimmedMean(latitudeSamples, sampleCount);
        double avgLon = trimmedMean(longitudeSamples, sampleCount);
        double avgAlt = trimmedMean(altitudeSamples, sampleCount);

        if (isfinite(avgLat) && isfinite(avgLon) && isfinite(avgAlt)) {
          rawLockedLat = avgLat;
          rawLockedLon = avgLon;
          lockedLat = avgLat;
          lockedLon = avgLon;
          lockedAlt = avgAlt;

          if (yawSampleCount > 0) {
            lockedYaw = circularMeanDegrees(yawSamples, yawSampleCount);
          } else {
            lockedYaw = NAN;
          }

          lockedValid = true;
          movementState = LOCKED;
          Serial.println("[STATE] AVERAGING -> LOCKED");
        }
      }

      break;
    }

    case LOCKED: {
      double distance = haversineMeters(rawLockedLat, rawLockedLon, currentLat, currentLon);

      if (speedFresh && currentSpeedMS >= SPEED_EXIT_STOP_MS_VAR) {
        movementState = MOVING;
        lockedValid = false;
        Serial.println("[STATE] LOCKED -> MOVING");
      } else if (distance > RELOCK_DISTANCE_M) {
        movementState = MOVING;
        lockedValid = false;
        Serial.println("[STATE] LOCKED -> MOVING (distance)");
      }

      break;
    }
  }
}

// ============================================================================
// updateLeds - 2 LEDs CON 3 ESTADOS + FALLA
// ============================================================================

void updateLeds() {
  uint32_t now = millis();
  static uint32_t ledBlinkMs = 0;
  static bool ledBlinkState = false;

  uint32_t blinkPeriodMs = 300;
  if (movementState == AVERAGING) {
    blinkPeriodMs = 150;
  }

  if (now - ledBlinkMs >= blinkPeriodMs) {
    ledBlinkMs = now;
    ledBlinkState = !ledBlinkState;
  }

  bool hasFault = !gnssValid || !imuAvailable || !magCalValid;

  // LED1 (Rojo)
  if (hasFault) {
    digitalWrite(LED1_PIN, LOW);
  } else if (movementState == MOVING) {
    digitalWrite(LED1_PIN, ledBlinkState ? HIGH : LOW);
  } else if (movementState == AVERAGING) {
    digitalWrite(LED1_PIN, ledBlinkState ? HIGH : LOW);
  } else if (movementState == LOCKED && (inputFixQuality >= 4 || hasActive)) {
    digitalWrite(LED1_PIN, LOW);
  } else {
    digitalWrite(LED1_PIN, LOW);
  }

  // LED2 (Verde)
  if (hasFault) {
    digitalWrite(LED2_PIN, LOW);
  } else if (movementState == MOVING) {
    digitalWrite(LED2_PIN, LOW);
  } else if (movementState == AVERAGING) {
    digitalWrite(LED2_PIN, ledBlinkState ? HIGH : LOW);
  } else if (movementState == LOCKED && (inputFixQuality >= 4 || hasActive)) {
    digitalWrite(LED2_PIN, HIGH);
  } else {
    digitalWrite(LED2_PIN, LOW);
  }
}

// ============================================================================
// BLE Functions - Prototipos adelantados
// ============================================================================

bool initializeBLE() {
  if (!BLE.begin()) {
    Serial.println("[BLE] Error inicializando BLE");
    bleReady = false;
    return false;
  }

  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setLocalName(BLE_DEVICE_NAME);
  BLE.setAdvertisedService(diagnosticBLEService);

  diagnosticBLEService.addCharacteristic(bleRxCharacteristic);
  diagnosticBLEService.addCharacteristic(bleTxCharacteristic);

  BLE.addService(diagnosticBLEService);

  bleRxCharacteristic.setEventHandler(BLEWritten, [](BLEDevice central, BLECharacteristic characteristic) {
    // Manejador de escritura
  });

  BLE.advertise();

  debugPrintf("[BLE] Advertencia iniciada como %s\n", BLE_DEVICE_NAME);
  bleReady = true;

  return true;
}

void maintainBLE() {
  BLEDevice central = BLE.central();

  if (central && !bleConnected) {
    debugPrintf("[BLE] Conectado: %s\n", central.address().c_str());
    bleConnected = true;
    bleCentral = central;
  } else if (!central && bleConnected) {
    debugPrintf("[BLE] Desconectado\n");
    bleConnected = false;
  }
}

void readBLECommands() {
  if (!bleConnected) return;

  while (bleRxCharacteristic.written()) {
    bleRxCharacteristic.written(false);
  }
}

void processBLECommand(const char* command) {
  if (command == nullptr) return;

  if (strcmp(command, "status") == 0) {
    sendBLEStatus();
  } else if (strcmp(command, "imu") == 0) {
    sendBLEIMU();
  } else if (strcmp(command, "com2") == 0) {
    sendBLECOM2();
  } else if (strcmp(command, "help") == 0) {
    sendBLEHelp();
  } else if (strcmp(command, "magcal status") == 0) {
    char text[128];
    snprintf(text, sizeof(text), "MAGCAL=%s\r\n", magCalValid ? "YES" : "NO");
    sendBLEText(text);
  }
}

void sendBLEText(const char* text) {
  if (!bleConnected || text == nullptr) return;

  size_t len = strlen(text);
  for (size_t i = 0; i < len; i += BLE_TX_CHUNK_SIZE) {
    size_t chunk = len - i > BLE_TX_CHUNK_SIZE ? BLE_TX_CHUNK_SIZE : len - i;
    bleTxCharacteristic.writeValue((uint8_t*)(text + i), chunk);
    delay(10);
  }
}

void sendBLEStatus() {
  char text[256];
  snprintf(text, sizeof(text),
    "[STATUS]\r\n"
    "State=%s Lock=%s\r\n"
    "GNSS=%s FIX=%d SAT=%d\r\n"
    "IMU=%s YAW=%s\r\n",
    movementStateText(),
    lockedValid ? "YES" : "NO",
    gnssValid ? "OK" : "FAIL",
    inputFixQuality,
    satelliteCount,
    imuAvailable ? "OK" : "FAIL",
    isnan(currentYaw) ? "N/A" : "OK"
  );
  sendBLEText(text);
}

void sendBLEIMU() {
  char text[256];
  snprintf(text, sizeof(text),
    "[IMU]\r\n"
    "Status=%s Addr=0x%02X\r\n"
    "Temp=%.1fC\r\n"
    "Roll=%.1f Pitch=%.1f\r\n"
    "Yaw=%s\r\n",
    imuAvailable ? "OK" : "FAIL",
    imuAddress,
    imuTempC,
    imuRoll,
    imuPitch,
    isnan(currentYaw) ? "N/A" : "OK"
  );
  sendBLEText(text);
}

void sendBLECOM2() {
  char text[128];
  snprintf(text, sizeof(text),
    "[COM2]\r\n"
    "Frames=%lu Bytes=%lu\r\n",
    (unsigned long)com2FramesSent,
    (unsigned long)com2BytesSent
  );
  sendBLEText(text);
}

void sendBLEHelp() {
  const char* help =
    "[COMANDOS]\r\n"
    "status - Estado actual\r\n"
    "imu - Datos IMU\r\n"
    "com2 - Estadísticas COM2\r\n"
    "magcal status - Calibración magnetómetro\r\n"
    "yawoff <val> - Offset Yaw\r\n"
    "help - Este mensaje\r\n";
  sendBLEText(help);
}

// ============================================================================
// sendBLEPeriodicDiagnostic - CON PRECISIÓN H_ERR, V_ERR
// ============================================================================

void sendBLEPeriodicDiagnostic() {
  if (!bleReady || !bleConnected) {
    return;
  }

  uint32_t now = millis();

  if (now - lastBleDiagnosticMs < BLE_DIAGNOSTIC_PERIOD_MS) {
    return;
  }

  lastBleDiagnosticMs = now;

  char yawText[20];

  if (isnan(currentYaw)) {
    strcpy(yawText, "N/A");
  } else {
    snprintf(
      yawText,
      sizeof(yawText),
      "%.1f",
      currentYaw
    );
  }

  char hErrText[16] = "N/A";
  char vErrText[16] = "N/A";

  // Mostrar precisión solo si disponible y reciente (<5s)
  if (isfinite(gnsHorizErr) && (now - lastGnstMs) < 5000) {
    snprintf(hErrText, sizeof(hErrText), "%.3f", gnsHorizErr);
  }
  if (isfinite(gnsVertErr) && (now - lastGnstMs) < 5000) {
    snprintf(vErrText, sizeof(vErrText), "%.3f", gnsVertErr);
  }

  char text[640];

  snprintf(
    text,
    sizeof(text),
    "\r\n"
    "[DIAG]\r\n"
    "GNSS=%s FIX_IN=%d HAS=%s SOL=%s SAT=%d HDOP=%s\r\n"
    "H_ERR=%sm V_ERR=%sm\r\n"
    "STATE=%s LOCK=%s SPD=%.3f m/s\r\n"
    "IMU=%s YAW=%s MAGCAL=%s\r\n"
    "RAW=%.8f,%.8f\r\n"
    "OUT=%.8f,%.8f FIX_OUT=%d\r\n"
    "COM2 frames=%lu bytes=%lu\r\n",
    gnssValid ? "OK" : "FAIL",
    inputFixQuality,
    hasActive ? "ON" : "OFF",
    solutionTypeText(),
    satelliteCount,
    hdopText,
    hErrText,
    vErrText,
    movementStateText(),
    lockedValid ? "YES" : "NO",
    currentSpeedMS,
    imuAvailable ? "OK" : "FAIL",
    yawText,
    magCalValid ? "YES" : "NO",
    currentLat,
    currentLon,
    lastOutputLat,
    lastOutputLon,
    lastOutputFixQuality,
    (unsigned long)com2FramesSent,
    (unsigned long)com2BytesSent
  );

  sendBLEText(text);
}

// ============================================================================
// IMU Functions - Stubs
// ============================================================================

bool initializeICM20948() {
  return true;
}

double readYaw() {
  return NAN;
}

// ============================================================================
// Calibración Magnetómetro - Stubs
// ============================================================================

void loadMagCalibration() {
  uint32_t magic = 0;
  EEPROM.get(EEPROM_MAGCAL_ADDR, magic);
  
  if (magic == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_MAGCAL_ADDR, magCal);
    magCalValid = true;
    debugPrintf("[MAGCAL] Cargada desde EEPROM\n");
  } else {
    magCalValid = false;
    debugPrintf("[MAGCAL] No encontrada en EEPROM\n");
  }
}

void saveMagCalibration() {
  EEPROM.put(EEPROM_MAGCAL_ADDR, magCal);
  debugPrintf("[MAGCAL] Guardada en EEPROM\n");
}

void startMagCalibration() {
  magCalRunning = true;
  magCalSamples = 0;
  magMinX = 0; magMaxX = 0;
  magMinY = 0; magMaxY = 0;
  magMinZ = 0; magMaxZ = 0;
  debugPrintf("[MAGCAL] Calibración iniciada\n");
}

bool stopMagCalibration() {
  magCalRunning = false;
  if (magCalSamples > 0) {
    magCal.offsetX = -(magMaxX + magMinX) / 2.0;
    magCal.offsetY = -(magMaxY + magMinY) / 2.0;
    magCal.offsetZ = -(magMaxZ + magMinZ) / 2.0;
    magCalValid = true;
    saveMagCalibration();
    debugPrintf("[MAGCAL] Calibración completada\n");
    return true;
  }
  return false;
}

void resetMagCalibration() {
  magCal.offsetX = 0.0;
  magCal.offsetY = 0.0;
  magCal.offsetZ = 0.0;
  magCalValid = false;
  debugPrintf("[MAGCAL] Reset\n");
}

// ============================================================================
// COM2 Output
// ============================================================================

void transmitGCGGA() {
  if (!lockedValid) {
    return;
  }

  // Construir y enviar GCGGA
  char gcgga[256];
  
  int lat_deg = (int)lockedLat;
  double lat_min = (fabs(lockedLat) - fabs(lat_deg)) * 60.0;
  
  int lon_deg = (int)lockedLon;
  double lon_min = (fabs(lockedLon) - fabs(lon_deg)) * 60.0;
  
  snprintf(gcgga, sizeof(gcgga),
    "$GCGGA,%s,%02d%06.3f,%c,%03d%06.3f,%c,%d,%d,%.1f,%.3f,M,0.0,M,,*",
    utcTime,
    abs(lat_deg), lat_min, lockedLat >= 0 ? 'N' : 'S',
    abs(lon_deg), lon_min, lockedLon >= 0 ? 'E' : 'W',
    lastOutputFixQuality,
    satelliteCount,
    atof(hdopText),
    lockedAlt
  );

  uint8_t checksum = calculateNmeaChecksum(gcgga + 1);
  char checksumHex[3];
  snprintf(checksumHex, sizeof(checksumHex), "%02X", checksum);
  
  strcat(gcgga, checksumHex);
  strcat(gcgga, "\r\n");

  softSerialWriteString(gcgga, strlen(gcgga));

  lastOutputLat = lockedLat;
  lastOutputLon = lockedLon;
  lastOutputAlt = lockedAlt;
  lastOutputFixQuality = inputFixQuality;

  com2FramesSent++;
  com2LastFrameMs = millis();
}

// ============================================================================
// setup
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n[SETUP] Iniciando RS232-FWS-GPS Rev.2.7 BLE");

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  softSerialInit();

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  delay(500);

  if (!initializeICM20948()) {
    Serial.println("[IMU] Falla inicializando ICM-20948");
    imuAvailable = false;
  }

  loadMagCalibration();

  if (!initializeBLE()) {
    Serial.println("[BLE] Falla inicializando BLE");
    bleReady = false;
  }

  Serial.println("[SETUP] Inicialización completada");
}

// ============================================================================
// loop
// ============================================================================

void loop() {
  readGNSS();

  if (imuAvailable) {
    readYaw();
  }

  maintainBLE();
  readBLECommands();

  updateMovementState();
  updateLeds();

  uint32_t now = millis();
  if (now - lastOutputMs >= OUTPUT_PERIOD_MS_VAR) {
    transmitGCGGA();
    lastOutputMs = now;
  }

  sendBLEPeriodicDiagnostic();
}
