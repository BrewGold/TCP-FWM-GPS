/*
  RS232-FMW-GPS - Rev.4.0
  Adafruit Metro RP2040 (core Arduino-Pico de Earle Philhower)

  Funciones:
  - GNSS UM980 conectado a Serial1, D0/D1, 115200 baudios
  - Salida NMEA GCGGA por COM2 (D2, UART PIO TX-only) y por Ethernet TCP
  - IMU ICM-20948 por I2C
  - Corrección de posición por yaw y offset de antena
  - Promedio y bloqueo de posición con el vehículo parado
  - Ethernet W5500 con:
      * puerto NMEA: 15919
      * puerto diagnóstico: 15920
  - Diagnóstico TCP con comandos:
    status, imu, com2, help, magcal start, magcal stop,
    magcal reset, yawoff <-180..180>, freq <1..10>, diag on/off

  Notas:
  - Sin ArduinoBLE
  - COM2 usa SerialPIO 38400/8N1, TX-only en D2, sin cambiar el shield
  - Se conserva el comportamiento base de Rev.2.5 en HAS/fix 5
  - El diagnóstico TCP sustituye al BLE

  La precisión horizontal estimada se calcula como UERE × HDOP.

  CAMBIOS Rev.4.0:
  - Base completa Rev.3.0, adaptada exclusivamente a Metro RP2040.
  - COM2 en D2 mediante PIO, con buffer de trama y vaciado no bloqueante.
  - GNSS Serial1 D0/D1, IMU SDA/SCL, Ethernet W5500 SPI por ICSP
    (CS D10, SD CS D4) y LEDs D5/D6 sin cambios de cableado.
  - Switch RX/TX: D0=RX, D1=TX para el shield (GPIO1 RX, GPIO0 TX).
  - Calibración en flash interna mediante EEPROM emulada del core.
  - Cola de diagnóstico acotada; no se ejecutan comandos truncados.
  - Se conservan IP fija de respaldo 192.168.1.22 y lógica de Rev.2.8.

  LIBRERÍAS:
  - SparkFun ICM-20948 Arduino Library
  - Ethernet (W5500)
  - SerialPIO, EEPROM, Wire y SPI incluidos en Arduino-Pico
*/

#include <Arduino.h>
#if !defined(ARDUINO_ADAFRUIT_METRO_RP2040) || !defined(ARDUINO_ARCH_RP2040)
#error "Rev.4 requiere Adafruit Metro RP2040 con el core Arduino-Pico de Earle Philhower"
#endif
#include <Wire.h>
#include <EEPROM.h>
#include <SerialPIO.h>
#include "ICM_20948.h"
#include <Ethernet.h>
#include <hardware/sync.h>
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

#define MAGCAL_MAGIC                0x49434D33UL

#define OFFSET_M_DEFAULT            0.55
#define DECLINATION_DEG_DEFAULT     1.0
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

#define NMEA_TCP_PORT               15919
#define DIAG_TCP_PORT               15920
#define TCP_DIAGNOSTIC_PERIOD_MS    5000
#define TCP_COMMAND_BUFFER_SIZE     96
#define NMEA_CLIENT_COUNT           4

// ============================================================================
// PROTOTIPOS
// ============================================================================

void debugPrintf(const char* format, ...);

void initializeCOM2();
void serviceCOM2();

bool initializeEthernet();
void maintainEthernet();
void acceptTCPClients();
void serviceTCPClients();
void readTCPCommands();
void processTCPCommand(const char* command);
void sendDiagText(const char* text);
void sendDiagPrintf(const char* format, ...);
void serviceDiagQueue();
void sendDiagStatus();
void sendDiagIMU();
void sendDiagCOM2();
void sendDiagHelp();
void sendDiagPeriodicDiagnostic();

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

double estimatedAccuracyMeters();

// ============================================================================
// SERVIDORES TCP ETHERNET
// ============================================================================

byte ethernetMac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

EthernetServer nmeaServer(NMEA_TCP_PORT);
EthernetServer diagServer(DIAG_TCP_PORT);
EthernetClient nmeaClients[NMEA_CLIENT_COUNT];
EthernetClient diagClient;

bool ethernetReady = false;
uint32_t lastEthernetCheckMs = 0;
uint32_t lastTcpDiagnosticMs = 0;
bool tcpDiagnosticEnabled = true;
char tcpCommandBuffer[TCP_COMMAND_BUFFER_SIZE];
uint8_t tcpCommandIndex = 0;
bool tcpCommandOverflow = false;
char diagQueue[4096];
size_t diagQueueHead = 0;
size_t diagQueueLength = 0;

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
uint32_t com2FramesDropped = 0;
bool gnssStarted = false;
bool gnssReading = false;

// ============================================================================
// ESTADO TCP
// ============================================================================

// ============================================================================
// COM2 - UART PIO, 38400/8N1, TX-ONLY EN D2
// ============================================================================

SerialPIO com2Serial(COM2_TX_PIN, NOPIN);
static bool com2Ready = false;
char com2Frame[240];
size_t com2FrameLength = 0;
size_t com2FrameIndex = 0;

void initializeCOM2() {
  com2Serial.begin(COM2_BAUD, SERIAL_8N1);
  com2Ready = (bool)com2Serial;
  debugPrintf(
    "[COM2] D%d a %d bps, UART PIO TX-only: %s\n",
    COM2_TX_PIN,
    COM2_BAUD,
    com2Ready ? "OK" : "ERROR"
  );
}

void serviceCOM2() {
  if (!com2Ready || com2FrameLength == 0) {
    return;
  }

  int available = com2Serial.availableForWrite();
  while (available-- > 0 && com2FrameIndex < com2FrameLength) {
    if (com2Serial.write((uint8_t)com2Frame[com2FrameIndex]) != 1) {
      return;
    }
    com2FrameIndex++;
    com2BytesSent++;
  }

  if (com2FrameIndex == com2FrameLength) {
    com2FramesSent++;
    com2LastFrameMs = millis();
    com2FrameLength = 0;
    com2FrameIndex = 0;
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
double currentHdop = 1.0;

uint32_t lastGgaMs = 0;
uint32_t lastRmcMs = 0;
uint32_t lastPubxMs = 0;

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
  MAGCAL_MAGIC,
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
// PRECISIÓN ESTIMADA SEGÚN HDOP Y FIX
// ============================================================================

double estimatedAccuracyMeters() {
  /*
    Precisión horizontal aproximada = UERE típico * HDOP.

    UERE según tipo de solución:
      GPS autónomo (fix 1) : ~3.00 m
      DGPS (fix 2)         : ~1.00 m
      RTK (fix 4)          : ~0.02 m
      HAS (fix 5)          : ~0.20 m
      Desconocido          : ~5.00 m
  */
  double uere;

  switch (solutionType) {
    case SOL_RTK:
      uere = 0.02;
      break;

    case SOL_HAS:
      uere = 0.20;
      break;

    case SOL_DGPS:
      uere = 1.00;
      break;

    case SOL_GPS:
      uere = 3.00;
      break;

    default:
      uere = 5.00;
      break;
  }

  return uere * currentHdop;
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

  currentHdop = hdop;

  /*
    Rev.2.5: el UM980 refleja el tipo de solución en el campo
    de calidad del GGA de entrada:

      1 = GPS autónomo
      2 = DGPS
      4 = RTK fijo
      5 = HAS (PPP / Float)

    HAS solo se considera activo con fix 5.
  */
  switch (fixQuality) {
    case 5:
      hasActive = true;
      solutionType = SOL_HAS;
      break;

    case 4:
      hasActive = false;
      solutionType = SOL_RTK;
      break;

    case 2:
      hasActive = false;
      solutionType = SOL_DGPS;
      break;

    case 1:
      hasActive = false;
      solutionType = SOL_GPS;
      break;

    default:
      hasActive = false;
      solutionType = SOL_UNKNOWN;
      break;
  }

  currentLat = latitude;
  currentLon = longitude;
  currentAlt = altitude;

  gnssValid = true;
  lastGgaMs = millis();

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
// RECEPCIÓN GNSS
// ============================================================================

void processGnssLine(const char* line) {
  if (
    line == nullptr ||
    line[0] == '\0'
  ) {
    return;
  }

  if (
    strncmp(line, "$GPGGA", 6) == 0 ||
    strncmp(line, "$GNGGA", 6) == 0 ||
    strncmp(line, "$GCGGA", 6) == 0
  ) {
    parseGGA(line);
  } else if (
    strncmp(line, "$GPRMC", 6) == 0 ||
    strncmp(line, "$GNRMC", 6) == 0 ||
    strncmp(line, "$GCRMC", 6) == 0
  ) {
    parseRMC(line);
  }
}

void readGNSS() {
  if (!gnssStarted || gnssReading || __get_current_exception() != 0) {
    return;
  }

  // El parser puede escribir diagnóstico USB y volver a invocar yield().
  gnssReading = true;
  while (Serial1.available() > 0) {
    char character = (char)Serial1.read();

    if (
      character == '$' ||
      character == '#'
    ) {
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

    if (
      gnssLineIndex <
      (int)sizeof(gnssLine) - 1
    ) {
      gnssLine[gnssLineIndex++] = character;
    } else {
      Serial.println("[GNSS] Línea demasiado larga");
      gnssLineIndex = 0;
    }
  }
  gnssReading = false;
}

void yield() {
  // Ethernet libera SPI antes de yield(); no acceder a SPI desde aquí.
  readGNSS();
  serviceCOM2();
}

// ============================================================================
// FLASH - CALIBRACIÓN MAGNETÓMETRO
// ============================================================================

void loadMagCalibration() {
  EEPROM.begin(sizeof(MagCalibration));
  MagCalibration stored;
  EEPROM.get(0, stored);

  if (
    stored.magic == MAGCAL_MAGIC &&
    isfinite(stored.offsetX) &&
    isfinite(stored.offsetY) &&
    isfinite(stored.offsetZ) &&
    isfinite(stored.yawMountOffset)
  ) {
    magCal = stored;
    magCalValid = true;

    YAW_MOUNT_OFFSET_DEG_VAR =
      magCal.yawMountOffset;

    debugPrintf(
      "[IMU] Calibración cargada: "
      "off=(%.2f, %.2f, %.2f) yawoff=%.1f\n",
      magCal.offsetX,
      magCal.offsetY,
      magCal.offsetZ,
      magCal.yawMountOffset
    );
  } else {
    magCal.magic = MAGCAL_MAGIC;
    magCal.offsetX = 0.0f;
    magCal.offsetY = 0.0f;
    magCal.offsetZ = 0.0f;
    magCal.yawMountOffset = 0.0f;

    magCalValid = false;

    Serial.println(
      "[IMU] Sin calibración en flash"
    );
  }
}

void saveMagCalibration() {
  magCal.magic = MAGCAL_MAGIC;

  magCal.yawMountOffset =
    (float)YAW_MOUNT_OFFSET_DEG_VAR;

  EEPROM.put(0, magCal);
  if (!EEPROM.commit()) {
    Serial.println("[IMU] Error guardando calibracion");
    return;
  }

  Serial.println(
    "[IMU] Calibración guardada en flash"
  );
}

void startMagCalibration() {
  magCalRunning = true;
  magCalSamples = 0;

  magMinX = 1e9;
  magMinY = 1e9;
  magMinZ = 1e9;

  magMaxX = -1e9;
  magMaxY = -1e9;
  magMaxZ = -1e9;

  Serial.println("[IMU] Calibración iniciada");
}

bool stopMagCalibration() {
  magCalRunning = false;

  if (magCalSamples < 200) {
    return false;
  }

  double spanX = magMaxX - magMinX;
  double spanY = magMaxY - magMinY;

  if (
    spanX < 10.0 ||
    spanY < 10.0
  ) {
    return false;
  }

  magCal.offsetX =
    (float)((magMaxX + magMinX) / 2.0);

  magCal.offsetY =
    (float)((magMaxY + magMinY) / 2.0);

  magCal.offsetZ =
    (float)((magMaxZ + magMinZ) / 2.0);

  magCalValid = true;
  imuFilterSeeded = false;

  saveMagCalibration();

  return true;
}

void resetMagCalibration() {
  magCalRunning = false;

  magCal.offsetX = 0.0f;
  magCal.offsetY = 0.0f;
  magCal.offsetZ = 0.0f;

  magCalValid = false;
  imuFilterSeeded = false;

  saveMagCalibration();
}

// ============================================================================
// ICM-20948
// ============================================================================

bool probeI2C(uint8_t address) {
  Wire.beginTransmission(address);

  return Wire.endTransmission() == 0;
}

bool initializeICM20948() {
  uint8_t address = 0;

  if (probeI2C(ICM_ADDRESS_PRIMARY)) {
    address = ICM_ADDRESS_PRIMARY;
  } else if (probeI2C(ICM_ADDRESS_SECONDARY)) {
    address = ICM_ADDRESS_SECONDARY;
  } else {
    Serial.println(
      "[IMU] ICM-20948 no responde "
      "en 0x69 ni 0x68"
    );

    imuAvailable = false;
    return false;
  }

  icm.begin(
    Wire,
    address == ICM_ADDRESS_PRIMARY ? 1 : 0
  );

  if (icm.status != ICM_20948_Stat_Ok) {
    debugPrintf(
      "[IMU] Error inicializando: %s\n",
      icm.statusString()
    );

    imuAvailable = false;
    return false;
  }

  imuAddress = address;
  imuAvailable = true;
  imuFilterSeeded = false;
  fusedHeading = NAN;
  currentYaw = NAN;

  lastImuDataMs = millis();
  lastImuMicros = micros();

  debugPrintf(
    "[IMU] ICM-20948 inicializado en 0x%02X\n",
    imuAddress
  );

  return true;
}

double computeTiltCompensatedHeading() {
  double ax = accFX;
  double ay = accFY;
  double az = accFZ;

  double norm = sqrt(
    ax * ax +
    ay * ay +
    az * az
  );

  if (norm < 100.0) {
    return NAN;
  }

  imuRoll = atan2(ay, az);

  imuPitch = atan2(
    -ax,
    sqrt(ay * ay + az * az)
  );

  double mx = magFX;
  double my = magFY;
  double mz = magFZ;

  double sinRoll = sin(imuRoll);
  double cosRoll = cos(imuRoll);
  double sinPitch = sin(imuPitch);
  double cosPitch = cos(imuPitch);

  double xh =
    mx * cosPitch +
    my * sinRoll * sinPitch +
    mz * cosRoll * sinPitch;

  double yh =
    my * cosRoll -
    mz * sinRoll;

  if (
    fabs(xh) < 1e-6 &&
    fabs(yh) < 1e-6
  ) {
    return NAN;
  }

  double heading =
    radiansToDegrees(atan2(-yh, xh));

  return normalizeAngle(heading);
}

double readYaw() {
  uint32_t now = millis();

  if (!imuAvailable) {
    if (
      now - lastImuRetryMs >=
      IMU_RETRY_MS
    ) {
      lastImuRetryMs = now;

      Serial.println(
        "[IMU] Reintentando ICM-20948"
      );

      initializeICM20948();
    }

    return NAN;
  }

  if (icm.dataReady()) {
    icm.getAGMT();

    if (
      icm.status !=
      ICM_20948_Stat_Ok
    ) {
      debugPrintf(
        "[IMU] Error de lectura: %s\n",
        icm.statusString()
      );

      imuAvailable = false;
      currentYaw = NAN;

      return NAN;
    }

    double rawAx = icm.accX();
    double rawAy = icm.accY();
    double rawAz = icm.accZ();

    double rawMx = icm.magX();
    double rawMy = -icm.magY();
    double rawMz = -icm.magZ();

    double rawGz = icm.gyrZ();

    imuTempC = icm.temp();

    if (magCalRunning) {
      if (rawMx < magMinX) magMinX = rawMx;
      if (rawMx > magMaxX) magMaxX = rawMx;

      if (rawMy < magMinY) magMinY = rawMy;
      if (rawMy > magMaxY) magMaxY = rawMy;

      if (rawMz < magMinZ) magMinZ = rawMz;
      if (rawMz > magMaxZ) magMaxZ = rawMz;

      magCalSamples++;
    }

    rawMx -= magCal.offsetX;
    rawMy -= magCal.offsetY;
    rawMz -= magCal.offsetZ;

    if (!imuFilterSeeded) {
      accFX = rawAx;
      accFY = rawAy;
      accFZ = rawAz;

      magFX = rawMx;
      magFY = rawMy;
      magFZ = rawMz;
    } else {
      accFX +=
        ACC_FILTER_ALPHA * (rawAx - accFX);

      accFY +=
        ACC_FILTER_ALPHA * (rawAy - accFY);

      accFZ +=
        ACC_FILTER_ALPHA * (rawAz - accFZ);

      magFX +=
        MAG_FILTER_ALPHA * (rawMx - magFX);

      magFY +=
        MAG_FILTER_ALPHA * (rawMy - magFY);

      magFZ +=
        MAG_FILTER_ALPHA * (rawMz - magFZ);
    }

    gyroZdps = rawGz;

    uint32_t nowUs = micros();

    double dt =
      (double)(nowUs - lastImuMicros) /
      1000000.0;

    lastImuMicros = nowUs;

    magHeading =
      computeTiltCompensatedHeading();

    if (!isnan(magHeading)) {
      if (
        !imuFilterSeeded ||
        isnan(fusedHeading) ||
        dt <= 0.0 ||
        dt > GYRO_MAX_DT_S
      ) {
        fusedHeading = magHeading;
        imuFilterSeeded = true;
      } else {
        double predicted =
          normalizeAngle(
            fusedHeading - gyroZdps * dt
          );

        double error =
          wrap180(magHeading - predicted);

        fusedHeading =
          normalizeAngle(
            predicted +
            (1.0 - YAW_GYRO_WEIGHT) *
            error
          );
      }

      double magneticYaw =
        normalizeAngle(
          fusedHeading +
          YAW_MOUNT_OFFSET_DEG_VAR
        );

      currentYaw =
        normalizeAngle(
          magneticYaw -
          DECLINATION_DEG_VAR
        );

      lastImuDataMs = now;
    }

    return currentYaw;
  }

  if (
    now - lastImuDataMs >=
    IMU_TIMEOUT_MS
  ) {
    if (
      now - lastImuMessageMs >=
      IMU_RETRY_MS
    ) {
      lastImuMessageMs = now;

      Serial.println(
        "[IMU] Timeout ICM-20948"
      );
    }

    currentYaw = NAN;
    imuAvailable = false;
    imuFilterSeeded = false;
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
    normalizeAngle(yaw + 270.0);

  double bearingRadians =
    degreesToRadians(bearing);

  double offsetRadians =
    OFFSET_M_VAR / EARTH_RADIUS_M;

  double latitudeOffset =
    offsetRadians *
    cos(bearingRadians);

  double latitudeCosine =
    cos(degreesToRadians(rawLat));

  if (fabs(latitudeCosine) < 0.000001) {
    return;
  }

  double longitudeOffset =
    offsetRadians *
    sin(bearingRadians) /
    latitudeCosine;

  *correctedLat =
    rawLat +
    radiansToDegrees(latitudeOffset);

  *correctedLon =
    rawLon +
    radiansToDegrees(longitudeOffset);
}

// ============================================================================
// PRUEBA DE ARRANQUE DE LEDS
// ============================================================================

void startupLedTest() {
  uint32_t startMs = millis();

  Serial.println(
    "[LED] Prueba de arranque 5 segundos"
  );

  while (
    millis() - startMs <
    STARTUP_LED_TEST_MS
  ) {
    uint32_t elapsedMs =
      millis() - startMs;

    bool state =
      (elapsedMs % STARTUP_LED_PERIOD_MS) <
      (STARTUP_LED_PERIOD_MS / 2);

    digitalWrite(
      LED1_PIN,
      state ? HIGH : LOW
    );

    digitalWrite(
      LED2_PIN,
      state ? HIGH : LOW
    );

    delay(10);
  }

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  Serial.println(
    "[LED] Fin prueba de arranque"
  );
}

// ============================================================================
// ESTADO NORMAL DE LEDS
// ============================================================================

void updateLeds() {
  uint32_t now = millis();

  /*
    LED1 / D5 (Rev.2.5):
    - Apagado        : sin GNSS válido.
    - Parpadeo 200ms : IMU no disponible.
    - Parpadeo 600ms : GNSS válido sin HAS (fix 1/2/4).
    - Fijo           : HAS activo (fix 5).
  */
  if (!gnssValid) {
    digitalWrite(LED1_PIN, LOW);
  } else if (!imuAvailable) {
    digitalWrite(
      LED1_PIN,
      (now % 200) < 100 ? HIGH : LOW
    );
  } else if (!hasActive) {
    // Fix 1/2/4 sin HAS: parpadeo, nunca fijo.
    digitalWrite(
      LED1_PIN,
      (now % 600) < 300 ? HIGH : LOW
    );
  } else {
    // HAS activo (fix 5): LED fijo.
    digitalWrite(LED1_PIN, HIGH);
  }

  // LED2 / D6.
  if (movementState == MOVING) {
    digitalWrite(LED2_PIN, LOW);
  } else if (movementState == AVERAGING) {
    digitalWrite(
      LED2_PIN,
      (now % 400) < 200 ? HIGH : LOW
    );
  } else if (
    movementState == LOCKED &&
    lockedValid
  ) {
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
  if (!gnssValid) {
    return 0;
  }

  if (
    movementState == LOCKED &&
    lockedValid
  ) {
    return 4;
  }

  if (hasActive) {
    return 2;
  }

  return 1;
}

void updateMovementState() {
  uint32_t now = millis();

  bool speedFresh =
    (now - lastRmcMs) <=
    SPEED_FRESHNESS_MS;

  bool ggaFresh =
    (now - lastGgaMs) <=
    GGA_FRESHNESS_MS;

  if (!speedFresh) {
    currentSpeedMS = 0.0;
  }

  if (!ggaFresh) {
    gnssValid = false;
    lockedValid = false;
  }

  switch (movementState) {
    case MOVING: {
      if (
        speedFresh &&
        currentSpeedMS <
        SPEED_ENTER_STOP_MS_VAR
      ) {
        if (lastStopCheckMs == 0) {
          lastStopCheckMs = now;
        }
      } else if (
        speedFresh &&
        currentSpeedMS >=
        SPEED_EXIT_STOP_MS_VAR
      ) {
        lastStopCheckMs = 0;
      }

      if (
        lastStopCheckMs > 0 &&
        now - lastStopCheckMs >=
        STOP_CONFIRMATION_MS
      ) {
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
      if (
        gnssValid &&
        lastGgaMs != lastSampledGgaMs &&
        sampleCount < MAX_SAMPLES
      ) {
        latitudeSamples[sampleCount] =
          currentLat;

        longitudeSamples[sampleCount] =
          currentLon;

        altitudeSamples[sampleCount] =
          currentAlt;

        sampleCount++;
        lastSampledGgaMs = lastGgaMs;

        if (
          imuAvailable &&
          !isnan(currentYaw) &&
          yawSampleCount < MAX_SAMPLES
        ) {
          yawSamples[yawSampleCount++] =
            currentYaw;
        }
      }

      if (
        speedFresh &&
        currentSpeedMS >
        SPEED_EXIT_STOP_MS_VAR
      ) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;

        clearAverageBuffers();

        Serial.println(
          "[STATE] AVERAGING -> MOVING"
        );

        break;
      }

      if (
        now - averagingStartMs >=
        AVERAGING_WINDOW_MS_VAR &&
        sampleCount > 0
      ) {
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

        double averageYaw = NAN;

        if (yawSampleCount > 0) {
          averageYaw =
            circularMeanDegrees(
              yawSamples,
              yawSampleCount
            );
        }

        if (
          isnan(averageRawLat) ||
          isnan(averageRawLon) ||
          isnan(averageAlt)
        ) {
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
          "raw=%.8f,%.8f "
          "out=%.8f,%.8f "
          "yaw=%.1f\n",
          rawLockedLat,
          rawLockedLon,
          lockedLat,
          lockedLon,
          lockedYaw
        );

        sendDiagStatus();
      }

      break;
    }

    case LOCKED: {
      if (
        speedFresh &&
        currentSpeedMS >
        SPEED_EXIT_STOP_MS_VAR
      ) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;

        clearAverageBuffers();

        Serial.println(
          "[STATE] LOCKED -> MOVING velocidad"
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
            "distancia=%.2f m\n",
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

  snprintf(
    output,
    outputSize,
    "%02d%07.4f",
    degrees,
    minutes
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

  snprintf(
    output,
    outputSize,
    "%03d%07.4f",
    degrees,
    minutes
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

  snprintf(
    altitudeText,
    sizeof(altitudeText),
    "%.1f",
    altitude
  );

  snprintf(
    output,
    outputSize,
    "$GCGGA,%s,%s,%c,%s,%c,"
    "%d,%02d,%s,%s,M,0.0,M,,",
    utcTime,
    latitudeText,
    latitudeHemisphere,
    longitudeText,
    longitudeHemisphere,
    fixQuality,
    satelliteCount,
    hdopText,
    altitudeText
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

  if (length + 5 >= sentenceSize) {
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
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  Ethernet.init(ETHERNET_CS_PIN);

  Serial.println(
    "[ETH] Inicializando Ethernet"
  );

  int dhcpResult =
    Ethernet.begin(ethernetMac);

  if (dhcpResult == 0) {
    Serial.println(
      "[ETH] DHCP no disponible, usando IP fija: 192.168.1.22"
    );

    IPAddress fallbackIP(192, 168, 1, 22);
    IPAddress dns(192, 168, 1, 1);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 255, 0);
    Ethernet.begin(
      ethernetMac,
      fallbackIP,
      dns,
      gateway,
      subnet
    );
  } else {
    Serial.print("[ETH] IP asignada por DHCP: ");
    Serial.println(Ethernet.localIP());
  }

  nmeaServer.begin();
  diagServer.begin();
  Serial.print("[ETH] Servidor NMEA TCP en puerto ");
  Serial.println(NMEA_TCP_PORT);
  Serial.print("[ETH] Servidor de diagnostico TCP en puerto ");
  Serial.println(DIAG_TCP_PORT);

  ethernetReady = true;

  return true;
}

void maintainEthernet() {
  uint32_t now = millis();

  if (
    now - lastEthernetCheckMs <
    5000
  ) {
    return;
  }

  lastEthernetCheckMs = now;

  if (
    Ethernet.hardwareStatus() ==
    EthernetNoHardware
  ) {
    ethernetReady = false;

    Serial.println(
      "[ETH] Shield no detectado"
    );

    return;
  }

  if (
    Ethernet.linkStatus() ==
    LinkOFF
  ) {
    ethernetReady = false;

    Serial.println(
      "[ETH] Cable desconectado"
    );

    return;
  }

  ethernetReady = true;
}

void acceptTCPClients() {
  if (!ethernetReady) {
    return;
  }

  EthernetClient incomingNmea = nmeaServer.accept();

  if (incomingNmea) {
    bool assigned = false;

    for (uint8_t i = 0; i < NMEA_CLIENT_COUNT; i++) {
      if (!nmeaClients[i].connected()) {
        nmeaClients[i].stop();
        nmeaClients[i] = incomingNmea;
        assigned = true;
        break;
      }
    }

    if (!assigned) {
      incomingNmea.stop();
    }
  }

  if (!diagClient || !diagClient.connected()) {
    diagClient.stop();
    diagQueueHead = 0;
    diagQueueLength = 0;
    diagClient = diagServer.accept();

    if (diagClient) {
      tcpCommandIndex = 0;
      tcpCommandOverflow = false;
      lastTcpDiagnosticMs = millis();
      sendDiagText(
        "\r\nFWD-GPS Rev.4.0 TCP diagnostic\r\n"
        "Type 'help' for commands\r\n"
      );
    }
  } else {
    EthernetClient extraDiagClient = diagServer.accept();

    if (extraDiagClient) {
      extraDiagClient.stop();
    }
  }
}

void serviceTCPClients() {
  acceptTCPClients();

  for (uint8_t i = 0; i < NMEA_CLIENT_COUNT; i++) {
    if (!nmeaClients[i].connected()) {
      nmeaClients[i].stop();
    }
  }

  readTCPCommands();
  serviceDiagQueue();
}

void transmitEthernet(const char* sentence) {
  if (
    !ethernetReady ||
    sentence == nullptr
  ) {
    return;
  }

  size_t length = strlen(sentence);

  for (uint8_t i = 0; i < NMEA_CLIENT_COUNT; i++) {
    readGNSS();
    if (
      nmeaClients[i].connected() &&
      (
        nmeaClients[i].availableForWrite() < (int)length ||
        nmeaClients[i].write(
          (const uint8_t*)sentence,
          length
        ) != length
      )
    ) {
      nmeaClients[i].stop();
    }
  }
}

void sendDiagText(const char* text) {
  if (
    !diagClient ||
    !diagClient.connected() ||
    text == nullptr
  ) {
    return;
  }

  size_t length = strlen(text);
  if (length > sizeof(diagQueue) - diagQueueLength) {
    diagClient.stop();
    diagQueueHead = 0;
    diagQueueLength = 0;
    return;
  }
  for (size_t i = 0; i < length; i++) {
    diagQueue[(diagQueueHead + diagQueueLength + i) % sizeof(diagQueue)] = text[i];
  }
  diagQueueLength += length;
  readGNSS();
}

void sendDiagPrintf(const char* format, ...) {
  char text[1536];
  va_list arguments;
  va_start(arguments, format);
  int length = vsnprintf(text, sizeof(text), format, arguments);
  va_end(arguments);
  if (length < 0 || (size_t)length >= sizeof(text)) {
    sendDiagText("ERROR: diagnostico demasiado largo\r\n");
    return;
  }
  sendDiagText(text);
}

void serviceDiagQueue() {
  if (!diagClient || !diagClient.connected()) {
    diagQueueHead = 0;
    diagQueueLength = 0;
    return;
  }
  if (diagQueueLength == 0) {
    return;
  }
  int available = diagClient.availableForWrite();
  if (available <= 0) {
    return;
  }
  size_t length = diagQueueLength;
  if (length > sizeof(diagQueue) - diagQueueHead) {
    length = sizeof(diagQueue) - diagQueueHead;
  }
  if (length > (size_t)available) {
    length = (size_t)available;
  }
  size_t written = diagClient.write((const uint8_t*)diagQueue + diagQueueHead, length);
  diagQueueHead = (diagQueueHead + written) % sizeof(diagQueue);
  diagQueueLength -= written;
  readGNSS();
}

// ============================================================================
// TCP - DIAGNÓSTICO STATUS
// ============================================================================

void sendDiagStatus() {
  if (!diagClient || !diagClient.connected()) {
    return;
  }

  char yawText[24];
  char lockedYawText[24];

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

  if (isnan(lockedYaw)) {
    strcpy(lockedYawText, "N/A");
  } else {
    snprintf(
      lockedYawText,
      sizeof(lockedYawText),
      "%.1f",
      lockedYaw
    );
  }

  sendDiagPrintf(
    "\r\n"
    "========== STATUS ==========\r\n"
    "GNSS: %s\r\n"
    "GGA entrada fix: %d\r\n"
    "HAS: %s\r\n"
    "Solucion: %s\r\n"
    "Satelites: %d\r\n"
    "HDOP: %s\r\n"
    "Precision est: %.2f m\r\n"
    "Hora UTC: %s\r\n"
    "\r\n"
    "Entrada GNSS:\r\n"
    "LAT: %.8f\r\n"
    "LON: %.8f\r\n"
    "ALT: %.2f m\r\n"
    "Velocidad: %.3f m/s\r\n"
    "Curso: %.1f deg\r\n"
    "\r\n"
    "IMU: %s\r\n"
    "Yaw actual: %s deg\r\n"
    "Yaw montaje: %.1f deg\r\n"
    "Declinacion: %.1f deg\r\n"
    "MagCal: %s\r\n"
    "\r\n"
    "Estado: %s\r\n"
    "Bloqueado: %s\r\n"
    "Muestras posicion: %d\r\n"
    "Muestras yaw: %d\r\n"
    "Yaw bloqueado: %s deg\r\n"
    "\r\n"
    "Salida COM2:\r\n"
    "LAT: %.8f\r\n"
    "LON: %.8f\r\n"
    "ALT: %.2f m\r\n"
    "Fix salida: %d\r\n"
    "Offset antena: %.2f m\r\n"
    "Frecuencia: %d Hz\r\n"
    "Tramas: %lu\r\n"
    "Bytes: %lu\r\n"
    "\r\n"
    "Ethernet TCP: NMEA %d / diagnostico %d (%s)\r\n"
    "============================\r\n",
    gnssValid ? "OK" : "FAIL",
    inputFixQuality,
    hasActive ? "ACTIVO" : "NO",
    solutionTypeText(),
    satelliteCount,
    hdopText,
    estimatedAccuracyMeters(),
    utcTime,
    currentLat,
    currentLon,
    currentAlt,
    currentSpeedMS,
    currentCourse,
    imuAvailable ? "OK" : "FAIL",
    yawText,
    YAW_MOUNT_OFFSET_DEG_VAR,
    DECLINATION_DEG_VAR,
    magCalValid ? "SI" : "NO",
    movementStateText(),
    lockedValid ? "SI" : "NO",
    sampleCount,
    yawSampleCount,
    lockedYawText,
    lastOutputLat,
    lastOutputLon,
    lastOutputAlt,
    lastOutputFixQuality,
    OFFSET_M_VAR,
    1000 / OUTPUT_PERIOD_MS_VAR,
    (unsigned long)com2FramesSent,
    (unsigned long)com2BytesSent,
    NMEA_TCP_PORT,
    DIAG_TCP_PORT,
    ethernetReady ? "OK" : "FAIL"
  );

}

// ============================================================================
// TCP - DIAGNÓSTICO IMU
// ============================================================================

void sendDiagIMU() {
  if (!diagClient || !diagClient.connected()) {
    return;
  }

  sendDiagPrintf(
    "\r\n"
    "======== ICM-20948 ========\r\n"
    "Estado: %s\r\n"
    "Direccion: 0x%02X\r\n"
    "\r\n"
    "ACC mg:\r\n"
    "X: %.2f\r\n"
    "Y: %.2f\r\n"
    "Z: %.2f\r\n"
    "\r\n"
    "MAG uT corregido:\r\n"
    "X: %.2f\r\n"
    "Y: %.2f\r\n"
    "Z: %.2f\r\n"
    "\r\n"
    "Gyro Z: %.2f dps\r\n"
    "Roll: %.2f deg\r\n"
    "Pitch: %.2f deg\r\n"
    "Rumbo magnetico: %.2f deg\r\n"
    "Rumbo fusionado: %.2f deg\r\n"
    "Yaw final: %.2f deg\r\n"
    "Temperatura: %.2f C\r\n"
    "\r\n"
    "MagCal valida: %s\r\n"
    "MagCal activa: %s\r\n"
    "Muestras MagCal: %lu\r\n"
    "Offset X: %.2f\r\n"
    "Offset Y: %.2f\r\n"
    "Offset Z: %.2f\r\n"
    "===========================\r\n",
    imuAvailable ? "OK" : "FAIL",
    imuAddress,
    accFX,
    accFY,
    accFZ,
    magFX,
    magFY,
    magFZ,
    gyroZdps,
    radiansToDegrees(imuRoll),
    radiansToDegrees(imuPitch),
    isnan(magHeading) ? -1.0 : magHeading,
    isnan(fusedHeading) ? -1.0 : fusedHeading,
    isnan(currentYaw) ? -1.0 : currentYaw,
    imuTempC,
    magCalValid ? "SI" : "NO",
    magCalRunning ? "SI" : "NO",
    (unsigned long)magCalSamples,
    magCal.offsetX,
    magCal.offsetY,
    magCal.offsetZ
  );

}

// ============================================================================
// TCP - DIAGNÓSTICO COM2
// ============================================================================

void sendDiagCOM2() {
  if (!diagClient || !diagClient.connected()) {
    return;
  }

  uint32_t now = millis();

  sendDiagPrintf(
    "\r\n"
    "=========== COM2 ===========\r\n"
    "Pin TX: D%d\r\n"
    "Velocidad: %d baud\r\n"
    "Modo: UART PIO TX-only 8N1\r\n"
    "UART: %s\r\n"
    "Tramas descartadas: %lu\r\n"
    "Espacio buffer TX: %u bytes\r\n"
    "Periodo: %u ms\r\n"
    "Frecuencia: %d Hz\r\n"
    "Tramas enviadas: %lu\r\n"
    "Bytes enviados: %lu\r\n"
    "Ultima trama hace: %lu ms\r\n"
    "Fix ultima salida: %d\r\n"
    "LAT salida: %.8f\r\n"
    "LON salida: %.8f\r\n"
    "ALT salida: %.2f m\r\n"
    "GNSS valido: %s\r\n"
    "Estado: %s\r\n"
    "Bloqueado: %s\r\n"
    "============================\r\n",
    COM2_TX_PIN,
    COM2_BAUD,
    com2Ready ? "OK" : "ERROR",
    (unsigned long)com2FramesDropped,
    (unsigned int)(sizeof(com2Frame) - com2FrameLength),
    OUTPUT_PERIOD_MS_VAR,
    1000 / OUTPUT_PERIOD_MS_VAR,
    (unsigned long)com2FramesSent,
    (unsigned long)com2BytesSent,
    com2LastFrameMs > 0
      ? (unsigned long)(now - com2LastFrameMs)
      : 0UL,
    lastOutputFixQuality,
    lastOutputLat,
    lastOutputLon,
    lastOutputAlt,
    gnssValid ? "SI" : "NO",
    movementStateText(),
    lockedValid ? "SI" : "NO"
  );

}

// ============================================================================
// TCP - AYUDA
// ============================================================================

void sendDiagHelp() {
  const char* help =
    "\r\n"
    "======= COMANDOS TCP =======\r\n"
    "status\r\n"
    "  Estado general completo.\r\n"
    "\r\n"
    "imu\r\n"
    "  Datos del ICM-20948.\r\n"
    "\r\n"
    "com2\r\n"
    "  Estado y contadores COM2.\r\n"
    "\r\n"
    "magcal start\r\n"
    "  Inicia calibracion.\r\n"
    "\r\n"
    "magcal stop\r\n"
    "  Finaliza y guarda.\r\n"
    "\r\n"
    "magcal reset\r\n"
    "  Borra calibracion.\r\n"
    "\r\n"
    "yawoff <grados>\r\n"
    "  Ajuste montaje -180..180.\r\n"
    "\r\n"
    "freq <1..10>\r\n"
    "  Frecuencia de salida NMEA/COM2 en Hz.\r\n"
    "\r\n"
    "diag on|off\r\n"
    "  Activa o desactiva el bloque periodico [DIAG].\r\n"
    "\r\n"
    "help\r\n"
    "  Muestra esta ayuda.\r\n"
    "============================\r\n";

  sendDiagText(help);
}

// ============================================================================
// TCP - COMANDOS
// ============================================================================

void processTCPCommand(const char* commandInput) {
  if (
    commandInput == nullptr ||
    commandInput[0] == '\0'
  ) {
    return;
  }

  char command[TCP_COMMAND_BUFFER_SIZE];

  strncpy(
    command,
    commandInput,
    sizeof(command) - 1
  );

  command[sizeof(command) - 1] = '\0';

  // Eliminar espacios iniciales.
  char* start = command;

  while (
    *start != '\0' &&
    isspace((unsigned char)*start)
  ) {
    start++;
  }

  // Eliminar espacios finales.
  size_t length = strlen(start);

  while (
    length > 0 &&
    isspace((unsigned char)start[length - 1])
  ) {
    start[length - 1] = '\0';
    length--;
  }

  // Convertir a minúsculas.
  for (size_t i = 0; start[i] != '\0'; i++) {
    start[i] =
      (char)tolower((unsigned char)start[i]);
  }

  Serial.print("[TCP CMD] ");
  Serial.println(start);

  if (strcmp(start, "status") == 0) {
    sendDiagStatus();
    return;
  }

  if (strcmp(start, "imu") == 0) {
    sendDiagIMU();
    return;
  }

  if (strcmp(start, "com2") == 0) {
    sendDiagCOM2();
    return;
  }

  if (strcmp(start, "help") == 0) {
    sendDiagHelp();
    return;
  }

  if (strcmp(start, "magcal start") == 0) {
    if (!imuAvailable) {
      sendDiagText(
        "ERROR: IMU no disponible\r\n"
      );

      return;
    }

    startMagCalibration();

    sendDiagText(
      "OK: calibracion iniciada.\r\n"
      "Gira lentamente el equipo en todos los ejes.\r\n"
      "Despues envia: magcal stop\r\n"
    );

    return;
  }

  if (strcmp(start, "magcal stop") == 0) {
    if (!magCalRunning) {
      sendDiagText(
        "ERROR: calibracion no iniciada\r\n"
      );

      return;
    }

    if (stopMagCalibration()) {
      char response[240];

      snprintf(
        response,
        sizeof(response),
        "OK: calibracion guardada\r\n"
        "Muestras: %lu\r\n"
        "Offset: %.2f, %.2f, %.2f\r\n",
        (unsigned long)magCalSamples,
        magCal.offsetX,
        magCal.offsetY,
        magCal.offsetZ
      );

      sendDiagText(response);
    } else {
      sendDiagText(
        "ERROR: calibracion insuficiente.\r\n"
        "Repite girando el equipo completamente.\r\n"
      );
    }

    return;
  }

  if (strcmp(start, "magcal reset") == 0) {
    resetMagCalibration();

    sendDiagText(
      "OK: calibracion borrada\r\n"
    );

    return;
  }

  if (strncmp(start, "yawoff ", 7) == 0) {
    char* endPointer = nullptr;

    double value =
      strtod(start + 7, &endPointer);

    if (
      endPointer == start + 7 ||
      *endPointer != '\0' ||
      !isfinite(value) ||
      value < -180.0 ||
      value > 180.0
    ) {
      sendDiagText(
        "ERROR: usa yawoff -180..180\r\n"
      );

      return;
    }

    YAW_MOUNT_OFFSET_DEG_VAR = value;
    saveMagCalibration();

    imuFilterSeeded = false;
    fusedHeading = NAN;

    char response[120];

    snprintf(
      response,
      sizeof(response),
      "OK: yawoff=%.1f grados\r\n",
      YAW_MOUNT_OFFSET_DEG_VAR
    );

    sendDiagText(response);

    return;
  }

  if (strncmp(start, "freq ", 5) == 0) {
    char* endPointer = nullptr;
    long frequency = strtol(start + 5, &endPointer, 10);

    if (
      endPointer == start + 5 ||
      *endPointer != '\0' ||
      frequency < 1 ||
      frequency > 10
    ) {
      sendDiagText("ERROR: usa freq 1..10\r\n");
      return;
    }

    OUTPUT_PERIOD_MS_VAR = 1000 / (uint16_t)frequency;

    char response[96];
    snprintf(
      response,
      sizeof(response),
      "OK: salida NMEA/COM2 a %ld Hz (%u ms)\r\n",
      frequency,
      OUTPUT_PERIOD_MS_VAR
    );
    sendDiagText(response);
    return;
  }

  if (strcmp(start, "diag on") == 0) {
    tcpDiagnosticEnabled = true;
    lastTcpDiagnosticMs = millis();
    sendDiagText("OK: diagnostico periodico activado\r\n");
    return;
  }

  if (strcmp(start, "diag off") == 0) {
    tcpDiagnosticEnabled = false;
    sendDiagText("OK: diagnostico periodico desactivado\r\n");
    return;
  }

  sendDiagText(
    "ERROR: comando desconocido\r\n"
    "Escribe: help\r\n"
  );
}

void readTCPCommands() {
  if (
    !diagClient ||
    !diagClient.connected()
  ) {
    return;
  }

  uint8_t processed = 0;

  while (diagClient.available() > 0 && processed < 64) {
    char character = (char)diagClient.read();
    processed++;

    if (
      character == '\r' ||
      character == '\n'
    ) {
      if (tcpCommandOverflow) {
        tcpCommandOverflow = false;
        tcpCommandIndex = 0;
        sendDiagText("ERROR: comando demasiado largo\r\n");
      } else if (tcpCommandIndex > 0) {
        tcpCommandBuffer[tcpCommandIndex] = '\0';
        processTCPCommand(tcpCommandBuffer);
        tcpCommandIndex = 0;
      }
      continue;
    }

    if (tcpCommandOverflow) {
      continue;
    }
    if (tcpCommandIndex < TCP_COMMAND_BUFFER_SIZE - 1) {
      tcpCommandBuffer[tcpCommandIndex++] = character;
    } else {
      tcpCommandOverflow = true;
    }
  }
}

// TCP - DIAGNÓSTICO PERIÓDICO
// ============================================================================

void sendDiagPeriodicDiagnostic() {
  if (
    !tcpDiagnosticEnabled ||
    !diagClient ||
    !diagClient.connected()
  ) {
    return;
  }

  uint32_t now = millis();

  if (
    now - lastTcpDiagnosticMs <
    TCP_DIAGNOSTIC_PERIOD_MS
  ) {
    return;
  }

  lastTcpDiagnosticMs = now;

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

  sendDiagPrintf(
    "\r\n"
    "[DIAG]\r\n"
    "GNSS=%s FIX_IN=%d HAS=%s SOL=%s SAT=%d\r\n"
    "HDOP=%s ACC=%.2f m\r\n"
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
    estimatedAccuracyMeters(),
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

}

// ============================================================================
// TRANSMISIÓN GCGGA
// ============================================================================

void transmitGCGGA() {
  uint32_t now = millis();

  if (
    now - lastOutputMs <
    OUTPUT_PERIOD_MS_VAR
  ) {
    return;
  }

  lastOutputMs = now;

  int fixQuality =
    getOutputFixQuality();

  if (
    !gnssValid ||
    fixQuality < 1
  ) {
    return;
  }

  double outputLat = currentLat;
  double outputLon = currentLon;
  double outputAlt = currentAlt;

  if (
    movementState == LOCKED &&
    lockedValid
  ) {
    /*
      En LOCKED se utiliza la posición promediada.

      lockedLat y lockedLon ya incluyen la corrección
      de offset calculada con lockedYaw.
    */
    outputLat = lockedLat;
    outputLon = lockedLon;
    outputAlt = lockedAlt;
  } else if (
    imuAvailable &&
    !isnan(currentYaw)
  ) {
    /*
      En MOVING o AVERAGING se corrige cada posición
      utilizando el yaw actual.
    */
    applyAntennaOffset(
      currentLat,
      currentLon,
      currentYaw,
      &outputLat,
      &outputLon
    );
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

  size_t sentenceLength =
    strlen(sentence);

  // Encolar solo tramas completas; serviceCOM2 vacía el FIFO PIO sin bloquear.
  if (com2Ready && com2FrameLength == 0 && sentenceLength <= sizeof(com2Frame)) {
    memcpy(com2Frame, sentence, sentenceLength);
    com2FrameLength = sentenceLength;
    com2FrameIndex = 0;
    lastOutputLat = outputLat;
    lastOutputLon = outputLon;
    lastOutputAlt = outputAlt;
    lastOutputFixQuality = fixQuality;
    serviceCOM2();
  } else {
    com2FramesDropped++;
  }

  // Ethernet W5500.
  transmitEthernet(sentence);

  // Monitor USB.
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
    "RS232-FMW-GPS Rev.4.0 TCP sin BLE"
  );
  Serial.println(
    "Adafruit Metro RP2040"
  );
  Serial.println(
    "=========================================="
  );
  Serial.println(
    "GNSS: Serial1 D0/D1 @ 115200"
  );
  Serial.println(
    "COM2: D2 @ 38400, UART PIO TX-only"
  );
  Serial.println(
    "Ethernet: W5500, NMEA TCP 15919, diagnostico TCP 15920"
  );
  Serial.println(
    "IMU: ICM-20948 I2C"
  );
  Serial.println(
    "HAS: detectado por fix 5 en GGA"
  );
  Serial.println(
    "=========================================="
  );

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  startupLedTest();

  // Iniciar RX GNSS antes de las tareas de I2C/IMU y Ethernet.
  Serial1.begin(GNSS_BAUD);
  gnssStarted = true;

  // I2C e IMU a 100 kHz en los pines SDA/SCL de Metro RP2040.
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  loadMagCalibration();

  if (!initializeICM20948()) {
    Serial.println(
      "[SETUP] IMU no disponible; "
      "se reintentará"
    );
  }

  // Salida COM2 por UART PIO.
  initializeCOM2();

  // Ethernet W5500.
  if (!initializeEthernet()) {
    Serial.println(
      "[SETUP] Ethernet no disponible"
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
  serviceCOM2();
  readGNSS();
  maintainEthernet();
  readGNSS();
  serviceTCPClients();

  readGNSS();
  readYaw();
  readGNSS();

  updateMovementState();
  updateLeds();

  transmitGCGGA();

  readGNSS();
  sendDiagPeriodicDiagnostic();
  serviceDiagQueue();
  serviceCOM2();
}