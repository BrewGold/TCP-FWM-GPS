#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

constexpr uint32_t SPEED_FRESHNESS_MS = 2000;
constexpr uint32_t GGA_FRESHNESS_MS = 2000;

enum SolutionType {
  SOL_UNKNOWN = 0,
  SOL_GPS = 1,
  SOL_DGPS = 2,
  SOL_RTK = 3,
  SOL_HAS = 4
};

SolutionType solutionType = SOL_UNKNOWN;

bool hasActive = false;

// ============================================================================
// VARIABLES GNSS
// ============================================================================

bool gnssValid = false;

double currentLat = 0.0;
double currentLon = 0.0;
double currentAlt = 0.0;
double currentSpeedMS = NAN;
double currentCourse = NAN;

int satelliteCount = 0;
int inputFixQuality = 0;

char utcTime[16] = "000000.00";
char hdopText[16] = "1.0";
double currentHdop = 1.0;

uint32_t lastGgaMs = 0;
uint32_t lastRmcMs = 0;

uint32_t lastOutputMs = 0;


const char* movementStateText() {
  return "MOVING";
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


bool parseDoubleField(const char* text, double* value) {
  if (
    text == nullptr ||
    value == nullptr ||
    text[0] == '\0'
  ) {
    return false;
  }

  bool digit = false;
  for (const char* p = text; *p; ++p) {
    if (*p >= '0' && *p <= '9') {
      digit = true;
    } else if (*p != '.' && !(p == text && (*p == '-' || *p == '+'))) {
      return false;
    }
  }
  if (!digit) {
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
  if (sentence == nullptr || sentence[0] != '$' || strlen(sentence) >= 240) {
    return false;
  }

  const char* asterisk = strchr(sentence, '*');

  if (
    asterisk == nullptr ||
    strlen(asterisk + 1) != 2
  ) {
    return false;
  }

  for (int i = 1; i <= 2; ++i) {
    char c = asterisk[i];
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
          (c >= 'a' && c <= 'f'))) {
      return false;
    }
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

bool parseCoordinate(const char* field, const char* hemisphere, int degreeDigits,
                     int limit, char positive, char negative, double* result) {
  if (!field || !hemisphere || !result ||
      strlen(field) < (size_t)(degreeDigits + 2) ||
      strlen(hemisphere) != 1 ||
      (hemisphere[0] != positive && hemisphere[0] != negative)) {
    return false;
  }
  int degrees = 0;
  for (int i = 0; i < degreeDigits; ++i) {
    if (field[i] < '0' || field[i] > '9') {
      return false;
    }
    degrees = degrees * 10 + field[i] - '0';
  }
  if (field[degreeDigits] < '0' || field[degreeDigits] > '9' ||
      field[degreeDigits + 1] < '0' || field[degreeDigits + 1] > '9' ||
      (field[degreeDigits + 2] != '\0' && field[degreeDigits + 2] != '.')) {
    return false;
  }
  double minutes;
  if (!parseDoubleField(field + degreeDigits, &minutes) ||
      minutes < 0 || minutes >= 60 || degrees > limit ||
      (degrees == limit && minutes != 0)) {
    return false;
  }
  *result = (degrees + minutes / 60.0) * (hemisphere[0] == negative ? -1 : 1);
  return true;
}

bool parseLatitude(
  const char* latitudeField,
  const char* hemisphereField,
  double* latitude
) {
  return parseCoordinate(latitudeField, hemisphereField, 2, 90, 'N', 'S', latitude);
}

bool parseLongitude(
  const char* longitudeField,
  const char* hemisphereField,
  double* longitude
) {
  return parseCoordinate(longitudeField, hemisphereField, 3, 180, 'E', 'W', longitude);
}

void invalidateFix() {
  gnssValid = false;
  hasActive = false;
  solutionType = SOL_UNKNOWN;
}

bool validUtc(const char* text) {
  size_t length = strlen(text);
  if (length < 6 || length >= sizeof(utcTime)) {
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    if (i == 6 && text[i] == '.' && length > 7) {
      continue;
    }
    if (text[i] < '0' || text[i] > '9' || i == 6) {
      return false;
    }
  }
  return (text[0] - '0') * 10 + text[1] - '0' < 24 &&
         (text[2] - '0') * 10 + text[3] - '0' < 60 &&
         (text[4] - '0') * 10 + text[5] - '0' <= 60;
}

// ============================================================================
// PARSER GGA
// ============================================================================

bool parseGGA(const char* line) {
  if (line == nullptr) {
    return false;
  }

  if (!validateNmeaChecksum(line)) {
    return false;
  }

  invalidateFix();
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

  if (fieldCount < 11 || strcmp(fields[10], "M") != 0 ||
      !validUtc(fields[1]) ||
      strlen(fields[6]) != 1 || fields[6][0] < '0' || fields[6][0] > '8') {
    return false;
  }

  int fixQuality = fields[6][0] - '0';
  inputFixQuality = fixQuality;

  if (fixQuality < 1) {
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

  double altitude;
  if (!parseDoubleField(fields[9], &altitude) || fabs(altitude) > 100000.0) {
    return false;
  }

  if (strlen(fields[7]) == 0 || strlen(fields[7]) > 3) {
    return false;
  }
  for (const char* p = fields[7]; *p; ++p) {
    if (*p < '0' || *p > '9') {
      return false;
    }
  }
  double hdop;
  if (!parseDoubleField(fields[8], &hdop) || hdop < 0 || hdop > 999.9) {
    return false;
  }

  strncpy(
    utcTime,
    fields[1],
    sizeof(utcTime) - 1
  );

  utcTime[sizeof(utcTime) - 1] = '\0';

  satelliteCount = atoi(fields[7]);

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
  currentSpeedMS = NAN;
  currentCourse = NAN;
  if (line == nullptr) {
    return false;
  }

  if (!validateNmeaChecksum(line)) {
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

  if (strcmp(fields[2], "A") != 0 || !validUtc(fields[1]) ||
      (fieldCount > 12 && fields[12][0] == 'N')) {
    return false;
  }

  double speedKnots = 0.0;
  double course = NAN;
  if (!parseDoubleField(fields[7], &speedKnots) || speedKnots < 0.0) {
    currentSpeedMS = NAN;
    return false;
  }
  if (!parseDoubleField(fields[8], &course) || course < 0 || course >= 360) {
    course = NAN;
  }

  double speed = speedKnots * 0.51444;
  if (!isfinite(speed)) {
    return false;
  }
  currentSpeedMS = speed;
  currentCourse = course;

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
    strncmp(line, "$GPGGA,", 7) == 0 ||
    strncmp(line, "$GNGGA,", 7) == 0 ||
    strncmp(line, "$GCGGA,", 7) == 0
  ) {
    parseGGA(line);
  } else if (
    strncmp(line, "$GPRMC,", 7) == 0 ||
    strncmp(line, "$GNRMC,", 7) == 0 ||
    strncmp(line, "$GCRMC,", 7) == 0
  ) {
    parseRMC(line);
  }
}


void updateGnssFreshness() {
  uint32_t now = millis();

  if (now - lastRmcMs > SPEED_FRESHNESS_MS) {
    currentSpeedMS = NAN;
    currentCourse = NAN;
  }
  if (now - lastGgaMs > GGA_FRESHNESS_MS) {
    invalidateFix();
  }
}

int getOutputFixQuality() {
  updateGnssFreshness();
  return gnssValid ? inputFixQuality : 0;
}

void formatSpeed(char* output, size_t size) {
  updateGnssFreshness();
  if (isfinite(currentSpeedMS)) {
    snprintf(output, size, "%.3f", currentSpeedMS);
  } else {
    snprintf(output, size, "N/A");
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
    "$GPGGA,%s,%s,%c,%s,%c,%d,%d,%s,%s,M,0.0,M,,",
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
