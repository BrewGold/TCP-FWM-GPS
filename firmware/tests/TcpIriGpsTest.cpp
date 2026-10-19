#include <cassert>
#include <cmath>
#include <iostream>
#include "../arduino/TCP-IRI-GPS/TCP-IRI-GPS.ino"

std::string nmea(const std::string& payload) {
  unsigned checksum = 0;
  for (char c : payload) checksum ^= static_cast<unsigned char>(c);
  char suffix[4];
  snprintf(suffix, sizeof(suffix), "*%02X", checksum);
  return "$" + payload + suffix;
}

std::string gga(int fix = 5, const std::string& lat = "4807.0380",
                const std::string& lon = "01131.0000", const std::string& alt = "545.4") {
  return nmea("GNGGA,123519.00," + lat + ",N," + lon + ",E," +
              std::to_string(fix) + ",12,0.8," + alt + ",M,0.0,M,,");
}

std::string rmc(const std::string& speed, const std::string& status = "A",
                const std::string& course = "84.4", const std::string& mode = "A") {
  return nmea("GNRMC,123519.00," + status +
              ",4807.0380,N,01131.0000,E," + speed + "," + course + ",230394,,," + mode);
}

std::string diagnostic(const char* command) {
  diagClient.output.clear();
  processTCPCommand(command);
  while (diagQueueLength) serviceDiagQueue();
  return diagClient.output;
}

void expectContains(const std::string& text, const std::string& part) {
  if (text.find(part) == std::string::npos) {
    std::cerr << "Missing: " << part << "\nIn: " << text << '\n';
    std::abort();
  }
}

void drainCom2() {
  while (com2FrameLength) serviceCOM2();
}

void testDirectOutput() {
  initializeCOM2();
  ethernetReady = true;
  nmeaClients[0].active = true;
  for (int fix : {1, 2, 4, 5}) {
    testMillis += 100;
    assert(parseGGA(gga(fix).c_str()));
    assert(getOutputFixQuality() == fix);
    assert(hasActive == (fix == 5));
    com2Serial.output.clear();
    nmeaClients[0].output.clear();
    transmitGCGGA();
    drainCom2();
    assert(com2Serial.output == nmeaClients[0].output);
    expectContains(com2Serial.output, "$GPGGA,123519.00,4807.0380,N,01131.0000,E," +
                   std::to_string(fix) + ",12,0.8,545.4,M,0.0,M,,");
    assert(com2Serial.output.substr(com2Serial.output.size() - 2) == "\r\n");
    assert(validateNmeaChecksum(com2Serial.output.substr(0, com2Serial.output.size() - 2).c_str()));
  }
  for (int i = 0; i < 250; ++i) {
    testMillis += 100;
    assert(parseRMC(rmc("0").c_str()));
    std::string lat = i % 2 ? "4810.0000" : "4800.0000";
    std::string lon = i % 2 ? "01210.0000" : "01100.0000";
    std::string alt = i % 2 ? "600.1" : "500.2";
    assert(parseGGA(gga(5, lat, lon, alt).c_str()));
    updateGnssFreshness();
    com2Serial.output.clear();
    nmeaClients[0].output.clear();
    transmitGCGGA();
    drainCom2();
    expectContains(com2Serial.output, lat + ",N," + lon + ",E,5,12,0.8," + alt);
    assert(com2Serial.output == nmeaClients[0].output);
    assert(strcmp(movementStateText(), "MOVING") == 0);
    updateLeds();
    assert(pinStates[LED2_PIN] == LOW);
    assert(pinStates[LED1_PIN] == HIGH);
  }
  size_t frames = com2FramesSent;
  testMillis += 2001;
  transmitGCGGA();
  assert(com2FramesSent == frames);
  assert(!gnssValid && !hasActive && getOutputFixQuality() == 0);
  updateLeds();
  assert(pinStates[LED1_PIN] == LOW);
}

void testSpeed() {
  char speed[32];
  currentSpeedMS = NAN;
  formatSpeed(speed, sizeof(speed));
  assert(std::string(speed) == "N/A");
  for (const auto& value : {std::pair{"10", 5.1444}, {"20", 10.2888}, {"0", 0.0}}) {
    assert(parseRMC(rmc(value.first).c_str()));
    assert(fabs(currentSpeedMS - value.second) < 0.00001);
  }
  for (const auto& invalid : {"", "-1", "nan", "inf", "10junk", "1e3", "0x10", " 2", "."}) {
    assert(parseRMC(rmc("10").c_str()));
    assert(!parseRMC(rmc(invalid).c_str()));
    formatSpeed(speed, sizeof(speed));
    assert(std::string(speed) == "N/A");
  }
  for (const auto& line : {rmc("10", "V"), rmc("10", "AA"), nmea("GNRMC,123519.00,A"),
                          rmc("10", "A", "", "N"), rmc("10") + "junk"}) {
    assert(parseRMC(rmc("20").c_str()));
    assert(!parseRMC(line.c_str()));
    assert(isnan(currentSpeedMS));
  }
  assert(parseRMC(rmc("10", "A", "").c_str()));
  assert(isnan(currentCourse) && isfinite(currentSpeedMS));
  std::string corrupt = rmc("20");
  corrupt.back() = corrupt.back() == '0' ? '1' : '0';
  assert(!parseRMC(corrupt.c_str()));
  assert(isnan(currentSpeedMS));
  assert(parseRMC(rmc("10").c_str()));
  testMillis += 2000;
  formatSpeed(speed, sizeof(speed));
  assert(std::string(speed) == "5.144");
  ++testMillis;
  formatSpeed(speed, sizeof(speed));
  assert(std::string(speed) == "N/A");
  testMillis = UINT32_MAX - 1000;
  assert(parseRMC(rmc("20").c_str()));
  testMillis += 1500;
  formatSpeed(speed, sizeof(speed));
  assert(std::string(speed) == "10.289");
  testMillis += 501;
  formatSpeed(speed, sizeof(speed));
  assert(std::string(speed) == "N/A");
}

void testValidation() {
  testMillis = 10000;
  assert(parseGGA(gga().c_str()));
  double lastLat = currentLat;
  uint32_t lastTime = lastGgaMs;
  std::string corrupt = gga(1, "4900.0000");
  corrupt.back() = corrupt.back() == '0' ? '1' : '0';
  assert(!parseGGA(corrupt.c_str()));
  assert(currentLat == lastLat && lastGgaMs == lastTime);
  for (const auto& invalid : {gga(0), gga(5, "4860.0000"), gga(5, "9100.0000"),
       gga(5, "4807junk"), gga(5, "4807.0000", "18100.0000"), gga(5, "4807.0000", "01100.0000", ""),
       gga(5, "4807.0000", "01100.0000", "NaN"), nmea("GNGGA,123519.00,,,,,5"),
       nmea("GNGGA,250000,4807.0380,N,01131.0000,E,5,12,0.8,545.4,M,0.0,M,,"),
       nmea("GNGGA,123519,4807.0380,X,01131.0000,E,5,12,0.8,545.4,M,0.0,M,,"),
       nmea("GNGGA,123519,4807.0380,N,01131.0000,E,5x,12,0.8,545.4,M,0.0,M,,")}) {
    assert(parseGGA(gga().c_str()));
    assert(!parseGGA(invalid.c_str()));
    assert(!gnssValid && !hasActive && getOutputFixQuality() == 0);
  }
  assert(!validateNmeaChecksum((gga() + "00").c_str()));
  gnssStarted = true;
  Serial1.feed(std::string(300, 'x') + "\n$" + std::string(300, 'x') + "\n");
  readGNSS();
  Serial1.feed(gga() + "\r");
  readGNSS();
  assert(!gnssValid);
  Serial1.feed("\n");
  readGNSS();
  assert(gnssValid);
  assert(parseGGA(gga(1).c_str()));
  testMillis = 12000; lastGgaMs = testMillis;
  updateLeds(); assert(pinStates[LED1_PIN] == HIGH);
  testMillis += 300;
  updateLeds(); assert(pinStates[LED1_PIN] == LOW);
}

void testDiagnostic() {
  ethernetReady = true;
  diagClient.active = true;
  testMillis = 20000;
  assert(parseGGA(gga().c_str()));
  assert(parseRMC(rmc("10").c_str()));
  std::string status = diagnostic("status");
  expectContains(status, "========== STATUS ==========");
  expectContains(status, "Producto: TCP-IRI-GPS");
  expectContains(status, "GNSS: OK");
  expectContains(status, "GGA entrada fix: 5");
  expectContains(status, "HAS: ACTIVO");
  expectContains(status, "Solucion: HAS");
  expectContains(status, "Velocidad: 5.144 m/s");
  expectContains(status, "============================");
  for (const auto& invalid : {rmc("10", "V"), rmc(""), rmc("-1")}) {
    assert(parseRMC(rmc("10").c_str()));
    assert(!parseRMC(invalid.c_str()));
    expectContains(diagnostic("status"), "Velocidad: N/A m/s");
  }
  testMillis += 5000;
  assert(parseGGA(gga().c_str()));
  assert(parseRMC(rmc("20").c_str()));
  diagClient.output.clear();
  sendDiagPeriodicDiagnostic();
  while (diagQueueLength) serviceDiagQueue();
  expectContains(diagClient.output, "[DIAG]\r\nPRODUCT=TCP-IRI-GPS");
  expectContains(diagClient.output, "STATE=MOVING LOCK=NO SPD=10.289 m/s");
  expectContains(diagClient.output, "IMU=OFF MODE=DIRECT");
  testMillis += 5000;
  diagClient.output.clear();
  sendDiagPeriodicDiagnostic();
  while (diagQueueLength) serviceDiagQueue();
  expectContains(diagClient.output, "GNSS=FAIL FIX_IN=5 HAS=OFF SOL=UNKNOWN");
  expectContains(diagClient.output, "SPD=N/A m/s");
  expectContains(diagnostic("status"), "Velocidad: N/A m/s");
  expectContains(diagnostic("help"), "TCP-IRI-GPS");
  assert(diagnostic("help").find("magcal start") == std::string::npos);
  for (const char* command : {"magcal start", "magcal stop", "magcal reset", "yawoff 15"})
    expectContains(diagnostic(command), "ERROR: TCP-IRI-GPS no usa magnetometro ni yaw");
  expectContains(diagnostic("imu"), "IMU OFF");
  expectContains(diagnostic("com2"), "38400 baud");
  expectContains(diagnostic("freq 1"), "OK:");
  assert(OUTPUT_PERIOD_MS_VAR == 1000);
  expectContains(diagnostic("freq 10"), "OK:");
  assert(OUTPUT_PERIOD_MS_VAR == 100);
  expectContains(diagnostic("freq 11"), "ERROR:");
  diagnostic("diag off"); assert(!tcpDiagnosticEnabled);
  diagnostic("diag on"); assert(tcpDiagnosticEnabled);
  diagnostic("debug hex on"); assert(hexCaptureState == HEX_CAPTURING);
  diagnostic("debug hex off"); assert(hexCaptureState == HEX_HEADER);
  hexCaptureState = HEX_IDLE;
  diagClient.feed(std::string(100, 'x') + "\nstatus\n");
  while (diagClient.available()) readTCPCommands();
  while (diagQueueLength) serviceDiagQueue();
  expectContains(diagClient.output, "ERROR: comando demasiado largo");
}

int main() {
  setup();
  assert(testMillis >= STARTUP_LED_TEST_MS);
  for (int pin : {LED1_PIN, LED2_PIN}) {
    assert(std::find(pinWrites.begin(), pinWrites.end(), std::pair{pin, HIGH}) != pinWrites.end());
    assert(pinStates[pin] == LOW);
  }
  testDirectOutput();
  testSpeed();
  testValidation();
  testDiagnostic();
  std::cout << "IRI: startup LEDs, direct GNSS/COM2/TCP, speed, validation and diagnostics passed\n";
}
