/*
  TCP-IRI-GPS - Adafruit Metro RP2040 / Arduino-Pico.
  GNSS directo sin IMU, promedio ni correccion de antena.
  Transportes W5500 y COM2 PIO conservados de TCP-FWM-GPS Rev.4.0.
*/

#include <Arduino.h>
#if !defined(ARDUINO_ADAFRUIT_METRO_RP2040) || !defined(ARDUINO_ARCH_RP2040)
#error "TCP-IRI-GPS requiere Adafruit Metro RP2040 con Arduino-Pico de Earle Philhower"
#endif
#include <SerialPIO.h>
#include <Ethernet.h>
#include <hardware/sync.h>
#include <pico/time.h>
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

#define STARTUP_LED_TEST_MS         5000
#define STARTUP_LED_PERIOD_MS       400

#define OUTPUT_PERIOD_MS_DEFAULT    100

#define NMEA_TCP_PORT               15919
#define DIAG_TCP_PORT               15920
#define TCP_DIAGNOSTIC_PERIOD_MS    5000
#define TCP_COMMAND_BUFFER_SIZE     96
#define NMEA_CLIENT_COUNT           4
#define ETHERNET_LINK_TIMEOUT_MS    5000
#define ETHERNET_RETRY_INITIAL_MS   1000
#define ETHERNET_RETRY_MAX_MS       30000
#define ETHERNET_SETUP_ATTEMPTS     3
#define HEX_CAPTURE_CAPACITY        8192
#define HEX_CAPTURE_DURATION_MS     10000
#define GNSS_RX_PIN 0
#define GNSS_TX_PIN 1

// ============================================================================
// PROTOTIPOS
// ============================================================================

void debugPrintf(const char* format, ...);

void initializeCOM2();
void serviceCOM2();

bool initializeEthernet(bool useDHCP = true);
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
void updateHexCapture();
void sendHexCapture();

void readGNSS();
void updateGnssFreshness();
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
bool ethernetRecovering = false;
uint32_t lastEthernetCheckMs = 0;
uint32_t lastEthernetAttemptMs = 0;
uint32_t ethernetRetryMs = ETHERNET_RETRY_INITIAL_MS;
IPAddress ethernetIP(192, 168, 1, 22);
IPAddress ethernetDNS(192, 168, 1, 1);
IPAddress ethernetGateway(192, 168, 1, 1);
IPAddress ethernetSubnet(255, 255, 255, 0);
uint32_t lastTcpDiagnosticMs = 0;
bool tcpDiagnosticEnabled = true;
char tcpCommandBuffer[TCP_COMMAND_BUFFER_SIZE];
uint8_t tcpCommandIndex = 0;
bool tcpCommandOverflow = false;
char diagQueue[4096];
size_t diagQueueHead = 0;
size_t diagQueueLength = 0;

// ============================================================================
// CAPTURA HEX SERIAL1 - INDEPENDIENTE DEL PARSER GNSS
// ============================================================================

enum HexCaptureState {
  HEX_IDLE,
  HEX_CAPTURING,
  HEX_HEADER,
  HEX_DATA,
  HEX_FOOTER,
  HEX_DRAINING
};

HexCaptureState hexCaptureState = HEX_IDLE;
uint8_t hexCaptureBuffer[HEX_CAPTURE_CAPACITY];
size_t hexCaptureHead = 0;
size_t hexCaptureLength = 0;
uint32_t hexCaptureDiscarded = 0;
uint32_t hexCaptureStartMs = 0;
uint32_t hexCaptureDurationMs = 0;

// ============================================================================
// VARIABLES CONFIGURABLES
// ============================================================================

uint16_t OUTPUT_PERIOD_MS_VAR = OUTPUT_PERIOD_MS_DEFAULT;

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
  if (!com2Ready || com2FrameLength == 0 || __get_current_exception() != 0) {
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

#include "Gnss.h"

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

void readGNSS() {
  if (!gnssStarted || gnssReading || __get_current_exception() != 0) {
    return;
  }

  // El parser puede escribir diagnóstico USB y volver a invocar yield().
  gnssReading = true;
  updateHexCapture();
  while (Serial1.available() > 0) {
    char character = (char)Serial1.read();


    if (hexCaptureState == HEX_CAPTURING) {
      updateHexCapture();
      if (hexCaptureState == HEX_CAPTURING) {
        hexCaptureBuffer[(hexCaptureHead + hexCaptureLength) % sizeof(hexCaptureBuffer)] =
          (uint8_t)character;
        if (hexCaptureLength < sizeof(hexCaptureBuffer)) {
          hexCaptureLength++;
        } else {
          hexCaptureHead = (hexCaptureHead + 1) % sizeof(hexCaptureBuffer);
          hexCaptureDiscarded++;
        }
      }
    }

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
  static bool generatingFrame = false;
  if (ethernetRecovering && !ethernetReady && !generatingFrame) {
    generatingFrame = true;
    updateGnssFreshness();
    transmitGCGGA(); // Ethernet deshabilitado: solo COM2 y USB.
    generatingFrame = false;
  }
}

// Arduino-Pico delay() no llama a yield(); atender GNSS/PIO durante las esperas.
void delay(unsigned long ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    yield();
    sleep_ms(1);
  }
}

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
  updateGnssFreshness();
  uint32_t now = millis();

  /*
    LED1 / D5 (Rev.2.5):
    - Apagado        : sin GNSS válido.
    - Parpadeo 600ms : GNSS válido sin HAS (fix 1/2/4).
    - Fijo           : HAS activo (fix 5).
  */
  if (!gnssValid) {
    digitalWrite(LED1_PIN, LOW);
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
  digitalWrite(LED2_PIN, LOW); // Siempre MOVING, sin promedio ni bloqueo.
}

// ============================================================================
// MÁQUINA DE ESTADOS
// ============================================================================


// ============================================================================
// ETHERNET
// ============================================================================

void prepareEthernetSPI() {
  // La SD no se usa; mantenerla deseleccionada antes de acceder al W5500.
  digitalWrite(SD_CS_PIN, HIGH);
  delayMicroseconds(1);
}

bool initializeEthernet(bool useDHCP) {
  ethernetReady = false;
  pinMode(SD_CS_PIN, OUTPUT);
  prepareEthernetSPI();
  for (uint8_t i = 0; i < NMEA_CLIENT_COUNT; i++) {
    nmeaClients[i].setConnectionTimeout(10);
    nmeaClients[i].stop();
  }
  diagClient.setConnectionTimeout(10);
  diagClient.stop();
  // Ethernet.begin() no cierra los sockets de escucha ni los clientes sin aceptar.
  for (uint8_t i = 0; i < MAX_SOCK_NUM; i++) {
    if (EthernetServer::server_port[i] == NMEA_TCP_PORT ||
        EthernetServer::server_port[i] == DIAG_TCP_PORT) {
      EthernetClient serverSocket(i);
      serverSocket.setConnectionTimeout(10);
      serverSocket.stop();
      EthernetServer::server_port[i] = 0;
    }
  }
  diagQueueHead = 0;
  diagQueueLength = 0;
  hexCaptureState = HEX_IDLE;
  tcpCommandIndex = 0;
  tcpCommandOverflow = false;

  Ethernet.init(ETHERNET_CS_PIN);

  Serial.println(
    "[ETH] Inicializando Ethernet"
  );

  // DHCP solo en setup: no detener la generacion COM2 durante la recuperacion.
  int dhcpResult = useDHCP ?
    Ethernet.begin(ethernetMac, ETHERNET_LINK_TIMEOUT_MS, 1000) : 0;

  if (dhcpResult == 0) {
    Ethernet.begin(
      ethernetMac,
      ethernetIP,
      ethernetDNS,
      ethernetGateway,
      ethernetSubnet
    );
    Serial.print("[ETH] IP de respaldo/recuperacion: ");
    Serial.println(ethernetIP);
  } else {
    ethernetIP = Ethernet.localIP();
    ethernetDNS = Ethernet.dnsServerIP();
    ethernetGateway = Ethernet.gatewayIP();
    ethernetSubnet = Ethernet.subnetMask();
    Serial.print("[ETH] IP asignada por DHCP: ");
    Serial.println(ethernetIP);
  }

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("[ETH] Shield no detectado; Servidor TCP: FAIL");
    lastEthernetAttemptMs = millis();
    return false;
  }

  if (Ethernet.localIP() == IPAddress(0, 0, 0, 0)) {
    Serial.println("[ETH] IP no configurada; Servidor TCP: FAIL");
    lastEthernetAttemptMs = millis();
    return false;
  }

  uint32_t linkWaitStart = millis();
  while (Ethernet.linkStatus() != LinkON) {
    if (!useDHCP || millis() - linkWaitStart >= ETHERNET_LINK_TIMEOUT_MS) {
      Serial.println("[ETH] Timeout esperando enlace; Servidor TCP: FAIL");
      lastEthernetAttemptMs = millis();
      return false;
    }
    delay(50);
    prepareEthernetSPI();
  }

  prepareEthernetSPI();
  nmeaServer.begin();
  prepareEthernetSPI();
  diagServer.begin();
  bool nmeaListening = (bool)nmeaServer;
  bool diagListening = (bool)diagServer;
  debugPrintf("[ETH] NMEA TCP %d: %s\n",
    NMEA_TCP_PORT, nmeaListening ? "LISTEN" : "FAIL");
  debugPrintf("[ETH] Diagnostico TCP %d: %s\n",
    DIAG_TCP_PORT, diagListening ? "LISTEN" : "FAIL");

  ethernetReady = nmeaListening && diagListening;
  Serial.println(ethernetReady ?
    "[ETH] Servidor TCP: OK" : "[ETH] Servidor TCP: FAIL");
  lastEthernetAttemptMs = millis();
  if (ethernetReady) {
    ethernetRetryMs = ETHERNET_RETRY_INITIAL_MS;
  }

  return ethernetReady;
}

void maintainEthernet() {
  uint32_t now = millis();

  if (
    now - lastEthernetCheckMs <
    1000
  ) {
    return;
  }

  lastEthernetCheckMs = now;
  prepareEthernetSPI();

  bool hardwarePresent = Ethernet.hardwareStatus() != EthernetNoHardware;
  bool linkUp = hardwarePresent && Ethernet.linkStatus() == LinkON;
  if (ethernetReady && !linkUp) {
    ethernetReady = false;
    ethernetRetryMs = ETHERNET_RETRY_INITIAL_MS;
    Serial.println("[ETH] Enlace perdido; se reintentara Ethernet");
  }

  if (!ethernetReady) {
    // No bloquear el loop esperando un cable desconectado.
    if (hardwarePresent && !linkUp) {
      return;
    }
    if (now - lastEthernetAttemptMs >= ethernetRetryMs) {
      ethernetRecovering = true;
      bool initialized = initializeEthernet(false);
      ethernetRecovering = false;
      if (!initialized) {
        ethernetRetryMs = min(ethernetRetryMs * 2,
          (uint32_t)ETHERNET_RETRY_MAX_MS);
      }
    }
    return;
  }

  // accept() consume el socket de escucha; recrearlo solo si falta.
  if (!nmeaServer) {
    nmeaServer.begin();
  }
  prepareEthernetSPI();
  if (!diagServer) {
    diagServer.begin();
  }
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
    hexCaptureState = HEX_IDLE;
    diagClient = diagServer.accept();

    if (diagClient) {
      tcpCommandIndex = 0;
      tcpCommandOverflow = false;
      lastTcpDiagnosticMs = millis();
      sendDiagText(
        "\r\nTCP-IRI-GPS TCP diagnostic\r\n"
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
  if (!ethernetReady) {
    return;
  }
  prepareEthernetSPI();
  acceptTCPClients();

  for (uint8_t i = 0; i < NMEA_CLIENT_COUNT; i++) {
    if (!nmeaClients[i].connected()) {
      nmeaClients[i].stop();
    }
  }

  readTCPCommands();
  sendHexCapture();
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
  prepareEthernetSPI();

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
  prepareEthernetSPI();
  if (
    !ethernetReady ||
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
    hexCaptureState = HEX_IDLE;
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
  prepareEthernetSPI();
  if (!ethernetReady || !diagClient || !diagClient.connected()) {
    diagQueueHead = 0;
    diagQueueLength = 0;
    hexCaptureState = HEX_IDLE;
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
// TCP - CAPTURA HEX (UNA LINEA POR VUELTA, SIN ESPERAR ESPACIO TCP)
// ============================================================================

void updateHexCapture() {
  if (
    hexCaptureState == HEX_CAPTURING &&
    millis() - hexCaptureStartMs >= HEX_CAPTURE_DURATION_MS
  ) {
    hexCaptureDurationMs = HEX_CAPTURE_DURATION_MS;
    hexCaptureState = HEX_HEADER;
  }
}

void sendHexCapture() {
  if (!ethernetReady || !diagClient || !diagClient.connected()) {
    hexCaptureState = HEX_IDLE;
    return;
  }

  updateHexCapture();
  if (hexCaptureState == HEX_DRAINING) {
    if (diagQueueLength == 0) {
      hexCaptureState = HEX_IDLE;
    }
    return;
  }
  char text[192];
  size_t bytes = 0;
  if (hexCaptureState == HEX_HEADER) {
    snprintf(
      text, sizeof(text),
      "\r\n[HEX CAPTURE]\r\n"
      "Duracion: %lu ms\r\n"
      "Bytes guardados: %lu\r\n"
      "Bytes descartados: %lu\r\n\r\n",
      (unsigned long)hexCaptureDurationMs,
      (unsigned long)hexCaptureLength,
      (unsigned long)hexCaptureDiscarded
    );
  } else if (hexCaptureState == HEX_DATA) {
    const char hexDigits[] = "0123456789ABCDEF";
    bytes = min(hexCaptureLength, (size_t)16);
    for (size_t i = 0; i < bytes; i++) {
      uint8_t value = hexCaptureBuffer[(hexCaptureHead + i) % sizeof(hexCaptureBuffer)];
      text[i * 3] = hexDigits[value >> 4];
      text[i * 3 + 1] = hexDigits[value & 0x0F];
      text[i * 3 + 2] = ' ';
    }
    text[bytes * 3] = '\r';
    text[bytes * 3 + 1] = '\n';
    text[bytes * 3 + 2] = '\0';
  } else if (hexCaptureState == HEX_FOOTER) {
    strcpy(text, "[FIN CAPTURA]\r\n");
  } else {
    return;
  }

  if (strlen(text) > sizeof(diagQueue) - diagQueueLength) {
    return;
  }
  sendDiagText(text);

  if (hexCaptureState == HEX_HEADER) {
    hexCaptureState = hexCaptureLength > 0 ? HEX_DATA : HEX_FOOTER;
  } else if (hexCaptureState == HEX_DATA) {
    hexCaptureHead = (hexCaptureHead + bytes) % sizeof(hexCaptureBuffer);
    hexCaptureLength -= bytes;
    if (hexCaptureLength == 0) {
      hexCaptureState = HEX_FOOTER;
    }
  } else if (hexCaptureState == HEX_FOOTER) {
    hexCaptureState = HEX_DRAINING;
  }
}

// ============================================================================
// TCP - DIAGNÓSTICO STATUS
// ============================================================================

void sendDiagStatus() {
  if (!ethernetReady || !diagClient || !diagClient.connected()) {
    return;
  }

  char speedText[32];
  formatSpeed(speedText, sizeof(speedText));
  sendDiagPrintf(
    "\r\n========== STATUS ==========\r\n"
    "Producto: TCP-IRI-GPS\r\n"
    "GNSS: %s\r\n"
    "GGA entrada fix: %d\r\n"
    "HAS: %s\r\n"
    "Solucion: %s\r\n"
    "Satelites: %d\r\n"
    "HDOP: %s\r\n"
    "Precision est: %.2f m\r\n"
    "Hora UTC: %s\r\n"
    "LAT: %.8f\r\nLON: %.8f\r\nALT: %.2f m\r\n"
    "Velocidad: %s m/s\r\n"
    "Curso: %.1f deg\r\n"
    "IMU: OFF (sin magnetometro)\r\n"
    "Estado: MOVING (GNSS directo, sin promedio ni offset)\r\n"
    "Fix salida: %d\r\n"
    "Frecuencia: %d Hz\r\n"
    "============================\r\n",
    gnssValid ? "OK" : "FAIL", inputFixQuality,
    hasActive ? "ACTIVO" : "NO", solutionTypeText(),
    satelliteCount, hdopText, estimatedAccuracyMeters(), utcTime,
    currentLat, currentLon, currentAlt, speedText, currentCourse,
    lastOutputFixQuality, 1000 / OUTPUT_PERIOD_MS_VAR
  );

  prepareEthernetSPI();
  sendDiagPrintf(
    "Servidor NMEA TCP %d: %s\r\n"
    "Servidor diagnostico TCP %d: %s\r\n",
    NMEA_TCP_PORT, (bool)nmeaServer ? "LISTEN" : "SIN ESCUCHA",
    DIAG_TCP_PORT, (bool)diagServer ? "LISTEN" : "SIN ESCUCHA"
  );
}

// ============================================================================
// TCP - DIAGNÓSTICO IMU
// ============================================================================

void sendDiagIMU() {
  sendDiagText("TCP-IRI-GPS: IMU OFF, sin magnetometro ni offset.\r\n");

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
    "NO"
  );

}

// ============================================================================
// TCP - AYUDA
// ============================================================================

void sendDiagHelp() {
  const char* help =
    "\r\n"
    "======= TCP-IRI-GPS: COMANDOS TCP =======\r\n"
    "GNSS directo; velocidad en m/s, N/A si no disponible.\r\n"
    "status\r\n"
    "  Estado general completo.\r\n"
    "\r\n"
    "com2\r\n"
    "  Estado y contadores COM2.\r\n"
    "\r\n"
    "freq <1..10>\r\n"
    "  Frecuencia de salida NMEA/COM2 en Hz.\r\n"
    "\r\n"
    "diag on|off\r\n"
    "  Activa o desactiva el bloque periodico [DIAG].\r\n"
    "\r\n"
    "debug hex on|off\r\n"
    "  Captura Serial1 10 s; off finaliza antes y envia los bytes.\r\n"
    "  Conserva los ultimos 8192 bytes y cuenta los descartados.\r\n"
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

  if (strcmp(start, "debug hex on") == 0) {
    if (hexCaptureState != HEX_IDLE) {
      sendDiagText("ERROR: captura ya en curso\r\n");
      return;
    }
    hexCaptureHead = 0;
    hexCaptureLength = 0;
    hexCaptureDiscarded = 0;
    hexCaptureDurationMs = 0;
    hexCaptureStartMs = millis();
    hexCaptureState = HEX_CAPTURING;
    sendDiagText("OK: captura iniciada\r\n");
    return;
  }

  if (strcmp(start, "debug hex off") == 0) {
    if (hexCaptureState == HEX_CAPTURING) {
      hexCaptureDurationMs = millis() - hexCaptureStartMs;
      hexCaptureState = HEX_HEADER;
    }
    sendDiagText("OK: captura detenida\r\n");
    return;
  }

  if (strncmp(start, "magcal", 6) == 0 || strncmp(start, "yawoff", 6) == 0) {
    sendDiagText("ERROR: TCP-IRI-GPS no usa magnetometro ni yaw.\r\n");
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
    (hexCaptureState != HEX_IDLE && hexCaptureState != HEX_CAPTURING) ||
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

  char speedText[32];
  formatSpeed(speedText, sizeof(speedText));

  sendDiagPrintf(
    "\r\n"
    "[DIAG]\r\n"
    "PRODUCT=TCP-IRI-GPS\r\n"
    "GNSS=%s FIX_IN=%d HAS=%s SOL=%s SAT=%d\r\n"
    "HDOP=%s ACC=%.2f m\r\n"
    "STATE=%s LOCK=%s SPD=%s m/s\r\n"
    "IMU=OFF MODE=DIRECT\r\n"
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
    "NO",
    speedText,
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
  char speedText[32];
  formatSpeed(speedText, sizeof(speedText));
  debugPrintf("[TCP-IRI-GPS] SPD=%s m/s\n", speedText);
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
    "TCP-IRI-GPS TCP sin BLE - GNSS directo"
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
    "IMU: OFF; sin magnetometro, promedio ni offset"
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

  // Iniciar RX GNSS antes de Ethernet.
  Serial1.setFIFOSize(2048);   // buffer RX grande: evita perder bytes a 115200
  Serial1.begin(GNSS_BAUD);
  gnssStarted = true;


  // Salida COM2 por UART PIO.
  initializeCOM2();

  // Ethernet W5500.
  for (uint8_t attempt = 0; attempt < ETHERNET_SETUP_ATTEMPTS; attempt++) {
    if (initializeEthernet()) {
      break;
    }
    if (attempt + 1 < ETHERNET_SETUP_ATTEMPTS) {
      debugPrintf("[ETH] Reintento en %lu ms\n", (unsigned long)ethernetRetryMs);
      delay(ethernetRetryMs);
      ethernetRetryMs = min(ethernetRetryMs * 2,
        (uint32_t)ETHERNET_RETRY_MAX_MS);
    }
  }
  if (!ethernetReady) {
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
  readGNSS();

  updateGnssFreshness();
  updateLeds();

  transmitGCGGA();

  readGNSS();
  sendDiagPeriodicDiagnostic();
  serviceDiagQueue();
  serviceCOM2();
}
