#include <cassert>
#include <cmath>
#include <iostream>
#include "../arduino/RS232-RWM-GPS_V4-1/RS232-RWM-GPS_V4-1.ino"

using Driver = Rev41::Socket<W5500TransportIO>;

void resetTest(uint32_t now = 1000) {
  assert(!SPI.active);
  W5100 = FakeW5500{};
  tcpTransport.reset();
  ethernetReady = false;
  resetTCPState();
  tcpTransport.reset();
  testMillis = now;
  lastOutputMs = now;
  lastEthernetCheckMs = now;
  ethernetRecovering = false;
  Ethernet.link = LinkON;
  Ethernet.hardware = 1;
  gnssStarted = false;
  gnssReading = false;
  gnssValid = false;
  imuAvailable = false;
  currentYaw = NAN;
  movementState = MOVING;
  lockedValid = false;
  OUTPUT_PERIOD_MS_VAR = 100;
  tcpDiagnosticEnabled = false;
  diagQueueOverflows = 0;
  diagQueueHighWater = 0;
  for (auto& socket : tcpTransport.sockets) socket.counters = {};
  initializeCOM2();
  com2Serial.output.clear();
}

void tick(uint32_t elapsed = 1) {
  testMillis += elapsed;
  unsigned reads = W5100.reads, writes = W5100.writes;
  serviceTCPClients();
  assert(!SPI.active);
  // A failed device cannot make one tick spin on any register.
  assert(W5100.reads - reads < 200);
  assert(W5100.writes - writes < 100);
}

void ticks(unsigned count) { while (count--) tick(); }

void boot() {
  resetTest();
  assert(initializeEthernet(false));
  ticks(3);
  assert(bool(nmeaServer) && bool(diagServer));
}

uint8_t connectClient(uint16_t port) {
  ticks(3);
  uint8_t id = 8;
  for (auto& socket : tcpTransport.sockets) {
    if (socket.port == port && socket.phase == Driver::Listening) { id = socket.id; break; }
  }
  assert(id < 8);
  W5100.sockets[id].status = Rev41::Established;
  tick();
  assert(tcpTransport.sockets[id].connected());
  return id;
}

std::string sentence(const std::string& payload) {
  unsigned checksum = 0;
  for (unsigned char c : payload) checksum ^= c;
  char suffix[8];
  snprintf(suffix, sizeof(suffix), "*%02X", checksum);
  return "$" + payload + suffix;
}

std::string gga(int fix) {
  return sentence("GNGGA,123519.00,4807.0380,N,01131.0000,E," +
    std::to_string(fix) + ",12,0.8,545.4,M,0.0,M,,");
}

void produce(int fix = 5) {
  assert(parseGGA(gga(fix).c_str()));
  transmitGCGGA();
  while (com2FrameLength) serviceCOM2();
}

void testIsolationAndCadence() {
  boot();
  uint8_t ids[4];
  for (auto& id : ids) id = connectClient(NMEA_TCP_PORT);
  uint8_t diag = connectClient(DIAG_TCP_PORT);
  assert(nmeaClients[0].timeout == 10 && diagClient.timeout == 10);
  ticks(5); // Drain welcome without EthernetClient I/O.
  for (auto id : ids) W5100.sockets[id].sends.clear();
  W5100.sockets[diag].sends.clear();
  W5100.sockets[ids[0]].ackSend = false;
  gnssStarted = true;
  for (unsigned sample = 0; sample < 40; ++sample) {
    testMillis += 100;
    Serial1.feed(gga(5) + "\r\n");
    readGNSS();
    produce();
    if (sample < 10) sendDiagText("alive\r\n");
    ticks(4);
    for (const auto& socket : tcpTransport.sockets)
      assert(socket.queued() <= 2 * Rev41::FrameCapacity);
  }
  assert(W5100.sockets[ids[0]].sends.size() == 1); // No ambiguous resend.
  assert(tcpTransport.sockets[ids[0]].counters.stalls == 1);
  for (unsigned i = 1; i < 4; ++i) assert(W5100.sockets[ids[i]].sends.size() == 40);
  assert(W5100.sockets[diag].sends.size() >= 10);
  assert(com2Serial.output.find("$GPGGA,") != std::string::npos && gnssValid);
  assert(tcpTransport.sockets[ids[1]].counters.sent == 40);
}

void adoptDirect(uint8_t id, uint16_t port = NMEA_TCP_PORT) {
  auto& socket = tcpTransport.sockets[id];
  assert(socket.listen(port, testMillis));
  socket.poll(testMillis + 1, 100, port == NMEA_TCP_PORT);
  socket.poll(testMillis + 2, 100, port == NMEA_TCP_PORT);
  W5100.sockets[id].status = Rev41::Established;
  socket.poll(testMillis + 3, 100, port == NMEA_TCP_PORT);
  socket.adopted = true;
  assert(socket.connected());
  testMillis += 4;
}

void offer(Driver& socket, const std::string& data) {
  assert(socket.offer(reinterpret_cast<const uint8_t*>(data.data()), data.size(), testMillis));
}

void testPassiveHandshakeInterleavings() {
  // SYNRECV is valid both before and after software observes LISTEN.
  for (bool observeListen : {false, true}) {
    resetTest(UINT32_MAX - 50);
    auto& socket = tcpTransport.sockets[0];
    auto& chip = W5100.sockets[0];
    assert(socket.listen(NMEA_TCP_PORT, testMillis));
    uint32_t generation = socket.generation;
    socket.poll(testMillis + 1, 100, true);
    assert(socket.phase == Driver::ListenCommand);
    if (observeListen) {
      socket.poll(testMillis + 2, 100, true);
      assert(socket.phase == Driver::Listening);
    }
    chip.status = Rev41::SynRecv;
    for (uint32_t elapsed : {3u, 100u, 3001u}) {
      socket.poll(testMillis + elapsed, 100, true);
      assert(socket.phase == Driver::Listening);
      assert(chip.status == Rev41::SynRecv && chip.command == 0);
      assert(socket.generation == generation);
      assert(socket.counters.errors == 0 && socket.counters.stalls == 0);
    }
    chip.status = Rev41::Established;
    socket.poll(testMillis + 3002, 100, true);
    assert(socket.phase == Driver::Peer);
    socket.adopted = true;
    assert(socket.connected());
    testMillis += 3003;
    offer(socket, "handshake\r\n");
    socket.poll(testMillis, 100, true);
    socket.poll(++testMillis, 100, true);
    assert(chip.sends.size() == 1 && chip.sends[0] == "handshake\r\n");
    assert(socket.counters.sent == 1 && !SPI.active);
  }

  // A completed LISTEN command may already have an established peer, even
  // when scheduling delays the first observation past the command deadline.
  for (uint32_t elapsed : {2u, 100u, 101u}) {
    resetTest();
    auto& socket = tcpTransport.sockets[0];
    assert(socket.listen(NMEA_TCP_PORT, testMillis));
    socket.poll(testMillis + 1, 100, true);
    assert(socket.phase == Driver::ListenCommand);
    W5100.sockets[0].status = Rev41::Established;
    socket.poll(testMillis + elapsed, 100, true);
    assert(socket.phase == Driver::Peer && socket.counters.errors == 0);
  }

  // CLOSED and CLOSE_WAIT during either listening phase must release the
  // socket cooperatively so a subsequent connection can reuse it.
  for (bool observeListen : {false, true}) {
    for (uint8_t status : {uint8_t(Rev41::Closed), uint8_t(0x1c)}) {
      resetTest();
      auto& socket = tcpTransport.sockets[0];
      auto& chip = W5100.sockets[0];
      assert(socket.listen(NMEA_TCP_PORT, testMillis));
      uint32_t generation = socket.generation;
      socket.poll(++testMillis, 100, true);
      if (observeListen) {
        socket.poll(++testMillis, 100, true);
        chip.status = Rev41::SynRecv;
        socket.poll(++testMillis, 100, true);
        assert(socket.phase == Driver::Listening);
      }
      chip.status = status;
      socket.poll(++testMillis, 100, true);
      assert(socket.phase == Driver::Closing);
      socket.poll(++testMillis, 100, true);
      socket.poll(++testMillis, 100, true);
      assert(socket.phase == Driver::Idle && socket.generation != generation);
      assert(socket.counters.errors == 0);
      adoptDirect(0);
      assert(socket.connected() && !SPI.active);
    }
  }

  // Valid status does not override a stuck command or extend its deadline.
  resetTest();
  auto& socket = tcpTransport.sockets[0];
  auto& chip = W5100.sockets[0];
  assert(socket.listen(NMEA_TCP_PORT, testMillis));
  chip.holdCommand = true;
  socket.poll(testMillis + 1, 100, true);
  chip.status = Rev41::Established;
  socket.poll(testMillis + 100, 100, true);
  assert(socket.phase == Driver::ListenCommand);
  socket.poll(testMillis + 101, 100, true);
  assert(socket.phase == Driver::Closing && socket.counters.stalls == 1);

  resetTest();
  assert(socket.listen(NMEA_TCP_PORT, testMillis));
  socket.poll(testMillis + 1, 100, true);
  chip.status = Rev41::Init;
  socket.poll(testMillis + 99, 100, true);
  assert(socket.phase == Driver::ListenCommand);
  socket.poll(testMillis + 100, 100, true);
  assert(socket.phase == Driver::Closing && socket.counters.stalls == 1);
}

void testWholeFramesLatestAndFreshness() {
  resetTest();
  adoptDirect(0);
  auto& socket = tcpTransport.sockets[0];
  auto& chip = W5100.sockets[0];
  chip.ackSend = false;
  offer(socket, "$active*00\r\n");
  socket.poll(testMillis, 100, true);
  for (unsigned i = 0; i < 10; ++i) {
    testMillis += 10;
    offer(socket, "$pending" + std::to_string(i) + "*00\r\n");
    socket.poll(testMillis, 100, true);
  }
  assert(chip.sends.size() == 1 && chip.sends[0] == "$active*00\r\n");
  assert(socket.counters.coalesced == 9);
  chip.interrupt |= Rev41::SendOK;
  socket.poll(testMillis, 100, true);
  chip.ackSend = true;
  socket.poll(testMillis, 100, true);
  assert(chip.sends.size() == 2 && chip.sends[1] == "$pending9*00\r\n");
  socket.poll(testMillis, 100, true);
  offer(socket, "$late*00\r\n");
  testMillis += 500;
  socket.poll(testMillis, 100, true);
  assert(socket.counters.expired == 1 && chip.sends.size() == 2);
  offer(socket, "$one*00\r\n");
  socket.poll(testMillis, 100, true);
  socket.poll(testMillis, 100, true);
  offer(socket, "$two*00\r\n");
  socket.poll(testMillis, 100, true);
  assert(chip.sends.size() == 3); // ACK and a pending sample do not bypass cadence.
  testMillis += 100;
  socket.poll(testMillis, 100, true);
  assert(chip.sends.size() == 4);
  testMillis += 1000; // No catch-up burst following a pause.
  socket.poll(testMillis, 100, true);
  for (unsigned i = 0; i < 20; ++i) socket.poll(testMillis, 100, true);
  assert(chip.sends.size() == 4);

  // Active sample waiting for local TX space may expire only before SEND.
  chip.free = 0;
  offer(socket, "$no-space*00\r\n");
  socket.poll(testMillis, 100, true);
  testMillis += 500;
  socket.poll(testMillis, 100, true);
  assert(socket.counters.expired == 2 && chip.sends.size() == 4);
  testMillis += 2500;
  socket.poll(testMillis, 100, true);
  assert(socket.phase == Driver::Closing && socket.counters.stalls == 1);
}

void testRegisterDeadlinesAndReuse() {
  resetTest(UINT32_MAX - 100);
  adoptDirect(0);
  auto& socket = tcpTransport.sockets[0];
  auto& chip = W5100.sockets[0];
  uint32_t generation = socket.generation;
  chip.ackSend = false;
  offer(socket, "rollover\r\n");
  socket.poll(testMillis, 100, true);
  testMillis += 2999;
  socket.poll(testMillis, 100, true);
  assert(socket.connected());
  ++testMillis;
  socket.poll(testMillis, 100, true);
  assert(socket.phase == Driver::Closing);
  socket.poll(++testMillis, 100, true);
  socket.poll(++testMillis, 100, true);
  assert(socket.phase == Driver::Idle && socket.generation != generation);
  chip.sends.clear();
  adoptDirect(0);
  assert(!socket.busy());
  offer(socket, "new-generation\r\n");
  socket.poll(testMillis, 100, true);
  chip.interrupt = Rev41::Timeout | Rev41::SendOK;
  socket.poll(++testMillis, 100, true);
  assert(socket.phase == Driver::Closing && chip.sends.size() == 1);
  assert(socket.counters.sent == 0); // TIMEOUT wins over simultaneous SEND_OK.

  resetTest(UINT32_MAX - 50);
  auto& stuck = tcpTransport.sockets[1];
  W5100.sockets[1].holdCommand = true;
  assert(stuck.listen(NMEA_TCP_PORT, testMillis));
  stuck.poll(testMillis + 99, 100, true);
  assert(stuck.phase == Driver::Opening);
  stuck.poll(testMillis + 100, 100, true);
  assert(stuck.phase == Driver::Closing);
  stuck.poll(testMillis + 200, 100, true);
  assert(stuck.phase == Driver::Quarantined);
  assert(!stuck.listen(NMEA_TCP_PORT, testMillis + 201));
  assert(!SPI.active);

  // Stuck SEND command, not only stuck OPEN.
  resetTest();
  adoptDirect(0);
  W5100.sockets[0].holdCommand = true;
  offer(tcpTransport.sockets[0], "command\r\n");
  tcpTransport.sockets[0].poll(testMillis, 100, true);
  tcpTransport.sockets[0].poll(testMillis + 100, 100, true);
  assert(tcpTransport.sockets[0].phase == Driver::Closing);

  resetTest();
  adoptDirect(0);
  auto& closing = tcpTransport.sockets[0];
  W5100.sockets[0].holdCommand = true;
  generation = closing.generation;
  closing.close(testMillis);
  closing.poll(testMillis, 100, true);
  closing.poll(testMillis + 100, 100, true);
  assert(closing.phase == Driver::Quarantined && closing.generation == generation);
  assert(initializeEthernet(false)); // Only explicit chip reset releases quarantine.
  assert(closing.generation != generation && !closing.busy());

  resetTest(UINT32_MAX - 200);
  adoptDirect(0);
  offer(tcpTransport.sockets[0], "expired-across-rollover\r\n");
  tcpTransport.sockets[0].poll(testMillis + 500, 100, true);
  assert(tcpTransport.sockets[0].counters.expired == 1 &&
    W5100.sockets[0].sends.empty());
}

void testWrapAndStableReads() {
  resetTest();
  adoptDirect(7);
  auto& socket = tcpTransport.sockets[7];
  auto& chip = W5100.sockets[7];
  chip.txWrite = chip.txRead = 65530;
  std::string frame(240, 'x');
  frame.replace(238, 2, "\r\n");
  offer(socket, frame);
  chip.unstableFree = true;
  socket.poll(testMillis, 100, true);
  assert(chip.sends.empty()); // Two differing register reads: defer, never spin.
  chip.unstableFree = false;
  socket.poll(++testMillis, 100, true);
  assert(chip.sends.size() == 1 && chip.sends[0] == frame);
  assert(chip.txWrite == uint16_t(65530 + 240));
  assert(W5100.sockets[6].tx[2047] == 0); // No adjacent socket overwritten.
  assert(!socket.offer(reinterpret_cast<const uint8_t*>(frame.data()), 241, testMillis));
  socket.poll(++testMillis, 100, true);
  chip.rxWrite = chip.rxRead = chip.rxConsumed = 65530;
  W5100.feed(7, frame);
  uint8_t data[64];
  std::string received;
  while (chip.rxSize) {
    size_t length = socket.receive(data, sizeof(data), testMillis);
    received.append(reinterpret_cast<const char*>(data), length);
    socket.poll(++testMillis, 100, true);
  }
  assert(received == frame && !SPI.active);
}

void testDiagnosticsAndOverflowIsolation() {
  boot();
  uint8_t nmea = connectClient(NMEA_TCP_PORT);
  uint8_t diag = connectClient(DIAG_TCP_PORT);
  ticks(5);
  W5100.sockets[diag].sends.clear();
  // Ring wraps, then asynchronous driver owns its copy while producer reuses ring.
  diagQueueHead = sizeof(diagQueue) - 7;
  std::string text(600, 'd');
  text += "\r\n";
  sendDiagText(text.c_str());
  for (unsigned i = 0; i < 30; ++i) tick();
  std::string output;
  for (const auto& chunk : W5100.sockets[diag].sends) {
    assert(chunk.size() <= Rev41::DiagChunk);
    output += chunk;
  }
  assert(output == text);
  W5100.sockets[diag].ackSend = false;
  sendDiagText("immutable active\r\n");
  ticks(2);
  std::string immutable = W5100.sockets[diag].sends.back();
  sendDiagText(std::string(sizeof(diagQueue), 'z').c_str());
  sendDiagText("overflow");
  assert(diagQueueOverflows == 1 && !diagConnected() && diagQueueLength == 0);
  assert(tcpTransport.sockets[nmea].connected());
  assert(W5100.sockets[diag].sends.back() == immutable);
  ticks(4);
  uint8_t replacement = connectClient(DIAG_TCP_PORT);
  assert(diagConnected() && !tcpCommandOverflow && tcpCommandIndex == 0);
  ticks(5);
  W5100.feed(replacement, std::string(140, 'x') + "\nfreq 2\n");
  tick();
  assert(W5100.sockets[replacement].rxSize == 84); // At most 64 bytes per tick.
  ticks(4);
  assert(OUTPUT_PERIOD_MS_VAR == 500 && !tcpCommandOverflow);
  W5100.sockets[replacement].ackSend = true;
  ticks(10);
  W5100.sockets[replacement].sends.clear();
  processTCPCommand("status");
  ticks(80);
  output.clear();
  for (const auto& chunk : W5100.sockets[replacement].sends) output += chunk;
  assert(output.find("========== STATUS") != std::string::npos);
  assert(output.find("Rev.4.1 TX") != std::string::npos);
  assert(output.find("SEND_OK=") != std::string::npos);
  assert(diagQueueOverflows == 1);
}

void testAdmissionDisconnectAndRecovery() {
  boot();
  for (unsigned i = 0; i < 4; ++i) connectClient(NMEA_TCP_PORT);
  connectClient(DIAG_TCP_PORT);
  ticks(3);
  uint8_t excess = 8;
  for (auto& socket : tcpTransport.sockets)
    if (socket.port == NMEA_TCP_PORT && socket.phase == Driver::Listening) excess = socket.id;
  assert(excess < 8);
  W5100.sockets[excess].status = Rev41::Established;
  tick();
  assert(tcpTransport.sockets[excess].phase == Driver::Closing);
  ticks(3);
  unsigned peers = 0;
  for (auto& socket : tcpTransport.sockets) peers += socket.connected();
  assert(peers == 5);
  uint8_t old = nmeaClients[0].getSocketNumber();
  uint32_t generation = nmeaGeneration[0];
  W5100.sockets[old].status = 0x1c; // CLOSE_WAIT: drop queues and close cooperatively.
  tick();
  assert(nmeaClients[0].getSocketNumber() == 8);
  ticks(4);
  uint8_t next = connectClient(NMEA_TCP_PORT);
  assert(next != old || nmeaGeneration[0] != generation);
  Ethernet.link = 0;
  testMillis += 1000;
  maintainEthernet();
  assert(!ethernetReady && diagQueueLength == 0);
  for (const auto& client : nmeaClients) assert(client.getSocketNumber() == 8);
  ticks(4);
  Ethernet.link = LinkON;
  testMillis += 1000;
  maintainEthernet();
  assert(ethernetReady);
  ticks(4);
  assert(bool(nmeaServer) && bool(diagServer));
  connectClient(NMEA_TCP_PORT);
}

void testUnchangedGgaAndCom2() {
  boot();
  uint8_t id = connectClient(NMEA_TCP_PORT);
  for (int fix : {1, 2, 4, 5}) {
    com2Serial.output.clear();
    testMillis += 100;
    produce(fix);
    ticks(3);
    const auto& frame = W5100.sockets[id].sends.back();
    assert(frame == com2Serial.output);
    // FWM's historical mapping is intentional: HAS -> 2, other moving -> 1.
    // IRI's preserve-input-fix policy must not leak into this revision.
    int expectedFix = fix == 5 ? 2 : 1;
    assert(frame.find("$GPGGA,123519.00,4807.0380,N,01131.0000,E," +
      std::to_string(expectedFix) + ",12,0.8,545.4,M,0.0,M,,") == 0);
    assert(frame.substr(frame.size() - 2) == "\r\n");
    assert(validateNmeaChecksum(frame.substr(0, frame.size() - 2).c_str()));
  }
  movementState = LOCKED;
  lockedValid = true;
  lockedLat = currentLat;
  lockedLon = currentLon;
  lockedAlt = currentAlt;
  testMillis += 100;
  produce(5);
  ticks(3);
  assert(W5100.sockets[id].sends.back().find(",E,4,12,") != std::string::npos);
  // yield must never nest SPI, including COM2-only recovery production.
  ethernetReady = false;
  ethernetRecovering = true;
  testMillis += 100;
  Rev41::Transaction<W5500TransportIO> transaction;
  yield();
  assert(SPI.active);
}

void testLoopProgressUnderDiagStall() {
  boot();
  uint8_t ids[4];
  for (auto& id : ids) id = connectClient(NMEA_TCP_PORT);
  uint8_t diag = connectClient(DIAG_TCP_PORT);
  ticks(5);
  W5100.sockets[diag].ackSend = false;
  sendDiagText("stalled diagnostic connection\r\n");
  gnssStarted = true;
  lastImuRetryMs = testMillis;
  uint32_t frames = com2FramesSent;
  for (unsigned ms = 0; ms < 4000; ++ms) {
    ++testMillis;
    if (ms % 100 == 0) Serial1.feed(gga(5) + "\r\n");
    loop();
    assert(!SPI.active);
  }
  assert(com2FramesSent - frames >= 39);
  assert(tcpTransport.sockets[diag].counters.stalls == 1 && !diagConnected());
  for (auto id : ids) {
    assert(tcpTransport.sockets[id].connected());
    assert(W5100.sockets[id].sends.size() >= 39);
  }
}

int main() {
  testPassiveHandshakeInterleavings();
  testIsolationAndCadence();
  testWholeFramesLatestAndFreshness();
  testRegisterDeadlinesAndReuse();
  testWrapAndStableReads();
  testDiagnosticsAndOverflowIsolation();
  testAdmissionDisconnectAndRecovery();
  testLoopProgressUnderDiagStall();
  testUnchangedGgaAndCom2();
  std::cout << "Rev4.1: transport + full firmware integration tests passed\n";
}
