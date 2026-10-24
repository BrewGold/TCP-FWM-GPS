#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace Rev41 {
enum Reg8 { Status, Command, Interrupt, Mode };
enum Reg16 { Free, TxWrite, RxSize, RxRead, Port };
enum : uint8_t {
  Closed = 0, Init = 0x13, Listen = 0x14, SynRecv = 0x16, Established = 0x17,
  Open = 1, StartListen = 2, Close = 0x10, Send = 0x20, Recv = 0x40,
  SendOK = 0x10, Timeout = 8
};
constexpr size_t SocketCount = 8;
constexpr size_t FrameCapacity = 240;
constexpr size_t DiagChunk = 128;
constexpr uint32_t FreshMs = 500, StallMs = 3000, CommandMs = 100;
struct Counters {
  uint32_t sent = 0, coalesced = 0, expired = 0, errors = 0, stalls = 0;
};

// IO owns one short SPI transaction per operation. No call here yields or waits.
template<class IO> struct Transaction {
  Transaction() { IO::begin(); }
  ~Transaction() { IO::end(); }
};

template<class IO> class Socket {
public:
  enum Phase { Idle, Opening, ListenCommand, Listening, Peer, Closing, Quarantined };
  Phase phase = Idle;
  uint8_t id = 0;
  uint16_t port = 0;
  uint32_t generation = 0;
  bool adopted = false;
  Counters counters;

  void reset() {
    ++generation;
    phase = Idle;
    port = 0;
    adopted = false;
    activeLength = pendingLength = 0;
    sending = commandPending = closeIssued = blocked = started = false;
  }
  bool connected() const { return phase == Peer && adopted; }
  bool busy() const { return activeLength || pendingLength; }
  size_t queued() const { return activeLength + pendingLength; }

  bool listen(uint16_t newPort, uint32_t now) {
    if (phase != Idle) return false;
    Transaction<IO> transaction;
    // Only this transport may allocate sockets after Ethernet.begin().
    if (IO::read8(id, Status) != Closed || IO::read8(id, Command)) return false;
    reset();
    port = newPort;
    IO::write8(id, Mode, 0x21); // TCP, no delayed ACK (Ethernet 2.0.2 default).
    IO::write16(id, Port, port);
    IO::write8(id, Interrupt, 0xff);
    issue(Open, now);
    phase = Opening;
    phaseAt = now;
    return true;
  }

  // Close is scheduled, never EthernetClient.stop(). A stuck chip is quarantined
  // until an explicit whole-chip reset, never handed to a different connection.
  void close(uint32_t now) {
    if (phase == Idle || phase == Closing || phase == Quarantined) return;
    phase = Closing;
    phaseAt = now;
    activeLength = pendingLength = 0;
    adopted = false;
    closeIssued = false;
  }

  bool offer(const uint8_t* data, size_t length, uint32_t now) {
    if (!connected() || !length || length > FrameCapacity) return false;
    if (pendingLength) ++counters.coalesced;
    memcpy(pending, data, length);
    pendingLength = length;
    pendingAt = now;
    return true;
  }

  // For diagnostics: ownership transfers by copying a whole bounded chunk.
  // The caller can consume its ring only after this succeeds.
  bool offerChunk(const uint8_t* data, size_t length, uint32_t now) {
    if (busy() || length > DiagChunk) return false;
    return offer(data, length, now);
  }

  void poll(uint32_t now, uint16_t cadence, bool fresh) {
    if (phase == Idle || phase == Quarantined) return;
    Transaction<IO> transaction;
    uint8_t status = IO::read8(id, Status);
    uint8_t command = IO::read8(id, Command);
    if (phase == Closing) {
      if (!command && status == Closed) {
        reset();
      } else if (uint32_t(now - phaseAt) >= CommandMs) {
        ++counters.errors;
        phase = Quarantined;
      } else if (!closeIssued && !command) {
        issue(Close, now);
        closeIssued = true;
      }
      return;
    }
    if (commandPending) {
      if (command) {
        if (uint32_t(now - commandAt) >= CommandMs) fail(now, true);
        return;
      }
      commandPending = false;
    } else if (command) {
      // Unexpected command: never overwrite it, nor guess who owns the socket.
      fail(now, false);
      return;
    }
    if (phase == Opening) {
      if (uint32_t(now - phaseAt) >= CommandMs) {
        fail(now, true);
      } else if (status == Init) {
        issue(StartListen, now);
        phase = ListenCommand;
      }
      return;
    }
    if (phase == ListenCommand || phase == Listening) {
      // Hardware may finish the passive handshake before LISTEN is observed.
      if (status == Established) phase = Peer;
      else if (status == Listen || status == SynRecv) phase = Listening;
      else if (phase == Listening || status != Init) close(now);
      else if (uint32_t(now - phaseAt) >= CommandMs) fail(now, true);
      return;
    }
    if (status != Established) {
      close(now);
      return;
    }
    uint8_t interrupt = IO::read8(id, Interrupt);
    if (interrupt & Timeout) {
      fail(now, false); // Ambiguous delivery: never retry on this connection.
      return;
    }
    if (sending) {
      if (interrupt & SendOK) {
        IO::write8(id, Interrupt, SendOK);
        activeLength = 0;
        sending = blocked = false;
        ++counters.sent; // W5500 SEND_OK, not application/FWM acknowledgement.
      } else if (uint32_t(now - activeAt) >= StallMs) {
        fail(now, true);
      }
      return;
    }
    if (blocked && uint32_t(now - blockedAt) >= StallMs) {
      fail(now, true);
      return;
    }
    if (fresh && pendingLength && uint32_t(now - pendingAt) >= FreshMs) {
      pendingLength = 0;
      ++counters.expired;
    }
    if (fresh && activeLength && uint32_t(now - activeSampleAt) >= FreshMs) {
      activeLength = 0; // Not copied to hardware yet: still safe to drop whole.
      ++counters.expired;
    }
    if (!activeLength && pendingLength &&
        (!started || uint32_t(now - lastStartAt) >= cadence)) {
      memcpy(active, pending, pendingLength);
      activeLength = pendingLength;
      activeSampleAt = pendingAt;
      pendingLength = 0;
    }
    if (!activeLength) return;
    // Two reads only, not the unbounded "read until stable" library helper.
    uint16_t first = IO::read16(id, Free);
    uint16_t second = IO::read16(id, Free);
    if (first != second || second > IO::bufferSize() || second < activeLength) {
      if (!blocked) { blocked = true; blockedAt = now; }
      return;
    }
    IO::write8(id, Interrupt, SendOK | Timeout);
    uint16_t pointer = IO::read16(id, TxWrite);
    IO::writeTx(id, pointer, active, activeLength);
    IO::write16(id, TxWrite, uint16_t(pointer + activeLength));
    issue(Send, now);
    sending = started = true;
    activeAt = lastStartAt = now;
  }

  size_t receive(uint8_t* data, size_t capacity, uint32_t now) {
    if (!connected() || commandPending || !capacity) return 0;
    Transaction<IO> transaction;
    if (IO::read8(id, Command) || IO::read8(id, Status) != Established) return 0;
    uint16_t first = IO::read16(id, RxSize);
    uint16_t second = IO::read16(id, RxSize);
    if (first != second || second > IO::bufferSize()) return 0;
    size_t length = second < capacity ? second : capacity;
    if (!length) return 0;
    uint16_t pointer = IO::read16(id, RxRead);
    IO::readRx(id, pointer, data, length);
    IO::write16(id, RxRead, uint16_t(pointer + length));
    issue(Recv, now);
    return length;
  }

private:
  uint8_t active[FrameCapacity], pending[FrameCapacity];
  size_t activeLength = 0, pendingLength = 0;
  bool sending = false, commandPending = false, closeIssued = false;
  bool blocked = false, started = false;
  uint32_t phaseAt = 0, commandAt = 0, pendingAt = 0, activeSampleAt = 0;
  uint32_t activeAt = 0, lastStartAt = 0, blockedAt = 0;
  void issue(uint8_t command, uint32_t now) {
    IO::write8(id, Command, command);
    commandPending = true;
    commandAt = now;
  }
  void fail(uint32_t now, bool stall) {
    ++counters.errors;
    if (stall) ++counters.stalls;
    close(now);
  }
};

template<class IO> struct Transport {
  Socket<IO> sockets[SocketCount];
  Transport() { reset(); }
  void reset() {
    for (size_t i = 0; i < SocketCount; ++i) {
      sockets[i].id = i;
      sockets[i].reset();
    }
  }
  void poll(uint32_t now, uint16_t cadence, uint16_t nmeaPort) {
    for (auto& socket : sockets)
      socket.poll(now, socket.port == nmeaPort ? cadence : 0, socket.port == nmeaPort);
  }
  bool listening(uint16_t port) const {
    for (const auto& socket : sockets)
      if (socket.port == port && socket.phase == Socket<IO>::Listening) return true;
    return false;
  }
  void listen(uint16_t port, uint32_t now) {
    for (const auto& socket : sockets)
      if (socket.port == port && (socket.phase == Socket<IO>::Opening ||
          socket.phase == Socket<IO>::ListenCommand ||
          socket.phase == Socket<IO>::Listening)) return;
    for (auto& socket : sockets) {
      if (socket.phase == Socket<IO>::Idle && socket.listen(port, now)) return;
    }
  }
};
} // namespace Rev41
