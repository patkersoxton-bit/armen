// LX16A.h -- driver for Hiwonder LX-16A serial bus servos.
//
// Pin-agnostic by design: this class only knows about an Arduino Stream
// (the common base class behind HardwareSerial, SoftwareSerial, and
// AltSoftSerial). It never touches a pin number itself -- whichever
// serial object you construct at the top of your sketch is what it
// talks through, so the choice of TX/RX pins lives entirely in the
// sketch, not in this file.
//
// Example:
//   #include <SoftwareSerial.h>
//   #include "LX16A.h"
//   SoftwareSerial busPort(rxPin, txPin);  // pick pins that are actually free
//   LX16ABus servos(busPort);
//
//   void setup() {
//     busPort.begin(115200);   // fixed baud rate, required by the servo
//     servos.move(1, 500, 700);
//   }
//
// Protocol notes (Hiwonder LX-series bus servo protocol):
//   Frame: 0x55 0x55 <id> <len> <cmd> <params...> <checksum>
//   len      = paramCount + 3
//   checksum = ~(id + len + cmd + sum(params)) & 0xFF
//   Position units are 0-1000, mapping linearly to 0-240 degrees.
//
// IMPORTANT gotcha found during bring-up: sending frames addressed to two
// different servo IDs back-to-back with no gap was unreliable in testing.
// moveMany() below defaults to a 20ms gap between frames for this reason --
// don't drop it to 0 without re-testing on your actual hardware.
//
// readPosition()'s command byte (SERVO_POS_READ = 28) matches the common
// Hiwonder LX-series protocol table but has NOT been independently verified
// against an authoritative Hiwonder protocol doc from inside this repo --
// double check it against real hardware behavior (or Hiwonder's own
// documentation) before trusting it. A bad reply is easy to detect (wrong
// checksum/length) and readPosition() reports that as a failure rather than
// returning garbage.

#ifndef LX16A_H
#define LX16A_H

#include <Arduino.h>

class LX16ABus {
public:
  explicit LX16ABus(Stream& port) : _port(port) {}

  // Move one servo to `position` (0-1000, i.e. 0-240 degrees) over `timeMs`.
  void move(uint8_t id, uint16_t position, uint16_t timeMs) {
    if (position > 1000) position = 1000;
    uint8_t params[4] = {
      (uint8_t)(position & 0xFF),
      (uint8_t)(position >> 8),
      (uint8_t)(timeMs & 0xFF),
      (uint8_t)(timeMs >> 8)
    };
    send(id, CMD_MOVE_TIME_WRITE, params, 4);
  }

  // Move several servos in one call, each frame separated by gapMs.
  // See the gotcha note above before reducing gapMs.
  void moveMany(const uint8_t* ids, const uint16_t* positions, uint8_t count,
                uint16_t timeMs, uint16_t gapMs = 20) {
    for (uint8_t i = 0; i < count; i++) {
      move(ids[i], positions[i], timeMs);
      if (i + 1 < count) delay(gapMs);
    }
  }

  // Cut (false) or restore (true) holding torque. Cutting torque lets you
  // hand-turn the horn -- useful for confirming a servo is alive/wired.
  void setLoad(uint8_t id, bool on) {
    uint8_t params[1] = { (uint8_t)(on ? 1 : 0) };
    send(id, CMD_LOAD_OR_UNLOAD_WRITE, params, 1);
  }

  // Re-address a servo. Only ever do this with ONE servo on the bus at a
  // time -- if two servos share currentId they'll both try to respond.
  void setId(uint8_t currentId, uint8_t newId) {
    uint8_t params[1] = { newId };
    send(currentId, CMD_ID_WRITE, params, 1);
  }

  // Query a servo's actual measured position (0-1000, i.e. 0-240 degrees).
  // Returns true and fills *position on success; false if the servo didn't
  // reply, or replied with a bad frame (checksum/length/id mismatch) after
  // one retry -- bring-up testing saw an occasional flaky reading on the
  // second servo in a continuous loop (see the file-header note), so this
  // never hands back a value it hasn't checksum-validated.
  bool readPosition(uint8_t id, int16_t* position, uint16_t timeoutMs = 30) {
    for (uint8_t attempt = 0; attempt < 2; attempt++) {
      while (_port.available()) _port.read();  // drop anything stale
      send(id, CMD_POS_READ, nullptr, 0);

      uint8_t frame[8];  // 0x55 0x55 id len cmd posLo posHi checksum
      if (readExactly(frame, sizeof(frame), timeoutMs) &&
          frame[0] == 0x55 && frame[1] == 0x55 &&
          frame[2] == id && frame[3] == 4 && frame[4] == CMD_POS_READ) {
        uint8_t sum = frame[2] + frame[3] + frame[4] + frame[5] + frame[6];
        uint8_t checksum = ~sum;
        if (checksum == frame[7]) {
          *position = (int16_t)(frame[5] | (frame[6] << 8));
          return true;
        }
      }
      // bad/short/garbled reply -- loop retries once before giving up
    }
    return false;
  }

private:
  static const uint8_t CMD_MOVE_TIME_WRITE = 1;
  static const uint8_t CMD_ID_WRITE = 13;
  static const uint8_t CMD_LOAD_OR_UNLOAD_WRITE = 31;
  static const uint8_t CMD_POS_READ = 28;

  Stream& _port;

  bool readExactly(uint8_t* buf, uint8_t len, uint16_t timeoutMs) {
    unsigned long deadline = millis() + timeoutMs;
    uint8_t got = 0;
    while (got < len && millis() < deadline) {
      if (_port.available()) buf[got++] = (uint8_t)_port.read();
    }
    return got == len;
  }

  void send(uint8_t id, uint8_t cmd, const uint8_t* params, uint8_t paramLen) {
    uint8_t length = paramLen + 3;
    uint8_t sum = id + length + cmd;
    for (uint8_t i = 0; i < paramLen; i++) sum += params[i];
    uint8_t checksum = ~sum;

    _port.write((uint8_t)0x55);
    _port.write((uint8_t)0x55);
    _port.write(id);
    _port.write(length);
    _port.write(cmd);
    for (uint8_t i = 0; i < paramLen; i++) _port.write(params[i]);
    _port.write(checksum);
  }
};

#endif
