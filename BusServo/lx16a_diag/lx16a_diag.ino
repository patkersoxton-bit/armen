// Interactive diagnostic: isolates why servo ID2 doesn't respond to
// Arduino commands in certain orderings, even though it works fine
// alone and from the PC terminal app.
//
// For each test case:
//   1. Both servos are reset to a known BASELINE position individually
//      (isolated sends, known-good pattern) so every test starts clean.
//   2. The test's specific command pattern is sent, targeting a position
//      far enough from BASELINE that any movement is unambiguous.
//   3. You watch the servo(s) and type 1 (moved as expected) or 0 (did
//      not move / wrong servo moved) into the Serial Monitor, then hit
//      Enter.
//   4. The result is logged and a full summary table is printed at the
//      end of the pass. Copy that table back out when done.
//
// Serial Monitor: 9600 baud, line ending doesn't matter.
// Wiring: AltSoftSerial pins 8 (RX) / 9 (TX), same as lx16a_sweep.

#include <AltSoftSerial.h>

AltSoftSerial busSerial;

const uint8_t SERVO_MOVE_TIME_WRITE = 1;
const uint8_t SERVO_ID_1 = 1;
const uint8_t SERVO_ID_2 = 2;

const uint16_t BASELINE = 500;
const uint16_t TARGET = 900;      // far enough from BASELINE (400 units,
                                   // ~96 degrees) to always be obvious
const uint16_t MOVE_TIME_MS = 700;
const uint16_t SETTLE_MS = 1000;  // wait for a move to physically finish

void lx16aSend(uint8_t id, uint8_t cmd, const uint8_t* params, uint8_t paramLen) {
  uint8_t length = paramLen + 3;
  uint8_t sum = id + length + cmd;
  for (uint8_t i = 0; i < paramLen; i++) sum += params[i];
  uint8_t checksum = ~sum;

  busSerial.write((uint8_t)0x55);
  busSerial.write((uint8_t)0x55);
  busSerial.write(id);
  busSerial.write(length);
  busSerial.write(cmd);
  for (uint8_t i = 0; i < paramLen; i++) busSerial.write(params[i]);
  busSerial.write(checksum);
}

void moveServo(uint8_t id, uint16_t position, uint16_t timeMs) {
  uint8_t params[4] = {
    (uint8_t)(position & 0xFF),
    (uint8_t)(position >> 8),
    (uint8_t)(timeMs & 0xFF),
    (uint8_t)(timeMs >> 8)
  };
  lx16aSend(id, SERVO_MOVE_TIME_WRITE, params, 4);
}

// Resets both servos to BASELINE using isolated sends (known to work
// individually), so test results aren't confounded by an unreliable reset.
void resetBaseline() {
  Serial.println("  (resetting both servos to baseline...)");
  moveServo(SERVO_ID_1, BASELINE, MOVE_TIME_MS);
  delay(SETTLE_MS);
  moveServo(SERVO_ID_2, BASELINE, MOVE_TIME_MS);
  delay(SETTLE_MS);
}

// Blocks until the user types a 1 or 0 into the Serial Monitor.
int readYesNo() {
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '0') return 0;
      if (c == '1') return 1;
      // ignore newlines/other input
    }
  }
}

struct TestCase {
  const char* name;
};

const int NUM_TESTS = 6;
const char* testNames[NUM_TESTS] = {
  "ID1 only (isolated)",
  "ID2 only (isolated)",
  "ID1 then ID2, 20ms gap (original order)",
  "ID2 then ID1, 20ms gap (swapped order)",
  "ID1 then ID2, 200ms gap",
  "ID2 then ID1, 200ms gap"
};
int results[NUM_TESTS];

void runTest(int index) {
  switch (index) {
    case 0:
      moveServo(SERVO_ID_1, TARGET, MOVE_TIME_MS);
      break;
    case 1:
      moveServo(SERVO_ID_2, TARGET, MOVE_TIME_MS);
      break;
    case 2:
      moveServo(SERVO_ID_1, TARGET, MOVE_TIME_MS);
      delay(20);
      moveServo(SERVO_ID_2, TARGET, MOVE_TIME_MS);
      break;
    case 3:
      moveServo(SERVO_ID_2, TARGET, MOVE_TIME_MS);
      delay(20);
      moveServo(SERVO_ID_1, TARGET, MOVE_TIME_MS);
      break;
    case 4:
      moveServo(SERVO_ID_1, TARGET, MOVE_TIME_MS);
      delay(200);
      moveServo(SERVO_ID_2, TARGET, MOVE_TIME_MS);
      break;
    case 5:
      moveServo(SERVO_ID_2, TARGET, MOVE_TIME_MS);
      delay(200);
      moveServo(SERVO_ID_1, TARGET, MOVE_TIME_MS);
      break;
  }
}

void setup() {
  Serial.begin(9600);
  busSerial.begin(115200);
  delay(500);
  Serial.println("=== LX-16A interactive diagnostic ===");
  Serial.println("For each test: watch the servos, then type 1 (moved as");
  Serial.println("expected) or 0 (did not / wrong one moved) and press Enter.");
  Serial.println();
}

void loop() {
  for (int i = 0; i < NUM_TESTS; i++) {
    resetBaseline();

    Serial.print("TEST ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(testNames[i]);
    Serial.print(" -> target ");
    Serial.println(TARGET);

    runTest(i);
    delay(SETTLE_MS);

    Serial.print("  Result? (1=moved as expected, 0=failed): ");
    int result = readYesNo();
    Serial.println(result);
    results[i] = result;
    Serial.println();
  }

  Serial.println("=== SUMMARY ===");
  for (int i = 0; i < NUM_TESTS; i++) {
    Serial.print("TEST ");
    Serial.print(i + 1);
    Serial.print(" [");
    Serial.print(testNames[i]);
    Serial.print("]: ");
    Serial.println(results[i] ? "PASS" : "FAIL");
  }
  Serial.println();
  Serial.println("Copy everything above and send it back. Halting now.");

  while (true) {
    delay(1000); // stop here, don't loop again automatically
  }
}
