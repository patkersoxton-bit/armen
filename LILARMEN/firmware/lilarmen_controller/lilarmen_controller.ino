/*
 * LILARMEN — Rotary Encoder Puppet Input Controller
 * Target: Arduino Nano (ATmega328P), 3x incremental rotary encoders
 *         (KY-040-style: CLK, DT, SW, +, GND). No motors — this board only
 *         ever reads, never actuates.
 *
 * LILARMEN is a hand-held scale puppet of Armen (see
 * armin_project_context.md's "puppet input device" plan): physically
 * manipulating it is meant to drive Armen's real joints in real time. Each
 * encoder tracks one joint's relative rotation, exactly analogous to how
 * firmware/stepper_controller.ino tracks a stepper's step count: open
 * loop, meaningful only relative to a "zero" reference pose, calibrated by
 * marking real min/max positions by hand and saving them to EEPROM.
 *
 * The PC side (desktop_app/lilarmen_link.py, desktop_app/lilarmen_teleop.py)
 * maps this board's calibrated range onto Armen's calibrated range per
 * axis — a ratiometric interpolation between two independently calibrated
 * ranges, which absorbs Armen's gearing/size differences automatically.
 * This board never needs to know Armen's gear ratios, step counts, or
 * physical scale.
 *
 * DECODING: polled + time-debounced, not interrupt-driven quadrature.
 * Cheap mechanical encoders (KY-040 and similar) bounce on every contact
 * transition — a naive interrupt-per-edge decoder (e.g. the popular
 * "Encoder" library used in an earlier version of this file) reads that
 * bounce as real transitions, which corrupts its internal state machine
 * and desyncs the count entirely, not just by a little: once desynced,
 * *every* subsequent transition can decode wrong until it happens to
 * resync. Symptoms: erratic jumps, counts running away, "zero" drifting
 * after a few turns, all worse at speed (bounce duration is roughly
 * fixed in time, so a fast turn gives it no time to settle between
 * transitions while a slow one does).
 *
 * Fix used here: each axis is sampled every loop() pass (cheap —
 * digitalRead, no interrupts, no library), but a raw CLK/DT reading is
 * only *accepted* as a new state once it has held steady for
 * DEBOUNCE_US — true time-based debouncing, applied before any Gray-code
 * decoding happens, not after. Accepted transitions are decoded against
 * the standard 2-bit Gray-code cycle (00->01->11->10->00); any transition
 * that doesn't land on an adjacent position in that cycle is ambiguous
 * (can only happen if a transition was missed entirely, which time
 * debouncing does not prevent at extreme speed) and is *dropped rather
 * than guessed* — the reference position resyncs to wherever the encoder
 * actually is instead of committing to a possibly-wrong direction, so a
 * missed micro-step costs a few counts of resolution but never desyncs
 * the whole axis the way the interrupt-driven approach did.
 *
 * If counts are still noisy after this change, the next thing to try is
 * hardware filtering: a 100nF ceramic capacitor from each CLK and DT pin
 * to GND (right at the encoder) is the standard, well-proven fix for
 * bouncy KY-040-style encoders and works alongside this firmware fix, not
 * instead of it. Loose breadboard connections produce identical symptoms
 * to switch bounce and are worth ruling out too — see LILARMEN/README.md.
 *
 * Protocol: newline-delimited JSON at 115200 baud, strict request/response
 * — same design as stepper_controller.ino (see that file and
 * COMMUNICATION_FIX.md for why: self-resynchronizing, no unsolicited
 * telemetry to collide with replies, "id" echoed so the PC can always match
 * a response to its request). Nothing here pushes data on its own; the PC
 * polls get_state at whatever rate it wants live position for
 * (lilarmen_teleop.py polls fast for responsiveness).
 *
 * There is no move/jog/enable/disable/estop command here — this board has
 * no motors and nothing it could do unsafely by itself. Physical safety for
 * the arm it drives lives entirely in Armen's own firmware bounds-clamping
 * and in lilarmen_teleop.py's gating logic, not here.
 *
 * Libraries (Arduino Library Manager):
 *   - ArduinoJson 6.x
 *   - EEPROM (bundled with the IDE)
 * (No encoder library — decoding is hand-rolled above for bounce control.)
 *
 * See LILARMEN/WIRING_SCHEMATIC.md for the full pin table and power notes.
 */

#include <ArduinoJson.h>
#include <EEPROM.h>

#define FW_VERSION "lilarmen-0.2"
#define BAUD_RATE 115200
#define NUM_AXES 3

// Physical wiring. This index is purely this board's own numbering — it
// does NOT have to match Armen's axis numbering. desktop_app/
// lilarmen_teleop.py's AXIS_MAP does that translation in software, so
// which encoder plugs into which header never has to match which Armen
// joint it drives; edit AXIS_MAP there to remap instead of re-wiring here.
// D2/D3 no longer need to be the hardware-interrupt pins now that decoding
// is polled rather than interrupt-driven — any digital pins work equally
// well — but the existing wiring is left as-is since there's no reason to
// change it.
const uint8_t PIN_CLK[NUM_AXES] = {2, 3, 8};
const uint8_t PIN_DT[NUM_AXES]  = {4, 6, 9};
const uint8_t PIN_SW[NUM_AXES]  = {5, 7, 10};

// ---------------------------------------------------------- encoder decode

// How long a raw CLK/DT reading must hold steady before it's trusted.
// Typical mechanical-encoder bounce settles within a few hundred
// microseconds to a few milliseconds. Raise this if counts are still
// noisy (at the cost of being able to track very fast spins); lower it if
// legitimate fast turns feel like they're being undercounted.
const unsigned long DEBOUNCE_US = 1000;

// Position of each 2-bit Gray-code state (clk<<1 | dt) within the 4-state
// quadrature cycle 00 -> 01 -> 11 -> 10 -> 00. Adjacent positions in this
// array are the only legitimate single-step transitions; anything else
// means a transition was missed and is not a value pair we can trust.
const int8_t GRAY_POSITION[4] = {0, 1, 3, 2};  // index by raw state 00,01,10,11

long axisCount[NUM_AXES];
uint8_t stableState[NUM_AXES];   // last accepted (debounced) 2-bit state
uint8_t rawState[NUM_AXES];      // most recent raw sample, may be mid-bounce
unsigned long rawChangeAt[NUM_AXES];

void initEncoders() {
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    pinMode(PIN_CLK[i], INPUT_PULLUP);
    pinMode(PIN_DT[i], INPUT_PULLUP);
    uint8_t s = (digitalRead(PIN_CLK[i]) << 1) | digitalRead(PIN_DT[i]);
    stableState[i] = s;
    rawState[i] = s;
    rawChangeAt[i] = micros();
    axisCount[i] = 0;
  }
}

void pollEncoders() {
  unsigned long now = micros();
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    uint8_t s = (digitalRead(PIN_CLK[i]) << 1) | digitalRead(PIN_DT[i]);
    if (s != rawState[i]) {
      rawState[i] = s;
      rawChangeAt[i] = now;
      continue;
    }
    if (s == stableState[i]) continue;
    if ((unsigned long)(now - rawChangeAt[i]) < DEBOUNCE_US) continue;

    // s has held steady for DEBOUNCE_US and differs from the last accepted
    // state — apply it.
    int8_t delta = GRAY_POSITION[s] - GRAY_POSITION[stableState[i]];
    if (delta == 3) delta = -1;       // wrapped forward past the cycle end
    else if (delta == -3) delta = 1;  // wrapped backward past the cycle start
    if (delta == 1 || delta == -1) {
      axisCount[i] += delta;
    }
    // delta == 0 can't happen here (s != stableState[i]); delta == +-2 is
    // the ambiguous "missed a state" case — resync stableState without
    // touching the count, deliberately, rather than guess a direction.
    stableState[i] = s;
  }
}

// Button roles are a PC-side convention (see lilarmen_teleop.py) — this
// firmware only reports debounced raw state, it never acts on a press
// itself:
//   axis 0 SW: toggle teleop enable/disable
//   axis 1 SW: reserved, reported but currently unused
//   axis 2 SW: PC-side E-STOP of Armen — the one case worth a physical
//              button even before teleop mapping is dialed in
const unsigned long BUTTON_DEBOUNCE_MS = 30;
bool buttonState[NUM_AXES];
bool buttonRaw[NUM_AXES];
unsigned long buttonChangeAt[NUM_AXES];

enum SysState : uint8_t { ST_READY, ST_CAL };
SysState sysState = ST_READY;

bool referenced = false;  // "zero" has run since boot
bool calibrated = false;

struct CalData {
  uint32_t magic;
  long minBound[NUM_AXES];
  long maxBound[NUM_AXES];
};
const uint32_t CAL_MAGIC = 0x4C494C31UL;  // "LIL1"
CalData cal;
bool markedMin[NUM_AXES];
bool markedMax[NUM_AXES];

char lineBuf[160];
uint8_t lineLen = 0;
bool lineOverflow = false;
long reqId = -1;
char outBuf[320];

void setup() {
  Serial.begin(BAUD_RATE);
  initEncoders();
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    // Most KY-040-style modules already carry onboard pull-ups to VCC on
    // CLK/DT/SW; the internal AVR pull-up enabled here is redundant with
    // that, which is harmless (just a slightly stronger pull-up).
    pinMode(PIN_SW[i], INPUT_PULLUP);
    buttonRaw[i] = buttonState[i] = digitalRead(PIN_SW[i]) == LOW;
    buttonChangeAt[i] = 0;
    markedMin[i] = false;
    markedMax[i] = false;
  }
  loadCal();
}

void loop() {
  pollSerial();
  pollEncoders();
  pollButtons();
}

void pollButtons() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    bool raw = digitalRead(PIN_SW[i]) == LOW;
    if (raw != buttonRaw[i]) {
      buttonRaw[i] = raw;
      buttonChangeAt[i] = now;
    } else if (raw != buttonState[i] && (now - buttonChangeAt[i]) >= BUTTON_DEBOUNCE_MS) {
      buttonState[i] = raw;
    }
  }
}

// ---------------------------------------------------------------- serial in

void pollSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineOverflow) {
        lineOverflow = false;
        lineLen = 0;
        reqId = -1;
        sendErr("error", F("line too long"));
      } else if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine();
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

void handleLine() {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, lineBuf);
  reqId = doc["id"] | -1L;

  if (err) {
    sendErr("error", F("invalid JSON"));
    return;
  }
  const char* cmd = doc["cmd"];
  if (!cmd) {
    sendErr("error", F("missing cmd"));
    return;
  }

  if      (strcmp(cmd, "ping") == 0)       cmdPing();
  else if (strcmp(cmd, "get_state") == 0)  sendState();
  else if (strcmp(cmd, "get_cal") == 0)    sendCal();
  else if (strcmp(cmd, "zero") == 0)       cmdZero();
  else if (strcmp(cmd, "cal_start") == 0)  cmdCalStart();
  else if (strcmp(cmd, "cal_end") == 0)    cmdCalEnd();
  else if (strcmp(cmd, "mark") == 0)       cmdMark(doc);
  else if (strcmp(cmd, "save_cal") == 0)   cmdSaveCal();
  else sendErr(cmd, F("unknown command"));
}

// ---------------------------------------------------------------- commands

void cmdPing() {
  char idStr[16];
  idField(idStr, sizeof(idStr));
  snprintf_P(outBuf, sizeof(outBuf),
             PSTR("{\"cmd\":\"ping\",\"status\":\"ok\"%s,\"fw\":\"" FW_VERSION "\",\"axes\":%d}"),
             idStr, NUM_AXES);
  sendBuffered();
}

void cmdZero() {
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    // Only the accumulated count resets — stableState/rawState/rawChangeAt
    // must NOT be touched, they track the encoder's actual current pin
    // state continuously and resetting them here would look like a
    // spurious transition on the very next sample.
    axisCount[i] = 0;
    markedMin[i] = false;
    markedMax[i] = false;
  }
  referenced = true;
  sendOk("zero", F("all axes zeroed at current pose"));
}

void cmdCalStart() {
  sysState = ST_CAL;
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    markedMin[i] = false;
    markedMax[i] = false;
  }
  sendOk("cal_start", F("calibration mode; zero at reference pose, then move each axis by hand and mark"));
}

void cmdCalEnd() {
  if (sysState == ST_CAL) sysState = ST_READY;
  sendOk("cal_end", F("calibration mode exited"));
}

void cmdMark(JsonDocument& doc) {
  if (sysState != ST_CAL) {
    sendErr("mark", F("mark only allowed in calibration mode"));
    return;
  }
  if (!referenced) {
    sendErr("mark", F("run zero at the reference pose first"));
    return;
  }
  int axis = doc["axis"] | -1;
  const char* which = doc["which"];
  if (axis < 0 || axis >= NUM_AXES || !which) {
    sendErr("mark", F("need axis 0-2 and which:min|max"));
    return;
  }
  long pos = axisCount[axis];
  bool isMin;
  if (strcmp(which, "min") == 0) {
    cal.minBound[axis] = pos;
    markedMin[axis] = true;
    isMin = true;
  } else if (strcmp(which, "max") == 0) {
    cal.maxBound[axis] = pos;
    markedMax[axis] = true;
    isMin = false;
  } else {
    sendErr("mark", F("which must be min or max"));
    return;
  }
  char idStr[16];
  idField(idStr, sizeof(idStr));
  snprintf_P(outBuf, sizeof(outBuf),
             PSTR("{\"cmd\":\"mark\",\"status\":\"ok\"%s,\"axis\":%d,\"which\":\"%S\",\"value\":%ld}"),
             idStr, axis, isMin ? PSTR("min") : PSTR("max"), pos);
  sendBuffered();
}

void cmdSaveCal() {
  // Unmarked axes are stored as min == max == 0, meaning "not usable for
  // ratiometric mapping" — lilarmen_teleop.py skips forwarding any axis in
  // that state and logs it once, since there's no meaningful "how far
  // through its travel is this encoder" without two real marked endpoints
  // (unlike Armen's FREE axes, which are intentionally-unbounded
  // continuous-rotation joints, not just unfinished calibration).
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    if (!markedMin[i] || !markedMax[i]) {
      cal.minBound[i] = 0;
      cal.maxBound[i] = 0;
    } else if (cal.minBound[i] > cal.maxBound[i]) {
      sendErr("save_cal", F("min > max on an axis; re-mark it"));
      return;
    }
  }
  cal.magic = CAL_MAGIC;
  EEPROM.put(0, cal);
  calibrated = true;
  sendOk("save_cal", F("bounds saved to EEPROM"));
}

// ---------------------------------------------------------------- responses

void sendBuffered() {
  Serial.print(outBuf);
  Serial.write('\n');
}

void idField(char* buf, size_t n) {
  if (reqId >= 0) {
    snprintf_P(buf, n, PSTR(",\"id\":%ld"), reqId);
  } else {
    buf[0] = '\0';
  }
}

PGM_P bstr(bool b) {
  return b ? PSTR("true") : PSTR("false");
}

void sendOk(const char* cmd, const __FlashStringHelper* msg) {
  char idStr[16];
  idField(idStr, sizeof(idStr));
  snprintf_P(outBuf, sizeof(outBuf),
             PSTR("{\"cmd\":\"%s\",\"status\":\"ok\"%s,\"message\":\"%S\"}"),
             cmd, idStr, (PGM_P)msg);
  sendBuffered();
}

void sendErr(const char* cmd, const __FlashStringHelper* err) {
  char idStr[16];
  idField(idStr, sizeof(idStr));
  snprintf_P(outBuf, sizeof(outBuf),
             PSTR("{\"cmd\":\"%s\",\"status\":\"error\"%s,\"error\":\"%S\"}"),
             cmd, idStr, (PGM_P)err);
  sendBuffered();
}

void sendState() {
  char idStr[16];
  idField(idStr, sizeof(idStr));
  snprintf_P(outBuf, sizeof(outBuf),
             PSTR("{\"cmd\":\"get_state\",\"status\":\"ok\"%s,\"state\":\"%S\","
                  "\"referenced\":%S,\"calibrated\":%S,"
                  "\"pos\":[%ld,%ld,%ld],\"buttons\":[%S,%S,%S]}"),
             idStr, (PGM_P)(sysState == ST_CAL ? F("cal") : F("ready")),
             bstr(referenced), bstr(calibrated),
             axisCount[0], axisCount[1], axisCount[2],
             bstr(buttonState[0]), bstr(buttonState[1]), bstr(buttonState[2]));
  sendBuffered();
}

void sendCal() {
  char idStr[16];
  idField(idStr, sizeof(idStr));
  snprintf_P(outBuf, sizeof(outBuf),
             PSTR("{\"cmd\":\"get_cal\",\"status\":\"ok\"%s,\"calibrated\":%S,"
                  "\"min\":[%ld,%ld,%ld],\"max\":[%ld,%ld,%ld],"
                  "\"marked_min\":[%S,%S,%S],\"marked_max\":[%S,%S,%S]}"),
             idStr, bstr(calibrated),
             cal.minBound[0], cal.minBound[1], cal.minBound[2],
             cal.maxBound[0], cal.maxBound[1], cal.maxBound[2],
             bstr(markedMin[0]), bstr(markedMin[1]), bstr(markedMin[2]),
             bstr(markedMax[0]), bstr(markedMax[1]), bstr(markedMax[2]));
  sendBuffered();
}

// ---------------------------------------------------------------- helpers

void loadCal() {
  EEPROM.get(0, cal);
  calibrated = (cal.magic == CAL_MAGIC);
  if (calibrated) {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
      if (cal.minBound[i] > cal.maxBound[i]) calibrated = false;
    }
  }
  if (!calibrated) {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
      cal.minBound[i] = 0;
      cal.maxBound[i] = 0;
    }
  }
}
