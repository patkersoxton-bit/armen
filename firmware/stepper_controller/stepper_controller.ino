/*
 * Armen — Stepper Arm Controller
 * Target: Arduino Uno + CNC Shield V3 + DRV8825 drivers + NEMA 17 steppers
 *
 * Replaces the ESP32/PCA9685 servo firmware: the actual kit (parts.jpg) is a
 * CNC stepper stack, so control is open-loop step counting instead of servo
 * angles. Wiring and calibration procedure: see STEPPER_MIGRATION.md.
 *
 * Protocol: newline-delimited JSON at 115200 baud, strict request/response.
 * The firmware prints exactly one JSON line per received command line and
 * echoes the request's "id" so the PC can match replies. It never sends
 * unsolicited output — this eliminates the telemetry/response collision and
 * framing-desync bugs the old CBOR length-prefix protocol suffered from
 * (see COMMUNICATION_FIX.md).
 *
 * Responses are streamed through sendBuffered(), which keeps calling the
 * steppers between bytes whenever the UART TX buffer is full — a blocking
 * Serial.print would otherwise freeze motion for ~10 ms per response and
 * cause audible stutter when the GUI polls during a jog.
 *
 * Position model (open loop — read this before trusting "pos"):
 *   - Positions are step counts relative to the pose where "zero" was last
 *     run. They are meaningless until zeroed and are lost whenever drivers
 *     are disabled or power is cycled (the arm can then be moved by hand).
 *   - Soft bounds (min/max steps per axis) are captured in calibration mode
 *     by jogging to each mechanical extreme and marking it, then saved to
 *     EEPROM. Bounds are only valid if "zero" is run at the same physical
 *     reference pose every session.
 *
 * Libraries (Arduino Library Manager):
 *   - AccelStepper
 *   - ArduinoJson 6.x (v7 also works but costs extra flash/RAM on the Uno)
 */

#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

#define FW_VERSION "stepper-0.4"
#define BAUD_RATE 115200
#define NUM_AXES 4

// CNC Shield V3 pin map. The A axis requires the two jumpers next to the A
// driver socket so A-STEP/A-DIR route to D12/D13.
#define PIN_ENABLE 8  // shared driver enable, active LOW

const uint8_t LIMIT_PINS[3] = {9, 10, 11};  // X, Y, Z endstop headers

AccelStepper steppers[NUM_AXES] = {
  AccelStepper(AccelStepper::DRIVER, 2, 5),   // X: STEP D2, DIR D5
  AccelStepper(AccelStepper::DRIVER, 3, 6),   // Y: STEP D3, DIR D6
  AccelStepper(AccelStepper::DRIVER, 4, 7),   // Z: STEP D4, DIR D7
  AccelStepper(AccelStepper::DRIVER, 12, 13)  // A: STEP D12, DIR D13
};

// Motion tuning, in steps. Boot defaults are deliberately conservative so
// they are sane in ANY microstep mode — 500 steps/s is 2.5 rev/s even in
// full-step (no jumper caps installed). With 1/32 microstepping, raise to
// ~3000/6000 via the set_motion command (the calibration GUI applies its
// motion-tuning values automatically on connect).
// Note: the Uno can generate ~4000 steps/s in total, shared across axes
// that move simultaneously; single-axis jogs can use all of it.
float maxSpeedSteps = 500.0;
float accelSteps = 1500.0;
// Sanity ceiling for a single jog command; keeps a typo survivable. This
// firmware only ever counts raw MOTOR steps — it has no notion of gear
// ratios, so all degree/ratio math for geared axes lives on the PC side
// (desktop_app/stepper_link.py). A joint behind a reduction gearbox needs
// many more raw steps to cover the same real travel than a direct-drive
// one (e.g. the base's 37:1 gearbox means "one output revolution" is 37
// motor revolutions = 118,400 steps at 3200 steps/rev), so this is sized
// for the largest ratio in use, not for direct drive. Keep in sync with
// MAX_JOG_STEPS in stepper_link.py.
const long MAX_JOG_STEPS = 500000;

enum SysState : uint8_t { ST_READY, ST_CAL, ST_ESTOP };
SysState sysState = ST_READY;

bool motorsEnabled = false;
bool referenced = false;   // "zero" has run and drivers stayed enabled since
bool calibrated = false;

struct CalData {
  uint32_t magic;
  long minBound[NUM_AXES];
  long maxBound[NUM_AXES];
};
const uint32_t CAL_MAGIC = 0x41524D31UL;  // "ARM1"
CalData cal;
bool markedMin[NUM_AXES];
bool markedMax[NUM_AXES];

char lineBuf[160];
uint8_t lineLen = 0;
bool lineOverflow = false;
long reqId = -1;
char outBuf[288];

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);  // drivers off until explicitly enabled

  for (uint8_t i = 0; i < 3; i++) {
    pinMode(LIMIT_PINS[i], INPUT_PULLUP);
  }

  for (uint8_t i = 0; i < NUM_AXES; i++) {
    steppers[i].setMaxSpeed(maxSpeedSteps);
    steppers[i].setAcceleration(accelSteps);
    markedMin[i] = false;
    markedMax[i] = false;
  }

  loadCal();
}

void loop() {
  pollSerial();
  runSteppers();
}

void runSteppers() {
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    steppers[i].run();
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
  else if (strcmp(cmd, "enable") == 0)     cmdEnable();
  else if (strcmp(cmd, "disable") == 0)    cmdDisable();
  else if (strcmp(cmd, "zero") == 0)       cmdZero();
  else if (strcmp(cmd, "cal_start") == 0)  cmdCalStart();
  else if (strcmp(cmd, "cal_end") == 0)    cmdCalEnd();
  else if (strcmp(cmd, "jog") == 0)        cmdJog(doc);
  else if (strcmp(cmd, "mark") == 0)       cmdMark(doc);
  else if (strcmp(cmd, "save_cal") == 0)   cmdSaveCal();
  else if (strcmp(cmd, "move") == 0)       cmdMove(doc);
  else if (strcmp(cmd, "set_motion") == 0) cmdSetMotion(doc);
  else if (strcmp(cmd, "stop") == 0)       cmdStop();
  else if (strcmp(cmd, "estop") == 0)      cmdEstop();
  else if (strcmp(cmd, "reset") == 0)      cmdReset();
  else sendErr(cmd, F("unknown command"));
}

// ---------------------------------------------------------------- commands

void cmdPing() {
  char idStr[16];
  idField(idStr, sizeof(idStr));
  snprintf_P(outBuf, sizeof(outBuf),
             PSTR("{\"cmd\":\"ping\",\"status\":\"ok\"%s,\"fw\":\"" FW_VERSION "\"}"),
             idStr);
  sendBuffered();
}

void cmdEnable() {
  digitalWrite(PIN_ENABLE, LOW);
  motorsEnabled = true;
  sendOk("enable", F("drivers enabled"));
}

void cmdDisable() {
  haltAll();
  digitalWrite(PIN_ENABLE, HIGH);
  motorsEnabled = false;
  // Without holding torque the arm can move (or fall!) — step counts are
  // no longer trustworthy.
  referenced = false;
  sendOk("disable", F("drivers disabled; position reference lost"));
}

void cmdZero() {
  if (anyMoving()) {
    sendErr("zero", F("wait for motion to stop"));
    return;
  }
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    steppers[i].setCurrentPosition(0);
    markedMin[i] = false;
    markedMax[i] = false;
  }
  referenced = true;
  sendOk("zero", F("all axes zeroed at current pose"));
}

void cmdCalStart() {
  if (!motorsEnabled) {
    sendErr("cal_start", F("enable motors first"));
    return;
  }
  if (sysState == ST_ESTOP) {
    sendErr("cal_start", F("reset estop first"));
    return;
  }
  sysState = ST_CAL;
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    markedMin[i] = false;
    markedMax[i] = false;
  }
  sendOk("cal_start", F("calibration mode; zero at reference pose, then jog and mark"));
}

void cmdCalEnd() {
  if (sysState == ST_CAL) sysState = ST_READY;
  sendOk("cal_end", F("calibration mode exited"));
}

void cmdJog(JsonDocument& doc) {
  if (sysState != ST_CAL) {
    sendErr("jog", F("jog only allowed in calibration mode"));
    return;
  }
  if (!motorsEnabled) {
    sendErr("jog", F("motors disabled"));
    return;
  }
  int axis = doc["axis"] | -1;
  long steps = doc["steps"] | 0L;
  if (axis < 0 || axis >= NUM_AXES) {
    sendErr("jog", F("axis must be 0-3"));
    return;
  }
  if (steps == 0 || steps > MAX_JOG_STEPS || steps < -MAX_JOG_STEPS) {
    sendErr("jog", F("steps must be non-zero and within +/-500000"));
    return;
  }
  steppers[axis].move(steps);
  sendOk("jog", F("jogging"));
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
  if (anyMoving()) {
    sendErr("mark", F("wait for motion to stop"));
    return;
  }
  int axis = doc["axis"] | -1;
  const char* which = doc["which"];
  if (axis < 0 || axis >= NUM_AXES || !which) {
    sendErr("mark", F("need axis 0-3 and which:min|max"));
    return;
  }
  long pos = steppers[axis].currentPosition();
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
  // Unmarked axes are stored as min == max == 0, which means FREE (no
  // clamping) — for continuous-rotation joints like the swashplate base.
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
  sendOk("save_cal", F("bounds saved to EEPROM (unmarked axes are FREE)"));
}

void cmdMove(JsonDocument& doc) {
  if (sysState == ST_ESTOP) {
    sendErr("move", F("reset estop first"));
    return;
  }
  if (sysState == ST_CAL) {
    sendErr("move", F("exit calibration mode first"));
    return;
  }
  if (!motorsEnabled) {
    sendErr("move", F("motors disabled"));
    return;
  }
  if (!referenced) {
    sendErr("move", F("not referenced; run zero at the reference pose"));
    return;
  }
  if (!calibrated) {
    sendErr("move", F("no saved bounds; run calibration first"));
    return;
  }
  JsonArray targets = doc["targets"];
  if (targets.isNull() || targets.size() != NUM_AXES) {
    sendErr("move", F("targets must be an array of 4 step positions"));
    return;
  }
  float speed = doc["speed"] | 0.5f;
  speed = constrain(speed, 0.05f, 1.0f);

  for (uint8_t i = 0; i < NUM_AXES; i++) {
    long t = targets[i] | 0L;
    // min == max marks a FREE axis (e.g. the 360° swashplate base): no
    // clamping. Axes with real marked bounds are clamped as before.
    if (cal.minBound[i] != cal.maxBound[i]) {
      t = constrain(t, cal.minBound[i], cal.maxBound[i]);
    }
    steppers[i].setMaxSpeed(maxSpeedSteps * speed);
    steppers[i].moveTo(t);
  }
  sendOk("move", F("moving (bounded axes clamped)"));
}

void cmdSetMotion(JsonDocument& doc) {
  float ms = doc["max_speed"] | maxSpeedSteps;
  float ac = doc["accel"] | accelSteps;
  maxSpeedSteps = constrain(ms, 100.0f, 4000.0f);
  accelSteps = constrain(ac, 100.0f, 20000.0f);
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    steppers[i].setMaxSpeed(maxSpeedSteps);
    steppers[i].setAcceleration(accelSteps);
  }
  char idStr[16];
  idField(idStr, sizeof(idStr));
  snprintf_P(outBuf, sizeof(outBuf),
             PSTR("{\"cmd\":\"set_motion\",\"status\":\"ok\"%s,\"max_speed\":%ld,\"accel\":%ld}"),
             idStr, (long)maxSpeedSteps, (long)accelSteps);
  sendBuffered();
}

void cmdStop() {
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    steppers[i].stop();  // decelerate to a stop
  }
  sendOk("stop", F("decelerating"));
}

void cmdEstop() {
  haltAll();  // instant halt; drivers stay enabled so the arm holds position
  sysState = ST_ESTOP;
  sendOk("estop", F("halted; drivers still holding; send reset to clear"));
}

void cmdReset() {
  if (sysState == ST_ESTOP) sysState = ST_READY;
  sendOk("reset", F("ready"));
}

// ---------------------------------------------------------------- responses

// Stream outBuf plus a newline, servicing the steppers whenever the UART TX
// buffer is full so responses never stall motion.
void sendBuffered() {
  size_t len = strlen(outBuf);
  for (size_t i = 0; i < len; i++) {
    while (Serial.availableForWrite() == 0) runSteppers();
    Serial.write(outBuf[i]);
  }
  while (Serial.availableForWrite() == 0) runSteppers();
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
                  "\"enabled\":%S,\"referenced\":%S,\"calibrated\":%S,\"moving\":%S,"
                  "\"pos\":[%ld,%ld,%ld,%ld],\"limits\":[%S,%S,%S]}"),
             idStr, (PGM_P)stateName(),
             bstr(motorsEnabled), bstr(referenced), bstr(calibrated), bstr(anyMoving()),
             steppers[0].currentPosition(), steppers[1].currentPosition(),
             steppers[2].currentPosition(), steppers[3].currentPosition(),
             bstr(digitalRead(LIMIT_PINS[0]) == LOW),   // pressed pulls to GND
             bstr(digitalRead(LIMIT_PINS[1]) == LOW),
             bstr(digitalRead(LIMIT_PINS[2]) == LOW));
  sendBuffered();
}

void sendCal() {
  char idStr[16];
  idField(idStr, sizeof(idStr));
  snprintf_P(outBuf, sizeof(outBuf),
             PSTR("{\"cmd\":\"get_cal\",\"status\":\"ok\"%s,\"calibrated\":%S,"
                  "\"min\":[%ld,%ld,%ld,%ld],\"max\":[%ld,%ld,%ld,%ld],"
                  "\"marked_min\":[%S,%S,%S,%S],\"marked_max\":[%S,%S,%S,%S]}"),
             idStr, bstr(calibrated),
             cal.minBound[0], cal.minBound[1], cal.minBound[2], cal.minBound[3],
             cal.maxBound[0], cal.maxBound[1], cal.maxBound[2], cal.maxBound[3],
             bstr(markedMin[0]), bstr(markedMin[1]), bstr(markedMin[2]), bstr(markedMin[3]),
             bstr(markedMax[0]), bstr(markedMax[1]), bstr(markedMax[2]), bstr(markedMax[3]));
  sendBuffered();
}

// ---------------------------------------------------------------- helpers

const __FlashStringHelper* stateName() {
  switch (sysState) {
    case ST_CAL:   return F("cal");
    case ST_ESTOP: return F("estop");
    default:       return F("ready");
  }
}

bool anyMoving() {
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    if (steppers[i].distanceToGo() != 0) return true;
  }
  return false;
}

void haltAll() {
  // setCurrentPosition() zeroes speed and makes the current position the
  // target — an immediate stop that keeps the step reference intact.
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    steppers[i].setCurrentPosition(steppers[i].currentPosition());
  }
}

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
