// Hiwonder LX-16A bus servo manual control console — Arduino Uno
//
// This is the dev/test rig for the LX16A.h driver (see the LX16A tab in
// this sketch; the canonical, actively-maintained copy is now
// firmware/stepper_controller/LX16A.h in the real stepper+servo project —
// this local copy predates its readPosition() addition and may drift).
// Type commands into the Serial Monitor to drive servos on demand, plus an
// optional background auto-sweep you can toggle on/off while still sending
// manual commands at the same time.
//
// ---- Pins: THE ONLY THING TO CHANGE WHEN PORTING THIS ELSEWHERE ----
// AltSoftSerial is fixed to pins 9 (TX) / 8 (RX) on an Uno -- that's what
// this dev rig uses, confirmed working. On the real project, once you know
// which pins the stepper shield leaves free, swap the two lines below
// (BUS_TX_PIN/BUS_RX_PIN are documentation only, AltSoftSerial doesn't
// take pin args) for e.g.:
//   SoftwareSerial busPort(rxPin, txPin);   // any two free digital pins
// and pass `busPort` into LX16ABus the same way. Nothing else in this
// file or in LX16A.h needs to change -- the driver only knows about a
// Stream, not a pin number.
#include <AltSoftSerial.h>
const uint8_t BUS_TX_PIN = 9; // fixed by AltSoftSerial on Uno, informational only
const uint8_t BUS_RX_PIN = 8; // fixed by AltSoftSerial on Uno, informational only
AltSoftSerial busPort;
// ----------------------------------------------------------------------

#include "LX16A.h"
LX16ABus servos(busPort);

const uint8_t SERVO_ID_1 = 1;
const uint8_t SERVO_ID_2 = 2;

const uint16_t BASELINE = 500;
const uint16_t DEFAULT_MOVE_TIME_MS = 700;

uint16_t pairGapMs = 20;

void resetBaseline() {
  servos.move(SERVO_ID_1, BASELINE, DEFAULT_MOVE_TIME_MS);
  delay(1000);
  servos.move(SERVO_ID_2, BASELINE, DEFAULT_MOVE_TIME_MS);
  delay(1000);
  Serial.println(F("Reset both servos to baseline (500)."));
}

// --- background auto-sweep (optional, toggled with 'a') ---

bool autoMode = false;
const uint16_t autoWaypoints[] = {200, 800, 350, 650, 500};
const uint8_t NUM_WAYPOINTS = sizeof(autoWaypoints) / sizeof(autoWaypoints[0]);
uint8_t waypointIndex = 0;
unsigned long lastAutoMove = 0;
const uint16_t AUTO_MOVE_TIME_MS = 900;
const uint16_t AUTO_PAUSE_MS = 300;

void autoTick() {
  if (!autoMode) return;
  unsigned long now = millis();
  if (now - lastAutoMove >= (unsigned long)(AUTO_MOVE_TIME_MS + AUTO_PAUSE_MS)) {
    uint16_t target = autoWaypoints[waypointIndex];
    uint8_t ids[2] = { SERVO_ID_1, SERVO_ID_2 };
    uint16_t positions[2] = { target, target };
    servos.moveMany(ids, positions, 2, AUTO_MOVE_TIME_MS, pairGapMs);

    Serial.print(F("[auto] -> waypoint "));
    Serial.println(target);

    waypointIndex = (waypointIndex + 1) % NUM_WAYPOINTS;
    lastAutoMove = now;
  }
}

// --- command console ---

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  m <id> <pos> [time_ms]           move one servo"));
  Serial.println(F("  b <pos1> <pos2> [time_ms] [gap]  move both back-to-back (ID1 then ID2)"));
  Serial.println(F("  g <gap_ms>                        set default gap used by 'b'/'a'"));
  Serial.println(F("  l <id> <0|1>                      unload(0)/load(1) torque"));
  Serial.println(F("  a <0|1>                            background auto-sweep on/off"));
  Serial.println(F("  r                                  reset both to baseline (500)"));
  Serial.println(F("  ?                                  show this help"));
}

void handleLine(char* line) {
  char* tok = strtok(line, " ");
  if (!tok) return;

  if (strcmp(tok, "m") == 0) {
    char* idTok = strtok(NULL, " ");
    char* posTok = strtok(NULL, " ");
    char* timeTok = strtok(NULL, " ");
    if (!idTok || !posTok) { Serial.println(F("usage: m <id> <pos> [time_ms]")); return; }
    uint8_t id = atoi(idTok);
    uint16_t pos = atoi(posTok);
    uint16_t t = timeTok ? atoi(timeTok) : DEFAULT_MOVE_TIME_MS;
    servos.move(id, pos, t);
    Serial.print(F("Sent: move ID "));
    Serial.print(id);
    Serial.print(F(" -> "));
    Serial.print(pos);
    Serial.print(F(" over "));
    Serial.print(t);
    Serial.println(F("ms"));
  }
  else if (strcmp(tok, "b") == 0) {
    char* p1Tok = strtok(NULL, " ");
    char* p2Tok = strtok(NULL, " ");
    char* timeTok = strtok(NULL, " ");
    char* gapTok = strtok(NULL, " ");
    if (!p1Tok || !p2Tok) { Serial.println(F("usage: b <pos1> <pos2> [time_ms] [gap_ms]")); return; }
    uint16_t p1 = atoi(p1Tok);
    uint16_t p2 = atoi(p2Tok);
    uint16_t t = timeTok ? atoi(timeTok) : DEFAULT_MOVE_TIME_MS;
    uint16_t gap = gapTok ? atoi(gapTok) : pairGapMs;
    uint8_t ids[2] = { SERVO_ID_1, SERVO_ID_2 };
    uint16_t positions[2] = { p1, p2 };
    servos.moveMany(ids, positions, 2, t, gap);
    Serial.print(F("Sent paired: ID1->"));
    Serial.print(p1);
    Serial.print(F(", ID2->"));
    Serial.print(p2);
    Serial.print(F(", gap="));
    Serial.println(gap);
  }
  else if (strcmp(tok, "g") == 0) {
    char* gapTok = strtok(NULL, " ");
    if (!gapTok) { Serial.println(F("usage: g <gap_ms>")); return; }
    pairGapMs = atoi(gapTok);
    Serial.print(F("Default pair gap set to "));
    Serial.println(pairGapMs);
  }
  else if (strcmp(tok, "l") == 0) {
    char* idTok = strtok(NULL, " ");
    char* onTok = strtok(NULL, " ");
    if (!idTok || !onTok) { Serial.println(F("usage: l <id> <0|1>")); return; }
    uint8_t id = atoi(idTok);
    uint8_t on = atoi(onTok);
    servos.setLoad(id, on != 0);
    Serial.print(F("Sent: "));
    Serial.print(on ? F("LOAD") : F("UNLOAD"));
    Serial.print(F(" for ID "));
    Serial.println(id);
  }
  else if (strcmp(tok, "a") == 0) {
    char* onTok = strtok(NULL, " ");
    if (!onTok) { Serial.println(F("usage: a <0|1>")); return; }
    autoMode = atoi(onTok) != 0;
    lastAutoMove = millis() - (AUTO_MOVE_TIME_MS + AUTO_PAUSE_MS); // fire immediately
    Serial.print(F("Auto-sweep "));
    Serial.println(autoMode ? F("ON") : F("OFF"));
  }
  else if (strcmp(tok, "r") == 0) {
    resetBaseline();
  }
  else if (strcmp(tok, "?") == 0 || strcmp(tok, "help") == 0) {
    printHelp();
  }
  else {
    Serial.print(F("Unknown command: "));
    Serial.println(tok);
    printHelp();
  }
}

char lineBuf[64];
uint8_t lineLen = 0;

void pollSerialConsole() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

void setup() {
  Serial.begin(9600);
  busPort.begin(115200);
  delay(500);
  Serial.println(F("=== LX-16A manual control console ==="));
  printHelp();
}

void loop() {
  pollSerialConsole(); // manual commands always live
  autoTick();           // background sweep, only moves when 'a 1' is active
}
