/*
 * AI Physical Desk Assistant - ESP32 Firmware
 * Phase 1: Full servo control with motion smoothing and idle animations
 */

#include <ArduinoJson.h>
#include <ESP32Servo.h>

// Communication settings
#define BAUD_RATE 115200
#define JSON_BUFFER_SIZE 512
#define UPDATE_RATE_HZ 50  // 50 Hz update rate
#define UPDATE_INTERVAL_MS (1000 / UPDATE_RATE_HZ)

// Servo pins (adjust as needed for your setup)
#define SERVO_BASE_PIN 2
#define SERVO_SHOULDER_PIN 4
#define SERVO_ELBOW_PIN 5
#define SERVO_WRIST_PITCH_PIN 18
#define SERVO_WRIST_ROLL_PIN 19
#define SERVO_GRIPPER_PIN 21

// Joint limits (degrees)
#define BASE_MIN 0
#define BASE_MAX 180
#define SHOULDER_MIN 15
#define SHOULDER_MAX 165
#define ELBOW_MIN 0
#define ELBOW_MAX 180
#define WRIST_PITCH_MIN 30
#define WRIST_PITCH_MAX 150
#define WRIST_ROLL_MIN 0
#define WRIST_ROLL_MAX 180
#define GRIPPER_MIN 10
#define GRIPPER_MAX 90

// Servo objects
Servo servoBase, servoShoulder, servoElbow, servoWristPitch, servoWristRoll, servoGripper;
Servo servos[6] = {servoBase, servoShoulder, servoElbow, servoWristPitch, servoWristRoll, servoGripper};
int servoPins[6] = {SERVO_BASE_PIN, SERVO_SHOULDER_PIN, SERVO_ELBOW_PIN, SERVO_WRIST_PITCH_PIN, SERVO_WRIST_ROLL_PIN, SERVO_GRIPPER_PIN};

// Motion control
float currentAngles[6] = {90.0, 45.0, 120.0, 90.0, 0.0, 30.0};  // Current positions
float targetAngles[6] = {90.0, 45.0, 120.0, 90.0, 0.0, 30.0};   // Target positions
float maxSpeeds[6] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0};      // Max degrees per second per joint
float currentSpeed = 0.5;  // Global speed scaling (0.0-1.0)

// System state
enum State { STATE_IDLE, STATE_MANUAL, STATE_ESTOP };
State currentState = STATE_MANUAL;
unsigned long lastUpdateTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastCommandTime = 0;
#define TELEMETRY_INTERVAL_MS 100  // Send telemetry every 100ms
#define COMMAND_TIMEOUT_MS 5000    // Disable servos if no command for 5 seconds

// Idle animation variables
enum IdleAnimation { IDLE_NONE, IDLE_BREATHING, IDLE_CURIOUS_TILT, IDLE_MICRO_ADJUST, IDLE_RESET };
IdleAnimation currentIdle = IDLE_NONE;
unsigned long idleStartTime = 0;
bool idleEnabled = true;

// Idle animation poses
struct Pose {
  float angles[6];
};

Pose neutralPose = {{90.0, 45.0, 120.0, 90.0, 0.0, 30.0}};
Pose breathingPoseA = {{90.0, 40.0, 125.0, 90.0, 0.0, 30.0}};
Pose breathingPoseB = {{90.0, 50.0, 115.0, 90.0, 0.0, 30.0}};
Pose curiousLeft = {{75.0, 45.0, 120.0, 75.0, 0.0, 30.0}};
Pose curiousRight = {{105.0, 45.0, 120.0, 105.0, 0.0, 30.0}};

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  // Initialize servos
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(currentAngles[i]);
  }

  // Set initial target to current
  memcpy(targetAngles, currentAngles, sizeof(currentAngles));
}

void loop() {
  unsigned long currentTime = millis();

  // Check for incoming commands
  if (Serial.available()) {
    String jsonString = Serial.readStringUntil('\n');
    jsonString.trim();
    if (jsonString.length() > 0) {
      processCommand(jsonString);
      lastCommandTime = currentTime;
    }
  }

  // Update motion at fixed rate
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
    updateMotion();
    lastUpdateTime = currentTime;
  }

  // Send telemetry periodically
  if (currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    sendTelemetry();
    lastTelemetryTime = currentTime;
  }

  // Safety timeout - disable servos if no command received
  if (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS) {
    emergencyStop();
  }

  // Handle idle animations
  if (currentState == STATE_IDLE && idleEnabled) {
    updateIdleAnimation(currentTime);
  }

  delay(1);  // Small delay to prevent overwhelming
}

void processCommand(String jsonString) {
  StaticJsonDocument<JSON_BUFFER_SIZE> doc;
  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    sendErrorResponse("Invalid JSON: " + String(error.c_str()));
    return;
  }

  if (!doc.containsKey("cmd")) {
    sendErrorResponse("Missing 'cmd' field");
    return;
  }

  String command = doc["cmd"];

  if (command == "set_joints") {
    handleSetJoints(doc);
  }
  else if (command == "play_idle") {
    handlePlayIdle(doc);
  }
  else if (command == "estop") {
    emergencyStop();
  }
  else if (command == "ping") {
    handlePing();
  }
  else if (command == "get_state") {
    handleGetState();
  }
  else {
    sendErrorResponse("Unknown command: " + command);
  }
}

void handleSetJoints(JsonDocument& doc) {
  if (!doc.containsKey("targets")) {
    sendErrorResponse("Missing 'targets' field");
    return;
  }

  JsonArray targets = doc["targets"];
  if (targets.size() != 6) {
    sendErrorResponse("targets must be array of 6 floats");
    return;
  }

  // Extract speed if provided
  if (doc.containsKey("speed")) {
    float speed = doc["speed"];
    if (speed >= 0.0 && speed <= 1.0) {
      currentSpeed = speed;
    }
  }

  // Validate and clamp joint angles
  bool valid = true;
  for (int i = 0; i < 6; i++) {
    float angle = targets[i];
    float clamped = clampAngle(i, angle);
    if (abs(angle - clamped) > 0.1) {
      valid = false;
    }
    targetAngles[i] = clamped;
  }

  currentState = STATE_MANUAL;

  // Send acknowledgment
  StaticJsonDocument<JSON_BUFFER_SIZE> response;
  response["cmd"] = "set_joints";
  response["status"] = "ok";
  response["message"] = "joints_set";
  if (!valid) {
    response["warning"] = "Some joint angles were clamped to limits";
  }
  serializeJson(response, Serial);
  Serial.println();
}

void handlePlayIdle(JsonDocument& doc) {
  String name = doc["name"] | "breathing";
  if (name == "breathing") {
    currentIdle = IDLE_BREATHING;
  } else if (name == "curious_tilt") {
    currentIdle = IDLE_CURIOUS_TILT;
  } else if (name == "micro_adjust") {
    currentIdle = IDLE_MICRO_ADJUST;
  } else if (name == "idle_reset") {
    currentIdle = IDLE_RESET;
  } else {
    sendErrorResponse("Unknown idle animation: " + name);
    return;
  }

  idleStartTime = millis();
  currentState = STATE_IDLE;

  StaticJsonDocument<JSON_BUFFER_SIZE> response;
  response["cmd"] = "play_idle";
  response["status"] = "ok";
  response["idle_animation"] = name;
  serializeJson(response, Serial);
  Serial.println();
}

void handlePing() {
  StaticJsonDocument<JSON_BUFFER_SIZE> response;
  response["cmd"] = "ping";
  response["status"] = "ok";
  response["message"] = "pong";
  response["state"] = getStateString();
  JsonArray joints = response.createNestedArray("joints");
  for (int i = 0; i < 6; i++) {
    joints.add(currentAngles[i]);
  }
  serializeJson(response, Serial);
  Serial.println();
}

void handleGetState() {
  StaticJsonDocument<JSON_BUFFER_SIZE> response;
  response["cmd"] = "get_state";
  response["status"] = "ok";
  response["state"] = getStateString();
  JsonArray joints = response.createNestedArray("joints");
  for (int i = 0; i < 6; i++) {
    joints.add(currentAngles[i]);
  }
  serializeJson(response, Serial);
  Serial.println();
}

void sendTelemetry() {
  StaticJsonDocument<JSON_BUFFER_SIZE> telemetry;
  telemetry["type"] = "telemetry";
  telemetry["state"] = getStateString();
  JsonArray joints = telemetry.createNestedArray("joints");
  for (int i = 0; i < 6; i++) {
    joints.add(currentAngles[i]);
  }
  serializeJson(telemetry, Serial);
  Serial.println();
}

void sendErrorResponse(String errorMessage) {
  StaticJsonDocument<JSON_BUFFER_SIZE> response;
  response["cmd"] = "error";
  response["status"] = "error";
  response["error"] = errorMessage;
  serializeJson(response, Serial);
  Serial.println();
}

void updateMotion() {
  if (currentState == STATE_ESTOP) {
    return;  // No motion in emergency stop
  }

  // Calculate max step size for this update interval
  float maxStep = (UPDATE_INTERVAL_MS / 1000.0) * 90.0 * currentSpeed;  // Base max 90 deg/sec scaled

  for (int i = 0; i < 6; i++) {
    float diff = targetAngles[i] - currentAngles[i];
    float step = constrain(diff, -maxStep, maxStep);
    currentAngles[i] += step;

    // Write to servo
    servos[i].write(currentAngles[i]);
  }
}

void updateIdleAnimation(unsigned long currentTime) {
  unsigned long elapsed = currentTime - idleStartTime;

  switch (currentIdle) {
    case IDLE_BREATHING:
      // Slow oscillation every 6 seconds
      if (elapsed % 6000 < 3000) {
        setTargetPose(breathingPoseA.angles);
      } else {
        setTargetPose(breathingPoseB.angles);
      }
      break;

    case IDLE_CURIOUS_TILT:
      // Tilt left/right every 4 seconds with pause
      if (elapsed % 8000 < 2000) {
        setTargetPose(curiousLeft.angles);
      } else if (elapsed % 8000 < 4000) {
        setTargetPose(neutralPose.angles);
      } else if (elapsed % 8000 < 6000) {
        setTargetPose(curiousRight.angles);
      } else {
        setTargetPose(neutralPose.angles);
      }
      break;

    case IDLE_MICRO_ADJUST:
      // Small random adjustments every 10-20 seconds
      if (elapsed % 20000 < 10000) {
        // Add small random offset to base and wrist
        targetAngles[0] = neutralPose.angles[0] + random(-2, 3);
        targetAngles[3] = neutralPose.angles[3] + random(-3, 4);
        targetAngles[4] = neutralPose.angles[4] + random(-1, 2);
      } else {
        setTargetPose(neutralPose.angles);
      }
      break;

    case IDLE_RESET:
      // Return to neutral
      setTargetPose(neutralPose.angles);
      currentIdle = IDLE_NONE;
      break;

    default:
      // No animation
      break;
  }
}

void setTargetPose(float angles[6]) {
  memcpy(targetAngles, angles, sizeof(targetAngles));
}

float clampAngle(int jointIndex, float angle) {
  switch (jointIndex) {
    case 0: return constrain(angle, BASE_MIN, BASE_MAX);
    case 1: return constrain(angle, SHOULDER_MIN, SHOULDER_MAX);
    case 2: return constrain(angle, ELBOW_MIN, ELBOW_MAX);
    case 3: return constrain(angle, WRIST_PITCH_MIN, WRIST_PITCH_MAX);
    case 4: return constrain(angle, WRIST_ROLL_MIN, WRIST_ROLL_MAX);
    case 5: return constrain(angle, GRIPPER_MIN, GRIPPER_MAX);
    default: return angle;
  }
}

void emergencyStop() {
  currentState = STATE_ESTOP;
  // Detach servos to prevent holding position
  for (int i = 0; i < 6; i++) {
    servos[i].detach();
  }
}

String getStateString() {
  switch (currentState) {
    case STATE_IDLE: return "idle";
    case STATE_MANUAL: return "manual";
    case STATE_ESTOP: return "estop";
    default: return "unknown";
  }
}
