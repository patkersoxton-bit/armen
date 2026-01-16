/*
 * AI Physical Desk Assistant - ESP32 Firmware
 * Phase 1: Full servo control with motion smoothing and idle animations
 * 
 * Updates:
 * - CBOR communication protocol with length-prefix framing
 * - PCA9685 16-channel PWM servo controller via I2C
 * - Fixed buffer size calculation and frame synchronization
 * - Test mode for single servo control (CH0)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <tinycbor.h>

// Communication settings
#define BAUD_RATE 115200
#define UPDATE_RATE_HZ 50  // 50 Hz update rate
#define UPDATE_INTERVAL_MS (1000 / UPDATE_RATE_HZ)

// PCA9685 settings
#define PCA9685_ADDRESS 0x40
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz
#define SERVOMIN 150   // Minimum pulse length count (out of 4096)
#define SERVOMAX 600   // Maximum pulse length count (out of 4096)

// Servo channel mapping on PCA9685
#define SERVO_BASE 0
#define SERVO_SHOULDER 1
#define SERVO_ELBOW 2
#define SERVO_WRIST_PITCH 3
#define SERVO_WRIST_ROLL 4
#define SERVO_GRIPPER 5

// Joint limits (degrees)
struct JointLimits {
  float min;
  float max;
};

JointLimits jointLimits[6] = {
  {0, 180},    // Base
  {15, 165},   // Shoulder
  {0, 180},    // Elbow
  {30, 150},   // Wrist Pitch
  {0, 180},    // Wrist Roll
  {10, 90}     // Gripper
};

// PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// Motion control
float currentAngles[6] = {90.0, 45.0, 120.0, 90.0, 0.0, 30.0};  // Current positions
float targetAngles[6] = {90.0, 45.0, 120.0, 90.0, 0.0, 30.0};   // Target positions
float maxSpeeds[6] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0};      // Max degrees per second per joint
float currentSpeed = 0.5;  // Global speed scaling (0.0-1.0)

// System state
enum State { STATE_IDLE, STATE_MANUAL, STATE_ESTOP, STATE_TEST };
State currentState = STATE_MANUAL;
unsigned long lastUpdateTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastCommandTime = 0;
#define TELEMETRY_INTERVAL_MS 500  // Send telemetry every 500ms (reduced frequency)
#define COMMAND_TIMEOUT_MS 10000   // Disable servos if no command for 10 seconds

// Test mode variables
bool testMode = false;

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

// CBOR communication buffer
#define MAX_CBOR_SIZE 512
uint8_t cborBuffer[MAX_CBOR_SIZE];

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  // Initialize I2C
  Wire.begin(21, 22);  // SDA=21, SCL=22 (ESP32 default)
  
  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);  // Allow oscillator to stabilize
  
  // Set servos to initial positions
  for (int i = 0; i < 6; i++) {
    setServoAnglePCA(i, currentAngles[i]);
  }
  
  // Set initial target to current
  memcpy(targetAngles, currentAngles, sizeof(currentAngles));
  
  Serial.println("ESP32 Arm Controller initialized with CBOR and PCA9685");
  Serial.flush();
  delay(100);
  
  lastCommandTime = millis();  // Initialize to prevent immediate timeout
}

void loop() {
  unsigned long currentTime = millis();

  // Check for incoming commands
  if (Serial.available() >= 2) {
    // Read length prefix (2 bytes, big-endian)
    uint8_t lengthBytes[2];
    Serial.readBytes(lengthBytes, 2);
    uint16_t messageLength = (lengthBytes[0] << 8) | lengthBytes[1];
    
    // Validate message length
    if (messageLength > 0 && messageLength <= MAX_CBOR_SIZE) {
      // Wait for full message with timeout
      unsigned long waitStart = millis();
      while (Serial.available() < messageLength && millis() - waitStart < 1000) {
        delay(1);
      }
      
      if (Serial.available() >= messageLength) {
        Serial.readBytes(cborBuffer, messageLength);
        processCommand(cborBuffer, messageLength);
        lastCommandTime = currentTime;
      } else {
        // Timeout waiting for message - clear buffer
        while (Serial.available()) Serial.read();
      }
    } else {
      // Invalid length - clear buffer to resync
      while (Serial.available()) Serial.read();
    }
  }

  // Update motion at fixed rate
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
    updateMotion();
    lastUpdateTime = currentTime;
  }

  // Send telemetry periodically (less frequent) - skip in test mode
  if (!testMode && currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    sendTelemetry();
    lastTelemetryTime = currentTime;
  }

  // Safety timeout - go to emergency stop if no command received
  if (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS) {
    if (currentState != STATE_ESTOP) {
      emergencyStop();
    }
  }

  // Handle idle animations
  if (currentState == STATE_IDLE && idleEnabled) {
    updateIdleAnimation(currentTime);
  }

  delay(1);  // Small delay to prevent overwhelming
}

void processCommand(uint8_t* data, uint16_t length) {
  // Initialize CBOR parser
  CborParser parser;
  CborValue map;
  CborError err;
  
  err = cbor_parser_init(data, length, 0, &parser, &map);
  if (err != CborNoError || !cbor_value_is_map(&map)) {
    sendErrorResponse("Invalid CBOR format");
    return;
  }
  
  // Enter the map
  CborValue element;
  err = cbor_value_enter_container(&map, &element);
  if (err != CborNoError) {
    sendErrorResponse("Failed to parse CBOR map");
    return;
  }
  
  String command = "";
  float targets[6];
  bool hasTargets = false;
  float speed = -1.0;
  String idleName = "";
  
  // Parse key-value pairs
  while (!cbor_value_at_end(&element)) {
    // Get key
    if (!cbor_value_is_text_string(&element)) {
      cbor_value_advance(&element);
      continue;
    }
    
    size_t keyLen;
    cbor_value_get_string_length(&element, &keyLen);
    char keyBuffer[64];
    if (keyLen >= sizeof(keyBuffer)) keyLen = sizeof(keyBuffer) - 1;
    cbor_value_copy_text_string(&element, keyBuffer, &keyLen, &element);
    keyBuffer[keyLen] = '\0';
    String key = String(keyBuffer);
    
    // Get value based on key
    if (key == "cmd" && cbor_value_is_text_string(&element)) {
      size_t valueLen;
      cbor_value_get_string_length(&element, &valueLen);
      char valueBuffer[64];
      if (valueLen >= sizeof(valueBuffer)) valueLen = sizeof(valueBuffer) - 1;
      cbor_value_copy_text_string(&element, valueBuffer, &valueLen, &element);
      valueBuffer[valueLen] = '\0';
      command = String(valueBuffer);
    } 
    else if (key == "targets" && cbor_value_is_array(&element)) {
      CborValue array;
      cbor_value_enter_container(&element, &array);
      int idx = 0;
      while (!cbor_value_at_end(&array) && idx < 6) {
        if (cbor_value_is_float(&array)) {
          float val;
          cbor_value_get_float(&array, &val);
          targets[idx++] = val;
        } else if (cbor_value_is_double(&array)) {
          double val;
          cbor_value_get_double(&array, &val);
          targets[idx++] = (float)val;
        }
        cbor_value_advance(&array);
      }
      if (idx == 6) hasTargets = true;
      cbor_value_leave_container(&element, &array);
    }
    else if (key == "speed") {
      if (cbor_value_is_float(&element)) {
        cbor_value_get_float(&element, &speed);
        cbor_value_advance(&element);
      } else if (cbor_value_is_double(&element)) {
        double val;
        cbor_value_get_double(&element, &val);
        speed = (float)val;
        cbor_value_advance(&element);
      } else {
        cbor_value_advance(&element);
      }
    }
    else if (key == "name" && cbor_value_is_text_string(&element)) {
      size_t valueLen;
      cbor_value_get_string_length(&element, &valueLen);
      char valueBuffer[64];
      if (valueLen >= sizeof(valueBuffer)) valueLen = sizeof(valueBuffer) - 1;
      cbor_value_copy_text_string(&element, valueBuffer, &valueLen, &element);
      valueBuffer[valueLen] = '\0';
      idleName = String(valueBuffer);
    }
    else {
      // Skip unknown value
      cbor_value_advance(&element);
    }
  }
  
  cbor_value_leave_container(&map, &element);
  
  // Execute command
  if (command == "set_joints") {
    if (hasTargets) {
      for (int i = 0; i < 6; i++) {
        targetAngles[i] = clampAngle(i, targets[i]);
      }
      if (speed >= 0.0 && speed <= 1.0) {
        currentSpeed = speed;
      }
      currentState = STATE_MANUAL;
      sendResponse("set_joints", "ok", "joints_set");
    } else {
      sendErrorResponse("Missing targets array");
    }
  } else if (command == "set_test_servo") {
    // Test mode: set single servo (channel 0) directly
    if (hasTargets && targets[0] >= 0) {
      float angle = clampAngle(0, targets[0]);
      currentAngles[0] = angle;
      targetAngles[0] = angle;
      setServoAnglePCA(0, angle);
      sendResponse("set_test_servo", "ok", "test_servo_set");
    } else {
      sendErrorResponse("Missing or invalid target angle");
    }
  } else if (command == "test_mode") {
    // Enter test mode - disable telemetry
    testMode = true;
    currentState = STATE_TEST;
    sendResponse("test_mode", "ok", "test_mode_enabled");
  } else if (command == "exit_test_mode") {
    // Exit test mode - re-enable telemetry
    testMode = false;
    currentState = STATE_MANUAL;
    sendResponse("exit_test_mode", "ok", "test_mode_disabled");
  } else if (command == "play_idle") {
    if (idleName != "") {
      handlePlayIdle(idleName);
    } else {
      sendErrorResponse("Missing idle name");
    }
  } else if (command == "estop") {
    emergencyStop();
    sendResponse("estop", "ok", "emergency_stop_activated");
  } else if (command == "ping") {
    sendPingResponse();
  } else if (command == "get_state") {
    sendStateResponse();
  } else if (command != "") {
    sendErrorResponse("Unknown command: " + command);
  }
}

void handlePlayIdle(String name) {
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
  sendResponse("play_idle", "ok", "animation_started");
}

void sendResponse(const char* cmd, const char* status, const char* message) {
  uint8_t buffer[256];
  CborEncoder encoder, mapEncoder;
  
  cbor_encoder_init(&encoder, buffer, sizeof(buffer), 0);
  cbor_encoder_create_map(&encoder, &mapEncoder, 3);
  
  cbor_encode_text_stringz(&mapEncoder, "cmd");
  cbor_encode_text_stringz(&mapEncoder, cmd);
  
  cbor_encode_text_stringz(&mapEncoder, "status");
  cbor_encode_text_stringz(&mapEncoder, status);
  
  cbor_encode_text_stringz(&mapEncoder, "message");
  cbor_encode_text_stringz(&mapEncoder, message);
  
  cbor_encoder_close_container(&encoder, &mapEncoder);
  
  // CRITICAL FIX: Get actual encoded length using TinyCBOR API
  size_t actualLength = sizeof(buffer) - cbor_encoder_get_buffer_size(&encoder, buffer);
  sendCBORMessage(buffer, actualLength);
}

void sendPingResponse() {
  uint8_t buffer[256];
  CborEncoder encoder, mapEncoder, arrayEncoder;
  
  cbor_encoder_init(&encoder, buffer, sizeof(buffer), 0);
  cbor_encoder_create_map(&encoder, &mapEncoder, 4);
  
  cbor_encode_text_stringz(&mapEncoder, "cmd");
  cbor_encode_text_stringz(&mapEncoder, "ping");
  
  cbor_encode_text_stringz(&mapEncoder, "status");
  cbor_encode_text_stringz(&mapEncoder, "ok");
  
  cbor_encode_text_stringz(&mapEncoder, "state");
  cbor_encode_text_stringz(&mapEncoder, getStateString());
  
  cbor_encode_text_stringz(&mapEncoder, "joints");
  cbor_encoder_create_array(&mapEncoder, &arrayEncoder, 6);
  for (int i = 0; i < 6; i++) {
    cbor_encode_float(&arrayEncoder, currentAngles[i]);
  }
  cbor_encoder_close_container(&mapEncoder, &arrayEncoder);
  
  cbor_encoder_close_container(&encoder, &mapEncoder);
  
  size_t actualLength = sizeof(buffer) - cbor_encoder_get_buffer_size(&encoder, buffer);
  sendCBORMessage(buffer, actualLength);
}

void sendStateResponse() {
  uint8_t buffer[256];
  CborEncoder encoder, mapEncoder, arrayEncoder;
  
  cbor_encoder_init(&encoder, buffer, sizeof(buffer), 0);
  cbor_encoder_create_map(&encoder, &mapEncoder, 4);
  
  cbor_encode_text_stringz(&mapEncoder, "cmd");
  cbor_encode_text_stringz(&mapEncoder, "get_state");
  
  cbor_encode_text_stringz(&mapEncoder, "status");
  cbor_encode_text_stringz(&mapEncoder, "ok");
  
  cbor_encode_text_stringz(&mapEncoder, "state");
  cbor_encode_text_stringz(&mapEncoder, getStateString());
  
  cbor_encode_text_stringz(&mapEncoder, "joints");
  cbor_encoder_create_array(&mapEncoder, &arrayEncoder, 6);
  for (int i = 0; i < 6; i++) {
    cbor_encode_float(&arrayEncoder, currentAngles[i]);
  }
  cbor_encoder_close_container(&mapEncoder, &arrayEncoder);
  
  cbor_encoder_close_container(&encoder, &mapEncoder);
  
  size_t actualLength = sizeof(buffer) - cbor_encoder_get_buffer_size(&encoder, buffer);
  sendCBORMessage(buffer, actualLength);
}

void sendTelemetry() {
  uint8_t buffer[256];
  CborEncoder encoder, mapEncoder, arrayEncoder;
  
  cbor_encoder_init(&encoder, buffer, sizeof(buffer), 0);
  cbor_encoder_create_map(&encoder, &mapEncoder, 3);
  
  cbor_encode_text_stringz(&mapEncoder, "type");
  cbor_encode_text_stringz(&mapEncoder, "telemetry");
  
  cbor_encode_text_stringz(&mapEncoder, "state");
  cbor_encode_text_stringz(&mapEncoder, getStateString());
  
  cbor_encode_text_stringz(&mapEncoder, "joints");
  cbor_encoder_create_array(&mapEncoder, &arrayEncoder, 6);
  for (int i = 0; i < 6; i++) {
    cbor_encode_float(&arrayEncoder, currentAngles[i]);
  }
  cbor_encoder_close_container(&mapEncoder, &arrayEncoder);
  
  cbor_encoder_close_container(&encoder, &mapEncoder);
  
  size_t actualLength = sizeof(buffer) - cbor_encoder_get_buffer_size(&encoder, buffer);
  sendCBORMessage(buffer, actualLength);
}

void sendErrorResponse(String errorMessage) {
  uint8_t buffer[256];
  CborEncoder encoder, mapEncoder;
  
  cbor_encoder_init(&encoder, buffer, sizeof(buffer), 0);
  cbor_encoder_create_map(&encoder, &mapEncoder, 3);
  
  cbor_encode_text_stringz(&mapEncoder, "cmd");
  cbor_encode_text_stringz(&mapEncoder, "error");
  
  cbor_encode_text_stringz(&mapEncoder, "status");
  cbor_encode_text_stringz(&mapEncoder, "error");
  
  cbor_encode_text_stringz(&mapEncoder, "error");
  cbor_encode_text_stringz(&mapEncoder, errorMessage.c_str());
  
  cbor_encoder_close_container(&encoder, &mapEncoder);
  
  size_t actualLength = sizeof(buffer) - cbor_encoder_get_buffer_size(&encoder, buffer);
  sendCBORMessage(buffer, actualLength);
}

void sendCBORMessage(uint8_t* buffer, uint16_t length) {
  // Send length prefix (2 bytes, big-endian)
  uint8_t lengthPrefix[2];
  lengthPrefix[0] = (length >> 8) & 0xFF;
  lengthPrefix[1] = length & 0xFF;
  Serial.write(lengthPrefix, 2);
  
  // Send CBOR data
  Serial.write(buffer, length);
  Serial.flush();
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

    // Write to servo via PCA9685
    setServoAnglePCA(i, currentAngles[i]);
  }
}

void setServoAnglePCA(uint8_t channel, float angle) {
  // Constrain angle to valid range
  angle = clampAngle(channel, angle);
  
  // Map angle to pulse length (in PWM counts for PCA9685)
  // Standard servos: 0° = ~1ms (SERVOMIN), 180° = ~2ms (SERVOMAX)
  int pulseCount = map(angle * 10, 0, 1800, SERVOMIN, SERVOMAX);
  
  // Set PWM
  pwm.setPWM(channel, 0, pulseCount);
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
  if (jointIndex >= 0 && jointIndex < 6) {
    return constrain(angle, jointLimits[jointIndex].min, jointLimits[jointIndex].max);
  }
  return angle;
}

void emergencyStop() {
  currentState = STATE_ESTOP;
  // Turn off all servo outputs
  for (int i = 0; i < 6; i++) {
    pwm.setPWM(i, 0, 0);
  }
}

const char* getStateString() {
  switch (currentState) {
    case STATE_IDLE: return "idle";
    case STATE_MANUAL: return "manual";
    case STATE_ESTOP: return "estop";
    case STATE_TEST: return "test";
    default: return "unknown";
  }
}
