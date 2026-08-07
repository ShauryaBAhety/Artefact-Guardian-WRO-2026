/*
  Artefact Guardian - ESP32 direct-servo firmware
  Protocol from the vision controller:
    PING
    STOP
    ROTATE_TO:<0-180>
    SHOULDER_TO:<0-180>
    ELBOW_TO:<0-180>
    TILT_TO:<0-180>
    WRIST_TO:<0-180>
    GRIP:<0-180>
  STOP and a missing heartbeat both freeze the arm at its current position.
  Set the mechanical limits below before fitting an artefact in the gripper.
*/
#include <ESP32Servo.h>
#include <ctype.h>
#include <math.h>
constexpr uint32_t BAUD_RATE = 115200;
constexpr uint32_t CONTROL_PERIOD_MS = 15;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 2000;
constexpr float GLIDE_FACTOR = 0.05;
constexpr size_t JOINT_COUNT = 6;
constexpr size_t RX_BUFFER_SIZE = 80;
enum Joint : uint8_t {
  BASE,
  SHOULDER,
  ELBOW,
  WRIST_TILT,
  WRIST_ROTATE,
  GRIPPER,
};
// GPIO12 is an ESP32 boot-strapping pin. If your board sometimes fails to
// boot, move the shoulder signal wire to a non-strapping GPIO and change it.
const int SERVO_PINS[JOINT_COUNT] = {13, 12, 25, 26, 27, 32};
// Tune these for YOUR physical arm. Do not assume 0-180 is mechanically safe.
const int MIN_ANGLE[JOINT_COUNT] = {15, 20, 55, 20, 20, 30};
const int MAX_ANGLE[JOINT_COUNT] = {170, 105, 145, 160, 160, 150};
const int SAFE_HOME[JOINT_COUNT] = {90, 92, 91, 90, 90, 30};
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoWristTilt;
Servo servoWristRotate;
Servo servoGripper;
Servo* servos[JOINT_COUNT] = {
  &servoBase, &servoShoulder, &servoElbow,
  &servoWristTilt, &servoWristRotate, &servoGripper,
};
float targetAngle[JOINT_COUNT];
float currentAngle[JOINT_COUNT];
char rxBuffer[RX_BUFFER_SIZE];
size_t rxLength = 0;
bool rxOverflow = false;
bool stopped = false;
bool watchdogLatched = false;
uint32_t lastMessageMs = 0;
int clampJointAngle(Joint joint, float angle) {
  if (!isfinite(angle)) {
    return -1;
  }
  int rounded = static_cast<int>(lroundf(angle));
  return constrain(rounded, MIN_ANGLE[joint], MAX_ANGLE[joint]);
}
void freezeArm() {
  for (size_t joint = 0; joint < JOINT_COUNT; ++joint) {
    targetAngle[joint] = currentAngle[joint];
  }
  stopped = true;
}
bool parseNumber(char* text, float& result) {
  char* end = nullptr;
  result = strtof(text, &end);
  if (end == text || !isfinite(result)) {
    return false;
  }
  while (*end != '\0' && isspace(static_cast<unsigned char>(*end))) {
    ++end;
  }
  return *end == '\0';
}
int jointForCommand(const char* command) {
  if (strcmp(command, "ROTATE_TO") == 0) return BASE;
  if (strcmp(command, "SHOULDER_TO") == 0) return SHOULDER;
  if (strcmp(command, "ELBOW_TO") == 0) return ELBOW;
  if (strcmp(command, "TILT_TO") == 0) return WRIST_TILT;
  if (strcmp(command, "WRIST_TO") == 0) return WRIST_ROTATE;
  if (strcmp(command, "GRIP") == 0) return GRIPPER;
  return -1;
}
void processCommand(char* line) {
  if (strcmp(line, "PING") == 0) {
    lastMessageMs = millis();
    return;
  }
  if (strcmp(line, "STOP") == 0) {
    lastMessageMs = millis();
    freezeArm();
    Serial.println("OK:STOP");
    return;
  }
  char* separator = strchr(line, ':');
  if (separator == nullptr) {
    Serial.println("ERR:FORMAT");
    return;
  }
  *separator = '\0';
  char* valueText = separator + 1;
  const int jointIndex = jointForCommand(line);
  if (jointIndex < 0) {
    Serial.println("ERR:COMMAND");
    return;
  }
  float requestedAngle = 0.0f;
  if (!parseNumber(valueText, requestedAngle)) {
    Serial.println("ERR:ANGLE");
    return;
  }
  const int safeAngle = clampJointAngle(static_cast<Joint>(jointIndex), requestedAngle);
  if (safeAngle < 0) {
    Serial.println("ERR:ANGLE");
    return;
  }
  targetAngle[jointIndex] = safeAngle;
  stopped = false;          // A fresh angle command intentionally resumes control.
  watchdogLatched = false;
  lastMessageMs = millis();
  Serial.printf("OK:%s:%d\n", line, safeAngle);
}
void readSerialNonBlocking() {
  while (Serial.available() > 0) {
    const char received = static_cast<char>(Serial.read());
    if (received == '\r') {
      continue;
    }
    if (received == '\n') {
      if (!rxOverflow && rxLength > 0) {
        rxBuffer[rxLength] = '\0';
        processCommand(rxBuffer);
      } else if (rxOverflow) {
        Serial.println("ERR:LINE_TOO_LONG");
      }
      rxLength = 0;
      rxOverflow = false;
      continue;
    }
    if (rxLength < RX_BUFFER_SIZE - 1 && !rxOverflow) {
      rxBuffer[rxLength++] = received;
    } else {
      rxOverflow = true;
    }
  }
}
void updateServos() {
  if (!stopped) {
    for (size_t joint = 0; joint < JOINT_COUNT; ++joint) {
      currentAngle[joint] += (targetAngle[joint] - currentAngle[joint]) * GLIDE_FACTOR;
    }
  }
  for (size_t joint = 0; joint < JOINT_COUNT; ++joint) {
    servos[joint]->write(static_cast<int>(lroundf(currentAngle[joint])));
  }
}
void setup() {
  Serial.begin(BAUD_RATE);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (size_t joint = 0; joint < JOINT_COUNT; ++joint) {
    targetAngle[joint] = SAFE_HOME[joint];
    currentAngle[joint] = SAFE_HOME[joint];
    servos[joint]->setPeriodHertz(50);
    servos[joint]->attach(SERVO_PINS[joint], 500, 2500);
    servos[joint]->write(SAFE_HOME[joint]);
  }
  lastMessageMs = millis();
  Serial.println("Artefact Guardian direct-servo firmware ready");
}
void loop() {
  readSerialNonBlocking();
  if (!watchdogLatched && millis() - lastMessageMs > HEARTBEAT_TIMEOUT_MS) {
    freezeArm();
    watchdogLatched = true;
    Serial.println("SAFE:HEARTBEAT_TIMEOUT");
  }
  updateServos();
  delay(CONTROL_PERIOD_MS);
}
