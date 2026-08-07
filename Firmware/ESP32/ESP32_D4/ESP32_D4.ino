/*
  Artefact Guardian - ESP32 direct-servo firmware (single-file build)
  Protocol:
    PING
    STOP
    ESTOP
    HOME
    ROTATE_TO:<0-180>
    SHOULDER_TO:<0-180>
    ELBOW_TO:<0-180>
    TILT_TO:<0-180>
    WRIST_TO:<0-180>
    GRIP:<0-180>
    MOVE:a,b,c,d,e,f
  Status: STATUS:UNHOMED|HOMING|READY|MOVING|IDLE|STOPPED|WATCHDOG|ERROR
  DONE is sent once, edge-triggered, when the arm settles at its target.
*/

#include <ESP32Servo.h>
#include <Preferences.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ============================================================
// CONFIG
// ============================================================
constexpr uint32_t BAUD_RATE      = 115200;
constexpr size_t   RX_BUFFER_SIZE = 96;

constexpr uint32_t CONTROL_PERIOD_MS    = 15;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 3000;

constexpr size_t JOINT_COUNT = 6;

// GPIO12 is an ESP32 boot-strapping pin. SHOULDER moved off it -> GPIO14.
const int SERVO_PINS[JOINT_COUNT] = {13, 14, 25, 26, 27, 32};

// Mechanical limits - tune for YOUR physical arm before fitting an artefact.
const int MIN_ANGLE[JOINT_COUNT] = {15, 20, 20, 20, 20, 30};
const int MAX_ANGLE[JOINT_COUNT] = {165, 160, 160, 160, 160, 150};
const int SAFE_HOME[JOINT_COUNT] = {90, 90, 90, 90, 90, 30};

// Per-joint glide factor: fraction of remaining error closed per cycle.
const float GLIDE_FACTOR[JOINT_COUNT] =
    {0.06f, 0.04f, 0.045f, 0.06f, 0.07f, 0.10f};

// Slower, conservative glide used only while homing from a possibly
// stale/estimated pose.
constexpr float HOMING_GLIDE_FACTOR = 0.02f;

// Below this error (deg) we snap to target instead of creeping forever -
// eliminates near-target servo buzzing/hunting.
constexpr float ANGLE_DEADBAND_DEG = 0.6f;

// Max angular change allowed per control cycle, in degrees.
constexpr float MAX_STEP_DEG_PER_CYCLE = 4.0f;

constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr int SERVO_PWM_HZ = 50;

constexpr char NVS_NAMESPACE[]  = "artefact_arm";
constexpr char NVS_KEY_PREFIX[] = "j"; // keys: j0..j5

// ============================================================
// TYPES
// ============================================================
enum Joint : uint8_t {
  BASE = 0,
  SHOULDER,
  ELBOW,
  WRIST_TILT,
  WRIST_ROTATE,
  GRIPPER,
};

enum class StopCause : uint8_t {
  NONE,
  USER,       // STOP command
  HEARTBEAT,  // no PING/command within HEARTBEAT_TIMEOUT_MS
  EMERGENCY,  // ESTOP command
};

enum class SystemState : uint8_t {
  UNHOMED,   // powered up, servos NOT attached, awaiting HOME command
  HOMING,    // gliding from last-known/estimated pose to SAFE_HOME
  READY,
  MOVING,
  IDLE,
  STOPPED,
  WATCHDOG,
};

// ============================================================
// PERSISTENCE (NVS) - last-known-settled angles survive reboot
// ============================================================
namespace persistence {

Preferences prefs;

void keyFor(size_t joint, char* out, size_t outSize) {
  snprintf(out, outSize, "%s%u", NVS_KEY_PREFIX, static_cast<unsigned>(joint));
}

// Returns false if nothing has ever been saved (first-ever boot / erased
// flash) - caller must not treat outAngles as meaningful in that case.
bool loadAngles(float outAngles[JOINT_COUNT]) {
  prefs.begin(NVS_NAMESPACE, /*readOnly=*/true);
  bool haveAll = true;
  char key[8];
  for (size_t j = 0; j < JOINT_COUNT; ++j) {
    keyFor(j, key, sizeof(key));
    if (!prefs.isKey(key)) {
      haveAll = false;
      break;
    }
    outAngles[j] = prefs.getFloat(key, NAN);
  }
  prefs.end();
  return haveAll;
}

// Call sparingly (on DONE / clean STOP) - not every control cycle - to
// limit flash wear.
void saveAngles(const float angles[JOINT_COUNT]) {
  prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
  char key[8];
  for (size_t j = 0; j < JOINT_COUNT; ++j) {
    keyFor(j, key, sizeof(key));
    prefs.putFloat(key, angles[j]);
  }
  prefs.end();
}

} // namespace persistence

// ============================================================
// SERVO CONTROL (hardware layer)
// ============================================================
namespace servo_control {

Servo servoBase, servoShoulder, servoElbow, servoWristTilt, servoWristRotate, servoGripper;
Servo* servos[JOINT_COUNT] = {
  &servoBase, &servoShoulder, &servoElbow,
  &servoWristTilt, &servoWristRotate, &servoGripper,
};
bool attached = false;

// Clamps a requested angle to this joint's mechanical limits.
// Returns -1 if the input is not finite (NaN/Inf) - "reject".
int clamp(Joint joint, float requestedAngle) {
  if (!isfinite(requestedAngle)) {
    return -1;
  }
  int rounded = static_cast<int>(lroundf(requestedAngle));
  if (rounded < MIN_ANGLE[joint]) rounded = MIN_ANGLE[joint];
  if (rounded > MAX_ANGLE[joint]) rounded = MAX_ANGLE[joint];
  return rounded;
}

// Configures PWM timers only. Does NOT attach or drive any servo, so no
// GPIO carries a PWM signal until attach() is explicitly called.
void init() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
}

// Attaches all servos and writes initialAngle[] as the very first pulse.
// Call this ONLY once the host has sent HOME - never automatically at boot -
// since there is no position feedback and an unconditional write here would
// be an uncommanded physical motion.
void attachAll(const float initialAngle[JOINT_COUNT]) {
  for (size_t j = 0; j < JOINT_COUNT; ++j) {
    servos[j]->setPeriodHertz(SERVO_PWM_HZ);
    servos[j]->attach(SERVO_PINS[j], SERVO_MIN_US, SERVO_MAX_US);
    servos[j]->write(static_cast<int>(lroundf(initialAngle[j])));
  }
  attached = true;
}

bool isAttached() { return attached; }

void writeAll(const float currentAngle[JOINT_COUNT]) {
  for (size_t j = 0; j < JOINT_COUNT; ++j) {
    servos[j]->write(static_cast<int>(lroundf(currentAngle[j])));
  }
}

} // namespace servo_control

// ============================================================
// MOTION CONTROL (interpolation / target tracking)
// ============================================================
namespace motion_control {

float targetAngle[JOINT_COUNT];
float currentAngle[JOINT_COUNT];
bool  useHomingGlide = false; // slower, more conservative glide while homing

void seedCurrentAngles(const float seedAngles[JOINT_COUNT]) {
  for (size_t j = 0; j < JOINT_COUNT; ++j) {
    currentAngle[j] = seedAngles[j];
  }
}

void setTarget(Joint joint, float clampedAngle) {
  targetAngle[joint] = clampedAngle;
}

void setAllTargets(const float clampedAngles[JOINT_COUNT]) {
  for (size_t j = 0; j < JOINT_COUNT; ++j) {
    targetAngle[j] = clampedAngles[j];
  }
}

// Freezes motion by making every joint's target equal to its current
// position, so nothing moves until a fresh command sets a new target.
void freezeInPlace() {
  for (size_t j = 0; j < JOINT_COUNT; ++j) {
    targetAngle[j] = currentAngle[j];
  }
}

// Advances interpolation by one control cycle.
// frozen=true holds position (no motion applied) - used during STOP/watchdog.
// Returns true if every joint is within ANGLE_DEADBAND_DEG of its target.
bool update(bool frozen) {
  bool allSettled = true;
  for (size_t j = 0; j < JOINT_COUNT; ++j) {
    const float error = targetAngle[j] - currentAngle[j];

    if (!frozen) {
      if (fabsf(error) <= ANGLE_DEADBAND_DEG) {
        currentAngle[j] = targetAngle[j];
      } else {
        const float glide = useHomingGlide ? HOMING_GLIDE_FACTOR : GLIDE_FACTOR[j];
        float step = error * glide;
        if (step > MAX_STEP_DEG_PER_CYCLE)  step = MAX_STEP_DEG_PER_CYCLE;
        if (step < -MAX_STEP_DEG_PER_CYCLE) step = -MAX_STEP_DEG_PER_CYCLE;
        currentAngle[j] += step;
      }
    }

    if (fabsf(targetAngle[j] - currentAngle[j]) > ANGLE_DEADBAND_DEG) {
      allSettled = false;
    }
  }
  return allSettled;
}

const float* currentAngles() { return currentAngle; }

} // namespace motion_control

// ============================================================
// SAFETY (heartbeat watchdog / stop-cause tracking)
// ============================================================
namespace safety {

uint32_t lastMessageMs = 0;
bool stopped = false;
bool latched = false; // watchdog and emergency both latch until explicitly cleared
StopCause stopCause = StopCause::NONE;

void init() {
  lastMessageMs = millis();
  stopped = false;
  latched = false;
  stopCause = StopCause::NONE;
}

// Call whenever a valid command/heartbeat arrives.
void notifyAlive() {
  lastMessageMs = millis();
}

// Call once per loop. Returns true the instant a heartbeat timeout trips.
bool pollHeartbeat() {
  if (!latched && (millis() - lastMessageMs > HEARTBEAT_TIMEOUT_MS)) {
    stopped = true;
    latched = true;
    stopCause = StopCause::HEARTBEAT;
    return true;
  }
  return false;
}

// Explicitly trigger a stop with a known cause (USER or EMERGENCY).
void triggerStop(StopCause cause) {
  stopped = true;
  stopCause = cause;
  if (cause == StopCause::EMERGENCY) {
    latched = true; // requires an explicit clear via a fresh motion command
  }
}

// Clears the stopped/latch state - called when a fresh motion command
// arrives, so the arm is allowed to move again.
void clearStop() {
  stopped = false;
  latched = false;
  stopCause = StopCause::NONE;
  lastMessageMs = millis();
}

bool isStopped()      { return stopped; }
StopCause cause()     { return stopCause; }

} // namespace safety

// ============================================================
// SERIAL PARSER (protocol parsing / dispatch)
// ============================================================
namespace serial_parser {

char   rxBuffer[RX_BUFFER_SIZE];
size_t rxLength = 0;
bool   rxOverflow = false;
bool   homingRequested = false; // consumed by main loop

void reportError(const char* code) {
  Serial.print("ERR:");
  Serial.println(code);
  Serial.println("STATUS:ERROR");
}

// Parses one numeric token in place. Rejects empty input, trailing
// garbage, and non-finite results (NaN/Inf from a bad strtof parse).
bool parseFloatToken(char* text, float& result) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
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
  if (strcmp(command, "ROTATE_TO")   == 0) return BASE;
  if (strcmp(command, "SHOULDER_TO") == 0) return SHOULDER;
  if (strcmp(command, "ELBOW_TO")    == 0) return ELBOW;
  if (strcmp(command, "TILT_TO")     == 0) return WRIST_TILT;
  if (strcmp(command, "WRIST_TO")    == 0) return WRIST_ROTATE;
  if (strcmp(command, "GRIP")        == 0) return GRIPPER;
  return -1;
}

// Handles "MOVE:a,b,c,d,e,f". Parses and validates all six values BEFORE
// applying any of them, so a malformed packet can never leave the arm with
// a half-updated set of targets.
void processMoveCommand(char* valueText) {
  if (!servo_control::isAttached()) {
    reportError("NOT_HOMED");
    return;
  }

  float  parsed[JOINT_COUNT];
  size_t count = 0;
  char*  cursor = valueText;

  while (cursor != nullptr && *cursor != '\0' && count < JOINT_COUNT) {
    char* commaAt = strchr(cursor, ',');
    if (commaAt != nullptr) {
      *commaAt = '\0';
    }
    float value;
    if (!parseFloatToken(cursor, value)) {
      reportError("MOVE_ANGLE");
      return;
    }
    parsed[count++] = value;
    cursor = (commaAt != nullptr) ? (commaAt + 1) : nullptr;
  }

  if (count != JOINT_COUNT || cursor != nullptr) {
    reportError("MOVE_COUNT");
    return;
  }

  int   clamped[JOINT_COUNT];
  float clampedFloat[JOINT_COUNT];
  for (size_t j = 0; j < JOINT_COUNT; ++j) {
    clamped[j] = servo_control::clamp(static_cast<Joint>(j), parsed[j]);
    if (clamped[j] < 0) {
      reportError("MOVE_ANGLE");
      return;
    }
    clampedFloat[j] = static_cast<float>(clamped[j]);
  }

  motion_control::setAllTargets(clampedFloat);
  safety::clearStop();
  safety::notifyAlive();
  Serial.printf("OK:MOVE:%d,%d,%d,%d,%d,%d\n",
                clamped[0], clamped[1], clamped[2],
                clamped[3], clamped[4], clamped[5]);
}

void processLine(char* line) {
  if (strcmp(line, "PING") == 0) {
    safety::notifyAlive();
    return;
  }
  if (strcmp(line, "STOP") == 0) {
    safety::notifyAlive();
    safety::triggerStop(StopCause::USER);
    motion_control::freezeInPlace();
    Serial.println("OK:STOP");
    return;
  }
  if (strcmp(line, "ESTOP") == 0) {
    safety::notifyAlive();
    safety::triggerStop(StopCause::EMERGENCY);
    motion_control::freezeInPlace();
    Serial.println("OK:ESTOP");
    return;
  }
  if (strcmp(line, "HOME") == 0) {
    homingRequested = true;
    Serial.println("OK:HOME");
    return;
  }

  char* separator = strchr(line, ':');
  if (separator == nullptr) {
    reportError("FORMAT");
    return;
  }
  *separator = '\0';
  char* valueText = separator + 1;

  if (strcmp(line, "MOVE") == 0) {
    processMoveCommand(valueText);
    return;
  }

  const int jointIndex = jointForCommand(line);
  if (jointIndex < 0) {
    reportError("COMMAND");
    return;
  }

  if (!servo_control::isAttached()) {
    reportError("NOT_HOMED");
    return;
  }

  float requestedAngle = 0.0f;
  if (!parseFloatToken(valueText, requestedAngle)) {
    reportError("ANGLE");
    return;
  }

  const int safeAngle = servo_control::clamp(static_cast<Joint>(jointIndex), requestedAngle);
  if (safeAngle < 0) {
    reportError("ANGLE");
    return;
  }

  motion_control::setTarget(static_cast<Joint>(jointIndex), static_cast<float>(safeAngle));
  safety::clearStop(); // a fresh angle command intentionally resumes control
  safety::notifyAlive();
  Serial.printf("OK:%s:%d\n", line, safeAngle);
}

// Reads whatever bytes are currently available (never blocks), assembles
// complete lines, and dispatches them. Safe to call every loop iteration.
void poll() {
  while (Serial.available() > 0) {
    const char received = static_cast<char>(Serial.read());

    if (received == '\r') {
      continue;
    }
    // Drop non-printable/corrupted bytes outright - guards against UART
    // line noise instead of letting it corrupt the buffer.
    if (received != '\n' && (received < 32 || received > 126)) {
      continue;
    }

    if (received == '\n') {
      if (rxOverflow) {
        reportError("LINE_TOO_LONG");
      } else if (rxLength > 0) {
        rxBuffer[rxLength] = '\0';
        processLine(rxBuffer);
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

bool consumeHomingRequest() {
  if (!homingRequested) return false;
  homingRequested = false;
  return true;
}

} // namespace serial_parser

// ============================================================
// MAIN
// ============================================================
SystemState currentState = SystemState::UNHOMED;
bool wasSettled = false; // for edge-triggered DONE

void reportState(SystemState newState) {
  if (newState == currentState) return;
  currentState = newState;
  switch (newState) {
    case SystemState::UNHOMED:  Serial.println("STATUS:UNHOMED");  break;
    case SystemState::HOMING:   Serial.println("STATUS:HOMING");   break;
    case SystemState::READY:    Serial.println("STATUS:READY");    break;
    case SystemState::MOVING:   Serial.println("STATUS:MOVING");   break;
    case SystemState::IDLE:     Serial.println("STATUS:IDLE");     break;
    case SystemState::STOPPED:  Serial.println("STATUS:STOPPED");  break;
    case SystemState::WATCHDOG: Serial.println("STATUS:WATCHDOG"); break;
  }
}

// Begins the homing sequence: loads last-known-settled angles from NVS if
// available (normal case after a clean prior run), otherwise falls back to
// SAFE_HOME as a one-time direct hold on the very first-ever boot. Servos
// are attached here for the first time - never automatically at power-up -
// so no uncommanded motion can occur before the host explicitly sends HOME.
void beginHoming() {
  float seed[JOINT_COUNT];
  bool havePersisted = persistence::loadAngles(seed);
  if (!havePersisted) {
    for (size_t j = 0; j < JOINT_COUNT; ++j) {
      seed[j] = static_cast<float>(SAFE_HOME[j]);
    }
  }

  float home[JOINT_COUNT];
  for (size_t j = 0; j < JOINT_COUNT; ++j) {
    home[j] = static_cast<float>(SAFE_HOME[j]);
  }

  servo_control::attachAll(seed);
  motion_control::seedCurrentAngles(seed);
  motion_control::useHomingGlide = true;
  motion_control::setAllTargets(home);
  safety::clearStop();
  safety::notifyAlive();
}

void setup() {
  Serial.begin(BAUD_RATE);
  servo_control::init();  // PWM timers only - no attach, no motion
  safety::init();
  Serial.println("Artefact Guardian ready - UNHOMED, send HOME to enable servos");
  reportState(SystemState::UNHOMED);
}

void loop() {
  serial_parser::poll();

  if (serial_parser::consumeHomingRequest()) {
    beginHoming();
    reportState(SystemState::HOMING);
  }

  if (!servo_control::isAttached()) {
    // Fully inert: no interpolation, no servo writes, no watchdog ticking.
    delay(CONTROL_PERIOD_MS);
    return;
  }

  if (safety::pollHeartbeat()) {
    motion_control::freezeInPlace();
  }

  const bool frozen  = safety::isStopped();
  const bool settled = motion_control::update(frozen);
  servo_control::writeAll(motion_control::currentAngles());

  if (settled && motion_control::useHomingGlide) {
    motion_control::useHomingGlide = false; // homing glide only applies once
  }

  if (settled && !wasSettled && !frozen) {
    Serial.println("DONE");
    persistence::saveAngles(motion_control::currentAngles());
  }
  wasSettled = settled;

  if (frozen) {
    reportState(safety::cause() == StopCause::HEARTBEAT
                    ? SystemState::WATCHDOG
                    : SystemState::STOPPED);
    if (safety::cause() == StopCause::USER) {
      persistence::saveAngles(motion_control::currentAngles());
    }
  } else if (currentState == SystemState::HOMING) {
    if (settled) reportState(SystemState::READY);
  } else if (!settled) {
    reportState(SystemState::MOVING);
  } else {
    reportState(SystemState::IDLE);
  }

  delay(CONTROL_PERIOD_MS);
}