/*
  5 DOF Servo Robotic Arm Firmware
  ESP32 Arduino Core
  Libraries: ESP32Servo, Preferences

  Servo Mapping:
    Base       -> GPIO13
    Shoulder   -> GPIO25
    Elbow      -> GPIO12
    Wrist      -> GPIO27
    Gripper    -> GPIO26

  Serial Commands:
    PING
    STOP
    ROTATE_TO:angle     (Base)
    SHOULDER_TO:angle   (Shoulder)
    ELBOW_TO:angle      (Elbow)
    TILT_TO:angle       (Wrist)
    GRIP:angle          (Gripper)
    WRIST_TO:angle      (ignored - robot has no dedicated wrist rotate servo)
    CALIBRATE
    SAVE
    HOME

  Calibration-mode-only commands:
    BASE+5 / BASE-5
    SHOULDER+5 / SHOULDER-5
    ELBOW+5 / ELBOW-5
    WRIST+5 / WRIST-5
    GRIP+2 / GRIP-2

  Motion: trapezoidal, acceleration-limited, velocity-based. No delay().
*/

#include <ESP32Servo.h>
#include <Preferences.h>

// ---------------------------------------------------------------------------
// Pin Mapping
// ---------------------------------------------------------------------------
#define PIN_BASE      13
#define PIN_SHOULDER  25
#define PIN_ELBOW     12
#define PIN_WRIST     27
#define PIN_GRIPPER   26

// ---------------------------------------------------------------------------
// Joint identifiers
// ---------------------------------------------------------------------------
enum JointId {
  JOINT_BASE = 0,
  JOINT_SHOULDER,
  JOINT_ELBOW,
  JOINT_WRIST,
  JOINT_GRIPPER,
  JOINT_COUNT
};

// ---------------------------------------------------------------------------
// Joint structure
// ---------------------------------------------------------------------------
struct Joint {
  Servo servo;
  int pin;

  float currentAngle;
  float targetAngle;
  float currentVelocity;   // deg/sec, signed

  float maxSpeed;          // deg/sec
  float acceleration;      // deg/sec^2

  float minAngle;
  float maxAngle;

  float homeAngle;

  const char* name;
};

Joint joints[JOINT_COUNT];

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
unsigned long lastMotionUpdateMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastCommandMs = 0;

const unsigned long MOTION_UPDATE_INTERVAL_MS = 20;   // 50 Hz motion loop
const unsigned long HEARTBEAT_INTERVAL_MS     = 1000; // 1 Hz heartbeat print
const unsigned long WATCHDOG_TIMEOUT_MS       = 5000; // no command -> stop

bool watchdogTripped = false;

// ---------------------------------------------------------------------------
// Mode
// ---------------------------------------------------------------------------
bool calibrationMode = false;
bool homingInProgress = false;

// ---------------------------------------------------------------------------
// Preferences (EEPROM-like storage)
// ---------------------------------------------------------------------------
Preferences prefs;
const char* PREFS_NAMESPACE = "robotarm";

// ---------------------------------------------------------------------------
// Serial input buffer
// ---------------------------------------------------------------------------
String serialBuffer = "";

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void setupJoints();
void updateMotion();
void updateJoint(Joint &j, float dtSeconds);
void moveJoint(JointId id, float targetAngle);
void processCommand(String cmd);
void handleCalibrationCommand(String cmd);
void saveCalibration();
void loadCalibration();
void applyServoOutput();
void stopAllJoints();
void sendAck(const String &cmdEcho);
bool jointsAtHome();
bool jointAtTarget(Joint &j);
float clampf(float v, float lo, float hi);

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB serial (non-blocking on most boards) */ }

  // Allow allocation of all timers for ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  setupJoints();

  prefs.begin(PREFS_NAMESPACE, false);
  loadCalibration();
  prefs.end();

  // On boot, assume physical servos are at their home position.
  for (int i = 0; i < JOINT_COUNT; i++) {
    joints[i].currentAngle  = joints[i].homeAngle;
    joints[i].targetAngle   = joints[i].homeAngle;
    joints[i].currentVelocity = 0.0f;
    joints[i].servo.write((int)joints[i].currentAngle);
  }

  lastMotionUpdateMs = millis();
  lastHeartbeatMs    = millis();
  lastCommandMs      = millis();

  Serial.println("READY");
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop() {
  // ---- Serial read (non-blocking) ----
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        serialBuffer.trim();
        if (serialBuffer.length() > 0) {
          lastCommandMs = millis();
          watchdogTripped = false;
          if (calibrationMode) {
            handleCalibrationCommand(serialBuffer);
          } else {
            processCommand(serialBuffer);
          }
        }
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 64) {
        // Guard against runaway buffers
        serialBuffer = "";
      }
    }
  }

  // ---- Watchdog ----
  unsigned long now = millis();
  if (!watchdogTripped && (now - lastCommandMs > WATCHDOG_TIMEOUT_MS)) {
    watchdogTripped = true;
    stopAllJoints();
    Serial.println("WATCHDOG TIMEOUT");
  }

  // ---- Motion update (fixed-ish timestep) ----
  if (now - lastMotionUpdateMs >= MOTION_UPDATE_INTERVAL_MS) {
    float dt = (now - lastMotionUpdateMs) / 1000.0f;
    lastMotionUpdateMs = now;
    updateMotion();
    for (int i = 0; i < JOINT_COUNT; i++) {
      (void)dt;
    }
  }

  // ---- Homing completion check ----
  if (homingInProgress && jointsAtHome()) {
    homingInProgress = false;
    Serial.println("HOME COMPLETE");
  }

  // ---- Heartbeat ----
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    Serial.println("HEARTBEAT");
  }
}

// ---------------------------------------------------------------------------
// Joint setup: pins, limits, speed/acceleration profiles
// ---------------------------------------------------------------------------
void setupJoints() {
  joints[JOINT_BASE].pin          = PIN_BASE;
  joints[JOINT_BASE].name         = "BASE";
  joints[JOINT_BASE].minAngle     = 0.0f;
  joints[JOINT_BASE].maxAngle     = 180.0f;
  joints[JOINT_BASE].maxSpeed     = 60.0f;   // deg/sec
  joints[JOINT_BASE].acceleration = 90.0f;   // deg/sec^2
  joints[JOINT_BASE].homeAngle    = 90.0f;

  joints[JOINT_SHOULDER].pin          = PIN_SHOULDER;
  joints[JOINT_SHOULDER].name         = "SHOULDER";
  joints[JOINT_SHOULDER].minAngle     = 0.0f;
  joints[JOINT_SHOULDER].maxAngle     = 180.0f;
  joints[JOINT_SHOULDER].maxSpeed     = 50.0f;
  joints[JOINT_SHOULDER].acceleration = 70.0f;
  joints[JOINT_SHOULDER].homeAngle    = 90.0f;

  joints[JOINT_ELBOW].pin          = PIN_ELBOW;
  joints[JOINT_ELBOW].name         = "ELBOW";
  joints[JOINT_ELBOW].minAngle     = 0.0f;
  joints[JOINT_ELBOW].maxAngle     = 180.0f;
  joints[JOINT_ELBOW].maxSpeed     = 55.0f;
  joints[JOINT_ELBOW].acceleration = 80.0f;
  joints[JOINT_ELBOW].homeAngle    = 90.0f;

  joints[JOINT_WRIST].pin          = PIN_WRIST;
  joints[JOINT_WRIST].name         = "WRIST";
  joints[JOINT_WRIST].minAngle     = 0.0f;
  joints[JOINT_WRIST].maxAngle     = 180.0f;
  joints[JOINT_WRIST].maxSpeed     = 65.0f;
  joints[JOINT_WRIST].acceleration = 100.0f;
  joints[JOINT_WRIST].homeAngle    = 90.0f;

  // Gripper: ~4x slower than the other joints, protects grasped artefacts
  joints[JOINT_GRIPPER].pin          = PIN_GRIPPER;
  joints[JOINT_GRIPPER].name         = "GRIPPER";
  joints[JOINT_GRIPPER].minAngle     = 0.0f;
  joints[JOINT_GRIPPER].maxAngle     = 180.0f;
  joints[JOINT_GRIPPER].maxSpeed     = 15.0f;  // ~60/4
  joints[JOINT_GRIPPER].acceleration = 20.0f;  // ~80/4
  joints[JOINT_GRIPPER].homeAngle    = 90.0f;

  for (int i = 0; i < JOINT_COUNT; i++) {
    joints[i].currentAngle    = joints[i].homeAngle;
    joints[i].targetAngle     = joints[i].homeAngle;
    joints[i].currentVelocity = 0.0f;
    joints[i].servo.setPeriodHertz(50);
    joints[i].servo.attach(joints[i].pin, 500, 2400);
  }
}

// ---------------------------------------------------------------------------
// Motion: trapezoidal acceleration-limited update for every joint
// ---------------------------------------------------------------------------
void updateMotion() {
  static unsigned long lastUpdate = millis();
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0f;
  lastUpdate = now;

  if (dt <= 0.0f) return;
  if (dt > 0.1f) dt = 0.1f; // clamp large gaps (e.g. after blocking serial reads)

  for (int i = 0; i < JOINT_COUNT; i++) {
    updateJoint(joints[i], dt);
  }

  applyServoOutput();
}

// Acceleration-limited, non-overshooting single-joint update.
void updateJoint(Joint &j, float dtSeconds) {
  float error = j.targetAngle - j.currentAngle;

  // Already essentially at target: snap and stop cleanly (no oscillation).
  if (fabs(error) < 0.05f && fabs(j.currentVelocity) < 0.5f) {
    j.currentAngle = j.targetAngle;
    j.currentVelocity = 0.0f;
    return;
  }

  float direction = (error >= 0.0f) ? 1.0f : -1.0f;
  float distanceToTarget = fabs(error);

  // Distance needed to decelerate from current speed to zero.
  float stoppingDistance = (j.currentVelocity * j.currentVelocity) /
                            (2.0f * j.acceleration);

  float desiredVelocity;

  if (distanceToTarget <= stoppingDistance) {
    // Decelerate toward zero so we land exactly on target, no overshoot.
    desiredVelocity = direction *
        sqrt(2.0f * j.acceleration * max(distanceToTarget, 0.0f));
  } else {
    // Accelerate (or cruise) toward max speed in the required direction.
    desiredVelocity = direction * j.maxSpeed;
  }

  // Apply acceleration limit when moving currentVelocity -> desiredVelocity.
  float velocityDelta = desiredVelocity - j.currentVelocity;
  float maxDeltaThisStep = j.acceleration * dtSeconds;
  velocityDelta = clampf(velocityDelta, -maxDeltaThisStep, maxDeltaThisStep);

  j.currentVelocity += velocityDelta;
  j.currentVelocity = clampf(j.currentVelocity, -j.maxSpeed, j.maxSpeed);

  // Integrate position.
  float step = j.currentVelocity * dtSeconds;

  // Never overshoot the target in a single step.
  if ((direction > 0 && step > distanceToTarget) ||
      (direction < 0 && -step > distanceToTarget)) {
    j.currentAngle = j.targetAngle;
    j.currentVelocity = 0.0f;
  } else {
    j.currentAngle += step;
  }

  j.currentAngle = clampf(j.currentAngle, j.minAngle, j.maxAngle);
}

void applyServoOutput() {
  for (int i = 0; i < JOINT_COUNT; i++) {
    int angleInt = (int)round(joints[i].currentAngle);
    angleInt = (int)clampf(angleInt, joints[i].minAngle, joints[i].maxAngle);
    joints[i].servo.write(angleInt);
  }
}

// ---------------------------------------------------------------------------
// Command a joint to move toward a new target (keeps current velocity,
// so continuous streams of targets glide instead of restarting motion).
// ---------------------------------------------------------------------------
void moveJoint(JointId id, float targetAngle) {
  Joint &j = joints[id];
  targetAngle = clampf(targetAngle, j.minAngle, j.maxAngle);
  j.targetAngle = targetAngle;
}

// ---------------------------------------------------------------------------
// Normal-mode command processing
// ---------------------------------------------------------------------------
void processCommand(String cmd) {
  cmd.trim();

  if (cmd.equalsIgnoreCase("PING")) {
    sendAck("PING");
    return;
  }

  if (cmd.equalsIgnoreCase("STOP")) {
    stopAllJoints();
    sendAck("STOP");
    return;
  }

  if (cmd.equalsIgnoreCase("CALIBRATE")) {
    calibrationMode = true;
    Serial.println("CALIBRATION MODE");
    return;
  }

  if (cmd.equalsIgnoreCase("SAVE")) {
    saveCalibration();
    Serial.println("SAVED");
    return;
  }

  if (cmd.equalsIgnoreCase("HOME")) {
    calibrationMode = false;
    homingInProgress = true;
    for (int i = 0; i < JOINT_COUNT; i++) {
      moveJoint((JointId)i, joints[i].homeAngle);
    }
    sendAck("HOME");
    return;
  }

  int sepIndex = cmd.indexOf(':');
  if (sepIndex < 0) {
    // Unrecognized command with no parameter — silently ignore.
    return;
  }

  String key = cmd.substring(0, sepIndex);
  String valueStr = cmd.substring(sepIndex + 1);
  valueStr.trim();
  float value = valueStr.toFloat();

  if (key.equalsIgnoreCase("ROTATE_TO")) {
    moveJoint(JOINT_BASE, value);
    sendAck("ROTATE_TO:" + valueStr);
  } else if (key.equalsIgnoreCase("SHOULDER_TO")) {
    moveJoint(JOINT_SHOULDER, value);
    sendAck("SHOULDER_TO:" + valueStr);
  } else if (key.equalsIgnoreCase("ELBOW_TO")) {
    moveJoint(JOINT_ELBOW, value);
    sendAck("ELBOW_TO:" + valueStr);
  } else if (key.equalsIgnoreCase("TILT_TO")) {
    moveJoint(JOINT_WRIST, value);
    sendAck("TILT_TO:" + valueStr);
  } else if (key.equalsIgnoreCase("GRIP")) {
    moveJoint(JOINT_GRIPPER, value);
    sendAck("GRIP:" + valueStr);
  } else if (key.equalsIgnoreCase("WRIST_TO")) {
    // Explicitly ignored: robot has no dedicated wrist-rotate servo.
    return;
  }
  // Any other unknown command is silently ignored.
}

// ---------------------------------------------------------------------------
// Calibration-mode command processing
// ---------------------------------------------------------------------------
void handleCalibrationCommand(String cmd) {
  cmd.trim();

  // Allow escaping calibration mode via the global commands too.
  if (cmd.equalsIgnoreCase("SAVE")) {
    saveCalibration();
    Serial.println("SAVED");
    return;
  }
  if (cmd.equalsIgnoreCase("HOME")) {
    calibrationMode = false;
    homingInProgress = true;
    for (int i = 0; i < JOINT_COUNT; i++) {
      moveJoint((JointId)i, joints[i].homeAngle);
    }
    sendAck("HOME");
    return;
  }
  if (cmd.equalsIgnoreCase("STOP")) {
    stopAllJoints();
    sendAck("STOP");
    return;
  }
  if (cmd.equalsIgnoreCase("PING")) {
    sendAck("PING");
    return;
  }

  struct CalMap {
    const char* token;
    JointId joint;
    float delta;
  };

  static const CalMap table[] = {
    { "BASE+5",      JOINT_BASE,     5.0f  },
    { "BASE-5",      JOINT_BASE,    -5.0f  },
    { "SHOULDER+5",  JOINT_SHOULDER, 5.0f  },
    { "SHOULDER-5",  JOINT_SHOULDER,-5.0f  },
    { "ELBOW+5",     JOINT_ELBOW,    5.0f  },
    { "ELBOW-5",     JOINT_ELBOW,   -5.0f  },
    { "WRIST+5",     JOINT_WRIST,    5.0f  },
    { "WRIST-5",     JOINT_WRIST,   -5.0f  },
    { "GRIP+2",      JOINT_GRIPPER,  2.0f  },
    { "GRIP-2",      JOINT_GRIPPER, -2.0f  },
  };

  for (unsigned int i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
    if (cmd.equalsIgnoreCase(table[i].token)) {
      Joint &j = joints[table[i].joint];
      float newTarget = j.targetAngle + table[i].delta;
      moveJoint(table[i].joint, newTarget);
      sendAck(String(table[i].token));
      return;
    }
  }

  // Unrecognized calibration command — ignored.
}

// ---------------------------------------------------------------------------
// Preferences: save / load calibration (home positions)
// ---------------------------------------------------------------------------
void saveCalibration() {
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putFloat("baseHome",     joints[JOINT_BASE].targetAngle);
  prefs.putFloat("shoulderHome", joints[JOINT_SHOULDER].targetAngle);
  prefs.putFloat("elbowHome",    joints[JOINT_ELBOW].targetAngle);
  prefs.putFloat("wristHome",    joints[JOINT_WRIST].targetAngle);
  prefs.putFloat("gripperHome",  joints[JOINT_GRIPPER].targetAngle);
  prefs.end();

  joints[JOINT_BASE].homeAngle     = joints[JOINT_BASE].targetAngle;
  joints[JOINT_SHOULDER].homeAngle = joints[JOINT_SHOULDER].targetAngle;
  joints[JOINT_ELBOW].homeAngle    = joints[JOINT_ELBOW].targetAngle;
  joints[JOINT_WRIST].homeAngle    = joints[JOINT_WRIST].targetAngle;
  joints[JOINT_GRIPPER].homeAngle  = joints[JOINT_GRIPPER].targetAngle;
}

void loadCalibration() {
  joints[JOINT_BASE].homeAngle =
      prefs.getFloat("baseHome", joints[JOINT_BASE].homeAngle);
  joints[JOINT_SHOULDER].homeAngle =
      prefs.getFloat("shoulderHome", joints[JOINT_SHOULDER].homeAngle);
  joints[JOINT_ELBOW].homeAngle =
      prefs.getFloat("elbowHome", joints[JOINT_ELBOW].homeAngle);
  joints[JOINT_WRIST].homeAngle =
      prefs.getFloat("wristHome", joints[JOINT_WRIST].homeAngle);
  joints[JOINT_GRIPPER].homeAngle =
      prefs.getFloat("gripperHome", joints[JOINT_GRIPPER].homeAngle);
}

// ---------------------------------------------------------------------------
// Stop: freeze every joint exactly where it is right now.
// ---------------------------------------------------------------------------
void stopAllJoints() {
  for (int i = 0; i < JOINT_COUNT; i++) {
    joints[i].targetAngle = joints[i].currentAngle;
    joints[i].currentVelocity = 0.0f;
  }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void sendAck(const String &cmdEcho) {
  Serial.print("ACK:");
  Serial.println(cmdEcho);
}

bool jointAtTarget(Joint &j) {
  return fabs(j.targetAngle - j.currentAngle) < 0.1f &&
         fabs(j.currentVelocity) < 0.1f;
}

bool jointsAtHome() {
  for (int i = 0; i < JOINT_COUNT; i++) {
    if (fabs(joints[i].currentAngle - joints[i].homeAngle) > 0.5f) {
      return false;
    }
  }
  return true;
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
