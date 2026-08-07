/*
  ===================================================================
  EXTREME SMOOTHING & ANTI-JITTER FIRMWARE FOR ESP32
  ===================================================================
  AMENDED for smoother motion (see comments marked "AMENDED"):
    1) Serial.setTimeout() lowered so a split incoming line can never
       stall the control loop and cause a "catch-up" jump.
    2) GLIDE_FACTOR lowered into the cinematic/gentle range.
    3) A hard per-loop degree cap (MAX_STEP_DEG) was added on top of
       the existing exponential glide, so a big target jump (e.g.
       engaging a new joint, or GRIP OPEN/CLOSE) can never produce a
       big physical step - velocity is now bounded no matter how far
       the target is.
  No command names, pins, or control logic were changed - Python and
  the gesture scheme work exactly as before.
  ===================================================================
*/

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Create Servo Objects
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoWristTilt;
Servo servoWristRotate; // Your 5th DOF
Servo servoGripper;

// Assign Safe ESP32 GPIO Pins
const int PIN_BASE         = 13;
const int PIN_SHOULDER     = 12;
const int PIN_ELBOW        = 25;
const int PIN_WRIST_TILT   = 26;
const int PIN_WRIST_ROTATE = 27;
const int PIN_GRIPPER      = 32;

// Target Angles (Where the camera wants the arm to go)
float targetBase        = 90.0;
float targetShoulder    = 90.0;
float targetElbow       = 90.0;
float targetWristTilt   = 90.0;
float targetWristRotate = 90.0;
float targetGripper     = 90.0; // Closed default

// Current Angles (Where the arm actually is right now)
float currentBase        = 90.0;
float currentShoulder    = 90.0;
float currentElbow       = 90.0;
float currentWristTilt   = 90.0;
float currentWristRotate = 90.0;
float currentGripper     = 90.0;

/* GLIDE FACTOR (The Magic Tuning Knob):
  0.01 to 0.05 = Ultra slow, cinematic, heavy fluid glide.
  0.08 to 0.15 = Snappy, smooth, ideal for robotics tracking.
  0.50+        = Fast, sharp (brings back camera twitching).
*/
const float GLIDE_FACTOR = 0.05;  // AMENDED: was 0.08 - moved into the gentle/cinematic range for fragile artefact handling

// AMENDED: hard velocity cap, independent of how large the target jump is.
// With a 15ms loop this caps speed at MAX_STEP_DEG / 0.015s.
// 1.0 deg/loop -> ~66 deg/sec max, a safe ceiling for handling delicate objects.
// Raise this slightly if the arm now feels sluggish; lower it if it still feels fast.
const float MAX_STEP_DEG = 1.0;

Adafruit_MPU6050 mpu;

// AMENDED: single helper used by all six servos so the cap is applied consistently.
// This does not change WHAT each servo does, only HOW FAST it's allowed to get there.
float moveToward(float current, float target, float glideFactor, float maxStepDeg) {
  float step = (target - current) * glideFactor;
  if (step > maxStepDeg)  step = maxStepDeg;
  if (step < -maxStepDeg) step = -maxStepDeg;
  return current + step;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);  // AMENDED: was default 1000ms - a slow/split incoming line could
                          // stall loop() for up to a second, then the arm would "catch up"
                          // with a burst of movement. 5ms is plenty for a line already
                          // arriving at 115200 baud and keeps the loop from ever blocking long.

  // Allocate hardware timers for ESP32 PWM channels
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach servos with standard 500us to 2500us pulse widths
  servoBase.attach(PIN_BASE, 500, 2500);
  servoShoulder.attach(PIN_SHOULDER, 500, 2500);
  servoElbow.attach(PIN_ELBOW, 500, 2500);
  servoWristTilt.attach(PIN_WRIST_TILT, 500, 2500);
  servoWristRotate.attach(PIN_WRIST_ROTATE, 500, 2500);
  servoGripper.attach(PIN_GRIPPER, 500, 2500);

  // Write starting positions
  servoBase.write((int)currentBase);
  servoShoulder.write((int)currentShoulder);
  servoElbow.write((int)currentElbow);
  servoWristTilt.write((int)currentWristTilt);
  servoWristRotate.write((int)currentWristRotate);
  servoGripper.write((int)currentGripper);

  // Wake up MPU-6050
  if (!mpu.begin()) {
    Serial.println("Warning: MPU-6050 not detected, skipping sensor init.");
  }
}

void loop() {
  // 1. Process Incoming Python Commands
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    int colonIndex = data.indexOf(':');
    if (colonIndex != -1) {
      String command = data.substring(0, colonIndex);
      String valueStr = data.substring(colonIndex + 1);
      float value = valueStr.toFloat();

      if (command == "ROTATE_TO")    targetBase        = constrain(value, 0, 180);
      else if (command == "SHOULDER_TO") targetShoulder    = constrain(value, 0, 180);
      else if (command == "ELBOW_TO")    targetElbow       = constrain(value, 0, 180);
      else if (command == "TILT_TO")     targetWristTilt   = constrain(value, 0, 180);
      else if (command == "WRIST_TO")    targetWristRotate = constrain(value, 0, 180);
      else if (command == "GRIP") {
        if (valueStr == "OPEN")  targetGripper = 30.0;  // Tune open angle
        if (valueStr == "CLOSE") targetGripper = 150.0; // Tune closed angle
      }
    }
  }

  // 2. Apply Mathematical Easing (The "Glide" Effect)
  // AMENDED: now goes through moveToward(), which applies GLIDE_FACTOR
  // AND clamps the step to MAX_STEP_DEG - same easing feel, but no
  // more large, fast first-steps on big target changes.
  currentBase        = moveToward(currentBase,        targetBase,        GLIDE_FACTOR, MAX_STEP_DEG);
  currentShoulder     = moveToward(currentShoulder,    targetShoulder,    GLIDE_FACTOR, MAX_STEP_DEG);
  currentElbow        = moveToward(currentElbow,       targetElbow,       GLIDE_FACTOR, MAX_STEP_DEG);
  currentWristTilt    = moveToward(currentWristTilt,   targetWristTilt,   GLIDE_FACTOR, MAX_STEP_DEG);
  currentWristRotate  = moveToward(currentWristRotate, targetWristRotate, GLIDE_FACTOR, MAX_STEP_DEG);
  currentGripper      = moveToward(currentGripper,     targetGripper,     GLIDE_FACTOR, MAX_STEP_DEG);

  // 3. Write Smooth Coordinates to Servos
  servoBase.write(round(currentBase));
  servoShoulder.write(round(currentShoulder));
  servoElbow.write(round(currentElbow));
  servoWristTilt.write(round(currentWristTilt));
  servoWristRotate.write(round(currentWristRotate));
  servoGripper.write(round(currentGripper));

  delay(15); // Consistent refresh cycle (~66Hz) matching standard servo updates
}
