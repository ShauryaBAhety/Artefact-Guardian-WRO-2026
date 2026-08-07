#include <Servo.h>

#define BAUD_RATE 115200

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoTilt;
Servo servoWrist;
Servo servoGrip;

// Change pins if needed
const int BASE_PIN      = 3;
const int SHOULDER_PIN  = 5;
const int ELBOW_PIN     = 6;
const int TILT_PIN      = 9;
const int WRIST_PIN     = 10;
const int GRIP_PIN      = 11;

String command = "";

void setup() {
  Serial.begin(BAUD_RATE);

  servoBase.attach(BASE_PIN);
  servoShoulder.attach(SHOULDER_PIN);
  servoElbow.attach(ELBOW_PIN);
  servoTilt.attach(TILT_PIN);
  servoWrist.attach(WRIST_PIN);
  servoGrip.attach(GRIP_PIN);

  servoBase.write(90);
  servoShoulder.write(90);
  servoElbow.write(90);
  servoTilt.write(90);
  servoWrist.write(90);
  servoGrip.write(30);

  Serial.println("Arduino Servo Controller Ready");
}

void loop() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {
      processCommand(command);
      command = "";
    } else if (c != '\r') {
      command += c;
    }
  }
}

void processCommand(String cmd) {

  cmd.trim();

  if (cmd.startsWith("ROTATE_TO:")) {
    servoBase.write(constrain(cmd.substring(10).toInt(),0,180));
  }

  else if (cmd.startsWith("SHOULDER_TO:")) {
    servoShoulder.write(constrain(cmd.substring(12).toInt(),0,180));
  }

  else if (cmd.startsWith("ELBOW_TO:")) {
    servoElbow.write(constrain(cmd.substring(9).toInt(),0,180));
  }

  else if (cmd.startsWith("TILT_TO:")) {
    servoTilt.write(constrain(cmd.substring(8).toInt(),0,180));
  }

  else if (cmd.startsWith("WRIST_TO:")) {
    servoWrist.write(constrain(cmd.substring(9).toInt(),0,180));
  }

  else if (cmd.startsWith("GRIP:")) {
    servoGrip.write(constrain(cmd.substring(5).toInt(),0,180));
  }

  else if (cmd == "PING") {
    Serial.println("PONG");
  }

  else if (cmd == "STOP") {
    // Arduino Servo library has no freeze command.
    // Servos simply remain at their current angle.
    Serial.println("OK:STOP");
  }
}