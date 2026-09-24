# Artefact Guardian 🤖

### A robotic arm that moves with hand gestures. Yes, we made a robot listen to our hands.

Artefact Guardian is our **WRO 2026 Future Innovators** project based on the theme **"Robots Meet Culture."**

The basic idea is pretty simple:

**Use computer vision to detect hand gestures → send commands to an ESP32 → move a robotic arm → handle artefacts without someone having to constantly touch them.**

We are still improving a lot of things, but this is where we've reached so far.

---

## So... what does it actually do?

The system uses a camera to look at your hand and detect its movement and gestures.

Those gestures are turned into commands, which are sent to an **ESP32**. The ESP32 then controls the servos on the arm.

Basically:

```text
Your hand
   ↓
Camera
   ↓
Hand tracking
   ↓
Gesture recognition
   ↓
ESP32
   ↓
Servos
   ↓
Robotic arm
   ↓
Artefact
```

We wanted the interaction to feel more natural than using a joystick or a bunch of buttons.

Also, waving at a robot is way cooler.

---

## Why did we make this?

Cultural artefacts can be extremely fragile and valuable. Moving them around manually isn't always ideal, especially when they need to be handled repeatedly.

Our idea was to build a system that can help with controlled handling while keeping direct human interaction to a minimum.

That's the idea at least.

Now we just have to make the robot actually cooperate.

---

## What are we using?

### Hardware

* **ESP32**
* **5 × DS3218 servo motors**
* **2 × 18650 batteries**
* **Buck converter**
* **Camera**
* Custom mechanical parts
* 3D-printed components

The five servos are used for the different movements of the arm, including the base, shoulder, elbow, wrist and gripper.

### Software

* **Python**
* **MediaPipe**
* Computer vision / hand tracking
* **ESP32 firmware**
* Servo control

### CAD

* **Fusion 360**
* 3D modelling
* Mechanical design and prototyping

---

## Gesture Control 🖐️

Our computer vision system detects the hand and uses the landmarks from it to figure out what we're doing.

Some of the gestures we've been using include:

```text
Open Palm  →  Activate / Control
Fist       →  Stop / Hold
Hand Motion → Arm movement
```

The exact control system is still being worked on and calibrated.

The robot does sometimes misunderstand us.

We're choosing to call that "AI behaviour."

---

## The arm

The arm has multiple degrees of movement controlled by the servo motors.

Current servo setup:

```text
Base
  ↓
Shoulder
  ↓
Elbow
  ↓
Wrist
  ↓
Gripper
```

The gripper is designed to pick up and move an object once the arm is positioned correctly.

Getting the arm to move is one thing.

Getting it to move **where we actually want it to move** is a completely different problem.

---

## Repository

Everything is being kept in this repo so the project doesn't become one giant folder called:

`FINAL_FINAL_REAL_ONE_USE_THIS`

Current structure:

```text
Artefact-Guardian-WRO-2026/
│
├── Artefact-Guardian-CAD/
│   └── CAD files and mechanical designs
│
├── Computer_vision/
│   └── Hand tracking and gesture recognition
│
├── Firmware/
│   └── ESP32/
│       └── Robot control code
│
├── Hardware/
│   └── Electronics and hardware information
│
├── documentation/
│   └── Notes, development and testing
│
└── .github/
```

---

## Development

This project has gone through a lot of trial and error.

We've tried different ideas for:

* Arm positioning
* IMU sensing
* Gesture recognition
* Servo control
* Mechanical design
* Computer vision
* Calibration

A fairly accurate representation of development has been:

```text
Build it
   ↓
Test it
   ↓
Something doesn't work
   ↓
Spend an unreasonable amount of time figuring out why
   ↓
Change three things
   ↓
It works
   ↓
Break something else
   ↓
Repeat
```

That's basically robotics.

---

## CAD & Mechanical Design

The mechanical parts of the robot are designed in Fusion 360 before being made.

This includes things like:

* Arm components
* Gripper
* Servo mounts
* Structural parts
* Different design iterations

Not every design made it to the final robot.

Some were good.

Some looked good until we actually tried to build them.

---

## Testing & Calibration

A large part of this project has been calibration.

We've been testing:

* Servo positions
* Joint limits
* Gripper movement
* Arm movement
* Gesture detection
* Power delivery
* Mechanical stability
* Repeatability

A robot can look perfectly aligned on a computer screen and then completely change its opinion once it exists in real life.

---

## Current Status

### ✅ Working

* ESP32 servo control
* Robotic arm movement
* Hand tracking
* Gesture-based control
* CAD development
* Basic hardware integration

### 🔧 Still working on

* Better gesture reliability
* More precise arm movement
* Calibration
* Object positioning
* Mechanical improvements
* Full system integration

There is still quite a bit left to improve.

---

## Future Ideas

Some things we'd like to add or improve:

* Better object detection
* Better wrist control
* More accurate positioning
* Collision detection
* Smoother movement
* Improved gripper design
* More reliable gesture recognition
* Better integration between the vision system and the arm
* Possibly using another camera for more accurate positioning

The goal is to get from:

> "Look, the robot moved."

to:

> "Look, it actually did exactly what we wanted."

---

## WRO 2026 🏆

**Competition:** World Robot Olympiad 2026 – Future Innovators
**Theme:** Robots Meet Culture
**Team:** JNS Future Minds
**School:** Jamnabai Narsee School, GIFT City, Gandhinagar

### Team

* **Shaurya Bahety**
* **Raj Singh**
* **Vivaan Vashistha**

---

## Photos / Videos

We'll keep adding photos, CAD renders and testing videos here as the project develops.

Because obviously you can't have a robotics project without showing the robot.

---

## Open Source

This project is open source, so you can look through the:

* Code
* CAD
* Hardware
* Documentation
* Experiments

Hopefully it is useful to someone else working on robotics, computer vision or just trying to make a robot do something vaguely useful.

Feedback and ideas are welcome.

---

## Built with

**ESP32 + Python + MediaPipe + Servo Motors + Fusion 360 + 3D Printing + a lot of debugging**

---

# Why Artefact Guardian?

We wanted to build something where robotics could actually interact with culture instead of just being another robot that follows a line around a track.

The idea is to use technology to make the handling of cultural artefacts more controlled and practical.

And also...

we really wanted to build a robotic arm.

So here we are.

---

## 🚀 That's Artefact Guardian

We're still building, testing and breaking things.

More updates coming as the robot gets better.

**Build → Test → Break → Fix → Repeat.**

That's pretty much the entire project.
