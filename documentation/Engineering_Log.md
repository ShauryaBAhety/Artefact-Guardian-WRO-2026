# Engineering Log — Artefact Guardian

**Project Name:** Artefact Guardian  
**Competition:** WRO Future Innovators 2026  
**Team Members:** Shaurya Bahety, Raj, Vivaan  
**Repository:** Artefact-Guardian-WRO-2026  

---

## 1. Project Overview & What We're Building

* **The Problem:** Cultural heritage sites and museums handle priceless, fragile artefacts. Using bare hands risks transferring oils, dirt, or accidental drops, while big industrial arms are way too stiff and aggressive for delicate handling.
* **Our Solution:** **Artefact Guardian** — a gesture-controlled robotic hand and arm. It uses a camera for visual tracking, smooth motor feedback to stay steady, and calibrated finger movement for safe object handling.
* **Core Goal:** Mirror a human operator's hand movements in real-time so we can move delicate objects safely without ever touching them directly.

---

## 2. Step-by-Step Development Log

### Phase 1: Brainstorming & Architecture
* **Picking our input style:** We weighed up flex-glove sensors vs. Leap Motion vs. cameras. We went with camera-based computer vision (OpenCV + MediaPipe) so the operator doesn't need to wear bulky gloves or hardware.
* **Choosing microcontrollers:** Settled on an **ESP32** for the main logic because it's fast, dual-core, and handles UART communication smoothly. Paired it with an **Arduino UNO** to handle signal routing to the servos.

### Phase 2: CAD Design & 3D Printing Iterations
* **Base Assembly:**
  * *Iteration 1:* Designed the initial base housing, but realized the vertical clearance/height was too cramped to mount and align the top-to-bottom motor setup.
  * *Iteration 2:* Rescaled and extended the base housing to accommodate the vertical motor profile comfortably.
* **ENSW Rotation & Joint Module:**
  * Suffered several print failures due to strict servo slot tolerances.
  * Iterated dimension measurements multiple times until the DS3218 servo seated flush without stress on the 3D walls.
* **Front & Back Movement Joint:**
  * Went through 2–3 complete redesigns. Early mechanical designs struggled with weight distribution and didn't print cleanly.
  * *Material Switch:* Initial test prints in ABS failed or warped terribly. Switched away from ABS to PLA/TPU for better bed adhesion and structural consistency.
* **Claws & Grippers:**
  * *Print #1:* The initial claw design turned out far too small to grip actual sample museum artefacts safely.
  * *Print #2:* Upscaled the claw geometry and printed flexible TPU gripping surfaces.
* **Actuation Mechanism:**
  * Replaced slipping cables with an **inextensible nylon thread**. When the motor rotates, it winds/pulls the nylon thread to actuate the claw mechanisms with zero elastic stretch.

### Phase 3: Electronics & Power Management
* **Power Struggles:** Early on, powering the servos directly off the main microcontroller board kept tripping brownout resets on the ESP32 whenever the motors spiked.
* **Dual-Power Fix:** Split the wiring into two power rails — a heavy 5V/6V DC line specifically for the motor bus, and an isolated supply line for the ESP32 and MPU6050. Tied everything back to a shared ground.
* **Wiring Setup:** Neatened up our harness, generated our `Wiring diagram.jpg`, and put together the final component list in `BOM.xlsx`.

### Phase 4: Computer Vision & Gesture Control
* **Hand Tracking:** Built a Python pipeline with **OpenCV** and **MediaPipe Hands** to lock onto 21 key hand landmarks at smooth frame rates.
* **Math & Angles:** Wrote geometric functions that convert relative distance between hand landmarks into actual servo angles.
* **Talking over Serial:** Created a fast USB-Serial protocol to send target angles from Python directly to the ESP32 without lag or lost data packets.

### Phase 5: Calibration & Tuning
* **Servo Range Safety:** Charted upper and lower pulse-width limits for all DS3218 motors (in `Servo_Calibration.md`) to keep them from grinding gears or binding against 3D-printed stops.
* **Taming Sensor Noise:** Filtered out raw IMU noise from the MPU6050 so motor vibrations wouldn't mess up our orientation readings.

---

## 3. Roadblocks & How We Solved Them

| # | What Went Wrong | Why It Happened | How We Fixed It |
|---|---|---|---|
| **1** | Servo Jitter & ESP32 Resetting | High current draw from DS3218 motors pulled down voltage on the main board. | Isolated the motor power rail from logic, threw in decoupling capacitors, and double-checked common grounds. |
| **2** | Base Motor Clearance Issue | Original base CAD height was too short to fit the top-to-bottom motor setup. | Extended the base geometry vertically in Fusion 360 and reprinted. |
| **3** | Material Failures & Slot Misalignments | ABS warped heavily on mechanical prints, and ENSW servo slots were slightly off. | Ditched ABS due to print instability, tuned printer dimensions over multiple test runs, and printed grippers in flexible TPU. |
| **4** | Claw Sizing & Tendon Slipping | First claws were too small for artefacts, and elastic/smooth cables slipped. | Upscaled claw CAD design and adopted an inextensible nylon thread drive pulled directly by motor rotation. |
| **5** | Video Lag & Frame Drops | Processing high-res video frames slowed down hand tracking. | Scaled down camera resolution slightly and tweaked MediaPipe's confidence threshold settings. |

---

## 4. Team Roles & Who Built What

* **Shaurya Bahety:** Computer Vision scripts (Python/OpenCV/MediaPipe), ESP32 code, serial comms, and tying the hardware and software together.
* **Raj:** CAD modeling in Fusion 360, 3D printing tweaks, mechanical structure, and physical assembly.
* **Vivaan:** Power layout, wiring harness, hardware bench testing, and keeping track of components/BOM.

---

