"""
===================================================================
ARTEFACT GUARDIAN - DUAL-HAND VISION CONTROLLER (Python)
===================================================================
Team: JNS Future Minds (WRO 2026 Future Innovators)

DUAL-HAND SCHEME:
  LEFT HAND  -> joint selector + discrete commands
    0 fingers (fist)  -> STOP
    1 finger          -> select BASE
    2 fingers         -> select ELBOW
    3 fingers         -> select SHOULDER
    4 fingers         -> select WRIST TILT
    5 fingers         -> select GRIP (claw)

  RIGHT HAND -> angle steerer
    Roll  (twist)  -> steers BASE, ELBOW, SHOULDER
    Pitch (tilt)   -> steers WRIST TILT
    Pinch (thumb-index distance) -> steers GRIP
    Open palm required for roll/pitch joints (not for GRIP)
    Press 'n' with open right palm to calibrate neutral pose

JOINT SWITCH SETTLE:
    After changing the selected joint, steering pauses briefly so the
    new joint does not inherit the previous palm orientation.

HAND-LOSS SAFETY:
    Right hand lost  -> stop sending angles; arm holds last position
    Left hand lost   -> clear joint selection after 1 second
    Both hands lost  -> auto STOP after 2 seconds

USAGE:
    python artefact_vision_dual_hand.py --demo
    python artefact_vision_dual_hand.py --port COM12

Keys: 'n' = calibrate neutral   'q' / ESC = quit

===================================================================
[FIRMWARE COMPAT] This file has been updated to talk to the new
modular Artefact Guardian ESP32 firmware (HOME / ESTOP / MOVE / GETPOS /
VERSION / INFO / STATUS:* / DONE / OK:* / ERR:*) while remaining fully
compatible with the older single-file firmware that only understood
PING / STOP / ROTATE_TO / SHOULDER_TO / ELBOW_TO / TILT_TO / WRIST_TO /
GRIP. All changes are confined to the ThreadedSerialComm class (the
communication layer) and are tagged "[FIRMWARE COMPAT]" below. No GUI,
vision, calibration, or control-logic code was modified.
===================================================================
"""

import argparse
import math
import queue
import threading
import time
from collections import deque

import cv2
import mediapipe as mp
import numpy as np
import serial

# ==================== CONSTANTS ====================
SELECTOR_HAND = "Left"
STEERER_HAND = "Right"

GESTURE_HOLD = 6
SELECT_HOLD_FRAMES = 5
OPEN_PALM_HOLD_FRAMES = 4
LEFT_LOST_CLEAR_JOINT_S = 1.0
BOTH_LOST_STOP_S = 2.0
JOINT_SWITCH_SETTLE_S = 0.85

FINGER_TIPS = {"index": 8, "middle": 12, "ring": 16, "pinky": 20}
FINGER_BASES = {"index": 6, "middle": 10, "ring": 14, "pinky": 18}

JOINT_BY_COUNT = {
    1: "BASE",
    2: "ELBOW",
    3: "SHOULDER",
    4: "WRIST_TILT",
    5: "GRIP",
}

JOINT_COMMAND_PREFIX = {
    "BASE": "ROTATE_TO",
    "ELBOW": "ELBOW_TO",
    "SHOULDER": "SHOULDER_TO",
    "WRIST_TILT": "TILT_TO",
    "GRIP": "GRIP",
}

# [FIRMWARE COMPAT] Fixed joint order expected by the firmware's
# "MOVE:a,b,c,d,e,f" packet (BASE, SHOULDER, ELBOW, WRIST_TILT,
# WRIST_ROTATE, GRIPPER). This app has no WRIST_ROTATE control surface
# today, so that slot is held at SERVO_CENTER whenever a MOVE packet is
# assembled (see ThreadedSerialComm.send_move_all).
MOVE_PACKET_JOINT_ORDER = ["BASE", "SHOULDER", "ELBOW", "WRIST_TILT", "WRIST_ROTATE", "GRIP"]

JOINT_STEER_AXIS = {
    "BASE": "roll",
    "ELBOW": "roll",
    "SHOULDER": "roll",
    "WRIST_TILT": "pitch",
    "GRIP": "pinch",
}

JOINT_LABELS = {
    "BASE": "Base",
    "ELBOW": "Elbow",
    "SHOULDER": "Shoulder",
    "WRIST_TILT": "Wrist Tilt",
    "GRIP": "Grip / Claw",
}

# [FIRMWARE COMPAT] Extended with the new commands the firmware accepts.
# Only STOP was referenced anywhere previously; the rest are provided so
# the communication layer has named, self-documenting constants instead
# of magic strings scattered around.
CMD_MAP = {
    "STOP": "STOP",
    "ESTOP": "ESTOP",
    "HOME": "HOME",
    "GETPOS": "GETPOS",
    "VERSION": "VERSION",
    "INFO": "INFO",
}

# Steering tuning
ROTATE_SMOOTHING_ALPHA = 0.3
ROLL_TO_ANGLE_SCALE = 1.5
PITCH_TO_ANGLE_SCALE = 1.8
SERVO_CENTER = 90
MIN_SEND_INTERVAL_S = 0.1
MIN_ANGLE_DELTA = 2

# Pinch -> gripper angle (small distance = closed, large = open)
GRIP_ANGLE_OPEN = 30
GRIP_ANGLE_CLOSED = 150
PINCH_DIST_MIN = 0.035
PINCH_DIST_MAX = 0.17

# [FIRMWARE COMPAT] Serial-layer safety/robustness tuning.
HEARTBEAT_INTERVAL_S = 1.0          # unchanged cadence from original code
RECONNECT_DELAY_S = 2.0             # wait between reconnect attempts
MIN_WRITE_INTERVAL_S = 0.02         # hard floor between any two writes (anti-flood)
HOME_RESEND_MIN_INTERVAL_S = 2.5    # don't spam HOME while waiting for READY/IDLE
READY_STATUSES = {"READY", "IDLE"}  # statuses that mean "safe to send motion"


# ==================== ARGUMENTS ====================
def parse_args():
    parser = argparse.ArgumentParser(description="Artefact Guardian Dual-Hand Vision Controller")
    parser.add_argument("--port", type=str, default="COM13", help="MCU COM port (e.g. COM12)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--camera", type=int, default=0, help="Webcam index")
    parser.add_argument("--demo", action="store_true", help="Run without hardware connected")
    parser.add_argument("--cooldown", type=float, default=0.3, help="Min seconds between identical repeated sends")
    parser.add_argument(
        "--settle-delay",
        type=float,
        default=JOINT_SWITCH_SETTLE_S,
        help="Seconds to wait after switching joints before steering resumes",
    )
    return parser.parse_args()


# ==================== SERIAL ====================
class ThreadedSerialComm:
    """
    [FIRMWARE COMPAT] Updated communication layer.

    Public interface used by the rest of the app is UNCHANGED:
        start(), send_command(cmd), close(), .connected

    Everything added for firmware compatibility is additive:
        - Background parsing of STATUS:*, DONE, OK:*, ERR:* replies.
        - Auto-HOME-once on ERR:NOT_HOMED, with motion commands held
          back until STATUS:READY / STATUS:IDLE is seen.
        - Capability detection for HOME/MOVE so old firmware (which
          doesn't understand them) degrades gracefully instead of
          getting stuck waiting for a reply that will never come.
        - send_move_all() helper that sends one MOVE packet instead of
          six individual commands, with automatic fallback.
        - Non-blocking read loop, auto-reconnect, and basic flood
          control, all on the existing background thread so the GUI
          thread is never touched or blocked.
    """

    def __init__(self, port, baud, cooldown):
        self.port = port
        self.baud = baud
        self.cooldown = cooldown
        self.queue = queue.Queue(maxsize=5)
        self.ser = None
        self.connected = False
        self.running = False
        self.thread = None
        self.last_command = None
        self.last_command_time = 0

        # [FIRMWARE COMPAT] Shared state written by the worker thread and
        # read by the GUI thread. Booleans/strings/floats are effectively
        # atomic under the GIL for this simple read/write pattern, so a
        # lock is used only around the small multi-field updates below.
        self._state_lock = threading.Lock()
        self.last_status = None          # e.g. "READY", "IDLE", "MOVING", ...
        self.last_done_time = 0.0        # timestamp of last DONE line seen
        self.last_error = None           # e.g. "NOT_HOMED", "ANGLE", ...
        self.last_version_info = None    # raw text from VERSION reply
        self.last_info = None            # raw text from INFO reply
        self.last_position = None        # raw text from GETPOS reply
        self.supports_home = None        # None=unknown, True/False once learned
        self.supports_move = None        # None=unknown, True/False once learned

        self._home_pending = False       # a HOME has been sent, awaiting ack/ready
        self._home_sent_at = 0.0
        self._awaiting_ready = False      # gate: hold motion cmds until READY/IDLE
        self._last_written_kind = None    # correlates OK:/ERR: replies to what we sent
        self._last_write_time = 0.0

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------
    # [FIRMWARE COMPAT] Connection is now a retry loop instead of a single
    # attempt, so a cable hiccup or the MCU resetting mid-match doesn't
    # permanently disconnect the controller.
    # ------------------------------------------------------------------
    def _serial_worker(self):
        while self.running:
            if not self._connect_once():
                # Could not open the port at all - wait and retry, unless
                # we've been asked to stop in the meantime.
                for _ in range(int(RECONNECT_DELAY_S * 10)):
                    if not self.running:
                        return
                    time.sleep(0.1)
                continue

            self._run_connected_loop()

            # _run_connected_loop only returns when the link drops (or on
            # shutdown). Clean up and, if still running, loop back to
            # reconnect automatically.
            self._teardown_serial()
            if not self.running:
                return
            print("[SERIAL] Disconnected - will attempt to reconnect...")

    def _connect_once(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            print("[SERIAL] Port opened. Holding 3s for the board to finish booting...")
            time.sleep(3.0)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.connected = True
            print(f"[SERIAL] Connected and ready on {self.port}")
            return True
        except Exception as e:
            print(f"[SERIAL] Connection failed: {e}")
            self.connected = False
            self.ser = None
            return False

    def _teardown_serial(self):
        self.connected = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _run_connected_loop(self):
        last_heartbeat_sent = 0
        while self.running and self.connected:
            now = time.time()

            # ---- 1. Drain and parse anything the MCU has sent back. ----
            try:
                self._read_incoming()
            except Exception as e:
                print(f"[SERIAL] Read failed: {e}")
                self.connected = False
                return

            # ---- 2. Send at most one queued outbound command. ----
            try:
                cmd = self.queue.get(timeout=0.05)
                self._dispatch_queued_command(cmd, now)
                self.queue.task_done()
            except queue.Empty:
                pass

            # ---- 3. Heartbeat, unchanged cadence from the original code. ----
            now = time.time()
            if now - last_heartbeat_sent >= HEARTBEAT_INTERVAL_S:
                if not self._write_line("PING"):
                    return
                last_heartbeat_sent = now

            # ---- 4. If we're waiting on homing, occasionally re-request it. ----
            # [FIRMWARE COMPAT] Prevents a permanently-stuck state if the
            # very first HOME was lost to a serial hiccup, while still
            # respecting "prevent duplicate HOME" (min resend interval).
            if self._awaiting_ready and self.supports_home is not False:
                if now - self._home_sent_at > HOME_RESEND_MIN_INTERVAL_S:
                    self._send_home_once(force=True)

    # ------------------------------------------------------------------
    # Outbound command handling
    # ------------------------------------------------------------------
    def _dispatch_queued_command(self, cmd, now):
        """
        [FIRMWARE COMPAT] Applies the NOT_HOMED gate and basic flood
        control to whatever the GUI thread queued via send_command().
        """
        # While waiting for the arm to finish homing, drop stale motion
        # commands rather than let them pile up - the operator's hand is
        # still moving live, so the newest command (sent again shortly
        # after) is what matters, not this now-outdated one.
        if self._awaiting_ready and self._is_motion_command(cmd):
            return

        if cmd == self.last_command and (now - self.last_command_time) < self.cooldown:
            return

        if not self._write_line(cmd, kind=self._classify_command(cmd)):
            return

        self.last_command = cmd
        self.last_command_time = now

    @staticmethod
    def _is_motion_command(cmd):
        if cmd in ("PING", "STOP", "ESTOP", "HOME", "GETPOS", "VERSION", "INFO"):
            return False
        return True  # ROTATE_TO:.. / SHOULDER_TO:.. / etc. / MOVE:..

    @staticmethod
    def _classify_command(cmd):
        if cmd.startswith("MOVE"):
            return "MOVE"
        if cmd == "HOME":
            return "HOME"
        return "OTHER"

    def _write_line(self, line, kind="OTHER"):
        """
        Low-level write with a hard anti-flood floor.
        Returns False if the write failed (caller treats as disconnect).
        """
        now = time.time()
        wait = MIN_WRITE_INTERVAL_S - (now - self._last_write_time)
        if wait > 0:
            time.sleep(wait)
        try:
            print(f"[TX -> MCU]: {line}")
            self.ser.write(f"{line}\n".encode("utf-8"))
            self.ser.flush()
            self._last_write_time = time.time()
            self._last_written_kind = kind
            return True
        except Exception as e:
            print(f"[SERIAL] Write failed: {e}")
            self.connected = False
            return False

    def _send_home_once(self, force=False):
        """
        [FIRMWARE COMPAT] Sends HOME, guarding against duplicates. Safe
        to call repeatedly; only actually writes when appropriate.
        """
        if self.supports_home is False:
            return
        now = time.time()
        if not force and self._home_pending:
            return
        if now - self._home_sent_at < HOME_RESEND_MIN_INTERVAL_S:
            return
        self._home_sent_at = now
        self._home_pending = True
        self._awaiting_ready = True
        self._write_line("HOME", kind="HOME")

    # ------------------------------------------------------------------
    # [FIRMWARE COMPAT] MOVE support with automatic fallback.
    # ------------------------------------------------------------------
    def send_move_all(self, joint_angles):
        """
        Send all six joints in a single MOVE:a,b,c,d,e,f packet if the
        connected firmware is known to support it; otherwise fall back
        to the original six-individual-command method transparently.

        joint_angles: dict with any subset of keys from
            {"BASE","SHOULDER","ELBOW","WRIST_TILT","WRIST_ROTATE","GRIP"}
        Missing joints are held at SERVO_CENTER for the purposes of the
        MOVE packet only (fallback path simply skips missing joints).
        """
        if self.supports_move is not False:
            values = []
            for joint in MOVE_PACKET_JOINT_ORDER:
                angle = joint_angles.get(joint, SERVO_CENTER)
                values.append(str(int(round(angle))))
            self.send_command("MOVE:" + ",".join(values))
            return

        # Fallback: original behaviour, one command per known joint.
        for joint, angle in joint_angles.items():
            prefix = JOINT_COMMAND_PREFIX.get(joint)
            if prefix:
                self.send_command(f"{prefix}:{int(round(angle))}")

    # ------------------------------------------------------------------
    # [FIRMWARE COMPAT] New named-command helpers (optional; not called
    # automatically anywhere that would change existing behaviour).
    # ------------------------------------------------------------------
    def request_home(self):
        self._send_home_once(force=True)

    def request_estop(self):
        self.send_command("ESTOP")

    def request_getpos(self):
        self.send_command("GETPOS")

    def request_version(self):
        self.send_command("VERSION")

    def request_info(self):
        self.send_command("INFO")

    # ------------------------------------------------------------------
    # Inbound parsing
    # ------------------------------------------------------------------
    def _read_incoming(self):
        """
        [FIRMWARE COMPAT] Drains and parses any complete lines currently
        buffered from the MCU. Never blocks: readline() respects the
        serial timeout(0.1s) set at open, and we only loop while
        in_waiting indicates more data is already available.
        """
        if self.ser is None:
            return
        while self.ser.in_waiting > 0:
            raw = self.ser.readline()
            if not raw:
                break
            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            if not line:
                continue
            self._handle_incoming_line(line)

    def _handle_incoming_line(self, line):
        """
        [FIRMWARE COMPAT] Central parser for STATUS:*, DONE, OK:*, ERR:*
        and the new VERSION/INFO/GETPOS replies. Unknown/malformed lines
        are silently ignored (robustness requirement) rather than raising.
        """
        try:
            if line.startswith("STATUS:"):
                status = line[len("STATUS:"):].strip()
                with self._state_lock:
                    self.last_status = status
                # Clear the homing gate once the arm reports it's usable.
                if status in READY_STATUSES:
                    self._awaiting_ready = False
                    self._home_pending = False
                if status == "UNHOMED":
                    self.supports_home = True  # firmware clearly implements homing
                return

            if line == "DONE":
                with self._state_lock:
                    self.last_done_time = time.time()
                return

            if line.startswith("OK:"):
                payload = line[len("OK:"):]
                if payload == "HOME":
                    self.supports_home = True
                    self._home_pending = True
                elif payload.startswith("MOVE"):
                    self.supports_move = True
                return

            if line.startswith("ERR:"):
                code = line[len("ERR:"):].strip()
                with self._state_lock:
                    self.last_error = code
                if code == "NOT_HOMED":
                    # Auto-home once, then hold motion commands until ready.
                    self._send_home_once()
                elif code in ("FORMAT", "COMMAND"):
                    # Correlate with whatever we last wrote to learn that
                    # this firmware build doesn't understand HOME/MOVE,
                    # so we stop waiting for a reply that won't come and
                    # fall back to the legacy behaviour automatically.
                    if self._last_written_kind == "HOME":
                        self.supports_home = False
                        self._awaiting_ready = False
                        self._home_pending = False
                    elif self._last_written_kind == "MOVE":
                        self.supports_move = False
                return

            if line.startswith("VERSION"):
                with self._state_lock:
                    self.last_version_info = line
                return

            if line.startswith("INFO"):
                with self._state_lock:
                    self.last_info = line
                return

            if line.startswith("POS") or line.startswith("GETPOS"):
                with self._state_lock:
                    self.last_position = line
                return

            # Anything else (e.g. old firmware's boot banner text) is
            # informational only - ignore it, per robustness requirement.
        except Exception as e:
            # A malformed/unexpected line must never take the worker
            # thread down.
            print(f"[SERIAL] Ignoring malformed line ({e!r}): {line!r}")

    # ------------------------------------------------------------------
    # Public read-only accessors for future use (GUI behaviour unchanged
    # today; these simply expose what's already being tracked).
    # ------------------------------------------------------------------
    def get_last_status(self):
        with self._state_lock:
            return self.last_status

    def get_last_done_time(self):
        with self._state_lock:
            return self.last_done_time

    def get_last_error(self):
        with self._state_lock:
            return self.last_error

    # ------------------------------------------------------------------
    # Unchanged public interface
    # ------------------------------------------------------------------
    def send_command(self, cmd):
        if not self.connected:
            return
        if self.queue.full():
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
        self.queue.put(cmd)

    def close(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass


# ==================== HAND HELPERS ====================
class HandAnalyzer:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.85,
            min_tracking_confidence=0.75,
        )

    def detect_both(self, frame_rgb):
        results = self.hands.process(frame_rgb)
        left_lm, right_lm = None, None
        if not results.multi_hand_landmarks:
            return None, None

        for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
            label = results.multi_handedness[i].classification[0].label
            landmarks = hand_landmarks.landmark
            if label == SELECTOR_HAND:
                left_lm = landmarks
            elif label == STEERER_HAND:
                right_lm = landmarks
        return left_lm, right_lm

    def draw_hand(self, frame, landmarks, color):
        h, w, _ = frame.shape
        points = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
        connections = [
            (0, 1), (1, 2), (2, 3), (3, 4),
            (0, 5), (5, 6), (6, 7), (7, 8),
            (0, 9), (9, 10), (10, 11), (11, 12),
            (0, 13), (13, 14), (14, 15), (15, 16),
            (0, 17), (17, 18), (18, 19), (19, 20),
            (5, 9), (9, 13), (13, 17),
        ]
        for i, j in connections:
            cv2.line(frame, points[i], points[j], color, 2)
        for p in points:
            cv2.circle(frame, p, 4, color, -1)


def get_finger_states(landmarks, handedness):
    thumb_tip, thumb_ip = landmarks[4], landmarks[3]
    thumb_up = thumb_tip.x < thumb_ip.x if handedness == "Right" else thumb_tip.x > thumb_ip.x
    return {
        "thumb": thumb_up,
        "index": landmarks[FINGER_TIPS["index"]].y < landmarks[FINGER_BASES["index"]].y,
        "middle": landmarks[FINGER_TIPS["middle"]].y < landmarks[FINGER_BASES["middle"]].y,
        "ring": landmarks[FINGER_TIPS["ring"]].y < landmarks[FINGER_BASES["ring"]].y,
        "pinky": landmarks[FINGER_TIPS["pinky"]].y < landmarks[FINGER_BASES["pinky"]].y,
    }


def finger_count(landmarks, handedness):
    return sum(get_finger_states(landmarks, handedness).values())


def is_open_palm(landmarks, handedness):
    return finger_count(landmarks, handedness) == 5


def palm_roll_angle(landmarks):
    index_mcp, pinky_mcp = landmarks[5], landmarks[17]
    dx = pinky_mcp.x - index_mcp.x
    dy = pinky_mcp.y - index_mcp.y
    return math.degrees(math.atan2(dy, dx))


def palm_pitch_angle(landmarks):
    wrist, middle_tip = landmarks[0], landmarks[12]
    dx = middle_tip.x - wrist.x
    dy = wrist.y - middle_tip.y
    return math.degrees(math.atan2(dy, max(abs(dx), 1e-4)))


def pinch_distance(landmarks):
    thumb_tip, index_tip = landmarks[4], landmarks[8]
    return math.sqrt((thumb_tip.x - index_tip.x) ** 2 + (thumb_tip.y - index_tip.y) ** 2)


def pinch_points(frame, landmarks):
    h, w, _ = frame.shape
    thumb = (int(landmarks[4].x * w), int(landmarks[4].y * h))
    index = (int(landmarks[8].x * w), int(landmarks[8].y * h))
    return thumb, index


# ==================== SAFETY / HAND-LOSS TRACKER ====================
class SafetyStateTracker:
    def __init__(self):
        self.last_left_seen = 0.0
        self.last_right_seen = 0.0
        self.last_any_seen = 0.0
        self.emergency_stop = False
        self.auto_stop_fired = False
        self.state = "READY"
        self.state_detail = "Show both hands to begin"

    def update(self, left_seen, right_seen, steering_active, selected_joint, calibrated, settling=False):
        now = time.time()
        if left_seen:
            self.last_left_seen = now
            self.last_any_seen = now
            self.auto_stop_fired = False
            if self.emergency_stop:
                self.emergency_stop = False
        if right_seen:
            self.last_right_seen = now
            self.last_any_seen = now
            self.auto_stop_fired = False
            if self.emergency_stop:
                self.emergency_stop = False

        left_lost = not left_seen and (now - self.last_left_seen) > LEFT_LOST_CLEAR_JOINT_S
        right_lost = not right_seen and self.last_right_seen > 0
        both_lost = (now - self.last_any_seen) > BOTH_LOST_STOP_S if self.last_any_seen > 0 else False

        if self.emergency_stop:
            self.state = "EMERGENCY_STOP"
            self.state_detail = "Movement halted — show hands to resume"
        elif both_lost:
            self.state = "EMERGENCY_STOP"
            self.state_detail = "Both hands lost — auto STOP engaged"
        elif settling and selected_joint:
            self.state = "SETTLING"
            self.state_detail = "Joint switched — hold steady..."
        elif steering_active and selected_joint:
            self.state = "STEERING_ACTIVE"
            axis = JOINT_STEER_AXIS[selected_joint]
            if axis == "pinch":
                self.state_detail = "Live control via thumb-index pinch"
            else:
                self.state_detail = f"Live control via palm {axis}"
        elif right_lost and selected_joint:
            self.state = "HOLDING_POSITION"
            self.state_detail = "Right hand lost — holding last safe angle"
        elif selected_joint:
            self.state = "SELECTING"
            axis = JOINT_STEER_AXIS[selected_joint]
            if axis == "pinch":
                self.state_detail = "Grip locked — pinch to open/close claw"
            else:
                self.state_detail = "Joint locked — open right palm to steer"
        elif left_seen:
            self.state = "SELECTING"
            self.state_detail = "Choose a joint with left-hand fingers"
        elif not calibrated:
            self.state = "READY"
            self.state_detail = "Press N with open right palm to calibrate"
        else:
            self.state = "READY"
            self.state_detail = "Waiting for operator hands"

        return left_lost, right_lost, both_lost and not self.auto_stop_fired

    def trigger_emergency_stop(self):
        self.emergency_stop = True
        self.auto_stop_fired = True

    def mark_auto_stop_sent(self):
        self.auto_stop_fired = True


# ==================== LEFT HAND: JOINT SELECTOR ====================
class JointSelectorEngine:
    def __init__(self):
        self.count_history = deque(maxlen=SELECT_HOLD_FRAMES)
        self.gesture_history = deque(maxlen=GESTURE_HOLD)
        self.selected_joint = None
        self.stable_gesture = "-"
        self.last_fired_discrete = "-"

    def _stable_count(self, count):
        self.count_history.append(count)
        if len(self.count_history) == SELECT_HOLD_FRAMES and len(set(self.count_history)) == 1:
            return self.count_history[0]
        return None

    def _classify_discrete(self, count):
        if count == 0:
            return "STOP"
        return "-"

    def update(self, landmarks):
        count = finger_count(landmarks, SELECTOR_HAND)
        stable_count = self._stable_count(count)

        if stable_count is not None:
            self.selected_joint = JOINT_BY_COUNT.get(stable_count)

        raw_gesture = self._classify_discrete(count)
        self.gesture_history.append(raw_gesture)
        if (
            len(self.gesture_history) == GESTURE_HOLD
            and len(set(self.gesture_history)) == 1
            and self.gesture_history[0] != "-"
        ):
            self.stable_gesture = self.gesture_history[0]

        discrete_to_fire = None
        if self.stable_gesture != "-" and self.stable_gesture != self.last_fired_discrete:
            discrete_to_fire = self.stable_gesture
            self.last_fired_discrete = self.stable_gesture

        return self.selected_joint, discrete_to_fire

    def clear_selection(self):
        self.selected_joint = None
        self.count_history.clear()

    def reset_discrete_latch(self):
        self.last_fired_discrete = "-"
        self.stable_gesture = "-"
        self.gesture_history.clear()


# ==================== RIGHT HAND: STEERING + CALIBRATION ====================
class PalmSteeringEngine:
    def __init__(self, settle_delay):
        self.settle_delay = settle_delay
        self.open_palm_history = deque(maxlen=OPEN_PALM_HOLD_FRAMES)
        self.steering_active = False
        self.settling = False
        self.settle_remaining = 0.0
        self.smoothed_roll_vec = None
        self.smoothed_pitch = None
        self.smoothed_pinch = None
        self.last_sent_angle = None
        self.last_send_time = 0
        self.current_target_angle = SERVO_CENTER
        self.current_axis = "roll"
        self.current_pinch_norm = 0.0
        self.engaged_joint = None
        self.joint_switch_at = 0.0
        self.neutral_roll = None
        self.neutral_pitch = None
        self.calibrated = False
        self.calibration_flash_until = 0.0

    @property
    def calibration_message(self):
        if time.time() < self.calibration_flash_until:
            return "Neutral pose saved"
        if not self.calibrated:
            return "Not calibrated — press N"
        return "Calibrated"

    def _smooth_angle_vec(self, angle_deg, state_vec):
        rad = math.radians(angle_deg)
        x, y = math.cos(rad), math.sin(rad)
        if state_vec is None:
            return (x, y)
        sx, sy = state_vec
        sx = (ROTATE_SMOOTHING_ALPHA * x) + ((1 - ROTATE_SMOOTHING_ALPHA) * sx)
        sy = (ROTATE_SMOOTHING_ALPHA * y) + ((1 - ROTATE_SMOOTHING_ALPHA) * sy)
        return (sx, sy)

    def _smooth_scalar(self, value, state):
        if state is None:
            return value
        return (ROTATE_SMOOTHING_ALPHA * value) + ((1 - ROTATE_SMOOTHING_ALPHA) * state)

    def _vec_to_deg(self, vec):
        return math.degrees(math.atan2(vec[1], vec[0]))

    def calibrate(self, landmarks):
        self.neutral_roll = palm_roll_angle(landmarks)
        self.neutral_pitch = palm_pitch_angle(landmarks)
        self.calibrated = True
        self.calibration_flash_until = time.time() + 2.5
        self.smoothed_roll_vec = None
        self.smoothed_pitch = None
        self.smoothed_pinch = None
        self.last_sent_angle = None
        print(f"[CALIBRATION] neutral roll={self.neutral_roll:.1f} pitch={self.neutral_pitch:.1f}")

    def _begin_joint_switch(self, selected_joint):
        if selected_joint == self.engaged_joint:
            return
        self.engaged_joint = selected_joint
        self.joint_switch_at = time.time()
        self.settling = True
        self.settle_remaining = self.settle_delay
        self.steering_active = False
        self.smoothed_roll_vec = None
        self.smoothed_pitch = None
        self.smoothed_pinch = None
        self.last_sent_angle = None
        self.last_send_time = 0
        self.open_palm_history.clear()
        print(f"[STEER] Joint switched to {selected_joint} — settling for {self.settle_delay:.1f}s")

    def _is_settling(self):
        if not self.settling:
            return False
        elapsed = time.time() - self.joint_switch_at
        self.settle_remaining = max(0.0, self.settle_delay - elapsed)
        if elapsed >= self.settle_delay:
            self.settling = False
            self.settle_remaining = 0.0
            return False
        return True

    def _roll_target(self, landmarks):
        roll = palm_roll_angle(landmarks)
        self.smoothed_roll_vec = self._smooth_angle_vec(roll, self.smoothed_roll_vec)
        smoothed_roll = self._vec_to_deg(self.smoothed_roll_vec)

        if self.calibrated:
            delta = smoothed_roll - self.neutral_roll
            while delta > 180:
                delta -= 360
            while delta < -180:
                delta += 360
            target = SERVO_CENTER + (delta * ROLL_TO_ANGLE_SCALE)
        else:
            target = SERVO_CENTER + (smoothed_roll * ROLL_TO_ANGLE_SCALE)
        return max(0, min(180, int(round(target))))

    def _pitch_target(self, landmarks):
        pitch = palm_pitch_angle(landmarks)
        self.smoothed_pitch = self._smooth_scalar(pitch, self.smoothed_pitch)

        if self.calibrated:
            delta = self.smoothed_pitch - self.neutral_pitch
            target = SERVO_CENTER + (delta * PITCH_TO_ANGLE_SCALE)
        else:
            target = SERVO_CENTER + (self.smoothed_pitch * PITCH_TO_ANGLE_SCALE)
        return max(0, min(180, int(round(target))))

    def _pinch_target(self, landmarks):
        dist = pinch_distance(landmarks)
        self.smoothed_pinch = self._smooth_scalar(dist, self.smoothed_pinch)
        t = (self.smoothed_pinch - PINCH_DIST_MIN) / (PINCH_DIST_MAX - PINCH_DIST_MIN)
        t = max(0.0, min(1.0, t))
        self.current_pinch_norm = t
        target = GRIP_ANGLE_OPEN + ((1.0 - t) * (GRIP_ANGLE_CLOSED - GRIP_ANGLE_OPEN))
        return max(GRIP_ANGLE_OPEN, min(GRIP_ANGLE_CLOSED, int(round(target))))

    def _ready_to_steer(self, landmarks, selected_joint):
        if selected_joint == "GRIP":
            return True
        self.open_palm_history.append(is_open_palm(landmarks, STEERER_HAND))
        if len(self.open_palm_history) == OPEN_PALM_HOLD_FRAMES and all(self.open_palm_history):
            return True
        if self.open_palm_history and not self.open_palm_history[-1]:
            return False
        return self.steering_active

    def update(self, landmarks, selected_joint):
        if selected_joint is None:
            self.steering_active = False
            self.settling = False
            return None

        self._begin_joint_switch(selected_joint)
        if self._is_settling():
            return None

        ready = self._ready_to_steer(landmarks, selected_joint)
        self.steering_active = ready

        if not ready:
            return None

        axis = JOINT_STEER_AXIS[selected_joint]
        self.current_axis = axis
        if axis == "pitch":
            target = self._pitch_target(landmarks)
        elif axis == "pinch":
            target = self._pinch_target(landmarks)
        else:
            target = self._roll_target(landmarks)
        self.current_target_angle = target

        now = time.time()
        big_enough_change = self.last_sent_angle is None or abs(target - self.last_sent_angle) >= MIN_ANGLE_DELTA
        enough_time_passed = (now - self.last_send_time) >= MIN_SEND_INTERVAL_S

        if big_enough_change and enough_time_passed:
            self.last_sent_angle = target
            self.last_send_time = now
            prefix = JOINT_COMMAND_PREFIX[selected_joint]
            return f"{prefix}:{target}"
        return None

    def pause_steering(self):
        self.steering_active = False
        self.open_palm_history.clear()

    def full_reset(self):
        self.pause_steering()
        self.engaged_joint = None
        self.settling = False
        self.settle_remaining = 0.0
        self.smoothed_roll_vec = None
        self.smoothed_pitch = None
        self.smoothed_pinch = None
        self.last_sent_angle = None


# ==================== HUD ====================
class LegacyHudRenderer:
    BG = (14, 16, 22)
    PANEL = (24, 28, 38)
    BORDER = (52, 58, 72)
    TEXT = (228, 232, 240)
    MUTED = (130, 138, 155)
    ACCENT = (0, 190, 255)
    GREEN = (70, 215, 115)
    ORANGE = (255, 175, 55)
    RED = (255, 70, 70)
    GRIP = (255, 205, 70)

    JOINT_ROWS = [
        (1, "BASE", "Roll"),
        (2, "ELBOW", "Roll"),
        (3, "SHOULDER", "Roll"),
        (4, "WRIST TILT", "Pitch"),
        (5, "GRIP", "Pinch"),
    ]

    @staticmethod
    def _panel(frame, x, y, w, h, alpha=0.82):
        overlay = frame.copy()
        cv2.rectangle(overlay, (x, y), (x + w, y + h), HudRenderer.PANEL, -1)
        cv2.rectangle(overlay, (x, y), (x + w, y + h), HudRenderer.BORDER, 1)
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    @staticmethod
    def _bar(frame, x, y, w, h, value01, fill, bg=(40, 44, 56)):
        value01 = max(0.0, min(1.0, value01))
        cv2.rectangle(frame, (x, y), (x + w, y + h), bg, -1)
        fill_w = int(w * value01)
        if fill_w > 0:
            cv2.rectangle(frame, (x, y), (x + fill_w, y + h), fill, -1)
        cv2.rectangle(frame, (x, y), (x + w, y + h), HudRenderer.BORDER, 1)

    @staticmethod
    def _status_dot(frame, x, y, active, color):
        dot = HudRenderer.GREEN if active else (70, 74, 86)
        cv2.circle(frame, (x, y), 5, dot if active else (70, 74, 86), -1)
        if active:
            cv2.circle(frame, (x, y), 5, color, 1)

    @staticmethod
    def draw(
        frame,
        *,
        left_seen,
        right_seen,
        selected_joint,
        steering_active,
        settling,
        settle_remaining,
        settle_total,
        target_angle,
        axis,
        pinch_norm,
        safety_state,
        safety_detail,
        calibrated,
        cal_message,
        mcu_connected,
        mcu_port,
        demo_mode,
        right_lm,
    ):
        h, w = frame.shape[:2]

        HudRenderer._panel(frame, 0, 0, w, 42, alpha=0.88)
        cv2.putText(frame, "ARTEFACT GUARDIAN", (16, 28), cv2.FONT_HERSHEY_DUPLEX, 0.72, HudRenderer.ACCENT, 1, cv2.LINE_AA)
        cv2.putText(frame, "Dual-Hand Vision Control", (250, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.48, HudRenderer.MUTED, 1, cv2.LINE_AA)

        panel_w, panel_h = 210, 210
        HudRenderer._panel(frame, 12, 52, panel_w, panel_h)
        cv2.putText(frame, "LEFT HAND", (24, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.46, HudRenderer.GREEN, 1, cv2.LINE_AA)
        HudRenderer._status_dot(frame, 130, 70, left_seen, HudRenderer.GREEN)
        cv2.putText(frame, "DETECTED" if left_seen else "NOT SEEN", (142, 74),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, HudRenderer.TEXT if left_seen else HudRenderer.MUTED, 1, cv2.LINE_AA)

        for i, (count, name, mode) in enumerate(HudRenderer.JOINT_ROWS):
            y = 96 + i * 28
            joint_key = JOINT_BY_COUNT.get(count)
            active = selected_joint == joint_key
            bg = (38, 48, 68) if active else (32, 36, 48)
            cv2.rectangle(frame, (22, y - 16), (210, y + 6), bg, -1)
            if active:
                cv2.rectangle(frame, (22, y - 16), (210, y + 6), HudRenderer.ACCENT, 1)
            finger_color = HudRenderer.GRIP if count == 5 else HudRenderer.ACCENT
            cv2.putText(frame, f"{count}", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, finger_color, 2, cv2.LINE_AA)
            cv2.putText(frame, name, (52, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, HudRenderer.TEXT, 1, cv2.LINE_AA)
            cv2.putText(frame, mode, (158, y), cv2.FONT_HERSHEY_SIMPLEX, 0.34, HudRenderer.MUTED, 1, cv2.LINE_AA)

        cv2.putText(frame, "0 = STOP", (24, 248), cv2.FONT_HERSHEY_SIMPLEX, 0.38, HudRenderer.RED, 1, cv2.LINE_AA)

        rp_x = w - 222
        HudRenderer._panel(frame, rp_x, 52, 210, 170)
        cv2.putText(frame, "RIGHT HAND", (rp_x + 12, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.46, HudRenderer.ACCENT, 1, cv2.LINE_AA)
        HudRenderer._status_dot(frame, rp_x + 118, 70, right_seen, HudRenderer.ACCENT)

        if selected_joint == "GRIP":
            steer_hint = "Pinch to control claw"
            steer_color = HudRenderer.GRIP
        elif selected_joint:
            steer_hint = "Open palm to steer"
            steer_color = HudRenderer.ACCENT
        else:
            steer_hint = "Waiting for joint"
            steer_color = HudRenderer.MUTED

        cv2.putText(frame, "DETECTED" if right_seen else "NOT SEEN", (rp_x + 130, 74),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, HudRenderer.TEXT if right_seen else HudRenderer.MUTED, 1, cv2.LINE_AA)
        cv2.putText(frame, steer_hint, (rp_x + 12, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.40, steer_color, 1, cv2.LINE_AA)

        if selected_joint:
            label = JOINT_LABELS.get(selected_joint, selected_joint)
            cv2.putText(frame, f"Active: {label}", (rp_x + 12, 126),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, HudRenderer.TEXT, 1, cv2.LINE_AA)
            cv2.putText(frame, f"Mode: {axis}", (rp_x + 12, 148),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, HudRenderer.MUTED, 1, cv2.LINE_AA)
            cv2.putText(frame, f"Target: {target_angle} deg", (rp_x + 12, 170),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, HudRenderer.ACCENT, 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, "No joint selected", (rp_x + 12, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, HudRenderer.MUTED, 1, cv2.LINE_AA)

        if selected_joint == "GRIP" and right_lm is not None:
            thumb, index = pinch_points(frame, right_lm)
            cv2.line(frame, thumb, index, HudRenderer.GRIP, 3)
            cv2.circle(frame, thumb, 7, HudRenderer.GRIP, -1)
            cv2.circle(frame, index, 7, HudRenderer.GRIP, -1)
            HudRenderer._bar(frame, rp_x + 12, 182, 186, 12, pinch_norm, HudRenderer.GRIP)
            cv2.putText(frame, "Pinch spread", (rp_x + 12, 206), cv2.FONT_HERSHEY_SIMPLEX, 0.34, HudRenderer.MUTED, 1, cv2.LINE_AA)

        HudRenderer._panel(frame, 0, h - 54, w, 54, alpha=0.88)
        if mcu_connected:
            mcu_text = f"MCU  {mcu_port}"
            mcu_color = HudRenderer.GREEN
        elif demo_mode:
            mcu_text = "MCU  DEMO MODE"
            mcu_color = HudRenderer.ORANGE
        else:
            mcu_text = "MCU  DISCONNECTED"
            mcu_color = HudRenderer.RED
        cv2.putText(frame, mcu_text, (16, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.46, mcu_color, 1, cv2.LINE_AA)

        cal_color = HudRenderer.GREEN if calibrated else HudRenderer.ORANGE
        cv2.putText(frame, cal_message, (170, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, cal_color, 1, cv2.LINE_AA)
        cv2.putText(frame, "N = calibrate   Q = quit", (w - 210, h - 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, HudRenderer.MUTED, 1, cv2.LINE_AA)

        state_colors = {
            "STEERING_ACTIVE": HudRenderer.GREEN,
            "SETTLING": HudRenderer.ORANGE,
            "HOLDING_POSITION": HudRenderer.ORANGE,
            "EMERGENCY_STOP": HudRenderer.RED,
            "SELECTING": HudRenderer.ACCENT,
            "READY": HudRenderer.MUTED,
        }
        badge_color = state_colors.get(safety_state, HudRenderer.MUTED)
        badge_w = 250
        HudRenderer._panel(frame, w - badge_w - 12, 52, badge_w, 58, alpha=0.88)
        cv2.putText(frame, "SAFETY", (w - badge_w + 4, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.40, HudRenderer.MUTED, 1, cv2.LINE_AA)
        cv2.putText(frame, safety_state.replace("_", " "), (w - badge_w + 4, 96),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, badge_color, 1, cv2.LINE_AA)

        if settling and settle_total > 0:
            progress = 1.0 - (settle_remaining / settle_total)
            overlay = frame.copy()
            cv2.rectangle(overlay, (w // 2 - 150, h // 2 - 36), (w // 2 + 150, h // 2 + 36), (18, 20, 28), -1)
            cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
            cv2.rectangle(frame, (w // 2 - 150, h // 2 - 36), (w // 2 + 150, h // 2 + 36), HudRenderer.ORANGE, 2)
            cv2.putText(frame, "Joint switched", (w // 2 - 108, h // 2 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, HudRenderer.TEXT, 1, cv2.LINE_AA)
            cv2.putText(frame, f"Hold steady... {settle_remaining:.1f}s", (w // 2 - 108, h // 2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, HudRenderer.ORANGE, 1, cv2.LINE_AA)
            HudRenderer._bar(frame, w // 2 - 120, h // 2 + 24, 240, 8, progress, HudRenderer.ORANGE)

        if steering_active and not settling:
            cv2.circle(frame, (w - 28, 28), 8, HudRenderer.GREEN, -1)


# ==================== MINIMAL FULL-SCREEN HUD ====================
class HudRenderer:
    """A deliberately quiet HUD: the camera and hand movement stay central."""

    PANEL = (22, 26, 34)
    BORDER = (58, 66, 82)
    TEXT = (236, 240, 246)
    MUTED = (150, 160, 174)
    ACCENT = (0, 194, 255)
    GREEN = (85, 225, 135)
    AMBER = (70, 185, 255)
    RED = (85, 85, 255)
    GOLD = (80, 210, 255)

    @staticmethod
    def _panel(frame, x, y, width, height, alpha=0.82):
        overlay = frame.copy()
        cv2.rectangle(overlay, (x, y), (x + width, y + height), HudRenderer.PANEL, -1)
        cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0, frame)
        cv2.rectangle(frame, (x, y), (x + width, y + height), HudRenderer.BORDER, 1)

    @staticmethod
    def _text(frame, text, point, scale, colour, thickness=1):
        cv2.putText(
            frame, text, point, cv2.FONT_HERSHEY_SIMPLEX, scale, colour,
            thickness, cv2.LINE_AA,
        )

    @staticmethod
    def draw(
        frame,
        *,
        left_seen,
        right_seen,
        selected_joint,
        steering_active,
        settling,
        settle_remaining,
        settle_total,
        target_angle,
        axis,
        pinch_norm,
        safety_state,
        safety_detail,
        calibrated,
        cal_message,
        mcu_connected,
        mcu_port,
        demo_mode,
        right_lm,
    ):
        h, w = frame.shape[:2]
        state_colours = {
            "STEERING_ACTIVE": HudRenderer.GREEN,
            "SETTLING": HudRenderer.AMBER,
            "HOLDING_POSITION": HudRenderer.AMBER,
            "EMERGENCY_STOP": HudRenderer.RED,
            "SELECTING": HudRenderer.ACCENT,
            "READY": HudRenderer.MUTED,
        }
        state_colour = state_colours.get(safety_state, HudRenderer.MUTED)

        # One small identity/status card replaces the old left and right panels.
        HudRenderer._panel(frame, 16, 16, 250, 60)
        HudRenderer._text(frame, "ARTEFACT GUARDIAN", (30, 40), 0.52, HudRenderer.GOLD)
        HudRenderer._text(frame, "DUAL-HAND CONTROL", (30, 61), 0.34, HudRenderer.MUTED)

        state_label = safety_state.replace("_", " ")
        state_width = 205
        HudRenderer._panel(frame, w - state_width - 16, 16, state_width, 60)
        cv2.circle(frame, (w - state_width + 4, 36), 6, state_colour, -1, cv2.LINE_AA)
        HudRenderer._text(frame, state_label, (w - state_width + 19, 41), 0.42, state_colour)
        if steering_active and not settling:
            HudRenderer._text(frame, "LIVE", (w - state_width + 19, 62), 0.34, HudRenderer.GREEN)
        elif selected_joint:
            HudRenderer._text(frame, "HOLD POSITION", (w - state_width + 19, 62), 0.31, HudRenderer.MUTED)

        if selected_joint:
            label = JOINT_LABELS.get(selected_joint, selected_joint)
            HudRenderer._panel(frame, 16, 88, 254, 58)
            HudRenderer._text(frame, label.upper(), (30, 113), 0.45, HudRenderer.TEXT)
            HudRenderer._text(frame, "ANGLE", (30, 133), 0.29, HudRenderer.MUTED)
            HudRenderer._text(
                frame,
                f"{target_angle:03d} DEG",
                (142, 128),
                0.46,
                HudRenderer.GREEN if steering_active else HudRenderer.ACCENT,
            )
            mode = axis.upper() if steering_active else "READY TO STEER"
            HudRenderer._text(frame, mode, (92, 142), 0.27, HudRenderer.MUTED)

        # The previous large centre modal is now a compact, non-blocking pill.
        if settling and settle_total > 0:
            pill_width, pill_height = 220, 42
            pill_x = (w - pill_width) // 2
            HudRenderer._panel(frame, pill_x, 16, pill_width, pill_height, alpha=0.90)
            seconds = max(0.0, settle_remaining)
            HudRenderer._text(frame, f"JOINT LOCKED | {seconds:.1f}s", (pill_x + 16, 40), 0.40, HudRenderer.AMBER)
            progress = 1.0 - (seconds / settle_total)
            cv2.rectangle(frame, (pill_x + 16, 47), (pill_x + pill_width - 16, 50), (54, 59, 72), -1)
            cv2.rectangle(
                frame,
                (pill_x + 16, 47),
                (pill_x + 16 + int((pill_width - 32) * max(0.0, min(1.0, progress))), 50),
                HudRenderer.AMBER,
                -1,
            )

        # Pinch feedback is local to the hand rather than another full panel.
        if selected_joint == "GRIP" and right_lm is not None:
            thumb, index = pinch_points(frame, right_lm)
            cv2.line(frame, thumb, index, HudRenderer.GOLD, 2, cv2.LINE_AA)
            cv2.circle(frame, thumb, 5, HudRenderer.GOLD, -1, cv2.LINE_AA)
            cv2.circle(frame, index, 5, HudRenderer.GOLD, -1, cv2.LINE_AA)

        # A thin footer holds only operating essentials.
        footer_h = 34
        HudRenderer._panel(frame, 0, h - footer_h, w, footer_h, alpha=0.88)
        link_text = f"MCU  {mcu_port}" if mcu_connected else ("DEMO MODE" if demo_mode else "MCU DISCONNECTED")
        link_colour = HudRenderer.GREEN if mcu_connected else (HudRenderer.AMBER if demo_mode else HudRenderer.RED)
        HudRenderer._text(frame, link_text, (16, h - 12), 0.36, link_colour)
        cal_colour = HudRenderer.GREEN if calibrated else HudRenderer.AMBER
        HudRenderer._text(frame, cal_message, (w // 2 - 72, h - 12), 0.34, cal_colour)
        HudRenderer._text(frame, "N  CALIBRATE   |   Q  EXIT", (w - 190, h - 12), 0.32, HudRenderer.MUTED)


# ==================== MAIN APPLICATION ====================
class ArtefactVisionDualHandApp:
    def __init__(self, args):
        self.args = args
        self.hand_analyzer = HandAnalyzer()
        self.selector_engine = JointSelectorEngine()
        self.steering_engine = PalmSteeringEngine(args.settle_delay)
        self.safety_tracker = SafetyStateTracker()
        self.serial_comm = None
        self.selected_joint = None
        self.discrete_gesture = "-"

        if not args.demo and args.port:
            self.serial_comm = ThreadedSerialComm(args.port, args.baud, args.cooldown)
            self.serial_comm.start()

        self.cap = cv2.VideoCapture(args.camera)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open webcam")

        # Request a widescreen image for a clean full-screen presentation.
        # Cameras that cannot supply 720p are centre-cropped below instead.
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.window_name = "Artefact Guardian - Dual Hand"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.running = True

    def _send_stop(self, reason):
        print(f"[SAFETY] STOP — {reason}")
        self.safety_tracker.trigger_emergency_stop()
        if self.serial_comm:
            self.serial_comm.send_command("STOP")
        self.selector_engine.clear_selection()
        self.steering_engine.full_reset()
        self.selected_joint = None

    def run(self):
        print("Artefact Guardian dual-hand controller running.")
        print("  Left  = joint selector (1-5)")
        print("  Right = palm roll/pitch OR pinch for grip")
        print("  N     = calibrate neutral pose")
        print("  Q/ESC = quit")

        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                break

            frame = cv2.flip(frame, 1)
            frame = self.crop_to_display_aspect(frame)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            left_lm, right_lm = self.hand_analyzer.detect_both(rgb)

            left_seen = left_lm is not None
            right_seen = right_lm is not None
            self.discrete_gesture = "-"

            if left_seen:
                self.hand_analyzer.draw_hand(frame, left_lm, (70, 215, 115))
                joint, discrete = self.selector_engine.update(left_lm)
                if joint is not None:
                    self.selected_joint = joint
                if discrete:
                    self.discrete_gesture = discrete
                    if discrete == "STOP":
                        self._send_stop("operator fist gesture")

            if right_seen:
                if self.selected_joint == "GRIP":
                    color = (255, 205, 70)
                elif self.selected_joint:
                    color = (0, 190, 255)
                else:
                    color = (80, 120, 200)
                self.hand_analyzer.draw_hand(frame, right_lm, color)

            left_lost, right_lost, should_auto_stop = self.safety_tracker.update(
                left_seen,
                right_seen,
                self.steering_engine.steering_active,
                self.selected_joint,
                self.steering_engine.calibrated,
                settling=self.steering_engine.settling,
            )

            if left_lost and self.selected_joint is not None:
                self.selector_engine.clear_selection()
                self.selected_joint = None
                self.steering_engine.full_reset()
                print("[SAFETY] Left hand lost — joint selection cleared")

            if right_seen and self.selected_joint and not self.safety_tracker.emergency_stop:
                cmd = self.steering_engine.update(right_lm, self.selected_joint)
                if cmd and self.serial_comm:
                    self.serial_comm.send_command(cmd)
            elif right_lost and self.selected_joint:
                self.steering_engine.pause_steering()
            elif not right_seen:
                self.steering_engine.pause_steering()

            if should_auto_stop:
                self._send_stop("both hands lost for 2+ seconds")
                self.safety_tracker.mark_auto_stop_sent()

            if not left_seen:
                self.selector_engine.reset_discrete_latch()

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                self.running = False
            elif key == ord("n") and right_seen and is_open_palm(right_lm, STEERER_HAND):
                self.steering_engine.calibrate(right_lm)

            HudRenderer.draw(
                frame,
                left_seen=left_seen,
                right_seen=right_seen,
                selected_joint=self.selected_joint,
                steering_active=self.steering_engine.steering_active,
                settling=self.steering_engine.settling,
                settle_remaining=self.steering_engine.settle_remaining,
                settle_total=self.args.settle_delay,
                target_angle=self.steering_engine.current_target_angle,
                axis=self.steering_engine.current_axis,
                pinch_norm=self.steering_engine.current_pinch_norm,
                safety_state=self.safety_tracker.state,
                safety_detail=self.safety_tracker.state_detail,
                calibrated=self.steering_engine.calibrated,
                cal_message=self.steering_engine.calibration_message,
                mcu_connected=bool(self.serial_comm and self.serial_comm.connected),
                mcu_port=self.args.port,
                demo_mode=self.args.demo,
                right_lm=right_lm,
            )
            self.show_fullscreen(frame)

        self.cleanup()

    def display_size(self):
        rect = cv2.getWindowImageRect(self.window_name)
        if rect and rect[2] > 0 and rect[3] > 0:
            return rect[2], rect[3]
        return 16, 9

    def crop_to_display_aspect(self, frame):
        """Centre-crop before drawing the HUD, preserving a true full screen."""
        screen_w, screen_h = self.display_size()
        target_aspect = screen_w / screen_h
        source_h, source_w = frame.shape[:2]
        source_aspect = source_w / source_h

        if source_aspect > target_aspect:
            crop_w = int(source_h * target_aspect)
            x = (source_w - crop_w) // 2
            return frame[:, x:x + crop_w].copy()

        crop_h = int(source_w / target_aspect)
        y = (source_h - crop_h) // 2
        return frame[y:y + crop_h, :].copy()

    def show_fullscreen(self, frame):
        screen_w, screen_h = self.display_size()
        full_frame = cv2.resize(
            frame, (screen_w, screen_h), interpolation=cv2.INTER_LINEAR
        )
        cv2.imshow(self.window_name, full_frame)

    def cleanup(self):
        self.cap.release()
        if self.serial_comm:
            self.serial_comm.close()
        cv2.destroyAllWindows()


def main():
    args = parse_args()
    app = ArtefactVisionDualHandApp(args)
    app.run()


if __name__ == "__main__":
    main()