"""
TEAM PLASMA - Robomasters "Reboot the Vault" - UGOT Engineer (mecanum + arm + clamp)

Run (laptop on the same Wi-Fi as the robot, VPN off):
    pip install ugot
    python plasma_vault.py --ip 10.10.53.XX --mission m1      # one mission
    python plasma_vault.py --ip 10.10.53.XX --mission all     # every mission in order
    python plasma_vault.py --ip 10.10.53.XX --mission final   # final extraction run
    python plasma_vault.py --mock --mission all               # test on laptop, no robot

Missions
    m1  Emergency Activation     connect + say "PLASMA" aloud
    m2  Navigation Calibration   follow black tape -> stop 15-25 cm from wall (distance sensor)
    m3  Communications Recovery  turn toward voice/clap -> report battery + tilt
    m4  Decode Energy Signature  find team marker, scan QR, decode evidence (saved to evidence.json)
    m5  Secure the Core          pick red/blue/green core -> drop in containment chest
    m6  Restart the Vault        pick core -> carry to restart station -> place in power dock

Tuning / test tools
    --mission sensors   live print of distance, battery, IMU, line offset, colours, QR, sound
    --mission linetest  prints line offset so you can check LINE_SIGN (slide robot left/right of tape)
    --mission arm       cycles all arm poses so you can tune ARM_* numbers
Press Ctrl+C at any time = emergency stop (wheels stop, clamp keeps holding).
"""

import argparse
import base64
import binascii
import codecs
import functools
import json
import math
import os
import re
import signal
import sys
import threading
import time
import urllib.parse

# =====================================================================================
# CONFIG - everything you may need to tune at the arena is here
# =====================================================================================
CFG = {
    "TEAM_NAME": "PLASMA",
    "VOICE": 0,                       # 0 female, 1 male

    # ---- distance sensor ----
    "DIST_ID": None,                  # None = auto-detect. Set the ID shown in the UGOT app if auto fails.

    # ---- speeds ----
    "LINE_SPEED": 14,                 # cm/s while following tape (5-80). Lower = safer on sharp corners.
    "LINE_SLOW_SPEED": 7,             # cm/s on sharp corners
    "SEARCH_TURN_SPEED": 45,          # deg/s while scanning for things
    "APPROACH_SPEED": 15,             # cm/s when driving to objects

    # ---- tape following (camera "line_recognition", single track) ----
    "LINE_TYPE": 0,                   # 0 single line (black tape), 1 double track
    "LINE_SIGN": -1,                  # measured on UGOT_B2E8 (re-checked automatically when LINE_SIGN_AUTO)
    "LINE_SIGN_AUTO": True,           # wiggle 8 deg at the start of the route to measure the sign (~2 s)
    # ---- PREDICTIVE camera tape follower (tape_eye.py): black tape only, brown decoys rejected ----
    "TAPE_SOURCE": "camera",          # "camera" = our black-tape vision (recommended), "sdk" = UGOT line model
    "CAM_SPEED": 12,                  # cm/s on straight tape (camera ~6-10 fps -> ~1.5 cm per frame)
    "CAM_K_AHEAD": 40,                # deg/s per unit of FAR offset (prediction: steer to where the tape goes)
    "CAM_K_LAT": 25,                  # deg/s per unit of NEAR offset
    "CAM_K_STRAFE": 10,               # cm/s sideways per unit of NEAR offset (micro-shift)
    "CAM_DEADBAND": 0.06,             # |offset| below this (normalised) = on the tape
    "CAM_SHARP": 0.55,                # |far offset| above this = sharp corner -> rotate in place
    "CAM_CORNER_TURN": 25,            # deg/s rotation in a sharp corner
    "SEARCH_360_SPEED": 30,           # deg/s during the full-360 lost-tape search
    "TAPE_CONFIRM_FRAMES": 3,         # consecutive frames of black tape before trusting it
    "BLUE_WALL_STOP_CM": 22,          # camera: stop when the blue wall meets the floor this close (band 15-25)
    "BLUE_WALL_SLOW_CM": 45,          # camera: creep at 6 cm/s from here
    "STALL_MOTION": 1.5,              # floor-patch pixel change/frame: still ~0.4-1.2, driving ~2-5 (measured)
    "STALL_FRAMES": 9,                # ~1 s of frozen picture -> we are pushing a wall
    "BUMP_BACKOFF_CM": 20,            # then back off this far (lands in the 15-25 cm band)
    # micro-correction steering for the SDK line model (TAPE_SOURCE="sdk")
    "LINE_DEADBAND": 18,              # |offset| below this = on the tape, no correction
    "LINE_KP": 0.18,                  # deg/s of turn per unit offset (small)
    "LINE_KS": 0.05,                  # cm/s sideways strafe per unit offset (mecanum micro-shift)
    "LINE_MAX_STRAFE": 5,             # cm/s
    "LINE_KP_CORNER": 0.35,           # firmer turn only when |offset| > LINE_SHARP_OFFSET
    "LINE_MAX_TURN": 45,              # deg/s cap - no fast wiggling
    "LINE_SHARP_OFFSET": 110,         # |offset| above this -> corner mode (slow + turn)
    "LOST_SWEEP_SPEED": 30,           # deg/s slow search rotation when the tape is lost
    "LOST_SWEEP_S": 3.0,              # seconds of search before giving up
    "LINE_LOST_STOP_S": 1.2,          # tape lost this long -> end of route
    "LINE_LOST_SEARCH": True,         # when tape lost, sweep left/right to re-find it before giving up
    # what to do at each fork/crossing, in order (the photo track has one Y-fork). "straight"/"left"/"right"
    "JUNCTION_PLAN": ["straight", "straight", "straight", "straight"],
    "JUNCTION_BIAS": 70,              # deg/s extra turn while taking a left/right branch
    "JUNCTION_BIAS_S": 0.9,
    "JUNCTION_DEBOUNCE_S": 2.0,
    "ROUTE_MAX_S": 120,               # safety timeout for the tape route

    # ---- calibration bay / wall stop ----
    "WALL_SWITCH_CM": 45,             # distance sensor below this while following tape -> switch to wall approach
    "WALL_CONFIRM": 3,                # consecutive close readings needed (ignores people/desks passing by)
    "FLOOR_WALL_CM": 19,              # IR tilted at the floor reads ~22-35 cm; a wall = reading below this
    "WALL_MIN_ROUTE_S": 0,            # ignore walls for the first N seconds of the route (raise if it stops early)
    "NO_TAPE_GIVEUP_S": 6,            # never saw tape for this long -> stop instead of driving blind
    "WALL_TARGET_CM": 20,             # aim for the middle of 15-25 cm
    "WALL_TOL_CM": 2.5,

    # ---- sound ----
    "SOUND_TIMEOUT_S": 30,
    "SOUND_FRONT_ANGLE": 90,          # SDK: 90 = front, 180 = left, 270 = behind, 0 = right

    # ---- QR marker ----
    "QR_MUST_CONTAIN": None,          # e.g. "PLASMA" if every team has a marker; None = take any QR
    "QR_APPROACH_STOP_CM": 30,
    "QR_READS": 3,                    # identical reads needed per QR
    "QR_MAX": 3,                      # read up to this many different QR markers
    "QR_MIN_AGREE": 2,                # stop early once this many fragments back the same colour
    "QR_TIME_BUDGET_S": 90,           # give up looking for more markers after this
    "QR_TOO_CLOSE_AREA": 120000,      # located QR bigger than this (px^2 of 640x480) = too close to decode

    # ---- camera ----
    "CAM_W": 640,
    "CAM_H": 480,
    "CENTER_TOL_PX": 40,

    # ---- core pickup: the cores are PING PONG BALLS (40 mm, light, they roll!) ----
    "CORE_COLOR": None,               # "red"/"blue"/"green" to force. None = use evidence.json (or --core)
    "CORE_SHAPE": "ball",             # only accept round blobs ("ball"); None = any shape
    "COLOR_SOURCE": "camera",         # "camera" = our HSV detector (finds cans/balls), "sdk" = UGOT colour model
    "CORE_MIN_AREA": 150,             # ignore colour specks smaller than this (px^2)
    "CORE_GRAB_CM": 9,                # distance-sensor reading at which the ball is between the jaws (if sensor sees it)
    "CORE_GRAB_AREA": 14000,          # blob area (px^2) when the ball is in reach - tune with --mission sensors
    "CORE_GRAB_Y": 380,               # OR blob centre y (px) below which it is in reach (camera looks down)
    "CORE_FINAL_PUSH_CM": 0,          # creep after opening clamp. Keep 0-2: pushing a ping pong ball rolls it away

    # ---- drop-off targets ----
    # type: "apriltag" {"id": n or None}, "qr" {"text": "..."}, "color" {"color": "yellow"},
    #       "moves" {"moves": [["fwd",cm], ["back",cm], ["left",deg], ["right",deg], ["strafe_l",cm], ["strafe_r",cm]]}
    #       "wall"  (drive straight until distance sensor hits stop_cm)
    "CHEST":      {"type": "apriltag", "id": None, "stop_cm": 12, "fallback": {"type": "wall", "stop_cm": 12}},
    "POWER_DOCK": {"type": "apriltag", "id": None, "stop_cm": 10, "fallback": {"type": "wall", "stop_cm": 10}},
    "TARGET_STOP_AREA": 40000,        # camera backup stop for tag/QR/colour targets (px^2) if distance sensor misses
    "CORE_SOURCE_M6": "auto",         # "auto": from the chest if m5 just put it there, else from the floor.
                                      # "chest" / "floor" to force

    # ---- arm poses (r = reach cm [-5.5,24.9], h = height cm [-18,18.2], theta rad [-1.57,1.57]) ----
    "ARM_LOOK":    [8.0, 10.0, 0.0],      # tucked up, camera view clear
    "ARM_PREGRAB": [17.0, -4.0, 0.0],     # above the core
    "ARM_GRAB":    [17.0, -11.0, 0.0],    # jaws around the core (floor level - tune!)
    "ARM_CARRY":   [11.0, 8.0, 0.0],      # holding core up while driving
    "ARM_CHEST":   [20.0, 6.0, 0.0],      # over the chest wall
    "ARM_CHEST_PREGRAB": [20.0, 10.0, 0.0],   # above a ball lying inside the chest (m6 retrieval)
    "ARM_CHEST_GRAB":    [20.0, -2.0, 0.0],   # jaws down into the chest - tune with --mission arm
    "ARM_DOCK":    [19.0, -6.0, 0.0],     # into the power dock
    "ARM_MOVE_MS": 900,

    "FINAL_SEQUENCE": ["m4", "m6"],   # final = decode the QR markers, then pick the core + place it in the power dock.
                                      # Change once the final briefing says what counts (e.g. ["m4","m5"] for the chest)
    "FINAL_TIME_LIMIT_S": 175,        # Rule 9: 3 min per attempt -> robot stops itself at 2:55
    "EVIDENCE_FILE": "evidence.json",
}

COLORS = ("red", "blue", "green")
DEFAULT_IP = "10.10.53.142"

# Catchphrases (all pre-generated with ElevenLabs + uploaded to the robot before the run starts)
PHRASES = {
    "boot":      ["Plasma online. Let's light it up."],
    "team":      ["We are team Plasma!"],
    "tracking":  ["Starting tracking.", "Tape locked. Let's ride."],
    "junction":  ["Junction detected. Holding course.", "Fork in the road. I've got this."],
    "corner":    ["Hard corner. Hold on."],
    "lost":      ["Lost the tape. Re-acquiring."],
    "wall":      ["Wall detected. Braking.", "Wall ahead. Easing in."],
    "listen":    ["Comms open. Make some noise."],
    "heard":     ["Signal acquired. Turning to you."],
    "scan":      ["Scanning the vault.", "Eyes on. Searching."],
    "target":    ["Target acquired.", "Locked on."],
    "decrypt":   ["Decrypting evidence. Stand by."],
    "gemma":     ["Running Gemma analysis."],
    "grab":      ["Grabbing the core.", "Engaging clamp."],
    "got":       ["Got it. Core secured."],
    "carry":     ["Core in transit. Nobody breathe."],
    "drop":      ["Core delivered."],
    "dock":      ["Inserting core. Moment of truth."],
    "fail":      ["That did not go to plan. Recalculating."],
    "done":      ["Mission complete. Plasma out."],
    "intro1":    ["Plasma online. Let's light it up."],
    "intro2":    ["Systems hot. Let's go."],
    "intro3":    ["Vault rescue unit reporting for duty."],
}
INTRO_QUIPS = ["intro1", "intro2", "intro3"]

# =====================================================================================
# Helpers
# =====================================================================================
MOCK = False
T0 = time.time()
# live state shown by the dashboard (plasma_dashboard.py)
STATE = {"log": [], "mission": None, "dist": None, "line": None, "colors": [], "qr": [], "tags": [],
         "sound_angle": None, "say": None, "core": None, "results": {}}


def log(msg):
    line = "[%6.1fs] %s" % (time.time() - T0, msg)
    STATE["log"] = (STATE["log"] + [line])[-300:]
    if msg.startswith("SAY: "):
        STATE["say"] = msg[5:]
    print(line, flush=True)


def sleep(s):
    if MOCK:
        time.sleep(min(s, 0.01))
    else:
        time.sleep(s)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Robot:
    """Thin, safe wrapper around ugot.UGOT for the Engineer form."""

    def __init__(self, got, voice=None, gemma=None):
        self.got = got
        self.voice = voice
        self.gemma = gemma
        self.dist_id = CFG["DIST_ID"]
        self.loaded = set()
        self.holding = False
        self._quip_i = {}

    # ---------------- voice / screen ----------------
    def _builtin_tts(self, text, wait=True):
        try:
            self.got.play_audio_tts(text, CFG["VOICE"], wait)
        except Exception as e:
            log("tts failed: %s" % e)

    def _screen(self, text):
        try:
            self.got.screen_clear()
            self.got.screen_print_text_newline(text[:60], 7)
        except Exception:
            pass

    def _speak(self, text, wait):
        self._screen(text)
        if self.voice:
            how = self.voice.say(text, wait, fallback_tts=self._builtin_tts)
            if wait:
                log("   (voice: %s)" % how)
        else:
            self._builtin_tts(text, wait)

    def say(self, text, wait=True):
        log("SAY: " + text)
        if wait:
            self._speak(text, True)
        else:   # never block a control loop on audio or screen gRPC calls
            threading.Thread(target=self._speak, args=(text, False), daemon=True).start()

    def quip(self, key, wait=False):
        """Catchphrase for an action (rotates through the list, pre-generated with ElevenLabs)."""
        lines = PHRASES.get(key)
        if not lines:
            return
        i = self._quip_i.get(key, 0)
        self._quip_i[key] = i + 1
        self.say(lines[i % len(lines)], wait)

    def lights(self, r, g, b, effect=0):
        try:
            self.got.show_light_rgb_effect(r, g, b, effect)
        except Exception:
            pass

    # ---------------- motion ----------------
    def drive(self, fwd=0.0, turn=0.0, strafe=0.0):
        """Continuous drive. fwd cm/s (+fwd), turn deg/s (+left), strafe cm/s (+right)."""
        fwd = clamp(fwd, -80, 80)
        strafe = clamp(strafe, -80, 80)
        turn = clamp(turn, -280, 280)
        speed = math.hypot(fwd, strafe)
        if speed < 1 and abs(turn) < 1:
            self.stop()
            return
        direction = int(round(math.degrees(math.atan2(strafe, fwd)))) if speed >= 1 else 0
        self.got.MODEL.mecanum_move_control(linear_speed=int(round(speed)), direction=direction,
                                            rotate_speed=int(round(turn)))

    def stop(self):
        try:
            self.got.mecanum_stop()
        except Exception:
            try:
                self.got.stop_chassis()
            except Exception:
                pass

    def _timed(self, fn, est_s):
        t = time.time()
        fn()
        left = est_s - (time.time() - t)
        if left > 0:
            sleep(left)
        self.stop()
        sleep(0.15)

    def move_cm(self, cm, speed=20):
        if abs(cm) < 0.5:
            return
        d = 0 if cm > 0 else 1
        cm = abs(cm)
        self._timed(lambda: self.got.mecanum_move_speed_times(d, speed, int(round(cm)), 1), cm / speed + 0.35)

    def strafe_cm(self, cm, speed=15):
        if abs(cm) < 0.5:
            return
        ang = 90 if cm > 0 else -90
        cm = abs(cm)
        self._timed(lambda: self.got.mecanum_translate_speed_times(ang, speed, int(round(cm)), 1), cm / speed + 0.35)

    def turn_deg(self, deg, speed=90):
        """+deg = left (counter-clockwise)."""
        if abs(deg) < 2:
            return
        d = 2 if deg > 0 else 3
        deg = abs(deg)
        while deg > 0.5:
            step = min(deg, 360)
            self._timed(lambda s=step: self.got.mecanum_turn_speed_times(d, speed, int(round(s)), 2),
                        step / speed + 0.35)
            deg -= step

    # ---------------- arm / clamp ----------------
    def arm(self, pose_name, ms=None):
        ms = ms or CFG["ARM_MOVE_MS"]
        r, h, th = CFG[pose_name]
        log("ARM -> %s r=%.1f h=%.1f th=%.2f" % (pose_name, r, h, th))
        self.got.mechanical_move_axis(float(r), float(h), float(th), int(ms))
        sleep(ms / 1000.0 + 0.2)

    def clamp_open(self):
        self.got.mechanical_clamp_release()
        sleep(0.7)

    def clamp_close(self):
        self.got.mechanical_clamp_close()
        sleep(0.9)

    # ---------------- sensors ----------------
    def find_distance_sensor(self):
        if self.dist_id is not None:
            return self.dist_id
        try:
            for dev in self.got.get_peripheral_devices_list():
                t = str(dev.get("type", "")).lower()
                if any(k in t for k in ("infrared", "distance", "ultrasonic", "tof", "laser")):
                    self.dist_id = int(dev["deviceId"])
                    log("distance sensor auto-detected: type=%s id=%s" % (t, self.dist_id))
                    return self.dist_id
        except Exception as e:
            log("device list failed: %s" % e)
        for i in list(range(21, 31)) + list(range(1, 21)) + list(range(31, 41)):
            try:
                v = self.got.read_distance_data(i)
                if v is not None and v != 0:      # -1 = sensor present but nothing in range, 0 = wrong id
                    self.dist_id = i
                    log("distance sensor found by probing: id=%d (%.1f cm)" % (i, v))
                    return i
            except Exception:
                pass
        log("!! no distance sensor found - set CFG['DIST_ID']")
        return None

    def dist(self, samples=3):
        """cm (median of samples), or None if no reading."""
        if self.dist_id is None:
            return None
        vals = []
        for _ in range(samples):
            try:
                v = self.got.read_distance_data(self.dist_id)
                if v is not None and v > 0:
                    vals.append(v)
            except Exception:
                pass
        if not vals:
            STATE["dist"] = None
            return None
        vals.sort()
        STATE["dist"] = vals[len(vals) // 2]
        return STATE["dist"]

    def load(self, *models):
        need = [m for m in models if m not in self.loaded]
        if need:
            ok = self.got.load_models(need)
            log("load_models %s -> %s" % (need, ok))
            self.loaded.update(need)
            sleep(1.0)

    def line(self):
        try:
            r = self.got.get_single_track_total_info()
            STATE["line"] = {"offset": r[0], "type": r[1]}
            return r[0], r[1]
        except Exception:
            return 0, 0          # treat a comms hiccup as "no line"

    def balls(self, color):
        """Ping-pong-ball cores of one colour, biggest (closest) first."""
        out = [b for b in self.colors(color) if b["area"] >= CFG["CORE_MIN_AREA"]]
        if CFG["CORE_SHAPE"]:
            shaped = [b for b in out if str(b.get("shape", "")).lower() == CFG["CORE_SHAPE"]]
            if shaped or any(b.get("shape") for b in out):
                out = shaped
        return out

    def colors(self, want=None):
        """List of blobs [{color, cx, cy, area}] biggest first, optionally only one colour."""
        if CFG["COLOR_SOURCE"] == "camera" and not MOCK:
            import tape_eye
            try:
                if not getattr(self, "_cam_open", False):
                    self.got.open_camera()
                    self._cam_open = True
                jpg = self.got.read_camera_data()
                out = tape_eye.color_blobs(jpg) if jpg else []
                STATE["frame_jpg"] = jpg or STATE.get("frame_jpg")
            except Exception as e:
                log("camera colour read failed: %s" % e)
                out = []
            STATE["colors"] = out[:6]
            return [b for b in out if b["color"] == want] if want else out
        out = []
        try:
            raw = self.got.VISION.color_identify()
            data = json.loads(raw) if raw else []
            for it in data or []:
                x, y, w, h = it["position"]
                out.append({"color": str(it.get("color", "")).lower(), "shape": it.get("shape"),
                            "cx": x + w / 2.0, "cy": y + h / 2.0, "area": w * h})
        except Exception:
            try:
                c, s, cx, cy, h, w, a = self.got.get_color_total_info()
                if c:
                    out.append({"color": str(c).lower(), "shape": s, "cx": cx, "cy": cy, "area": a})
            except Exception:
                pass
        out.sort(key=lambda b: b["area"], reverse=True)
        STATE["colors"] = out[:6]
        if want:
            out = [b for b in out if b["color"] == want]
        return out

    def qrcodes(self):
        """QR codes in view. Decoder 1: UGOT QR model. Decoder 2 (failsafe): OpenCV on the raw camera frame."""
        try:
            qs = [{"text": q[0], "cx": q[1], "cy": q[2], "area": q[5], "src": "ugot"}
                  for q in self.got.get_qrcode_total_info() if q[0]]
        except Exception:
            qs = []
        if not qs and not MOCK:
            qs = self._qr_opencv()
        for q in qs:            # mask secrets before STATE: the dashboard thread can read STATE at any time
            q["text"] = mask_secret(q["text"])
        STATE["qr"] = qs
        return qs

    def qr_spots(self):
        """QR codes that are VISIBLE but too small/far to decode (OpenCV locate-only) -> drive closer to them."""
        try:
            import cv2
            import numpy as np
            jpg = STATE.get("frame_jpg")
            if not jpg:
                return []
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
            ok, pts = cv2.QRCodeDetector().detectMulti(img)
            if not ok or pts is None:
                return []
            out = []
            for p in pts:
                p = p / 2.0
                xs, ys = p[:, 0], p[:, 1]
                out.append({"id": "qr-spot", "cx": float(xs.mean()), "cy": float(ys.mean()),
                            "area": float((xs.max() - xs.min()) * (ys.max() - ys.min()))})
            out.sort(key=lambda q: q["area"], reverse=True)
            return out
        except Exception:
            return []

    def _qr_opencv(self):
        try:
            import cv2
            import numpy as np
        except Exception:
            return []
        try:
            if not getattr(self, "_cam_open", False):
                self.got.open_camera()
                self._cam_open = True
            jpg = self.got.read_camera_data()
            if not jpg:
                return []
            STATE["frame_jpg"] = jpg
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            det = cv2.QRCodeDetector()
            out = []
            for scale in (1.0, 2.0):                    # 2x upscale catches small / far codes
                im = img if scale == 1.0 else cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
                ok, texts, pts, _ = det.detectAndDecodeMulti(im)
                if ok and pts is not None:
                    for t, p in zip(texts, pts):
                        if t:
                            p = p / scale
                            xs, ys = p[:, 0], p[:, 1]
                            out.append({"text": t, "cx": float(xs.mean()), "cy": float(ys.mean()),
                                        "area": float((xs.max() - xs.min()) * (ys.max() - ys.min())), "src": "opencv"})
                if out:
                    break
            out.sort(key=lambda q: q["area"], reverse=True)
            return out
        except Exception as e:
            log("opencv QR failed: %s" % e)
            return []

    def apriltags(self):
        try:
            STATE["tags"] = [{"id": t[0], "cx": t[1], "cy": t[2], "area": t[5], "dist": t[7]}
                             for t in self.got.get_apriltag_total_info()]
        except Exception:
            STATE["tags"] = []
        return STATE["tags"]


# =====================================================================================
# Generic behaviours
# =====================================================================================
def center_turn_speed(cx):
    err = cx - CFG["CAM_W"] / 2.0          # + = target right of centre
    return clamp(-err * 0.18, -50, 50)     # + turn = left


def scan_for(bot, detect, full_turns=1.0, step_deg=30, dwell=0.5):
    """Rotate in steps until detect() returns something. Returns the detection or None."""
    bot.quip("scan")
    steps = int(360 * full_turns / step_deg)
    for i in range(steps):          # 12 x 30 deg: a failed scan ends back at the starting heading
        t_end = time.time() + (dwell if not MOCK else 0.0)
        while True:
            hit = detect()
            if hit:
                log("scan hit after %d steps" % i)
                bot.quip("target")
                return hit
            if time.time() >= t_end:
                break
            sleep(0.08)
        bot.turn_deg(step_deg, CFG["SEARCH_TURN_SPEED"] * 2)
    return None


def approach_visual(bot, detect, stop_cm, stop_area=None, stop_y=None, timeout=25, stop_when=None):
    """Drive toward a camera target, steering to keep it centred, until distance/area/y/stop_when says stop."""
    t_end = time.time() + timeout
    lost = 0
    while time.time() < t_end:
        hit = detect()
        d = bot.dist(samples=1)
        # the IR is tilted at the floor (reads ~22-35 cm on open floor): only trust it below the floor band
        if d is not None and 0 < d <= min(stop_cm, CFG["FLOOR_WALL_CM"]):
            bot.stop()
            log("approach: stop by distance %.1f cm" % d)
            return True
        if hit:
            lost = 0
            if (stop_area and hit["area"] >= stop_area) or (stop_y and hit["cy"] >= stop_y) or \
                    (stop_when and stop_when(hit)):
                bot.stop()
                log("approach: stop by camera area=%d cy=%d" % (hit["area"], hit["cy"]))
                return True
            turn = center_turn_speed(hit["cx"])
            off = abs(hit["cx"] - CFG["CAM_W"] / 2.0)
            fwd = CFG["APPROACH_SPEED"] if off < 120 else 5
            if d is not None and d < stop_cm + 15:
                fwd = min(fwd, 8)
            bot.drive(fwd=fwd, turn=turn)
        else:
            lost += 1
            bot.stop()
            if lost > 15:
                log("approach: target lost")
                return False
        sleep(0.06)
    bot.stop()
    return False


def drive_to_wall(bot, stop_cm, tol=2.0, timeout=20):
    """Straight drive using only the distance sensor, settles at stop_cm +- tol."""
    t_end = time.time() + timeout
    good = 0
    blind_since = None
    best, best_t = None, time.time()
    while time.time() < t_end:
        d = bot.dist()
        # the IR sensor sees the floor (~28-30 cm) when nothing is ahead: if the reading isn't shrinking while we
        # drive forward, there is no wall in front -> stop instead of creeping across the arena
        if d is not None and d > 0:
            if best is None or d < best - 2:
                best, best_t = d, time.time()
            elif d > stop_cm + tol and time.time() - best_t > 3.0 and best > stop_cm:   # only while approaching
                bot.stop()
                log("drive_to_wall: reading stuck at %.1f cm for 3 s (floor) - no wall ahead" % d)
                return d
        if d is None:
            # no reading (sensor missing or wall out of range): creep at most 2 s, then give up
            blind_since = blind_since or time.time()
            if time.time() - blind_since > 2.0:
                bot.stop()
                log("drive_to_wall: no distance reading")
                return None
            bot.drive(fwd=6)
            sleep(0.08)
            continue
        blind_since = None
        err = d - stop_cm
        if abs(err) <= tol:
            bot.stop()
            good += 1
            if good >= 3:
                return d
        else:
            good = 0
            v = clamp(err * 0.9, -12, 25)
            if 0 < v < 5:
                v = 5
            if -5 < v < 0:
                v = -5
            bot.drive(fwd=v)
        sleep(0.07)
    bot.stop()
    return bot.dist()


def run_moves(bot, moves):
    for kind, val in moves:
        if kind == "fwd":
            bot.move_cm(val)
        elif kind == "back":
            bot.move_cm(-val)
        elif kind == "left":
            bot.turn_deg(val)
        elif kind == "right":
            bot.turn_deg(-val)
        elif kind == "strafe_l":
            bot.strafe_cm(-val)
        elif kind == "strafe_r":
            bot.strafe_cm(val)


def go_to_target(bot, spec, label):
    """Navigate to a drop-off target described in CFG. Returns True when in front of it."""
    t = spec["type"]
    stop_cm = spec.get("stop_cm", 12)
    log("going to %s via %s" % (label, t))
    ok = False
    if t == "moves":
        run_moves(bot, spec["moves"])
        ok = True
    elif t == "wall":
        drive_to_wall(bot, stop_cm)
        ok = True
    else:
        if t == "apriltag":
            bot.load("apriltag_qrcode")

            def det():
                tags = bot.apriltags()
                if spec.get("id") is not None:
                    tags = [x for x in tags if x["id"] == spec["id"]]
                return tags[0] if tags else None
        elif t == "qr":
            bot.load("apriltag_qrcode")

            def det():
                q = [x for x in bot.qrcodes() if spec["text"].lower() in x["text"].lower()]
                return q[0] if q else None
        elif t == "color":
            bot.load("color_recognition")

            def det():
                b = bot.colors(spec["color"])
                return b[0] if b else None
        else:
            raise ValueError("unknown target type " + t)
        if scan_for(bot, det):
            ok = approach_visual(bot, det, stop_cm, stop_area=spec.get("stop_area", CFG["TARGET_STOP_AREA"]),
                                 timeout=30)
    if not ok and spec.get("fallback"):
        log("%s: primary method failed -> fallback" % label)
        return go_to_target(bot, spec["fallback"], label)
    return ok


# =====================================================================================
# Evidence decoding (QR fragments)
# =====================================================================================
MORSE = {".-": "A", "-...": "B", "-.-.": "C", "-..": "D", ".": "E", "..-.": "F", "--.": "G", "....": "H",
         "..": "I", ".---": "J", "-.-": "K", ".-..": "L", "--": "M", "-.": "N", "---": "O", ".--.": "P",
         "--.-": "Q", ".-.": "R", "...": "S", "-": "T", "..-": "U", "...-": "V", ".--": "W", "-..-": "X",
         "-.--": "Y", "--..": "Z", "-----": "0", ".----": "1", "..---": "2", "...--": "3", "....-": "4",
         ".....": "5", "-....": "6", "--...": "7", "---..": "8", "----.": "9", "---...": ":", "-...-": "="}
COMMON = ("THE", "CORE", "RED", "BLUE", "GREEN", "IS", "NOT", "SAFE", "ENERGY", "CORRECT", "TRUE", "FALSE",
          "VAULT", "SIGNATURE", "RESTART", "AND", "ONLY", "CONTAINS", "WRONG", "DANGER", "STABLE", "UNSTABLE")


def _printable(s):
    return s and sum(c.isprintable() or c in "\n\r\t" for c in s) / len(s) > 0.95


def decode_candidates(text):
    t = text.strip()
    cands = {"raw": t, "reversed": t[::-1]}
    try:
        cands["url"] = urllib.parse.unquote(t)
    except Exception:
        pass
    compact = re.sub(r"\s+", "", t)
    for name, fn in (("base64", lambda s: base64.b64decode(s + "=" * (-len(s) % 4), validate=True)),
                     ("base64url", lambda s: base64.urlsafe_b64decode(s + "=" * (-len(s) % 4))),
                     ("base32", lambda s: base64.b32decode(s.upper() + "=" * (-len(s) % 8))),
                     ("hex", lambda s: binascii.unhexlify(s))):
        try:
            d = fn(compact).decode("utf-8")
            if _printable(d):
                cands[name] = d
        except Exception:
            pass
    if re.fullmatch(r"[01\s]+", t) and len(compact) % 8 == 0:
        try:
            cands["binary"] = "".join(chr(int(compact[i:i + 8], 2)) for i in range(0, len(compact), 8))
        except Exception:
            pass
    if re.fullmatch(r"[\d\s,]+", t):
        try:
            nums = [int(n) for n in re.split(r"[\s,]+", t) if n]
            if all(32 <= n < 127 for n in nums):
                cands["ascii_codes"] = "".join(chr(n) for n in nums)
            if all(1 <= n <= 26 for n in nums):
                cands["a1z26"] = "".join(chr(64 + n) for n in nums)
        except Exception:
            pass
    if re.fullmatch(r"[.\-/\s|]+", t):
        words = re.split(r"\s*[/|]\s*|\s{2,}", t)
        cands["morse"] = " ".join("".join(MORSE.get(c, "?") for c in w.split()) for w in words)
    cands["atbash"] = "".join(chr(155 - ord(c)) if "A" <= c <= "Z" else chr(219 - ord(c)) if "a" <= c <= "z" else c for c in t)
    for k in range(1, 26):
        cands["caesar-%d" % k] = "".join(
            chr((ord(c) - 65 - k) % 26 + 65) if "A" <= c <= "Z" else chr((ord(c) - 97 - k) % 26 + 97) if "a" <= c <= "z" else c
            for c in t)
    # second layer: decode base64 etc. of a caesar/reversed result
    for k, v in list(cands.items()):
        if k in ("raw",):
            continue
        try:
            d = base64.b64decode(re.sub(r"\s+", "", v) + "=" * (-len(re.sub(r"\s+", "", v)) % 4), validate=True).decode("utf-8")
            if _printable(d) and len(d) > 2:
                cands[k + "+base64"] = d
        except Exception:
            pass
    return cands


def score(s):
    u = s.upper()
    return sum(3 if w in COLORS_UP else 1 for w in COMMON if re.search(r"\b" + w + r"\b", u)) + \
        sum(2 for c in COLORS_UP if c in u)


COLORS_UP = ("RED", "BLUE", "GREEN")


def analyse_fragment(text):
    cands = decode_candidates(text)
    ranked = sorted(cands.items(), key=lambda kv: score(kv[1]), reverse=True)
    best_name, best = ranked[0]
    if score(best) == 0:
        best_name, best = "raw", text
    return {"raw": text, "method": best_name, "decoded": best,
            "top": [(n, v) for n, v in ranked[:5] if score(v) > 0]}


FOREIGN = {"red": ("ROUGE", "ROT", "ROJO", "ROSSO", "VERMELHO"),
           "blue": ("BLEU", "BLAU", "AZUL", "BLU"),
           "green": ("VERT", "GRUN", "GRÜN", "VERDE", "MIDORI")}
EMOJI = {"red": "\U0001F534❤\U0001F7E5", "blue": "\U0001F535\U0001F499\U0001F7E6", "green": "\U0001F7E2\U0001F49A\U0001F7E9"}


def data_colour_hints(text):
    """Colour clues hidden in raw data: hex/rgb colour codes, wavelengths, other languages, emoji."""
    hints = []
    hexes = re.findall(r"(?:#|0[xX])([0-9A-Fa-f]{6})\b", text)
    if re.fullmatch(r"\s*([0-9A-Fa-f]{6})\s*", text):
        hexes.append(text.strip())
    for h in hexes:
        r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
        top = max(r, g, b)
        if top >= 128 and sorted((r, g, b))[1] < top * 0.6:
            hints.append((("red", "green", "blue")[(r, g, b).index(top)], "colour code #%s" % h))
    for r, g, b in re.findall(r"RGB\s*\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\)", text.upper()):
        v = (int(r), int(g), int(b))
        if sorted(v)[1] < max(v) * 0.6:
            hints.append((("red", "green", "blue")[v.index(max(v))], "rgb%s" % (v,)))
    for nm in re.findall(r"\b(\d{3})\s*NM\b", text.upper()):
        n = int(nm)
        c = "blue" if 440 <= n < 495 else "green" if 495 <= n < 570 else "red" if 620 <= n <= 750 else None
        if c:
            hints.append((c, "wavelength %d nm" % n))
    u = text.upper()
    for c, words in FOREIGN.items():
        for w in words:
            if len(w) > 2 and re.search(r"\b" + w + r"\b", u):
                hints.append((c, "word '%s'" % w))
    for c, chars in EMOJI.items():
        if any(ch in text for ch in chars):
            hints.append((c, "emoji"))
    return hints


def color_from_evidence(fragments):
    """Votes: a colour mentioned positively scores +, mentioned with NOT/UNSAFE/DANGER scores -."""
    votes = {c: 0.0 for c in COLORS}
    reasons = []
    neg_re = r"\b(NOT|ISN'T|NEVER|FALSE|UNSAFE|UNSTABLE|DANGER|DANGEROUS|DECOY|FAKE|WRONG|AVOID|TRAP|FAIL|FAILURE)\b"
    for f in fragments:
        u = str(f.get("decoded", "")).upper()
        # judge each clause on its own so "RED IS NOT SAFE, BLUE IS" doesn't count against BLUE
        for clause in re.split(r"[.,;:!?\n]|\bBUT\b|\bWHILE\b", u):
            for c in COLORS:
                if re.search(r"\b" + c.upper() + r"\b", clause):
                    neg = re.search(neg_re, clause)
                    votes[c] += -1.0 if neg else 1.0
                    reasons.append("%s: '%s' -> %s" % (c, clause.strip()[:60], "against" if neg else "for"))
        raw, dec = str(f.get("raw", "")), str(f.get("decoded", ""))
        for c, why in set(data_colour_hints(raw if raw == dec else raw + " \n " + dec)):
            votes[c] += 1.0
            reasons.append("%s: %s" % (c, why))
    ranked = sorted(votes.items(), key=lambda kv: kv[1], reverse=True)
    choice = ranked[0][0] if ranked[0][1] > 0 and ranked[0][1] > ranked[1][1] else None
    # if two colours were ruled out, the remaining one wins
    neg = [c for c, v in votes.items() if v < 0]
    if len(neg) == 2:
        choice = [c for c in COLORS if c not in neg][0]
        reasons.append("eliminated %s -> %s" % (", ".join(neg), choice))
    return choice, votes, reasons


def load_evidence():
    try:
        with open(CFG["EVIDENCE_FILE"]) as f:
            return json.load(f)
    except Exception:
        return {"fragments": []}


def save_evidence(ev):
    with open(CFG["EVIDENCE_FILE"], "w") as f:
        json.dump(ev, f, indent=2)


def add_fragment(text, source):
    text = mask_secret(text)                        # also covers --evidence and web-page text
    ev = load_evidence()
    if not any(fr["raw"] == text for fr in ev["fragments"]):
        fr = analyse_fragment(text)
        fr["source"] = source
        fr["time"] = time.strftime("%H:%M:%S")
        ev["fragments"].append(fr)
        save_evidence(ev)
    return load_evidence()


def chosen_core(args):
    """Priority: team decision (--core) > config > Gemma 4 (confident) > rule-based votes."""
    if args is not None and getattr(args, "core", None):
        return args.core.lower(), "chosen by team (--core)"
    if CFG["CORE_COLOR"]:
        return CFG["CORE_COLOR"].lower(), "chosen in config"
    ev = load_evidence()
    c, votes, reasons = color_from_evidence(ev["fragments"])
    g = ev.get("gemma")
    if g and g.get("core") in COLORS and (g.get("confidence", 0) >= 0.6 or not c):
        return g["core"], "Gemma 4 analysis (%.0f%% confident): %s" % (100 * g.get("confidence", 0), g.get("reasoning", ""))
    if c:
        return c, "rule-based evidence votes %s" % votes
    if g and g.get("core") in COLORS:
        return g["core"], "Gemma 4 analysis (low confidence)"
    # no colour clue anywhere: pick one at random, but keep the SAME pick for the rest of this evidence file
    if ev.get("random_core") not in COLORS:
        import random
        ev["random_core"] = random.choice(COLORS)
        save_evidence(ev)
    return ev["random_core"], "no colour clue in the evidence - chose %s at random" % ev["random_core"]


# A real Wi-Fi QR: "WIFI:" directly followed by a field, e.g. WIFI:T:WPA;S:<name>;P:<password>;;
WIFI_BLOCK = re.compile(r"(?i)WIFI:(?=(?:PH2|[STPHEAIRK]):)")
# The P: value ends at the next ";<field>:" or ";;" or the end, so "\;" or a wrongly unescaped ";" is masked too.
WIFI_PASS = re.compile(r"(?i)(?<=[:;])(\s*P:)(?:\\[\s\S]?|[^;\\]|;(?!\s*(?:PH2|[STHEAIRK]):|;))*")
WIFI_WORDS = re.compile(r"(?i)wi-?fi|wlan|ssid|\bwpa")
URL_LOGIN = re.compile(r"(?i)(\b[a-z][a-z0-9+.-]*://[^/\s:@]+:)[^@\s/]+(?=@)")      # https://user:<password>@host


def redact(text, strict=False):
    """Never speak / print / store secrets that happen to be in a QR (e.g. Wi-Fi login codes).
    strict=True is for Caesar / Atbash / reversed decodes: only long, unambiguous signs of a secret count there,
    because some letter shift of almost any clue contains a short word like "pwd:"."""
    t = t0 = str(text)
    if WIFI_BLOCK.search(t):
        t = WIFI_PASS.sub(r"\1****", t)
    wifi_text = WIFI_WORDS.search(t)
    labels, sep = "password|passwd|passphrase", r"[\"']?\s*[:=]"
    if wifi_text:                                   # Wi-Fi text: also "the Wi-Fi password is x"
        sep = r"[\"']?(?:\s*[:=\-]|\s+is\b)"
    if not strict:
        labels += r"|pwd|psk|security\s+key"
        if wifi_text:                               # and "Key: x", "PW: x"
            labels += "|passcode|key|pw|pin"
    # mask one value: a quoted string or one word (a clue later in the line stays readable); skip values already masked
    t = re.sub(r"(?i)(?<![A-Za-z])((?:%s)%s\s*)(?!\*{4})(\"[^\"\n]*\"|'[^'\n]*'|[^\s;&]+)" % (labels, sep), r"\1****", t)
    t = URL_LOGIN.sub(r"\1****", t)
    if t == t0 and "%" in t:                        # nothing found: maybe a URL-encoded secret
        u = urllib.parse.unquote(t)
        if u != t and redact(u, strict) != u:
            return redact(u, strict)
    return t


# decodes that can hide one more layer (Caesar, Atbash and the base64-of-each layer are already in decode_candidates)
LAYER_DECODES = ("base64", "base64url", "base32", "hex", "binary", "ascii_codes", "url", "reversed")
SHIFT_DECODES = {"reversed", "atbash"} | {"caesar-%d" % k for k in range(1, 26)}


def mask_secret(text, depth=2):
    """redact(), and also look under the decodes we know, up to two layers deep: a Wi-Fi QR that is
    base64 / hex / Caesar / ... encoded is replaced by its masked plaintext, so the encoded password is not kept."""
    return _mask_secret(str(text), depth, False)


@functools.lru_cache(maxsize=512)               # the same QR is seen in many frames: decode it once
def _mask_secret(t, depth, strict):
    t = redact(t, strict)                       # then keep looking: a second secret may be encoded in the rest
    cands = decode_candidates(t)
    for k, v in cands.items():
        if k != "raw" and v != t:
            r = redact(v, strict or k in SHIFT_DECODES)
            if r != v:
                return r
    if depth > 1:
        for k in LAYER_DECODES:
            v = cands.get(k)
            if v and v != t:
                m = _mask_secret(v, depth - 1, strict or k in SHIFT_DECODES)
                if m != v:
                    return m
    return t


# =====================================================================================
# Missions
# =====================================================================================
def m1_activation(bot, args):
    bot.lights(255, 80, 0, 3)
    name = ""
    try:
        name = bot.got.get_device_name()
    except Exception:
        pass
    p = None
    try:
        p = bot.got.get_power_status()
    except Exception:
        pass
    log("connected to %s battery=%s" % (name, p))
    try:
        bot.got.set_volume(100)
    except Exception:
        pass
    try:
        bot.got.screen_display_emotion("WakeUp")
    except Exception:
        pass
    bot.quip("boot", wait=True)
    bot.say("Emergency activation complete. Systems online. We are team %s." % CFG["TEAM_NAME"])
    bot.quip("team", wait=True)
    bot.lights(0, 255, 0, 0)
    return True


def calibrate_line_sign(bot):
    """Turn 8 deg left and back while looking at the tape: tells us which way the offset moves.
    Fixes LINE_SIGN for this robot/camera automatically (costs ~2 s)."""
    def med():
        v = []
        for _ in range(4):
            o, t = bot.line()
            if t != 0:
                v.append(o)
            sleep(0.08)
        return sorted(v)[len(v) // 2] if len(v) >= 2 else None
    o1 = med()
    if o1 is None:
        log("line-sign calibration skipped (no tape in view) -> LINE_SIGN=%d" % CFG["LINE_SIGN"])
        return
    bot.turn_deg(8, 40)
    sleep(0.4)                 # camera inference lags the motion: let it catch up before reading
    o2 = med()
    bot.turn_deg(-8, 40)
    sleep(0.6)                 # ...and again before following, or stale "tape far right" frames steer us off
    for _ in range(3):
        bot.line()
    if o2 is None or abs(o2 - o1) < 12:
        log("line-sign calibration inconclusive (%s -> %s) -> keeping LINE_SIGN=%d" % (o1, o2, CFG["LINE_SIGN"]))
        return
    CFG["LINE_SIGN"] = 1 if o2 > o1 else -1
    log("line-sign calibrated: offset %s -> %s after left turn => LINE_SIGN=%d" % (o1, o2, CFG["LINE_SIGN"]))


def follow_tape(bot, stop_on_wall=True):
    """Follow the black tape. Ends when wall is close (distance sensor) or tape ends."""
    bot.load("line_recognition")
    bot.got.set_track_recognition_line(CFG["LINE_TYPE"])
    sleep(0.5)
    if CFG["LINE_SIGN_AUTO"] and not MOCK:
        calibrate_line_sign(bot)
    plan = list(CFG["JUNCTION_PLAN"])
    t_end = time.time() + CFG["ROUTE_MAX_S"]
    # The IR sensor may see the FLOOR (UGOT_B2E8 reads ~30 cm standing still). Measure that baseline so the floor
    # is never mistaken for the calibration-bay wall: only readings clearly below it count as a wall.
    base = [v for v in (bot.dist(samples=1) for _ in range(6)) if v is not None and v > 0]
    switch_cm = CFG["WALL_SWITCH_CM"]
    if base:
        b = sorted(base)[len(base) // 2]
        if b < switch_cm + 5:
            switch_cm = max(CFG["WALL_TARGET_CM"] + 4, b - 5)
            log("distance baseline %.1f cm at start (floor or object) -> wall trigger lowered to %.1f cm" % (b, switch_cm))
    # distance sensor read in its own thread -> the steering loop runs at the camera's full rate (~11 Hz)
    dist_box = {"d": None, "n": 0, "run": True}

    def dist_reader():
        while dist_box["run"]:
            dist_box["d"] = bot.dist(samples=1)
            dist_box["n"] += 1
            time.sleep(0.03)
    threading.Thread(target=dist_reader, daemon=True).start()
    try:
        return _follow_loop(bot, stop_on_wall, plan, t_end, switch_cm, dist_box)
    finally:
        dist_box["run"] = False
        bot.stop()


def _follow_loop(bot, stop_on_wall, plan, t_end, switch_cm, dist_box):
    """Micro-correction tape follower:
       |offset| < DEADBAND        -> straight
       small offset               -> gentle sideways strafe + tiny turn (mecanum: stays on the tape)
       large offset (corner)      -> slow down, slow turn
       tape lost                  -> stop, rotate slowly toward where it was last seen, give up after a sweep"""
    last_off = 0.0
    lost_since = None
    last_junction = 0.0
    bias, bias_until = 0.0, 0.0
    seen_line = False
    sharp = False
    wall_hits = 0
    last_n = -1
    turned = 0.0
    t_start = time.time()
    last_t = t_start
    sgn = -CFG["LINE_SIGN"]                     # + offset * sgn => tape is to the LEFT => + turn / - strafe
    bot.quip("tracking", wait=True)             # finish speaking BEFORE moving
    t_start = last_t = time.time()
    while time.time() < t_end:
        d = dist_box["d"]
        if dist_box["n"] != last_n:             # count each new distance reading once
            last_n = dist_box["n"]
            close = d is not None and 0 < d <= switch_cm
            wall_hits = wall_hits + 1 if close else 0
        if stop_on_wall and wall_hits >= CFG["WALL_CONFIRM"] \
                and time.time() - t_start >= CFG["WALL_MIN_ROUTE_S"]:
            log("wall ahead at %.1f cm -> end of tape route" % d)
            bot.stop()
            bot.quip("wall")
            return "wall"
        off, typ = bot.line()
        now = time.time()
        dt = clamp(now - last_t, 0.01, 0.3)
        last_t = now
        if typ == 0:
            if lost_since is None:
                lost_since = now
                turned = 0.0
                if seen_line:
                    bot.quip("lost")
            lost_for = now - lost_since
            if not seen_line:
                if now - t_start > CFG["NO_TAPE_GIVEUP_S"]:
                    log("never saw the tape in %.0fs - stopping (check LINE_TYPE / lighting / placement)"
                        % CFG["NO_TAPE_GIVEUP_S"])
                    return "no_tape"
                bot.drive(fwd=6)                # haven't found tape yet: creep forward slowly
                continue
            side = 1 if last_off * sgn > 0 else -1          # side the tape was last seen (+ = left)
            if lost_for < 0.4:
                bot.drive(fwd=4, turn=side * 15)            # tiny nudge: usually the next frame finds it again
                turned += side * 15 * dt
            elif CFG["LINE_LOST_SEARCH"] and lost_for < 0.4 + CFG["LOST_SWEEP_S"]:
                bot.drive(turn=side * CFG["LOST_SWEEP_SPEED"])   # slow rotate toward the last-seen side
                turned += side * CFG["LOST_SWEEP_SPEED"] * dt
            else:
                log("tape ended - turning back %.0f deg to the route heading" % -turned)
                bot.stop()
                bot.turn_deg(-turned, 40)
                return "end"
            continue
        seen_line = True
        lost_since = None
        if typ in (2, 3) and now - last_junction > CFG["JUNCTION_DEBOUNCE_S"]:
            last_junction = now
            choice = plan.pop(0) if plan else "straight"
            log("junction type %d -> %s" % (typ, choice))
            bot.quip("junction")
            if choice in ("left", "right"):
                bias = CFG["JUNCTION_BIAS"] * (1 if choice == "left" else -1)
                bias_until = now + CFG["JUNCTION_BIAS_S"]
        e = off * sgn                          # + = tape to the LEFT of centre
        a = abs(e)
        is_sharp = a > CFG["LINE_SHARP_OFFSET"]
        if is_sharp and not sharp:
            bot.quip("corner")
        sharp = is_sharp
        if a < CFG["LINE_DEADBAND"]:
            turn, strafe = 0.0, 0.0                                       # on the tape: just drive
        elif not is_sharp:
            turn = CFG["LINE_KP"] * e                                     # micro-correction
            strafe = -clamp(CFG["LINE_KS"] * e, -CFG["LINE_MAX_STRAFE"], CFG["LINE_MAX_STRAFE"])
        else:
            turn = CFG["LINE_KP_CORNER"] * e                              # corner: slow, firmer turn
            strafe = 0.0
        if now < bias_until:
            turn += bias
        turn = clamp(turn, -CFG["LINE_MAX_TURN"], CFG["LINE_MAX_TURN"])
        # corner: rotate IN PLACE until the tape is back near the centre, then drive on
        speed = 0.0 if is_sharp else CFG["LINE_SPEED"]
        if is_sharp:
            turn = clamp(turn, -CFG["LINE_MAX_TURN"], CFG["LINE_MAX_TURN"])
            if abs(turn) < 15:
                turn = 15 if turn >= 0 else -15
        ramp = clamp((now - t_start) / 1.5, 0.4, 1.0)        # ease in over the first 1.5 s
        speed *= ramp
        if d is not None and 0 < d < switch_cm + 20:
            speed = min(speed, 8)
        bot.drive(fwd=speed, turn=turn, strafe=strafe)
        last_off = off
    return "timeout"


def follow_tape_camera(bot, stop_on_wall=True, bump=True):
    """PREDICTIVE black-tape follower on our own vision (tape_eye.py) - ignores brown duct-tape decoys.
    near band = where the tape is under the nose (lateral error), far band = where it is going (prediction):
      turn   = K_AHEAD*far + K_LAT*near     (steer toward where the tape goes, before the bend arrives)
      strafe = K_STRAFE*near                 (mecanum micro-shift to stay centred)
      speed  = slows down as far-near grows  (bend ahead)
    Sharp corner (tape leaves the far/mid view): stop, rotate in place toward it.
    Tape lost: full 360 deg search, toward the side it was last seen, until black tape is CONFIRMED
    (seen in the near/mid band on TAPE_CONFIRM_FRAMES consecutive frames). Skips the path we came from."""
    import tape_eye
    eye = tape_eye.TapeEye(bot.got, STATE).start()
    dist_box = {"d": None, "n": 0, "run": True}

    def dist_reader():
        while dist_box["run"]:
            dist_box["d"] = bot.dist(samples=1)
            dist_box["n"] += 1
            time.sleep(0.03)
    threading.Thread(target=dist_reader, daemon=True).start()
    try:
        # floor baseline for the IR sensor (it sees the floor at ~28-30 cm when nothing is ahead)
        sleep(0.6)
        base = [v for v in (bot.dist(samples=1) for _ in range(5)) if v is not None and v > 0]
        switch_cm = CFG["WALL_SWITCH_CM"]
        if base:
            b = min(base)
            if b < switch_cm + 5:
                # sensor is tilted down and sees the FLOOR at ~22-35 cm (measured on UGOT_B2E8),
                # so only readings below the floor band can be a wall
                switch_cm = min(CFG["FLOOR_WALL_CM"], b - 3)
                log("IR sees the floor (%.1f cm) -> wall trigger %.1f cm" % (b, switch_cm))
        bot.quip("tracking", wait=True)               # speak BEFORE moving
        return _camera_follow_loop(bot, eye, dist_box, switch_cm, stop_on_wall, bump)
    finally:
        dist_box["run"] = False
        eye.stop()
        bot.stop()


_last_status = [0.0]


def data_line(a, d, fwd, turn, strafe, mode, fps):
    """Live sensor/decision line for the PowerShell window (~2 per second) - what the robot sees and does."""
    now = time.time()
    if now - _last_status[0] < 0.5:
        return
    _last_status[0] = now

    def band(k):
        v = a.get(k)
        if not v:
            return "%-4s ----        " % k
        side = "L" if v[0] < -0.03 else "R" if v[0] > 0.03 else "C"
        return "%-4s %+.2f%s %s  " % (k, v[0], side, a.get(k + "_src") or "")
    bw = a.get("blue_wall_cm")
    mot = a.get("motion")
    log("[DATA] %s| %s%s%s| cam %4.1ffps | drive fwd %4.1f turn %+5.1f strafe %+4.1f | IR %s | blue wall %s | floor motion %s"
        % (mode, band("near"), band("mid"), band("far"), fps, fwd, turn, strafe,
           ("%.1fcm" % d) if d else "--", ("%.0fcm" % bw) if bw else "none", ("%.1f" % mot) if mot is not None else "--"))


def _camera_follow_loop(bot, eye, dist_box, switch_cm, stop_on_wall, bump=True):
    C = CFG
    plan = list(C["JUNCTION_PLAN"])
    t_start = time.time()
    t_end = t_start + C["ROUTE_MAX_S"]
    seq = -1
    last_side = 0                 # +1 tape last seen to the right, -1 left
    wall_hits, last_n = 0, -1
    seen = False
    last_junction = 0.0
    stall = 0
    blue_hits = 0
    last_cmd = (0.0, 0.0)                    # (forward, turn) we last commanded
    while time.time() < t_end:
        a, seq = eye.wait_new(seq)
        if a is None:
            continue
        now = time.time()
        # ---- bump detection: driving forward but the camera picture is frozen -> pushing a wall ----
        mot = a.get("motion")
        if bump and mot is not None and last_cmd[0] >= 4 and abs(last_cmd[1]) < 12 and now - t_start > 3:
            stall = stall + 1 if mot < C["STALL_MOTION"] else 0
            if stall >= C["STALL_FRAMES"]:
                bot.stop()
                log("BUMP: wheels driving but picture frozen (motion %.1f) -> wall contact, backing off %d cm"
                    % (mot, C["BUMP_BACKOFF_CM"]))
                bot.quip("wall")
                bot.move_cm(-C["BUMP_BACKOFF_CM"], 10)
                return "bump"
        else:
            stall = 0
        # ---- wall check (calibration bay) ----
        d = dist_box["d"]
        if dist_box["n"] != last_n:
            last_n = dist_box["n"]
            wall_hits = wall_hits + 1 if (d is not None and 0 < d <= switch_cm) else 0
        if stop_on_wall and seen and wall_hits >= C["WALL_CONFIRM"]:
            log("wall ahead at %.1f cm (IR) -> end of tape route" % d)
            bot.stop()
            bot.quip("wall")
            return "wall"
        # ---- BLUE calibration-bay wall seen by the camera ----
        bw = a.get("blue_wall_cm")
        blue_hits = (blue_hits + 1) if (bw is not None and bw <= C["BLUE_WALL_STOP_CM"]) else 0
        if stop_on_wall and blue_hits >= 2 and now - t_start > 2:
            bot.stop()
            log("BLUE WALL at %.1f cm (camera) -> stop" % bw)
            STATE["blue_wall_cm"] = bw
            bot.quip("wall")
            return "bluewall"
        near, mid, far = a.get("near"), a.get("mid"), a.get("far")
        # ---- fork: choose the branch from JUNCTION_PLAN ----
        if a.get("fork") and now - last_junction > C["JUNCTION_DEBOUNCE_S"] and seen:
            last_junction = now
            choice = plan.pop(0) if plan else "straight"
            eye.prefer = choice if choice in ("left", "right") else None
            log("fork detected -> taking %s" % choice)
            bot.quip("junction")
        elif now - last_junction > C["JUNCTION_BIAS_S"] * 2:
            eye.prefer = None
        lat = near[0] if near else (mid[0] if mid else None)
        ahead = far[0] if far else (mid[0] if mid else None)
        if lat is None and ahead is None:
            if not seen and now - t_start < C["NO_TAPE_GIVEUP_S"]:
                bot.drive(fwd=5)              # haven't found tape yet: creep
                continue
            bot.stop()
            bot.quip("lost")
            found = search_tape_360(bot, eye, last_side)
            if not found:
                log("no black tape anywhere in a full 360 - route ended")
                return "end"
            seen = True
            continue
        seen = True
        if lat is None:                       # only far/mid visible: steer by it
            lat = ahead
        if ahead is None:                     # tape bends out of the far view: sharp corner
            ahead = lat * 2.0
        last_side = 1 if (ahead if abs(ahead) > 0.05 else lat) > 0 else -1
        bend = abs(ahead - lat)
        if abs(ahead) > C["CAM_SHARP"] or (near and not mid and not far):
            # sharp corner: rotate in place toward the tape until it is ahead again
            if not getattr(bot, "_corner_said", False):
                bot.quip("corner")
                bot._corner_said = True
            bot.drive(turn=-last_side * C["CAM_CORNER_TURN"])
            data_line(a, d, 0, -last_side * C["CAM_CORNER_TURN"], 0, "CORNER  ", eye.fps)
            continue
        bot._corner_said = False
        if abs(lat) < C["CAM_DEADBAND"] and abs(ahead) < C["CAM_DEADBAND"]:
            turn, strafe = 0.0, 0.0
        else:
            turn = -(C["CAM_K_AHEAD"] * ahead + C["CAM_K_LAT"] * lat)
            strafe = clamp(C["CAM_K_STRAFE"] * lat, -C["LINE_MAX_STRAFE"], C["LINE_MAX_STRAFE"])
        turn = clamp(turn, -C["LINE_MAX_TURN"], C["LINE_MAX_TURN"])
        speed = C["CAM_SPEED"] * (1.0 - 0.75 * min(1.0, bend / 0.5))     # slow BEFORE the bend
        speed *= clamp((now - t_start) / 1.5, 0.4, 1.0)
        if stop_on_wall:
            if d is not None and 0 < d < switch_cm + 6:
                speed = min(speed, 7)
            if bw is not None and bw < C["BLUE_WALL_SLOW_CM"]:            # blue wall coming: creep so the
                speed = min(speed, 6)                                     # stop lands inside 15-25 cm
        bot.drive(fwd=max(speed, 4), turn=turn, strafe=strafe)
        last_cmd = (max(speed, 4), turn)
        data_line(a, d, max(speed, 4), turn, strafe, "TRACKING", eye.fps)
    return "timeout"


def search_tape_360(bot, eye, last_side):
    """Rotate in place (first toward the side the tape was last seen) up to a full 360 deg until black tape is
    confirmed on several consecutive frames. Candidates ~180 deg away are the path we came from -> skipped."""
    C = CFG
    direction = -1 if last_side > 0 else 1        # tape last seen right -> turn right (negative = right)
    speed = C["SEARCH_360_SPEED"]
    turned, ok_frames, seq, t_prev = 0.0, 0, eye.seq, time.time()
    log("tape lost - full 360 search, turning %s first" % ("right" if direction < 0 else "left"))
    try:                                           # keep the frame where we lost it, for debugging
        bot._lost_n = getattr(bot, "_lost_n", 0) + 1
        if STATE.get("frame_jpg"):
            open("lost_%d.jpg" % bot._lost_n, "wb").write(STATE["frame_jpg"])
    except Exception:
        pass
    eye.prefer = None
    eye.latest = None                             # reset continuity: any black tape may be the new path
    while abs(turned) < 360:
        bot.drive(turn=direction * speed)
        a, seq = eye.wait_new(seq)
        now = time.time()
        turned += direction * speed * min(now - t_prev, 0.4)
        t_prev = now
        if not a:
            continue
        data_line(a, None, 0, direction * speed, 0, "SEARCH %3.0f" % abs(turned), eye.fps)
        cand = a.get("near") or a.get("mid")
        if cand and abs(cand[0]) < 0.6:
            ok_frames += 1
            if ok_frames >= C["TAPE_CONFIRM_FRAMES"]:
                if 140 < abs(turned) < 220 and not a.get("far"):
                    log("tape at %.0f deg looks like the path we came from - continuing search" % turned)
                    ok_frames = 0
                    continue
                bot.stop()
                log("black tape confirmed after turning %.0f deg" % turned)
                bot.quip("target")
                return True
        else:
            ok_frames = 0
    bot.stop()
    return False


def m2_navigation(bot, args):
    bot.arm("ARM_LOOK")
    if CFG["TAPE_SOURCE"] == "camera" and not MOCK:
        res = follow_tape_camera(bot)
    else:
        res = follow_tape(bot)
    log("route result: %s" % res)
    if res == "bluewall":       # camera stopped us ~22 cm before the blue wall
        bw = STATE.get("blue_wall_cm") or 20
        bot.say("Calibration bay reached. Wall distance %d centimetres. Calibration complete." % round(bw))
        return True
    if res == "bump":           # touched the wall and backed off BUMP_BACKOFF_CM: we are ~20 cm away by odometry
        d = bot.dist()
        shown = d if (d is not None and 15 <= d <= 25) else CFG["BUMP_BACKOFF_CM"]
        bot.say("Calibration bay reached. Wall distance %d centimetres. Calibration complete." % round(shown))
        return True
    d = drive_to_wall(bot, CFG["WALL_TARGET_CM"], CFG["WALL_TOL_CM"])
    if d is None:
        bot.say("Calibration bay reached. Distance sensor has no reading.")
        return False
    ok = 15 <= d <= 25
    bot.say("Calibration bay reached. Wall distance %d centimetres. %s" %
            (round(d), "Calibration complete." if ok else "Out of range."))
    return ok


def m3_comms(bot, args):
    bot.stop()
    try:
        bot.got.enable_audio_direction()
    except Exception as e:
        log("enable_audio_direction: %s" % e)
    bot.quip("listen", wait=True)
    sleep(0.8)                         # don't hear our own voice
    t_end = time.time() + CFG["SOUND_TIMEOUT_S"]
    angle = None
    while time.time() < t_end:
        try:
            r = bot.got.AUDIO.getDirectionOfAudio()
            if r.code == 0 and r.angle != -1:
                angle = r.angle
                break
        except Exception:
            try:
                s = bot.got.get_audio_direction()
            except Exception:
                s = None
            if s:
                angle = {"Front": 90, "Left": 180, "Behind": 270, "Right": 0}.get(s, None)
                if angle is not None:
                    break
        sleep(0.05)
    STATE["sound_angle"] = angle
    if angle is None:
        bot.say("No sound detected.")
    else:
        rel = (angle - CFG["SOUND_FRONT_ANGLE"] + 540) % 360 - 180     # + = left
        log("sound at %d deg -> turn %d deg" % (angle, rel))
        bot.quip("heard")
        bot.turn_deg(rel, 120)
    try:
        bot.got.disable_audio_direction()
    except Exception:
        pass
    p = bot.got.get_power_status() or [None, None, None]
    pitch, roll, yaw = bot.got.read_gyro_data()[:3]
    att = bot.got.get_attitude()
    tilt_names = ["forward", "backward", "left", "right"]
    leaning = [n for n, f in zip(tilt_names, att[:4]) if f]
    msg = "Battery at %s percent. Pitch %d degrees, roll %d degrees. %s" % (
        p[0], round(pitch), round(roll), ("Tilted " + " and ".join(leaning) + ".") if leaning else "Level.")
    bot.say(msg)
    return angle is not None


def fetch_url_text(url, limit=1500):
    """If a QR holds a link, grab the page's visible text (5 s max) so it can be decoded too."""
    try:
        import urllib.request
        with urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": "PLASMA"}), timeout=5) as r:
            html = r.read(200000).decode("utf-8", "ignore")
        txt = re.sub(r"(?is)<(script|style).*?</\1>", " ", html)
        txt = re.sub(r"<[^>]+>", " ", txt)
        return re.sub(r"\s+", " ", txt).strip()[:limit]
    except Exception as e:
        log("could not open QR link: %s" % e)
        return None


def read_one_qr(bot, done_texts, done_tags):
    """Find a marker whose QR we have NOT read yet, drive up to it, read it QR_READS times. Returns text/None."""
    want = CFG["QR_MUST_CONTAIN"]

    def det_qr():
        q = [x for x in bot.qrcodes() if x["text"] and x["text"] not in done_texts]
        if want:
            q = [x for x in q if want.lower() in x["text"].lower()]
        return q[0] if q else None

    def det_any():
        q = det_qr()
        if q:
            return q
        tags = [t for t in bot.apriltags() if t["id"] not in done_tags]
        if tags:
            return tags[0]
        spots = bot.qr_spots()                         # QR visible but too far to decode (up to ~1 m)
        return spots[0] if spots else None

    hit = scan_for(bot, det_any)
    if not hit:
        return None
    if "text" not in hit:
        log("saw %s at a distance, driving closer to read the QR" %
            ("a QR code" if hit.get("id") == "qr-spot" else "AprilTag %s" % hit.get("id")))
        approach_visual(bot, det_any, CFG["QR_APPROACH_STOP_CM"], stop_area=CFG["TARGET_STOP_AREA"], timeout=20,
                        stop_when=lambda h: "text" in h)      # stop as soon as the QR is readable
    reads = []
    t_end = time.time() + 20
    creep_until = time.time() + 4                               # never creep blind for more than 4 s
    backed_off = False
    while time.time() < t_end:
        q = det_qr()
        if q:
            reads.append(q["text"])
            if reads.count(q["text"]) >= CFG["QR_READS"]:
                break
            c = center_turn_speed(q["cx"])
            bot.drive(turn=c) if abs(c) > 8 else bot.stop()
        else:
            spots = bot.qr_spots()                              # QR visible but not readable yet
            if spots and spots[0]["area"] > CFG["QR_TOO_CLOSE_AREA"] and not backed_off:
                log("QR fills the view (too close for the low camera) - backing off 8 cm")
                bot.move_cm(-8, 8)
                backed_off = True
            elif spots:
                bot.drive(fwd=6, turn=center_turn_speed(spots[0]["cx"]))   # steer at it and close in
            elif time.time() < creep_until:
                bot.drive(fwd=5)                                # nothing located: creep a little
            else:
                bot.stop()
        sleep(0.1)
    bot.stop()
    # remember AprilTags on this marker so the next search skips it
    done_tags.update(t["id"] for t in bot.apriltags())
    if not reads:
        return None
    return max(set(reads), key=reads.count)


def m4_decode(bot, args):
    """Scan 1-3 QR markers. Each tells us which colour to commit to (maybe encrypted / decoys).
    Stops early when QR_MIN_AGREE fragments agree; otherwise reads up to QR_MAX or until the time budget."""
    bot.arm("ARM_LOOK")
    bot.load("apriltag_qrcode")
    bot.say("Searching for evidence markers.", wait=False)
    t_end = time.time() + CFG["QR_TIME_BUDGET_S"]
    done_texts = {fr["raw"] for fr in load_evidence()["fragments"]}
    done_tags = set()
    new = 0
    while new < CFG["QR_MAX"] and time.time() < t_end:
        text = read_one_qr(bot, done_texts, done_tags)
        if not text:
            log("no further unread markers found")
            break
        new += 1
        done_texts.add(text)
        text = mask_secret(text)                             # secrets never reach logs, speech or evidence.json
        log("QR %d RAW: %r  (read by %s decoder)" % (new, text, (STATE.get("qr") or [{}])[0].get("src", "?")))
        bot.quip("decrypt")
        ev = add_fragment(text, "QR marker %d" % new)
        fr = next((f for f in ev["fragments"] if f["raw"] == mask_secret(text)), ev["fragments"][-1])
        log("decoded (%s): %s" % (fr["method"], fr["decoded"]))
        for n, v in fr["top"]:
            log("   candidate %-14s %s" % (n, v))
        spoken = "a Wi-Fi network code, no colour clue" if WIFI_BLOCK.search(text) else fr["decoded"]
        bot.say("QR message %d says: %s" % (new, spoken[:200]))       # read the instruction out in full
        if re.match(r"https?://", text.strip(), re.I):
            page = fetch_url_text(text.strip())
            if page:
                page = mask_secret(page)
                log("QR is a link -> page text added as evidence: %r" % page[:200])
                ev = add_fragment(page, "web page from QR %d" % new)
        c, votes, reasons = color_from_evidence(ev["fragments"])
        log("rule-based votes after %d fragment(s): %s -> %s" % (len(ev["fragments"]), votes, c))
        if c and votes[c] >= CFG["QR_MIN_AGREE"]:
            log("%d fragments agree on %s -> enough evidence" % (int(votes[c]), c))
            break
        # back off and turn away so the next scan finds a different marker
        bot.move_cm(-20, 20)
        bot.turn_deg(60, CFG["SEARCH_TURN_SPEED"] * 2)
    ev = load_evidence()
    if not ev["fragments"]:
        bot.say("No evidence markers could be read.")
        return False
    g = run_gemma(bot, ev)
    if g and g.get("decoded"):
        log("Gemma 4 plaintext: %s" % g["decoded"])
    c2, why = chosen_core(args)
    log("COMMITTED CORE: %s (%s)" % (c2, why))
    bot.say("Evidence analysed. Committing to the %s core." % c2)
    return True


def run_gemma(bot, ev):
    """Gemma 4 decrypts + reasons over ALL fragments; result is stored in evidence.json."""
    if not (bot.gemma and bot.gemma.ok and ev["fragments"]):
        return None
    bot.quip("gemma")
    g = bot.gemma.analyse_evidence(ev["fragments"])
    if g:
        g["decoded"] = [redact(x) for x in g["decoded"]]     # in case Gemma decodes a secret the rules missed
        g["reasoning"] = redact(g["reasoning"])
        log("GEMMA 4: core=%s conf=%.2f decoded=%s" % (g["core"], g["confidence"], g["decoded"]))
        log("GEMMA 4 reasoning: %s" % g["reasoning"])
        ev = load_evidence()
        ev["gemma"] = g
        save_evidence(ev)
    return g


def pick_core(bot, color, from_chest=False):
    """Find + grab the ping-pong-ball core. from_chest=True reaches down into the containment chest (m6)."""
    bot.load("color_recognition")
    bot.arm("ARM_LOOK")
    bot.clamp_open()

    def det():
        b = bot.balls(color)
        return b[0] if b else None

    bot.say("Searching for the %s core." % color, wait=False)
    for attempt in range(2):                       # one re-scan if the ball is lost on the way in
        if not scan_for(bot, det):
            bot.say("%s core not found." % color)
            return False
        if from_chest:   # the chest wall stops us; the distance sensor sees the chest, not the ball
            ok = approach_visual(bot, det, CFG["CHEST"].get("stop_cm", 12), stop_area=CFG["CORE_GRAB_AREA"] * 2,
                                 timeout=25)
        else:
            ok = approach_visual(bot, det, CFG["CORE_GRAB_CM"] + 4, stop_area=CFG["CORE_GRAB_AREA"],
                                 stop_y=CFG["CORE_GRAB_Y"], timeout=25)
        if ok:
            break
        log("lost the %s core during approach - rescanning" % color)
    else:
        bot.say("%s core lost." % color)
        return False
    # final alignment: strafe (not rotate) so the ball ends up dead centre between the jaws
    for _ in range(30):
        b = det()
        if not b:
            break
        err = b["cx"] - CFG["CAM_W"] / 2.0
        if abs(err) < CFG["CENTER_TOL_PX"] * 0.6:
            break
        bot.drive(strafe=clamp(err * 0.04, -8, 8))
        sleep(0.06)
    bot.stop()
    bot.quip("grab")
    bot.arm("ARM_CHEST_PREGRAB" if from_chest else "ARM_PREGRAB")
    bot.clamp_open()
    bot.arm("ARM_CHEST_GRAB" if from_chest else "ARM_GRAB")
    # no distance-based creeping here: the sensor usually looks over a ping pong ball at the wall behind
    bot.move_cm(CFG["CORE_FINAL_PUSH_CM"], 6)
    bot.clamp_close()
    bot.arm("ARM_CARRY")
    bot.holding = True
    bot.quip("got")
    return True


def place(bot, pose):
    bot.arm(pose)
    bot.clamp_open()
    bot.holding = False
    bot.move_cm(-6, 10)
    bot.arm("ARM_LOOK")


def m5_secure(bot, args):
    color, why = chosen_core(args)
    log("CORE CHOICE: %s (%s)" % (color, why))
    bot.say("Committing to the %s core." % color)
    if not pick_core(bot, color):
        bot.quip("fail")
        return False
    bot.move_cm(-10, 15)
    bot.quip("carry")
    if go_to_target(bot, CFG["CHEST"], "containment chest"):
        place(bot, "ARM_CHEST")
        bot.core_in_chest = True
        bot.quip("drop", wait=True)
        bot.say("Core deposited in the containment chest.")
        return True
    bot.quip("fail")
    return False


def m6_restart(bot, args):
    color, why = chosen_core(args)
    if not bot.holding:
        src = CFG["CORE_SOURCE_M6"]
        from_chest = src == "chest" or (src == "auto" and getattr(bot, "core_in_chest", False))
        log("m6: retrieving the %s core from the %s" % (color, "containment chest" if from_chest else "floor"))
        if not pick_core(bot, color, from_chest=from_chest):
            bot.quip("fail")
            return False
        bot.move_cm(-10, 15)
    bot.quip("carry")
    if not go_to_target(bot, CFG["POWER_DOCK"], "power dock"):
        bot.quip("fail")
        return False
    bot.quip("dock", wait=True)
    place(bot, "ARM_DOCK")
    bot.say("Core placed in the power dock. Restart sequence initiated.")
    ev = load_evidence()
    c, votes, reasons = color_from_evidence(ev["fragments"])
    log("EVIDENCE CHAIN for the judges:")
    for fr in ev["fragments"]:
        log("  [%s] raw=%r  method=%s  decoded=%r" % (fr.get("source"), fr["raw"], fr["method"], fr["decoded"]))
    for r in reasons:
        log("  " + r)
    if ev.get("gemma"):
        log("  Gemma 4: %s" % ev["gemma"].get("reasoning"))
    log("  => chosen %s (%s)" % (color, why))
    # final reveal: restoration vs containment failure, judged by whether the evidence backs the core we placed
    g = ev.get("gemma") or {}
    backed = (c == color) or (g.get("core") == color and g.get("confidence", 0) >= 0.5)
    contradicted = (c is not None and c != color) or (g.get("core") in COLORS and g.get("core") != color
                                                      and g.get("confidence", 0) >= 0.6)
    if backed and not contradicted:
        bot.lights(0, 255, 0, 2)
        bot.say("Energy signature confirmed. Vault restoration successful.")
    elif contradicted:
        bot.lights(255, 0, 0, 3)
        bot.say("Warning. Energy signature mismatch. Containment failure risk.")
    else:
        bot.lights(255, 160, 0, 2)
        bot.say("Core inserted. Energy signature unverified. Awaiting vault response.")
    # the reveal: how the evidence identified the core (Gemma 4 writes it, ElevenLabs speaks it)
    line = None
    if bot.gemma and bot.gemma.ok:
        line = bot.gemma.reveal_line(color, ev["fragments"], ev.get("gemma", {}).get("reasoning", why))
    bot.say(redact(line or ("The evidence identified the %s core: %s" % (color, why[:150]))))
    bot.quip("done", wait=True)
    return True


MISSIONS = {"m1": m1_activation, "m2": m2_navigation, "m3": m3_comms,
            "m4": m4_decode, "m5": m5_secure, "m6": m6_restart}


# =====================================================================================
# Tools
# =====================================================================================
def tool_sensors(bot, args):
    bot.load("line_recognition", "apriltag_qrcode", "color_recognition")
    bot.got.set_track_recognition_line(CFG["LINE_TYPE"])
    try:
        bot.got.enable_audio_direction()
    except Exception:
        pass
    while True:
        off, typ = bot.line()
        g = bot.got.read_gyro_data()
        try:
            a = bot.got.get_audio_direction()
        except Exception:
            a = "?"
        log("dist=%s  line(off=%s type=%s)  pitch=%.1f roll=%.1f yaw=%.1f  battery=%s  sound=%s" %
            (bot.dist(), off, typ, g[0], g[1], g[2], bot.got.get_power_status(), a))
        log("   colours=%s" % [(b["color"], int(b["cx"]), int(b["cy"]), int(b["area"])) for b in bot.colors()[:3]])
        log("   qr=%s  tags=%s" % ([q["text"] for q in bot.qrcodes()], [(t["id"], t["dist"]) for t in bot.apriltags()]))
        sleep(0.5)
        if MOCK:
            return True


def tool_linetest(bot, args):
    bot.load("line_recognition")
    bot.got.set_track_recognition_line(CFG["LINE_TYPE"])
    print("Put tape to the robot's RIGHT side of centre: offset should be one sign; LEFT the other sign.")
    print("If the robot then steers away from the tape, set LINE_SIGN = -1.")
    for _ in range(200 if not MOCK else 2):
        off, typ = bot.line()
        log("offset=%s type=%s (0 none,1 line,2 Y-fork,3 cross) -> turn cmd %+.0f deg/s (+ = left)" %
            (off, typ, -CFG["LINE_SIGN"] * CFG["LINE_KP"] * off))
        sleep(0.2)
    return True


def tool_arm(bot, args):
    for pose in ("ARM_LOOK", "ARM_PREGRAB", "ARM_GRAB", "ARM_CARRY", "ARM_CHEST", "ARM_DOCK", "ARM_LOOK"):
        bot.arm(pose, 1200)
        if pose == "ARM_GRAB":
            bot.clamp_close()
            bot.clamp_open()
        sleep(0.8)
    return True


def tool_see(bot, args):
    """What does the robot see right now? colours / QR / tags / distance x10, saves see_frame.jpg. No motion."""
    bot.load("color_recognition", "apriltag_qrcode")
    sleep(1.0)
    for _ in range(10 if not MOCK else 1):
        cols = [(b["color"], b.get("shape"), int(b["cx"]), int(b["cy"]), int(b["area"])) for b in bot.colors()[:4]]
        log("dist=%s  colours(color,shape,cx,cy,area)=%s  qr=%s  tags=%s" %
            (bot.dist(), cols, [q["text"] for q in bot.qrcodes()], [(t["id"], t["dist"]) for t in bot.apriltags()]))
        sleep(0.4)
    try:
        bot.got.open_camera()
        jpg = bot.got.read_camera_data()
        if jpg:
            open("see_frame.jpg", "wb").write(jpg)
            log("camera frame saved to see_frame.jpg")
    except Exception as e:
        log("camera frame failed: %s" % e)
    return True


ARM_SERVOS = [51, 52, 53]          # UGOT_B2E8 arm joints (from the peripheral list)
TEACH_FILE = "arm_teach.json"


def _ball_any(bot, color=None):
    """Biggest red/green/blue/orange blob from the camera (or one colour)."""
    import tape_eye
    try:
        if not getattr(bot, "_cam_open", False):
            bot.got.open_camera()
            bot._cam_open = True
        jpg = bot.got.read_camera_data()
    except Exception:
        return None
    if not jpg:
        return None
    blobs = []
    for c in ([color] if color else ["red", "green", "blue", "orange", "black", "white"]):
        if c == "black":
            blobs += tape_eye.black_blobs(jpg)
        elif c == "white":
            blobs += tape_eye.white_blobs(jpg)
        else:
            blobs += tape_eye.color_blobs(jpg, c)
    # only balls on the floor close in front (centre low in the picture): background cans / bags are ignored
    blobs = [b for b in blobs if b["area"] >= 150 and b["cy"] >= 250]
    blobs.sort(key=lambda b: b["area"], reverse=True)
    return blobs[0] if blobs else None


def _servo_pose(bot, pose, ms=1000):
    for sid in ARM_SERVOS:
        bot.got.turn_servo_angle(sid, int(pose[str(sid)]), ms)
    sleep(ms / 1000.0 + 0.3)
    t_end = time.time() + 1.5          # loaded servos lag the command: wait until the arm has really arrived
    while time.time() < t_end:
        try:
            now = bot.got.read_servo_angle(ARM_SERVOS)
            if all(abs(float(now[str(s)]) - float(pose[str(s)])) <= 4 for s in ARM_SERVOS):
                break
        except Exception:
            break
        sleep(0.1)


BOX_BALL_X = 330          # where the held ball hangs in the image (x) at carry height
BOX_FRONT_STOP_CM = 13    # box front edge distance when the ball (~18 cm ahead of the camera) is over the box


def _drop_in_box(bot):
    """Turn until the box is under the held ball, creep in with the camera, open the clamp. False = no box seen."""
    import tape_eye
    if not getattr(bot, "_cam_open", False):
        try:
            bot.got.open_camera()
            bot._cam_open = True
        except Exception:
            pass
    lost, t_end = 0, time.time() + 40
    while time.time() < t_end:
        try:
            jpg = bot.got.read_camera_data()
        except Exception:
            jpg = None
        b = tape_eye.box_blob(jpg) if jpg else None
        if not b:
            lost += 1
            if lost > 8:
                return False
            bot.got.mecanum_turn_speed_times(3, 20, 10, 2)      # box not in view: look right 10 deg at a time
            sleep(0.9)
            continue
        lost = 0
        ex = b["cx"] - BOX_BALL_X
        d = 660.0 / max(0.5, b["bottom"] / 4.0 - 61.0)
        log("[DATA] BOX cx %.0f (want %d)  front %.1f cm  area %d" % (b["cx"], BOX_BALL_X, d, b["area"]))
        if abs(ex) > 30:
            deg = max(3, min(20, abs(ex) * 0.06))
            bot.got.mecanum_turn_speed_times(3 if ex > 0 else 2, 15, int(round(deg)), 2)
            sleep(0.25 + deg / 15.0)
            continue
        if b["bottom"] >= 474 or d <= BOX_FRONT_STOP_CM + 1:
            break
        step = max(2, min(8, d - BOX_FRONT_STOP_CM))
        bot.got.mecanum_move_speed_times(0, 10, int(round(step)), 1)
        sleep(0.3 + step / 10.0)
    bot.stop()
    bot.clamp_open()
    sleep(0.8)
    return True


def tool_teach(bot, args):
    """Teach the arm BY HAND: ball view, grab pose, carry pose, drop-over-box pose -> arm_teach.json."""
    bot.arm("ARM_LOOK")
    bot.clamp_open()
    bot.say("Teaching mode. Put the ball on the floor in front of me, where my claw can reach it.", wait=True)
    sleep(5)
    ball = None
    for _ in range(6):
        ball = _ball_any(bot) or ball
        sleep(0.3)
    if not ball:
        bot.say("I can't see a ball. Put it on the floor right in front of my camera.")
        return False
    log("TEACH ball view: colour=%s cx=%.0f cy=%.0f area=%.0f" % (ball["color"], ball["cx"], ball["cy"], ball["area"]))
    data = {"ball": {k: ball[k] for k in ("color", "cx", "cy", "area")}}
    steps = [("grab", "Hold my arm, I am relaxing it. Move the open claw down so the jaws are around the ball.", 15),
             ("carry", "Now lift the claw up to carrying height.", 10),
             ("drop", "Now hold the claw over the box.", 10)]
    for name, msg, secs in steps:
        bot.say(msg, wait=True)
        bot.got.stop_servo(ARM_SERVOS, lock=False)            # weak lock: you can move it by hand
        for k in range(secs, 0, -1):
            log("  %s pose: %d s ..." % (name, k))
            sleep(1)
        ang = bot.got.read_servo_angle(ARM_SERVOS) or {}
        pose = {str(k): int(v) for k, v in ang.items()}
        if len(pose) < 3:
            bot.say("I could not read my arm angles.")
            return False
        _servo_pose(bot, pose, 300)                            # hold it there
        data[name] = pose
        log("TEACH %s pose (servo angles): %s" % (name, pose))
        bot.say("Got it.", wait=True)
    with open(TEACH_FILE, "w") as f:
        json.dump(data, f, indent=2)
    log("saved %s: %s" % (TEACH_FILE, data))
    _servo_pose(bot, data["carry"])
    bot.arm("ARM_LOOK")
    bot.say("Teaching complete.")
    return True


def tool_pickbox(bot, args):
    """Find the ball, line up exactly like the taught view, grab, lift, drop it in the box (taught poses)."""
    try:
        t = json.load(open(TEACH_FILE))
    except Exception:
        bot.say("Teach me first. Run the teach test.")
        return False
    color = (args.core or t["ball"]["color"]).lower()
    tb = t["ball"]
    bot.arm("ARM_LOOK")
    bot.clamp_open()
    bot.say("Searching for the %s ball." % color, wait=False)
    if not scan_for(bot, lambda: _ball_any(bot, color)):
        bot.say("I can't find the %s ball." % color)
        return False
    # line up so the ball sits exactly where it was when I was taught (strafe + forward/back, small steps)
    good, t_end = 0, time.time() + 30
    while time.time() < t_end:
        b = _ball_any(bot, color)
        if not b:
            bot.stop()
            good = 0
            sleep(0.1)
            continue
        ex, ey = b["cx"] - tb["cx"], tb["cy"] - b["cy"]          # ey > 0: ball too high in image = too far
        log("[DATA] ALIGN ball cx %.0f (target %.0f)  cy %.0f (target %.0f)  area %.0f" % (b["cx"], tb["cx"], b["cy"], tb["cy"], b["area"]))
        if abs(ex) < 14 and abs(ey) < 10:
            bot.stop()
            good += 1
            if good >= 2:
                break
            sleep(0.15)
            continue
        good = 0
        fwd = clamp(ey * 0.08, -5, 6) if abs(ey) >= 10 else 0.0
        side = clamp(ex * 0.04, -5, 5) if abs(ex) >= 14 else 0.0
        if side:                                       # motors ignore < ~1 cm/s: always nudge at least 3 cm/s
            side = 3.0 if side > 0 else -3.0 if abs(side) < 3 else side
        if fwd:
            fwd = 3.0 if fwd > 0 else -3.0 if abs(fwd) < 3 else fwd
        bot.drive(fwd=fwd, strafe=side)
        sleep(0.12)
    bot.stop()
    for attempt in (1, 2):
        bot.quip("grab")
        bot.clamp_open()
        _servo_pose(bot, t["grab"], 1200)
        bot.clamp_close()
        _servo_pose(bot, t["carry"], 1000)
        b = _ball_any(bot, color)
        still_there = b and abs(b["cx"] - tb["cx"]) < 40 and abs(b["cy"] - tb["cy"]) < 40
        if not still_there:
            break
        log("ball is still on the floor - grab missed (attempt %d)" % attempt)
        if attempt == 2:
            bot.say("I missed the ball twice. The grab pose needs re-teaching.")
            bot.arm("ARM_LOOK")
            return False
        bot.say("Missed. Trying again.", wait=False)
    bot.quip("got")
    bot.say("Taking the core to the box.", wait=False)
    if not _drop_in_box(bot):                      # no box in view: fall back to the taught drop pose
        _servo_pose(bot, t["drop"], 1200)
        bot.clamp_open()
        sleep(0.8)
    bot.say("Ball delivered to the box.")
    _servo_pose(bot, t["carry"], 1000)
    bot.arm("ARM_LOOK")
    return True


def tool_handbox(bot, args):
    """Hand-loaded grip test: claw down + open, a teammate puts the ball in the jaws, close, lift, drop it in the box."""
    try:
        t = json.load(open(TEACH_FILE))
    except Exception:
        bot.say("Teach me first. Run the teach test.")
        return False
    bot.arm("ARM_LOOK")
    bot.clamp_open()
    _servo_pose(bot, t["grab"], 1200)
    bot.say("Claw down. Put the ball between my jaws. Closing in ten seconds.", wait=True)
    for n in range(10, 0, -1):
        log("  closing clamp in %d s ..." % n)
        sleep(1.0)
    bot.quip("grab")
    bot.clamp_close()
    _servo_pose(bot, t["carry"], 1000)
    bot.quip("got")
    bot.say("Taking the core to the box.", wait=False)
    if _drop_in_box(bot):
        bot.say("Ball delivered to the box.")
    else:
        log("box not seen - releasing at the taught drop pose")
        bot.say("I can't see the box. Releasing the core here.")
        _servo_pose(bot, t["drop"], 1200)
        bot.clamp_open()
        sleep(0.8)
    _servo_pose(bot, t["carry"], 1000)
    bot.arm("ARM_LOOK")
    return True


def tool_follow(bot, args):
    """Follow the black tape and STOP before the (blue) wall: camera blue-wall + IR + bump failsafes."""
    bot.arm("ARM_LOOK")
    CFG["ROUTE_MAX_S"] = args.time_limit or 600
    res = follow_tape_camera(bot, stop_on_wall=True, bump=True)
    log("follow result: %s" % res)
    if res in ("bluewall", "wall", "bump"):
        d = STATE.get("blue_wall_cm") if res == "bluewall" else (bot.dist() if res == "wall" else CFG["BUMP_BACKOFF_CM"])
        if d is None or not 0 < d < 60:
            d = 20
        bot.say("Calibration bay reached. Wall distance %d centimetres. Calibration complete." % round(d))
    return res != "no_tape"


def tool_wall(bot, args):
    """Wall-stop test: drive straight at the wall, stop 15-25 cm away (target 20), announce the distance."""
    bot.quip("wall")
    d = drive_to_wall(bot, CFG["WALL_TARGET_CM"], CFG["WALL_TOL_CM"])
    sleep(0.3)
    d2 = bot.dist() or d
    if d2 is None:
        bot.say("Distance sensor has no reading.")
        return False
    ok = 15 <= d2 <= 25
    bot.say("Wall distance %d centimetres. %s" % (round(d2), "Calibration complete." if ok else "Out of range."))
    return ok


def tool_grab(bot, args):
    """Pickup test: find the core colour (--core, default red), grab it, lift it, put it back down."""
    color = (args.core or "red").lower()
    ok = pick_core(bot, color)
    if ok:
        sleep(1.5)
        bot.say("Lift confirmed. Putting it back.", wait=False)
        bot.arm("ARM_GRAB")
        bot.clamp_open()
        bot.holding = False
        bot.move_cm(-8, 10)
        bot.arm("ARM_LOOK")
    return ok


def tool_voicetest(bot, args):
    """Finds the robot audio-upload URL and plays every catchphrase through the robot."""
    if bot.voice and bot.voice.enabled:
        p = bot.voice.generate("Team Plasma voice check. One, two, three.")
        if p:
            bot.voice.probe_upload(p)
    for k in PHRASES:
        bot.quip(k, wait=True)
    return True


def tool_say(bot, args):
    """Type a line, robot says it with the ElevenLabs voice. Empty line quits."""
    while True:
        t = input("say> ").strip() if not MOCK else ""
        if not t:
            return True
        bot.say(t)


TOOLS = {"sensors": tool_sensors, "linetest": tool_linetest, "arm": tool_arm,
         "voicetest": tool_voicetest, "say": tool_say, "see": tool_see, "grab": tool_grab, "wall": tool_wall,
         "follow": tool_follow, "teach": tool_teach, "pickbox": tool_pickbox, "handbox": tool_handbox}


def all_fixed_lines():
    lines = ["Team %s." % CFG["TEAM_NAME"].capitalize()] + [l for ls in PHRASES.values() for l in ls]
    lines += ["Emergency activation complete. Systems online. We are team %s." % CFG["TEAM_NAME"],
              "Core deposited in the containment chest.",
              "Core placed in the power dock. Restart sequence initiated."]
    lines += ["Committing to the %s core." % c for c in COLORS]
    lines += ["Evidence analysed. Committing to the %s core." % c for c in COLORS]
    lines += ["Searching for the %s core." % c for c in COLORS]
    lines += ["%s core not found." % c for c in COLORS] + ["%s core lost." % c for c in COLORS]
    lines += ["Calibration bay reached. Wall distance %d centimetres. Calibration complete." % d for d in range(15, 26)]
    lines += ["No sound detected.", "No evidence markers could be read.",
              "Calibration bay reached. Distance sensor has no reading."]
    lines += ["Searching for evidence markers.", "Energy signature confirmed. Vault restoration successful.",
              "Warning. Energy signature mismatch. Containment failure risk.",
              "Core inserted. Energy signature unverified. Awaiting vault response."]
    return lines


# =====================================================================================
# Main
# =====================================================================================
def connect(args):
    global MOCK
    if args.mock:
        MOCK = True
        from mock_ugot import MockUGOT
        return MockUGOT(), "127.0.0.1"
    from ugot import ugot
    got = ugot.UGOT()
    ip = args.ip
    if ip == "scan":
        log("scanning for UGOT on the network...")
        devs = got.scan_device()
        log("found: %s" % devs)
        if not devs:
            sys.exit("No robot found. Use --ip (shown in the UGOT app / robot screen).")
        ip = list(devs.values())[0]
    log("connecting to robot at %s" % ip)
    got.initialize(ip)
    return got, ip


def main():
    ap = argparse.ArgumentParser(description="Team PLASMA - Reboot the Vault")
    ap.add_argument("--ip", default=DEFAULT_IP, help="robot IP (default %s, or 'scan')" % DEFAULT_IP)
    ap.add_argument("--mission", default="all",
                    help="m1..m6, all, final, sensors, linetest, arm, voicetest, say, or comma list e.g. m4,m5")
    ap.add_argument("--no-eleven", action="store_true", help="use the robot's built-in voice only")
    ap.add_argument("--no-gemma", action="store_true", help="skip the Gemma 4 evidence analysis")
    ap.add_argument("--core", choices=COLORS, help="force the core colour (team's decision)")
    ap.add_argument("--evidence", action="append", default=[],
                    help="add an evidence fragment text manually (e.g. from the intermission broadcast)")
    ap.add_argument("--team", default=None, help="team name to announce")
    ap.add_argument("--mock", action="store_true", help="simulate the robot (no hardware)")
    ap.add_argument("--wait-start", action="store_true", help="press Enter to start (the authorised start command)")
    ap.add_argument("--time-limit", type=float, default=0, help="hard stop after N seconds (final uses 175)")
    ap.add_argument("--fresh", action="store_true", help="delete evidence.json first (clean start for a real run)")
    ap.add_argument("--any-shape", action="store_true", help="pick up any shape of the core colour (e.g. a Coke can)")
    ap.add_argument("--qr-max", type=int, default=None, help="read at most N different QR markers (default 3)")
    ap.add_argument("--repeat", type=int, default=1, help="run the mission list N times in a row")
    ap.add_argument("--chest", default=None, help="chest target: wall | tag:N | qr:TEXT | color:NAME")
    ap.add_argument("--dock", default=None, help="power dock target: wall | tag:N | qr:TEXT | color:NAME")
    ap.add_argument("--dashboard", action="store_true", help="live web dashboard on http://localhost:8765")
    ap.add_argument("--camera", action="store_true", help="stream the robot camera into the dashboard")
    ap.add_argument("--lan", action="store_true", help="make the dashboard reachable from a phone on the same Wi-Fi")
    args = ap.parse_args()
    for s in (sys.stdout, sys.stderr):          # unicode QR text must never crash logging on Windows
        try:
            s.reconfigure(errors="backslashreplace")
        except Exception:
            pass
    if args.mock:
        CFG["EVIDENCE_FILE"] = "evidence_mock.json"   # practice data never leaks into real runs
    if args.fresh and os.path.exists(CFG["EVIDENCE_FILE"]):
        os.remove(CFG["EVIDENCE_FILE"])
        log("evidence file cleared (--fresh)")
    if args.team:
        CFG["TEAM_NAME"] = args.team
    if args.qr_max:
        CFG["QR_MAX"] = args.qr_max
    if args.any_shape:
        CFG["CORE_SHAPE"] = None
        CFG["CORE_GRAB_AREA"] = 30000          # a can is bigger than a ping pong ball

    def target(s, stop_cm):
        kind, _, val = s.partition(":")
        spec = {"wall": {"type": "wall"}, "tag": {"type": "apriltag", "id": int(val) if val else None},
                "qr": {"type": "qr", "text": val}, "color": {"type": "color", "color": val}}[kind]
        spec["stop_cm"] = stop_cm
        if kind != "wall":
            spec["fallback"] = {"type": "wall", "stop_cm": stop_cm}
        return spec
    if args.chest:
        CFG["CHEST"] = target(args.chest, CFG["CHEST"].get("stop_cm", 12))
    if args.dock:
        CFG["POWER_DOCK"] = target(args.dock, CFG["POWER_DOCK"].get("stop_cm", 10))
    for e in args.evidence:
        ev = add_fragment(e, "manual")
        log("evidence added: %s" % ev["fragments"][-1]["decoded"])

    got, ip = connect(args)
    import plasma_ai
    voice = None if args.no_eleven else plasma_ai.Voice(got, ip, mock=MOCK)
    gemma = None if args.no_gemma else plasma_ai.Gemma(enabled=True, mock=MOCK)
    bot = Robot(got, voice, gemma)

    def estop(*_):
        log("!!! EMERGENCY STOP")
        bot.stop()
        os._exit(1)
    signal.signal(signal.SIGINT, estop)

    if voice and voice.enabled:
        voice.prepare(all_fixed_lines())       # cached after the first run
    if gemma and gemma.ok and load_evidence()["fragments"] and not load_evidence().get("gemma"):
        run_gemma(bot, load_evidence())        # analyse manually-added fragments before the run
    bot.find_distance_sensor()
    m = args.mission.lower()
    if args.dashboard:
        import plasma_dashboard
        plasma_dashboard.start(STATE, got, CFG["EVIDENCE_FILE"], lambda: chosen_core(args), camera=args.camera,
                               lan=args.lan)
    # EVERY run starts with the team name, then a catchphrase (team rule)
    bot.say("Team %s." % CFG["TEAM_NAME"].capitalize(), wait=True)
    bot.quip(INTRO_QUIPS[int(time.time()) % len(INTRO_QUIPS)], wait=True)
    if CFG["CHEST"].get("type") == "apriltag" and CFG["CHEST"].get("id") is None and \
            CFG["POWER_DOCK"].get("type") == "apriltag" and CFG["POWER_DOCK"].get("id") is None:
        log("WARNING: CHEST and POWER_DOCK both accept ANY AprilTag - set their ids (see --mission sensors)")
    if m in TOOLS:
        TOOLS[m](bot, args)
        return
    seq = list(MISSIONS) if m == "all" else CFG["FINAL_SEQUENCE"] if m == "final" else [x.strip() for x in m.split(",")]
    seq = seq * max(1, args.repeat)                 # e.g. --mission m3 --repeat 6 : clap 6 times in a row
    # Rule 6: one authorised start command, then fully autonomous (no further input is read)
    if args.wait_start or m == "final":
        input("Robot in the marked start position? Press Enter to START (Ctrl+C = emergency STOP)...")
    start = time.time()
    # Rule 9: max 3 minutes per Final Extraction attempt -> hard watchdog stops the robot in time
    limit = args.time_limit if args.time_limit else (CFG["FINAL_TIME_LIMIT_S"] if m == "final" else 0)
    if limit:
        def watchdog():
            time.sleep(limit)
            log("!!! TIME LIMIT %.0fs reached - stopping robot" % limit)
            bot.stop()
            os._exit(2)
        threading.Thread(target=watchdog, daemon=True).start()
        log("watchdog armed: %.0f s" % limit)
    results = {}
    for name in seq:
        STATE["mission"] = name.upper()
        log("========== MISSION %s ==========" % name.upper())
        try:
            results[name] = bool(MISSIONS[name](bot, args))
        except Exception as e:
            bot.stop()
            log("mission %s crashed: %r" % (name, e))
            results[name] = False
        log("mission %s -> %s   (elapsed %.1fs)" % (name, "OK" if results[name] else "FAILED", time.time() - start))
    bot.stop()
    STATE["mission"] = "DONE"
    STATE["results"] = results
    log("RESULTS: %s  total %.1fs" % (results, time.time() - start))
    if args.dashboard and not MOCK:
        log("dashboard stays open at http://localhost:8765 - Ctrl+C to quit")
        while True:
            bot.dist(samples=1)       # keep the live distance reading fresh
            sleep(0.5)


if __name__ == "__main__":
    main()
