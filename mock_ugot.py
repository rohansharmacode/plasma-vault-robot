"""Fake UGOT so plasma_vault.py can be tested without the robot: python plasma_vault.py --mock --mission all"""
import json
import random


class _R:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Model:
    def __init__(self, sim):
        self.sim = sim

    def mecanum_move_control(self, linear_speed=0, direction=0, rotate_speed=0, time=0, mileage=0, target_angle=0, client_type=0):
        fwd = linear_speed if abs(direction) < 90 else -linear_speed
        self.sim.v = fwd


class _Audio:
    def getDirectionOfAudio(self):
        return _R(code=0, angle=200)

    def playAudioFile(self, name, t):
        return _R(code=0)


class _Vision:
    def __init__(self, sim):
        self.sim = sim

    def color_identify(self):
        s = self.sim
        s.calls += 1
        if s.calls < 3:
            return json.dumps([])
        area = min(80000, 8000 + s.calls * 2500)
        return json.dumps([
            {"color": "blue", "shape": "ball", "position": [300, 300, int(area ** 0.5), int(area ** 0.5)]},
            {"color": "blue", "shape": "square", "position": [10, 10, 300, 300]},
            {"color": "red", "shape": "ball", "position": [50, 200, 60, 60]},
        ])


class MockUGOT:
    def __init__(self):
        self.v = 0
        self.distance = 180.0
        self.calls = 0
        self.line_calls = 0
        self.MODEL = _Model(self)
        self.AUDIO = _Audio()
        self.VISION = _Vision(self)

    # chassis
    def mecanum_stop(self): self.v = 0
    def stop_chassis(self): self.v = 0
    def mecanum_move_speed_times(self, d, s, t, u): self.distance += (-t if d == 0 else t)
    def mecanum_translate_speed_times(self, a, s, t, u): pass
    def mecanum_turn_speed_times(self, d, s, t, u): pass

    # arm servos (teach mode)
    def stop_servo(self, ids, lock=False): pass
    def read_servo_angle(self, ids): return {"51": 10, "52": -40, "53": 30}
    def turn_servo_angle(self, sid, angle, ms, wait=False): pass
    def open_camera(self): pass
    def read_camera_data(self): return None

    # arm
    def mechanical_move_axis(self, r, h, th, ms): assert -5.5 <= r <= 24.9 and -18 <= h <= 18.2
    def mechanical_clamp_release(self): pass
    def mechanical_clamp_close(self): pass

    # io
    def play_audio_tts(self, text, v, wait): pass
    def screen_clear(self): pass
    def screen_print_text_newline(self, t, c): pass
    def screen_display_emotion(self, n, loop=False): pass
    def show_light_rgb_effect(self, r, g, b, e): pass
    def set_volume(self, v): pass
    def get_device_name(self): return "UGOT_MOCK"
    def get_power_status(self): return [87, False, 0]
    def get_peripheral_devices_list(self): return [{"type": "Infrared", "deviceId": "21", "firmware": "1", "serial": "x"}]

    def read_distance_data(self, i):
        if self.v:
            self.distance = max(3.0, self.distance - self.v * 0.05)
        return round(self.distance, 1)

    def read_gyro_data(self): return [1.3, -0.4, 12.0, 0, 0, 0, 0, 0, 9.8]
    def get_attitude(self): return [False] * 7
    def enable_audio_direction(self): pass
    def disable_audio_direction(self): pass
    def get_audio_direction(self): return "Left"

    # vision
    def load_models(self, m): return True
    def set_track_recognition_line(self, t): pass

    def get_single_track_total_info(self):
        self.line_calls += 1
        typ = 2 if self.line_calls == 20 else 1
        return [random.randint(-60, 60), typ, 0, 0]

    def get_color_total_info(self): return ["Blue", "Cube", 320, 300, 100, 100, 10000]
    # three markers: base64 "COMMIT TO THE BLUE CORE", ROT13 "RED IS A DECOY", ROT13 "THE BLUE CORE IS SAFE"
    QRS = ["Q09NTUlUIFRPIFRIRSBCTFVFIENPUkU=", "ERQ VF N QRPBL", "GUR OYHR PBER VF FNSR"]

    def get_qrcode_total_info(self):
        self.qr_calls = getattr(self, "qr_calls", 0) + 1
        return [[self.QRS[min(self.qr_calls // 6, 2)], 330, 240, 120, 120, 14400]]
    def get_apriltag_total_info(self): return [[3, 330.0, 240.0, 60, 60, 3600, 20, 28, 40, 0, 0, 0, 0, 0]]
