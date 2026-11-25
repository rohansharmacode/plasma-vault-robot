"""Diagnostic: turn 360 deg in 20 deg steps, save a frame per step, try every QR decoder on it."""
import os, sys, time
WS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, WS)
os.chdir(WS)
import cv2
import numpy as np
from ugot import ugot
from plasma_vault import mask_secret     # never print a Wi-Fi password from a QR

os.makedirs("qrsweep", exist_ok=True)
g = ugot.UGOT()
g.initialize("10.10.53.142")
g.load_models(["apriltag_qrcode"])
g.open_camera()
time.sleep(0.5)
det = cv2.QRCodeDetector()
try:
    aru = cv2.QRCodeDetectorAruco()
except Exception:
    aru = None
for i in range(18):
    time.sleep(0.6)
    res = []
    for k in range(2):
        jpg = g.read_camera_data()
        if not jpg:
            continue
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if k == 0:
            open("qrsweep/step_%02d.jpg" % i, "wb").write(jpg)
        for name, d in (("cv", det), ("aruco", aru)):
            if d is None:
                continue
            for sc in (0.5, 1.0, 2.0):
                im = img if sc == 1.0 else cv2.resize(img, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA if sc < 1 else cv2.INTER_CUBIC)
                try:
                    ok, texts, pts, _ = d.detectAndDecodeMulti(im)
                except Exception as e:
                    ok, texts, pts = False, (), None
                if ok:
                    res.append((name, sc, [mask_secret(t)[:40] for t in texts]))
                try:
                    okl, ptsl = d.detectMulti(im)
                    if okl and ptsl is not None:
                        res.append((name + "-locate", sc, (ptsl / sc).astype(int).tolist()))
                except Exception:
                    pass
    try:
        u = [[mask_secret(q[0])] + list(q[1:]) for q in g.get_qrcode_total_info()]
    except Exception as e:
        u = "err %s" % e
    print("step %02d (%3d deg) ugot=%s  %s" % (i, i * 20, u, res), flush=True)
    g.mecanum_turn_speed_times(2, 30, 20, 2)
    time.sleep(1.0)
