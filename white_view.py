"""Re-record the taught ball view using only the white-ball detector, then patch arm_teach.json."""
import json, os, sys, time
WS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, WS)
os.chdir(WS)
import tape_eye
from ugot import ugot

g = ugot.UGOT()
g.initialize("10.10.53.142")
g.mechanical_move_axis(8, 10, 0, 800)   # ARM_LOOK
time.sleep(1.2)
g.open_camera()
time.sleep(0.5)
hits, last = [], None
for i in range(12):
    jpg = g.read_camera_data()
    if not jpg:
        continue
    last = jpg
    b = tape_eye.white_blobs(jpg)
    print(i, [(x["cx"], x["cy"], x["area"]) for x in b[:3]])
    if b:
        hits.append(b[0])
    time.sleep(0.15)
if last:
    open("teach_view.jpg", "wb").write(last)
if len(hits) < 4:
    print("WHITE BALL NOT FOUND (hits=%d) - frame saved to teach_view.jpg" % len(hits))
    sys.exit(1)
hits.sort(key=lambda x: x["area"])
m = hits[len(hits) // 2]
t = json.load(open("arm_teach.json"))
t["ball"] = {"color": "white", "cx": float(m["cx"]), "cy": float(m["cy"]), "area": int(m["area"])}
json.dump(t, open("arm_teach.json", "w"), indent=1)
print("PATCHED ball view:", t["ball"])
