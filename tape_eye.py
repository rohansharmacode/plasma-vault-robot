"""TapeEye - our own BLACK-tape detector on raw camera frames (replaces the SDK line model).

Why: the arena has light-brown duct-tape decoys. The SDK line model reports them as lines/crossroads.
Black tape is far darker AND greyer than light-brown tape or concrete, so we classify pixels ourselves:
    black tape  : max(R,G,B) < dark_thr  and  max-min < sat_thr        (dark + colourless)
    brown decoy : bright (~150-200) and orange-ish                       -> rejected
    concrete    : bright grey                                            -> rejected
Predictive: the tape centre is measured in 3 bands (NEAR / MID / FAR ahead of the robot). The FAR-NEAR
difference is the direction the tape is heading, so the controller turns + slows BEFORE a bend.
Continuity gating: in each band we pick the tape run closest to where it was last frame (or the side the
JUNCTION_PLAN asks for at a fork) and reject runs that are too wide to be tape (shoes, legs, shadows).
"""
import io
import threading
import time

from PIL import Image, ImageChops, ImageFilter

W, H = 160, 120                      # analysis resolution (camera is 640x480)

EYE_CFG = {
    "HORIZON_Y": 64,                 # rows above this are not floor (camera looks almost straight ahead)
    # bands (y0, y1) in the 160x120 image, nearest first, with min/max plausible tape width in px.
    # Calibrated on UGOT_B2E8 sitting on the tape: tape is ~48 px wide at the bottom, ~32 mid, ~17 far.
    "BANDS": {"near": (100, 119, 8, 75), "mid": (84, 99, 5, 55), "far": (68, 83, 3, 35)},
    "DARK_FRAC": 0.55,               # dark threshold = this x median floor brightness (adapts to lighting)
    "DARK_MAX": 110,                 # ...but never above this (tape has a grey sheen / glare)
    "SAT_MAX": 45,                   # max(R,G,B)-min(R,G,B) above this = coloured (brown) -> not black tape
    "MIN_ROWS": 0.35,                # a column counts as tape if >= this fraction of band rows are black
    "MAX_JUMP": 0.45,                # max change of a band's centre between frames (normalised -1..1)
    "FETCH_THREADS": 2,
}


def _close_gaps(cols, gap):
    """Fill False gaps up to `gap` px between True runs (glare/shine holes on the tape)."""
    out = list(cols)
    last = None
    for x, v in enumerate(cols):
        if v:
            if last is not None and 1 < x - last <= gap + 1:
                for k in range(last + 1, x):
                    out[k] = True
            last = x
    return out


def _runs(cols, min_w, max_w):
    """Contiguous True runs in a column mask -> list of (centre_px, width)."""
    out, start = [], None
    cols = _close_gaps(cols, max(2, max_w // 5))       # shiny tape glare must not split it into a "fork"
    for x, v in enumerate(cols + [False]):
        if v and start is None:
            start = x
        elif not v and start is not None:
            w = x - start
            if min_w <= w <= max_w:
                out.append(((start + x - 1) / 2.0, w))
            start = None
    return out


def _band_runs(mp, b0, b1, max_w):
    """Row-by-row tape finder (works for THIN and DIAGONAL tape): find dark segments in every row of the band,
    group segments that line up across rows, keep groups present in enough rows.
    -> list of (centre_px, width_px)."""
    segs = []                                          # (x_centre, width, row)
    for y in range(b0, b1 + 1):
        row = [mp[x, y] > 0 for x in range(W)]
        for c, w in _runs(row, 1, max_w):
            segs.append((c, w, y))
    if not segs:
        return []
    segs.sort()
    groups, cur = [], [segs[0]]
    for s in segs[1:]:
        if s[0] - cur[-1][0] <= 6 + cur[-1][1] / 2.0:  # lines up with the previous segment (allows diagonal)
            cur.append(s)
        else:
            groups.append(cur)
            cur = [s]
    groups.append(cur)
    need = max(2, int((b1 - b0 + 1) * EYE_CFG["MIN_ROWS"]))
    out = []
    for g in groups:
        rows = len(set(s[2] for s in g))
        if rows < need:
            continue                                   # specks / dirt: only in a few rows
        xs = sorted(s[0] for s in g)
        ws = sorted(s[1] for s in g)
        out.append((xs[len(xs) // 2], ws[len(ws) // 2]))
    return out


def black_mask(im):
    """RGB image -> 'L' mask (255 = black tape pixel). Fast: PIL channel ops, no per-pixel Python."""
    r, g, b = im.split()
    hi = ImageChops.lighter(ImageChops.lighter(r, g), b)        # max channel = brightness
    lo = ImageChops.darker(ImageChops.darker(r, g), b)
    sat = ImageChops.subtract(hi, lo)                           # colourfulness (brown tape is orange-ish)
    floor = hi.crop((0, EYE_CFG["HORIZON_Y"], W, H)).resize((40, 12))
    vals = sorted(floor.getdata())
    med = vals[len(vals) // 2] if vals else 128
    thr = min(EYE_CFG["DARK_MAX"], int(med * EYE_CFG["DARK_FRAC"]))
    dark = hi.point(lambda v: 255 if v < thr else 0)
    grey = sat.point(lambda v: 255 if v < EYE_CFG["SAT_MAX"] else 0)
    return ImageChops.multiply(dark, grey), thr


def analyse(jpg, prev=None, prefer=None):
    """jpg bytes -> {band: (x_norm, width) or None, 'fork': bool, 'thr': int}
    x_norm: -1 (left edge) .. +1 (right edge) in the image."""
    im = Image.open(io.BytesIO(jpg))
    im.draft("RGB", (320, 240))                                  # fast JPEG downscale while decoding
    im = im.convert("RGB").resize((W, H))
    mask, thr = black_mask(im)
    res = {"thr": thr, "fork": False}
    mp = mask.load()
    norm = lambda c: (c - (W - 1) / 2.0) / ((W - 1) / 2.0)
    for name, (b0, b1, min_w, max_w) in EYE_CFG["BANDS"].items():
        # METHOD 1 (original, proven on the robot): columns that are black in >= MIN_ROWS of the band's rows
        colfrac = list(mask.crop((0, b0, W, b1 + 1)).resize((W, 1), Image.BOX).getdata())
        cols = [v >= 255 * EYE_CFG["MIN_ROWS"] for v in colfrac]
        runs = [(norm(c), w) for c, w in _runs(cols, min_w, max_w)]
        res[name + "_src"] = "col"
        if len(runs) >= 2 and name != "far":     # far band sees clutter near the horizon: not a fork
            res["fork"] = True
        if not runs:
            # METHOD 2 (failsafe): row-by-row segments grouped across rows -> finds THIN / steep diagonal tape
            runs = [(norm(c), w) for c, w in _band_runs(mp, b0, b1, max_w)]
            res[name + "_src"] = "row" if runs else None
        pick = None
        if runs:
            p = (prev or {}).get(name)
            if prefer in ("left", "right") and len(runs) >= 2:
                pick = min(runs, key=lambda r: r[0]) if prefer == "left" else max(runs, key=lambda r: r[0])
            elif p is not None:
                pick = min(runs, key=lambda r: abs(r[0] - p[0]))
                if abs(pick[0] - p[0]) > EYE_CFG["MAX_JUMP"]:
                    pick = None                      # sudden jump = not our tape (shoe, decoy edge, shadow)
            else:
                pick = min(runs, key=lambda r: abs(r[0]))    # first frame: the run nearest the centre
        res[name] = pick
    return res


# ---------------------------------------------------------------------------------------------
# Camera wall / obstacle distance. The camera sits ~10 cm above the floor looking straight ahead,
# so the row where an obstacle meets the floor maps to distance:  d = K / (row - HORIZON).
# Calibrated on UGOT_B2E8: Coke can at 21 cm (IR) touched the floor at row 92.5 of 120 -> K ~ 660.
# ---------------------------------------------------------------------------------------------
WALL_CFG = {"HORIZON": 61.0, "K": 660.0, "BAND": (40, 120), "DIFF": 38, "FRAC": 0.55}


def wall_distance(jpg):
    """-> (distance_cm or None, contact_row). Scans up from the bottom of the centre band for the first rows
    that stop looking like floor (black tape and tan decoy tape count as floor)."""
    im = Image.open(io.BytesIO(jpg))
    im.draft("RGB", (320, 240))
    im = im.convert("RGB").resize((W, H))
    px = im.load()
    # black tape (incl. its shiny glare) is part of the floor: mask it, widened by a few px
    tmask = black_mask(im)[0].filter(ImageFilter.MaxFilter(17)).load()
    x0, x1 = WALL_CFG["BAND"]
    ref = sorted(sum(px[x, y]) // 3 for y in range(112, 120) for x in range(x0, x1, 2))
    floor_v = ref[len(ref) // 2]
    hits = 0
    for y in range(H - 2, int(WALL_CFG["HORIZON"]) + 2, -1):
        bad = 0
        for x in range(x0, x1, 2):
            r, g, b = px[x, y]
            v = (r + g + b) // 3
            hi, lo = max(r, g, b), min(r, g, b)
            tape = tmask[x, y] > 0 or (hi < 100 and hi - lo < 45)   # black tape (and its glare) on the floor
            tan = r > g > b and r - b > 30 and v > 100          # brown decoy tape on the floor
            if not tape and not tan and abs(v - floor_v) > WALL_CFG["DIFF"]:
                bad += 1
        if bad >= WALL_CFG["FRAC"] * ((x1 - x0) // 2):
            hits += 1
            if hits >= 2:
                row = y + 1.5
                return WALL_CFG["K"] / max(0.5, row - WALL_CFG["HORIZON"]), row
        else:
            hits = 0
    return None, None


def blue_wall_distance(jpg):
    """The calibration-bay wall is BLUE. Find the lowest row where the CENTRE of the picture is mostly blue
    (= where the blue wall meets the floor) and convert it to cm with the floor-contact formula.
    The blue banner at the SIDE of the arena is ignored (only the centre 50% of the image counts)."""
    im = Image.open(io.BytesIO(jpg))
    im.draft("RGB", (320, 240))
    hsv = im.convert("RGB").resize((W, H)).convert("HSV")
    px = hsv.load()
    c = COLOR_CFG["blue"]
    x0, x1 = 40, 120
    for y in range(H - 1, int(WALL_CFG["HORIZON"]) - 8, -1):
        n = 0
        for x in range(x0, x1, 2):
            h, s, v = px[x, y]
            if c["hue"][0][0] <= h <= c["hue"][0][1] and s >= c["sat"] and v >= c["val"]:
                n += 1
        if n >= 0.5 * ((x1 - x0) // 2):
            return WALL_CFG["K"] / max(0.5, y + 0.5 - WALL_CFG["HORIZON"])
    return None


# ---------------------------------------------------------------------------------------------
# Colour blobs (energy cores): the UGOT colour model only knows its own balls/cubes and returned
# nothing for a Coke can, so we segment saturated red / green / blue ourselves.
# PIL HSV: hue 0-255 (red wraps around 0), sat 0-255, val 0-255.
# ---------------------------------------------------------------------------------------------
COLOR_CFG = {
    "red":   {"hue": [(0, 9), (243, 255)], "sat": 110, "val": 60},
    "green": {"hue": [(60, 120)], "sat": 80, "val": 40},
    "blue":  {"hue": [(135, 180)], "sat": 90, "val": 40},
    "orange": {"hue": [(10, 26)], "sat": 140, "val": 90},     # orange ping pong ball (only when asked for)
    "MIN_AREA": 12,              # px in the 160x120 analysis image (~ 200 px^2 at 640x480)
    "FLOOR_Y": 55,               # blob must reach below this row (cores sit on the floor, not on walls/banners)
}


def color_blobs(jpg, want=None):
    """jpg -> list of {color, cx, cy, area, w, h, shape} in 640x480 pixel units, biggest first."""
    im = Image.open(io.BytesIO(jpg))
    im.draft("RGB", (320, 240))
    hsv = im.convert("RGB").resize((W, H)).convert("HSV")
    h, s, v = hsv.split()
    out = []
    for name in ([want] if want else ("red", "green", "blue")):
        c = COLOR_CFG[name]
        hm = None
        for lo, hi in c["hue"]:
            part = h.point(lambda x, lo=lo, hi=hi: 255 if lo <= x <= hi else 0)
            hm = part if hm is None else ImageChops.lighter(hm, part)
        sm = s.point(lambda x, t=c["sat"]: 255 if x >= t else 0)
        vm = v.point(lambda x, t=c["val"]: 255 if x >= t else 0)
        mask = ImageChops.multiply(ImageChops.multiply(hm, sm), vm)
        out += [dict(b, color=name) for b in _components(mask)]
    out = [b for b in out if b["area"] >= COLOR_CFG["MIN_AREA"] * 16 and b["bottom"] >= COLOR_CFG["FLOOR_Y"] * 4
           and not b["edge_tall"]]           # tall blob touching the frame edge = wall / banner, not a core
    out.sort(key=lambda b: b["area"], reverse=True)
    return out


def black_blobs(jpg):
    """Round BLACK objects on the floor close in front (e.g. a ~7 cm ball made of black tape).
    Long tape lines (not round) and far shoes/bags (above the near-floor zone) are rejected."""
    im = Image.open(io.BytesIO(jpg))
    im.draft("RGB", (320, 240))
    im = im.convert("RGB").resize((W, H))
    mask, _ = black_mask(im)
    # close the glare holes of shiny / crumpled tape so the ball becomes one solid blob
    mask = mask.filter(ImageFilter.MaxFilter(7)).filter(ImageFilter.MinFilter(7))
    mask.paste(0, (0, 0, W, 62))       # floor only: legs / trousers / bags above the horizon can't merge in
    out = []
    for b in _components(mask):
        ratio = b["w"] / float(b["h"])
        fill = b["area"] / float(b["w"] * b["h"])
        # a ball fills ~78% of its box (a tape wedge ~55%); its centre sits low = on the floor close in front
        if 0.6 < ratio < 1.7 and fill > 0.6 and b["area"] >= 300 and b["cy"] >= 250 \
                and not b["edge_tall"]:
            out.append(dict(b, color="black"))
    out.sort(key=lambda b: b["area"], reverse=True)
    return out


def white_blobs(jpg):
    """Round WHITE objects on the floor close in front (white ping pong ball): clearly brighter than the
    concrete and colourless; round + compact (floor shine is irregular); centre low (white shoes are far)."""
    im = Image.open(io.BytesIO(jpg))
    im.draft("RGB", (320, 240))
    im = im.convert("RGB").resize((W, H))
    r, g, b = im.split()
    hi = ImageChops.lighter(ImageChops.lighter(r, g), b)
    lo = ImageChops.darker(ImageChops.darker(r, g), b)
    sat = ImageChops.subtract(hi, lo)
    vals = sorted(hi.crop((0, EYE_CFG["HORIZON_Y"], W, H)).resize((40, 12)).getdata())
    floor = vals[len(vals) // 2] if vals else 170
    thr = min(250, floor + 50)                 # floor shine reaches ~floor+35-45; a white ball is brighter
    bright = hi.point(lambda v: 255 if v >= thr else 0)
    grey = sat.point(lambda v: 255 if v < 45 else 0)
    raw = ImageChops.multiply(bright, grey)
    # pass 1: smooth round ball (ping pong). pass 2: crumpled paper ball - only its lit top is bright,
    # so accept a flatter, patchier blob (still floor-only, inside the frame, not edge-tall).
    for k, rmax, fmin in ((3, 1.7, 0.6), (5, 2.4, 0.4)):
        mask = raw.filter(ImageFilter.MaxFilter(k)).filter(ImageFilter.MinFilter(k))
        mask.paste(0, (0, 0, W, 62))
        out = []
        for bl in _components(mask):
            ratio = bl["w"] / float(bl["h"])
            fill = bl["area"] / float(bl["w"] * bl["h"])
            inside = bl["cx"] - bl["w"] / 2.0 > 4 and bl["cx"] + bl["w"] / 2.0 < 636 and bl["bottom"] < 476
            if 0.6 < ratio < rmax and fill > fmin and 700 <= bl["area"] <= 60000 and bl["cy"] >= 250 \
                    and inside and not bl["edge_tall"]:   # cut off by the frame edge = floor shine, not a ball
                out.append(dict(bl, color="white"))
        if out:
            break
    out.sort(key=lambda x: x["area"], reverse=True)
    return out


def box_blob(jpg):
    """The drop box (tan / cork container standing on the floor) -> biggest blob or None.
    Must reach below the horizon (stands on the floor); cardboard boxes in the background stay above it."""
    im = Image.open(io.BytesIO(jpg))
    im.draft("RGB", (320, 240))
    hsv = im.convert("RGB").resize((W, H)).convert("HSV")
    h, s, v = hsv.split()
    hm = h.point(lambda x: 255 if 12 <= x <= 40 else 0)
    sm = s.point(lambda x: 255 if 40 <= x <= 180 else 0)
    vm = v.point(lambda x: 255 if x >= 90 else 0)
    mask = ImageChops.multiply(ImageChops.multiply(hm, sm), vm)
    mask.paste(0, (0, 0, W, 50))
    # bottom >= 290 px = floor contact closer than ~55 cm: the tan UBTECH cartons behind (~1.1-1.3 m) are ignored.
    # Nearest (lowest bottom edge) wins, not biggest - a far carton can look bigger than the near box.
    blobs = [b for b in _components(mask) if b["bottom"] >= 290 and b["area"] >= 400]
    blobs.sort(key=lambda b: (b["bottom"], b["area"]), reverse=True)
    return blobs[0] if blobs else None


def _components(mask):
    """Connected blobs in a 160x120 'L' mask (4-neighbour flood fill) -> boxes scaled to 640x480."""
    px = mask.load()
    seen = set()
    blobs = []
    for y in range(H):
        for x in range(W):
            if px[x, y] and (x, y) not in seen:
                stack = [(x, y)]
                seen.add((x, y))
                n, x0, x1, y0, y1 = 0, x, x, y, y
                while stack:
                    cx, cy = stack.pop()
                    n += 1
                    x0, x1, y0, y1 = min(x0, cx), max(x1, cx), min(y0, cy), max(y1, cy)
                    for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                        if 0 <= nx < W and 0 <= ny < H and px[nx, ny] and (nx, ny) not in seen:
                            seen.add((nx, ny))
                            stack.append((nx, ny))
                if n >= 3:
                    w, hh = (x1 - x0 + 1) * 4, (y1 - y0 + 1) * 4
                    fill = n * 16.0 / (w * hh)
                    blobs.append({"cx": (x0 + x1 + 1) * 2.0, "cy": (y0 + y1 + 1) * 2.0, "area": n * 16,
                                  "w": w, "h": hh, "bottom": (y1 + 1) * 4,
                                  "edge_tall": (x0 == 0 or x1 == W - 1) and hh > 2 * w,
                                  "shape": "ball" if 0.6 < w / float(hh) < 1.6 and fill > 0.55 else "other"})
    return blobs


class TapeEye:
    """Background camera reader + analyser. eye.latest -> newest analysis, eye.wait_new() blocks for the next."""

    def __init__(self, got, state=None):
        self.got = got
        self.state = state if state is not None else {}
        self.latest = None
        self.seq = 0
        self.prefer = None
        self.run = False
        self.cv = threading.Condition()
        self._last_frame_t = 0.0
        self._thumb = None
        self.fps = 0.0

    def start(self):
        try:
            self.got.open_camera()
        except Exception:
            pass
        self.run = True
        self.state["eye_active"] = True
        for _ in range(EYE_CFG["FETCH_THREADS"]):
            threading.Thread(target=self._loop, daemon=True).start()
        return self

    def stop(self):
        self.run = False
        self.state["eye_active"] = False

    def _loop(self):
        while self.run:
            t_req = time.time()
            try:
                jpg = self.got.read_camera_data()
            except Exception:
                jpg = None
            if not jpg:
                time.sleep(0.05)
                continue
            with self.cv:
                if t_req < self._last_frame_t:           # an older request finished late: drop it
                    continue
                prev = self.latest
            try:
                a = analyse(jpg, prev, self.prefer)
                a["wall_cm"] = wall_distance(jpg)[0]       # generic obstacle hint (slow down only)
                a["blue_wall_cm"] = blue_wall_distance(jpg)  # the blue calibration-bay wall (stops the robot)
                # image motion: if the wheels drive but the picture stops changing, we are pushing a wall
                # floor directly ahead only (bottom 30%): people walking in the background don't count
                full = Image.open(io.BytesIO(jpg)).convert("L")
                fw, fh = full.size
                thumb = full.crop((0, int(fh * 0.7), fw, fh)).resize((32, 24))
                with self.cv:
                    if self._thumb is not None:
                        d = ImageChops.difference(thumb, self._thumb)
                        a["motion"] = sum(d.getdata()) / (32.0 * 24.0)
                    self._thumb = thumb
            except Exception:
                continue
            with self.cv:
                if t_req < self._last_frame_t:
                    continue
                dt = t_req - self._last_frame_t if self._last_frame_t else 0
                self._last_frame_t = t_req
                if dt > 0:
                    self.fps = 0.8 * self.fps + 0.2 * (1.0 / dt) if self.fps else 1.0 / dt
                a["t"] = t_req
                self.latest = a
                self.seq += 1
                self.state["frame_jpg"] = jpg
                self.state["tape"] = {k: (round(v[0], 3) if v else None) for k, v in a.items()
                                      if k in ("near", "mid", "far")}
                self.state["tape"]["fork"] = a["fork"]
                self.state["tape"]["fps"] = round(self.fps, 1)
                self.cv.notify_all()

    def wait_new(self, last_seq, timeout=0.6):
        with self.cv:
            self.cv.wait_for(lambda: self.seq != last_seq, timeout=timeout)
            return self.latest, self.seq
