"""
PLASMA AI layer
  * ElevenLabs voice: every spoken line is generated with ElevenLabs, uploaded to the robot over
    its IP (HTTP, port 7000) and played on the robot speaker with play_sound_upload().
    Falls back to laptop speaker, then to the robot's built-in TTS, so the robot never goes silent.
  * Gemma 4 (open-weight, runs locally through Ollama): decrypts the QR evidence fragments, picks
    the core with a written justification, and writes the final "how the evidence identified it" reveal.

Setup
    set ELEVENLABS_API_KEY=sk_...            (PowerShell: $env:ELEVENLABS_API_KEY="sk_...")
    ollama pull gemma4                         (any gemma4 tag works, it is auto-detected)
"""

import hashlib
import json
import os
import platform
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
import wave

AI_CFG = {
    "ELEVEN_API_KEY": os.environ.get("ELEVENLABS_API_KEY", ""),
    # premade ElevenLabs voices (not clones of real people). Swap the ID for any voice in your library.
    #   George  JBFqnCBsd6RMkjVDRZzb  warm British narrator (default)
    #   Adam    pNInz6obpgDQGcFmaJgB  deep movie-trailer voice
    #   Brian   nPczCjzI2devNBz1zQrb  deep, resonant
    "ELEVEN_VOICE_ID": os.environ.get("ELEVENLABS_VOICE_ID", "JBFqnCBsd6RMkjVDRZzb"),
    "ELEVEN_MODEL": "eleven_multilingual_v2",
    "ELEVEN_STABILITY": 0.35,           # lower = more dramatic
    "ELEVEN_STYLE": 0.6,
    "SAMPLE_RATE": 22050,               # pcm_22050 -> wrapped as .wav (plays everywhere)
    "CACHE_DIR": "voice_cache",
    # robot upload: first URL that the robot can then play is remembered in voice_cache/endpoint.txt
    "UPLOAD_URL": os.environ.get("UGOT_AUDIO_UPLOAD_URL", ""),   # e.g. http://{ip}:7000/audio/upload
    "UPLOAD_CANDIDATES": [
        "http://{ip}:7000/audio/upload", "http://{ip}:7000/audio/file", "http://{ip}:7000/audio",
        "http://{ip}:7000/upload/audio", "http://{ip}:7000/audio/sound", "http://{ip}:7000/sound/upload",
        "http://{ip}:7000/file/upload", "http://{ip}:7000/upload", "http://{ip}:7000/audio/upload_file",
        "http://{ip}:7000/vision/audio",
    ],
    "UPLOAD_FIELD": "file",
    "LAPTOP_FALLBACK": True,            # if robot upload is impossible, play ElevenLabs audio on the laptop
    "LIVE_TTS_TIMEOUT": 8,              # seconds allowed to generate a new (uncached) line mid-run

    "GEMMA_URL": None,                  # from OLLAMA_HOST, normalised below
    "GEMMA_MODEL": os.environ.get("GEMMA_MODEL", "gemma4"),
    "GEMMA_TIMEOUT": 25,                # never let the LLM eat the 3-minute run; rule-based result is the fallback
}


def _ollama_url():
    h = os.environ.get("OLLAMA_HOST", "127.0.0.1:11434").strip()
    if "://" not in h:
        h = "http://" + h
    h = h.replace("0.0.0.0", "127.0.0.1").rstrip("/")
    if h.count(":") == 1:
        h += ":11434"
    return h


AI_CFG["GEMMA_URL"] = _ollama_url()


def _log(msg):
    print("[ai] " + msg, flush=True)


# =====================================================================================
# ElevenLabs voice
# =====================================================================================
class Voice:
    def __init__(self, got, robot_ip, mock=False):
        self.got = got
        self.ip = robot_ip
        self.mock = mock
        self.enabled = bool(AI_CFG["ELEVEN_API_KEY"]) or mock
        self.uploaded = set()
        self.endpoint = AI_CFG["UPLOAD_URL"].format(ip=robot_ip) if AI_CFG["UPLOAD_URL"] else None
        self.robot_upload_ok = None          # unknown until probed
        self.failed = set()                  # lines ElevenLabs could not generate (don't retry mid-run)
        self.offline = False
        self.lock = threading.Lock()
        if mock:   # never let fake (silent) clips or a fake endpoint leak into the real cache
            AI_CFG["CACHE_DIR"] = AI_CFG["CACHE_DIR"] + "_mock"
        os.makedirs(AI_CFG["CACHE_DIR"], exist_ok=True)
        ep_file = os.path.join(AI_CFG["CACHE_DIR"], "endpoint.txt")
        if not self.endpoint and os.path.exists(ep_file):
            saved = open(ep_file).read().strip()
            if saved:
                self.endpoint = saved.replace("{ip}", robot_ip)
        if not self.enabled:
            _log("ELEVENLABS_API_KEY not set -> robot built-in voice will be used")

    # ---------- ElevenLabs ----------
    def _path(self, text):
        h = hashlib.sha1((AI_CFG["ELEVEN_VOICE_ID"] + "|" + text).encode()).hexdigest()[:12]
        return os.path.join(AI_CFG["CACHE_DIR"], "plasma_%s.wav" % h)

    def generate(self, text, timeout=30):
        path = self._path(text)
        if os.path.exists(path) and os.path.getsize(path) > 100:
            return path
        if self.mock:
            self._write_wav(path, b"\x00\x00" * 2205)
            return path
        if not AI_CFG["ELEVEN_API_KEY"] or self.offline:
            return None
        url = "https://api.elevenlabs.io/v1/text-to-speech/%s?output_format=pcm_%d" % (
            AI_CFG["ELEVEN_VOICE_ID"], AI_CFG["SAMPLE_RATE"])
        body = json.dumps({
            "text": text,
            "model_id": AI_CFG["ELEVEN_MODEL"],
            "voice_settings": {"stability": AI_CFG["ELEVEN_STABILITY"], "similarity_boost": 0.8,
                               "style": AI_CFG["ELEVEN_STYLE"], "use_speaker_boost": True},
        }).encode()
        req = urllib.request.Request(url, data=body, method="POST", headers={
            "xi-api-key": AI_CFG["ELEVEN_API_KEY"], "Content-Type": "application/json", "Accept": "audio/*"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                pcm = r.read()
        except urllib.error.HTTPError as e:
            _log("ElevenLabs error %s: %s" % (e.code, e.read()[:200]))
            return None
        except Exception as e:
            _log("ElevenLabs unreachable (%s) -> offline: cached lines + robot voice only" % e)
            self.offline = True          # circuit breaker: don't stall the robot retrying every line
            return None
        self._write_wav(path, pcm)
        return path

    @staticmethod
    def _write_wav(path, pcm):
        with wave.open(path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(AI_CFG["SAMPLE_RATE"])
            w.writeframes(pcm)

    # ---------- robot upload ----------
    def _post_file(self, url, path):
        if self.mock:
            return True
        boundary = uuid.uuid4().hex
        name = os.path.basename(path)
        data = open(path, "rb").read()
        body = (("--%s\r\nContent-Disposition: form-data; name=\"%s\"; filename=\"%s\"\r\n"
                 "Content-Type: audio/wav\r\n\r\n") % (boundary, AI_CFG["UPLOAD_FIELD"], name)).encode() + data + \
               ("\r\n--%s--\r\n" % boundary).encode()
        req = urllib.request.Request(url, data=body, method="POST",
                                     headers={"Content-Type": "multipart/form-data; boundary=" + boundary})
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                txt = r.read().decode("utf-8", "ignore")
                if r.status != 200:
                    return False
                try:
                    j = json.loads(txt)
                    return j.get("code", 0) in (0, 200, "0") and not j.get("error")
                except Exception:
                    return "error" not in txt.lower()
        except Exception:
            return False

    def _robot_play(self, name, wait):
        """Returns True if the robot accepted the file (code 0). wait=True also waits for the clip to finish,
        whether or not the robot call blocks (so lines never cut each other off / mic doesn't hear us)."""
        if self.mock:
            return True
        path = os.path.join(AI_CFG["CACHE_DIR"], name)
        try:
            if wait:
                t0 = time.time()
                r = self.got.AUDIO.playAudioFile(name, 0)       # 0 = uploaded audio
                ok = getattr(r, "code", 0) == 0
                if ok:
                    time.sleep(max(0.0, self._duration(path) - (time.time() - t0)) + 0.15)
                return ok
            threading.Thread(target=self.got.AUDIO.playAudioFile, args=(name, 0), daemon=True).start()
            return True
        except Exception as e:
            _log("robot play failed: %s" % e)
            return False

    @staticmethod
    def _duration(path):
        try:
            with wave.open(path, "rb") as w:
                return w.getnframes() / float(w.getframerate())
        except Exception:
            return 0.0

    def probe_upload(self, path):
        """Find the robot's audio upload URL by uploading + playing a test file."""
        cands = [self.endpoint] if self.endpoint else []
        cands += [c.format(ip=self.ip) for c in AI_CFG["UPLOAD_CANDIDATES"]]
        name = os.path.basename(path)
        for url in cands:
            if not url:
                continue
            ok = self._post_file(url, path)
            _log("upload probe %s -> %s" % (url, ok))
            if ok and self._robot_play(name, wait=True):
                self.endpoint = url
                self.robot_upload_ok = True
                self.uploaded.add(name)
                with open(os.path.join(AI_CFG["CACHE_DIR"], "endpoint.txt"), "w") as f:
                    f.write(url.replace(self.ip, "{ip}"))
                _log("robot audio upload works via %s" % url)
                return True
        self.robot_upload_ok = False
        _log("no robot upload URL worked -> set UGOT_AUDIO_UPLOAD_URL. Using laptop speaker fallback.")
        return False

    def upload(self, path):
        name = os.path.basename(path)
        if name in self.uploaded:
            return True
        if self.robot_upload_ok is None:
            return self.probe_upload(path)
        if not self.robot_upload_ok:
            return False
        if self._post_file(self.endpoint, path):
            self.uploaded.add(name)
            return True
        return False

    # ---------- playback ----------
    def _laptop_play(self, path, wait):
        if self.mock:
            return True
        try:
            if platform.system() == "Windows":
                import winsound
                flags = winsound.SND_FILENAME | (0 if wait else winsound.SND_ASYNC)
                winsound.PlaySound(path, flags)
            elif platform.system() == "Darwin":
                p = subprocess.Popen(["afplay", path])
                if wait:
                    p.wait()
            else:
                p = subprocess.Popen(["aplay", "-q", path])
                if wait:
                    p.wait()
            return True
        except Exception as e:
            _log("laptop play failed: %s" % e)
            return False

    def prepare(self, lines):
        """Pre-generate + upload every fixed line before the run starts (no latency during the run)."""
        if not self.enabled:
            return
        lines = list(dict.fromkeys(lines))
        _log("preparing %d ElevenLabs lines..." % len(lines))
        for i, t in enumerate(lines):
            p = self.generate(t)
            if p:
                self.upload(p)
        _log("voice ready (robot upload: %s)" % self.robot_upload_ok)

    def say(self, text, wait=True, fallback_tts=None):
        """Speak with ElevenLabs; returns how it was played."""
        if self.enabled and text not in self.failed:
            path = self._path(text)
            if not os.path.exists(path):
                path = self.generate(text, timeout=AI_CFG["LIVE_TTS_TIMEOUT"])
                if not path:
                    self.failed.add(text)
            if path:
                with self.lock:              # one upload/probe at a time
                    up = self.upload(path)
                if up and self._robot_play(os.path.basename(path), wait):
                    return "robot-elevenlabs"
                if AI_CFG["LAPTOP_FALLBACK"] and self._laptop_play(path, wait):
                    return "laptop-elevenlabs"
        if fallback_tts:
            fallback_tts(text, wait)
        return "robot-tts"


# =====================================================================================
# Gemma 4 (local, open weights, via Ollama)
# =====================================================================================
class Gemma:
    def __init__(self, enabled=True, mock=False):
        self.mock = mock
        self.model = None
        if not enabled:
            return
        if mock:
            self.model = "gemma4-mock"
            return
        try:
            with urllib.request.urlopen(AI_CFG["GEMMA_URL"] + "/api/tags", timeout=3) as r:
                tags = [m["name"] for m in json.loads(r.read())["models"]]
            want = AI_CFG["GEMMA_MODEL"].lower()
            match = [t for t in tags if t.lower().startswith(want)] or \
                [t for t in tags if "gemma" in t.lower() and "embed" not in t.lower()]
            if match:
                self.model = match[0]
                _log("Gemma model: %s - warming up (first load can take a while)..." % self.model)
                self.ask("Reply with OK.", as_json=False, timeout=180)     # load weights BEFORE the timed run
                _log("Gemma ready")
            else:
                _log("no Gemma model in Ollama (have %s) -> run: ollama pull %s" % (tags, AI_CFG["GEMMA_MODEL"]))
        except Exception as e:
            _log("Ollama not reachable (%s) -> Gemma disabled, rule-based decoder only" % e)

    @property
    def ok(self):
        return self.model is not None

    def ask(self, prompt, as_json=True, timeout=None):
        if not self.ok:
            return None
        if self.mock:
            return {"decoded": ["THE BLUE CORE IS STABLE"], "core": "blue", "confidence": 0.9,
                    "reasoning": "mock"} if as_json else "Mock reveal."
        body = {"model": self.model, "prompt": prompt, "stream": False, "keep_alive": "60m",
                "options": {"temperature": 0.2, "num_predict": 500, "num_ctx": 8192}}
        if as_json:
            body["format"] = "json"
        req = urllib.request.Request(AI_CFG["GEMMA_URL"] + "/api/generate", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=timeout or AI_CFG["GEMMA_TIMEOUT"]) as r:
                txt = json.loads(r.read())["response"]
            return json.loads(txt) if as_json else txt.strip()
        except Exception as e:
            _log("Gemma call failed: %s" % e)
            return None

    def analyse_evidence(self, fragments):
        """fragments: list of {raw, method, decoded, top}. Returns {core, confidence, reasoning, decoded}."""
        lines = []
        for i, f in enumerate(fragments[-6:], 1):
            lines.append("Fragment %d raw: %r" % (i, str(f["raw"])[:300]))
            for n, v in f.get("top", [])[:4]:
                lines.append("   automatic decode attempt (%s): %r" % (n, str(v)[:300]))
        prompt = (
            "You are the code-breaking module of a rescue robot in a facility called THE VAULT.\n"
            "Three energy cores exist: RED, BLUE, GREEN. Exactly one is safe/correct.\n"
            "Encrypted evidence fragments were scanned from QR codes. They may use Caesar/ROT, "
            "base64, hex, binary, reversed text, atbash, Morse, A1Z26, Vigenere or riddles; "
            "fragments may need to be combined, and a fragment may rule a colour OUT.\n\n"
            + "\n".join(lines) +
            "\n\nDecrypt every fragment, combine them, and decide which core is correct.\n"
            "Reply ONLY with JSON: {\"decoded\": [\"plaintext of each fragment\"], "
            "\"core\": \"red|blue|green|unknown\", \"confidence\": 0.0-1.0, "
            "\"reasoning\": \"short step-by-step explanation\"}")
        r = self.ask(prompt)
        if not isinstance(r, dict):
            return None
        core = str(r.get("core", "unknown")).lower()
        if core not in ("red", "blue", "green"):
            core = None
        try:
            conf = float(r.get("confidence", 0))
        except Exception:
            conf = 0.0
        if conf > 1:
            conf /= 100.0                                   # model answered in percent
        conf = max(0.0, min(1.0, conf))
        dec = r.get("decoded", [])                          # LLMs ignore shapes: normalise to list[str]
        if isinstance(dec, str):
            dec = [dec]
        elif isinstance(dec, dict):
            dec = list(dec.values())
        elif not isinstance(dec, list):
            dec = []
        dec = [str(x.get("plaintext") or x.get("decoded") or x) if isinstance(x, dict) else str(x) for x in dec]
        return {"core": core, "confidence": conf, "reasoning": str(r.get("reasoning", ""))[:600], "decoded": dec}

    def reveal_line(self, color, fragments, reasoning):
        prompt = ("You are PLASMA, a dramatic rescue robot. In at most two short spoken sentences, "
                  "explain to the judges how the decrypted evidence identified the %s core. "
                  "Evidence: %s. Reasoning: %s. No emojis, no markdown." %
                  (color, [f.get("decoded") for f in fragments], reasoning))
        r = self.ask(prompt, as_json=False, timeout=25)
        return r[:300] if r else None
