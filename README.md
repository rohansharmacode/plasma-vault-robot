# Team PLASMA: Reboot the Vault

Code for our UBTECH UGOT Engineer robot (mecanum chassis, 3-joint arm, clamp, front camera, IR distance sensor) at the Robomasters Hackathon. After one start command the robot does the vault missions on its own and says what it is doing at every step. Every run starts with "Team Plasma" and a catchphrase.

The voice can be an **ElevenLabs** voice (set `ELEVENLABS_API_KEY`). Without a key the robot uses its built-in voice. **Gemma 4** (open weights, run locally with Ollama) can read the QR evidence. Without it a rule-based decoder chooses the core.

## Tested on the robot

| Test | Result | Launcher |
|---|---|---|
| Says the team name at the start of every run (M1) | works | every run |
| Follows the black tape with the camera, ignores the light-brown decoy tape, does a 360-degree search at sharp turns (M2) | works through bends | `run_follow.bat` |
| Turns to a clap, says battery % and tilt (M3) | 3 of 3 | `run_clap.bat` |
| Turns until it finds a QR, reads it aloud, commits to the core colour (M4) | works with a simple QR | `run_qr.bat` |
| Grip test: claw goes down, a teammate puts the ball in the jaws, the robot lifts it, finds the box with the camera and drops the ball in | works | `run_handbox.bat` |
| Live dashboard: camera, sensors, QR, evidence, log | works on the laptop (localhost) | `--dashboard --camera` |

Known limits:
* The stop 15-25 cm before the blue wall does not work yet. The camera distance estimate is wrong.
* A very dense printed QR at about 70 cm is not read. Its squares are too small for the 640x480 camera, and the arm cable can hang in front of the camera.
* Fully autonomous pickup (`pickbox`, M5, M6) is not yet tested end to end.
* Gemma 4 is not tested on the robot. The rule-based decoder gets 22 of 29 test cases right (see `MODEL_BENCH.md`).
* `--lan` (dashboard on other devices) did not work in our tests. If it does work on your network, anyone on that network can see the camera, the log and the QR text, because the dashboard has no login.

## Setup

```
pip install ugot pillow opencv-python numpy
python plasma_vault.py --mission m1
```

* Robot IP: default `10.10.53.142`. Use `--ip x.x.x.x` to change it or `--ip scan` to search the network.
* Optional ElevenLabs voice: `$env:ELEVENLABS_API_KEY="sk_..."` (PowerShell) or `export ELEVENLABS_API_KEY=sk_...`. The code makes the clip on the laptop and tries to upload it to the robot. If no upload URL works, the clip plays on the laptop. Set `UGOT_AUDIO_UPLOAD_URL` if you know the robot's upload URL. `--no-eleven` uses the robot's built-in voice.
* Optional Gemma 4: install Ollama and pull a model (see `MODEL_BENCH.md`). `--no-gemma` turns it off.
* **Ctrl+C = emergency stop.**
* The `run_*.bat` files are shortcuts for our team laptop. They call Python 3.12 from `%LOCALAPPDATA%` with `--no-gemma --no-eleven`. The clap, QR, pickbox and handbox launchers also open the dashboard.

## Missions

| Mission | What the code does | Command |
|---|---|---|
| 1 Emergency Activation | Connects, says the team name | `--mission m1` |
| 2 Navigation Calibration | Follows the black tape with the camera. Tries to stop at the blue wall (camera blue-wall estimate, IR sensor or bump back-off). The wall stop does not work yet | `--mission m2` |
| 3 Communications Recovery | Turns to the sound, says battery % and tilt | `--mission m3` |
| 4 Decode the Energy Signature | Turns to find QR codes, drives closer if needed, reads them aloud, decodes them, saves `evidence.json` | `--mission m4 --fresh` |
| 5 Secure the Core | Finds the core of the chosen colour, grabs it, drops it in the chest | `--mission m5 [--core blue]` |
| 6 Restart the Vault | Carries the core to the power dock and explains the evidence aloud | `--mission m6` |
| Final Extraction | Runs `FINAL_SEQUENCE` after one start command (3 min max) | `--mission final --wait-start` |

`--fresh` deletes `evidence.json` first. Use it for every real run, otherwise QR codes from an earlier run count as already read.

## Tools

| `--mission` | What it does |
|---|---|
| `follow` | Follows the tape and stops before the blue wall (camera, IR and bump checks) |
| `teach` | Teach the arm by hand: ball view, grab, carry and drop poses, saved in `arm_teach.json` |
| `handbox` | Grip test with a hand-loaded ball (claw down, close, lift, camera-guided drop into the box) |
| `pickbox` | Finds the ball, lines up with the taught view, grabs it and drops it in the box |
| `see` | What the camera sees now: colours, QR, tags, distance. No motion |
| `sensors` | Live sensor values (Ctrl+C to stop) |
| `wall`, `grab`, `arm`, `linetest`, `voicetest`, `say` | Single tests for the wall stop, grab, arm poses, SDK line offset, voice and typed speech |

After `teach`, `white_view.py` records the ball view again with the white-ball detector. `qr_sweep.py` turns 360 degrees and tries every QR decoder at each step. Both use a fixed robot IP.

## Evidence and the core decision

* Each QR fragment goes into `evidence.json` with automatic decodes: base64/32, hex, binary, ASCII codes, A1Z26, Morse, Caesar 1-25, Atbash, reversed, URL, plus a base64 decode of each of those results.
* A Wi-Fi QR is spoken as "a Wi-Fi network code". Its password is masked as soon as the QR is read. The password does not reach the log, the dashboard text, the speech, Gemma or the evidence file.
  * Also masked: a Wi-Fi QR under up to two layers of the decodes above (for example base64 of hex), `password: x`, `pwd=x`, `"password":"x"`, a URL login (`user:x@host`), and in Wi-Fi text `Key: x` or "the password is x".
  * Not masked: a code the decoder does not know, a Caesar shift over hex or base32, Wi-Fi fields without the `WIFI:` prefix, the bare label `pass`, the words after the first word of an unquoted password, and the camera picture (the dashboard camera view and saved frames show the QR itself).
* Choice order: `--core` (the team's decision) > config > Gemma 4 (60% confidence or more, or any confidence when the rules find nothing) > rule-based votes > Gemma 4 with low confidence > a random colour, kept for the rest of that evidence file.
* Add a clue by hand: `python plasma_vault.py --evidence "TEXT" --mission m4`.

## Where the settings are

* `CFG` at the top of `plasma_vault.py`: missions, speeds, arm poses, targets.
* `tape_eye.py`: `EYE_CFG` (tape), `WALL_CFG` (blue wall), `COLOR_CFG` (colours).
* `arm_teach.json`: the arm poses taught by hand.

## Files

* `plasma_vault.py`: missions, navigation, arm, CLI
* `tape_eye.py`: camera vision: black tape, blue wall, colour, ball and box detection
* `plasma_ai.py`: ElevenLabs voice and Gemma 4 via Ollama
* `plasma_dashboard.py`: live dashboard at http://localhost:8765
* `mock_ugot.py`: fake robot that checks the mission flow without hardware: `python plasma_vault.py --mock --mission all` (it does not test camera vision)
* `model_bench.py`, `MODEL_BENCH.md`: evidence-decoder benchmark (rule-based vs local LLMs)
* `skills/ugot-vault-robot/SKILL.md`: agent skill so a coding agent can run and tune the robot
* `white_view.py`, `qr_sweep.py`: diagnostic scripts (see Tools)
* `arm_teach.json`: taught arm poses and ball view
* `run_*.bat`: launchers for our team laptop
* `QR_collect_green_core.png`, `test_qr.png` (shown full screen by `test_qr.html`), `qr_green.svg`: test QR codes we made for practice

License: MIT
