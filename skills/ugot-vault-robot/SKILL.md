---
name: ugot-vault-robot
description: Operate, tune and debug the Team PLASMA UGOT Engineer robot (mecanum + arm + clamp) for the Robomasters "Reboot the Vault" missions. Tested on the robot - camera tape following, turning to a clap, reading a simple QR, hand-loaded grip and drop into a box. Not working or untested - the blue-wall stop, Gemma 4 decoding, autonomous core pickup (M5/M6/pickbox). Use when asked to run a mission, calibrate the robot, or analyse the QR evidence for the team's core decision.
license: MIT
---

# UGOT Vault Robot

## Run a mission
`python plasma_vault.py --mission <m1|m2|m3|m4|m5|m6|all|final>` (robot IP defaults to 10.10.53.142, `--ip` to change).
Add `--mock` to simulate without hardware. Ctrl+C is the emergency stop.

## Calibrate before the first real run
1. `--mission sensors` and read the live values (Ctrl+C to stop).
2. `--mission follow` to check the camera tape follower. Tape thresholds are `EYE_CFG` in `tape_eye.py`.
3. `--mission teach` to teach the arm by hand (saved in `arm_teach.json`), then `--mission handbox` to check the grip.
4. Set the `CHEST` and `POWER_DOCK` target specs (apriltag / qr / color / moves / wall) in `CFG`. The `--chest` / `--dock` flags take only `wall`, `tag:N`, `qr:TEXT` or `color:NAME`.
5. Use `--fresh` for every real run so old QR evidence is not reused.

## Decide the energy core
1. Add every clue: `python plasma_vault.py --evidence "<text>" --mission m4` (or scan the QR with m4).
2. Read `evidence.json`. `fragments[*].top` lists the automatic decodes and `gemma` holds the Gemma 4 verdict and reasoning.
3. The team commits with `--core red|blue|green`. The robot never overrides that.

## Troubleshooting
- No distance reading: set `CFG["DIST_ID"]` to the ID shown in the UGOT app.
- Robot silent with ElevenLabs: run `--mission voicetest`, then set `UGOT_AUDIO_UPLOAD_URL`.
- Gemma not found: install Ollama (see `MODEL_BENCH.md`), `ollama pull gemma4`, and make sure `ollama serve` is running.
- Grab misses: run `--mission teach` again, then `white_view.py` to record the ball view with the white-ball detector.
- `final`, `--wait-start` and `say` wait for keyboard input.
