@echo off
rem TEAM PLASMA - Mission 3: turn towards a voice/clap, then report battery and tilt. Repeats 3 times.
cd /d "%~dp0"
"%LOCALAPPDATA%\Programs\Python\Python312\python.exe" plasma_vault.py --mission m3 --repeat 3 --no-gemma --no-eleven --time-limit 180 --dashboard --camera
