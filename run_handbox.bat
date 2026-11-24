@echo off
rem TEAM PLASMA - claw goes down, put the ball in the jaws (10 s), it lifts it and drops it in the box. Ctrl+C = stop.
cd /d "%~dp0"
"%LOCALAPPDATA%\Programs\Python\Python312\python.exe" plasma_vault.py --mission handbox --no-gemma --no-eleven --time-limit 120 --dashboard --camera
