@echo off
rem TEAM PLASMA - find the ball, grab it, drop it in the box (uses the taught arm poses). Ctrl+C = stop.
cd /d "%~dp0"
"%LOCALAPPDATA%\Programs\Python\Python312\python.exe" plasma_vault.py --mission pickbox --no-gemma --no-eleven --time-limit 120 --dashboard --camera
