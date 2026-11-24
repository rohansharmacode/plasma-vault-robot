@echo off
rem TEAM PLASMA - QR evidence scan (Mission 4): rotate until a QR is found, read + decode it. Ctrl+C = stop.
cd /d "%~dp0"
"%LOCALAPPDATA%\Programs\Python\Python312\python.exe" plasma_vault.py --mission m4 --fresh --qr-max 1 --no-gemma --no-eleven --time-limit 120 --dashboard --camera
