@echo off
rem TEAM PLASMA - teach the arm by hand (ball view, grab, carry, drop-over-box). Follow the robot's voice.
cd /d "%~dp0"
"%LOCALAPPDATA%\Programs\Python\Python312\python.exe" plasma_vault.py --mission teach --no-gemma --no-eleven
