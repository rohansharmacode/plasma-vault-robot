@echo off
rem TEAM PLASMA - what does the camera see right now (QR / colours / distance). No motion.
cd /d "%~dp0"
"%LOCALAPPDATA%\Programs\Python\Python312\python.exe" plasma_vault.py --mission see --no-gemma --no-eleven
