@echo off
rem TEAM PLASMA - line following + stop before the blue wall. Ctrl+C = emergency stop.
cd /d "%~dp0"
"%LOCALAPPDATA%\Programs\Python\Python312\python.exe" plasma_vault.py --mission follow --no-gemma --no-eleven --time-limit 180
