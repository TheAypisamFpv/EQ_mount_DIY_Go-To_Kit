@echo off
cd /d "%~dp0"
".venv\Scripts\python.exe" tracker_gui.py > launch_log.txt 2>&1
pause
