@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo Please run scripts\setup.ps1 with Python 3.12 first.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" scripts\simulate.py --medium water --viewer
if errorlevel 1 pause
