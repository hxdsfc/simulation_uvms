param([string]$Python = "python")
$ErrorActionPreference = "Stop"
Set-Location (Split-Path $PSScriptRoot -Parent)
& $Python -c "import sys; assert sys.version_info >= (3, 10), 'Python 3.10+ required; recommend 3.12'"
if ($LASTEXITCODE -ne 0) { throw "Please supply Python 3.12 via -Python <path>" }
if (-not (Test-Path '.venv\Scripts\python.exe')) {
    & $Python -m venv .venv
    if ($LASTEXITCODE -ne 0) { throw "venv creation failed" }
}
& .\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
if ($LASTEXITCODE -ne 0) { throw "Dependency installation failed" }
& .\.venv\Scripts\python.exe scripts\build_model.py
if ($LASTEXITCODE -ne 0) { throw "Model generation failed" }
