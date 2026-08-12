param(
    [string]$ClassifierRoot = "D:\code\fcr_ros2\classfier\fcr_speech_interpreter",
    [string]$AsrRoot = "D:\code\fcr_ros2\voice\server"
)

$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

if (-not (Test-Path -LiteralPath $ClassifierRoot)) {
    throw "Classifier project not found: $ClassifierRoot"
}
if (-not (Test-Path -LiteralPath "$AsrRoot\listener\listener.py")) {
    throw "ASR listener module not found: $AsrRoot\listener\listener.py"
}

if (-not (Test-Path -LiteralPath ".venv\Scripts\python.exe")) {
    py -3.12 -m venv .venv
}

$Python = "$PSScriptRoot\.venv\Scripts\python.exe"
& $Python -m pip install --upgrade pip
& $Python -m pip install -r "$ClassifierRoot\requirements.txt"
& $Python -m pip install -r "$PSScriptRoot\requirements.txt"

Write-Host "Running installation smoke checks..."
& $Python -c "import sys; sys.path[:0]=[r'$ClassifierRoot',r'$AsrRoot']; from classifier import DoubleLayerClassifier; from splitter import Splitter; from listener import Listener; import numpy, requests, sounddevice, torch, transformers, qwen_asr; print('Python modules and dependencies: OK')"
if ($LASTEXITCODE -ne 0) {
    throw "Python dependency smoke check failed"
}

if (-not (Test-Path -LiteralPath "$PSScriptRoot\config.json")) {
    Copy-Item -LiteralPath "$PSScriptRoot\config.example.json" `
        -Destination "$PSScriptRoot\config.json"
}

Write-Host "Installed. Edit config.json, set FCR_VOICE_AUTH_TOKEN, then run:"
Write-Host "  .\run_voice_agent.ps1"
