param(
    [string]$Config = "$PSScriptRoot\config.json"
)

$ErrorActionPreference = "Stop"
$Python = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $Python)) {
    throw "Virtual environment not found. Run install_voice_agent.ps1 first."
}
if (-not (Test-Path -LiteralPath $Config)) {
    throw "Configuration file not found: $Config"
}

Set-Location -LiteralPath $PSScriptRoot
& $Python "$PSScriptRoot\fcr_voice_agent.py" --config $Config
exit $LASTEXITCODE
