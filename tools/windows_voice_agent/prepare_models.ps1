param()

$ErrorActionPreference = "Stop"
$Python = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
$Models = Join-Path $PSScriptRoot "models"

if (-not (Test-Path -LiteralPath $Python)) {
    throw "Virtual environment not found. Run install_voice_agent.ps1 first."
}
if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    throw "Git is required. Install Git for Windows first."
}
if (-not (Get-Command git-lfs -ErrorAction SilentlyContinue)) {
    throw "Git LFS is required. Install it, then run: git lfs install"
}

New-Item -ItemType Directory -Force -Path $Models | Out-Null
git lfs install | Out-Host
if ($LASTEXITCODE -ne 0) { throw "git lfs install failed" }

function Get-ModelRepository {
    param(
        [Parameter(Mandatory = $true)][string]$Repository,
        [Parameter(Mandatory = $true)][string]$Destination,
        [Parameter(Mandatory = $true)][string]$WeightFile
    )
    $weight = Join-Path $Destination $WeightFile
    if (Test-Path -LiteralPath $weight) {
        Write-Host "Model already ready: $Destination"
        return
    }
    if (Test-Path -LiteralPath $Destination) {
        throw "Incomplete model directory exists: $Destination. Move it aside and retry."
    }
    Write-Host "Cloning $Repository -> $Destination"
    git clone $Repository $Destination
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $weight)) {
        throw "Model clone did not produce $WeightFile in $Destination"
    }
}

Get-ModelRepository `
    -Repository "https://huggingface.co/xlx2673/classifier" `
    -Destination (Join-Path $Models "classifier_8class") `
    -WeightFile "model.safetensors"

Get-ModelRepository `
    -Repository "https://huggingface.co/BAAI/bge-base-zh-v1.5" `
    -Destination (Join-Path $Models "bge-base-zh-v1.5") `
    -WeightFile "pytorch_model.bin"

$Script = @'
from pathlib import Path
from transformers import AutoConfig
from sentence_transformers import SentenceTransformer

root = Path(r"__MODELS__")
labels = int(AutoConfig.from_pretrained(
    root / "classifier_8class", local_files_only=True
).num_labels)
if labels != 8:
    raise RuntimeError(f"expected an 8-label classifier, got {labels}")
model = SentenceTransformer(
    str(root / "bge-base-zh-v1.5"), local_files_only=True, device="cpu"
)
shape = model.encode(["开始跟随"]).shape
if shape != (1, 768):
    raise RuntimeError(f"unexpected BGE output shape: {shape}")
print("Local model preparation complete; classifier labels=8, BGE dims=768")
'@
$Script = $Script.Replace("__MODELS__", $Models.Replace("\", "\\"))
$Script | & $Python -
if ($LASTEXITCODE -ne 0) { throw "Local model validation failed" }

Write-Host "Models are ready. Normal agent startup is fully offline."
