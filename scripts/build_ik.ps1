$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$vcvars = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path $vcvars)) {
    $vcvars = "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
}
if (-not (Test-Path $vcvars)) {
    throw "vcvars64.bat not found. Install Visual Studio Build Tools with the C++ workload."
}

Write-Host "CONDA_PREFIX=$env:CONDA_PREFIX"
Write-Host "CONDA_DEFAULT_ENV=$env:CONDA_DEFAULT_ENV"

$candidates = @()
if ($env:CONDA_PREFIX) {
    if ($env:CONDA_DEFAULT_ENV -and $env:CONDA_DEFAULT_ENV -ne "base") {
        $candidates += Join-Path $env:CONDA_PREFIX "envs\$($env:CONDA_DEFAULT_ENV)\python.exe"
    }
    $candidates += Join-Path $env:CONDA_PREFIX "python.exe"
}
$path_python = Get-Command python -ErrorAction SilentlyContinue
if ($path_python) {
    $candidates += $path_python.Source
}
$candidates = $candidates | Where-Object { $_ -and (Test-Path $_) } | Select-Object -Unique
if (-not $candidates) {
    throw "No Python executable found. Activate the existing conda environment first."
}

$python = $null
foreach ($candidate in $candidates) {
    $ErrorActionPreference = "Continue"
    $output = & $candidate -c "import sys, Cython; print(sys.executable); print(Cython.__version__)" 2>&1
    $code = $LASTEXITCODE
    $ErrorActionPreference = "Stop"
    if ($code -eq 0) {
        $python = $candidate
        Write-Host "Using Python: $python"
        Write-Host "Cython $($output | Select-Object -Last 1)"
        break
    }
    Write-Host "Skipping $candidate (Cython not available)"
}

if (-not $python) {
    Write-Host "Checked:"
    $candidates | ForEach-Object { Write-Host "  $_" }
    throw "No Python with Cython found. Activate the existing conda environment and run: pip install -r requirements.txt"
}

Push-Location $root
try {
    $command = "call `"$vcvars`" >nul 2>&1 && `"$python`" setup.py build_ext --inplace 2>&1"
    cmd /c $command
    if ($LASTEXITCODE -ne 0) {
        throw "IK build failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}
