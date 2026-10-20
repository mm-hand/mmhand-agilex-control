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
if ($env:CONDA_PREFIX) {
    $python = Join-Path $env:CONDA_PREFIX "python.exe"
} else {
    $python = (Get-Command python -ErrorAction Stop).Source
}
if (-not (Test-Path $python)) {
    throw "Python executable not found: $python. Activate the existing conda environment first."
}
Write-Host "Using Python: $python"

$ErrorActionPreference = "Continue"
$cython_output = & $python -c "import Cython; print(Cython.__version__)" 2>&1
$cython_exit = $LASTEXITCODE
$ErrorActionPreference = "Stop"
if ($cython_exit -ne 0) {
    $cython_output | ForEach-Object { Write-Host $_ }
    throw "Cython is not installed in $python. Activate the environment and run: pip install -r requirements.txt"
}
Write-Host "Cython $cython_output"

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
