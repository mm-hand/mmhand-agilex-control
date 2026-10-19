$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$vcvars = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path $vcvars)) {
    $vcvars = "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
}
if (-not (Test-Path $vcvars)) {
    throw "vcvars64.bat not found. Install Visual Studio Build Tools with the C++ workload."
}
Push-Location $root
try {
    cmd /c "call `"$vcvars`" >nul && python setup.py build_ext --inplace"
} finally {
    Pop-Location
}
