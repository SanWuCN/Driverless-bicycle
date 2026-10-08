$ErrorActionPreference = "Stop"

$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$VenvRoot = Join-Path $ProjectRoot ".venv-control-station-build"
$PythonExe = Join-Path $VenvRoot "Scripts\python.exe"

if (-not (Test-Path $PythonExe)) {
    python -m venv $VenvRoot
}

& $PythonExe -m pip install --disable-pip-version-check -r (Join-Path $ProjectRoot "requirements-control-station.txt")
& $PythonExe -m pip install --disable-pip-version-check "pyinstaller>=6,<7"

$DistRoot = Join-Path $ProjectRoot "dist\windows"
$WorkRoot = Join-Path $ProjectRoot "build\pyinstaller-windows"
$StaticRoot = Join-Path $ProjectRoot "tools\control_station_web"
$EntryPoint = Join-Path $ProjectRoot "tools\control_station.py"

& $PythonExe -m PyInstaller `
    --noconfirm `
    --clean `
    --windowed `
    --name BikeControlStation `
    --add-data "${StaticRoot};tools/control_station_web" `
    --collect-all odrive `
    --collect-all fibre `
    --paths (Join-Path $ProjectRoot "tools") `
    --distpath $DistRoot `
    --workpath $WorkRoot `
    --specpath $WorkRoot `
    $EntryPoint

$AppDirectory = Join-Path $DistRoot "BikeControlStation"
$Archive = Join-Path $ProjectRoot "dist\BikeControlStation-Windows-x64.zip"
if (Test-Path $Archive) {
    Remove-Item -Force $Archive
}
Compress-Archive -Path $AppDirectory -DestinationPath $Archive -CompressionLevel Optimal
Write-Host "Windows package: $Archive"
