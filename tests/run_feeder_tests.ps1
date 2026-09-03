param(
    [string]$Compiler = 'C:\mingw64\bin\g++.exe',
    [string]$OutputDir = (Join-Path $env:TEMP 'robomaster-feeder-tests')
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$src = Join-Path $repo 'STM32F405'
$stub = Join-Path $PSScriptRoot 'host_stubs'
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
$failed = 0
foreach ($name in @('feeder_command_test', 'pid_reset_test', 'feeder_direction_test', 'feeder_diagnostics_test')) {
    $exe = Join-Path $OutputDir ($name + '.exe')
    & $Compiler -std=c++17 -Wall -Wextra -Werror -I $stub -I $src (Join-Path $PSScriptRoot ($name + '.cpp')) -o $exe
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $name" }
    & $exe
    if ($LASTEXITCODE -ne 0) { $failed++ }
}
if ($failed -ne 0) { Write-Output "FAILED: $failed suites"; exit 1 }
Write-Output 'PASS: all feeder host tests'
