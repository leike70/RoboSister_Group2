<#
.SYNOPSIS
Read-only comparison of the two vehicle projects' shared files.

.DESCRIPTION
Lists common files, reports missing/extra copies and content differences.
It never copies or overwrites a vehicle's own Config/app_config.c or
Vehicle/app_vehicle.c. Those files are explicitly allowed to differ.
Exit code 1 means a shared-file drift was found.
#>
[CmdletBinding()]
param([string]$RepoRoot = "")

$ErrorActionPreference = 'Stop'
if ([string]::IsNullOrWhiteSpace($RepoRoot)) {
    $scriptPath = $MyInvocation.MyCommand.Path
    $RepoRoot = Split-Path -Parent (Split-Path -Parent $scriptPath)
}
$eng = Join-Path $RepoRoot 'project file\Engineering robots'
$tur = Join-Path $RepoRoot 'project file\Turret Robot'
$commonDirs = @(
    'App\Common', 'App\Remote', 'App\Chassis', 'App\Lift', 'App\Attitude',
    'App\Power', 'App\Protection', 'App\Motor', 'Bsp',
    'User_Driver\Remote', 'User_Driver\IMU', 'User_Driver\Motor_Driver',
    'Task'
)
$commonFiles = @(
    'App\Vehicle\app_vehicle.h',
    'Config\app_config.h', 'Config\imu_config.h', 'Config\imu_config.c',
    'User_Driver\pid_driver.h', 'User_Driver\pid_driver.cpp',
    'Core\Src\main.c', 'Core\Src\freertos.c', 'Core\Src\spi.c', 'Core\Src\tim.c',
    'Core\Inc\tim.h',
    'cmake\gcc-arm-none-eabi.cmake'
)
$allowedDifferent = @('App\Vehicle\app_vehicle.c', 'Config\app_config.c')
$problems = New-Object System.Collections.Generic.List[string]

function Add-File([string]$rel) {
    $a = Join-Path $eng $rel
    $b = Join-Path $tur $rel
    $aExists = Test-Path -LiteralPath $a -PathType Leaf
    $bExists = Test-Path -LiteralPath $b -PathType Leaf
    if ($aExists -ne $bExists) {
        $problems.Add("ONLY ONE SIDE: $rel (eng=$aExists tur=$bExists)")
        return
    }
    if (-not $aExists) { return }
    $ha = (Get-FileHash -LiteralPath $a -Algorithm SHA256).Hash
    $hb = (Get-FileHash -LiteralPath $b -Algorithm SHA256).Hash
    if ($ha -ne $hb) { $problems.Add("CONTENT DIFF: $rel") }
}

foreach ($dir in $commonDirs) {
    $aDir = Join-Path $eng $dir
    $bDir = Join-Path $tur $dir
    if (-not (Test-Path -LiteralPath $aDir -PathType Container) -or
        -not (Test-Path -LiteralPath $bDir -PathType Container)) {
        $problems.Add("MISSING COMMON DIR: $dir")
        continue
    }
    $aFiles = Get-ChildItem -LiteralPath $aDir -File -Recurse
    $bFiles = Get-ChildItem -LiteralPath $bDir -File -Recurse
    $aRel = $aFiles | ForEach-Object { $_.FullName.Substring($aDir.Length + 1) }
    $bRel = $bFiles | ForEach-Object { $_.FullName.Substring($bDir.Length + 1) }
    foreach ($rel in ($aRel + $bRel | Sort-Object -Unique)) {
        Add-File (Join-Path $dir $rel)
    }
}
foreach ($rel in $commonFiles) { Add-File $rel }

Write-Host "Shared-file check (read-only)"
Write-Host "  engineering: $eng"
Write-Host "  turret:      $tur"
Write-Host "  allowed differences: $($allowedDifferent -join ', ')"
if ($problems.Count -eq 0) {
    Write-Host '  RESULT: synchronized'
    exit 0
}
$problems | ForEach-Object { Write-Host "  $_" }
Write-Host "  RESULT: $($problems.Count) problem(s)"
exit 1
