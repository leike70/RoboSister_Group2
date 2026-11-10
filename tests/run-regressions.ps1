param([string]$Compiler)
$ErrorActionPreference = 'Stop'
$testRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $testRoot
if (!$Compiler) {
    $compilerCommand = Get-Command gcc -ErrorAction SilentlyContinue
    if ($compilerCommand) { $Compiler = $compilerCommand.Source }
    else {
        $Compiler = @('D:\CODE\CLion 2026.2.2\bin\mingw\bin\gcc.exe',
                      'C:\Program Files (x86)\Dev-Cpp\MinGW64\bin\gcc.exe') |
            Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    }
}
if (!$Compiler) { throw 'Provide an installed native Windows GCC using -Compiler.' }
$testBuild = Join-Path $repoRoot 'work\host-regressions'
$previousPath = $env:PATH
try {
    $env:PATH = "$(Split-Path -Parent $Compiler);$previousPath"
    & cmake -S $testRoot -B $testBuild -G Ninja "-DCMAKE_C_COMPILER=$Compiler"
    if ($LASTEXITCODE -ne 0) { throw 'Regression configuration failed.' }
    & cmake --build $testBuild
    if ($LASTEXITCODE -ne 0) { throw 'Regression compilation failed.' }
    $ctest = Join-Path (Split-Path -Parent (Get-Command cmake).Source) 'ctest.exe'
    & $ctest --test-dir $testBuild --output-on-failure
    if ($LASTEXITCODE -ne 0) { throw 'Regression tests failed.' }
} finally {
    $env:PATH = $previousPath
}
