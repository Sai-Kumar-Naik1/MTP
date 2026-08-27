# Installs a JDK (if missing) and builds ENHSP from source into tools/enhsp.
# Re-run safely at any time - it skips steps that are already done.

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$enhspDir = Join-Path $repoRoot "tools\enhsp"

function Find-Java {
    $cmd = Get-Command java.exe -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    $bases = @(
        (Join-Path $env:ProgramFiles "Eclipse Adoptium"),
        (Join-Path $env:ProgramFiles "Java")
    )
    foreach ($base in $bases) {
        if (Test-Path $base) {
            $jdk = Get-ChildItem $base -Directory -Filter "jdk-*" -ErrorAction SilentlyContinue |
                Sort-Object Name -Descending | Select-Object -First 1
            if ($jdk) {
                $exe = Join-Path $jdk.FullName "bin\java.exe"
                if (Test-Path $exe) { return $exe }
            }
        }
    }
    return $null
}

$java = Find-Java
if (-not $java) {
    Write-Host "No Java found - installing Eclipse Temurin 17 JDK via winget..."
    winget install --id EclipseAdoptium.Temurin.17.JDK -e --accept-package-agreements --accept-source-agreements
    $java = Find-Java
    if (-not $java) {
        throw "Java install did not produce a usable java.exe - check the winget output above."
    }
}
Write-Host "Using Java: $java"

if (-not (Test-Path $enhspDir)) {
    Write-Host "Cloning ENHSP into $enhspDir ..."
    git clone --depth 1 https://github.com/hstairs/enhsp $enhspDir
}

$jar = Join-Path $enhspDir "enhsp-dist\enhsp.jar"
if (Test-Path $jar) {
    Write-Host "ENHSP already built at $jar"
} else {
    Write-Host "Building ENHSP (bash compile)..."
    Push-Location $enhspDir
    try {
        bash compile
    } finally {
        Pop-Location
    }
    if (-not (Test-Path $jar)) {
        throw "Build finished but $jar was not produced - see the compile output above."
    }
    Write-Host "Built $jar"
}
