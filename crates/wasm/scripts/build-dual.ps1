# build-dual.ps1
# Automates building both Standard and SIMD versions of the WASM module.

$ErrorActionPreference = "Stop"

$root = Get-Location
$wasmDir = Join-Path $root "crates/wasm"
$pkgDir = Join-Path $wasmDir "pkg"

Write-Host "Cleaning up pkg directory..."
if (Test-Path $pkgDir) { Remove-Item -Recurse -Force $pkgDir }
New-Item -ItemType Directory -Path $pkgDir | Out-Null

# 1. Build Standard Version
Write-Host "Building Standard Version..."
Set-Location $wasmDir
wasm-pack build --target web --out-dir pkg/dist-std --scope webarkit

# 2. Build SIMD Version
Write-Host "Building SIMD Version..."
$env:RUSTFLAGS = "-C target-feature=+simd128"
wasm-pack build --target web --out-dir pkg/dist-simd --scope webarkit -- --features simd
$env:RUSTFLAGS = ""

# 3. Create Unified Package Infrastructure
Write-Host "Generating unified package.json..."

$stdPkgJsonPath = Join-Path $pkgDir "dist-std/package.json"
$pkgJson = Get-Content $stdPkgJsonPath -Raw | ConvertFrom-Json

# Add exports for standard and simd
# We use a PS custom object for nesting
$exports = [PSCustomObject]@{
    "." = "./dist-std/webarkitlib_wasm.js"
    "./simd" = "./dist-simd/webarkitlib_wasm.js"
}

if (-not $pkgJson.PSObject.Properties['exports']) {
    $pkgJson | Add-Member -MemberType NoteProperty -Name "exports" -Value $exports
} else {
    $pkgJson.exports = $exports
}

# Ensure the name is scoped
$pkgJson.name = "@webarkit/webarkitlib-wasm"

$finalPkgJsonPath = Join-Path $pkgDir "package.json"
$pkgJson | ConvertTo-Json -Depth 10 | Set-Content $finalPkgJsonPath

# Copy Readme to pkg root
$readmePath = Join-Path $wasmDir "README.md"
if (-not (Test-Path $readmePath)) {
    $readmePath = Join-Path $root "README.md"
}
if (Test-Path $readmePath) {
    Copy-Item $readmePath $pkgDir
}

Write-Host "Dual build complete! Package ready in crates/wasm/pkg"
Set-Location $root
