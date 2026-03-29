# build-dual.ps1
# Automates building both Standard and SIMD versions of the WASM module.

$ErrorActionPreference = "Stop"

# Get the directory where the script is located
$scriptDir = $PSScriptRoot
$wasmDir = Join-Path $scriptDir ".." | Resolve-Path
$root = Join-Path $wasmDir "../.." | Resolve-Path
$pkgDir = Join-Path $wasmDir "pkg"

Write-Host "Cleaning up pkg directory..."
if (Test-Path $pkgDir) { Remove-Item -Recurse -Force $pkgDir }
New-Item -ItemType Directory -Path $pkgDir | Out-Null

# 1. Build Standard Version
# Copy LICENSE to wasm crate root so wasm-pack finds it
Copy-Item (Join-Path $root "LICENSE") -Destination $wasmDir

Write-Host "Building Standard Version..."
Set-Location $wasmDir
wasm-pack build --target web --out-dir pkg/dist-std --scope webarkit

# 2. Build SIMD Version
Write-Host "Building SIMD Version..."
$env:RUSTFLAGS = "-C target-feature=+simd128"
wasm-pack build --target web --out-dir pkg/dist-simd --scope webarkit -- --features simd
$env:RUSTFLAGS = ""

# Remove wasm-pack generated .gitignore files that block npm publish
Remove-Item -Force (Join-Path $pkgDir "dist-std/.gitignore") -ErrorAction SilentlyContinue
Remove-Item -Force (Join-Path $pkgDir "dist-simd/.gitignore") -ErrorAction SilentlyContinue

# 3. Create Unified Package Infrastructure
Write-Host "Generating unified package.json..."

$stdPkgJsonPath = Join-Path $pkgDir "dist-std/package.json"
$pkgJson = Get-Content $stdPkgJsonPath -Raw | ConvertFrom-Json

# Set explicit files array so npm includes both dist directories
$filesArray = @("dist-std/", "dist-simd/")
if ($pkgJson.PSObject.Properties['files']) {
    $pkgJson.files = $filesArray
} else {
    $pkgJson | Add-Member -MemberType NoteProperty -Name "files" -Value $filesArray
}

# Update main and types to point into dist-std
$pkgJson.main = "dist-std/webarkitlib_wasm.js"
$pkgJson.types = "dist-std/webarkitlib_wasm.d.ts"

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

Copy-Item (Join-Path $root "LICENSE") -Destination $pkgDir

Write-Host "Dual build complete! Package ready in crates/wasm/pkg"
Set-Location $root
