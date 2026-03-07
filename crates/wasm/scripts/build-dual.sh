#!/bin/bash
# build-dual.sh
# Automates building both Standard and SIMD versions of the WASM module for Linux/macOS.

set -e

WASMIDIR=$(pwd)
PKGDIR="$WASMIDIR/pkg"
ROOT=$(cd ../.. && pwd)

echo "Cleaning up pkg directory..."
rm -rf "$PKGDIR"
mkdir -p "$PKGDIR"

# 1. Build Standard Version
# Copy LICENSE to wasm crate root so wasm-pack finds it
cp "$ROOT/LICENSE" "$WASMDIR/"

echo "Building Standard Version..."
wasm-pack build --target web --out-dir pkg/dist-std --scope webarkit

# 2. Build SIMD Version
echo "Building SIMD Version..."
export RUSTFLAGS="-C target-feature=+simd128"
wasm-pack build --target web --out-dir pkg/dist-simd --scope webarkit -- --features simd
unset RUSTFLAGS

# 3. Create Unified Package Infrastructure
echo "Generating unified package.json..."

# Use node to manipulate package.json if available, or just use sed/simple concatenation
# Since we are in a dev environment, Node is likely present.
node <<EOF
const fs = require('fs');
const path = require('path');

const stdPkgJsonPath = path.join('$PKGDIR', 'dist-std', 'package.json');
const pkgJson = JSON.parse(fs.readFileSync(stdPkgJsonPath, 'utf8'));

// Delete files array since wasm-pack generates it based on subfolder content
// We want the root pkg to contain everything.
delete pkgJson.files;

// Add exports for standard and simd
pkgJson.exports = {
    ".": "./dist-std/webarkitlib_wasm.js",
    "./simd": "./dist-simd/webarkitlib_wasm.js"
};

// Add types versions if needed, but for now just pointing to the generated ones
// Note: name should be the full scoped name
pkgJson.name = "@webarkit/webarkitlib-wasm";

const finalPkgJsonPath = path.join('$PKGDIR', 'package.json');
fs.writeFileSync(finalPkgJsonPath, JSON.stringify(pkgJson, null, 2));
EOF

# Copy Readme to pkg root
if [ -f "$WASMIDIR/README.md" ]; then
    cp "$WASMIDIR/README.md" "$PKGDIR/"
else
    cp "$ROOT/README.md" "$PKGDIR/"
fi

cp "$ROOT/LICENSE" "$PKGDIR/"

echo "Dual build complete! Package ready in crates/wasm/pkg"
