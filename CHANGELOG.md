# Changelog - webarkit/webarkitlib-rs

All notable changes to this project will be documented in this file.

## [0.1.6] - 2026-03-25

### 🐛 Bug Fixes

- Correct get_cpara calling

### 🚀 Features

- Introduce transformation matrix smoothing filter pub fn ar_filter_trans_mat and quaternion/matrix utility functions.
- Improved ar_patt_save pattern extraction and saving utilities with image-based API
- Add automatic marker region detection and pattern extraction preview to generate_patt.rs
- Enhance pattern generation by loading camera parameters from file with fallback to identity mapping
- Add flexible pattern extraction supporting multiple pixel formats and color/mono modes in ar_patt_get_image
- Add CLI options, batch mode, and diagnostic outputs to generate_patt.rs
- Add --verbose and --debug options for detailed diagnostic and extraction logging in generate_patt.rs

### 🧪 Testing

- Enable cleanup of test pattern file in pattern.rs

## [0.1.5] - 2026-03-12

### Added
- **Version Printing System**: Added a new version module with `get_version()` and `print_version()` functions. The version is now printed at startup to aid in debugging (#13).
- **Webcam AR Example for WASM**: Introduced a real-time webcam demonstration for WASM, transitioning the demo site from static image detection to live tracking. Renamed the original image detection example for clarity (#9).
- **Comprehensive SIMD Enhancements**: Expanded SIMD support across the library with major performance optimizations and updated technical documentation explaining the architecture (#6).

### Changed
- **Unified Barcode Examples**: Consolidated `barcode.rs` and `barcode_4x4.rs` into a single, parameterized example using `clap` for command-line arguments. This improves example maintainability and developer UX (#7).
- **WASM Build Infrastructure**: Unified the WASM build process with a new OS-detecting Node.js script. This ensures the `npm run build:wasm` command correctly generates both standard and SIMD modules across different operating systems (#11).
- **Improved WASM Bindings**: Enhanced `WasmARHandle` and `MarkerResult` with more granular methods and full mapping of `ARMarkerInfo` to support complex AR interactions (#9).

## [0.1.4] - 2026-03-07

### Fixed
- **CI/CD Build Permissions**: Resolved `Permission denied` error in the NPM publication step by explicitly setting the execute bit for the dual-build script.

## [0.1.3] - 2026-03-07

### Added
- **Matrix Code (Barcode) Support**:
  - Implemented `ar_matrix_code_get_id` for decoding barcode markers (3x3 to 6x6).
  - Added BCH and Hamming ECC error correction for robust barcode reading.
  - New diagnostic tool `debug_labeling.rs` for visualizing image segmentation.
  - Dedicated barcode examples (`barcode.rs`, `barcode_4x4.rs`).
- **Dual WASM Build Pipeline**:
  - Added `build-dual.ps1` and `build-dual.sh` scripts to automate generating both Standard and SIMD-optimized WASM modules.
  - Unified `package.json` exports for dual-loading support.
- **Enhanced Web Demo**:
  - Real-time engine switching (Standard vs. SIMD).
  - Added "Adaptive Threshold" visualization and `get_threshold()` diagnostic.
  - Implemented WASM module cache-busting for reliable development updates.

### Fixed
- **SIMD Luma Rounding**: Corrected a rounding discrepancy in the WASM SIMD `grayscale` implementation to match the standard scalar version exactly.
- **Matrix Grid Sampling**: Improved homography-based grid sampling to handle different pixel formats correctly.

### Changed
- **Infrastructure**: Updated root `README.md` with instructions for the new dual-build system and barcode support.

## [0.1.2] - 2026-03-05

### Fixed
- **NPM Publication Recovery**: Version bump to clear "shadow" states from previous failed attempts and ensure public access visibility.

## [0.1.1] - 2026-03-05

### Fixed
- **Crates.io Publication**: Resolved missing metadata (description, license, readme) and corrected category slugs/keyword limits.
- **NPM Scoped Access**: Fixed E402/E403 errors by configuring public access for the `@webarkit` scope.

## [0.1.0] - 2026-03-05

### Added
- **Complete C to Rust Port**:
  - Core math and matrix utilities (`matrix.rs`, `math.rs`).
  - ICP pose estimation and coordinate conversion helpers.
  - Image processing pipeline: Thresholding, Contour extraction, and Labeling.
  - Template pattern matching for marker detection.
  - 3D Pose Estimation from square markers.
  - AR2 robust feature tracking.
- **WASM Support**:
  - High-performance memory bridge for zero-copy image processing in browsers.
  - `wasm-bindgen` API surface compatible with modern web patterns.
  - Interactive browser-based demonstration.
- **Granular SIMD Optimizations**:
  - `simd-image`: SSE4.1/WASM SIMD accelerated Grayscale and Box Filters.
  - `simd-pattern`: High-throughput 32-bit Dot Product yielding **2.3x speedup**.
- **Performance Benchmarking Suite**:
  - Comparative benchmarks between original C implementation and the new Rust core.
  - Automated performance reporting via Criterion.
- **CI/CD Automation**:
  - Automated testing and build validation for Native and WASM.
  - Lean Release Workflow focused on benchmarking and documentation.

### Changed
- **Enhanced Image Processing**:
  - `ar_labeling` optimized with **Union-Find (DSU) and path compression**, reducing label merging complexity to near $O(1)$.

### Performance Milestone
- Achieved **Performance Parity** with the original C implementation: **~404 µs (Rust)** vs **~332 µs (C)** on 429x317 resolution.
- Demonstrated superior per-pixel scaling at higher resolutions (e.g., 640x480).
- Overall **2.3x speedup** in pattern matching via SIMD.
