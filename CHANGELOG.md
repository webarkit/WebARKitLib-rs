# Changelog

All notable changes to this project will be documented in this file.

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
