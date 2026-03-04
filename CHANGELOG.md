# Changelog

All notable changes to this project will be documented in this file.

## [1.0.0-beta] - 2024-03-04

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
- Refactored Image Box Filter to a separable $O(N \times K)$ implementation.
- Centralized image processing logic in `core` for cross-platform availability.

### Performance Milestone
- Achieved **~2.3x speedup** in pattern matching vs local scalar Rust baseline.
- Significant gains in image preprocessing passes (Grayscale & Filtering).
