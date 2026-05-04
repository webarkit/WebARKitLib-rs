# Changelog - webarkit/webarkitlib-rs

All notable changes to this project will be documented in this file.

## [0.3.8] - 2026-05-04

### Last action before  `Milestone 6 — Math & homography in pure Rust`

### ⚙️ Miscellaneous Tasks

- Pass --features log-helpers when running simple example
- Remove accidental .DS_Store from repo changes
- Convert println!/eprintln! to arlog_*! in examples and bench

### 🎨 Styling

- Apply cargo fmt --all (fixes CI fmt check)
- *(core)* Fix formatting issues
- *(marker)* Apply cargo fmt
- *(diagnostic)* Apply stable rustfmt wrapping (#104 CI)

### 🐛 Bug Fixes

- *(clippy)* Clear all warnings and add clippy -D warnings to CI
- *(clippy)* Fix 5 CI-only clippy warnings (explicit_counter_loop, manual_checked_ops)
- *(marker)* Populate cf_patt/id_patt and copy to final id/cf per mode
- *(core)* Port confidence_cutoff from C and add regression tests (issue #92)
- *(marker)* Wire cutoff_phase from MatchOk/MatchError result codes (#88)
- *(marker)* Resolve merge conflict in cutoff_phase tests
- *(simple,marker)* Honour the buff/pixel_format contract (#103)

### 📚 Documentation

- *(issue-86)* Add implementation plan
- Update CLAUDE.md with documentation and style guidelines
- *(issue-103)* Correct findings — Rust/C cf close but not byte-equal

### 🚀 Features

- *(marker)* Add finalize_marker_id_cf_dir helper
- *(types)* Add MatchOk, MatchError, From<MatchError> for ARMarkerInfoCutoffPhase
- *(core)* Add ar_detect_marker orchestrator for full marker detection pipeline
- *(types)* Add AR tracking history mode constants (#101)
- *(marker)* Port arDetectMarker tracking history pipeline (#96)
- *(diagnostic)* Add dump_patt + diff_patt for issue #103

### 🚜 Refactor

- *(core)* Move AR modules into ar/ subfolder (closes #82)
- *(types)* Defer MatchOk.global_id field to issue #89
- *(pattern)* Pattern_match returns Result<MatchOk, MatchError>
- *(matrix)* Ar_matrix_code_get_id returns Result<MatchOk, MatchError>

### 🧪 Testing

- *(marker)* Add failing tests for finalize_marker_id_cf_dir helper
- *(marker)* Add confidence_cutoff edge case and mode-specific unit tests (#96)
- *(marker)* Extract history phases as helpers and add 12 unit tests (#96)

## [0.3.7] - 2026-04-23

### 🐛 Bug Fixes

- Replace PNG banner with JPG version in README and assets

## [0.3.6] - 2026-04-23

### 🐛 Bug Fixes

- *(ci)* Harden publish-crates checkout against submodule HTTP 500s

## [0.3.5] - 2026-04-23

### 🐛 Bug Fixes

- *(ci)* Add submodules: recursive to release.yml checkout steps

## [0.3.4] - 2026-04-23

### Preliminary action before  `Milestone 6 — Math & homography in pure Rust`

### 🎨 Styling

- Apply cargo fmt to build.rs

### 🐛 Bug Fixes

- *(ffi-backend)* Vendor WebARKitLib C++ via git submodule (#72)

### 📚 Documentation

- *(arlog)* Pin log-helpers on load_nft example and clarify usage
- *(log)* Use arlog_i! instead of println! in doc examples
- Add CLAUDE.md with project conventions + HEADER.txt template

### 🚀 Features

- *(arlog)* Port ARToolKit logging API over the log crate facade (#57)
- *(arlog)* Add verbose init helpers with timestamp + module path
- *(arlog)* Sweep existing println/debug calls to arlog_* (Pass A)
- *(log)* Fill in missing arlog_d! calls in marker.rs (Pass B2)
- *(log)* Fill in missing arlog_* calls in ar2/tracking.rs (Pass B2)
- *(log)* Port informational arlog_i! calls in ar2/feature_map.rs (B2)
- *(log)* Wire arlog backend into nft_marker_gen example
- *(log)* Add arlog_* at Err sites in matrix.rs + bch.rs (Pass B2)
- *(log)* Retrofit log + return Err pattern across 5 modules (Pass B3)
- Include assets directory in Cargo.toml for build process

### 🚜 Refactor

- *(log)* Swap bare log::* for arlog_* vocabulary (Pass B1)

## [0.3.3] - 2026-04-20

### Milestone 5 — NFT marker creation pipeline

### ⚡ Performance

- *(ar2)* Parallelize feature_map Stage 3 with Rayon (1.49x full-image)
- *(ar2)* SIMD get_similarity (SSE4.1+AVX2) + parallel pyramid (1.70x total)

### 🎨 Styling

- *(ar2)* Apply cargo fmt
- Fix cargo fmt in feature_map_bench and feature_map tests

### 🐛 Bug Fixes

- *(nft_marker_gen)* Fix merge() in-place return + scoped Write import + add reference image
- *(kpm)* Fix generate() producing 0 FREAK features in .fset3 (#51)
- *(ar2)* Include all pyramid levels in fset even when 0 features detected

### 📚 Documentation

- Update README and lib.rs with NFT marker generation pipeline
- Add SIMD/parallel build instructions and per-step timing
- Clarify WASM section — npm package vs local demo build

### 🚀 Features

- *(ar2)* Port ar2GenFeatureMap into feature_map.rs
- *(ar2)* Add .iset and .fset write support + rename gen_image_layer
- *(ar2)* Add ar2_gen_image_set, refactor ar2_gen_feature_map, add nft_marker_gen example (#49)
- *(ar2)* Add JPEG compression to .iset save matching C++ ar2WriteImageSet
- *(nft_marker_gen)* Improve logging to match NFT-Marker-Creator-App

### 🚜 Refactor

- *(nft_pipeline)* Improve documentation and isolate feature extraction comparison with C-generated markers

## [0.3.2] - 2026-04-07

### Milestone 4 — KPM pipeline correctness

### 🐛 Bug Fixes

- Fset3 KpmImageInfo field order and enable full pipeline test
- Improve assertion formatting in test_matched_db_id_zero_finds_pose
- Formatting style causing CI failure
- Gate simple_nft example behind ffi-backend feature
- Formatting in the new surface.rs file

### 🚀 Features

- Add simple_nft example demonstrating KPM detection → AR2 tracking pipeline
- Add WasmNFTHandle for AR2 tracking in WASM and NFT demo page
- Add pinball demo image and update simple NFT example to use it

### 🚜 Refactor

- Add ar2_read_surface_set to match C++ nftSimple pattern
- Extract surface types and functions into new ar2/surface.rs

## [0.3.1] - 2026-04-06

### Milestone 3 — Architectural consolidation

### ⚙️ Miscellaneous Tasks

- Update version to 0.3.0 in package.json and enhance release process in MAINTAINERS.md

### 🎨 Styling

- Fix cargo fmt issues in cpp_backend.rs and lib.rs

### 🐛 Bug Fixes

- *(example)* Fix index out of bounds panic in debug_labeling

### 🚀 Features

- *(readme)* Add Rust banner image to README.md

### 🚜 Refactor

- *(kpm)* Move kpm crate into core as submodule (M3-1)
- *(ar2)* Move ar2 crate into core as submodule (M3-2)
- *(workspace)* Consolidate ar2 and kpm into core crate (#45)
- *(ci)* Update checkout action to v6 and streamline cargo commands
- *(readme)* Enhance project documentation and add usage instructions
- *(docs)* Update license information and improve module documentation in bch.rs, filter.rs, and pose.rs

## [0.3.0] - 2026-04-04

### Milestone 2 — KPM layer in pure Rust (FreakMatcher still C++)

### 🐛 Bug Fixes

- *(kpm)* Use >= 0 for matched_id check to accept db_id 0 (#43)
- *(benchmark)* Exclude KPM targets from default build

### 🚀 Features

- *(kpm)* Port KpmRefDataSet I/O — generate, save, load, merge (#26)
- *(kpm)* Port kpm_matching orchestration and add load_fset3 example (#27)
- *(kpm)* Expose query accessors in C API and add set_ref_data_set (#36)
- *(ar2)* Port AR2 imageSet and featureSet I/O to Rust (#37)
- *(kpm)* Replace load_fset3 example with load_nft and add fset3 test
- *(kpm)* Add regression test suite with C++ baseline validation (#28)

## [0.2.0] - 2026-04-02

### Milestone 1: Kpm C++ FFI Backend and Crate Scaffolding

### ⚙️ Miscellaneous Tasks

- Add dedicated kpm-build job, exclude kpm from workspace checks (#30)
- *(kpm)* Update bindgen dependency to version 0.72.1
- *(kpm)* Add LGPL-3.0 license headers to all source files

### 🎨 Styling

- *(kpm)* Apply cargo fmt formatting

### 🚀 Features

- *(kpm)* Add crate scaffold with C++ FFI build system (#21)
- *(kpm)* Port KPM types and structs to Rust (#31)
- *(kpm)* Add FreakMatcherBackend trait and supporting types (#23)
- *(kpm)* Implement CppFreakMatcher FFI backend (#24)
- *(kpm)* Implement KpmHandle struct and improve crate documentation (#25)

## [0.1.7] - 2026-03-29

### ⚙️ Miscellaneous Tasks

- Update GitHub Actions to Node.js 24-compatible versions

### 🐛 Bug Fixes

- Include dist-std and dist-simd folders in npm package

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
