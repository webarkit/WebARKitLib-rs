# Changelog - webarkit/webarkitlib-rs

All notable changes to this project will be documented in this file.

## [0.7.0] - 2026-06-05

## Milestone 9 — VisualDatabase & pure Rust backend

### ⚙️ Miscellaneous Tasks

- *(examples)* Dump db_id pyramid in simple_nft_dual to aid #160 triage
- *(corner-error)* Wire the absolute_corner_error gate into kpm-build (#166)
- *(submodule)* Bump WebARKitLib for std::map matcher-determinism fix (refs #170)
- *(submodule)* Retarget WebARKitLib pin to post-#39 merged master SHA (#170)
- *(bridge)* Bump jsartoolkit-nft to 1.10.0 + sharp to 0.34.5 + regen sidecar + widen POSE_ROT_TOL (#170)
- *(examples)* Convert simple_nft to arlog macros (PR 4/4 for #90)

### 🐛 Bug Fixes

- *(test)* Regenerate kpm_regression baseline and close CI gap (#155)
- *(test)* Restore Linux baseline + gate test_full_pipeline_pose to Linux
- *(tests)* Adapt corner-error gate to cross-platform variance (drop pinball-demo, widen epsilon to 2.0 px) (#166)
- *(kpm)* Use BTreeMap for Hough vote tally and VisualDatabase keyframes to remove Rust-side matcher nondeterminism (#170)

### 📚 Documentation

- *(kpm)* Add end-to-end pose comparison to M9-2 design doc §10
- Switch regen capture recipe from arlog_e! to arlog_i! (#155)
- *(examples)* Clarify simple_nft_dual.rs reports homography-level comparison (#157)
- *(tests)* Add README for annotated_frames fixtures (#166)
- *(readme)* Mark M9 (#139) complete, refresh roadmap + nft_marker_gen

### 🚀 Features

- *(kpm)* Port VisualDatabase to Rust (M9-1, #140)
- *(kpm)* Move BHC feature index from FeatureMatcher onto Keyframe (#146)
- *(kpm)* Port HoughSimilarityVoting::autoAdjustXYNumBins (#150)
- *(kpm)* Implement RustFreakMatcher and DualFreakMatcher (M9-2, #141)
- *(examples)* Add simple_nft_dual.rs with DualFreakMatcher and per-frame divergence reporting (#157)
- *(tools)* Add annotate_corners HTML tool for absolute corner-error fixtures (#166)
- *(tools)* Per-corner edit + cursor-centered wheel zoom in annotate_corners (#166)
- *(examples)* Save C++ and Rust marker-outline PNGs in simple_nft_dual to visualize divergence
- *(tests)* Add cross-stack parity gate vs jsartoolkitNFT-Node (jsartoolkitNFT#584 Track 2, refs #170, #166 Track B)
- *(kpm)* Remove C++ FFI as default — pure Rust FreakMatcher complete (#142)
- *(kpm)* M9 — pure Rust KPM/NFT pipeline complete (#176)

### 🧪 Testing

- *(kpm)* Redefine M9 parity gate as corner reprojection (#152)
- *(fixtures)* Add annotated corner fixtures for the absolute corner-error gate (#166)
- *(fixtures)* Set annotator to kalwalt in the 5 annotated corner JSONs (#166)
- *(kpm)* Add absolute corner-error regression gate against annotated ground truth (#166)
- *(fixtures)* Replace blurry seq fixtures with screen-captured shots; drop nondeterministic frames (#166)
- *(kpm)* Add temporary REGEN block to capture post-fix EXPECTED_FULL_POSE on Linux
- *(kpm)* Regenerate test_full_pipeline_pose baseline against post-fix Linux values; drop REGEN block
- *(corner-error)* Regen baseline post-fix on Linux + widen epsilon to 3.5 px (residual BHC variance) (#170)

﻿## [0.6.1] - 2026-05-24

### 🐛 Bug Fixes

- *(ci)* Remove invalid --ignore-timeouts flag from tarpaulin config
- *(ci)* Add CODECOV_TOKEN to codecov upload step

### 🚀 Features

- Add Tarpaulin-based code coverage report

## [0.6.0] - 2026-05-18

## Milestone 8 — FREAK descriptor & DoG detector 

### ⚙️ Miscellaneous Tasks

- *(core)* Add purecv dependency for milestone 8 refs #125 #126

### 🐛 Bug Fixes

- *(kpm)* Relax dual-mode Gaussian-pyramid test to <= 1 ULP tolerance
- *(kpm)* Disable FP contraction in C++ build to restore cross-platform parity
- *(kpm)* Use full 15-digit C++ literals for orientation smoothing kernel
- *(kpm)* Silence clippy::excessive_precision on SMOOTH_KERNEL

### 📚 Documentation

- Add §3.5 documenting bilinear interpolation formula equivalence
- Add §3.4 documenting the NONMAX_CHECK macro replacement
- Update README and ARCHITECTURE with M8 milestone details

### 🚀 Features

- *(kpm)* Port BoxFilterPyramid8u to Rust (M8 step 1)
- *(kpm)* Port interpolate.h + BinomialPyramid32f to Rust (M8 step 2)
- *(kpm)* Port DoG detector + OrientationAssignment to Rust (M8 step 3)
- *(kpm)* Port FREAK descriptor + Keyframe to Rust (M8 step 4)

### 🧪 Testing

- *(kpm)* Add dual-mode test for DoG detector with find_orientation=true

## [0.5.1] - 2026-05-14

### ⚙️ Miscellaneous Tasks

- Convert generators to arlog_*! macros (PR 3/4 for #90)

### 🎨 Styling

- Fix rustfmt import order in nft_marker_gen.rs

### 🐛 Bug Fixes

- *(examples)* Use arlog_rel! for help banner and prompt UI per review

## [0.5.0] - 2026-05-13

### Milestone 7 — Hough voting & feature matching in pure Rust

### 🚀 Features

- *(kpm)* Implement Hough similarity voting algorithm (M7)
- *(kpm)* Implement binary hierarchical clustering and k-medoids for FREAK descriptors
- *(kpm)* Port FeatureStore and FeatureMatcher with three match variants
- *(kpm)* Port ArrayShuffle for BHC parity with C++ baseline

### 🐛 Bug Fixes

- *(kpm)* Fix arlog macro imports and unused variable warnings (#109)
- *(kpm)* Apply clippy and fmt fixes to hough.rs for M7
- *(kpm)* Resolve clippy warnings in clustering module
- *(kpm)* Resolve remaining clippy warnings in clustering
- *(kpm)* Include <limits> in kpm_c_api.cpp for GCC build
- *(kpm)* Move <limits> include before matcher headers for GCC build
- *(build)* Use platform-appropriate C++ stdlib (libc++ on macOS, libstdc++ on Linux)
- *(kpm)* Use combined absolute+relative tolerance in M6-2 dual-mode tests
- *(kpm)* Widen relative tolerance to 1e-5 for M6-2 dual-mode tests

### 🧪 Testing

- *(kpm)* Add dual-mode FFI tests for FeatureMatcher (M7-3)
- *(kpm)* Strengthen dual-mode FeatureMatcher tests with pair equality and global-best invariants

### 📚 Documentation

- Add pre-commit verification workflow and arlog import pattern to CLAUDE.md

### 🎨 Styling

- Fix formatting from cargo fmt

### ⚙️ Miscellaneous Tasks

- Run dual-mode tests in CI to enforce C++ parity
- Scope dual-mode tests to --lib to avoid pre-existing integration test failures

## [0.4.1] - 2026-05-09

### Patch release — Matrix code GlobalID, paramGL ports, and quality fixes

### 🚀 Features

- *(matrix)* Add ARMatrixCodeType::GlobalID variant and MatchOk.global_id field
- *(bch)* Implement BCH(127,64,22) decoder for AR_MATRIX_CODE_GLOBAL_ID
- *(matrix)* Add 14x14 GlobalID bit extraction with 4-direction traversal
- *(matrix)* Wire AR_MATRIX_CODE_GLOBAL_ID into ar_matrix_code_get_id and ar_get_marker_info
- *(matrix)* Add GlobalID support to matrix code type and update parsing logic
- *(core)* Implement ar_param_change_size and argl_camera_frustum_rh
- *(param_gl)* Port remaining three paramGL.c functions to param_gl.rs

### 🐛 Bug Fixes

- *(param_gl)* Fix clippy::doc_overindented_list_items on line 249
- *(arlog)* Scope test log capture to the thread running the arlog test

### 🚜 Refactor

- *(param)* Change ar_param_change_size to mutate ARParam in-place

### 📚 Documentation

- *(param_gl)* Fix C source reference from argl.c to paramGL.c

### ⚙️ Miscellaneous Tasks

- Finish converting load_nft.rs to arlog_*! macros

## [0.4.0] - 2026-05-06

### Milestone 6 — Math & homography in pure Rust

### 🐛 Bug Fixes

- *(kpm)* Satisfy clippy on freak::math constants and helpers
- *(bench)* Resolve SIMD benchmark compilation errors

### 📚 Documentation

- *(kpm)* Clarify SolveLinearSystem2x2 is dead in OUR build context

### 🚀 Features

- *(kpm)* Port FREAK math utilities from C++ (Milestone 6 Phase 1-3)
- *(kpm)* Port linear algebra and linear solvers (M6-2)
- *(kpm)* Port homography pipeline + Padé matrix exp (M6-3)

### 🧪 Testing

- *(kpm)* Add dual-mode validation for FREAK math fast-paths
- *(kpm)* Add dual-mode validation for M6-2 linear solvers
- *(kpm)* Add dual-mode validation for M6-3 homography pipeline

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
