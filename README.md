# WebARKitLib-rs 🦀

![WebARKitLib-rs](./assets/WebARKitLib-Rust-banner.jpg)

[![CI](https://github.com/webarkit/WebARKitLib-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/webarkit/WebARKitLib-rs/actions/workflows/ci.yml)
[![Miri](https://github.com/webarkit/WebARKitLib-rs/actions/workflows/miri.yml/badge.svg)](https://github.com/webarkit/WebARKitLib-rs/actions/workflows/miri.yml)
[![codecov](https://codecov.io/gh/webarkit/WebARKitLib-rs/branch/main/graph/badge.svg)](https://codecov.io/gh/webarkit/WebARKitLib-rs)
[![Crates.io](https://img.shields.io/crates/v/webarkitlib-rs.svg)](https://crates.io/crates/webarkitlib-rs)
[![npm](https://img.shields.io/npm/v/@webarkit/webarkitlib-wasm.svg)](https://www.npmjs.com/package/@webarkit/webarkitlib-wasm)
[![GitHub stars](https://img.shields.io/github/stars/webarkit/WebARKitLib-rs.svg?style=social)](https://github.com/webarkit/WebARKitLib-rs/stargazers)

**WebARKitLib-rs** is a high-performance, memory-safe Rust port of the [WebARKitLib](https://github.com/webarkit/WebARKitLib) (originally C/C++). 

This project aims to provide a pure-Rust implementation of the core ARToolKit algorithms, targeting both **native** systems and **WebAssembly (WASM)** for high-performance augmented reality in the browser.

> [!NOTE]
> This project is currently a **Work in Progress**. Core marker detection and pose estimation are functional. KPM/NFT (Natural Feature Tracking) is in an early stage with partial Rust porting -- a fully idiomatic Rust KPM pipeline with a working example is the primary goal for v1.0.0.

## 🌟 Key Features

- **Pure Rust**: Built for safety, speed, and modern concurrency. ([crates.io](https://crates.io/crates/webarkitlib-rs))
- **Dual WASM Strategy**: Automated release of both **Standard** and **SIMD** binaries to maximize performance and compatibility.
- **SIMD-Accelerated Pattern Matching**: Uses `i16` fixed-point arithmetic and WASM `i32x4_dot_i16x8` instructions for ultra-fast correlation matching.
- **Barcode Marker Support**: Robust decoding for square matrix markers (3x3 to 6x6) with ECC.
- **WASM Ready**: High-performance tracking in the browser via WebAssembly. ([@webarkit/webarkitlib-wasm](https://www.npmjs.com/package/@webarkit/webarkitlib-wasm))
- **Side-Effect Free**: Pure mathematical engine, easy to test and integrate.
- **CI-Integrated Benchmarking**: Performance parity with original C implementation.

## ⚡ Performance & Accuracy

- **SIMD-Accelerated Pattern Matching**: Uses `i16` fixed-point arithmetic for the core correlation loops, leveraging WASM `i32x4_dot_i16x8` instructions for significant speedups.
- **Numerical Parity**: Our SIMD and Scalar paths are calibrated to produce bit-identical results (including rounding logic in grayscale and thresholding), ensuring deterministic behavior across all devices.
- **Sub-millisecond Tracking**: Core detection loop is optimized for sub-millisecond execution on standard ARM/x86 hardware.

## 🚀 Getting Started

### Add to Your Project

Add `webarkitlib-rs` to your `Cargo.toml`:

```toml
[dependencies]
webarkitlib-rs = "0.7"
```

### Pure Rust tracking (no C++ compiler required)

Since **M9-3 (#142)**, the default backend is the pure-Rust
`FreakMatcher`. A plain `cargo add webarkitlib-rs` (or the snippet
above) gives you a fully functional NFT/KPM tracker with **no C++
toolchain, libclang, or `cc` required at build time**:

```bash
cargo build         # works on machines without clang/libclang/cc
cargo test
```

The whole `crates/core/build.rs` `FreakMatcher` C++ compilation path
is gated behind `cfg!(feature = "ffi-backend")` — opt-in only.

### Building without C++

If you are integrating WebARKitLib-rs from a containerized environment
or any host that doesn't have a C++ toolchain installed, you don't
need to do anything special:

```toml
[dependencies]
webarkitlib-rs = "0.7"  # default backend is pure Rust
```

```bash
cargo build  # succeeds without a C++ compiler installed
```

### Opt-in: C++ FFI backend (for validation only)

The C++ `FreakMatcher` is still available as an opt-in backend, used
internally for cross-validation (`--features dual-mode`), regression
testing (`kpm_regression`), and the cross-stack parity gate
(`cross_stack_parity`). It is **no longer required for production
NFT tracking**:

```toml
[dependencies]
webarkitlib-rs = { version = "0.7", features = ["ffi-backend"] }
```

> When installing from crates.io, no extra setup is required — the C++
> sources needed by `ffi-backend` ship inside the published crate.

### Contributing — clone with submodules

This repository uses a git submodule for the WebARKitLib C++ sources that
back the `ffi-backend` feature. Clone recursively:

```bash
git clone --recursive https://github.com/webarkit/WebARKitLib-rs.git
```

If you already cloned without `--recursive`:

```bash
git submodule update --init --recursive
```

The submodule lives at `crates/core/third_party/WebARKitLib` and is pinned
to a specific upstream commit. Building from a non-recursive clone will
fail early in `build.rs` with an actionable message.

### Native Rust Example

To see the marker detection in action on your local machine, run the provided simple example:

```bash
cargo run --example simple
```

This example loads a camera parameter file, a marker (pattern or barcode), and a sample image, performing detection and outputting the 3D pose extrinsics.

#### NFT Marker Generation Example

Generate NFT (Natural Feature Tracking) marker files compatible with ARnft and NFT-Marker-Creator-App:

```bash
# Pure Rust, no C++ toolchain — produces a COMPLETE marker (.iset + .fset + .fset3):
cargo run --release --example nft_marker_gen -- \
  --input path/to/image.jpg \
  --output path/to/output_name \
  --dpi 220

# With SIMD acceleration + Rayon parallelism (recommended for x86_64):
cargo run --release --features "simd-x86-sse41,simd-x86-avx2" \
  --example nft_marker_gen -- \
  --input path/to/image.jpg \
  --output path/to/output_name \
  --dpi 220
```

> Since #179, `.fset3` (FREAK descriptors) is generated by the pure-Rust
> `RustFreakMatcher` — a single invocation produces a complete marker with no
> C++ toolchain.

> **Important:** Always use the `--release` flag. Debug builds are 5–10× slower due to missing compiler optimizations.

Output files (all produced on the pure-Rust default):

| File | Description |
|---|---|
| `<output>.iset` | JPEG-compressed image pyramid (~300 KB, matches C++ `ar2WriteImageSet()` format) |
| `<output>.fset` | Feature map with one entry per pyramid level |
| `<output>.fset3` | FREAK descriptors for KPM-based recognition (pure-Rust `RustFreakMatcher`) |

**Options:**

| Flag | Long form | Description | Default |
|------|-----------|-------------|---------|
| `-i` | `--input` | Path to the source image (JPEG/PNG) | required |
| `-o` | `--output` | Output base path (without extension) | required |
| `-d` | `--dpi` | Source image resolution in DPI | required |
| `-l` | `--level` | Tracking extraction level (0–4) | `2` |

**Performance feature flags:**

| Feature flag | What it enables |
|---|---|
| `ffi-backend` | C++ FREAK backend (opt-in since M9-3 #142). Used only for development cross-validation (`dual-mode`, `cross_stack_parity`); all of `nft_marker_gen` (`.iset` + `.fset` + `.fset3`) and runtime NFT tracking run on the pure-Rust default. |
| `simd-x86-sse41` | SSE4.1 SIMD acceleration for feature map correlation (`get_similarity`) |
| `simd-x86-avx2` | AVX2+FMA SIMD acceleration for feature map correlation (faster than SSE4.1) |
| `simd` | Umbrella flag — enables all SIMD optimizations (SSE4.1, AVX2, WASM SIMD, image, pattern) |

Rayon-based parallelism is always enabled (no feature flag needed): pyramid level generation and Stage-3 feature scoring run in parallel across CPU cores.

#### Barcode Detection Example

A unified, parameterized barcode example is available for testing all supported matrix code types:

```bash
# Default: 3x3 matrix code type, auto-sweeps threshold 60–180
cargo run --example barcode

# Specify a matrix code type (e.g. 4x4) and supply the matching marker image
cargo run --example barcode -- -m 4x4 -i crates/core/examples/Data/marker_07_4x4.jpg

# Use a fixed threshold instead of auto-sweeping
cargo run --example barcode -- -m 3x3 -t 100

# Full example: type, threshold, and custom image path
cargo run --example barcode -- -m 5x5 -t 120 -i path/to/marker.jpg
```

**Options:**

| Flag | Long form | Description | Default |
|------|-----------|-------------|---------|
| `-m` | `--matrix-code-type` | Matrix code type (`3x3`, `3x3parity65`, `3x3hamming63`, `4x4`, `4x4bch1393`, `4x4bch1355`, `5x5`, `5x5bch22125`, `5x5bch2277`, `6x6`) | `3x3` |
| `-t` | `--threshold` | Fixed labeling threshold (0–255). When omitted, sweeps 60–180 in steps of 20 | *(auto-sweep)* |
| `-i` | `--image` | Path to the input image | bundled 3x3 marker image |

### WebAssembly (WASM)

The WASM port allows you to run the AR engine directly in most modern browsers.

#### Using the npm package

For integration into your own project, install the pre-built package from npm — no local build required:

```bash
npm install @webarkit/webarkitlib-wasm
```

See [@webarkit/webarkitlib-wasm](https://www.npmjs.com/package/@webarkit/webarkitlib-wasm) for API documentation and usage examples.

#### Running the bundled demos

The `crates/wasm/www` folder contains interactive web demos. To run them you need to build the WASM modules locally:

1. **Build the modules** (generates both Standard and SIMD bundles):
   ```bash
   npm run build:wasm
   ```
   > **Alternatively**, download the pre-built `wasm-package` artifact from the latest [CI run](https://github.com/webarkit/WebARKitLib-rs/actions) and extract its contents into `crates/wasm/pkg/`.

2. **Serve the demo**:
   ```bash
   cd crates/wasm/www
   npx serve .
   ```
   - **`simple.html`** – static image demo with engine selector and threshold visualization.
   - **`simple_video_marker_example.html`** – live webcam demo with engine selector, marker type (pattern/barcode) selector, and threshold slider.

## 📝 Logging

WebARKitLib-rs uses the standard [`log`](https://crates.io/crates/log) crate facade, with ARToolKit-style macros (`arlog_d!`, `arlog_i!`, `arlog_w!`, `arlog_e!`, `arlog_rel!`, `arlog_perror!`) that mirror the C `ARLOG*` API:

| C macro                   | Rust macro               |
|---------------------------|--------------------------|
| `ARLOGd("x=%d", x);`      | `arlog_d!("x={}", x);`   |
| `ARLOGi("x=%d", x);`      | `arlog_i!("x={}", x);`   |
| `ARLOGw("x=%d", x);`      | `arlog_w!("x={}", x);`   |
| `ARLOGe("x=%d", x);`      | `arlog_e!("x={}", x);`   |
| `ARLOG("banner\n");`      | `arlog_rel!("banner");`  |
| `ARLOGperror("open");`    | `arlog_perror!("open");` |

### Quick start — reproduce ARToolKit's C output format

Enable the `log-helpers` feature and call the bundled initializer once in your binary:

```toml
[dependencies]
webarkitlib-rs = { version = "0.7", features = ["log-helpers"] }
```

```rust
fn main() {
    webarkitlib_rs::arlog::ar_log_init_default();
    // ... your application
}
```

Produces `[info] ...`, `[warning] ...`, `[error] ...`, `[debug] ...` — matching the C output exactly. Release-info messages (`arlog_rel!`) print unprefixed.

### Verbose mode

For richer output (timestamp + originating module), use the verbose initializer:

```rust
fn main() {
    webarkitlib_rs::arlog::ar_log_init_default_verbose();
    // ... your application
}
```

Produces a single bracketed header per line:

```text
[info - 2026-04-21T14:23:45Z - webarkitlib_rs::marker] hello
[warning - 2026-04-21T14:23:45Z - webarkitlib_rs::ar2] low mem
[error - 2026-04-21T14:23:45Z - webarkitlib_rs::kpm] bad fd
```

Filtering still respects `RUST_LOG`; only the formatting changes. `arlog_rel!` messages stay prefix-free in verbose mode too. On `wasm32`, use `ar_log_init_wasm_verbose()`, which raises the level to `Debug` (browser DevTools renders the timestamp and source location natively).

### Controlling verbosity

Set via environment variable (honored by the default helper):

```bash
RUST_LOG=debug cargo run --example simple
```

Or programmatically:

```rust
use webarkitlib_rs::arlog::{set_ar_log_level, ArLogLevel};
set_ar_log_level(ArLogLevel::Debug);
```

### Other backends

Because it's the `log` crate facade, any compatible backend works:

- **WASM / browser** — `console_log` (or the bundled `ar_log_init_wasm()` helper under the `log-helpers` feature)
- **Android** — `android_logger`
- **Apple (iOS/macOS)** — `oslog`
- **Structured / OpenTelemetry** — `tracing` + `tracing-log`

No library code change is needed — pick the backend in your application's entry point.

## 📈 Code Coverage

Coverage reports are generated automatically on every push and pull request via the [coverage workflow](.github/workflows/coverage.yml) using [cargo-tarpaulin](https://github.com/xd009642/tarpaulin) and uploaded to [Codecov](https://codecov.io/gh/webarkit/WebARKitLib-rs).

### Generate a Coverage Report Locally

```bash
# Install tarpaulin (once)
cargo install cargo-tarpaulin

# Run the helper script — produces coverage/index.html
./scripts/coverage.sh
```

Or run tarpaulin directly:

```bash
cargo tarpaulin --workspace --out Html --output-dir coverage --timeout 300
```

### Coverage Targets

| Area | Target |
|------|--------|
| Minimum overall | 75 % |
| Desirable overall | 85 %+ |
| Marker tracking (critical path) | 95 %+ |

---

## 📊 Benchmarking

We maintain a strict performance comparison with the original C library to ensure our Rust port remains competitive. 

Detailed SIMD performance results and reproduction steps can be found in the [BENCHMARKS.md](crates/core/benches/BENCHMARKS.md) file.

### Running the Comparison

1.  **Bootstrap the C library**:
    ```bash
    cd benchmarks/c_benchmark
    python ../bootstrap.py --bootstrap-file libraries.json
    ```

    > Note: this Python bootstrap is only required for the standalone
    > **C benchmark** build here. The Rust `ffi-backend` feature gets its
    > C++ sources from the git submodule at
    > `crates/core/third_party/WebARKitLib` and does **not** require
    > Python.

2.  **Execute the Suite**:
    ```bash
    # Rust Benchmark
    cargo bench -p webarkitlib-rs --bench marker_bench

    # C Benchmark (Manual build required once)
    cd benchmarks/c_benchmark/build
    cmake --build . --config Release
    ./c_benchmark ../../data/camera_para.dat ../../data/patt.hiro ../../data/hiro.raw 429 317
    ```

### Tracking Performance

We use `criterion` to track performance over time. You can save specific snapshots as "baselines":

```bash
# Save a milestone baseline
cargo bench -- --save-baseline milestone-20260307
```

## 🏗️ Project Structure

The workspace contains two crates:

- **`crates/core`** (`webarkitlib-rs`): The unified core AR engine (pure Rust), including:
  - `ar2` module: NFT marker generation pipeline — image pyramid (`ar2_gen_image_set`), feature map (`ar2_gen_feature_map`), JPEG-compressed `.iset` save, `.fset` / `.fset3` I/O.
  - `kpm` module: Keypoint Matching with a pluggable `FreakMatcherBackend` trait (M9-2). Since M9-3 (#142), the **default backend is pure Rust** — `cargo build` requires no C++ toolchain. The C++ FFI backend is opt-in via `--features ffi-backend`.
    - `kpm::rust_backend`: Default `RustFreakMatcher` + `DualFreakMatcher` (parity-asserting wrapper used by regression tests) (M9-2).
    - `kpm::cpp_backend`: Opt-in C++ `CppFreakMatcher` FFI shim, gated behind `cfg(feature = "ffi-backend")` (M9-3).
    - `kpm::freak::visual_database`: Pure-Rust `VisualDatabase` — descriptor storage, BHC index, `queryByFeatures`, homography-guided refinement (M9-1).
    - `kpm::freak::math` / `kpm::freak::homography`: pure-Rust math & homography pipeline (M6).
    - `kpm::freak::hough`: Hough similarity voting — 4D-binned voting scheme for finding the consistent similarity transformation across matched feature pairs (M7). Uses `BTreeMap` for deterministic bin iteration across platforms (M9 #171).
    - `kpm::freak::clustering`: K-Medoids + Binary Hierarchical Clustering (BHC) vocabulary tree for fast approximate-NN search on 96-byte FREAK descriptors. Byte-identical PRNG (`FastRandom` / `ArrayShuffle`) for C++ parity (M7).
    - `kpm::freak::matcher`: `FeatureStore` + `FeatureMatcher` with three match variants (brute, BHC-indexed, homography-guided), each gated by C++-faithful ratio test and `maxima` filtering (M7).
  - Core modules: image processing, pattern matching, labeling, ICP, pose estimation.
- **`crates/wasm`** (`webarkitlib-wasm`): WASM bindings, dual-build scripts, and diagnostic web demo.
- `benchmarks`: C vs Rust performance comparison suite.
- [`ARCHITECTURE.md`](ARCHITECTURE.md): Detailed technical overview of the library's design and SIMD optimizations.

## 🗺️ Roadmap

### Completed Milestones

- **M1 -- KPM/NFT Core (partial)**: Ported the initial KPM (Keypoint Matching) scaffolding -- binary feature types, matching orchestration, ICP-based pose refinement, and `.fset3` / `.iset` / `.fset` I/O. The C++ FreakMatcher FFI backend works but the full pipeline is not yet wired end-to-end.
- **M2 -- AR2 I/O**: Ported AR2 image set (`.iset`) and feature set (`.fset`) binary I/O from C to Rust.
- **M3 -- Architectural Consolidation**: Unified the workspace from 4 crates down to 2 (`webarkitlib-rs` + `webarkitlib-wasm`), with KPM and AR2 as submodules of the core crate.
- **M4 -- Working NFT Marker Generator**: End-to-end `nft_marker_gen` example producing `.iset` / `.fset` / `.fset3` files fully compatible with ARnft and NFT-Marker-Creator-App, including:
  - JPEG-compressed `.iset` save matching C++ `ar2WriteImageSet()` format (~300 KB vs ~10 MB raw)
  - All pyramid levels included in `.fset` (including 0-feature levels), matching C++ output exactly
  - FREAK descriptor extraction via `kpm_extract_features` C API (7000+ features per marker)
  - Detailed step-by-step logging with timestamps and per-level statistics
- **M5 -- NFT Pipeline Performance**: Rayon parallelism for pyramid generation and Stage-3 feature scoring, plus optional SSE4.1/AVX2+FMA SIMD vectorization of the `get_similarity` correlation kernel. ~1.7× total speedup on x86_64.
- **M6 -- Math & homography in pure Rust**: Ported all free mathematical functions from FreakMatcher into `crates/core/src/kpm/freak/math.rs` and `freak/homography.rs` (~1476 lines across 5 C++ headers). Eliminates the only remaining Eigen dependency: the 3×3 matrix exponential in `IncrementalHomographyFromLieWeights`.
- **M7 -- Hough voting & feature matching in pure Rust**: Ported the full FreakMatcher matching pipeline into `crates/core/src/kpm/freak/{hough,clustering,matcher}.rs` (~2790 LOC total):
  - Hough similarity voting (4D bin discretization) — 734 LOC
  - K-Medoids clustering + Binary Hierarchical Clustering vocabulary tree, including a byte-identical `FastRandom` / `ArrayShuffle` PRNG port — 889 LOC
  - `FeatureStore` + `FeatureMatcher` with three match variants (brute, BHC-indexed, homography-guided), C++-faithful ratio test (default 0.7), and `maxima` filtering — 1167 LOC
  - **Dual-mode FFI tests run in CI on Linux / macOS / Windows** with sorted-pair equality vs the C++ baseline for brute/indexed/guided matchers (the M7 milestone validation gate). Surfaced and fixed three latent cross-platform issues: macOS `libc++` linking, GCC `<limits>` include order, and ARM64 FMA tolerance in M6-2 solvers.
- **M8 -- KPM image pyramid & feature extraction in pure Rust** (4 steps):
  - **Step 1**: `BoxFilterPyramid8u` — box filtering for efficient 8-bit grayscale pyramid construction
  - **Step 2**: `interpolate.h` + `BinomialPyramid32f` — binomial pyramid and interpolation utilities for scale-space analysis
  - **Step 3**: `DoG detector` + `OrientationAssignment` — difference-of-Gaussians keypoint detection and dominant orientation estimation
  - **Step 4**: `FREAK descriptor` + `Keyframe` — native Rust FREAK descriptor computation and keyframe pipeline
  - Completes the KPM feature extraction and image pyramid components in pure Rust
- **M9 -- End-to-end pure-Rust KPM/NFT pipeline** ([#139](https://github.com/webarkit/WebARKitLib-rs/issues/139), delivered by [PR #176](https://github.com/webarkit/WebARKitLib-rs/pull/176)) — 3 sub-milestones, 16 sub-PRs, +9k LOC:
  - **M9-1** ([#140](https://github.com/webarkit/WebARKitLib-rs/issues/140)): Pure-Rust `VisualDatabase` port — FREAK descriptor storage, BHC vocabulary tree, Hough voting, homography-guided matching, byte-identical `FastRandom`/`ArrayShuffle` PRNG vs C++.
  - **M9-2** ([#141](https://github.com/webarkit/WebARKitLib-rs/issues/141)): Pluggable `FreakMatcherBackend` trait — `RustFreakMatcher` (default), `CppFreakMatcher` (FFI), and `DualFreakMatcher` (parity-asserting). New `simple_nft_dual` diagnostic example.
  - **M9-3** ([#142](https://github.com/webarkit/WebARKitLib-rs/issues/142)): **Pure Rust is now the default backend.** `cargo build` works on machines with **no C++ toolchain (no clang / libclang / cc)**. The C++ FFI is opt-in via `--features ffi-backend` for cross-validation and `nft_marker_gen` only. A dedicated `pure-rust-build` CI job guards the invariant.
  - **Cross-cutting wins**:
    - Cross-platform / cross-stack matcher determinism — Rust `HashMap` → `BTreeMap` and upstream C++ `unordered_map` → `std::map` ([WebARKitLib#39](https://github.com/webarkit/WebARKitLib/pull/39)).
    - Hand-annotated absolute corner-error regression gate (browser-based annotation tool + 5 fixtures + Linux CI gate). Finding: pure-Rust backend is more accurate than C++ on `pinball-demo` (5.27 px vs 18.79 px max corner error).
    - Cross-stack parity gate against `@webarkit/jsartoolkit-nft@1.10.0` ([jsartoolkitNFT#584](https://github.com/webarkit/jsartoolkitNFT/pull/584)) — guarantees native Rust pose matches what production WASM consumers see.

### 🎯 Short-term Goals (toward v1.0.0)
- **KPM-specific benchmark** ([deferred from #142](https://github.com/webarkit/WebARKitLib-rs/issues/142)): Add a dedicated `kpm_bench.rs` Criterion bench so we can verify the pure-Rust backend stays within 20% of C++ on `pinball-demo`. The existing `marker_bench` only measures barcode marker detection.
- **WASM browser examples for KPM/NFT** ([#161](https://github.com/webarkit/WebARKitLib-rs/issues/161)): End-to-end runnable browser demo of the pure-Rust NFT pipeline.
- **Raise M9 patch coverage** ([#177](https://github.com/webarkit/WebARKitLib-rs/issues/177)): Lift M9 modules from 84.76% → ≥90% patch coverage with no file under 85%.
- **Dependency upgrades**: Criterion 0.5 → 0.8 ([#174](https://github.com/webarkit/WebARKitLib-rs/issues/174)).
- **Enhanced Documentation**: Expand API reference with complete module-level docs, integration walkthroughs for JS/TS, and detailed usage examples.
- **WASM Memory Management**: Improve resource cleanup when switching engines or markers in long-running browser sessions.

### 🔭 Long-term Vision (post v1.0.0)
- **Multi-Marker Support**: Port `arMulti` logic to enable tracking of multiple markers simultaneously.
- **Advanced Video Abstraction**: Develop a cross-platform video handling layer to simplify integration with various input sources.

## 📜 Credits

This project is a port of the excellent [WebARKitLib](https://github.com/webarkit/WebARKitLib) project. Special thanks to the original ARToolKit contributors.

## 📄 License

This project is licensed under the LGPLv3 License - see the [LICENSE](LICENSE) file for details.
