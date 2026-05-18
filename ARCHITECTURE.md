# WebARKitLib.rs Architecture

WebARKitLib.rs is a high-performance system library for Augmented Reality, ported from C/C++ to Rust. It is designed to be side-effect-free, targeting both native and WebAssembly (WASM) environments.

## Core Design Principles

1.  **Pure Systems Programming**: Focus on low-level arithmetic and image processing without external side effects (like direct camera access or rendering).
2.  **Safety First**: Leverages Rust's memory safety guarantees, using `unsafe` only where strictly necessary for performance (SIMD) or C++ FFI.
3.  **SIMD Acceleration**: Uses platform-specific SIMD intrinsics (x86 SSE4.1/AVX2 and WASM SIMD128) to accelerate bottle-neck operations.
4.  **WASM Optimized**: Designed to be compiled to WASM and used in web environments with minimal overhead.

## Project Structure

The project is organized as a Cargo workspace with two crates:

### `crates/core` — `webarkitlib-rs`

The unified core library containing all AR functionality:

-   **Image processing** (`image_proc`): filters, thresholding, histogram.
-   **Pattern matching** (`pattern`): template tracking and pattern recognition.
-   **Labeling** (`labeling`): connected component labeling.
-   **Math / Matrix** (`math`, `matrix`): linear algebra and geometric calculations.
-   **ICP** (`icp`): Iterative Closest Point pose refinement.
-   **Pose estimation** (`pose`): camera pose from marker geometry.
-   **AR2 module** (`ar2`): NFT (Natural Feature Tracking) subsystem:
    -   `ar2::tracking` — runtime tracking structs and algorithms.
    -   `ar2::image_set` — `.iset` image pyramid I/O.
    -   `ar2::feature_set` — `.fset` feature point I/O.
-   **KPM module** (`kpm`): Keypoint Matching subsystem (FREAK descriptor-based tracking):
    -   `kpm::handle` — high-level KPM handle and matching orchestration.
    -   `kpm::backend` — pluggable feature-extraction backend trait and error types.
    -   `kpm::cpp_backend` — C++ FreakMatcher FFI backend (feature-gated: `ffi-backend`).
    -   `kpm::matching` — per-frame matching and ICP-based pose estimation.
    -   `kpm::ref_data_set` — `.fset3` reference data I/O and compression modes.
    -   `kpm::types` — KPM data structures and constants.
    -   `kpm::freak` — FREAK descriptor math, homography, and matching utilities (pure Rust port of `WebARKitLib/lib/SRC/KPM/FreakMatcher`):
        -   `freak::math` — linear algebra (matrix operations, linear solvers) and Padé matrix exponential.
        -   `freak::homography` — homography estimation and refinement pipeline.
        -   `freak::hough` — Hough similarity voting (4D bin discretization over translation × angle × scale) for filtering matches by transformation consistency.
        -   `freak::clustering` — K-Medoids partitioning + Binary Hierarchical Clustering (BHC) vocabulary tree for fast approximate-NN search on 96-byte FREAK descriptors. Hamming distance via 24×32-bit bit-magic. Includes a byte-identical port of C++ `vision::FastRandom` / `vision::ArrayShuffle` so the BHC tree topology matches the C++ baseline given the same seed.
        -   `freak::matcher` — `FeatureStore` (points + flat descriptor buffer) and `FeatureMatcher` with three match variants: brute force, BHC-indexed (fast path), and homography-guided (spatial filter via 3×3 inverse + `tr` radius). All three apply the C++ ratio test (default 0.7) and filter by `FeaturePoint::maxima`.
        -   `freak::image_pyramid` — image pyramid construction via `BoxFilterPyramid8u` (box filtering for 8-bit grayscale) and `BinomialPyramid32f` (32-bit floating-point binomial pyramid with scale-space interpolation).
        -   `freak::feature_extraction` — keypoint detection via Difference-of-Gaussians (DoG), dominant orientation assignment via circular gradient voting, and FREAK descriptor computation with native Rust implementation of the FREAK binary pattern and bit-pair comparison.
-   **Types** (`types`): core data structures (`ARHandle`, `ARParam`, etc.).

### `crates/wasm` — `webarkitlib-wasm`

WASM wrapper and JavaScript/TypeScript glue code for browser targets.
Depends only on `webarkitlib-rs` (the core crate).

## Feature Flags

| Feature        | Description |
|----------------|-------------|
| `simd`         | Umbrella: enables all SIMD sub-features |
| `simd-wasm32`  | WASM SIMD128 intrinsics |
| `simd-x86-sse41` | x86 SSE4.1 intrinsics |
| `log-helpers`  | Enable logging infrastructure (installs `env_logger` for desktop/tests, `console_log` for WASM) |
| `ffi-backend`  | Compile the C++ FreakMatcher library and generate FFI bindings |
| `dual-mode`    | Enables FFI-based parity tests that validate pure-Rust ports against the live C++ baseline (M6 math/solvers/homography, M7 BHC/matcher, PRNG). Transitively enables `ffi-backend`. Run in CI on Linux/macOS/Windows. |

## SIMD Strategy

Performance-critical functions are optimized using SIMD. The strategy involves:

-   **Granular Feature Flags**: Users can opt-in to SIMD optimizations via cargo features (`simd-wasm32`, `simd-x86-sse41`, or the umbrella `simd`).
-   **Static Dispatch**: SIMD implementations are chosen at compile-time based on target architecture and enabled features.
-   **Fixed-Point Arithmetic**: Many operations use fixed-point arithmetic (`i16` or `i32`) to leverage integer SIMD performance.

### Key Optimized Components

-   **Image Filters**: `box_filter_h` and `box_filter_v` are optimized for x86 and WASM.
-   **Pattern Matching**: Fixed-point dot product and correlation calculations.

## Building and Testing

### Native
```bash
cargo build --release --features simd
cargo test --workspace --features simd
```

### With C++ FFI backend
```bash
# Bootstrap C++ sources first (one-time setup)
cd benchmarks/c_benchmark && python ../bootstrap.py --bootstrap-file libraries.json && cd ../..

cargo build --features ffi-backend
cargo test --workspace --features ffi-backend
```

### Dual-mode parity tests (Rust ↔ C++)

Validates the pure-Rust ports against the live C++ baseline via FFI shims.
Requires the C++ submodule to be initialized:

```bash
cargo test -p webarkitlib-rs --lib --features dual-mode
```

CI runs this step on Linux, macOS, and Windows on every push.

### WASM
```bash
cd crates/wasm
wasm-pack build -- --features simd
```

## Performance Benchmarks

Benchmarks are located in `crates/core/benches`. To run them:
```bash
cargo bench --features simd
```
