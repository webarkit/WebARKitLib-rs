# WebARKitLib.rs Architecture

WebARKitLib.rs is a high-performance system library for Augmented Reality, ported from C/C++ to Rust. It is designed to be side-effect-free, targeting both native and WebAssembly (WASM) environments.

## Core Design Principles

1.  **Pure Systems Programming**: Focus on low-level arithmetic and image processing without external side effects (like direct camera access or rendering).
2.  **Safety First**: Leverages Rust's memory safety guarantees, using `unsafe` only where strictly necessary for performance (SIMD).
3.  **SIMD Acceleration**: Uses platform-specific SIMD intrinsics (x86 SSE4.1/AVX2 and WASM SIMD128) to accelerate bottle-neck operations.
4.  **WASM Optimized**: Designed to be compiled to WASM and used in web environments with minimal overhead.

## Project Structure

The project is organized as a Cargo workspace with several crates:

-   `crates/core`: The core logic of the library, including:
    -   `image_proc`: Image processing utilities (filters, thresholding).
    -   `pattern`: Pattern matching and template tracking.
    -   `labeling`: Connected component labeling.
    -   `math`/`matrix`: Linear algebra and geometric calculations.
    -   `types`: Core data structures.
-   `crates/wasm`: The WASM wrapper and JavaScript Glue code.

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
