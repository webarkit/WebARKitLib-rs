# WebARKitLib.rs Core Benchmarks

**Last Updated**: 2026-06-10 (chore: criterion 0.5.1 → 0.8 — #174)

This document tracks the performance of critical image processing and pattern matching functions in the `webarkitlib_rs` core crate.

## SIMD Performance (x86_64 SSE4.1)

The following benchmarks were conducted on an x86_64 system with SSE4.1 support enabled via the `simd` feature.

| Function | Implementation | Median Time | Speedup |
| :--- | :--- | :--- | :--- |
| `rgba_to_gray` | Scalar | 629.57 µs | - |
| | SIMD (SSE4.1) | 238.42 µs | **2.64x** |
| `dot_product` | Scalar | 326.45 ns | - |
| | SIMD (SSE4.1) | 60.24 ns | **5.42x** |
| `box_filter_h` | Scalar | 1.8386 ms | - |
| | SIMD (SSE4.1) | 1.7245 ms | 1.07x |
| `box_filter_v` | Scalar | 2.3240 ms | - |
| | SIMD (SSE4.1) | 765.77 µs | **3.03x** |

### Analysis

1.  **`dot_product`**: The most significant gain (6.4x) was achieved here. As a purely compute-bound task processing `i16` values, it maps perfectly to 128-bit SIMD registers (processing 8 elements at once).
2.  **`rgba_to_gray`**: Doubling the speed of the grayscale conversion is a major win for the main processing pipeline. Further gains might be limited by memory bandwidth.
3.  **`box_filter`**:
    *   **Vertical Pass**: Shows a strong ~3x speedup by processing 16 columns of pixels in parallel.
    *   **Horizontal Pass**: Currently shows no improvement. This is common in horizontal filters due to the overhead of unaligned memory access patterns or being entirely memory-bound.

## Reproducing Benchmarks

To run these benchmarks on your own machine:

```powershell
# Run with SIMD optimizations enabled (requires SSE4.1 on x86)
cargo bench --features simd --bench simd_bench

# Run without SIMD (Scalar only)
cargo bench --bench simd_bench
```

## Setup Details

- **Tooling**: [Criterion.rs](https://github.com/bheisler/criterion.rs) `0.8`
- **Target OS**: Windows
- **Target Architecture**: x86_64 (SSE4.1)
- **Frame Size**: 640x480 (typical AR video resolution)
- **Toolchain**: `rustc 1.94.0 (stable)`
- **SIMD activation**: SSE4.1 intrinsics are gated by `#[cfg(target_feature = "sse4.1")]` — set `RUSTFLAGS="-C target-feature=+sse4.1"` (or `target-cpu=native`) when running `simd_bench` to exercise the SIMD paths.

## KPM / NFT performance (M9-3 status)

Issue #142's acceptance criterion calls for the pure-Rust NFT pipeline
to run within 20% of the C++ backend on `pinball-demo`. As of M9-3,
**there is no dedicated benchmark exercising the KPM / FreakMatcher
path**. The existing `marker_bench` measures `ar_detect_marker`
(barcode/template marker detection), which doesn't touch the
FreakMatcher and therefore can't distinguish pure-Rust from the C++
FFI backend.

### Functional parity evidence (in lieu of wall-clock numbers)

The Rust and C++ backends agree on the meaningful outputs across
several test suites:

| Test | What it asserts | Status post-#170 |
|------|------------------|------------------|
| `test_dual_mode_no_divergence_on_pinball` | M9 #152 tier-2: `max_corner_displacement < 2.0 px` between C++ and Rust homographies on `found.jpg/img.jpg` | ✅ zero divergences |
| `absolute_corner_error` (#166 Track A) | Each backend's max corner-error against hand-annotated ground truth stays within baseline + 3.5 px epsilon | ✅ green; **Rust is more accurate than C++** on `pinball-demo` (Rust 5.27 px vs C++ 18.79 px) |
| `cross_stack_parity` (jsartoolkitNFT#584 Track 2) | C++ FFI and Rust pose agree with jsartoolkitNFT-Node within rot 0.08 / trans 10 mm | ✅ green |
| `kpm_regression::test_full_pipeline_pose` | Linux C++ pose matches committed numerical baseline to 1e-2 | ✅ green |

The within-20% perf target is treated as **deferred, not failed**:
the functional evidence shows Rust meets parity by every quality
metric we measure, and #142 explicitly permits deferring the
quantitative perf check to a follow-up: *"If slower, open a follow-up
performance issue rather than blocking this PR."*

### `kpm_bench.rs` — dedicated KPM wall-clock (#225)

`kpm_bench.rs` loads `pinball.fset3` + `pinball-demo.jpg` and times a
single `KpmHandle::kpm_matching` query (setup — ref-data load + handle
build — is done once outside the measured loop). Run it with:

```sh
cargo bench -p webarkitlib-rs --bench kpm_bench
```

Baseline (pure-Rust `RustFreakMatcher`, `pinball-demo.jpg` at
2000×1500, release build):

| Backend | Query time (median) |
|---------|---------------------|
| Rust (`RustFreakMatcher`) | ~0.30 s |

The within-20% comparison against the C++ `CppFreakMatcher` is wired
by building with `--features ffi-backend` and swapping the backend in
the bench; numbers are hardware-dependent, so treat the committed
figure as an order-of-magnitude reference, not a hard CI gate.
