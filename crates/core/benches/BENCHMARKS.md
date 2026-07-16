# WebARKitLib.rs Core Benchmarks

**Last Updated**: 2026-07-15 (perf: add dedicated `kpm_bench.rs` — #225)

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

The crate registers these benches (all `harness = false`):

| Bench | Measures |
| :--- | :--- |
| `simd_bench` | Scalar vs SIMD kernels — the table above (`--features simd-x86-sse41`) |
| `marker_bench` | `ar_detect_marker` + pose — barcode/template marker pipeline |
| `feature_map_bench` | `ar2_gen_feature_map` — NFT marker generation (`--features log-helpers`) |
| `pyramid_bench` | Box-filter pyramid downsample (kept-but-unused reference, #203) |
| `gaussian_pyramid_bench` | Gaussian scale-space pyramid build (#200/#201/#207) |
| `kpm_bench` | `KpmHandle::kpm_matching` — KPM/NFT detection (#225; see below) |

## Setup Details

- **Tooling**: [Criterion.rs](https://github.com/bheisler/criterion.rs) `0.8`
- **Target OS**: Windows
- **Target Architecture**: x86_64 (SSE4.1)
- **Frame Size**: 640x480 (typical AR video resolution)
- **Toolchain**: `rustc 1.94.0 (stable)`
- **SIMD activation**: SSE4.1 intrinsics are gated by `#[cfg(target_feature = "sse4.1")]` — set `RUSTFLAGS="-C target-feature=+sse4.1"` (or `target-cpu=native`) when running `simd_bench` to exercise the SIMD paths.

## KPM / NFT performance

Issue #142's acceptance criterion calls for the pure-Rust NFT pipeline
to run within 20% of the C++ backend on `pinball-demo`. `marker_bench`
measures `ar_detect_marker` (barcode/template marker detection), which
never touches the FreakMatcher and therefore can't distinguish the
pure-Rust backend from the C++ FFI one — hence the dedicated
`kpm_bench.rs` below, added in #225.

### `kpm_bench.rs` — dedicated KPM wall-clock (#225)

Times a single `KpmHandle::kpm_matching` query — the per-frame NFT
detection path — using the pure-Rust `RustFreakMatcher` backend.

| | |
|---|---|
| **Measures** | one `KpmHandle::kpm_matching` call (detection only) |
| **Reference marker** | `pinball.fset3`, assigned to page 0 |
| **Query image** | `pinball-demo.jpg` — 2000×1500, converted to luma |
| **Fixtures** | `crates/core/examples/Data/` (shared with the `simple_nft` example and the KPM regression tests) |
| **Criterion config** | `sample_size = 10`, 15 s measurement — a full query is heavy |

```sh
cargo bench -p webarkitlib-rs --bench kpm_bench
```

Setup — reference-data load and handle construction — happens once
**outside** the measured loop, so only `kpm_matching` is timed.

Baseline (release build, x86_64):

| Backend | Query time (median) |
|---------|---------------------|
| Rust (`RustFreakMatcher`) | ~0.30 s |

To produce the C++ side of the #142 within-20% comparison, build with
`--features ffi-backend` and swap `RustFreakMatcher` for
`CppFreakMatcher` in the bench. Wall-clock numbers are
hardware-dependent, so treat the committed figure as an
order-of-magnitude reference rather than a hard CI gate.

### Functional parity evidence

Beyond wall-clock timing, the Rust and C++ backends agree on the
meaningful outputs across several test suites:

| Test | What it asserts | Status post-#170 |
|------|------------------|------------------|
| `test_dual_mode_no_divergence_on_pinball` | M9 #152 tier-2: `max_corner_displacement < 2.0 px` between C++ and Rust homographies on `found.jpg/img.jpg` | ✅ zero divergences |
| `absolute_corner_error` (#166 Track A) | Each backend's max corner-error against hand-annotated ground truth stays within baseline + 3.5 px epsilon | ✅ green; **Rust is more accurate than C++** on `pinball-demo` (Rust 5.27 px vs C++ 18.79 px) |
| `cross_stack_parity` (jsartoolkitNFT#584 Track 2) | C++ FFI and Rust pose agree with jsartoolkitNFT-Node within rot 0.08 / trans 10 mm | ✅ green |
| `kpm_regression::test_full_pipeline_pose` | Linux C++ pose matches committed numerical baseline to 1e-2 | ✅ green |

Taken together: Rust meets parity by every quality metric we measure,
and `kpm_bench` now provides the wall-clock baseline needed to catch
performance regressions in the FreakMatcher pipeline. The formal
within-20% comparison against C++ remains a manual, opt-in run
(`--features ffi-backend`) rather than a CI gate, per #142: *"If
slower, open a follow-up performance issue rather than blocking this
PR."*
