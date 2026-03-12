# WebARKitLib.rs Core Benchmarks

**Last Updated**: 2026-03-11 15:37:20 (UTC+1)

This document tracks the performance of critical image processing and pattern matching functions in the `webarkitlib_rs` core crate.

## SIMD Performance (x86_64 SSE4.1)

The following benchmarks were conducted on an x86_64 system with SSE4.1 support enabled via the `simd` feature.

| Function | Implementation | Average Time | Speedup |
| :--- | :--- | :--- | :--- |
| `rgba_to_gray` | Scalar | 550.68 µs | - |
| | SIMD (SSE4.1) | 232.72 µs | **2.37x** |
| `dot_product` | Scalar | 347.85 ns | - |
| | SIMD (SSE4.1) | 54.56 ns | **6.44x** |
| `box_filter_h` | Scalar | 1.731 ms | - |
| | SIMD (SSE4.1) | 1.734 ms | 1.00x |
| `box_filter_v` | Scalar | 2.590 ms | - |
| | SIMD (SSE4.1) | 886.84 µs | **2.92x** |

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

- **Tooling**: [Criterion.rs](https://github.com/bheisler/criterion.rs)
- **Target OS**: Windows
- **Target Architecture**: x86_64 (SSE4.1)
- **Frame Size**: 640x480 (typical AR video resolution)
