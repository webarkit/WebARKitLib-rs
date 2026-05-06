## Summary
SIMD benchmarks fail to compile due to unconditional imports of conditionally-compiled functions and missing `required-features` declaration.

## Environment
- **Product/Service**: WebARKitLib-rs
- **Crate**: `crates/core`
- **Component**: Benchmarks (`simd_bench`)

## Reproduction Steps
1. Run `cargo bench --features simd`
2. Observe compilation error in `crates/core/benches/simd_bench.rs`

## Expected Behavior
Benchmarks compile and run successfully when `--features simd` is enabled, comparing scalar and SIMD implementations.

## Actual Behavior
Compilation fails with `error[E0432]: unresolved imports` for SIMD functions:
- `box_filter_h_simd_x86`
- `box_filter_v_simd_x86`
- `rgba_to_gray_simd_x86`
- `dot_product_simd_x86`

## Error Details
```
error[E0432]: unresolved imports `webarkitlib_rs::image_proc::box_filter_h_simd_x86`, ...
   --> crates\core\benches\simd_bench.rs:39:26

note: found an item that was configured out
   --> crates\core\src\ar\image_proc.rs:576:15
```

The functions are gated behind `#[cfg(all(feature = "simd-image", target_arch = "x86_64", target_feature = "sse4.1"))]`.

## Root Cause
1. **Missing `required-features`**: The `simd_bench` benchmark in `Cargo.toml` didn't declare `required-features = ["simd-x86-sse41"]`, so cargo wouldn't ensure the feature is enabled when building the benchmark.
2. **Unconditional imports**: The benchmark's top-level imports tried to import SIMD functions unconditionally, but they're only compiled when the `target_feature = "sse4.1"` cfg is true.

## Impact
**Medium** - Benchmarks cannot be run with SIMD features enabled, blocking performance testing of optimized implementations.

## Solution
1. Added `required-features = ["simd-x86-sse41"]` to the `simd_bench` configuration in `Cargo.toml`
2. Made SIMD function imports conditional using `#[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]` guards in `simd_bench.rs`

## Files Modified
- `crates/core/Cargo.toml` — Added required-features to simd_bench
- `crates/core/benches/simd_bench.rs` — Made SIMD imports conditional

## Additional Context
The benchmark code already had conditional compilation guards around SIMD function calls (lines 54, 73, 105), but the imports at the top were unconditional, causing the mismatch.