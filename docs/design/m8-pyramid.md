# Milestone 8 — Step 1: Image Pyramid (Box Filter)

**Status**: Design approved, ready for implementation
**Branch**: `feat/m8-freak-descriptor-work` (PR target: `feat/m8-freak-detector`)
**Issues**: #125, #126
**Author**: Walter Perdan ([@kalwalt](https://github.com/kalwalt))
**Date**: 2026-05-14

---

## 1. Understanding Summary

- **What**: Port the C++ `BoxFilterPyramid8u` (from `WebARKitLib/lib/SRC/KPM/FreakMatcher/detectors/pyramid.{h,cpp}`) to Rust as `crates/core/src/kpm/freak/pyramid.rs`.
- **Why**: First step of Milestone 8 (FREAK descriptor & DoG detector). The pyramid is the foundation the DoG detector will sit on.
- **Who**: Internal infrastructure — used by the upcoming DoG detector and downstream KPM matching pipeline.
- **Success criterion**: Output `Matrix<u8>` byte-identical to the C++ `BoxFilterDecimate` baseline.
- **Non-goals (this PR)**: DoG detector, SIMD intrinsics, rayon parallelization, 128-byte chunked memory layout, Harris detector port (dead code).

---

## 2. Decision Log

| # | Decision | Alternatives considered | Rationale |
|---|----------|-------------------------|-----------|
| 1 | Port the `BoxFilterPyramid8u` C++ algorithm bit-for-bit (2×2 average with `+2 >> 2` rounding) | Simplified standard box filter; defer-and-optimize | User chose Option A — exact parity required for validating against C++ baseline |
| 2 | `build()` returns `Result<(), PyramidError>` instead of `void` | Infallible signature; hybrid (silent failure + log) | Project idiom (CLAUDE.md §1): errors not panics; log + return Err pattern from PRs #76/#77 |
| 3 | Scalar-only this PR; SIMD/rayon in follow-up PR with benchmarks | Scalar + SIMD + rayon all-in-one PR; scalar + rayon only | Matches PRs #76/#77 pattern; small reviewable PR; "measure first" rule |
| 4 | Keep `scale_factor: f32` in API, but only `2.0` accepted (else error) | Drop the parameter entirely; accept arbitrary scales | Future-proof API surface while honoring C++ limitation |
| 5 | Output dims: `ceil((src-1)/2)` per C++ `init()` formula | `(src/scale).round()` from user's draft spec | Bit-for-bit parity requires the C++ dimension formula |
| 6 | Drop the 128-byte chunking; plain row-major loop | Port the C++ chunking verbatim | Chunking is cache-only; output unchanged; defer with benchmarks |
| 7 | `build()` is idempotent — clears `self.levels` then populates | Append; error on re-build | Rust idiom; cheap; intuitive |
| 8 | Hard error `PyramidError::LevelTooSmall { level }` if a level would be 0-sized | Stop early + warn; require pre-flight check | Project's log + return Err pattern; lets caller decide retry strategy |

---

## 3. Assumptions

1. `purecv 0.4.0` `Matrix<u8>` has `rows`, `cols`, `channels` fields, `as_slice()`, `get(r,c,ch)`, `zeros(h,w,ch)`, and `from_vec(...)`. To be verified before coding.
2. `scale_factor != 2.0` returns `InvalidScaleFactor(f32)` — leaves the door open for future binomial / arbitrary-scale pyramids.
3. Test data: synthetic gradient image (`pixel = (r * cols + c) as u8`) is sufficient for unit tests; no fixture files needed.
4. License header: every new `.rs` file uses `.claude/HEADER.txt` template, year 2026.
5. Logging: `use crate::{arlog_e, arlog_w};`; use `arlog_e!` for each error path.
6. No `CHANGELOG.md` edits in this PR (release-only rule).
7. Branch: `feat/m8-freak-descriptor-work` → PR target `feat/m8-freak-detector`.

---

## 4. Final Design

### 4.1 File layout

- **New file**: `crates/core/src/kpm/freak/pyramid.rs`
- **Edit**: `crates/core/src/kpm/freak/mod.rs` — add `pub mod pyramid;` and `pub use pyramid::{Pyramid, PyramidError};`

### 4.2 Module doc

```rust
//! Image pyramid built by repeated box-filter downsampling.
//!
//! Ported from `WebARKitLib/lib/SRC/KPM/FreakMatcher/detectors/pyramid.{h,cpp}`
//! (`BoxFilterPyramid8u` / `BoxFilterDecimate`).
//!
//! C equivalent: `vision::BoxFilterPyramid8u`
```

### 4.3 Public API

```rust
pub struct Pyramid {
    pub levels: Vec<Matrix<u8>>,
    pub scale_factor: f32,
    num_levels: usize,  // private; remembers constructor param
}

#[derive(Debug, Clone, PartialEq)]
pub enum PyramidError {
    EmptyImage,
    InvalidScaleFactor(f32),
    ZeroLevels,
    LevelTooSmall { level: usize },
}

impl Pyramid {
    #[must_use]
    pub fn new(num_levels: usize, scale_factor: f32) -> Self;

    pub fn build(&mut self, image: &Matrix<u8>) -> Result<(), PyramidError>;

    #[must_use]
    pub fn num_levels(&self) -> usize;

    #[must_use]
    pub fn level(&self, i: usize) -> &Matrix<u8>;
}
```

### 4.4 Algorithm

Per output pixel:
```
dst[i', j'] = (src[2i', 2j']   + src[2i', 2j'+1]
             + src[2i'+1, 2j'] + src[2i'+1, 2j'+1]
             + 2) >> 2
```

- `u16` accumulator (matches C++ `unsigned short`; max sum = `4·255 + 2 = 1022`).
- Output dims: `new_h = ceil((src.rows-1)/2)`, `new_w = ceil((src.cols-1)/2)`.
- No `unsafe`, no SIMD intrinsics, no chunking.

### 4.5 Error handling

Every error site uses the project's "log + return Err" pattern:
```rust
if image.rows == 0 || image.cols == 0 {
    arlog_e!("Pyramid::build: input image is empty");
    return Err(PyramidError::EmptyImage);
}
```

### 4.6 Tests

| Test name | Purpose |
|-----------|---------|
| `test_pyramid_level_count` | (User spec) `new(4, 2.0)` → 4 levels |
| `test_pyramid_level_dimensions_shrink` | (User spec) `ceil((src-1)/2)` formula |
| `test_pyramid_pixel_values_in_range` | (User spec) no overflow, fully populated |
| `test_pyramid_box_filter_known_values` | Hand-computed 4×4 → 2×2 to verify exact algorithm |
| `test_pyramid_build_is_idempotent` | Re-build clears prior state |
| `test_pyramid_empty_image_returns_error` | `EmptyImage` variant |
| `test_pyramid_invalid_scale_returns_error` | `InvalidScaleFactor(1.5)` |
| `test_pyramid_zero_levels_returns_error` | `ZeroLevels` |
| `test_pyramid_level_too_small_returns_error` | 5 levels on 4×4 input → `LevelTooSmall` |

**Verification**: `cargo test -p webarkitlib-rs -- kpm::freak::pyramid`

---

## 5. Follow-up Work (Out of Scope This PR)

- **Step 2** of M8: DoG detector (uses this pyramid)
- **Step 3** of M8: FREAK descriptor extraction
- **Step 4** of M8: Pipeline integration
- **Perf PR #1**: `criterion` benchmark for `downsample`
- **Perf PR #2**: SIMD path (AVX2/SSE4.1/wasm32) gated behind feature flags, with benchmark proof of speedup
- **Perf PR #3** (maybe): Chunked memory layout if benchmarks show cache misses dominate

---

## 6. Open Risks

1. **`purecv 0.4.0` API verification** — design assumes specific method names (`from_vec`, `as_slice`, field access). Will verify before coding; minor adaptations possible.
2. **Bit-for-bit verification** — we have no automated comparison against the C++ baseline in this PR. `test_pyramid_box_filter_known_values` uses hand-computed values as a proxy. Full A/B testing against the C++ baseline is a separate validation effort.
