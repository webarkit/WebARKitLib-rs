# Milestone 8 — Step 2: Bilinear Interpolation + Gaussian Scale-Space Pyramid

**Status**: Design approved, ready for implementation
**Branch**: `feat/m8-2-interpolate-gaussian-pyramid` (PR target: `feat/m8-freak-detector`)
**Issue**: #127
**Author**: Walter Perdan ([@kalwalt](https://github.com/kalwalt))
**Date**: 2026-05-15

---

## 1. Understanding Summary

- **What**: Port `interpolate.h` + `gaussian_scale_space_pyramid.{h,cpp}` to two new Rust modules:
  - `crates/core/src/kpm/freak/interpolate.rs`
  - `crates/core/src/kpm/freak/gaussian_pyramid.rs`
- **Why**: Step 2 of Milestone 8. The Gaussian scale-space pyramid is the input to the DoG detector (M8-3); bilinear interpolation is used throughout the FREAK pipeline.
- **Who**: Internal infrastructure consumed by M8-3 (DoG), M8-4 (FREAK descriptor), and downstream KPM matching.
- **Success criterion**: `f32` pyramid output is byte-identical to C++ `BinomialPyramid32f::build`, verified by a live FFI dual-mode test (`#[cfg(feature = "dual-mode")]`).
- **Non-goals (this PR)**: DoG detector, FREAK descriptor, SIMD/rayon optimization, harris.* dead code, fixture-based testing.

---

## 2. Decision Log

| # | Decision | Alternatives considered | Rationale |
|---|----------|-------------------------|-----------|
| 1 | Faithful C++ port — `[1,4,6,4,1]/16` binomial filter, `Matrix<f32>` storage, 3 scales/octave hardcoded | True Gaussian with `ceil(3*sigma)*2+1` kernel + u8 storage; hybrid u8 storage with binomial filter | "Port" means preserve behavior. C++ uses a fixed binomial filter (not sigma-parameterized) and f32 storage. Bit-for-bit parity matters because M8-3 (DoG) was tuned against this exact output |
| 2 | Port all helpers from the C++ header in this PR (`bilinear_upsample_point`, `bilinear_downsample_point`, `num_octaves_for`, `effective_sigma`, `kfactor`, `locate`) | Minimum surface only; defer point-mapping helpers to M8-3 | Same C++ header; M8-3 needs them; each is 5–20 lines; tests are trivial |
| 3 | Split into 3 sibling modules: `pyramid.rs` (M8-1, existing) + `gaussian_pyramid.rs` (new) + `interpolate.rs` (new) | Everything in `pyramid.rs`; `pyramid.rs` + one new file | Mirrors C++ file organization; keeps each file focused; interpolation is reusable independent of any pyramid type |
| 4 | Live FFI dual-mode test (existing project pattern) | Pre-generated binary fixture file in `tests/fixtures/` | Matches PRs #76, #77, clustering test; no binary blob in git; output always current vs C++ baseline |
| 5 | Per-level FFI shim: `webarkit_cpp_binomial_pyramid_build_level(...)` | Whole-pyramid shim with array of buffers | Simpler C-ABI; one rebuild per test call is negligible cost for a 32×32 input |
| 6 | Constructor `new(num_octaves: usize)` only — drop `num_levels` and `sigma_0` from original prompt | Original prompt: `new(num_octaves, num_levels, sigma_0)` | C++ `BinomialPyramid32f` hardcodes 3 scales (build sequence is 3-specific). Sigma is *derived*: `sigma(oct, scale) = k^scale * 2^oct` with `k = 2^(1/(num_scales-1))` |
| 7 | `build(&mut self, image: &Matrix<u8>) -> Result<(), GaussianPyramidError>`; idempotent (clear & rebuild) | C++ `void` signature; non-idempotent | Mirrors M8-1; project "log + return Err" idiom (CLAUDE.md §1) |
| 8 | `NUM_SCALES_PER_OCTAVE = 3` exposed as public `const` | Magic number; constructor parameter | Avoids magic number in callers; documents the C++ contract |
| 9 | Bilinear API: 3 concrete fns (u8/f32/raw-slice), `f32` return for u8 input, OOB → `0.0` | Generic-over-T with trait; literal C++ u8-returns-u8 | Matches prompt; preserves precision; safer than C++ `ASSERT`s |

---

## 3. Final API

### 3.1 `interpolate.rs`

```rust
pub fn bilinear_interpolate_u8(image: &Matrix<u8>, x: f32, y: f32) -> f32;
pub fn bilinear_interpolate_f32(image: &Matrix<f32>, x: f32, y: f32) -> f32;
pub fn bilinear_interpolate(data: &[u8], width: usize, height: usize, x: f32, y: f32) -> f32;
pub fn bilinear_upsample_point(x: f32, y: f32, octave: i32) -> (f32, f32);
pub fn bilinear_downsample_point(x: f32, y: f32, octave: i32) -> (f32, f32);
```

Formula: `result = (1-dy) * ((1-dx) * p00 + dx * p01) + dy * ((1-dx) * p10 + dx * p11)`
where `dx = x - floor(x)`, `dy = y - floor(y)`.

### 3.2 `gaussian_pyramid.rs`

```rust
pub struct GaussianScaleSpacePyramid {
    pub octaves: Vec<Vec<Matrix<f32>>>,
    pub num_octaves: usize,
    pub kfactor: f32,
    pub one_over_log_k: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub enum GaussianPyramidError {
    EmptyImage,
    ImageTooSmall { rows: usize, cols: usize },
    ZeroOctaves,
    OctaveTooSmall { octave: usize, rows: usize, cols: usize },
}

impl GaussianScaleSpacePyramid {
    pub const NUM_SCALES_PER_OCTAVE: usize = 3;

    pub fn new(num_octaves: usize) -> Self;
    pub fn build(&mut self, image: &Matrix<u8>) -> Result<(), GaussianPyramidError>;
    pub fn level(&self, octave: usize, scale: usize) -> &Matrix<f32>;
    pub fn num_scales_per_octave(&self) -> usize;
    pub fn effective_sigma(&self, octave: usize, scale: usize) -> f32;
    pub fn locate(&self, sigma: f32) -> (usize, usize);
}

pub fn num_octaves_for(width: usize, height: usize, min_size: usize) -> usize;
```

### 3.3 Build sequence (per C++ `BinomialPyramid32f::build`)

- **Octave 0**:
  - `octaves[0][0] = filter(input)` — u8 → f32
  - `octaves[0][1] = filter(octaves[0][0])` — f32 → f32
  - `octaves[0][2] = filter(filter(octaves[0][1]))` — 2 applications
- **Octave i (i > 0)**:
  - `octaves[i][0] = downsample(octaves[i-1][2])` — half-size, bilinear 2×2 average
  - `octaves[i][1] = filter(octaves[i][0])`
  - `octaves[i][2] = filter(filter(octaves[i][1]))`

Filter coefficients (separable):
- **H pass**: `6c + 4(l+r) + (ll+rr)` (u16 accumulator for u8 src, f32 for f32 src)
- **V pass**: same shape × `1/256`
- **Border**: replicate edge pixels for 2-pixel border

### 3.4 FFI shim (`kpm_c_api.h/.cpp`)

```c
int webarkit_cpp_binomial_pyramid_build_level(
    const uint8_t* src, int src_w, int src_h,
    int num_octaves,
    int target_octave, int target_scale,
    float* dst_out, int dst_capacity_floats);
```

Returns 0 on success; non-zero codes for validation failure, octave too small, buffer overflow, C++ exception.

---

## 4. Assumptions

1. `purecv 0.4.0` supports `Matrix<f32>` with the same surface as `Matrix<u8>`. Will verify before coding.
2. The H pass output max is `16 × 255 = 4080` (fits `u16`); V pass output `* 1/256` is exactly representable in `f32`, so bit-for-bit parity with C++ is achievable.
3. C++ `Image` may have row stride > width when `AUTO_STEP` aligns; FFI shim copies row-by-row using `lvl.step() / sizeof(float)`.
4. `apply_filter_twice` shared-buffer optimization in C++ does not affect output (filter is deterministic); Rust port allocates a fresh intermediate buffer per pass — same final bytes.
5. Test data: synthetic gradient (`pixel = (r * cols + c) & 0xFF`); no fixture files in git.
6. License header: `.claude/HEADER.txt`, year 2026.
7. No `CHANGELOG.md` edits (release-only).

---

## 5. Test Plan

### 5.1 `interpolate.rs` (4 tests)

- `test_bilinear_interpolate_at_integer_coords`
- `test_bilinear_interpolate_out_of_bounds_returns_zero`
- `test_bilinear_interpolate_midpoint` — verifies `(0.5, 0.5)` produces average of 4 corners
- `test_bilinear_upsample_downsample_roundtrip`

### 5.2 `gaussian_pyramid.rs` (9 unit tests + 1 dual-mode)

- `test_kfactor_for_3_scales` — `kfactor ≈ √2`
- `test_gaussian_pyramid_octave_count`
- `test_gaussian_pyramid_octave_downsamples` — `level(oct, 0).cols == src_cols >> oct`
- `test_gaussian_pyramid_sigma_increases` — within and across octaves
- `test_locate_clamps_to_pyramid_bounds`
- `test_gaussian_pyramid_build_is_idempotent`
- `test_gaussian_pyramid_empty_image_returns_error`
- `test_gaussian_pyramid_image_too_small_returns_error`
- `test_gaussian_pyramid_zero_octaves_returns_error`
- `test_num_octaves_for`
- `test_gaussian_pyramid_pixels_match_cpp` (`#[cfg(feature = "dual-mode")]`) — byte-for-byte `f32::to_bits()` parity at all (octave, scale) levels for a 32×32 gradient input

**Verification**: `cargo test -p webarkitlib-rs -- kpm::freak` + `cargo test -p webarkitlib-rs --lib --features dual-mode`.

---

## 6. Follow-up Work (Out of Scope This PR)

- **M8-3**: DoG scale-invariant detector (will consume this pyramid)
- **M8-4**: FREAK descriptor extraction
- **Perf**: criterion benchmark, then SIMD path for the binomial filter (file a follow-up issue analogous to #131/#132 for `Pyramid` once this lands)

---

## 7. Open Risks

1. **purecv `Matrix<f32>` API parity with `Matrix<u8>`** — will verify before coding; minor adaptations possible.
2. **f32 bit-for-bit parity** — relies on deterministic integer arithmetic + `* (1.0 / 256.0)`. If LLVM reassociates the binomial sum differently than the C++ compiler, parity could break. Mitigation: use the *exact* additive order from the C++ source (no commutative rewriting).

   **Update (post-merge of M8-1, pre-merge of M8-2)**: This risk materialized on macOS. Apple clang on ARM64 emits FMA (fused multiply-add) instructions for the binomial filter expression by default, while MSVC (Windows) and GCC (Linux) do not. The result is a 1-ULP difference in C++ output between macOS and the other platforms. The dual-mode test was relaxed to ≤1 ULP tolerance — strict bit equality is impractical given cross-platform C++ FP contraction. The 1-ULP tolerance still catches any algorithm error while accommodating the platform variance. Local 1-ULP bugs (like the H-pass parenthesization caught during implementation) are still detected because they produce >1 ULP errors at higher scales.
3. **FFI shim must compile cleanly on all CI targets** — including macOS (post-#117 libc++ fix) and the flaky macOS rust-cache (#134). If #134 isn't fixed, expect occasional CI re-runs.
