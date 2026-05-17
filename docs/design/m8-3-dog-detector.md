# Milestone 8 — Step 3: DoG Detector + Orientation Assignment

**Status**: Design approved, ready for implementation
**Branch**: `feat/m8-3-dog-detector` (PR target: `feat/m8-freak-detector`)
**Issue**: #128
**Author**: Walter Perdan ([@kalwalt](https://github.com/kalwalt))
**Date**: 2026-05-16

---

## 1. Understanding Summary

- **What**: Port `orientation_assignment.{h,cpp}` and `DoG_scale_invariant_detector.{h,cpp}` to two new Rust sibling modules:
  - `crates/core/src/kpm/freak/orientation.rs`
  - `crates/core/src/kpm/freak/detector.rs`
- **Why**: Step 3 of Milestone 8. The Difference-of-Gaussians (DoG) detector finds scale-invariant keypoints from the Gaussian pyramid (M8-2 output); these keypoints feed the FREAK descriptor (M8-4) and the downstream KPM matching pipeline.
- **Who**: Internal infrastructure consumed by M8-4 (FREAK descriptor) and downstream KPM matching.
- **Success criterion**: Live FFI dual-mode test: Rust detector keypoint count agrees with C++ within `±5` on `found.jpg`, with identical detector configuration on both sides. Plus 6 unit tests for shape/coverage/correctness.
- **Non-goals (this PR)**:
  - FREAK descriptor extraction (M8-4)
  - SIMD/rayon optimization
  - Harris detector port (dead code — explicitly excluded by issue)
  - Public two-phase `OrientationAssignment` API (`compute_gradients` separated from `compute`) — deferred to M8-4
  - Full 7-parameter `OrientationAssignment::new(...)` constructor — using hardcoded FREAK defaults

---

## 2. Decision Log

| # | Decision | Alternatives considered | Rationale |
|---|----------|-------------------------|-----------|
| 1 | **Option C scope**: faithful on algorithm-critical bits (3-way Hessian dispatch, bucket pruning, `Matrix<f32>` DoG, rich `DoGFeaturePoint`, all detector params), simplified on `OrientationAssignment` public surface | Option A (fully faithful, ~1500 lines); Option B (spec-as-written, naive top-N pruning, single Hessian variant) | Keeps PR reviewable while preserving algorithm correctness; deferred surface promotion lands naturally in M8-4 where it's actually needed |
| 2 | **Option B FeaturePoint**: new `DoGFeaturePoint` (9 fields) in `detector.rs`; `hough.rs::FeaturePoint` unchanged; `From<&DoGFeaturePoint>` conversion | Option A: extend existing `hough.rs::FeaturePoint` with 4 new fields; Option C: move to shared `feature_point.rs` module | Mirrors C++ architecture (`vision::FeaturePoint` in `matchers/` is persistent type; `DoGScaleInvariantDetector::FeaturePoint` is working type with diagnostics); avoids coupling this PR to `hough.rs` |
| 3 | **B: Split into 2 modules** — `orientation.rs` + `detector.rs` | Option A: single `detector.rs` (~1300 lines mixing two concerns); Option C: 3-way split | Mirrors C++ file organization; matches M8-2 splitting pattern; each file ~600 lines and forward-compatible with M8-4 API promotion |
| 4 | **A: Live FFI shim** `webarkit_cpp_dog_detect_count(...)` parameterized by detector config | Option B: pre-generated `tests/fixtures/dog_keypoint_count.txt`; Option C: hybrid xtask-regenerated | Matches M8-2 pattern; no binary/text blob in git; both sides run identical configuration |
| 5 | **C: `MAX_TIE_DIVERGENCE = 5`** for keypoint count tolerance | Option A: exact match (`assert_eq!`); Option B: ±10% (issue spec literal) | Issue's ±10% would allow ±50 keypoints — too loose to detect real bugs. ±5 catches sort tie-breaking variance only. Tighten to exact in a follow-up if CI shows consistent equality |
| 6 | **`detect()` is infallible**: returns `Vec<DoGFeaturePoint>` directly | `Result<Vec<DoGFeaturePoint>, DetectorError>` (M8-1/M8-2 pattern) | Pre-validated pyramid input; no failure modes that aren't already caught upstream; matches C++ `void detect()`; empty Vec = "no keypoints found", not error |
| 7 | **`DoGScaleInvariantDetector::new()` is infallible**: validates config with `debug_assert!` | `Result<Self, DetectorError>` | Constructor just stores numeric params; bad values trip debug asserts |

---

## 3. Final API

### 3.1 `orientation.rs`

```rust
pub struct OrientationAssignment {
    num_bins: usize,                       // FREAK default: 36
    gaussian_expansion_factor: f32,        // FREAK default: 1.5
    support_region_expansion_factor: f32,  // FREAK default: 3.0
    num_smoothing_iterations: usize,       // FREAK default: 5
    peak_threshold: f32,                   // FREAK default: 0.8
}

impl OrientationAssignment {
    #[must_use]
    pub fn new() -> Self;

    #[must_use]
    pub fn compute(
        &self,
        gradient: &Matrix<f32>,   // channels = 2: (magnitude, angle) interleaved
        x: f32,
        y: f32,
        sigma: f32,
    ) -> Vec<f32>;
}

/// Build a 2-channel (magnitude, angle) gradient image from a pyramid level.
#[must_use]
pub fn compute_gradient_image(level: &Matrix<f32>) -> Matrix<f32>;
```

### 3.2 `detector.rs`

```rust
#[derive(Debug, Clone, Copy)]
pub struct DoGFeaturePoint {
    pub x: f32,
    pub y: f32,
    pub angle: f32,        // 0 if find_orientation == false
    pub octave: i32,
    pub scale: i32,        // integer scale index (0..NUM_SCALES_PER_OCTAVE)
    pub sp_scale: f32,     // sub-pixel scale offset, in (-0.5, 0.5)
    pub score: f32,        // signed DoG response at refined location
    pub sigma: f32,        // characteristic Gaussian sigma after refinement
    pub edge_score: f32,
}

impl From<&DoGFeaturePoint> for super::hough::FeaturePoint {
    fn from(d: &DoGFeaturePoint) -> Self;  // projects to persistent type
}

pub struct DoGScaleInvariantDetector {
    laplacian_threshold: f32,
    edge_threshold: f32,
    max_subpixel_distance_sqr: f32,    // FREAK default: 3.0
    num_buckets_x: usize,              // FREAK default: 10
    num_buckets_y: usize,              // FREAK default: 10
    max_num_feature_points: usize,
    find_orientation: bool,
    orientation_assignment: OrientationAssignment,
}

impl DoGScaleInvariantDetector {
    pub const DEFAULT_MAX_NUM_FEATURE_POINTS: usize = 5000;

    pub fn new(
        laplacian_threshold: f32,
        edge_threshold: f32,
        max_num_feature_points: usize,
        find_orientation: bool,
    ) -> Self;

    pub fn detect(
        &self,
        pyramid: &GaussianScaleSpacePyramid,
    ) -> Vec<DoGFeaturePoint>;
}
```

### 3.3 detect() pipeline

1. **Build DoG pyramid**: `dog[oct][s] = pyramid.level(oct, s+1) - pyramid.level(oct, s)` for `s` in `0..NUM_DOG_PER_OCTAVE` (= 2 with 3 scales/octave). `Matrix<f32>` storage.
2. **Find 3D extrema**: for each `(oct, dog_idx, r, c)`, compare to all 26 neighbours in the 3×3×3 cube. Cross-octave neighbours accessed via bilinear interpolation.
3. **Contrast threshold**: discard `|value| < laplacian_threshold`.
4. **Sub-pixel refinement** (`compute_subpixel_hessian` dispatcher):
   - `SameOctave` — all three laplacians same size
   - `FineOctavePair` — lap0/lap1 same size, lap2 half size
   - `CoarseOctavePair` — lap0 double size, lap1/lap2 same
   Solve 3×3 `H · δ = b` via Gaussian elimination. Discard if `||δ||² > max_subpixel_distance_sqr` or H is degenerate.
5. **Edge rejection**: `compute_edge_score` from H's top-left 2×2; discard if `|score| ≥ (edge_threshold + 1)² / edge_threshold`.
6. **Orientation assignment** (when `find_orientation == true`): precompute one gradient image per pyramid level; for each refined point query `OrientationAssignment::compute(...)`; emit one copy per dominant peak (zero peaks ⇒ keypoint dropped, matches C++).
7. **Bucket pruning**: distribute keypoints across `num_buckets_x × num_buckets_y` fine-image spatial buckets; take best-by-|score| per bucket round-robin until `max_num_feature_points` reached.

### 3.4 Replacing the C++ `NONMAX_CHECK` preprocessor macro

The C++ `extractFeatures` uses an `#define NONMAX_CHECK(OPERATOR, VALUE)` macro that expands to a chain of 26 short-circuit `&&` comparisons (9 from `im0`, 8 from `im1` excluding the center, 9 from `im2`). It is parameterized by an operator token (`>` or `<`) so a single macro covers both maxima and minima detection, and a single chain runs at full speed because each comparison is inlined by the preprocessor.

Rust doesn't let you pass operator tokens as macro parameters as cleanly, so the macro is replaced by **an `enum NonMaxOp { Greater, Less }` plus three private inlined helper functions**, one per dimension pattern:

| Function | C++ macro instance |
|---|---|
| `nonmax_same_octave(op, val, im0, im1, im2, w, row, col)` | SameOctave (all three `im*` same dims) |
| `nonmax_fine_octave(op, val, im0, im1, im2, w, row, col, ds_x, ds_y)` | FineOctavePair (`im2` half-sized → 9 `bilinear_interpolate_f32` lookups) |
| `nonmax_coarse_octave(op, val, im0, im1, im2, w, row, col, us_x, us_y)` | CoarseOctavePair (`im0` double-sized → 9 `bilinear_interpolate_f32` lookups) |

Each helper builds a `cmp` closure with a `match op { Greater => a > b, Less => a < b }` and chains the 26 `cmp(val, neighbor)` calls with `&&`. The call site mirrors the C++ `if/else if`:

```rust
let extrema = nonmax_same_octave(NonMaxOp::Greater, value, d0, d1, d2, w, row, col)
    || nonmax_same_octave(NonMaxOp::Less, value, d0, d1, d2, w, row, col);
```

**Trade-offs considered:**

- **Trait-generic `fn nonmax<C: Fn(f32, f32) -> bool>(cmp: C, ...)`** — would force monomorphization twice per call site and bloat the call graph; rejected.
- **`macro_rules!` macro** — would match the C++ structure most literally, but Rust hygiene requires explicit captures of every variable, making the macro definition harder to read than the function form; rejected.
- **Inline expansion** of all 156 comparisons (26 × 2 operators × 3 patterns) — rejected as repetitive.

The enum-with-closure form is shorter than the alternatives and the `#[inline]` annotation gives the compiler the same opportunity to specialize as the C++ macro got at preprocess time. In optimized builds the `match op` is constant-folded per call site because the caller passes a literal `NonMaxOp::Greater` or `NonMaxOp::Less`.

### 3.5 FFI shim (`kpm_c_api.h/.cpp`)

```c
int webarkit_cpp_dog_detect_count(
    const unsigned char* src, int src_w, int src_h,
    int num_octaves,
    float laplacian_threshold, float edge_threshold,
    int max_num_feature_points, int find_orientation,
    int* count_out);
```

Returns 0 on success; non-zero codes for validation failure / C++ exception. ~90 lines of C++.

---

## 4. Assumptions

1. `Matrix<f32>` API surface (verified in M8-2) supports all needed operations: `as_slice`, `as_mut_slice`, `from_vec`, `zeros`, `rows`, `cols`, `channels` fields.
2. `bilinear_interpolate_f32` from `interpolate.rs` (M8-2) is the right API for sub-pixel gradient access on `Matrix<f32>` gradient images.
3. `bilinear_upsample_point` / `bilinear_downsample_point` from `interpolate.rs` (M8-2) handle the cross-octave Hessian variants and the `From<&DoGFeaturePoint>` fine-image projection.
4. `found.jpg` lives at `benchmarks/data/found.jpg` (verified); loaded as grayscale `Matrix<u8>` in tests via the `image` crate.
5. License header on both new files; year 2026; author Walter Perdan @kalwalt.
6. Sort stability: Rust's `sort_by` is stable, C++ `std::sort` is not guaranteed stable. The `MAX_TIE_DIVERGENCE = 5` tolerance covers any tie-related variance.
7. The `-ffp-contract=off` flag added in M8-2 keeps C++ floating-point output deterministic across platforms; all numeric helpers in this PR inherit that determinism.

### 4.1 C++-verified defaults (sourced from `DoG_scale_invariant_detector.cpp` lines 108–134)

| Parameter | Value | Source |
|---|---|---|
| `laplacian_threshold` | `0` (no contrast filter) | `DoGScaleInvariantDetector::DoGScaleInvariantDetector` |
| `edge_threshold` | `10` → `hessian_threshold = (10+1)²/10 = 12.1` | same |
| `max_subpixel_distance_sqr` | `9` (`3*3` — already squared; name has `Sqr` suffix) | same |
| `num_buckets_x = num_buckets_y` | `10` | same |
| `find_orientation` | `true` | same |
| `max_num_feature_points` (`kMaxNumFeaturePoints`) | `5000` | same |
| `kMaxNumOrientations` | `36` (used both as histogram bin count and per-keypoint orientation cap) | header constant |
| OA `num_bins` | `36` (= `kMaxNumOrientations`) | `alloc(...)` call site |
| OA `gaussian_expansion_factor` | `3.0` | `alloc(...)` call site |
| OA `support_region_expansion_factor` | `1.5` | `alloc(...)` call site |
| OA `num_smoothing_iterations` | `5` | `alloc(...)` call site |
| OA `peak_threshold` | `0.8` | `alloc(...)` call site |

### 4.2 Corrections to §3 found during implementation

- **DoG direction**: `dog[s] = pyramid[s] - pyramid[s+1]` (less-blurred minus more-blurred), per C++ `difference_image_binomial` (lines 85–106). The opposite direction was assumed in the original §3.3 draft.
- **Fine-image coordinates from extraction**: `(x, y)` are upsampled to fine-image coordinates **at extraction time** via `bilinear_upsample_point`, matching C++. The original draft stored octave-local coords until the `From` projection. Now stored as fine-image coords throughout `DoGFeaturePoint`.
- **Pipeline order**: prune → orient (matches C++ `detect()`), **not** orient → prune. Pruning before orientation avoids computing orientations for keypoints that get dropped.
- **`sp_scale` semantics**: full sub-pixel scale value `scale + u[2]`, clipped to `[0, NUM_DOG_PER_OCTAVE]`. Not an offset in `(-0.5, 0.5)` as originally drafted.
- **`OrientationAssignment` gradient channel order**: `(angle, magnitude)` — matches C++ `ComputePolarGradients` which writes `atan2(dy, dx) + π` to channel 0 and `sqrt(dx² + dy²)` to channel 1. The angle channel is shifted into `[0, 2π]` (atan2 result `+ π`). The original draft had `(magnitude, angle)`.
- **OA factor names swapped in original draft**: `gaussian_expansion_factor = 3.0` and `support_region_expansion_factor = 1.5` (not the reverse). Total support radius = `3.0 · 1.5 · sigma = 4.5 · sigma`.

---

## 5. Test Plan

| Test | Source | Notes |
|---|---|---|
| `test_orientation_assignment_horizontal_gradient` | Issue spec | Synthetic level (`pixel = col as f32`); call `compute_gradient_image` → `OrientationAssignment::compute`; dominant orientation within tolerance of π/2 |
| `test_orientation_assignment_empty_when_no_gradients` | Hardening | Flat image returns empty Vec |
| `test_dog_detector_finds_keypoints_on_real_image` | Issue spec | Load `found.jpg`; 3-octave pyramid; assert > 50 keypoints |
| `test_dog_detector_keypoints_within_image_bounds` | Issue spec | All `(x, y)` ∈ `[0, w) × [0, h)` after `From<&DoGFeaturePoint>` projection |
| `test_dog_feature_point_from_conversion` | Hardening | Hand-crafted `DoGFeaturePoint` projects correctly; `maxima = score >= 0` |
| `test_dog_detector_zero_octave_pyramid_is_handled` | Hardening | Degenerate pyramid; no panic, possibly empty Vec |
| `test_dog_keypoints_match_cpp_count` | Issue spec (modified) | `#[cfg(feature = "dual-mode")]`; live FFI; `\|rust - cpp\| <= 5` |

**Verification commands:**

```
cargo test -p webarkitlib-rs -- kpm::freak::orientation kpm::freak::detector
cargo test -p webarkitlib-rs --lib --features dual-mode -- kpm::freak::detector
cargo clippy -p webarkitlib-rs -- -D warnings
```

---

## 6. Follow-up Work (Out of Scope This PR)

The Option C deferred items, naturally absorbed into **M8-4 (FREAK descriptor)** since the descriptor will share gradient images with `OrientationAssignment`:

- **Public two-phase `OrientationAssignment` API** — `compute_gradients(&mut self, pyramid)` and `compute(...)` as separate public methods, with caller-owned gradient image cache reused across detect/describe calls.
- **Full 7-parameter `OrientationAssignment::new(...)`** — replace hardcoded FREAK defaults with constructor parameters.
- **Reusable gradient image cache** — currently rebuilt per `detect()` call; M8-4 will expose the cache so describe() can reuse it.

Also deferred (analogous to #131/#132 for the box-filter pyramid):

- **`criterion` benchmark** for `detect()` end-to-end and `compute_gradient_image` hot path.
- **SIMD path** for `compute_gradient_image` (largest hot loop in M8-3) and `dog_image`.

---

## 7. Open Risks

1. **`OrientationAssignment` default values** — hardcoded FREAK defaults in §3.1 are best-effort cross-references; if any differ from the actual C++ FreakMatcher facade call site, the dual-mode test count will diverge. Mitigation: verify defaults from C++ source before coding; document the source location.

2. **Cross-octave Hessian dispatch indexing** — the 3 variants depend on consistent (oct, dog_idx) → (lap0, lap1, lap2) mapping. Off-by-one in the boundary cases would produce wrong refinement and lose keypoints near octave transitions. Mitigation: extensive unit tests with hand-crafted DoG pyramids that span octave boundaries.

3. **Bucket pruning sort stability** — C++ `std::sort` is unstable; Rust `sort_by` is stable. Where multiple keypoints have identical |score| (rare but possible), the pruning order will differ. The `MAX_TIE_DIVERGENCE = 5` tolerance covers this. If CI shows divergence > 5 we know there's a real bug to find.

4. **f32 cross-platform parity (legacy)** — the M8-2 `-ffp-contract=off` build flag should keep this PR deterministic across platforms. If macOS CI still produces a different keypoint count, investigate whether new code paths introduce FP contraction opportunities not covered by the existing flag.
