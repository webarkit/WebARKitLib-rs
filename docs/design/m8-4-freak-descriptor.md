# Milestone 8 — Step 4: FREAK Descriptor + Keyframe

**Status**: Design approved, ready for implementation
**Branch**: `feat/m8-4-freak-descriptor` (PR target: `feat/m8-freak-detector`)
**Issue**: #129
**Author**: Walter Perdan ([@kalwalt](https://github.com/kalwalt))
**Date**: 2026-05-17

---

## 1. Understanding Summary

- **What**: Port `freak.h` + `freak.cpp` + `keyframe.h` to two new Rust sibling modules:
  - `crates/core/src/kpm/freak/descriptor.rs` — FREAK extraction (84-byte data in a 96-byte slot)
  - `crates/core/src/kpm/freak/keyframe.rs` — passive container (mirrors C++ `Keyframe<96>`)
- Plus: replace the M7 `find_features` stub in `hough.rs` with a real `vision::FindFeatures`-style orchestrator.
- **Why**: Final step of Milestone 8. Closes the FREAK pipeline (pyramid → DoG detector → FREAK descriptor → `FeatureStore`). Makes the M7 KPM matching pipeline end-to-end-runnable in pure Rust.
- **Who**: Internal infrastructure consumed by KPM matching (already in main from M7).
- **Success criterion**: Live FFI dual-mode test — Rust descriptor bytes match C++ within `MAX_HAMMING_PER_DESCRIPTOR = 2` for the top-10 keypoints on `found.jpg`. Plus unit tests for descriptor length, padding, reproducibility, and Keyframe population.
- **Non-goals (this PR)**:
  - Public two-phase `OrientationAssignment` API + 7-param ctor + reusable gradient cache — these M8-3-deferred items have no real consumer in M8-4 (the FREAK descriptor doesn't read gradient images). Tracked in #138.
  - FREAK matching itself (already in M7 via `FeatureMatcher`)
  - SIMD/rayon for the descriptor inner loop (follow-up)
  - Harris detector (still dead code)

---

## 2. Decision Log

| # | Decision | Alternatives considered | Rationale |
|---|----------|-------------------------|-----------|
| 1 | **Faithful C++ port** — 96-byte storage layout (84 data + 12 zero padding), `samples[i] < samples[j]` comparison, `bitstring_set_bit` LSB-first packing, 666 pairs from 37 receptors | "spec-as-written" with 96 actual descriptor bytes; hybrid | Required for byte-for-byte parity in the dual-mode Hamming-distance test; matches M7's `hamming_distance_96` |
| 2 | **Defer M8-3 OA items indefinitely** — tracked in #138 | Absorb in M8-4 (was the original plan) | YAGNI: FREAK descriptor samples the Gaussian pyramid directly; no real consumer for the two-phase OA API or gradient cache. Documented in corrections on #128 + #129 |
| 3 | **Split into 2 modules** — `descriptor.rs` + `keyframe.rs` | Single file; 3-way split | Mirrors C++ file organization (`freak.{h,cpp}` vs `keyframe.h`); matches M8-2/M8-3 pattern |
| 4 | **Live FFI shim, Rust supplies keypoints** (option A.1) — `webarkit_cpp_extract_freak_descriptors(...)` | Pre-generated fixture file (issue spec literal); hybrid | Matches M8-2/M8-3 pattern; no binary blob; **isolates descriptor algorithm from detection variance** (descriptor parity is tested independently) |
| 5 | **`MAX_HAMMING_PER_DESCRIPTOR = 2`** for top-10 keypoint dual-mode test | Exact match (0); larger tolerance (≥ 5) | Issue spec says 0; we allow up to 2 bits / 666 (~0.3%) for libm/FMA cross-platform variance in sign decisions. Tight enough to catch real algorithm bugs |
| 6 | **Caller-owns model** with free `find_features` mirroring C++ `vision::FindFeatures` | `Keyframe::build(image, detector, pyramid)` (user prompt) | Faithful to C++ (Keyframe is a passive container; orchestration is a free function in `visual_database.h`). Fills the M7 `find_features` stub |
| 7 | **Free function `extract_freak_descriptors(pyramid, keypoints, &mut Vec<u8>)`** | Marker struct `FreakDescriptor::compute(...)` (user prompt); stateful `FreakExtractor` (C++ literal) | C++ "state" is all constants — naturally `const` at module scope in Rust. Free function is idiomatic; raw-bytes output decouples descriptor from `FeatureStore` and improves testability |
| 8 | **96-byte storage with 84 bytes of descriptor data + 12 zero padding** | 84-byte descriptors (initial draft) | C++ `FREAKExtractor::extract` calls `store.setNumBytesPerFeature(96)` then writes 84 bytes via `ExtractFREAK84`; bytes 84..96 stay zero. M7's `hamming_distance_96` assumes 96-byte descriptors |

---

## 3. Final API

### 3.1 `descriptor.rs`

```rust
/// Storage size per FREAK descriptor (matches C++ `setNumBytesPerFeature(96)`).
/// Bytes 0..84 carry 666 packed bits; bytes 84..96 are zero padding.
pub const FREAK_DESCRIPTOR_BYTES: usize = 96;

/// Compute FREAK descriptors for each keypoint and append to `out`.
///
/// After: `out.len()` has grown by `keypoints.len() * FREAK_DESCRIPTOR_BYTES`.
/// Each 96-byte slot is zero-initialized and 84 bytes are filled with
/// packed bit comparisons; bytes 84..96 remain zero.
///
/// Caller responsibility: `keypoint.angle` should be set by
/// `OrientationAssignment` (M8-3) — otherwise descriptors use `angle = 0.0`
/// and become rotation-variant.
pub fn extract_freak_descriptors(
    pyramid: &GaussianScaleSpacePyramid,
    keypoints: &[FeaturePoint],
    out: &mut Vec<u8>,
);
```

### 3.2 `keyframe.rs`

```rust
pub struct Keyframe {
    pub store: FeatureStore,  // bytes_per_feature = 96
    pub width: i32,
    pub height: i32,
}

impl Keyframe {
    pub fn new(width: i32, height: i32) -> Result<Self, KpmError>;
}
```

### 3.3 Orchestration (`hough.rs` — replaces M7 stub)

```rust
pub fn find_features(
    keyframe: &mut Keyframe,
    pyramid: &GaussianScaleSpacePyramid,
    detector: &DoGScaleInvariantDetector,
) -> Result<(), KpmError> {
    // 1. dog_points = detector.detect(pyramid)
    // 2. points: Vec<FeaturePoint> = dog_points.iter().map(Into::into).collect()
    // 3. extract_freak_descriptors(pyramid, &points, &mut buf)
    // 4. for (point, desc_slice) in zip(points, buf.chunks_exact(96)): keyframe.store.add(*point, desc_slice)?
}
```

### 3.4 Algorithm (per keypoint)

1. **Similarity transform** with `transform_scale = max(keypoint.scale · 7.0, 1.0)`; precompute `cs = transform_scale · cos(angle)`, `sn = transform_scale · sin(angle)`.
2. **Transform sigmas**: `s_n = sigma_n · transform_scale` for each ring (and center).
3. **Sample 6 rings** in C++ order (ring5 → ring4 → … → ring0 → center):
   - Per ring: `pyramid.locate(s_n)` → `(octave, scale_idx)`
   - Per receptor: apply transform `(rx, ry) = (kp.x + cs·px − sn·py, kp.y + sn·px + cs·py)`, sample via `bilinear_interpolate_f32` on `pyramid.level(octave, scale_idx)` after `bilinear_downsample_point`.
4. **Pack 666 pairwise comparisons** `samples[i] < samples[j]` (for `0 ≤ i < j < 37`) into the first 84 bytes via LSB-first `desc[pos/8] |= 1 << (pos%8)`.
5. Bytes 84..96 stay zero (from `out.resize(start + 96, 0)`).

### 3.5 FFI shim (`kpm_c_api.h/.cpp`)

```c
int webarkit_cpp_extract_freak_descriptors(
    const unsigned char* src, int src_w, int src_h,
    int num_octaves,
    const float* keypoints,    /* 4 floats per keypoint: x, y, angle, scale */
    int num_keypoints,
    unsigned char* dst_out,
    int dst_capacity_bytes);
```

C++ implementation: build `BinomialPyramid32f`, construct `std::vector<vision::FeaturePoint>` from Rust-supplied data, instantiate `FREAKExtractor`, call `extract(store, pyramid, points)`, `memcpy` `num_keypoints * 96` bytes out. Returns 0 on success; non-zero codes for validation / capacity / exception.

---

## 4. Assumptions

1. The 7 sigma constants and 6 ring arrays from `freak84-inline.h` are ported verbatim as Rust `const f32` values at module scope in `descriptor.rs`. `mExpansionFactor = 7.0` is also a `const`.
2. The Rust `FeaturePoint` (5 fields from `hough.rs`) matches C++ `vision::FeaturePoint` layout for our purposes; the `maxima` field is unused by FREAK extraction.
3. `pyramid.locate(sigma) -> (usize, usize)` from M8-2, `bilinear_downsample_point` + `bilinear_interpolate_f32` from M8-2, `From<&DoGFeaturePoint> for FeaturePoint` from M8-3 — all reused as-is.
4. `FeatureStore::new(96)` (M7-3) accepts 96 as `bytes_per_feature` and exposes `add(point, &[u8])`.
5. The existing `Keyframe` placeholder in `hough.rs:395–398` and the M7 `find_features` stub are removed/replaced. Mirrors the M8-2 / M8-3 placeholder-cleanup pattern.
6. License header on both new files; year 2026; author Walter Perdan @kalwalt.
7. `found.jpg` at `benchmarks/data/found.jpg` is the dual-mode test input.
8. `-ffp-contract=off` from M8-2 still applies to the C++ build, so libm/FMA cross-platform variance is bounded to ~1–2 ULPs per arithmetic op. The 666-bit descriptor allows for ~2 bit flips due to this variance, hence `MAX_HAMMING_PER_DESCRIPTOR = 2`.

---

## 5. Test Plan

### 5.1 `descriptor.rs` (5 unit + 1 dual-mode)

| Test | Source | Notes |
|---|---|---|
| `test_freak_descriptor_length_one_keypoint` | Issue spec | 1 keypoint → `out.len() == 96` |
| `test_freak_descriptor_length_multiple_keypoints` | Issue spec | 5 keypoints → 480 bytes |
| `test_freak_descriptor_padding_bytes_are_zero` | Hardening | `out[84..96] == [0; 12]` |
| `test_freak_descriptor_empty_input` | Hardening | Empty input doesn't panic; `out` unchanged |
| `test_freak_descriptor_is_reproducible` | Issue spec | Two runs on identical inputs → byte-equal outputs |
| `test_freak_descriptors_match_cpp_baseline` (`#[cfg(feature = "dual-mode")]`) | Issue spec (modified to live FFI) | Top-10 keypoints (Rust-selected) → C++ shim describes the same keypoints → Hamming distance ≤ 2 per descriptor |

### 5.2 `keyframe.rs` (2 unit)

| Test | Source | Notes |
|---|---|---|
| `test_keyframe_new_stores_dimensions` | Hardening | Constructor wiring; `bytes_per_feature == 96` |
| `test_find_features_populates_keyframe` | Issue spec (renamed) | On `found.jpg` → `keyframe.store.num_features() > 50` |

**Verification commands:**

```
cargo test -p webarkitlib-rs -- kpm::freak::descriptor kpm::freak::keyframe
cargo test -p webarkitlib-rs --lib --features dual-mode -- kpm::freak::descriptor
cargo clippy -p webarkitlib-rs -- -D warnings
```

---

## 6. Follow-up Work (Out of Scope This PR)

- **#138** — Promote `OrientationAssignment` API surface when a real consumer surfaces (multi-frame matcher, tuning experiments, external callers).
- **Performance** — `criterion` benchmark for `extract_freak_descriptors`; SIMD path for the 666-pair packing loop (analogous to #131/#132).
- **Reference-image building** — the `BinaryHierarchicalClustering` index that C++ `Keyframe<96>::buildIndex()` constructs is not built here. M7's KPM matching builds its own index when needed; this remains decoupled.

---

## 7. Open Risks

1. **C++ libm / FMA cross-platform variance** — the 666 sign decisions are sensitive to ULP-level FP variance in `sin`, `cos`, and `bilinear_interpolate_f32`. The `MAX_HAMMING_PER_DESCRIPTOR = 2` tolerance accommodates this. If macOS/Linux CI shows ≤ 2 consistently we can tighten to 0 in a follow-up.
2. **`pyramid.locate(sigma)` boundary behavior** — at very small or large sigmas, `locate` may clamp to a different `(octave, scale)` than C++. M8-2 verified the clamp matches C++ but the dual-mode count test (find_orientation = true) showed 0 divergence, suggesting `locate` is faithful. Worth re-verifying when M8-4 dual-mode runs.
3. **Sample ordering** — C++ samples in ring5 → ring4 → … → center order. Reordering would produce a fundamentally different bit pattern incompatible with the C++ baseline. The Rust port preserves the C++ order explicitly.
4. **`store.feature(0)` pointer arithmetic in the FFI shim** — relies on `BinaryFeatureStore` storing features contiguously with no inter-feature padding. C++ source confirms this layout (`feature(i)` is computed as `&data[i * bytes_per_feature]`).
