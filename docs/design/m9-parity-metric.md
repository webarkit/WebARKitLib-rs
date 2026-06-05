# Milestone 9 — Redefine the Dual-Mode Parity Metric

**Status**: Design approved, ready for implementation
**Branch**: `feat/dual-mode-pose-parity` (PR target: `feat/freak-visual-database`)
**Parent milestone issue**: [#139](https://github.com/webarkit/WebARKitLib-rs/issues/139)
**Issue**: [#152](https://github.com/webarkit/WebARKitLib-rs/issues/152)
**Related**: [#140](https://github.com/webarkit/WebARKitLib-rs/issues/140) / [#145](https://github.com/webarkit/WebARKitLib-rs/pull/145) (M9-1 — gate introduced), [#146](https://github.com/webarkit/WebARKitLib-rs/issues/146) / [#149](https://github.com/webarkit/WebARKitLib-rs/pull/149) (M9 BHC architecture — R1 origin), [#150](https://github.com/webarkit/WebARKitLib-rs/issues/150) / [#151](https://github.com/webarkit/WebARKitLib-rs/pull/151) (M9 auto-adjust — third suspect ruled out)
**Author**: Walter Perdan ([@kalwalt](https://github.com/kalwalt))
**Date**: 2026-05-20

---

## 1. Understanding Summary

- **What**: Replace `test_visual_database_matches_cpp_pipeline`'s absolute-inlier-count assertion (`|rust - cpp| <= 5`) with a **corner-reprojection-error** assertion against the homography output. Un-`#[ignore]` the test once it passes.
- **Why**: Close the M9 milestone parity gate that has been `#[ignore]`d since M9-1 (#140) at `diff=15` inliers. Three algorithmic suspects investigated and closed across #145 / #149 / #151 — all proven byte-equivalent to C++ at the unit level. Residual gap is BHC tree-topology cross-language nondeterminism (M9 #146 R1), which is structural and unfixable from the Rust side. The new metric is intrinsically invariant to that variance.
- **Who for**: Internal — closes the M9 milestone parity gate. M9-2 (#141) will inherit the metric pattern via the heads-up comment posted to that issue.
- **Key constraints**: (a) Max corner reprojection error within an adaptive tolerance per M9 #146 Decision 10 (`max(2.0, observed)` px, ceiling 5 px). (b) Existing tests must continue to pass. (c) `cargo clippy --all-targets --all-features -- --deny warnings` clean. (d) The test runs by default after this PR — no more `#[ignore]`.
- **Non-goals**: No FFI changes (C++ already exposes the 3×3 homography via `kpm_query`'s `pose_out[0..9]`). No 3D pose extraction (we're comparing homographies, not poses). No M9-2 work (deferred to #141). No reworking of `matched_geometry` API or `homography.rs` public surface.

---

## 2. The diagnostic trail (why we redefine)

The original M9-1 parity gate assertion was:

```rust
let diff = (rust_inliers as i32 - cpp_inliers as i32).abs();
assert!(diff <= 5, "inlier count divergence: rust={} cpp={}", rust_inliers, cpp_inliers);
```

This failed at `rust=441, cpp=456 (diff=15)` and stayed there across three PRs of algorithmic fixes:

| Investigated suspect | Closed in | Effect on diff |
|---|---|---|
| `find_hough_matches` stub (no bin-distance filtering) | M9-1 (#145) | unchanged at 15 |
| BHC settings `(num_hypotheses=1, max_nodes_to_pop=0)` vs C++ `(128, 8)` | #146 / #149 | unchanged at 15 |
| `autoAdjustXYNumBins` not ported (fixed 12×12 vs C++ auto-sized) | #150 / #151 | unchanged at 15 |

Both #149 and #151 shipped dedicated dual-mode FFI tests that prove the Rust algorithmic ports are byte-equivalent to C++ at the unit level:
- BHC partition logic — byte-identical when configured with C++ `Keyframe::buildIndex` settings.
- `autoAdjustXYNumBins` — byte-identical across 40/40 seeded random trials.
- `fast_median_f32` + `partial_sort_f32` — byte-identical across 50/50 trials.

The pipeline math is correct. The residual gap is **BHC tree-topology cross-language nondeterminism** (M9 #146 R1): both Rust (`BTreeMap`/`HashMap`) and C++ (`std::unordered_map`) use unordered-key maps when grouping K-medoids assignments into child clusters during BHC build (`binary_hierarchical_clustering.h:217`). Hash orderings differ across toolchains → BHC trees differ → matches differ → downstream metrics differ by a stable ~15 inliers.

The BHC algorithm tolerates this (priority-queue traversal handles ties), but **byte-equivalent cross-language tree-build determinism isn't achievable** without patching the WebARKit C++ source to use `std::map` instead of `std::unordered_map`. That patch was considered and rejected as a path forward (high coordination cost, third-party submodule edit, M9 can't wait on upstream).

**Conclusion**: the absolute-inlier-count metric is the wrong question. What matters is whether both pipelines converge on the same end-to-end matching result — and that result is best measured by where the homography warps the reference image, not by which specific feature matches contributed.

---

## 3. The C++ `pose_out` layout finding

Issue #152's R3 worried that *"the Rust side returns a 3×3 homography from query → matched ref; C++ `kpm_query` returns a 3×4 pose. These are different things — the redefinition needs to compare like-with-like."*

Reading [`kpm_c_api.cpp:156-166`](../../crates/core/src/kpm/kpm_c_api.cpp) carefully:

```cpp
// Copy the 3x3 homography matrix into the first 9 elements of pose_out.
const float* geom = handle->db->matchedGeometry();
if (geom) {
    std::memcpy(pose_out, geom, 9 * sizeof(float));
} else {
    std::memset(pose_out, 0, 9 * sizeof(float));
}
// Zero the remaining 3 elements.
pose_out[9] = 0.0f;
pose_out[10] = 0.0f;
pose_out[11] = 0.0f;
```

The C++ `kpm_query`'s `pose_out[12]` is actually **the 3×3 homography in the first 9 elements + 3 trailing zeros for FFI convenience.** Both sides produce the **same object** — a 3×3 row-major homography from reference → query, normalized so `H[8] = 1` (per M6's `RobustHomography::find` docstring).

**Implication**: no new FFI shim needed, and the metric is naturally "homography accuracy" rather than the originally-proposed "pose + translation". The issue body's R3 is closed by inspection — no implementation work required.

---

## 4. Decision Log

| # | Decision | Alternatives considered | Rationale |
|---|----------|-------------------------|-----------|
| 1 | **Corner reprojection error** as the primary parity metric — max pixel displacement of the 4 reference-image corners projected through Rust H vs C++ H | Frobenius norm of (H_rust - H_cpp); both as primary + secondary | Geometrically meaningful, language-agnostic, intuitive tolerance unit (pixels). Frobenius is abstract and dimensionally inconsistent. |
| 2 | **Drop the inlier-count assertion entirely** | Keep as a loose sanity check (e.g. `|diff| <= 50`); tighten to ~25 inliers | Reprojection error already captures end-to-end correctness; the inlier count is a noisy proxy that creates contradictory failure modes. |
| 3 | **Adaptive tolerance** `max(2.0, observed)` px with 5 px ceiling requiring investigation | Hard `<= 5 px`; hard `<= 1 px` | Mirrors M9 #146 Decision 10. Measure during implementation; if observed > 5 px, root-cause before merging. |
| 4 | **Private helper inside the test module** — `fn reproject_corners(...)` in `#[cfg(test)] mod tests` of `visual_database.rs` | Public utility in `homography.rs`; inline 4× calls | YAGNI-correct: only caller is the parity test. Promote later when a real second caller emerges. |
| 5 | **Keep test name `test_visual_database_matches_cpp_pipeline`** — docstring + assertion update only | Rename to `_homography_matches_cpp_pipeline`; rename to `_dual_mode_pose_parity_on_pinball` | Preserves git blame continuity through M9-1 → #146 → #150 → #152. The name is still accurate: it does check that Rust matches C++ on the pipeline output. |
| 6 | **Leave M9-2 milestone-gate redefinition for the M9-2 PR** (heads-up comment already posted on #141) | Stub the M9-2 test in this PR | Can't redefine a test that doesn't exist yet (`DualFreakMatcher` lands in M9-2). This PR establishes the metric blueprint; M9-2 applies it. |
| 7 | **Write `docs/design/m9-parity-metric.md`** | Skip the design doc; rely on PR body + test docstring | Matches the project pattern set by m9-1, m9-keyframe-bhc-index, m9-hough-auto-adjust-xy-bins. Captures the full diagnostic trail discoverably. |
| 8 | **`max(corner_displacements)`** not mean | Mean only; both max + mean | Max is tighter and simpler. The 4 corners are correlated; mean is just a softer version of max. |
| 9 | **Helper signature `[[f32; 2]; 4]`** array-of-arrays | `[(f32, f32); 4]`; `[f32; 8]` flat | Consistent with the rest of the `homography.rs` API — `multiply_point_homography_inhomogenous` already takes `&[f32; 2]` for points. |
| 10 | **Print observed values via `arlog_i!` before the assertion** | Only print on failure (via `assert!` message) | Future tightening of the tolerance is data-driven: read the CI log for the actual value. Cheap; matches the diagnostic style of M9 #146 / #150. |

---

## 5. Final Design

### 5.1 Files touched

```
crates/core/src/kpm/freak/visual_database.rs   ← rewrite test + add private helper
docs/design/m9-parity-metric.md                ← NEW (this file)
```

No `homography.rs` changes, no FFI changes, no `Cargo.toml` changes, no new public surface.

### 5.2 Private helper

```rust
// In #[cfg(test)] mod tests of visual_database.rs:

/// Project the four corners of a reference image of size (w, h) through
/// the given 3x3 row-major homography. Returns the 4 projected points
/// in order: top-left, top-right, bottom-right, bottom-left.
///
/// Used by `test_visual_database_matches_cpp_pipeline` to measure how
/// differently the Rust vs C++ pipelines warp the reference image —
/// a metric intrinsically invariant to BHC tree-topology cross-language
/// nondeterminism (M9 #146 R1).
#[cfg(feature = "dual-mode")]
fn reproject_corners(h: &[f32; 9], w: i32, h_dim: i32) -> [[f32; 2]; 4] {
    let corners: [[f32; 2]; 4] = [
        [0.0, 0.0],
        [w as f32, 0.0],
        [w as f32, h_dim as f32],
        [0.0, h_dim as f32],
    ];
    let mut out = [[0.0_f32; 2]; 4];
    for (i, c) in corners.iter().enumerate() {
        multiply_point_homography_inhomogenous(&mut out[i], h, c);
    }
    out
}
```

### 5.3 Rewritten test (post-#152 shape)

```rust
/// M9 dual-mode parity gate.
///
/// Asserts that the Rust and C++ pipelines warp the reference image into
/// the same query-image region within a few pixels of agreement — a metric
/// intrinsically invariant to BHC tree-topology cross-language
/// nondeterminism. See `docs/design/m9-parity-metric.md` for the full
/// diagnostic trail (why we redefined this from absolute inlier count to
/// homography corner reprojection).
///
/// Tolerance follows the M9 #146 Decision 10 pattern: assert `<= max(2.0,
/// observed)` rounded up to nearest pixel, with a 5 px ceiling that
/// requires investigation before merging.
#[test]
#[cfg(feature = "dual-mode")]
fn test_visual_database_matches_cpp_pipeline() {
    use crate::arlog_i;
    use crate::kpm::kpm_ffi;

    let reference = load_grayscale("../../benchmarks/data/found.jpg");
    let query = load_grayscale("../../benchmarks/data/img.jpg");
    let ref_w = reference.cols as i32;
    let ref_h = reference.rows as i32;

    // --- Rust path ---
    let mut db = VisualDatabase::new().expect("new");
    db.add_image(&reference, 0).expect("add_image");
    let rust_matched = db.query(&query).expect("query");
    let rust_id = db.matched_db_id();
    let rust_h = *db.matched_geometry().expect("Rust matched, must have geometry");

    // --- C++ path via existing kpm_query (no new FFI needed) ---
    let cpp_h: [f32; 9];
    let cpp_id;
    unsafe {
        let handle = kpm_ffi::kpm_create(query.cols as i32, query.rows as i32);
        assert!(!handle.is_null());
        let rc = kpm_ffi::kpm_add_ref_image(
            handle, reference.data.as_ptr(),
            ref_w, ref_h, 72.0, 0, 0,
        );
        assert!(rc >= 0);

        let mut pose = [0.0f32; 12];
        let mut error_out = 0.0f32;
        let mut page_no_out = -1i32;
        let rc = kpm_ffi::kpm_query(
            handle, query.data.as_ptr(),
            query.cols as i32, query.rows as i32,
            pose.as_mut_ptr(), &mut error_out, &mut page_no_out,
        );
        assert!(rc >= 0, "C++ kpm_query failed");

        cpp_id = kpm_ffi::kpm_matched_id(handle);
        // pose[0..9] is the 3x3 row-major homography; pose[9..12] is zero-padding.
        // See kpm_c_api.cpp:156-166.
        cpp_h = [pose[0], pose[1], pose[2], pose[3], pose[4],
                 pose[5], pose[6], pose[7], pose[8]];
        kpm_ffi::kpm_destroy(handle);
    }

    // --- Sanity ---
    assert!(rust_matched);
    assert_eq!(rust_id, cpp_id);

    // --- Corner reprojection parity (M9 #152) ---
    let rust_corners = reproject_corners(&rust_h, ref_w, ref_h);
    let cpp_corners  = reproject_corners(&cpp_h,  ref_w, ref_h);
    let mut max_displacement = 0.0_f32;
    let mut per_corner = [0.0_f32; 4];
    for i in 0..4 {
        let dx = rust_corners[i][0] - cpp_corners[i][0];
        let dy = rust_corners[i][1] - cpp_corners[i][1];
        let d = (dx * dx + dy * dy).sqrt();
        per_corner[i] = d;
        if d > max_displacement {
            max_displacement = d;
        }
    }

    arlog_i!(
        "M9 dual-mode parity: max corner displacement = {:.4} px \
         (per corner: tl={:.4}, tr={:.4}, br={:.4}, bl={:.4})",
        max_displacement, per_corner[0], per_corner[1], per_corner[2], per_corner[3]
    );

    // Tolerance set per M9 #146 Decision 10 after measurement (see §10.5).
    // Observed during implementation: 0.24 px. Floor of 2.0 gives ~8× margin.
    const TOLERANCE_PX: f32 = 2.0;
    assert!(
        max_displacement <= TOLERANCE_PX,
        "max corner displacement = {:.4} px > tolerance {} px",
        max_displacement, TOLERANCE_PX
    );
}
```

---

## 6. Assumptions

- **A1.** C++ `kpm_query`'s `pose_out[0..9]` IS the 3×3 row-major homography from reference → query, normalized so `pose_out[8] == 1.0`. **Verified** by inspection of `kpm_c_api.cpp:156-166` and the M6 `RobustHomography::find` docstring. (See §3 of this doc.)
- **A2.** Both pipelines, given the byte-equivalent pyramid+DoG+FREAK upstream (verified in M8 dual-mode tests), produce homographies that warp the reference into approximately the same query-image region. The "approximately" tolerance is what we'll measure during implementation — expected to be sub-pixel to a few pixels at worst.
- **A3.** The observed max corner reprojection error on the pinball pair will be ≤ 5 px. If it's > 5 px, the BHC nondeterminism causes more geometric divergence than expected and we'll need to investigate before merging.
- **A4.** No FFI changes are needed. The C++ side already exposes everything through the existing `kpm_query` shim. **Verified** by inspection of `kpm_c_api.{h,cpp}`.

---

## 7. Risks

- **R1 (low).** Observed max corner displacement exceeds the 5 px ceiling. *Mitigation*: investigate before merging. If R1 triggers, the BHC variance manifests as a larger geometric shift than the algorithmic-byte-equivalence work suggested — likely indicates a deeper issue (e.g., RANSAC seed divergence we missed, or a homography normalization bug). Don't widen the ceiling without root-causing.
- **R2 (low).** Tolerance just above the observed value may be brittle to small upstream changes (e.g., future M8 pyramid tweak shifts the descriptors slightly, displaces the homography by 0.5 px, breaks the test). *Mitigation*: the floor of `2.0` px provides headroom for natural float-rounding drift. If observed is `1.2 px`, asserting `<= 2.0 px` gives ~67% safety margin. Future upstream changes that legitimately shift the value can update the tolerance in their own PR (one-line change, with the new observed value in the commit message).
- **R3 (very low).** C++ `kpm_query` returns a different homography on the same input than we measured during M9 #146 / #150 implementations. *Mitigation*: A1 verified the layout from source. The C++ pipeline is deterministic given the same input; the homography won't change unless the C++ source changes.

---

## 8. Files modified (estimate)

| File | Change | Est. LOC |
|---|---|---|
| `crates/core/src/kpm/freak/visual_database.rs` | rewrite test body + add private `reproject_corners` helper, remove `#[ignore]` | ~80 |
| `docs/design/m9-parity-metric.md` | **NEW** | ~270 |
| **Total** | | **~350** |

Smaller than any of the three preceding M9 PRs by design — this one closes the loop on diagnosis work that those PRs did.

---

## 9. Verification (CLAUDE.md §5)

```
cargo fmt --all -- --check
cargo build --all-features
cargo clippy --all-targets --all-features -- --deny warnings
cargo test --all-features                                  # full suite
cargo test --features dual-mode -- test_visual_database_matches_cpp_pipeline
```

All five steps clean; the parity gate runs by default and asserts the new tolerance.

---

## 10. Exit criteria

- [x] Understanding Lock confirmed by author 2026-05-20.
- [x] Final design approved across both brainstorming sections.
- [x] Decision log: D1–D10.
- [x] Assumptions: A1–A4.
- [x] Risks: R1–R3.
- [x] Implementation complete; tolerance set per measurement (§10.5).

---

## 10.5. Measured outcome (post-implementation)

The first run of the rewritten test on the pinball pair
(`benchmarks/data/found.jpg` + `img.jpg`) measured:

```
max_displacement = 0.237754 px
per corner: tl=0.109074, tr=0.237754, br=0.068354, bl=0.055763
```

All four corners agree to within a quarter of a pixel between Rust and
C++ pipelines — **sub-pixel parity** despite the underlying 15-inlier
divergence in the matching layer. This empirically confirms what we
suspected: even with different specific feature matches contributing to
RANSAC, the algorithm finds the same dominant geometric consensus
because the matches are drawn from the same underlying images.

**Tolerance set**: `TOLERANCE_PX = max(2.0, ceil(0.237754)) = 2.0 px`.
This is **8.4× the observed value**, giving substantial safety margin
against:
- Future float-rounding drift in upstream M6–M8 components (e.g. a
  pyramid filter coefficient change).
- Hardware/toolchain rounding variation (Windows MSVC vs Linux GCC).
- Small RANSAC-seed-induced drift from future M6 / M7 work.

**Assumptions A2 and A3 validated**: both predicted sub-pixel agreement
and ≤ 5 px ceiling clearance.

**Risks R1, R2, R3 did not materialize**:
- R1 (observed > 5 px): observed = 0.24 px. Comfortable margin.
- R2 (tolerance brittle): 8.4× margin makes this very unlikely.
- R3 (C++ homography surprise): A1 verified by source inspection; no
  runtime surprise.

The diagnostic trail from M9-1 → #146 → #150 → #152 is now complete:
- M9-1: introduced the parity gate, observed 15-inlier divergence.
- #146 / #149: ruled out BHC settings + traversal as a cause.
- #150 / #151: ruled out the missing autoAdjustXYNumBins port as a cause.
- **#152 (this PR)**: redefined the metric to one that's intrinsically
  invariant to BHC tree-topology variance. Test now passes byte-equivalently.

---

## 10.6. What this means for the M9 milestone

`test_visual_database_matches_cpp_pipeline` now runs by default on every
`cargo test --features dual-mode --lib` invocation, asserting that the
Rust and C++ pipelines produce essentially the same homography on the
pinball pair. **The M9 parity gate is closed.**

The heads-up on [#141](https://github.com/webarkit/WebARKitLib-rs/issues/141)
recommends that M9-2's `test_dual_mode_no_divergence_on_pinball` adopt
the same corner-reprojection metric instead of its current
"zero divergence" framing. With this PR landed, the path is clear:
M9-2 lands, M9-3 flips the default off `ffi-backend`, and Milestone 9
closes.
