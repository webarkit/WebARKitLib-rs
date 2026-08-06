# `clippy::too_many_arguments` audit (#83)

Classification of every `#[allow(clippy::too_many_arguments)]` suppression
site in the codebase, and the planned action for each. Tracks the
incremental cleanup requested in
[#83](https://github.com/webarkit/WebARKitLib-rs/issues/83).

**Context:** all sites are currently `#[allow]`-silenced (CI's strict
`--all-targets --all-features -D warnings` gate is green because of the
allows, not because the functions are refactored). #83 asks to *reduce*
them by grouping tightly-coupled arguments into param/context structs where
that is idiomatic and safe.

**Decision (per maintainer, 2026-07-15):** refactor the private/internal
helpers into param structs now (zero API risk); on the public-API and
SIMD-locked sites, keep the allow and add a `// rationale:` comment. A full
public-API restructure is deferred to a deliberate pre-1.0 decision.

---

## #83 original scope — `ar/` + `ar2/`

### Refactor now — private helpers, zero API risk
| Function | Location | Args | Call sites |
|----------|----------|------|-----------|
| `sample_grid` | `ar/matrix.rs` | 8 | 2 (in-module) |
| `make_template` | `ar2/feature_map.rs` | 9 | 3 (in-module + test) |
| `select_features` | `ar2/feature_map.rs` | 10 | 1 (test) |

### Keep allow + rationale — public C-faithful API (breaking to refactor)
| Function | Location |
|----------|----------|
| `ar_labeling` | `ar/labeling.rs` |
| `ar_detect_marker2` | `ar/marker.rs` |
| `ar_get_marker_info` | `ar/marker.rs` |
| `ar_patt_save` | `ar/pattern.rs` |
| `ar_patt_get_image` | `ar/pattern.rs` |
| `ar_patt_get_image2` | `ar/pattern.rs` |
| `ar2_tracking_2d_sub` | `ar2/tracking.rs` |
| `ar2_get_best_matching` | `ar2/tracking.rs` |
| `ar2_get_best_matching_sub_fine` | `ar2/tracking.rs` |

### Keep allow + rationale — SIMD runtime-dispatch, signatures locked (CLAUDE.md §7.4)
| Function | Location |
|----------|----------|
| `get_similarity` | `ar2/feature_map.rs` |
| `get_similarity_scalar` | `ar2/feature_map.rs` |
| `get_similarity_sse41` | `ar2/feature_map.rs` |
| `get_similarity_avx2` | `ar2/feature_map.rs` |

---

## Added since #83 — `kpm/freak/` (later increment)

| Function | Location | Recommended |
|----------|----------|-------------|
| `homography_4_points_geometrically_consistent` | `kpm/freak/homography.rs` | private math — refactor candidate (defer*) |
| `condition_4_points_2d` | `kpm/freak/homography.rs` | private math — defer* |
| `denormalize_homography` | `kpm/freak/homography.rs` | private math — defer* |
| `solve_homography_4_points_inhomogeneous` | `kpm/freak/homography.rs` | private math — defer* |
| `solve_homography_4_points` | `kpm/freak/homography.rs` | private math — defer* |
| `compute_homography_normal_equations_post_multiply` | `kpm/freak/homography.rs` | private math — defer* |
| `preemptive_robust_homography` | `kpm/freak/homography.rs` | `pub` — rationale |
| `polish_homography` | `kpm/freak/homography.rs` | `pub` — rationale |
| `HoughSimilarityVoting::new` | `kpm/freak/hough.rs` | `pub` — rationale |
| `HoughSimilarityVoting::new_auto_xy` | `kpm/freak/hough.rs` | `pub` — rationale |
| `find_hough_similarity` | `kpm/freak/hough.rs` | `pub` — rationale |
| `webarkit_cpp_auto_adjust_xy_num_bins` | `kpm/freak/hough.rs` | FFI extern — leave (matches shim cfg) |
| `generate` | `kpm/ref_data_set.rs` | `pub` — rationale |
| module-wide `#![allow]` | `kpm/freak/detector.rs` | narrow to per-fn later |

\* The homography math helpers are private (so technically refactorable),
but they're bit-parity-sensitive numerical C ports where flat
`(points, matrix, scalars)` signatures often read clearer than a struct.
Defer to a deliberate pass rather than obscure the math.

---

## Incremental plan

1. **Done (PR #228):** refactored `sample_grid`, `make_template`,
   `select_features`; added rationale comments to the public/SIMD `ar`/`ar2`
   allows.
2. **Done:** `kpm/freak` + `ref_data_set` rationale comments — the
   homography numerical routines, the Hough constructors / voting, and
   `KpmRefDataSet::generate`. The `webarkit_cpp_*` FFI extern shim and the
   `detector.rs` module-level allow are left as-is: both already carry an
   explanatory rationale comment, and narrowing the 34-fn detector module
   would be churn with no lint benefit.
3. **Pre-1.0 (maybe):** deliberate public-API restructure of the
   C-faithful `ar`/`ar2` entry points into param structs (breaking). This is
   the only remaining item; every current allow is now either removed or
   carries a `// rationale:` note, so #83 can be closed and this restructure
   tracked as its own pre-1.0 API task if desired.
