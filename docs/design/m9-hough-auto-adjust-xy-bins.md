# Milestone 9 — Port `HoughSimilarityVoting::autoAdjustXYNumBins`

**Status**: Design approved, ready for implementation
**Branch**: `feat/hough-auto-adjust-xy-bins` (PR target: `feat/freak-visual-database`)
**Parent milestone issue**: [#139](https://github.com/webarkit/WebARKitLib-rs/issues/139)
**Issue**: [#150](https://github.com/webarkit/WebARKitLib-rs/issues/150)
**Related**: [#140](https://github.com/webarkit/WebARKitLib-rs/issues/140) (M9-1 baseline / R1 origin), [#146](https://github.com/webarkit/WebARKitLib-rs/issues/146) / [#149](https://github.com/webarkit/WebARKitLib-rs/pull/149) (M9 BHC index, R3 origin)
**Author**: Walter Perdan ([@kalwalt](https://github.com/kalwalt))
**Date**: 2026-05-20

---

## 1. Understanding Summary

- **What**: Port C++ `HoughSimilarityVoting::autoAdjustXYNumBins` (`hough_similarity_voting.cpp:204-236`) — the missing piece that auto-sizes the x/y bin grid based on the median projected dimension of input matches per voting pass. Wire it into the Rust `find_hough_similarity` flow and update `visual_database.rs::make_hough_voter` to use the auto-adjust path that C++ uses in production (`visual_database.h:312`).
- **Why**: Close the M9 dual-mode parity gate. `test_visual_database_matches_cpp_pipeline` has been `#[ignore]`'d since M9-1 (#145) at `diff=15` inliers and remained at `diff=15` after the BHC infrastructure landed in #149. The M9 #146 design doc Assumption A3 named `autoAdjustXYNumBins` as the remaining likely contributor; this PR validates that diagnosis.
- **Who for**: Internal — `make_hough_voter` in `visual_database.rs` is the only production caller of `BinParams::new`. Library users gain a named `BinParams::new_auto_xy(...)` factory that maps 1:1 with C++ `HoughSimilarityVoting::init(_, _, _, _, 0, 0, ...)`.
- **Key constraints**:
  - Bit-equivalent `(num_x_bins, num_y_bins)` to C++ on the same seeded inputs (dedicated dual-mode FFI test).
  - `fast_median_f32` bit-equivalent to C++ `FastMedian<float>` (separate diagnostic shim).
  - Existing matcher and BHC dual-mode tests must continue to pass.
  - `cargo clippy --all-targets --all-features -- --deny warnings` clean.
  - Un-`#[ignore]` the milestone parity gate; tolerance per #146 Decision 10 (`max(2, observed_diff)`).
- **Non-goals**: No SIMD/rayon. No public "manual override" path (YAGNI). No rework of M6 RANSAC `fast_median` (the private tuple-shaped one stays untouched). No new public config structs.

---

## 2. Decision Log

| # | Decision | Alternatives considered | Rationale |
|---|----------|-------------------------|-----------|
| 1 | **`BinParams::new_auto_xy(...)` factory** + private `HoughSimilarityVoting::recompute_xy_bins_from_matches` method, invoked inline at the top of `find_hough_similarity` | Sentinel `num_x_bins == 0` in existing `new`; runtime `enable_auto_adjust_xy()` method | Idiomatic Rust; one named entry point; matches the single-call-per-(query,ref) semantics of C++ batched `vote(ins, ref, size)`. The factory makes intent explicit. |
| 2 | **`pub fn fast_median_f32(values: &mut [f32]) -> f32`** in `crates/core/src/kpm/freak/math.rs` (sibling of `safe_division_f32`) | Stdlib `slice::select_nth_unchecked`; promote+generalize existing `fast_median` in homography.rs | Bit-equivalent to C++ `FastMedian<float>` on ties (lower-midpoint convention, not arithmetic mean). Low risk; no churn to existing RANSAC `fast_median`. |
| 3 | **Make `BinParams.num_x_bins` / `num_y_bins` private**; add private `set_xy_bins(x, y)` that recomputes `a` and `b` atomically | Keep `pub` and document the contract; recompute strides on every getter | Strides can never go stale. Grep-verified safe: no external readers/writers of these fields. |
| 4 | **Add dedicated dual-mode FFI test** `auto_adjust_xy_num_bins_matches_cpp` via new C++ shim `webarkit_cpp_auto_adjust_xy_num_bins` | Rely solely on the end-to-end parity test; pure-Rust analytic-formula test | Isolates auto-adjust parity from the rest of the pipeline. Strong diagnostic value if the end-to-end test drifts. |
| 5 | **Remove `HOUGH_NUM_X_BINS` / `HOUGH_NUM_Y_BINS` constants** from `visual_database.rs` once `make_hough_voter` switches to auto-adjust | Keep them as documented dead code; keep as feature-flagged override path | Dead code under `--deny warnings`. Git history captures the prior workaround. C++ has no equivalent constants — only `HOUGH_NUM_ANGLE_BINS = 12` and `HOUGH_NUM_SCALE_BINS = 10` (from `visual_database.h:312`) stay. |
| 6 | **No new SIMD / rayon / benchmarks** | Add a microbench for fast_median_f32; pre-sort projected_dim | Median runs once per (query, ref) pair — not a hot path. Scope discipline matches M9 #146. |
| 7 | **`BinParams::new_auto_xy(...)` initializes `num_x_bins = num_y_bins = 5`** (the clamp floor used by `recompute_xy_bins_from_matches`) | Initialize to 0 (matching C++ pre-vote state); initialize to MAX so first vote always recomputes | Keeps the BinParams in a valid state pre-recompute. Strides `a` and `b` are non-zero. Avoids edge cases if a caller queries the voter without first running `find_hough_similarity`. |
| 8 | **C++ FFI shim lives in `crates/core/src/kpm/kpm_c_api.{h,cpp}`** alongside the M9 #146 BHC shim | Separate file `hough_c_api.{h,cpp}` | Consistency with #146's shim placement. Single point of build-system inclusion. |
| 9 | **Un-`#[ignore]` `test_visual_database_matches_cpp_pipeline` by default**; soft-skip via `#[cfg_attr(feature = "skip-parity-gate", ignore = "...")]` | Remove `#[ignore]` cleanly; keep `#[ignore]` and require manual enable | Default behaviour is to enforce parity (this PR's stated goal). Soft-skip provides an escape hatch for devs with broken FFI builds or triaging unrelated work without weakening the gate. Add `skip-parity-gate = []` feature to Cargo.toml. |
| 10 | **Preemptive lower-level dual-mode shim** `webarkit_cpp_partial_sort_f32` + `partial_sort_f32_matches_cpp` test | Add only if `auto_adjust_xy_num_bins_matches_cpp` fails | Parallels M7's layering (`fast_random` + `array_shuffle` both have dedicated dual-mode tests). Localizes failure: if median is wrong, the lower test catches it; if median is right but auto-adjust still diverges, the bug is in the surrounding math. |

---

## 3. Final Design

### 3.1 New public API (`math.rs`)

```rust
pub fn fast_median_f32(values: &mut [f32]) -> f32;
// Private:
fn partial_sort_f32(values: &mut [f32], k: usize);
```

Direct port of C++ `FastMedian<float>` (single-value overload, not the tuple
overload used by RANSAC). Lower-midpoint semantics on even-length inputs.
~30–40 LOC including the private helper.

### 3.2 `BinParams` changes (`hough.rs`)

- `num_x_bins` and `num_y_bins` become private fields.
- New `pub fn num_x_bins(&self) -> i32` / `pub fn num_y_bins(&self) -> i32` getters.
- New `pub fn new_auto_xy(...)` factory: same args as `new` minus `num_x_bins` / `num_y_bins`; initializes both to 5 and sets `auto_adjust_xy: bool = true`.
- New `pub(crate) fn set_xy_bins(&mut self, x: i32, y: i32)`: atomically updates `num_x_bins`, `num_y_bins`, recomputes `a` and `b` strides.
- New internal field `auto_adjust_xy: bool` (default `false`).

### 3.3 `HoughSimilarityVoting` changes (`hough.rs`)

- New private method:
  ```rust
  fn recompute_xy_bins_from_matches(
      &mut self,
      query_points: &[FeaturePoint],
      ref_points: &[FeaturePoint],
      matches: &[HoughMatch],
  ) -> Result<(), KpmError>;
  ```
  Mirrors C++ `autoAdjustXYNumBins`: builds the `projected_dim` array,
  calls `fast_median_f32`, computes `bin_size = 0.25 × median`, clamps
  `num_x_bins`/`num_y_bins` to ≥ 5, calls `params.set_xy_bins(x, y)`.
- `find_hough_similarity`: insert auto-adjust invocation right after the
  empty-match guard:
  ```rust
  if voting.params.auto_adjust_xy {
      voting.recompute_xy_bins_from_matches(query_points, ref_points, matches)?;
  }
  ```

### 3.4 `visual_database.rs` changes

- Remove `HOUGH_NUM_X_BINS` and `HOUGH_NUM_Y_BINS` constants.
- `make_hough_voter` switches from `BinParams::new(...)` to `BinParams::new_auto_xy(...)`.
- Un-`#[ignore]` `test_visual_database_matches_cpp_pipeline`; add `#[cfg_attr(feature = "skip-parity-gate", ignore = "...")]`.
- Tolerance per #146 Decision 10 — set after measuring the post-implementation diff.

### 3.5 New C++ FFI shims (`kpm_c_api.h` + `kpm_c_api.cpp`)

```c
int webarkit_cpp_partial_sort_f32(float* values, int n, int k);
int webarkit_cpp_auto_adjust_xy_num_bins(
    const float* ins, const float* ref_pts, int size,
    int ref_image_width, int ref_image_height,
    float min_x, float max_x, float min_y, float max_y,
    int num_angle_bins, int num_scale_bins,
    int* out_num_x_bins, int* out_num_y_bins);
```

`partial_sort_f32` returns 0 on success; mutates `values` in place; caller reads `values[k]`. `auto_adjust_xy_num_bins` returns 0 on success; writes bin counts to out pointers.

### 3.6 `Cargo.toml` addition

```toml
[features]
# ... existing
skip-parity-gate = []
```

---

## 4. Tests

### 4.1 Unit tests in `math.rs`

- `test_fast_median_f32_odd_length` — middle element semantics.
- `test_fast_median_f32_even_length_uses_lower_midpoint` — C++ lower-midpoint convention (not arithmetic mean).
- `test_fast_median_f32_single_element` — n=1 edge case.
- `test_fast_median_f32_already_sorted` — n=100 stress.

### 4.2 Unit tests in `hough.rs`

- `test_bin_params_new_auto_xy_initial_state` — `num_x_bins=5`, `num_y_bins=5`, `auto_adjust_xy=true`.
- `test_bin_params_new_disables_auto_adjust` — backwards-compat: `new(...)` keeps `auto_adjust_xy=false`.
- `test_recompute_xy_bins_from_matches_known_input` — hand-crafted matches → assert analytic formula.
- `test_recompute_xy_bins_from_matches_clamps_at_5` — degenerate inputs floor at 5.

### 4.3 Dual-mode FFI tests

In `math.rs` `dual_mode_tests` (or new module):
- `partial_sort_f32_matches_cpp` — seeded random `[f32]` inputs, multiple k values. Asserts bit-identical result with C++ `vision::PartialSort<float>`.

In `hough.rs` `dual_mode_tests`:
- `auto_adjust_xy_num_bins_matches_cpp` — seeded random match pairs, asserts bit-identical `(num_x_bins, num_y_bins)` with C++ `autoAdjustXYNumBins`.

### 4.4 The milestone gate

In `visual_database.rs`:
- `test_visual_database_matches_cpp_pipeline` un-`#[ignore]`'d. Apply tolerance per #146 Decision 10 (`max(2, observed_diff)`).
- **Closing this is the success criterion for the PR.**

### 4.5 Verification workflow (CLAUDE.md §5)

```
cargo fmt --all -- --check
cargo build --all-features
cargo clippy --all-targets --all-features -- --deny warnings
cargo test --all-features                                  # parity gate active
cargo test --all-features --features skip-parity-gate      # gate skipped, sanity check
```

All five steps clean.

---

## 5. Assumptions

- **A1.** Rust `fast_median_f32` port using the same `partial_sort` algorithm as C++ `PartialSort` produces bit-equivalent results on tied inputs. Verified by the dedicated `partial_sort_f32_matches_cpp` dual-mode test (D10).
- **A2.** Closing the auto-adjust gap brings `test_visual_database_matches_cpp_pipeline` to `diff ≤ 2` on the pinball pair. If diff lands at 0–2 → tolerance `±2`. If 3 → `±3`. If ≥ 4 → investigate before merging.
- **A3.** The C++ `mAutoAdjustXYNumBins = true` flag is set exactly once per `init()` call when `numXBins == 0 && numYBins == 0`. Rust mirrors via `auto_adjust_xy: bool` set by `new_auto_xy`, cleared by `new`.
- **A4.** Auto-adjust recomputation runs once per `find_hough_similarity` call (matching C++ `vote(ins, ref, size)` semantics), not per individual `vote()` call. The `vote(x, y, angle, scale)` primitive is unaffected.

---

## 6. Risks (post-implementation status)

- **R1 (DID NOT MATERIALIZE).** `partial_sort_f32` byte-equivalent to C++.
  Dual-mode test `dual_mode_partial_sort_f32_matches_cpp` passes 50/50 trials
  including injected duplicates. The Lomuto partition port worked first
  try. Tie-break ordering tracks C++ exactly.

- **R2 (MATERIALIZED differently than predicted).** Parity gate **still
  shows `diff=15` inliers** — identical to M9-1 (#140) and M9 #146 (#149).
  The auto-adjust dual-mode test `auto_adjust_xy_num_bins_matches_cpp`
  passes 40/40 trials, proving the Rust port is byte-equivalent to C++ at
  the algorithm level. End-to-end, however, Rust's auto-adjust runs on
  *different inputs* than C++'s because the upstream BHC produces
  different match sets (the unresolved cross-language tree-topology
  nondeterminism from M9 #146 R1). The auto-adjust port is correct; the
  divergence is upstream.
  **Resolution**: re-`#[ignore]` `test_visual_database_matches_cpp_pipeline`
  with an updated docstring naming BHC tree-topology nondeterminism as the
  root cause. The Decision 9 soft-skip feature was removed (redundant
  alongside the unconditional `#[ignore]`).

- **R3 (DID NOT MATERIALIZE).** C++ `autoAdjustXYNumBins` is indeed
  `private` on the C++ class (confirmed during implementation), but the
  shim sidesteps the access issue cleanly by reimplementing the formula
  using public primitives (`vision::SafeDivision` + `vision::FastMedian`).
  No `friend` declaration needed; no patches to the third-party submodule.

## 6.5. Post-implementation summary

| What this PR delivers | Status |
|---|---|
| `fast_median_f32` + `partial_sort_f32` (bit-equivalent to C++) | ✅ |
| Diagnostic dual-mode test `dual_mode_partial_sort_f32_matches_cpp` (50 trials) | ✅ pass |
| `BinParams::new_auto_xy` factory + `set_xy_bins` atomic mutator | ✅ |
| Private `auto_adjust_xy` field + `num_x_bins`/`num_y_bins` getters | ✅ |
| `HoughSimilarityVoting::recompute_xy_bins_from_matches` | ✅ |
| `find_hough_similarity` invokes auto-adjust when flag is set | ✅ |
| Diagnostic dual-mode test `auto_adjust_xy_num_bins_matches_cpp` (40 trials) | ✅ pass |
| C++ FFI shims `webarkit_cpp_partial_sort_f32` + `webarkit_cpp_auto_adjust_xy_num_bins` | ✅ |
| `make_hough_voter` switched to auto-adjust path; old constants removed | ✅ |
| Close R1 dual-mode parity gate | ❌ — root cause is upstream BHC architectural nondeterminism (M9 #146 R1) |

The diagnosis from this PR is the most valuable outcome: by adding the
auto-adjust port and proving it byte-equivalent to C++ at the algorithm
level, we ruled it out as the cause of the residual gap. The remaining
divergence has been narrowed to the BHC tree-topology nondeterminism
that has persisted since M9-1 and is structural, not algorithmic.

---

## 7. Files modified (estimate)

| File | Change | Est. LOC |
|---|---|---|
| `crates/core/src/kpm/freak/math.rs` | +`fast_median_f32`, +`partial_sort_f32`, +4 unit tests, +1 dual-mode test | ~120 |
| `crates/core/src/kpm/freak/hough.rs` | BinParams visibility + `new_auto_xy` + `set_xy_bins`, +`auto_adjust_xy` field, HoughSimilarityVoting `recompute_xy_bins_from_matches`, `find_hough_similarity` invocation, +4 unit tests, +1 dual-mode test | ~180 |
| `crates/core/src/kpm/freak/visual_database.rs` | Remove HOUGH_NUM_X/Y_BINS constants, switch `make_hough_voter` to auto-adjust, un-`#[ignore]` parity test + add `cfg_attr` soft-skip | ~40 |
| `crates/core/src/kpm/kpm_c_api.h` | +2 declarations | ~15 |
| `crates/core/src/kpm/kpm_c_api.cpp` | +2 shim implementations | ~50 |
| `crates/core/Cargo.toml` | +`skip-parity-gate = []` feature | ~2 |
| **Total** | | **~407** |

---

## 8. Exit criteria

- [x] Understanding Lock confirmed by author 2026-05-20.
- [x] Final design approved across all four brainstorming sections + D10.
- [x] Decision log: D1–D10.
- [x] Assumptions: A1–A4.
- [x] Risks: R1–R3.
- [ ] Implementation handoff.
