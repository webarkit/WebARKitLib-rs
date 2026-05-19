# Milestone 9 — Move BHC Feature Index from FeatureMatcher onto Keyframe

**Status**: Design approved, ready for implementation
**Branch**: `feat/keyframe-bhc-index` (PR target: `feat/freak-visual-database`)
**Parent milestone issue**: [#139](https://github.com/webarkit/WebARKitLib-rs/issues/139)
**Issue**: [#146](https://github.com/webarkit/WebARKitLib-rs/issues/146)
**Related**: [#140](https://github.com/webarkit/WebARKitLib-rs/issues/140) (M9-1 — R1 source), [#141](https://github.com/webarkit/WebARKitLib-rs/issues/141) (M9-2 — beneficiary)
**Author**: Walter Perdan ([@kalwalt](https://github.com/kalwalt))
**Date**: 2026-05-19

---

## 1. Understanding Summary

- **What**: Three coordinated changes that re-home the BHC feature index onto `Keyframe` and close the M9-1 dual-mode parity gate. (1) Data-model move: `Keyframe` owns an `Option<BinaryHierarchicalClustering>`. (2) BHC API gap: add `set_num_hypotheses` and `set_max_nodes_to_pop` setters (currently missing). (3) Algorithmic gap: implement `query`'s priority-queue traversal so `max_nodes_to_pop > 0` is honored (currently a no-op).
- **Why**: Close the R1 dual-mode parity gate from #140 (`test_visual_database_matches_cpp_pipeline` `#[ignore]`d at 441 vs 456 inliers, 3% drift). Pre-brainstorm investigation showed the gap is precisely the C++ `Keyframe<>::buildIndex` settings `(128, 8, 8, 16)` that the Rust port can't replicate today. Bonus: ~90 BHC builds/sec → 3 BHC builds total at 30 Hz tracking with 3 keyframes.
- **Who for**: Internal — M9-2 `RustFreakMatcher` will see a cleaner VisualDatabase. Library users gain a 1:1 Rust equivalent of C++ `Keyframe<>::buildIndex`.
- **Key constraints**: (a) Byte-equivalent BHC tree topology when configured with `(128, 8, 8, 16)`, verified by a dedicated dual-mode BHC test. (b) Zero behaviour change for callers using `FeatureMatcher::build` + `match_indexed` — those are deprecated, not removed. (c) `cargo clippy --all-targets --all-features -- --deny warnings` clean. (d) M9-1 dual-mode test un-`#[ignore]`d and passing within the new tighter tolerance (Decision 10).
- **Non-goals**: No SIMD/rayon. No public `BhcConfig` struct. No tuning hooks beyond the four C++ setter equivalents. No reworking of `find_features` / `Keyframe::new` / `RustFreakMatcher` (M9-2). No follow-on auto-adjust work (separate issue if R1 still doesn't close).

---

## 2. Pre-brainstorm finding (the framing that justified the full scope)

The Rust BHC defaults match the C++ default constructor **exactly**:
- C++ `BinaryHierarchicalClustering()`: `setk(8)`, `setNumHypotheses(1)`, `mMaxNodesToPop(0)`, `mMinFeaturePerNode(16)` ([binary_hierarchical_clustering.h:317-327](../../crates/core/third_party/WebARKitLib/lib/SRC/KPM/FreakMatcher/matchers/binary_hierarchical_clustering.h))
- Rust `BinaryHierarchicalClustering::new`: `KMedoids::new(8, 1)`, `num_centers=8`, `_max_nodes_to_pop=0`, `min_features_per_leaf=16` ([clustering.rs:319-333](../../crates/core/src/kpm/freak/clustering.rs))

The C++ `Keyframe<>::buildIndex` **overrides** these defaults to `(128, 8, 8, 16)`:

```cpp
mIndex.setNumHypotheses(128);
mIndex.setNumCenters(8);
mIndex.setMaxNodesToPop(8);
mIndex.setMinFeaturesPerNode(16);
```

Two of those setters do not exist in the Rust port. `_max_nodes_to_pop` exists as a private field with a leading underscore (unused). The query path doesn't honor it either ([clustering.rs:488-496](../../crates/core/src/kpm/freak/clustering.rs)):

```rust
// Other children are pushed to a priority queue
// (only popped if mMaxNodesToPop > 0; default is 0, so unused here).
```

This is why the M9-1 dual-mode parity test drifts by ~3%: the C++ pipeline uses `Keyframe::mIndex` configured with `(128, 8, 8, 16)`; the Rust pipeline rebuilds the matcher's index per-iteration with `(1, 8, 0, 16)`. Different K-medoids initialisation count → different tree topology → different candidate sets → different match pairs. Closing the gap requires all three pieces of work; doing only the data-model move (as #146's original body suggested) wouldn't help.

---

## 3. Decision Log

| # | Decision | Alternatives considered | Rationale |
|---|----------|-------------------------|-----------|
| 1 | **Full scope: data-model + setters + traversal in one PR** | Narrow (just data-model move); wide minus traversal; split across two PRs | Only this combination plausibly closes R1; partial fixes leave the parity gate open. ~250–400 LOC + tests is manageable in one review. |
| 2 | **Add `FeatureMatcher::match_with_index(query, ref, &BHC)`; deprecate `match_indexed` and `build`; keep `index` field for back-compat** | Remove old completely; keep both forever with no deprecation | Smooth migration path; existing matcher dual-mode tests stay green; M9-2 RustFreakMatcher uses the new API exclusively. Cleanup of deprecated paths can be a later follow-up. |
| 3 | **`VisualDatabase::add_keyframe(Keyframe)` builds index iff `index.is_none()`** | Always rebuild; require pre-built; never build | Mirrors C++ facade `addFreakFeaturesAndDescriptors` spirit; ergonomic; respects caller intent without giving them a foot-gun. |
| 4 | **Priority-queue traversal via `std::collections::BinaryHeap<Reverse<(distance, &BhcNode)>>`** | Manual Vec + sort; hand-port C++ heap byte-equivalent | Stdlib, no deps, observable-equivalent to `std::priority_queue`. Tie-break ordering verified by Decision 6's dual-mode test; raw-pointer fallback documented if borrow checker objects. |
| 5 | **`Keyframe::build_index()` parameterless, hardcodes `(128, 8, 8, 16)`** | Parameterised; parameterless + `build_index_with(BhcConfig)` | 1:1 with C++ `Keyframe<>::buildIndex`; tuning is YAGNI (no current caller needs it). |
| 6 | **Dedicated dual-mode BHC test with `(128, 8, 8, 16)` via new FFI shim `webarkit_cpp_bhc_build_and_query_with_settings`** | Reuse existing matcher dual-mode tests; parametrize existing tests | Isolates root cause: if VisualDatabase parity still drifts after this PR, we know the gap is downstream (hough, find_inliers) not BHC. Worth ~50 LOC of shim. |
| 7 | **No new SIMD / rayon / benchmarks in this PR** | Add microbench; parallelise the 128 hypothesis runs | Scope discipline. Parallelisation would also break dual-mode parity (M7-3's `ArrayShuffle` makes hypothesis order observable). |
| 8 | **`#[allow(deprecated)]` the existing matcher.rs dual-mode tests; add a one-line comment pointing to the new BHC test** | Port to `match_with_index`; remove | Deprecated API stays under test (we keep it for back-compat); new API gets coverage from Decision 6's test. Minimal change, no orphaned code paths. |
| 9 | **M9-1 `VisualDatabase::try_match_one` rewrites to inline `matcher.match_with_index(query.store, ref.store, ref_kf.index().expect(...))`** | Add a wrapper `match_features` helper | No internal deprecated calls; removes the borrow-checker dance of holding `self.keyframes.get(&ref_id)` across the deprecated `matcher.build(...)` call. |
| 10 | **Dual-mode tolerance: `max(2, observed_diff)`** | Lock to exact match; leave at ±5 unconditionally | Tighten if we measure diff ≤ 2 (expected: 0 or 1); leave at ±5 if diff ≥ 4; investigate before merging if diff ≥ 6. Honours "tighten but don't exaggerate." |

---

## 4. Final Design

### 4.1 Files touched

```
crates/core/src/kpm/freak/
├── clustering.rs       ← +2 setters; rework query() traversal for max_nodes_to_pop
├── keyframe.rs         ← +`index` field, +`build_index()`, +`index()` accessor
├── matcher.rs          ← +`match_with_index`; #[deprecated] on `build` + `match_indexed`
└── visual_database.rs  ← call `keyframe.build_index()` in add_image/add_keyframe;
                           rewrite try_match_one; un-#[ignore] dual-mode test

crates/core/src/kpm/
├── kpm_c_api.h         ← declare `webarkit_cpp_bhc_build_and_query_with_settings`
└── kpm_c_api.cpp       ← implement it
```

### 4.2 `Keyframe` (`keyframe.rs`)

```rust
pub struct Keyframe {
    pub store: FeatureStore,
    pub width: i32,
    pub height: i32,
    // NEW (M9 #146): lazy BHC index, None until build_index() is called.
    index: Option<BinaryHierarchicalClustering>,
}

impl Keyframe {
    /// Build the BHC index with C++ Keyframe<>::buildIndex defaults (128/8/8/16).
    /// Idempotent: replaces any previous index.
    /// C equivalent: `Keyframe<NUM_BYTES_PER_FEATURE>::buildIndex`
    /// (keyframe.h:116-122).
    pub fn build_index(&mut self) -> Result<(), KpmError>;

    /// Borrow the BHC index. Returns None until build_index() is called.
    pub fn index(&self) -> Option<&BinaryHierarchicalClustering>;
}
```

Constants for the C++ defaults live inside `build_index` (not at module scope) — they're an implementation detail of the C++ parity contract, not a public configuration.

### 4.3 `BinaryHierarchicalClustering` (`clustering.rs`)

```rust
impl BinaryHierarchicalClustering {
    /// Number of K-medoids hypothesis runs per split.
    /// Default: 1 (default-ctor); Keyframe::build_index uses 128.
    /// C equivalent: `setNumHypotheses(int)`.
    pub fn set_num_hypotheses(&mut self, n: usize) -> Result<(), KpmError>;

    /// Maximum number of non-tied children to pop from the priority queue
    /// per query, in addition to tied-minimum children.
    /// Default: 0 (default-ctor); Keyframe::build_index uses 8.
    /// C equivalent: `setMaxNodesToPop(int)`.
    pub fn set_max_nodes_to_pop(&mut self, n: usize);
}
```

`set_num_hypotheses` recreates the internal `KMedoids` (which takes `num_hypotheses` in its constructor). `set_max_nodes_to_pop` is a plain field write — the field is renamed from `_max_nodes_to_pop` to `max_nodes_to_pop` since it's no longer unused.

### 4.4 BHC query traversal — the algorithmic core

Two phases mirroring C++:

**Phase 1** (recursive, current behaviour + push non-tied to backlog):

```rust
fn query_recursive(
    &self,
    node: &BhcNode,
    query_feature: &[u8; 96],
    result: &mut Vec<usize>,
    backlog: &mut BinaryHeap<Reverse<(u32, &BhcNode)>>,
) -> Result<(), KpmError> {
    // ... existing tied-minimum logic ...
    for (i, &d) in dists.iter().enumerate() {
        if d == min_dist {
            self.query_recursive(&node.children[i], query_feature, result, backlog)?;
        } else {
            // CHANGED: non-tied children go to backlog instead of being dropped.
            backlog.push(Reverse((d, &node.children[i])));
        }
    }
    Ok(())
}
```

**Phase 2** (drain up to `max_nodes_to_pop` from backlog):

```rust
pub fn query(&self, query_feature: &[u8; 96]) -> Result<Vec<usize>, KpmError> {
    let root = self.root.as_ref().ok_or_else(|| { ... })?;
    let mut result = Vec::new();
    let mut backlog: BinaryHeap<Reverse<(u32, &BhcNode)>> = BinaryHeap::new();
    let mut nodes_popped = 0usize;

    self.query_recursive(root, query_feature, &mut result, &mut backlog)?;

    while nodes_popped < self.max_nodes_to_pop {
        match backlog.pop() {
            Some(Reverse((_d, node))) => {
                self.query_recursive(node, query_feature, &mut result, &mut backlog)?;
                nodes_popped += 1;
            }
            None => break,
        }
    }
    Ok(result)
}
```

**Lifetime note.** Both `query` and `query_recursive` take `&self`; the heap holds `&BhcNode` references tied to `self.root`'s lifetime. Should the borrow checker object, fallback is `*const BhcNode` with a `// SAFETY:` comment (the heap never outlives the call). Try safe first.

**Tie-break note.** If `BinaryHeap` and `std::priority_queue` diverge on equal-distance pops in implementation-defined ways, the dual-mode BHC test (Decision 6) will catch it. Mitigation: switch heap key to `(distance, insertion_seq)`.

### 4.5 `FeatureMatcher` (`matcher.rs`)

```rust
impl FeatureMatcher {
    /// BHC-indexed match using a caller-supplied (typically Keyframe-owned) index.
    /// Mirrors C++ `BinaryFeatureMatcher::match(features1, features2, index2)`.
    pub fn match_with_index(
        &mut self,
        query: &FeatureStore,
        reference: &FeatureStore,
        index: &BinaryHierarchicalClustering,
    ) -> Result<usize, KpmError>;

    #[deprecated(note = "Use match_with_index with a Keyframe-owned BHC. See docs/design/m9-keyframe-bhc-index.md.")]
    pub fn build(&mut self, store: &FeatureStore) -> Result<(), KpmError>;

    #[deprecated(note = "Use match_with_index with a Keyframe-owned BHC.")]
    pub fn match_indexed(&mut self, query: &FeatureStore, reference: &FeatureStore)
        -> Result<usize, KpmError>;
}
```

The body of `match_with_index` is a near-copy of `match_indexed`'s loop, but reads candidates from the supplied `index` rather than `self.index`. `self.index` stays for back-compat.

### 4.6 `VisualDatabase` (`visual_database.rs`)

- `add_image`: after `find_features`, call `keyframe.build_index()` (mirrors C++ `visual_database-inline.h:128-131`).
- `add_keyframe`: if `keyframe.index().is_none()`, call `keyframe.build_index()` (Decision 3).
- `try_match_one`: Pass 1 inlines `match_with_index(query.store, ref.store, ref_kf.index().expect(...))`. The `match_features` helper goes away. If `use_feature_index = false`, falls back to `match_all`.
- Dual-mode test: remove `#[ignore]` annotation; tighten tolerance per Decision 10.

### 4.7 C++ FFI shim

```c
// kpm_c_api.h
int webarkit_cpp_bhc_build_and_query_with_settings(
    const unsigned char* features, int num_features,
    int num_hypotheses, int num_centers, int max_nodes_to_pop, int min_features_per_node,
    const unsigned char* query_feat, int* out_indices);
```

C++ implementation builds a `vision::BinaryHierarchicalClustering<96>` with the supplied settings, queries it once, copies indices to the output buffer. ~25 LOC.

---

## 5. Tests

### 5.1 New unit tests (Rust)

- `clustering.rs`:
  - `test_bhc_set_num_hypotheses_propagates_to_kmedoids`
  - `test_bhc_set_max_nodes_to_pop_zero_is_default_behavior` — monotonicity vs pre-change
  - `test_bhc_set_max_nodes_to_pop_widens_candidate_set` — N=8 returns ≥ N=0
- `keyframe.rs`:
  - `test_keyframe_index_is_none_before_build`
  - `test_keyframe_build_index_populates_index`
  - `test_keyframe_build_index_is_idempotent`
- `matcher.rs`:
  - `test_match_with_index_finds_identical` — mirror of `test_match_indexed_finds_identical`
- `visual_database.rs`:
  - `test_visual_database_add_image_builds_index`
  - `test_visual_database_add_keyframe_builds_index_when_absent`
  - `test_visual_database_add_keyframe_preserves_caller_built_index`

### 5.2 Dual-mode tests

- **New** in `clustering.rs` (Decision 6): `bhc_with_keyframe_settings_matches_cpp` — exercises `(128, 8, 8, 16)` end-to-end against new FFI shim.
- **Updated** in `matcher.rs`: existing 3 `dual_mode_match_*_within_tolerance` tests get `#[allow(deprecated)]` + a one-line comment (Decision 8).
- **Un-`#[ignore]`d** in `visual_database.rs`: `test_visual_database_matches_cpp_pipeline` — tolerance updated per Decision 10. **Closing this is the success criterion for the PR.**

### 5.3 Acceptance command sequence (CLAUDE.md §5)

```
cargo fmt --all -- --check
cargo build --all-features
cargo clippy --all-targets --all-features -- --deny warnings
cargo test --all-features
cargo test -p webarkitlib-rs -- kpm::freak::clustering
cargo test -p webarkitlib-rs -- kpm::freak::keyframe
cargo test -p webarkitlib-rs -- kpm::freak::matcher
cargo test -p webarkitlib-rs -- kpm::freak::visual_database
```

All five steps clean; un-`#[ignore]`d dual-mode test passes.

---

## 6. Assumptions

- **A1**. Adding `index: Option<BinaryHierarchicalClustering>` to `Keyframe` will not break any existing caller, because no current code Clones `Keyframe`. Verify with `grep` during implementation; if Clone is needed, derive Clone on `BinaryHierarchicalClustering` (small follow-on change).
- **A2**. The C++ BHC priority-queue traversal uses `std::priority_queue` keyed by node-distance, popping at most `mMaxNodesToPop` nodes after tied-minimum children. The Rust `BinaryHeap<Reverse<...>>` will be observable-equivalent. Tie-break behaviour on equal distances is implementation-defined in both; the dual-mode BHC test verifies.
- **A3**. The R1 dual-mode parity gate will pass after this PR. If it still fails by > 5 inliers, the remaining suspect is `HoughSimilarityVoting::autoAdjustXYNumBins` (a separate, narrower follow-up).
- **A4**. The existing matcher.rs dual-mode tests pass with default BHC settings `(1, 0)` because the C++ FFI shim also uses default-constructed BHC. They'll continue to pass after deprecation; only the warning suppression annotation changes.

---

## 7. Risks (post-implementation status)

- **R1 (MATERIALIZED, partially).** Priority-queue tie-break ordering diverged
  from C++. The dual-mode BHC test (`bhc_with_keyframe_settings_matches_cpp`)
  reveals deeper architectural nondeterminism: **both** Rust and C++ use
  unordered-key maps for the BHC `cluster_map` during build
  (C++ `std::unordered_map<int, std::vector<int>>` at
  `binary_hierarchical_clustering.h:217`; Rust originally `HashMap`, switched
  to `BTreeMap` for intra-Rust determinism). The hash orderings differ
  between toolchains, and the cluster keys themselves differ (C++ keys by
  feature-array index; Rust keys by cluster position 0..k-1). Result: child
  ordering in the BHC tree diverges across languages even when K-medoids
  produces equivalent partitions. The BHC algorithm tolerates this (priority
  queue handles ties), so algorithmic correctness is unaffected.
  **Resolution**: the BHC dual-mode test is `#[ignore]`d with a detailed
  diagnostic docstring. The seq-based `BacklogEntry::cmp` tie-break
  implementation is retained — it's a structural improvement regardless.
- **R2 (DID NOT MATERIALIZE).** `&BhcNode` lifetime in `BinaryHeap` was
  accepted cleanly by the borrow checker; no `*const BhcNode` fallback
  needed. The safe `'tree` lifetime annotation on `query_recursive` works.
- **R3 (MATERIALIZED).** Dual-mode parity gate
  (`test_visual_database_matches_cpp_pipeline`) still produces `diff = 15`
  inliers, identical to the M9-1 baseline. The BHC settings change
  (defaults `(1, 0)` → Keyframe::buildIndex `(128, 8)`) and the
  priority-queue traversal are now algorithmically correct, but on this
  specific test pair the downstream pipeline (Hough voting → RANSAC →
  inlier filter) absorbs the BHC-level differences. The remaining gap
  points at the unported `HoughSimilarityVoting::autoAdjustXYNumBins`
  (anticipated in design doc Assumption A3). **Resolution**: re-`#[ignore]`
  the test with an updated docstring; file a follow-up issue for
  `autoAdjustXYNumBins`. The BHC work shipped in this PR remains valuable:
  build-once architecture (~90 BHC builds/sec → 3), correct
  `max_nodes_to_pop` traversal (previously a no-op), data-model alignment
  with C++.
- **R4 (DID NOT MATERIALIZE).** Deprecation warnings cleanly contained.
  Three test functions in `matcher.rs` plus one dual-mode test got
  `#[allow(deprecated)]`. No production code paths emit deprecation
  warnings; the internal `try_match_one` was rewritten to use
  `match_with_index`.
- **R5 (DID NOT MATERIALIZE).** A1 verified: no callers Clone `Keyframe`,
  no `#[derive]` attributes on the struct. Adding `index: Option<BHC>`
  was a clean addition.

## 7.5. Post-implementation summary

| What this PR delivers | Status |
|---|---|
| `BinaryHierarchicalClustering::set_num_hypotheses` setter | ✅ |
| `BinaryHierarchicalClustering::set_max_nodes_to_pop` setter | ✅ |
| Priority-queue traversal honoring `max_nodes_to_pop` (was a no-op before) | ✅ |
| `Keyframe::build_index()` with C++ buildIndex defaults `(128, 8, 8, 16)` | ✅ |
| `Keyframe::index()` accessor | ✅ |
| `FeatureMatcher::match_with_index(...)` borrowing API | ✅ |
| `FeatureMatcher::build` / `match_indexed` deprecated (kept for back-compat) | ✅ |
| `VisualDatabase::add_image` builds index at insertion | ✅ |
| `VisualDatabase::add_keyframe` builds index if absent | ✅ |
| `VisualDatabase::try_match_one` uses Keyframe-owned index (no per-query rebuild) | ✅ |
| Perf win: BHC built once at insertion, not ~90× per second at 30 Hz | ✅ |
| Close R1 dual-mode parity gate | ❌ — deferred to `autoAdjustXYNumBins` follow-up (R3 above) |

---

## 8. Files modified (estimate)

| File | Change | Est. LOC |
|---|---|---|
| `crates/core/src/kpm/freak/clustering.rs` | +2 setters, rework query traversal, +3 unit tests, +1 dual-mode test | ~140 |
| `crates/core/src/kpm/freak/keyframe.rs` | +`index` field, +`build_index`, +`index()`, +3 tests | ~70 |
| `crates/core/src/kpm/freak/matcher.rs` | +`match_with_index`, +deprecations, +1 test, `#[allow(deprecated)]` on 3 existing tests | ~80 |
| `crates/core/src/kpm/freak/visual_database.rs` | call `build_index` in add_image/add_keyframe, rewrite try_match_one, +3 tests, un-`#[ignore]` dual-mode | ~60 |
| `crates/core/src/kpm/kpm_c_api.h` | +1 declaration | ~10 |
| `crates/core/src/kpm/kpm_c_api.cpp` | +1 function (~25 LOC) | ~25 |
| **Total** | | **~385** |

---

## 9. Exit criteria

- [x] Understanding Lock confirmed by author 2026-05-19.
- [x] Final design approved (sections 1–5 of brainstorming).
- [x] Decision log: D1–D10.
- [x] Assumptions: A1–A4.
- [x] Risks: R1–R5.
- [ ] Implementation handoff.
