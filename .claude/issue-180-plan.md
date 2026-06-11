# Issue #180 — Tighten CI Clippy Gate to `--all-targets --all-features`

**Status**: Planning complete, ready for implementation.
**Branch base**: `dev` (HEAD `a30b033` at planning time, post-PR #183 criterion 0.8 merge).
**Tracking issue**: [#180](https://github.com/webarkit/WebARKitLib-rs/issues/180).

---

## Understanding Summary

- **What**: Close the gap between CI's clippy gate (`--workspace -- -D warnings`, default features, lib targets only) and the strict local check (`--all-targets --all-features -- -D warnings`) — a ~70-lint delta on `dev` HEAD.
- **Why**: Contributor onboarding (new contributors run the strict check and assume the codebase is broken); the strict gate catches real lints in `ffi-backend` / `dual-mode` / `simd-*` paths CI never sees today.
- **Who**: WebARKitLib-rs contributors; no end-user behavior change.
- **How**: Execute issue #180's 5-PR sequence as the spec — trivial wins → FFI shim annotations → `ar/marker.rs` field-reassign cleanup → SIMD `#[allow]` with rationale → tighten CI in `.github/workflows/ci.yml`.
- **Non-goals**:
  - No `clippy.toml` file (deferred to M10+ if SIMD surface grows).
  - No `FeatureMapConfig` / struct refactor (SIMD signatures are locked by runtime-dispatch contract).
  - No deletion of `webarkit_cpp_*` FFI shims unless truly orphaned.
  - No `CHANGELOG.md` edits (project rule).
  - Not blocking v0.7.0 (already released).

---

## Decision Log

| # | Decision | Alternatives considered | Why this choice |
|---|---|---|---|
| 1 | Follow #180's 5-PR sequence verbatim | Refine ordering; merge PRs; re-baseline lint counts first | Issue is well-scoped; counts may drift but plan structure is sound |
| 2 | **(Updated during PR 2)** Tighten each extern block's cfg from `#[cfg(feature = "dual-mode")]` to `#[cfg(all(test, feature = "dual-mode"))]` — matches caller cfg, lint disappears because the extern only exists when callers do. Originally planned `#[allow(dead_code)]` + rationale. | Original plan (allow + rationale); wire up callers (scope creep); bulk delete (irreversible) | Evidence at PR 2 time: all 17 "dead" shims have callers, but the callers are gated `cfg(all(test, feature = "dual-mode"))` while the extern blocks were gated only on `cfg(feature = "dual-mode")`. The cfg mismatch was the real root cause; tightening fixes it honestly. Same effort per block (one line), zero behavior change. |
| 3 | PR 4: inline `#[allow(clippy::too_many_arguments)]` + rationale on `get_similarity_sse41` / `get_similarity_avx2` | `clippy.toml` threshold raise; bundle args into struct | SIMD signatures locked by runtime-dispatch contract; inline is scoped, threshold raise loses signal everywhere |
| 4 | Defer `clippy.toml` | Add now with `too-many-arguments-threshold = 9` | Only 2 sites today; revisit when SIMD surface grows in M10+ |
| 5 | Keep CI on `dtolnay/rust-toolchain@stable` after PR 5 | Pin to a specific rustc version; document known-good rustc | Accept the churn; v0.7.0 is shipped, future rustc lint additions fix in follow-up PRs |
| 6 | Branch off `dev`, one PR per branch | Stacked PRs; single mega-PR | Project convention (CLAUDE.md §4); keeps reviews focused |

---

## Assumptions

- **A1**: CI keeps `dtolnay/rust-toolchain@stable` after PR 5 tightens the gate. When a new rustc adds lints, fix in a follow-up PR. No rustc pin.
- **A2**: PR 4 stays its own PR (not folded into PR 1) for review clarity, even though it's only 2 `#[allow]` lines. Revisit at execution time if it feels artificial.
- **A3**: Branch naming follows `chore/clippy-180-pr<N>-<slug>` — branched off `dev`, one PR per branch, conventional commits per CLAUDE.md §6.
- **A4**: Re-baseline the lint inventory at the start of PR 1, not now. Counts may differ from #180's ~70 due to rustc drift since #180 was filed.

---

## Open Risks

- **R1** — *Rustc drift since #180*: lints may have been promoted/demoted between #180's filing and today. PR 1's scope could expand or contract. **Mitigation**: re-baseline before opening PR 1; update #180 with actual counts.
- **R2** — *FFI shim mislabeled*: a `webarkit_cpp_*` extern annotated as "kept for dual-mode parity" might actually be orphaned. **Mitigation**: run `cargo test --features dual-mode` and `cargo test --features ffi-backend` after PR 2; any annotated shim that has no caller under any feature combo gets deleted in the same PR.
- **R3** — *`ar/marker.rs` init-order dependency*: 44 mechanical `field-reassign-with-default` fixes touch many lines. Struct-init syntax could surface a subtle init-order dependency in `ARHandle` construction. **Mitigation**: rely on existing tests; if any fail, revert that specific site to the default+reassign pattern with an `#[allow]` + rationale.
- **R4** — *CI tightening fails after merge*: PR 5 flips the gate, then unrelated work introduces new lints under `--all-features` that the contributor didn't catch locally. **Mitigation**: PR 5 description includes a one-liner of the exact local command (`cargo clippy --all-targets --all-features -- -D warnings`); update CLAUDE.md §5 to match (already aligned) and add a §6 checklist note.

---

## Implementation Plan — 5 PRs

All PRs: branched off `dev`, opened against `dev`, conventional commits, pre-commit checklist (CLAUDE.md §5) green before push.

### PR 1 — Trivial wins (~10 lints)

**Branch**: `chore/clippy-180-pr1-trivial-wins`
**Scope**:
- Unused variables: `crates/core/src/ar/image_proc.rs:217` (`image_temp_u16`), `:218` (`image2`) — prefix `_` or remove.
- `items_after_test_module`: move items above `#[cfg(test)] mod tests` in `crates/core/src/ar/math.rs:831` and `ar/pattern.rs:1050`.
- `needless_deref`: `crates/core/src/kpm/freak/homography.rs:2645`.
- `needless_range_loop` (4 sites): convert to `iter().enumerate()` in `ar2/feature_map.rs`, `ar2/image_set.rs`, `ar/pattern.rs`.

**Verification**:
```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings  # the new lints here pass
cargo test --all-features
```

**Commit**: `fix(core): clean up trivial clippy lints under --all-targets --all-features (#180)`

---

### PR 2 — Dead FFI shim hygiene (~14 lints)

**Branch**: `chore/clippy-180-pr2-ffi-shims`
**Scope**: `crates/core/src/kpm/freak/{clustering,homography,math,hough,matcher}.rs` — for each dead `extern "C" { fn webarkit_cpp_*(...) }` block:

**Implemented (Decision #2 updated)**: change each block's cfg gate from `#[cfg(feature = "dual-mode")]` to `#[cfg(all(test, feature = "dual-mode"))]` so the extern only exists when its caller (also `cfg(all(test, feature = "dual-mode"))`) does. Add a one-line comment above each block referencing #180 for traceability. Verified all 17 callers still resolve and dual-mode tests still pass.

**Verification**: same triad as PR 1, plus `cargo test --features dual-mode --lib` and `cargo test --features ffi-backend` green.

**Commit**: `chore(kpm): document or remove dead webarkit_cpp_* FFI shims (#180)`

---

### PR 3 — `ar/marker.rs` field-reassign-with-default (44 lints)

**Branch**: `chore/clippy-180-pr3-ar-marker-init`
**Scope**: `crates/core/src/ar/marker.rs` — mechanical conversion of `let mut h = ARHandle::default(); h.field = x; h.other = y;` to struct-init syntax `let h = ARHandle { field: x, other: y, ..Default::default() };`.

**Process**:
- One commit per logical group (e.g. one per call site / constructor) to make review tractable.
- If any site fails tests after conversion, revert that specific site with `#[allow(clippy::field_reassign_with_default)]` + `// rationale: init order matters here because <reason>`.

**Verification**: same triad as PR 1, with extra attention to `cargo test --all-features` since `ARHandle` is core path.

**Commit**: `refactor(ar): convert ARHandle construction to struct-init syntax (#180)`

---

### PR 4 — SIMD `too_many_arguments` `#[allow]` (2 lints)

**Branch**: `chore/clippy-180-pr4-simd-too-many-args`
**Scope**: `crates/core/src/ar2/feature_map.rs:275` and `:387` — add to each:

```rust
#[allow(clippy::too_many_arguments)]
// rationale: SIMD variant of get_similarity; signature locked to match
// scalar fallback and sibling SIMD impl for runtime dispatch via
// is_x86_feature_detected!.
```

**Verification**: same triad as PR 1.

**Commit**: `chore(ar2): allow too_many_arguments on SIMD get_similarity variants (#180)`

---

### PR 5 — Tighten CI

**Branch**: `chore/clippy-180-pr5-ci-tighten`
**Pre-req**: PRs 1–4 merged; `cargo clippy --all-targets --all-features -- -D warnings` clean on `dev`.

**Scope**: `.github/workflows/ci.yml`
- `build-and-test` job: replace `cargo clippy --workspace -- -D warnings` with `cargo clippy --workspace --all-targets --all-features -- -D warnings`.
- `pure-rust-build` job: keep `cargo clippy -p webarkitlib-rs -- -D warnings` (this job intentionally verifies the no-features path stays clean — that's a different invariant).

**Verification**: open PR, watch CI go green. If a lint slips in between PR 4 merge and PR 5 (rustc drift, A1), fix in this PR.

**Commit**: `ci: tighten clippy gate to --all-targets --all-features (#180)`

---

## Closure

When PR 5 merges:
- Close issue #180.
- Note in the closing comment whether actual lint count matched #180's ~70 estimate (data point for future audits).
- Confirm CLAUDE.md §5 is still accurate (it already prescribes the strict command — no update needed).
