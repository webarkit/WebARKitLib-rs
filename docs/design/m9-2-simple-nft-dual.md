# Milestone 9 — Step 2 follow-up: `simple_nft_dual.rs` diagnostic example

**Status**: Design approved, ready for implementation
**Branch**: `feat/m9-2-simple-nft-dual` (PR target: `feat/freak-visual-database`)
**Parent milestone issue**: [#139](https://github.com/webarkit/WebARKitLib-rs/issues/139)
**Issue**: [#157](https://github.com/webarkit/WebARKitLib-rs/issues/157)
**Depends on**: [#141](https://github.com/webarkit/WebARKitLib-rs/issues/141) / [#156](https://github.com/webarkit/WebARKitLib-rs/pull/156) (M9-2 — `DualFreakMatcher` and concrete `matched_geometry()` accessors) — merged into `feat/freak-visual-database`
**Unblocks (informational)**: contributors verifying M9-2 design doc §10 numbers locally before [#142](https://github.com/webarkit/WebARKitLib-rs/issues/142) (M9-3) flips the default off `ffi-backend`
**Author**: Walter Perdan ([@kalwalt](https://github.com/kalwalt))
**Date**: 2026-05-23

---

## 1. Understanding Summary

- **What**: Create `crates/core/examples/simple_nft_dual.rs` — a diagnostic sibling of `simple_nft.rs` that exercises `DualFreakMatcher` on the pinball reference image and reports per-query divergence + side-by-side homographies. Add two tiny public accessors on `DualFreakMatcher` so the example can read each backend's matched homography.
- **Why**: Final end-to-end integration signal before M9-3 (#142) flips the default backend off `ffi-backend`. While both backends are still routinely available, this example lets contributors reproduce the M9-2 design doc §10 parity numbers locally and catch any late-arriving cross-backend regression.
- **Who for**: WebARKitLib-rs contributors debugging Rust-backend regressions; reviewers cross-checking M9-2 §10. After M9-3 lands, it becomes a niche debugging tool.
- **Key constraints**:
  - Compiles only under `--features dual-mode` (transitively pulls `ffi-backend`).
  - Logging via `arlog_*!` (per `CLAUDE.md` §2) — no `println!` in the new file.
  - No new tests (M9-2 already has `test_dual_mode_no_divergence_on_pinball`); no CI integration; no C-FFI surface changes.
  - Branch off `origin/feat/freak-visual-database`; PR back into `feat/freak-visual-database`.
- **Non-goals**:
  - No conversion of the existing `simple_nft.rs` to `arlog_*!` (separate scope — [#90](https://github.com/webarkit/WebARKitLib-rs/issues/90) PR 4).
  - No multi-frame iteration loop (single query — mirror `simple_nft.rs`).
  - No `max_corner_displacement` promotion to public API (only one caller).
  - No downcast helper on `FreakMatcherBackend` trait.

---

## 2. Decision Log

| # | Decision | Alternatives | Rationale |
|---|----------|--------------|-----------|
| D1 | **Two-phase structure**: Phase A drives `DualFreakMatcher` directly for diagnostics; Phase B uses a fresh `KpmHandle` + `CppFreakMatcher` for the production pose-estimation + AR2 pipeline | (a) Move dual into KpmHandle — loses post-move access since `FreakMatcherBackend` lacks `Any` bound and inner state is unreachable through `Box<dyn>`; (b) add `as_any` to trait | No API surface change beyond the two accessors in D4. Cleanest "diagnostic vs production" mental model for a reader. C++ ground-truth pose feeding AR2 is identical either way (`DualFreakMatcher::query` returns C++ result per M9-2 D5). |
| D2 | **Output**: print both 3×3 homographies side-by-side + max corner displacement + one 3×4 KPM pose (C++-derived) + AR2 refined pose | (a) Compute two 3×4 poses via `kpm_util_get_pose_binary` for both backends — adds ~50 LOC + couples example to internals; (b) divergence-summary only — too minimal | Strictly uses `matched_geometry()` (the API the issue references) and reports the M9 #152 tier-2 metric. Keeps the example ~80 LOC. |
| D3 | **Single query iteration** (mirror `simple_nft.rs`) | 3 iterations (mirror M9-2 milestone-gate test) | Contributor reading both files side-by-side sees a near-symbol-level diff; "example shape ≠ test shape" stays clean. Multi-iteration coverage already lives in `test_dual_mode_no_divergence_on_pinball`. |
| D4 | **Two tiny accessors on `DualFreakMatcher`**: `cpp_matched_geometry()` and `rust_matched_geometry()`, each returning `Option<&[f32; 9]>` | (a) Three-matcher pattern (DualFreakMatcher + standalone CppFreakMatcher + standalone RustFreakMatcher) — triples setup + runs three queries; (b) also promote `max_corner_displacement` to public | Smallest delta. Not an FFI change. Rust-only, already `#[cfg(feature = "dual-mode")]`-gated. `max_corner_displacement` stays private — example reimplements ~10 LOC inline (identical algorithm). |
| D5 | **Branch off `origin/feat/freak-visual-database`, PR back into same** | Branch off `dev` | DualFreakMatcher only exists on the M9 integration branch. Matches the M9 PR pattern (#140, #141, etc.). |
| D6 | **Use `arlog_*!` macros from day one** in the new file: `arlog_i!` for narrative output, `arlog_e!` for error sites, call `ar_log_init_default()` at startup | `println!` (matches existing `simple_nft.rs` style); mix | Per CLAUDE.md §2 — all new code uses the project log macros. Issue [#90](https://github.com/webarkit/WebARKitLib-rs/issues/90) PR 4 will eventually retrofit `simple_nft.rs`; we don't preempt that scope. |
| D7 | **Don't touch `simple_nft.rs`** | Combined PR converting it to `arlog_*!` (closes part of #90) too | Strict CLAUDE.md "fresh branch per issue/sub-issue" rule. The two issues are logically related but procedurally distinct. |
| D8 | **`Cargo.toml` `[[example]]` entry: `required-features = ["dual-mode", "log-helpers"]`** | `["dual-mode"]` only (the issue's literal spec); add `log-helpers` to `dual-mode`'s feature chain | `dual-mode` for `DualFreakMatcher`; `log-helpers` for the `ar_log_init_default()` logger init so the arlog macros emit. The issue's literal spec was written assuming `println!`-based output; with arlog we need both. Adding `log-helpers` to `dual-mode`'s chain would force every `dual-mode` consumer to pull `env_logger`/`console_log` — wrong scope. |
| D9 | **Phase A loads ref data manually**: replicates `KpmHandle::set_ref_data_set`'s feature-feeding loop inline (~30 LOC) for the `DualFreakMatcher` | (a) Factor a shared helper into the library; (b) use `add_image` with a raw reference image extracted from the `.iset` surface set | (a) is library-API scope creep for a one-off example. (b) wouldn't be apples-to-apples with `simple_nft.rs`'s `.fset3`-driven path. Inline loop is honest about what `set_ref_data_set` does and the duplication is contained to one file. |
| D10 | **Phase B uses `CppFreakMatcher` inside `KpmHandle`**, not `DualFreakMatcher` | Move the same `DualFreakMatcher` from Phase A into `KpmHandle` | Can't — `Box<dyn FreakMatcherBackend>` is one-way (no downcast). Re-creating a fresh matcher for Phase B is the cost of D1. Two queries on a static image is negligible. `CppFreakMatcher` produces identical ground-truth pose to `DualFreakMatcher::query` (per M9-2 D5). |

---

## 3. Final Design

### 3.1 File layout

| File | Change |
|---|---|
| `crates/core/src/kpm/rust_backend.rs` | **Modify** — add `cpp_matched_geometry()` and `rust_matched_geometry()` to `impl DualFreakMatcher`. Both `#[cfg(feature = "dual-mode")]`-gated (the entire `impl DualFreakMatcher` block already is). |
| `crates/core/examples/simple_nft_dual.rs` | **Create** — new diagnostic example. LGPL-3.0 header per `.claude/HEADER.txt`. |
| `crates/core/Cargo.toml` | **Modify** — add `[[example]]` entry with `required-features = ["dual-mode", "log-helpers"]`. |

No other files touched. No CHANGELOG.md edit (release-only per CLAUDE.md §4).

### 3.2 `DualFreakMatcher` accessor additions

```rust
#[cfg(feature = "dual-mode")]
impl DualFreakMatcher {
    // ... existing items unchanged ...

    /// Homography from the most recent `query` as observed by the C++
    /// backend, or `None` if no query has matched yet. Used by the
    /// `simple_nft_dual` example to print backend-by-backend geometry.
    pub fn cpp_matched_geometry(&self) -> Option<&[f32; 9]> {
        self.cpp.matched_geometry()
    }

    /// Homography from the most recent `query` as observed by the
    /// pure-Rust backend, or `None` if no query has matched yet.
    pub fn rust_matched_geometry(&self) -> Option<&[f32; 9]> {
        self.rust.matched_geometry()
    }
}
```

Both delegate to the existing concrete-impl accessors. Zero new state; zero behaviour change for current callers.

### 3.3 Example structure

Module docstring opens with:

> Diagnostic sibling of `simple_nft.rs`. Drives `DualFreakMatcher` instead of `RustFreakMatcher` to compare C++ vs pure-Rust homographies on a static image. For production use, see `simple_nft.rs`.

Flow:

```text
[init logger via ar_log_init_default()]

Step 1: Load camera_para.dat              -> ARParam (scaled to image)
Step 2: Load pinball-demo.jpg as luma     -> Vec<u8>
Step 3: Load pinball.fset3                -> KpmRefDataSet

Phase A — DualFreakMatcher diagnostic
─────────────────────────────────────
Step 4a: DualFreakMatcher::new(w, h)
Step 4b: For each page/image in ref_data:
            dual.add_freak_features(...)   [inline loop, ~30 LOC]
Step 4c: dual.query(&luma)
Step 4d: arlog_i! output:
            - divergence_count
            - last_divergence_reason (if any)
            - cpp_matched_geometry (3×3 H)
            - rust_matched_geometry (3×3 H)
            - max corner displacement (computed inline against
              the reference image dimensions from ref_data.page_info[0])

Phase B — Production pipeline (mirror simple_nft.rs)
───────────────────────────────────────────────────
Step 5a: Reload .fset3 + .iset/.fset surface set
Step 5b: CppFreakMatcher::new(w, h)
Step 5c: KpmHandle::new(...) with cpp backend
Step 5d: kpm_handle.set_ref_data_set(ref_data)
Step 5e: kpm_handle.kpm_matching(&luma)
Step 5f: kpm_handle.get_pose() -> 3×4 pose
            arlog_i! the pose
Step 5g: surface_set.set_init_trans(cam_pose); AR2 tracking
            arlog_i! the refined pose
```

### 3.4 Inline `max_corner_displacement` (D4)

```rust
fn project(h: &[f32; 9], x: f32, y: f32) -> (f32, f32) {
    let w = h[6] * x + h[7] * y + h[8];
    ((h[0] * x + h[1] * y + h[2]) / w, (h[3] * x + h[4] * y + h[5]) / w)
}

fn max_corner_displacement(cpp_h: &[f32; 9], rust_h: &[f32; 9], rw: f32, rh: f32) -> f32 {
    let corners = [(0.0, 0.0), (rw, 0.0), (rw, rh), (0.0, rh)];
    corners
        .iter()
        .map(|&(x, y)| {
            let (cx, cy) = project(cpp_h, x, y);
            let (rx, ry) = project(rust_h, x, y);
            ((cx - rx).powi(2) + (cy - ry).powi(2)).sqrt()
        })
        .fold(0.0_f32, f32::max)
}
```

Mirrors `DualFreakMatcher::reproject_corners` + `max_corner_displacement` (which stay private per D4). Reference dimensions sourced from `ref_data.page_info[0].image_info[0].{width,height}`.

### 3.5 `Cargo.toml` entry

```toml
[[example]]
name = "simple_nft_dual"
# M9-2 #157: diagnostic sibling of simple_nft, drives DualFreakMatcher.
# - dual-mode  -> DualFreakMatcher (transitively pulls ffi-backend)
# - log-helpers -> ar_log_init_default() so arlog_* macros emit output
required-features = ["dual-mode", "log-helpers"]
```

### 3.6 Run command (run-book in module docstring)

```sh
# With required-features declared, cargo auto-enables them:
cargo run -p webarkitlib-rs --example simple_nft_dual

# Explicit form:
cargo run -p webarkitlib-rs --features "dual-mode log-helpers" --example simple_nft_dual
```

### 3.7 Observed output (measured during implementation)

On `pinball-demo.jpg`:

- **Tier-1 (matched_id) divergence**: none — both backends agree on `matched_id = 2` (one of 9 internal db_ids spanning the page's image-scale variants).
- **Tier-2 (corner reprojection) divergence**: ~13.80 px on db_id=2 with reference dimensions 595×745. This exceeds the 2.0 px tolerance, so `divergence_count = 1` after a single query and `last_divergence_reason` populates with the reprojection message.
- C++ KPM error = **7.1455** and C++ 3×4 pose row 0 = `[0.9862, 0.1671, 0.0641, -182.1635]` — matches §10 of `m9-2-rust-backend.md` exactly.
- The C++ pose feeds AR2 cleanly (per M9-2 D5); AR2 returns the usual single-static-frame error code, identical to `simple_nft.rs` behavior.
- The example's internally-computed corner displacement matches `DualFreakMatcher`'s internal tier-2 number to four decimals (we look up the matched id's ref dims via the `Vec<(i32, i32)>` returned from `feed_ref_data`, exactly mirroring `DualFreakMatcher::ref_dims`).

**Interpretation correction vs. the original issue**: Issue #157 anticipated "zero divergences, two near-identical poses, max corner displacement < 2.0 px" on pinball. That expectation was extrapolated from §10's `simple_nft` measurement (which never used `DualFreakMatcher` — it ran the two backends through `KpmHandle` separately). The actual `DualFreakMatcher` measurement reported here is what the milestone-gate test asserts for the `found.jpg`/`img.jpg` pair (zero divergence) but **not** for pinball. The pinball divergence is the cross-language BHC-variance envelope §10 already discusses — sub-degree rotation and sub-percent translation differences, but on the corner-reprojection metric they manifest as ~14 px because the matched scale (db_id=2) is at a smaller reference resolution where any rotation/translation difference projects to more pixels.

This is informational, not a regression — the example surfaces a real signal that the milestone-gate test's fixture choice happens to miss.

### 3.8 Risks acknowledged

| Risk | Mitigation |
|---|---|
| AR2 tracking may fail on single frame (already documented in `simple_nft.rs`) | Same fallback message style; KPM pose still printed |
| Float-noise reproducibility across machines may make exact §10 numbers differ slightly | Module docstring explicitly says "or very close to them, modulo nondeterministic float behavior" |
| `log-helpers` feature requirement is a tiny departure from issue's literal spec | Documented in D8; the issue's spec predates the arlog decision |

---

## 4. Implementation Checklist

- [ ] `git fetch origin && git checkout -b feat/m9-2-simple-nft-dual origin/feat/freak-visual-database`
- [ ] Add `cpp_matched_geometry()` + `rust_matched_geometry()` to `impl DualFreakMatcher` in `crates/core/src/kpm/rust_backend.rs`
- [ ] Create `crates/core/examples/simple_nft_dual.rs` with LGPL-3.0 header
- [ ] Add `[[example]]` entry to `crates/core/Cargo.toml`
- [ ] `cargo fmt --all`
- [ ] `cargo build --all-features` clean
- [ ] `cargo clippy --all-targets --all-features -- --deny warnings` clean
- [ ] `cargo test --all-features` green (M9-2 dual-mode test still passes)
- [ ] `cargo run --example simple_nft_dual` produces expected output (divergence_count = 0, two near-identical homographies, max corner displacement < 2.0 px)
- [ ] PR title: `feat(examples): add simple_nft_dual.rs with DualFreakMatcher and per-frame divergence reporting`
- [ ] PR body references #157 and notes the §10 reference numbers
- [ ] PR base: `feat/freak-visual-database`

---

## 5. References

- Issue [#157](https://github.com/webarkit/WebARKitLib-rs/issues/157) — this work
- Issue [#141 comment-4482406138](https://github.com/webarkit/WebARKitLib-rs/issues/141#issuecomment-4482406138) — original "optional deliverable" framing of M9-2
- [docs/design/m9-2-rust-backend.md §10](./m9-2-rust-backend.md) — reference parity numbers
- M9 #152 — corner reprojection metric (`max_displacement` < 2.0 px tolerance)
- Issue [#90](https://github.com/webarkit/WebARKitLib-rs/issues/90) PR 4 — future `simple_nft.rs` arlog conversion (not in this PR)
- [`CLAUDE.md`](../../CLAUDE.md) §2 — `arlog_*!` logging convention
