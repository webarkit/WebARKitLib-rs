# Issue #86 — Fix `cf_patt`/`id_patt` (and symmetric matrix-mode bug) Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make `ar_get_marker_info` populate the per-mode marker fields (`*_patt`, `*_matrix`) and the final fields (`id`, `cf`, `dir`) according to the ARToolKit C contract, fixing two symmetric porting bugs in one change.

**Architecture:** Refactor `ar_get_marker_info` in `crates/core/src/ar/marker.rs` to mirror `arGetMarkerInfo.c` lines 77-100: the template branch writes only `*_patt` fields, the matrix branch writes only `*_matrix` fields, and a small post-branch helper copies into the final `id`/`cf`/`dir` based on `patt_detect_mode`. Extract the copy logic into a private `finalize_marker_id_cf_dir` helper so it can be unit-tested in isolation without needing a real image pipeline.

**Tech Stack:** Rust (stable), `cargo test`, `cargo fmt`, `cargo clippy`. No new dependencies.

---

## Issue context (read once, refer back as needed)

- **Issue:** [#86 — marker.cf_patt / marker.id_patt not populated when template match succeeds](https://github.com/webarkit/WebARKitLib-rs/issues/86)
- **C reference:** [`AR/arGetMarkerInfo.c` lines 77-100](https://github.com/webarkit/WebARKitLib/blob/master/lib/SRC/AR/arGetMarkerInfo.c#L77-L100) (vendored at `crates/core/third_party/WebARKitLib/lib/SRC/AR/arGetMarkerInfo.c`).
- **Discovery context:** PR #85 added a diagnostic log line to `simple.rs` printing `cf_patt / id_patt / cf_matrix / id_matrix`. The new line surfaced that these fields stay at defaults even when matching succeeds.

### Current bug surface (file: `crates/core/src/ar/marker.rs`)

| Branch | Lines | What it writes | What it should write |
|---|---|---|---|
| Matrix | 850-887 | `id_matrix / dir_matrix / cf_matrix` ✅ | also copy into `id / dir / cf` when mode is matrix-only |
| Template | 890-967 | `id / dir / cf` directly ❌ | should write `id_patt / dir_patt / cf_patt`, then copy into `id / dir / cf` when mode is template-only |

### Detection-mode constants (file: `crates/core/src/types.rs`)

The relevant constants used throughout — values are referenced in the post-branch logic:

- `AR_TEMPLATE_MATCHING_COLOR = 0`
- `AR_TEMPLATE_MATCHING_MONO = 1`
- `AR_MATRIX_CODE_DETECTION = 2`
- `AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX_CODE_DETECTION = 3`
- `AR_TEMPLATE_MATCHING_MONO_AND_MATRIX_CODE_DETECTION = 4`

### Mode → final-field semantics (from C reference)

```text
template-only modes (0, 1)         → id/cf/dir <- id_patt / cf_patt / dir_patt
matrix-only mode (2)               → id/cf/dir <- id_matrix / cf_matrix / dir_matrix
mixed modes (3, 4)                 → id/cf/dir untouched (caller picks per-mode field)
```

---

## Decision Log

| # | Decision | Alternatives considered | Why |
|---|---|---|---|
| 1 | C-faithful refactor of both branches | Strict patch (only fix template branch) | The matrix-only path has the symmetric latent bug. Same root cause, same function. Fixing once avoids a second issue/PR cycle. |
| 2 | Extract post-branch copy into `finalize_marker_id_cf_dir` helper | Inline `match` block | Unit-testable without a real image pipeline. Tiny helper, clear name, no overhead. |
| 3 | Unit test in `crates/core/src/ar/marker.rs` `#[cfg(test)] mod tests` | Integration test in `tests/`; example-based test | Per CLAUDE.md "unit tests live next to the code". Helper is module-local, so unit tests are the right tool. |
| 4 | No CHANGELOG edit | — | CLAUDE.md "CHANGELOG.md is release-only". |
| 5 | Branch off `dev` | Stack on top of #85 branch | CLAUDE.md "fresh branch per issue". |

---

## Task 1: Set up branch and verify baseline

**Files:** none modified (worktree state setup).

**Step 1: Verify we're starting from a clean tree**

```bash
git status
```
Expected: clean working tree (or only the design doc untracked).

**Step 2: Create the feature branch off origin/dev**

```bash
git fetch origin
git checkout -b issue/86-cf-patt-not-populated origin/dev
```
Expected: `Switched to a new branch 'issue/86-cf-patt-not-populated'`.

**Step 3: Verify baseline tests pass on the new branch**

```bash
cargo test --all-features -p webarkitlib-rs --lib ar::marker
```
Expected: `test_ar_detect_marker2_empty` passes. Note any pre-existing failures (e.g. `test_full_pipeline_pose` in `kpm_regression.rs`) — they should be unrelated.

**Step 4: Commit the design doc**

```bash
git add .claude/issue-86-design.md
git commit -m "docs(issue-86): add implementation plan"
```

---

## Task 2: Write failing unit test for the helper (TDD)

**Files:**
- Modify: `crates/core/src/ar/marker.rs` (test module at the bottom, currently lines 978-1003)

**Step 1: Write the failing test**

Add inside the existing `#[cfg(test)] mod tests { ... }` block, after `test_ar_detect_marker2_empty`:

```rust
    /// Verifies `finalize_marker_id_cf_dir` copies the right per-mode fields
    /// into the final `id` / `cf` / `dir` for each detection mode, mirroring
    /// `arGetMarkerInfo.c` lines 92-100.
    #[test]
    fn test_finalize_marker_id_cf_dir_template_color() {
        let mut m = ARMarkerInfo::default();
        m.id_patt = 7;
        m.dir_patt = 2;
        m.cf_patt = 0.85;
        m.id_matrix = 99; // should be ignored
        m.dir_matrix = 3;
        m.cf_matrix = 0.5;

        finalize_marker_id_cf_dir(&mut m, crate::pattern::AR_TEMPLATE_MATCHING_COLOR);

        assert_eq!(m.id, 7);
        assert_eq!(m.dir, 2);
        assert!((m.cf - 0.85).abs() < 1e-9);
    }

    #[test]
    fn test_finalize_marker_id_cf_dir_template_mono() {
        let mut m = ARMarkerInfo::default();
        m.id_patt = 4;
        m.dir_patt = 1;
        m.cf_patt = 0.7;

        finalize_marker_id_cf_dir(&mut m, crate::pattern::AR_TEMPLATE_MATCHING_MONO);

        assert_eq!(m.id, 4);
        assert_eq!(m.dir, 1);
        assert!((m.cf - 0.7).abs() < 1e-9);
    }

    #[test]
    fn test_finalize_marker_id_cf_dir_matrix_only() {
        let mut m = ARMarkerInfo::default();
        m.id_patt = 99; // should be ignored
        m.dir_patt = 2;
        m.cf_patt = 0.5;
        m.id_matrix = 12;
        m.dir_matrix = 0;
        m.cf_matrix = 0.92;

        finalize_marker_id_cf_dir(&mut m, crate::types::AR_MATRIX_CODE_DETECTION);

        assert_eq!(m.id, 12);
        assert_eq!(m.dir, 0);
        assert!((m.cf - 0.92).abs() < 1e-9);
    }

    #[test]
    fn test_finalize_marker_id_cf_dir_mixed_color_matrix_leaves_finals_alone() {
        let mut m = ARMarkerInfo::default();
        // Pre-set finals to sentinel values; verify they're not overwritten.
        m.id = -42;
        m.dir = -42;
        m.cf = -42.0;
        m.id_patt = 1;
        m.dir_patt = 1;
        m.cf_patt = 0.1;
        m.id_matrix = 2;
        m.dir_matrix = 2;
        m.cf_matrix = 0.2;

        finalize_marker_id_cf_dir(
            &mut m,
            crate::types::AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX_CODE_DETECTION,
        );

        assert_eq!(m.id, -42);
        assert_eq!(m.dir, -42);
        assert!((m.cf - -42.0).abs() < 1e-9);
    }

    #[test]
    fn test_finalize_marker_id_cf_dir_mixed_mono_matrix_leaves_finals_alone() {
        let mut m = ARMarkerInfo::default();
        m.id = -42;
        m.dir = -42;
        m.cf = -42.0;

        finalize_marker_id_cf_dir(
            &mut m,
            crate::types::AR_TEMPLATE_MATCHING_MONO_AND_MATRIX_CODE_DETECTION,
        );

        assert_eq!(m.id, -42);
        assert_eq!(m.dir, -42);
        assert!((m.cf - -42.0).abs() < 1e-9);
    }
```

**Step 2: Run tests to verify they fail with "function not found"**

```bash
cargo test --all-features -p webarkitlib-rs --lib ar::marker::tests::test_finalize_marker_id_cf_dir
```
Expected: compile error — `cannot find function 'finalize_marker_id_cf_dir' in this scope` (or similar). That confirms the tests are wired up correctly.

**Step 3: Commit**

```bash
git add crates/core/src/ar/marker.rs
git commit -m "test(marker): add failing tests for finalize_marker_id_cf_dir helper"
```

---

## Task 3: Implement the `finalize_marker_id_cf_dir` helper

**Files:**
- Modify: `crates/core/src/ar/marker.rs` — add helper above the `#[cfg(test)]` block (around current line 977, just before the closing `Ok(())` of `ar_get_marker_info`'s caller scope is irrelevant — place the new helper as a top-level `fn` between `ar_get_marker_info` and the test module).

**Step 1: Add the helper**

Insert just before `#[cfg(test)]` (currently line 978). Exact code:

```rust
/// Copy the per-mode marker fields (`*_patt` or `*_matrix`) into the final
/// `id` / `cf` / `dir` based on the detection mode. Mirrors
/// `arGetMarkerInfo.c` lines 92-100.
///
/// Mixed modes (`AR_TEMPLATE_MATCHING_*_AND_MATRIX_CODE_DETECTION`) leave the
/// final fields untouched — callers must inspect `*_patt` / `*_matrix`
/// themselves to decide.
fn finalize_marker_id_cf_dir(marker: &mut ARMarkerInfo, patt_detect_mode: i32) {
    if patt_detect_mode == crate::pattern::AR_TEMPLATE_MATCHING_COLOR
        || patt_detect_mode == crate::pattern::AR_TEMPLATE_MATCHING_MONO
    {
        marker.id = marker.id_patt;
        marker.dir = marker.dir_patt;
        marker.cf = marker.cf_patt;
    } else if patt_detect_mode == crate::types::AR_MATRIX_CODE_DETECTION {
        marker.id = marker.id_matrix;
        marker.dir = marker.dir_matrix;
        marker.cf = marker.cf_matrix;
    }
    // Mixed modes: do nothing.
}
```

**Step 2: Run the failing tests — they should now pass**

```bash
cargo test --all-features -p webarkitlib-rs --lib ar::marker::tests::test_finalize_marker_id_cf_dir
```
Expected: 5 tests pass.

**Step 3: Run full marker test suite to confirm no regressions**

```bash
cargo test --all-features -p webarkitlib-rs --lib ar::marker
```
Expected: all marker tests pass including the existing `test_ar_detect_marker2_empty`.

**Step 4: Commit**

```bash
git add crates/core/src/ar/marker.rs
git commit -m "feat(marker): add finalize_marker_id_cf_dir helper"
```

---

## Task 4: Rewire the matrix branch to NOT touch `id/cf/dir` directly

**Files:**
- Modify: `crates/core/src/ar/marker.rs` lines 850-887 (matrix branch)

The matrix branch already writes only `id_matrix / dir_matrix / cf_matrix` — no change there. But we need to confirm it doesn't *accidentally* set `id`/`cf`/`dir` anywhere. Read the block one more time and verify.

**Step 1: Re-read the matrix branch**

```bash
# Just inspect, no edit needed.
```

The current matrix branch (lines 850-887) does NOT touch `marker_info[j].id` / `.cf` / `.dir`. ✅ No change required for this task — but documenting it explicitly so the next task can wire the post-branch copy in cleanly.

**Step 2: (no-op task)** — skip commit, proceed to Task 5.

---

## Task 5: Rewire the template branch to write `*_patt` instead of `id/cf/dir`

**Files:**
- Modify: `crates/core/src/ar/marker.rs` lines 945-967 (the inner `if/else` cascade in the template branch).

**Step 1: Replace the `id/dir/cf` writes with `id_patt/dir_patt/cf_patt` writes**

Find this block (currently lines 945-967):

```rust
                        if match_res.is_ok() && p_code >= 0 {
                            marker_info[j].id = p_code;
                            marker_info[j].dir = p_dir;
                            marker_info[j].cf = p_cf;
                        } else {
                            marker_info[j].id = -1;
                            marker_info[j].dir = 0;
                            marker_info[j].cf = p_cf;
                        }
                    } else {
                        marker_info[j].id = -1;
                        marker_info[j].dir = 0;
                        marker_info[j].cf = -1.0;
                    }
                } else {
                    marker_info[j].id = -1;
                    marker_info[j].dir = 0;
                    marker_info[j].cf = 0.0;
                }
            } else {
                marker_info[j].id = -1;
                marker_info[j].dir = 0;
                marker_info[j].cf = 0.0;
            }
```

Replace with (mechanical `id`→`id_patt`, `dir`→`dir_patt`, `cf`→`cf_patt`):

```rust
                        if match_res.is_ok() && p_code >= 0 {
                            marker_info[j].id_patt = p_code;
                            marker_info[j].dir_patt = p_dir;
                            marker_info[j].cf_patt = p_cf;
                        } else {
                            marker_info[j].id_patt = -1;
                            marker_info[j].dir_patt = 0;
                            marker_info[j].cf_patt = p_cf;
                        }
                    } else {
                        marker_info[j].id_patt = -1;
                        marker_info[j].dir_patt = 0;
                        marker_info[j].cf_patt = -1.0;
                    }
                } else {
                    marker_info[j].id_patt = -1;
                    marker_info[j].dir_patt = 0;
                    marker_info[j].cf_patt = 0.0;
                }
            } else {
                marker_info[j].id_patt = -1;
                marker_info[j].dir_patt = 0;
                marker_info[j].cf_patt = 0.0;
            }
```

**Step 2: Build to confirm it compiles**

```bash
cargo build --all-features -p webarkitlib-rs
```
Expected: clean build (warnings unrelated to this file are fine).

**Step 3: DO NOT commit yet — the next task closes the loop with the post-branch call**

The crate compiles, but `marker.id` is now never written in template-only mode. We'll fix that in Task 6 by inserting the helper call. Going through these steps separately preserves the bisection trail.

---

## Task 6: Insert the post-branch `finalize_marker_id_cf_dir` call

**Files:**
- Modify: `crates/core/src/ar/marker.rs` immediately after line 969 (the closing `}` of the template branch) and before the `j += 1;` on line 971.

**Step 1: Add the call**

Find this (currently around lines 968-972):

```rust
            } else {
                marker_info[j].id_patt = -1;
                marker_info[j].dir_patt = 0;
                marker_info[j].cf_patt = 0.0;
            }
        }

        j += 1;
    }
```

Replace with:

```rust
            } else {
                marker_info[j].id_patt = -1;
                marker_info[j].dir_patt = 0;
                marker_info[j].cf_patt = 0.0;
            }
        }

        // Mirror arGetMarkerInfo.c lines 92-100: copy the per-mode fields
        // into the final `id` / `cf` / `dir` based on detection mode.
        finalize_marker_id_cf_dir(&mut marker_info[j], patt_detect_mode);

        j += 1;
    }
```

**Step 2: Build & run all marker tests**

```bash
cargo build --all-features -p webarkitlib-rs
cargo test --all-features -p webarkitlib-rs --lib ar::marker
```
Expected: clean build, all marker tests pass.

**Step 3: Commit Task 5 + Task 6 together (the rewire only makes sense as one atomic change)**

```bash
git add crates/core/src/ar/marker.rs
git commit -m "fix(marker): populate cf_patt/id_patt and copy to final id/cf per mode

Closes #86. The template branch of ar_get_marker_info was writing
marker.id / marker.cf / marker.dir directly, never touching the
per-mode id_patt / cf_patt / dir_patt fields. Symmetrically, the
matrix branch wrote only id_matrix / cf_matrix / dir_matrix and
never set the final id / cf / dir.

Refactor mirrors arGetMarkerInfo.c lines 77-100:
- Template branch writes only *_patt fields.
- Matrix branch keeps writing *_matrix fields (unchanged).
- New finalize_marker_id_cf_dir helper copies per-mode fields into
  the final id/cf/dir based on patt_detect_mode (matrix-only,
  template-only, or mixed-no-copy).

The matrix-only mode case (where marker.id was previously left at
-1 even on a successful barcode decode) is fixed by the same change."
```

---

## Task 7: Verify with the live `simple` example

**Files:** none modified.

**Step 1: Run the simple example end-to-end**

```bash
cargo run --release --example simple --features log-helpers
```

**Step 2: Verify the diagnostic line shows non-default `*_patt` values**

Look for the per-marker block. **Before this fix** it printed:

```
[info]   Confidence (CF): 0.0965
[info]   Matched ID: 0
[info]   Template-match CF/ID: 0.0000 / -1  (cfMatrix: 0.0000, idMatrix: -1)
```

**After this fix** expect:

```
[info]   Confidence (CF): 0.0965
[info]   Matched ID: 0
[info]   Template-match CF/ID: 0.0965 / 0  (cfMatrix: 0.0000, idMatrix: -1)
```

(`cf_patt == cf` and `id_patt == id`. Matrix fields stay at defaults because we're in template-only mode.)

**Step 3:** No commit; verification only.

---

## Task 8: Final pre-PR checklist (CLAUDE.md §5)

**Files:** none modified.

**Step 1: Format check**

```bash
cargo fmt --all -- --check
```
Expected: no output. If diff: run `cargo fmt --all` and commit as a fixup.

**Step 2: Build all features**

```bash
cargo build --all-features
```
Expected: clean.

**Step 3: Clippy**

```bash
cargo clippy --all-targets --all-features
```
Expected: no new warnings in `marker.rs`. Pre-existing warnings elsewhere are fine.

**Step 4: Full test suite**

```bash
cargo test --all-features
```
Expected: only the pre-existing `test_full_pipeline_pose` failure (unrelated to this change). All marker tests pass, all five new helper tests pass.

**Step 5: Push and open PR**

```bash
git push -u origin issue/86-cf-patt-not-populated
gh pr create --base dev --title "Fix marker.cf_patt/id_patt and symmetric matrix-mode bug" --body "..."
```

PR body template:

```markdown
## Summary

Closes #86. Refactor `ar_get_marker_info` in `crates/core/src/ar/marker.rs` to mirror the ARToolKit C contract from `arGetMarkerInfo.c` lines 77-100. Fixes two symmetric porting bugs in one change:

1. **Reported in #86**: template branch never wrote `id_patt / cf_patt / dir_patt`.
2. **Symmetric latent bug**: matrix-only mode never wrote the final `marker.id / marker.cf / marker.dir`.

## Changes

- **Matrix branch**: unchanged — already wrote `*_matrix` correctly.
- **Template branch**: rewired from writing `id/dir/cf` directly to writing `id_patt/dir_patt/cf_patt`.
- **New helper** `finalize_marker_id_cf_dir`: copies per-mode fields into the final `id/cf/dir` based on `patt_detect_mode` (template-only → `*_patt`; matrix-only → `*_matrix`; mixed → no-op, caller decides).
- **Five new unit tests** in `marker.rs` `mod tests`: cover all five detection-mode constants.

## Verification

`cargo run --release --example simple --features log-helpers`:

| | Before | After |
|---|---|---|
| `cf` | 0.0965 | 0.0965 |
| `id` | 0 | 0 |
| `cf_patt` | **0.0000** | **0.0965** |
| `id_patt` | **-1** | **0** |

## Test plan

- [x] `cargo fmt --all -- --check`
- [x] `cargo build --all-features`
- [x] `cargo clippy --all-targets --all-features`
- [x] `cargo test --all-features` — only pre-existing `test_full_pipeline_pose` failure (unrelated)
- [x] `cargo run --example simple --features log-helpers` shows `cf_patt == cf` / `id_patt == id`
```

---

## Quality Gates Summary

- [ ] `cargo fmt --all -- --check` clean
- [ ] `cargo build --all-features` clean
- [ ] `cargo clippy --all-targets --all-features` no new warnings in `marker.rs`
- [ ] `cargo test --all-features` — five new tests pass; existing tests unaffected
- [ ] `simple` example shows `cf_patt`/`id_patt` populated when template match succeeds
- [ ] Branch is `issue/86-cf-patt-not-populated` off `origin/dev`
- [ ] No `println!` / `eprintln!` added
- [ ] No `CHANGELOG.md` edit
- [ ] LGPL-3.0 header preserved (no new files)

## Out of scope (explicitly)

- Refactoring `pattern_match` or `ar_matrix_code_get_id` — they already produce the right values.
- Changing the `ARMarkerInfo` field layout.
- Rebuilding the diagnostic output in `simple.rs` — already done in PR #85.
- Filing a separate issue for the matrix-only bug — fixing it inline.
- SIMD / multithreading — not relevant to this code path.
