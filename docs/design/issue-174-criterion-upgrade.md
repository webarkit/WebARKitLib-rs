# Issue #174 — Upgrade criterion 0.5.1 → 0.8.x

Design document for the dependency bump described in
[issue #174](https://github.com/webarkit/WebARKitLib-rs/issues/174).
Produced via the `/brainstorming` workflow on 2026-06-10.

---

## 1. Understanding Lock (confirmed)

**What is being built**
A dependency-only upgrade of `criterion` from `0.5.1` → `0.8` in
`crates/core/Cargo.toml`, plus the minimal source fixes in the three
bench files to match criterion 0.8's API.

**Why it exists**
`criterion 0.5.1` (May 2023) is ~3 years stale; `0.8.2` (Feb 2026) is
the current stable. Surfaced as a follow-up to M9-3 (#142), intentionally
separated per CLAUDE.md's "fresh branch per sub-issue" rule — the
upgrade is unrelated to the "remove C++ FFI as default" theme of #142,
and the 0.5 → 0.8 span has API breaks that deserve their own focused
review.

**Who it is for**
WebARKitLib-rs maintainers running local benchmarks, and CI's
`benchmarks` job.

**Key constraints**
- No API changes outside the three bench files.
- Pin style: `criterion = "0.8"` (matches the existing dev-dep style).
- Single bench matrix for `BENCHMARKS.md`: `--features simd`.
- `BENCHMARKS.md` refresh is **scope-driven by the bench run output**
  (see Decision #4).
- All four CLAUDE.md §5 pre-commit checks must pass.

**Explicit non-goals**
- No new benchmarks (KPM/NFT-specific gaps tracked separately under #142).
- No migration to `divan` / `iai`.
- No bumps to other dev-deps.
- No changes to library code.

**Assumptions**
1. The 0.5 → 0.8 break surface in these benches is limited to
   `criterion::black_box` → `std::hint::black_box`. Verified by
   compiling.
2. `Criterion::default()`, `criterion_group!`, `criterion_main!`, and
   `group.sample_size(N)` are stable across 0.5 → 0.8. (Spot-checked
   criterion 0.8 changelog.)
3. `BENCHMARKS.md` edits will be scope-driven by the bench run output:
   minimal numeric refresh by default; add sections only if the run
   reveals something that needs explanation.
4. CI's `benchmarks` job uses the same `cargo bench` invocation and
   doesn't pin criterion separately.

---

## 2. Decision Log

| #  | Decision                                                                                   | Alternatives                                  | Why                                                                                                              |
|----|--------------------------------------------------------------------------------------------|-----------------------------------------------|------------------------------------------------------------------------------------------------------------------|
| 1  | Pin as `criterion = "0.8"`                                                                 | `"0.8.2"`, `"=0.8.2"`                         | Matches existing dev-dep style (`clap = "4.5"`); allows patch updates. User-confirmed.                            |
| 2  | Single PR, two commits (deps+code, then doc)                                               | One commit / per-file commits                 | Clean blame across deps vs doc; one logical change → one PR.                                                      |
| 3  | Bench matrix: `--features simd` only                                                       | + scalar / + ffi-backend / + dual-mode        | Matches issue acceptance; minimizes scope creep. User-confirmed.                                                  |
| 4  | `BENCHMARKS.md`: scope-driven refresh                                                      | Full rewrite / skip-numbers                   | "Full rewrite" overstates the work for a dep bump; decide minimal-edit vs added section after seeing run output. |
| 5  | `black_box` migration: `criterion::black_box` → `std::hint::black_box`                     | Keep `criterion::black_box` (removed in 0.7+) | Required by 0.7+. No call-site changes, just the import line.                                                     |
| 6  | Verify MSRV before pushing                                                                 | Bump MSRV silently                            | criterion 0.8 needs Rust 1.74+; if workspace MSRV is lower, surface as its own decision before pushing.           |

---

## 3. File Changes

### 3.1 `crates/core/Cargo.toml`

```toml
# was
criterion = "0.5.1"
# after
criterion = "0.8"
```

### 3.2 `crates/core/benches/{marker_bench,feature_map_bench,simd_bench}.rs`

```rust
// before
use criterion::{black_box, criterion_group, criterion_main, Criterion};
// after
use criterion::{criterion_group, criterion_main, Criterion};
use std::hint::black_box;
```

No other call-site changes expected.

### 3.3 `crates/core/benches/BENCHMARKS.md`

Scope-driven after the bench run:

- **Default path (minimal edit)**: replace numbers table, bump the
  "Tooling" line to `criterion 0.8`, refresh the timestamp.
- **If the run reveals a shape change** (new metric reported, restructured
  output): add a small section explaining the delta.
- **If a bench had to be restructured to compile**: document the
  restructuring in a new subsection.

---

## 4. Execution Sequence

1. Branch off `dev` → `chore/criterion-0.8-upgrade`.
2. Bump `criterion = "0.8"` in `crates/core/Cargo.toml`.
3. Fix `black_box` imports in the 3 bench files.
4. **Compile gate**: `cargo build --benches --all-features`. If it fails
   outside the `black_box` line, stop and re-evaluate (assumption #1
   broken).
5. **Pre-commit gates** (CLAUDE.md §5):
   - `cargo fmt --all`
   - `cargo build --all-features`
   - `cargo clippy --all-targets --all-features -- --deny warnings`
   - `cargo test --all-features`
6. Commit 1: `chore(deps): upgrade criterion 0.5.1 → 0.8`.
7. Bench run: `cargo bench -p webarkitlib-rs --features simd`. Capture
   output.
8. Diff old vs new numbers in `BENCHMARKS.md`. Apply scope-driven edit.
9. Commit 2: `doc(benches): refresh BENCHMARKS.md for criterion 0.8`.
10. Push branch, open PR against `dev` titled
    `chore(deps): upgrade criterion 0.5.1 → 0.8.x`.

---

## 5. Risks and Rollback

- **Risk**: a transitive of criterion 0.8 conflicts with another
  dev-dep. *Mitigation*: `cargo tree -p webarkitlib-rs --target all`
  if `cargo build` errors.
- **Risk**: workspace MSRV is below criterion 0.8's requirement
  (Rust 1.74+). *Mitigation*: check `rust-version` before pushing;
  surface a separate decision if a bump is needed.
- **Rollback**: revert the two commits. No library-code or public API
  changes, so no downstream breakage is possible.

---

## 6. Testing Strategy

- `cargo build --benches` is the real correctness gate — benches don't
  have unit tests.
- `cargo bench --features simd` must produce numbers for every existing
  bench (issue acceptance criterion).
- `cargo test --all-features` must stay green (sanity check; should be
  unaffected by a dev-dep bump).

---

## 7. Acceptance (mirrors issue #174)

- [ ] `cargo bench -p webarkitlib-rs --features simd` builds and runs
      cleanly with criterion 0.8.
- [ ] All existing benches produce output (no API-mismatch panics).
- [ ] `BENCHMARKS.md` refreshed with new numbers.
- [ ] `cargo fmt --all -- --check` and
      `cargo clippy --all-targets --all-features -- --deny warnings`
      green.
- [ ] CI's `benchmarks` job passes on the upgrade PR.
