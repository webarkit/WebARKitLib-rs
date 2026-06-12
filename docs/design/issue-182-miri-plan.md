# Issue #182 — Miri UB Validation for Pure-Rust Code Paths

Design plan produced via `/brainstorming` skill.
Source issue: https://github.com/webarkit/WebARKitLib-rs/issues/182
Branch: `ci/miri-validation-182` (off `dev`).

---

## Understanding Summary

- **What**: Add a Miri-based undefined-behavior validation CI job
  (`cargo +nightly miri test -p webarkitlib-rs --lib`) and accompanying
  `docs/miri.md`.
- **Why**: Catch use-after-free, OOB reads, invalid `unsafe` invariants,
  uninitialized-memory reads, and reference-aliasing violations in the
  freshly-ported KPM/FREAK/AR/AR2 code before contributors build on top
  of it. Sister to #180 (clippy strictness) as the second half of the
  post-M9 safety net.
- **Who**: Maintainers and contributors touching `unsafe`, FFI
  boundaries, or hot loops with unchecked slicing.
- **Scope**: Pure-Rust default features, lib targets only. Excludes
  `ffi-backend` (Miri can't execute C++), `simd-x86-*` (Miri x86 SIMD
  support is incomplete), and FFI-dependent integration tests under
  `tests/`.
- **Non-goals**: Fixing whatever UB Miri surfaces in this PR — each
  finding gets its own follow-up PR, mirroring the #180 cleanup series
  pattern.

## Modules under Miri validation

Explicit list (per user correction during brainstorming — issue draft
only highlighted KPM/FREAK):

- `crates/core/src/ar/` — labeling, pattern matching, marker decoding,
  image processing (C ARToolKit port).
- `crates/core/src/ar2/` — image_set, feature_map, byteorder I/O.
- `crates/core/src/kpm/` — KPM pipeline.
- `crates/core/src/kpm/freak/` — math, homography, clustering, matcher,
  visual_database (M6→M9 fresh port).

## Assumptions

- `kpm-build (ubuntu-latest)` is the right neighbor for the Miri job in
  `ci.yml` — both are pure-Rust correctness gates.
- No `rust-toolchain.toml` change needed; nightly stays scoped to the
  Miri job.
- Conventional Commits: PR is
  `ci(miri): add Miri UB validation for pure-Rust code paths (#182)`.
- The repo-wide `on:` block (`push: branches: ['**']` + `pull_request:`)
  requires a job-level `if:` guard to restrict to PRs against
  `dev`/`main` (per Q2 decision).

## Non-Functional Requirements

- **CI cost**: Single Ubuntu runner, no matrix. Expected 10–30 min on
  warm cache. Acceptable per issue.
- **Reliability**: `continue-on-error: true` initially; transient
  nightly breaks → manual rerun.
- **Maintenance**: Nightly pin bumped manually when Miri breaks or
  every ~3 months. No automation.
- **Security**: None — checkout + rustup + cache action; no secrets.
- **Scope discipline**: This PR adds *only* CI infra + docs. Any UB
  Miri surfaces → separate fix PRs.

## Decision Log

| # | Decision | Alternatives | Rationale |
|---|---|---|---|
| 1 | `continue-on-error: true`, ratchet later | Block PR until clean; bundle fixes here | Matches #180 ratcheting precedent; decouples infra from unknown-size UB cleanup |
| 2 | Trigger on PRs to `dev`/`main` only | Every push; nightly schedule; both | Pre-merge gate is the actual value; nightly schedule = YAGNI |
| 3 | `Swatinem/rust-cache@v2`, Miri-keyed | No cache; explicit sysroot cache | Standard; handles Miri sysroot when keyed on toolchain |
| 4 | Pin nightly date inline in workflow | Floating nightly; `rust-toolchain.toml` | Local to Miri job; survives nightly regressions; doesn't bleed into stable jobs |
| 5 | `docs/miri.md` (new file) | Section in `CONTRIBUTING.md`; both | Single canonical URL for CI annotations & follow-up PRs |
| 6 | Standalone top-level `miri:` job | Step in existing job; reusable workflow | Matches repo pattern; trivial ratchet to required; no nightly contamination of stable jobs |
| 7 | Scope description names `ar/`, `ar2/`, `kpm/`, `kpm/freak/` explicitly | KPM/FREAK only (per issue draft) | User correction — `ar/` modules contain the C ARToolKit port surface and matter equally |
| 8 | `-Zmiri-strict-provenance` enabled in CI | Default provenance | Catches pointer-provenance bugs that loose-provenance hides; appropriate for a careful port |
| 9 | Job-level `if:` guard for PR-to-dev/main scoping | Workflow-level `on:` override (not possible per-job); separate workflow file | Cleanest; one line; doesn't touch other jobs' triggers |

## Final Design

### Workflow job (added to `.github/workflows/ci.yml`)

```yaml
miri:
  name: miri (pure-Rust UB validation)
  runs-on: ubuntu-latest
  # Per Q2: restrict to PRs against dev/main only.
  if: github.event_name == 'pull_request' &&
      (github.base_ref == 'dev' || github.base_ref == 'main')
  continue-on-error: true  # ratchet to false after UB fix PRs land (#182)
  env:
    MIRI_NIGHTLY: nightly-2026-06-01
  steps:
    - uses: actions/checkout@v5
    - name: Install pinned nightly + Miri
      run: |
        rustup toolchain install $MIRI_NIGHTLY --component miri --profile minimal
        rustup default $MIRI_NIGHTLY
        cargo miri setup
    - name: Rust cache
      uses: Swatinem/rust-cache@v2
      with:
        key: miri-${{ env.MIRI_NIGHTLY }}
        shared-key: miri
    - name: Run Miri (pure-Rust, lib targets only)
      run: cargo miri test -p webarkitlib-rs --lib
      env:
        MIRIFLAGS: -Zmiri-backtrace=full -Zmiri-strict-provenance
```

### `docs/miri.md` skeleton

Sections:
1. What Miri validates here (module list).
2. What Miri does NOT validate (`ffi-backend`, SIMD, FFI integration tests).
3. Running locally (commands).
4. Investigating failures (`MIRIFLAGS` knobs).
5. Bumping the nightly pin (policy).
6. CI gate status (`continue-on-error` → required ratchet).

## Risks

| Risk | Mitigation |
|---|---|
| Nightly Miri broken on the pinned date | Pick known-good date during PR; document bump procedure |
| First-run UB findings overwhelming | `continue-on-error: true`; separate follow-up PRs |
| CI minute cost on PRs | PR-scoped via `if:` guard; rust-cache keeps warm runs cheap |
| Cache pollution between nightly bumps | Cache key includes `MIRI_NIGHTLY` → automatic invalidation |

## Exit Criteria — all met

- [x] Understanding Lock confirmed
- [x] Design approach A (standalone top-level job) accepted
- [x] Assumptions documented
- [x] Risks acknowledged
- [x] Decision Log complete

## Out of Scope

- Fixing any UB findings surfaced by the first Miri run.
- Adding Miri to `simd-*` or `ffi-backend` paths.
- Multi-OS Miri matrix (Linux-only is sufficient — Miri's findings are
  not OS-specific for pure-Rust code).
