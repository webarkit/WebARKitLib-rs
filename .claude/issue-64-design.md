# Issue #64 (M6-2) — Design Document

**Date:** 2026-04-26
**Branch:** `feature/milestone6-kpm-linear-algebra` (off `feature/milestone6-kpm-math-port`)
**Target:** `feature/milestone6-kpm-math-port` (milestone integration branch)
**Submodule rev:** `656436e36b`
**Companion docs:** [issue-64-usage-audit.md](issue-64-usage-audit.md)

---

## Understanding Summary

- **What is being built:** A pure-Rust port of FreakMatcher's `linear_algebra.h` and `linear_solvers.h` headers, added to the existing `crates/core/src/kpm/freak/math.rs` from M6-1. ~14 public functions + 8 private helpers, covering linear-system solvers (2×2, symmetric 3×3), the 8×9 DLT null-vector solver (column-pivoted Gram-Schmidt), and supporting vector/matrix primitives.
- **Why it exists:** Provides the linear-solver primitives that M6-3's homography port (`solve_null_vector_8x9_destructive` is the DLT core) and the broader M6 effort to replace the C++ FFI layer depend on.
- **Who it is for:** WebARKitLib-rs core developers building toward the homography milestone, and downstream `webarkitlib-wasm` consumers.
- **Key constraints:**
  - Must match C++ behavior for the 3 live solvers — validated via dual-mode FFI tests.
  - Follow project conventions: LGPL header, `#[inline(always)]`, `arlog_*!` macros (no `eprintln!`), no `CHANGELOG.md` edits, fresh branch per sub-issue.
  - Match M6-1 stylistic patterns exactly (hybrid generic/concrete signatures; comprehensive unit tests; dual-mode for live functions only).
- **Explicit non-goals:**
  - **Out of scope:** `cauchy_cost`/`cauchy_cost_jac` (deferred to M6-3 per chat correction); functions in `linear_algebra.h` not on the deliverable list (`Identity3x3`, `MatrixInverse3x3`, `SubVector2`, symmetric-extend helpers); SIMD optimization; refactoring `math.rs` into multiple files.

## Assumptions

1. **PR targets `feature/milestone6-kpm-math-port`** — confirmed by maintainer. The 3 M6 sub-issues will land on this milestone integration branch, which then merges to `dev` as one cohesive milestone.
2. **`matrix.h` helpers in scope:** `Multiply_2x2_2x1` and `Multiply_3x3_3x1` (private) are needed by the 2×2 and symmetric 3×3 solvers respectively. Only these two helpers are ported, not the rest of `matrix.h`.
3. **`dual-mode` feature** still implies `ffi-backend` (wired in M6-1 `Cargo.toml`). M6-2 dual-mode tests reuse existing FFI scaffolding.
4. **Tolerances:** `1e-5` element-wise for the 2×2 and symmetric 3×3 dual-mode tests; `1e-4` for the null-vector test (with sign-ambiguity handling — see Section 3).
5. **Submodule rev `656436e36b`** stays pinned for the duration of M6-2.

## Decision Log

| # | Decision | Alternatives | Rationale |
|---|----------|--------------|-----------|
| 1 | Drop `cauchy_cost`/`cauchy_cost_jac` from M6-2 scope | Port them anyway; derive Jacobian inline | Italian chat correction: tightly coupled to `RobustHomography` Gauss-Newton optimizer; logical home is M6-3 with `homography.rs`. Comment posted on #65 to add to that scope. |
| 2 | Helpers as private (`fn`, no `pub`) | Public; inline into callers | Minimal public surface; preserves C++ structure |
| 3 | Comprehensive tests for every function (~25 unit tests) | Spec-minimum (2 tests); targeted middle | Matches M6-1 quality bar; M6-3 depends on correctness |
| 4 | Hybrid signatures — generic for trivial, concrete `f32` for math-heavy | All concrete; all generic | Matches M6-1; future-proof for f64; concrete only where `sqrt`/epsilon thresholds force it |
| 5 | Single file (`math.rs`) | Split into `linear_algebra.rs`; submodule pattern | Literal spec compliance; can refactor later |
| 6 | Dual-mode for the 3 live solver entrypoints only | All solvers; just null-vector; no dual-mode | Audit shows only 3 have production callers (matches M6-1's `fastexp6`-only pattern) |
| 7 | Dead-code functions still ported with status doc comments | Skip them | Matches M6-1's treatment of `fastsqrt1`/`fastatan2`/`fastatan2_360` |
| 8 | Approach A: 2-commit cadence (port + tests in one; dual-mode in second) | 3-commit split by source file; single big commit | Mirrors M6-1 PR #91 review shape exactly |

---

## Final Design

### Section 1 — File layout & module structure

The existing M6-1 `math.rs` (1065 LOC) already has section dividers. M6-2 inserts in 3 places:

```
1. License header                                 (unchanged)
2. Module doc                                     ← updated text
3. Constants                                      (unchanged)
4. "indexing.h Functions"                         (unchanged)
5. "math_utils.h Functions"                       (unchanged)
NEW 5a. "linear_algebra.h Functions"              ← 11 listed deliverables + 5 private helpers
NEW 5b. "linear_solvers.h Functions"              ← 3 listed deliverables + 1 consolidated + 2 private helpers
6. #[cfg(test)] mod tests {                       ← appended with ~18 new tests
7. extern "C" block (dual-mode FFI declarations)  ← appended with 3 declarations
8. mod dual_mode_tests                            ← appended with 3 cross-validation tests
```

### Section 2 — Function signatures & API

#### Public API additions (~14 functions)

From `linear_algebra.h` — **concrete `f32`** for solvers, **generic** for trivial vector ops:

```rust
pub fn determinant_symmetric_3x3(a: &[f32; 9]) -> f32
pub fn solve_linear_system_2x2(x: &mut [f32; 2], a: &[f32; 4], b: &[f32; 2]) -> bool
pub fn solve_symmetric_linear_system_3x3(x: &mut [f32; 3], a: &[f32; 9], b: &[f32; 3]) -> bool
pub fn dot_product_4<T>(a: &[T; 4], b: &[T; 4]) -> T  where T: Mul + Add + Copy
pub fn dot_product_9<T>(a: &[T; 9], b: &[T; 9]) -> T
pub fn sum_squares_9<T>(x: &[T; 9]) -> T
pub fn scale_vector_4<T>(dst: &mut [T; 4], src: &[T; 4], s: T)
pub fn scale_vector_8<T>(dst: &mut [T; 8], src: &[T; 8], s: T)
pub fn scale_vector_9<T>(dst: &mut [T; 9], src: &[T; 9], s: T)
pub fn accumulate_scaled_vector_9<T>(dst: &mut [T; 9], src: &[T; 9], s: T)
pub fn add_scaled_vectors_4<T>(w: &mut [T; 4], u: &[T; 4], v: &[T; 4], a: T, b: T)
pub fn update_outer_product_2x2<T>(a: &mut [T; 4], x: &[T; 2])
pub fn update_gauss_newton_operations_2x2<T>(b: &mut [T; 2], j: &[T; 2], residual: T)
```

From `linear_solvers.h`:

```rust
pub fn accumulate_projection_9<T>(x: &mut [T; 9], e: &[T; 9], a: &[T; 9])
pub fn solve_null_vector_8x9_destructive(x: &mut [f32; 9], a: &mut [f32; 72]) -> bool
pub fn solve_tridiagonal_destructive(x: &mut [f32], a: &[f32], b: &[f32], c: &mut [f32]) -> bool
```

#### Private helpers

```rust
// linear_algebra.h derivatives
fn cofactor_2x2<T>(a: T, b: T, c: T, d: T) -> T            // 4-arg form: ad - bc
fn cofactor_2x2_sym<T>(a: T, b: T, c: T) -> T              // 3-arg symmetric form: ac - b²
fn determinant_2x2<T>(a: &[T; 4]) -> T                     // = cofactor_2x2(...)
fn matrix_inverse_2x2(b: &mut [f32; 4], a: &[f32; 4], threshold: f32) -> bool
fn matrix_inverse_symmetric_3x3(b: &mut [f32; 9], a: &[f32; 9], threshold: f32) -> bool

// matrix.h derivatives (only the 2 needed)
fn multiply_2x2_2x1<T>(y: &mut [T; 2], a: &[T; 4], x: &[T; 2])
fn multiply_3x3_3x1<T>(y: &mut [T; 3], a: &[T; 9], x: &[T; 3])

// linear_solvers.h consolidation
fn orthogonalize_pivot_8x9_basis(step: usize, q: &mut [f32; 72], a: &mut [f32; 72]) -> bool
fn orthogonalize_identity_row(x: &mut [f32; 9], q: &[f32; 72], i: usize) -> f32
fn orthogonalize_identity_8x9(x: &mut [f32; 9], q: &[f32; 72]) -> bool
```

All functions marked `#[inline(always)]`. All have doc comments with C++ source citation and upstream usage status (live / dead / transitively-live).

### Section 3 — `solve_null_vector_8x9_destructive` algorithm

The 8 C++ `OrthogonalizePivot8x9Basis*` functions consolidate into a single Rust function with a `step: usize` parameter (0..=7). Step 0 is special-cased (no previous basis to project against; also seeds Q from A). Steps 1..=7 follow a uniform pattern: project remaining columns onto the previous basis, find the pivot column (largest squared-norm), swap to position `step`, normalize.

Caller pattern in `solve_null_vector_8x9_destructive`:

```rust
let mut q = [0.0_f32; 72];
for step in 0..8 {
    if !orthogonalize_pivot_8x9_basis(step, &mut q, a) {
        arlog_e!("solve_null_vector_8x9_destructive: column {} is rank-deficient", step);
        return false;
    }
}
orthogonalize_identity_8x9(x, &q)
```

**Sign ambiguity:** A null vector is defined only up to sign — Rust and C++ may legitimately produce vectors that differ by a global sign flip due to swap-tie-breaking. **The dual-mode test compares `|dot(rust_x, cpp_x)|` to `1.0`** rather than element-wise.

**Failure threshold:** Match C++ exactly (`ss == 0.0` strict equality, no epsilon). Rare in practice but real degeneracy.

### Section 4 — Error handling & edge cases

| Function | Failure condition |
|----------|-------------------|
| `solve_linear_system_2x2` | `\|det A\| <= f32::EPSILON` (matches C++ `std::numeric_limits<T>::epsilon()`) |
| `solve_symmetric_linear_system_3x3` | `\|det A\| <= f32::EPSILON` |
| `solve_null_vector_8x9_destructive` | column rank-deficient (`ss == 0.0` strict) |
| `solve_tridiagonal_destructive` | `b[0] == 0.0` or forward-elimination divisor == 0 |

Every false-return uses the CLAUDE.md "log + return Err" pattern with `arlog_e!`. The math layer always logs (per CLAUDE.md taxonomy these are wiring/misconfiguration errors). M6-3 RANSAC callers can guard with their own log filtering if too noisy.

**NaN handling:** not validated. Garbage in, garbage out — matches C++ semantics.
**Pivot tie-breaking:** strict `>` (leftmost wins) — matches M6-1 `max_index_*` and C++ `MaxIndex*`.
**`solve_tridiagonal_destructive`:** `debug_assert!` on slice lengths at entry; release builds rely on indexing panics for undersize inputs (preferable to silent corruption).

### Section 5 — Test strategy

**~18 new unit tests** in the existing `mod tests` block:

| Category | Functions | Test approach | Tolerance |
|----------|-----------|---------------|-----------|
| Trivial vector ops | `dot_product_*`, `scale_vector_*`, `sum_squares_9`, etc. | Hand-computed expected values | Exact (`assert_eq!`) |
| Determinants | `determinant_symmetric_3x3`, helpers | Known matrix → known scalar | `1e-5` |
| Matrix inverses (private) | `matrix_inverse_*` | A·A⁻¹ ≈ I | `1e-5` |
| Linear solvers | `solve_linear_system_2x2`, `solve_symmetric_linear_system_3x3` | Construct A, x_known, b = A·x_known; assert solver recovers x_known | **`1e-4` per spec** |
| Singular detection | All solvers | Pass singular A, assert returns `false` | n/a |
| **DLT null vector (per spec)** | `solve_null_vector_8x9_destructive` | 4 known point correspondences → known H → build A; assert `\|A·x\| < 1e-4` | **`1e-4` per spec** |
| Tridiagonal | `solve_tridiagonal_destructive` | 5×5 system with known solution | `1e-5` |
| Multiply helpers | `multiply_*` | Hand-computed | Exact |

**3 dual-mode tests** in `mod dual_mode_tests`:

```rust
#[test]
fn solve_linear_system_2x2_matches_cpp() {
    // 1000 random non-singular 2x2 systems, seeded RNG (rand::SeedableRng = StdRng::seed_from_u64(0xDEADBEEF))
    // assert (rust - cpp).abs() < 1e-5 element-wise
}

#[test]
fn solve_symmetric_linear_system_3x3_matches_cpp() {
    // 500 random SPD 3x3 systems (build A = M*M^T + 0.1*I to ensure positive-definite)
    // tolerance 1e-5 element-wise
}

#[test]
fn solve_null_vector_8x9_destructive_matches_cpp() {
    // 100 random rank-8 8x9 matrices (random 4-point homography correspondences)
    // sign ambiguity: assert (1.0 - dot(rust_x, cpp_x).abs()) < 1e-4
    // logs max divergence via arlog_e! (per CLAUDE.md, no eprintln!)
}
```

`rand` is already a `[dependencies]` entry — no new crates.

### Section 6 — Verification & PR plan

**Implementation order within Commit 1:**
1. Constants & module-doc updates
2. Private helpers (`cofactor_2x2`, `determinant_2x2`, `multiply_*`)
3. Trivial vector ops (`dot_product_*`, `scale_vector_*`, etc.)
4. Determinant + matrix-inverse helpers (private)
5. Public solvers (`solve_linear_system_2x2`, `solve_symmetric_linear_system_3x3`, `determinant_symmetric_3x3`)
6. Linear-solvers section (`accumulate_projection_9` → `orthogonalize_pivot_8x9_basis` → `orthogonalize_identity_*` → `solve_null_vector_8x9_destructive`)
7. `solve_tridiagonal_destructive` (dead code; port last)
8. All ~18 new unit tests appended to `mod tests`
9. `cargo fmt`, `cargo test`, `cargo clippy --workspace -- -D warnings`

**Commit 2:**
1. Add 3 FFI declarations to `kpm_c_api.h`
2. Add 3 thin `extern "C"` wrappers to `kpm_c_api.cpp` (calling `vision::Solve*`)
3. Add 3 Rust `extern` declarations + 3 dual-mode test bodies
4. `cargo test --features dual-mode -- kpm::freak::math`
5. `cargo clippy --workspace -- -D warnings` (catch CI failures locally before push)

**Pre-PR checklist (matches M6-1 / CLAUDE.md §5):**

- [ ] `cargo fmt --all -- --check` clean
- [ ] `cargo build -p webarkitlib-rs --all-features` clean
- [ ] `cargo clippy --workspace -- -D warnings` clean (the **exact** CI command)
- [ ] `cargo test -p webarkitlib-rs -- kpm::freak::math` — all M6-1 + M6-2 unit tests green
- [ ] `cargo test -p webarkitlib-rs --features dual-mode -- kpm::freak::math` green
- [ ] LGPL-3.0 header preserved on `math.rs`
- [ ] No `println!`/`eprintln!` introduced (uses `arlog_e!`)
- [ ] `CHANGELOG.md` not touched
- [ ] All public functions have doc comments with C++ source citation + upstream-status note

**PR plan:**
- **Target:** `feature/milestone6-kpm-math-port`
- **Title:** `feat(kpm): port linear algebra + linear solvers (M6-2)`
- **Body:** mirrors M6-1 PR #91 structure (Summary / What changed / Validation results / Reviewer notes)
- **Refs:** #64

---

## Risks & Mitigations

| Risk | Mitigation |
|------|-----------|
| Sign-flip false-failure in null-vector dual-mode test | `\|dot(rust, cpp)\|` comparison (Section 3) |
| Pivot tie-breaking divergence | Strict `>` matches C++ `MaxIndex*` exactly |
| `solve_null_vector` numerical instability on rank-deficient inputs | Explicit `ss == 0.0` check + `arlog_e!`; dual-mode test only feeds rank-8 inputs |
| Dead-code functions surfacing as warnings under clippy `-D warnings` | Same approach as M6-1 — port them but the `bool` return values from solvers ensure they don't trigger dead-code lints; add `#[allow]` only if needed |
| `f32::EPSILON` vs C++ `numeric_limits<float>::epsilon()` divergence | They are bit-identical (1.19209290e-07); validated by dual-mode tests |
