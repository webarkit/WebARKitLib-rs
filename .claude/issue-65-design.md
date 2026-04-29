# Issue #65 (M6-3) — Design Document

**Date:** 2026-04-28
**Branch:** `feature/milestone6-kpm-homography` (off `feature/milestone6-kpm-math-port`)
**Target:** `feature/milestone6-kpm-math-port` (milestone integration branch)
**Companion docs:** [issue-64-design.md](issue-64-design.md), [issue-64-usage-audit.md](issue-64-usage-audit.md)

---

## Understanding Summary

- **What is being built:** A new file `crates/core/src/kpm/freak/homography.rs` (~1600–1900 LOC) porting `math/homography.h` (218 lines), `math/robustifiers.h` (53 lines), and `homography_estimation/robust_homography.h` (673 lines) to pure Rust. Provides geometric primitives, Lie algebra basis, Cauchy robust cost + IRLS Jacobian, RANSAC, and IRLS Polish. Plus needed transitive helpers from `homography_solver.h`, `cholesky_linear_solvers.h`, `rand.h`, `partial_sort.h`, `geometry.h`, `matrix.h`. Replaces Eigen's `.exp()` with a pure-Rust Padé(3,3) approximation (`mat3_exp_pade`).
- **Why it exists:** Completes Milestone 6 by providing the homography pipeline (`RobustHomography::new()` + `find()`) in pure Rust. Eliminates the only Eigen dependency in the eventual pure-Rust path. Builds on M6-1's math utilities and M6-2's `solve_null_vector_8x9_destructive` (the DLT core).
- **Who it is for:** WebARKitLib-rs core developers and downstream `webarkitlib-wasm` consumers building toward an Eigen-free, FFI-free pipeline.
- **Key constraints:**
  - Bit-equivalent to the C++ baseline: port the 32-bit LCG (`seed = 214013*seed + 2531011`) exactly so dual-mode RANSAC tests can compare element-wise within 1e-5 tolerance.
  - Follow project conventions: LGPL header, `#[inline(always)]`, `arlog_e!` (no `eprintln!`), no `CHANGELOG.md` edits, fresh branch from `feature/milestone6-kpm-math-port`.
  - Match M6-1/M6-2 stylistic patterns: hybrid generic/concrete signatures; comprehensive unit tests; dual-mode for live entrypoints.
- **Explicit non-goals:**
  - **Out of scope:** Removing Eigen from the C++ build (it stays until the FFI is fully retired in a later milestone — M6-3 only ensures the pure-Rust path doesn't need it). SIMD optimization. Refactoring `math.rs` into multiple files. Porting upstream functions outside the explicit deliverable list (e.g., `MultiplyPointSimilarityInhomogenous` has 36 callers but those are in detector code we don't recompile).
- **In scope:** Full `RobustHomography::find()` = RANSAC + IRLS Polish, including the Cauchy IRLS Jacobian (`cauchy_derivative`), the 8×8 Cholesky solver, the Lie Jacobian, and the LM regularizer.

## Assumptions

1. **PR target:** `feature/milestone6-kpm-math-port` (integration branch, same as M6-2). Branch `feature/milestone6-kpm-homography` already created.
2. **Submodule rev** stays at the current pinned version for the duration of M6-3.
3. **Tolerances:**
   - `mat3_exp_pade` fixture test: 1e-5 vs captured Eigen output.
   - `preemptive_robust_homography` dual-mode: 1e-5 element-wise.
   - `RobustHomography::find()` dual-mode: 1e-5 element-wise.
4. **Default constants** match C++: `cauchy_scale=0.01`, `num_hypotheses=1024`, `max_trials=1064`, `chunk_size=50`.
5. **`rand` crate** already in `[dependencies]` (from M6-2 dual-mode tests). No new crate dependencies.
6. **`mat3_exp_pade` fixture** input + expected output captured from a single reference C++ run; both committed as Rust `const` arrays.
7. **f32::EPSILON** for singularity thresholds (matches M6-2).

## Decision Log

| # | Decision | Alternatives | Rationale |
|---|----------|--------------|-----------|
| 1 | **Option A** — full pipeline (RANSAC + Polish) in one PR | B (RANSAC-only, defer Polish to M6-4); C (split into 3 commits) | One push to full C++ parity; no follow-up issue needed; ~1.5× M6-2 size but tractable |
| 2 | **Strategy A** — hybrid tests: fixture for `mat3_exp_pade`, dual-mode FFI for `preemptive_robust_homography` and `find()`, unit tests for everything else | B (JSON fixtures); C (constants only) | Matches M6-1/M6-2 review shape; bit-equivalence via LCG; no JSON dep |
| 3 | **Bit-equivalent RNG** — port the C++ 32-bit LCG exactly using `wrapping_mul/add` | Use Rust's `rand` crate (looser tolerance) | Enables strict element-wise dual-mode comparison (1e-5 instead of 1e-3) |
| 4 | **Helpers as private** (`fn`, no `pub`) | Public; inline | Minimal public surface; matches M6-2 |
| 5 | **Single `homography.rs`** file (~1700 LOC) | Split by topic | Literal spec compliance; matches M6-2's `math.rs` precedent |
| 6 | **Cauchy stack in `homography.rs`** | Place `cauchy_cost` in `math.rs` | Per #65 rescoping: tightly coupled to `RobustHomography` |
| 7 | **Hybrid signatures** — generic for trivial vector ops, concrete `f32` for math-heavy | All concrete; all generic | Matches M6-1/M6-2 |
| 8 | **`mat3_exp_pade` is f32-only**, Padé(3,3) with `p1=1/120, p2=1/10, p3=1/2`; uses 3×3 Gauss-elim (private helper) | Higher-order Padé; scaling-and-squaring | Coefficients per the user's spec; sl(3,R) inputs in homography stay in the radius of convergence for Padé(3,3) |
| 9 | **Cholesky for 8×8 only** — `solve_positive_definite_system_8x8(...)` | Generic-size Cholesky | Matches M6-2's pattern of concrete-size solvers; simpler API |
| 10 | **dual-mode FFI bridges** — 3 wrappers: `webarkit_cpp_mat3_exp_pade_via_eigen` (calls `eigenMat.exp()`), `webarkit_cpp_preemptive_robust_homography`, `webarkit_cpp_robust_homography_find` | Cover everything; cover only `find()` | Sufficient to validate the 3 algorithmically distinct steps; matches M6-1/M6-2 cadence |

---

## Final Design

### Section 1 — File layout & module structure

New file: `crates/core/src/kpm/freak/homography.rs` with internal section dividers:

```
1. License header
2. Module doc
3. Imports + re-exports from math.rs
4. Constants (defaults: cauchy_scale, num_hypotheses, etc.)
5. "homography.h Functions"
6. "robustifiers.h: Cauchy cost"
7. "Padé(3,3) matrix exponential"
8. "Lie algebra basis"
9. "DLT helpers"
10. "RANSAC RNG + utility"
11. "RANSAC: preemptive_robust_homography"
12. "IRLS Polish: Cauchy derivative + Lie Jac"
13. "IRLS Polish: normal equations + Cholesky + polish"
14. "RobustHomography struct"
15. #[cfg(test)] mod tests
16. extern "C" block (3 dual-mode FFI decls)
17. #[cfg(all(test, feature = "dual-mode"))] mod dual_mode_tests
```

Update `crates/core/src/kpm/freak/mod.rs`:
```rust
pub mod math;        // existing
pub mod homography;  // new
```

### Section 2 — Function signatures & API

#### Public API (~22 functions)

```rust
// homography.h
pub fn similarity<T>(h: &mut [T; 9], x: T, y: T, angle: T, scale: T)
pub fn similarity_2x2<T>(s: &mut [T; 4], angle: T, scale: T)
pub fn create_similarity_transformation_2d<T>(h: &mut [T; 9], cx: T, cy: T, angle: T, scale: T)
pub fn multiply_point_homography_inhomogenous<T>(xp: &mut [T; 2], h: &[T; 9], x: &[T; 2])
pub fn homography_points_geometrically_consistent(h: &[f32; 9], pts: &[f32], n: usize) -> bool
pub fn normalize_homography(h: &mut [f32; 9])

// robustifiers.h
pub fn cauchy_cost_scalar<T>(x: T, one_over_scale2: T) -> T
pub fn cauchy_cost_2d<T>(x: T, y: T, one_over_scale2: T) -> T
pub fn cauchy_cost<T>(x: &[T; 2], one_over_scale2: T) -> T
pub fn cauchy_projective_reprojection_cost(h: &[f32; 9], p: &[f32; 2], q: &[f32; 2], one_over_scale2: f32) -> f32
pub fn cauchy_projective_reprojection_cost_total(h: &[f32; 9], p: &[f32], q: &[f32], n: usize, one_over_scale2: f32) -> f32

// Padé(3,3) — Eigen replacement
pub fn mat3_exp_pade(m: &[f32; 9]) -> [f32; 9]

// Lie algebra
pub fn lie_algebra_sum<T>(a: &mut [T; 9], x: &[T; 8])
pub fn incremental_homography_from_lie_weights(h: &mut [f32; 9], x: &[f32; 8])
pub fn update_projective_motion_post_multiply(hp: &mut [f32; 9], h: &[f32; 9], x0: &[f32; 8])

// RANSAC + Polish entrypoints
pub fn preemptive_robust_homography(/* ... */) -> bool
pub fn polish_homography(/* ... */) -> bool

// Public class
pub struct RobustHomography { /* fields */ }
impl RobustHomography {
    pub fn new(...) -> Self
    pub fn find(...) -> bool
    pub fn find_with_test_points(...) -> bool
}
```

#### Private helpers (~14 fns)

`multiply_3x3_3x3`, `multiply_and_accumulate_at_a/at_x`, `symmetric_extend_upper_to_lower_8x8`, `fast_random`, `array_shuffle`, `fast_median`, `solve_homography_4_points`, `homography_3/4_points_geometrically_consistent`, `cauchy_derivative`, `homography_lie_jacobian`, `robust_homography_lie_jacobian_post_multiply`, `compute_homography_normal_equations_post_multiply`, `regularize_levenberg_marquardt_8x8`, `solve_positive_definite_system_8x8`.

### Section 3 — Algorithm fidelity notes

**`mat3_exp_pade`** — Padé(3,3) `exp(M) ≈ (V+U)·(V−U)⁻¹` where:
- `M2 = M·M`, `M3 = M2·M`
- `U = M·(p1·M2 + p3·I)` with `p1=1/120`, `p3=1/2`
- `V = p2·M2 + I` with `p2=1/10`
- Solve `(V−U)·X = (V+U)` via 3×3 Gaussian elimination (3 columns)
- Tolerance vs Eigen: 1e-5 for sl(3,R) inputs in homography range

**`fast_random` / `array_shuffle`** — bit-equivalent C++ LCG:
```rust
fn fast_random(seed: &mut i32) -> i32 {
    *seed = (214013i32).wrapping_mul(*seed).wrapping_add(2531011);
    (*seed >> 16) & 0x7FFF
}
```

**`solve_positive_definite_system_8x8`** — standard Cholesky `A = L·Lᵀ` then forward+back substitution. Specialized to 8×8 (only call site).

### Section 4 — Test strategy

**Unit tests (~25):**

| Category | Approach | Tolerance |
|----------|----------|-----------|
| Homography primitives | Hand-computed | Exact / 1e-5 |
| Geometric checks | Known consistent + degenerate inputs | Exact (bool) |
| Cauchy stack | x=0 → cost=0; reference values | 1e-6 |
| `mat3_exp_pade` (fixture) | Hardcoded constants from one C++ Eigen run | 1e-5 |
| Lie functions | Hand-computed | 1e-5 |
| RANSAC RNG | Reference C++ output for seed=1234, first 100 calls | Exact (i32) |
| DLT helpers | 4-point identity correspondences | 1e-4 |
| Cholesky | A = M·Mᵀ + λI random; verify A·x = b | 1e-5 |
| IRLS Jacobians | Numerical diff cross-check | 1e-3 |

**Dual-mode tests (2):**
- `preemptive_robust_homography_matches_cpp` — random correspondences, seeded; element-wise 1e-5
- `robust_homography_find_matches_cpp` — same; element-wise 1e-5 (full RANSAC + Polish)

**FFI bridges (3) in `kpm_c_api.cpp/h`:**
```cpp
float webarkit_cpp_mat3_exp_pade_via_eigen(float out[9], const float in[9]);
int webarkit_cpp_preemptive_robust_homography(...);
int webarkit_cpp_robust_homography_find(...);
```

### Section 5 — Doc-comment template (every public fn)

```rust
/// One-line summary.
///
/// Detailed math description (formula, what it computes).
///
/// # Arguments
/// * `h` — output 3×3 homography in row-major layout
///
/// # Returns
/// `true` on success, `false` if … (logged via `arlog_e!`).
///
/// # C++ equivalent
/// `vision::FunctionName<T>` from `<header>:<line>`. Live at `<caller>:<line>`
/// in upstream FreakMatcher pipeline (or notes dead-code status). Validated
/// bit-equivalent against C++ via dual-mode FFI test `<test_name>`.
///
/// # Example
/// ```ignore
/// let mut h = [0.0_f32; 9];
/// similarity(&mut h, 0.0, 0.0, 0.0, 1.0);
/// ```
```

### Section 6 — Implementation order

**Commit 1 (port + unit tests):**
1. Module setup (header, doc, imports, mod.rs update)
2. homography.h primitives + tests
3. Cauchy stack + tests
4. `mat3_exp_pade` + fixture test (capture C++ Eigen output once)
5. Lie functions + tests
6. DLT helpers + tests
7. RANSAC RNG (LCG-equivalent) + reference-value test
8. `fast_median` + test
9. `preemptive_robust_homography` + analytical test
10. IRLS chain (Cauchy deriv → Lie Jac → normal eq → LM → Cholesky → polish) + tests
11. `RobustHomography` struct + new + find + find_with_test_points + tests
12. fmt + clippy + tests

**Commit 2 (dual-mode):**
1. 3 FFI declarations to `kpm_c_api.h`
2. 3 thin wrappers to `kpm_c_api.cpp`
3. 3 Rust extern + 2 dual-mode tests
4. fmt + clippy + tests with `--features dual-mode`

### Section 7 — Pre-PR checklist

- [ ] `cargo fmt --all -- --check` clean
- [ ] `cargo build -p webarkitlib-rs --all-features` succeeds
- [ ] `cargo clippy --workspace -- -D warnings` clean (matches CI exactly)
- [ ] `cargo test -p webarkitlib-rs -- kpm::freak::homography` ✓
- [ ] `cargo test -p webarkitlib-rs --features dual-mode -- kpm::freak::homography` ✓
- [ ] LGPL-3.0 header on `homography.rs`
- [ ] No `println!`/`eprintln!` (uses `arlog_e!` only)
- [ ] No `CHANGELOG.md` edits
- [ ] Every public function has the standard 5-section doc template
- [ ] PR target: `feature/milestone6-kpm-math-port`
- [ ] PR title: `feat(kpm): port homography pipeline + Padé matrix exp (M6-3)`

---

## Risks & Mitigations

| Risk | Mitigation |
|------|-----------|
| `mat3_exp_pade` precision drift | Fixture test against captured Eigen output; Padé(3,3) has well-characterized error for small ‖M‖ |
| RNG bit-divergence | `wrapping_mul`/`wrapping_add` in `fast_random`; reference-value test against captured C++ output |
| Polish step convergence differs from C++ | Same LCG → same RANSAC seed → same initial H; identical FP ops in same order → bit-equivalent within accumulated rounding (1e-5) |
| `mat3_exp_pade` fixture is captured once and may go stale | Comment in fixture noting capture conditions; Padé replaces Eigen, so it's stable |
| 8×8 Cholesky catastrophic cancellation on near-singular SPD | LM regularizer (`reg_JtJ = JtJ + λ·diag(JtJ)`) keeps system well-conditioned; matches C++ |
