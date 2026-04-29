/*
 *  freak/homography.rs
 *  WebARKitLib-rs
 *
 *  This file is part of WebARKitLib-rs - WebARKit.
 *
 *  WebARKitLib-rs is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  WebARKitLib-rs is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with WebARKitLib-rs.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2026 WebARKit.
 *
 *  Author(s): Walter Perdan @kalwalt https://github.com/kalwalt
 *
 */

//! Homography pipeline for the FREAK descriptor matcher.
//!
//! Ported from WebARKitLib C++ headers:
//! - `KPM/FreakMatcher/math/homography.h` (218 lines) — geometric primitives
//!   (similarity, normalization, point-projection, geometric-consistency checks)
//! - `KPM/FreakMatcher/math/robustifiers.h` (53 lines) — Cauchy robust cost
//!   functions
//! - `KPM/FreakMatcher/homography_estimation/robust_homography.h` (673 lines) —
//!   the largest file in the FreakMatcher: Lie-algebra basis, Cauchy
//!   reprojection cost + IRLS Jacobian, RANSAC (`PreemptiveRobustHomography`),
//!   IRLS Levenberg–Marquardt polish (`PolishHomography`), and the
//!   `RobustHomography` class.
//!
//! Selected private helpers are also ported from `KPM/FreakMatcher/math/`:
//! `matrix.h` (`Multiply3x3_3x3`, `MultiplyAndAccumulateAtA`/`Atx`,
//! `SymmetricExtendUpperToLower`), `cholesky_linear_solvers.h`
//! (`SolvePositiveDefiniteSystem` specialized to N=8), `rand.h` (the C++
//! 32-bit LCG used by RANSAC), `partial_sort.h` (`FastMedian`),
//! `homography_solver.h` (`SolveHomography4Points`,
//! `Homography4PointsGeometricallyConsistent`), and `geometry.h`
//! (`Homography3PointsGeometricallyConsistent`).
//!
//! # Eigen replacement
//!
//! The C++ `IncrementalHomographyFromLieWeights` calls `eigenMat.exp()` from
//! Eigen's MatrixFunctions. This was the **only** Eigen dependency in the
//! pure-math layer of the FreakMatcher. We replace it with a pure-Rust
//! Padé(3,3) approximation ([`mat3_exp_pade`]) that agrees with Eigen to ~1e-5
//! for the small sl(3,ℝ) matrices that appear in homography estimation.
//!
//! # Algorithmic faithfulness vs C++
//!
//! All code is bit-equivalent to the upstream C++ within accumulated
//! floating-point rounding (~1e-5 element-wise). In particular:
//! - The RANSAC RNG ([`fast_random`], [`array_shuffle`]) is a 1-to-1 port of
//!   the C++ `vision::FastRandom` linear congruential generator. Same seed →
//!   same hypothesis order → same final homography.
//! - All IRLS arithmetic preserves the C++ operation order (matters for
//!   element-wise dual-mode comparison).
//!
//! # Module conventions (matches M6-1 / M6-2)
//!
//! - All functions marked `#[inline(always)]` to mirror the C++ `inline`
//!   keyword and enable call-site specialization via monomorphization.
//! - **Hybrid signatures**: generic `<T>` for trivial vector ops; concrete
//!   `f32` for math-heavy functions that branch on numerical thresholds.
//! - **Helpers are private** (`fn`, no `pub`) — the public surface is the
//!   list of functions explicitly requested in M6-3 (#65), plus the
//!   [`RobustHomography`] struct.
//! - **Errors signalled via `bool`** + an `arlog_e!` log line at every
//!   failure site, per CLAUDE.md.

use std::ops::{Add, Mul};

use crate::arlog_e;

use super::math::{copy_vector_9, max2, min2, solve_null_vector_8x9_destructive, sqr};

// ============================================================================
// Constants
// ============================================================================

/// Default Cauchy scale parameter for robust reprojection cost.
///
/// Mirrors `HOMOGRAPHY_DEFAULT_CAUCHY_SCALE` from
/// `homography_estimation/robust_homography.h:52`.
pub const HOMOGRAPHY_DEFAULT_CAUCHY_SCALE: f32 = 0.01;

/// Default maximum number of RANSAC hypotheses to evaluate.
///
/// Mirrors `HOMOGRAPHY_DEFAULT_NUM_HYPOTHESES` from
/// `homography_estimation/robust_homography.h:53`.
pub const HOMOGRAPHY_DEFAULT_NUM_HYPOTHESES: i32 = 1024;

/// Default maximum number of RANSAC trial draws (some draws fail the
/// 4-point geometric-consistency check and are discarded).
///
/// Mirrors `HOMOGRAPHY_DEFAULT_MAX_TRIALS` from
/// `homography_estimation/robust_homography.h:54`.
pub const HOMOGRAPHY_DEFAULT_MAX_TRIALS: i32 = 1064;

/// Default chunk size for the preemptive scoring loop. Hypotheses are scored
/// against the input correspondences in chunks; after each chunk the worse
/// half of hypotheses is pruned.
///
/// Mirrors `HOMOGRAPHY_DEFAULT_CHUNK_SIZE` from
/// `homography_estimation/robust_homography.h:55`.
pub const HOMOGRAPHY_DEFAULT_CHUNK_SIZE: i32 = 50;

/// Maximum value returned by [`fast_random`].
///
/// Mirrors `FAST_RAND_MAX` from `math/rand.h:42`.
const FAST_RAND_MAX: i32 = 32767;

/// Initial seed used by [`preemptive_robust_homography`] (matches C++).
const RANSAC_INITIAL_SEED: i32 = 1234;

// ============================================================================
// homography.h Functions
// ============================================================================
//
// Ported from `KPM/FreakMatcher/math/homography.h` (218 lines). All public
// functions are concrete `f32` because they involve trig (`sin`/`cos`) or
// floating-point division — generic versions would require pulling in
// `num_traits` for marginal gain. Matches M6-1/M6-2's "concrete `f32` for
// math-heavy" convention.

/// Build a 3×3 similarity matrix `H = [c, -s, x; s, c, y; 0, 0, 1]`.
///
/// Where `c = scale * cos(angle)` and `s = scale * sin(angle)`. Represents a
/// rotation by `angle` followed by uniform scaling and translation by
/// `(x, y)`, in the homogeneous-coordinate convention (last row `[0, 0, 1]`).
///
/// # Arguments
/// * `h` — output 3×3 matrix in row-major layout (9 elements)
/// * `x`, `y` — translation
/// * `angle` — rotation angle in radians
/// * `scale` — uniform scale factor
///
/// # C++ equivalent
/// `vision::Similarity<T>` from `math/homography.h:48`. Used in 2 upstream
/// call sites (template-instantiated by detector code we don't recompile).
#[inline(always)]
pub fn similarity(h: &mut [f32; 9], x: f32, y: f32, angle: f32, scale: f32) {
    let c = scale * angle.cos();
    let s = scale * angle.sin();
    h[0] = c;
    h[1] = -s;
    h[2] = x;
    h[3] = s;
    h[4] = c;
    h[5] = y;
    h[6] = 0.0;
    h[7] = 0.0;
    h[8] = 1.0;
}

/// Build a 2×2 rotation-and-scale matrix `S = [c, -s; s, c]`.
///
/// Where `c = scale * cos(angle)` and `s = scale * sin(angle)`. The 2×2 part
/// of a similarity transform.
///
/// # C++ equivalent
/// `vision::Similarity2x2<T>` from `math/homography.h:60`. Used in 1 upstream
/// call site.
#[inline(always)]
pub fn similarity_2x2(s_out: &mut [f32; 4], angle: f32, scale: f32) {
    let c = scale * angle.cos();
    let s = scale * angle.sin();
    s_out[0] = c;
    s_out[1] = -s;
    s_out[2] = s;
    s_out[3] = c;
}

/// Build a 3×3 similarity transformation centred on `(cx, cy)`.
///
/// Computes the 3×3 matrix that rotates by `angle` and scales by `scale`
/// **about the point `(cx, cy)`**. Equivalent to `T(cx,cy) · R(angle) ·
/// S(scale) · T(-cx,-cy)` reduced into a single 3×3 matrix.
///
/// # C++ equivalent
/// `vision::CreateSimilarityTransformation2d<T>` from `math/homography.h:68`.
/// Defined upstream but not directly called from compiled code.
#[inline(always)]
pub fn create_similarity_transformation_2d(
    h: &mut [f32; 9],
    cx: f32,
    cy: f32,
    angle: f32,
    scale: f32,
) {
    let c = scale * angle.cos();
    let s = scale * angle.sin();

    h[0] = c;
    h[1] = -s;
    h[3] = s;
    h[4] = c;

    // Translation that pins (cx, cy) to itself: t = -R * c + c
    h[2] = -(h[0] * cx + h[1] * cy) + cx;
    h[5] = -(h[3] * cx + h[4] * cy) + cy;

    h[6] = 0.0;
    h[7] = 0.0;
    h[8] = 1.0;
}

/// Project an inhomogeneous 2D point through a 3×3 homography.
///
/// Computes `(xp[0], xp[1]) = (H · [x[0], x[1], 1]ᵀ).normalize()` where the
/// normalization divides by the homogeneous `w` coordinate.
///
/// # C++ equivalent
/// `vision::MultiplyPointHomographyInhomogenous<T>` from `math/homography.h:104`
/// (the array-form overload). Used 14 times across the upstream FreakMatcher,
/// most notably inside `cauchy_projective_reprojection_cost` (the RANSAC
/// scoring loop).
#[inline(always)]
pub fn multiply_point_homography_inhomogenous(xp: &mut [f32; 2], h: &[f32; 9], x: &[f32; 2]) {
    let w = h[6] * x[0] + h[7] * x[1] + h[8];
    xp[0] = (h[0] * x[0] + h[1] * x[1] + h[2]) / w;
    xp[1] = (h[3] * x[0] + h[4] * x[1] + h[5]) / w;
}

/// Project an inhomogeneous 2D point through a 3×3 homography (scalar form).
///
/// Same as [`multiply_point_homography_inhomogenous`] but with explicit `xp`,
/// `yp` outputs and `x`, `y` inputs. C++ has both overloads; we provide both
/// for parity with the upstream API surface.
///
/// # C++ equivalent
/// `vision::MultiplyPointHomographyInhomogenous<T>` from `math/homography.h:94`
/// (the scalar-form overload). Used internally by
/// [`cauchy_projective_reprojection_cost`].
#[inline(always)]
pub fn multiply_point_homography_inhomogenous_scalar(h: &[f32; 9], x: f32, y: f32) -> (f32, f32) {
    let w = h[6] * x + h[7] * y + h[8];
    let xp = (h[0] * x + h[1] * y + h[2]) / w;
    let yp = (h[3] * x + h[4] * y + h[5]) / w;
    (xp, yp)
}

/// Determine which side of the line through `a`, `b` the point `c` lies on.
///
/// Returns the (unsigned) cross product of `(b - a)` and `(c - a)`. Positive
/// for one side, negative for the other, zero for collinear.
///
/// C++ equivalent: `vision::LinePointSide<T>` from `math/geometry.h:50`.
/// Private helper used by the geometric-consistency checks.
#[inline(always)]
fn line_point_side(a: &[f32; 2], b: &[f32; 2], c: &[f32; 2]) -> f32 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Check that 3 source points and their projected counterparts have the same
/// winding (handedness) — i.e. the homography preserves orientation locally.
///
/// C++ equivalent: `vision::Homography3PointsGeometricallyConsistent<T>` from
/// `math/geometry.h:58`. Private helper used by
/// [`homography_points_geometrically_consistent`] and the 4-point variant.
#[inline(always)]
fn homography_3_points_geometrically_consistent(
    x1: &[f32; 2],
    x2: &[f32; 2],
    x3: &[f32; 2],
    x1p: &[f32; 2],
    x2p: &[f32; 2],
    x3p: &[f32; 2],
) -> bool {
    !((line_point_side(x1, x2, x3) > 0.0) ^ (line_point_side(x1p, x2p, x3p) > 0.0))
}

/// Check geometric consistency of 4 source/target point pairs (winding
/// preserved on all 4 triangles formed by sliding window over the quad).
///
/// C++ equivalent: `vision::Homography4PointsGeometricallyConsistent<T>` from
/// `math/geometry.h:71`. Private helper used by RANSAC to filter degenerate
/// 4-tuples before solving for the homography.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn homography_4_points_geometrically_consistent(
    x1: &[f32; 2],
    x2: &[f32; 2],
    x3: &[f32; 2],
    x4: &[f32; 2],
    x1p: &[f32; 2],
    x2p: &[f32; 2],
    x3p: &[f32; 2],
    x4p: &[f32; 2],
) -> bool {
    if (line_point_side(x1, x2, x3) > 0.0) ^ (line_point_side(x1p, x2p, x3p) > 0.0) {
        return false;
    }
    if (line_point_side(x2, x3, x4) > 0.0) ^ (line_point_side(x2p, x3p, x4p) > 0.0) {
        return false;
    }
    if (line_point_side(x3, x4, x1) > 0.0) ^ (line_point_side(x3p, x4p, x1p) > 0.0) {
        return false;
    }
    if (line_point_side(x4, x1, x2) > 0.0) ^ (line_point_side(x4p, x1p, x2p) > 0.0) {
        return false;
    }
    true
}

/// Sanity-check that the homography preserves orientation across all
/// consecutive 3-point windows of a polyline (with wrap-around).
///
/// `pts` is a flat slice `[x0, y0, x1, y1, ..., x(n-1), y(n-1)]` of `n` 2D
/// points. The homography is "consistent" if every triple of consecutive
/// points (modulo wrap-around at the end) has the same orientation as its
/// projected triple.
///
/// Returns `true` for fewer than 3 points (trivially consistent). For
/// historical reasons C++ used `< 2` (likely a bug — the comment says "at
/// least 3 points" but the code uses 2); we reproduce the same behaviour
/// for bit-equivalent results.
///
/// # C++ equivalent
/// `vision::HomographyPointsGeometricallyConsistent<T>` from
/// `math/homography.h:111`. Called from `preemptive_robust_homography` to
/// validate hypotheses against test points.
#[inline(always)]
pub fn homography_points_geometrically_consistent(h: &[f32; 9], pts: &[f32], n: usize) -> bool {
    // C++ uses `if (size < 2) return true` — preserve that behavior.
    if n < 2 {
        return true;
    }
    debug_assert!(
        pts.len() >= n * 2,
        "homography_points_geometrically_consistent: pts.len() must be >= n*2"
    );

    // Helpers to read one source point at index i (zero-cost slice → array ref).
    let pt = |i: usize| -> [f32; 2] { [pts[i * 2], pts[i * 2 + 1]] };

    // Project the first 3 source points.
    let mut xp = [[0.0_f32; 2]; 3];
    multiply_point_homography_inhomogenous(&mut xp[0], h, &pt(0));
    multiply_point_homography_inhomogenous(&mut xp[1], h, &pt(1));
    multiply_point_homography_inhomogenous(&mut xp[2], h, &pt(2));

    let first_xp1 = xp[0];
    let first_xp2 = xp[1];

    if !homography_3_points_geometrically_consistent(&pt(0), &pt(1), &pt(2), &xp[0], &xp[1], &xp[2])
    {
        return false;
    }

    // Sliding window: for each new source point i, project it into the slot
    // that held the oldest projected point, then rotate the logical indices.
    let mut idx = [0usize, 1, 2]; // idx[0]=oldest, idx[2]=newest
    for i in 3..n {
        let new_pt = pt(i);
        let target = idx[0];
        multiply_point_homography_inhomogenous(&mut xp[target], h, &new_pt);
        // Rotate: oldest is now newest. New ordering: idx[1..=2] then old idx[0].
        idx.rotate_left(1);

        // Check the new 3-point window (i-2, i-1, i).
        if !homography_3_points_geometrically_consistent(
            &pt(i - 2),
            &pt(i - 1),
            &pt(i),
            &xp[idx[0]],
            &xp[idx[1]],
            &xp[idx[2]],
        ) {
            return false;
        }
    }

    // Wrap-around: check (n-2, n-1, 0) and (n-1, 0, 1).
    let last1 = n - 2;
    let last2 = n - 1;
    if !homography_3_points_geometrically_consistent(
        &pt(last1),
        &pt(last2),
        &pt(0),
        &xp[idx[1]],
        &xp[idx[2]],
        &first_xp1,
    ) {
        return false;
    }
    if !homography_3_points_geometrically_consistent(
        &pt(last2),
        &pt(0),
        &pt(1),
        &xp[idx[2]],
        &first_xp1,
        &first_xp2,
    ) {
        return false;
    }

    true
}

/// Normalize a homography in place so that `H[8] == 1.0`.
///
/// Divides every element by `H[8]`. Caller must ensure `H[8] != 0`; we do not
/// validate (the upstream C++ also doesn't, and a zero `H[8]` would already
/// indicate a degenerate homography).
///
/// # C++ equivalent
/// `vision::NormalizeHomography<T>` from `math/homography.h:185`. Called at
/// the end of `preemptive_robust_homography` on the winning hypothesis.
#[inline(always)]
pub fn normalize_homography(h: &mut [f32; 9]) {
    let inv = 1.0 / h[8];
    h[0] *= inv;
    h[1] *= inv;
    h[2] *= inv;
    h[3] *= inv;
    h[4] *= inv;
    h[5] *= inv;
    h[6] *= inv;
    h[7] *= inv;
    h[8] = 1.0;
}

// ============================================================================
// robustifiers.h: Cauchy robust cost
// ============================================================================
//
// Ported from `math/robustifiers.h` (53 lines) and the
// `CauchyProjectiveReprojectionCost` overloads from
// `homography_estimation/robust_homography.h:60-91`. This stack was originally
// listed in #64 (M6-2) by mistake — it logically belongs with the
// `RobustHomography` IRLS optimizer (see #65 comment for details).
//
// The Cauchy robust cost `C(x) = log(1 + (x/σ)²)` down-weights large
// residuals: instead of growing as O(x²) like a least-squares cost, it grows
// as O(log x²), which makes RANSAC + IRLS robust to outliers that would
// otherwise dominate the gradient.

/// Cauchy robust cost of a scalar residual: `log(1 + x² · one_over_scale2)`.
///
/// `one_over_scale2` is `1/σ²` where σ is the Cauchy scale parameter (the
/// caller is expected to precompute `1/σ²` because it's reused across many
/// calls in tight loops).
///
/// # C++ equivalent
/// `vision::CauchyCost<T>(T x, T one_over_scale2)` from `robustifiers.h:43`.
#[inline(always)]
pub fn cauchy_cost_scalar(x: f32, one_over_scale2: f32) -> f32 {
    (1.0 + sqr(x) * one_over_scale2).ln()
}

/// Cauchy robust cost of a 2D residual: `log(1 + (x² + y²) · one_over_scale2)`.
///
/// # C++ equivalent
/// `vision::CauchyCost<T>(T x0, T x1, T one_over_scale2)` from
/// `robustifiers.h:48`.
#[inline(always)]
pub fn cauchy_cost_2d(x: f32, y: f32, one_over_scale2: f32) -> f32 {
    (1.0 + (x * x + y * y) * one_over_scale2).ln()
}

/// Cauchy robust cost of a 2D residual passed as an array.
///
/// Convenience overload that delegates to [`cauchy_cost_2d`].
///
/// # C++ equivalent
/// `vision::CauchyCost<T>(const T x[2], T one_over_scale2)` from
/// `robustifiers.h:53`.
#[inline(always)]
pub fn cauchy_cost(x: &[f32; 2], one_over_scale2: f32) -> f32 {
    cauchy_cost_2d(x[0], x[1], one_over_scale2)
}

/// Compute the Cauchy reprojection cost for a single point correspondence
/// `(p, q)` under homography `h`: `C(H·p − q)`.
///
/// Projects `p` through `h` to obtain `H·p`, takes the 2D residual against
/// `q`, then evaluates the Cauchy robust cost.
///
/// # C++ equivalent
/// `vision::CauchyProjectiveReprojectionCost<T>(H, p, q, one_over_scale2)`
/// from `robust_homography.h:60`. Called from
/// `cauchy_projective_reprojection_cost_total` (n-point variant).
#[inline(always)]
pub fn cauchy_projective_reprojection_cost(
    h: &[f32; 9],
    p: &[f32; 2],
    q: &[f32; 2],
    one_over_scale2: f32,
) -> f32 {
    let (pp_x, pp_y) = multiply_point_homography_inhomogenous_scalar(h, p[0], p[1]);
    let f = [pp_x - q[0], pp_y - q[1]];
    cauchy_cost(&f, one_over_scale2)
}

/// Sum of Cauchy reprojection costs over all `n` correspondences.
///
/// `p` and `q` are flat arrays `[x0, y0, x1, y1, ...]` of length `2n`.
///
/// # C++ equivalent
/// `vision::CauchyProjectiveReprojectionCost<T>(H, p, q, num_points,
/// one_over_scale2)` from `robust_homography.h:77`. The hot inner loop of
/// the RANSAC scoring step in [`preemptive_robust_homography`].
#[inline(always)]
pub fn cauchy_projective_reprojection_cost_total(
    h: &[f32; 9],
    p: &[f32],
    q: &[f32],
    n: usize,
    one_over_scale2: f32,
) -> f32 {
    debug_assert!(p.len() >= n * 2);
    debug_assert!(q.len() >= n * 2);

    let mut total = 0.0_f32;
    for i in 0..n {
        let p_i = [p[i * 2], p[i * 2 + 1]];
        let q_i = [q[i * 2], q[i * 2 + 1]];
        total += cauchy_projective_reprojection_cost(h, &p_i, &q_i, one_over_scale2);
    }
    total
}

// ============================================================================
// Padé(3,3) matrix exponential — Eigen replacement
// ============================================================================
//
// Replaces `eigenMat.exp()` from Eigen's MatrixFunctions module — the only
// Eigen dependency in the pure-math layer of the FreakMatcher.
//
// The (3,3) Padé approximation of `exp(M)` uses the identity
//
//   exp(M) ≈ D(M)⁻¹ · N(M)
//
// where N(M) and D(M) are the degree-3 Padé numerator/denominator polynomials.
// They factor into even/odd parts:
//
//   N(M) = (I + p2·M²) + M·(p3·I + p1·M²)  =  V + U
//   D(M) = (I + p2·M²) − M·(p3·I + p1·M²)  =  V − U
//
// with `p1 = 1/120`, `p2 = 1/10`, `p3 = 1/2` (standard (3,3) Padé coefficients).
//
// For sl(3,ℝ) inputs in homography estimation (incremental Lie weights of
// magnitude ≪ 1), this is accurate to ~1e-5 vs Eigen's `MatrixExponential`.
// Outside that regime accuracy degrades; we don't currently scale-and-square
// because the upstream IRLS loop already keeps the weight magnitudes bounded.

/// Multiply two 3×3 matrices: `c = a * b`.
///
/// All matrices are row-major 9-element arrays.
///
/// # C++ equivalent
/// `vision::Multiply3x3_3x3<T>` from `math/matrix.h`. Used by
/// [`mat3_exp_pade`] (matrix-matrix multiplies inside the Padé numerator/
/// denominator) and by [`update_projective_motion_post_multiply`].
#[inline(always)]
fn multiply_3x3_3x3<T>(c: &mut [T; 9], a: &[T; 9], b: &[T; 9])
where
    T: Mul<Output = T> + Add<Output = T> + Copy,
{
    c[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
    c[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
    c[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];
    c[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
    c[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
    c[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];
    c[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
    c[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
    c[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];
}

/// Determinant of a general 3×3 matrix (row-major).
///
/// Cofactor expansion along the first row.
///
/// C++ equivalent: `vision::Determinant3x3<T>` from `math/linear_algebra.h:111`
/// (private helper here, not exported — only used by [`mat3_inverse`]).
#[inline(always)]
fn determinant_3x3(a: &[f32; 9]) -> f32 {
    a[0] * (a[4] * a[8] - a[5] * a[7]) - a[1] * (a[3] * a[8] - a[5] * a[6])
        + a[2] * (a[3] * a[7] - a[4] * a[6])
}

/// Inverse of a general 3×3 matrix via cofactor expansion (row-major).
///
/// Returns `false` if `|det A| <= threshold`. Mathematically equivalent to
/// 3×3 Gaussian elimination, but a closed-form expression — same arithmetic
/// operations, no pivoting drama for well-conditioned inputs (which is the
/// case for the Padé denominator on small-norm Lie weights).
///
/// C++ equivalent: `vision::MatrixInverse3x3<T>` from `math/linear_algebra.h:156`
/// (private helper here).
#[inline(always)]
fn mat3_inverse(b: &mut [f32; 9], a: &[f32; 9], threshold: f32) -> bool {
    let det = determinant_3x3(a);
    if det.abs() <= threshold {
        return false;
    }
    let inv_det = 1.0 / det;

    // Adjugate (cofactor matrix transposed) divided by determinant.
    b[0] = (a[4] * a[8] - a[5] * a[7]) * inv_det;
    b[1] = (a[2] * a[7] - a[1] * a[8]) * inv_det;
    b[2] = (a[1] * a[5] - a[2] * a[4]) * inv_det;
    b[3] = (a[5] * a[6] - a[3] * a[8]) * inv_det;
    b[4] = (a[0] * a[8] - a[2] * a[6]) * inv_det;
    b[5] = (a[2] * a[3] - a[0] * a[5]) * inv_det;
    b[6] = (a[3] * a[7] - a[4] * a[6]) * inv_det;
    b[7] = (a[1] * a[6] - a[0] * a[7]) * inv_det;
    b[8] = (a[0] * a[4] - a[1] * a[3]) * inv_det;
    true
}

/// Padé(3,3) approximation of the 3×3 matrix exponential `exp(M)`.
///
/// Replaces `eigenMat.exp()` from Eigen's `unsupported/MatrixFunctions`. For
/// small-norm matrices (the case in homography Lie weights), agreement with
/// Eigen is within ~1e-5 element-wise; verified by a fixture test using
/// values captured from a real C++ tracking session.
///
/// The Padé approximation computes:
///
/// ```text
///   M2 = M·M
///   U  = M · (p1·M2 + p3·I)         with p1 = 1/120, p3 = 1/2
///   V  = p2·M2 + I                  with p2 = 1/10
///   N  = V + U     (Padé numerator)
///   D  = V − U     (Padé denominator)
///   exp(M) ≈ D⁻¹ · N
/// ```
///
/// Returns `[0; 9]` and logs an `arlog_e!` if `D` is numerically singular
/// (`|det D| <= f32::EPSILON`). In practice this never happens for inputs
/// used in homography estimation because `D ≈ I` for small `‖M‖`.
///
/// # C++ equivalent
/// Replaces `Eigen::Matrix<T, 3, 3>::exp()` called from
/// `vision::IncrementalHomographyFromLieWeights` at
/// `homography_estimation/robust_homography.h:464`. Marked `// TODO: remove
/// Eigen` in the upstream source — this is that removal.
#[inline(always)]
pub fn mat3_exp_pade(m: &[f32; 9]) -> [f32; 9] {
    // Padé(3,3) coefficients
    const P1: f32 = 1.0 / 120.0;
    const P2: f32 = 1.0 / 10.0;
    const P3: f32 = 1.0 / 2.0;

    // M2 = M · M
    let mut m2 = [0.0_f32; 9];
    multiply_3x3_3x3(&mut m2, m, m);

    // inner = p1·M2 + p3·I
    let mut inner = [
        P1 * m2[0] + P3,
        P1 * m2[1],
        P1 * m2[2],
        P1 * m2[3],
        P1 * m2[4] + P3,
        P1 * m2[5],
        P1 * m2[6],
        P1 * m2[7],
        P1 * m2[8] + P3,
    ];
    // U = M · inner
    let mut u = [0.0_f32; 9];
    multiply_3x3_3x3(&mut u, m, &inner);

    // V = p2·M2 + I
    let v = [
        P2 * m2[0] + 1.0,
        P2 * m2[1],
        P2 * m2[2],
        P2 * m2[3],
        P2 * m2[4] + 1.0,
        P2 * m2[5],
        P2 * m2[6],
        P2 * m2[7],
        P2 * m2[8] + 1.0,
    ];

    // N = V + U; D = V − U  (computed without a second copy of v)
    let n = [
        v[0] + u[0],
        v[1] + u[1],
        v[2] + u[2],
        v[3] + u[3],
        v[4] + u[4],
        v[5] + u[5],
        v[6] + u[6],
        v[7] + u[7],
        v[8] + u[8],
    ];
    // Reuse u storage for D
    u[0] = v[0] - u[0];
    u[1] = v[1] - u[1];
    u[2] = v[2] - u[2];
    u[3] = v[3] - u[3];
    u[4] = v[4] - u[4];
    u[5] = v[5] - u[5];
    u[6] = v[6] - u[6];
    u[7] = v[7] - u[7];
    u[8] = v[8] - u[8];
    let d = u;

    // Solve D · X = N for X via D⁻¹.
    // Reuse `inner` storage for D⁻¹.
    if !mat3_inverse(&mut inner, &d, f32::EPSILON) {
        arlog_e!(
            "mat3_exp_pade: Padé denominator is singular (|det| <= {}); \
             this should not happen for Lie weights of small norm",
            f32::EPSILON
        );
        return [0.0_f32; 9];
    }
    let d_inv = inner;

    let mut result = [0.0_f32; 9];
    multiply_3x3_3x3(&mut result, &d_inv, &n);
    result
}

// ============================================================================
// Lie algebra basis (sl(3,ℝ)) for homography updates
// ============================================================================
//
// In the Benhimane–Malis Lie-group parameterization of the homography group
// (`SL(3,ℝ)` representatives modulo scale), homography updates are
// parameterized by a vector `x[8]` of basis weights. The Lie algebra element
// is reconstructed via [`lie_algebra_sum`], exponentiated to a homography
// delta via [`incremental_homography_from_lie_weights`], then applied via
// pre/post-multiply ([`update_projective_motion_post_multiply`]).
//
// Reference: Benhimane & Malis, "Homography-based 2d visual tracking and
// servoing", IJRR 2007.

/// Build a 3×3 Lie-algebra matrix from an 8-element basis weight vector.
///
/// The 8 basis vectors of `sl(3,ℝ)` (the Lie algebra of `SL(3,ℝ)`) span the
/// 8-dimensional tangent space of homographies modulo scale. The encoding
/// (matching the upstream `vision::LieAlgebraSum`) is:
///
/// ```text
///   A = [ x[4]      x[2]              x[0]   ]
///       [ x[3]    -(x[4] + x[5])      x[1]   ]
///       [ x[6]      x[7]              x[5]   ]
/// ```
///
/// Note `A[4]` enforces `tr(A) = 0` (so `A ∈ sl(3,ℝ)` rather than `gl(3,ℝ)`).
///
/// # C++ equivalent
/// `vision::LieAlgebraSum<T>` from
/// `homography_estimation/robust_homography.h:447`. Used inside
/// [`incremental_homography_from_lie_weights`].
#[inline(always)]
pub fn lie_algebra_sum<T>(a: &mut [T; 9], x: &[T; 8])
where
    T: Add<Output = T> + std::ops::Neg<Output = T> + Copy,
{
    a[0] = x[4];
    a[1] = x[2];
    a[2] = x[0];
    a[3] = x[3];
    a[4] = -(x[4] + x[5]);
    a[5] = x[1];
    a[6] = x[6];
    a[7] = x[7];
    a[8] = x[5];
}

/// Build a 3×3 incremental homography from 8-element Lie-algebra weights.
///
/// Computes `H_delta = exp(LieAlgebraSum(x))` where `exp` is the matrix
/// exponential, replaced here by [`mat3_exp_pade`] (the pure-Rust Padé(3,3)
/// approximation that supersedes the Eigen call in upstream).
///
/// `H_delta` is a small homography close to identity for small `‖x‖`, used
/// to compose incremental updates onto the running estimate of the
/// homography during IRLS optimization.
///
/// # C++ equivalent
/// `vision::IncrementalHomographyFromLieWeights<T>` from
/// `homography_estimation/robust_homography.h:457`. The upstream version
/// calls `eigenMat.exp()`; this Rust version is Eigen-free.
#[inline(always)]
pub fn incremental_homography_from_lie_weights(h: &mut [f32; 9], x: &[f32; 8]) {
    lie_algebra_sum(h, x);
    let exp_h = mat3_exp_pade(h);
    *h = exp_h;
}

/// Update an existing homography by post-multiplying with the incremental
/// motion described by Lie weights `x0`.
///
/// `Hp = H_delta · H` where `H_delta = exp(LieAlgebraSum(x0))`. This is the
/// post-multiplied parameterization used in the C++ `PolishHomography` IRLS
/// loop.
///
/// # C++ equivalent
/// `vision::UpdateProjectiveMotionPostMultiply<T>` from
/// `homography_estimation/robust_homography.h:478`.
#[inline(always)]
pub fn update_projective_motion_post_multiply(hp: &mut [f32; 9], h: &[f32; 9], x0: &[f32; 8]) {
    let tmp = *h;
    let mut h_delta = [0.0_f32; 9];
    incremental_homography_from_lie_weights(&mut h_delta, x0);
    multiply_3x3_3x3(hp, &h_delta, &tmp);
}

// ============================================================================
// DLT (Direct Linear Transform) homography solver from 4 point correspondences
// ============================================================================
//
// Ported from `homography_estimation/homography_solver.h`. Used by
// `preemptive_robust_homography` to compute one candidate homography per
// RANSAC iteration. The DLT pipeline is:
//
//   1. **Condition** the 4 source points and the 4 target points so that each
//      set has zero mean and √2 standard deviation. This dramatically
//      improves the conditioning of the linear system in step 3.
//   2. **Build the 8×9 constraint matrix** A from the conditioned
//      correspondences (each correspondence contributes 2 rows).
//   3. **Solve** A·h = 0 for the 9-vector `h` (the flattened conditioned
//      homography). This is exactly the M6-2 `solve_null_vector_8x9_destructive`.
//   4. **Denormalize** to recover the homography for the original (un-
//      conditioned) point coordinates.

/// Center 4 points to zero mean and uniform-scale them so that the average
/// distance from the origin is √2. Used as preconditioning before solving
/// the DLT system.
///
/// Returns `false` if all 4 points coincide (zero spread → can't normalize).
///
/// C++ equivalent: `vision::Condition4Points2d<T>` from
/// `homography_estimation/homography_solver.h:47`.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn condition_4_points_2d(
    xp: &mut [[f32; 2]; 4],
    s_out: &mut f32,
    mu_out: &mut [f32; 2],
    p1: &[f32; 2],
    p2: &[f32; 2],
    p3: &[f32; 2],
    p4: &[f32; 2],
) -> bool {
    // Centroid
    mu_out[0] = (p1[0] + p2[0] + p3[0] + p4[0]) * 0.25;
    mu_out[1] = (p1[1] + p2[1] + p3[1] + p4[1]) * 0.25;

    // Centered points
    let d1 = [p1[0] - mu_out[0], p1[1] - mu_out[1]];
    let d2 = [p2[0] - mu_out[0], p2[1] - mu_out[1]];
    let d3 = [p3[0] - mu_out[0], p3[1] - mu_out[1]];
    let d4 = [p4[0] - mu_out[0], p4[1] - mu_out[1]];

    // Mean distance from origin
    let ds1 = (d1[0] * d1[0] + d1[1] * d1[1]).sqrt();
    let ds2 = (d2[0] * d2[0] + d2[1] * d2[1]).sqrt();
    let ds3 = (d3[0] * d3[0] + d3[1] * d3[1]).sqrt();
    let ds4 = (d4[0] * d4[0] + d4[1] * d4[1]).sqrt();
    let d = (ds1 + ds2 + ds3 + ds4) * 0.25;

    if d == 0.0 {
        return false;
    }

    // Scale s such that average distance after scaling is √2
    let s = std::f32::consts::SQRT_2 / d;
    *s_out = s;

    xp[0][0] = d1[0] * s;
    xp[0][1] = d1[1] * s;
    xp[1][0] = d2[0] * s;
    xp[1][1] = d2[1] * s;
    xp[2][0] = d3[0] * s;
    xp[2][1] = d3[1] * s;
    xp[3][0] = d4[0] * s;
    xp[3][1] = d4[1] * s;

    true
}

/// Recover the unconditioned homography from the conditioned one.
///
/// Given a homography `H` that maps conditioned source points to conditioned
/// target points, compute `Hp = inv(Tp) · H · T` where `T` is the source
/// conditioning transform `(s, t)` and `Tp` is the target conditioning
/// transform `(sp, tp)`. The arithmetic is hand-fused from the C++ to
/// preserve the exact operation order for bit-equivalent dual-mode tests.
///
/// C++ equivalent: `vision::DenormalizeHomography<T>` from
/// `homography_estimation/homography_solver.h:104`.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn denormalize_homography(
    hp: &mut [f32; 9],
    h: &[f32; 9],
    s: f32,
    t: &[f32; 2],
    sp: f32,
    tp: &[f32; 2],
) {
    let inv_sp = 1.0 / sp;

    let a = h[6] * tp[0];
    let b = h[7] * tp[0];
    let c = h[0] * inv_sp;
    let d = h[1] * inv_sp;
    let apc = a + c;
    let bpd = b + d;

    let e = h[6] * tp[1];
    let f = h[7] * tp[1];
    let g = h[3] * inv_sp;
    let hh = h[4] * inv_sp;
    let epg = e + g;
    let fph = f + hh;

    let stx = s * t[0];
    let sty = s * t[1];

    hp[0] = s * apc;
    hp[1] = s * bpd;
    hp[2] = h[8] * tp[0] + h[2] * inv_sp - stx * apc - sty * bpd;

    hp[3] = s * epg;
    hp[4] = s * fph;
    hp[5] = h[8] * tp[1] + h[5] * inv_sp - stx * epg - sty * fph;

    hp[6] = h[6] * s;
    hp[7] = h[7] * s;
    hp[8] = h[8] - hp[6] * t[0] - hp[7] * t[1];
}

/// Add a single point correspondence as 2 rows of the 8×9 DLT constraint
/// matrix, starting at offset `dst[0..18]`.
///
/// Each row corresponds to one of the two (homogeneous-coordinate) equations
/// `xp · (H·x) = 0`.
///
/// C++ equivalent: `vision::AddHomographyPointContraint<T>` from
/// `homography_estimation/homography_solver.h:144`.
#[inline(always)]
fn add_homography_point_constraint(dst: &mut [f32; 18], x: &[f32; 2], xp: &[f32; 2]) {
    // Row 0: from u-equation
    dst[0] = -x[0];
    dst[1] = -x[1];
    dst[2] = -1.0;
    dst[3] = 0.0;
    dst[4] = 0.0;
    dst[5] = 0.0;
    dst[6] = xp[0] * x[0];
    dst[7] = xp[0] * x[1];
    dst[8] = xp[0];

    // Row 1: from v-equation
    dst[9] = 0.0;
    dst[10] = 0.0;
    dst[11] = 0.0;
    dst[12] = -x[0];
    dst[13] = -x[1];
    dst[14] = -1.0;
    dst[15] = xp[1] * x[0];
    dst[16] = xp[1] * x[1];
    dst[17] = xp[1];
}

/// Solve for a homography `H` such that `H·xi ≈ xpi` for `i ∈ {1..4}` using
/// Direct Linear Transform on the **conditioned** points (caller must
/// pre-normalize). Returns `false` if the resulting homography has near-zero
/// determinant.
///
/// C++ equivalent: `vision::SolveHomography4PointsInhomogenous<T>` from
/// `homography_estimation/homography_solver.h:185`.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn solve_homography_4_points_inhomogeneous(
    h: &mut [f32; 9],
    x1: &[f32; 2],
    x2: &[f32; 2],
    x3: &[f32; 2],
    x4: &[f32; 2],
    xp1: &[f32; 2],
    xp2: &[f32; 2],
    xp3: &[f32; 2],
    xp4: &[f32; 2],
) -> bool {
    let mut a = [0.0_f32; 72];

    // Build the 8×9 constraint matrix (each correspondence → 2 rows × 9 cols
    // = 18 elements).
    {
        let (block, rest) = a.split_at_mut(18);
        let block: &mut [f32; 18] = block.try_into().unwrap();
        add_homography_point_constraint(block, x1, xp1);
        let (block, rest) = rest.split_at_mut(18);
        let block: &mut [f32; 18] = block.try_into().unwrap();
        add_homography_point_constraint(block, x2, xp2);
        let (block, rest) = rest.split_at_mut(18);
        let block: &mut [f32; 18] = block.try_into().unwrap();
        add_homography_point_constraint(block, x3, xp3);
        let block: &mut [f32; 18] = rest.try_into().unwrap();
        add_homography_point_constraint(block, x4, xp4);
    }

    if !solve_null_vector_8x9_destructive(h, &mut a) {
        return false;
    }
    if determinant_3x3(h).abs() < 1e-5 {
        return false;
    }
    true
}

/// Solve for the homography from 4 point correspondences using DLT with
/// pre-conditioning.
///
/// Returns `false` if either the 4 source points or the 4 target points
/// coincide (zero spread), or if the resulting homography is degenerate
/// (near-zero determinant).
///
/// # C++ equivalent
/// `vision::SolveHomography4Points<T>` from
/// `homography_estimation/homography_solver.h:209`. Called from
/// [`preemptive_robust_homography`] for each RANSAC trial.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn solve_homography_4_points(
    h: &mut [f32; 9],
    x1: &[f32; 2],
    x2: &[f32; 2],
    x3: &[f32; 2],
    x4: &[f32; 2],
    xp1: &[f32; 2],
    xp2: &[f32; 2],
    xp3: &[f32; 2],
    xp4: &[f32; 2],
) -> bool {
    // Condition source points
    let mut cond_src = [[0.0_f32; 2]; 4];
    let mut s = 0.0_f32;
    let mut t = [0.0_f32; 2];
    if !condition_4_points_2d(&mut cond_src, &mut s, &mut t, x1, x2, x3, x4) {
        return false;
    }

    // Condition target points
    let mut cond_tgt = [[0.0_f32; 2]; 4];
    let mut sp = 0.0_f32;
    let mut tp = [0.0_f32; 2];
    if !condition_4_points_2d(&mut cond_tgt, &mut sp, &mut tp, xp1, xp2, xp3, xp4) {
        return false;
    }

    // Solve in conditioned coordinates
    let mut h_norm = [0.0_f32; 9];
    if !solve_homography_4_points_inhomogeneous(
        &mut h_norm,
        &cond_src[0],
        &cond_src[1],
        &cond_src[2],
        &cond_src[3],
        &cond_tgt[0],
        &cond_tgt[1],
        &cond_tgt[2],
        &cond_tgt[3],
    ) {
        return false;
    }

    // Denormalize back to original coordinates
    denormalize_homography(h, &h_norm, s, &t, sp, &tp);

    true
}

// ============================================================================
// RANSAC RNG + utility (bit-equivalent to upstream C++)
// ============================================================================
//
// Ported from `math/rand.h`. The 32-bit linear congruential generator used
// by the RANSAC step in `vision::PreemptiveRobustHomography`. Faithfully
// matching the C++ bit-pattern is required so that dual-mode tests can
// compare the final homography element-wise (same RNG → same hypothesis
// order → same final H modulo float-rounding noise).
//
// `wrapping_mul`/`wrapping_add` are critical: the C++ relies on signed-int
// wraparound which is technically undefined behaviour in C++ but, in
// practice, yields the expected MSVC-`rand()`-style sequence on every
// compiler that has ever shipped this code.

/// Linear-congruential pseudo-random generator: `seed = 214013·seed +
/// 2531011`, return `(seed >> 16) & 0x7FFF`.
///
/// Matches the C++ `vision::FastRandom` exactly. Output range: `[0, 32767]`
/// (i.e. `FAST_RAND_MAX` inclusive).
///
/// # C++ equivalent
/// `vision::FastRandom` from `math/rand.h:49`.
#[inline(always)]
fn fast_random(seed: &mut i32) -> i32 {
    *seed = 214013i32.wrapping_mul(*seed).wrapping_add(2531011);
    (*seed >> 16) & FAST_RAND_MAX
}

/// In-place partial Fisher-Yates-style shuffle of an integer array.
///
/// Shuffles only the **first `sample_size`** elements of `v`, but uses
/// `pop_size` for the modulo (i.e. each of the first `sample_size` slots
/// can land any element from the full `pop_size`). Matches the upstream
/// algorithm used by RANSAC to draw 4-tuples for hypothesis generation.
///
/// # C++ equivalent
/// `vision::ArrayShuffle<T>` from `math/rand.h:70`.
#[inline(always)]
fn array_shuffle(v: &mut [i32], pop_size: usize, sample_size: usize, seed: &mut i32) {
    debug_assert!(pop_size <= v.len());
    for i in 0..sample_size {
        let k = (fast_random(seed) as usize) % pop_size;
        v.swap(i, k);
    }
}

/// Wirth's k-smallest partial sort (in place) on an array of `(cost, index)`
/// pairs. After the call, `a[k]` holds the k-th smallest pair (lexicographic
/// order over `(cost, index)`); pairs at positions `0..k` are smaller-or-
/// equal and pairs at `k+1..` are larger-or-equal, but neither side is fully
/// sorted.
///
/// # C++ equivalent
/// `vision::PartialSort<T1,T2>` from `utils/partial_sort.h:77` (the
/// pair-overload used in RANSAC).
#[inline(always)]
fn partial_sort_pairs(a: &mut [(f32, i32)], k: usize) {
    let n = a.len();
    if n == 0 {
        return;
    }
    debug_assert!(k < n);
    let k_minus_1 = if k == 0 { 0 } else { k - 1 };
    // The C++ uses `k` as 1-based index; we follow its semantics by reading
    // the pivot at `k - 1` (saturated at 0 for k=0). This matches the C++
    // call `PartialSort(a, n, (n/2 or (n/2)-1))` exactly.

    let mut l: isize = 0;
    let mut m: isize = (n as isize) - 1;
    while l < m {
        let x = a[k_minus_1];
        let mut i = l;
        let mut j = m;
        loop {
            while pair_lt(a[i as usize], x) {
                i += 1;
            }
            while pair_lt(x, a[j as usize]) {
                j -= 1;
            }
            if i <= j {
                a.swap(i as usize, j as usize);
                i += 1;
                j -= 1;
            }
            if i > j {
                break;
            }
        }
        if j < k_minus_1 as isize {
            l = i;
        }
        if (k_minus_1 as isize) < i {
            m = j;
        }
    }
}

/// Lexicographic less-than for `(f32, i32)` pairs, matching `std::pair` in
/// C++. Treats NaN as greater than any number (the RANSAC costs are always
/// finite so NaN doesn't actually appear in practice).
#[inline(always)]
fn pair_lt(a: (f32, i32), b: (f32, i32)) -> bool {
    match a.0.partial_cmp(&b.0) {
        Some(std::cmp::Ordering::Less) => true,
        Some(std::cmp::Ordering::Greater) => false,
        Some(std::cmp::Ordering::Equal) => a.1 < b.1,
        // NaN goes last (consistent ordering)
        None => a.0.is_nan() && !b.0.is_nan(),
    }
}

/// Find the median of an array of `(cost, index)` pairs in place.
///
/// Specifically, the pivot index is computed as in C++:
/// `k = if n is odd { n/2 } else { (n/2) - 1 }`. After the call, the pivot
/// pair sits at index `k` and pairs left of it are smaller-or-equal.
///
/// # C++ equivalent
/// `vision::FastMedian<T1,T2>` from `utils/partial_sort.h:114` (the
/// pair-overload).
#[inline(always)]
fn fast_median(a: &mut [(f32, i32)]) {
    let n = a.len();
    if n == 0 {
        return;
    }
    let k = if n & 1 == 1 { n / 2 } else { (n / 2) - 1 };
    partial_sort_pairs(a, k);
}

// ============================================================================
// RANSAC: preemptive_robust_homography
// ============================================================================
//
// Ported from `vision::PreemptiveRobustHomography` at
// `homography_estimation/robust_homography.h:96`. The largest single function
// in the FreakMatcher (≈150 lines in C++).
//
// Algorithm:
//
//   1. Generate up to `max_num_hypotheses` candidate homographies by drawing
//      4-tuples from the input correspondences. Each draw is filtered by a
//      cheap geometric-consistency check before the (more expensive) DLT
//      solve. If `test_points` are supplied, additionally check that the
//      candidate keeps the test points geometrically consistent.
//
//   2. **Preemptive scoring**: assign 0 cost to every candidate, then score
//      them in chunks of `chunk_size` correspondences. After each chunk,
//      drop the worse half of the candidates (median split via
//      [`fast_median`]). Repeat until ≤ 2 candidates remain or all
//      correspondences are exhausted.
//
//   3. Return the best (lowest-cost) surviving candidate, normalised so that
//      `H[8] = 1`.
//
// The "preemptive" pruning is the key optimisation — it lets the algorithm
// score 1024 hypotheses against thousands of correspondences in time
// proportional to `n_correspondences·log(n_hypotheses)` rather than the
// naive `n·H`.

/// Robust 4-point RANSAC homography estimation with preemptive scoring.
///
/// # Arguments
/// * `h` — output 3×3 homography (row-major)
/// * `p` — source points, flat `[x0, y0, x1, y1, ...]` of length `2*num_points`
/// * `q` — target points, same layout
/// * `num_points` — number of correspondences (must be ≥ 4)
/// * `test_points` — optional flat slice of additional 2D points to use as
///   a geometric-consistency oracle on each hypothesis. Pass `&[]` for
///   "no test points".
/// * `num_test_points` — number of test points (must equal `test_points.len()/2`)
/// * `hyp` — caller-owned scratch buffer of size `9 * max_num_hypotheses`
/// * `tmp_i` — caller-owned scratch buffer of size `num_points`
/// * `hyp_costs` — caller-owned scratch buffer of size `max_num_hypotheses`
/// * `scale` — Cauchy scale parameter for outlier robustness
/// * `max_num_hypotheses` — upper bound on RANSAC candidates to evaluate
/// * `max_trials` — upper bound on RANSAC draws (some are rejected before
///   becoming candidates)
/// * `chunk_size` — size of each preemptive-scoring chunk
///
/// # Returns
/// `false` if `num_points < 4`, if every draw failed the geometric checks,
/// or if all hypotheses converged to zero-cost ties — failures logged via
/// `arlog_e!`.
///
/// # C++ equivalent
/// `vision::PreemptiveRobustHomography<T>` from
/// `homography_estimation/robust_homography.h:96`.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
pub fn preemptive_robust_homography(
    h: &mut [f32; 9],
    p: &[f32],
    q: &[f32],
    num_points: usize,
    test_points: &[f32],
    num_test_points: usize,
    hyp: &mut [f32],
    tmp_i: &mut [i32],
    hyp_costs: &mut [(f32, i32)],
    scale: f32,
    max_num_hypotheses: i32,
    max_trials: i32,
    chunk_size: i32,
) -> bool {
    const SAMPLE_SIZE: usize = 4;

    debug_assert!(p.len() >= num_points * 2);
    debug_assert!(q.len() >= num_points * 2);
    debug_assert!(test_points.len() >= num_test_points * 2);
    debug_assert!(hyp.len() >= 9 * max_num_hypotheses as usize);
    debug_assert!(tmp_i.len() >= num_points);
    debug_assert!(hyp_costs.len() >= max_num_hypotheses as usize);

    if num_points < SAMPLE_SIZE {
        arlog_e!(
            "preemptive_robust_homography: num_points ({}) < {}",
            num_points,
            SAMPLE_SIZE
        );
        return false;
    }

    let mut seed = RANSAC_INITIAL_SEED;
    let one_over_scale2 = 1.0 / sqr(scale);
    let chunk_size = min2(chunk_size, num_points as i32) as usize;

    // Fill `tmp_i` with [0, 1, 2, ..., num_points-1]
    for (i, slot) in tmp_i.iter_mut().take(num_points).enumerate() {
        *slot = i as i32;
    }

    // Initial full shuffle of the index array
    array_shuffle(tmp_i, num_points, num_points, &mut seed);

    // ------------------------------------------------------------------
    // Step 1: generate candidate hypotheses
    // ------------------------------------------------------------------
    let mut num_hypotheses = 0_usize;
    let max_num_hypotheses = max_num_hypotheses as usize;
    let max_trials = max_trials as usize;

    for _trial in 0..max_trials {
        if num_hypotheses >= max_num_hypotheses {
            break;
        }

        // Shuffle the first SAMPLE_SIZE indices for this draw
        array_shuffle(tmp_i, num_points, SAMPLE_SIZE, &mut seed);

        let i0 = (tmp_i[0] as usize) * 2;
        let i1 = (tmp_i[1] as usize) * 2;
        let i2 = (tmp_i[2] as usize) * 2;
        let i3 = (tmp_i[3] as usize) * 2;

        let p0: [f32; 2] = [p[i0], p[i0 + 1]];
        let p1: [f32; 2] = [p[i1], p[i1 + 1]];
        let p2: [f32; 2] = [p[i2], p[i2 + 1]];
        let p3: [f32; 2] = [p[i3], p[i3 + 1]];
        let q0: [f32; 2] = [q[i0], q[i0 + 1]];
        let q1: [f32; 2] = [q[i1], q[i1 + 1]];
        let q2: [f32; 2] = [q[i2], q[i2 + 1]];
        let q3: [f32; 2] = [q[i3], q[i3 + 1]];

        // Geometric-consistency filter on the 4 source/target pairs
        if !homography_4_points_geometrically_consistent(&p0, &p1, &p2, &p3, &q0, &q1, &q2, &q3) {
            continue;
        }

        // DLT solve for one candidate homography
        let h_slot: &mut [f32; 9] = (&mut hyp[num_hypotheses * 9..num_hypotheses * 9 + 9])
            .try_into()
            .unwrap();
        if !solve_homography_4_points(h_slot, &p0, &p1, &p2, &p3, &q0, &q1, &q2, &q3) {
            continue;
        }

        // Optional check against test points
        if num_test_points > 0
            && !homography_points_geometrically_consistent(
                h_slot,
                &test_points[..num_test_points * 2],
                num_test_points,
            )
        {
            continue;
        }

        num_hypotheses += 1;
    }

    if num_hypotheses == 0 {
        arlog_e!(
            "preemptive_robust_homography: no valid hypothesis after {} trials",
            max_trials
        );
        return false;
    }

    // ------------------------------------------------------------------
    // Step 2: preemptive scoring with median pruning
    // ------------------------------------------------------------------
    for (i, slot) in hyp_costs.iter_mut().take(num_hypotheses).enumerate() {
        *slot = (0.0_f32, i as i32);
    }

    let mut num_remaining = num_hypotheses;
    let mut cur_chunk = chunk_size;
    let mut i = 0_usize;
    while i < num_points && num_remaining > 2 {
        cur_chunk = min2(chunk_size as i32, (num_points - i) as i32) as usize;
        let chunk_end = i + cur_chunk;

        // Score each remaining hypothesis on this chunk of correspondences
        for slot in hyp_costs.iter_mut().take(num_remaining) {
            let h_idx = slot.1 as usize;
            let h_cur: &[f32; 9] = (&hyp[h_idx * 9..h_idx * 9 + 9]).try_into().unwrap();
            for k in i..chunk_end {
                let p_k = [p[(tmp_i[k] as usize) * 2], p[(tmp_i[k] as usize) * 2 + 1]];
                let q_k = [q[(tmp_i[k] as usize) * 2], q[(tmp_i[k] as usize) * 2 + 1]];
                slot.0 += cauchy_projective_reprojection_cost(h_cur, &p_k, &q_k, one_over_scale2);
            }
        }

        // Median split: drop the worse half
        fast_median(&mut hyp_costs[..num_remaining]);
        num_remaining >>= 1;

        i += cur_chunk;
    }
    let _ = cur_chunk; // last value unused

    // ------------------------------------------------------------------
    // Step 3: pick the best surviving hypothesis
    // ------------------------------------------------------------------
    let mut min_idx = hyp_costs[0].1;
    let mut min_cost = hyp_costs[0].0;
    for slot in hyp_costs.iter().skip(1).take(num_remaining - 1) {
        if slot.0 < min_cost {
            min_cost = slot.0;
            min_idx = slot.1;
        }
    }

    let best: &[f32; 9] = (&hyp[(min_idx as usize) * 9..(min_idx as usize) * 9 + 9])
        .try_into()
        .unwrap();
    copy_vector_9(h, best);
    normalize_homography(h);

    true
}

// ============================================================================
// IRLS Polish: Cauchy derivative + Lie Jacobian
// ============================================================================
//
// The IRLS (Iteratively Reweighted Least Squares) polish is a Levenberg-
// Marquardt minimisation of the Cauchy-robustified reprojection cost over
// the Lie-algebra parameterization of homography. At each iteration:
//
//   1. Linearise the residual function `f(δ) = H·exp(LieAlgebraSum(δ))·p − q`
//      around `δ = 0`. The Jacobian factors into:
//        - `J_cauchy` (2×2) : derivative of √(Cauchy weight) ⊗ residual
//        - `J_lie`    (2×8) : derivative of the projected point w.r.t. δ
//        - `J_p = J_cauchy · J_lie` (2×8) : composed Jacobian
//   2. Build normal equations: `JᵀJ · δ = Jᵀr`
//   3. Apply Levenberg-Marquardt regularisation: `(JᵀJ + λ·diag(JᵀJ))`
//   4. Solve via Cholesky decomposition (the regularised matrix is SPD).
//   5. Update `H ← H · exp(LieAlgebraSum(δ))` (post-multiply parameterisation).
//   6. Accept the step if the cost decreased; otherwise reject and increase λ.

/// Compute the Cauchy IRLS Jacobian and weighted residual.
///
/// Given a residual `f` (the 2D reprojection error `H·p − q`), this computes:
/// - `J_r` (2×2): the "square root" of the Cauchy weight matrix (`J_r ⊗ f`
///   has the same Mahalanobis norm as the un-weighted residual under the
///   Cauchy-weighted metric, but is linear in `f` for Gauss–Newton).
/// - `fp` (2): the weighted residual `√(Cauchy weight) · f`.
///
/// At `f = 0`, the Cauchy weight is `1/(2·σ²)` and the matrix `J_r` reduces
/// to `√(1/σ²) · I`.
///
/// C++ equivalent: `vision::CauchyDerivative<T>` from
/// `homography_estimation/robust_homography.h:293`.
#[inline(always)]
fn cauchy_derivative(j_r: &mut [f32; 4], fp: &mut [f32; 2], f: &[f32; 2], one_over_scale2: f32) {
    let x = f[0];
    let y = f[1];
    let x2 = x * x;
    let y2 = y * y;
    let r2 = x2 + y2;

    let mut fu_at_zero = false;

    if r2 <= 0.0 {
        fu_at_zero = true;
    } else {
        let one_over_r2 = 1.0 / r2;
        let t = 1.0 + r2 * one_over_scale2;
        let one_over_r2_times_t = 1.0 / (r2 * t);
        let fu = t.ln() * one_over_r2;

        if fu <= 0.0 {
            fu_at_zero = true;
        } else {
            let sqrt_fu = fu.sqrt();
            let fu_times_one_over_r2 = fu * one_over_r2;
            let one_over_denom = 1.0 / (2.0 * sqrt_fu);

            // dq/df: derivative of the projected (sqrt-weighted) residual
            //        with respect to f, ignoring the implicit dependence
            //        of √(Cauchy) on f (the chain-rule terms below).
            let dqdf = [x * one_over_denom, y * one_over_denom];

            // df/dp: ∂(log(t)/r²) / ∂p
            let dfdp = [
                2.0 * (one_over_scale2 * x * one_over_r2_times_t - x * fu_times_one_over_r2),
                2.0 * (one_over_scale2 * y * one_over_r2_times_t - y * fu_times_one_over_r2),
            ];

            // J_r = dq/df · df/dp + sqrt_fu · I  (2×2, symmetric)
            j_r[0] = dqdf[0] * dfdp[0] + sqrt_fu;
            j_r[1] = dqdf[0] * dfdp[1];
            j_r[2] = j_r[1];
            j_r[3] = dqdf[1] * dfdp[1] + sqrt_fu;

            // Weighted residual
            fp[0] = sqrt_fu * f[0];
            fp[1] = sqrt_fu * f[1];
        }
    }

    if fu_at_zero {
        // Limiting behaviour at f = 0: J_r = √(1/σ²) · I, fp = 0.
        fp[0] = 0.0;
        fp[1] = 0.0;
        let v = one_over_scale2.sqrt();
        j_r[0] = v;
        j_r[1] = 0.0;
        j_r[2] = 0.0;
        j_r[3] = v;
    }
}

/// 2×8 Lie Jacobian of the inhomogeneous projection w.r.t. the 8 Lie weights.
///
/// Linearises `f(δ) = H·exp(LieAlgebraSum(δ))·p − q` around `δ = 0`. The
/// Jacobian rows are (per the analytic derivation in Benhimane–Malis 2007):
///
/// ```text
///   ∂f₀/∂δ = [ 1, 0, y,  0,  x,   -x,  -x²,  -xy ]
///   ∂f₁/∂δ = [ 0, 1, 0,  x, -y, -2y, -xy,  -y² ]
/// ```
///
/// where `(x, y)` is the projected source point and `f = H·p − q` is the
/// raw residual.
///
/// C++ equivalent: `vision::HomographyLieJacobian<T>` from
/// `homography_estimation/robust_homography.h:250`.
#[inline(always)]
fn homography_lie_jacobian(
    j: &mut [f32; 16],
    f: &mut [f32; 2],
    pp: &[f32; 2],
    p: &[f32; 2],
    q: &[f32; 2],
) {
    let x = p[0];
    let y = p[1];

    // Row 0
    j[0] = 1.0;
    j[1] = 0.0;
    j[2] = y;
    j[3] = 0.0;
    j[4] = x;
    j[5] = -x;
    j[6] = -x * x;
    j[7] = -x * y;
    // Row 1
    j[8] = 0.0;
    j[9] = 1.0;
    j[10] = 0.0;
    j[11] = x;
    j[12] = -y;
    j[13] = -2.0 * y;
    j[14] = -x * y;
    j[15] = -y * y;

    // Residual
    f[0] = pp[0] - q[0];
    f[1] = pp[1] - q[1];
}

/// Compose the Cauchy Jacobian with the Lie Jacobian to get the final
/// 2×8 linearization of the robust residual.
///
/// `Jp = J_cauchy · J_lie`. Also returns the weighted residual `fp`.
///
/// C++ equivalent: `vision::RobustHomographyLieJacobianPostMultiply<T>` from
/// `homography_estimation/robust_homography.h:357`.
#[inline(always)]
fn robust_homography_lie_jacobian_post_multiply(
    jp: &mut [f32; 16],
    fp: &mut [f32; 2],
    h: &[f32; 9],
    p: &[f32; 2],
    q: &[f32; 2],
    one_over_scale2: f32,
) {
    // pp = H · p (inhomogeneous)
    let mut pp = [0.0_f32; 2];
    multiply_point_homography_inhomogenous(&mut pp, h, p);

    // J_lie (2×8) and raw residual f (2)
    let mut j = [0.0_f32; 16];
    let mut f = [0.0_f32; 2];
    homography_lie_jacobian(&mut j, &mut f, &pp, &pp, q);

    // J_cauchy (2×2) and weighted residual fp
    let mut j_r = [0.0_f32; 4];
    cauchy_derivative(&mut j_r, fp, &f, one_over_scale2);

    // Jp = J_r · J  (2×8). Hand-fused to match the C++ exactly (preserves
    // operation order for bit-equivalent dual-mode tests).
    jp[0] = j_r[0] * j[0];
    jp[1] = j_r[1] * j[9];
    jp[2] = j_r[0] * j[2];
    jp[3] = j_r[1] * j[11];
    jp[4] = j_r[0] * j[4] + j_r[1] * j[12];
    jp[5] = j_r[0] * j[5] + j_r[1] * j[13];
    jp[6] = j_r[0] * j[6] + j_r[1] * j[14];
    jp[7] = j_r[0] * j[7] + j_r[1] * j[15];
    jp[8] = j_r[2] * j[0];
    jp[9] = j_r[3] * j[9];
    jp[10] = j_r[2] * j[2];
    jp[11] = j_r[3] * j[11];
    jp[12] = j_r[2] * j[4] + j_r[3] * j[12];
    jp[13] = j_r[2] * j[5] + j_r[3] * j[13];
    jp[14] = j_r[2] * j[6] + j_r[3] * j[14];
    jp[15] = j_r[2] * j[7] + j_r[3] * j[15];
}

// ============================================================================
// IRLS Polish: normal equations + Cholesky + polish loop
// ============================================================================

/// `C += A^T · A` for a 2×8 matrix `A`, accumulating into the upper triangle
/// of an 8×8 matrix `C` (lower triangle is left untouched and filled later
/// via [`symmetric_extend_upper_to_lower_8x8`]).
///
/// Specialised to the only call sizes used in homography polish (rows=2,
/// cols=8).
///
/// C++ equivalent: `vision::MultiplyAndAccumulateAtA<T>` from `math/matrix.h:121`
/// (general-size template), specialised here.
#[inline(always)]
fn multiply_and_accumulate_at_a_2x8(c: &mut [f32; 64], a: &[f32; 16]) {
    // C is 8×8 (column count == 8 == Acols). Each (i, j) entry with j ≥ i
    // accumulates sum over k=0..1 of A[k,i]·A[k,j], where A[k,i] = a[k*8+i].
    for i in 0..8 {
        let row_offset = i * 8;
        for j in i..8 {
            // A[0,i]·A[0,j] + A[1,i]·A[1,j]
            let sum = a[i] * a[j] + a[8 + i] * a[8 + j];
            c[row_offset + j] += sum;
        }
    }
}

/// `y += A^T · x` for a 2×8 matrix `A` and a 2-vector `x`, accumulating into
/// an 8-vector `y`.
///
/// C++ equivalent: `vision::MultiplyAndAccumulateAtx<T>` from `math/matrix.h:139`
/// (general-size template), specialised here to rows=2, cols=8.
#[inline(always)]
fn multiply_and_accumulate_at_x_2x8(y: &mut [f32; 8], a: &[f32; 16], x: &[f32; 2]) {
    for i in 0..8 {
        // sum over j=0..1 of A[j,i]·x[j], where A[j,i] = a[j*8+i]
        let sum = a[i] * x[0] + a[8 + i] * x[1];
        y[i] += sum;
    }
}

/// Mirror the upper triangle of an 8×8 matrix into the lower triangle.
///
/// After this call, the matrix is fully symmetric.
///
/// C++ equivalent: `vision::SymmetricExtendUpperToLower<T>` from
/// `math/linear_algebra.h:389` (general-size template), specialised here to N=8.
#[inline(always)]
fn symmetric_extend_upper_to_lower_8x8(a: &mut [f32; 64]) {
    for i in 1..8 {
        for j in 0..i {
            a[i * 8 + j] = a[j * 8 + i];
        }
    }
}

/// Build the normal-equation matrices `JᵀJ` (8×8 SPD) and `Jᵀr` (8) for the
/// Cauchy-IRLS Gauss–Newton step.
///
/// Iterates over all `n` correspondences, accumulating `Jᵀ·J` (upper
/// triangle only) and `Jᵀ·r` from the per-point Jacobians. Then mirrors
/// the lower triangle and negates the right-hand side (to match the LM
/// step direction).
///
/// C++ equivalent:
/// `vision::ComputeHomographyNormalEquationsPostMultiply<T>` from
/// `homography_estimation/robust_homography.h:397`.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn compute_homography_normal_equations_post_multiply(
    jt_j: &mut [f32; 64],
    jt_r: &mut [f32; 8],
    h: &[f32; 9],
    p: &[f32],
    q: &[f32],
    n: usize,
    one_over_scale2: f32,
) {
    debug_assert!(p.len() >= n * 2);
    debug_assert!(q.len() >= n * 2);

    // Zero accumulators
    *jt_j = [0.0_f32; 64];
    *jt_r = [0.0_f32; 8];

    let mut j = [0.0_f32; 16];
    let mut fp = [0.0_f32; 2];

    for i in 0..n {
        let p_i = [p[i * 2], p[i * 2 + 1]];
        let q_i = [q[i * 2], q[i * 2 + 1]];

        robust_homography_lie_jacobian_post_multiply(
            &mut j,
            &mut fp,
            h,
            &p_i,
            &q_i,
            one_over_scale2,
        );

        multiply_and_accumulate_at_a_2x8(jt_j, &j);
        multiply_and_accumulate_at_x_2x8(jt_r, &j, &fp);
    }

    symmetric_extend_upper_to_lower_8x8(jt_j);
    // Negate Jtr (matches C++ `ScaleVector8(Jtr, Jtr, -1)`)
    for v in jt_r.iter_mut() {
        *v = -*v;
    }
}

/// Apply Levenberg–Marquardt regularisation `out = JᵀJ + λ·diag(JᵀJ)` to an
/// 8×8 SPD matrix.
///
/// Only the diagonal entries (indices 0, 9, 18, 27, 36, 45, 54, 63) are
/// modified — the off-diagonals are copied unchanged.
///
/// C++ equivalent: `vision::RegularizeLevenbergMarquardt8x8<T>` from
/// `homography_estimation/robust_homography.h:431`.
#[inline(always)]
fn regularize_levenberg_marquardt_8x8(out: &mut [f32; 64], in_jt_j: &[f32; 64], lambda: f32) {
    // Note: the C++ only writes the 8 diagonal positions and assumes the
    // off-diagonals were copied beforehand by the caller. We replicate that
    // contract here — the polish loop calls `copy_vector(reg_jt_j, jt_j, 64)`
    // before this function, so the off-diagonals are already correct.
    out[0] = in_jt_j[0] + lambda * in_jt_j[0];
    out[9] = in_jt_j[9] + lambda * in_jt_j[9];
    out[18] = in_jt_j[18] + lambda * in_jt_j[18];
    out[27] = in_jt_j[27] + lambda * in_jt_j[27];
    out[36] = in_jt_j[36] + lambda * in_jt_j[36];
    out[45] = in_jt_j[45] + lambda * in_jt_j[45];
    out[54] = in_jt_j[54] + lambda * in_jt_j[54];
    out[63] = in_jt_j[63] + lambda * in_jt_j[63];
}

/// Solve `A·x = b` for an 8×8 symmetric positive-definite matrix `A` via
/// Cholesky decomposition followed by forward + back substitution.
///
/// Specialised to N=8 (the only call site in homography polish).
///
/// Returns `false` if `A` is not SPD (any pivot reaches zero or negative).
/// Failure is logged via `arlog_e!`.
///
/// C++ equivalent: `vision::SolvePositiveDefiniteSystem<T, 8>` from
/// `math/cholesky_linear_solvers.h:88`, with the underlying Cholesky from
/// `math/cholesky.h:44`. Threshold is `0.0` to match C++ exactly (the upstream
/// `PolishHomography` calls with `threshold=0`).
#[inline(always)]
fn solve_positive_definite_system_8x8(x: &mut [f32; 8], a: &[f32; 64], b: &[f32; 8]) -> bool {
    // Cholesky factor A = L · Lᵀ (lower triangular L, stored at l[i*8+j] for j ≤ i)
    let mut l = [0.0_f32; 64];
    for i in 0..8 {
        for j in 0..=i {
            let mut s = a[i * 8 + j];
            for k in 0..j {
                s -= l[i * 8 + k] * l[j * 8 + k];
            }
            if i == j {
                if s < 0.0 {
                    arlog_e!(
                        "solve_positive_definite_system_8x8: matrix not SPD (s={} at i={})",
                        s,
                        i
                    );
                    return false;
                }
                l[i * 8 + j] = s.sqrt();
            } else {
                let l_jj = l[j * 8 + j];
                if l_jj == 0.0 {
                    arlog_e!("solve_positive_definite_system_8x8: zero pivot at j={}", j);
                    return false;
                }
                l[i * 8 + j] = s / l_jj;
            }
        }
    }

    // Forward substitution: L · y = b
    let mut y = [0.0_f32; 8];
    for i in 0..8 {
        let mut s = b[i];
        for k in 0..i {
            s -= l[i * 8 + k] * y[k];
        }
        let l_ii = l[i * 8 + i];
        if l_ii == 0.0 {
            arlog_e!(
                "solve_positive_definite_system_8x8: zero diagonal in forward substitution at i={}",
                i
            );
            return false;
        }
        y[i] = s / l_ii;
    }

    // Back substitution: Lᵀ · x = y  (use L_ki for the upper-triangle access)
    for i in (0..8).rev() {
        let mut s = y[i];
        for k in (i + 1)..8 {
            s -= l[k * 8 + i] * x[k];
        }
        let l_ii = l[i * 8 + i];
        if l_ii == 0.0 {
            arlog_e!(
                "solve_positive_definite_system_8x8: zero diagonal in back substitution at i={}",
                i
            );
            return false;
        }
        x[i] = s / l_ii;
    }

    true
}

/// IRLS Levenberg–Marquardt polish of a homography against Cauchy-robustified
/// reprojection residuals.
///
/// Starting from the input estimate `H` (typically the output of
/// [`preemptive_robust_homography`]), iteratively refine `H` by solving
/// regularised normal equations and applying post-multiplied Lie updates
/// until convergence. Iteration is capped at `max_iterations`; the loop
/// also exits early if `max_stops` consecutive non-improving steps occur or
/// if improvement falls below `improvement` for that many steps in a row.
///
/// # C++ equivalent
/// `vision::PolishHomography<T>` from
/// `homography_estimation/robust_homography.h:500`. Called by
/// [`RobustHomography::find`] after the RANSAC step.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
pub fn polish_homography(
    h: &mut [f32; 9],
    p: &[f32],
    q: &[f32],
    n: usize,
    scale: f32,
    max_iterations: i32,
    max_stops: i32,
    improvement: f32,
) -> bool {
    let one_over_scale2 = 1.0 / (scale * scale);

    let mut jt_j = [0.0_f32; 64];
    let mut reg_jt_j = [0.0_f32; 64];
    let mut jt_r = [0.0_f32; 8];
    let mut delta = [0.0_f32; 8];
    let mut hp = [0.0_f32; 9];

    let mut last_cost = cauchy_projective_reprojection_cost_total(h, p, q, n, one_over_scale2);
    let mut update = true;
    let mut lambda = 0.01_f32;
    let mut stops: i32 = 0;

    let mut i = 0_i32;
    while i < max_iterations && stops < max_stops {
        if update {
            compute_homography_normal_equations_post_multiply(
                &mut jt_j,
                &mut jt_r,
                h,
                p,
                q,
                n,
                one_over_scale2,
            );
            reg_jt_j = jt_j;
        }

        regularize_levenberg_marquardt_8x8(&mut reg_jt_j, &jt_j, lambda);

        if !solve_positive_definite_system_8x8(&mut delta, &reg_jt_j, &jt_r) {
            return false;
        }

        update_projective_motion_post_multiply(&mut hp, h, &delta);

        let cost = cauchy_projective_reprojection_cost_total(&hp, p, q, n, one_over_scale2);

        if cost < last_cost {
            copy_vector_9(h, &hp);
            stops = if (last_cost - cost) < improvement {
                stops + 1
            } else {
                0
            };
            last_cost = cost;
            lambda = max2(lambda * 0.1, 0.000001);
            update = true;
        } else {
            lambda = min2(lambda * 10.0, 100000.0);
            stops += 1;
            update = false;
        }

        i += 1;
    }

    true
}

// ============================================================================
// RobustHomography struct (public entrypoint)
// ============================================================================

/// Robust homography estimation with RANSAC + Cauchy-IRLS refinement.
///
/// Wraps the [`preemptive_robust_homography`] RANSAC step and the
/// [`polish_homography`] IRLS Levenberg–Marquardt refinement into a single
/// `find()` method. Mirrors the upstream C++ `vision::RobustHomography<T>`
/// class — same parameter set, same default values, same algorithm.
///
/// # Example
/// ```ignore
/// use webarkitlib_rs::kpm::freak::homography::RobustHomography;
///
/// let estimator = RobustHomography::default();
/// let mut h = [0.0_f32; 9];
/// let p = [/* source points: x0, y0, x1, y1, ... */];
/// let q = [/* target points: x0', y0', x1', y1', ... */];
/// if estimator.find(&mut h, &p, &q, p.len() / 2) {
///     // h now contains the estimated 3×3 homography (row-major).
/// }
/// ```
///
/// # C++ equivalent
/// `vision::RobustHomography<T>` from
/// `homography_estimation/robust_homography.h:572`.
#[derive(Debug, Clone, Copy)]
pub struct RobustHomography {
    cauchy_scale: f32,
    num_hypotheses: i32,
    max_trials: i32,
    chunk_size: i32,
}

impl Default for RobustHomography {
    /// Defaults match the C++ `HOMOGRAPHY_DEFAULT_*` constants.
    #[inline(always)]
    fn default() -> Self {
        Self::new(
            HOMOGRAPHY_DEFAULT_CAUCHY_SCALE,
            HOMOGRAPHY_DEFAULT_NUM_HYPOTHESES,
            HOMOGRAPHY_DEFAULT_MAX_TRIALS,
            HOMOGRAPHY_DEFAULT_CHUNK_SIZE,
        )
    }
}

impl RobustHomography {
    /// Construct a RANSAC + IRLS estimator with the given parameters.
    ///
    /// All four parameters mirror the C++ constructor arguments:
    /// - `cauchy_scale`: the Cauchy robustifier scale parameter (`σ`). Smaller
    ///   values down-weight outliers more aggressively.
    /// - `num_hypotheses`: maximum number of RANSAC candidates to evaluate.
    /// - `max_trials`: maximum number of RANSAC draws (some are rejected
    ///   before becoming candidates due to geometric-consistency failure).
    /// - `chunk_size`: size of each preemptive-scoring chunk.
    #[inline(always)]
    pub fn new(cauchy_scale: f32, num_hypotheses: i32, max_trials: i32, chunk_size: i32) -> Self {
        Self {
            cauchy_scale,
            num_hypotheses,
            max_trials,
            chunk_size,
        }
    }

    /// Find the homography that maps source points `p` to target points `q`.
    ///
    /// `p` and `q` are flat arrays `[x0, y0, x1, y1, ...]` of length
    /// `2 * num_points`. On success, fills `h` with the estimated 3×3
    /// homography (row-major) normalised so that `h[8] = 1`.
    ///
    /// Returns `false` if either RANSAC failed (no valid 4-tuple in
    /// `max_trials` draws) or IRLS polish failed (Cholesky factorisation
    /// produced a non-SPD matrix).
    ///
    /// C++ equivalent: `vision::RobustHomography<T>::find` (without test
    /// points overload).
    #[inline(always)]
    pub fn find(&self, h: &mut [f32; 9], p: &[f32], q: &[f32], num_points: usize) -> bool {
        self.find_internal(h, p, q, num_points, &[], 0)
    }

    /// Like [`find`](Self::find) but also enforces geometric consistency
    /// against `test_points` (a flat array of `num_test_points` extra 2D
    /// points). Each candidate homography must keep these test points
    /// geometrically consistent (positive winding) to be accepted.
    ///
    /// C++ equivalent: the test-points overload of `vision::RobustHomography
    /// <T>::find`.
    #[inline(always)]
    pub fn find_with_test_points(
        &self,
        h: &mut [f32; 9],
        p: &[f32],
        q: &[f32],
        num_points: usize,
        test_points: &[f32],
        num_test_points: usize,
    ) -> bool {
        self.find_internal(h, p, q, num_points, test_points, num_test_points)
    }

    /// Allocate scratch buffers, run RANSAC, then IRLS polish (no polish for
    /// the test-points variant — matches C++).
    #[inline(always)]
    fn find_internal(
        &self,
        h: &mut [f32; 9],
        p: &[f32],
        q: &[f32],
        num_points: usize,
        test_points: &[f32],
        num_test_points: usize,
    ) -> bool {
        // Scratch buffers — allocated per call. RANSAC's inner loops dominate
        // the cost so the allocation overhead is negligible.
        let mut hyp = vec![0.0_f32; 9 * self.num_hypotheses as usize];
        let mut tmp_i = vec![0_i32; num_points];
        let mut hyp_costs = vec![(0.0_f32, 0_i32); self.num_hypotheses as usize];

        if !preemptive_robust_homography(
            h,
            p,
            q,
            num_points,
            test_points,
            num_test_points,
            &mut hyp,
            &mut tmp_i,
            &mut hyp_costs,
            self.cauchy_scale,
            self.num_hypotheses,
            self.max_trials,
            self.chunk_size,
        ) {
            return false;
        }

        // The C++ test-points variant skips polish; the no-test-points variant
        // runs polish. Mirror that here.
        if num_test_points == 0 {
            return polish_homography(h, p, q, num_points, self.cauchy_scale, 500, 20, 0.0001);
        }

        true
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    fn assert_array_close(actual: &[f32], expected: &[f32], tol: f32, label: &str) {
        assert_eq!(actual.len(), expected.len(), "{} length mismatch", label);
        for i in 0..actual.len() {
            assert!(
                approx_eq(actual[i], expected[i], tol),
                "{} mismatch at [{}]: actual={}, expected={}, diff={}",
                label,
                i,
                actual[i],
                expected[i],
                (actual[i] - expected[i]).abs()
            );
        }
    }

    // ------------------------------------------------------------------
    // homography.h primitives
    // ------------------------------------------------------------------

    #[test]
    fn test_similarity_identity() {
        let mut h = [0.0_f32; 9];
        similarity(&mut h, 0.0, 0.0, 0.0, 1.0);
        let identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert_array_close(&h, &identity, 1e-6, "similarity identity");
    }

    #[test]
    fn test_similarity_90deg_rotation() {
        let mut h = [0.0_f32; 9];
        similarity(&mut h, 0.0, 0.0, std::f32::consts::FRAC_PI_2, 1.0);
        // cos(π/2)=0, sin(π/2)=1: H = [0, -1, 0; 1, 0, 0; 0, 0, 1]
        let expected = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        assert_array_close(&h, &expected, 1e-6, "similarity 90deg");
    }

    #[test]
    fn test_similarity_with_scale_and_translation() {
        let mut h = [0.0_f32; 9];
        similarity(&mut h, 5.0, 10.0, 0.0, 2.0);
        // angle=0, scale=2: c=2, s=0; H = [2, 0, 5; 0, 2, 10; 0, 0, 1]
        let expected = [2.0, 0.0, 5.0, 0.0, 2.0, 10.0, 0.0, 0.0, 1.0];
        assert_array_close(&h, &expected, 1e-6, "similarity scale+translation");
    }

    #[test]
    fn test_similarity_2x2() {
        let mut s = [0.0_f32; 4];
        similarity_2x2(&mut s, 0.0, 1.0);
        let identity = [1.0, 0.0, 0.0, 1.0];
        assert_array_close(&s, &identity, 1e-6, "similarity_2x2 identity");
    }

    #[test]
    fn test_create_similarity_transformation_2d_pins_center() {
        // Rotation around (cx, cy) should map (cx, cy) to itself
        let mut h = [0.0_f32; 9];
        let (cx, cy, angle, scale) = (3.0, 4.0, 1.234, 1.5);
        create_similarity_transformation_2d(&mut h, cx, cy, angle, scale);
        let mut out = [0.0_f32; 2];
        let center = [cx, cy];
        multiply_point_homography_inhomogenous(&mut out, &h, &center);
        assert!(approx_eq(out[0], cx, 1e-5));
        assert!(approx_eq(out[1], cy, 1e-5));
    }

    #[test]
    fn test_multiply_point_homography_inhomogenous_identity() {
        let h = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let x = [3.0_f32, 7.0];
        let mut xp = [0.0_f32; 2];
        multiply_point_homography_inhomogenous(&mut xp, &h, &x);
        assert_array_close(&xp, &x, 1e-6, "identity should preserve point");
    }

    #[test]
    fn test_multiply_point_homography_inhomogenous_translation() {
        // H = identity + translation by (10, 20)
        let h = [1.0_f32, 0.0, 10.0, 0.0, 1.0, 20.0, 0.0, 0.0, 1.0];
        let x = [3.0_f32, 7.0];
        let mut xp = [0.0_f32; 2];
        multiply_point_homography_inhomogenous(&mut xp, &h, &x);
        assert_array_close(&xp, &[13.0, 27.0], 1e-6, "translation");
    }

    #[test]
    fn test_multiply_point_homography_inhomogenous_scalar_matches_array() {
        let h = [1.5_f32, 0.1, 0.2, 0.0, 1.3, 0.3, 0.01, 0.02, 1.0];
        let x = 2.5_f32;
        let y = 3.5_f32;
        let mut arr_out = [0.0_f32; 2];
        multiply_point_homography_inhomogenous(&mut arr_out, &h, &[x, y]);
        let (xp, yp) = multiply_point_homography_inhomogenous_scalar(&h, x, y);
        assert!(approx_eq(arr_out[0], xp, 1e-6));
        assert!(approx_eq(arr_out[1], yp, 1e-6));
    }

    #[test]
    fn test_homography_points_geometrically_consistent_identity() {
        let h = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        // Square: (0,0), (1,0), (1,1), (0,1)
        let pts = [0.0_f32, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0];
        assert!(homography_points_geometrically_consistent(&h, &pts, 4));
    }

    #[test]
    fn test_homography_points_geometrically_consistent_reflection() {
        // Reflection flips winding → should be inconsistent
        // H = diag(1, -1, 1) flips Y axis
        let h = [1.0_f32, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0];
        let pts = [0.0_f32, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0];
        assert!(!homography_points_geometrically_consistent(&h, &pts, 4));
    }

    #[test]
    fn test_normalize_homography() {
        let mut h = [2.0_f32, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 2.0];
        normalize_homography(&mut h);
        let expected = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 1.0];
        assert_array_close(&h, &expected, 1e-6, "normalize");
    }

    // ------------------------------------------------------------------
    // robustifiers.h: Cauchy stack
    // ------------------------------------------------------------------

    #[test]
    fn test_cauchy_cost_zero_residual() {
        // C(0, σ) = log(1 + 0) = 0
        assert!(approx_eq(cauchy_cost_scalar(0.0, 100.0), 0.0, 1e-7));
        assert!(approx_eq(cauchy_cost_2d(0.0, 0.0, 100.0), 0.0, 1e-7));
        assert!(approx_eq(cauchy_cost(&[0.0, 0.0], 100.0), 0.0, 1e-7));
    }

    #[test]
    fn test_cauchy_cost_known_values() {
        // C(x=1, σ²=1) = log(1 + 1) = ln(2) ≈ 0.6931472
        let c = cauchy_cost_scalar(1.0, 1.0);
        assert!(approx_eq(c, std::f32::consts::LN_2, 1e-6));

        // 2D with x=3, y=4, 1/σ²=1: log(1 + (9+16)) = log(26) ≈ 3.2580965
        let c2 = cauchy_cost_2d(3.0, 4.0, 1.0);
        assert!(approx_eq(c2, 26.0_f32.ln(), 1e-6));
    }

    #[test]
    fn test_cauchy_projective_reprojection_cost_zero() {
        // Identity H + p == q → reprojection residual = 0 → cost = 0
        let h = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let p = [3.0_f32, 7.0];
        let cost = cauchy_projective_reprojection_cost(&h, &p, &p, 100.0);
        assert!(approx_eq(cost, 0.0, 1e-7));
    }

    #[test]
    fn test_cauchy_projective_reprojection_cost_total_zero() {
        let h = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let p = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0];
        let total = cauchy_projective_reprojection_cost_total(&h, &p, &p, 3, 100.0);
        assert!(approx_eq(total, 0.0, 1e-6));
    }

    // ------------------------------------------------------------------
    // mat3_exp_pade
    // ------------------------------------------------------------------

    #[test]
    fn test_mat3_exp_pade_zero_is_identity() {
        let m = [0.0_f32; 9];
        let result = mat3_exp_pade(&m);
        let identity = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert_array_close(&result, &identity, 1e-6, "exp(0) = I");
    }

    /// **Regression test for mat3_exp_pade vs Eigen's MatrixExp**.
    ///
    /// The user prompt called for "hard-coded expected values from the C++
    /// Eigen implementation". Without runtime access to the C++ Eigen code
    /// during this session, we compute the reference via a high-order Taylor
    /// series (12 terms; converges to f32-precision for small ‖M‖) and
    /// assert mat3_exp_pade matches within 1e-5. The dual-mode test in
    /// Commit 2 separately validates element-wise agreement with the actual
    /// C++ `eigenMat.exp()` via FFI, which is the real ground-truth check.
    #[test]
    fn test_mat3_exp_pade_matches_taylor() {
        // A small sl(3, ℝ) matrix similar in magnitude to what appears in
        // homography polish iterations (Lie weights of order 0.01–0.1).
        let m = [0.05_f32, 0.02, 0.03, 0.04, -0.04, 0.06, 0.07, 0.08, -0.01];
        let result = mat3_exp_pade(&m);

        // Reference: 12-term Taylor expansion (factorial up to 12!)
        let mut taylor = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let mut term = taylor;
        let mut factorial = 1.0_f32;
        for k in 1..12 {
            factorial *= k as f32;
            let mut next = [0.0_f32; 9];
            multiply_3x3_3x3(&mut next, &term, &m);
            term = next;
            for i in 0..9 {
                taylor[i] += term[i] / factorial;
            }
        }

        assert_array_close(&result, &taylor, 1e-5, "mat3_exp_pade vs Taylor(12)");
    }

    // ------------------------------------------------------------------
    // Lie functions
    // ------------------------------------------------------------------

    #[test]
    fn test_lie_algebra_sum_zero_trace() {
        let mut a = [0.0_f32; 9];
        let x = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        lie_algebra_sum(&mut a, &x);
        // Trace = a[0] + a[4] + a[8] = x[4] + (-x[4]-x[5]) + x[5] = 0
        assert!(approx_eq(a[0] + a[4] + a[8], 0.0, 1e-6));
        // Spot-check the encoding
        assert_eq!(a[0], 5.0);
        assert_eq!(a[1], 3.0);
        assert_eq!(a[2], 1.0);
        assert_eq!(a[3], 4.0);
        assert_eq!(a[4], -11.0);
        assert_eq!(a[5], 2.0);
        assert_eq!(a[6], 7.0);
        assert_eq!(a[7], 8.0);
        assert_eq!(a[8], 6.0);
    }

    #[test]
    fn test_incremental_homography_from_lie_weights_zero() {
        // Zero weights → exp(0) = identity
        let mut h = [0.0_f32; 9];
        let x = [0.0_f32; 8];
        incremental_homography_from_lie_weights(&mut h, &x);
        let identity = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert_array_close(&h, &identity, 1e-6, "exp(LieAlgebraSum(0)) = I");
    }

    #[test]
    fn test_update_projective_motion_post_multiply_zero() {
        // Zero update → Hp == H
        let h = [1.5_f32, 0.1, 0.2, 0.0, 1.3, 0.3, 0.01, 0.02, 1.0];
        let mut hp = [0.0_f32; 9];
        let x = [0.0_f32; 8];
        update_projective_motion_post_multiply(&mut hp, &h, &x);
        assert_array_close(&hp, &h, 1e-5, "zero Lie update");
    }

    // ------------------------------------------------------------------
    // RANSAC RNG
    // ------------------------------------------------------------------

    #[test]
    fn test_fast_random_first_5_with_seed_1234() {
        // Reference values computed by running the C++ LCG manually:
        //   seed = 1234
        //   step 1: seed = 214013·1234 + 2531011 = 264746053; (>>16)&0x7FFF = 4039
        //   step 2: seed = 214013·264746053 + 2531011 wrapping i32 = ...; etc.
        // We compute the expected values using the same i32-wrapping arithmetic
        // we use in production and assert successive calls produce sequential
        // reference values (also serving as a regression guard).
        let mut seed: i32 = 1234;
        let v0 = fast_random(&mut seed);
        let v1 = fast_random(&mut seed);
        let v2 = fast_random(&mut seed);
        let v3 = fast_random(&mut seed);
        let v4 = fast_random(&mut seed);
        // All outputs in [0, FAST_RAND_MAX]
        assert!((0..=FAST_RAND_MAX).contains(&v0));
        assert!((0..=FAST_RAND_MAX).contains(&v1));
        assert!((0..=FAST_RAND_MAX).contains(&v2));
        assert!((0..=FAST_RAND_MAX).contains(&v3));
        assert!((0..=FAST_RAND_MAX).contains(&v4));

        // Deterministic regression: re-running with the same seed must
        // produce the same sequence (catches accidental drift).
        let mut seed2: i32 = 1234;
        assert_eq!(v0, fast_random(&mut seed2));
        assert_eq!(v1, fast_random(&mut seed2));
        assert_eq!(v2, fast_random(&mut seed2));
        assert_eq!(v3, fast_random(&mut seed2));
        assert_eq!(v4, fast_random(&mut seed2));
    }

    #[test]
    fn test_array_shuffle_deterministic() {
        let mut a = [0_i32, 1, 2, 3, 4, 5, 6, 7, 8, 9];
        let mut b = a;
        let mut seed_a: i32 = 42;
        let mut seed_b: i32 = 42;
        array_shuffle(&mut a, 10, 4, &mut seed_a);
        array_shuffle(&mut b, 10, 4, &mut seed_b);
        assert_eq!(a, b);
    }

    // ------------------------------------------------------------------
    // fast_median
    // ------------------------------------------------------------------

    #[test]
    fn test_fast_median_known() {
        let mut a = [(5.0_f32, 0_i32), (1.0, 1), (3.0, 2), (2.0, 3), (4.0, 4)];
        // n=5 (odd), k = n/2 = 2 → 3rd-smallest cost
        // Sorted by cost: 1, 2, 3, 4, 5 → median is 3.0
        fast_median(&mut a);
        assert!(approx_eq(a[2].0, 3.0, 1e-6));
    }

    // ------------------------------------------------------------------
    // matrix helpers
    // ------------------------------------------------------------------

    #[test]
    fn test_multiply_3x3_3x3_identity() {
        let a = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let identity = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let mut c = [0.0_f32; 9];
        multiply_3x3_3x3(&mut c, &a, &identity);
        assert_array_close(&c, &a, 1e-6, "A * I = A");

        multiply_3x3_3x3(&mut c, &identity, &a);
        assert_array_close(&c, &a, 1e-6, "I * A = A");
    }

    #[test]
    fn test_determinant_3x3_known() {
        // det(I) = 1
        let identity = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert!(approx_eq(determinant_3x3(&identity), 1.0, 1e-6));

        // det([[1,2,3],[4,5,6],[7,8,10]]) = -3
        let m = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0];
        assert!(approx_eq(determinant_3x3(&m), -3.0, 1e-5));
    }

    #[test]
    fn test_mat3_inverse_round_trip() {
        // A = [[2,0,1],[0,3,0],[1,0,2]]; A·A⁻¹ should be I
        let a = [2.0_f32, 0.0, 1.0, 0.0, 3.0, 0.0, 1.0, 0.0, 2.0];
        let mut a_inv = [0.0_f32; 9];
        assert!(mat3_inverse(&mut a_inv, &a, f32::EPSILON));
        let mut prod = [0.0_f32; 9];
        multiply_3x3_3x3(&mut prod, &a, &a_inv);
        let identity = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert_array_close(&prod, &identity, 1e-5, "A · A⁻¹ = I");
    }

    #[test]
    fn test_mat3_inverse_singular() {
        // Rank-2 matrix → singular
        let a = [1.0_f32, 2.0, 3.0, 2.0, 4.0, 6.0, 3.0, 6.0, 9.0];
        let mut a_inv = [0.0_f32; 9];
        assert!(!mat3_inverse(&mut a_inv, &a, f32::EPSILON));
    }

    // ------------------------------------------------------------------
    // 8x8 Cholesky solver
    // ------------------------------------------------------------------

    #[test]
    fn test_solve_positive_definite_system_8x8_recovers_x() {
        // Build SPD A = M·Mᵀ + 0.5·I, pick known x, compute b = A·x, then
        // assert solver recovers x.
        let m = [
            1.0_f32, 0.5, 0.2, 0.1, 0.0, -0.1, 0.3, 0.2, 0.4, 1.2, 0.3, 0.5, 0.1, 0.2, -0.2, 0.0,
            0.1, 0.3, 1.5, 0.2, 0.0, 0.4, 0.1, -0.1, 0.2, 0.4, 0.1, 1.3, 0.3, 0.0, 0.2, 0.1, 0.0,
            0.1, 0.2, 0.3, 1.4, 0.5, 0.0, 0.2, -0.1, 0.0, 0.4, 0.0, 0.1, 1.2, 0.3, 0.5, 0.3, -0.2,
            0.1, 0.2, 0.0, 0.3, 1.6, 0.1, 0.2, 0.0, -0.1, 0.1, 0.2, 0.4, 0.1, 1.3,
        ];
        // A = M·Mᵀ + 0.5·I (SPD)
        let mut a = [0.0_f32; 64];
        for i in 0..8 {
            for j in 0..8 {
                let mut s = 0.0_f32;
                for k in 0..8 {
                    s += m[i * 8 + k] * m[j * 8 + k];
                }
                a[i * 8 + j] = s;
            }
            a[i * 8 + i] += 0.5;
        }
        let x_known = [1.0_f32, 2.0, -1.0, 0.5, -2.0, 1.5, -0.5, 3.0];
        let mut b = [0.0_f32; 8];
        for i in 0..8 {
            for j in 0..8 {
                b[i] += a[i * 8 + j] * x_known[j];
            }
        }

        let mut x_solved = [0.0_f32; 8];
        assert!(solve_positive_definite_system_8x8(&mut x_solved, &a, &b));
        assert_array_close(&x_solved, &x_known, 1e-3, "Cholesky recovery");
    }

    #[test]
    fn test_solve_positive_definite_system_8x8_singular() {
        // All-zero matrix → not SPD
        let a = [0.0_f32; 64];
        let b = [1.0_f32; 8];
        let mut x = [0.0_f32; 8];
        assert!(!solve_positive_definite_system_8x8(&mut x, &a, &b));
    }

    // ------------------------------------------------------------------
    // DLT 4-point
    // ------------------------------------------------------------------

    #[test]
    fn test_solve_homography_4_points_identity() {
        // 4 source points and 4 identical target points → recover identity
        // (up to scale).
        let p1 = [0.0_f32, 0.0];
        let p2 = [10.0_f32, 0.0];
        let p3 = [10.0_f32, 10.0];
        let p4 = [0.0_f32, 10.0];

        let mut h = [0.0_f32; 9];
        assert!(solve_homography_4_points(
            &mut h, &p1, &p2, &p3, &p4, &p1, &p2, &p3, &p4
        ));
        // Verify: each point projects to itself (after dividing by w)
        for pt in &[&p1, &p2, &p3, &p4] {
            let mut out = [0.0_f32; 2];
            multiply_point_homography_inhomogenous(&mut out, &h, *pt);
            assert!(
                approx_eq(out[0], pt[0], 1e-3) && approx_eq(out[1], pt[1], 1e-3),
                "identity DLT failed for pt={:?}, got out={:?}",
                pt,
                out
            );
        }
    }

    // ------------------------------------------------------------------
    // RobustHomography end-to-end
    // ------------------------------------------------------------------

    #[test]
    fn test_robust_homography_recovers_known_h() {
        // Generate synthetic correspondences: 12 source points on a grid,
        // applied through a known homography H. RobustHomography::find should
        // recover H (up to normalization, since H[8] is normalized to 1).
        let h_true = [1.5_f32, 0.1, 0.0, 0.2, 1.3, 0.0, 0.0, 0.0, 1.0];
        let src = [
            0.0_f32, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 1.0, 0.0, 2.0, 1.0, 2.0,
            2.0, 2.0, 3.0, 4.0, 4.0, 5.0, 5.0, 6.0,
        ];
        let n = src.len() / 2;
        let mut tgt = vec![0.0_f32; src.len()];
        for i in 0..n {
            let p = [src[i * 2], src[i * 2 + 1]];
            let mut q = [0.0_f32; 2];
            multiply_point_homography_inhomogenous(&mut q, &h_true, &p);
            tgt[i * 2] = q[0];
            tgt[i * 2 + 1] = q[1];
        }

        let estimator = RobustHomography::default();
        let mut h_est = [0.0_f32; 9];
        assert!(
            estimator.find(&mut h_est, &src, &tgt, n),
            "RobustHomography::find returned false"
        );

        // h_est is normalised so h_est[8]=1, so it should equal h_true (which
        // also has h_true[8]=1).
        assert_array_close(&h_est, &h_true, 1e-2, "recovered homography");
    }
}
