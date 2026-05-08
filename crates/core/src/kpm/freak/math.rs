/*
 *  freak/math.rs
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

//! Math utilities for the FREAK descriptor matcher.
//!
//! Ported from WebARKitLib C++ headers:
//! - `KPM/FreakMatcher/math/indexing.h` (251 lines) — vector/array indexing and manipulation
//! - `KPM/FreakMatcher/math/math_utils.h` (196 lines) — fast math approximations
//! - `KPM/FreakMatcher/math/linear_algebra.h` (400 lines) — determinants, inverses, linear solvers
//! - `KPM/FreakMatcher/math/linear_solvers.h` (411 lines) — DLT null-vector and tridiagonal solvers
//!
//! Selected private helpers are also ported from `KPM/FreakMatcher/math/matrix.h`.
//!
//! All functions are marked `#[inline(always)]` to match C++ `inline` keyword behavior
//! and enable call-site specialization via monomorphization.

use std::ops::{Add, AddAssign, Mul, Sub, SubAssign};

use crate::arlog_e;

// ============================================================================
// Constants
// ============================================================================

/// π (pi) in f64 precision.
///
/// Re-exports `std::f64::consts::PI` to match the C++ `#define PI` from
/// `math_utils.h:42`.
pub const PI: f64 = std::f64::consts::PI;

/// π (pi) in f32 precision.
///
/// Re-exports `std::f32::consts::PI` to match the C++ `#define PI` from
/// `math_utils.h:42`.
pub const PI_F: f32 = std::f32::consts::PI;

/// 1 / (2π).
///
/// Computed from `PI_F` to retain full f32 precision. Matches the C++
/// `#define ONE_OVER_2PI 0.159154943091895` from `math_utils.h:43`.
pub const ONE_OVER_2PI: f32 = 1.0 / (2.0 * PI_F);

/// √2 (square root of 2).
///
/// Re-exports `std::f32::consts::SQRT_2` to match the C++
/// `#define SQRT2 1.41421356237309504880` from `math_utils.h:44`.
pub const SQRT2: f32 = std::f32::consts::SQRT_2;

/// π / 180 — conversion factor from degrees to radians (f32)
pub const DEG_TO_RAD_F: f32 = PI_F / 180.0;

/// π / 180 — conversion factor from degrees to radians (f64)
pub const DEG_TO_RAD: f64 = PI / 180.0;

// ============================================================================
// indexing.h Functions
// ============================================================================

/// Zero out a 3-element vector.
///
/// C++ equivalent: `ZeroVector3<T>`
#[inline(always)]
pub fn zero_vector_3<T: Default>(v: &mut [T; 3]) {
    v[0] = T::default();
    v[1] = T::default();
    v[2] = T::default();
}

/// Zero out an n-element vector.
///
/// C++ equivalent: `ZeroVector<T>`
#[inline(always)]
pub fn zero_vector<T: Default>(v: &mut [T]) {
    for elem in v.iter_mut() {
        *elem = T::default();
    }
}

/// Return the maximum of two values.
///
/// C++ equivalent: `max2<T>`
#[inline(always)]
pub fn max2<T: PartialOrd>(a: T, b: T) -> T {
    if a > b {
        a
    } else {
        b
    }
}

/// Return the minimum of two values.
///
/// C++ equivalent: `min2<T>`
#[inline(always)]
pub fn min2<T: PartialOrd + Copy>(a: T, b: T) -> T {
    if a < b {
        a
    } else {
        b
    }
}

/// Return the minimum of three values.
///
/// C++ equivalent: `min3<T>`
#[inline(always)]
pub fn min3<T: PartialOrd + Copy>(a: T, b: T, c: T) -> T {
    min2(min2(a, b), c)
}

/// Return the minimum of four values.
///
/// C++ equivalent: `min4<T>`
#[inline(always)]
pub fn min4<T: PartialOrd + Copy>(a: T, b: T, c: T, d: T) -> T {
    min2(min3(a, b, c), d)
}

/// Return the index of the maximum element in a 2-element array.
///
/// C++ equivalent: `MaxIndex2<T>`
#[inline(always)]
pub fn max_index_2<T: PartialOrd>(arr: &[T; 2]) -> usize {
    if arr[0] > arr[1] {
        0
    } else {
        1
    }
}

/// Return the index of the maximum element in a 3-element array.
///
/// C++ equivalent: `MaxIndex3<T>`
#[inline(always)]
pub fn max_index_3<T: PartialOrd>(arr: &[T; 3]) -> usize {
    let mut max_idx = 0;
    if arr[1] > arr[max_idx] {
        max_idx = 1;
    }
    if arr[2] > arr[max_idx] {
        max_idx = 2;
    }
    max_idx
}

/// Return the index of the maximum element in a 4-element array.
///
/// C++ equivalent: `MaxIndex4<T>`
#[inline(always)]
pub fn max_index_4<T: PartialOrd>(arr: &[T; 4]) -> usize {
    let mut max_idx = 0;
    for i in 1..4 {
        if arr[i] > arr[max_idx] {
            max_idx = i;
        }
    }
    max_idx
}

/// Return the index of the maximum element in a 5-element array.
///
/// C++ equivalent: `MaxIndex5<T>`
#[inline(always)]
pub fn max_index_5<T: PartialOrd>(arr: &[T; 5]) -> usize {
    let mut max_idx = 0;
    for i in 1..5 {
        if arr[i] > arr[max_idx] {
            max_idx = i;
        }
    }
    max_idx
}

/// Return the index of the maximum element in a 6-element array.
///
/// C++ equivalent: `MaxIndex6<T>`
#[inline(always)]
pub fn max_index_6<T: PartialOrd>(arr: &[T; 6]) -> usize {
    let mut max_idx = 0;
    for i in 1..6 {
        if arr[i] > arr[max_idx] {
            max_idx = i;
        }
    }
    max_idx
}

/// Return the index of the maximum element in a 7-element array.
///
/// C++ equivalent: `MaxIndex7<T>`
#[inline(always)]
pub fn max_index_7<T: PartialOrd>(arr: &[T; 7]) -> usize {
    let mut max_idx = 0;
    for i in 1..7 {
        if arr[i] > arr[max_idx] {
            max_idx = i;
        }
    }
    max_idx
}

/// Return the index of the maximum element in an 8-element array.
///
/// C++ equivalent: `MaxIndex8<T>`
#[inline(always)]
pub fn max_index_8<T: PartialOrd>(arr: &[T; 8]) -> usize {
    let mut max_idx = 0;
    for i in 1..8 {
        if arr[i] > arr[max_idx] {
            max_idx = i;
        }
    }
    max_idx
}

/// Return the index of the maximum element in a 9-element array.
///
/// C++ equivalent: `MaxIndex9<T>`
#[inline(always)]
pub fn max_index_9<T: PartialOrd>(arr: &[T; 9]) -> usize {
    let mut max_idx = 0;
    for i in 1..9 {
        if arr[i] > arr[max_idx] {
            max_idx = i;
        }
    }
    max_idx
}

/// Copy a 2-element vector from source to destination.
///
/// C++ equivalent: `CopyVector2<T>`
#[inline(always)]
pub fn copy_vector_2<T: Copy>(dst: &mut [T; 2], src: &[T; 2]) {
    dst[0] = src[0];
    dst[1] = src[1];
}

/// Copy a 3-element vector from source to destination.
///
/// C++ equivalent: `CopyVector3<T>`
#[inline(always)]
pub fn copy_vector_3<T: Copy>(dst: &mut [T; 3], src: &[T; 3]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

/// Copy a 4-element vector from source to destination.
///
/// C++ equivalent: `CopyVector4<T>`
#[inline(always)]
pub fn copy_vector_4<T: Copy>(dst: &mut [T; 4], src: &[T; 4]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
}

/// Copy a 5-element vector from source to destination.
///
/// C++ equivalent: `CopyVector5<T>`
#[inline(always)]
pub fn copy_vector_5<T: Copy>(dst: &mut [T; 5], src: &[T; 5]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
    dst[4] = src[4];
}

/// Copy a 6-element vector from source to destination.
///
/// C++ equivalent: `CopyVector6<T>`
#[inline(always)]
pub fn copy_vector_6<T: Copy>(dst: &mut [T; 6], src: &[T; 6]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
    dst[4] = src[4];
    dst[5] = src[5];
}

/// Copy a 7-element vector from source to destination.
///
/// C++ equivalent: `CopyVector7<T>`
#[inline(always)]
pub fn copy_vector_7<T: Copy>(dst: &mut [T; 7], src: &[T; 7]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
    dst[4] = src[4];
    dst[5] = src[5];
    dst[6] = src[6];
}

/// Copy an 8-element vector from source to destination.
///
/// C++ equivalent: `CopyVector8<T>`
#[inline(always)]
pub fn copy_vector_8<T: Copy>(dst: &mut [T; 8], src: &[T; 8]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
    dst[4] = src[4];
    dst[5] = src[5];
    dst[6] = src[6];
    dst[7] = src[7];
}

/// Copy a 9-element vector from source to destination.
///
/// C++ equivalent: `CopyVector9<T>`
#[inline(always)]
pub fn copy_vector_9<T: Copy>(dst: &mut [T; 9], src: &[T; 9]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
    dst[4] = src[4];
    dst[5] = src[5];
    dst[6] = src[6];
    dst[7] = src[7];
    dst[8] = src[8];
}

/// Copy an n-element vector from source to destination.
///
/// C++ equivalent: `CopyVector<T>`
#[inline(always)]
pub fn copy_vector<T: Copy>(dst: &mut [T], src: &[T]) {
    dst.copy_from_slice(src);
}

/// Swap two 9-element arrays.
///
/// C++ equivalent: `Swap9<T>`
#[inline(always)]
pub fn swap_9<T>(a: &mut [T; 9], b: &mut [T; 9]) {
    std::mem::swap(a, b);
}

/// Set a specific bit in a bitstring (array of u8).
///
/// C++ equivalent: `bitstring_set_bit`
#[inline(always)]
pub fn bitstring_set_bit(bitstring: &mut [u8], pos: usize, bit: u8) {
    let byte_idx = pos / 8;
    let bit_idx = pos % 8;
    if bit != 0 {
        bitstring[byte_idx] |= 1 << bit_idx;
    } else {
        bitstring[byte_idx] &= !(1 << bit_idx);
    }
}

/// Get a specific bit from a bitstring (array of u8).
///
/// C++ equivalent: `bitstring_get_bit`
#[inline(always)]
pub fn bitstring_get_bit(bitstring: &[u8], pos: usize) -> u8 {
    let byte_idx = pos / 8;
    let bit_idx = pos % 8;
    (bitstring[byte_idx] >> bit_idx) & 1
}

/// Create a sequential vector [x0, x0+1, x0+2, ...] for f32 types.
///
/// C++ equivalent: `SequentialVector<T>` for `T = float`.
#[inline(always)]
pub fn sequential_vector_f32(v: &mut [f32], x0: f32) {
    for (i, elem) in v.iter_mut().enumerate() {
        *elem = x0 + i as f32;
    }
}

/// Create a sequential vector [x0, x0+1, x0+2, ...] for i32 types.
///
/// C++ equivalent: `SequentialVector<T>` for `T = int`.
#[inline(always)]
pub fn sequential_vector_i32(v: &mut [i32], x0: i32) {
    for (val, elem) in (x0..).zip(v.iter_mut()) {
        *elem = val;
    }
}

// ============================================================================
// math_utils.h Functions
// ============================================================================

/// Return the square of a value: x * x.
///
/// C++ equivalent: `sqr<T>`
#[inline(always)]
pub fn sqr<T: std::ops::Mul<Output = T> + Copy>(x: T) -> T {
    x * x
}

/// Round a floating-point value to the nearest integer.
///
/// Implements: floor(x + 0.5)
///
/// C++ equivalent: `round<T>`
#[inline(always)]
pub fn round_f32(x: f32) -> f32 {
    (x + 0.5).floor()
}

/// Round a f64 value to the nearest integer.
#[inline(always)]
pub fn round_f64(x: f64) -> f64 {
    (x + 0.5).floor()
}

/// Compute log base 2 of a value.
///
/// C++ equivalent: `log2<T>`
#[inline(always)]
pub fn log2_f32(x: f32) -> f32 {
    x.log2()
}

/// Compute log base 2 of a f64 value.
#[inline(always)]
pub fn log2_f64(x: f64) -> f64 {
    x.log2()
}

/// Compute log base b of a value.
///
/// C++ equivalent: `logb<T>`
#[inline(always)]
pub fn logb_f32(x: f32, b: f32) -> f32 {
    x.ln() / b.ln()
}

/// Compute log base b of a f64 value.
#[inline(always)]
pub fn logb_f64(x: f64, b: f64) -> f64 {
    x.ln() / b.ln()
}

/// Safe reciprocal: 1/x, returns 1 if x == 0.
///
/// C++ equivalent: `SafeReciprical<T>`
#[inline(always)]
pub fn safe_reciprocal_f32(x: f32) -> f32 {
    if x == 0.0 {
        1.0
    } else {
        1.0 / x
    }
}

/// Safe reciprocal: 1/x, returns 1 if x == 0 (f64 version).
#[inline(always)]
pub fn safe_reciprocal_f64(x: f64) -> f64 {
    if x == 0.0 {
        1.0
    } else {
        1.0 / x
    }
}

/// Safe division: x/y, returns x if y == 0.
///
/// C++ equivalent: `SafeDivision<T>`
#[inline(always)]
pub fn safe_division_f32(x: f32, y: f32) -> f32 {
    if y == 0.0 {
        x
    } else {
        x / y
    }
}

/// Safe division: x/y, returns x if y == 0 (f64 version).
#[inline(always)]
pub fn safe_division_f64(x: f64, y: f64) -> f64 {
    if y == 0.0 {
        x
    } else {
        x / y
    }
}

/// Clamp a scalar value to a range [min, max].
///
/// C++ equivalent: `ClipScalar<T>`
#[inline(always)]
pub fn clip_scalar_f32(x: f32, min: f32, max: f32) -> f32 {
    if x < min {
        min
    } else if x > max {
        max
    } else {
        x
    }
}

/// Clamp a scalar value to a range [min, max] (f64 version).
#[inline(always)]
pub fn clip_scalar_f64(x: f64, min: f64, max: f64) -> f64 {
    if x < min {
        min
    } else if x > max {
        max
    } else {
        x
    }
}

/// Convert degrees to radians.
///
/// C++ equivalent: `deg2rad<T>`
#[inline(always)]
pub fn deg2rad_f32(deg: f32) -> f32 {
    deg * DEG_TO_RAD_F
}

/// Convert degrees to radians (f64 version).
#[inline(always)]
pub fn deg2rad_f64(deg: f64) -> f64 {
    deg * DEG_TO_RAD
}

/// Fast atan2 approximation using polynomial coefficients.
///
/// Returns angle in radians in the range [-π, π].
/// Uses polynomial approximation: angle += (0.1821*r² - 0.9675)*r
///
/// C++ equivalent: `fastatan2` — defined in `math_utils.h:93` but **never
/// called** from anywhere in the upstream WebARKitLib FreakMatcher
/// (verified at submodule rev 656436e). Ported here for completeness and
/// future use; dual-mode validation against the C++ baseline still applies.
#[inline(always)]
pub fn fast_atan2(y: f32, x: f32) -> f32 {
    let abs_y = y.abs() + 1e-7;

    if x == 0.0 && y == 0.0 {
        0.0
    } else if x > 0.0 {
        let r = (x - abs_y) / (x + abs_y);
        let mut angle = PI_F / 4.0;
        angle += (0.1821 * r * r - 0.9675) * r;
        if y < 0.0 {
            -angle
        } else {
            angle
        }
    } else {
        let r = (x + abs_y) / (abs_y - x);
        let mut angle = 3.0 * PI_F / 4.0;
        angle += (0.1821 * r * r - 0.9675) * r;
        if y < 0.0 {
            -angle
        } else {
            angle
        }
    }
}

/// Fast atan2 returning degrees in [0, 360].
///
/// C++ equivalent: `fastatan2_360` — defined in `math_utils.h:116` but
/// **never called** from anywhere in the upstream WebARKitLib FreakMatcher
/// (verified at submodule rev 656436e). The original C++ also returns
/// radians despite its `_360` suffix; our Rust version converts to degrees
/// in [0, 360] to match the apparent intent.
#[inline(always)]
pub fn fast_atan2_360(y: f32, x: f32) -> f32 {
    let rad = fast_atan2(y, x);
    let deg = rad / DEG_TO_RAD_F;
    if deg < 0.0 {
        deg + 360.0
    } else {
        deg
    }
}

/// Fast square root using the Quake fast inverse square root algorithm.
///
/// Computes √x with low precision but high speed. Uses bit manipulation
/// to compute 1/√x via the magic constant 0x5f3759df, then applies
/// Newton-Raphson refinement, and finally multiplies by x to get √x.
///
/// C++ equivalent: `fastsqrt1` — defined in `math_utils.h:142` but **never
/// called** from anywhere in the upstream WebARKitLib FreakMatcher
/// (verified at submodule rev 656436e).
///
/// # Upstream divergence
///
/// The C++ source contains an apparent bug: `u.x = (int)x;` casts the
/// input float through `int` (truncating to the integer part) **before**
/// applying the magic-constant trick. This means the Quake bit-manipulation
/// operates on the bits of `(float)truncate(x)` rather than the bits of `x`,
/// losing fractional-input precision. Our Rust port uses `x.to_bits()`
/// directly — the canonical Quake algorithm — so it produces more accurate
/// results. The dual-mode test in this module observes ~28% relative error
/// at fractional inputs and asserts only a loose bound (0.5) to document
/// rather than mask the divergence. If you ever wire this up against the
/// C++ baseline in production, replicate the (int) cast in Rust to match
/// upstream bit-for-bit; otherwise this Rust implementation is preferable.
#[inline(always)]
pub fn fast_sqrt_inv(x: f32) -> f32 {
    // SAFETY: uses transmute for float bit manipulation via to_bits/from_bits,
    // safe because input is finite positive f32 and we're just reinterpreting bits
    let xhalf = 0.5 * x;
    let i = x.to_bits();
    let i = 0x5f3759df - (i >> 1);
    let y = f32::from_bits(i);
    let y = y * (1.5 - xhalf * y * y);
    x * y
}

/// Fast 6-term Taylor series approximation of exp(x).
///
/// Coefficients: [720, 360, 120, 30, 6, 1] corresponding to x^5, x^4, ..., x^0 terms.
///
/// C++ equivalent: `fastexp6<T>` — this is the **only** function in
/// `math_utils.h` that is actually called from the upstream FreakMatcher
/// algorithm (used at `detectors/orientation_assignment.cpp:186` for
/// Gaussian weighting during keypoint orientation assignment). Dual-mode
/// validation confirms agreement with the C++ baseline to ~8.5e-7 relative
/// error across the input range.
#[inline(always)]
pub fn fast_exp6_f32(x: f32) -> f32 {
    1.0 + x
        * (1.0
            + x * (0.5
                + x * (1.0 / 6.0 + x * (1.0 / 24.0 + x * (1.0 / 120.0 + x * (1.0 / 720.0))))))
}

/// Fast 6-term Taylor series approximation of exp(x) (f64 version).
#[inline(always)]
pub fn fast_exp6_f64(x: f64) -> f64 {
    1.0 + x
        * (1.0
            + x * (0.5
                + x * (1.0 / 6.0 + x * (1.0 / 24.0 + x * (1.0 / 120.0 + x * (1.0 / 720.0))))))
}

// ============================================================================
// linear_algebra.h Functions
// ============================================================================
//
// Ported from `KPM/FreakMatcher/math/linear_algebra.h` (400 lines). Selected
// private helpers from `KPM/FreakMatcher/math/matrix.h` are also included.
//
// Live entrypoints in upstream (verified at submodule rev 656436e36b):
//   - SolveLinearSystem2x2          → detectors/harris.cpp:633
//   - SolveSymmetricLinearSystem3x3 → detectors/DoG_scale_invariant_detector.cpp:510
// All other functions in this section are either reachable transitively from
// the live entrypoints, or are flagged as upstream dead code in their doc
// comments (matching M6-1's treatment of `fastsqrt1`).

// ---------- Trivial vector ops (generic) ----------

/// Compute the dot product of two 4-element vectors.
///
/// C++ equivalent: `DotProduct4<T>` — used at `detectors/harris.cpp` for
/// gradient-product accumulation.
#[inline(always)]
pub fn dot_product_4<T>(a: &[T; 4], b: &[T; 4]) -> T
where
    T: Mul<Output = T> + Add<Output = T> + Copy,
{
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

/// Compute the dot product of two 9-element vectors.
///
/// C++ equivalent: `DotProduct9<T>` — reached transitively via
/// `accumulate_projection_9` inside `solve_null_vector_8x9_destructive`.
#[inline(always)]
pub fn dot_product_9<T>(a: &[T; 9], b: &[T; 9]) -> T
where
    T: Mul<Output = T> + Add<Output = T> + Copy,
{
    a[0] * b[0]
        + a[1] * b[1]
        + a[2] * b[2]
        + a[3] * b[3]
        + a[4] * b[4]
        + a[5] * b[5]
        + a[6] * b[6]
        + a[7] * b[7]
        + a[8] * b[8]
}

/// Compute the sum of squares of a 9-element vector.
///
/// C++ equivalent: `SumSquares9<T>` — reached transitively inside
/// `solve_null_vector_8x9_destructive` for column-norm pivoting.
#[inline(always)]
pub fn sum_squares_9<T>(x: &[T; 9]) -> T
where
    T: Mul<Output = T> + Add<Output = T> + Copy,
{
    dot_product_9(x, x)
}

/// Scale a 4-element vector: `dst = src * s`.
///
/// C++ equivalent: `ScaleVector4<T>`.
#[inline(always)]
pub fn scale_vector_4<T>(dst: &mut [T; 4], src: &[T; 4], s: T)
where
    T: Mul<Output = T> + Copy,
{
    dst[0] = src[0] * s;
    dst[1] = src[1] * s;
    dst[2] = src[2] * s;
    dst[3] = src[3] * s;
}

/// Scale an 8-element vector: `dst = src * s`.
///
/// C++ equivalent: `ScaleVector8<T>`.
#[inline(always)]
pub fn scale_vector_8<T>(dst: &mut [T; 8], src: &[T; 8], s: T)
where
    T: Mul<Output = T> + Copy,
{
    dst[0] = src[0] * s;
    dst[1] = src[1] * s;
    dst[2] = src[2] * s;
    dst[3] = src[3] * s;
    dst[4] = src[4] * s;
    dst[5] = src[5] * s;
    dst[6] = src[6] * s;
    dst[7] = src[7] * s;
}

/// Scale a 9-element vector: `dst = src * s`.
///
/// C++ equivalent: `ScaleVector9<T>` — reached transitively inside
/// `solve_null_vector_8x9_destructive` for basis-vector normalization.
#[inline(always)]
pub fn scale_vector_9<T>(dst: &mut [T; 9], src: &[T; 9], s: T)
where
    T: Mul<Output = T> + Copy,
{
    dst[0] = src[0] * s;
    dst[1] = src[1] * s;
    dst[2] = src[2] * s;
    dst[3] = src[3] * s;
    dst[4] = src[4] * s;
    dst[5] = src[5] * s;
    dst[6] = src[6] * s;
    dst[7] = src[7] * s;
    dst[8] = src[8] * s;
}

/// Accumulate a scaled 9-element vector: `dst += src * s`.
///
/// C++ equivalent: `AccumulateScaledVector9<T>` — reached transitively inside
/// `solve_null_vector_8x9_destructive` for the final null-vector recovery.
#[inline(always)]
pub fn accumulate_scaled_vector_9<T>(dst: &mut [T; 9], src: &[T; 9], s: T)
where
    T: Mul<Output = T> + AddAssign + Copy,
{
    dst[0] += src[0] * s;
    dst[1] += src[1] * s;
    dst[2] += src[2] * s;
    dst[3] += src[3] * s;
    dst[4] += src[4] * s;
    dst[5] += src[5] * s;
    dst[6] += src[6] * s;
    dst[7] += src[7] * s;
    dst[8] += src[8] * s;
}

/// Compute the linear combination `w = u*a + v*b` of two 4-element vectors.
///
/// C++ equivalent: `AddScaledVectors4<T>`.
#[inline(always)]
pub fn add_scaled_vectors_4<T>(w: &mut [T; 4], u: &[T; 4], v: &[T; 4], a: T, b: T)
where
    T: Mul<Output = T> + Add<Output = T> + Copy,
{
    w[0] = a * u[0] + b * v[0];
    w[1] = a * u[1] + b * v[1];
    w[2] = a * u[2] + b * v[2];
    w[3] = a * u[3] + b * v[3];
}

/// Update an upper-triangular 2×2 outer product accumulator: `A += x * x^T`.
///
/// Only writes the 3 unique entries of the symmetric 2×2 matrix
/// (`A[0]`, `A[1]`, `A[3]`); `A[2]` is left untouched (mirror of `A[1]`).
///
/// C++ equivalent: `UpdateOuterProduct2x2<T>` — defined in `linear_algebra.h:351`
/// but **never called** anywhere in the upstream WebARKitLib FreakMatcher
/// (verified at submodule rev 656436e36b). Ported for completeness.
#[inline(always)]
pub fn update_outer_product_2x2<T>(a: &mut [T; 4], x: &[T; 2])
where
    T: Mul<Output = T> + AddAssign + Copy,
{
    a[0] += x[0] * x[0];
    a[1] += x[0] * x[1];
    a[3] += x[1] * x[1];
}

/// Accumulate a Gauss-Newton normal-equation right-hand side: `b -= J * residual`.
///
/// C++ equivalent: `UpdateGaussNewtonOperations2x2<T>` — defined in
/// `linear_algebra.h:361` but **never called** anywhere in the upstream
/// WebARKitLib FreakMatcher (verified at submodule rev 656436e36b).
/// Ported for completeness.
#[inline(always)]
pub fn update_gauss_newton_operations_2x2<T>(b: &mut [T; 2], j: &[T; 2], residual: T)
where
    T: Mul<Output = T> + SubAssign + Copy,
{
    b[0] -= j[0] * residual;
    b[1] -= j[1] * residual;
}

// ---------- Cofactor / determinant helpers (private) ----------

/// 4-arg cofactor of a 2×2 matrix `[[a, b], [c, d]]`: `a*d - b*c`.
///
/// C++ equivalent: `Cofactor2x2(a, b, c, d)` from `linear_algebra.h:72`.
#[inline(always)]
fn cofactor_2x2<T>(a: T, b: T, c: T, d: T) -> T
where
    T: Mul<Output = T> + Sub<Output = T> + Copy,
{
    a * d - b * c
}

/// 3-arg cofactor of a symmetric 2×2 matrix `[[a, b], [b, c]]`: `a*c - b*b`.
///
/// C++ equivalent: `Cofactor2x2(a, b, c)` overload from `linear_algebra.h:96`.
#[inline(always)]
fn cofactor_2x2_sym<T>(a: T, b: T, c: T) -> T
where
    T: Mul<Output = T> + Sub<Output = T> + Copy,
{
    a * c - b * b
}

/// Determinant of a 2×2 matrix laid out row-major as `[A00, A01, A10, A11]`.
///
/// C++ equivalent: `Determinant2x2<T>` from `linear_algebra.h:104`.
#[inline(always)]
fn determinant_2x2<T>(a: &[T; 4]) -> T
where
    T: Mul<Output = T> + Sub<Output = T> + Copy,
{
    cofactor_2x2(a[0], a[1], a[2], a[3])
}

/// Determinant of a symmetric 3×3 matrix laid out row-major.
///
/// `det(A) = -A[8]*A[1]^2 + 2*A[1]*A[2]*A[5] - A[4]*A[2]^2 - A[0]*A[5]^2 + A[0]*A[4]*A[8]`
///
/// Caller must ensure the input is symmetric (`A[1] == A[3]`, `A[2] == A[6]`,
/// `A[5] == A[7]`); only the upper-triangular entries are read.
///
/// C++ equivalent: `DeterminantSymmetric3x3<T>` from `linear_algebra.h:122` —
/// reached transitively via `matrix_inverse_symmetric_3x3` inside
/// `solve_symmetric_linear_system_3x3` (live at
/// `detectors/DoG_scale_invariant_detector.cpp:510`).
#[inline(always)]
pub fn determinant_symmetric_3x3(a: &[f32; 9]) -> f32 {
    -a[8] * sqr(a[1]) + 2.0 * a[1] * a[2] * a[5] - a[4] * sqr(a[2]) - a[0] * sqr(a[5])
        + a[0] * a[4] * a[8]
}

// ---------- Matrix-vector multiply helpers (private, from matrix.h) ----------

/// Multiply a 2×2 matrix by a 2-vector: `y = A * x`.
///
/// C++ equivalent: `Multiply_2x2_2x1<T>` from `matrix.h:73`.
#[inline(always)]
fn multiply_2x2_2x1<T>(y: &mut [T; 2], a: &[T; 4], x: &[T; 2])
where
    T: Mul<Output = T> + Add<Output = T> + Copy,
{
    y[0] = a[0] * x[0] + a[1] * x[1];
    y[1] = a[2] * x[0] + a[3] * x[1];
}

/// Multiply a 3×3 matrix by a 3-vector: `y = A * x`.
///
/// C++ equivalent: `Multiply_3x3_3x1<T>` from `matrix.h:82`.
#[inline(always)]
fn multiply_3x3_3x1<T>(y: &mut [T; 3], a: &[T; 9], x: &[T; 3])
where
    T: Mul<Output = T> + Add<Output = T> + Copy,
{
    y[0] = a[0] * x[0] + a[1] * x[1] + a[2] * x[2];
    y[1] = a[3] * x[0] + a[4] * x[1] + a[5] * x[2];
    y[2] = a[6] * x[0] + a[7] * x[1] + a[8] * x[2];
}

// ---------- Matrix inverse helpers (private) ----------

/// Compute the inverse of a 2×2 matrix `A` into `B`. Returns `false` if
/// `|det A| <= threshold`.
///
/// C++ equivalent: `MatrixInverse2x2<T>` from `linear_algebra.h:135`.
#[inline(always)]
fn matrix_inverse_2x2(b: &mut [f32; 4], a: &[f32; 4], threshold: f32) -> bool {
    let det = determinant_2x2(a);
    if det.abs() <= threshold {
        return false;
    }
    let inv_det = 1.0 / det;
    b[0] = a[3] * inv_det;
    b[1] = -a[1] * inv_det;
    b[2] = -a[2] * inv_det;
    b[3] = a[0] * inv_det;
    true
}

/// Compute the inverse of a symmetric 3×3 matrix `A` into `B`. Returns `false`
/// if `|det A| <= threshold`. Caller must ensure `A` is symmetric.
///
/// C++ equivalent: `MatrixInverseSymmetric3x3<T>` from `linear_algebra.h:182`.
#[inline(always)]
fn matrix_inverse_symmetric_3x3(b: &mut [f32; 9], a: &[f32; 9], threshold: f32) -> bool {
    let det = determinant_symmetric_3x3(a);
    if det.abs() <= threshold {
        return false;
    }
    let inv_det = 1.0 / det;

    b[0] = cofactor_2x2_sym(a[4], a[5], a[8]) * inv_det;
    b[1] = cofactor_2x2(a[2], a[1], a[8], a[7]) * inv_det;
    b[2] = cofactor_2x2(a[1], a[2], a[4], a[5]) * inv_det;
    b[4] = cofactor_2x2_sym(a[0], a[2], a[8]) * inv_det;
    b[5] = cofactor_2x2(a[2], a[0], a[5], a[3]) * inv_det;
    b[8] = cofactor_2x2_sym(a[0], a[1], a[4]) * inv_det;

    // Symmetric mirror entries
    b[3] = b[1];
    b[6] = b[2];
    b[7] = b[5];

    true
}

// ---------- Public linear solvers ----------

/// Solve a 2×2 linear system `A * x = b` for `x`. Returns `false` if `A` is
/// singular (`|det A| <= f32::EPSILON`); the failure is logged via `arlog_e!`.
///
/// `A` is laid out row-major as `[A00, A01, A10, A11]`.
///
/// C++ equivalent: `SolveLinearSystem2x2<T>` from `linear_algebra.h:210`.
/// The only upstream call site is `detectors/harris.cpp:633` (sub-pixel
/// refinement of Harris corners), but **`harris.cpp` is not part of our
/// compiled C++ subset** — `crates/core/build.rs` only compiles the DoG
/// pipeline (`DoG_scale_invariant_detector.cpp`, `pyramid.cpp`,
/// `gradients.cpp`, `orientation_assignment.cpp`, `freak.cpp`,
/// `hough_similarity_voting.cpp`, plus the visual-database facade), and no
/// compiled file includes `harris.h`. So this function is effectively dead
/// code in the WebARKitLib-rs build context. Ported for completeness; same
/// treatment as M6-1's `fastsqrt1`.
#[inline(always)]
pub fn solve_linear_system_2x2(x: &mut [f32; 2], a: &[f32; 4], b: &[f32; 2]) -> bool {
    let mut a_inv = [0.0_f32; 4];
    if !matrix_inverse_2x2(&mut a_inv, a, f32::EPSILON) {
        arlog_e!(
            "solve_linear_system_2x2: matrix is singular (|det| <= {})",
            f32::EPSILON
        );
        return false;
    }
    multiply_2x2_2x1(x, &a_inv, b);
    true
}

/// Solve a symmetric 3×3 linear system `A * x = b` for `x`. Returns `false`
/// if `A` is singular (`|det A| <= f32::EPSILON`); the failure is logged via
/// `arlog_e!`.
///
/// Caller must ensure `A` is symmetric (only the upper-triangular entries are
/// read).
///
/// C++ equivalent: `SolveSymmetricLinearSystem3x3<T>` from `linear_algebra.h:228`
/// — **live** at `detectors/DoG_scale_invariant_detector.cpp:510` (sub-pixel
/// refinement of DoG keypoints).
#[inline(always)]
pub fn solve_symmetric_linear_system_3x3(x: &mut [f32; 3], a: &[f32; 9], b: &[f32; 3]) -> bool {
    let mut a_inv = [0.0_f32; 9];
    if !matrix_inverse_symmetric_3x3(&mut a_inv, a, f32::EPSILON) {
        arlog_e!(
            "solve_symmetric_linear_system_3x3: matrix is singular (|det| <= {})",
            f32::EPSILON
        );
        return false;
    }
    multiply_3x3_3x1(x, &a_inv, b);
    true
}

// ============================================================================
// linear_solvers.h Functions
// ============================================================================
//
// Ported from `KPM/FreakMatcher/math/linear_solvers.h` (411 lines).
//
// Live entrypoint in upstream:
//   - SolveNullVector8x9Destructive → homography_estimation/homography_solver.h:197
//
// `SolveTridiagonalDestructive` is defined in `linear_solvers.h:385` but
// never called anywhere in upstream — ported for completeness.

/// Project a 9-vector `a` onto the orthogonal complement of a normalized basis
/// vector `e`, accumulating into `x`: `x -= dot(a, e) * e`.
///
/// C++ equivalent: `AccumulateProjection9<T>` from `linear_solvers.h:50` —
/// reached transitively inside `solve_null_vector_8x9_destructive` during
/// Gram-Schmidt orthogonalization.
#[inline(always)]
pub fn accumulate_projection_9<T>(x: &mut [T; 9], e: &[T; 9], a: &[T; 9])
where
    T: Mul<Output = T> + Add<Output = T> + SubAssign + Copy,
{
    let d = dot_product_9(a, e);
    x[0] -= d * e[0];
    x[1] -= d * e[1];
    x[2] -= d * e[2];
    x[3] -= d * e[3];
    x[4] -= d * e[4];
    x[5] -= d * e[5];
    x[6] -= d * e[6];
    x[7] -= d * e[7];
    x[8] -= d * e[8];
}

/// One step of column-pivoted Gram-Schmidt on the 8×9 matrix used by the DLT
/// homography solver.
///
/// Consolidates the 8 C++ functions `OrthogonalizePivot8x9Basis0..7` from
/// `linear_solvers.h:69-260` into a single function parameterized by `step`
/// (0..=7). Step 0 is special-cased: there is no previous basis to project
/// against, and the function additionally seeds `q[9..72]` from `a[9..72]`.
/// Steps 1..=7 follow a uniform pattern: project the remaining `8 - step`
/// columns onto the previous basis vector, find the pivot column (largest
/// squared-norm), swap it to position `step` (in both `q` and `a` so that the
/// homography reconstruction still tracks the input correspondences), and
/// normalize the pivot column in `q`.
///
/// Returns `false` if the pivot squared-norm is exactly zero (rank-deficient
/// system); the caller logs at the public solver level.
#[inline(always)]
fn orthogonalize_pivot_8x9_basis(step: usize, q: &mut [f32; 72], a: &mut [f32; 72]) -> bool {
    debug_assert!(step < 8);

    // Step 0 — no previous basis; seed Q from A.
    if step == 0 {
        let mut ss = [0.0_f32; 8];
        for (i, ss_i) in ss.iter_mut().enumerate() {
            let base = i * 9;
            let mut s = 0.0_f32;
            for j in 0..9 {
                s += a[base + j] * a[base + j];
            }
            *ss_i = s;
        }
        let pivot = (1..8).fold(0_usize, |best, i| if ss[i] > ss[best] { i } else { best });
        if ss[pivot] == 0.0 {
            return false;
        }
        if pivot > 0 {
            for j in 0..9 {
                a.swap(j, pivot * 9 + j);
            }
        }
        let inv_norm = 1.0 / ss[pivot].sqrt();
        for j in 0..9 {
            q[j] = a[j] * inv_norm;
        }
        // Seed remaining columns of Q from A
        q[9..72].copy_from_slice(&a[9..72]);
        return true;
    }

    let base = step * 9;
    let prev_base = base - 9;
    let remaining = 8 - step;

    // Project remaining columns onto the previous basis q[prev_base..base]:
    //   for col in step..8: q[col*9..col*9+9] -= dot(a[col*9..], q[prev_base..]) * q[prev_base..]
    for col in step..8 {
        let col_base = col * 9;
        let mut d = 0.0_f32;
        for j in 0..9 {
            d += a[col_base + j] * q[prev_base + j];
        }
        for j in 0..9 {
            q[col_base + j] -= d * q[prev_base + j];
        }
    }

    // Find pivot among remaining columns of Q (largest squared-norm).
    let mut ss = [0.0_f32; 7];
    for (i, ss_i) in ss.iter_mut().take(remaining).enumerate() {
        let col_base = (step + i) * 9;
        let mut s = 0.0_f32;
        for j in 0..9 {
            s += q[col_base + j] * q[col_base + j];
        }
        *ss_i = s;
    }
    let pivot = (1..remaining).fold(0_usize, |best, i| if ss[i] > ss[best] { i } else { best });
    if ss[pivot] == 0.0 {
        return false;
    }

    // Swap pivot to position `step` in both Q and A.
    if pivot > 0 {
        for j in 0..9 {
            q.swap(base + j, base + pivot * 9 + j);
            a.swap(base + j, base + pivot * 9 + j);
        }
    }

    // Normalize the pivot column.
    let inv_norm = 1.0 / ss[pivot].sqrt();
    for j in 0..9 {
        q[base + j] *= inv_norm;
    }

    true
}

/// Orthogonalize the `i`-th identity row against the basis `Q`, returning the
/// pre-normalization residual norm. The orthogonalized residual is written
/// into `x` (normalized to unit length); returns 0.0 if the residual collapses
/// to zero.
///
/// C++ equivalent: `OrthogonalizeIdentity8x9<T>(x, Q, i)` from
/// `linear_solvers.h:294` and the `OrthogonalizeIdentityRow0` special case
/// from `linear_solvers.h:270` (which is just this function with `i = 0`).
#[inline(always)]
fn orthogonalize_identity_row(x: &mut [f32; 9], q: &[f32; 72], i: usize) -> f32 {
    // x = e_i - sum_{k=0..8} Q[k*9 + i] * Q[k*9..k*9+9]
    //   where e_i is the i-th identity column (1 at position i, 0 elsewhere).
    for j in 0..9 {
        x[j] = -q[i] * q[j];
    }
    x[i] += 1.0;

    for k in 1..8 {
        let q_base = k * 9;
        let s = -q[q_base + i];
        for j in 0..9 {
            x[j] += s * q[q_base + j];
        }
    }

    let ss = sum_squares_9(x);
    if ss == 0.0 {
        return 0.0;
    }
    let w = ss.sqrt();
    let inv_w = 1.0 / w;
    for elem in x.iter_mut() {
        *elem *= inv_w;
    }
    w
}

/// Recover the null vector of the orthonormal basis `Q[72]` (8 basis vectors
/// of length 9). For each of the 9 identity columns, project it onto the
/// orthogonal complement of `Q`; the column with the largest residual norm is
/// the null direction.
///
/// Returns `false` if all residuals collapse to zero (degenerate system).
///
/// C++ equivalent: `OrthogonalizeIdentity8x9<T>(x, Q)` dispatcher from
/// `linear_solvers.h:317`.
#[inline(always)]
fn orthogonalize_identity_8x9(x: &mut [f32; 9], q: &[f32; 72]) -> bool {
    let mut all_x = [[0.0_f32; 9]; 9];
    let mut all_w = [0.0_f32; 9];

    for i in 0..9 {
        all_w[i] = orthogonalize_identity_row(&mut all_x[i], q, i);
    }

    let pivot = max_index_9(&all_w);
    if all_w[pivot] == 0.0 {
        return false;
    }
    *x = all_x[pivot];
    true
}

/// Solve for the null vector `x` of an 8×9 matrix `A` such that `A * x = 0`.
/// The matrix `A` is destroyed in the process (column-swaps during pivoting).
/// Uses QR decomposition via column-pivoted modified Gram-Schmidt.
///
/// `A` is laid out row-major: 8 rows of 9 elements each, total 72 elements.
///
/// **Sign ambiguity**: a null vector is defined only up to sign. Different
/// implementations (or even the same implementation on different inputs) may
/// produce vectors that differ by a global sign flip. Callers comparing to a
/// reference implementation should compare `|dot(x_a, x_b)|` to `1.0`, not
/// element-wise.
///
/// Returns `false` if a column is rank-deficient (squared-norm exactly zero
/// at any Gram-Schmidt step). Failure is logged via `arlog_e!`.
///
/// C++ equivalent: `SolveNullVector8x9Destructive<T>` from
/// `linear_solvers.h:349` — **live** at
/// `homography_estimation/homography_solver.h:197` as the algorithmic core
/// of the DLT homography solver.
#[inline(always)]
pub fn solve_null_vector_8x9_destructive(x: &mut [f32; 9], a: &mut [f32; 72]) -> bool {
    let mut q = [0.0_f32; 72];
    for step in 0..8 {
        if !orthogonalize_pivot_8x9_basis(step, &mut q, a) {
            arlog_e!(
                "solve_null_vector_8x9_destructive: rank-deficient at step {}",
                step
            );
            return false;
        }
    }
    if !orthogonalize_identity_8x9(x, &q) {
        arlog_e!("solve_null_vector_8x9_destructive: identity-orthogonalization failed");
        return false;
    }
    true
}

/// Solve a tridiagonal linear system `A * x = v` using the Thomas algorithm.
///
/// Layout:
/// - `a` (length `n - 1`): below-diagonal entries `a[i] = A[i+1, i]`
/// - `b` (length `n`): diagonal entries `b[i] = A[i, i]`
/// - `c` (length `n`, mutated): above-diagonal entries `c[i] = A[i, i+1]`,
///   over-allocated by one (last entry unused but writable for the algorithm)
/// - `x` (length `n`): on entry, the right-hand side `v`; on exit, the solution
///
/// Returns `false` if `b[0] == 0` or any forward-elimination divisor is zero.
/// Failure is logged via `arlog_e!`.
///
/// C++ equivalent: `SolveTridiagonalDestructive<T>` from `linear_solvers.h:385`
/// — defined but **never called** anywhere in the upstream WebARKitLib
/// FreakMatcher (verified at submodule rev 656436e36b). Ported for completeness.
#[inline(always)]
pub fn solve_tridiagonal_destructive(x: &mut [f32], a: &[f32], b: &[f32], c: &mut [f32]) -> bool {
    let n = x.len();
    debug_assert_eq!(
        b.len(),
        n,
        "solve_tridiagonal_destructive: b.len() must equal x.len()"
    );
    debug_assert_eq!(
        c.len(),
        n,
        "solve_tridiagonal_destructive: c.len() must equal x.len()"
    );
    debug_assert_eq!(
        a.len(),
        n - 1,
        "solve_tridiagonal_destructive: a.len() must equal x.len() - 1"
    );

    if b[0] == 0.0 {
        arlog_e!("solve_tridiagonal_destructive: b[0] is zero (no leading pivot)");
        return false;
    }
    c[0] /= b[0];
    x[0] /= b[0];

    // Forward elimination
    for i in 1..n {
        let d = b[i] - a[i - 1] * c[i - 1];
        if d == 0.0 {
            arlog_e!(
                "solve_tridiagonal_destructive: zero divisor at row {} (singular system)",
                i
            );
            return false;
        }
        let m = 1.0 / d;
        c[i] *= m;
        x[i] = (x[i] - a[i - 1] * x[i - 1]) * m;
    }

    // Back substitution
    for i in (0..n - 1).rev() {
        x[i] -= c[i] * x[i + 1];
    }

    true
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON_F32: f32 = 1e-5;

    // ========================================================================
    // indexing.h Tests
    // ========================================================================

    #[test]
    fn test_zero_vector_3() {
        let mut v: [f32; 3] = [1.0, 2.0, 3.0];
        zero_vector_3(&mut v);
        assert_eq!(v, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_zero_vector() {
        let mut v: [f32; 5] = [1.0, 2.0, 3.0, 4.0, 5.0];
        zero_vector(&mut v);
        for elem in &v {
            assert_eq!(*elem, 0.0);
        }
    }

    #[test]
    fn test_max2() {
        assert_eq!(max2(3, 7), 7);
        assert_eq!(max2(7, 3), 7);
        assert_eq!(max2(5, 5), 5);
    }

    #[test]
    fn test_min2() {
        assert_eq!(min2(3, 7), 3);
        assert_eq!(min2(7, 3), 3);
        assert_eq!(min2(5, 5), 5);
    }

    #[test]
    fn test_min3() {
        assert_eq!(min3(3, 1, 5), 1);
        assert_eq!(min3(5, 3, 1), 1);
    }

    #[test]
    fn test_min4() {
        assert_eq!(min4(3, 1, 5, 2), 1);
        assert_eq!(min4(5, 3, 1, 2), 1);
    }

    #[test]
    fn test_max_index_2() {
        assert_eq!(max_index_2(&[3.0, 7.0]), 1);
        assert_eq!(max_index_2(&[7.0, 3.0]), 0);
    }

    #[test]
    fn test_max_index_3() {
        assert_eq!(max_index_3(&[3.0, 7.0, 2.0]), 1);
        assert_eq!(max_index_3(&[7.0, 3.0, 2.0]), 0);
        assert_eq!(max_index_3(&[3.0, 2.0, 7.0]), 2);
    }

    #[test]
    fn test_max_index_9() {
        let arr = [1.0, 2.0, 3.0, 9.0, 5.0, 6.0, 7.0, 8.0, 4.0];
        assert_eq!(max_index_9(&arr), 3);
    }

    #[test]
    fn test_copy_vector_3() {
        let src = [1.0, 2.0, 3.0];
        let mut dst = [0.0, 0.0, 0.0];
        copy_vector_3(&mut dst, &src);
        assert_eq!(dst, src);
    }

    #[test]
    fn test_copy_vector_9() {
        let src = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let mut dst = [0.0; 9];
        copy_vector_9(&mut dst, &src);
        assert_eq!(dst, src);
    }

    #[test]
    fn test_swap_9() {
        let mut a = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let mut b = [10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0];
        let a_orig = a;
        let b_orig = b;
        swap_9(&mut a, &mut b);
        assert_eq!(a, b_orig);
        assert_eq!(b, a_orig);
    }

    #[test]
    fn test_bitstring_set_get() {
        let mut bits = [0u8; 2];
        bitstring_set_bit(&mut bits, 3, 1);
        assert_eq!(bitstring_get_bit(&bits, 3), 1);
        assert_eq!(bitstring_get_bit(&bits, 4), 0);

        bitstring_set_bit(&mut bits, 3, 0);
        assert_eq!(bitstring_get_bit(&bits, 3), 0);
    }

    #[test]
    fn test_sequential_vector_f32() {
        let mut v = [0.0; 5];
        sequential_vector_f32(&mut v, 2.0);
        assert_eq!(v, [2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_sequential_vector_i32() {
        let mut v = [0; 5];
        sequential_vector_i32(&mut v, 10);
        assert_eq!(v, [10, 11, 12, 13, 14]);
    }

    // ========================================================================
    // math_utils.h Tests
    // ========================================================================

    #[test]
    fn test_sqr() {
        assert_eq!(sqr(3.0_f32), 9.0);
        assert_eq!(sqr(-5.0_f32), 25.0);
        assert_eq!(sqr(0.0_f32), 0.0);
    }

    #[test]
    fn test_round_f32() {
        assert!((round_f32(2.3) - 2.0).abs() < EPSILON_F32);
        assert!((round_f32(2.6) - 3.0).abs() < EPSILON_F32);
        assert!((round_f32(2.5) - 3.0).abs() < EPSILON_F32);
    }

    #[test]
    fn test_log2_f32() {
        assert!((log2_f32(8.0) - 3.0).abs() < EPSILON_F32);
        assert!((log2_f32(1.0) - 0.0).abs() < EPSILON_F32);
        assert!((log2_f32(0.5) - (-1.0)).abs() < EPSILON_F32);
    }

    #[test]
    fn test_logb_f32() {
        assert!((logb_f32(100.0, 10.0) - 2.0).abs() < EPSILON_F32);
        assert!((logb_f32(8.0, 2.0) - 3.0).abs() < EPSILON_F32);
    }

    #[test]
    fn test_safe_reciprocal_f32() {
        assert!((safe_reciprocal_f32(2.0) - 0.5).abs() < EPSILON_F32);
        assert_eq!(safe_reciprocal_f32(0.0), 1.0);
    }

    #[test]
    fn test_safe_division_f32() {
        assert!((safe_division_f32(10.0, 2.0) - 5.0).abs() < EPSILON_F32);
        assert_eq!(safe_division_f32(5.0, 0.0), 5.0);
    }

    #[test]
    fn test_clip_scalar_f32() {
        assert_eq!(clip_scalar_f32(5.0, 1.0, 10.0), 5.0);
        assert_eq!(clip_scalar_f32(0.0, 1.0, 10.0), 1.0);
        assert_eq!(clip_scalar_f32(15.0, 1.0, 10.0), 10.0);
    }

    #[test]
    fn test_deg2rad_f32() {
        assert!((deg2rad_f32(0.0) - 0.0).abs() < EPSILON_F32);
        assert!((deg2rad_f32(180.0) - PI_F).abs() < EPSILON_F32);
        assert!((deg2rad_f32(90.0) - PI_F / 2.0).abs() < EPSILON_F32);
    }

    #[test]
    fn test_fast_atan2() {
        let angles_deg: [f32; 16] = [
            0.0, 22.5, 45.0, 67.5, 90.0, 112.5, 135.0, 157.5, 180.0, 202.5, 225.0, 247.5, 270.0,
            292.5, 315.0, 337.5,
        ];

        for &angle_deg in &angles_deg {
            let angle_rad = angle_deg.to_radians();
            let (sin_a, cos_a) = angle_rad.sin_cos();
            let approx = fast_atan2(sin_a, cos_a);
            let expected = angle_rad;

            // Normalize angles to [-π, π] for comparison
            let mut normalized_approx = approx;
            let mut normalized_expected = expected;
            while normalized_approx > PI_F {
                normalized_approx -= 2.0 * PI_F;
            }
            while normalized_approx < -PI_F {
                normalized_approx += 2.0 * PI_F;
            }
            while normalized_expected > PI_F {
                normalized_expected -= 2.0 * PI_F;
            }
            while normalized_expected < -PI_F {
                normalized_expected += 2.0 * PI_F;
            }

            let error = (normalized_approx - normalized_expected).abs();
            assert!(
                error < 0.015,
                "angle_deg={}, error={}, approx={}, expected={}",
                angle_deg,
                error,
                normalized_approx,
                normalized_expected
            );
        }
    }

    #[test]
    fn test_fast_atan2_360() {
        let result = fast_atan2_360(0.0, 1.0);
        assert!(result >= 0.0 && result <= 360.0);
    }

    #[test]
    fn test_fast_sqrt_inv() {
        let x = 4.0_f32;
        let approx = fast_sqrt_inv(x);
        let expected = x.sqrt();
        assert!(
            (approx - expected).abs() < 0.01,
            "approx={}, expected={}",
            approx,
            expected
        );

        let x = 9.0_f32;
        let approx = fast_sqrt_inv(x);
        let expected = x.sqrt();
        assert!(
            (approx - expected).abs() < 0.01,
            "approx={}, expected={}",
            approx,
            expected
        );
    }

    #[test]
    fn test_fast_exp6_f32() {
        let x = 0.0_f32;
        let result = fast_exp6_f32(x);
        assert!((result - 1.0).abs() < EPSILON_F32);

        let x = 1.0_f32;
        let result = fast_exp6_f32(x);
        let expected = std::f32::consts::E;
        assert!(
            (result - expected).abs() < 1e-3,
            "result={}, expected={}",
            result,
            expected
        );
    }

    // ========================================================================
    // linear_algebra.h Tests (M6-2)
    // ========================================================================

    #[test]
    fn test_dot_product_4() {
        let a = [1.0_f32, 2.0, 3.0, 4.0];
        let b = [5.0_f32, 6.0, 7.0, 8.0];
        // 1*5 + 2*6 + 3*7 + 4*8 = 5 + 12 + 21 + 32 = 70
        assert_eq!(dot_product_4(&a, &b), 70.0);
    }

    #[test]
    fn test_dot_product_9() {
        let a = [1.0_f32; 9];
        let b = [2.0_f32; 9];
        assert_eq!(dot_product_9(&a, &b), 18.0); // 1*2 * 9 = 18

        let a = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        // sum_{i=1..9} i*i = 285
        assert_eq!(dot_product_9(&a, &a), 285.0);
    }

    #[test]
    fn test_sum_squares_9() {
        let x = [3.0_f32, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert_eq!(sum_squares_9(&x), 25.0);
    }

    #[test]
    fn test_scale_vector_4() {
        let mut dst = [0.0_f32; 4];
        let src = [1.0_f32, 2.0, 3.0, 4.0];
        scale_vector_4(&mut dst, &src, 2.0);
        assert_eq!(dst, [2.0, 4.0, 6.0, 8.0]);
    }

    #[test]
    fn test_scale_vector_8() {
        let mut dst = [0.0_f32; 8];
        let src = [1.0_f32; 8];
        scale_vector_8(&mut dst, &src, 3.0);
        assert_eq!(dst, [3.0; 8]);
    }

    #[test]
    fn test_scale_vector_9() {
        let mut dst = [0.0_f32; 9];
        let src = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        scale_vector_9(&mut dst, &src, -1.0);
        assert_eq!(dst, [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0, -9.0]);
    }

    #[test]
    fn test_accumulate_scaled_vector_9() {
        let mut dst = [10.0_f32; 9];
        let src = [1.0_f32; 9];
        accumulate_scaled_vector_9(&mut dst, &src, 2.0);
        assert_eq!(dst, [12.0; 9]);
    }

    #[test]
    fn test_add_scaled_vectors_4() {
        let mut w = [0.0_f32; 4];
        let u = [1.0_f32, 2.0, 3.0, 4.0];
        let v = [10.0_f32, 20.0, 30.0, 40.0];
        // w = 2*u + 3*v
        add_scaled_vectors_4(&mut w, &u, &v, 2.0, 3.0);
        assert_eq!(w, [32.0, 64.0, 96.0, 128.0]);
    }

    #[test]
    fn test_update_outer_product_2x2() {
        let mut a = [0.0_f32, 0.0, 99.0, 0.0]; // a[2] is the mirror entry, untouched
        let x = [3.0_f32, 4.0];
        update_outer_product_2x2(&mut a, &x);
        assert_eq!(a[0], 9.0); // x0*x0
        assert_eq!(a[1], 12.0); // x0*x1
        assert_eq!(a[2], 99.0); // untouched
        assert_eq!(a[3], 16.0); // x1*x1
    }

    #[test]
    fn test_update_gauss_newton_operations_2x2() {
        let mut b = [10.0_f32, 20.0];
        let j = [2.0_f32, 3.0];
        // b -= J * residual; residual = 4
        update_gauss_newton_operations_2x2(&mut b, &j, 4.0);
        assert_eq!(b, [10.0 - 8.0, 20.0 - 12.0]);
    }

    #[test]
    fn test_cofactor_2x2_4arg() {
        // det([[1, 2], [3, 4]]) = 1*4 - 2*3 = -2
        assert_eq!(cofactor_2x2(1.0_f32, 2.0, 3.0, 4.0), -2.0);
    }

    #[test]
    fn test_cofactor_2x2_sym_3arg() {
        // det of symmetric [[2, 1], [1, 3]] = 2*3 - 1*1 = 5
        assert_eq!(cofactor_2x2_sym(2.0_f32, 1.0, 3.0), 5.0);
    }

    #[test]
    fn test_determinant_2x2() {
        assert_eq!(determinant_2x2(&[1.0_f32, 2.0, 3.0, 4.0]), -2.0);
        assert_eq!(determinant_2x2(&[2.0_f32, 0.0, 0.0, 5.0]), 10.0);
    }

    #[test]
    fn test_determinant_symmetric_3x3() {
        // Identity → det = 1
        let identity = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert!((determinant_symmetric_3x3(&identity) - 1.0).abs() < 1e-5);

        // 2*I → det = 8
        let two_identity = [2.0_f32, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0];
        assert!((determinant_symmetric_3x3(&two_identity) - 8.0).abs() < 1e-5);

        // [[2,1,1],[1,2,1],[1,1,2]] (symmetric) → det = 4
        let sym = [2.0_f32, 1.0, 1.0, 1.0, 2.0, 1.0, 1.0, 1.0, 2.0];
        assert!(
            (determinant_symmetric_3x3(&sym) - 4.0).abs() < 1e-5,
            "got {}",
            determinant_symmetric_3x3(&sym)
        );
    }

    #[test]
    fn test_multiply_2x2_2x1() {
        let mut y = [0.0_f32; 2];
        let a = [1.0_f32, 2.0, 3.0, 4.0]; // [[1,2],[3,4]]
        let x = [5.0_f32, 6.0];
        multiply_2x2_2x1(&mut y, &a, &x);
        assert_eq!(y, [1.0 * 5.0 + 2.0 * 6.0, 3.0 * 5.0 + 4.0 * 6.0]); // [17, 39]
    }

    #[test]
    fn test_multiply_3x3_3x1() {
        let mut y = [0.0_f32; 3];
        let a = [1.0_f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]; // identity
        let x = [7.0_f32, 8.0, 9.0];
        multiply_3x3_3x1(&mut y, &a, &x);
        assert_eq!(y, x);
    }

    #[test]
    fn test_matrix_inverse_2x2() {
        let mut b = [0.0_f32; 4];
        // A = [[1, 2], [3, 4]], det = -2; A^-1 = [[-2, 1], [1.5, -0.5]]
        let a = [1.0_f32, 2.0, 3.0, 4.0];
        assert!(matrix_inverse_2x2(&mut b, &a, f32::EPSILON));
        assert!((b[0] - (-2.0)).abs() < 1e-5);
        assert!((b[1] - 1.0).abs() < 1e-5);
        assert!((b[2] - 1.5).abs() < 1e-5);
        assert!((b[3] - (-0.5)).abs() < 1e-5);
    }

    #[test]
    fn test_matrix_inverse_2x2_singular() {
        let mut b = [0.0_f32; 4];
        // A = [[1, 2], [2, 4]], det = 0
        let a = [1.0_f32, 2.0, 2.0, 4.0];
        assert!(!matrix_inverse_2x2(&mut b, &a, f32::EPSILON));
    }

    #[test]
    fn test_matrix_inverse_symmetric_3x3() {
        let mut b = [0.0_f32; 9];
        // A = 2*I → A^-1 = 0.5*I
        let a = [2.0_f32, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0];
        assert!(matrix_inverse_symmetric_3x3(&mut b, &a, f32::EPSILON));
        let expected = [0.5_f32, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5];
        for i in 0..9 {
            assert!(
                (b[i] - expected[i]).abs() < 1e-5,
                "b[{}]={}, expected={}",
                i,
                b[i],
                expected[i]
            );
        }
    }

    #[test]
    fn test_matrix_inverse_symmetric_3x3_singular() {
        let mut b = [0.0_f32; 9];
        // Rank-2 symmetric matrix: outer product v*v^T with v=(1,2,3)
        // det = 0
        let a = [1.0_f32, 2.0, 3.0, 2.0, 4.0, 6.0, 3.0, 6.0, 9.0];
        assert!(!matrix_inverse_symmetric_3x3(&mut b, &a, f32::EPSILON));
    }

    #[test]
    fn test_solve_linear_system_2x2() {
        // A = [[2, 1], [1, 3]], x_known = [1, 2], b = A*x = [4, 7]
        let a = [2.0_f32, 1.0, 1.0, 3.0];
        let b = [4.0_f32, 7.0];
        let mut x = [0.0_f32; 2];
        assert!(solve_linear_system_2x2(&mut x, &a, &b));
        assert!((x[0] - 1.0).abs() < 1e-4, "x[0]={}", x[0]);
        assert!((x[1] - 2.0).abs() < 1e-4, "x[1]={}", x[1]);
    }

    #[test]
    fn test_solve_linear_system_2x2_singular() {
        let a = [1.0_f32, 2.0, 2.0, 4.0]; // det = 0
        let b = [1.0_f32, 2.0];
        let mut x = [0.0_f32; 2];
        assert!(!solve_linear_system_2x2(&mut x, &a, &b));
    }

    #[test]
    fn test_solve_symmetric_linear_system_3x3() {
        // A = symmetric [[2,1,1],[1,2,1],[1,1,2]], x_known = [1, 2, 3]
        // b = A * x_known = [2+2+3, 1+4+3, 1+2+6] = [7, 8, 9]
        let a = [2.0_f32, 1.0, 1.0, 1.0, 2.0, 1.0, 1.0, 1.0, 2.0];
        let b = [7.0_f32, 8.0, 9.0];
        let mut x = [0.0_f32; 3];
        assert!(solve_symmetric_linear_system_3x3(&mut x, &a, &b));
        assert!((x[0] - 1.0).abs() < 1e-4, "x[0]={}", x[0]);
        assert!((x[1] - 2.0).abs() < 1e-4, "x[1]={}", x[1]);
        assert!((x[2] - 3.0).abs() < 1e-4, "x[2]={}", x[2]);
    }

    #[test]
    fn test_solve_symmetric_linear_system_3x3_singular() {
        let a = [1.0_f32, 2.0, 3.0, 2.0, 4.0, 6.0, 3.0, 6.0, 9.0]; // rank-1, det=0
        let b = [1.0_f32, 1.0, 1.0];
        let mut x = [0.0_f32; 3];
        assert!(!solve_symmetric_linear_system_3x3(&mut x, &a, &b));
    }

    // ========================================================================
    // linear_solvers.h Tests (M6-2)
    // ========================================================================

    #[test]
    fn test_accumulate_projection_9() {
        // e is a unit vector along axis 0
        let e = [1.0_f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let a = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let mut x = a; // start equal to a
        accumulate_projection_9(&mut x, &e, &a);
        // dot(a, e) = a[0] = 1; x[0] should become 0; others unchanged
        assert!(x[0].abs() < 1e-6);
        for i in 1..9 {
            assert_eq!(x[i], a[i]);
        }
    }

    /// Build the 8x9 DLT design matrix from 4 point correspondences.
    /// Each correspondence (p, q) contributes 2 rows.
    fn dlt_8x9(p: &[(f32, f32); 4], q: &[(f32, f32); 4]) -> [f32; 72] {
        let mut a = [0.0_f32; 72];
        for k in 0..4 {
            let (u, v) = p[k];
            let (up, vp) = q[k];
            let row_u = [-u, -v, -1.0, 0.0, 0.0, 0.0, up * u, up * v, up];
            let row_v = [0.0, 0.0, 0.0, -u, -v, -1.0, vp * u, vp * v, vp];
            a[k * 18..k * 18 + 9].copy_from_slice(&row_u);
            a[k * 18 + 9..k * 18 + 18].copy_from_slice(&row_v);
        }
        a
    }

    #[test]
    fn test_solve_null_vector_8x9_destructive() {
        // Known homography H = [1.5, 0.1, 0.0,  0.2, 1.3, 0.0,  0.0, 0.0, 1.0]
        // Apply to 4 source points → 4 target points; build 8x9 design matrix;
        // solve_null_vector should recover ±H.
        let p = [(1.0_f32, 1.0), (2.0, 1.0), (1.0, 2.0), (3.0, 4.0)];
        let q = [(1.6_f32, 1.5), (3.1, 1.7), (1.7, 2.8), (4.9, 5.8)];

        let mut a = dlt_8x9(&p, &q);
        let a_check = a; // keep a copy because solve destroys A

        let mut x = [0.0_f32; 9];
        assert!(solve_null_vector_8x9_destructive(&mut x, &mut a));

        // |A * x| should be approximately zero, per row
        for row in 0..8 {
            let dot: f32 = (0..9).map(|j| a_check[row * 9 + j] * x[j]).sum();
            assert!(
                dot.abs() < 1e-4,
                "row {} residual = {} exceeds tolerance",
                row,
                dot
            );
        }

        // x should be unit length (normalized null vector)
        let norm_sq: f32 = (0..9).map(|j| x[j] * x[j]).sum();
        assert!(
            (norm_sq - 1.0).abs() < 1e-4,
            "null vector not unit length: |x|^2 = {}",
            norm_sq
        );
    }

    #[test]
    fn test_solve_tridiagonal_destructive() {
        // 5x5 tridiagonal: diagonal = 2, off-diagonal = 1
        // x_known = [1, 2, 3, 4, 5]
        // v = A * x_known computed by hand:
        //   row 0: 2*1 + 1*2           = 4
        //   row 1: 1*1 + 2*2 + 1*3     = 8
        //   row 2: 1*2 + 2*3 + 1*4     = 12
        //   row 3: 1*3 + 2*4 + 1*5     = 16
        //   row 4: 1*4 + 2*5           = 14
        let a = [1.0_f32, 1.0, 1.0, 1.0]; // below-diagonal (n-1 entries)
        let b = [2.0_f32, 2.0, 2.0, 2.0, 2.0]; // diagonal
        let mut c = [1.0_f32, 1.0, 1.0, 1.0, 0.0]; // above-diagonal (over-allocated)
        let mut x = [4.0_f32, 8.0, 12.0, 16.0, 14.0];

        assert!(solve_tridiagonal_destructive(&mut x, &a, &b, &mut c));
        let expected = [1.0_f32, 2.0, 3.0, 4.0, 5.0];
        for i in 0..5 {
            assert!(
                (x[i] - expected[i]).abs() < 1e-5,
                "x[{}] = {}, expected {}",
                i,
                x[i],
                expected[i]
            );
        }
    }

    #[test]
    fn test_solve_tridiagonal_destructive_singular() {
        // b[0] == 0 → no leading pivot
        let a = [1.0_f32];
        let b = [0.0_f32, 1.0];
        let mut c = [1.0_f32, 0.0];
        let mut x = [1.0_f32, 1.0];
        assert!(!solve_tridiagonal_destructive(&mut x, &a, &b, &mut c));
    }
}

// ============================================================================
// Dual-mode validation against the C++ baseline (Milestone 6, #63)
// ============================================================================
//
// When the `dual-mode` feature is enabled (which transitively enables
// `ffi-backend`), the C++ math functions in WebARKitLib are linked in via the
// `webarkit_cpp_*` wrappers in `kpm_c_api.cpp`. The tests below sweep across
// the input domain and assert that the pure-Rust ports produce results within
// a small tolerance of the C++ baseline.

#[cfg(feature = "dual-mode")]
extern "C" {
    // M6-1 (#63) — math_utils.h
    fn webarkit_cpp_fast_atan2(y: f32, x: f32) -> f32;
    fn webarkit_cpp_fast_sqrt1(x: f32) -> f32;
    fn webarkit_cpp_fast_exp6_f32(x: f32) -> f32;

    // M6-2 (#64) — linear_algebra.h / linear_solvers.h. Return 1 on success,
    // 0 on failure (C ABI int instead of bool for portability).
    fn webarkit_cpp_solve_linear_system_2x2(x: *mut f32, a: *const f32, b: *const f32) -> i32;
    fn webarkit_cpp_solve_symmetric_linear_system_3x3(
        x: *mut f32,
        a: *const f32,
        b: *const f32,
    ) -> i32;
    fn webarkit_cpp_solve_null_vector_8x9_destructive(x: *mut f32, a: *mut f32) -> i32;
}

#[cfg(all(test, feature = "dual-mode"))]
mod dual_mode_tests {
    use super::*;
    use crate::arlog_e;

    /// Sweep `fast_atan2` over a 201×201 grid covering all four quadrants
    /// and the axes; assert Rust and C++ outputs agree to 1e-6.
    #[test]
    fn fast_atan2_matches_cpp_across_quadrants() {
        let mut max_diff = 0.0_f32;
        let mut worst_inputs = (0.0_f32, 0.0_f32);
        for y_q in -100..=100 {
            for x_q in -100..=100 {
                let y = y_q as f32 / 10.0;
                let x = x_q as f32 / 10.0;
                let rust = fast_atan2(y, x);
                let cpp = unsafe { webarkit_cpp_fast_atan2(y, x) };
                let diff = (rust - cpp).abs();
                if diff > max_diff {
                    max_diff = diff;
                    worst_inputs = (y, x);
                }
                assert!(
                    diff < 1e-6,
                    "fast_atan2 diverged at y={}, x={}: rust={}, cpp={}, diff={}",
                    y,
                    x,
                    rust,
                    cpp,
                    diff
                );
            }
        }
        arlog_e!(
            "fast_atan2: max diff = {} over 40,401 inputs (worst at y={}, x={})",
            max_diff,
            worst_inputs.0,
            worst_inputs.1
        );
    }

    /// Sweep `fast_sqrt_inv` (which actually returns √x — see C++
    /// `fastsqrt1`) over positive reals. The loose tolerance accommodates
    /// the upstream `(int)x` truncation quirk documented on
    /// [`fast_sqrt_inv`]. Function is unused in the upstream algorithm.
    #[test]
    fn fast_sqrt1_matches_cpp() {
        let mut max_rel_err = 0.0_f32;
        let mut worst_x = 0.0_f32;
        for x_q in 1..=10000 {
            let x = x_q as f32 / 100.0;
            let rust = fast_sqrt_inv(x);
            let cpp = unsafe { webarkit_cpp_fast_sqrt1(x) };
            let denom = cpp.abs().max(1e-30);
            let rel_err = ((rust - cpp).abs()) / denom;
            if rel_err > max_rel_err {
                max_rel_err = rel_err;
                worst_x = x;
            }
        }
        arlog_e!(
            "fast_sqrt_inv: max relative err = {} over 10,000 inputs (worst at x={})",
            max_rel_err,
            worst_x
        );
        // Tolerance accommodates the upstream (int) cast in fastsqrt1.
        assert!(
            max_rel_err < 0.5,
            "fast_sqrt_inv diverged from C++ baseline: max_rel_err={} at x={}",
            max_rel_err,
            worst_x
        );
    }

    /// Sweep `fast_exp6_f32` over [-2.0, 2.0] in steps of 0.01 and assert
    /// bit-near agreement with the C++ baseline.
    #[test]
    fn fast_exp6_matches_cpp() {
        let mut max_rel_err = 0.0_f32;
        let mut worst_x = 0.0_f32;
        for x_q in -200..=200 {
            let x = x_q as f32 / 100.0;
            let rust = fast_exp6_f32(x);
            let cpp = unsafe { webarkit_cpp_fast_exp6_f32(x) };
            let denom = cpp.abs().max(1e-30);
            let rel_err = ((rust - cpp).abs()) / denom;
            if rel_err > max_rel_err {
                max_rel_err = rel_err;
                worst_x = x;
            }
            assert!(
                rel_err < 1e-5,
                "fast_exp6 diverged at x={}: rust={}, cpp={}, rel_err={}",
                x,
                rust,
                cpp,
                rel_err
            );
        }
        arlog_e!(
            "fast_exp6: max relative err = {} over 401 inputs (worst at x={})",
            max_rel_err,
            worst_x
        );
    }

    // ========================================================================
    // M6-2 (#64) — linear_algebra.h / linear_solvers.h dual-mode tests
    // ========================================================================

    /// Sweep `solve_linear_system_2x2` over 1000 random non-degenerate 2×2
    /// systems and assert Rust and C++ agree element-wise to 1e-5.
    #[test]
    fn solve_linear_system_2x2_matches_cpp() {
        use rand::rngs::StdRng;
        use rand::{Rng, SeedableRng};

        let mut rng = StdRng::seed_from_u64(0xDEADBEEF);
        let mut max_diff = 0.0_f32;
        let mut compared = 0u32;

        for _ in 0..1000 {
            let a: [f32; 4] = std::array::from_fn(|_| rng.random_range(-1.0_f32..1.0));
            let b: [f32; 2] = std::array::from_fn(|_| rng.random_range(-1.0_f32..1.0));

            let mut x_rust = [0.0_f32; 2];
            let mut x_cpp = [0.0_f32; 2];

            let r = solve_linear_system_2x2(&mut x_rust, &a, &b);
            let c = unsafe {
                webarkit_cpp_solve_linear_system_2x2(x_cpp.as_mut_ptr(), a.as_ptr(), b.as_ptr())
            } != 0;

            assert_eq!(r, c, "Rust and C++ disagreed on success/failure");

            if r {
                for i in 0..2 {
                    let diff = (x_rust[i] - x_cpp[i]).abs();
                    if diff > max_diff {
                        max_diff = diff;
                    }
                    // Combined absolute + relative tolerance: f32 has ~7
                    // decimal digits, so values around magnitude M need
                    // tolerance ~ M * 1e-6. The 1e-5 floor handles values
                    // near zero. This accommodates platform-specific FMA
                    // rounding differences (Apple Silicon vs x86_64).
                    let tol = 1e-5_f32.max(x_rust[i].abs() * 1e-6);
                    assert!(
                        diff < tol,
                        "solve_linear_system_2x2 diverged at x[{}]: rust={}, cpp={}, diff={}, tol={}",
                        i,
                        x_rust[i],
                        x_cpp[i],
                        diff,
                        tol
                    );
                }
                compared += 1;
            }
        }

        arlog_e!(
            "solve_linear_system_2x2: max diff = {} over {} comparisons",
            max_diff,
            compared
        );
    }

    /// Sweep `solve_symmetric_linear_system_3x3` over 500 random SPD 3×3
    /// systems (built as `M*M^T + 0.1*I` to guarantee positive-definiteness)
    /// and assert Rust and C++ agree element-wise to 1e-5.
    #[test]
    fn solve_symmetric_linear_system_3x3_matches_cpp() {
        use rand::rngs::StdRng;
        use rand::{Rng, SeedableRng};

        let mut rng = StdRng::seed_from_u64(0xDEADBEEF);
        let mut max_diff = 0.0_f32;
        let mut compared = 0u32;

        for _ in 0..500 {
            // Build SPD A = M*M^T + 0.1*I
            let m: [f32; 9] = std::array::from_fn(|_| rng.random_range(-1.0_f32..1.0));
            let mut a = [0.0_f32; 9];
            for i in 0..3 {
                for j in 0..3 {
                    let mut s = 0.0_f32;
                    for k in 0..3 {
                        s += m[i * 3 + k] * m[j * 3 + k];
                    }
                    a[i * 3 + j] = s;
                }
            }
            a[0] += 0.1;
            a[4] += 0.1;
            a[8] += 0.1;

            let b: [f32; 3] = std::array::from_fn(|_| rng.random_range(-1.0_f32..1.0));

            let mut x_rust = [0.0_f32; 3];
            let mut x_cpp = [0.0_f32; 3];

            let r = solve_symmetric_linear_system_3x3(&mut x_rust, &a, &b);
            let c = unsafe {
                webarkit_cpp_solve_symmetric_linear_system_3x3(
                    x_cpp.as_mut_ptr(),
                    a.as_ptr(),
                    b.as_ptr(),
                )
            } != 0;

            assert_eq!(r, c, "Rust and C++ disagreed on success/failure");

            if r {
                for i in 0..3 {
                    let diff = (x_rust[i] - x_cpp[i]).abs();
                    if diff > max_diff {
                        max_diff = diff;
                    }
                    // See solve_linear_system_2x2_matches_cpp for tolerance rationale.
                    let tol = 1e-5_f32.max(x_rust[i].abs() * 1e-6);
                    assert!(
                        diff < tol,
                        "solve_symmetric_linear_system_3x3 diverged at x[{}]: rust={}, cpp={}, diff={}, tol={}",
                        i,
                        x_rust[i],
                        x_cpp[i],
                        diff,
                        tol
                    );
                }
                compared += 1;
            }
        }

        arlog_e!(
            "solve_symmetric_linear_system_3x3: max diff = {} over {} comparisons",
            max_diff,
            compared
        );
    }

    /// Sweep `solve_null_vector_8x9_destructive` over 100 random 8×9 matrices
    /// and assert Rust and C++ produce the same null direction.
    ///
    /// A null vector is defined only up to sign; comparison is `|dot(rust, cpp)|`
    /// against `1.0` (not element-wise). Tolerance: 1e-4 per spec.
    #[test]
    fn solve_null_vector_8x9_destructive_matches_cpp() {
        use rand::rngs::StdRng;
        use rand::{Rng, SeedableRng};

        let mut rng = StdRng::seed_from_u64(0xDEADBEEF);
        let mut min_dot = 1.0_f32;
        let mut compared = 0u32;

        for _ in 0..100 {
            let a_orig: [f32; 72] = std::array::from_fn(|_| rng.random_range(-1.0_f32..1.0));

            let mut a_rust = a_orig;
            let mut a_cpp = a_orig;
            let mut x_rust = [0.0_f32; 9];
            let mut x_cpp = [0.0_f32; 9];

            let r = solve_null_vector_8x9_destructive(&mut x_rust, &mut a_rust);
            let c = unsafe {
                webarkit_cpp_solve_null_vector_8x9_destructive(
                    x_cpp.as_mut_ptr(),
                    a_cpp.as_mut_ptr(),
                )
            } != 0;

            assert_eq!(r, c, "Rust and C++ disagreed on success/failure");

            if r {
                // Sign-ambiguous: |dot| should be ~1.0 (both vectors unit length,
                // possibly differing by a global sign flip)
                let dot: f32 = (0..9).map(|i| x_rust[i] * x_cpp[i]).sum();
                let abs_dot = dot.abs();
                if abs_dot < min_dot {
                    min_dot = abs_dot;
                }
                assert!(
                    (1.0 - abs_dot) < 1e-4,
                    "solve_null_vector_8x9 diverged: |dot(rust, cpp)| = {} (expected ~1.0)",
                    abs_dot
                );
                compared += 1;
            }
        }

        arlog_e!(
            "solve_null_vector_8x9_destructive: min |dot(rust, cpp)| = {} over {} comparisons",
            min_dot,
            compared
        );
    }
}
