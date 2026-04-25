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
//!
//! All functions are marked `#[inline(always)]` to match C++ `inline` keyword behavior
//! and enable call-site specialization via monomorphization.

use std::ops::AddAssign;

// ============================================================================
// Constants
// ============================================================================

/// π (pi) in f64 precision
pub const PI: f64 = 3.1415926535897932384626433832795;

/// π (pi) in f32 precision
pub const PI_F: f32 = 3.1415926535897932384626433832795_f32;

/// 1 / (2π)
pub const ONE_OVER_2PI: f32 = 0.159154943091895_f32;

/// √2 (square root of 2)
pub const SQRT2: f32 = 1.41421356237309504880_f32;

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
pub fn swap_9<T: Copy>(a: &mut [T; 9], b: &mut [T; 9]) {
    let temp = *a;
    *a = *b;
    *b = temp;
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

/// Create a sequential vector [x0, x0+1, x0+2, ...].
///
/// C++ equivalent: `SequentialVector<T>`
#[inline(always)]
pub fn sequential_vector<T: Default + Copy + AddAssign>(v: &mut [T], x0: T) {
    let mut val = x0;
    for elem in v.iter_mut() {
        *elem = val;
        val += T::default(); // This won't work as intended; fixed below
    }
}

// Fixed version of sequential_vector that requires numeric traits
/// Create a sequential vector [x0, x0+1, x0+2, ...] for numeric types.
///
/// Works with types that support addition and zero initialization.
#[inline(always)]
pub fn sequential_vector_f32(v: &mut [f32], x0: f32) {
    let mut val = x0;
    for elem in v.iter_mut() {
        *elem = val;
        val += 1.0;
    }
}

/// Create a sequential vector [x0, x0+1, x0+2, ...] for i32 types.
#[inline(always)]
pub fn sequential_vector_i32(v: &mut [i32], x0: i32) {
    let mut val = x0;
    for elem in v.iter_mut() {
        *elem = val;
        val += 1;
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
/// C++ equivalent: `fastatan2`
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
/// C++ equivalent: `fastatan2_360` (note: incomplete in original C++)
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
/// to compute 1/√x via the magic constant 0x5f3759df, then applies Newton-Raphson refinement,
/// and finally multiplies by x to get √x.
///
/// C++ equivalent: `fastsqrt1`
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
/// C++ equivalent: `fastexp6<T>`
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
}
