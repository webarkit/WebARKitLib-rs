/*
 *  pyramid.rs
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

//! Image pyramid built by repeated box-filter downsampling.
//!
//! Ported from `WebARKitLib/lib/SRC/KPM/FreakMatcher/detectors/pyramid.{h,cpp}`
//! (`BoxFilterPyramid8u` / `BoxFilterDecimate`).
//!
//! The pyramid produces successively half-sized levels via a 2x2 box-filter
//! average with rounding: `(p0 + p1 + p2 + p3 + 2) >> 2`. Output is
//! byte-identical to the C++ baseline.
//!
//! C equivalent: `vision::BoxFilterPyramid8u`

use crate::{arlog_e, arlog_w};
use purecv::core::Matrix;

/// Errors that can occur while building a [`Pyramid`].
#[derive(Debug, Clone, PartialEq)]
pub enum PyramidError {
    /// Input image had zero rows or zero columns.
    EmptyImage,
    /// `scale_factor` is not supported. Currently only `2.0` is accepted
    /// (matches the C++ `BoxFilterPyramid8u` behavior).
    InvalidScaleFactor(f32),
    /// `num_levels` was 0 — a pyramid must have at least one level.
    ZeroLevels,
    /// Downsampling at this level would produce a 0-sized image.
    /// Returned when the input image is too small for the requested
    /// number of levels.
    LevelTooSmall { level: usize },
}

impl std::fmt::Display for PyramidError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyImage => write!(f, "input image is empty (0 rows or 0 cols)"),
            Self::InvalidScaleFactor(s) => {
                write!(
                    f,
                    "scale_factor {s} is not supported (only 2.0 is accepted)"
                )
            }
            Self::ZeroLevels => write!(f, "num_levels must be >= 1"),
            Self::LevelTooSmall { level } => {
                write!(f, "downsampled image at level {level} would be 0-sized")
            }
        }
    }
}

impl std::error::Error for PyramidError {}

/// Image pyramid built from a single grayscale input via repeated 2x2
/// box-filter downsampling.
///
/// Level 0 is a copy of the input. Each subsequent level has dimensions
/// `ceil((prev.rows - 1) / 2) x ceil((prev.cols - 1) / 2)` to match the
/// C++ `BoxFilterPyramid8u::init()` formula.
#[derive(Debug, Clone)]
pub struct Pyramid {
    /// Levels from finest (index 0 = input copy) to coarsest.
    pub levels: Vec<Matrix<u8>>,
    /// Downsample factor between successive levels. Currently must be `2.0`.
    pub scale_factor: f32,
    /// Number of levels requested at construction.
    num_levels: usize,
}

impl Pyramid {
    /// Create an empty pyramid configured for `num_levels` levels, each
    /// `scale_factor` smaller than the previous. Levels are populated by
    /// [`Pyramid::build`].
    ///
    /// Only `scale_factor == 2.0` is currently supported; any other value
    /// causes [`Pyramid::build`] to return
    /// [`PyramidError::InvalidScaleFactor`].
    #[must_use]
    pub fn new(num_levels: usize, scale_factor: f32) -> Self {
        Self {
            levels: Vec::with_capacity(num_levels),
            scale_factor,
            num_levels,
        }
    }

    /// Number of populated levels (i.e. levels successfully built so far).
    #[must_use]
    pub fn num_levels(&self) -> usize {
        self.levels.len()
    }

    /// Borrow level `i`.
    ///
    /// # Panics
    ///
    /// Panics if `i >= self.num_levels()`.
    #[must_use]
    pub fn level(&self, i: usize) -> &Matrix<u8> {
        &self.levels[i]
    }

    /// Populate `self.levels` from `image`.
    ///
    /// - Level 0 is a deep copy of the input.
    /// - Level k > 0 is `downsample(level k-1)`.
    ///
    /// Calling `build` again on the same pyramid clears prior state first
    /// (idempotent).
    pub fn build(&mut self, image: &Matrix<u8>) -> Result<(), PyramidError> {
        if self.num_levels == 0 {
            arlog_e!("Pyramid::build: num_levels is 0");
            return Err(PyramidError::ZeroLevels);
        }
        if (self.scale_factor - 2.0).abs() > f32::EPSILON {
            arlog_e!(
                "Pyramid::build: scale_factor {} not supported (only 2.0)",
                self.scale_factor
            );
            return Err(PyramidError::InvalidScaleFactor(self.scale_factor));
        }
        if image.rows == 0 || image.cols == 0 {
            arlog_e!("Pyramid::build: input image is empty");
            return Err(PyramidError::EmptyImage);
        }
        if image.channels != 1 {
            arlog_w!(
                "Pyramid::build: input has {} channels; only the first channel is read",
                image.channels
            );
        }

        // Idempotent: clear any previous build state.
        self.levels.clear();

        // Level 0 = deep copy of the input.
        self.levels.push(image.clone());

        // Levels 1..num_levels.
        for k in 1..self.num_levels {
            let prev = &self.levels[k - 1];
            let new_h = (prev.rows.saturating_sub(1) as f32 / 2.0).ceil() as usize;
            let new_w = (prev.cols.saturating_sub(1) as f32 / 2.0).ceil() as usize;
            if new_h == 0 || new_w == 0 {
                arlog_e!("Pyramid::build: level {k} would be 0-sized");
                return Err(PyramidError::LevelTooSmall { level: k });
            }
            self.levels.push(downsample(prev));
        }

        Ok(())
    }
}

/// 2x2 box-filter downsample, byte-identical to C++ `BoxFilterDecimate`.
///
/// Dispatches to the fastest available implementation at runtime
/// (AVX2 → SSE4.1 on x86_64, simd128 on wasm32), falling back to
/// [`downsample_scalar`]. Every path produces bit-for-bit identical
/// output to the scalar baseline — see the property tests below.
///
/// Output dimensions: `new_h = ceil((src.rows - 1) / 2)`,
/// `new_w = ceil((src.cols - 1) / 2)`.
///
/// # Note on visibility
///
/// `downsample` / `downsample_scalar` (and the SIMD variants) are `pub`
/// so the criterion benchmark (a separate crate) can measure them
/// directly. They are not part of a stability guarantee — prefer
/// [`Pyramid::build`].
#[must_use]
pub fn downsample(src: &Matrix<u8>) -> Matrix<u8> {
    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-avx2"))]
    {
        if is_x86_feature_detected!("avx2") {
            // SAFETY: the `avx2` target feature is confirmed present at
            // runtime, satisfying the precondition of `downsample_avx2`.
            return unsafe { downsample_avx2(src) };
        }
    }
    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-sse41"))]
    {
        if is_x86_feature_detected!("sse4.1") {
            // SAFETY: the `sse4.1` target feature is confirmed present at
            // runtime, satisfying the precondition of `downsample_sse41`.
            return unsafe { downsample_sse41(src) };
        }
    }
    #[cfg(all(
        target_arch = "wasm32",
        feature = "simd-wasm32",
        target_feature = "simd128"
    ))]
    {
        // SAFETY: `simd128` is guaranteed by the `target_feature` cfg gate,
        // so the intrinsics in `downsample_wasm` are always valid here.
        return unsafe { downsample_wasm(src) };
    }

    #[allow(unreachable_code)]
    downsample_scalar(src)
}

/// Output dimensions of a single downsample step: `(new_h, new_w)`.
#[inline]
fn downsample_dims(src: &Matrix<u8>) -> (usize, usize) {
    let new_h = (src.rows.saturating_sub(1) as f32 / 2.0).ceil() as usize;
    let new_w = (src.cols.saturating_sub(1) as f32 / 2.0).ceil() as usize;
    (new_h, new_w)
}

/// Scalar fill of one output row for output columns `start..dst_row.len()`.
///
/// Shared by the scalar implementation and the SIMD remainder tails so the
/// rounding (`(a + b + c + d + 2) >> 2`) is defined in exactly one place.
#[inline]
fn downsample_row_tail_scalar(row0: &[u8], row1: &[u8], dst_row: &mut [u8], start: usize) {
    for (out_col, dst) in dst_row.iter_mut().enumerate().skip(start) {
        let c = out_col * 2;
        // u16 promotion prevents u8 wrap; max sum = 4*255 + 2 = 1022.
        let sum: u16 =
            row0[c] as u16 + row0[c + 1] as u16 + row1[c] as u16 + row1[c + 1] as u16 + 2;
        *dst = (sum >> 2) as u8;
    }
}

/// Scalar 2x2 box-filter downsample, byte-identical to C++
/// `BoxFilterDecimate`.
///
/// Output dimensions: `new_h = ceil((src.rows - 1) / 2)`,
/// `new_w = ceil((src.cols - 1) / 2)`.
///
/// Per output pixel, sum the four corresponding source pixels (as `u16`
/// to avoid overflow), add 2 for rounding, then shift right by 2. This
/// is the combined effect of `HorizontalBoxFilterDecimate` plus
/// `VerticalBoxFilter` from `pyramid-inline.h`.
#[must_use]
pub fn downsample_scalar(src: &Matrix<u8>) -> Matrix<u8> {
    let (new_h, new_w) = downsample_dims(src);

    let src_data = src.as_slice();
    let src_cols = src.cols;

    let mut dst_data = vec![0u8; new_h * new_w];

    for out_row in 0..new_h {
        let r0 = (out_row * 2) * src_cols;
        let r1 = r0 + src_cols;
        let row0 = &src_data[r0..r0 + src_cols];
        let row1 = &src_data[r1..r1 + src_cols];
        let dst_row = &mut dst_data[out_row * new_w..(out_row + 1) * new_w];
        downsample_row_tail_scalar(row0, row1, dst_row, 0);
    }

    Matrix::<u8>::from_vec(new_h, new_w, 1, dst_data)
}

/// AVX2 2x2 box-filter downsample. Processes 32 output columns per
/// iteration (64 source columns) using 256-bit lanes; the remaining
/// columns of each row fall back to [`downsample_row_tail_scalar`].
///
/// Output is byte-identical to [`downsample_scalar`].
///
/// # Safety
///
/// The caller must ensure the `avx2` target feature is available at
/// runtime (the [`downsample`] dispatcher guarantees this via
/// `is_x86_feature_detected!`).
#[cfg(all(target_arch = "x86_64", feature = "simd-x86-avx2"))]
#[target_feature(enable = "avx2")]
#[must_use]
pub unsafe fn downsample_avx2(src: &Matrix<u8>) -> Matrix<u8> {
    use std::arch::x86_64::*;

    let (new_h, new_w) = downsample_dims(src);
    let src_data = src.as_slice();
    let src_cols = src.cols;
    let mut dst_data = vec![0u8; new_h * new_w];

    // Number of leading output columns we can serve with full 32-wide
    // blocks. A block at output col `o` reads source cols `[2o, 2o + 64)`
    // and writes `[o, o + 32)`; both stay in bounds while
    // `2 * (o + 32) <= src_cols` (which also implies `o + 32 <= new_w`).
    let simd_cols = if src_cols >= 64 {
        ((src_cols - 64) / 2 / 32) * 32 + 32
    } else {
        0
    };

    let ones = _mm256_set1_epi8(1);
    let two = _mm256_set1_epi16(2);

    for out_row in 0..new_h {
        let r0 = (out_row * 2) * src_cols;
        let r1 = r0 + src_cols;
        let row0 = &src_data[r0..r0 + src_cols];
        let row1 = &src_data[r1..r1 + src_cols];
        let dst_row = &mut dst_data[out_row * new_w..(out_row + 1) * new_w];

        let mut o = 0;
        while o < simd_cols {
            let c = o * 2;
            // SAFETY: `c + 64 <= src_cols` by construction of `simd_cols`,
            // and `o + 32 <= new_w`, so all loads/stores are in bounds.
            let a0 = _mm256_loadu_si256(row0.as_ptr().add(c) as *const __m256i);
            let a1 = _mm256_loadu_si256(row0.as_ptr().add(c + 32) as *const __m256i);
            let b0 = _mm256_loadu_si256(row1.as_ptr().add(c) as *const __m256i);
            let b1 = _mm256_loadu_si256(row1.as_ptr().add(c + 32) as *const __m256i);

            // Horizontal pairwise sums of adjacent bytes -> u16 lanes.
            let h0lo = _mm256_maddubs_epi16(a0, ones);
            let h0hi = _mm256_maddubs_epi16(a1, ones);
            let h1lo = _mm256_maddubs_epi16(b0, ones);
            let h1hi = _mm256_maddubs_epi16(b1, ones);

            // Vertical: (h0 + h1 + 2) >> 2.
            let slo = _mm256_srli_epi16(_mm256_add_epi16(_mm256_add_epi16(h0lo, h1lo), two), 2);
            let shi = _mm256_srli_epi16(_mm256_add_epi16(_mm256_add_epi16(h0hi, h1hi), two), 2);

            // packus works per 128-bit lane, interleaving the halves;
            // permute the 64-bit chunks back into sequential order.
            let packed = _mm256_packus_epi16(slo, shi);
            let out = _mm256_permute4x64_epi64(packed, 0xD8);
            _mm256_storeu_si256(dst_row.as_mut_ptr().add(o) as *mut __m256i, out);

            o += 32;
        }

        downsample_row_tail_scalar(row0, row1, dst_row, simd_cols);
    }

    Matrix::<u8>::from_vec(new_h, new_w, 1, dst_data)
}

/// SSE4.1 2x2 box-filter downsample. Processes 16 output columns per
/// iteration (32 source columns) using 128-bit lanes; the remaining
/// columns of each row fall back to [`downsample_row_tail_scalar`].
///
/// Output is byte-identical to [`downsample_scalar`].
///
/// # Safety
///
/// The caller must ensure the `sse4.1` target feature is available at
/// runtime (the [`downsample`] dispatcher guarantees this via
/// `is_x86_feature_detected!`).
#[cfg(all(target_arch = "x86_64", feature = "simd-x86-sse41"))]
#[target_feature(enable = "sse4.1")]
#[must_use]
pub unsafe fn downsample_sse41(src: &Matrix<u8>) -> Matrix<u8> {
    use std::arch::x86_64::*;

    let (new_h, new_w) = downsample_dims(src);
    let src_data = src.as_slice();
    let src_cols = src.cols;
    let mut dst_data = vec![0u8; new_h * new_w];

    // A block at output col `o` reads source cols `[2o, 2o + 32)` and
    // writes `[o, o + 16)`; safe while `2 * (o + 16) <= src_cols`.
    let simd_cols = if src_cols >= 32 {
        ((src_cols - 32) / 2 / 16) * 16 + 16
    } else {
        0
    };

    let ones = _mm_set1_epi8(1);
    let two = _mm_set1_epi16(2);

    for out_row in 0..new_h {
        let r0 = (out_row * 2) * src_cols;
        let r1 = r0 + src_cols;
        let row0 = &src_data[r0..r0 + src_cols];
        let row1 = &src_data[r1..r1 + src_cols];
        let dst_row = &mut dst_data[out_row * new_w..(out_row + 1) * new_w];

        let mut o = 0;
        while o < simd_cols {
            let c = o * 2;
            // SAFETY: `c + 32 <= src_cols` by construction of `simd_cols`,
            // and `o + 16 <= new_w`, so all loads/stores are in bounds.
            let a0 = _mm_loadu_si128(row0.as_ptr().add(c) as *const __m128i);
            let a1 = _mm_loadu_si128(row0.as_ptr().add(c + 16) as *const __m128i);
            let b0 = _mm_loadu_si128(row1.as_ptr().add(c) as *const __m128i);
            let b1 = _mm_loadu_si128(row1.as_ptr().add(c + 16) as *const __m128i);

            let h0lo = _mm_maddubs_epi16(a0, ones);
            let h0hi = _mm_maddubs_epi16(a1, ones);
            let h1lo = _mm_maddubs_epi16(b0, ones);
            let h1hi = _mm_maddubs_epi16(b1, ones);

            let slo = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(h0lo, h1lo), two), 2);
            let shi = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(h0hi, h1hi), two), 2);

            let out = _mm_packus_epi16(slo, shi);
            _mm_storeu_si128(dst_row.as_mut_ptr().add(o) as *mut __m128i, out);

            o += 16;
        }

        downsample_row_tail_scalar(row0, row1, dst_row, simd_cols);
    }

    Matrix::<u8>::from_vec(new_h, new_w, 1, dst_data)
}

/// wasm32 SIMD (`simd128`) 2x2 box-filter downsample. Processes 16 output
/// columns per iteration (32 source columns); the remaining columns of
/// each row fall back to [`downsample_row_tail_scalar`].
///
/// Output is byte-identical to [`downsample_scalar`].
///
/// # Safety
///
/// Requires the `simd128` target feature, which is guaranteed by the
/// `#[cfg(target_feature = "simd128")]` gate on the [`downsample`]
/// dispatcher call site.
#[cfg(all(
    target_arch = "wasm32",
    feature = "simd-wasm32",
    target_feature = "simd128"
))]
#[target_feature(enable = "simd128")]
#[must_use]
pub unsafe fn downsample_wasm(src: &Matrix<u8>) -> Matrix<u8> {
    use std::arch::wasm32::*;

    let (new_h, new_w) = downsample_dims(src);
    let src_data = src.as_slice();
    let src_cols = src.cols;
    let mut dst_data = vec![0u8; new_h * new_w];

    // A block at output col `o` reads source cols `[2o, 2o + 32)` and
    // writes `[o, o + 16)`; safe while `2 * (o + 16) <= src_cols`.
    let simd_cols = if src_cols >= 32 {
        ((src_cols - 32) / 2 / 16) * 16 + 16
    } else {
        0
    };

    let two = i16x8_splat(2);

    for out_row in 0..new_h {
        let r0 = (out_row * 2) * src_cols;
        let r1 = r0 + src_cols;
        let row0 = &src_data[r0..r0 + src_cols];
        let row1 = &src_data[r1..r1 + src_cols];
        let dst_row = &mut dst_data[out_row * new_w..(out_row + 1) * new_w];

        let mut o = 0;
        while o < simd_cols {
            let c = o * 2;
            // SAFETY: `c + 32 <= src_cols` by construction of `simd_cols`,
            // and `o + 16 <= new_w`, so all loads/stores are in bounds.
            let a0 = v128_load(row0.as_ptr().add(c) as *const v128);
            let a1 = v128_load(row0.as_ptr().add(c + 16) as *const v128);
            let b0 = v128_load(row1.as_ptr().add(c) as *const v128);
            let b1 = v128_load(row1.as_ptr().add(c + 16) as *const v128);

            // Horizontal pairwise sums of adjacent u8 lanes -> u16 lanes.
            let h0lo = u16x8_extadd_pairwise_u8x16(a0);
            let h0hi = u16x8_extadd_pairwise_u8x16(a1);
            let h1lo = u16x8_extadd_pairwise_u8x16(b0);
            let h1hi = u16x8_extadd_pairwise_u8x16(b1);

            let slo = u16x8_shr(i16x8_add(i16x8_add(h0lo, h1lo), two), 2);
            let shi = u16x8_shr(i16x8_add(i16x8_add(h0hi, h1hi), two), 2);

            let out = u8x16_narrow_i16x8(slo, shi);
            v128_store(dst_row.as_mut_ptr().add(o) as *mut v128, out);

            o += 16;
        }

        downsample_row_tail_scalar(row0, row1, dst_row, simd_cols);
    }

    Matrix::<u8>::from_vec(new_h, new_w, 1, dst_data)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a synthetic gradient image: pixel(r, c) = (r * cols + c) as u8.
    fn gradient(rows: usize, cols: usize) -> Matrix<u8> {
        let data: Vec<u8> = (0..rows * cols).map(|i| (i & 0xFF) as u8).collect();
        Matrix::<u8>::from_vec(rows, cols, 1, data)
    }

    // ── User-specified tests ─────────────────────────────────────────────

    #[test]
    fn test_pyramid_level_count() {
        let img = gradient(64, 64);
        let mut p = Pyramid::new(4, 2.0);
        p.build(&img).expect("build should succeed");
        assert_eq!(p.num_levels(), 4);
    }

    #[test]
    fn test_pyramid_level_dimensions_shrink() {
        let img = gradient(64, 64);
        let mut p = Pyramid::new(3, 2.0);
        p.build(&img).unwrap();

        // Level 0: same size as input.
        assert_eq!(p.level(0).rows, 64);
        assert_eq!(p.level(0).cols, 64);
        // Level 1: ceil((64 - 1) / 2) = 32.
        assert_eq!(p.level(1).rows, 32);
        assert_eq!(p.level(1).cols, 32);
        // Level 2: ceil((32 - 1) / 2) = 16.
        assert_eq!(p.level(2).rows, 16);
        assert_eq!(p.level(2).cols, 16);
    }

    #[test]
    fn test_pyramid_pixel_values_in_range() {
        let img = gradient(32, 32);
        let mut p = Pyramid::new(3, 2.0);
        p.build(&img).unwrap();
        // u8 is inherently in [0, 255]; verify every level is fully
        // populated with the right number of pixels.
        for k in 0..p.num_levels() {
            let lvl = p.level(k);
            assert!(lvl.rows > 0 && lvl.cols > 0);
            assert_eq!(lvl.as_slice().len(), lvl.rows * lvl.cols);
        }
    }

    // ── Hardening tests ──────────────────────────────────────────────────

    #[test]
    fn test_pyramid_box_filter_known_values() {
        // 4x4 input where the exact output can be hand-computed.
        // Input:
        //   [ 10,  20,  30,  40]
        //   [ 50,  60,  70,  80]
        //   [ 90, 100, 110, 120]
        //   [130, 140, 150, 160]
        //
        // Output (ceil(3/2) = 2 x 2):
        //   dst[0,0] = (10  + 20  + 50  + 60  + 2) >> 2 = 142 >> 2 = 35
        //   dst[0,1] = (30  + 40  + 70  + 80  + 2) >> 2 = 222 >> 2 = 55
        //   dst[1,0] = (90  + 100 + 130 + 140 + 2) >> 2 = 462 >> 2 = 115
        //   dst[1,1] = (110 + 120 + 150 + 160 + 2) >> 2 = 542 >> 2 = 135
        let data = vec![
            10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160,
        ];
        let src = Matrix::<u8>::from_vec(4, 4, 1, data);
        let mut p = Pyramid::new(2, 2.0);
        p.build(&src).unwrap();
        assert_eq!(p.level(1).as_slice(), &[35, 55, 115, 135]);
    }

    #[test]
    fn test_pyramid_build_is_idempotent() {
        let img1 = gradient(32, 32);
        let img2 = gradient(16, 16);
        let mut p = Pyramid::new(2, 2.0);
        p.build(&img1).unwrap();
        p.build(&img2).unwrap();
        // Levels reflect img2, not img1.
        assert_eq!(p.num_levels(), 2);
        assert_eq!(p.level(0).rows, 16);
        assert_eq!(p.level(0).cols, 16);
    }

    #[test]
    fn test_pyramid_empty_image_returns_error() {
        let img = Matrix::<u8>::zeros(0, 0, 1);
        let mut p = Pyramid::new(2, 2.0);
        assert_eq!(p.build(&img), Err(PyramidError::EmptyImage));
    }

    #[test]
    fn test_pyramid_invalid_scale_returns_error() {
        let img = gradient(16, 16);
        let mut p = Pyramid::new(2, 1.5);
        assert!(matches!(
            p.build(&img),
            Err(PyramidError::InvalidScaleFactor(_))
        ));
    }

    #[test]
    fn test_pyramid_zero_levels_returns_error() {
        let img = gradient(16, 16);
        let mut p = Pyramid::new(0, 2.0);
        assert_eq!(p.build(&img), Err(PyramidError::ZeroLevels));
    }

    #[test]
    fn test_pyramid_level_too_small_returns_error() {
        // 4x4 image: ceil(3/2)=2, ceil(1/2)=1, ceil(0/2)=0 -> fails at level 3.
        let img = gradient(4, 4);
        let mut p = Pyramid::new(5, 2.0);
        assert!(matches!(
            p.build(&img),
            Err(PyramidError::LevelTooSmall { .. })
        ));
    }

    // ── SIMD parity (#132) ───────────────────────────────────────────────

    /// Deterministic pseudo-random image, seeded so failures reproduce.
    #[cfg(any(
        all(target_arch = "x86_64", feature = "simd-x86-sse41"),
        all(target_arch = "x86_64", feature = "simd-x86-avx2"),
    ))]
    fn random_img(rows: usize, cols: usize, seed: u64) -> Matrix<u8> {
        use rand::{Rng, SeedableRng};
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let data: Vec<u8> = (0..rows * cols).map(|_| rng.random::<u8>()).collect();
        Matrix::<u8>::from_vec(rows, cols, 1, data)
    }

    /// Sizes chosen to exercise the SIMD main loop, the scalar remainder
    /// tail, odd dimensions, sub-block widths, and block boundaries.
    #[cfg(any(
        all(target_arch = "x86_64", feature = "simd-x86-sse41"),
        all(target_arch = "x86_64", feature = "simd-x86-avx2"),
    ))]
    const PARITY_SIZES: &[(usize, usize)] = &[
        (2, 2),
        (3, 3),
        (4, 4),
        (5, 7),
        (15, 15),
        (16, 16),
        (17, 17),
        (31, 33),
        (32, 32),
        (33, 31),
        (63, 65),
        (64, 64),
        (65, 63),
        (100, 100),
        (129, 127),
        (200, 320),
    ];

    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-sse41"))]
    #[test]
    fn test_downsample_sse41_matches_scalar() {
        if !is_x86_feature_detected!("sse4.1") {
            eprintln!("sse4.1 not available at runtime; skipping parity test");
            return;
        }
        for (i, &(rows, cols)) in PARITY_SIZES.iter().enumerate() {
            let img = random_img(rows, cols, 0x5E_5E_00 + i as u64);
            let scalar = downsample_scalar(&img);
            // SAFETY: guarded by the runtime `sse4.1` check above.
            let simd = unsafe { downsample_sse41(&img) };
            assert_eq!(
                scalar.as_slice(),
                simd.as_slice(),
                "sse41 mismatch at {rows}x{cols}"
            );
        }
    }

    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-avx2"))]
    #[test]
    fn test_downsample_avx2_matches_scalar() {
        if !is_x86_feature_detected!("avx2") {
            eprintln!("avx2 not available at runtime; skipping parity test");
            return;
        }
        for (i, &(rows, cols)) in PARITY_SIZES.iter().enumerate() {
            let img = random_img(rows, cols, 0xA2_A2_00 + i as u64);
            let scalar = downsample_scalar(&img);
            // SAFETY: guarded by the runtime `avx2` check above.
            let simd = unsafe { downsample_avx2(&img) };
            assert_eq!(
                scalar.as_slice(),
                simd.as_slice(),
                "avx2 mismatch at {rows}x{cols}"
            );
        }
    }

    /// The public dispatcher must agree with the scalar baseline regardless
    /// of which path it selects at runtime.
    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-sse41"))]
    #[test]
    fn test_downsample_dispatch_matches_scalar() {
        for (i, &(rows, cols)) in PARITY_SIZES.iter().enumerate() {
            let img = random_img(rows, cols, 0xD1_5A_00 + i as u64);
            assert_eq!(
                downsample_scalar(&img).as_slice(),
                downsample(&img).as_slice(),
                "dispatch mismatch at {rows}x{cols}"
            );
        }
    }
}
