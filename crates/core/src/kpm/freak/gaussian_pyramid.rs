/*
 *  gaussian_pyramid.rs
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

//! Binomial Gaussian scale-space pyramid (f32 levels, 3 scales per octave).
//!
//! Ported from
//! `WebARKitLib/lib/SRC/KPM/FreakMatcher/detectors/gaussian_scale_space_pyramid.{h,cpp}`
//! (the `BinomialPyramid32f` concrete impl plus its `GaussianScaleSpacePyramid`
//! base). The C++ inheritance hierarchy is flattened into one Rust type since
//! `BinomialPyramid32f` is the only concrete implementation in use.
//!
//! Input is `Matrix<u8>` (grayscale); output is `Matrix<f32>` per level
//! (matches C++ `IMAGE_F32` storage). The binomial filter is a fixed 5-tap
//! `[1, 4, 6, 4, 1]` kernel applied separably (H then V), with `1 / 256`
//! normalization on the V pass. Border handling replicates edge pixels for
//! the 2-pixel border on each side, matching the C++ exactly.

use crate::{arlog_e, arlog_w};
use purecv::core::Matrix;

/// Errors that can occur while building a [`GaussianScaleSpacePyramid`].
#[derive(Debug, Clone, PartialEq)]
pub enum GaussianPyramidError {
    /// Input image had zero rows or zero columns.
    EmptyImage,
    /// Input image is smaller than 5x5; the binomial filter needs 2 border
    /// pixels on each side.
    ImageTooSmall { rows: usize, cols: usize },
    /// `num_octaves` was 0 — a pyramid must have at least one octave.
    ZeroOctaves,
    /// Halving for `octave` would produce a level smaller than 5x5.
    OctaveTooSmall {
        octave: usize,
        rows: usize,
        cols: usize,
    },
}

impl std::fmt::Display for GaussianPyramidError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyImage => write!(f, "input image is empty (0 rows or 0 cols)"),
            Self::ImageTooSmall { rows, cols } => write!(
                f,
                "input image is {rows}x{cols}; binomial filter requires >= 5x5"
            ),
            Self::ZeroOctaves => write!(f, "num_octaves must be >= 1"),
            Self::OctaveTooSmall { octave, rows, cols } => write!(
                f,
                "octave {octave} would be {rows}x{cols}; binomial filter requires >= 5x5"
            ),
        }
    }
}

impl std::error::Error for GaussianPyramidError {}

/// Binomial Gaussian scale-space pyramid.
///
/// Each octave contains exactly [`Self::NUM_SCALES_PER_OCTAVE`] = 3 levels.
/// Level dimensions: octave `o` has size `(rows >> o, cols >> o)`. Level
/// storage is `Matrix<f32>` (row-major).
#[derive(Debug, Clone)]
pub struct GaussianScaleSpacePyramid {
    /// `octaves[oct][scale]` — `Matrix<f32>` for the level.
    pub octaves: Vec<Vec<Matrix<f32>>>,
    /// Number of octaves requested at construction.
    pub num_octaves: usize,
    /// Scale step factor: `k = 2^(1 / (NUM_SCALES_PER_OCTAVE - 1)) = sqrt(2)`.
    pub kfactor: f32,
    /// `1 / ln(k)` — precomputed for [`Self::locate`].
    pub one_over_log_k: f32,
}

impl GaussianScaleSpacePyramid {
    /// C++ `BinomialPyramid32f::alloc` hardcodes 3 scales per octave; the
    /// build sequence (`filter`, `filter`, `filter_twice`) is 3-specific.
    pub const NUM_SCALES_PER_OCTAVE: usize = 3;

    /// Create an empty pyramid configured for `num_octaves` octaves.
    /// Levels are populated by [`Self::build`].
    #[must_use]
    pub fn new(num_octaves: usize) -> Self {
        let k = 2f32.powf(1.0 / (Self::NUM_SCALES_PER_OCTAVE as f32 - 1.0));
        Self {
            octaves: Vec::with_capacity(num_octaves),
            num_octaves,
            kfactor: k,
            one_over_log_k: 1.0 / k.ln(),
        }
    }

    /// Number of scales per octave (always [`Self::NUM_SCALES_PER_OCTAVE`]).
    #[must_use]
    pub fn num_scales_per_octave(&self) -> usize {
        Self::NUM_SCALES_PER_OCTAVE
    }

    /// Borrow level at `(octave, scale)`.
    ///
    /// # Panics
    ///
    /// Panics if `octave >= num_octaves` or `scale >= NUM_SCALES_PER_OCTAVE`.
    #[must_use]
    pub fn level(&self, octave: usize, scale: usize) -> &Matrix<f32> {
        &self.octaves[octave][scale]
    }

    /// Effective sigma at `(octave, scale)`: `k^scale * 2^octave`.
    #[must_use]
    pub fn effective_sigma(&self, octave: usize, scale: usize) -> f32 {
        self.kfactor.powi(scale as i32) * (1u32 << octave) as f32
    }

    /// Snap `sigma` to the nearest `(octave, scale)` pair. Clamped to the
    /// pyramid bounds.
    ///
    /// C equivalent: `vision::GaussianScaleSpacePyramid::locate(int&, int&, float)`.
    #[must_use]
    pub fn locate(&self, sigma: f32) -> (usize, usize) {
        let mut octave = sigma.log2().floor() as i32;
        let octave_pow = if octave >= 0 {
            (1i32 << octave) as f32
        } else {
            1.0 / (1i32 << -octave) as f32
        };
        let fscale = (sigma / octave_pow).ln() * self.one_over_log_k;
        let mut scale = fscale.round() as i32;

        // C++: if scale is the last in octave, bump to next octave's scale 0
        // (coarser octaves are preferred for efficiency).
        if scale == (Self::NUM_SCALES_PER_OCTAVE as i32) - 1 {
            octave += 1;
            scale = 0;
        }

        // Clamp to pyramid bounds.
        if octave < 0 {
            return (0, 0);
        }
        if (octave as usize) >= self.num_octaves {
            return (self.num_octaves - 1, Self::NUM_SCALES_PER_OCTAVE - 1);
        }
        let scale = scale.clamp(0, Self::NUM_SCALES_PER_OCTAVE as i32 - 1) as usize;
        (octave as usize, scale)
    }

    /// Populate `self.octaves` from `image`. Idempotent (clears prior state).
    ///
    /// Build sequence per C++ `BinomialPyramid32f::build`:
    /// - Octave 0:
    ///   - `octaves[0][0] = filter(input)` (u8 → f32)
    ///   - `octaves[0][1] = filter(octaves[0][0])` (f32 → f32)
    ///   - `octaves[0][2] = filter(filter(octaves[0][1]))` (2 applications)
    /// - Octave i (i > 0):
    ///   - `octaves[i][0] = downsample(octaves[i-1][2])`
    ///   - `octaves[i][1] = filter(octaves[i][0])`
    ///   - `octaves[i][2] = filter(filter(octaves[i][1]))`
    pub fn build(&mut self, image: &Matrix<u8>) -> Result<(), GaussianPyramidError> {
        if self.num_octaves == 0 {
            arlog_e!("GaussianScaleSpacePyramid::build: num_octaves is 0");
            return Err(GaussianPyramidError::ZeroOctaves);
        }
        if image.rows == 0 || image.cols == 0 {
            arlog_e!("GaussianScaleSpacePyramid::build: input image is empty");
            return Err(GaussianPyramidError::EmptyImage);
        }
        if image.rows < 5 || image.cols < 5 {
            arlog_e!(
                "GaussianScaleSpacePyramid::build: image too small ({}x{}); need >= 5x5",
                image.rows,
                image.cols
            );
            return Err(GaussianPyramidError::ImageTooSmall {
                rows: image.rows,
                cols: image.cols,
            });
        }
        if image.channels != 1 {
            arlog_w!(
                "GaussianScaleSpacePyramid::build: input has {} channels; only the first is used",
                image.channels
            );
        }

        self.octaves.clear();

        // Octave 0.
        let l0 = binomial_4th_order_u8_to_f32(image);
        let l1 = binomial_4th_order_f32_to_f32(&l0);
        let l2_tmp = binomial_4th_order_f32_to_f32(&l1);
        let l2 = binomial_4th_order_f32_to_f32(&l2_tmp);
        self.octaves.push(vec![l0, l1, l2]);

        // Octaves 1..num_octaves.
        for oct in 1..self.num_octaves {
            let prev_last = &self.octaves[oct - 1][Self::NUM_SCALES_PER_OCTAVE - 1];
            let new_h = prev_last.rows >> 1;
            let new_w = prev_last.cols >> 1;
            if new_h < 5 || new_w < 5 {
                arlog_e!(
                    "GaussianScaleSpacePyramid::build: octave {oct} would be {new_h}x{new_w}; need >= 5x5"
                );
                return Err(GaussianPyramidError::OctaveTooSmall {
                    octave: oct,
                    rows: new_h,
                    cols: new_w,
                });
            }
            let l0 = downsample_bilinear_f32(prev_last);
            let l1 = binomial_4th_order_f32_to_f32(&l0);
            let l2_tmp = binomial_4th_order_f32_to_f32(&l1);
            let l2 = binomial_4th_order_f32_to_f32(&l2_tmp);
            self.octaves.push(vec![l0, l1, l2]);
        }

        Ok(())
    }
}

/// Compute the number of octaves usable for a given image size, given the
/// minimum coarsest-octave dimension.
///
/// C equivalent: `vision::numOctaves`.
#[must_use]
pub fn num_octaves_for(width: usize, height: usize, min_size: usize) -> usize {
    let mut w = width;
    let mut h = height;
    let mut n = 0;
    while w >= min_size && h >= min_size {
        w >>= 1;
        h >>= 1;
        n += 1;
    }
    n
}

// ─────────────────────────────────────────────────────────────────────────
// Private filter helpers — byte-for-byte port of C++ `binomial_4th_order`
// and `downsample_bilinear` from `gaussian_scale_space_pyramid.cpp`.
// ─────────────────────────────────────────────────────────────────────────

// Note on visibility: the binomial dispatchers, their `*_scalar` /
// `*_avx2` / `*_sse41` variants, and the `downsample_bilinear_f32*`
// helpers are `pub` so the criterion benchmark (a separate crate) can
// measure them directly. They are not a stability guarantee — prefer
// [`GaussianScaleSpacePyramid::build`].

// u8 → f32 binomial helpers. The H pass is exact `u16` integer arithmetic
// (max 4080); the V pass sums in `u32` then does a single `as f32 * INV_256`.
// Integer ops are associative/exact, and the lone float multiply matches the
// scalar exactly, so SIMD variants reusing these for borders/remainder stay
// bit-for-bit identical (validated by the `dual-mode` parity test).

/// Horizontal-pass border columns (0, 1, width-2, width-1), u8 → u16.
#[inline]
fn binomial_h_borders_u8(s: &[u8], t: &mut [u16], width: usize) {
    t[0] = 6 * s[0] as u16 + 4 * (s[0] as u16 + s[1] as u16) + (s[0] as u16 + s[2] as u16);
    t[1] = 6 * s[1] as u16 + 4 * (s[0] as u16 + s[2] as u16) + (s[0] as u16 + s[3] as u16);
    let c = width - 2;
    t[c] = 6 * s[c] as u16
        + 4 * (s[c - 1] as u16 + s[c + 1] as u16)
        + (s[c - 2] as u16 + s[c + 1] as u16);
    let c = width - 1;
    t[c] = 6 * s[c] as u16 + 4 * (s[c - 1] as u16 + s[c] as u16) + (s[c - 2] as u16 + s[c] as u16);
}

/// Horizontal-pass interior columns `start..end`, u8 → u16.
#[inline]
fn binomial_h_interior_u8(s: &[u8], t: &mut [u16], start: usize, end: usize) {
    for col in start..end {
        t[col] = 6 * s[col] as u16
            + 4 * (s[col - 1] as u16 + s[col + 1] as u16)
            + (s[col - 2] as u16 + s[col + 2] as u16);
    }
}

/// Vertical-pass border rows (0, 1, height-2, height-1), u16 → f32.
#[inline]
fn binomial_v_borders_u8(tmp: &[u16], dst: &mut [f32], width: usize, height: usize) {
    for col in 0..width {
        let p = tmp[col] as u32;
        let pp1 = tmp[width + col] as u32;
        let pp2 = tmp[2 * width + col] as u32;
        dst[col] = ((6 * p + 4 * (p + pp1) + p + pp2) as f32) * INV_256;
        let pm = tmp[col] as u32;
        let p = tmp[width + col] as u32;
        let pp1 = tmp[2 * width + col] as u32;
        let pp2 = tmp[3 * width + col] as u32;
        dst[width + col] = ((6 * p + 4 * (pm + pp1) + pm + pp2) as f32) * INV_256;
    }
    let h = height;
    for col in 0..width {
        let pm2 = tmp[(h - 4) * width + col] as u32;
        let pm1 = tmp[(h - 3) * width + col] as u32;
        let p = tmp[(h - 2) * width + col] as u32;
        let pp = tmp[(h - 1) * width + col] as u32;
        dst[(h - 2) * width + col] = ((6 * p + 4 * (pm1 + pp) + pm2 + pp) as f32) * INV_256;
        let pm2 = tmp[(h - 3) * width + col] as u32;
        let pm1 = tmp[(h - 2) * width + col] as u32;
        let p = tmp[(h - 1) * width + col] as u32;
        dst[(h - 1) * width + col] = ((6 * p + 4 * (pm1 + p) + pm2 + p) as f32) * INV_256;
    }
}

/// Vertical-pass interior row `row`, columns `start..end`, u16 → f32.
#[inline]
fn binomial_v_interior_u8(
    tmp: &[u16],
    dst: &mut [f32],
    row: usize,
    width: usize,
    start: usize,
    end: usize,
) {
    let row_off = row * width;
    for col in start..end {
        let pm2 = tmp[(row - 2) * width + col] as u32;
        let pm1 = tmp[(row - 1) * width + col] as u32;
        let p = tmp[row_off + col] as u32;
        let pp1 = tmp[(row + 1) * width + col] as u32;
        let pp2 = tmp[(row + 2) * width + col] as u32;
        dst[row_off + col] = ((6 * p + 4 * (pm1 + pp1) + pm2 + pp2) as f32) * INV_256;
    }
}

/// 5-tap separable `[1, 4, 6, 4, 1]` binomial filter, u8 source → f32 dest.
///
/// H pass uses `u16` accumulator (max value `16 * 255 = 4080`, fits `u16`).
/// V pass multiplies by `1 / 256` to yield `f32` output. Border replication:
/// edge pixels extend the 2-pixel border on each side.
///
/// C equivalent: `vision::binomial_4th_order(float*, unsigned short*, const unsigned char*, ...)`.
#[must_use]
pub fn binomial_4th_order_u8_to_f32_scalar(src: &Matrix<u8>) -> Matrix<f32> {
    let width = src.cols;
    let height = src.rows;
    debug_assert!(width >= 5 && height >= 5);

    let src_data = src.as_slice();
    let mut tmp = vec![0u16; width * height];

    // Horizontal pass.
    for row in 0..height {
        let row_off = row * width;
        let s = &src_data[row_off..row_off + width];
        let t = &mut tmp[row_off..row_off + width];
        binomial_h_borders_u8(s, t, width);
        binomial_h_interior_u8(s, t, 2, width - 2);
    }

    // Vertical pass.
    let mut dst_data = vec![0f32; width * height];
    binomial_v_borders_u8(&tmp, &mut dst_data, width, height);
    for row in 2..height - 2 {
        binomial_v_interior_u8(&tmp, &mut dst_data, row, width, 0, width);
    }

    Matrix::<f32>::from_vec(height, width, 1, dst_data)
}

/// AVX2 u8 → f32 binomial filter. H pass vectorized 16 cols/iter (exact
/// `i16` integer math); V pass 8 cols/iter (`i32` sum → `f32 * INV_256`).
/// Borders/remainder use the shared scalar helpers, so output is
/// bit-for-bit identical to [`binomial_4th_order_u8_to_f32_scalar`].
///
/// # Safety
///
/// Caller must ensure the `avx2` target feature is available at runtime
/// (the [`binomial_4th_order_u8_to_f32`] dispatcher guarantees this).
#[cfg(all(target_arch = "x86_64", feature = "simd-x86-avx2"))]
#[target_feature(enable = "avx2")]
#[must_use]
pub unsafe fn binomial_4th_order_u8_to_f32_avx2(src: &Matrix<u8>) -> Matrix<f32> {
    use std::arch::x86_64::*;

    let width = src.cols;
    let height = src.rows;
    debug_assert!(width >= 5 && height >= 5);

    let src_data = src.as_slice();
    let mut tmp = vec![0u16; width * height];

    let six16 = _mm256_set1_epi16(6);
    let four16 = _mm256_set1_epi16(4);

    // Horizontal pass (16 cols/iter). Values <= 4080 fit i16 exactly.
    for row in 0..height {
        let row_off = row * width;
        let s = &src_data[row_off..row_off + width];
        let t = &mut tmp[row_off..row_off + width];
        binomial_h_borders_u8(s, t, width);
        let end = width - 2;
        let mut col = 2;
        while col + 16 <= end {
            // SAFETY: col >= 2 and col + 16 <= width - 2, so each 16-byte load
            // (s[col-2 ..= col+17]) and the 16×u16 store to t[col..col+16] are
            // in bounds.
            let c0 =
                _mm256_cvtepu8_epi16(_mm_loadu_si128(s.as_ptr().add(col - 2) as *const __m128i));
            let c1 =
                _mm256_cvtepu8_epi16(_mm_loadu_si128(s.as_ptr().add(col - 1) as *const __m128i));
            let c2 = _mm256_cvtepu8_epi16(_mm_loadu_si128(s.as_ptr().add(col) as *const __m128i));
            let c3 =
                _mm256_cvtepu8_epi16(_mm_loadu_si128(s.as_ptr().add(col + 1) as *const __m128i));
            let c4 =
                _mm256_cvtepu8_epi16(_mm_loadu_si128(s.as_ptr().add(col + 2) as *const __m128i));
            let inner = _mm256_add_epi16(c1, c3);
            let mut r = _mm256_add_epi16(
                _mm256_mullo_epi16(six16, c2),
                _mm256_mullo_epi16(four16, inner),
            );
            r = _mm256_add_epi16(r, c0);
            r = _mm256_add_epi16(r, c4);
            _mm256_storeu_si256(t.as_mut_ptr().add(col) as *mut __m256i, r);
            col += 16;
        }
        binomial_h_interior_u8(s, t, col, end);
    }

    // Vertical pass (8 cols/iter). Sum <= 65280 fits i32; convert then scale.
    let mut dst_data = vec![0f32; width * height];
    binomial_v_borders_u8(&tmp, &mut dst_data, width, height);
    let six32 = _mm256_set1_epi32(6);
    let four32 = _mm256_set1_epi32(4);
    let inv = _mm256_set1_ps(INV_256);
    for row in 2..height - 2 {
        let row_off = row * width;
        let mut col = 0;
        while col + 8 <= width {
            // SAFETY: col + 8 <= width and 2 <= row < height-2, so each 8×u16
            // load and the 8×f32 store to dst[row_off+col..+8] are in bounds.
            let pm2 = _mm256_cvtepu16_epi32(_mm_loadu_si128(
                tmp.as_ptr().add((row - 2) * width + col) as *const __m128i,
            ));
            let pm1 = _mm256_cvtepu16_epi32(_mm_loadu_si128(
                tmp.as_ptr().add((row - 1) * width + col) as *const __m128i,
            ));
            let p = _mm256_cvtepu16_epi32(_mm_loadu_si128(
                tmp.as_ptr().add(row_off + col) as *const __m128i
            ));
            let pp1 = _mm256_cvtepu16_epi32(_mm_loadu_si128(
                tmp.as_ptr().add((row + 1) * width + col) as *const __m128i,
            ));
            let pp2 = _mm256_cvtepu16_epi32(_mm_loadu_si128(
                tmp.as_ptr().add((row + 2) * width + col) as *const __m128i,
            ));
            let inner = _mm256_add_epi32(pm1, pp1);
            let mut r = _mm256_add_epi32(
                _mm256_mullo_epi32(six32, p),
                _mm256_mullo_epi32(four32, inner),
            );
            r = _mm256_add_epi32(r, pm2);
            r = _mm256_add_epi32(r, pp2);
            let rf = _mm256_mul_ps(_mm256_cvtepi32_ps(r), inv);
            _mm256_storeu_ps(dst_data.as_mut_ptr().add(row_off + col), rf);
            col += 8;
        }
        binomial_v_interior_u8(&tmp, &mut dst_data, row, width, col, width);
    }

    Matrix::<f32>::from_vec(height, width, 1, dst_data)
}

/// SSE4.1 u8 → f32 binomial filter. H pass 8 cols/iter, V pass 4 cols/iter.
/// Bit-for-bit identical to [`binomial_4th_order_u8_to_f32_scalar`].
///
/// # Safety
///
/// Caller must ensure the `sse4.1` target feature is available at runtime
/// (the [`binomial_4th_order_u8_to_f32`] dispatcher guarantees this).
#[cfg(all(target_arch = "x86_64", feature = "simd-x86-sse41"))]
#[target_feature(enable = "sse4.1")]
#[must_use]
pub unsafe fn binomial_4th_order_u8_to_f32_sse41(src: &Matrix<u8>) -> Matrix<f32> {
    use std::arch::x86_64::*;

    let width = src.cols;
    let height = src.rows;
    debug_assert!(width >= 5 && height >= 5);

    let src_data = src.as_slice();
    let mut tmp = vec![0u16; width * height];

    let six16 = _mm_set1_epi16(6);
    let four16 = _mm_set1_epi16(4);

    for row in 0..height {
        let row_off = row * width;
        let s = &src_data[row_off..row_off + width];
        let t = &mut tmp[row_off..row_off + width];
        binomial_h_borders_u8(s, t, width);
        let end = width - 2;
        let mut col = 2;
        while col + 8 <= end {
            // SAFETY: col >= 2 and col + 8 <= width - 2, so each 8-byte load
            // (s[col-2 ..= col+9]) and the 8×u16 store to t[col..col+8] are
            // in bounds.
            let c0 = _mm_cvtepu8_epi16(_mm_loadl_epi64(s.as_ptr().add(col - 2) as *const __m128i));
            let c1 = _mm_cvtepu8_epi16(_mm_loadl_epi64(s.as_ptr().add(col - 1) as *const __m128i));
            let c2 = _mm_cvtepu8_epi16(_mm_loadl_epi64(s.as_ptr().add(col) as *const __m128i));
            let c3 = _mm_cvtepu8_epi16(_mm_loadl_epi64(s.as_ptr().add(col + 1) as *const __m128i));
            let c4 = _mm_cvtepu8_epi16(_mm_loadl_epi64(s.as_ptr().add(col + 2) as *const __m128i));
            let inner = _mm_add_epi16(c1, c3);
            let mut r = _mm_add_epi16(_mm_mullo_epi16(six16, c2), _mm_mullo_epi16(four16, inner));
            r = _mm_add_epi16(r, c0);
            r = _mm_add_epi16(r, c4);
            _mm_storeu_si128(t.as_mut_ptr().add(col) as *mut __m128i, r);
            col += 8;
        }
        binomial_h_interior_u8(s, t, col, end);
    }

    let mut dst_data = vec![0f32; width * height];
    binomial_v_borders_u8(&tmp, &mut dst_data, width, height);
    let six32 = _mm_set1_epi32(6);
    let four32 = _mm_set1_epi32(4);
    let inv = _mm_set1_ps(INV_256);
    for row in 2..height - 2 {
        let row_off = row * width;
        let mut col = 0;
        while col + 4 <= width {
            // SAFETY: col + 4 <= width and 2 <= row < height-2, so each 4×u16
            // load and the 4×f32 store to dst[row_off+col..+4] are in bounds.
            let pm2 = _mm_cvtepu16_epi32(_mm_loadl_epi64(
                tmp.as_ptr().add((row - 2) * width + col) as *const __m128i,
            ));
            let pm1 = _mm_cvtepu16_epi32(_mm_loadl_epi64(
                tmp.as_ptr().add((row - 1) * width + col) as *const __m128i,
            ));
            let p = _mm_cvtepu16_epi32(_mm_loadl_epi64(
                tmp.as_ptr().add(row_off + col) as *const __m128i
            ));
            let pp1 = _mm_cvtepu16_epi32(_mm_loadl_epi64(
                tmp.as_ptr().add((row + 1) * width + col) as *const __m128i,
            ));
            let pp2 = _mm_cvtepu16_epi32(_mm_loadl_epi64(
                tmp.as_ptr().add((row + 2) * width + col) as *const __m128i,
            ));
            let inner = _mm_add_epi32(pm1, pp1);
            let mut r = _mm_add_epi32(_mm_mullo_epi32(six32, p), _mm_mullo_epi32(four32, inner));
            r = _mm_add_epi32(r, pm2);
            r = _mm_add_epi32(r, pp2);
            let rf = _mm_mul_ps(_mm_cvtepi32_ps(r), inv);
            _mm_storeu_ps(dst_data.as_mut_ptr().add(row_off + col), rf);
            col += 4;
        }
        binomial_v_interior_u8(&tmp, &mut dst_data, row, width, col, width);
    }

    Matrix::<f32>::from_vec(height, width, 1, dst_data)
}

/// 5-tap separable `[1, 4, 6, 4, 1]` binomial filter, u8 → f32.
///
/// Dispatches AVX2 → SSE4.1 → scalar on x86_64 (runtime detection); scalar
/// elsewhere. Every path is bit-for-bit identical to
/// [`binomial_4th_order_u8_to_f32_scalar`].
///
/// (A wasm32 SIMD path is a small follow-up: this filter runs once per
/// build — octave 0 — so the dominant per-octave cost is the f32 → f32
/// filter, which does have a wasm32 path.)
#[must_use]
pub fn binomial_4th_order_u8_to_f32(src: &Matrix<u8>) -> Matrix<f32> {
    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-avx2"))]
    {
        if is_x86_feature_detected!("avx2") {
            // SAFETY: avx2 confirmed available at runtime.
            return unsafe { binomial_4th_order_u8_to_f32_avx2(src) };
        }
    }
    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-sse41"))]
    {
        if is_x86_feature_detected!("sse4.1") {
            // SAFETY: sse4.1 confirmed available at runtime.
            return unsafe { binomial_4th_order_u8_to_f32_sse41(src) };
        }
    }

    #[allow(unreachable_code)]
    binomial_4th_order_u8_to_f32_scalar(src)
}

/// 5-tap separable `[1, 4, 6, 4, 1]` binomial filter, f32 → f32.
///
/// Dispatches AVX2 → SSE4.1 → scalar on x86_64 (runtime detection),
/// simd128 on wasm32. Every path is bit-for-bit identical to
/// [`binomial_4th_order_f32_to_f32_scalar`] (no FMA, matched op order).
#[must_use]
pub fn binomial_4th_order_f32_to_f32(src: &Matrix<f32>) -> Matrix<f32> {
    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-avx2"))]
    {
        if is_x86_feature_detected!("avx2") {
            // SAFETY: avx2 confirmed available at runtime.
            return unsafe { binomial_4th_order_f32_to_f32_avx2(src) };
        }
    }
    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-sse41"))]
    {
        if is_x86_feature_detected!("sse4.1") {
            // SAFETY: sse4.1 confirmed available at runtime.
            return unsafe { binomial_4th_order_f32_to_f32_sse41(src) };
        }
    }
    #[cfg(all(
        target_arch = "wasm32",
        feature = "simd-wasm32",
        target_feature = "simd128"
    ))]
    {
        // SAFETY: simd128 guaranteed by the cfg gate.
        return unsafe { binomial_4th_order_f32_to_f32_wasm(src) };
    }

    #[allow(unreachable_code)]
    binomial_4th_order_f32_to_f32_scalar(src)
}

/// `1 / 256` normalization applied on the vertical pass of the binomial
/// filter. Exactly representable in f32.
const INV_256: f32 = 1.0 / 256.0;

// The four helpers below are the single source of truth for the binomial
// filter arithmetic. The scalar reference and every SIMD variant call them
// for the borders and the scalar remainder, so the exact f32 operation
// order (`((6*c + 4*(l+r)) + ll) + rr`, no FMA) is defined in one place and
// stays bit-for-bit identical to the C++ baseline (validated by the
// `dual-mode` parity test).

/// Horizontal-pass border columns (0, 1, width-2, width-1) for one row.
/// `s` and `t` are the row slices of the source and the temp buffer.
#[inline]
fn binomial_h_borders_f32(s: &[f32], t: &mut [f32], width: usize) {
    t[0] = 6.0 * s[0] + 4.0 * (s[0] + s[1]) + s[0] + s[2];
    t[1] = 6.0 * s[1] + 4.0 * (s[0] + s[2]) + s[0] + s[3];
    let c = width - 2;
    t[c] = 6.0 * s[c] + 4.0 * (s[c - 1] + s[c + 1]) + s[c - 2] + s[c + 1];
    let c = width - 1;
    t[c] = 6.0 * s[c] + 4.0 * (s[c - 1] + s[c]) + s[c - 2] + s[c];
}

/// Horizontal-pass interior columns `start..end` (each reads `s[col-2..=col+2]`).
#[inline]
fn binomial_h_interior_f32(s: &[f32], t: &mut [f32], start: usize, end: usize) {
    for col in start..end {
        t[col] = 6.0 * s[col] + 4.0 * (s[col - 1] + s[col + 1]) + s[col - 2] + s[col + 2];
    }
}

/// Vertical-pass border rows (0, 1, height-2, height-1) for all columns.
#[inline]
fn binomial_v_borders_f32(tmp: &[f32], dst: &mut [f32], width: usize, height: usize) {
    for col in 0..width {
        // row 0
        let p = tmp[col];
        let pp1 = tmp[width + col];
        let pp2 = tmp[2 * width + col];
        dst[col] = (6.0 * p + 4.0 * (p + pp1) + p + pp2) * INV_256;
        // row 1
        let pm = tmp[col];
        let p = tmp[width + col];
        let pp1 = tmp[2 * width + col];
        let pp2 = tmp[3 * width + col];
        dst[width + col] = (6.0 * p + 4.0 * (pm + pp1) + pm + pp2) * INV_256;
    }
    let h = height;
    for col in 0..width {
        let pm2 = tmp[(h - 4) * width + col];
        let pm1 = tmp[(h - 3) * width + col];
        let p = tmp[(h - 2) * width + col];
        let pp = tmp[(h - 1) * width + col];
        dst[(h - 2) * width + col] = (6.0 * p + 4.0 * (pm1 + pp) + pm2 + pp) * INV_256;
        let pm2 = tmp[(h - 3) * width + col];
        let pm1 = tmp[(h - 2) * width + col];
        let p = tmp[(h - 1) * width + col];
        dst[(h - 1) * width + col] = (6.0 * p + 4.0 * (pm1 + p) + pm2 + p) * INV_256;
    }
}

/// Vertical-pass interior row `row`, columns `start..end`.
#[inline]
fn binomial_v_interior_f32(
    tmp: &[f32],
    dst: &mut [f32],
    row: usize,
    width: usize,
    start: usize,
    end: usize,
) {
    let row_off = row * width;
    for col in start..end {
        let pm2 = tmp[(row - 2) * width + col];
        let pm1 = tmp[(row - 1) * width + col];
        let p = tmp[row_off + col];
        let pp1 = tmp[(row + 1) * width + col];
        let pp2 = tmp[(row + 2) * width + col];
        dst[row_off + col] = (6.0 * p + 4.0 * (pm1 + pp1) + pm2 + pp2) * INV_256;
    }
}

/// 5-tap separable `[1, 4, 6, 4, 1]` binomial filter, f32 → f32. Used for
/// the second and third levels within an octave and for all levels in
/// non-zero octaves.
///
/// C equivalent: `vision::binomial_4th_order(float*, float*, const float*, ...)`.
#[must_use]
pub fn binomial_4th_order_f32_to_f32_scalar(src: &Matrix<f32>) -> Matrix<f32> {
    let width = src.cols;
    let height = src.rows;
    debug_assert!(width >= 5 && height >= 5);

    let src_data = src.as_slice();
    let mut tmp = vec![0f32; width * height];

    // Horizontal pass.
    for row in 0..height {
        let row_off = row * width;
        let s = &src_data[row_off..row_off + width];
        let t = &mut tmp[row_off..row_off + width];
        binomial_h_borders_f32(s, t, width);
        binomial_h_interior_f32(s, t, 2, width - 2);
    }

    // Vertical pass.
    let mut dst_data = vec![0f32; width * height];
    binomial_v_borders_f32(&tmp, &mut dst_data, width, height);
    for row in 2..height - 2 {
        binomial_v_interior_f32(&tmp, &mut dst_data, row, width, 0, width);
    }

    Matrix::<f32>::from_vec(height, width, 1, dst_data)
}

/// AVX2 f32 → f32 binomial filter. Vectorizes the interior of both passes
/// (8 lanes/iter); borders and the per-row remainder use the shared scalar
/// helpers, so output is **bit-for-bit identical** to
/// [`binomial_4th_order_f32_to_f32_scalar`].
///
/// No FMA is used (separate `mul`/`add`), matching the C++ baseline built
/// with `-ffp-contract=off` and the scalar operation order.
///
/// # Safety
///
/// Caller must ensure the `avx2` target feature is available at runtime
/// (the [`binomial_4th_order_f32_to_f32`] dispatcher guarantees this).
#[cfg(all(target_arch = "x86_64", feature = "simd-x86-avx2"))]
#[target_feature(enable = "avx2")]
#[must_use]
pub unsafe fn binomial_4th_order_f32_to_f32_avx2(src: &Matrix<f32>) -> Matrix<f32> {
    use std::arch::x86_64::*;

    let width = src.cols;
    let height = src.rows;
    debug_assert!(width >= 5 && height >= 5);

    let src_data = src.as_slice();
    let mut tmp = vec![0f32; width * height];

    let six = _mm256_set1_ps(6.0);
    let four = _mm256_set1_ps(4.0);

    // Horizontal pass.
    for row in 0..height {
        let row_off = row * width;
        let s = &src_data[row_off..row_off + width];
        let t = &mut tmp[row_off..row_off + width];
        binomial_h_borders_f32(s, t, width);
        let end = width - 2;
        let mut col = 2;
        while col + 8 <= end {
            // SAFETY: col >= 2 and col + 8 <= width - 2, so reads of
            // s[col-2 ..= col+9] and the write to t[col..col+8] are in bounds.
            let c0 = _mm256_loadu_ps(s.as_ptr().add(col - 2));
            let c1 = _mm256_loadu_ps(s.as_ptr().add(col - 1));
            let c2 = _mm256_loadu_ps(s.as_ptr().add(col));
            let c3 = _mm256_loadu_ps(s.as_ptr().add(col + 1));
            let c4 = _mm256_loadu_ps(s.as_ptr().add(col + 2));
            let inner = _mm256_add_ps(c1, c3);
            let mut r = _mm256_add_ps(_mm256_mul_ps(six, c2), _mm256_mul_ps(four, inner));
            r = _mm256_add_ps(r, c0);
            r = _mm256_add_ps(r, c4);
            _mm256_storeu_ps(t.as_mut_ptr().add(col), r);
            col += 8;
        }
        binomial_h_interior_f32(s, t, col, end);
    }

    // Vertical pass.
    let mut dst_data = vec![0f32; width * height];
    binomial_v_borders_f32(&tmp, &mut dst_data, width, height);
    let inv = _mm256_set1_ps(INV_256);
    for row in 2..height - 2 {
        let row_off = row * width;
        let mut col = 0;
        while col + 8 <= width {
            // SAFETY: col + 8 <= width and 2 <= row < height-2, so the five
            // row reads and the write to dst[row_off+col..+8] are in bounds.
            let pm2 = _mm256_loadu_ps(tmp.as_ptr().add((row - 2) * width + col));
            let pm1 = _mm256_loadu_ps(tmp.as_ptr().add((row - 1) * width + col));
            let p = _mm256_loadu_ps(tmp.as_ptr().add(row_off + col));
            let pp1 = _mm256_loadu_ps(tmp.as_ptr().add((row + 1) * width + col));
            let pp2 = _mm256_loadu_ps(tmp.as_ptr().add((row + 2) * width + col));
            let inner = _mm256_add_ps(pm1, pp1);
            let mut r = _mm256_add_ps(_mm256_mul_ps(six, p), _mm256_mul_ps(four, inner));
            r = _mm256_add_ps(r, pm2);
            r = _mm256_add_ps(r, pp2);
            r = _mm256_mul_ps(r, inv);
            _mm256_storeu_ps(dst_data.as_mut_ptr().add(row_off + col), r);
            col += 8;
        }
        binomial_v_interior_f32(&tmp, &mut dst_data, row, width, col, width);
    }

    Matrix::<f32>::from_vec(height, width, 1, dst_data)
}

/// SSE4.1 f32 → f32 binomial filter (4 lanes/iter). Bit-for-bit identical
/// to [`binomial_4th_order_f32_to_f32_scalar`]; no FMA.
///
/// # Safety
///
/// Caller must ensure the `sse4.1` target feature is available at runtime
/// (the [`binomial_4th_order_f32_to_f32`] dispatcher guarantees this).
#[cfg(all(target_arch = "x86_64", feature = "simd-x86-sse41"))]
#[target_feature(enable = "sse4.1")]
#[must_use]
pub unsafe fn binomial_4th_order_f32_to_f32_sse41(src: &Matrix<f32>) -> Matrix<f32> {
    use std::arch::x86_64::*;

    let width = src.cols;
    let height = src.rows;
    debug_assert!(width >= 5 && height >= 5);

    let src_data = src.as_slice();
    let mut tmp = vec![0f32; width * height];

    let six = _mm_set1_ps(6.0);
    let four = _mm_set1_ps(4.0);

    for row in 0..height {
        let row_off = row * width;
        let s = &src_data[row_off..row_off + width];
        let t = &mut tmp[row_off..row_off + width];
        binomial_h_borders_f32(s, t, width);
        let end = width - 2;
        let mut col = 2;
        while col + 4 <= end {
            // SAFETY: col >= 2 and col + 4 <= width - 2, so reads of
            // s[col-2 ..= col+5] and the write to t[col..col+4] are in bounds.
            let c0 = _mm_loadu_ps(s.as_ptr().add(col - 2));
            let c1 = _mm_loadu_ps(s.as_ptr().add(col - 1));
            let c2 = _mm_loadu_ps(s.as_ptr().add(col));
            let c3 = _mm_loadu_ps(s.as_ptr().add(col + 1));
            let c4 = _mm_loadu_ps(s.as_ptr().add(col + 2));
            let inner = _mm_add_ps(c1, c3);
            let mut r = _mm_add_ps(_mm_mul_ps(six, c2), _mm_mul_ps(four, inner));
            r = _mm_add_ps(r, c0);
            r = _mm_add_ps(r, c4);
            _mm_storeu_ps(t.as_mut_ptr().add(col), r);
            col += 4;
        }
        binomial_h_interior_f32(s, t, col, end);
    }

    let mut dst_data = vec![0f32; width * height];
    binomial_v_borders_f32(&tmp, &mut dst_data, width, height);
    let inv = _mm_set1_ps(INV_256);
    for row in 2..height - 2 {
        let row_off = row * width;
        let mut col = 0;
        while col + 4 <= width {
            // SAFETY: col + 4 <= width and 2 <= row < height-2, so the five
            // row reads and the write to dst[row_off+col..+4] are in bounds.
            let pm2 = _mm_loadu_ps(tmp.as_ptr().add((row - 2) * width + col));
            let pm1 = _mm_loadu_ps(tmp.as_ptr().add((row - 1) * width + col));
            let p = _mm_loadu_ps(tmp.as_ptr().add(row_off + col));
            let pp1 = _mm_loadu_ps(tmp.as_ptr().add((row + 1) * width + col));
            let pp2 = _mm_loadu_ps(tmp.as_ptr().add((row + 2) * width + col));
            let inner = _mm_add_ps(pm1, pp1);
            let mut r = _mm_add_ps(_mm_mul_ps(six, p), _mm_mul_ps(four, inner));
            r = _mm_add_ps(r, pm2);
            r = _mm_add_ps(r, pp2);
            r = _mm_mul_ps(r, inv);
            _mm_storeu_ps(dst_data.as_mut_ptr().add(row_off + col), r);
            col += 4;
        }
        binomial_v_interior_f32(&tmp, &mut dst_data, row, width, col, width);
    }

    Matrix::<f32>::from_vec(height, width, 1, dst_data)
}

/// wasm32 `simd128` f32 → f32 binomial filter (4 lanes/iter). Bit-for-bit
/// identical to [`binomial_4th_order_f32_to_f32_scalar`]; no FMA.
///
/// # Safety
///
/// Requires the `simd128` target feature, guaranteed by the `cfg` gate on
/// the [`binomial_4th_order_f32_to_f32`] dispatcher call site.
#[cfg(all(
    target_arch = "wasm32",
    feature = "simd-wasm32",
    target_feature = "simd128"
))]
#[target_feature(enable = "simd128")]
#[must_use]
pub unsafe fn binomial_4th_order_f32_to_f32_wasm(src: &Matrix<f32>) -> Matrix<f32> {
    use std::arch::wasm32::*;

    let width = src.cols;
    let height = src.rows;
    debug_assert!(width >= 5 && height >= 5);

    let src_data = src.as_slice();
    let mut tmp = vec![0f32; width * height];

    let six = f32x4_splat(6.0);
    let four = f32x4_splat(4.0);

    for row in 0..height {
        let row_off = row * width;
        let s = &src_data[row_off..row_off + width];
        let t = &mut tmp[row_off..row_off + width];
        binomial_h_borders_f32(s, t, width);
        let end = width - 2;
        let mut col = 2;
        while col + 4 <= end {
            // SAFETY: col >= 2 and col + 4 <= width - 2, so reads of
            // s[col-2 ..= col+5] and the write to t[col..col+4] are in bounds.
            let c0 = v128_load(s.as_ptr().add(col - 2) as *const v128);
            let c1 = v128_load(s.as_ptr().add(col - 1) as *const v128);
            let c2 = v128_load(s.as_ptr().add(col) as *const v128);
            let c3 = v128_load(s.as_ptr().add(col + 1) as *const v128);
            let c4 = v128_load(s.as_ptr().add(col + 2) as *const v128);
            let inner = f32x4_add(c1, c3);
            let mut r = f32x4_add(f32x4_mul(six, c2), f32x4_mul(four, inner));
            r = f32x4_add(r, c0);
            r = f32x4_add(r, c4);
            v128_store(t.as_mut_ptr().add(col) as *mut v128, r);
            col += 4;
        }
        binomial_h_interior_f32(s, t, col, end);
    }

    let mut dst_data = vec![0f32; width * height];
    binomial_v_borders_f32(&tmp, &mut dst_data, width, height);
    let inv = f32x4_splat(INV_256);
    for row in 2..height - 2 {
        let row_off = row * width;
        let mut col = 0;
        while col + 4 <= width {
            // SAFETY: col + 4 <= width and 2 <= row < height-2, so the five
            // row reads and the write to dst[row_off+col..+4] are in bounds.
            let pm2 = v128_load(tmp.as_ptr().add((row - 2) * width + col) as *const v128);
            let pm1 = v128_load(tmp.as_ptr().add((row - 1) * width + col) as *const v128);
            let p = v128_load(tmp.as_ptr().add(row_off + col) as *const v128);
            let pp1 = v128_load(tmp.as_ptr().add((row + 1) * width + col) as *const v128);
            let pp2 = v128_load(tmp.as_ptr().add((row + 2) * width + col) as *const v128);
            let inner = f32x4_add(pm1, pp1);
            let mut r = f32x4_add(f32x4_mul(six, p), f32x4_mul(four, inner));
            r = f32x4_add(r, pm2);
            r = f32x4_add(r, pp2);
            r = f32x4_mul(r, inv);
            v128_store(dst_data.as_mut_ptr().add(row_off + col) as *mut v128, r);
            col += 4;
        }
        binomial_v_interior_f32(&tmp, &mut dst_data, row, width, col, width);
    }

    Matrix::<f32>::from_vec(height, width, 1, dst_data)
}

/// 2x2 bilinear downsample. Output dims: `(src.rows >> 1, src.cols >> 1)`.
///
/// Dispatches to the fastest available implementation; currently scalar
/// only (SIMD paths land in #202). See [`downsample_bilinear_f32_scalar`].
#[must_use]
pub fn downsample_bilinear_f32(src: &Matrix<f32>) -> Matrix<f32> {
    downsample_bilinear_f32_scalar(src)
}

/// 2x2 bilinear downsample. Output dims: `(src.rows >> 1, src.cols >> 1)`.
/// Per output pixel: `(p00 + p01 + p10 + p11) * 0.25`. No `ceil` adjustment
/// (unlike the M8-1 box-filter pyramid).
///
/// C equivalent: `vision::downsample_bilinear`.
#[must_use]
pub fn downsample_bilinear_f32_scalar(src: &Matrix<f32>) -> Matrix<f32> {
    let dst_w = src.cols >> 1;
    let dst_h = src.rows >> 1;
    let src_data = src.as_slice();
    let src_w = src.cols;

    let mut dst_data = Vec::<f32>::with_capacity(dst_w * dst_h);
    for row in 0..dst_h {
        let r0 = (row * 2) * src_w;
        let r1 = r0 + src_w;
        for col in 0..dst_w {
            let c = col * 2;
            let sum =
                src_data[r0 + c] + src_data[r0 + c + 1] + src_data[r1 + c] + src_data[r1 + c + 1];
            dst_data.push(sum * 0.25);
        }
    }
    Matrix::<f32>::from_vec(dst_h, dst_w, 1, dst_data)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gradient_u8(rows: usize, cols: usize) -> Matrix<u8> {
        let data: Vec<u8> = (0..rows * cols).map(|i| (i & 0xFF) as u8).collect();
        Matrix::<u8>::from_vec(rows, cols, 1, data)
    }

    // ── SIMD parity (#201) ───────────────────────────────────────────────

    /// Deterministic pseudo-random f32 image (values in `[0, 256)`), seeded
    /// so failures reproduce.
    #[cfg(any(
        all(target_arch = "x86_64", feature = "simd-x86-sse41"),
        all(target_arch = "x86_64", feature = "simd-x86-avx2"),
    ))]
    fn random_f32(rows: usize, cols: usize, seed: u64) -> Matrix<f32> {
        use rand::{Rng, SeedableRng};
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let data: Vec<f32> = (0..rows * cols)
            .map(|_| rng.random::<u8>() as f32)
            .collect();
        Matrix::<f32>::from_vec(rows, cols, 1, data)
    }

    /// Sizes exercising the SIMD interior, the scalar remainder of both the
    /// 8-wide (AVX2) and 4-wide (SSE) loops, tiny images (all-scalar
    /// interior), and odd dimensions. All `>= 5x5` (filter precondition).
    #[cfg(any(
        all(target_arch = "x86_64", feature = "simd-x86-sse41"),
        all(target_arch = "x86_64", feature = "simd-x86-avx2"),
    ))]
    const GAUSS_PARITY_SIZES: &[(usize, usize)] = &[
        (5, 5),
        (5, 7),
        (7, 9),
        (8, 8),
        (9, 11),
        (10, 10),
        (11, 13),
        (16, 16),
        (17, 17),
        (19, 23),
        (32, 32),
        (33, 31),
        (64, 64),
        (65, 63),
        (100, 100),
        (128, 130),
    ];

    /// Assert two f32 matrices are bit-for-bit identical.
    #[cfg(any(
        all(target_arch = "x86_64", feature = "simd-x86-sse41"),
        all(target_arch = "x86_64", feature = "simd-x86-avx2"),
    ))]
    fn assert_bits_eq(a: &Matrix<f32>, b: &Matrix<f32>, what: &str) {
        assert_eq!(a.as_slice().len(), b.as_slice().len(), "{what}: size");
        for (i, (&x, &y)) in a.as_slice().iter().zip(b.as_slice().iter()).enumerate() {
            assert_eq!(x.to_bits(), y.to_bits(), "{what}: bit mismatch at {i}");
        }
    }

    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-avx2"))]
    #[test]
    fn test_binomial_f32_avx2_matches_scalar() {
        if !is_x86_feature_detected!("avx2") {
            eprintln!("avx2 unavailable; skipping");
            return;
        }
        for (i, &(rows, cols)) in GAUSS_PARITY_SIZES.iter().enumerate() {
            let src = random_f32(rows, cols, 0x00B1_0A20 + i as u64);
            let scalar = binomial_4th_order_f32_to_f32_scalar(&src);
            // SAFETY: guarded by the runtime avx2 check above.
            let simd = unsafe { binomial_4th_order_f32_to_f32_avx2(&src) };
            assert_bits_eq(&scalar, &simd, &format!("avx2 {rows}x{cols}"));
        }
    }

    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-sse41"))]
    #[test]
    fn test_binomial_f32_sse41_matches_scalar() {
        if !is_x86_feature_detected!("sse4.1") {
            eprintln!("sse4.1 unavailable; skipping");
            return;
        }
        for (i, &(rows, cols)) in GAUSS_PARITY_SIZES.iter().enumerate() {
            let src = random_f32(rows, cols, 0x0055_B100 + i as u64);
            let scalar = binomial_4th_order_f32_to_f32_scalar(&src);
            // SAFETY: guarded by the runtime sse4.1 check above.
            let simd = unsafe { binomial_4th_order_f32_to_f32_sse41(&src) };
            assert_bits_eq(&scalar, &simd, &format!("sse41 {rows}x{cols}"));
        }
    }

    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-sse41"))]
    #[test]
    fn test_binomial_f32_dispatch_matches_scalar() {
        for (i, &(rows, cols)) in GAUSS_PARITY_SIZES.iter().enumerate() {
            let src = random_f32(rows, cols, 0x0D15_B100 + i as u64);
            let scalar = binomial_4th_order_f32_to_f32_scalar(&src);
            let dispatched = binomial_4th_order_f32_to_f32(&src);
            assert_bits_eq(&scalar, &dispatched, &format!("dispatch {rows}x{cols}"));
        }
    }

    /// Deterministic pseudo-random u8 image, seeded so failures reproduce.
    #[cfg(any(
        all(target_arch = "x86_64", feature = "simd-x86-sse41"),
        all(target_arch = "x86_64", feature = "simd-x86-avx2"),
    ))]
    fn random_u8(rows: usize, cols: usize, seed: u64) -> Matrix<u8> {
        use rand::{Rng, SeedableRng};
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let data: Vec<u8> = (0..rows * cols).map(|_| rng.random::<u8>()).collect();
        Matrix::<u8>::from_vec(rows, cols, 1, data)
    }

    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-avx2"))]
    #[test]
    fn test_binomial_u8_avx2_matches_scalar() {
        if !is_x86_feature_detected!("avx2") {
            eprintln!("avx2 unavailable; skipping");
            return;
        }
        for (i, &(rows, cols)) in GAUSS_PARITY_SIZES.iter().enumerate() {
            let src = random_u8(rows, cols, 0x00C8_0A20 + i as u64);
            let scalar = binomial_4th_order_u8_to_f32_scalar(&src);
            // SAFETY: guarded by the runtime avx2 check above.
            let simd = unsafe { binomial_4th_order_u8_to_f32_avx2(&src) };
            assert_bits_eq(&scalar, &simd, &format!("u8 avx2 {rows}x{cols}"));
        }
    }

    #[cfg(all(target_arch = "x86_64", feature = "simd-x86-sse41"))]
    #[test]
    fn test_binomial_u8_sse41_matches_scalar() {
        if !is_x86_feature_detected!("sse4.1") {
            eprintln!("sse4.1 unavailable; skipping");
            return;
        }
        for (i, &(rows, cols)) in GAUSS_PARITY_SIZES.iter().enumerate() {
            let src = random_u8(rows, cols, 0x0055_C800 + i as u64);
            let scalar = binomial_4th_order_u8_to_f32_scalar(&src);
            // SAFETY: guarded by the runtime sse4.1 check above.
            let simd = unsafe { binomial_4th_order_u8_to_f32_sse41(&src) };
            assert_bits_eq(&scalar, &simd, &format!("u8 sse41 {rows}x{cols}"));
        }
    }

    // ── Configuration sanity ─────────────────────────────────────────────

    #[test]
    fn test_kfactor_for_3_scales() {
        let p = GaussianScaleSpacePyramid::new(1);
        assert!(
            (p.kfactor - 2f32.sqrt()).abs() < 1e-6,
            "expected sqrt(2), got {}",
            p.kfactor
        );
    }

    #[test]
    fn test_gaussian_pyramid_octave_count() {
        let img = gradient_u8(64, 64);
        let mut p = GaussianScaleSpacePyramid::new(3);
        p.build(&img).unwrap();
        assert_eq!(p.octaves.len(), 3);
        for oct in &p.octaves {
            assert_eq!(oct.len(), GaussianScaleSpacePyramid::NUM_SCALES_PER_OCTAVE);
        }
    }

    #[test]
    fn test_gaussian_pyramid_octave_downsamples() {
        let img = gradient_u8(64, 64);
        let mut p = GaussianScaleSpacePyramid::new(3);
        p.build(&img).unwrap();
        assert_eq!(p.level(0, 0).cols, 64);
        assert_eq!(p.level(1, 0).cols, 32);
        assert_eq!(p.level(2, 0).cols, 16);
        assert_eq!(p.level(0, 0).rows, 64);
        assert_eq!(p.level(1, 0).rows, 32);
        assert_eq!(p.level(2, 0).rows, 16);
    }

    #[test]
    fn test_gaussian_pyramid_sigma_increases() {
        let p = GaussianScaleSpacePyramid::new(3);
        // Within an octave: sigma grows by k each scale.
        for oct in 0..3 {
            for s in 0..2 {
                assert!(
                    p.effective_sigma(oct, s + 1) > p.effective_sigma(oct, s),
                    "sigma did not increase within octave {oct} from scale {s} to {}",
                    s + 1
                );
            }
        }
        // Across octaves: sigma doubles each octave at the same scale.
        for s in 0..3 {
            assert!(
                p.effective_sigma(1, s) > p.effective_sigma(0, s),
                "sigma did not increase from octave 0 to 1 at scale {s}"
            );
        }
        // effective_sigma(0, 0) is the implicit base sigma == 1.0.
        assert!((p.effective_sigma(0, 0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_locate_clamps_to_pyramid_bounds() {
        let p = GaussianScaleSpacePyramid::new(2);
        // Very small sigma → (0, 0).
        assert_eq!(p.locate(0.1), (0, 0));
        // Very large sigma → last octave, last scale.
        let (o, s) = p.locate(1000.0);
        assert_eq!(o, 1);
        assert_eq!(s, GaussianScaleSpacePyramid::NUM_SCALES_PER_OCTAVE - 1);
    }

    #[test]
    fn test_gaussian_pyramid_build_is_idempotent() {
        let img1 = gradient_u8(32, 32);
        let img2 = gradient_u8(16, 16);
        let mut p = GaussianScaleSpacePyramid::new(2);
        p.build(&img1).unwrap();
        p.build(&img2).unwrap();
        assert_eq!(p.octaves.len(), 2);
        // Reflects img2, not img1.
        assert_eq!(p.level(0, 0).rows, 16);
        assert_eq!(p.level(0, 0).cols, 16);
    }

    // ── Error variants ───────────────────────────────────────────────────

    #[test]
    fn test_gaussian_pyramid_empty_image_returns_error() {
        let img = Matrix::<u8>::zeros(0, 0, 1);
        let mut p = GaussianScaleSpacePyramid::new(2);
        assert_eq!(p.build(&img), Err(GaussianPyramidError::EmptyImage));
    }

    #[test]
    fn test_gaussian_pyramid_image_too_small_returns_error() {
        let img = gradient_u8(4, 4); // < 5
        let mut p = GaussianScaleSpacePyramid::new(1);
        assert!(matches!(
            p.build(&img),
            Err(GaussianPyramidError::ImageTooSmall { .. })
        ));
    }

    #[test]
    fn test_gaussian_pyramid_zero_octaves_returns_error() {
        let img = gradient_u8(16, 16);
        let mut p = GaussianScaleSpacePyramid::new(0);
        assert_eq!(p.build(&img), Err(GaussianPyramidError::ZeroOctaves));
    }

    #[test]
    fn test_num_octaves_for() {
        // 64x64, min_size 8: 64→32→16→8→4 (stop because 4 < 8) → 4 octaves.
        assert_eq!(num_octaves_for(64, 64, 8), 4);
        // 100x50, min_size 16: limited by height. 50→25→12 (stop) → 2 octaves.
        assert_eq!(num_octaves_for(100, 50, 16), 2);
    }

    // ── Dual-mode: byte-for-byte parity vs C++ ───────────────────────────

    #[cfg(feature = "dual-mode")]
    extern "C" {
        fn webarkit_cpp_binomial_pyramid_build_level(
            src: *const u8,
            src_w: i32,
            src_h: i32,
            num_octaves: i32,
            target_octave: i32,
            target_scale: i32,
            dst_out: *mut f32,
            dst_capacity_floats: i32,
        ) -> i32;
    }

    #[cfg(feature = "dual-mode")]
    fn cpp_build_level(
        src: &[u8],
        src_w: usize,
        src_h: usize,
        num_octaves: usize,
        target_octave: usize,
        target_scale: usize,
    ) -> Vec<f32> {
        let lvl_w = src_w >> target_octave;
        let lvl_h = src_h >> target_octave;
        let mut dst = vec![0f32; lvl_w * lvl_h];
        // SAFETY: src and dst are valid for the declared lengths; the C++
        // shim checks capacity and copies row-by-row.
        let rc = unsafe {
            webarkit_cpp_binomial_pyramid_build_level(
                src.as_ptr(),
                src_w as i32,
                src_h as i32,
                num_octaves as i32,
                target_octave as i32,
                target_scale as i32,
                dst.as_mut_ptr(),
                dst.len() as i32,
            )
        };
        assert_eq!(rc, 0, "C++ shim returned error {rc}");
        dst
    }

    #[test]
    #[cfg(feature = "dual-mode")]
    fn test_gaussian_pyramid_pixels_match_cpp() {
        // Strict byte-for-byte f32 parity. The C++ side is compiled with
        // `-ffp-contract=off` (see build.rs) to prevent Apple clang on ARM64
        // from emitting FMA instructions that would drift from the non-FMA
        // Rust output by multiple ULPs.
        let src_w = 32;
        let src_h = 32;
        let src = gradient_u8(src_h, src_w);
        let num_octaves = 3;

        let mut rust_pyr = GaussianScaleSpacePyramid::new(num_octaves);
        rust_pyr.build(&src).unwrap();

        for oct in 0..num_octaves {
            for scale in 0..GaussianScaleSpacePyramid::NUM_SCALES_PER_OCTAVE {
                let cpp = cpp_build_level(src.as_slice(), src_w, src_h, num_octaves, oct, scale);
                let rust = rust_pyr.level(oct, scale).as_slice();
                assert_eq!(rust.len(), cpp.len(), "size mismatch at ({oct}, {scale})");
                for (i, (&r, &c)) in rust.iter().zip(cpp.iter()).enumerate() {
                    assert_eq!(
                        r.to_bits(),
                        c.to_bits(),
                        "f32 bit mismatch at ({oct}, {scale}) idx {i}: rust={r}, cpp={c}"
                    );
                }
            }
        }
    }
}
