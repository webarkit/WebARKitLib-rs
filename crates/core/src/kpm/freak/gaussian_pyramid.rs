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

/// 5-tap separable `[1, 4, 6, 4, 1]` binomial filter, u8 source → f32 dest.
///
/// H pass uses `u16` accumulator (max value `16 * 255 = 4080`, fits `u16`).
/// V pass multiplies by `1 / 256` to yield `f32` output. Border replication:
/// edge pixels extend the 2-pixel border on each side.
///
/// C equivalent: `vision::binomial_4th_order(float*, unsigned short*, const unsigned char*, ...)`.
fn binomial_4th_order_u8_to_f32(src: &Matrix<u8>) -> Matrix<f32> {
    let width = src.cols;
    let height = src.rows;
    debug_assert!(width >= 5 && height >= 5);

    let src_data = src.as_slice();
    let mut tmp = vec![0u16; width * height];

    // Horizontal pass.
    for row in 0..height {
        let row_off = row * width;
        let s = &src_data[row_off..row_off + width];
        // col 0: xm2 = xm1 = c = s[0], xp1 = s[1], xp2 = s[2].
        tmp[row_off] =
            6 * s[0] as u16 + 4 * (s[0] as u16 + s[1] as u16) + (s[0] as u16 + s[2] as u16);
        // col 1: xm2 = s[0], xm1 = s[0], c = s[1], xp1 = s[2], xp2 = s[3].
        tmp[row_off + 1] =
            6 * s[1] as u16 + 4 * (s[0] as u16 + s[2] as u16) + (s[0] as u16 + s[3] as u16);
        // Non-border cols.
        for col in 2..width - 2 {
            tmp[row_off + col] = 6 * s[col] as u16
                + 4 * (s[col - 1] as u16 + s[col + 1] as u16)
                + (s[col - 2] as u16 + s[col + 2] as u16);
        }
        // col width-2: xp2 clamped to s[width-1].
        let c = width - 2;
        tmp[row_off + c] = 6 * s[c] as u16
            + 4 * (s[c - 1] as u16 + s[c + 1] as u16)
            + (s[c - 2] as u16 + s[c + 1] as u16);
        // col width-1: xp1 = xp2 = s[width-1].
        let c = width - 1;
        tmp[row_off + c] =
            6 * s[c] as u16 + 4 * (s[c - 1] as u16 + s[c] as u16) + (s[c - 2] as u16 + s[c] as u16);
    }

    // Vertical pass.
    let mut dst_data = vec![0f32; width * height];
    const INV_256: f32 = 1.0 / 256.0;

    // Top border: rows 0 and 1.
    for col in 0..width {
        // row 0: pm2 = pm1 = p = tmp[col]; pp1 = tmp[width+col]; pp2 = tmp[2w+col].
        let p = tmp[col] as u32;
        let pp1 = tmp[width + col] as u32;
        let pp2 = tmp[2 * width + col] as u32;
        dst_data[col] = ((6 * p + 4 * (p + pp1) + p + pp2) as f32) * INV_256;
        // row 1: pm2 = pm1 = tmp[col]; p = tmp[w+col]; pp1 = tmp[2w+col]; pp2 = tmp[3w+col].
        let pm = tmp[col] as u32;
        let p = tmp[width + col] as u32;
        let pp1 = tmp[2 * width + col] as u32;
        let pp2 = tmp[3 * width + col] as u32;
        dst_data[width + col] = ((6 * p + 4 * (pm + pp1) + pm + pp2) as f32) * INV_256;
    }

    // Non-border rows.
    for row in 2..height - 2 {
        let row_off = row * width;
        for col in 0..width {
            let pm2 = tmp[(row - 2) * width + col] as u32;
            let pm1 = tmp[(row - 1) * width + col] as u32;
            let p = tmp[row_off + col] as u32;
            let pp1 = tmp[(row + 1) * width + col] as u32;
            let pp2 = tmp[(row + 2) * width + col] as u32;
            dst_data[row_off + col] = ((6 * p + 4 * (pm1 + pp1) + pm2 + pp2) as f32) * INV_256;
        }
    }

    // Bottom border: rows h-2 and h-1.
    let h = height;
    for col in 0..width {
        // row h-2: pp1 = pp2 = tmp[(h-1)*w + col].
        let pm2 = tmp[(h - 4) * width + col] as u32;
        let pm1 = tmp[(h - 3) * width + col] as u32;
        let p = tmp[(h - 2) * width + col] as u32;
        let pp = tmp[(h - 1) * width + col] as u32;
        dst_data[(h - 2) * width + col] = ((6 * p + 4 * (pm1 + pp) + pm2 + pp) as f32) * INV_256;
        // row h-1: p = pp1 = pp2 = tmp[(h-1)*w + col].
        let pm2 = tmp[(h - 3) * width + col] as u32;
        let pm1 = tmp[(h - 2) * width + col] as u32;
        let p = tmp[(h - 1) * width + col] as u32;
        dst_data[(h - 1) * width + col] = ((6 * p + 4 * (pm1 + p) + pm2 + p) as f32) * INV_256;
    }

    Matrix::<f32>::from_vec(height, width, 1, dst_data)
}

/// 5-tap separable `[1, 4, 6, 4, 1]` binomial filter, f32 → f32. Used for
/// the second and third levels within an octave and for all levels in
/// non-zero octaves.
///
/// C equivalent: `vision::binomial_4th_order(float*, float*, const float*, ...)`.
fn binomial_4th_order_f32_to_f32(src: &Matrix<f32>) -> Matrix<f32> {
    let width = src.cols;
    let height = src.rows;
    debug_assert!(width >= 5 && height >= 5);

    let src_data = src.as_slice();
    let mut tmp = vec![0f32; width * height];

    // Horizontal pass. Addition order matches C++ left-to-right exactly to
    // preserve f32 bit-for-bit parity (no parenthesizing of the (ll + rr) term).
    for row in 0..height {
        let row_off = row * width;
        let s = &src_data[row_off..row_off + width];
        tmp[row_off] = 6.0 * s[0] + 4.0 * (s[0] + s[1]) + s[0] + s[2];
        tmp[row_off + 1] = 6.0 * s[1] + 4.0 * (s[0] + s[2]) + s[0] + s[3];
        for col in 2..width - 2 {
            tmp[row_off + col] =
                6.0 * s[col] + 4.0 * (s[col - 1] + s[col + 1]) + s[col - 2] + s[col + 2];
        }
        let c = width - 2;
        tmp[row_off + c] = 6.0 * s[c] + 4.0 * (s[c - 1] + s[c + 1]) + s[c - 2] + s[c + 1];
        let c = width - 1;
        tmp[row_off + c] = 6.0 * s[c] + 4.0 * (s[c - 1] + s[c]) + s[c - 2] + s[c];
    }

    // Vertical pass.
    let mut dst_data = vec![0f32; width * height];
    const INV_256: f32 = 1.0 / 256.0;

    for col in 0..width {
        // row 0
        let p = tmp[col];
        let pp1 = tmp[width + col];
        let pp2 = tmp[2 * width + col];
        dst_data[col] = (6.0 * p + 4.0 * (p + pp1) + p + pp2) * INV_256;
        // row 1
        let pm = tmp[col];
        let p = tmp[width + col];
        let pp1 = tmp[2 * width + col];
        let pp2 = tmp[3 * width + col];
        dst_data[width + col] = (6.0 * p + 4.0 * (pm + pp1) + pm + pp2) * INV_256;
    }

    for row in 2..height - 2 {
        let row_off = row * width;
        for col in 0..width {
            let pm2 = tmp[(row - 2) * width + col];
            let pm1 = tmp[(row - 1) * width + col];
            let p = tmp[row_off + col];
            let pp1 = tmp[(row + 1) * width + col];
            let pp2 = tmp[(row + 2) * width + col];
            dst_data[row_off + col] = (6.0 * p + 4.0 * (pm1 + pp1) + pm2 + pp2) * INV_256;
        }
    }

    let h = height;
    for col in 0..width {
        let pm2 = tmp[(h - 4) * width + col];
        let pm1 = tmp[(h - 3) * width + col];
        let p = tmp[(h - 2) * width + col];
        let pp = tmp[(h - 1) * width + col];
        dst_data[(h - 2) * width + col] = (6.0 * p + 4.0 * (pm1 + pp) + pm2 + pp) * INV_256;
        let pm2 = tmp[(h - 3) * width + col];
        let pm1 = tmp[(h - 2) * width + col];
        let p = tmp[(h - 1) * width + col];
        dst_data[(h - 1) * width + col] = (6.0 * p + 4.0 * (pm1 + p) + pm2 + p) * INV_256;
    }

    Matrix::<f32>::from_vec(height, width, 1, dst_data)
}

/// 2x2 bilinear downsample. Output dims: `(src.rows >> 1, src.cols >> 1)`.
/// Per output pixel: `(p00 + p01 + p10 + p11) * 0.25`. No `ceil` adjustment
/// (unlike the M8-1 box-filter pyramid).
///
/// C equivalent: `vision::downsample_bilinear`.
fn downsample_bilinear_f32(src: &Matrix<f32>) -> Matrix<f32> {
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
