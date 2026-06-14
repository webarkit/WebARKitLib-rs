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
/// Dispatches to the fastest available implementation at runtime,
/// falling back to [`downsample_scalar`]. SIMD paths are added by #132;
/// for now this is a thin wrapper over the scalar implementation, but it
/// is the stable entry point callers (and benchmarks) should target.
///
/// Output dimensions: `new_h = ceil((src.rows - 1) / 2)`,
/// `new_w = ceil((src.cols - 1) / 2)`.
///
/// # Note on visibility
///
/// `downsample` / `downsample_scalar` are `pub` so the criterion
/// benchmark (a separate crate) can measure them directly. They are not
/// part of a stability guarantee — prefer [`Pyramid::build`].
#[must_use]
pub fn downsample(src: &Matrix<u8>) -> Matrix<u8> {
    downsample_scalar(src)
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
    let new_h = (src.rows.saturating_sub(1) as f32 / 2.0).ceil() as usize;
    let new_w = (src.cols.saturating_sub(1) as f32 / 2.0).ceil() as usize;

    let src_data = src.as_slice();
    let src_cols = src.cols;

    let mut dst_data = Vec::<u8>::with_capacity(new_h * new_w);

    for out_row in 0..new_h {
        let r0 = (out_row * 2) * src_cols;
        let r1 = r0 + src_cols;
        let row0 = &src_data[r0..r0 + src_cols];
        let row1 = &src_data[r1..r1 + src_cols];

        for out_col in 0..new_w {
            let c = out_col * 2;
            // u16 promotion prevents u8 wrap; max sum = 4*255 + 2 = 1022.
            let sum: u16 =
                row0[c] as u16 + row0[c + 1] as u16 + row1[c] as u16 + row1[c + 1] as u16 + 2;
            dst_data.push((sum >> 2) as u8);
        }
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
}
