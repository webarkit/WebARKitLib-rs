/*
 *  interpolate.rs
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

//! Bilinear interpolation utilities and pyramid coordinate mappings.
//!
//! Ported from `WebARKitLib/lib/SRC/KPM/FreakMatcher/detectors/interpolate.h`
//! and the point-mapping helpers in
//! `WebARKitLib/lib/SRC/KPM/FreakMatcher/detectors/gaussian_scale_space_pyramid.h`.
//!
//! The OOB returns `0.0` behavior is a Rust safety improvement over the C++
//! `ASSERT`s; callers that need strict in-bounds checks can pre-validate.

use purecv::core::Matrix;

/// Bilinear interpolation at `(x, y)` on a `Matrix<u8>` level.
///
/// Returns `0.0` if `(x, y)` requires a neighbor pixel outside the image
/// (i.e. `x < 0`, `y < 0`, `x + 1 >= cols`, or `y + 1 >= rows`).
///
/// C equivalent: `vision::bilinear_interpolation<unsigned char, float>`.
#[inline(always)]
pub fn bilinear_interpolate_u8(image: &Matrix<u8>, x: f32, y: f32) -> f32 {
    bilinear_interpolate(image.as_slice(), image.cols, image.rows, x, y)
}

/// Bilinear interpolation at `(x, y)` on a `Matrix<f32>` level.
///
/// Returns `0.0` if `(x, y)` is out of bounds.
///
/// C equivalent: `vision::bilinear_interpolation<float, float>`.
#[inline(always)]
pub fn bilinear_interpolate_f32(image: &Matrix<f32>, x: f32, y: f32) -> f32 {
    let cols = image.cols;
    let rows = image.rows;
    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    if x0 < 0 || y0 < 0 {
        return 0.0;
    }
    let x0u = x0 as usize;
    let y0u = y0 as usize;
    if x0u + 1 >= cols || y0u + 1 >= rows {
        return 0.0;
    }
    let dx = x - x0 as f32;
    let dy = y - y0 as f32;
    let data = image.as_slice();
    let p00 = data[y0u * cols + x0u];
    let p01 = data[y0u * cols + x0u + 1];
    let p10 = data[(y0u + 1) * cols + x0u];
    let p11 = data[(y0u + 1) * cols + x0u + 1];
    (1.0 - dy) * ((1.0 - dx) * p00 + dx * p01) + dy * ((1.0 - dx) * p10 + dx * p11)
}

/// Bilinear interpolation on a raw row-major `&[u8]` buffer.
///
/// Used by FREAK descriptor sampling where the buffer may be a slice of a
/// `Matrix<u8>` level. Returns `0.0` if `(x, y)` is out of bounds.
#[inline(always)]
pub fn bilinear_interpolate(data: &[u8], width: usize, height: usize, x: f32, y: f32) -> f32 {
    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    if x0 < 0 || y0 < 0 {
        return 0.0;
    }
    let x0u = x0 as usize;
    let y0u = y0 as usize;
    if x0u + 1 >= width || y0u + 1 >= height {
        return 0.0;
    }
    let dx = x - x0 as f32;
    let dy = y - y0 as f32;
    let p00 = data[y0u * width + x0u] as f32;
    let p01 = data[y0u * width + x0u + 1] as f32;
    let p10 = data[(y0u + 1) * width + x0u] as f32;
    let p11 = data[(y0u + 1) * width + x0u + 1] as f32;
    (1.0 - dy) * ((1.0 - dx) * p00 + dx * p01) + dy * ((1.0 - dx) * p10 + dx * p11)
}

/// Map an octave-local point back to fine-image coordinates.
///
/// `xp = x * 2^octave + 2^(octave-1) - 0.5`, same for `yp`.
///
/// C equivalent: `vision::bilinear_upsample_point` (3-arg variant).
#[inline(always)]
#[must_use]
pub fn bilinear_upsample_point(x: f32, y: f32, octave: i32) -> (f32, f32) {
    let a = 2f32.powi(octave - 1) - 0.5;
    let b = (1i32 << octave) as f32;
    (x * b + a, y * b + a)
}

/// Map a fine-image point down to octave-local coordinates.
///
/// `xp = x / 2^octave + 1 / (2 * 2^octave) - 0.5`, same for `yp`.
///
/// C equivalent: `vision::bilinear_downsample_point` (3-arg variant).
#[inline(always)]
#[must_use]
pub fn bilinear_downsample_point(x: f32, y: f32, octave: i32) -> (f32, f32) {
    let a = 1.0 / (1i32 << octave) as f32;
    let b = 0.5 * a - 0.5;
    (x * a + b, y * a + b)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gradient_u8(rows: usize, cols: usize) -> Matrix<u8> {
        let data: Vec<u8> = (0..rows * cols).map(|i| (i & 0xFF) as u8).collect();
        Matrix::<u8>::from_vec(rows, cols, 1, data)
    }

    #[test]
    fn test_bilinear_interpolate_at_integer_coords() {
        let img = gradient_u8(8, 8);
        // pixel(y=2, x=3) = 2*8 + 3 = 19.
        let v = bilinear_interpolate_u8(&img, 3.0, 2.0);
        assert!((v - 19.0).abs() < 1e-4, "expected 19.0, got {v}");
    }

    #[test]
    fn test_bilinear_interpolate_out_of_bounds_returns_zero() {
        let img = gradient_u8(4, 4);
        assert_eq!(bilinear_interpolate_u8(&img, -0.1, 1.0), 0.0);
        assert_eq!(bilinear_interpolate_u8(&img, 1.0, -0.1), 0.0);
        // x = 3.0 needs neighbor at col 4 (OOB; cols = 4).
        assert_eq!(bilinear_interpolate_u8(&img, 3.0, 1.0), 0.0);
        assert_eq!(bilinear_interpolate_u8(&img, 1.0, 3.0), 0.0);
    }

    #[test]
    fn test_bilinear_interpolate_midpoint() {
        // 2x2 image: [10, 20; 30, 40]. At (0.5, 0.5) = average = 25.
        let img = Matrix::<u8>::from_vec(2, 2, 1, vec![10, 20, 30, 40]);
        let v = bilinear_interpolate_u8(&img, 0.5, 0.5);
        assert!((v - 25.0).abs() < 1e-4, "expected 25.0, got {v}");
    }

    #[test]
    fn test_bilinear_upsample_downsample_roundtrip() {
        let (xp, yp) = bilinear_upsample_point(10.0, 20.0, 2);
        let (x_back, y_back) = bilinear_downsample_point(xp, yp, 2);
        assert!((x_back - 10.0).abs() < 1e-4);
        assert!((y_back - 20.0).abs() < 1e-4);
    }
}
