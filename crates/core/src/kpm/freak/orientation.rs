/*
 *  orientation.rs
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

//! Orientation assignment via gradient histograms.
//!
//! Ported from `WebARKitLib/.../detectors/orientation_assignment.{h,cpp}`
//! and `detectors/gradients.{h,cpp}`. The public two-phase C++ API
//! (`computeGradients` + `compute`) is simplified for M8-3 to a single
//! [`OrientationAssignment::compute`] method taking a precomputed
//! gradient image. The detector batches gradient computation across all
//! pyramid levels via [`compute_polar_gradient_image`].
//!
//! M8-4 (FREAK descriptor) will share the gradient cache with this
//! module and may promote the API to expose the cache directly.

use purecv::core::Matrix;

use std::f32::consts::PI;

const TWO_PI: f32 = 2.0 * PI;
const ONE_OVER_2PI: f32 = 1.0 / TWO_PI;

/// Smoothing kernel (Gaussian sigma=1).
/// Matches the C++ inline kernel in
/// `orientation_assignment.cpp::compute` (smoothing loop) byte-for-byte.
/// The 15-digit literals are taken verbatim from the C++ source with the
/// explicit `_f32` suffix so Rust rounds them directly to the same f32
/// bit pattern as the C++ `float` literals.
const SMOOTH_KERNEL: [f32; 3] = [
    0.274_068_619_061_197_f32,
    0.451_862_761_877_606_f32,
    0.274_068_619_061_197_f32,
];

/// Compute orientations at sub-pixel keypoint locations using a gradient
/// histogram over a Gaussian-weighted circular window around the keypoint.
///
/// C equivalent: `vision::OrientationAssignment`.
///
/// **Defaults**: matches the C++ `DoGScaleInvariantDetector::alloc(...)`
/// call site (lines 126-134 of `DoG_scale_invariant_detector.cpp`):
/// `num_bins = 36`, `gaussian_expansion_factor = 3.0`,
/// `support_region_expansion_factor = 1.5`, `num_smoothing_iterations = 5`,
/// `peak_threshold = 0.8`.
pub struct OrientationAssignment {
    num_bins: usize,
    gaussian_expansion_factor: f32,
    support_region_expansion_factor: f32,
    num_smoothing_iterations: usize,
    peak_threshold: f32,
}

impl Default for OrientationAssignment {
    fn default() -> Self {
        Self::new()
    }
}

impl OrientationAssignment {
    /// Construct with hardcoded FREAK pipeline defaults.
    ///
    /// M8-4 will promote these to constructor parameters.
    #[must_use]
    pub fn new() -> Self {
        Self {
            num_bins: 36,
            gaussian_expansion_factor: 3.0,
            support_region_expansion_factor: 1.5,
            num_smoothing_iterations: 5,
            peak_threshold: 0.8,
        }
    }

    /// Number of histogram bins.
    #[must_use]
    pub fn num_bins(&self) -> usize {
        self.num_bins
    }

    /// Compute dominant orientations (radians, in `[0, 2π)`) for the
    /// keypoint at `(x, y)` with characteristic `sigma`, using the
    /// precomputed `gradient` image (`channels = 2`, interleaved as
    /// `(angle, magnitude)` — matches C++ `ComputePolarGradients`).
    ///
    /// Returns one orientation per histogram peak that is **both** a
    /// strict local maximum **and** above `peak_threshold × max_peak`.
    /// Each peak is sub-pixel-refined via a parabolic fit to its three
    /// neighboring bins (matches C++ `Quadratic3Points`).
    ///
    /// Returns an empty `Vec` if `(x, y)` is outside the image or the
    /// histogram has no positive peak.
    #[must_use]
    pub fn compute(&self, gradient: &Matrix<f32>, x: f32, y: f32, sigma: f32) -> Vec<f32> {
        debug_assert_eq!(
            gradient.channels, 2,
            "gradient image must have 2 channels (angle, magnitude)"
        );

        let width = gradient.cols;
        let height = gradient.rows;
        let data = gradient.as_slice();

        // Reject negative coordinates at the float level. The C++ casts
        // `(int)(x + 0.5f)` rounds -1.0 to 0 (truncation toward zero), so
        // checking only the integer bounds would let small negative values
        // slip through. Real callers (DoG detector) always pass
        // non-negative coordinates; this is a Rust safety guard.
        if x < 0.0 || y < 0.0 {
            return Vec::new();
        }
        let xi = (x + 0.5) as i32;
        let yi = (y + 0.5) as i32;
        if (xi as usize) >= width || (yi as usize) >= height {
            return Vec::new();
        }

        // Gaussian window: σ_window = max(1, gaussian_expansion_factor · sigma).
        let gw_sigma = (self.gaussian_expansion_factor * sigma).max(1.0);
        let gw_scale = -1.0 / (2.0 * gw_sigma * gw_sigma);

        // Circular support radius.
        let radius = self.support_region_expansion_factor * gw_sigma;
        let radius2 = (radius * radius).ceil();
        let radius_int = (radius + 0.5) as i32;

        // Clip the box to image bounds.
        let x0 = (xi - radius_int).max(0) as usize;
        let x1 = (xi + radius_int).min(width as i32 - 1) as usize;
        let y0 = (yi - radius_int).max(0) as usize;
        let y1 = (yi + radius_int).min(height as i32 - 1) as usize;

        // Build the orientation histogram.
        let mut hist = vec![0.0f32; self.num_bins];
        for yp in y0..=y1 {
            let dy = yp as f32 - y;
            let dy2 = dy * dy;
            let row_base = yp * width * 2;
            for xp in x0..=x1 {
                let dx = xp as f32 - x;
                let r2 = dx * dx + dy2;
                if r2 > radius2 {
                    continue;
                }
                let idx = row_base + xp * 2;
                let angle = data[idx];
                let mag = data[idx + 1];

                // Gaussian weight.
                let w = (r2 * gw_scale).exp();

                // Sub-bin location.
                let fbin = self.num_bins as f32 * angle * ONE_OVER_2PI;
                bilinear_histogram_update(&mut hist, fbin, w * mag, self.num_bins);
            }
        }

        // Smooth circularly with the C++ kernel.
        let mut buf = vec![0.0f32; self.num_bins];
        for _ in 0..self.num_smoothing_iterations {
            smooth_orientation_histogram_circular(&mut buf, &hist, &SMOOTH_KERNEL);
            hist.copy_from_slice(&buf);
        }

        // Find peak height.
        let max_height = hist.iter().copied().fold(0.0f32, f32::max);
        if max_height <= 0.0 {
            return Vec::new();
        }

        // Find peaks: strict local max above threshold; sub-pixel-refine
        // via parabolic fit.
        let mut angles: Vec<f32> = Vec::new();
        let threshold = self.peak_threshold * max_height;
        for i in 0..self.num_bins {
            let h0 = hist[i];
            if h0 <= threshold {
                continue;
            }
            let hm1 = hist[(i + self.num_bins - 1) % self.num_bins];
            let hp1 = hist[(i + 1) % self.num_bins];
            if !(h0 > hm1 && h0 > hp1) {
                continue;
            }

            // Parabolic fit through (i-1, hm1), (i, h0), (i+1, hp1).
            // Critical point: x* = i + 0.5 · (hm1 - hp1) / (hm1 - 2·h0 + hp1).
            let denom = hm1 - 2.0 * h0 + hp1;
            let fbin = if denom.abs() > f32::EPSILON {
                i as f32 + 0.5 * (hm1 - hp1) / denom
            } else {
                i as f32
            };

            let nb = self.num_bins as f32;
            let angle = (TWO_PI * (fbin + 0.5 + nb) / nb) % TWO_PI;
            angles.push(angle);
        }

        angles
    }
}

/// Vote `magnitude` into a circular `num_bins` histogram at fractional
/// position `fbin`, splitting between two adjacent bins by linear weight.
///
/// C equivalent: `vision::bilinear_histogram_update` (inline in
/// `orientation_assignment.h`).
#[inline]
fn bilinear_histogram_update(hist: &mut [f32], fbin: f32, magnitude: f32, num_bins: usize) {
    let bin = (fbin - 0.5).floor() as i32;
    let w2 = fbin - bin as f32 - 0.5;
    let w1 = 1.0 - w2;
    let nb = num_bins as i32;
    let b1 = ((bin % nb + nb) % nb) as usize;
    let b2 = (((bin + 1) % nb + nb) % nb) as usize;
    hist[b1] += w1 * magnitude;
    hist[b2] += w2 * magnitude;
}

/// One pass of a 3-tap circular smoothing convolution.
///
/// C equivalent: `vision::SmoothOrientationHistogram`.
#[inline]
fn smooth_orientation_histogram_circular(dst: &mut [f32], src: &[f32], kernel: &[f32; 3]) {
    let n = src.len();
    debug_assert_eq!(dst.len(), n);
    if n < 2 {
        dst.copy_from_slice(src);
        return;
    }
    let first = src[0];
    let mut prev = src[n - 1];
    for i in 0..(n - 1) {
        let cur = src[i];
        dst[i] = kernel[0] * prev + kernel[1] * cur + kernel[2] * src[i + 1];
        prev = cur;
    }
    dst[n - 1] = kernel[0] * prev + kernel[1] * src[n - 1] + kernel[2] * first;
}

/// Build a 2-channel `(angle, magnitude)` polar gradient image from a
/// pyramid level. Channel layout matches C++ `ComputePolarGradients`:
///
/// - Channel 0 = `atan2(dy, dx) + π` (shifted to `[0, 2π]`)
/// - Channel 1 = `sqrt(dx² + dy²)`
///
/// Borders use one-sided differences; interior uses central differences.
///
/// C equivalent: `vision::ComputePolarGradients`.
#[must_use]
pub fn compute_polar_gradient_image(level: &Matrix<f32>) -> Matrix<f32> {
    debug_assert_eq!(level.channels, 1, "input level must be single-channel");
    let w = level.cols;
    let h = level.rows;
    let src = level.as_slice();

    let mut dst = vec![0.0f32; w * h * 2];

    // Helper closures for one-sided / central differences.
    let dx_at = |row: usize, col: usize| -> f32 {
        if col == 0 {
            src[row * w + 1] - src[row * w]
        } else if col == w - 1 {
            src[row * w + col] - src[row * w + col - 1]
        } else {
            src[row * w + col + 1] - src[row * w + col - 1]
        }
    };
    let dy_at = |row: usize, col: usize| -> f32 {
        if row == 0 {
            src[w + col] - src[col]
        } else if row == h - 1 {
            src[(h - 1) * w + col] - src[(h - 2) * w + col]
        } else {
            src[(row + 1) * w + col] - src[(row - 1) * w + col]
        }
    };

    for row in 0..h {
        for col in 0..w {
            let dx = dx_at(row, col);
            let dy = dy_at(row, col);
            let angle = dy.atan2(dx) + PI;
            let mag = (dx * dx + dy * dy).sqrt();
            let idx = (row * w + col) * 2;
            dst[idx] = angle;
            dst[idx + 1] = mag;
        }
    }

    Matrix::<f32>::from_vec(h, w, 2, dst)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a synthetic constant horizontal-gradient image: `pixel = col`.
    /// dx is constant; dy is 0. atan2(0, dx>0) = 0 → angle channel ≈ π
    /// (after the +π shift). That maps to bin `num_bins / 2`; the
    /// dominant orientation in our normalized convention is π/2 +
    /// half-bin offset due to the `(fbin + 0.5)` shift in the final
    /// angle formula.
    fn horizontal_gradient_level(rows: usize, cols: usize) -> Matrix<f32> {
        let data: Vec<f32> = (0..rows * cols).map(|i| (i % cols) as f32).collect();
        Matrix::<f32>::from_vec(rows, cols, 1, data)
    }

    /// Build a synthetic image with a *slightly diagonal* gradient:
    /// `pixel = col + 0.5 * row`. This produces gradient `(dx=2, dy=1)`
    /// in the interior — a single dominant direction that does **not**
    /// straddle two adjacent histogram bins symmetrically.
    ///
    /// A purely horizontal gradient (`pixel = col`) makes `fbin = 18.0`
    /// exactly, splitting votes 50/50 across bins 17 and 18. After
    /// smoothing with a kernel that sums to exactly 1.0, those two bins
    /// remain bit-for-bit equal, and the strict-greater-than peak check
    /// (`h0 > hm1 && h0 > hp1`) fails at both. The diagonal offset
    /// breaks that perfect symmetry and produces a clear single peak.
    fn diagonal_gradient_level(rows: usize, cols: usize) -> Matrix<f32> {
        let data: Vec<f32> = (0..rows * cols)
            .map(|i| {
                let row = i / cols;
                let col = i % cols;
                col as f32 + 0.5 * row as f32
            })
            .collect();
        Matrix::<f32>::from_vec(rows, cols, 1, data)
    }

    fn flat_level(rows: usize, cols: usize, value: f32) -> Matrix<f32> {
        Matrix::<f32>::from_vec(rows, cols, 1, vec![value; rows * cols])
    }

    #[test]
    fn test_orientation_assignment_horizontal_gradient() {
        // Slightly diagonal gradient: dx = 2, dy = 1 in the interior.
        // atan2(dy=1, dx=2) ≈ 0.4636 rad. After the `+π` shift applied
        // by ComputePolarGradients, the angle channel value is
        // ≈ π + 0.4636 ≈ 3.6052 rad.
        //
        // Histogram bin for that angle:
        //   fbin = num_bins · angle / (2π) = 36 · 3.6052 / (2π) ≈ 20.66
        // → bilinear update votes mostly into bin 20 (with some into 21).
        // After smoothing, the peak sits near bin 20-21.
        //
        // Final-angle formula maps the sub-bin peak back to an angle
        // close to the gradient direction (≈ 3.6052 rad), accounting for
        // the half-bin offset in the C++ formula.
        //
        // We use a diagonal rather than a purely horizontal gradient
        // because the latter produces fbin = 18.0 exactly (votes split
        // 50/50 across bins 17 and 18); after smoothing with the kernel
        // that sums to exactly 1.0, those two bins stay bit-for-bit
        // equal and the strict-greater-than peak check fails at both.
        let level = diagonal_gradient_level(64, 64);
        let gradient = compute_polar_gradient_image(&level);
        let oa = OrientationAssignment::new();
        let angles = oa.compute(&gradient, 32.0, 32.0, 1.0);

        assert!(!angles.is_empty(), "expected at least one orientation");

        // Expected angle: roughly the gradient direction, π + atan2(1, 2).
        let expected = PI + 0.5f32.atan2(1.0);
        let best = angles
            .iter()
            .copied()
            .min_by(|a, b| {
                (a - expected)
                    .abs()
                    .partial_cmp(&(b - expected).abs())
                    .unwrap()
            })
            .unwrap();
        assert!(
            (best - expected).abs() < 0.3,
            "expected dominant orientation near {expected}, got {best} (all = {angles:?})"
        );
    }

    #[test]
    fn test_orientation_assignment_empty_when_no_gradients() {
        // Flat image → zero gradients everywhere → empty result.
        let level = flat_level(32, 32, 128.0);
        let gradient = compute_polar_gradient_image(&level);
        let oa = OrientationAssignment::new();
        let angles = oa.compute(&gradient, 16.0, 16.0, 1.0);
        assert!(
            angles.is_empty(),
            "flat image should produce zero orientations, got {angles:?}"
        );
    }

    #[test]
    fn test_orientation_assignment_out_of_bounds_returns_empty() {
        let level = horizontal_gradient_level(16, 16);
        let gradient = compute_polar_gradient_image(&level);
        let oa = OrientationAssignment::new();
        assert!(oa.compute(&gradient, -1.0, 8.0, 1.0).is_empty());
        assert!(oa.compute(&gradient, 8.0, -1.0, 1.0).is_empty());
        assert!(oa.compute(&gradient, 16.0, 8.0, 1.0).is_empty());
        assert!(oa.compute(&gradient, 8.0, 16.0, 1.0).is_empty());
    }

    #[test]
    fn test_polar_gradient_horizontal_image() {
        // pixel = col, so dx is constant. atan2(0, +dx) + π = π
        // for interior cols; magnitude is the difference.
        let level = horizontal_gradient_level(8, 8);
        let g = compute_polar_gradient_image(&level);
        // Pick an interior pixel.
        let row = 4usize;
        let col = 4usize;
        let idx = (row * 8 + col) * 2;
        let data = g.as_slice();
        let angle = data[idx];
        let mag = data[idx + 1];
        // Interior central diff: dx = col+1 - (col-1) = 2, dy = 0.
        // atan2(0, 2) = 0; angle channel = 0 + π = π.
        assert!((angle - PI).abs() < 1e-4, "expected π, got {angle}");
        // magnitude = sqrt(4) = 2.
        assert!((mag - 2.0).abs() < 1e-4, "expected 2.0, got {mag}");
    }

    #[test]
    fn test_polar_gradient_output_shape() {
        let level = horizontal_gradient_level(16, 24);
        let g = compute_polar_gradient_image(&level);
        assert_eq!(g.rows, 16);
        assert_eq!(g.cols, 24);
        assert_eq!(g.channels, 2);
        assert_eq!(g.as_slice().len(), 16 * 24 * 2);
    }
}
