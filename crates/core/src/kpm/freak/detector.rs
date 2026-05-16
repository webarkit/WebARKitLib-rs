/*
 *  detector.rs
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

//! Scale-invariant keypoint detector based on the Difference-of-Gaussians
//! (DoG) of the Gaussian scale-space pyramid.
//!
//! Ported from `WebARKitLib/.../detectors/DoG_scale_invariant_detector.{h,cpp}`.
//! Uses 3D extrema in the DoG scale-space, Lowe-style sub-pixel refinement
//! via the 3×3 Hessian (with cross-octave variants when extrema straddle
//! octave boundaries), edge-curvature rejection, bucket-based spatial
//! pruning for keypoint diversity, and gradient-histogram orientation
//! assignment.
//!
//! ## Pipeline (matches C++ `DoGScaleInvariantDetector::detect`)
//!
//! 1. Build DoG pyramid: `dog[s] = pyramid[s] - pyramid[s+1]` (less-blurred
//!    minus more-blurred).
//! 2. 3D extrema across the DoG flat-indexed image stack (3 spatial-pattern
//!    variants for cross-octave neighbours).
//! 3. Sub-pixel refinement via 3×3 Hessian solve, with cross-octave
//!    variants.
//! 4. Edge-curvature rejection (`(Dxx+Dyy)² / det`).
//! 5. Bucket-based spatial pruning (best-K per bucket).
//! 6. Orientation assignment via [`OrientationAssignment`] (only if
//!    `find_orientation == true`).

// Private extrema-extraction helpers below take many parameters by design:
// they receive the three laplacian slices, dimension context, and dispatch
// metadata. Grouping into a struct would obscure the parallel structure
// with the three C++ inline expansions of `NONMAX_CHECK`.
#![allow(clippy::too_many_arguments)]
// solve_3x3 implements Gaussian elimination on a 3×3 augmented matrix.
// Index-based iteration mirrors standard textbook formulations and stays
// clearer than iterator-based equivalents for this size.
#![allow(clippy::needless_range_loop)]

use crate::arlog_w;
use purecv::core::Matrix;

use super::gaussian_pyramid::GaussianScaleSpacePyramid;
use super::hough::FeaturePoint;
use super::interpolate::{
    bilinear_downsample_point, bilinear_interpolate_f32, bilinear_upsample_point,
};
use super::orientation::{compute_polar_gradient_image, OrientationAssignment};

/// Number of DoG levels per octave. With 3 Gaussian scales per octave,
/// adjacent pairs produce `3 - 1 = 2` DoG levels.
pub const NUM_DOG_PER_OCTAVE: usize = GaussianScaleSpacePyramid::NUM_SCALES_PER_OCTAVE - 1;

/// A keypoint detected by [`DoGScaleInvariantDetector`].
///
/// This is the detector's working type, carrying refinement diagnostics
/// the FREAK descriptor (M8-4) needs to sample the right pyramid level.
/// For storage in the visual database (Hough voting, FREAK matching), use
/// the [`From<&DoGFeaturePoint>`] projection to [`FeaturePoint`].
///
/// C equivalent: `vision::DoGScaleInvariantDetector::FeaturePoint`.
///
/// Note: `(x, y)` are stored in **fine-image** coordinates (after
/// `bilinear_upsample_point`), matching C++ `mFeaturePoints[i].x/y`.
#[derive(Debug, Clone, Copy)]
pub struct DoGFeaturePoint {
    /// Sub-pixel x in the fine image.
    pub x: f32,
    /// Sub-pixel y in the fine image.
    pub y: f32,
    /// Dominant orientation in radians, in [0, 2π). Zero if
    /// `find_orientation == false` at detection time.
    pub angle: f32,
    /// Octave index in the source pyramid.
    pub octave: i32,
    /// Integer DoG-scale index within the octave (0..NUM_DOG_PER_OCTAVE).
    pub scale: i32,
    /// Sub-pixel scale value: `scale + u[2]` from refinement, clipped to
    /// `[0, NUM_DOG_PER_OCTAVE]`. Matches C++ `kp.sp_scale`.
    pub sp_scale: f32,
    /// Refined DoG response (signed). `score >= 0` ⇒ local maximum,
    /// `score < 0` ⇒ local minimum.
    pub score: f32,
    /// Characteristic Gaussian sigma after refinement:
    /// `effective_sigma(octave, sp_scale)`.
    pub sigma: f32,
    /// Edge-curvature score from `compute_edge_score`. Lower magnitude =
    /// more corner-like; rejected if `|edge_score| ≥ (et + 1)² / et`.
    pub edge_score: f32,
}

impl From<&DoGFeaturePoint> for FeaturePoint {
    fn from(d: &DoGFeaturePoint) -> Self {
        FeaturePoint {
            x: d.x,
            y: d.y,
            angle: d.angle,
            scale: d.sigma, // persistent type holds characteristic radius
            maxima: d.score >= 0.0,
        }
    }
}

/// Scale-invariant keypoint detector.
///
/// C equivalent: `vision::DoGScaleInvariantDetector`.
///
/// **Defaults**: matches the C++ constructor (lines 108-119 of
/// `DoG_scale_invariant_detector.cpp`):
/// - `laplacian_threshold = 0` (no contrast filter by default)
/// - `edge_threshold = 10` (Hessian threshold = `(10+1)²/10 = 12.1`)
/// - `max_subpixel_distance_sqr = 9` (`3*3`; reject if `‖δ‖² > 9`)
/// - `num_buckets_x = num_buckets_y = 10`
/// - `find_orientation = true`
/// - `max_num_feature_points = 5000`
pub struct DoGScaleInvariantDetector {
    laplacian_threshold: f32,
    edge_threshold: f32,
    max_subpixel_distance_sqr: f32,
    num_buckets_x: usize,
    num_buckets_y: usize,
    max_num_feature_points: usize,
    find_orientation: bool,
    orientation_assignment: OrientationAssignment,
}

impl DoGScaleInvariantDetector {
    /// Maximum keypoint cap; matches C++ `kMaxNumFeaturePoints = 5000`.
    pub const DEFAULT_MAX_NUM_FEATURE_POINTS: usize = 5000;

    /// Construct with config. Other parameters take FREAK defaults.
    #[must_use]
    pub fn new(
        laplacian_threshold: f32,
        edge_threshold: f32,
        max_num_feature_points: usize,
        find_orientation: bool,
    ) -> Self {
        debug_assert!(edge_threshold > 0.0, "edge_threshold must be positive");
        Self {
            laplacian_threshold,
            edge_threshold,
            max_subpixel_distance_sqr: 9.0, // C++ 3*3
            num_buckets_x: 10,
            num_buckets_y: 10,
            max_num_feature_points,
            find_orientation,
            orientation_assignment: OrientationAssignment::new(),
        }
    }

    /// Detect scale-invariant feature points in the Gaussian pyramid.
    #[must_use]
    pub fn detect(&self, pyramid: &GaussianScaleSpacePyramid) -> Vec<DoGFeaturePoint> {
        if pyramid.num_octaves == 0 {
            arlog_w!("DoGScaleInvariantDetector::detect: pyramid has zero octaves");
            return Vec::new();
        }

        // (a) Build the DoG pyramid (flat-indexed).
        let dog = build_dog_pyramid_flat(pyramid);
        if dog.len() < 3 {
            // Need at least one interior DoG index for 3D extrema.
            return Vec::new();
        }

        // (b) Extract minima/maxima with cross-octave dimension dispatch.
        let mut points = extract_features(&dog, pyramid, self.laplacian_threshold);

        // (c, d, e) Sub-pixel refinement + edge rejection.
        refine_subpixel_locations(
            &mut points,
            &dog,
            pyramid,
            self.laplacian_threshold,
            self.edge_threshold,
            self.max_subpixel_distance_sqr,
        );

        // (f) Bucket pruning (BEFORE orientation, matches C++).
        let fine_w = pyramid.level(0, 0).cols;
        let fine_h = pyramid.level(0, 0).rows;
        let points = prune_features_bucketed(
            points,
            fine_w,
            fine_h,
            self.num_buckets_x,
            self.num_buckets_y,
            self.max_num_feature_points,
        );

        // (g) Orientation assignment.
        if self.find_orientation {
            assign_orientations(points, pyramid, &self.orientation_assignment)
        } else {
            points
        }
    }
}

// ──────────────────────────────────────────────────────────────────────
// DoG pyramid construction
// ──────────────────────────────────────────────────────────────────────

/// Build the flat-indexed DoG pyramid.
///
/// `dog[octave * NUM_DOG_PER_OCTAVE + s]` for `s` in
/// `0..NUM_DOG_PER_OCTAVE`. Each entry is a `Matrix<f32>` with the same
/// dimensions as the source octave.
///
/// **DoG direction**: `dog[s] = pyramid[s] - pyramid[s+1]` (less-blurred
/// minus more-blurred), matching C++ `difference_image_binomial`.
fn build_dog_pyramid_flat(pyramid: &GaussianScaleSpacePyramid) -> Vec<Matrix<f32>> {
    let mut dog = Vec::with_capacity(pyramid.num_octaves * NUM_DOG_PER_OCTAVE);
    for oct in 0..pyramid.num_octaves {
        for s in 0..NUM_DOG_PER_OCTAVE {
            dog.push(dog_image(pyramid.level(oct, s), pyramid.level(oct, s + 1)));
        }
    }
    dog
}

/// Per-pixel difference image: `dst = a - b`.
fn dog_image(a: &Matrix<f32>, b: &Matrix<f32>) -> Matrix<f32> {
    debug_assert!(a.rows == b.rows && a.cols == b.cols);
    let data: Vec<f32> = a
        .as_slice()
        .iter()
        .zip(b.as_slice().iter())
        .map(|(av, bv)| av - bv)
        .collect();
    Matrix::<f32>::from_vec(a.rows, a.cols, 1, data)
}

// ──────────────────────────────────────────────────────────────────────
// Extrema extraction
// ──────────────────────────────────────────────────────────────────────

fn extract_features(
    dog: &[Matrix<f32>],
    pyramid: &GaussianScaleSpacePyramid,
    laplacian_threshold: f32,
) -> Vec<DoGFeaturePoint> {
    let lap_sqr_threshold = laplacian_threshold * laplacian_threshold;
    let mut out: Vec<DoGFeaturePoint> = Vec::new();

    for i in 1..(dog.len() - 1) {
        let im0 = &dog[i - 1];
        let im1 = &dog[i];
        let im2 = &dog[i + 1];

        let octave = (i / NUM_DOG_PER_OCTAVE) as i32;
        let scale = (i % NUM_DOG_PER_OCTAVE) as i32;

        let same = im0.cols == im1.cols
            && im0.cols == im2.cols
            && im0.rows == im1.rows
            && im0.rows == im2.rows;
        let fine_octave_pair = im0.cols == im1.cols
            && (im1.cols >> 1) == im2.cols
            && im0.rows == im1.rows
            && (im1.rows >> 1) == im2.rows;
        let coarse_octave_pair = (im0.cols >> 1) == im1.cols
            && im1.cols == im2.cols
            && (im0.rows >> 1) == im1.rows
            && im1.rows == im2.rows;

        if same {
            extract_features_same_octave(
                &mut out,
                im0,
                im1,
                im2,
                octave,
                scale,
                pyramid,
                lap_sqr_threshold,
            );
        } else if fine_octave_pair {
            extract_features_fine_octave_pair(
                &mut out,
                im0,
                im1,
                im2,
                octave,
                scale,
                pyramid,
                lap_sqr_threshold,
            );
        } else if coarse_octave_pair {
            extract_features_coarse_octave_pair(
                &mut out,
                im0,
                im1,
                im2,
                octave,
                scale,
                pyramid,
                lap_sqr_threshold,
            );
        } else {
            arlog_w!("extract_features: inconsistent DoG dimensions at index {i}; skipping");
        }
    }

    out
}

#[inline]
fn nonmax_same_octave(
    op: NonMaxOp,
    val: f32,
    im0: &[f32],
    im1: &[f32],
    im2: &[f32],
    w: usize,
    row: usize,
    col: usize,
) -> bool {
    let i0 = (row - 1) * w + col - 1;
    let i1 = (row - 1) * w + col;
    let i2 = (row - 1) * w + col + 1;
    let j0 = row * w + col - 1;
    let j1 = row * w + col;
    let j2 = row * w + col + 1;
    let k0 = (row + 1) * w + col - 1;
    let k1 = (row + 1) * w + col;
    let k2 = (row + 1) * w + col + 1;
    let cmp = |a: f32, b: f32| match op {
        NonMaxOp::Greater => a > b,
        NonMaxOp::Less => a < b,
    };
    // im0 (9)
    cmp(val, im0[i0]) && cmp(val, im0[i1]) && cmp(val, im0[i2])
        && cmp(val, im0[j0]) && cmp(val, im0[j1]) && cmp(val, im0[j2])
        && cmp(val, im0[k0]) && cmp(val, im0[k1]) && cmp(val, im0[k2])
        // im1 (8, skip center)
        && cmp(val, im1[i0]) && cmp(val, im1[i1]) && cmp(val, im1[i2])
        && cmp(val, im1[j0]) && cmp(val, im1[j2])
        && cmp(val, im1[k0]) && cmp(val, im1[k1]) && cmp(val, im1[k2])
        // im2 (9)
        && cmp(val, im2[i0]) && cmp(val, im2[i1]) && cmp(val, im2[i2])
        && cmp(val, im2[j0]) && cmp(val, im2[j1]) && cmp(val, im2[j2])
        && cmp(val, im2[k0]) && cmp(val, im2[k1]) && cmp(val, im2[k2])
}

#[derive(Copy, Clone)]
enum NonMaxOp {
    Greater,
    Less,
}

fn extract_features_same_octave(
    out: &mut Vec<DoGFeaturePoint>,
    im0: &Matrix<f32>,
    im1: &Matrix<f32>,
    im2: &Matrix<f32>,
    octave: i32,
    scale: i32,
    pyramid: &GaussianScaleSpacePyramid,
    lap_sqr_threshold: f32,
) {
    let w = im1.cols;
    let h = im1.rows;
    if w < 3 || h < 3 {
        return;
    }
    let d0 = im0.as_slice();
    let d1 = im1.as_slice();
    let d2 = im2.as_slice();
    for row in 1..(h - 1) {
        for col in 1..(w - 1) {
            let value = d1[row * w + col];
            if value * value < lap_sqr_threshold {
                continue;
            }
            let extrema = nonmax_same_octave(NonMaxOp::Greater, value, d0, d1, d2, w, row, col)
                || nonmax_same_octave(NonMaxOp::Less, value, d0, d1, d2, w, row, col);
            if extrema {
                push_extremum(out, pyramid, octave, scale, col as f32, row as f32, value);
            }
        }
    }
}

fn extract_features_fine_octave_pair(
    out: &mut Vec<DoGFeaturePoint>,
    im0: &Matrix<f32>,
    im1: &Matrix<f32>,
    im2: &Matrix<f32>,
    octave: i32,
    scale: i32,
    pyramid: &GaussianScaleSpacePyramid,
    lap_sqr_threshold: f32,
) {
    // im0/im1 same size as octave; im2 half size (next coarser octave).
    let w = im1.cols;
    let h = im1.rows;
    if w < 3 || h < 3 {
        return;
    }
    let d0 = im0.as_slice();
    let d1 = im1.as_slice();
    // C++ loop bounds: end_x = floor(((im2.width-1) - 0.5) * 2 + 0.5), same for y.
    // Conservative: stop at w-2 / h-2 (interior).
    let end_x = (((im2.cols as f32 - 1.0) - 0.5) * 2.0 + 0.5).floor() as usize;
    let end_y = (((im2.rows as f32 - 1.0) - 0.5) * 2.0 + 0.5).floor() as usize;
    let end_x = end_x.min(w - 1);
    let end_y = end_y.min(h - 1);

    for row in 2..end_y {
        for col in 2..end_x {
            let value = d1[row * w + col];
            if value * value < lap_sqr_threshold {
                continue;
            }

            let ds_x = col as f32 * 0.5 - 0.25;
            let ds_y = row as f32 * 0.5 - 0.25;

            let extrema = nonmax_fine_octave(
                NonMaxOp::Greater,
                value,
                d0,
                d1,
                im2,
                w,
                row,
                col,
                ds_x,
                ds_y,
            ) || nonmax_fine_octave(
                NonMaxOp::Less,
                value,
                d0,
                d1,
                im2,
                w,
                row,
                col,
                ds_x,
                ds_y,
            );
            if extrema {
                push_extremum(out, pyramid, octave, scale, col as f32, row as f32, value);
            }
        }
    }
}

#[inline]
fn nonmax_fine_octave(
    op: NonMaxOp,
    val: f32,
    im0: &[f32],
    im1: &[f32],
    im2: &Matrix<f32>,
    w: usize,
    row: usize,
    col: usize,
    ds_x: f32,
    ds_y: f32,
) -> bool {
    let cmp = |a: f32, b: f32| match op {
        NonMaxOp::Greater => a > b,
        NonMaxOp::Less => a < b,
    };
    let i_off = (row - 1) * w + col;
    let j_off = row * w + col;
    let k_off = (row + 1) * w + col;
    // im0 (9)
    cmp(val, im0[i_off - 1]) && cmp(val, im0[i_off]) && cmp(val, im0[i_off + 1])
        && cmp(val, im0[j_off - 1]) && cmp(val, im0[j_off]) && cmp(val, im0[j_off + 1])
        && cmp(val, im0[k_off - 1]) && cmp(val, im0[k_off]) && cmp(val, im0[k_off + 1])
        // im1 (8)
        && cmp(val, im1[i_off - 1]) && cmp(val, im1[i_off]) && cmp(val, im1[i_off + 1])
        && cmp(val, im1[j_off - 1]) && cmp(val, im1[j_off + 1])
        && cmp(val, im1[k_off - 1]) && cmp(val, im1[k_off]) && cmp(val, im1[k_off + 1])
        // im2 (9, bilinear)
        && cmp(val, bilinear_interpolate_f32(im2, ds_x - 0.5, ds_y - 0.5))
        && cmp(val, bilinear_interpolate_f32(im2, ds_x, ds_y - 0.5))
        && cmp(val, bilinear_interpolate_f32(im2, ds_x + 0.5, ds_y - 0.5))
        && cmp(val, bilinear_interpolate_f32(im2, ds_x - 0.5, ds_y))
        && cmp(val, bilinear_interpolate_f32(im2, ds_x, ds_y))
        && cmp(val, bilinear_interpolate_f32(im2, ds_x + 0.5, ds_y))
        && cmp(val, bilinear_interpolate_f32(im2, ds_x - 0.5, ds_y + 0.5))
        && cmp(val, bilinear_interpolate_f32(im2, ds_x, ds_y + 0.5))
        && cmp(val, bilinear_interpolate_f32(im2, ds_x + 0.5, ds_y + 0.5))
}

fn extract_features_coarse_octave_pair(
    out: &mut Vec<DoGFeaturePoint>,
    im0: &Matrix<f32>,
    im1: &Matrix<f32>,
    im2: &Matrix<f32>,
    octave: i32,
    scale: i32,
    pyramid: &GaussianScaleSpacePyramid,
    lap_sqr_threshold: f32,
) {
    // im0 double size (previous finer octave); im1/im2 at this octave.
    let w = im1.cols;
    let h = im1.rows;
    if w < 3 || h < 3 {
        return;
    }
    let d1 = im1.as_slice();
    let d2 = im2.as_slice();

    for row in 1..(h - 1) {
        for col in 1..(w - 1) {
            let value = d1[row * w + col];
            if value * value < lap_sqr_threshold {
                continue;
            }

            let us_x = (col << 1) as f32 + 0.5;
            let us_y = (row << 1) as f32 + 0.5;

            let extrema = nonmax_coarse_octave(
                NonMaxOp::Greater,
                value,
                im0,
                d1,
                d2,
                w,
                row,
                col,
                us_x,
                us_y,
            ) || nonmax_coarse_octave(
                NonMaxOp::Less,
                value,
                im0,
                d1,
                d2,
                w,
                row,
                col,
                us_x,
                us_y,
            );
            if extrema {
                push_extremum(out, pyramid, octave, scale, col as f32, row as f32, value);
            }
        }
    }
}

#[inline]
fn nonmax_coarse_octave(
    op: NonMaxOp,
    val: f32,
    im0: &Matrix<f32>,
    im1: &[f32],
    im2: &[f32],
    w: usize,
    row: usize,
    col: usize,
    us_x: f32,
    us_y: f32,
) -> bool {
    let cmp = |a: f32, b: f32| match op {
        NonMaxOp::Greater => a > b,
        NonMaxOp::Less => a < b,
    };
    let i_off = (row - 1) * w + col;
    let j_off = row * w + col;
    let k_off = (row + 1) * w + col;
    // im1 (8)
    cmp(val, im1[i_off - 1]) && cmp(val, im1[i_off]) && cmp(val, im1[i_off + 1])
        && cmp(val, im1[j_off - 1]) && cmp(val, im1[j_off + 1])
        && cmp(val, im1[k_off - 1]) && cmp(val, im1[k_off]) && cmp(val, im1[k_off + 1])
        // im2 (9)
        && cmp(val, im2[i_off - 1]) && cmp(val, im2[i_off]) && cmp(val, im2[i_off + 1])
        && cmp(val, im2[j_off - 1]) && cmp(val, im2[j_off]) && cmp(val, im2[j_off + 1])
        && cmp(val, im2[k_off - 1]) && cmp(val, im2[k_off]) && cmp(val, im2[k_off + 1])
        // im0 (9, bilinear at double-scale)
        && cmp(val, bilinear_interpolate_f32(im0, us_x - 2.0, us_y - 2.0))
        && cmp(val, bilinear_interpolate_f32(im0, us_x, us_y - 2.0))
        && cmp(val, bilinear_interpolate_f32(im0, us_x + 2.0, us_y - 2.0))
        && cmp(val, bilinear_interpolate_f32(im0, us_x - 2.0, us_y))
        && cmp(val, bilinear_interpolate_f32(im0, us_x, us_y))
        && cmp(val, bilinear_interpolate_f32(im0, us_x + 2.0, us_y))
        && cmp(val, bilinear_interpolate_f32(im0, us_x - 2.0, us_y + 2.0))
        && cmp(val, bilinear_interpolate_f32(im0, us_x, us_y + 2.0))
        && cmp(val, bilinear_interpolate_f32(im0, us_x + 2.0, us_y + 2.0))
}

#[inline]
fn push_extremum(
    out: &mut Vec<DoGFeaturePoint>,
    pyramid: &GaussianScaleSpacePyramid,
    octave: i32,
    scale: i32,
    col: f32,
    row: f32,
    value: f32,
) {
    let (fx, fy) = bilinear_upsample_point(col, row, octave);
    out.push(DoGFeaturePoint {
        x: fx,
        y: fy,
        angle: 0.0,
        octave,
        scale,
        sp_scale: scale as f32,
        score: value,
        sigma: pyramid.effective_sigma(octave as usize, scale as usize),
        edge_score: 0.0,
    });
}

// ──────────────────────────────────────────────────────────────────────
// Sub-pixel refinement
// ──────────────────────────────────────────────────────────────────────

fn refine_subpixel_locations(
    points: &mut Vec<DoGFeaturePoint>,
    dog: &[Matrix<f32>],
    pyramid: &GaussianScaleSpacePyramid,
    laplacian_threshold: f32,
    edge_threshold: f32,
    max_subpixel_distance_sqr: f32,
) {
    let lap_sqr = laplacian_threshold * laplacian_threshold;
    let hessian_threshold = (edge_threshold + 1.0).powi(2) / edge_threshold;
    let fine_w = pyramid.level(0, 0).cols as f32;
    let fine_h = pyramid.level(0, 0).rows as f32;

    let mut write = 0usize;
    for i in 0..points.len() {
        let kp = points[i];
        let lap_index = (kp.octave as usize) * NUM_DOG_PER_OCTAVE + (kp.scale as usize);

        if lap_index == 0 || lap_index + 1 >= dog.len() {
            continue; // need lap0 and lap2
        }

        // Downsample fine-image (x, y) to octave-local coords.
        let (xp, yp) = bilinear_downsample_point(kp.x, kp.y, kp.octave);
        let x = (xp + 0.5) as i32;
        let y = (yp + 0.5) as i32;

        let lap0 = &dog[lap_index - 1];
        let lap1 = &dog[lap_index];
        let lap2 = &dog[lap_index + 1];

        if x < 1 || (x + 1) as usize >= lap1.cols || y < 1 || (y + 1) as usize >= lap1.rows {
            continue;
        }

        let Some((a, b)) = compute_subpixel_hessian(lap0, lap1, lap2, x, y) else {
            continue;
        };

        let Some(u) = solve_3x3(&a, &b) else {
            continue;
        };

        if u[0] * u[0] + u[1] * u[1] > max_subpixel_distance_sqr {
            continue;
        }

        let Some(edge_score) = compute_edge_score(&a) else {
            continue;
        };

        // Refined DoG response: linear correction using the gradient.
        let center_val = lap1.as_slice()[(y as usize) * lap1.cols + (x as usize)];
        let refined_score = center_val - (b[0] * u[0] + b[1] * u[1] + b[2] * u[2]);

        // Sub-pixel scale (full value, clipped to [0, num_dog]).
        let mut sp_scale = kp.scale as f32 + u[2];
        sp_scale = sp_scale.clamp(0.0, NUM_DOG_PER_OCTAVE as f32);

        // Upsample refined location to fine-image coords.
        let (fx, fy) = bilinear_upsample_point(xp + u[0], yp + u[1], kp.octave);

        if edge_score.abs() < hessian_threshold
            && refined_score * refined_score >= lap_sqr
            && fx >= 0.0
            && fx < fine_w
            && fy >= 0.0
            && fy < fine_h
        {
            let sigma = pyramid_effective_sigma_fractional(pyramid, kp.octave, sp_scale);
            points[write] = DoGFeaturePoint {
                x: fx,
                y: fy,
                angle: 0.0,
                octave: kp.octave,
                scale: kp.scale,
                sp_scale,
                score: refined_score,
                sigma,
                edge_score,
            };
            write += 1;
        }
    }
    points.truncate(write);
}

/// Linearly interpolate `effective_sigma` along sub-pixel scale.
fn pyramid_effective_sigma_fractional(
    pyramid: &GaussianScaleSpacePyramid,
    octave: i32,
    sp_scale: f32,
) -> f32 {
    // sigma(octave, scale) = k^scale · 2^octave.
    pyramid.kfactor.powf(sp_scale) * (1u32 << (octave as usize)) as f32
}

/// Dispatch to one of three Hessian variants based on dimension pattern.
fn compute_subpixel_hessian(
    lap0: &Matrix<f32>,
    lap1: &Matrix<f32>,
    lap2: &Matrix<f32>,
    x: i32,
    y: i32,
) -> Option<([f32; 9], [f32; 3])> {
    if lap0.cols == lap1.cols
        && lap1.cols == lap2.cols
        && lap0.rows == lap1.rows
        && lap1.rows == lap2.rows
    {
        Some(hessian_same_octave(
            lap0, lap1, lap2, x as usize, y as usize,
        ))
    } else if lap0.cols == lap1.cols
        && (lap1.cols >> 1) == lap2.cols
        && lap0.rows == lap1.rows
        && (lap1.rows >> 1) == lap2.rows
    {
        Some(hessian_fine_octave_pair(
            lap0, lap1, lap2, x as usize, y as usize,
        ))
    } else if (lap0.cols >> 1) == lap1.cols
        && lap1.cols == lap2.cols
        && (lap0.rows >> 1) == lap1.rows
        && lap1.rows == lap2.rows
    {
        Some(hessian_coarse_octave_pair(
            lap0, lap1, lap2, x as usize, y as usize,
        ))
    } else {
        None
    }
}

/// Spatial derivatives at (x, y) in `im` (interior pixel only).
#[inline]
fn spatial_derivatives(im: &Matrix<f32>, x: usize, y: usize) -> (f32, f32, f32, f32, f32) {
    let w = im.cols;
    let d = im.as_slice();
    let p = d[y * w + x];
    let p_left = d[y * w + x - 1];
    let p_right = d[y * w + x + 1];
    let p_up = d[(y - 1) * w + x];
    let p_dn = d[(y + 1) * w + x];
    let p_ul = d[(y - 1) * w + x - 1];
    let p_ur = d[(y - 1) * w + x + 1];
    let p_dl = d[(y + 1) * w + x - 1];
    let p_dr = d[(y + 1) * w + x + 1];

    let dx = 0.5 * (p_right - p_left);
    let dy = 0.5 * (p_dn - p_up);
    let dxx = p_left - 2.0 * p + p_right;
    let dyy = p_up - 2.0 * p + p_dn;
    let dxy = 0.25 * ((p_ul + p_dr) - (p_ur + p_dl));
    (dx, dy, dxx, dyy, dxy)
}

fn hessian_same_octave(
    lap0: &Matrix<f32>,
    lap1: &Matrix<f32>,
    lap2: &Matrix<f32>,
    x: usize,
    y: usize,
) -> ([f32; 9], [f32; 3]) {
    let (dx, dy, dxx, dyy, dxy) = spatial_derivatives(lap1, x, y);
    let w = lap1.cols;
    let l0 = lap0.as_slice();
    let l2 = lap2.as_slice();
    let l1c = lap1.as_slice()[y * w + x];
    let l0c = l0[y * w + x];
    let l2c = l2[y * w + x];
    let ds = 0.5 * (l2c - l0c);
    let dss = l0c + (-2.0 * l1c) + l2c;
    let dxs =
        0.25 * ((l0[y * w + x - 1] - l0[y * w + x + 1]) + (-l2[y * w + x - 1] + l2[y * w + x + 1]));
    let dys = 0.25
        * ((l0[(y - 1) * w + x] - l0[(y + 1) * w + x])
            + (-l2[(y - 1) * w + x] + l2[(y + 1) * w + x]));
    let h = [dxx, dxy, dxs, dxy, dyy, dys, dxs, dys, dss];
    let b = [-dx, -dy, -ds];
    (h, b)
}

fn hessian_fine_octave_pair(
    lap0: &Matrix<f32>,
    lap1: &Matrix<f32>,
    lap2: &Matrix<f32>,
    x: usize,
    y: usize,
) -> ([f32; 9], [f32; 3]) {
    let (dx, dy, dxx, dyy, dxy) = spatial_derivatives(lap1, x, y);
    let w = lap1.cols;
    let l1c = lap1.as_slice()[y * w + x];

    // lap2 is half-sized (next coarser octave).
    let (x_div2, y_div2) = bilinear_downsample_point(x as f32, y as f32, 1);
    let val = bilinear_interpolate_f32(lap2, x_div2, y_div2);

    let l0 = lap0.as_slice();

    // C++ uses lap2 (coarser octave) at (x_div2 ± 0.5, y_div2 ± 0.5).
    let lap2_px_pos = bilinear_interpolate_f32(lap2, x_div2 + 0.5, y_div2);
    let lap2_px_neg = bilinear_interpolate_f32(lap2, x_div2 - 0.5, y_div2);
    let lap2_py_pos = bilinear_interpolate_f32(lap2, x_div2, y_div2 + 0.5);
    let lap2_py_neg = bilinear_interpolate_f32(lap2, x_div2, y_div2 - 0.5);

    let ds = 0.5 * (val - l0[y * w + x]);
    let dss = l0[y * w + x] + (-2.0 * l1c) + val;
    let dxs = 0.25 * ((l0[y * w + x - 1] + lap2_px_pos) - (l0[y * w + x + 1] + lap2_px_neg));
    let dys = 0.25 * ((l0[(y - 1) * w + x] + lap2_py_pos) - (l0[(y + 1) * w + x] + lap2_py_neg));
    let h = [dxx, dxy, dxs, dxy, dyy, dys, dxs, dys, dss];
    let b = [-dx, -dy, -ds];
    (h, b)
}

fn hessian_coarse_octave_pair(
    lap0: &Matrix<f32>,
    lap1: &Matrix<f32>,
    lap2: &Matrix<f32>,
    x: usize,
    y: usize,
) -> ([f32; 9], [f32; 3]) {
    let (dx, dy, dxx, dyy, dxy) = spatial_derivatives(lap1, x, y);
    let w = lap1.cols;
    let l1c = lap1.as_slice()[y * w + x];

    // lap0 is double-sized (previous finer octave).
    let (x_mul2, y_mul2) = bilinear_upsample_point(x as f32, y as f32, 1);
    let val = bilinear_interpolate_f32(lap0, x_mul2, y_mul2);

    let l2 = lap2.as_slice();

    let lap0_px_pos = bilinear_interpolate_f32(lap0, x_mul2 - 2.0, y_mul2);
    let lap0_px_neg = bilinear_interpolate_f32(lap0, x_mul2 + 2.0, y_mul2);
    let lap0_py_pos = bilinear_interpolate_f32(lap0, x_mul2, y_mul2 - 2.0);
    let lap0_py_neg = bilinear_interpolate_f32(lap0, x_mul2, y_mul2 + 2.0);

    let ds = 0.5 * (l2[y * w + x] - val);
    let dss = val + (-2.0 * l1c) + l2[y * w + x];
    let dxs = 0.25 * ((lap0_px_pos + l2[y * w + x + 1]) - (lap0_px_neg + l2[y * w + x - 1]));
    let dys = 0.25 * ((lap0_py_pos + l2[(y + 1) * w + x]) - (lap0_py_neg + l2[(y - 1) * w + x]));
    let h = [dxx, dxy, dxs, dxy, dyy, dys, dxs, dys, dss];
    let b = [-dx, -dy, -ds];
    (h, b)
}

/// Edge-curvature score. Returns `None` if the 2×2 determinant is zero.
fn compute_edge_score(h: &[f32; 9]) -> Option<f32> {
    let dxx = h[0];
    let dxy = h[1];
    let dyy = h[4];
    let det = dxx * dyy - dxy * dxy;
    if det == 0.0 {
        return None;
    }
    Some((dxx + dyy).powi(2) / det)
}

/// Solve a 3×3 linear system via Gaussian elimination with partial
/// pivoting. Returns `None` on near-singular matrices.
fn solve_3x3(a: &[f32; 9], b: &[f32; 3]) -> Option<[f32; 3]> {
    let mut m = [
        [a[0], a[1], a[2], b[0]],
        [a[3], a[4], a[5], b[1]],
        [a[6], a[7], a[8], b[2]],
    ];
    // Forward elimination with pivoting.
    for k in 0..3 {
        // Find pivot.
        let mut max_row = k;
        let mut max_val = m[k][k].abs();
        for i in (k + 1)..3 {
            let v = m[i][k].abs();
            if v > max_val {
                max_val = v;
                max_row = i;
            }
        }
        if max_val < 1e-12 {
            return None;
        }
        if max_row != k {
            m.swap(k, max_row);
        }
        for i in (k + 1)..3 {
            let factor = m[i][k] / m[k][k];
            for j in k..4 {
                m[i][j] -= factor * m[k][j];
            }
        }
    }
    // Back substitution.
    let mut x = [0.0f32; 3];
    for i in (0..3).rev() {
        let mut sum = m[i][3];
        for j in (i + 1)..3 {
            sum -= m[i][j] * x[j];
        }
        x[i] = sum / m[i][i];
    }
    Some(x)
}

// ──────────────────────────────────────────────────────────────────────
// Bucket pruning
// ──────────────────────────────────────────────────────────────────────

/// Distribute keypoints across an `nx × ny` spatial grid (on the
/// fine-image coordinates) and keep the top-K per bucket where
/// `K = max_total / (nx * ny)`.
///
/// C equivalent: `vision::PruneDoGFeatures` (uses `std::nth_element`
/// per bucket).
fn prune_features_bucketed(
    points: Vec<DoGFeaturePoint>,
    fine_w: usize,
    fine_h: usize,
    nx: usize,
    ny: usize,
    max_total: usize,
) -> Vec<DoGFeaturePoint> {
    if points.len() <= max_total {
        return points;
    }
    let num_buckets = nx * ny;
    if num_buckets == 0 {
        return points;
    }
    let num_per_bucket = max_total / num_buckets;
    if num_per_bucket == 0 {
        return points;
    }
    let dx = (fine_w as f32 / nx as f32).ceil().max(1.0) as usize;
    let dy = (fine_h as f32 / ny as f32).ceil().max(1.0) as usize;

    let mut buckets: Vec<Vec<(f32, usize)>> = vec![Vec::new(); num_buckets];
    for (i, p) in points.iter().enumerate() {
        let bx = ((p.x as usize) / dx).min(nx - 1);
        let by = ((p.y as usize) / dy).min(ny - 1);
        buckets[bx * ny + by].push((p.score.abs(), i));
    }

    let mut out = Vec::with_capacity(max_total);
    for mut bucket in buckets {
        let bucket_len = bucket.len();
        let n = bucket_len.min(num_per_bucket);
        if n == 0 {
            continue;
        }
        // Partial sort descending by |score|. select_nth_unstable_by
        // partitions so that bucket[0..pivot] are all >= bucket[pivot..],
        // where pivot = n - 1. After the call, the top-n entries occupy
        // positions 0..n (in unspecified internal order).
        let pivot = (n - 1).min(bucket_len - 1);
        bucket.select_nth_unstable_by(pivot, |a, b| {
            b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal)
        });
        for k in 0..n {
            out.push(points[bucket[k].1]);
        }
    }
    out
}

// ──────────────────────────────────────────────────────────────────────
// Orientation assignment
// ──────────────────────────────────────────────────────────────────────

fn assign_orientations(
    refined: Vec<DoGFeaturePoint>,
    pyramid: &GaussianScaleSpacePyramid,
    oa: &OrientationAssignment,
) -> Vec<DoGFeaturePoint> {
    let num_scales = GaussianScaleSpacePyramid::NUM_SCALES_PER_OCTAVE;
    // Precompute gradient images for every (octave, scale).
    let mut gradients: Vec<Matrix<f32>> = Vec::with_capacity(pyramid.num_octaves * num_scales);
    for oct in 0..pyramid.num_octaves {
        for scale in 0..num_scales {
            gradients.push(compute_polar_gradient_image(pyramid.level(oct, scale)));
        }
    }

    let mut out = Vec::with_capacity(refined.len());
    for kp in &refined {
        // Downsample fine-image (x, y, sigma) to octave-local for OA.
        let inv = 1.0 / (1u32 << (kp.octave as usize)) as f32;
        let off = 0.5 * inv - 0.5;
        let mut x_oct = kp.x * inv + off;
        let mut y_oct = kp.y * inv + off;
        let sigma_oct = kp.sigma * inv;

        // Clip to octave bounds.
        let g_idx = (kp.octave as usize) * num_scales + (kp.scale as usize);
        if g_idx >= gradients.len() {
            continue;
        }
        let g = &gradients[g_idx];
        x_oct = x_oct.clamp(0.0, g.cols as f32 - 1.0);
        y_oct = y_oct.clamp(0.0, g.rows as f32 - 1.0);

        let angles = oa.compute(g, x_oct, y_oct, sigma_oct);
        for angle in angles {
            let mut copy = *kp;
            copy.angle = angle;
            out.push(copy);
        }
    }
    out
}

// ══════════════════════════════════════════════════════════════════════
// Tests
// ══════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    fn load_grayscale(path: &str) -> Matrix<u8> {
        let img = image::open(path).expect("load test image").to_luma8();
        let (w, h) = img.dimensions();
        Matrix::<u8>::from_vec(h as usize, w as usize, 1, img.into_raw())
    }

    fn build_test_pyramid(img: &Matrix<u8>, num_octaves: usize) -> GaussianScaleSpacePyramid {
        let mut p = GaussianScaleSpacePyramid::new(num_octaves);
        p.build(img).expect("build gaussian pyramid");
        p
    }

    #[test]
    fn test_dog_feature_point_from_conversion() {
        let d = DoGFeaturePoint {
            x: 12.5,
            y: 34.5,
            angle: 1.57,
            octave: 1,
            scale: 0,
            sp_scale: 0.25,
            score: -0.04,
            sigma: 2.1,
            edge_score: 5.0,
        };
        let fp: FeaturePoint = (&d).into();
        assert!((fp.x - 12.5).abs() < 1e-6);
        assert!((fp.y - 34.5).abs() < 1e-6);
        assert!((fp.angle - 1.57).abs() < 1e-6);
        assert!((fp.scale - 2.1).abs() < 1e-6);
        // score < 0 => minima.
        assert!(!fp.maxima);

        let d2 = DoGFeaturePoint { score: 0.5, ..d };
        let fp2: FeaturePoint = (&d2).into();
        assert!(fp2.maxima);
    }

    #[test]
    fn test_dog_detector_zero_octave_pyramid_is_handled() {
        // Smallest valid pyramid: 1 octave, smallest valid input.
        // GaussianScaleSpacePyramid requires >= 5x5 input.
        let img = Matrix::<u8>::from_vec(8, 8, 1, vec![100u8; 64]);
        let mut p = GaussianScaleSpacePyramid::new(1);
        p.build(&img).expect("build");
        let det = DoGScaleInvariantDetector::new(0.0, 10.0, 5000, false);
        let pts = det.detect(&p);
        // With only 1 octave we have 2 DoG levels (indices 0,1). The
        // interior loop `1..size-1` is empty for size=2, so no extrema.
        assert!(pts.is_empty(), "expected zero keypoints, got {}", pts.len());
    }

    #[test]
    fn test_dog_detector_finds_keypoints_on_real_image() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let pyr = build_test_pyramid(&img, 3);
        let det = DoGScaleInvariantDetector::new(0.0, 10.0, 5000, false);
        let pts = det.detect(&pyr);
        assert!(pts.len() > 50, "expected > 50 keypoints, got {}", pts.len());
    }

    #[test]
    fn test_dog_detector_keypoints_within_image_bounds() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let pyr = build_test_pyramid(&img, 3);
        let det = DoGScaleInvariantDetector::new(0.0, 10.0, 5000, false);
        let pts = det.detect(&pyr);
        let w = img.cols as f32;
        let h = img.rows as f32;
        for p in &pts {
            let fp: FeaturePoint = p.into();
            assert!(
                fp.x >= 0.0 && fp.x < w && fp.y >= 0.0 && fp.y < h,
                "keypoint ({}, {}) outside [0, {})x[0, {})",
                fp.x,
                fp.y,
                w,
                h
            );
        }
    }

    // ── Dual-mode: keypoint count agreement with C++ ──────────────────

    #[cfg(feature = "dual-mode")]
    extern "C" {
        fn webarkit_cpp_dog_detect_count(
            src: *const u8,
            src_w: i32,
            src_h: i32,
            num_octaves: i32,
            laplacian_threshold: f32,
            edge_threshold: f32,
            max_num_feature_points: i32,
            find_orientation: i32,
            count_out: *mut i32,
        ) -> i32;
    }

    #[cfg(feature = "dual-mode")]
    fn cpp_detect_count(
        img: &Matrix<u8>,
        num_octaves: usize,
        laplacian_threshold: f32,
        edge_threshold: f32,
        max_num_feature_points: usize,
        find_orientation: bool,
    ) -> i32 {
        let mut count: i32 = 0;
        // SAFETY: the shim validates pointers and dimensions; img owns
        // the source buffer; count_out points to a stack-local i32.
        let rc = unsafe {
            webarkit_cpp_dog_detect_count(
                img.as_slice().as_ptr(),
                img.cols as i32,
                img.rows as i32,
                num_octaves as i32,
                laplacian_threshold,
                edge_threshold,
                max_num_feature_points as i32,
                if find_orientation { 1 } else { 0 },
                &mut count,
            )
        };
        assert_eq!(rc, 0, "C++ shim returned error {rc}");
        count
    }

    #[test]
    #[cfg(feature = "dual-mode")]
    fn test_dog_keypoints_match_cpp_count() {
        // Tolerance covers sort tie-breaking and bucket ordering variance
        // only. Any algorithm-level error would dwarf this.
        const MAX_TIE_DIVERGENCE: i32 = 5;

        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let num_octaves = 3;
        let laplacian_threshold = 0.0;
        let edge_threshold = 10.0;
        let max_pts = 5000;
        let find_orientation = false;

        let pyr = build_test_pyramid(&img, num_octaves);
        let det = DoGScaleInvariantDetector::new(
            laplacian_threshold,
            edge_threshold,
            max_pts,
            find_orientation,
        );
        let rust_count = det.detect(&pyr).len() as i32;

        let cpp_count = cpp_detect_count(
            &img,
            num_octaves,
            laplacian_threshold,
            edge_threshold,
            max_pts,
            find_orientation,
        );

        let diff = (rust_count - cpp_count).abs();
        assert!(
            diff <= MAX_TIE_DIVERGENCE,
            "keypoint count divergence: rust={rust_count}, cpp={cpp_count}, |diff|={diff} > {MAX_TIE_DIVERGENCE}"
        );
    }
}
