/*
 *  rust_backend.rs
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

//! Pure-Rust [`FreakMatcherBackend`] over [`VisualDatabase`] (M9-2, #141).
//!
//! `RustFreakMatcher` wraps the assembled M6–M8 + M9-1 pipeline behind the
//! same trait as `CppFreakMatcher`, making it a drop-in replacement. When
//! the `dual-mode` feature is enabled, [`DualFreakMatcher`] runs both
//! backends side-by-side and reports any divergence between them — the
//! final verification step before M9-3 flips the default off `ffi-backend`.
//!
//! See `docs/design/m9-2-rust-backend.md` for the design rationale and
//! decision log.

use std::collections::HashMap;

use purecv::core::Matrix;

use crate::kpm::backend::{
    FeaturePoint, FreakMatcherBackend, KpmError, Match, Point3d, QueryResult,
};
use crate::kpm::freak::descriptor::FREAK_DESCRIPTOR_BYTES;
use crate::kpm::freak::detector::DoGScaleInvariantDetector;
use crate::kpm::freak::gaussian_pyramid::{num_octaves_for, GaussianScaleSpacePyramid};
use crate::kpm::freak::hough::{self, find_features};
use crate::kpm::freak::keyframe::Keyframe;
use crate::kpm::freak::visual_database::VisualDatabase;

#[cfg(feature = "dual-mode")]
use crate::arlog_w;
#[cfg(feature = "dual-mode")]
use crate::kpm::freak::homography::multiply_point_homography_inhomogenous;
#[cfg(feature = "dual-mode")]
use crate::kpm::CppFreakMatcher;

// ─────────────────────────────────────────────────────────────────────────
// FeaturePoint bridge (D3)
// ─────────────────────────────────────────────────────────────────────────
//
// The trait surface uses `backend::FeaturePoint`; `Keyframe::store` and the
// rest of the M6–M8 pipeline use `hough::FeaturePoint`. Same fields, different
// types. Cross-module ergonomics via `.into()`.

impl From<&FeaturePoint> for hough::FeaturePoint {
    fn from(p: &FeaturePoint) -> Self {
        Self {
            x: p.x,
            y: p.y,
            angle: p.angle,
            scale: p.scale,
            maxima: p.maxima,
        }
    }
}

impl From<&hough::FeaturePoint> for FeaturePoint {
    fn from(p: &hough::FeaturePoint) -> Self {
        Self {
            x: p.x,
            y: p.y,
            angle: p.angle,
            scale: p.scale,
            maxima: p.maxima,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
// RustFreakMatcher
// ─────────────────────────────────────────────────────────────────────────

/// Pure-Rust feature-matching backend backed by [`VisualDatabase`].
///
/// Implements [`FreakMatcherBackend`] as a drop-in replacement for
/// [`CppFreakMatcher`](crate::kpm::CppFreakMatcher). After M9-3 lands,
/// this becomes the default backend.
///
/// # Constructor parity
///
/// `new(xsize, ysize)` mirrors `CppFreakMatcher::new(xsize, ysize)` to
/// allow drop-in substitution at call sites. The arguments are **not used
/// internally** — `VisualDatabase` auto-allocates its pyramid + detector
/// on the first call. They are accepted for ABI parity only.
///
/// # 3D feature points
///
/// The C++ facade stores per-image 3D points in `mPoint3d[image_id]`. We
/// keep the equivalent side-table on `RustFreakMatcher` rather than on
/// `VisualDatabase` (M9-1 D5: NFT-domain state belongs on the matcher
/// wrapper). Backed by `HashMap<usize, Vec<Point3d>>` with O(1) lookups.
pub struct RustFreakMatcher {
    /// The underlying matching engine.
    db: VisualDatabase,

    /// 3D world-coordinate points per stored reference image, keyed by
    /// the `db_id` used in [`add_freak_features`](
    /// FreakMatcherBackend::add_freak_features). Mirrors the C++ facade's
    /// `mPoint3d` (Group B side-table; deferred from #148 to M9-2).
    points_3d: HashMap<usize, Vec<Point3d>>,

    /// Cached per-query inliers in the trait's [`Match`] type. Rebuilt on
    /// each call to [`query`](FreakMatcherBackend::query). The underlying
    /// `db.inliers()` returns `&[hough::Match]` with identical shape but
    /// a different type — caching the converted form lets the trait's
    /// borrow-returning `inliers()` accessor work.
    cached_inliers: Vec<Match>,

    /// Cached query feature points in the trait's [`FeaturePoint`] type.
    /// Same conversion reason as `cached_inliers`.
    cached_query_points: Vec<FeaturePoint>,
}

impl RustFreakMatcher {
    /// Construct a new pure-Rust FreakMatcher backend.
    ///
    /// `_xsize` and `_ysize` are accepted for ABI parity with
    /// [`CppFreakMatcher::new`](crate::kpm::CppFreakMatcher::new) but are
    /// not used internally — `VisualDatabase` auto-allocates per call.
    pub fn new(_xsize: i32, _ysize: i32) -> Result<Self, KpmError> {
        Ok(Self {
            db: VisualDatabase::new()?,
            points_3d: HashMap::new(),
            cached_inliers: Vec::new(),
            cached_query_points: Vec::new(),
        })
    }

    /// Borrow the 3x3 row-major homography from the most recent
    /// successful query, or `None` if the last query missed.
    ///
    /// Delegates to [`VisualDatabase::matched_geometry`]. Used by
    /// [`DualFreakMatcher`] for the tier-2 corner-reprojection divergence
    /// check (M9-2 #141, D16). Not part of the [`FreakMatcherBackend`]
    /// trait — concrete-impl-only accessor.
    pub fn matched_geometry(&self) -> Option<&[f32; 9]> {
        self.db.matched_geometry()
    }
}

// SAFETY: `VisualDatabase` and its members (BHC trees, KMedoids state,
// HashMap-backed keyframe storage) are all owned data with no shared
// mutability. No `Rc`, `RefCell`, or raw pointers in the field graph.
// The `assert_send` test below enforces this at compile time.

impl FreakMatcherBackend for RustFreakMatcher {
    fn add_image(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
        image_id: usize,
    ) -> Result<(), KpmError> {
        if image.len() < width * height {
            return Err(KpmError::InvalidInput(format!(
                "image too short: got {} bytes, need {}",
                image.len(),
                width * height
            )));
        }
        let mat = Matrix::<u8>::from_vec(height, width, 1, image[..width * height].to_vec());
        self.db.add_image(&mat, image_id)
    }

    fn add_freak_features(
        &mut self,
        points: &[FeaturePoint],
        descriptors: &[u8],
        points_3d: &[Point3d],
        width: usize,
        height: usize,
        db_id: usize,
    ) -> Result<(), KpmError> {
        if points.is_empty() {
            return Ok(());
        }
        if descriptors.len() < points.len() * FREAK_DESCRIPTOR_BYTES {
            return Err(KpmError::InvalidInput(format!(
                "descriptors too short: got {} bytes, need {}",
                descriptors.len(),
                points.len() * FREAK_DESCRIPTOR_BYTES
            )));
        }
        if points_3d.len() < points.len() {
            return Err(KpmError::InvalidInput(format!(
                "points_3d too short: got {}, need {}",
                points_3d.len(),
                points.len()
            )));
        }

        // Build the Keyframe from the supplied features. `add_keyframe`
        // builds the BHC index for us if absent (per M9 #146 D3).
        let mut kf = Keyframe::new(width as i32, height as i32)?;
        for (i, pt) in points.iter().enumerate() {
            let desc = &descriptors[i * FREAK_DESCRIPTOR_BYTES..(i + 1) * FREAK_DESCRIPTOR_BYTES];
            let hough_pt: hough::FeaturePoint = pt.into();
            kf.store.add(hough_pt, desc)?;
        }

        self.db.add_keyframe(kf, db_id)?;
        self.points_3d
            .insert(db_id, points_3d[..points.len()].to_vec());
        Ok(())
    }

    fn query(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
    ) -> Result<QueryResult, KpmError> {
        if image.len() < width * height {
            return Err(KpmError::InvalidInput(format!(
                "image too short: got {} bytes, need {}",
                image.len(),
                width * height
            )));
        }
        let mat = Matrix::<u8>::from_vec(height, width, 1, image[..width * height].to_vec());
        let matched = self.db.query(&mat)?;

        // Rebuild caches in trait types.
        self.cached_inliers = self
            .db
            .inliers()
            .iter()
            .map(|m| Match {
                ins: m.ins,
                ref_: m.ref_,
            })
            .collect();
        self.cached_query_points = self
            .db
            .query_keyframe()
            .map(|kf| {
                (0..kf.store.num_features())
                    .map(|i| kf.store.point(i).into())
                    .collect()
            })
            .unwrap_or_default();

        Ok(QueryResult {
            matched_id: self.db.matched_db_id(),
            inlier_count: if matched {
                self.cached_inliers.len()
            } else {
                0
            },
        })
    }

    fn inliers(&self) -> &[Match] {
        &self.cached_inliers
    }

    fn matched_id(&self) -> i32 {
        self.db.matched_db_id()
    }

    fn query_feature_points(&self) -> &[FeaturePoint] {
        &self.cached_query_points
    }

    fn get_3d_feature_points(&self, image_id: usize) -> &[Point3d] {
        self.points_3d
            .get(&image_id)
            .map(|v| v.as_slice())
            .unwrap_or(&[])
    }

    fn extract_features(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
    ) -> Result<(Vec<FeaturePoint>, Vec<u8>), KpmError> {
        if image.len() < width * height {
            return Err(KpmError::InvalidInput(format!(
                "image too short: got {} bytes, need {}",
                image.len(),
                width * height
            )));
        }
        let mat = Matrix::<u8>::from_vec(height, width, 1, image[..width * height].to_vec());

        // Mirror VisualDatabase's pyramid + detector setup (without
        // storing anything). Same hardcoded defaults as M9-1's
        // `VisualDatabase::new` for parity.
        let n_oct = num_octaves_for(width, height, 8);
        let mut pyr = GaussianScaleSpacePyramid::new(n_oct);
        pyr.build(&mat)
            .map_err(|e| KpmError::InternalError(format!("pyramid build failed: {}", e)))?;
        let det = DoGScaleInvariantDetector::new(3.0, 4.0, 500, true);

        let mut kf = Keyframe::new(width as i32, height as i32)?;
        find_features(&mut kf, &pyr, &det)?;

        let points: Vec<FeaturePoint> = (0..kf.store.num_features())
            .map(|i| kf.store.point(i).into())
            .collect();
        let mut descriptors = Vec::with_capacity(kf.store.num_features() * FREAK_DESCRIPTOR_BYTES);
        for i in 0..kf.store.num_features() {
            descriptors.extend_from_slice(kf.store.descriptor(i));
        }
        Ok((points, descriptors))
    }
}

// ─────────────────────────────────────────────────────────────────────────
// DualFreakMatcher (only when dual-mode is enabled)
// ─────────────────────────────────────────────────────────────────────────

/// Runs both [`CppFreakMatcher`] and [`RustFreakMatcher`] side-by-side on
/// the same inputs, and reports any divergence between them.
///
/// This is the final verification tool before M9-3 flips the default
/// backend off `ffi-backend`. The trait surface delegates `query` to both
/// backends but returns C++ results as ground truth (D5). Non-`query`
/// methods feed identical inputs to both backends so the divergence check
/// remains meaningful per-query; read-back accessors delegate to C++.
///
/// # Divergence check (D4)
///
/// Two-tier check per call to [`query`](FreakMatcherBackend::query):
///
/// 1. **Tier 1 — matched_id mismatch**: cheap and dispositive. If the two
///    backends disagree on which database id matched, that's a clear
///    divergence; log it and skip tier 2 (no shared geometry to compare).
/// 2. **Tier 2 — corner reprojection (M9 #152 pattern)**: only if both
///    backends matched the same id. Project the four reference corners
///    through each backend's homography; if the max corner displacement
///    exceeds `TIER2_TOLERANCE_PX`, log a divergence.
///
/// Divergences are counted on [`Self::divergence_count`] and the most
/// recent reason is available via [`Self::last_divergence_reason`]. Tests
/// assert on the count rather than parsing log output (D6).
#[cfg(feature = "dual-mode")]
pub struct DualFreakMatcher {
    cpp: CppFreakMatcher,
    rust: RustFreakMatcher,
    /// Reference image dimensions captured at `add_image` / `add_freak_features`
    /// time, indexed by image id. Used by the tier-2 corner-reprojection check.
    ref_dims: HashMap<usize, (i32, i32)>,
    /// Number of `query` calls that produced any divergence (tier-1 or tier-2).
    divergence_count: usize,
    /// Human-readable description of the most recent divergence, or `None`
    /// if no divergence has been observed.
    last_divergence_reason: Option<String>,
}

#[cfg(feature = "dual-mode")]
impl DualFreakMatcher {
    /// Tolerance for the tier-2 corner-reprojection divergence check.
    /// Mirrors the M9 #152 milestone gate tolerance.
    const TIER2_TOLERANCE_PX: f32 = 2.0;

    /// Construct a `DualFreakMatcher` driving both backends with the same
    /// expected frame dimensions.
    pub fn new(xsize: i32, ysize: i32) -> Result<Self, KpmError> {
        Ok(Self {
            cpp: CppFreakMatcher::new(xsize, ysize)?,
            rust: RustFreakMatcher::new(xsize, ysize)?,
            ref_dims: HashMap::new(),
            divergence_count: 0,
            last_divergence_reason: None,
        })
    }

    /// Number of `query` calls that observed any divergence between the
    /// two backends. The M9 milestone gate asserts this is zero across
    /// the pinball test sequence.
    pub fn divergence_count(&self) -> usize {
        self.divergence_count
    }

    /// Human-readable description of the most recent divergence, or
    /// `None` if no divergence has been observed.
    pub fn last_divergence_reason(&self) -> Option<&str> {
        self.last_divergence_reason.as_deref()
    }

    /// Homography from the most recent `query` as observed by the C++
    /// backend, or `None` if no query has matched yet. Delegates to
    /// [`CppFreakMatcher::matched_geometry`]. Used by the
    /// `simple_nft_dual` example (#157) to print per-backend geometry.
    pub fn cpp_matched_geometry(&self) -> Option<&[f32; 9]> {
        self.cpp.matched_geometry()
    }

    /// Homography from the most recent `query` as observed by the
    /// pure-Rust backend, or `None` if no query has matched yet.
    /// Delegates to [`RustFreakMatcher::matched_geometry`].
    pub fn rust_matched_geometry(&self) -> Option<&[f32; 9]> {
        self.rust.matched_geometry()
    }

    /// Project the four reference-image corners through `h` (3x3 row-major
    /// homography). Returns the projected points in order: top-left,
    /// top-right, bottom-right, bottom-left. Mirrors M9 #152's
    /// `reproject_corners` test helper.
    fn reproject_corners(h: &[f32; 9], w: i32, h_dim: i32) -> [[f32; 2]; 4] {
        let corners: [[f32; 2]; 4] = [
            [0.0, 0.0],
            [w as f32, 0.0],
            [w as f32, h_dim as f32],
            [0.0, h_dim as f32],
        ];
        let mut out = [[0.0_f32; 2]; 4];
        for (i, c) in corners.iter().enumerate() {
            multiply_point_homography_inhomogenous(&mut out[i], h, c);
        }
        out
    }

    /// Compute the max per-corner displacement between two homographies'
    /// reprojections of the reference corners. Mirrors M9 #152's metric.
    fn max_corner_displacement(cpp_h: &[f32; 9], rust_h: &[f32; 9], ref_w: i32, ref_h: i32) -> f32 {
        let cpp = Self::reproject_corners(cpp_h, ref_w, ref_h);
        let rust = Self::reproject_corners(rust_h, ref_w, ref_h);
        let mut max_disp = 0.0_f32;
        for i in 0..4 {
            let dx = cpp[i][0] - rust[i][0];
            let dy = cpp[i][1] - rust[i][1];
            let d = (dx * dx + dy * dy).sqrt();
            if d > max_disp {
                max_disp = d;
            }
        }
        max_disp
    }
}

#[cfg(feature = "dual-mode")]
impl FreakMatcherBackend for DualFreakMatcher {
    fn add_image(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
        image_id: usize,
    ) -> Result<(), KpmError> {
        self.cpp.add_image(image, width, height, image_id)?;
        self.rust.add_image(image, width, height, image_id)?;
        self.ref_dims
            .insert(image_id, (width as i32, height as i32));
        Ok(())
    }

    fn add_freak_features(
        &mut self,
        points: &[FeaturePoint],
        descriptors: &[u8],
        points_3d: &[Point3d],
        width: usize,
        height: usize,
        db_id: usize,
    ) -> Result<(), KpmError> {
        self.cpp
            .add_freak_features(points, descriptors, points_3d, width, height, db_id)?;
        self.rust
            .add_freak_features(points, descriptors, points_3d, width, height, db_id)?;
        self.ref_dims.insert(db_id, (width as i32, height as i32));
        Ok(())
    }

    fn query(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
    ) -> Result<QueryResult, KpmError> {
        let cpp_result = self.cpp.query(image, width, height)?;
        let rust_result = self.rust.query(image, width, height)?;

        // Tier 1: matched_id mismatch — cheap, dispositive.
        if cpp_result.matched_id != rust_result.matched_id {
            let reason = format!(
                "matched_id mismatch: cpp={} rust={}",
                cpp_result.matched_id, rust_result.matched_id
            );
            arlog_w!("DualMatcher divergence: {}", reason);
            self.divergence_count += 1;
            self.last_divergence_reason = Some(reason);
        } else if cpp_result.matched_id >= 0 {
            // Tier 2: both matched the same id — compare homographies via
            // corner reprojection (M9 #152 pattern).
            let id = cpp_result.matched_id as usize;
            if let (Some(cpp_h), Some(rust_h), Some(&(ref_w, ref_h))) = (
                self.cpp.matched_geometry(),
                self.rust.matched_geometry(),
                self.ref_dims.get(&id),
            ) {
                let max_disp = Self::max_corner_displacement(cpp_h, rust_h, ref_w, ref_h);
                if max_disp > Self::TIER2_TOLERANCE_PX {
                    let reason = format!(
                        "corner reprojection: max_displacement = {:.4} px > {} px tolerance \
                         (both matched id={})",
                        max_disp,
                        Self::TIER2_TOLERANCE_PX,
                        cpp_result.matched_id
                    );
                    arlog_w!("DualMatcher divergence: {}", reason);
                    self.divergence_count += 1;
                    self.last_divergence_reason = Some(reason);
                }
            }
        }

        // C++ result is the ground truth until M9-3 (D5).
        Ok(cpp_result)
    }

    fn inliers(&self) -> &[Match] {
        self.cpp.inliers()
    }

    fn matched_id(&self) -> i32 {
        self.cpp.matched_id()
    }

    fn query_feature_points(&self) -> &[FeaturePoint] {
        self.cpp.query_feature_points()
    }

    fn get_3d_feature_points(&self, image_id: usize) -> &[Point3d] {
        self.cpp.get_3d_feature_points(image_id)
    }

    fn extract_features(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
    ) -> Result<(Vec<FeaturePoint>, Vec<u8>), KpmError> {
        self.cpp.extract_features(image, width, height)
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Load a grayscale image as (raw bytes, width, height).
    fn load_grayscale(path: &str) -> (Vec<u8>, usize, usize) {
        let img = image::open(path).expect("load test image").to_luma8();
        let (w, h) = img.dimensions();
        (img.into_raw(), w as usize, h as usize)
    }

    /// Compile-time `Send` check (A1 from the design doc). The
    /// `FreakMatcherBackend` trait requires `Send`; this test enforces it
    /// for `RustFreakMatcher` at build time.
    #[test]
    fn rust_freak_matcher_is_send() {
        fn assert_send<T: Send>() {}
        assert_send::<RustFreakMatcher>();
    }

    /// Issue #141 required test: add a reference image, query with a
    /// different image, expect a successful match.
    #[test]
    fn test_rust_freak_matcher_implements_backend() {
        let (ref_img, rw, rh) = load_grayscale("../../benchmarks/data/found.jpg");
        let (qry_img, qw, qh) = load_grayscale("../../benchmarks/data/img.jpg");

        let mut matcher = RustFreakMatcher::new(qw as i32, qh as i32).unwrap();
        matcher.add_image(&ref_img, rw, rh, 0).unwrap();
        let result = matcher.query(&qry_img, qw, qh).unwrap();

        assert!(
            result.matched_id >= 0,
            "RustFreakMatcher should match found.jpg on the pinball query"
        );
        assert!(matcher.matched_id() >= 0);
        assert!(!matcher.inliers().is_empty());
        assert!(matcher.matched_geometry().is_some());
        assert!(!matcher.query_feature_points().is_empty());
    }

    /// Covers the `extract_features` path used by
    /// `KpmRefDataSet::generate()`. Detects features without storing.
    #[test]
    fn test_rust_freak_matcher_extract_features() {
        let (img, w, h) = load_grayscale("../../benchmarks/data/found.jpg");
        let mut matcher = RustFreakMatcher::new(w as i32, h as i32).unwrap();
        let (points, descriptors) = matcher.extract_features(&img, w, h).unwrap();
        assert!(!points.is_empty(), "should detect features in found.jpg");
        assert_eq!(
            descriptors.len(),
            points.len() * FREAK_DESCRIPTOR_BYTES,
            "descriptors must be {} bytes per feature",
            FREAK_DESCRIPTOR_BYTES
        );
    }

    // ----------------------------------------------------------------
    // M9 milestone gate (#141) — dual-mode parity on the pinball pair
    // ----------------------------------------------------------------

    /// **M9 milestone gate.** Runs the same query through `DualFreakMatcher`
    /// 3 iterations on the pinball pair and asserts that neither tier-1
    /// (matched_id mismatch) nor tier-2 (corner reprojection > 2.0 px)
    /// divergence appears.
    ///
    /// The tier-2 metric is the M9 #152 corner-reprojection check —
    /// invariant to BHC tree-topology cross-language nondeterminism per
    /// M9 #146 R1.
    ///
    /// If this test passes, M9-3 (#142) can flip the default backend off
    /// `ffi-backend` with confidence.
    #[test]
    #[cfg(feature = "dual-mode")]
    fn test_dual_mode_no_divergence_on_pinball() {
        let (ref_img, rw, rh) = load_grayscale("../../benchmarks/data/found.jpg");
        let (qry_img, qw, qh) = load_grayscale("../../benchmarks/data/img.jpg");

        let mut dual = DualFreakMatcher::new(qw as i32, qh as i32).unwrap();
        dual.add_image(&ref_img, rw, rh, 0).unwrap();

        // 3 iterations of the same query frame (D7).
        for frame in 0..3 {
            let result = dual.query(&qry_img, qw, qh).unwrap();
            assert!(
                result.matched_id >= 0,
                "frame {}: must match (C++ ground truth)",
                frame
            );
        }

        assert_eq!(
            dual.divergence_count(),
            0,
            "expected zero divergences across 3 iterations; last reason: {:?}",
            dual.last_divergence_reason()
        );
    }

    /// Covers the `add_freak_features` path (pre-built features → keyframe
    /// insertion). Extracts features from `found.jpg`, feeds them back as
    /// the database, queries with the same image, expects a self-match.
    #[test]
    fn test_rust_freak_matcher_add_freak_features() {
        let (ref_img, rw, rh) = load_grayscale("../../benchmarks/data/found.jpg");
        let mut matcher = RustFreakMatcher::new(rw as i32, rh as i32).unwrap();

        // Extract from the reference image.
        let (points, descriptors) = matcher.extract_features(&ref_img, rw, rh).unwrap();
        assert!(!points.is_empty());

        // Build matching 3D points (Z = 0 for planar targets, matches NFT
        // convention).
        let points_3d: Vec<Point3d> = points
            .iter()
            .map(|p| Point3d {
                x: p.x,
                y: p.y,
                z: 0.0,
            })
            .collect();

        matcher
            .add_freak_features(&points, &descriptors, &points_3d, rw, rh, 0)
            .unwrap();

        // Verify 3D points were stored.
        assert_eq!(matcher.get_3d_feature_points(0).len(), points.len());
        assert!(
            matcher.get_3d_feature_points(99).is_empty(),
            "missing id returns empty"
        );

        // Self-query should match.
        let result = matcher.query(&ref_img, rw, rh).unwrap();
        assert!(result.matched_id >= 0, "self-query must match");
    }
}
