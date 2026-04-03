/*
 *  matching.rs
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

//! Per-frame KPM matching orchestration and pose estimation.
//!
//! This module contains:
//!
//! - [`KpmHandle::kpm_matching`] — the central per-frame tracking function
//!   (ported from `kpmMatching()` in `kpmMatching.cpp`, BINARY_FEATURE=1).
//! - [`kpm_util_get_pose_binary`] — ICP-based pose estimation for binary
//!   (FREAK) feature matches.
//! - [`MatchResult`] — a convenience wrapper around [`QueryResult`](crate::types::QueryResult).

use crate::backend::{FeaturePoint, KpmError, Match, Point3d};
use crate::handle::{KpmHandle, KpmProcMode};
use crate::ref_data_set::resize_image;
use crate::types::{KpmCoord2D, QueryResult};

use webarkitlib_rs::icp::{
    icp_create_handle, icp_delete_handle, icp_get_init_xw2xc_from_planar_data, icp_point,
    ICP2DCoordT, ICP3DCoordT, ICPDataT,
};
use webarkitlib_rs::types::{ARParamLT, ARdouble};

// ---------------------------------------------------------------------------
// KpmHandle::kpm_matching — central per-frame tracking function
// ---------------------------------------------------------------------------

impl KpmHandle {
    /// Run one frame of KPM feature matching and pose estimation.
    ///
    /// This is the Rust equivalent of `kpmMatching()` in the C++ code
    /// (BINARY_FEATURE=1 path). It:
    ///
    /// 1. Optionally resizes the input image according to [`KpmProcMode`].
    /// 2. Queries the backend for feature matches.
    /// 3. Populates `in_data_set` with (optionally undistorted) 2D
    ///    coordinates scaled back to full resolution.
    /// 4. Calls [`kpm_util_get_pose_binary`] for the matched page.
    /// 5. Resets all `skip_f` flags.
    ///
    /// # Arguments
    ///
    /// * `image_luma` — single-channel grayscale frame, `xsize * ysize` bytes.
    pub fn kpm_matching(&mut self, image_luma: &[u8]) -> Result<(), KpmError> {
        let xsize = self.xsize as usize;
        let ysize = self.ysize as usize;

        // (a) Resize image if not FullSize.
        let (image, _xsize2, _ysize2) = if self.proc_mode == KpmProcMode::FullSize {
            (image_luma.to_vec(), xsize, ysize)
        } else {
            resize_image(image_luma, xsize, ysize, self.proc_mode)
        };

        // (b) Query the backend.
        self.matcher
            .query(&image, self.xsize as usize, self.ysize as usize)?;

        // (c) Populate in_data_set.
        let points = self.matcher.query_feature_points().to_vec();
        self.in_data_set.num = points.len() as i32;

        if self.in_data_set.num != 0 {
            // (d) Build input coordinate list.
            let scale = self.proc_mode.scale_factor();
            let mut coords = Vec::with_capacity(points.len());

            for pt in &points {
                let sx = pt.x * scale;
                let sy = pt.y * scale;

                let (ix, iy) = if let Some(ref cparam) = self.cparam_lt {
                    cparam.param_ltf.observ2ideal(sx, sy).unwrap_or((sx, sy))
                } else {
                    (sx, sy)
                };

                coords.push(KpmCoord2D { x: ix, y: iy });
            }
            self.in_data_set.coord = coords;

            // Reset all cam_pose_f = -1.
            for r in &mut self.result {
                r.cam_pose_f = -1;
            }

            // (d-cont) If the backend matched a page, compute pose.
            let matched_id = self.matcher.matched_id();
            if matched_id > 0 {
                let matched_page_no = self.page_ids[matched_id as usize];

                // Find the result slot for this page.
                if let Some(res) = self
                    .result
                    .iter_mut()
                    .find(|r| r.page_no == matched_page_no)
                {
                    if res.skip_f == 0 {
                        let matches = self.matcher.inliers().to_vec();
                        let ref_points_3d = self
                            .matcher
                            .get_3d_feature_points(matched_id as usize)
                            .to_vec();

                        let pose_result = kpm_util_get_pose_binary(
                            self.cparam_lt.as_deref(),
                            &matches,
                            &ref_points_3d,
                            &points,
                        );

                        if let Ok((pose, error)) = pose_result {
                            res.cam_pose = pose;
                            res.cam_pose_f = 0;
                            res.inlier_num = matches.len() as i32;
                            res.error = error;
                            res.page_no = matched_page_no;
                        }
                    }
                }
            }
        } else {
            // (e) No features detected — invalidate all poses.
            for r in &mut self.result {
                r.cam_pose_f = -1;
            }
        }

        // (f) Reset all skip flags.
        for r in &mut self.result {
            r.skip_f = 0;
        }

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// kpm_util_get_pose_binary — ICP pose estimation for FREAK matches
// ---------------------------------------------------------------------------

/// Estimate a camera pose from binary (FREAK) feature matches using ICP.
///
/// This is the Rust equivalent of `kpmUtilGetPose_binary()` in the C++
/// code. It:
///
/// 1. Builds 2D ↔ 3D correspondences from the match indices.
/// 2. Computes an initial pose from the planar data.
/// 3. Refines it with ICP.
/// 4. Rejects poses with reprojection error > 10.0.
///
/// # Arguments
///
/// * `cparam_lt`     — camera parameters (required for `mat` projection matrix).
/// * `matches`       — index pairs linking query features to reference features.
/// * `ref_points_3d` — 3D world coordinates of reference features.
/// * `input_points`  — 2D positions of query features.
///
/// # Returns
///
/// `Ok((cam_pose_3x4, error))` on success, or a [`KpmError`] on failure.
pub fn kpm_util_get_pose_binary(
    cparam_lt: Option<&ARParamLT>,
    matches: &[Match],
    ref_points_3d: &[Point3d],
    input_points: &[FeaturePoint],
) -> Result<([[f32; 4]; 3], f32), KpmError> {
    let num = matches.len();

    if num < 4 {
        return Err(KpmError::NotEnoughPoints { got: num, need: 4 });
    }

    // Build 2D screen coords and 3D world coords from match indices.
    let mut s_coord: Vec<ICP2DCoordT> = Vec::with_capacity(num);
    let mut w_coord: Vec<ICP3DCoordT> = Vec::with_capacity(num);

    for m in matches {
        s_coord.push(ICP2DCoordT {
            x: input_points[m.ins].x as ARdouble,
            y: input_points[m.ins].y as ARdouble,
        });
        w_coord.push(ICP3DCoordT {
            x: ref_points_3d[m.ref_].x as ARdouble,
            y: ref_points_3d[m.ref_].y as ARdouble,
            z: 0.0,
        });
    }

    // Get the camera projection matrix.
    let mat_xc2u = cparam_lt.map(|p| p.param.mat).unwrap_or_else(|| {
        // Identity-like default if no camera params provided.
        let mut m = [[0.0; 4]; 3];
        m[0][0] = 1.0;
        m[1][1] = 1.0;
        m[2][2] = 1.0;
        m
    });

    // Compute initial pose from planar data.
    let mut init_mat_xw2xc = [[0.0 as ARdouble; 4]; 3];
    icp_get_init_xw2xc_from_planar_data(&mat_xc2u, &s_coord, &w_coord, num, &mut init_mat_xw2xc)
        .map_err(|e| {
            KpmError::InternalError(format!("icp_get_init_xw2xc_from_planar_data: {e}"))
        })?;

    // Create ICP handle.
    let mut icp_handle = icp_create_handle(&mat_xc2u)
        .map_err(|e| KpmError::InternalError(format!("icp_create_handle: {e}")))?;

    // Build ICP data.
    let icp_data = ICPDataT {
        screen_coord: s_coord,
        world_coord: w_coord,
    };

    // Run ICP refinement.
    let mut mat_xw2xc = [[0.0 as ARdouble; 4]; 3];
    let icp_result = icp_point(
        unsafe { &*icp_handle },
        &icp_data,
        &init_mat_xw2xc,
        &mut mat_xw2xc,
    );

    // Clean up handle.
    let _ = icp_delete_handle(&mut icp_handle);

    let err = icp_result.map_err(|e| KpmError::InternalError(format!("icp_point: {e}")))?;

    let error = err as f32;
    if error > 10.0 {
        return Err(KpmError::InternalError(format!(
            "pose error too large: {error}"
        )));
    }

    // Cast ARdouble (f64) pose to f32.
    let mut cam_pose = [[0.0f32; 4]; 3];
    for r in 0..3 {
        for c in 0..4 {
            cam_pose[r][c] = mat_xw2xc[r][c] as f32;
        }
    }

    Ok((cam_pose, error))
}

// ---------------------------------------------------------------------------
// MatchResult — convenience wrapper (kept for backwards compatibility)
// ---------------------------------------------------------------------------

/// Outcome of a single matching operation.
///
/// Wraps an `Option<QueryResult>` and exposes helper methods to inspect
/// the match without pattern matching.
///
/// # Examples
///
/// ```rust
/// use webarkitlib_kpm::matching::MatchResult;
///
/// let miss = MatchResult::not_found();
/// assert!(!miss.is_match());
/// assert!(miss.result().is_none());
/// ```
pub struct MatchResult {
    inner: Option<QueryResult>,
}

impl MatchResult {
    /// Creates a successful match result.
    pub fn found(result: QueryResult) -> Self {
        Self {
            inner: Some(result),
        }
    }

    /// Creates a "no match" result.
    pub fn not_found() -> Self {
        Self { inner: None }
    }

    /// Returns `true` if a match was found.
    pub fn is_match(&self) -> bool {
        self.inner.is_some()
    }

    /// Returns a reference to the inner [`QueryResult`], if any.
    pub fn result(&self) -> Option<&QueryResult> {
        self.inner.as_ref()
    }

    /// Consumes `self` and returns the inner [`QueryResult`], if any.
    pub fn into_result(self) -> Option<QueryResult> {
        self.inner
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kpm_util_get_pose_binary_too_few_points() {
        let matches = vec![
            Match { ins: 0, ref_: 0 },
            Match { ins: 1, ref_: 1 },
            Match { ins: 2, ref_: 2 },
        ];
        let ref_3d = vec![
            Point3d {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Point3d {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Point3d {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
        ];
        let input_pts = vec![
            FeaturePoint {
                x: 100.0,
                y: 100.0,
                ..Default::default()
            },
            FeaturePoint {
                x: 200.0,
                y: 100.0,
                ..Default::default()
            },
            FeaturePoint {
                x: 100.0,
                y: 200.0,
                ..Default::default()
            },
        ];

        let result = kpm_util_get_pose_binary(None, &matches, &ref_3d, &input_pts);
        assert!(result.is_err());
        match result.unwrap_err() {
            KpmError::NotEnoughPoints { got, need } => {
                assert_eq!(got, 3);
                assert_eq!(need, 4);
            }
            other => panic!("Expected NotEnoughPoints, got: {other:?}"),
        }
    }

    #[cfg(feature = "ffi-backend")]
    #[test]
    fn test_kpm_matching_does_not_panic() {
        use crate::cpp_backend::CppFreakMatcher;

        // Create a small synthetic grayscale image for the smoke test.
        let width = 640;
        let height = 480;
        let image_luma = vec![128u8; width * height];

        let backend = CppFreakMatcher::new(width as i32, height as i32).unwrap();
        let mut handle = KpmHandle::new(width as i32, height as i32, None, Box::new(backend));

        let result = handle.kpm_matching(&image_luma);
        assert!(result.is_ok());
    }
}
