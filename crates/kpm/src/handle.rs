/*
 *  handle.rs
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

//! Central KPM handle and configuration enums.
//!
//! This module contains [`KpmHandle`], the main entry point for all KPM
//! tracking operations. It owns a boxed [`FreakMatcherBackend`] and
//! coordinates reference-image management, query execution, and result
//! retrieval.

use crate::backend::{FreakMatcherBackend, KpmError};
use crate::types::{KpmInputDataSet, KpmRefDataSet, KpmResult, DB_IMAGE_MAX};
use std::sync::Arc;
use webarkitlib_rs::types::ARParamLT;

/// Processing resolution mode for KPM detection.
///
/// Controls how much the input camera frame is down-scaled before feature
/// extraction. Smaller sizes are faster but may miss fine details.
///
/// The [`scale_factor`](KpmProcMode::scale_factor) method returns the
/// divisor applied to each image dimension.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KpmProcMode {
    /// Process at the original camera resolution (scale = 1.0).
    FullSize,
    /// Process at 2/3 of the original resolution (scale = 1.5).
    TwoThirdSize,
    /// Process at half the original resolution (scale = 2.0).
    HalfSize,
    /// Process at 1/3 of the original resolution (scale = 3.0).
    OneThirdSize,
    /// Process at 1/4 of the original resolution (scale = 4.0).
    QuarterSize,
}

impl KpmProcMode {
    /// Returns the scale divisor applied to each image dimension.
    ///
    /// For example, [`HalfSize`](KpmProcMode::HalfSize) returns `2.0`,
    /// meaning a 640x480 frame becomes 320x240 internally.
    pub fn scale_factor(&self) -> f32 {
        match self {
            KpmProcMode::FullSize => 1.0,
            KpmProcMode::TwoThirdSize => 1.5,
            KpmProcMode::HalfSize => 2.0,
            KpmProcMode::OneThirdSize => 3.0,
            KpmProcMode::QuarterSize => 4.0,
        }
    }
}

/// Pose estimation mode for KPM matching.
///
/// Determines whether the tracker computes a full 6-DOF camera pose
/// (requiring calibrated camera parameters) or a planar homography.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KpmPoseMode {
    /// Full six-degrees-of-freedom pose estimation.
    /// Requires valid camera parameters ([`ARParamLT`]).
    Pose6DOF,
    /// Planar homography estimation (3x3 matrix).
    /// Does not require camera calibration.
    Homography,
}

/// Central handle for KPM (Key-Point Matching) operations.
///
/// This is the Rust equivalent of the C++ `KpmHandle` from `kpmHandle.cpp`.
/// It owns a [`Box<dyn FreakMatcherBackend>`] and coordinates all tracking
/// operations: adding reference images, running queries against camera
/// frames, and collecting pose results.
///
/// # Architecture
///
/// The handle uses **dynamic dispatch** (`dyn FreakMatcherBackend`) so that
/// the C++ FFI backend ([`CppFreakMatcher`](crate::CppFreakMatcher)) can
/// be transparently swapped for a future pure-Rust implementation without
/// changing any calling code.
///
/// # Camera parameters
///
/// The optional `cparam_lt` field holds camera intrinsics wrapped in
/// [`Arc`](std::sync::Arc). `Arc` (Atomically Reference Counted) is a
/// thread-safe smart pointer that enables multiple owners of the same
/// data. The internal reference count is incremented and decremented
/// atomically, so it is safe to share across threads (unlike `Rc`).
/// Camera parameters are typically created once and shared read-only
/// across the tracking pipeline, making `Arc` an ideal fit — no need to
/// duplicate the data for each handle. When the last `Arc` pointing to
/// the data is dropped, the memory is freed automatically.
///
/// # Example
///
/// ```rust,no_run
/// use webarkitlib_kpm::{CppFreakMatcher, KpmHandle, KpmProcMode};
///
/// let backend = CppFreakMatcher::new(640, 480).unwrap();
/// let mut handle = KpmHandle::new(640, 480, None, Box::new(backend));
///
/// // Optionally change the processing resolution.
/// handle.set_proc_mode(KpmProcMode::HalfSize).unwrap();
/// ```
pub struct KpmHandle {
    /// The feature-matching backend (C++ FFI or pure-Rust).
    pub matcher: Box<dyn FreakMatcherBackend>,
    /// Reference data set containing FREAK descriptors and 3D coordinates.
    pub ref_data_set: KpmRefDataSet,
    /// Input data set holding 2D coordinates extracted from the current frame.
    pub in_data_set: KpmInputDataSet,
    /// Per-page matching results from the most recent query.
    pub result: Vec<KpmResult>,
    /// Shared camera intrinsics and lens-distortion look-up table.
    ///
    /// Wrapped in [`Arc`](std::sync::Arc) (Atomically Reference Counted)
    /// so that multiple handles or pipeline stages can share the same
    /// calibration data without copying. `Arc` is thread-safe: its
    /// reference count is updated atomically, and the inner data is freed
    /// when the last `Arc` clone is dropped.
    pub cparam_lt: Option<Arc<ARParamLT>>,
    /// Current processing resolution mode.
    pub proc_mode: KpmProcMode,
    /// Current pose estimation mode (6-DOF or homography).
    pub pose_mode: KpmPoseMode,
    /// Camera frame width in pixels.
    pub xsize: i32,
    /// Camera frame height in pixels.
    pub ysize: i32,
    /// Maximum number of features to detect per frame (`-1` = unlimited).
    pub detected_max_feature: i32,
    /// Per-page identifier array, indexed by internal page slot.
    pub page_ids: [i32; DB_IMAGE_MAX],
}

impl KpmHandle {
    /// Creates a new [`KpmHandle`].
    ///
    /// # Arguments
    ///
    /// * `xsize`     — camera frame width in pixels.
    /// * `ysize`     — camera frame height in pixels.
    /// * `cparam_lt` — optional shared camera parameters. Pass `None` when
    ///   using [`KpmPoseMode::Homography`].
    /// * `backend`   — a boxed [`FreakMatcherBackend`] implementation
    ///   (e.g. [`CppFreakMatcher`](crate::CppFreakMatcher)).
    ///
    /// # Defaults
    ///
    /// * `proc_mode` = [`KpmProcMode::FullSize`]
    /// * `pose_mode` = [`KpmPoseMode::Pose6DOF`]
    /// * `detected_max_feature` = `-1` (unlimited)
    pub fn new(
        xsize: i32,
        ysize: i32,
        cparam_lt: Option<Arc<ARParamLT>>,
        backend: Box<dyn FreakMatcherBackend>,
    ) -> Self {
        Self {
            matcher: backend,
            ref_data_set: KpmRefDataSet::default(),
            in_data_set: KpmInputDataSet::default(),
            result: Vec::new(),
            cparam_lt,
            proc_mode: KpmProcMode::FullSize,
            pose_mode: KpmPoseMode::Pose6DOF,
            xsize,
            ysize,
            detected_max_feature: -1,
            page_ids: [0i32; DB_IMAGE_MAX],
        }
    }

    /// Returns the camera frame width in pixels.
    pub fn get_xsize(&self) -> i32 {
        self.xsize
    }

    /// Returns the camera frame height in pixels.
    pub fn get_ysize(&self) -> i32 {
        self.ysize
    }

    /// Sets the processing resolution mode.
    ///
    /// This controls how much the input frame is down-scaled before
    /// feature detection. See [`KpmProcMode`] for available options.
    pub fn set_proc_mode(&mut self, mode: KpmProcMode) -> Result<(), KpmError> {
        self.proc_mode = mode;
        Ok(())
    }

    /// Sets the maximum number of features to detect per frame.
    ///
    /// Pass `-1` for unlimited (the default).
    pub fn set_detected_feature_max(&mut self, n: i32) {
        self.detected_max_feature = n;
    }

    /// Marks the given page numbers as "skip" during matching.
    ///
    /// For each entry in `skip_pages`, any [`KpmResult`] whose `page_no`
    /// matches will have its `skip_f` field set to `1`, causing the
    /// matcher to ignore that page on the next query.
    pub fn set_matching_skip_page(&mut self, skip_pages: &[i32]) -> Result<(), KpmError> {
        for result in &mut self.result {
            if skip_pages.contains(&result.page_no) {
                result.skip_f = 1;
            }
        }
        Ok(())
    }

    /// Returns all per-page matching results from the most recent query.
    pub fn get_result(&self) -> &[KpmResult] {
        &self.result
    }

    /// Returns the first valid pose from the results.
    ///
    /// A result is considered valid when its `cam_pose_f` field equals `0`.
    ///
    /// # Returns
    ///
    /// `Some((cam_pose, page_no, error))` for the first valid result, or
    /// `None` if every result has a non-zero `cam_pose_f` (i.e. no valid
    /// pose was estimated).
    pub fn get_pose(&self) -> Option<(&[[f32; 4]; 3], i32, f32)> {
        self.result
            .iter()
            .find(|r| r.cam_pose_f == 0)
            .map(|r| (&r.cam_pose, r.page_no, r.error))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::{FeaturePoint, Match, Point3d, QueryResult};

    // --- Mock backend for testing ---

    struct MockBackend;

    impl FreakMatcherBackend for MockBackend {
        fn add_image(
            &mut self,
            _image: &[u8],
            _width: usize,
            _height: usize,
            _image_id: usize,
        ) -> Result<(), KpmError> {
            todo!()
        }

        fn add_freak_features(
            &mut self,
            _points: &[FeaturePoint],
            _descriptors: &[u8],
            _points_3d: &[Point3d],
            _width: usize,
            _height: usize,
            _db_id: usize,
        ) -> Result<(), KpmError> {
            todo!()
        }

        fn query(
            &mut self,
            _image: &[u8],
            _width: usize,
            _height: usize,
        ) -> Result<QueryResult, KpmError> {
            todo!()
        }

        fn inliers(&self) -> &[Match] {
            todo!()
        }

        fn matched_id(&self) -> i32 {
            todo!()
        }

        fn query_feature_points(&self) -> &[FeaturePoint] {
            todo!()
        }

        fn get_3d_feature_points(&self, _image_id: usize) -> &[Point3d] {
            todo!()
        }
    }

    #[test]
    fn test_kpm_handle_new() {
        let handle = KpmHandle::new(640, 480, None, Box::new(MockBackend));

        assert_eq!(handle.get_xsize(), 640);
        assert_eq!(handle.get_ysize(), 480);
        assert_eq!(handle.proc_mode, KpmProcMode::FullSize);
        assert_eq!(handle.pose_mode, KpmPoseMode::Pose6DOF);
        assert_eq!(handle.detected_max_feature, -1);
        assert!(handle.result.is_empty());
        assert!(handle.cparam_lt.is_none());
    }

    #[test]
    fn test_kpm_proc_mode_scale_factor() {
        assert_eq!(KpmProcMode::FullSize.scale_factor(), 1.0);
        assert_eq!(KpmProcMode::TwoThirdSize.scale_factor(), 1.5);
        assert_eq!(KpmProcMode::HalfSize.scale_factor(), 2.0);
        assert_eq!(KpmProcMode::OneThirdSize.scale_factor(), 3.0);
        assert_eq!(KpmProcMode::QuarterSize.scale_factor(), 4.0);
    }

    #[test]
    fn test_kpm_get_pose_none_when_no_valid_result() {
        let mut handle = KpmHandle::new(640, 480, None, Box::new(MockBackend));

        // Add results with cam_pose_f = -1 (invalid).
        handle.result.push(KpmResult {
            cam_pose_f: -1,
            page_no: 0,
            ..KpmResult::default()
        });
        handle.result.push(KpmResult {
            cam_pose_f: -1,
            page_no: 1,
            ..KpmResult::default()
        });

        assert!(handle.get_pose().is_none());
    }
}
