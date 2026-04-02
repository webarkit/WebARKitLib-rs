/*
 *  types.rs
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

//! KPM data types ported from the C headers `kpmType.h` and `kpm.h`.
//!
//! These are pure Rust types (no `#[repr(C)]`) used throughout the crate
//! to represent reference data, query results, and geometric primitives.
//! FFI communication goes through the opaque C API (`kpm_c_api.h`)
//! instead.

// ---------------------------------------------------------------------------
// Constants ported from kpmType.h and kpm.h
// ---------------------------------------------------------------------------

/// Size (in bytes) of a single FREAK descriptor.
///
/// Each descriptor is a 96-byte binary string used for fast
/// Hamming-distance matching.
pub const FREAK_SUB_DIMENSION: usize = 96;

/// Maximum number of reference images the database can hold.
pub const DB_IMAGE_MAX: usize = 1024;

/// Maximum number of corner points detected in a single frame.
pub const MAX_CORNER_POINTS: usize = 2000;

// ---------------------------------------------------------------------------
// Types ported from kpmType.h
// ---------------------------------------------------------------------------

/// 2D coordinate used for feature-point positions and projections.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmCoord2D {
    /// Horizontal position (pixels).
    pub x: f32,
    /// Vertical position (pixels).
    pub y: f32,
}

/// Metadata for a single image within a reference page.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmImageInfo {
    /// Unique image number within the page.
    pub image_no: i32,
    /// Image width in pixels.
    pub width: i32,
    /// Image height in pixels.
    pub height: i32,
}

/// Metadata for a reference page (a tracked planar target).
///
/// A page can contain multiple images at different resolutions to
/// improve tracking robustness across scales.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmPageInfo {
    /// Unique page number.
    pub page_no: i32,
    /// Number of images associated with this page.
    pub image_num: i32,
    /// Per-image metadata.
    pub image_info: Vec<KpmImageInfo>,
}

/// A single FREAK (Fast Retina Keypoint) feature descriptor.
///
/// The descriptor is a 96-byte binary string ([`FREAK_SUB_DIMENSION`])
/// designed for fast Hamming-distance comparison, together with the
/// keypoint's orientation, scale, and maxima flag.
#[derive(Debug, Clone, PartialEq)]
pub struct FreakFeature {
    /// 96-byte binary descriptor vector.
    pub v: [u8; FREAK_SUB_DIMENSION],
    /// Keypoint orientation in radians.
    pub angle: f32,
    /// Keypoint scale (pyramid level).
    pub scale: f32,
    /// Whether this keypoint is a local maximum (`1`) or not (`0`).
    pub maxima: i32,
}

impl Default for FreakFeature {
    fn default() -> Self {
        Self {
            v: [0u8; FREAK_SUB_DIMENSION],
            angle: 0.0,
            scale: 0.0,
            maxima: 0,
        }
    }
}

/// A reference data point linking a 2D image position, a 3D world
/// position, and a FREAK descriptor.
///
/// These are stored in [`KpmRefDataSet`] and used during matching to
/// establish 2D-3D correspondences for pose estimation.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmRefData {
    /// 2D position of the feature in the reference image (pixels).
    pub coord2d: KpmCoord2D,
    /// 3D world coordinate computed from the 2D position and DPI
    /// (millimetres, centred on the image).
    pub coord3d: KpmCoord2D,
    /// FREAK descriptor for this feature.
    pub feature_vec: FreakFeature,
    /// Page number this feature belongs to.
    pub page_no: i32,
    /// Reference-image number within the page.
    pub ref_image_no: i32,
}

impl KpmRefData {
    /// Returns the raw descriptor bytes for this feature point.
    pub fn feature_vec_as_bytes(&self) -> &[u8] {
        &self.feature_vec.v
    }
}

/// Complete set of reference data for all registered pages.
///
/// Populated by [`KpmHandle`](crate::KpmHandle) when reference images
/// are added, and consumed by the matcher during queries.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmRefDataSet {
    /// All reference feature points across every page.
    pub ref_point: Vec<KpmRefData>,
    /// Total number of reference points (kept in sync with `ref_point.len()`).
    pub num: i32,
    /// Per-page metadata.
    pub page_info: Vec<KpmPageInfo>,
    /// Total number of pages (kept in sync with `page_info.len()`).
    pub page_num: i32,
}

/// Set of 2D input coordinates extracted from the current camera frame.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmInputDataSet {
    /// Detected feature coordinates.
    pub coord: Vec<KpmCoord2D>,
    /// Number of detected features (kept in sync with `coord.len()`).
    pub num: i32,
}

// ---------------------------------------------------------------------------
// Types ported from kpm.h
// ---------------------------------------------------------------------------

/// Per-page result of a KPM matching + pose-estimation query.
///
/// After a call to `kpm_query`, one of these is produced for every
/// registered page. Inspect `cam_pose_f` to decide whether the pose
/// is valid (`0`) or not (`-1`).
#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmResult {
    /// 3x4 camera pose matrix (row-major).
    ///
    /// Layout: `[[r00 r01 r02 tx], [r10 r11 r12 ty], [r20 r21 r22 tz]]`.
    pub cam_pose: [[f32; 4]; 3],
    /// Pose validity flag: `0` = valid, `-1` = not estimated.
    pub cam_pose_f: i32,
    /// Page number this result refers to.
    pub page_no: i32,
    /// Number of RANSAC inliers that support this pose.
    pub inlier_num: i32,
    /// Reprojection error of the estimated pose.
    pub error: f32,
    /// Skip flag: `1` = this page was excluded from matching.
    pub skip_f: i32,
}

// ---------------------------------------------------------------------------
// Corner-point types
// ---------------------------------------------------------------------------

/// 2D point with `f32` coordinates.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Point2f {
    pub x: f32,
    pub y: f32,
}

/// 2D point with integer coordinates, used for corner detection.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Point2i {
    pub x: i32,
    pub y: i32,
}

/// Fixed-capacity buffer of detected corner points.
///
/// Uses a statically-sized array of [`MAX_CORNER_POINTS`] entries to
/// avoid heap allocation on the hot detection path.
#[derive(Debug, Clone, PartialEq)]
pub struct CornerPoints {
    /// Number of valid entries in `pt`.
    pub num: i32,
    /// Corner positions (only the first `num` entries are meaningful).
    pub pt: [Point2i; MAX_CORNER_POINTS],
}

impl Default for CornerPoints {
    fn default() -> Self {
        Self {
            num: 0,
            pt: [Point2i::default(); MAX_CORNER_POINTS],
        }
    }
}

// ---------------------------------------------------------------------------
// FFI bridge types (used by cpp_backend)
// ---------------------------------------------------------------------------

/// A 3x3 homography matrix stored in row-major order.
///
/// Used to represent the planar transformation between a reference
/// image and the current camera frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Homography3x3(pub [f32; 9]);

impl Default for Homography3x3 {
    fn default() -> Self {
        Self([0.0; 9])
    }
}

/// Result from a query operation (FFI bridge level).
///
/// This type is used internally by [`CppFreakMatcher`](crate::CppFreakMatcher)
/// to shuttle data across the FFI boundary. Higher-level code should
/// prefer [`backend::QueryResult`](crate::backend::QueryResult).
#[derive(Debug, Clone, PartialEq)]
pub struct QueryResult {
    /// Page number of the matched reference image, or `-1` if no match.
    pub page_no: i32,
    /// 3x3 homography from the reference image to the camera frame.
    pub homography: Homography3x3,
    /// Reprojection error (`0.0` on success, `-1.0` on failure).
    pub error: f32,
}

/// A reference image to be added to the KPM database.
///
/// Contains the raw grayscale pixel data together with metadata needed
/// to compute 3D world coordinates from pixel positions.
#[derive(Debug, Clone, PartialEq)]
pub struct RefImage {
    /// Grayscale pixel data (one byte per pixel, row-major).
    pub data: Vec<u8>,
    /// Image width in pixels.
    pub width: i32,
    /// Image height in pixels.
    pub height: i32,
    /// Dots per inch — used to convert pixel positions to millimetres.
    pub dpi: f32,
    /// Page number this image belongs to.
    pub page_no: i32,
    /// Image number within the page.
    pub image_no: i32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kpm_coord2d_default() {
        let coord = KpmCoord2D::default();
        assert_eq!(coord.x, 0.0);
        assert_eq!(coord.y, 0.0);
    }

    #[test]
    fn test_freak_feature_default() {
        let feat = FreakFeature::default();
        assert_eq!(feat.v, [0u8; FREAK_SUB_DIMENSION]);
        assert_eq!(feat.angle, 0.0);
    }

    #[test]
    fn test_kpm_result_default() {
        let result = KpmResult::default();
        assert_eq!(result.cam_pose_f, 0);
    }

    #[test]
    fn test_kpm_ref_data_feature_vec_as_bytes() {
        let mut ref_data = KpmRefData::default();
        ref_data.feature_vec.v[0] = 42;
        let bytes = ref_data.feature_vec_as_bytes();
        assert_eq!(bytes[0], 42);
    }
}
