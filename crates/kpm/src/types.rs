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

// Constants ported from kpmType.h and kpm.h

pub const FREAK_SUB_DIMENSION: usize = 96;
pub const DB_IMAGE_MAX: usize = 1024;
pub const MAX_CORNER_POINTS: usize = 2000;

// --- Types ported from kpmType.h ---

#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmCoord2D {
    pub x: f32,
    pub y: f32,
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmImageInfo {
    pub image_no: i32,
    pub width: i32,
    pub height: i32,
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmPageInfo {
    pub page_no: i32,
    pub image_num: i32,
    pub image_info: Vec<KpmImageInfo>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FreakFeature {
    pub v: [u8; FREAK_SUB_DIMENSION],
    pub angle: f32,
    pub scale: f32,
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

#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmRefData {
    pub coord2d: KpmCoord2D,
    pub coord3d: KpmCoord2D,
    pub feature_vec: FreakFeature,
    pub page_no: i32,
    pub ref_image_no: i32,
}

impl KpmRefData {
    pub fn feature_vec_as_bytes(&self) -> &[u8] {
        &self.feature_vec.v
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmRefDataSet {
    pub ref_point: Vec<KpmRefData>,
    pub num: i32,
    pub page_info: Vec<KpmPageInfo>,
    pub page_num: i32,
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmInputDataSet {
    pub coord: Vec<KpmCoord2D>,
    pub num: i32,
}

// --- Types ported from kpm.h ---

#[derive(Debug, Default, Clone, PartialEq)]
pub struct KpmResult {
    pub cam_pose: [[f32; 4]; 3],
    pub cam_pose_f: i32,
    pub page_no: i32,
    pub inlier_num: i32,
    pub error: f32,
    pub skip_f: i32,
}

// --- Corner point types ---

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Point2f {
    pub x: f32,
    pub y: f32,
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Point2i {
    pub x: i32,
    pub y: i32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CornerPoints {
    pub num: i32,
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

// --- FFI bridge types (used by cpp_backend) ---

/// A 3x3 homography matrix stored in row-major order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Homography3x3(pub [f32; 9]);

impl Default for Homography3x3 {
    fn default() -> Self {
        Self([0.0; 9])
    }
}

/// Result from a query operation.
#[derive(Debug, Clone, PartialEq)]
pub struct QueryResult {
    pub page_no: i32,
    pub homography: Homography3x3,
    pub error: f32,
}

/// A reference image to be added to the database.
#[derive(Debug, Clone, PartialEq)]
pub struct RefImage {
    pub data: Vec<u8>,
    pub width: i32,
    pub height: i32,
    pub dpi: f32,
    pub page_no: i32,
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
