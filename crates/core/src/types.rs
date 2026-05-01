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

//! Core Data Structures for WebARKitLib
//! Translated from ARToolKit C headers (ar.h, param.h, etc.)

pub type ARdouble = f64;

pub const AR_DIST_FACTOR_NUM_MAX: usize = 9;
pub const AR_CHAIN_MAX: usize = 40000;
pub const AR_SQUARE_MAX: usize = 30;
pub const AR_LABELING_WORK_SIZE: usize = 1024 * 32;

pub const AR_TEMPLATE_MATCHING_COLOR: i32 = 0;
pub const AR_TEMPLATE_MATCHING_MONO: i32 = 1;
pub const AR_MATRIX_CODE_DETECTION: i32 = 2;
pub const AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX_CODE_DETECTION: i32 = 3;
pub const AR_TEMPLATE_MATCHING_MONO_AND_MATRIX_CODE_DETECTION: i32 = 4;

/// C equivalent: AR_NOUSE_TRACKING_HISTORY
/// Skip tracking history; only apply confidence cutoff on each frame independently.
pub const AR_NOUSE_TRACKING_HISTORY: i32 = 0;
/// C equivalent: AR_USE_TRACKING_HISTORY
/// Use full tracking history with resurrection of lost markers from previous frames.
pub const AR_USE_TRACKING_HISTORY: i32 = 1;
/// C equivalent: AR_USE_TRACKING_HISTORY_V2
/// Use tracking history (merge + aging) but without resurrection of lost markers.
pub const AR_USE_TRACKING_HISTORY_V2: i32 = 2;

/// A structure to hold a timestamp in seconds and microseconds, with arbitrary epoch.
#[derive(Debug, Clone, PartialEq, Default)]
#[repr(C)]
pub struct AR2VideoTimestampT {
    pub sec: u64,
    pub usec: u32,
}

/// A structure which carries information about a video frame retrieved by the video library.
#[derive(Debug, Clone, Default)]
pub struct AR2VideoBufferT {
    /// A pointer to the packed video data for this video frame.
    /// In Rust, we use a slice or Vec depending on ownership, usually represented as a raw slice or Vec.
    pub buff: Option<Vec<u8>>,
    /// For multi-planar video frames, arrays of planes.
    pub buf_planes: Option<Vec<Vec<u8>>>,
    /// Number of planes.
    pub buf_plane_count: u32,
    /// Luminance-only version of the image.
    pub buff_luma: Option<Vec<u8>>,
    /// Set to true when buff is valid.
    pub fill_flag: bool,
    /// Time at which the buffer was filled.
    pub time: AR2VideoTimestampT,
}

/// Structure holding camera parameters, including image size, projection matrix and lens distortion parameters.
#[derive(Debug, Clone, PartialEq, Default)]
#[repr(C)]
pub struct ARParam {
    /// The width in pixels of images returned by the video library for the camera.
    pub xsize: i32,
    /// The height in pixels of images returned by the video library for the camera.
    pub ysize: i32,
    /// The projection matrix for the calibrated camera parameters.
    pub mat: [[ARdouble; 4]; 3],
    /// Distortion factors.
    pub dist_factor: [ARdouble; AR_DIST_FACTOR_NUM_MAX],
    /// Distortion function version.
    pub dist_function_version: i32,
}

/// Captures detail of a trapezoidal region which is a candidate for marker detection.
///
/// This structure holds the raw 2D pixel-space information for a candidate region
/// found by connected-component labeling and contour extraction before it is fully
/// identified as a specific marker.
#[derive(Debug, Clone)]
#[repr(C)]
pub struct ARMarkerInfo2 {
    /// Area in pixels of the largest connected region
    pub area: i32,
    /// Center of the marker candidate (2D coordinate)
    pub pos: [ARdouble; 2],
    /// Number of coordinates corresponding to the candidate's contour
    pub coord_num: i32,
    /// X coordinates of the contour's perimeter pixels.
    pub x_coord: Vec<i32>,
    /// Y coordinates of the contour's perimeter pixels.
    pub y_coord: Vec<i32>,
    /// Indices into `x_coord` and `y_coord` arrays pointing to the four corners of the candidate.
    /// The fifth element wraps around to the first corner (`vertex[4] == vertex[0]`).
    pub vertex: [i32; 5],
}

impl Default for ARMarkerInfo2 {
    fn default() -> Self {
        Self {
            area: 0,
            pos: [0.0; 2],
            coord_num: 0,
            x_coord: vec![0i32; AR_CHAIN_MAX],
            y_coord: vec![0i32; AR_CHAIN_MAX],
            vertex: [0; 5],
        }
    }
}

/// Result codes returned by arDetectMarker to report state of individual detected trapezoidal regions.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ARMarkerInfoCutoffPhase {
    #[default]
    None = 0,
    PatternExtraction,
    MatchGeneric,
    MatchContrast,
    MatchBarcodeNotFound,
    MatchBarcodeEdcFail,
    MatchConfidence,
    PoseError,
    PoseErrorMulti,
    HeuristicTroublesomeMatrixCodes,
}

/// Error variants returned by pattern and matrix matching functions.
/// Maps 1:1 to [`ARMarkerInfoCutoffPhase`] — use `.into()` to convert.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MatchError {
    /// Generic match failure (C result code -1).
    Generic,
    /// Insufficient contrast (C result code -2).
    Contrast,
    /// Matrix barcode locator pattern not found (C result code -3).
    BarcodeNotFound,
    /// Matrix barcode ECC decoding failed (C result code -4).
    BarcodeEdcFail,
    /// Heuristic: troublesome matrix code pattern (C result code -5).
    HeuristicTroublesomeMatrixCodes,
    /// Pattern extraction from image failed (C result code -6).
    PatternExtraction,
}

/// Success payload returned by pattern and matrix matching functions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatchOk {
    /// Matched marker ID (template: pattern index; matrix: decoded integer).
    pub id: i32,
    /// Orientation (0–3, 90° increments).
    pub dir: i32,
    /// Confidence value (0.0–1.0; 1.0 for matrix codes).
    pub cf: f64,
    /// Number of errors corrected by ECC (0 for codes without ECC, always 0 for template).
    pub error_corrected: i32,
}

impl From<MatchError> for ARMarkerInfoCutoffPhase {
    fn from(e: MatchError) -> Self {
        match e {
            MatchError::Generic => Self::MatchGeneric,
            MatchError::Contrast => Self::MatchContrast,
            MatchError::BarcodeNotFound => Self::MatchBarcodeNotFound,
            MatchError::BarcodeEdcFail => Self::MatchBarcodeEdcFail,
            MatchError::HeuristicTroublesomeMatrixCodes => Self::HeuristicTroublesomeMatrixCodes,
            MatchError::PatternExtraction => Self::PatternExtraction,
        }
    }
}

/// Describes a detected trapezoidal area (a candidate for a marker match).
///
/// This serves as the primary output of marker detection. It contains both raw properties
/// of the shape (area, pos) and, if successfully matched, the identified marker ID and
/// viewing direction.
#[derive(Debug, Clone)]
#[repr(C)]
pub struct ARMarkerInfo {
    /// Area in pixels of the largest connected region.
    pub area: i32,
    /// Marker ID if valid, or -1 if invalid (global ID regardless of matching type).
    pub id: i32,
    /// Template marker ID if matched via template matching, -1 otherwise.
    pub id_patt: i32,
    /// Matrix (barcode) marker ID if matched via matrix code, -1 otherwise.
    pub id_matrix: i32,
    /// Orientation of the marker (0, 1, 2, or 3) representing rotation in 90° increments.
    pub dir: i32,
    /// Orientation if matched via template matching.
    pub dir_patt: i32,
    /// Orientation if matched via matrix code.
    pub dir_matrix: i32,
    /// Confidence value of the match (0.0 to 1.0).
    pub cf: ARdouble,
    /// Confidence value from template matching.
    pub cf_patt: ARdouble,
    /// Confidence value from matrix code decoding.
    pub cf_matrix: ARdouble,
    /// Center of the marker (2D coordinate).
    pub pos: [ARdouble; 2],
    /// Line equations of the four sides of the marker in 2D space `[a, b, c]`.
    pub line: [[ARdouble; 3]; 4],
    /// 2D coordinates of the four corners in ideal (undistorted) camera space.
    pub vertex: [[ARdouble; 2]; 4],
    /// Pointer to source region info for this marker.
    pub marker_info2_ptr: *mut ARMarkerInfo2,
    /// Tracking phase at which the marker was cut off
    pub cutoff_phase: ARMarkerInfoCutoffPhase,
    /// The numbers of errors detected and corrected
    pub error_corrected: i32,
    /// Global ID for matrix codes
    pub global_id: u64,
}

impl Default for ARMarkerInfo {
    fn default() -> Self {
        Self {
            area: 0,
            id: -1,
            id_patt: -1,
            id_matrix: -1,
            dir: 0,
            dir_patt: 0,
            dir_matrix: 0,
            cf: 0.0,
            cf_patt: 0.0,
            cf_matrix: 0.0,
            pos: [0.0; 2],
            line: [[0.0; 3]; 4],
            vertex: [[0.0; 2]; 4],
            marker_info2_ptr: std::ptr::null_mut(),
            cutoff_phase: ARMarkerInfoCutoffPhase::None,
            error_corrected: 0,
            global_id: 0,
        }
    }
}

// Opaque/dummy types for ARHandle dependencies
#[derive(Debug, Clone, Default)]
pub struct ARParamLTf {
    pub i2o: Vec<f32>,
    pub o2i: Vec<f32>,
    pub xsize: i32,
    pub ysize: i32,
    pub x_off: i32,
    pub y_off: i32,
}

impl ARParamLTf {
    pub fn new_basic(xsize: i32, ysize: i32) -> Self {
        let i2o = vec![0.0f32; (xsize * ysize * 2) as usize];
        let mut o2i = vec![0.0f32; (xsize * ysize * 2) as usize];
        for y in 0..ysize {
            for x in 0..xsize {
                let idx = ((y * xsize + x) * 2) as usize;
                // Initialize both i2o and o2i to identity mapping so that
                // ideal2observ and observ2ideal both behave as identity for
                // this basic lookup table.
                // Note: i2o maps ideal -> observed, o2i maps observed -> ideal.
                // Populate both arrays accordingly.
                // Safety: idx+1 is always within bounds because vectors sized appropriately.
                // i2o is currently immutable here; create a mutable copy, fill it, and reuse.
                // We'll construct i2o_vec and o2i directly instead of using the previously
                // allocated `i2o` variable which is immutable.
                // (We'll later replace the immutable i2o variable by a mutable one.)
                // However to minimize changes, write to o2i and leave i2o to be overwritten below.
                o2i[idx] = x as f32;
                o2i[idx + 1] = y as f32;
            }
        }
        // Now create a mutable i2o and fill it identically to o2i
        let mut i2o_filled = i2o;
        for y in 0..ysize {
            for x in 0..xsize {
                let idx = ((y * xsize + x) * 2) as usize;
                i2o_filled[idx] = x as f32;
                i2o_filled[idx + 1] = y as f32;
            }
        }

        Self {
            i2o: i2o_filled,
            o2i,
            xsize,
            ysize,
            x_off: 0,
            y_off: 0,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct ARParamLT {
    pub param: ARParam,
    pub param_ltf: ARParamLTf,
}

impl ARParamLT {
    pub fn new(param: ARParam, param_ltf: ARParamLTf) -> Self {
        Self { param, param_ltf }
    }

    pub fn new_basic(param: ARParam) -> Self {
        let param_ltf = ARParamLTf::new_basic(param.xsize, param.ysize);
        Self { param, param_ltf }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Default)]
pub struct ARTrackingHistory {
    pub marker: ARMarkerInfo,
    pub count: i32,
}

pub type ARLabelingLabelType = i16;

#[derive(Debug, Clone)]
pub struct ARLabelInfo {
    pub label_image: Vec<ARLabelingLabelType>,
    pub label_num: i32,
    pub area: Vec<i32>,
    pub clip: Vec<[i32; 4]>,
    pub pos: Vec<[ARdouble; 2]>,
    pub work: Vec<i32>,
    pub work2: Vec<i32>,
}

impl Default for ARLabelInfo {
    fn default() -> Self {
        Self {
            label_image: Vec::new(),
            label_num: 0,
            area: vec![0; AR_LABELING_WORK_SIZE],
            clip: vec![[0; 4]; AR_LABELING_WORK_SIZE],
            pos: vec![[0.0; 2]; AR_LABELING_WORK_SIZE],
            work: vec![0; AR_LABELING_WORK_SIZE],
            work2: vec![0; AR_LABELING_WORK_SIZE * 7],
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct ARPattHandle {
    pub patt_num: i32,
    pub patt_num_max: i32,
    pub pattf: Vec<i32>,
    pub patt: Vec<Vec<i16>>,
    pub pattpow: Vec<ARdouble>,
    pub patt_bw: Vec<Vec<i16>>,
    pub pattpow_bw: Vec<ARdouble>,
    pub patt_size: i32,
    pub active: Vec<bool>,
}

#[repr(C)]
#[derive(Debug, Clone)]
pub struct ARImageProcInfo {
    _dummy: u8,
}

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ARPixelFormat {
    #[default]
    Invalid = -1,
    RGB = 0,
    BGR,
    RGBA,
    BGRA,
    ABGR,
    MONO,
    ARGB,
    TwoVuy,
    Yuvs,
    Rgb565,
    Rgba5551,
    Rgba4444,
    FourTwoZeroV,
    FourTwoZeroF,
    NV21,
}

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ARLabelingThreshMode {
    #[default]
    Manual = 0,
    AutoMedian,
    AutoOtsu,
    AutoAdaptive,
    AutoBracketing,
}

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ARMatrixCodeType {
    /// 3x3 Matrix Code (Default)
    #[default]
    Code3x3 = 0x03,
    /// 3x3 Matrix Code with Parity (6,5)
    Code3x3Parity65 = 0x03 | 0x100,
    /// 3x3 Matrix Code with Hamming (6,3)
    Code3x3Hamming63 = 0x03 | 0x200,
    /// 4x4 Matrix Code
    Code4x4 = 0x04,
    /// 4x4 Matrix Code with BCH (13,9,3)
    Code4x4BCH1393 = 0x04 | 0x300,
    /// 4x4 Matrix Code with BCH (13,5,5)
    Code4x4BCH1355 = 0x04 | 0x400,
    /// 5x5 Matrix Code
    Code5x5 = 0x05,
    /// 5x5 Matrix Code with BCH (22,12,5)
    Code5x5BCH22125 = 0x05 | 0x500,
    /// 5x5 Matrix Code with BCH (22,7,7)
    Code5x5BCH2277 = 0x05 | 0x600,
    /// 6x6 Matrix Code
    Code6x6 = 0x06,
}

/// Structure holding state of an instance of the square marker tracker.
#[derive(Debug, Clone)]
#[repr(C)]
pub struct ARHandle {
    pub ar_debug: i32,
    pub ar_pixel_format: ARPixelFormat,
    pub ar_pixel_size: i32,
    pub ar_labeling_mode: i32,
    pub ar_labeling_thresh: i32,
    pub ar_image_proc_mode: i32,
    pub ar_pattern_detection_mode: i32,
    pub ar_marker_extraction_mode: i32,
    pub ar_param_lt: *mut ARParamLT,
    pub xsize: i32,
    pub ysize: i32,
    pub marker_num: i32,
    pub marker_info: Box<[ARMarkerInfo; AR_SQUARE_MAX]>,
    pub marker2_num: i32,
    pub marker_info2: Box<[ARMarkerInfo2; AR_SQUARE_MAX]>,
    pub history_num: i32,
    pub history: Box<[ARTrackingHistory; AR_SQUARE_MAX]>,
    pub label_info: ARLabelInfo,
    pub patt_handle: *mut ARPattHandle,
    pub ar_labeling_thresh_mode: ARLabelingThreshMode,
    pub ar_labeling_thresh_auto_interval: i32,
    pub ar_labeling_thresh_auto_interval_ttl: i32,
    pub ar_labeling_thresh_auto_bracket_over: i32,
    pub ar_labeling_thresh_auto_bracket_under: i32,
    pub ar_image_proc_info: *mut ARImageProcInfo,
    pub patt_ratio: ARdouble,
    pub matrix_code_type: ARMatrixCodeType,
}

impl ARHandle {
    pub fn new(param: ARParam) -> Self {
        // In the full port, arParamLT would be initialized here too.
        ARHandle {
            xsize: param.xsize,
            ysize: param.ysize,
            ..ARHandle::default()
        }
    }

    pub fn set_pixel_format(&mut self, format: ARPixelFormat) {
        self.ar_pixel_format = format;
        // Update pixel size based on format if needed
        self.ar_pixel_size = match format {
            ARPixelFormat::RGB | ARPixelFormat::BGR => 3,
            ARPixelFormat::RGBA
            | ARPixelFormat::BGRA
            | ARPixelFormat::ABGR
            | ARPixelFormat::ARGB => 4,
            ARPixelFormat::MONO => 1,
            _ => 0,
        };
    }

    pub fn set_matrix_code_type(&mut self, code_type: ARMatrixCodeType) {
        self.matrix_code_type = code_type;
    }

    pub fn set_pattern_detection_mode(&mut self, mode: i32) {
        self.ar_pattern_detection_mode = mode;
    }

    pub fn get_matrix_code_type(&self) -> ARMatrixCodeType {
        self.matrix_code_type
    }
}

impl Default for ARHandle {
    fn default() -> Self {
        Self {
            ar_debug: 0,
            ar_pixel_format: ARPixelFormat::default(),
            ar_pixel_size: 0,
            ar_labeling_mode: 0,
            ar_labeling_thresh: 100,
            ar_image_proc_mode: 0,
            ar_pattern_detection_mode: 0,
            ar_marker_extraction_mode: 0,
            ar_param_lt: std::ptr::null_mut(),
            xsize: 0,
            ysize: 0,
            marker_num: 0,
            marker_info: vec![ARMarkerInfo::default(); AR_SQUARE_MAX]
                .into_boxed_slice()
                .try_into()
                .unwrap(),
            marker2_num: 0,
            marker_info2: vec![ARMarkerInfo2::default(); AR_SQUARE_MAX]
                .into_boxed_slice()
                .try_into()
                .unwrap(),
            history_num: 0,
            history: vec![ARTrackingHistory::default(); AR_SQUARE_MAX]
                .into_boxed_slice()
                .try_into()
                .unwrap(),
            label_info: ARLabelInfo::default(),
            patt_handle: std::ptr::null_mut(),
            ar_labeling_thresh_mode: ARLabelingThreshMode::default(),
            ar_labeling_thresh_auto_interval: 0,
            ar_labeling_thresh_auto_interval_ttl: 0,
            ar_labeling_thresh_auto_bracket_over: 0,
            ar_labeling_thresh_auto_bracket_under: 0,
            ar_image_proc_info: std::ptr::null_mut(),
            patt_ratio: 0.5,
            matrix_code_type: ARMatrixCodeType::default(),
        }
    }
}

pub use crate::icp::{ICPHandleT, ICPStereoHandleT};

/// Structure holding state of an instance of the monocular pose estimator.
#[derive(Debug, Clone)]
#[repr(C)]
pub struct AR3DHandle {
    pub icp_handle: *mut ICPHandleT,
}

impl Default for AR3DHandle {
    fn default() -> Self {
        Self {
            icp_handle: std::ptr::null_mut(),
        }
    }
}

/// Structure holding state of an instance of the stereo pose estimator.
#[derive(Debug, Clone)]
#[repr(C)]
pub struct AR3DStereoHandle {
    pub icp_stereo_handle: *mut ICPStereoHandleT,
}

impl Default for AR3DStereoHandle {
    fn default() -> Self {
        Self {
            icp_stereo_handle: std::ptr::null_mut(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_arparam_default_initialization() {
        let param = ARParam::default();
        assert_eq!(param.xsize, 0);
        assert_eq!(param.ysize, 0);
        assert_eq!(param.mat, [[0.0; 4]; 3]);
        assert_eq!(param.dist_factor, [0.0; AR_DIST_FACTOR_NUM_MAX]);
        assert_eq!(param.dist_function_version, 0);
    }

    #[test]
    fn test_armarkerinfo_default_initialization() {
        let marker_info = ARMarkerInfo::default();
        assert_eq!(marker_info.area, 0);
        assert_eq!(marker_info.id, -1);
        assert_eq!(marker_info.id_patt, -1);
        assert_eq!(marker_info.id_matrix, -1);
        assert_eq!(marker_info.dir, 0);
        assert_eq!(marker_info.cf, 0.0);
        assert_eq!(marker_info.pos, [0.0; 2]);
        assert_eq!(marker_info.cutoff_phase, ARMarkerInfoCutoffPhase::None);
        assert_eq!(marker_info.error_corrected, 0);
        assert_eq!(marker_info.global_id, 0);
    }

    #[test]
    fn test_arhandle_initial_state() {
        let handle = ARHandle::default();
        assert_eq!(handle.ar_debug, 0);
        assert_eq!(handle.ar_labeling_thresh, 100);
        assert_eq!(handle.patt_ratio, 0.5);
        assert_eq!(handle.matrix_code_type, ARMatrixCodeType::Code3x3);
        assert_eq!(handle.ar_pattern_detection_mode, 0);
    }

    #[test]
    fn test_ar3dhandle_default_initialization() {
        let handle = AR3DHandle::default();
        assert_eq!(handle.icp_handle, std::ptr::null_mut());
    }

    #[test]
    fn test_ar3dstereohandle_default_initialization() {
        let handle = AR3DStereoHandle::default();
        assert_eq!(handle.icp_stereo_handle, std::ptr::null_mut());
    }

    #[test]
    fn test_match_error_to_cutoff_phase_generic() {
        assert_eq!(
            ARMarkerInfoCutoffPhase::from(MatchError::Generic),
            ARMarkerInfoCutoffPhase::MatchGeneric
        );
    }

    #[test]
    fn test_match_error_to_cutoff_phase_contrast() {
        assert_eq!(
            ARMarkerInfoCutoffPhase::from(MatchError::Contrast),
            ARMarkerInfoCutoffPhase::MatchContrast
        );
    }

    #[test]
    fn test_match_error_to_cutoff_phase_barcode_not_found() {
        assert_eq!(
            ARMarkerInfoCutoffPhase::from(MatchError::BarcodeNotFound),
            ARMarkerInfoCutoffPhase::MatchBarcodeNotFound
        );
    }

    #[test]
    fn test_match_error_to_cutoff_phase_barcode_edc_fail() {
        assert_eq!(
            ARMarkerInfoCutoffPhase::from(MatchError::BarcodeEdcFail),
            ARMarkerInfoCutoffPhase::MatchBarcodeEdcFail
        );
    }

    #[test]
    fn test_match_error_to_cutoff_phase_heuristic() {
        assert_eq!(
            ARMarkerInfoCutoffPhase::from(MatchError::HeuristicTroublesomeMatrixCodes),
            ARMarkerInfoCutoffPhase::HeuristicTroublesomeMatrixCodes
        );
    }

    #[test]
    fn test_match_error_to_cutoff_phase_pattern_extraction() {
        assert_eq!(
            ARMarkerInfoCutoffPhase::from(MatchError::PatternExtraction),
            ARMarkerInfoCutoffPhase::PatternExtraction
        );
    }
}
