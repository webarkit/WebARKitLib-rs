//! Core Data Structures for WebARKitLib
//! Translated from ARToolKit C headers (ar.h, param.h, etc.)

pub type ARdouble = f64;

pub const AR_DIST_FACTOR_NUM_MAX: usize = 9;
pub const AR_CHAIN_MAX: usize = 10000;
pub const AR_SQUARE_MAX: usize = 30;
pub const AR_LABELING_WORK_SIZE: usize = 1024 * 32;

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
#[derive(Debug, Clone)]
#[repr(C)]
pub struct ARMarkerInfo2 {
    pub area: i32,
    pub pos: [ARdouble; 2],
    pub coord_num: i32,
    pub x_coord: [i32; AR_CHAIN_MAX],
    pub y_coord: [i32; AR_CHAIN_MAX],
    pub vertex: [i32; 5],
}

impl Default for ARMarkerInfo2 {
    fn default() -> Self {
        Self {
            area: 0,
            pos: [0.0; 2],
            coord_num: 0,
            x_coord: [0; AR_CHAIN_MAX],
            y_coord: [0; AR_CHAIN_MAX],
            vertex: [0; 5],
        }
    }
}

/// Result codes returned by arDetectMarker to report state of individual detected trapezoidal regions.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ARMarkerInfoCutoffPhase {
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

impl Default for ARMarkerInfoCutoffPhase {
    fn default() -> Self {
        ARMarkerInfoCutoffPhase::None
    }
}

/// Describes a detected trapezoidal area (a candidate for a marker match).
#[derive(Debug, Clone)]
#[repr(C)]
pub struct ARMarkerInfo {
    /// Area in pixels of the largest connected region
    pub area: i32,
    /// Marker ID if valid, or -1 if invalid
    pub id: i32,
    pub id_patt: i32,
    pub id_matrix: i32,
    /// Marker direction (0 to 3)
    pub dir: i32,
    pub dir_patt: i32,
    pub dir_matrix: i32,
    /// Marker matching confidence (0.0 to 1.0)
    pub cf: ARdouble,
    pub cf_patt: ARdouble,
    pub cf_matrix: ARdouble,
    /// 2D position of the centre of the marker
    pub pos: [ARdouble; 2],
    /// Line equations for the 4 sides of the marker
    pub line: [[ARdouble; 3]; 4],
    /// 2D positions of the corners of the marker
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
#[repr(C)]
#[derive(Debug, Clone)]
pub struct ARParamLTf {
    pub i2o: *mut f32,
    pub o2i: *mut f32,
    pub xsize: i32,
    pub ysize: i32,
    pub x_off: i32,
    pub y_off: i32,
}

#[repr(C)]
#[derive(Debug, Clone)]
pub struct ARParamLT {
    pub param: ARParam,
    pub param_ltf: ARParamLTf,
}

#[repr(C)]
#[derive(Debug, Clone)]
pub struct ARTrackingHistory {
    pub marker: ARMarkerInfo,
    pub count: i32,
}

impl Default for ARTrackingHistory {
    fn default() -> Self {
        Self {
            marker: ARMarkerInfo::default(),
            count: 0,
        }
    }
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

#[repr(C)]
#[derive(Debug, Clone)]
pub struct ARPattHandle {
    _dummy: u8,
}

#[repr(C)]
#[derive(Debug, Clone)]
pub struct ARImageProcInfo {
    _dummy: u8,
}

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ARPixelFormat {
    Invalid = 0,
}

impl Default for ARPixelFormat {
    fn default() -> Self {
        ARPixelFormat::Invalid
    }
}

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ARLabelingThreshMode {
    Manual = 0,
    AutoMedian,
    AutoOtsu,
    AutoAdaptive,
    AutoBracketing,
}

impl Default for ARLabelingThreshMode {
    fn default() -> Self {
        ARLabelingThreshMode::Manual
    }
}

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ARMatrixCodeType {
    Code3x3 = 0x03,
}

impl Default for ARMatrixCodeType {
    fn default() -> Self {
        ARMatrixCodeType::Code3x3
    }
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
}



