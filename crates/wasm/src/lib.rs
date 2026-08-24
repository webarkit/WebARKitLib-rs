/*
 *  lib.rs
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

//! # WebARKitLib-rs WASM
//!
//! This crate provides the WebAssembly bindings for `webarkitlib-rs`.
//! It allows using the core AR detection and pose estimation algorithms in the browser.
//!
//! ## Key Components
//!
//! - `WasmARHandle`: The main interface for marker detection and pose estimation.
//! - `init_wasm`: Combined initializer — installs the panic hook and logs the library version.
//! - `init_panic_hook`: Utility to get better Rust panic messages in the browser console.

use std::io::Cursor;
use wasm_bindgen::prelude::*;
use webarkitlib_rs::ar2::{
    ar2_read_surface_set_from_bytes, ar2_surface_set_marker_info, ar2_tracking, AR2Handle,
    AR2SurfaceSet,
};
use webarkitlib_rs::icp::icp_create_handle;
use webarkitlib_rs::image_proc::{rgba_to_gray, ARImageProcInfo};
use webarkitlib_rs::marker::ar_detect_marker;
use webarkitlib_rs::pattern::ar_patt_load_from_buffer;
use webarkitlib_rs::pose::{ar_3d_create_handle, ar_3d_delete_handle, ar_get_trans_mat_square};
use webarkitlib_rs::types::{
    AR2VideoBufferT, AR3DHandle, ARHandle, ARLabelingThreshMode, ARMatrixCodeType, ARParam,
    ARParamLT, ARPattHandle, ARPixelFormat,
};
use webarkitlib_rs::version;

// KPM detection (the `simple_nft.rs` steps 3a + 4 surface).
use std::sync::Arc;
use webarkitlib_rs::kpm::ref_data_set::KPM_CHANGE_PAGE_NO_ALL_PAGES;
use webarkitlib_rs::kpm::types::KpmRefDataSet;
use webarkitlib_rs::kpm::{KpmHandle, RustFreakMatcher};

/// Returns the current version string of the library.
#[wasm_bindgen]
pub fn get_version() -> String {
    version::get_version().to_string()
}

/// Logs the library name and version to the browser console.
#[wasm_bindgen]
pub fn print_version() {
    let msg = format!("WebARKitLib-rs v{}", version::get_version());
    web_sys::console::log_1(&msg.into());
}

/// Initializes the panic hook for better Rust panic messages in the browser console.
#[wasm_bindgen]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

/// Initializes the WASM module: installs the panic hook and logs the library version.
#[wasm_bindgen]
pub fn init_wasm() {
    console_error_panic_hook::set_once();
    // Route arlog_*! output to the browser console (DevTools) via console_log.
    #[cfg(all(feature = "log-helpers", target_arch = "wasm32"))]
    webarkitlib_rs::arlog::ar_log_init_wasm();
    print_version();
}

#[wasm_bindgen]
pub struct WasmARHandle {
    handle: ARHandle,
    ar3d_handle: *mut AR3DHandle,
    param: ARParam,
}

#[wasm_bindgen]
impl WasmARHandle {
    #[wasm_bindgen(constructor)]
    pub fn new(param_bytes: &[u8]) -> Result<WasmARHandle, JsValue> {
        let cursor = Cursor::new(param_bytes);
        let param = ARParam::load(cursor)
            .map_err(|e| JsValue::from_str(&format!("Failed to load param: {}", e)))?;

        let ar3d_handle = ar_3d_create_handle(&param).map_err(JsValue::from_str)?;

        let mut handle = ARHandle::new(param.clone());
        handle.set_pixel_format(ARPixelFormat::RGBA);

        // Initialize pattern handle
        let patt_handle = Box::into_raw(Box::new(ARPattHandle::new(16, 25)));
        handle.patt_handle = patt_handle;

        // Initialize lookup table handle
        let ar_param_lt = Box::into_raw(Box::new(ARParamLT::new_basic(param.clone())));
        handle.ar_param_lt = ar_param_lt;

        Ok(WasmARHandle {
            handle,
            ar3d_handle,
            param,
        })
    }

    pub fn load_pattern(&mut self, patt_content: &str) -> Result<i32, JsValue> {
        if self.handle.patt_handle.is_null() {
            return Err(JsValue::from_str("Pattern handle is null"));
        }
        let patt_handle = unsafe { &mut *self.handle.patt_handle };
        let idx = ar_patt_load_from_buffer(patt_handle, patt_content).map_err(JsValue::from_str)?;
        Ok(idx)
    }

    pub fn set_threshold(&mut self, thresh: i32) {
        self.handle.ar_labeling_thresh = thresh;
        self.handle.ar_labeling_thresh_mode = ARLabelingThreshMode::Manual;
    }

    pub fn set_threshold_mode(&mut self, mode: i32) {
        self.handle.ar_labeling_thresh_mode = match mode {
            0 => ARLabelingThreshMode::Manual,
            2 => ARLabelingThreshMode::AutoOtsu,
            _ => ARLabelingThreshMode::Manual,
        };
    }

    pub fn set_debug_mode(&mut self, debug: bool) {
        self.handle.ar_debug = if debug { 1 } else { 0 };
    }

    /// Set the pattern detection mode.
    /// 0 = template matching colour (pattern markers),
    /// 1 = template matching mono,
    /// 2 = matrix code detection (barcode markers),
    /// 3 = colour + matrix code,
    /// 4 = mono + matrix code.
    pub fn set_pattern_detection_mode(&mut self, mode: i32) {
        self.handle.set_pattern_detection_mode(mode);
    }

    /// Set the matrix code type used for barcode detection.
    /// Maps integer values to `ARMatrixCodeType` variants:
    /// 3=3x3, 259=3x3Parity65, 515=3x3Hamming63,
    /// 4=4x4, 772=4x4BCH1393, 1028=4x4BCH1355,
    /// 5=5x5, 1285=5x5BCH22125, 1541=5x5BCH2277, 6=6x6.
    pub fn set_matrix_code_type(&mut self, code_type: i32) {
        let ct = match code_type {
            3 => ARMatrixCodeType::Code3x3,
            259 => ARMatrixCodeType::Code3x3Parity65,
            515 => ARMatrixCodeType::Code3x3Hamming63,
            4 => ARMatrixCodeType::Code4x4,
            772 => ARMatrixCodeType::Code4x4BCH1393,
            1028 => ARMatrixCodeType::Code4x4BCH1355,
            5 => ARMatrixCodeType::Code5x5,
            1285 => ARMatrixCodeType::Code5x5BCH22125,
            1541 => ARMatrixCodeType::Code5x5BCH2277,
            6 => ARMatrixCodeType::Code6x6,
            _ => {
                web_sys::console::warn_1(&JsValue::from_str(&format!(
                    "[WebARKit] Unknown matrix code type {code_type}, falling back to Code3x3"
                )));
                ARMatrixCodeType::Code3x3
            }
        };
        self.handle.set_matrix_code_type(ct);
    }

    pub fn detect_markers(
        &mut self,
        frame: &[u8],
        width: i32,
        height: i32,
    ) -> Result<JsValue, JsValue> {
        // Sync handle dimensions with actual frame dimensions
        if self.handle.xsize != width || self.handle.ysize != height {
            self.handle.xsize = width;
            self.handle.ysize = height;

            // Recreate lookup table for new dimensions
            if !self.handle.ar_param_lt.is_null() {
                unsafe {
                    let _ = Box::from_raw(self.handle.ar_param_lt);
                }
            }
            let mut new_param = self.param.clone();
            new_param.xsize = width;
            new_param.ysize = height;
            let ar_param_lt = Box::into_raw(Box::new(ARParamLT::new_basic(new_param)));
            self.handle.ar_param_lt = ar_param_lt;
        }

        let luma = rgba_to_gray(frame);

        // Handle auto-thresholding if requested
        if matches!(
            self.handle.ar_labeling_thresh_mode,
            ARLabelingThreshMode::AutoOtsu
        ) {
            let mut ipi = ARImageProcInfo::new(width, height);
            if let Ok(otsu) = ipi.luma_hist_and_otsu(&luma) {
                self.handle.ar_labeling_thresh = otsu as i32;
            }
        }

        let video_buffer = AR2VideoBufferT {
            buff: Some(frame.to_vec()),
            buff_luma: Some(luma),
            fill_flag: true,
            ..Default::default()
        };

        ar_detect_marker(&mut self.handle, &video_buffer).map_err(JsValue::from_str)?;

        let mut results = Vec::new();
        for i in 0..self.handle.marker_num as usize {
            let marker = &self.handle.marker_info[i];
            results.push(MarkerResult {
                area: marker.area,
                id: marker.id,
                id_patt: marker.id_patt,
                id_matrix: marker.id_matrix,
                dir: marker.dir,
                dir_patt: marker.dir_patt,
                dir_matrix: marker.dir_matrix,
                cf: marker.cf,
                cf_patt: marker.cf_patt,
                cf_matrix: marker.cf_matrix,
                pos: marker.pos,
                line: marker.line,
                vertex: marker.vertex,
                cutoff_phase: marker.cutoff_phase as i32,
                error_corrected: marker.error_corrected,
                global_id: marker.global_id,
            });
        }

        serde_wasm_bindgen::to_value(&results)
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
    }

    /// Get the current threshold used for binarization.
    pub fn get_threshold(&self) -> i32 {
        self.handle.ar_labeling_thresh
    }

    pub fn get_trans_mat(&self, marker_idx: usize, width: f64) -> Result<JsValue, JsValue> {
        if marker_idx >= self.handle.marker_num as usize {
            return Err(JsValue::from_str("Invalid marker index"));
        }

        let marker_info = &self.handle.marker_info[marker_idx];
        let mut conv = [[0.0; 4]; 3];

        let ar3d_ref = unsafe { &*self.ar3d_handle };

        let icp_error = ar_get_trans_mat_square(ar3d_ref, marker_info, width, &mut conv)
            .map_err(JsValue::from_str)?;

        // Flatten 3x4 to 12 floats
        let mut flat = [0.0f32; 12];
        for r in 0..3 {
            for c in 0..4 {
                flat[r * 4 + c] = conv[r][c] as f32;
            }
        }

        let result = PoseResult {
            matrix: flat.to_vec(),
            icp_error: icp_error as f32,
        };

        serde_wasm_bindgen::to_value(&result)
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
    }
}

impl Drop for WasmARHandle {
    fn drop(&mut self) {
        let _ = ar_3d_delete_handle(&mut self.ar3d_handle);
        if !self.handle.patt_handle.is_null() {
            unsafe {
                let _ = Box::from_raw(self.handle.patt_handle);
            }
        }
        if !self.handle.ar_param_lt.is_null() {
            unsafe {
                let _ = Box::from_raw(self.handle.ar_param_lt);
            }
        }
    }
}

/// Full mapping of `ARMarkerInfo` for consumption by JavaScript.
///
/// All fields mirror `ARMarkerInfo` in `crates/core/src/types.rs`.
/// The raw pointer (`marker_info2_ptr`) is intentionally omitted.
#[derive(serde::Serialize)]
pub struct MarkerResult {
    /// Area in pixels of the largest connected region.
    pub area: i32,
    /// Global marker ID (-1 if unmatched).
    pub id: i32,
    /// Template (pattern) marker ID (-1 if not matched by template).
    pub id_patt: i32,
    /// Matrix (barcode) marker ID (-1 if not matched by barcode).
    pub id_matrix: i32,
    /// Marker orientation (0–3, 90° increments).
    pub dir: i32,
    /// Orientation from template matching.
    pub dir_patt: i32,
    /// Orientation from matrix code decoding.
    pub dir_matrix: i32,
    /// Confidence of the best match (0.0–1.0).
    pub cf: f64,
    /// Confidence from template matching.
    pub cf_patt: f64,
    /// Confidence from matrix code decoding.
    pub cf_matrix: f64,
    /// Centre of the marker in 2D pixel space.
    pub pos: [f64; 2],
    /// Line equations `[a, b, c]` for each of the four sides.
    pub line: [[f64; 3]; 4],
    /// 2D coordinates of the four corners in undistorted camera space.
    pub vertex: [[f64; 2]; 4],
    /// Tracking phase at which this candidate was cut off (maps to `ARMarkerInfoCutoffPhase` as i32).
    pub cutoff_phase: i32,
    /// Number of errors detected and corrected (ECC).
    pub error_corrected: i32,
    /// Global ID for matrix codes.
    pub global_id: u64,
}

#[derive(serde::Serialize)]
pub struct PoseResult {
    pub matrix: Vec<f32>,
    pub icp_error: f32,
}

// ===========================================================================
// WasmNFTHandle — NFT (Natural Feature Tracking) for WASM
// ===========================================================================

/// NFT tracking result returned to JavaScript.
#[derive(serde::Serialize)]
pub struct NFTTrackingResult {
    /// Whether tracking succeeded this frame.
    pub found: bool,
    /// 3×4 camera pose matrix (12 floats, row-major).
    pub matrix: Vec<f32>,
    /// Reprojection error (lower is better).
    pub error: f32,
    /// Number of continuous tracking frames.
    pub cont_num: i32,
}

/// WebAssembly handle for NFT (Natural Feature Tracking).
///
/// This handle manages the AR2 tracking pipeline, which uses template
/// matching on image pyramids to refine an initial camera pose. The
/// initial pose can be provided from JavaScript (e.g. from a separate
/// KPM detection step or a prior frame).
///
/// ## Usage from JavaScript
///
/// ```js
/// const nft = new WasmNFTHandle(cameraParamBytes, width, height);
/// nft.load_nft_marker(isetBytes, fsetBytes);
/// nft.set_initial_pose(matrix12Floats);
/// const result = nft.track(rgbaFrame, width, height);
/// if (result.found) {
///     // Use result.matrix (3x4 pose)
/// }
/// ```
#[wasm_bindgen]
pub struct WasmNFTHandle {
    ar2_handle: AR2Handle,
    surface_set: AR2SurfaceSet,
    /// Marker base width from .iset scale[0].
    marker_width: i32,
    /// Marker base height from .iset scale[0].
    marker_height: i32,
    /// Marker DPI from .iset scale[0].
    marker_dpi: f32,
    /// Whether NFT marker data has been loaded.
    loaded: bool,
}

#[wasm_bindgen]
impl WasmNFTHandle {
    /// Create a new NFT tracking handle.
    ///
    /// # Arguments
    ///
    /// * `param_bytes` — Camera parameter file contents (`camera_para.dat`).
    /// * `width` — Camera frame width in pixels.
    /// * `height` — Camera frame height in pixels.
    #[wasm_bindgen(constructor)]
    pub fn new(param_bytes: &[u8], width: i32, height: i32) -> Result<WasmNFTHandle, JsValue> {
        let cursor = Cursor::new(param_bytes);
        let mut param = ARParam::load(cursor)
            .map_err(|e| JsValue::from_str(&format!("Failed to load camera param: {}", e)))?;

        // Scale camera parameters to match the requested frame size.
        // For digital cameras with square pixels, scale focal lengths isotropically
        // (based on height) to preserve fx == fy, and adjust optical center proportionally.
        let scale = height as f64 / param.ysize as f64;
        let orig_cx = param.mat[0][2];
        let orig_cy = param.mat[1][2];

        param.mat[0][0] *= scale;
        param.mat[0][1] *= scale;
        param.mat[1][0] *= scale;
        param.mat[1][1] *= scale;

        param.mat[0][2] = (orig_cx / param.xsize as f64) * width as f64;
        param.mat[1][2] = (orig_cy / param.ysize as f64) * height as f64;

        param.xsize = width;
        param.ysize = height;

        // Create AR2Handle.
        let mut ar2_handle = AR2Handle::new(width, height, ARPixelFormat::MONO);

        // Set up camera parameters.
        let param_lt = Box::new(ARParamLT::new_basic(param.clone()));
        ar2_handle.cparam_lt = Box::into_raw(param_lt);

        // Set up ICP handle.
        let icp_handle_ptr = icp_create_handle(&param.mat)
            .map_err(|e| JsValue::from_str(&format!("Failed to create ICP handle: {}", e)))?;
        ar2_handle.icp_handle = icp_handle_ptr;

        Ok(WasmNFTHandle {
            ar2_handle,
            surface_set: AR2SurfaceSet::default(),
            marker_width: 0,
            marker_height: 0,
            marker_dpi: 0.0,
            loaded: false,
        })
    }

    /// Load an NFT marker from .iset and .fset binary data.
    ///
    /// This is the WASM equivalent of `ar2ReadSurfaceSet()` in the C API.
    /// It internally loads both the image pyramid (.iset) and feature points
    /// (.fset) and constructs the tracking surface set.
    ///
    /// # Arguments
    ///
    /// * `iset_bytes` — Contents of the `.iset` file (image pyramid).
    /// * `fset_bytes` — Contents of the `.fset` file (feature points).
    pub fn load_nft_marker(&mut self, iset_bytes: &[u8], fset_bytes: &[u8]) -> Result<(), JsValue> {
        // Build surface set from both .iset and .fset data.
        self.surface_set = ar2_read_surface_set_from_bytes(iset_bytes, fset_bytes)
            .map_err(|e| JsValue::from_str(&format!("Failed to load surface set: {}", e)))?;

        // Store marker dimensions from the surface set.
        if let Some((w, h, dpi)) = ar2_surface_set_marker_info(&self.surface_set) {
            self.marker_width = w;
            self.marker_height = h;
            self.marker_dpi = dpi;
        }

        self.loaded = true;

        let num_scales = self
            .surface_set
            .surface
            .first()
            .and_then(|s| s.feature_set.as_ref())
            .map(|fs| fs.list.len())
            .unwrap_or(0);

        let msg = format!(
            "[WebARKit NFT] Marker loaded: {}x{} @ {:.0} DPI, {} feature scales",
            self.marker_width, self.marker_height, self.marker_dpi, num_scales
        );
        web_sys::console::log_1(&msg.into());

        Ok(())
    }

    /// Set the initial camera pose for tracking.
    ///
    /// The pose is a 3×4 matrix provided as 12 floats in row-major order:
    /// `[r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz]`
    ///
    /// This must be called before `track()` can succeed, typically using
    /// a pose obtained from KPM detection.
    pub fn set_initial_pose(&mut self, pose: &[f32]) -> Result<(), JsValue> {
        if pose.len() != 12 {
            return Err(JsValue::from_str(
                "Pose must be 12 floats (3x4 matrix, row-major)",
            ));
        }
        if !self.loaded {
            return Err(JsValue::from_str(
                "NFT marker not loaded — call load_nft_marker first",
            ));
        }

        let mut mat = [[0.0f32; 4]; 3];
        for r in 0..3 {
            for c in 0..4 {
                mat[r][c] = pose[r * 4 + c];
            }
        }

        self.surface_set.trans1 = mat;
        self.surface_set.trans2 = mat;
        self.surface_set.trans3 = mat;
        self.surface_set.cont_num = 1;

        Ok(())
    }

    /// Run one frame of AR2 tracking.
    ///
    /// # Arguments
    ///
    /// * `frame` — RGBA pixel data of the camera frame.
    /// * `width` — Frame width in pixels.
    /// * `height` — Frame height in pixels.
    ///
    /// # Returns
    ///
    /// An `NFTTrackingResult` with the tracking status and refined pose.
    pub fn track(&mut self, frame: &[u8], _width: i32, _height: i32) -> Result<JsValue, JsValue> {
        if !self.loaded {
            return Err(JsValue::from_str("NFT marker not loaded"));
        }

        if self.surface_set.cont_num <= 0 {
            return serde_wasm_bindgen::to_value(&NFTTrackingResult {
                found: false,
                matrix: vec![0.0; 12],
                error: -1.0,
                cont_num: 0,
            })
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)));
        }

        // Convert RGBA to grayscale.
        let luma = rgba_to_gray(frame);

        let mut trans = self.surface_set.trans1;
        let mut err = 0.0f32;

        match ar2_tracking(
            &mut self.ar2_handle,
            &mut self.surface_set,
            &luma,
            &mut trans,
            &mut err,
        ) {
            Ok(()) => {
                // Flatten 3x4 to 12 floats.
                let mut flat = vec![0.0f32; 12];
                for r in 0..3 {
                    for c in 0..4 {
                        flat[r * 4 + c] = trans[r][c];
                    }
                }

                Ok(serde_wasm_bindgen::to_value(&NFTTrackingResult {
                    found: true,
                    matrix: flat,
                    error: err,
                    cont_num: self.surface_set.cont_num,
                })
                .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))?)
            }
            Err(_code) => Ok(serde_wasm_bindgen::to_value(&NFTTrackingResult {
                found: false,
                matrix: vec![0.0; 12],
                error: err,
                cont_num: self.surface_set.cont_num,
            })
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))?),
        }
    }

    /// Get the marker width from the loaded .iset data.
    pub fn get_marker_width(&self) -> i32 {
        self.marker_width
    }

    /// Get the marker height from the loaded .iset data.
    pub fn get_marker_height(&self) -> i32 {
        self.marker_height
    }

    /// Get the marker DPI from the loaded .iset data.
    pub fn get_marker_dpi(&self) -> f32 {
        self.marker_dpi
    }

    /// Check whether an NFT marker has been loaded.
    pub fn is_loaded(&self) -> bool {
        self.loaded
    }

    /// Get the current tracking continuity count.
    pub fn get_cont_num(&self) -> i32 {
        self.surface_set.cont_num
    }

    /// Reset tracking state (forces re-detection).
    pub fn reset_tracking(&mut self) {
        self.surface_set.cont_num = 0;
    }

    /// Get camera intrinsic parameters `[fx, fy, cx, cy]` from `camera_para.dat`.
    pub fn get_camera_intrinsics(&self) -> Box<[f32]> {
        unsafe {
            if !self.ar2_handle.cparam_lt.is_null() {
                let mat = &(*self.ar2_handle.cparam_lt).param.mat;
                vec![
                    mat[0][0] as f32,
                    mat[1][1] as f32,
                    mat[0][2] as f32,
                    mat[1][2] as f32,
                ]
                .into_boxed_slice()
            } else {
                vec![0.0, 0.0, 0.0, 0.0].into_boxed_slice()
            }
        }
    }

    /// Get the 4x4 OpenGL projection matrix for Three.js / WebGL rendering.
    pub fn get_projection_matrix(&self, near: f32, far: f32) -> Box<[f32]> {
        unsafe {
            if self.ar2_handle.cparam_lt.is_null() {
                return vec![0.0; 16].into_boxed_slice();
            }
            let param = &(*self.ar2_handle.cparam_lt).param;
            let fx = param.mat[0][0] as f32;
            let fy = param.mat[1][1] as f32;
            let cx = param.mat[0][2] as f32;
            let cy = param.mat[1][2] as f32;
            let width = param.xsize as f32;
            let height = param.ysize as f32;

            let mut proj = vec![0.0f32; 16];
            proj[0] = 2.0 * fx / width;
            proj[5] = 2.0 * fy / height;
            proj[8] = (2.0 * cx / width) - 1.0;
            proj[9] = (2.0 * cy / height) - 1.0;
            proj[10] = -(far + near) / (far - near);
            proj[11] = -1.0;
            proj[14] = -(2.0 * far * near) / (far - near);
            proj.into_boxed_slice()
        }
    }
}

impl Drop for WasmNFTHandle {
    fn drop(&mut self) {
        unsafe {
            if !self.ar2_handle.cparam_lt.is_null() {
                let _ = Box::from_raw(self.ar2_handle.cparam_lt);
                self.ar2_handle.cparam_lt = std::ptr::null_mut();
            }
            if !self.ar2_handle.icp_handle.is_null() {
                let _ = Box::from_raw(self.ar2_handle.icp_handle);
                self.ar2_handle.icp_handle = std::ptr::null_mut();
            }
        }
    }
}

/// Result of [`WasmKpmHandle::detect`]: the detected 3×4 pose (row-major, 12
/// floats), the matched page number, and the matching error.
#[derive(serde::Serialize)]
struct KpmDetectResult {
    pose: [f32; 12],
    page: i32,
    error: f32,
}

/// KPM (Keypoint Matching) detection handle — the WASM equivalent of
/// `simple_nft.rs` steps 3a + 4. Loads NFT reference data (`.fset3`) and
/// detects the marker's initial 3×4 pose in a query frame using the pure-Rust
/// [`RustFreakMatcher`].
///
/// Pair with [`WasmNFTHandle`] for AR2 tracking: feed `detect()`'s pose into
/// [`WasmNFTHandle::set_initial_pose`].
#[wasm_bindgen]
pub struct WasmKpmHandle {
    handle: KpmHandle,
    width: i32,
    height: i32,
    loaded: bool,
}

#[wasm_bindgen]
impl WasmKpmHandle {
    /// Create a KPM detection handle.
    ///
    /// * `param_bytes` — `camera_para.dat` contents.
    /// * `width` / `height` — query-frame size in pixels.
    #[wasm_bindgen(constructor)]
    pub fn new(param_bytes: &[u8], width: i32, height: i32) -> Result<WasmKpmHandle, JsValue> {
        let cursor = Cursor::new(param_bytes);
        let mut param = ARParam::load(cursor)
            .map_err(|e| JsValue::from_str(&format!("Failed to load camera param: {}", e)))?;

        // Scale camera parameters to match the requested frame size.
        // For digital cameras with square pixels, scale focal lengths isotropically
        // (based on height) to preserve fx == fy, and adjust optical center proportionally.
        let scale = height as f64 / param.ysize as f64;
        let orig_cx = param.mat[0][2];
        let orig_cy = param.mat[1][2];

        param.mat[0][0] *= scale;
        param.mat[0][1] *= scale;
        param.mat[1][0] *= scale;
        param.mat[1][1] *= scale;

        param.mat[0][2] = (orig_cx / param.xsize as f64) * width as f64;
        param.mat[1][2] = (orig_cy / param.ysize as f64) * height as f64;

        param.xsize = width;
        param.ysize = height;

        let param_lt = Arc::new(ARParamLT::new_basic(param));
        let backend = RustFreakMatcher::new(width, height)
            .map_err(|e| JsValue::from_str(&format!("Failed to create FREAK matcher: {:?}", e)))?;
        let handle = KpmHandle::new(width, height, Some(param_lt), Box::new(backend));

        Ok(WasmKpmHandle {
            handle,
            width,
            height,
            loaded: false,
        })
    }

    /// Load NFT reference data from `.fset3` bytes (KPM reference keypoints).
    /// All pages are remapped to page 0 (single-marker setup).
    pub fn load_ref_data(&mut self, fset3_bytes: &[u8]) -> Result<(), JsValue> {
        let mut ref_data = KpmRefDataSet::load_from_bytes(fset3_bytes)
            .map_err(|e| JsValue::from_str(&format!("Failed to load .fset3: {}", e)))?;
        ref_data.change_page_no(KPM_CHANGE_PAGE_NO_ALL_PAGES, 0);
        self.handle
            .set_ref_data_set(ref_data)
            .map_err(|e| JsValue::from_str(&format!("Failed to set ref data: {:?}", e)))?;
        self.loaded = true;
        Ok(())
    }

    /// Run KPM detection on an RGBA frame (`width * height * 4` bytes, e.g. a
    /// canvas `ImageData.data` buffer).
    ///
    /// Returns `{ pose: number[12], page, error }` on a match, or `null` if no
    /// marker was found.
    pub fn detect(&mut self, rgba_bytes: &[u8]) -> Result<JsValue, JsValue> {
        if !self.loaded {
            return Err(JsValue::from_str(
                "reference data not loaded — call load_ref_data first",
            ));
        }
        let expected = (self.width * self.height * 4) as usize;
        if rgba_bytes.len() != expected {
            return Err(JsValue::from_str(&format!(
                "rgba length {} != expected {} ({}x{}x4)",
                rgba_bytes.len(),
                expected,
                self.width,
                self.height
            )));
        }

        let luma = rgba_to_gray(rgba_bytes);
        self.handle
            .kpm_matching(&luma)
            .map_err(|e| JsValue::from_str(&format!("kpm_matching failed: {:?}", e)))?;

        match self.handle.get_pose() {
            Some((cam_pose, page, error)) => {
                let mut pose = [0f32; 12];
                for (r, row) in cam_pose.iter().enumerate() {
                    pose[r * 4..r * 4 + 4].copy_from_slice(row);
                }
                let result = KpmDetectResult { pose, page, error };
                serde_wasm_bindgen::to_value(&result)
                    .map_err(|e| JsValue::from_str(&format!("serialize failed: {}", e)))
            }
            None => Ok(JsValue::NULL),
        }
    }

    /// Whether reference data has been loaded.
    pub fn is_loaded(&self) -> bool {
        self.loaded
    }

    /// Get camera intrinsic parameters `[fx, fy, cx, cy]` from `camera_para.dat`.
    pub fn get_camera_intrinsics(&self) -> Box<[f32]> {
        if let Some(param_lt) = &self.handle.cparam_lt {
            let mat = &param_lt.param.mat;
            vec![
                mat[0][0] as f32,
                mat[1][1] as f32,
                mat[0][2] as f32,
                mat[1][2] as f32,
            ]
            .into_boxed_slice()
        } else {
            vec![0.0, 0.0, 0.0, 0.0].into_boxed_slice()
        }
    }

    /// Get the 4x4 OpenGL projection matrix for Three.js / WebGL rendering.
    pub fn get_projection_matrix(&self, near: f32, far: f32) -> Box<[f32]> {
        if let Some(param_lt) = &self.handle.cparam_lt {
            let param = &param_lt.param;
            let fx = param.mat[0][0] as f32;
            let fy = param.mat[1][1] as f32;
            let cx = param.mat[0][2] as f32;
            let cy = param.mat[1][2] as f32;
            let width = param.xsize as f32;
            let height = param.ysize as f32;

            let mut proj = vec![0.0f32; 16];
            proj[0] = 2.0 * fx / width;
            proj[5] = 2.0 * fy / height;
            proj[8] = (2.0 * cx / width) - 1.0;
            proj[9] = (2.0 * cy / height) - 1.0;
            proj[10] = -(far + near) / (far - near);
            proj[11] = -1.0;
            proj[14] = -(2.0 * far * near) / (far - near);
            proj.into_boxed_slice()
        } else {
            vec![0.0; 16].into_boxed_slice()
        }
    }
}
