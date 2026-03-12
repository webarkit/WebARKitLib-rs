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
//! - `init_panic_hook`: Utility to get better Rust panic messages in the browser console.

use wasm_bindgen::prelude::*;
use webarkitlib_rs::types::{ARHandle, ARParam, ARPixelFormat, AR2VideoBufferT, AR3DHandle, ARPattHandle, ARParamLT, ARLabelingThreshMode, ARMatrixCodeType};
use webarkitlib_rs::image_proc::{ARImageProcInfo, rgba_to_gray};
use webarkitlib_rs::marker::ar_detect_marker;
use webarkitlib_rs::pose::{ar_3d_create_handle, ar_3d_delete_handle, ar_get_trans_mat_square};
use webarkitlib_rs::pattern::ar_patt_load_from_buffer;
use std::io::Cursor;

#[wasm_bindgen]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
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
        
        let ar3d_handle = ar_3d_create_handle(&param)
            .map_err(|e| JsValue::from_str(e))?;

        let mut handle = ARHandle::new(param.clone());
        handle.set_pixel_format(ARPixelFormat::RGBA);

        // Initialize pattern handle
        let patt_handle = Box::into_raw(Box::new(ARPattHandle::new(16, 25)));
        handle.patt_handle = patt_handle;
        
        // Initialize lookup table handle
        let ar_param_lt = Box::into_raw(Box::new(ARParamLT::new_basic(param.clone())));
        handle.ar_param_lt = ar_param_lt;
        
        Ok(WasmARHandle { handle, ar3d_handle, param })
    }

    pub fn load_pattern(&mut self, patt_content: &str) -> Result<i32, JsValue> {
        if self.handle.patt_handle.is_null() {
            return Err(JsValue::from_str("Pattern handle is null"));
        }
        let patt_handle = unsafe { &mut *self.handle.patt_handle };
        let idx = ar_patt_load_from_buffer(patt_handle, patt_content)
            .map_err(|e| JsValue::from_str(e))?;
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
            3   => ARMatrixCodeType::Code3x3,
            259 => ARMatrixCodeType::Code3x3Parity65,
            515 => ARMatrixCodeType::Code3x3Hamming63,
            4   => ARMatrixCodeType::Code4x4,
            772 => ARMatrixCodeType::Code4x4BCH1393,
            1028 => ARMatrixCodeType::Code4x4BCH1355,
            5   => ARMatrixCodeType::Code5x5,
            1285 => ARMatrixCodeType::Code5x5BCH22125,
            1541 => ARMatrixCodeType::Code5x5BCH2277,
            6   => ARMatrixCodeType::Code6x6,
            _   => {
                web_sys::console::warn_1(&JsValue::from_str(&format!(
                    "[WebARKit] Unknown matrix code type {code_type}, falling back to Code3x3"
                )));
                ARMatrixCodeType::Code3x3
            }
        };
        self.handle.set_matrix_code_type(ct);
    }

    pub fn detect_markers(&mut self, frame: &[u8], width: i32, height: i32) -> Result<JsValue, JsValue> {
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
        if matches!(self.handle.ar_labeling_thresh_mode, ARLabelingThreshMode::AutoOtsu) {
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
        
        ar_detect_marker(&mut self.handle, &video_buffer)
            .map_err(|e| JsValue::from_str(e))?;
            
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
        
        Ok(serde_wasm_bindgen::to_value(&results)
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))?)
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
            .map_err(|e| JsValue::from_str(e))?;
            
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

        Ok(serde_wasm_bindgen::to_value(&result)
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))?)
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

