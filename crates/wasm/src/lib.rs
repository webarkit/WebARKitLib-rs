use wasm_bindgen::prelude::*;
use core::types::{ARHandle, ARParam, ARPixelFormat, AR2VideoBufferT, AR3DHandle, ARPattHandle, ARParamLT, ARLabelingThreshMode};
use core::image_proc::ARImageProcInfo;
use core::marker::ar_detect_marker;
use core::pose::{ar_3d_create_handle, ar_3d_delete_handle, ar_get_trans_mat_square};
use core::pattern::ar_patt_load_from_buffer;
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
                id: marker.id,
                cf: marker.cf as f32,
                pos: [marker.pos[0] as f32, marker.pos[1] as f32],
            });
        }
        
        Ok(serde_wasm_bindgen::to_value(&results)
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))?)
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

#[derive(serde::Serialize)]
pub struct MarkerResult {
    pub id: i32,
    pub cf: f32,
    pub pos: [f32; 2],
}

#[derive(serde::Serialize)]
pub struct PoseResult {
    pub matrix: Vec<f32>,
    pub icp_error: f32,
}

fn rgba_to_gray(rgba: &[u8]) -> Vec<u8> {
    let mut gray = Vec::with_capacity(rgba.len() / 4);
    for chunk in rgba.chunks_exact(4) {
        // Use more precise BT.601 coefficients
        let r = chunk[0] as f32;
        let g = chunk[1] as f32;
        let b = chunk[2] as f32;
        gray.push((0.2989 * r + 0.5870 * g + 0.1140 * b + 0.5) as u8);
    }
    gray
}
