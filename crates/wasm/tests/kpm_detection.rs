/*
 *  kpm_detection.rs
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

//! Headless wasm-bindgen tests for the KPM detection binding (#161).
//!
//! Run with: `wasm-pack test --node` (or `--headless --chrome`) from
//! `crates/wasm`. These exercise `WasmKpmHandle` in a real wasm runtime —
//! construction (camera-param parse + FREAK matcher init) and reference-data
//! loading from in-memory `.fset3` bytes (`KpmRefDataSet::load_from_bytes`).
//!
//! Full `detect()` is covered end-to-end by the native `simple_nft` pipeline
//! (same pure-Rust code) and the browser demo (`www/simple_nft_example.html`);
//! it needs a decoded RGBA frame, out of scope for this lightweight runtime
//! check.

#![cfg(target_arch = "wasm32")]

use wasm_bindgen_test::*;
use webarkitlib_wasm::WasmKpmHandle;

/// Demo assets, baked into the test binary.
const CAMERA_PARA: &[u8] = include_bytes!("../www/assets/camera_para.dat");
const PINBALL_FSET3: &[u8] = include_bytes!("../www/assets/pinball.fset3");

#[wasm_bindgen_test]
fn kpm_handle_constructs_and_loads_ref_data() {
    let mut handle =
        WasmKpmHandle::new(CAMERA_PARA, 640, 480).expect("WasmKpmHandle::new should succeed");
    assert!(!handle.is_loaded(), "should start unloaded");

    handle
        .load_ref_data(PINBALL_FSET3)
        .expect("load_ref_data should parse the .fset3 bytes");
    assert!(handle.is_loaded(), "should be loaded after load_ref_data");
}

#[wasm_bindgen_test]
fn detect_before_load_errors() {
    let mut handle = WasmKpmHandle::new(CAMERA_PARA, 4, 4).expect("new");
    // 4x4x4 RGBA; detection must refuse before reference data is loaded.
    let rgba = [0u8; 4 * 4 * 4];
    assert!(
        handle.detect(&rgba).is_err(),
        "detect should error before load_ref_data"
    );
}
