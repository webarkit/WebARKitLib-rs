/*
 *  simple_nft.rs
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

//! Simple NFT (Natural Feature Tracking) example.
//!
//! Demonstrates the complete KPM detection → AR2 tracking pipeline on a
//! single static image. This is the Rust equivalent of the NFT tracking
//! flow in jsartoolkitNFT.
//!
//! The pipeline:
//!
//! ```text
//! pinball-demo.jpg ──► KPM detection ──► initial 3×4 pose
//!                      (kpm_matching)        │
//!                                            ▼
//!                                     AR2 tracking ──► refined 3×4 pose
//!                                     (ar2_tracking)
//! ```
//!
//! Run with:
//!
//! ```sh
//! cargo run -p webarkitlib-rs --example simple_nft
//! ```
//!
//! Uses [`RustFreakMatcher`] (M9-2, #141) — the pure-Rust FreakMatcher
//! backend over the assembled M6–M8 + M9-1 pipeline. This example is the
//! end-to-end integration signal for the Rust backend: if `cargo run
//! --example simple_nft` produces a sane pose, the pure-Rust pipeline
//! composes correctly with the AR2 tracker.

use std::io::Cursor;
use std::path::Path;
use std::sync::Arc;

use webarkitlib_rs::ar2::{
    ar2_read_surface_set, ar2_surface_set_marker_info, ar2_tracking, AR2Handle,
};
use webarkitlib_rs::icp::icp_create_handle;
use webarkitlib_rs::kpm::types::KpmRefDataSet;
use webarkitlib_rs::kpm::{KpmHandle, RustFreakMatcher};
use webarkitlib_rs::types::{ARParam, ARParamLT, ARPixelFormat};
use webarkitlib_rs::{arlog_i, arlog_rel, arlog_w};

fn main() {
    webarkitlib_rs::arlog::ar_log_init_default();

    let data_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("Data");

    arlog_rel!("========================================");
    arlog_rel!("  WebARKitLib-rs — Simple NFT Example");
    arlog_rel!("========================================");
    arlog_rel!("");

    // ---------------------------------------------------------------
    // Step 1: Load camera parameters
    // ---------------------------------------------------------------
    let param_path = data_dir.join("camera_para.dat");
    arlog_i!("Step 1: Loading camera parameters...");
    let param_bytes = std::fs::read(&param_path).expect("failed to read camera_para.dat");
    let mut param =
        ARParam::load(Cursor::new(&param_bytes)).expect("failed to parse camera params");
    arlog_i!(
        "  Camera (original): {}x{}, mat[0][0]={:.2}",
        param.xsize,
        param.ysize,
        param.mat[0][0]
    );

    // ---------------------------------------------------------------
    // Step 2: Load test image (pinball demo) as grayscale
    //
    // We decode the JPEG directly and convert RGB → luma using BT.601,
    // then scale the camera parameters to match the image size.
    // ---------------------------------------------------------------
    let img_path = data_dir.join("pinball-demo.jpg");
    arlog_rel!("");
    arlog_i!("Step 2: Loading test image...");
    let jpeg_bytes = std::fs::read(&img_path).expect("failed to read pinball-demo.jpg");
    let mut decoder = jpeg_decoder::Decoder::new(Cursor::new(&jpeg_bytes));
    let pixels = decoder.decode().expect("JPEG decode failed");
    let info = decoder.info().expect("no JPEG info");
    let width = info.width as i32;
    let height = info.height as i32;
    arlog_i!("  Image: {}x{}", width, height);

    // Convert RGB → luma using BT.601 integer formula.
    let luma: Vec<u8> = pixels
        .chunks_exact(3)
        .map(|rgb| ((rgb[0] as u32 * 77 + rgb[1] as u32 * 150 + rgb[2] as u32 * 29) >> 8) as u8)
        .collect();
    arlog_i!("  Luma: {} bytes", luma.len());

    // Scale camera parameters to match image dimensions
    // (equivalent to arParamChangeSize in the C++ code).
    let sx = width as f64 / param.xsize as f64;
    let sy = height as f64 / param.ysize as f64;
    for col in 0..4 {
        param.mat[0][col] *= sx;
        param.mat[1][col] *= sy;
    }
    param.xsize = width;
    param.ysize = height;
    arlog_i!(
        "  Camera (scaled):   {}x{}, mat[0][0]={:.2}",
        param.xsize,
        param.ysize,
        param.mat[0][0]
    );

    // ---------------------------------------------------------------
    // Step 3: Load NFT marker data
    //
    // Following the nftSimple.c pattern:
    //   1. Load .fset3 via kpmLoadRefDataSet()
    //   2. Load surface set via ar2ReadSurfaceSet() which internally
    //      loads both .iset and .fset from the base name.
    //
    // See: https://github.com/webarkit/artoolkit5/blob/66aa1cc12e1bdeb12ee6af5746dc4ff6f3ba34cb/examples/nftSimple/nftSimple.c#L420-L479
    // ---------------------------------------------------------------
    let marker_name = "pinball";
    let marker_base = data_dir.join(marker_name);
    arlog_rel!("");
    arlog_i!("Step 3: Loading NFT marker '{}'...", marker_name);

    // 3a: .fset3 (KPM reference data)
    let fset3_path = data_dir.join(format!("{}.fset3", marker_name));
    let mut ref_data = KpmRefDataSet::load(&fset3_path).expect("failed to load .fset3");
    // Assign page 0 (matches C++ multi-marker setup for single marker).
    ref_data.change_page_no(
        webarkitlib_rs::kpm::ref_data_set::KPM_CHANGE_PAGE_NO_ALL_PAGES,
        0,
    );
    arlog_i!(
        "  .fset3: {} features, {} pages",
        ref_data.num,
        ref_data.page_num
    );

    // 3b: Load surface set (reads both .iset and .fset internally).
    //   C++: surfaceSet = ar2ReadSurfaceSet(datasetPathname, "fset", NULL)
    let mut surface_set =
        ar2_read_surface_set(&marker_base).expect("failed to load surface set (.iset + .fset)");
    if let Some((w, h, dpi)) = ar2_surface_set_marker_info(&surface_set) {
        arlog_i!("  Surface set: marker {}x{} @ {:.0} DPI", w, h, dpi);
    }
    let total_features: usize = surface_set
        .surface
        .iter()
        .filter_map(|s| s.feature_set.as_ref())
        .flat_map(|fs| &fs.list)
        .map(|fp| fp.coord.len())
        .sum();
    let num_scales = surface_set
        .surface
        .first()
        .and_then(|s| s.image_set.as_ref())
        .map(|is| is.scale.len())
        .unwrap_or(0);
    arlog_i!(
        "  Surface set: {} image scales, {} total features",
        num_scales,
        total_features
    );

    // ---------------------------------------------------------------
    // Step 4: KPM Detection — find the marker in the image
    // ---------------------------------------------------------------
    arlog_rel!("");
    arlog_i!("Step 4: Running KPM detection...");

    let param_lt = ARParamLT::new_basic(param.clone());
    let param_lt_arc = Arc::new(param_lt);

    let backend =
        RustFreakMatcher::new(width, height).expect("failed to create RustFreakMatcher backend");
    let mut kpm_handle =
        KpmHandle::new(width, height, Some(param_lt_arc.clone()), Box::new(backend));

    kpm_handle
        .set_ref_data_set(ref_data)
        .expect("failed to set ref data set");
    arlog_i!("  Reference data loaded into KPM backend.");

    kpm_handle.kpm_matching(&luma).expect("kpm_matching failed");

    let pose = kpm_handle.get_pose();
    match pose {
        Some((cam_pose, page_no, error)) => {
            arlog_i!("  ✓ KPM match found! Page={}, error={:.4}", page_no, error);
            arlog_i!("  Initial 3×4 pose matrix:");
            for row in cam_pose {
                arlog_i!(
                    "    [{:>10.4} {:>10.4} {:>10.4} {:>10.4}]",
                    row[0],
                    row[1],
                    row[2],
                    row[3]
                );
            }

            // ---------------------------------------------------------------
            // Step 5: AR2 Tracking — refine the pose
            //
            //   C++: ar2SetInitTrans(surfaceSet, trackingTrans);
            //        ar2Tracking(ar2Handle, surfaceSet, buff, trackingTrans, &err);
            // ---------------------------------------------------------------
            arlog_rel!("");
            arlog_i!("Step 5: Running AR2 tracking (pose refinement)...");

            // Set initial pose from KPM detection.
            //   C++: ar2SetInitTrans(surfaceSet[detectedPage], trackingTrans);
            surface_set.set_init_trans(cam_pose);

            // Create AR2Handle with camera params and ICP.
            let mut ar2_handle = AR2Handle::new(width, height, ARPixelFormat::MONO);

            // Set up camera parameters pointer.
            let param_lt_for_ar2 = Box::new(ARParamLT::new_basic(param.clone()));
            ar2_handle.cparam_lt = Box::into_raw(param_lt_for_ar2);

            // Set up ICP handle.
            let icp_handle_ptr =
                icp_create_handle(&param.mat).expect("failed to create ICP handle");
            ar2_handle.icp_handle = icp_handle_ptr;

            // Run AR2 tracking.
            let mut refined_pose = *cam_pose;
            let mut tracking_err = 0.0f32;

            match ar2_tracking(
                &mut ar2_handle,
                &mut surface_set,
                &luma,
                &mut refined_pose,
                &mut tracking_err,
            ) {
                Ok(()) => {
                    arlog_i!("  ✓ AR2 tracking succeeded! Error={:.4}", tracking_err);
                    arlog_i!("  Refined 3×4 pose matrix:");
                    for row in &refined_pose {
                        arlog_i!(
                            "    [{:>10.4} {:>10.4} {:>10.4} {:>10.4}]",
                            row[0],
                            row[1],
                            row[2],
                            row[3]
                        );
                    }
                }
                Err(code) => {
                    arlog_w!("  ✗ AR2 tracking returned error code: {}", code);
                    arlog_i!("    (This is expected for a single static frame —");
                    arlog_i!("     AR2 tracking typically needs multi-frame continuity.)");
                    arlog_i!("  The KPM initial pose above is still valid and usable.");
                }
            }

            // Clean up raw pointers.
            unsafe {
                if !ar2_handle.cparam_lt.is_null() {
                    let _ = Box::from_raw(ar2_handle.cparam_lt);
                    ar2_handle.cparam_lt = std::ptr::null_mut();
                }
                if !ar2_handle.icp_handle.is_null() {
                    let _ = Box::from_raw(ar2_handle.icp_handle);
                    ar2_handle.icp_handle = std::ptr::null_mut();
                }
            }
        }
        None => {
            arlog_w!("  ✗ No KPM match found.");
            arlog_i!("    The marker may not be visible in the test image,");
            arlog_i!("    or the reference data may not match this image.");
        }
    }

    arlog_rel!("");
    arlog_rel!("========================================");
    arlog_rel!("  Simple NFT example complete.");
    arlog_rel!("========================================");
}
