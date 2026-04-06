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
//! cargo run -p webarkitlib-rs --features ffi-backend --example simple_nft
//! ```

use std::io::Cursor;
use std::path::Path;
use std::sync::Arc;

use webarkitlib_rs::ar2::{
    ar2_tracking, AR2FeatureCoord, AR2FeaturePoints, AR2FeatureSet, AR2FeatureSetT, AR2Handle,
    AR2Image, AR2ImageSet, AR2ImageSetT, AR2Surface, AR2SurfaceSet, AR2_BLUR_IMAGE_MAX,
};
use webarkitlib_rs::icp::icp_create_handle;
use webarkitlib_rs::kpm::types::KpmRefDataSet;
use webarkitlib_rs::kpm::{CppFreakMatcher, KpmHandle};
use webarkitlib_rs::types::{ARParam, ARParamLT, ARPixelFormat};

fn main() {
    let data_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("Data");

    println!("========================================");
    println!("  WebARKitLib-rs — Simple NFT Example");
    println!("========================================\n");

    // ---------------------------------------------------------------
    // Step 1: Load camera parameters
    // ---------------------------------------------------------------
    let param_path = data_dir.join("camera_para.dat");
    println!("Step 1: Loading camera parameters...");
    let param_bytes = std::fs::read(&param_path).expect("failed to read camera_para.dat");
    let mut param =
        ARParam::load(Cursor::new(&param_bytes)).expect("failed to parse camera params");
    println!(
        "  Camera (original): {}x{}, mat[0][0]={:.2}",
        param.xsize, param.ysize, param.mat[0][0]
    );

    // ---------------------------------------------------------------
    // Step 2: Load test image (pinball demo) as grayscale
    //
    // We decode the JPEG directly and convert RGB → luma using BT.601,
    // then scale the camera parameters to match the image size.
    // ---------------------------------------------------------------
    let img_path = data_dir.join("pinball-demo.jpg");
    println!("\nStep 2: Loading test image...");
    let jpeg_bytes = std::fs::read(&img_path).expect("failed to read pinball-demo.jpg");
    let mut decoder = jpeg_decoder::Decoder::new(Cursor::new(&jpeg_bytes));
    let pixels = decoder.decode().expect("JPEG decode failed");
    let info = decoder.info().expect("no JPEG info");
    let width = info.width as i32;
    let height = info.height as i32;
    println!("  Image: {}x{}", width, height);

    // Convert RGB → luma using BT.601 integer formula.
    let luma: Vec<u8> = pixels
        .chunks_exact(3)
        .map(|rgb| ((rgb[0] as u32 * 77 + rgb[1] as u32 * 150 + rgb[2] as u32 * 29) >> 8) as u8)
        .collect();
    println!("  Luma: {} bytes", luma.len());

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
    println!(
        "  Camera (scaled):   {}x{}, mat[0][0]={:.2}",
        param.xsize, param.ysize, param.mat[0][0]
    );

    // ---------------------------------------------------------------
    // Step 3: Load NFT marker data
    // ---------------------------------------------------------------
    let marker_name = "pinball";
    println!("\nStep 3: Loading NFT marker '{}'...", marker_name);

    // 3a: .fset3 (KPM reference data)
    let fset3_path = data_dir.join(format!("{}.fset3", marker_name));
    let mut ref_data = KpmRefDataSet::load(&fset3_path).expect("failed to load .fset3");
    // Assign page 0 (matches C++ multi-marker setup for single marker).
    ref_data.change_page_no(
        webarkitlib_rs::kpm::ref_data_set::KPM_CHANGE_PAGE_NO_ALL_PAGES,
        0,
    );
    println!(
        "  .fset3: {} features, {} pages",
        ref_data.num, ref_data.page_num
    );

    // 3b: .iset (image pyramid)
    let iset_path = data_dir.join(format!("{}.iset", marker_name));
    let image_set = AR2ImageSetT::load(&iset_path).expect("failed to load .iset");
    let base = &image_set.scale[0];
    println!(
        "  .iset:  {} scales, base {}x{} @ {:.0} DPI",
        image_set.num(),
        base.xsize,
        base.ysize,
        base.dpi
    );

    // 3c: .fset (AR2 features)
    let fset_path = data_dir.join(format!("{}.fset", marker_name));
    let feature_set = AR2FeatureSetT::load(&fset_path).expect("failed to load .fset");
    let total_features: usize = feature_set.list.iter().map(|fp| fp.num()).sum();
    println!(
        "  .fset:  {} scales, {} total features",
        feature_set.num(),
        total_features
    );

    // ---------------------------------------------------------------
    // Step 4: KPM Detection — find the marker in the image
    // ---------------------------------------------------------------
    println!("\nStep 4: Running KPM detection...");

    let param_lt = ARParamLT::new_basic(param.clone());
    let param_lt_arc = Arc::new(param_lt);

    let backend =
        CppFreakMatcher::new(width, height).expect("failed to create CppFreakMatcher backend");
    let mut kpm_handle = KpmHandle::new(width, height, Some(param_lt_arc.clone()), Box::new(backend));

    kpm_handle
        .set_ref_data_set(ref_data)
        .expect("failed to set ref data set");
    println!("  Reference data loaded into KPM backend.");

    kpm_handle
        .kpm_matching(&luma)
        .expect("kpm_matching failed");

    let pose = kpm_handle.get_pose();
    match pose {
        Some((cam_pose, page_no, error)) => {
            println!("  ✓ KPM match found! Page={}, error={:.4}", page_no, error);
            println!("  Initial 3×4 pose matrix:");
            for r in 0..3 {
                println!(
                    "    [{:>10.4} {:>10.4} {:>10.4} {:>10.4}]",
                    cam_pose[r][0], cam_pose[r][1], cam_pose[r][2], cam_pose[r][3]
                );
            }

            // ---------------------------------------------------------------
            // Step 5: AR2 Tracking — refine the pose
            // ---------------------------------------------------------------
            println!("\nStep 5: Running AR2 tracking (pose refinement)...");

            // Convert I/O types to tracking types.
            let tracking_image_set = convert_image_set(&image_set);
            let tracking_feature_set = convert_feature_set(&feature_set);

            // Build surface with identity transform (single marker).
            let mut identity = [[0.0f32; 4]; 3];
            identity[0][0] = 1.0;
            identity[1][1] = 1.0;
            identity[2][2] = 1.0;

            let surface = AR2Surface {
                image_set: Some(tracking_image_set),
                feature_set: Some(tracking_feature_set),
                trans: identity,
                itrans: identity,
            };

            let mut surface_set = AR2SurfaceSet {
                surface: vec![surface],
                trans1: *cam_pose,   // Initial pose from KPM
                trans2: *cam_pose,
                trans3: *cam_pose,
                cont_num: 1,         // Mark as having one continuous frame
                ..Default::default()
            };

            // Create AR2Handle with camera params and ICP.
            let mut ar2_handle = AR2Handle::new(width, height, ARPixelFormat::MONO);

            // Set up camera parameters pointer.
            let param_lt_for_ar2 = Box::new(ARParamLT::new_basic(param.clone()));
            ar2_handle.cparam_lt = Box::into_raw(param_lt_for_ar2);

            // Set up ICP handle.
            let icp_handle_ptr = icp_create_handle(&param.mat)
                .expect("failed to create ICP handle");
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
                    println!("  ✓ AR2 tracking succeeded! Error={:.4}", tracking_err);
                    println!("  Refined 3×4 pose matrix:");
                    for r in 0..3 {
                        println!(
                            "    [{:>10.4} {:>10.4} {:>10.4} {:>10.4}]",
                            refined_pose[r][0],
                            refined_pose[r][1],
                            refined_pose[r][2],
                            refined_pose[r][3]
                        );
                    }
                }
                Err(code) => {
                    println!("  ✗ AR2 tracking returned error code: {}", code);
                    println!("    (This is expected for a single static frame —");
                    println!("     AR2 tracking typically needs multi-frame continuity.)");
                    println!("  The KPM initial pose above is still valid and usable.");
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
            println!("  ✗ No KPM match found.");
            println!("    The marker may not be visible in the test image,");
            println!("    or the reference data may not match this image.");
        }
    }

    println!("\n========================================");
    println!("  Simple NFT example complete.");
    println!("========================================");
}

/// Convert an `AR2ImageSetT` (I/O type) to `AR2ImageSet` (tracking type).
///
/// The tracking module expects `AR2Image` with `img_bw_blur` containing
/// multiple blur levels. We populate blur level 0 with the raw pixels
/// and leave higher levels as `None`.
fn convert_image_set(io_set: &AR2ImageSetT) -> AR2ImageSet {
    let mut scales = Vec::with_capacity(io_set.scale.len());

    for io_img in &io_set.scale {
        let mut blur_levels: Vec<Option<Vec<u8>>> = Vec::with_capacity(AR2_BLUR_IMAGE_MAX);

        // Level 0 = original image pixels.
        blur_levels.push(Some(io_img.img_bw.clone()));

        // Fill remaining levels with None.
        for _ in 1..AR2_BLUR_IMAGE_MAX {
            blur_levels.push(None);
        }

        scales.push(AR2Image {
            img_bw_blur: blur_levels,
            xsize: io_img.xsize,
            ysize: io_img.ysize,
            dpi: io_img.dpi,
        });
    }

    AR2ImageSet { scale: scales }
}

/// Convert an `AR2FeatureSetT` (I/O type) to `AR2FeatureSet` (tracking type).
fn convert_feature_set(io_set: &AR2FeatureSetT) -> AR2FeatureSet {
    let mut list = Vec::with_capacity(io_set.list.len());

    for io_fp in &io_set.list {
        let coord: Vec<AR2FeatureCoord> = io_fp
            .coord
            .iter()
            .map(|c| AR2FeatureCoord {
                x: c.x,
                y: c.y,
                mx: c.mx,
                my: c.my,
                max_sim: c.max_sim,
            })
            .collect();

        list.push(AR2FeaturePoints {
            coord,
            scale: io_fp.scale,
            maxdpi: io_fp.maxdpi,
            mindpi: io_fp.mindpi,
        });
    }

    AR2FeatureSet { list }
}
