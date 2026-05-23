/*
 *  simple_nft_dual.rs
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

//! Diagnostic NFT example exercising [`DualFreakMatcher`].
//!
//! This is the **diagnostic** sibling of [`simple_nft.rs`](./simple_nft.rs);
//! `simple_nft.rs` is the **production** example. Both run the same KPM
//! detection → AR2 tracking flow on the pinball reference image, but this
//! one drives [`DualFreakMatcher`] (M9-2, #141) so the C++ and pure-Rust
//! FREAK backends can be compared on the same query.
//!
//! What you should see (on the bundled pinball assets, modulo float noise
//! across machines):
//!
//! - Both backends agree on the matched id (no tier-1 / matched_id divergence).
//! - Per-backend KPM error matching `docs/design/m9-2-rust-backend.md` §10
//!   (~7.14 C++ / ~5.09 Rust).
//! - Two homographies printed side-by-side; their corner reprojections
//!   typically diverge by single-digit-to-low-double-digit pixels on
//!   pinball — this is the cross-language BHC-variance envelope §10
//!   discusses, and it is **larger** than the M9 #152 tier-2 tolerance
//!   (2.0 px) so a non-zero `divergence_count` is the normal observation
//!   on this dataset. The milestone-gate test
//!   (`test_dual_mode_no_divergence_on_pinball`) covers the `found.jpg`/
//!   `img.jpg` pair where divergence stays at zero.
//!
//! Read non-zero divergences here as informational, not as a regression
//! — the C++ pose still feeds AR2 cleanly (M9-2 D5).
//!
//! After M9-3 flips the default backend off `ffi-backend` (#142), this
//! example becomes a niche debugging tool — most useful right now,
//! while both backends are still routinely available.
//!
//! Run with:
//!
//! ```sh
//! cargo run -p webarkitlib-rs --example simple_nft_dual
//! # or, explicit:
//! cargo run -p webarkitlib-rs --features "dual-mode log-helpers" --example simple_nft_dual
//! ```

use std::io::Cursor;
use std::path::Path;
use std::sync::Arc;

use webarkitlib_rs::ar2::{
    ar2_read_surface_set, ar2_surface_set_marker_info, ar2_tracking, AR2Handle,
};
use webarkitlib_rs::arlog::ar_log_init_default;
use webarkitlib_rs::icp::icp_create_handle;
use webarkitlib_rs::kpm::types::{KpmRefDataSet, FREAK_SUB_DIMENSION};
use webarkitlib_rs::kpm::{
    CppFreakMatcher, DualFreakMatcher, FeaturePoint, FreakMatcherBackend, KpmHandle, Point3d,
};
use webarkitlib_rs::types::{ARParam, ARParamLT, ARPixelFormat};
use webarkitlib_rs::{arlog_e, arlog_i};

/// Project a 2D point through a 3×3 row-major homography (inhomogeneous).
fn project(h: &[f32; 9], x: f32, y: f32) -> (f32, f32) {
    let w = h[6] * x + h[7] * y + h[8];
    (
        (h[0] * x + h[1] * y + h[2]) / w,
        (h[3] * x + h[4] * y + h[5]) / w,
    )
}

/// Max per-corner displacement between two homographies' reprojections
/// of the reference-image corners. Mirrors `DualFreakMatcher`'s private
/// tier-2 metric (M9 #152 corner-reprojection check) so the example can
/// report the same number for transparency.
fn max_corner_displacement(cpp_h: &[f32; 9], rust_h: &[f32; 9], rw: f32, rh: f32) -> f32 {
    let corners = [(0.0, 0.0), (rw, 0.0), (rw, rh), (0.0, rh)];
    corners
        .iter()
        .map(|&(x, y)| {
            let (cx, cy) = project(cpp_h, x, y);
            let (rx, ry) = project(rust_h, x, y);
            ((cx - rx).powi(2) + (cy - ry).powi(2)).sqrt()
        })
        .fold(0.0_f32, f32::max)
}

/// Feed a [`KpmRefDataSet`] into a `FreakMatcherBackend` by replicating
/// the page/image grouping logic from `KpmHandle::set_ref_data_set`.
/// Phase A doesn't go through `KpmHandle` (we need post-query access to
/// the concrete `DualFreakMatcher`), so this loop lives in the example.
///
/// Returns the per-`db_id` reference image dimensions so the example can
/// reproduce `DualFreakMatcher`'s tier-2 corner-reprojection metric for
/// the matched id.
fn feed_ref_data<M: FreakMatcherBackend>(
    matcher: &mut M,
    ref_data: &KpmRefDataSet,
) -> Result<Vec<(i32, i32)>, Box<dyn std::error::Error>> {
    let mut dims: Vec<(i32, i32)> = Vec::new();
    let mut db_id: usize = 0;
    for page in &ref_data.page_info {
        for img in &page.image_info {
            let mut points: Vec<FeaturePoint> = Vec::new();
            let mut descriptors: Vec<u8> = Vec::new();
            let mut points_3d: Vec<Point3d> = Vec::new();

            for rp in &ref_data.ref_point {
                if rp.ref_image_no == img.image_no && rp.page_no == page.page_no {
                    points.push(FeaturePoint {
                        x: rp.coord2d.x,
                        y: rp.coord2d.y,
                        angle: rp.feature_vec.angle,
                        scale: rp.feature_vec.scale,
                        maxima: rp.feature_vec.maxima != 0,
                    });
                    points_3d.push(Point3d {
                        x: rp.coord3d.x,
                        y: rp.coord3d.y,
                        z: 0.0,
                    });
                    descriptors.extend_from_slice(&rp.feature_vec.v[..FREAK_SUB_DIMENSION]);
                }
            }

            matcher.add_freak_features(
                &points,
                &descriptors,
                &points_3d,
                img.width as usize,
                img.height as usize,
                db_id,
            )?;

            dims.push((img.width, img.height));
            db_id += 1;
        }
    }
    Ok(dims)
}

fn main() {
    ar_log_init_default();

    let data_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("Data");

    arlog_i!("========================================");
    arlog_i!("  WebARKitLib-rs — Simple NFT Dual Example");
    arlog_i!("========================================");

    // ---------------------------------------------------------------
    // Step 1: Load camera parameters
    // ---------------------------------------------------------------
    let param_path = data_dir.join("camera_para.dat");
    arlog_i!("Step 1: Loading camera parameters...");
    let param_bytes = match std::fs::read(&param_path) {
        Ok(b) => b,
        Err(e) => {
            arlog_e!("failed to read camera_para.dat: {e}");
            return;
        }
    };
    let mut param = match ARParam::load(Cursor::new(&param_bytes)) {
        Ok(p) => p,
        Err(e) => {
            arlog_e!("failed to parse camera params: {e}");
            return;
        }
    };
    arlog_i!(
        "  Camera (original): {}x{}, mat[0][0]={:.2}",
        param.xsize,
        param.ysize,
        param.mat[0][0]
    );

    // ---------------------------------------------------------------
    // Step 2: Load test image (pinball demo) as grayscale luma
    // ---------------------------------------------------------------
    let img_path = data_dir.join("pinball-demo.jpg");
    arlog_i!("Step 2: Loading test image...");
    let jpeg_bytes = match std::fs::read(&img_path) {
        Ok(b) => b,
        Err(e) => {
            arlog_e!("failed to read pinball-demo.jpg: {e}");
            return;
        }
    };
    let mut decoder = jpeg_decoder::Decoder::new(Cursor::new(&jpeg_bytes));
    let pixels = match decoder.decode() {
        Ok(p) => p,
        Err(e) => {
            arlog_e!("JPEG decode failed: {e}");
            return;
        }
    };
    let info = match decoder.info() {
        Some(i) => i,
        None => {
            arlog_e!("no JPEG info");
            return;
        }
    };
    let width = info.width as i32;
    let height = info.height as i32;
    arlog_i!("  Image: {}x{}", width, height);

    // RGB → luma (BT.601 integer formula).
    let luma: Vec<u8> = pixels
        .chunks_exact(3)
        .map(|rgb| ((rgb[0] as u32 * 77 + rgb[1] as u32 * 150 + rgb[2] as u32 * 29) >> 8) as u8)
        .collect();
    arlog_i!("  Luma: {} bytes", luma.len());

    // Scale camera params to match image dimensions.
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
    // Step 3: Load NFT marker reference data
    // ---------------------------------------------------------------
    let marker_name = "pinball";
    let marker_base = data_dir.join(marker_name);
    arlog_i!("Step 3: Loading NFT marker '{}'...", marker_name);

    let fset3_path = data_dir.join(format!("{}.fset3", marker_name));
    let mut ref_data_a = match KpmRefDataSet::load(&fset3_path) {
        Ok(r) => r,
        Err(e) => {
            arlog_e!("failed to load .fset3: {e}");
            return;
        }
    };
    ref_data_a.change_page_no(
        webarkitlib_rs::kpm::ref_data_set::KPM_CHANGE_PAGE_NO_ALL_PAGES,
        0,
    );
    arlog_i!(
        "  .fset3: {} features, {} pages",
        ref_data_a.num,
        ref_data_a.page_num
    );

    // ---------------------------------------------------------------
    // Phase A — DualFreakMatcher diagnostic
    //
    // Drives the dual matcher directly (bypassing KpmHandle) so we can
    // read per-backend state after the query. KpmHandle wraps the
    // backend in Box<dyn FreakMatcherBackend>, which forbids recovering
    // the concrete DualFreakMatcher type — hence the split phases.
    // ---------------------------------------------------------------
    arlog_i!("Phase A: DualFreakMatcher diagnostic");

    let mut dual = match DualFreakMatcher::new(width, height) {
        Ok(d) => d,
        Err(e) => {
            arlog_e!("failed to create DualFreakMatcher: {e:?}");
            return;
        }
    };
    let ref_dims = match feed_ref_data(&mut dual, &ref_data_a) {
        Ok(d) => d,
        Err(e) => {
            arlog_e!("failed to feed ref data into DualFreakMatcher: {e}");
            return;
        }
    };
    arlog_i!(
        "  Reference data loaded into both backends ({} db_ids).",
        ref_dims.len()
    );

    let dual_result = match dual.query(&luma, width as usize, height as usize) {
        Ok(r) => r,
        Err(e) => {
            arlog_e!("DualFreakMatcher::query failed: {e:?}");
            return;
        }
    };
    arlog_i!(
        "  Query complete. matched_id = {} (C++ ground truth)",
        dual_result.matched_id
    );

    arlog_i!("  divergence_count       = {}", dual.divergence_count());
    match dual.last_divergence_reason() {
        Some(reason) => arlog_i!("  last_divergence_reason = {reason}"),
        None => arlog_i!("  last_divergence_reason = (none)"),
    }

    match (dual.cpp_matched_geometry(), dual.rust_matched_geometry()) {
        (Some(cpp_h), Some(rust_h)) => {
            arlog_i!("  C++ homography (3x3):");
            for r in 0..3 {
                arlog_i!(
                    "    [{:>10.4} {:>10.4} {:>10.4}]",
                    cpp_h[r * 3],
                    cpp_h[r * 3 + 1],
                    cpp_h[r * 3 + 2]
                );
            }
            arlog_i!("  Rust homography (3x3):");
            for r in 0..3 {
                arlog_i!(
                    "    [{:>10.4} {:>10.4} {:>10.4}]",
                    rust_h[r * 3],
                    rust_h[r * 3 + 1],
                    rust_h[r * 3 + 2]
                );
            }
            // Look up the matched id's reference dimensions so the
            // displacement metric matches DualFreakMatcher's internal
            // tier-2 check exactly.
            let matched = dual_result.matched_id;
            match (matched >= 0)
                .then(|| matched as usize)
                .and_then(|i| ref_dims.get(i))
            {
                Some(&(ref_w, ref_h)) => {
                    let max_disp =
                        max_corner_displacement(cpp_h, rust_h, ref_w as f32, ref_h as f32);
                    arlog_i!(
                        "  max corner displacement = {:.4} px (ref {}x{}, M9 #152 tolerance: 2.0 px)",
                        max_disp,
                        ref_w,
                        ref_h
                    );
                }
                None => {
                    arlog_i!(
                        "  (no ref dims for matched_id={}; skipping corner-displacement metric)",
                        matched
                    );
                }
            }
        }
        _ => {
            arlog_i!("  (one or both backends produced no homography — no comparison possible)");
        }
    }

    // ---------------------------------------------------------------
    // Phase B — Production pipeline (mirror simple_nft.rs flow)
    //
    // Fresh KpmHandle + CppFreakMatcher. DualFreakMatcher::query returns
    // C++ as ground truth (M9-2 D5), so using CppFreakMatcher directly
    // here produces the same pose AR2 would receive in either case.
    // ---------------------------------------------------------------
    arlog_i!("Phase B: Production pipeline (KpmHandle + CppFreakMatcher)");

    let ref_data_b = match KpmRefDataSet::load(&fset3_path) {
        Ok(mut r) => {
            r.change_page_no(
                webarkitlib_rs::kpm::ref_data_set::KPM_CHANGE_PAGE_NO_ALL_PAGES,
                0,
            );
            r
        }
        Err(e) => {
            arlog_e!("failed to reload .fset3 for Phase B: {e}");
            return;
        }
    };

    let mut surface_set = match ar2_read_surface_set(&marker_base) {
        Ok(s) => s,
        Err(e) => {
            arlog_e!("failed to load surface set: {e:?}");
            return;
        }
    };
    if let Some((w, h, dpi)) = ar2_surface_set_marker_info(&surface_set) {
        arlog_i!("  Surface set: marker {}x{} @ {:.0} DPI", w, h, dpi);
    }

    let param_lt = ARParamLT::new_basic(param.clone());
    let param_lt_arc = Arc::new(param_lt);

    let cpp_backend = match CppFreakMatcher::new(width, height) {
        Ok(b) => b,
        Err(e) => {
            arlog_e!("failed to create CppFreakMatcher: {e:?}");
            return;
        }
    };
    let mut kpm_handle = KpmHandle::new(
        width,
        height,
        Some(param_lt_arc.clone()),
        Box::new(cpp_backend),
    );

    if let Err(e) = kpm_handle.set_ref_data_set(ref_data_b) {
        arlog_e!("failed to set ref data set: {e:?}");
        return;
    }

    if let Err(e) = kpm_handle.kpm_matching(&luma) {
        arlog_e!("kpm_matching failed: {e:?}");
        return;
    }

    let pose = kpm_handle.get_pose();
    let Some((cam_pose, page_no, error)) = pose else {
        arlog_i!("  KPM produced no valid pose — example ends here.");
        return;
    };

    arlog_i!("  KPM match: page={}, error={:.4}", page_no, error);
    arlog_i!("  Initial 3x4 pose matrix:");
    for r in 0..3 {
        arlog_i!(
            "    [{:>10.4} {:>10.4} {:>10.4} {:>10.4}]",
            cam_pose[r][0],
            cam_pose[r][1],
            cam_pose[r][2],
            cam_pose[r][3]
        );
    }

    // AR2 tracking refinement (same flow as simple_nft.rs).
    arlog_i!("  Running AR2 tracking...");
    surface_set.set_init_trans(cam_pose);

    let mut ar2_handle = AR2Handle::new(width, height, ARPixelFormat::MONO);
    let param_lt_for_ar2 = Box::new(ARParamLT::new_basic(param.clone()));
    ar2_handle.cparam_lt = Box::into_raw(param_lt_for_ar2);
    let icp_handle_ptr = match icp_create_handle(&param.mat) {
        Ok(p) => p,
        Err(e) => {
            arlog_e!("failed to create ICP handle: {e:?}");
            return;
        }
    };
    ar2_handle.icp_handle = icp_handle_ptr;

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
            arlog_i!("  AR2 tracking succeeded! Error={:.4}", tracking_err);
            arlog_i!("  Refined 3x4 pose matrix:");
            for r in 0..3 {
                arlog_i!(
                    "    [{:>10.4} {:>10.4} {:>10.4} {:>10.4}]",
                    refined_pose[r][0],
                    refined_pose[r][1],
                    refined_pose[r][2],
                    refined_pose[r][3]
                );
            }
        }
        Err(code) => {
            arlog_i!(
                "  AR2 tracking returned error code: {} (expected for a single static frame)",
                code
            );
            arlog_i!("  The KPM initial pose above is still valid and usable.");
        }
    }

    // Clean up raw pointers held by AR2Handle.
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

    arlog_i!("========================================");
    arlog_i!("  Simple NFT Dual example complete.");
    arlog_i!("========================================");
}
