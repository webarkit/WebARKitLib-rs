#![allow(clippy::chunks_exact_to_as_chunks)]
/*
 *  absolute_corner_error.rs
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

//! Absolute corner-error gate (#166 Track A).
//!
//! For each `.corners.json` fixture in
//! `crates/core/tests/fixtures/annotated_frames/`, this test:
//!
//! 1. Loads the JPEG it references (searching the fixtures dir first,
//!    then `examples/Data/` for the existing `pinball-demo.jpg`).
//! 2. Runs the frame through `DualFreakMatcher` so both backends'
//!    homographies are captured in a single query.
//! 3. Projects the matched-scale reference-image corners through each
//!    backend's `H` into query-image pixel space.
//! 4. Compares those projections against the hand-annotated ground-truth
//!    corners and records `max_i ‖projected_i − annotated_i‖` per
//!    backend.
//!
//! The current numbers are committed in `baseline.json` alongside the
//! fixtures. The test asserts that each cell is no worse than its
//! baseline (`current − baseline ≤ REGRESSION_EPSILON_PX`), so:
//!
//! - **CI stays green** on day 1 — both backends meet their own baseline.
//! - A PR that **regresses** either backend on any frame fails this test
//!   with a precise per-frame, per-backend message.
//! - A PR that **improves** either backend gets a reviewer note to
//!   regenerate the baseline so subsequent PRs are gated against the new,
//!   tighter floor.
//!
//! To regenerate the baseline after an intentional improvement, run:
//!
//! ```sh
//! WEBARKIT_REGEN_CORNER_BASELINE=1 \
//!   cargo test --test absolute_corner_error --features dual-mode -- --nocapture
//! ```
//!
//! See issue #166 for the full design rationale and #160 for the
//! divergence investigation that motivated this gate.

#![cfg(feature = "dual-mode")]

use std::collections::BTreeMap;
use std::fs;
use std::io::Cursor;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use webarkitlib_rs::kpm::types::{KpmRefDataSet, FREAK_SUB_DIMENSION};
use webarkitlib_rs::kpm::{DualFreakMatcher, FeaturePoint, FreakMatcherBackend, Point3d};

// ─────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────

const FIXTURES_DIR: &str = "tests/fixtures/annotated_frames";
const EXAMPLES_DATA: &str = "examples/Data";
const BASELINE_FILE: &str = "tests/fixtures/annotated_frames/baseline.json";
const REGEN_ENV: &str = "WEBARKIT_REGEN_CORNER_BASELINE";
const MARKER_NAME: &str = "pinball";

/// Float-noise + cross-platform tolerance for the regression gate.
///
/// Below this threshold, a per-cell delta is treated as noise; above
/// it, the cell is flagged as a regression.
///
/// **Why 3.5 px**: the gate has to absorb these sources of variance:
///
/// 1. Float-noise on identical input (sub-pixel).
/// 2. Cross-platform float-arithmetic / stdlib-version drift in the
///    KPM + ICP pipeline. CI runs on Linux (libstdc++); contributors
///    often run on Windows or macOS.
/// 3. **Residual cross-platform BHC variance** post-#170 fix.
///    `webarkit/WebARKitLib#39` switched the C++ matcher from
///    `std::unordered_map` to `std::map` (mirrored on the Rust port
///    by #171), which fixed tier-1 cross-platform divergence
///    (matched_id now agrees across platforms — verified on
///    `pinball-demo.jpg`). However, the BHC `cluster_map_t` change
///    also reordered cluster iteration; on `pinball-seq4.jpg` this
///    produces a different inlier set at the homography fit between
///    Linux and Windows, yielding ~2.85 px of cross-platform max-err
///    drift even though `matched_id` agrees. Cause: residual
///    float-arithmetic order differences in the inner-loop math
///    (likely Eigen SIMD codegen or libstdc++ vs MSVC CRT math
///    differences). Tracked as a follow-up to #170.
///
/// 3.5 px is chosen to absorb (1)–(3) on every fixture we ship today,
/// while still tight enough to flag a real backend regression.
/// Was 2.0 pre-#172; expanded to 3.5 to absorb the seq4 BHC variance.
/// Once the residual cross-platform variance source is identified and
/// fixed (the open follow-up of #170), this can tighten back to
/// ~0.5 px.
const REGRESSION_EPSILON_PX: f32 = 3.5;

const ROLES: [&str; 4] = ["top_left", "top_right", "bottom_right", "bottom_left"];

// ─────────────────────────────────────────────────────────────────────────
// JSON schemas
// ─────────────────────────────────────────────────────────────────────────

#[derive(Deserialize, Debug)]
struct CornerAnnotation {
    #[allow(dead_code)]
    schema: u32,
    image: String,
    image_dims: [i32; 2],
    #[allow(dead_code)]
    annotator: String,
    #[allow(dead_code)]
    date: String,
    marker_corners_px: Vec<CornerPoint>,
    #[allow(dead_code)]
    tolerance_px: f32,
    #[allow(dead_code)]
    notes: String,
}

#[derive(Deserialize, Debug)]
struct CornerPoint {
    role: String,
    x: f32,
    y: f32,
}

#[derive(Deserialize, Serialize, Debug, Clone)]
struct Baseline {
    schema: u32,
    /// Free-text note recording when / how this baseline was
    /// regenerated. Helps future readers understand what produced the
    /// numbers (which OS, rustc, branch).
    regenerated_with: String,
    per_frame: BTreeMap<String, BackendErrors>,
}

/// Per-frame measurement record.
///
/// `cpp_max_err_px` and `rust_max_err_px` are `None` if that backend
/// (or the dual matcher overall) didn't match — preserving "no match"
/// status across regen so a later change that suddenly DOES match
/// (or stops matching) shows up as a regression / improvement.
///
/// `matched_id` is the C++ ground-truth matched id (the value
/// `DualFreakMatcher::query` returns). When tier-1 divergence occurs,
/// Rust internally matched a different id; `tier1_diverged` flags that
/// case so future readers know the Rust number was computed against
/// C++'s ref_dims and is therefore approximate.
#[derive(Deserialize, Serialize, Debug, Clone)]
struct BackendErrors {
    cpp_max_err_px: Option<f32>,
    rust_max_err_px: Option<f32>,
    matched_id: i32,
    ref_dims: Option<[i32; 2]>,
    #[serde(default)]
    tier1_diverged: bool,
}

// ─────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// Resolve an image file by name, searching the annotated-frames fixture
/// directory first and falling back to `examples/Data/` (so
/// `pinball-demo.jpg`, which is shared with the examples, is found
/// without being duplicated into the test tree).
fn resolve_image_path(name: &str) -> Option<PathBuf> {
    let here = manifest_dir();
    let candidates = [
        here.join(FIXTURES_DIR).join(name),
        here.join(EXAMPLES_DATA).join(name),
    ];
    candidates.into_iter().find(|p| p.exists())
}

/// Project a 2D point through a 3×3 row-major homography (inhomogeneous).
fn project(h: &[f32; 9], x: f32, y: f32) -> (f32, f32) {
    let w = h[6] * x + h[7] * y + h[8];
    (
        (h[0] * x + h[1] * y + h[2]) / w,
        (h[3] * x + h[4] * y + h[5]) / w,
    )
}

/// Project the four reference-image corners (in `TL/TR/BR/BL` order, the
/// same order as the annotation JSONs) through `h` into query-image
/// pixel coordinates.
fn reproject_corners(h: &[f32; 9], rw: f32, rh: f32) -> [(f32, f32); 4] {
    [
        project(h, 0.0, 0.0),
        project(h, rw, 0.0),
        project(h, rw, rh),
        project(h, 0.0, rh),
    ]
}

/// Decode a JPEG file to grayscale luma via BT.601, returning the luma
/// buffer plus the image dimensions.
fn decode_jpeg_luma(path: &Path) -> (Vec<u8>, i32, i32) {
    let jpeg_bytes = fs::read(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let mut decoder = jpeg_decoder::Decoder::new(Cursor::new(&jpeg_bytes));
    let pixels = decoder.decode().expect("JPEG decode failed");
    let info = decoder.info().expect("no JPEG info");
    let width = info.width as i32;
    let height = info.height as i32;
    let luma: Vec<u8> = pixels
        .chunks_exact(3)
        .map(|rgb| ((rgb[0] as u32 * 77 + rgb[1] as u32 * 150 + rgb[2] as u32 * 29) >> 8) as u8)
        .collect();
    (luma, width, height)
}

/// Feed a [`KpmRefDataSet`] into a backend by replicating the
/// page/image grouping logic from `KpmHandle::set_ref_data_set`.
/// Returns the per-db_id reference image dimensions in the same order
/// the backend's internal `ref_dims` HashMap was populated.
fn feed_ref_data<M: FreakMatcherBackend>(
    matcher: &mut M,
    ref_data: &KpmRefDataSet,
) -> Vec<(i32, i32)> {
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

            matcher
                .add_freak_features(
                    &points,
                    &descriptors,
                    &points_3d,
                    img.width as usize,
                    img.height as usize,
                    db_id,
                )
                .expect("add_freak_features");

            dims.push((img.width, img.height));
            db_id += 1;
        }
    }
    dims
}

/// Max per-corner Euclidean distance between annotated ground-truth and
/// projected corners. Assumes both slices are in the same `TL/TR/BR/BL`
/// order.
fn max_corner_error(annotated: &[CornerPoint], projected: &[(f32, f32); 4]) -> f32 {
    annotated
        .iter()
        .zip(projected.iter())
        .map(|(a, &(px, py))| {
            let dx = px - a.x;
            let dy = py - a.y;
            (dx * dx + dy * dy).sqrt()
        })
        .fold(0.0_f32, f32::max)
}

/// Validate that the annotation has all four corners in the expected
/// `TL/TR/BR/BL` order — the only ordering the projection math assumes.
fn assert_role_order(annotation: &CornerAnnotation) {
    assert_eq!(
        annotation.marker_corners_px.len(),
        4,
        "{}: expected 4 corner annotations, got {}",
        annotation.image,
        annotation.marker_corners_px.len()
    );
    for (i, c) in annotation.marker_corners_px.iter().enumerate() {
        assert_eq!(
            c.role, ROLES[i],
            "{}: corner[{i}] role = {:?}, expected {:?}",
            annotation.image, c.role, ROLES[i]
        );
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Per-frame measurement
// ─────────────────────────────────────────────────────────────────────────

fn measure_one_frame(annotation: &CornerAnnotation, frame_path: &Path) -> BackendErrors {
    let (luma, w, h) = decode_jpeg_luma(frame_path);
    assert_eq!(
        w, annotation.image_dims[0],
        "{}: decoded width {} differs from JSON {}",
        annotation.image, w, annotation.image_dims[0]
    );
    assert_eq!(
        h, annotation.image_dims[1],
        "{}: decoded height {} differs from JSON {}",
        annotation.image, h, annotation.image_dims[1]
    );

    let fset3_path = manifest_dir()
        .join(EXAMPLES_DATA)
        .join(format!("{MARKER_NAME}.fset3"));
    let mut ref_data = KpmRefDataSet::load(&fset3_path).expect("load .fset3");
    ref_data.change_page_no(
        webarkitlib_rs::kpm::ref_data_set::KPM_CHANGE_PAGE_NO_ALL_PAGES,
        0,
    );

    let mut dual = DualFreakMatcher::new(w, h).expect("DualFreakMatcher::new");
    let dims = feed_ref_data(&mut dual, &ref_data);

    let result = dual
        .query(&luma, w as usize, h as usize)
        .expect("dual.query");

    // C++ ground-truth matched_id (this is what `dual.query` returns).
    // If negative, neither backend's homography is usable here.
    if result.matched_id < 0 {
        return BackendErrors {
            cpp_max_err_px: None,
            rust_max_err_px: None,
            matched_id: -1,
            ref_dims: None,
            tier1_diverged: false,
        };
    }

    let matched_id = result.matched_id as usize;
    let (ref_w, ref_h) = dims[matched_id];

    // `tier1_diverged` is true when divergence_count > 0 and the
    // reason starts with "matched_id mismatch" — in that case Rust
    // matched a different id, so its homography is fit to a different
    // ref scale than the one we're using for the reprojection. The
    // Rust error number is still recorded (it's computed against
    // C++'s ref_dims) but is approximate.
    let tier1_diverged = dual.divergence_count() > 0
        && dual
            .last_divergence_reason()
            .map(|r| r.starts_with("matched_id mismatch"))
            .unwrap_or(false);

    let cpp_err = dual.cpp_matched_geometry().map(|h| {
        let proj = reproject_corners(h, ref_w as f32, ref_h as f32);
        max_corner_error(&annotation.marker_corners_px, &proj)
    });
    let rust_err = dual.rust_matched_geometry().map(|h| {
        let proj = reproject_corners(h, ref_w as f32, ref_h as f32);
        max_corner_error(&annotation.marker_corners_px, &proj)
    });

    BackendErrors {
        cpp_max_err_px: cpp_err,
        rust_max_err_px: rust_err,
        matched_id: result.matched_id,
        ref_dims: Some([ref_w, ref_h]),
        tier1_diverged,
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Test
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_absolute_corner_error_against_annotations() {
    let fixtures_dir = manifest_dir().join(FIXTURES_DIR);
    let mut entries: Vec<_> = fs::read_dir(&fixtures_dir)
        .unwrap_or_else(|e| panic!("read_dir {}: {e}", fixtures_dir.display()))
        .filter_map(|e| e.ok())
        .filter(|e| {
            let name = e.file_name();
            let s = name.to_string_lossy();
            s.ends_with(".corners.json")
        })
        .collect();
    entries.sort_by_key(|e| e.file_name());

    assert!(
        !entries.is_empty(),
        "no `.corners.json` fixtures found under {}",
        fixtures_dir.display()
    );

    let mut measurements: BTreeMap<String, BackendErrors> = BTreeMap::new();

    println!();
    println!(
        "Absolute corner-error gate (#166 Track A) — {} frames",
        entries.len()
    );
    println!();
    println!(
        "| {:<24} | {:>10} | {:>11} | {:>16} | {:>17} | {:>6} |",
        "Frame", "matched_id", "ref dims", "C++ max err (px)", "Rust max err (px)", "tier-1"
    );
    println!(
        "|--------------------------|------------|-------------|------------------|-------------------|--------|"
    );

    for entry in &entries {
        let json_bytes = fs::read(entry.path()).expect("read corners.json");
        let annotation: CornerAnnotation = serde_json::from_slice(&json_bytes)
            .unwrap_or_else(|e| panic!("parse {}: {e}", entry.path().display()));
        assert_role_order(&annotation);

        let image_path = resolve_image_path(&annotation.image).unwrap_or_else(|| {
            panic!(
                "{}: image '{}' not found in {} or {}",
                entry.path().display(),
                annotation.image,
                FIXTURES_DIR,
                EXAMPLES_DATA
            )
        });

        let errors = measure_one_frame(&annotation, &image_path);

        let dims_str = errors
            .ref_dims
            .map(|[w, h]| format!("{w}x{h}"))
            .unwrap_or_else(|| "—".to_string());
        let cpp_str = errors
            .cpp_max_err_px
            .map(|v| format!("{v:.4}"))
            .unwrap_or_else(|| "no-match".to_string());
        let rust_str = errors
            .rust_max_err_px
            .map(|v| format!("{v:.4}"))
            .unwrap_or_else(|| "no-match".to_string());
        let tier1_str = if errors.tier1_diverged {
            "DIVERGED"
        } else {
            "ok"
        };
        println!(
            "| {:<24} | {:>10} | {:>11} | {:>16} | {:>17} | {:>6} |",
            annotation.image, errors.matched_id, dims_str, cpp_str, rust_str, tier1_str,
        );

        measurements.insert(annotation.image.clone(), errors);
    }

    // ────────────────────────────────────────────────────────────────────
    // Baseline regen mode (opt-in via WEBARKIT_REGEN_CORNER_BASELINE=1)
    // ────────────────────────────────────────────────────────────────────
    let baseline_path = manifest_dir().join(BASELINE_FILE);

    if std::env::var(REGEN_ENV).is_ok() {
        let baseline = Baseline {
            schema: 1,
            regenerated_with: format!(
                "regenerated {} via WEBARKIT_REGEN_CORNER_BASELINE=1",
                chrono::Utc::now().format("%Y-%m-%dT%H:%M:%SZ")
            ),
            per_frame: measurements,
        };
        let serialized = serde_json::to_string_pretty(&baseline).expect("serialize baseline");
        fs::write(&baseline_path, serialized + "\n").expect("write baseline");
        println!();
        println!("Baseline regenerated -> {}", baseline_path.display());
        return;
    }

    // ────────────────────────────────────────────────────────────────────
    // Regression check against baseline.json
    // ────────────────────────────────────────────────────────────────────
    let baseline_bytes = fs::read(&baseline_path).unwrap_or_else(|e| {
        panic!(
            "read baseline {}: {e}\n\
             If you intentionally introduced new fixtures or a backend change, \
             regenerate with:\n  \
             WEBARKIT_REGEN_CORNER_BASELINE=1 cargo test --test absolute_corner_error \
             --features dual-mode -- --nocapture",
            baseline_path.display()
        )
    });
    let baseline: Baseline = serde_json::from_slice(&baseline_bytes).expect("parse baseline.json");

    let mut failures: Vec<String> = Vec::new();
    let mut improvements: Vec<String> = Vec::new();

    /// Compare a single backend's current measurement against its baseline.
    /// Pushes a regression failure or an improvement note into the
    /// appropriate vector. Handles all four (Some/None) × (Some/None)
    /// transitions sensibly.
    fn compare_cell(
        frame: &str,
        backend: &str,
        current: Option<f32>,
        baseline: Option<f32>,
        failures: &mut Vec<String>,
        improvements: &mut Vec<String>,
    ) {
        match (current, baseline) {
            (Some(cur), Some(base)) => {
                let delta = cur - base;
                if delta > REGRESSION_EPSILON_PX {
                    failures.push(format!(
                        "{frame}: {backend} regressed: current={cur:.4} px, baseline={base:.4} px \
                         (delta {delta:+.4} > {REGRESSION_EPSILON_PX:.1} epsilon)"
                    ));
                } else if delta < -REGRESSION_EPSILON_PX {
                    improvements.push(format!(
                        "  {frame}: {backend} improved: current={cur:.4} px, baseline={base:.4} px \
                         (delta {delta:+.4})"
                    ));
                }
            }
            (None, Some(base)) => {
                failures.push(format!(
                    "{frame}: {backend} stopped matching (was {base:.4} px in baseline)"
                ));
            }
            (Some(cur), None) => {
                improvements.push(format!(
                    "  {frame}: {backend} started matching at {cur:.4} px \
                     (baseline was no-match)"
                ));
            }
            (None, None) => { /* both no-match — stable, no signal */ }
        }
    }

    for (frame, current) in &measurements {
        let Some(b) = baseline.per_frame.get(frame) else {
            failures.push(format!(
                "no baseline entry for '{frame}'; regenerate with {REGEN_ENV}=1"
            ));
            continue;
        };
        compare_cell(
            frame,
            "C++",
            current.cpp_max_err_px,
            b.cpp_max_err_px,
            &mut failures,
            &mut improvements,
        );
        compare_cell(
            frame,
            "Rust",
            current.rust_max_err_px,
            b.rust_max_err_px,
            &mut failures,
            &mut improvements,
        );
        // Tier-1 transitions: catching them here means a divergence
        // appearing or disappearing is loud, not silent.
        if current.tier1_diverged && !b.tier1_diverged {
            failures.push(format!(
                "{frame}: tier-1 (matched_id) divergence appeared between backends; \
                 baseline had agreement"
            ));
        } else if !current.tier1_diverged && b.tier1_diverged {
            improvements.push(format!(
                "  {frame}: tier-1 (matched_id) divergence resolved; backends now agree"
            ));
        }
    }

    if !improvements.is_empty() {
        println!();
        println!(
            "Improvements detected (> {} px better than baseline). \
             Consider regenerating the baseline so future PRs are gated against \
             the new floor:",
            REGRESSION_EPSILON_PX
        );
        for line in &improvements {
            println!("{line}");
        }
        println!(
            "  -> WEBARKIT_REGEN_CORNER_BASELINE=1 cargo test --test absolute_corner_error \
             --features dual-mode -- --nocapture"
        );
    }

    assert!(
        failures.is_empty(),
        "{} corner-error regression(s) detected:\n  {}\n\
         If these regressions are intentional, regenerate the baseline:\n  \
         {REGEN_ENV}=1 cargo test --test absolute_corner_error --features dual-mode \
         -- --nocapture",
        failures.len(),
        failures.join("\n  ")
    );
}
