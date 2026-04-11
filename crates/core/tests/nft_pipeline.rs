/*
 *  tests/nft_pipeline.rs
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

//! End-to-end NFT marker creation pipeline test.
//!
//! Tests the full Rust-only NFT marker creation pipeline:
//! load image → ar2_gen_image_set → ar2_gen_feature_map → save → reload → verify.

use webarkitlib_rs::ar2::feature_map::ar2_gen_feature_map;
use webarkitlib_rs::ar2::feature_set::AR2FeatureSetT;
use webarkitlib_rs::ar2::image_set::{ar2_gen_image_set, AR2ImageSetT};

/// Full NFT marker creation pipeline: image → image set → feature set → save → load.
///
/// Loads `pinball.jpg`, builds an image pyramid via `ar2_gen_image_set`
/// (cube-root-of-2 DPI stepping), generates features via `ar2_gen_feature_map`,
/// saves both `.iset` and `.fset` to temporary files, reloads them, and
/// verifies lossless roundtrips.
///
/// This test is `#[ignore]` because `ar2_gen_feature_map` is compute-intensive.
/// Run it explicitly with:
///
/// ```bash
/// cargo test -p webarkitlib-rs --test nft_pipeline -- --ignored --release
/// ```
#[test]
#[ignore]
fn test_full_nft_marker_creation_pipeline() {
    // Load pinball.jpg — keep as RGB (nc=3) to exercise the colour path.
    let img = image::open("examples/Data/pinball.jpg").expect("failed to open pinball.jpg");
    let rgb = img.to_rgb8();
    let w = rgb.width() as i32;
    let h = rgb.height() as i32;
    let data = rgb.into_raw(); // length = w * h * 3

    // Step 1: build the image pyramid.
    let image_set = ar2_gen_image_set(&data, w, h, 3, 72.0).expect("ar2_gen_image_set failed");
    assert!(
        image_set.num() >= 2,
        "expected ≥ 2 pyramid levels, got {}",
        image_set.num()
    );

    // Step 2: generate features from the pyramid.
    let feature_set =
        ar2_gen_feature_map(&image_set, 16, 64, 2).expect("ar2_gen_feature_map failed");
    assert!(
        !feature_set.list.is_empty(),
        "feature set should have at least one scale"
    );
    let total_features: usize = feature_set.list.iter().map(|p| p.coord.len()).sum();
    assert!(total_features > 0, "should produce at least one feature");

    // Step 3: save + reload .iset and verify roundtrip.
    let tmp_iset = tempfile::NamedTempFile::new().unwrap();
    image_set.save(tmp_iset.path()).unwrap();
    let reloaded_iset = AR2ImageSetT::load(tmp_iset.path()).unwrap();
    assert_eq!(
        reloaded_iset.num(),
        image_set.num(),
        ".iset scale count mismatch"
    );
    for (orig, loaded) in image_set.scale.iter().zip(reloaded_iset.scale.iter()) {
        assert_eq!(loaded.xsize, orig.xsize);
        assert_eq!(loaded.ysize, orig.ysize);
        assert!((loaded.dpi - orig.dpi).abs() < 1e-3);
        assert_eq!(loaded.img_bw, orig.img_bw);
    }

    // Step 4: save + reload .fset and verify roundtrip.
    let tmp_fset = tempfile::NamedTempFile::new().unwrap();
    feature_set.save(tmp_fset.path()).unwrap();
    let reloaded_fset = AR2FeatureSetT::load(tmp_fset.path()).unwrap();
    assert_eq!(
        reloaded_fset.num(),
        feature_set.num(),
        ".fset scale count mismatch after roundtrip"
    );
    for (orig_pts, load_pts) in feature_set.list.iter().zip(reloaded_fset.list.iter()) {
        assert_eq!(load_pts.scale, orig_pts.scale);
        assert_eq!(load_pts.num(), orig_pts.num());
        assert!((load_pts.maxdpi - orig_pts.maxdpi).abs() < 1e-4);
        assert!((load_pts.mindpi - orig_pts.mindpi).abs() < 1e-4);

        if !orig_pts.coord.is_empty() {
            let oc = &orig_pts.coord[0];
            let lc = &load_pts.coord[0];
            assert_eq!(lc.x, oc.x);
            assert_eq!(lc.y, oc.y);
            assert!(
                (lc.mx - oc.mx).abs() < 1e-4,
                "first coord mx mismatch: {} vs {}",
                lc.mx,
                oc.mx
            );
        }
    }
}

/// Structural comparison of Rust-generated markers against C-generated reference files.
///
/// Generates `.iset` and `.fset` from `pinball.jpg` and compares them
/// structurally against the pre-existing C-generated `pinball.iset` /
/// `pinball.fset` files in `examples/Data/`. Validates:
///
/// - Same number of pyramid levels
/// - Same DPI values per level (within ±0.5 DPI)
/// - Feature counts within ±10% per scale (float arithmetic differs across
///   compilers/platforms, so byte-for-byte identity is not expected)
///
/// This test is `#[ignore]` because `ar2_gen_feature_map` is compute-intensive.
/// Run explicitly with:
///
/// ```bash
/// cargo test -p webarkitlib-rs --test nft_pipeline compare -- --ignored --release
/// ```
#[test]
#[ignore]
fn test_compare_with_c_generated_pinball_marker() {
    // Load pinball.jpg as grayscale (nc=1).
    let img = image::open("examples/Data/pinball.jpg").expect("failed to open pinball.jpg");
    let gray = img.to_luma8();
    let w = gray.width() as i32;
    let h = gray.height() as i32;
    let data = gray.into_raw();

    // Generate from Rust pipeline.
    let image_set = ar2_gen_image_set(&data, w, h, 1, 72.0).expect("ar2_gen_image_set failed");
    let feature_set =
        ar2_gen_feature_map(&image_set, 10, 250, 2).expect("ar2_gen_feature_map failed");

    // Load C-generated reference files.
    let ref_iset = AR2ImageSetT::load("examples/Data/pinball.iset")
        .expect("failed to load reference pinball.iset");
    let ref_fset = AR2FeatureSetT::load("examples/Data/pinball.fset")
        .expect("failed to load reference pinball.fset");

    // --- .iset comparison ---

    assert_eq!(
        image_set.num(),
        ref_iset.num(),
        "pyramid level count mismatch: Rust {} vs C {}",
        image_set.num(),
        ref_iset.num()
    );

    for (i, (rust_scale, c_scale)) in image_set
        .scale
        .iter()
        .zip(ref_iset.scale.iter())
        .enumerate()
    {
        assert!(
            (rust_scale.dpi - c_scale.dpi).abs() < 0.5,
            "level {} DPI mismatch: Rust {:.3} vs C {:.3}",
            i,
            rust_scale.dpi,
            c_scale.dpi
        );
        // Pixel dimensions should match (same DPI stepping = same output size).
        assert_eq!(
            rust_scale.xsize, c_scale.xsize,
            "level {} xsize mismatch: {} vs {}",
            i, rust_scale.xsize, c_scale.xsize
        );
        assert_eq!(
            rust_scale.ysize, c_scale.ysize,
            "level {} ysize mismatch: {} vs {}",
            i, rust_scale.ysize, c_scale.ysize
        );
    }

    // --- .fset comparison ---

    // Feature set may have fewer entries (empty scales are skipped), but
    // total features should be within ±10% of the C reference.
    let rust_total: usize = feature_set.list.iter().map(|p| p.coord.len()).sum();
    let c_total: usize = ref_fset.list.iter().map(|p| p.coord.len()).sum();

    assert!(
        rust_total > 0,
        "Rust pipeline produced no features — pipeline may be broken"
    );
    assert!(
        c_total > 0,
        "C reference has no features — reference file may be corrupt"
    );

    let ratio = rust_total as f64 / c_total as f64;
    assert!(
        ratio >= 0.90 && ratio <= 1.10,
        "feature count diverges: Rust {} vs C {} ({:.1}% — expected within ±10%)",
        rust_total,
        c_total,
        ratio * 100.0
    );
}
