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
//! This is the first complete Rust-only NFT marker creation test:
//! load image → generate features → save .fset → reload → verify roundtrip.

use webarkitlib_rs::ar2::feature_map::ar2_gen_feature_map;
use webarkitlib_rs::ar2::feature_set::AR2FeatureSetT;

/// Full NFT marker creation pipeline: image → feature map → save → load.
///
/// Loads `pinball.jpg`, generates features via `ar2_gen_feature_map`,
/// saves the resulting `.fset` to a temporary file, reloads it, and
/// verifies that the roundtrip is lossless.
///
/// This test is `#[ignore]` because `ar2_gen_feature_map` is
/// compute-intensive. Run it explicitly with:
///
/// ```bash
/// cargo test -p webarkitlib-rs --test nft_pipeline -- --ignored --release
/// ```
#[test]
#[ignore]
fn test_full_nft_marker_creation_pipeline() {
    // Load pinball.jpg and convert to grayscale.
    let img = image::open("examples/Data/pinball.jpg").expect("failed to open pinball.jpg");
    let gray = img.to_luma8();
    let w = gray.width() as i32;
    let h = gray.height() as i32;
    let data = gray.into_raw();

    // Generate features.
    let feature_set =
        ar2_gen_feature_map(&data, w, h, 72.0, 16, 64).expect("ar2_gen_feature_map failed");
    assert!(
        !feature_set.list.is_empty(),
        "feature set should have at least one scale"
    );
    let total_features: usize = feature_set.list.iter().map(|p| p.coord.len()).sum();
    assert!(total_features > 0, "should produce at least one feature");

    // Save to a temporary .fset file.
    let tmp = tempfile::NamedTempFile::new().unwrap();
    feature_set.save(tmp.path()).unwrap();

    // Reload and verify roundtrip.
    let reloaded = AR2FeatureSetT::load(tmp.path()).unwrap();
    assert_eq!(
        reloaded.num(),
        feature_set.num(),
        "scale count must match after roundtrip"
    );

    // Verify each scale's metadata and first coordinate.
    for (orig_pts, load_pts) in feature_set.list.iter().zip(reloaded.list.iter()) {
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
