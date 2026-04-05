/*
 *  ar2_pinball_io.rs
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

//! Integration tests loading real NFT marker files (pinball).

use std::path::PathBuf;
use webarkitlib_rs::ar2::{AR2FeatureSetT, AR2ImageSetT};

fn data_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("Data")
}

#[test]
fn test_load_pinball_iset() {
    let path = data_dir().join("pinball.iset");
    let iset = AR2ImageSetT::load(&path).expect("failed to load pinball.iset");

    // The pinball marker has 9 scales.
    assert_eq!(iset.num(), 9);

    // Scale 0 should be the highest DPI (base image from JPEG).
    let base = &iset.scale[0];
    assert!(base.xsize > 0, "base xsize must be positive");
    assert!(base.ysize > 0, "base ysize must be positive");
    assert!(base.dpi > 0.0, "base DPI must be positive");
    assert_eq!(
        base.img_bw.len(),
        (base.xsize * base.ysize) as usize,
        "pixel count must match dimensions"
    );

    // Each subsequent scale should have lower or equal DPI and smaller dimensions.
    for i in 1..iset.num() {
        let prev = &iset.scale[i - 1];
        let curr = &iset.scale[i];
        assert!(
            curr.dpi <= prev.dpi,
            "scale {} DPI ({}) should be <= scale {} DPI ({})",
            i,
            curr.dpi,
            i - 1,
            prev.dpi
        );
        assert!(curr.xsize > 0);
        assert!(curr.ysize > 0);
        assert_eq!(
            curr.img_bw.len(),
            (curr.xsize * curr.ysize) as usize,
            "scale {} pixel count mismatch",
            i
        );
    }

    // Print summary for manual inspection.
    println!("pinball.iset: {} scales", iset.num());
    for (i, s) in iset.scale.iter().enumerate() {
        println!(
            "  scale[{}]: {}x{} @ {:.1} DPI ({} bytes)",
            i,
            s.xsize,
            s.ysize,
            s.dpi,
            s.img_bw.len()
        );
    }
}

#[test]
fn test_load_pinball_fset() {
    let path = data_dir().join("pinball.fset");
    let fset = AR2FeatureSetT::load(&path).expect("failed to load pinball.fset");

    // The pinball marker has 9 scales matching the .iset.
    assert_eq!(fset.num(), 9);

    // Each scale should have features.
    let total_features: usize = fset.list.iter().map(|fp| fp.num()).sum();
    assert!(
        total_features > 0,
        "total features should be > 0, got {}",
        total_features
    );

    // Check structure of each scale.
    for (i, fp) in fset.list.iter().enumerate() {
        assert!(fp.maxdpi > 0.0, "scale {} maxdpi should be positive", i);
        assert!(
            fp.mindpi >= 0.0,
            "scale {} mindpi should be non-negative",
            i
        );
        assert!(
            fp.maxdpi >= fp.mindpi,
            "scale {} maxdpi ({}) should be >= mindpi ({})",
            i,
            fp.maxdpi,
            fp.mindpi
        );

        for coord in &fp.coord {
            assert!(coord.x >= 0, "feature x should be non-negative");
            assert!(coord.y >= 0, "feature y should be non-negative");
        }
    }

    // Print summary for manual inspection.
    println!(
        "pinball.fset: {} scales, {} total features",
        fset.num(),
        total_features
    );
    for (i, fp) in fset.list.iter().enumerate() {
        println!(
            "  scale[{}]: {} features, DPI range {:.1}–{:.1}",
            i,
            fp.num(),
            fp.mindpi,
            fp.maxdpi
        );
    }
}
