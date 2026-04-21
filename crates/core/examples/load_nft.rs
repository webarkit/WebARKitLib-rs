/*
 *  load_nft.rs
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

//! Load and inspect a complete NFT marker (.fset3, .iset, .fset).
//!
//! This is a simplified Rust equivalent of the `addNFTMarkers` function in
//! [jsartoolkitNFT](https://github.com/webarkit/jsartoolkitNFT/blob/master/emscripten/ARToolKitNFT_js.cpp#L338-L423).
//!
//! It loads all three NFT marker files for a given dataset prefix and prints
//! a summary of the loaded data.
//!
//! Run with:
//!
//! ```sh
//! cargo run -p webarkitlib-rs --example load_nft
//! ```
//!
//! This example uses [`webarkitlib_rs::arlog`] for logging via `arlog_i!`
//! and installs the bundled `env_logger`-based backend with
//! [`webarkitlib_rs::arlog::ar_log_init_default`]. The initializer is gated
//! behind the `log-helpers` Cargo feature, which is declared as
//! `required-features` for this example in `crates/core/Cargo.toml` — so
//! `cargo run --example load_nft` enables it automatically; no `--features`
//! flag needed.
//!
//! Adjust verbosity via the `RUST_LOG` env var (e.g. `RUST_LOG=debug`) or
//! programmatically with [`webarkitlib_rs::arlog::set_ar_log_level`].

use std::path::Path;
use webarkitlib_rs::ar2::{AR2FeatureSetT, AR2ImageSetT};
use webarkitlib_rs::arlog_i;
use webarkitlib_rs::kpm::types::KpmRefDataSet;

fn main() {
    webarkitlib_rs::arlog::ar_log_init_default();
    let data_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("Data");
    let marker_name = "pinball";

    arlog_i!("Loading NFT marker: '{}'", marker_name);
    println!("========================================\n");

    // ---------------------------------------------------------------
    // Step 1: Load KPM data (.fset3)
    //   C++: kpmLoadRefDataSet(datasetPathname, "fset3", &refDataSet)
    // ---------------------------------------------------------------
    let fset3_path = data_dir.join(format!("{}.fset3", marker_name));
    println!("Reading {}.fset3 ...", marker_name);

    let ref_data = KpmRefDataSet::load(&fset3_path).expect("failed to load .fset3");

    // In the C++ flow, page numbers are assigned and datasets are merged
    // for multi-marker support:
    //   kpmChangePageNoOfRefDataSet(refDataSet, KpmChangePageNoAllPages, pageNo)
    //   kpmMergeRefDataSet(&refDataSet, &refDataSet2)
    // Here we just load a single marker (page 0).
    let page_no = 0;
    println!("  Assigned page no. {}", page_no);
    println!(
        "  KPM features: {}, pages: {}",
        ref_data.num, ref_data.page_num
    );
    for page in &ref_data.page_info {
        println!(
            "  Page {}: {} images in pyramid",
            page.page_no, page.image_num
        );
        for img in &page.image_info {
            println!(
                "    image_no={}, {}x{}",
                img.image_no, img.width, img.height
            );
        }
    }
    println!("  Done.\n");

    // ---------------------------------------------------------------
    // Step 2: Load AR2 image set (.iset)
    //   C++: loaded internally by ar2ReadSurfaceSet → imageSet
    // ---------------------------------------------------------------
    let iset_path = data_dir.join(format!("{}.iset", marker_name));
    println!("Reading {}.iset ...", marker_name);

    let image_set = AR2ImageSetT::load(&iset_path).expect("failed to load .iset");

    // Extract marker info like the C++ code does:
    //   nft.width_NFT  = surfaceSet->surface[0].imageSet->scale[0]->xsize
    //   nft.height_NFT = surfaceSet->surface[0].imageSet->scale[0]->ysize
    //   nft.dpi_NFT    = surfaceSet->surface[0].imageSet->scale[0]->dpi
    let base = &image_set.scale[0];
    let width_nft = base.xsize;
    let height_nft = base.ysize;
    let dpi_nft = base.dpi;

    println!("  NFT num. of ImageSet: {}", image_set.num());
    println!("  NFT marker width    : {}", width_nft);
    println!("  NFT marker height   : {}", height_nft);
    println!("  NFT marker dpi      : {:.0}", dpi_nft);

    // Print all scales.
    for (i, scale) in image_set.scale.iter().enumerate() {
        println!(
            "    scale[{}]: {}x{} @ {:.1} DPI ({} bytes)",
            i,
            scale.xsize,
            scale.ysize,
            scale.dpi,
            scale.img_bw.len()
        );
    }
    println!("  Done.\n");

    // ---------------------------------------------------------------
    // Step 3: Load AR2 feature set (.fset)
    //   C++: loaded internally by ar2ReadSurfaceSet → featureSet
    // ---------------------------------------------------------------
    let fset_path = data_dir.join(format!("{}.fset", marker_name));
    println!("Reading {}.fset ...", marker_name);

    let feature_set = AR2FeatureSetT::load(&fset_path).expect("failed to load .fset");

    let total_features: usize = feature_set.list.iter().map(|fp| fp.num()).sum();
    println!(
        "  AR2 feature scales: {}, total features: {}",
        feature_set.num(),
        total_features
    );
    for (i, fp) in feature_set.list.iter().enumerate() {
        println!(
            "    scale[{}]: {} features, DPI range {:.1}\u{2013}{:.1}",
            i,
            fp.num(),
            fp.mindpi,
            fp.maxdpi
        );
    }
    println!("  Done.\n");

    // ---------------------------------------------------------------
    // Summary
    // ---------------------------------------------------------------
    println!("========================================");
    println!("Loading of NFT data complete.");
    println!(
        "  Marker '{}': {}x{} @ {:.0} DPI",
        marker_name, width_nft, height_nft, dpi_nft
    );
    println!(
        "  {} KPM features, {} AR2 features across {} scales",
        ref_data.num,
        total_features,
        feature_set.num()
    );
}
