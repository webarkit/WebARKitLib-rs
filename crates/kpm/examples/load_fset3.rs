/*
 *  load_fset3.rs
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

//! Load and inspect a `.fset3` (KPM FREAK reference data set) file.
//!
//! Run with:
//!
//! ```sh
//! cargo run -p webarkitlib-kpm --example load_fset3
//! ```

use std::path::Path;
use webarkitlib_kpm::types::KpmRefDataSet;

fn main() {
    let fset3_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("data")
        .join("pinball.fset3");

    println!("Loading: {}", fset3_path.display());

    let ref_data = KpmRefDataSet::load(&fset3_path).expect("failed to load .fset3 file");

    // --- Summary ---
    println!("\n=== KpmRefDataSet Summary ===");
    println!("Total feature points : {}", ref_data.num);
    println!("Total pages          : {}", ref_data.page_num);

    // --- Page info ---
    for (i, page) in ref_data.page_info.iter().enumerate() {
        println!("\n--- Page {} ---", i);
        println!("  page_no   : {}", page.page_no);
        println!("  image_num : {}", page.image_num);
        for img in &page.image_info {
            println!(
                "    image_no={}, {}x{}",
                img.image_no, img.width, img.height
            );
        }
    }

    // --- Sample features ---
    let sample_count = 5.min(ref_data.ref_point.len());
    println!("\n--- First {} feature points ---", sample_count);
    for (i, pt) in ref_data.ref_point.iter().take(sample_count).enumerate() {
        println!(
            "  [{}] coord2d=({:.1}, {:.1})  coord3d=({:.1}, {:.1})  \
             angle={:.3}  scale={:.3}  page={}  ref_image={}",
            i,
            pt.coord2d.x,
            pt.coord2d.y,
            pt.coord3d.x,
            pt.coord3d.y,
            pt.feature_vec.angle,
            pt.feature_vec.scale,
            pt.page_no,
            pt.ref_image_no,
        );
    }

    // --- Descriptor preview (first feature) ---
    if let Some(pt) = ref_data.ref_point.first() {
        let desc = &pt.feature_vec.v;
        println!(
            "\n--- First descriptor (96 bytes, hex) ---\n  {:02x?}",
            &desc[..16]
        );
        println!("  ... ({} bytes total)", desc.len());
    }

    println!("\nDone.");
}
