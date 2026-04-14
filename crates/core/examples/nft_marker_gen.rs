/*
 *  nft_marker_gen.rs
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

//! NFT marker creation example.
//!
//! A pure-Rust replacement for the C/Emscripten `markerCreator` tool.
//! Generates `.iset`, `.fset`, and (with `--features ffi-backend`) `.fset3`
//! from a JPEG input image.
//!
//! ## Usage
//!
//! ```bash
//! # Pure-Rust output (.iset + .fset only):
//! cargo run --example nft_marker_gen -- --input marker.jpg --output marker --dpi 72
//!
//! # Full output including .fset3 (requires C++ FREAK backend):
//! cargo run --example nft_marker_gen --features ffi-backend -- \
//!     --input marker.jpg --output marker --dpi 72
//! ```
//!
//! ## Output files
//!
//! | File | Contents |
//! |---|---|
//! | `<output>.iset` | Multi-resolution image pyramid (always generated) |
//! | `<output>.fset` | AR2 feature points per scale (always generated) |
//! | `<output>.fset3` | KPM/FREAK keypoints (`ffi-backend` only) |

use chrono::Local;
use clap::Parser;
use std::path::Path;
use std::time::Instant;
use webarkitlib_rs::ar2::{ar2_gen_feature_map, ar2_gen_image_set};

/// KPM feature density for `.fset3` generation (default extraction level 1).
///
/// Matches `KPM_SURF_FEATURE_DENSITY_L1` in `markerCreator.cpp`.
#[cfg(feature = "ffi-backend")]
const KPM_SURF_FEATURE_DENSITY: i32 = 100;

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

/// Generate NFT marker files (.iset, .fset, .fset3) from a JPEG source image.
///
/// Ports the `markerCreator` C/Emscripten tool to pure Rust.
/// Output files are compatible with the WebARKit NFT tracking runtime.
#[derive(Parser, Debug)]
#[command(
    name = "nft_marker_gen",
    about = "Generate NFT marker files from a JPEG image"
)]
struct Cli {
    /// Input JPEG image path.
    #[arg(short, long)]
    input: std::path::PathBuf,

    /// Output base name (without extension).
    ///
    /// Writes <output>.iset, <output>.fset, and (with ffi-backend) <output>.fset3.
    #[arg(short, long)]
    output: std::path::PathBuf,

    /// Source image DPI (dots per inch).
    #[arg(long, default_value_t = 72.0)]
    dpi: f32,

    /// Tracking extraction level (0–4; default 2).
    ///
    /// Higher levels produce more features with tighter quality thresholds.
    /// Matches the `-level=n` flag in the C `markerCreator` tool.
    #[arg(long, default_value_t = 2)]
    level: u8,

    /// Skip the "no .fset3" confirmation prompt (for CI / scripted use).
    #[arg(short, long)]
    yes: bool,
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Format a file's byte size as "X KB".
fn file_kb(path: &Path) -> String {
    std::fs::metadata(path)
        .map(|m| format!("{} KB", m.len() / 1024))
        .unwrap_or_else(|_| "? KB".into())
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

fn main() {
    let cli = Cli::parse();
    let start = Instant::now();
    let start_time = Local::now();

    // -----------------------------------------------------------------------
    // Warn the user if .fset3 won't be generated (no ffi-backend).
    // -----------------------------------------------------------------------
    #[cfg(not(feature = "ffi-backend"))]
    {
        use std::io::Write as _;
        if !cli.yes {
            eprintln!();
            eprintln!(
                "Warning: built without `ffi-backend` feature — .fset3 will NOT be generated."
            );
            eprintln!("An NFT marker without .fset3 cannot be used for initialization/detection.");
            eprint!("Continue anyway? [y/N] ");
            let _ = std::io::stdout().flush();
            let mut answer = String::new();
            std::io::stdin()
                .read_line(&mut answer)
                .expect("failed to read input");
            if !answer.trim().eq_ignore_ascii_case("y") {
                eprintln!("Aborted.");
                std::process::exit(1);
            }
            eprintln!();
        }
    }

    // -----------------------------------------------------------------------
    // Header
    // -----------------------------------------------------------------------
    println!("--");
    println!(
        "Generator started at {}",
        start_time.format("%Y-%m-%d %H:%M:%S %z")
    );
    println!();

    // -----------------------------------------------------------------------
    // Load input image as RGB.
    // -----------------------------------------------------------------------
    let img = image::open(&cli.input).unwrap_or_else(|e| {
        eprintln!("Error: failed to open {:?}: {}", cli.input, e);
        std::process::exit(1);
    });
    let rgb = img.to_rgb8();
    let w = rgb.width() as i32;
    let h = rgb.height() as i32;
    let nc = 3;
    let pixels = rgb.into_raw(); // nc=3, length = w * h * 3

    println!("Tracking Extraction Level = {}", cli.level);
    println!();
    println!(
        "Input:  {:?}  (xsize={}, ysize={}, channels={}, dpi={:.1})",
        cli.input, w, h, nc, cli.dpi
    );
    println!("Output: {:?}.*", cli.output);
    println!();

    // -----------------------------------------------------------------------
    // Step 1: build image pyramid → .iset
    // -----------------------------------------------------------------------
    println!("Generating ImageSet...");
    let step_start = Instant::now();

    let image_set = ar2_gen_image_set(&pixels, w, h, nc, cli.dpi).unwrap_or_else(|e| {
        eprintln!("Error: ar2_gen_image_set failed: {}", e);
        std::process::exit(1);
    });

    // Print DPI levels.
    for (i, scale) in image_set.scale.iter().enumerate() {
        println!("  Image DPI ({}): {:.6}", image_set.num() - i, scale.dpi);
    }
    println!();

    let iset_path = cli.output.with_extension("iset");
    println!("Saving to {:?}...", iset_path);
    image_set.save(&iset_path).unwrap_or_else(|e| {
        eprintln!("Error: failed to save {:?}: {}", iset_path, e);
        std::process::exit(1);
    });

    let first_dpi = image_set.scale.first().map_or(0.0, |s| s.dpi);
    let last_dpi = image_set.scale.last().map_or(0.0, |s| s.dpi);
    println!(
        "  Done. {} levels: {:.1} -> {:.1} dpi  ({}, {:.1}s)",
        image_set.num(),
        first_dpi,
        last_dpi,
        file_kb(&iset_path),
        step_start.elapsed().as_secs_f64()
    );
    println!();

    // -----------------------------------------------------------------------
    // Step 2: generate AR2 features → .fset
    // -----------------------------------------------------------------------
    println!("Generating FeatureList...");
    let step_start = Instant::now();

    // search_size = AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE1 = 10 (from C config.h)
    let feature_set = ar2_gen_feature_map(&image_set, 10, 250, cli.level).unwrap_or_else(|e| {
        eprintln!("Error: ar2_gen_feature_map failed: {}", e);
        std::process::exit(1);
    });

    // Print per-level feature counts.
    for pts in &feature_set.list {
        let scale_img = &image_set.scale[pts.scale as usize];
        println!(
            "  {:.6} dpi ({}x{}): {} features selected",
            scale_img.dpi,
            scale_img.xsize,
            scale_img.ysize,
            pts.coord.len()
        );
    }
    println!();

    let fset_path = cli.output.with_extension("fset");
    println!("Saving FeatureSet to {:?}...", fset_path);
    feature_set.save(&fset_path).unwrap_or_else(|e| {
        eprintln!("Error: failed to save {:?}: {}", fset_path, e);
        std::process::exit(1);
    });

    let total_features: usize = feature_set.list.iter().map(|p| p.coord.len()).sum();
    println!(
        "  Done. {} levels, {} features total  ({}, {:.1}s)",
        feature_set.num(),
        total_features,
        file_kb(&fset_path),
        step_start.elapsed().as_secs_f64()
    );
    println!();

    // -----------------------------------------------------------------------
    // Step 3: generate KPM/FREAK keypoints → .fset3  (ffi-backend only)
    // -----------------------------------------------------------------------
    #[cfg(feature = "ffi-backend")]
    {
        use webarkitlib_rs::kpm::types::KpmRefDataSet;
        use webarkitlib_rs::kpm::{CppFreakMatcher, KpmCompMode, KpmError, KpmProcMode};

        println!("Generating FeatureSet3...");
        let step_start = Instant::now();

        let fset3_path = cli.output.with_extension("fset3");
        let mut combined: Option<KpmRefDataSet> = None;

        for (image_no, scale) in image_set.scale.iter().enumerate() {
            // Max features proportional to image area (matching markerCreator.cpp).
            let max_feat = KPM_SURF_FEATURE_DENSITY * scale.xsize * scale.ysize / (480 * 360);

            // Fresh matcher for each scale (scales may have different dimensions).
            let mut matcher = CppFreakMatcher::new(scale.xsize, scale.ysize).unwrap_or_else(|e| {
                eprintln!("Error: CppFreakMatcher::new failed: {:?}", e);
                std::process::exit(1);
            });

            let result = KpmRefDataSet::generate(
                &scale.img_bw,
                scale.xsize,
                scale.ysize,
                scale.dpi,
                KpmProcMode::FullSize,
                KpmCompMode::None,
                max_feat,
                1, // page_no = 1 (matching markerCreator.cpp)
                image_no as i32,
                &mut matcher,
            );

            match result {
                Ok(ref_data) => {
                    println!(
                        "  ({}, {}) {:.6} dpi: Freak features - {}",
                        scale.xsize, scale.ysize, scale.dpi, ref_data.num
                    );
                    combined = Some(match combined.take() {
                        Some(mut existing) => {
                            existing.merge(ref_data);
                            existing
                        }
                        None => ref_data,
                    });
                }
                Err(KpmError::InvalidInput(_)) => {
                    println!(
                        "  ({}, {}) {:.6} dpi: Freak features - 0 (scale too small)",
                        scale.xsize, scale.ysize, scale.dpi
                    );
                }
                Err(e) => {
                    eprintln!(
                        "Warning: KpmRefDataSet::generate failed for scale {}: {:?}",
                        image_no, e
                    );
                }
            }
        }
        println!();

        println!("Saving FeatureSet3 to {:?}...", fset3_path);
        if let Some(ref_data) = combined {
            let total_kpm = ref_data.num as usize;
            ref_data.save(&fset3_path).unwrap_or_else(|e| {
                eprintln!("Error: failed to save {:?}: {}", fset3_path, e);
                std::process::exit(1);
            });
            println!(
                "  Done. {} FREAK features total  ({}, {:.1}s)",
                total_kpm,
                file_kb(&fset3_path),
                step_start.elapsed().as_secs_f64()
            );
        } else {
            eprintln!("Warning: no KPM features extracted — .fset3 not written.");
        }
    }

    #[cfg(not(feature = "ffi-backend"))]
    println!("[fset3] skipped (build with --features ffi-backend to generate)");

    // -----------------------------------------------------------------------
    // Done
    // -----------------------------------------------------------------------
    let finish_time = Local::now();
    let elapsed = start.elapsed().as_secs_f64();
    println!();
    println!(
        "Generator finished at {}",
        finish_time.format("%Y-%m-%d %H:%M:%S %z")
    );
    println!("--");
    println!();
    println!("NFTMarkerGenerator took {:.6} seconds to execute", elapsed);
}
