use image::ImageReader;
use image::{GrayImage, RgbImage, Rgba, RgbaImage};
use imageproc::contrast::otsu_level;
use imageproc::drawing::draw_hollow_rect_mut;
use imageproc::rect::Rect;
use std::env;
use std::fs;
use std::io::Write;
use std::path::Path;
use std::str::FromStr;
use webarkitlib_rs::pattern::{ar_patt_get_image2, ar_patt_save};
use webarkitlib_rs::types::{ARMarkerInfo, ARParam, ARParamLT, ARParamLTf, ARPixelFormat};
use webarkitlib_rs::{arlog_e, arlog_i, arlog_w};

/*
 English documentation / notes for this example

 Purpose:
 - This example extracts a marker pattern (.patt) from an input image and optionally compares
   it with a reference file (e.g. patt.hiro). It also produces diagnostic images (extracted_pattern, overlay)
   and a CSV report when run in batch mode.

 Important options and behavior:
 - --with-border
     Indicates that the input image ALREADY CONTAINS the marker border. When set, the program
     DOES NOT add any border and proceeds directly to extraction.

 - --border N
     Forces adding a black border of N pixels on each side of the input image (override). If present,
     this option takes precedence over automatic calculation.

 - --patt-ratio F
     The ratio pattern/(pattern + 2*border). Default is 0.5.
     It is used to compute an automatic border size when none is provided and --with-border is not set.

 Automatic border calculation (current behavior):
 - If you do NOT pass --with-border and do NOT specify --border, the border (in pixels) is computed by:
     patt_ratio = pattern_side / full_marker_side
     full_marker_side = pattern_side / patt_ratio
     border = round((full_marker_side - pattern_side) / 2)

 - The code uses the minimum of width and height (min_dim) of the input image as the pattern_side to be
   conservative for non-square images. Thus:
     full_side = round(min_dim / patt_ratio)
     border = round((full_side - min_dim) / 2)

 - Example: input image 216 px, patt_ratio = 0.5
     full_side = round(216 / 0.5) = 432
     border = round((432 - 216) / 2) = 108
     resulting marker size = 108 + 216 + 108 = 432

 Tuning notes / options:
 - Rounding: the implementation currently uses round() (so 107.5 -> 108). If you prefer floor()
   to avoid expansion, we can change the rounding behavior.
 - Base dimension: we use min(width, height). If you prefer to always use width (or height), we can adapt this.
 - Border color: the added border is initialized to black (pixel 0). If you'd rather use white or another color,
   that can be changed easily.

 Outputs produced:
 - crates/core/examples/Data/experiments/generated*.patt  (created .patt files)
 - crates/core/examples/Data/experiments/extracted_pattern*.png  (extracted images)
 - crates/core/examples/Data/experiments/overlay*.png  (debug overlays with bounding box)
 - crates/core/examples/Data/experiments/report.csv  (if run with --batch)

 Example commands (PowerShell):
   cargo run --manifest-path crates/core/Cargo.toml --example generate_patt -- --help
   cargo run --manifest-path crates/core/Cargo.toml --example generate_patt -- --input crates/core/examples/Data/HIRO-test.jpg
   cargo run --manifest-path crates/core/Cargo.toml --example generate_patt -- --input img.jpg --with-border
   cargo run --manifest-path crates/core/Cargo.toml --example generate_patt -- --input img.jpg --border 107 --patt-ratio 0.5
   cargo run --manifest-path crates/core/Cargo.toml --example generate_patt -- --input img.jpg --ref ../../benchmarks/data/patt.hiro

 If you want me to change behavior (floor/ceil, use width instead of min_dim, white border, logging of the
 computed border value, generating all 4 extracted rotations as separate PNGs, etc.), tell me which option
 and I'll apply it.
*/

fn print_help_and_exit() {
    eprintln!("generate_patt example — usage:");
    eprintln!("  --input PATH           input image (default: examples/Data/HIRO-test.jpg)");
    eprintln!("  --out PATH             output .patt path (default: ./crates/core/examples/Data/generated.patt)");
    eprintln!("  --camera PATH          camera_para.dat path to build ARParamLT (optional)");
    eprintln!("  --border N             add black border of N pixels around input (default: 0)");
    eprintln!("  --patt-size N          pattern size in pixels (default: 16)");
    eprintln!(
        "  --sample-factor N      sample factor multiplier passed to extraction (default: 4)"
    );
    eprintln!("  --flip-v               flip input image vertically before processing");
    eprintln!("  --channel-order rgb|bgr interpret input buffer as rgb or bgr (default: rgb)");
    eprintln!("  --with-border          indicate the INPUT IMAGE ALREADY CONTAINS THE MARKER BORDER (no auto-add)");
    eprintln!("  --patt-ratio F         pattern ratio (pattern/(pattern+2*border)), default 0.5");
    eprintln!("  --batch                run automatic experiments and write CSV report");
    eprintln!("  --quiet                reduce printed output");
    eprintln!("  --verbose              print verbose diagnostic info (full_side, min_dim) when border is auto-computed");
    eprintln!("  --debug                print very verbose intermediate extraction values (threshold, bbox, vertices, params)");
    eprintln!("  --help                 show this help");
    std::process::exit(0);
}

#[derive(Clone)]
struct Config {
    input: String,
    output: String,
    camera: Option<String>,
    ref_patt: Option<String>,
    border: usize,
    patt_size: usize,
    sample_factor: usize,
    flip_v: bool,
    channel_order: String,
    quiet: bool,
    with_border: bool,
    patt_ratio: f64,
    // border_color removed: always use black border when adding
    verbose: bool,
    debug: bool,
}

fn default_paths() -> (String, String) {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let input = Path::new(manifest_dir)
        .join("examples")
        .join("Data")
        .join("HIRO-test.jpg");
    let out = Path::new("./crates/core/examples/Data/generated.patt");
    (
        input.to_string_lossy().into_owned(),
        out.to_string_lossy().into_owned(),
    )
}

fn run_once(
    cfg: &Config,
    suffix: &str,
) -> Result<(usize, u8, u8, f64, Option<f64>, Option<usize>, Option<bool>), String> {
    // Load image
    let image = ImageReader::open(&cfg.input)
        .map_err(|e| format!("open image: {}", e))?
        .decode()
        .map_err(|e| format!("decode image: {}", e))?;
    let mut image_buf = image.to_rgb8();
    let mut width = image_buf.width();
    let mut height = image_buf.height();

    // optional border: either explicit via --border or auto-computed when --with-border is NOT set
    let mut applied_border: usize = cfg.border;
    // store computed values when auto-calculated so we can log them later
    let mut computed_full_side: Option<f64> = None;
    let mut computed_min_dim: Option<f64> = None;
    let mut used_dimension: Option<String> = None; // e.g. "min" or "width"
    if applied_border == 0 && !cfg.with_border {
        // compute border so that patt_ratio = pattern_side / full_marker_side
        // full_marker_side = pattern_side / patt_ratio
        // border = (full_marker_side - pattern_side) / 2
        // use the minimum dimension of input to be conservative
        let min_dim = (width.min(height)) as f64;
        // record used dimension and value
        computed_min_dim = Some(min_dim);
        used_dimension = Some("min".into());
        let full_side = (min_dim / cfg.patt_ratio).round();
        // record computed full_side for verbose logging
        computed_full_side = Some(full_side);
        let border_f = ((full_side - min_dim) / 2.0).round();
        if border_f > 0.0 {
            applied_border = border_f as usize;
        } else {
            applied_border = 0;
        }
    }

    // Determine how border was chosen for logging
    let border_source = if cfg.border > 0 {
        "explicit"
    } else if applied_border > 0 {
        "auto"
    } else {
        "none"
    };

    if applied_border > 0 {
        // log the computed/applied border
        arlog_i!(
            "Applied border = {} px (source={})",
            applied_border,
            border_source
        );

        // If border was auto-computed and --verbose was passed, print verbose info: full_side, patt_ratio and min_dim
        if border_source == "auto" && cfg.verbose {
            if let Some(fs) = computed_full_side {
                if let Some(md) = computed_min_dim.as_ref() {
                    let used = used_dimension.as_deref().unwrap_or("min");
                    arlog_i!("Verbose: computed full_marker_side = {}  patt_ratio = {}  used={} min_dim={}", fs, cfg.patt_ratio, used, md);
                } else {
                    arlog_i!(
                        "Verbose: computed full_marker_side = {}  patt_ratio = {}",
                        fs,
                        cfg.patt_ratio
                    );
                }
            }
        }

        let b = applied_border as u32;
        let new_w = width + 2 * b;
        let new_h = height + 2 * b;
        // fill border with black (ARToolKit expects marker border as black/white contrast; black border is used here)
        let mut framed: RgbImage = RgbImage::from_pixel(new_w, new_h, image::Rgb([0u8, 0u8, 0u8]));
        for y in 0..height {
            for x in 0..width {
                let p = image_buf.get_pixel(x, y);
                framed.put_pixel(x + b, y + b, *p);
            }
        }
        image_buf = framed;
        width = new_w;
        height = new_h;
    } else {
        arlog_i!("No border applied (source={})", border_source);
    }

    // flip
    if cfg.flip_v {
        let row_bytes = (width as usize) * 3;
        let mut flipped = Vec::with_capacity((width * height * 3) as usize);
        for y in 0..(height as usize) {
            let src_row = (height as usize - 1 - y) * row_bytes;
            flipped.extend_from_slice(&image_buf.as_raw()[src_row..src_row + row_bytes]);
        }
        image_buf =
            RgbImage::from_raw(width, height, flipped).ok_or("failed to build flipped image")?;
    }

    // channel order
    if cfg.channel_order == "bgr" {
        let mut raw = image_buf.into_raw();
        for i in (0..raw.len()).step_by(3) {
            raw.swap(i, i + 2);
        }
        image_buf = RgbImage::from_raw(width, height, raw).ok_or("failed to rebuild bgr image")?;
    }

    // grayscale for Otsu
    let gray: GrayImage = image::DynamicImage::ImageRgb8(image_buf.clone()).to_luma8();

    // keep a copy for overlay visualization (RGBA) before converting to raw
    let overlay_png: RgbaImage = image::DynamicImage::ImageRgb8(image_buf.clone()).to_rgba8();
    let mut overlay_img = overlay_png.clone();

    let img = image_buf.into_raw();
    let xsize = width as usize;
    let ysize = height as usize;
    let pixel_format = ARPixelFormat::RGB;
    let patt_ratio = cfg.patt_ratio;

    // camera param ltf
    let param_ltf: ARParamLTf = {
        let cparam_path = if let Some(ref p) = cfg.camera {
            std::path::Path::new(p).to_path_buf()
        } else {
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("examples")
                .join("Data")
                .join("camera_para.dat")
        };
        let param = if cparam_path.exists() {
            let f = fs::File::open(&cparam_path).map_err(|e| format!("open camera: {}", e))?;
            ARParam::load(f).map_err(|e| format!("load camera: {}", e))?
        } else {
            ARParam::default()
        };
        ARParamLT::new_basic(param).param_ltf.clone()
    };

    // detect via Otsu
    let thresh = otsu_level(&gray);
    if cfg.debug {
        arlog_i!("Debug: Otsu threshold = {}", thresh);
    }
    let mut minx = width as i32;
    let mut miny = height as i32;
    let mut maxx = 0i32;
    let mut maxy = 0i32;
    for (y, row) in gray.rows().enumerate() {
        for (x, pixel) in row.enumerate() {
            let v = pixel[0];
            if v < thresh {
                if (x as i32) < minx {
                    minx = x as i32;
                }
                if (y as i32) < miny {
                    miny = y as i32;
                }
                if (x as i32) > maxx {
                    maxx = x as i32;
                }
                if (y as i32) > maxy {
                    maxy = y as i32;
                }
            }
        }
    }
    if maxx == 0 && maxy == 0 {
        let side = ((width.min(height) as f32) * 0.6) as i32;
        let cx = (width as i32) / 2;
        let cy = (height as i32) / 2;
        minx = cx - side / 2;
        if minx < 0 {
            minx = 0
        }
        miny = cy - side / 2;
        if miny < 0 {
            miny = 0
        }
        maxx = cx + side / 2;
        if maxx >= width as i32 {
            maxx = width as i32 - 1
        }
        maxy = cy + side / 2;
        if maxy >= height as i32 {
            maxy = height as i32 - 1
        }
        if cfg.debug {
            arlog_i!(
                "Debug: fallback bbox used -> minx={}, miny={}, maxx={}, maxy={}",
                minx,
                miny,
                maxx,
                maxy
            );
        }
    }
    if cfg.debug {
        arlog_i!(
            "Debug: bbox -> minx={}, miny={}, maxx={}, maxy={}",
            minx,
            miny,
            maxx,
            maxy
        );
    }

    let mut marker = ARMarkerInfo::default();
    marker.vertex = [
        [minx as f64, miny as f64],
        [maxx as f64, miny as f64],
        [maxx as f64, maxy as f64],
        [minx as f64, maxy as f64],
    ];
    if cfg.debug {
        arlog_i!("Debug: marker vertices: {:?}", marker.vertex);
    }

    // prepare output names
    let out_dir = Path::new("./crates/core/examples/Data/experiments");
    let _ = fs::create_dir_all(out_dir);
    let extracted_name = format!("extracted_pattern{}.png", suffix);
    let patt_name = format!("generated{}.patt", suffix);
    let extracted_path = out_dir.join(&extracted_name);
    let patt_path = out_dir.join(&patt_name);

    // draw bounding box on overlay image and save for diagnosis
    // Clamp coordinates within image
    let ox = overlay_img.width() as i32;
    let oy = overlay_img.height() as i32;
    let bx0 = minx.max(0).min(ox - 1) as i32;
    let by0 = miny.max(0).min(oy - 1) as i32;
    let bx1 = maxx.max(0).min(ox - 1) as i32;
    let by1 = maxy.max(0).min(oy - 1) as i32;
    let rect = Rect::at(bx0, by0).of_size((bx1 - bx0).max(1) as u32, (by1 - by0).max(1) as u32);
    draw_hollow_rect_mut(&mut overlay_img, rect, Rgba([255u8, 0u8, 0u8, 255u8]));
    let overlay_name = format!("overlay{}.png", suffix);
    let overlay_path = out_dir.join(&overlay_name);
    let _ = overlay_img.save(&overlay_path);

    // extract pattern image
    let mut ext_patt = vec![0u8; (cfg.patt_size * cfg.patt_size * 3) as usize];
    if cfg.debug {
        arlog_i!("Debug: calling ar_patt_get_image2 with patt_size={} sample_size={} xsize={} ysize={} pixel_format={:?}", cfg.patt_size, cfg.patt_size * cfg.sample_factor, xsize, ysize, pixel_format);
    }
    if let Err(e) = ar_patt_get_image2(
        0,
        ARPixelFormat::RGB as i32,
        cfg.patt_size as usize,
        (cfg.patt_size * cfg.sample_factor) as usize,
        &img,
        xsize,
        ysize,
        pixel_format,
        &param_ltf,
        &marker.vertex,
        patt_ratio,
        &mut ext_patt,
    ) {
        return Err(format!("ar_patt_get_image2 failed: {}", e));
    }
    if cfg.debug {
        arlog_i!(
            "Debug: ar_patt_get_image2 returned, ext_patt.len()={}",
            ext_patt.len()
        );
    }

    // save extracted PNG
    let mut out_img = image::RgbImage::new(cfg.patt_size as u32, cfg.patt_size as u32);
    for y in 0..cfg.patt_size as usize {
        for x in 0..cfg.patt_size as usize {
            let idx = (y * cfg.patt_size as usize + x) * 3;
            let b = ext_patt[idx];
            let g = ext_patt[idx + 1];
            let r = ext_patt[idx + 2];
            out_img.put_pixel(x as u32, y as u32, image::Rgb([r, g, b]));
        }
    }
    out_img
        .save(&extracted_path)
        .map_err(|e| format!("save extracted: {}", e))?;

    // save .patt
    if cfg.debug {
        arlog_i!(
            "Debug: calling ar_patt_save with patt_ratio={} patt_size={} patt_path={}",
            patt_ratio,
            cfg.patt_size,
            patt_path.display()
        );
    }
    ar_patt_save(
        &img,
        xsize,
        ysize,
        pixel_format,
        &param_ltf,
        0,
        &marker,
        patt_ratio,
        cfg.patt_size as usize,
        &patt_path,
    )
    .map_err(|e| format!("ar_patt_save failed: {}", e))?;
    if cfg.debug {
        arlog_i!("Debug: ar_patt_save completed");
    }

    // compute stats
    let saved_content = fs::read_to_string(&patt_path).map_err(|e| format!("read patt: {}", e))?;
    let mut vals: Vec<i64> = Vec::new();
    for tok in saved_content.split_whitespace() {
        if let Ok(v) = i64::from_str(tok) {
            if (0..=255).contains(&v) {
                vals.push(v);
            }
        }
    }
    if vals.is_empty() {
        return Err("no numeric tokens in .patt".into());
    }
    let count = vals.len();
    let min = *vals.iter().min().unwrap() as u8;
    let max = *vals.iter().max().unwrap() as u8;
    let sum: i64 = vals.iter().sum();
    let mean = (sum as f64) / (count as f64);

    // Optional comparison with reference .patt (patt.hiro)
    let mut ref_mean = None::<f64>;
    let mut ref_best_idx = None::<usize>;
    let mut ref_best_flip = None::<bool>;
    if let Some(ref_path) = &cfg.ref_patt {
        // parse generated blocks
        if let Ok(gen_blocks) = parse_patt_blocks(&saved_content, cfg.patt_size) {
            if let Ok(ref_content) = fs::read_to_string(ref_path) {
                if let Ok(ref_blocks) = parse_patt_blocks(&ref_content, cfg.patt_size) {
                    // compare gen_blocks[0] against each ref block, with optional flip
                    let gen0 = &gen_blocks[0];
                    let mut best = f64::INFINITY;
                    let mut best_idx = 0usize;
                    let mut best_flip = false;
                    for (i, rb) in ref_blocks.iter().enumerate() {
                        // direct
                        let d = mean_abs_diff_u8(gen0, rb);
                        if d < best {
                            best = d;
                            best_idx = i;
                            best_flip = false;
                        }
                        // vertical flip of ref
                        let rb_flip = flip_vert_u8(rb, cfg.patt_size);
                        let d2 = mean_abs_diff_u8(gen0, &rb_flip);
                        if d2 < best {
                            best = d2;
                            best_idx = i;
                            best_flip = true;
                        }
                    }
                    ref_mean = Some(best);
                    ref_best_idx = Some(best_idx);
                    ref_best_flip = Some(best_flip);
                }
            }
        }
    }

    if !cfg.quiet {
        if let Some(rm) = ref_mean {
            arlog_i!("Wrote: {}  {}  count={} min={} max={} mean={:.2} ref_mean={:.2} ref_idx={} ref_flip={}", patt_path.display(), extracted_path.display(), count, min, max, mean, rm, ref_best_idx.unwrap_or(0), ref_best_flip.unwrap_or(false));
        } else {
            arlog_i!(
                "Wrote: {}  {}  count={} min={} max={} mean={:.2}",
                patt_path.display(),
                extracted_path.display(),
                count,
                min,
                max,
                mean
            );
        }
    }

    Ok((count, min, max, mean, ref_mean, ref_best_idx, ref_best_flip))
}

// Parse .patt textual content into up to 4 orientation blocks. Each block is a Vec<u8>
fn parse_patt_blocks(content: &str, patt_size: usize) -> Result<Vec<Vec<u8>>, String> {
    let mut tokens: Vec<u8> = Vec::new();
    for tok in content.split_whitespace() {
        if let Ok(v) = u8::from_str(tok) {
            tokens.push(v);
        }
    }
    let block_len = patt_size * patt_size * 3;
    if tokens.len() == block_len {
        return Ok(vec![tokens]);
    }
    if tokens.len() % block_len != 0 {
        return Err(format!(
            "unexpected token count {} (not multiple of block_len {})",
            tokens.len(),
            block_len
        ));
    }
    let blocks = tokens.chunks(block_len).map(|c| c.to_vec()).collect();
    Ok(blocks)
}

fn mean_abs_diff_u8(a: &Vec<u8>, b: &Vec<u8>) -> f64 {
    if a.len() != b.len() {
        return f64::INFINITY;
    }
    let mut s: f64 = 0.0;
    for i in 0..a.len() {
        s += (a[i] as f64 - b[i] as f64).abs();
    }
    s / (a.len() as f64)
}

fn flip_vert_u8(buf: &Vec<u8>, patt_size: usize) -> Vec<u8> {
    let mut out = vec![0u8; buf.len()];
    let row_bytes = patt_size * 3;
    for y in 0..patt_size {
        let src_row = y * row_bytes;
        let dst_row = (patt_size - 1 - y) * row_bytes;
        out[dst_row..dst_row + row_bytes].copy_from_slice(&buf[src_row..src_row + row_bytes]);
    }
    out
}

fn main() {
    webarkitlib_rs::arlog::ar_log_init_default();
    // parse args
    let (default_input, default_out) = default_paths();
    let mut cfg = Config {
        input: default_input,
        output: default_out,
        camera: None,
        ref_patt: None,
        border: 0,
        patt_size: 16,
        sample_factor: 4,
        flip_v: false,
        channel_order: "rgb".into(),
        quiet: false,
        with_border: false,
        patt_ratio: 0.5,
        verbose: false,
        debug: false,
    };

    let mut batch = false;
    let mut args = env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--help" | "-h" => print_help_and_exit(),
            "--input" => {
                if let Some(v) = args.next() {
                    cfg.input = v;
                }
            }
            "--out" => {
                if let Some(v) = args.next() {
                    cfg.output = v;
                }
            }
            "--camera" => {
                if let Some(v) = args.next() {
                    cfg.camera = Some(v);
                }
            }
            "--ref" => {
                if let Some(v) = args.next() {
                    cfg.ref_patt = Some(v);
                }
            }
            "--border" => {
                if let Some(v) = args.next() {
                    cfg.border = usize::from_str(&v).unwrap_or(0);
                }
            }
            "--with-border" => {
                cfg.with_border = true;
            }
            "--patt-ratio" => {
                if let Some(v) = args.next() {
                    cfg.patt_ratio = f64::from_str(&v).unwrap_or(0.5);
                }
            }
            "--patt-size" => {
                if let Some(v) = args.next() {
                    cfg.patt_size = usize::from_str(&v).unwrap_or(16);
                }
            }
            "--sample-factor" => {
                if let Some(v) = args.next() {
                    cfg.sample_factor = usize::from_str(&v).unwrap_or(4);
                }
            }
            "--flip-v" => cfg.flip_v = true,
            "--verbose" => cfg.verbose = true,
            "--debug" => cfg.debug = true,
            "--channel-order" => {
                if let Some(v) = args.next() {
                    cfg.channel_order = v.to_lowercase();
                }
            }
            "--batch" => batch = true,
            "--quiet" => cfg.quiet = true,
            other => arlog_w!("Unknown arg: {}", other),
        }
    }

    if batch {
        // prepare experiments combos
        let borders = vec![0usize, 8usize, 16usize];
        let flips = vec![false, true];
        let channels = vec!["rgb", "bgr"];
        let out_dir = Path::new("./crates/core/examples/Data/experiments");
        let _ = fs::create_dir_all(out_dir);
        let mut csv = fs::File::create(out_dir.join("report.csv")).expect("create csv");
        writeln!(csv, "border,flip,channel,patt_size,sample_factor,count,min,max,mean,ref_mean,ref_idx,ref_flip,extracted,patt").ok();

        for &b in &borders {
            for &f in &flips {
                for &ch in &channels {
                    let mut c = cfg.clone();
                    c.border = b;
                    c.flip_v = f;
                    c.channel_order = ch.into();
                    let suffix = format!("_b{}_{:1}_{:}", b, if f { 1 } else { 0 }, ch);
                    match run_once(&c, &suffix) {
                        Ok((count, min, max, mean, ref_mean, ref_idx, ref_flip)) => {
                            let extracted = format!("extracted_pattern{}.png", suffix);
                            let patt = format!("generated{}.patt", suffix);
                            let ref_mean_str: String = ref_mean
                                .map(|v| format!("{:.2}", v))
                                .unwrap_or_else(|| String::new());
                            let ref_idx_str: String = ref_idx
                                .map(|i| i.to_string())
                                .unwrap_or_else(|| String::new());
                            let ref_flip_str: String = ref_flip
                                .map(|b| {
                                    if b {
                                        String::from("1")
                                    } else {
                                        String::from("0")
                                    }
                                })
                                .unwrap_or_else(|| String::new());
                            writeln!(
                                csv,
                                "{},{},{},{},{},{},{},{},{:.2},{},{},{},{},{}",
                                b,
                                f as u8,
                                ch,
                                c.patt_size,
                                c.sample_factor,
                                count,
                                min,
                                max,
                                mean,
                                ref_mean_str,
                                ref_idx_str,
                                ref_flip_str,
                                extracted,
                                patt
                            )
                            .ok();
                        }
                        Err(e) => {
                            writeln!(csv, "{},{},{},ERROR,,,,,,{}", b, f as u8, ch, e).ok();
                        }
                    }
                }
            }
        }
        arlog_i!("Batch experiments finished. Report: ./crates/core/examples/Data/experiments/report.csv");
        return;
    }

    // Single run
    match run_once(&cfg, "") {
        Ok((_count, _min, _max, _mean, _ref_mean, _ref_idx, _ref_flip)) => {
            if !cfg.quiet {
                arlog_i!("Done.");
            }
        }
        Err(e) => arlog_e!("Error: {}", e),
    }
}
