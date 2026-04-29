use clap::Parser;
use image::ImageReader;
use std::fs::File;
use webarkitlib_rs::arlog_i;
use webarkitlib_rs::marker::ar_detect_marker;
use webarkitlib_rs::types::{
    AR2VideoBufferT, AR2VideoTimestampT, ARHandle, ARMatrixCodeType, ARParam, ARParamLT,
    ARParamLTf, ARPixelFormat, AR_MATRIX_CODE_DETECTION,
};
use webarkitlib_rs::version;

/// WebARKitLib-rs Barcode Detection Example
#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    /// Matrix code type to use for detection.
    /// Accepted values: 3x3, 3x3parity65, 3x3hamming63, 4x4, 4x4bch1393,
    ///                  4x4bch1355, 5x5, 5x5bch22125, 5x5bch2277, 6x6
    #[arg(short = 'm', long, default_value = "3x3")]
    matrix_code_type: String,

    /// Labeling threshold (0–255). When omitted, the example iterates over
    /// the range 60–180 (step 20) and stops on the first successful detection.
    #[arg(short = 't', long)]
    threshold: Option<i32>,

    /// Path to the input image. Defaults to the bundled 3x3 marker image.
    #[arg(short = 'i', long)]
    image: Option<String>,
}

fn parse_matrix_code_type(s: &str) -> Result<ARMatrixCodeType, String> {
    match s.to_lowercase().as_str() {
        "3x3" => Ok(ARMatrixCodeType::Code3x3),
        "3x3parity65" => Ok(ARMatrixCodeType::Code3x3Parity65),
        "3x3hamming63" => Ok(ARMatrixCodeType::Code3x3Hamming63),
        "4x4" => Ok(ARMatrixCodeType::Code4x4),
        "4x4bch1393" => Ok(ARMatrixCodeType::Code4x4BCH1393),
        "4x4bch1355" => Ok(ARMatrixCodeType::Code4x4BCH1355),
        "5x5" => Ok(ARMatrixCodeType::Code5x5),
        "5x5bch22125" => Ok(ARMatrixCodeType::Code5x5BCH22125),
        "5x5bch2277" => Ok(ARMatrixCodeType::Code5x5BCH2277),
        "6x6" => Ok(ARMatrixCodeType::Code6x6),
        other => Err(format!("Unknown matrix code type: '{other}'")),
    }
}

/// Return the first path that exists from the list of candidates.
fn resolve_path<'a>(candidates: &[&'a str]) -> &'a str {
    candidates
        .iter()
        .copied()
        .find(|p| std::path::Path::new(p).exists())
        .unwrap_or(candidates[0])
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Initialize Logging
    webarkitlib_rs::arlog::ar_log_init_default();

    let args = Args::parse();

    let code_type = parse_matrix_code_type(&args.matrix_code_type)?;
    arlog_i!("WebARKitLib-rs v{}", version::get_version());
    arlog_i!(
        "WebARKitLib-rs Barcode Detection Example (matrix_code_type={:?})",
        code_type
    );

    // 2. Load Camera Parameters
    let camera_para_path = resolve_path(&[
        "benchmarks/data/camera_para.dat",
        "../../benchmarks/data/camera_para.dat",
    ]);
    let param_file = File::open(camera_para_path)
        .map_err(|e| format!("Failed to open camera parameters at {camera_para_path}: {e}"))?;
    let mut param = ARParam::load(param_file).expect("Failed to read camera parameters");
    arlog_i!(
        "Loaded camera parameters from {}: xsize={}, ysize={}",
        camera_para_path,
        param.xsize,
        param.ysize
    );

    // 3. Resolve image path
    let default_image_primary = "crates/core/examples/Data/marker_05_3x3.jpg";
    let default_image_alt = "examples/Data/marker_05_3x3.jpg";
    let image_path_str: String = match &args.image {
        Some(p) => p.clone(),
        None => resolve_path(&[default_image_primary, default_image_alt]).to_owned(),
    };
    let image_path = image_path_str.as_str();

    arlog_i!("Loading image {}...", image_path);
    let full_img = ImageReader::open(image_path)
        .map_err(|e| format!("Failed to open image at {image_path}: {e}"))?
        .decode()
        .map_err(|e| format!("Failed to decode image at {image_path}: {e}"))?;
    let width = full_img.width() as i32;
    let height = full_img.height() as i32;
    arlog_i!("Image dimensions: {}x{}", width, height);

    // Derive output directory
    let data_dir = std::path::Path::new(image_path)
        .parent()
        .unwrap_or(std::path::Path::new("."));

    // Override the camera parameters with the actual image dimensions
    param.xsize = width;
    param.ysize = height;

    let luma_img = full_img.to_luma8();
    let color_img = full_img.to_rgb8();

    let rust_luma_path = data_dir.join("rust_luma.png");
    let rust_color_path = data_dir.join("rust_color.png");
    luma_img
        .save(&rust_luma_path)
        .expect("Failed to save luma image");
    color_img
        .save(&rust_color_path)
        .expect("Failed to save color image");

    // 4. Build threshold iterator: single value or sweep
    let thresholds: Box<dyn Iterator<Item = i32>> = match args.threshold {
        Some(t) => Box::new(std::iter::once(t)),
        None => Box::new((60..=180).step_by(20)),
    };

    for thresh in thresholds {
        arlog_i!("--- Testing BlackRegion, Threshold: {} ---", thresh);

        let mut ar_handle = ARHandle::default();
        ar_handle.xsize = width;
        ar_handle.ysize = height;
        ar_handle.ar_debug = 1;
        ar_handle.set_pixel_format(ARPixelFormat::RGB);
        ar_handle.ar_labeling_thresh = thresh;
        ar_handle.ar_labeling_mode = 0; // BlackRegion
        ar_handle.set_pattern_detection_mode(AR_MATRIX_CODE_DETECTION);
        ar_handle.set_matrix_code_type(code_type);

        let mut param_ltf = ARParamLTf::default();
        param_ltf.xsize = width;
        param_ltf.ysize = height;
        param_ltf.o2i = vec![0.0; (width * height * 2) as usize];
        for y in 0..height {
            for x in 0..width {
                let idx = ((y * width + x) * 2) as usize;
                param_ltf.o2i[idx] = x as f32;
                param_ltf.o2i[idx + 1] = y as f32;
            }
        }
        let mut param_lt = Box::new(ARParamLT {
            param: param.clone(),
            param_ltf,
        });
        ar_handle.ar_param_lt = &mut *param_lt;

        let frame = AR2VideoBufferT {
            buff: Some(color_img.clone().into_raw()),
            buff_luma: Some(luma_img.clone().into_raw()),
            buf_planes: None,
            buf_plane_count: 0,
            fill_flag: true,
            time: AR2VideoTimestampT { sec: 1, usec: 0 },
        };

        if let Ok(_) = ar_detect_marker(&mut ar_handle, &frame) {
            arlog_i!("Found {} square candidates.", ar_handle.marker2_num);
            arlog_i!("Found {} valid barcode markers.", ar_handle.marker_num);
            let mut found_valid = false;
            for i in 0..ar_handle.marker_num as usize {
                let m = &ar_handle.marker_info[i];
                arlog_i!("  >> MATCH: ID={}, CF={:.4}", m.id_matrix, m.cf_matrix);
                if m.id_matrix >= 0 {
                    found_valid = true;
                }
            }
            if found_valid {
                arlog_i!("Valid matrix code found! Stopping loop.");
                break;
            }
        }
    }

    Ok(())
}
