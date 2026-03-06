use webarkitlib_rs::marker::ar_detect_marker;
use webarkitlib_rs::types::{
    ARHandle, ARPixelFormat, ARMatrixCodeType, AR_MATRIX_CODE_DETECTION, 
    ARParam, ARParamLT, ARParamLTf, AR2VideoBufferT, AR2VideoTimestampT
};
use image::ImageReader;
use std::fs::File;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Initialize Logging
    std::env::set_var("RUST_LOG", "trace");
    env_logger::init();
    println!("WebARKitLib-rs Barcode Detection Example (Ultra-Low Threshold)");

    // 2. Load Camera Parameters
    let camera_para_path = "benchmarks/data/camera_para.dat";
    let param_file = File::open(camera_para_path).expect("Failed to open camera_para.dat");
    let mut param = ARParam::load(param_file).expect("Failed to read camera_para.dat");
    println!("Loaded camera_para.dat: xsize={}, ysize={}", param.xsize, param.ysize);
    
    // 3. Load Image
    let image_path = "crates/core/examples/Data/marker_05_3x3.jpg";
    println!("Loading image {}...", image_path);
    let full_img = ImageReader::open(image_path).unwrap().decode().unwrap();
    let width = full_img.width() as i32;
    let height = full_img.height() as i32;
    println!("Image dimensions: {}x{}", width, height);

    // Override the camera parameters with the actual image dimensions
    param.xsize = width;
    param.ysize = height;

    let luma_img = full_img.to_luma8();
    let color_img = full_img.to_rgb8();
    
    luma_img.save("target/rust_luma.png").unwrap();
    color_img.save("target/rust_color.png").unwrap();

    for thresh in (60..=180).step_by(20) {
        println!("\n--- Testing BlackRegion, Threshold: {} ---", thresh);
        
        let mut ar_handle = ARHandle::default();
        ar_handle.xsize = width;
        ar_handle.ysize = height;
        ar_handle.ar_debug = 1;
        ar_handle.set_pixel_format(ARPixelFormat::RGB); 
        ar_handle.ar_labeling_thresh = thresh;
        ar_handle.ar_labeling_mode = 0; // BlackRegion
        ar_handle.set_pattern_detection_mode(AR_MATRIX_CODE_DETECTION);
        ar_handle.set_matrix_code_type(ARMatrixCodeType::Code3x3); 
        
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
        let mut param_lt = Box::new(ARParamLT { param: param.clone(), param_ltf });
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
            println!("Found {} square candidates.", ar_handle.marker2_num);
            println!("Found {} valid barcode markers.", ar_handle.marker_num);
            let mut found_valid = false;
            for i in 0..ar_handle.marker_num as usize {
                let m = &ar_handle.marker_info[i];
                println!("  >> MATCH: ID={}, CF={:.4}", m.id_matrix, m.cf_matrix);
                if m.id_matrix >= 0 {
                    found_valid = true;
                }
            }
            if found_valid {
                println!("Valid matrix code found! Stopping loop.");
                break;
            }
        }
    }

    Ok(())
}
