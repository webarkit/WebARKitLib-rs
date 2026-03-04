use core::{
    marker::ar_detect_marker,
    pattern::ar_patt_load,
    pose::{ar_3d_create_handle, ar_get_trans_mat_square, ar_3d_delete_handle},
    types::{AR2VideoBufferT, AR2VideoTimestampT, ARHandle, ARMatrixCodeType, ARPixelFormat, ARParamLT, ARParamLTf, ARParam},
    image_proc::ARImageProcInfo,
};
use image::ImageReader;
use std::fs::File;

fn main() {
    env_logger::init();
    println!("WebARKitLib Example: Simple Marker Detection");

    // Use command line args or defaults
    let args: Vec<String> = std::env::args().collect();
    let cparam_path = if args.len() > 1 { &args[1] } else { "benchmarks/data/camera_para.dat" };
    let patt_path = if args.len() > 2 { &args[2] } else { "benchmarks/data/patt.hiro" };
    let img_path = if args.len() > 3 { &args[3] } else { "benchmarks/data/img.jpg" };

    // Load ARParam
    println!("Loading camera parameters from {}...", cparam_path);
    let param_file = File::open(cparam_path).expect("Failed to open camera_para.dat");
    let param = ARParam::load(param_file).expect("Failed to read camera_para.dat");
    
    // Load image
    println!("Loading image {}...", img_path);
    let img = ImageReader::open(img_path).unwrap().decode().unwrap();
    let width = img.width() as i32;
    let height = img.height() as i32;
    println!("Image dimensions: {}x{}", width, height);

    let luma_img = img.to_luma8();
    let color_img = img.to_rgba8(); 
    
    // SAVE RAW LUMA FOR C BENCHMARK
    {
        use std::io::Write;
        let mut f = File::create("../../benchmarks/data/hiro.raw").expect("Failed to create hiro.raw");
        f.write_all(luma_img.as_raw()).expect("Failed to write hiro.raw");
        println!("Exported benchmarks/data/hiro.raw for C benchmark.");
    }

    // Calculate Otsu threshold
    let mut ipi = ARImageProcInfo::new(width, height);
    let otsu_thresh = ipi.luma_hist_and_otsu(luma_img.as_raw()).expect("Failed to calculate Otsu threshold");
    println!("Calculated Otsu threshold: {}", otsu_thresh);
    
    // Debug: Save thresholded image to disk so we can see what the AR tracker sees
    let mut thresh_img = image::GrayImage::from_vec(width as u32, height as u32, luma_img.as_raw().clone()).unwrap();
    for p in thresh_img.pixels_mut() {
        if p[0] <= otsu_thresh as u8 {
            p[0] = 0; // Black region
        } else {
            p[0] = 255; // White region
        }
    }
    thresh_img.save("../../benchmarks/data/thresh.png").unwrap();
    
    // We mock an identity lookup table for the image size to avoid distortion failure
    let mut param_ltf = ARParamLTf::default();
    param_ltf.xsize = width;
    param_ltf.ysize = height;
    param_ltf.x_off = 0;
    param_ltf.y_off = 0;
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

    println!("Initializing AR3DHandle for pose estimation...");
    let mut ar3d_handle_ptr = ar_3d_create_handle(&param).expect("Failed to create AR3DHandle");

    // Initialize the main tracking handle
    let mut ar_handle = ARHandle::default();
    ar_handle.ar_debug = 1;
    ar_handle.xsize = width;
    ar_handle.ysize = height;
    ar_handle.ar_pixel_format = ARPixelFormat::Invalid; // N/A for raw Luma
    ar_handle.ar_image_proc_mode = 0; // FrameImage
    ar_handle.ar_pattern_detection_mode = 0; // Template matching color/mono
    ar_handle.ar_labeling_mode = 0; // Black region
    ar_handle.ar_labeling_thresh = otsu_thresh as i32;
    ar_handle.patt_ratio = 0.5;
    ar_handle.matrix_code_type = ARMatrixCodeType::Code3x3;
    ar_handle.ar_param_lt = &mut *param_lt;

    // Allocate pattern handle and load real pattern
    let mut patt_handle = core::types::ARPattHandle::default();
    patt_handle.patt_num_max = 50;
    patt_handle.patt_size = 16;
    patt_handle.pattf = vec![0; 50];
    patt_handle.patt = vec![vec![0; 16 * 16 * 3 * 4]; 50]; 
    patt_handle.pattpow = vec![0.0; 50 * 4];
    patt_handle.patt_bw = vec![vec![0; 16 * 16 * 4]; 50];
    patt_handle.pattpow_bw = vec![0.0; 50 * 4];
    
    println!("Loading hiro pattern from {}...", patt_path);
    match ar_patt_load(&mut patt_handle, patt_path) {
        Ok(idx) => println!("Pattern loaded successfully at index {}.", idx),
        Err(e) => {
            eprintln!("Failed to load pattern: {}", e);
            return;
        }
    }

    // Pass the populated pattern handle into the main ARHandle map
    let mut boxed_patt_handle = Box::new(patt_handle);
    ar_handle.patt_handle = &mut *boxed_patt_handle;
    
    let luma_buffer_vec = luma_img.into_raw();
    let mut out_img = color_img.clone();
    let color_buffer_vec = color_img.into_raw();
    
    // Construct the AR2VideoBufferT pointing to our image data
    let frame = AR2VideoBufferT {
        buff: Some(color_buffer_vec),       // Color data
        buff_luma: Some(luma_buffer_vec),          // Gray data used for binarization
        buf_planes: None,
        buf_plane_count: 0,
        fill_flag: true,
        time: AR2VideoTimestampT { sec: 1, usec: 0 },
    };

    println!("Passing image to ar_detect_marker...");
    
    // Run the marker detection pipeline
    match ar_detect_marker(&mut ar_handle, &frame) {
        Ok(_) => {
            println!("Detection pipeline finished successfully.");
            println!("Detected {} potential markers.", ar_handle.marker_num);
            
            for i in 0..ar_handle.marker_num as usize {
                let marker = &ar_handle.marker_info[i];
                println!("Marker [{}]:", i);
                println!("  Area: {}", marker.area);
                println!("  Pos (X,Y): {:.2}, {:.2}", marker.pos[0], marker.pos[1]);
                println!("  Confidence (CF): {:.4}", marker.cf);
                println!("  Matched ID: {}", marker.id);
                println!("  Orientation dir: {}", marker.dir);

                if marker.id >= 0 {
                    let v = marker.vertex;
                    let color = image::Rgba([0u8, 0u8, 255u8, 255u8]); // Blue rectangle
                    imageproc::drawing::draw_line_segment_mut(&mut out_img, (v[0][0] as f32, v[0][1] as f32), (v[1][0] as f32, v[1][1] as f32), color);
                    imageproc::drawing::draw_line_segment_mut(&mut out_img, (v[1][0] as f32, v[1][1] as f32), (v[2][0] as f32, v[2][1] as f32), color);
                    imageproc::drawing::draw_line_segment_mut(&mut out_img, (v[2][0] as f32, v[2][1] as f32), (v[3][0] as f32, v[3][1] as f32), color);
                    imageproc::drawing::draw_line_segment_mut(&mut out_img, (v[3][0] as f32, v[3][1] as f32), (v[0][0] as f32, v[0][1] as f32), color);

                    let mut trans_mat = [[0.0; 4]; 3];
                    // width parameter defaults to 80.0 mm in ARToolKit standard examples
                    let err = ar_get_trans_mat_square(unsafe { &*ar3d_handle_ptr }, marker, 80.0, &mut trans_mat).unwrap_or(100000000.0);
                    println!("  Extracted 3D Pose (ICP Error: {:.4}):", err);
                    println!("    [{:>8.4}, {:>8.4}, {:>8.4}, {:>8.4}]", trans_mat[0][0], trans_mat[0][1], trans_mat[0][2], trans_mat[0][3]);
                    println!("    [{:>8.4}, {:>8.4}, {:>8.4}, {:>8.4}]", trans_mat[1][0], trans_mat[1][1], trans_mat[1][2], trans_mat[1][3]);
                    println!("    [{:>8.4}, {:>8.4}, {:>8.4}, {:>8.4}]", trans_mat[2][0], trans_mat[2][1], trans_mat[2][2], trans_mat[2][3]);
                }
            }

            println!("Saving found.jpg...");
            let rgb_img = image::DynamicImage::ImageRgba8(out_img).into_rgb8();
            rgb_img.save("examples/Data/found.jpg").unwrap();
        },
        Err(e) => eprintln!("Error during marker detection: {}", e)
    }

    // Debug: Save colored label image to disk
    println!("Saving debug label image...");
    
    // Cleanup 3D Extrinsics Handle
    ar_3d_delete_handle(&mut ar3d_handle_ptr).expect("Failed to delete AR3DHandle");
    let mut color_map = std::collections::HashMap::new();
    let mut label_img = image::RgbImage::new(width as u32, height as u32);
    
    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) as usize;
            let label_val = ar_handle.label_info.label_image[idx];
            
            if label_val > 0 {
                let color = color_map.entry(label_val).or_insert_with(|| {
                    let r = (label_val.wrapping_mul(83)) % 255;
                    let g = (label_val.wrapping_mul(123)) % 255;
                    let b = (label_val.wrapping_mul(211)) % 255;
                    image::Rgb([r as u8, g as u8, b as u8])
                });
                label_img.put_pixel(x as u32, y as u32, *color);
            } else {
                label_img.put_pixel(x as u32, y as u32, image::Rgb([0, 0, 0])); // Black for bg
            }
        }
    }
    label_img.save("examples/Data/label.png").unwrap();
}
