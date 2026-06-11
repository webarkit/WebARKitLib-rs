/*
 *  simple.rs
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

use image::ImageReader;
use std::fs::File;
use webarkitlib_rs::{
    arlog_e, arlog_i,
    image_proc::ARImageProcInfo,
    marker::ar_detect_marker,
    pattern::ar_patt_load,
    pose::{ar_3d_create_handle, ar_3d_delete_handle, ar_get_trans_mat_square},
    types::{
        AR2VideoBufferT, AR2VideoTimestampT, ARHandle, ARMatrixCodeType, ARParam, ARParamLT,
        ARParamLTf, ARPixelFormat,
    },
    version,
};

fn main() {
    webarkitlib_rs::arlog::ar_log_init_default();
    arlog_i!("WebARKitLib-rs v{}", version::get_version());
    arlog_i!("WebARKitLib Example: Simple Marker Detection");

    // Use command line args or defaults
    let args: Vec<String> = std::env::args().collect();

    let cparam_path_default = "benchmarks/data/camera_para.dat";
    let cparam_path = if args.len() > 1 {
        &args[1]
    } else {
        if !std::path::Path::new(cparam_path_default).exists()
            && std::path::Path::new("../../benchmarks/data/camera_para.dat").exists()
        {
            "../../benchmarks/data/camera_para.dat"
        } else {
            cparam_path_default
        }
    };

    let patt_path_default = "benchmarks/data/patt.hiro";
    let patt_path = if args.len() > 2 {
        &args[2]
    } else {
        if !std::path::Path::new(patt_path_default).exists()
            && std::path::Path::new("../../benchmarks/data/patt.hiro").exists()
        {
            "../../benchmarks/data/patt.hiro"
        } else {
            patt_path_default
        }
    };

    let img_path_default = "benchmarks/data/img.jpg";
    let img_path = if args.len() > 3 {
        &args[3]
    } else {
        if !std::path::Path::new(img_path_default).exists()
            && std::path::Path::new("../../benchmarks/data/img.jpg").exists()
        {
            "../../benchmarks/data/img.jpg"
        } else {
            img_path_default
        }
    };

    // Derive output paths from input image path for robustness
    let img_path_buf = std::path::PathBuf::from(img_path);
    let data_dir = img_path_buf.parent().unwrap_or(std::path::Path::new("."));
    let hiro_raw_path = data_dir.join("hiro.raw");
    let found_jpg_path = data_dir.join("found.jpg");

    // Load ARParam
    arlog_i!("Loading camera parameters from {}...", cparam_path);
    let param_file = File::open(cparam_path)
        .unwrap_or_else(|_| panic!("Failed to open camera parameters: {}", cparam_path));
    let param = ARParam::load(param_file).expect("Failed to read camera parameters");

    // Load image
    arlog_i!("Loading image {}...", img_path);
    let img = ImageReader::open(img_path)
        .unwrap_or_else(|_| panic!("Failed to open image: {}", img_path))
        .decode()
        .expect("Failed to decode image");
    let width = img.width() as i32;
    let height = img.height() as i32;
    arlog_i!("Image dimensions: {}x{}", width, height);

    let luma_img = img.to_luma8();
    let color_img = img.to_rgba8();

    // SAVE RAW LUMA FOR C BENCHMARK
    {
        use std::io::Write;
        arlog_i!("Exporting {} for C benchmark...", hiro_raw_path.display());
        let mut f = File::create(&hiro_raw_path)
            .unwrap_or_else(|_| panic!("Failed to create {}", hiro_raw_path.display()));
        f.write_all(luma_img.as_raw())
            .expect("Failed to write luma raw data");
    }

    // Calculate Otsu threshold
    let mut ipi = ARImageProcInfo::new(width, height);
    let otsu_thresh = ipi
        .luma_hist_and_otsu(luma_img.as_raw())
        .expect("Failed to calculate Otsu threshold");
    arlog_i!("Calculated Otsu threshold: {}", otsu_thresh);

    // Debug: Save thresholded image to disk so we can see what the AR tracker sees
    let thresh_path = data_dir.join("thresh.png");
    let mut thresh_img =
        image::GrayImage::from_vec(width as u32, height as u32, luma_img.as_raw().clone()).unwrap();
    for p in thresh_img.pixels_mut() {
        if p[0] <= otsu_thresh as u8 {
            p[0] = 0; // Black region
        } else {
            p[0] = 255; // White region
        }
    }
    thresh_img
        .save(&thresh_path)
        .expect("Failed to save thresholded image");
    arlog_i!("Saved thresholded image to {}", thresh_path.display());

    // We mock an identity lookup table for the image size to avoid distortion failure
    let mut param_ltf = ARParamLTf {
        xsize: width,
        ysize: height,
        x_off: 0,
        y_off: 0,
        o2i: vec![0.0; (width * height * 2) as usize],
        ..Default::default()
    };
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

    arlog_i!("Initializing AR3DHandle for pose estimation...");
    let mut ar3d_handle_ptr = ar_3d_create_handle(&param).expect("Failed to create AR3DHandle");

    // Initialize the main tracking handle
    let mut ar_handle = ARHandle {
        ar_debug: 1,
        xsize: width,
        ysize: height,
        ar_image_proc_mode: 0,        // FrameImage
        ar_pattern_detection_mode: 0, // Template matching color/mono
        ar_labeling_mode: 0,          // Black region
        ar_labeling_thresh: otsu_thresh as i32,
        patt_ratio: 0.5,
        matrix_code_type: ARMatrixCodeType::Code3x3,
        ar_param_lt: &mut *param_lt,
        ..Default::default()
    };
    ar_handle.set_pixel_format(ARPixelFormat::MONO); // raw luma input

    // Allocate pattern handle and load real pattern
    let mut patt_handle = webarkitlib_rs::types::ARPattHandle {
        patt_num_max: 50,
        patt_size: 16,
        pattf: vec![0; 50],
        patt: vec![vec![0; 16 * 16 * 3 * 4]; 50],
        pattpow: vec![0.0; 50 * 4],
        patt_bw: vec![vec![0; 16 * 16 * 4]; 50],
        pattpow_bw: vec![0.0; 50 * 4],
        ..Default::default()
    };

    arlog_i!("Loading hiro pattern from {}...", patt_path);
    match ar_patt_load(&mut patt_handle, patt_path) {
        Ok(idx) => arlog_i!("Pattern loaded successfully at index {}.", idx),
        Err(e) => {
            arlog_e!("Failed to load pattern: {}", e);
            return;
        }
    }

    // Pass the populated pattern handle into the main ARHandle map
    let mut boxed_patt_handle = Box::new(patt_handle);
    ar_handle.patt_handle = &mut *boxed_patt_handle;

    let luma_buffer_vec = luma_img.into_raw();
    let mut out_img = color_img.clone();

    // ARToolKit5's contract (cf. artoolkit5/video2.c:682): `buff` MUST hold
    // data in the format declared by ar_pixel_format. When pixel_format is
    // MONO, that means buff IS the luma data — buffLuma is just an alias.
    // Feeding RGBA into buff while declaring MONO produces silent low-cf
    // detections (see issue #103).
    let frame = AR2VideoBufferT {
        buff: Some(luma_buffer_vec.clone()), // pixfmt=MONO → buff IS luma
        buff_luma: Some(luma_buffer_vec.clone()),
        buf_planes: None,
        buf_plane_count: 0,
        fill_flag: true,
        time: AR2VideoTimestampT { sec: 1, usec: 0 },
    };

    arlog_i!("Passing image to ar_detect_marker...");

    // Run the marker detection pipeline
    match ar_detect_marker(&mut ar_handle, &frame) {
        Ok(_) => {
            arlog_i!("Detection pipeline finished successfully.");
            arlog_i!("Detected {} potential markers.", ar_handle.marker_num);

            for i in 0..ar_handle.marker_num as usize {
                let marker = &ar_handle.marker_info[i];
                arlog_i!("Marker [{}]:", i);
                arlog_i!("  Area: {}", marker.area);
                arlog_i!("  Pos (X,Y): {:.2}, {:.2}", marker.pos[0], marker.pos[1]);
                arlog_i!("  Confidence (CF): {:.4}", marker.cf);
                arlog_i!("  Matched ID: {}", marker.id);
                arlog_i!(
                    "  Template-match CF/ID: {:.4} / {}  (cfMatrix: {:.4}, idMatrix: {})",
                    marker.cf_patt,
                    marker.id_patt,
                    marker.cf_matrix,
                    marker.id_matrix
                );
                arlog_i!("  Orientation dir: {}", marker.dir);

                if marker.id >= 0 {
                    let v = marker.vertex;
                    let color = image::Rgba([0u8, 0u8, 255u8, 255u8]); // Blue rectangle
                    imageproc::drawing::draw_line_segment_mut(
                        &mut out_img,
                        (v[0][0] as f32, v[0][1] as f32),
                        (v[1][0] as f32, v[1][1] as f32),
                        color,
                    );
                    imageproc::drawing::draw_line_segment_mut(
                        &mut out_img,
                        (v[1][0] as f32, v[1][1] as f32),
                        (v[2][0] as f32, v[2][1] as f32),
                        color,
                    );
                    imageproc::drawing::draw_line_segment_mut(
                        &mut out_img,
                        (v[2][0] as f32, v[2][1] as f32),
                        (v[3][0] as f32, v[3][1] as f32),
                        color,
                    );
                    imageproc::drawing::draw_line_segment_mut(
                        &mut out_img,
                        (v[3][0] as f32, v[3][1] as f32),
                        (v[0][0] as f32, v[0][1] as f32),
                        color,
                    );

                    let mut trans_mat = [[0.0; 4]; 3];
                    // width parameter defaults to 80.0 mm in ARToolKit standard examples
                    let err = ar_get_trans_mat_square(
                        unsafe { &*ar3d_handle_ptr },
                        marker,
                        80.0,
                        &mut trans_mat,
                    )
                    .unwrap_or(100000000.0);
                    arlog_i!("  Extracted 3D Pose (ICP Error: {:.4}):", err);
                    arlog_i!(
                        "    [{:>8.4}, {:>8.4}, {:>8.4}, {:>8.4}]",
                        trans_mat[0][0],
                        trans_mat[0][1],
                        trans_mat[0][2],
                        trans_mat[0][3]
                    );
                    arlog_i!(
                        "    [{:>8.4}, {:>8.4}, {:>8.4}, {:>8.4}]",
                        trans_mat[1][0],
                        trans_mat[1][1],
                        trans_mat[1][2],
                        trans_mat[1][3]
                    );
                    arlog_i!(
                        "    [{:>8.4}, {:>8.4}, {:>8.4}, {:>8.4}]",
                        trans_mat[2][0],
                        trans_mat[2][1],
                        trans_mat[2][2],
                        trans_mat[2][3]
                    );
                }
            }

            arlog_i!("Saving results to {}...", found_jpg_path.display());
            let rgb_img = image::DynamicImage::ImageRgba8(out_img).into_rgb8();
            rgb_img
                .save(&found_jpg_path)
                .expect("Failed to save found.jpg");
        }
        Err(e) => arlog_e!("Error during marker detection: {}", e),
    }

    // Debug: Save colored label image to disk
    arlog_i!("Saving debug label image...");

    // Cleanup 3D Extrinsics Handle
    ar_3d_delete_handle(&mut ar3d_handle_ptr).expect("Failed to delete AR3DHandle");

    if ar_handle.label_info.label_image.is_empty() {
        arlog_i!("  (skipped: no label image — marker detection did not run)");
        return;
    }

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
    let label_path = data_dir.join("label.png");
    label_img
        .save(&label_path)
        .expect("Failed to save label.png");
    arlog_i!("Saved label debug image to {}", label_path.display());
}
