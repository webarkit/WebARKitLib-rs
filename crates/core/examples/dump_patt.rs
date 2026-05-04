/*
 *  dump_patt.rs
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

//! Diagnostic harness for issue #103 (low CF from template matching).
//!
//! Mirrors the C-side `dump_patt` (`benchmarks/c_benchmark/dump_patt.c`):
//! runs `ar_detect_marker` once and re-extracts the matched pattern patch
//! via `ar_patt_get_image` using the contour data preserved in
//! `ar_handle.marker_info2[sel]`. The extracted patch is written to disk as
//! raw bytes for byte-level comparison with the C reference.
//!
//! Outputs (under `--out-dir`, default `benchmarks/data/`):
//!   - `rs_ext_patt.bin`        raw ext_patt bytes
//!   - `rs_ext_patt_meta.txt`   patt_size, channels, mode, vertices, cf, id, dir
//!
//! See [docs/issue-103-fix-plan.md](../../../../docs/issue-103-fix-plan.md)
//! and [benchmarks/data/README.md](../../../../benchmarks/data/README.md)
//! for the full diagnostic recipe.

use image::ImageReader;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use webarkitlib_rs::{
    arlog_e, arlog_i,
    image_proc::ARImageProcInfo,
    marker::ar_detect_marker,
    pattern::{ar_patt_get_image, ar_patt_load, AR_TEMPLATE_MATCHING_COLOR},
    types::{
        AR2VideoBufferT, AR2VideoTimestampT, ARHandle, ARMatrixCodeType, ARParam, ARParamLT,
        ARParamLTf, ARPixelFormat, AR_MATRIX_CODE_DETECTION,
    },
    version,
};

fn main() {
    webarkitlib_rs::arlog::ar_log_init_default();
    arlog_i!("WebARKitLib-rs v{}", version::get_version());
    arlog_i!("Diagnostic: dumping ar_patt_get_image output for issue #103");

    let args: Vec<String> = std::env::args().collect();
    let cparam_path = arg_or_default(&args, 1, "benchmarks/data/camera_para.dat");
    let patt_path = arg_or_default(&args, 2, "benchmarks/data/patt.hiro");
    let img_path = arg_or_default(&args, 3, "benchmarks/data/img.jpg");
    let out_dir = PathBuf::from(arg_or_default(&args, 4, "benchmarks/data"));

    arlog_i!("Loading camera parameters from {}", cparam_path);
    let param_file =
        File::open(&cparam_path).unwrap_or_else(|_| panic!("Failed to open {}", cparam_path));
    let param = ARParam::load(param_file).expect("Failed to read camera parameters");

    arlog_i!("Loading image {}", img_path);
    let img = ImageReader::open(&img_path)
        .unwrap_or_else(|_| panic!("Failed to open {}", img_path))
        .decode()
        .expect("Failed to decode image");
    let width = img.width() as i32;
    let height = img.height() as i32;
    let luma_img = img.to_luma8();

    // Export the raw luma so the C dumper consumes byte-identical input.
    let hiro_raw_path = out_dir.join("hiro.raw");
    {
        let mut f = File::create(&hiro_raw_path)
            .unwrap_or_else(|_| panic!("Failed to create {}", hiro_raw_path.display()));
        f.write_all(luma_img.as_raw()).expect("write raw luma");
    }
    arlog_i!(
        "Exported raw luma to {} ({} bytes)",
        hiro_raw_path.display(),
        width * height
    );

    // Otsu threshold (same arithmetic the simple example uses).
    let mut ipi = ARImageProcInfo::new(width, height);
    let otsu_thresh = ipi
        .luma_hist_and_otsu(luma_img.as_raw())
        .expect("Otsu failed");
    arlog_i!("Otsu threshold: {}", otsu_thresh);

    // Identity LT (skip lens distortion correction for the diagnostic).
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
    let mut param_lt = Box::new(ARParamLT { param, param_ltf });

    let mut ar_handle = ARHandle::default();
    ar_handle.xsize = width;
    ar_handle.ysize = height;
    ar_handle.set_pixel_format(ARPixelFormat::MONO);
    ar_handle.ar_image_proc_mode = 0; // FrameImage
    ar_handle.ar_pattern_detection_mode = AR_TEMPLATE_MATCHING_COLOR;
    ar_handle.ar_labeling_mode = 0;
    ar_handle.ar_labeling_thresh = otsu_thresh as i32;
    ar_handle.patt_ratio = 0.5;
    ar_handle.matrix_code_type = ARMatrixCodeType::Code3x3;
    ar_handle.ar_param_lt = &mut *param_lt;

    let mut patt_handle = webarkitlib_rs::types::ARPattHandle::default();
    patt_handle.patt_num_max = 50;
    patt_handle.patt_size = 16;
    patt_handle.pattf = vec![0; 50];
    patt_handle.patt = vec![vec![0; 16 * 16 * 3 * 4]; 50];
    patt_handle.pattpow = vec![0.0; 50 * 4];
    patt_handle.patt_bw = vec![vec![0; 16 * 16 * 4]; 50];
    patt_handle.pattpow_bw = vec![0.0; 50 * 4];

    arlog_i!("Loading hiro pattern from {}", patt_path);
    if let Err(e) = ar_patt_load(&mut patt_handle, &patt_path) {
        arlog_e!("ar_patt_load failed: {}", e);
        return;
    }

    let mut boxed_patt_handle = Box::new(patt_handle);
    ar_handle.patt_handle = &mut *boxed_patt_handle;

    let luma_buffer_vec = luma_img.into_raw();
    let frame = AR2VideoBufferT {
        buff: Some(luma_buffer_vec.clone()),
        buff_luma: Some(luma_buffer_vec.clone()),
        buf_planes: None,
        buf_plane_count: 0,
        fill_flag: true,
        time: AR2VideoTimestampT { sec: 1, usec: 0 },
    };

    arlog_i!("Running ar_detect_marker...");
    if let Err(e) = ar_detect_marker(&mut ar_handle, &frame) {
        arlog_e!("ar_detect_marker failed: {}", e);
        return;
    }
    if ar_handle.marker_num <= 0 {
        arlog_e!("No markers detected (marker_num={})", ar_handle.marker_num);
        return;
    }

    // Pick the first marker (matches the C dumper's selection rule).
    let sel: usize = (0..ar_handle.marker_num as usize)
        .find(|&i| ar_handle.marker_info[i].id >= 0)
        .unwrap_or(0);
    let marker = ar_handle.marker_info[sel].clone();
    let info2 = ar_handle.marker_info2[sel].clone();

    let patt_size = boxed_patt_handle.patt_size as usize;
    let patt_max_size = patt_size * 2;
    let channels = if ar_handle.ar_pattern_detection_mode == AR_TEMPLATE_MATCHING_COLOR {
        3
    } else {
        1
    };
    let ext_len = patt_size * patt_size * channels;
    let mut ext_patt = vec![0u8; ext_len];

    let vertex_indices: [usize; 4] = [
        info2.vertex[0] as usize,
        info2.vertex[1] as usize,
        info2.vertex[2] as usize,
        info2.vertex[3] as usize,
    ];

    arlog_i!("Calling ar_patt_get_image to re-extract the patch...");
    if let Err(e) = ar_patt_get_image(
        ar_handle.ar_image_proc_mode,
        ar_handle.ar_pattern_detection_mode,
        patt_size,
        patt_max_size,
        &luma_buffer_vec,
        width as usize,
        height as usize,
        ar_handle.ar_pixel_format,
        &info2.x_coord,
        &info2.y_coord,
        &vertex_indices,
        ar_handle.patt_ratio,
        &mut ext_patt,
    ) {
        arlog_e!("ar_patt_get_image failed: {}", e);
        return;
    }

    let bin_path = out_dir.join("rs_ext_patt.bin");
    let meta_path = out_dir.join("rs_ext_patt_meta.txt");

    File::create(&bin_path)
        .and_then(|mut f| f.write_all(&ext_patt))
        .expect("write rs_ext_patt.bin");

    let mut meta = String::new();
    meta.push_str(&format!("patt_size = {}\n", patt_size));
    meta.push_str(&format!("channels  = {}\n", channels));
    meta.push_str(&format!(
        "mode      = {}\n",
        mode_name(ar_handle.ar_pattern_detection_mode)
    ));
    meta.push_str(&format!(
        "pixfmt    = {}\n",
        pixfmt_name(ar_handle.ar_pixel_format)
    ));
    meta.push_str(&format!("xsize     = {}\n", width));
    meta.push_str(&format!("ysize     = {}\n", height));
    meta.push_str(&format!("patt_ratio = {:.6}\n", ar_handle.patt_ratio));
    meta.push_str(&format!(
        "selected  = {} (of {} markers)\n",
        sel, ar_handle.marker_num
    ));
    for i in 0..4 {
        let idx = vertex_indices[i];
        meta.push_str(&format!(
            "vertex[{}] = ({}, {})\n",
            i, info2.x_coord[idx], info2.y_coord[idx]
        ));
    }
    for i in 0..4 {
        meta.push_str(&format!(
            "marker_vertex[{}] = ({:.4}, {:.4})\n",
            i, marker.vertex[i][0], marker.vertex[i][1]
        ));
    }
    meta.push_str(&format!("id        = {}\n", marker.id));
    meta.push_str(&format!("id_patt   = {}\n", marker.id_patt));
    meta.push_str(&format!("cf        = {:.6}\n", marker.cf));
    meta.push_str(&format!("cf_patt   = {:.6}\n", marker.cf_patt));
    meta.push_str(&format!("dir       = {}\n", marker.dir));
    File::create(&meta_path)
        .and_then(|mut f| f.write_all(meta.as_bytes()))
        .expect("write rs_ext_patt_meta.txt");

    arlog_i!("Wrote {} ({} bytes)", bin_path.display(), ext_patt.len());
    arlog_i!("Wrote {}", meta_path.display());
    arlog_i!("cf={:.4} id={} dir={}", marker.cf, marker.id, marker.dir);
}

fn arg_or_default(args: &[String], idx: usize, default: &str) -> String {
    args.get(idx)
        .cloned()
        .unwrap_or_else(|| default.to_string())
}

fn mode_name(m: i32) -> &'static str {
    if m == AR_TEMPLATE_MATCHING_COLOR {
        "AR_TEMPLATE_MATCHING_COLOR"
    } else if m == webarkitlib_rs::pattern::AR_TEMPLATE_MATCHING_MONO {
        "AR_TEMPLATE_MATCHING_MONO"
    } else if m == AR_MATRIX_CODE_DETECTION {
        "AR_MATRIX_CODE_DETECTION"
    } else {
        "<unknown>"
    }
}

fn pixfmt_name(f: ARPixelFormat) -> &'static str {
    match f {
        ARPixelFormat::RGB => "AR_PIXEL_FORMAT_RGB",
        ARPixelFormat::BGR => "AR_PIXEL_FORMAT_BGR",
        ARPixelFormat::RGBA => "AR_PIXEL_FORMAT_RGBA",
        ARPixelFormat::BGRA => "AR_PIXEL_FORMAT_BGRA",
        ARPixelFormat::ABGR => "AR_PIXEL_FORMAT_ABGR",
        ARPixelFormat::ARGB => "AR_PIXEL_FORMAT_ARGB",
        ARPixelFormat::MONO => "AR_PIXEL_FORMAT_MONO",
        _ => "<unknown>",
    }
}
