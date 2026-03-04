/*
 *  marker_bench.rs
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

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use webarkitlib_rs::{
    marker::ar_detect_marker,
    pattern::ar_patt_load,
    pose::{ar_3d_create_handle, ar_get_trans_mat_square},
    types::{AR2VideoBufferT, AR2VideoTimestampT, ARHandle, ARPixelFormat, ARParamLT, ARParamLTf, ARParam},
};
use std::fs::File;
use std::io::Read;

fn marker_detection_benchmark(c: &mut Criterion) {
    let cparam_path = "../../benchmarks/data/camera_para.dat";
    let patt_path = "../../benchmarks/data/patt.hiro";
    let img_path = "../../benchmarks/data/hiro.raw";

    let param_file = File::open(cparam_path).expect("Failed to open camera_para.dat");
    let param = ARParam::load(param_file).expect("Failed to read camera_para.dat");
    
    let mut luma_file = File::open(img_path).expect("Failed to open hiro.raw");
    let mut luma_buffer_vec = Vec::new();
    luma_file.read_to_end(&mut luma_buffer_vec).expect("Failed to read hiro.raw");
    
    let width = 429;
    let height = 317;

    let mut param_ltf = ARParamLTf::new_basic(width, height);
    let mut param_lt = Box::new(ARParamLT {
        param: param.clone(),
        param_ltf,
    });

    let mut ar_handle = ARHandle::default();
    ar_handle.xsize = width;
    ar_handle.ysize = height;
    ar_handle.ar_pixel_format = ARPixelFormat::Invalid;
    ar_handle.ar_labeling_thresh = 100; // Standard threshold
    ar_handle.ar_param_lt = &mut *param_lt;

    let mut patt_handle = core::types::ARPattHandle::new(16, 50);
    ar_patt_load(&mut patt_handle, patt_path).expect("Failed to load pattern");
    let mut boxed_patt_handle = Box::new(patt_handle);
    ar_handle.patt_handle = &mut *boxed_patt_handle;

    let frame = AR2VideoBufferT {
        buff: Some(luma_buffer_vec.clone()),
        buff_luma: Some(luma_buffer_vec),
        buf_planes: None,
        buf_plane_count: 0,
        fill_flag: true,
        time: AR2VideoTimestampT { sec: 1, usec: 0 },
    };

    let ar_3d_handle_ptr = ar_3d_create_handle(&param).expect("Failed to create AR3DHandle");
    let ar_3d_handle = unsafe { &*ar_3d_handle_ptr };

    c.bench_function("ar_full_pipeline_429x317", |b| {
        b.iter(|| {
            let _ = ar_detect_marker(black_box(&mut ar_handle), black_box(&frame)).unwrap();
            if ar_handle.marker_num > 0 {
                let marker_info = &ar_handle.marker_info[0];
                let mut trans = [[0.0; 4]; 3];
                let _ = ar_get_trans_mat_square(black_box(ar_3d_handle), black_box(marker_info), 80.0, &mut trans).unwrap();
            }
        })
    });
}

criterion_group!(benches, marker_detection_benchmark);
criterion_main!(benches);
