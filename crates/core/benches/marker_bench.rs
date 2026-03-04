use criterion::{black_box, criterion_group, criterion_main, Criterion};
use core::{
    marker::ar_detect_marker,
    pattern::ar_patt_load,
    pose::{ar_3d_create_handle, ar_get_trans_mat_square},
    types::{AR2VideoBufferT, AR2VideoTimestampT, ARHandle, ARMatrixCodeType, ARPixelFormat, ARParamLT, ARParamLTf, ARParam},
    image_proc::ARImageProcInfo,
};
use image::ImageReader;
use std::fs::File;

fn marker_detection_benchmark(c: &mut Criterion) {
    // Load setup data (we do this outside the loop)
    let cparam_path = "../../benchmarks/data/camera_para.dat";
    let patt_path = "../../benchmarks/data/patt.hiro";
    let img_path = "../../benchmarks/data/img.jpg";

    let param_file = File::open(cparam_path).expect("Failed to open camera_para.dat");
    let param = ARParam::load(param_file).expect("Failed to read camera_para.dat");
    
    let img = ImageReader::open(img_path).unwrap().decode().unwrap();
    let width = img.width() as i32;
    let height = img.height() as i32;
    let luma_img = img.to_luma8();
    let color_img = img.to_rgba8(); 

    let mut ipi = ARImageProcInfo::new(width, height);
    let otsu_thresh = ipi.luma_hist_and_otsu(luma_img.as_raw()).expect("Failed to calculate Otsu threshold");

    let mut param_ltf = ARParamLTf::new_basic(width, height);
    let mut param_lt = Box::new(ARParamLT {
        param: param.clone(),
        param_ltf,
    });

    let mut ar_handle = ARHandle::default();
    ar_handle.xsize = width;
    ar_handle.ysize = height;
    ar_handle.ar_pixel_format = ARPixelFormat::Invalid;
    ar_handle.ar_labeling_thresh = otsu_thresh as i32;
    ar_handle.ar_param_lt = &mut *param_lt;

    let mut patt_handle = core::types::ARPattHandle::default();
    patt_handle.patt_num_max = 50;
    patt_handle.patt_size = 16;
    patt_handle.pattf = vec![0; 50];
    patt_handle.patt = vec![vec![0; 16 * 16 * 3 * 4]; 50]; 
    patt_handle.pattpow = vec![0.0; 50 * 4];
    patt_handle.patt_bw = vec![vec![0; 16 * 16 * 4]; 50];
    patt_handle.pattpow_bw = vec![0.0; 50 * 4];
    
    ar_patt_load(&mut patt_handle, patt_path).expect("Failed to load pattern");
    let mut boxed_patt_handle = Box::new(patt_handle);
    ar_handle.patt_handle = &mut *boxed_patt_handle;

    let luma_buffer_vec = luma_img.into_raw();
    let color_buffer_vec = color_img.into_raw();
    
    let frame = AR2VideoBufferT {
        buff: Some(color_buffer_vec),
        buff_luma: Some(luma_buffer_vec),
        buf_planes: None,
        buf_plane_count: 0,
        fill_flag: true,
        time: AR2VideoTimestampT { sec: 1, usec: 0 },
    };

    let ar_3d_handle_ptr = ar_3d_create_handle(&param).expect("Failed to create AR3DHandle");
    let ar_3d_handle = unsafe { &*ar_3d_handle_ptr };

    c.bench_function("ar_detect_marker_plus_pose", |b| {
        b.iter(|| {
            // We use black_box to prevent the compiler from optimizing away the call
            let _ = ar_detect_marker(black_box(&mut ar_handle), black_box(&frame)).unwrap();
            let num_markers = ar_handle.marker_num;
            if num_markers > 0 {
                let marker_info = &ar_handle.marker_info[0];
                if marker_info.id == 0 {
                    let mut trans = [[0.0; 4]; 3];
                    let _ = ar_get_trans_mat_square(black_box(ar_3d_handle), black_box(marker_info), 80.0, &mut trans).unwrap();
                }
            };
        })
    });
}

criterion_group!(benches, marker_detection_benchmark);
criterion_main!(benches);
