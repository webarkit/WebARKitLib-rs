// Temporary debug test to understand query behavior
#![allow(dead_code)]

use std::path::Path;

fn data_dir() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("Data")
}

#[cfg(feature = "ffi-backend")]
#[test]
fn debug_query_pipeline() {
    use std::sync::Arc;
    use webarkitlib_rs::kpm::types::KpmRefDataSet;
    use webarkitlib_rs::kpm::{CppFreakMatcher, KpmHandle};
    use webarkitlib_rs::types::{ARParam, ARParamLT};
    use webarkitlib_rs::kpm::backend::FreakMatcherBackend;

    // Load camera parameters.
    let cam_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent().unwrap()
        .parent().unwrap()
        .join("benchmarks")
        .join("data")
        .join("camera_para.dat");
    let cam_file = std::fs::File::open(&cam_path).expect("failed to open camera_para.dat");
    let mut cparam =
        ARParam::load(std::io::BufReader::new(cam_file)).expect("failed to load camera_para.dat");

    // Load query image (JPEG → RGB → luma).
    let img_path = data_dir().join("pinball-demo.jpg");
    let jpeg_bytes = std::fs::read(&img_path).expect("failed to read pinball-demo.jpg");
    let mut decoder = jpeg_decoder::Decoder::new(std::io::Cursor::new(&jpeg_bytes));
    let pixels = decoder.decode().expect("JPEG decode failed");
    let info = decoder.info().unwrap();
    let (w, h) = (info.width as i32, info.height as i32);
    eprintln!("Image: {}x{}", w, h);

    let luma: Vec<u8> = pixels
        .chunks_exact(3)
        .map(|rgb| ((rgb[0] as u32 * 77 + rgb[1] as u32 * 150 + rgb[2] as u32 * 29) >> 8) as u8)
        .collect();

    // Resize camera params.
    let sx = w as f64 / cparam.xsize as f64;
    let sy = h as f64 / cparam.ysize as f64;
    for col in 0..4 {
        cparam.mat[0][col] *= sx;
        cparam.mat[1][col] *= sy;
    }
    cparam.xsize = w;
    cparam.ysize = h;
    let cparam_lt = Arc::new(ARParamLT::new_basic(cparam));

    // Load .fset3.
    let fset3_path = data_dir().join("pinball.fset3");
    let mut ref_data_set = KpmRefDataSet::load(&fset3_path).expect("failed to load pinball.fset3");
    
    eprintln!("ref_data_set: num={}, page_num={}", ref_data_set.num, ref_data_set.page_num);
    for (pi, page) in ref_data_set.page_info.iter().enumerate() {
        eprintln!("  page[{}]: page_no={}, image_num={}", pi, page.page_no, page.image_num);
        for (ii, img) in page.image_info.iter().enumerate() {
            let count = ref_data_set.ref_point.iter()
                .filter(|rp| rp.ref_image_no == img.image_no && rp.page_no == page.page_no)
                .count();
            eprintln!("    image[{}]: image_no={}, {}x{}, features={}", 
                      ii, img.image_no, img.width, img.height, count);
        }
    }

    ref_data_set.change_page_no(
        webarkitlib_rs::kpm::ref_data_set::KPM_CHANGE_PAGE_NO_ALL_PAGES,
        0,
    );

    // Create backend.
    let mut backend = CppFreakMatcher::new(w, h).expect("failed to create CppFreakMatcher");
    
    // Manually add features to debug.
    let mut db_id: usize = 0;
    for page in &ref_data_set.page_info {
        for img in &page.image_info {
            let mut points = Vec::new();
            let mut descriptors = Vec::new();
            let mut points_3d = Vec::new();

            for rp in &ref_data_set.ref_point {
                if rp.ref_image_no == img.image_no && rp.page_no == page.page_no {
                    points.push(webarkitlib_rs::kpm::backend::FeaturePoint {
                        x: rp.coord2d.x,
                        y: rp.coord2d.y,
                        angle: rp.feature_vec.angle,
                        scale: rp.feature_vec.scale,
                        maxima: rp.feature_vec.maxima != 0,
                    });
                    points_3d.push(webarkitlib_rs::kpm::backend::Point3d {
                        x: rp.coord3d.x,
                        y: rp.coord3d.y,
                        z: 0.0,
                    });
                    descriptors.extend_from_slice(&rp.feature_vec.v[..96]);
                }
            }
            
            eprintln!("Adding db_id={}, page_no={}, image_no={}, points={}, desc_bytes={}, w={}, h={}",
                      db_id, page.page_no, img.image_no, points.len(), descriptors.len(), 
                      img.width, img.height);
            
            backend.add_freak_features(
                &points,
                &descriptors,
                &points_3d,
                img.width as usize,
                img.height as usize,
                db_id,
            ).expect("add_freak_features failed");
            
            db_id += 1;
        }
    }
    
    eprintln!("Total db_ids added: {}", db_id);
    
    // Now query.
    let result = backend.query(&luma, w as usize, h as usize);
    eprintln!("Query result: {:?}", result);
    
    let matched_id = backend.matched_id();
    eprintln!("matched_id() = {}", matched_id);
    
    let qf = backend.query_feature_points();
    eprintln!("query_feature_points count: {}", qf.len());
    
    let inliers = backend.inliers();
    eprintln!("inliers count: {}", inliers.len());
}
