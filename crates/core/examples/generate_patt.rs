use std::fs;
use std::path::Path;
use image::ImageReader;
use image::{GrayImage, Luma};
use imageproc::contrast::otsu_level;
use webarkitlib_rs::pattern::{ar_patt_save, ar_patt_get_image2};
use webarkitlib_rs::types::{ARPixelFormat, ARParam, ARParamLT, ARParamLTf, ARMarkerInfo};

fn main() {
    // Configurazione del file di output
    let output_path = "./crates/core/examples/Data/generated.patt";
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let img_path = Path::new(manifest_dir)
        .join("examples")
        .join("Data")
        .join("HIRO-test.jpg");
    let patt_size = 16;

    let image = ImageReader::open(img_path).unwrap().decode().unwrap();
    let image_buf = image.to_rgb8();
    let img = image_buf.into_raw();
    let width = image.width();
    let height = image.height();
    println!("image buffer length: {} width:{} height:{}", img.len(), width, height);
    // Use actual image dimensions for sampling
    let xsize = width as usize;
    let ysize = height as usize;
    let pixel_format = ARPixelFormat::RGB;
    let patt_ratio = 0.5;

    // Prefer to build the lookup table (`ARParamLT`) from real camera parameters
    // when available. Try to load `benchmarks/data/camera_para.dat` (same path
    // used by the `simple` example). If not found, fall back to an identity table.
    let param_ltf = {
        // Candidate path (project-relative) to the camera parameter file.
        let cparam_rel = std::path::Path::new("examples").join("Data").join("camera_para.dat");
        let manifest_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let cparam_path = manifest_dir.join(&cparam_rel);

        let param = if cparam_path.exists() {
            // Load ARParam from file (BigEndian .dat format)
            let f = fs::File::open(&cparam_path).expect(&format!("Failed to open camera parameters: {:?}", cparam_path));
            ARParam::load(f).expect("Failed to load camera parameters")
        } else {
            // fallback to default ARParam (identity) if camera params not present
            println!("Warning: camera parameter file not found at {:?}, using identity parameters", cparam_path);
            ARParam::default()
        };

        // Build ARParamLT containing a proper lookup table; new_basic will
        // construct an ARParamLTf appropriate for the image size.
        let param_lt = ARParamLT::new_basic(param);
        // Extract the lookup-table portion (ARParamLTf). Clone because we need owned.
        param_lt.param_ltf.clone()
    };


    // Attempt to auto-detect the marker region using Otsu threshold on grayscale image
    let gray: GrayImage = image.to_luma8();
    let thresh = otsu_level(&gray);
    let mut minx = width as i32;
    let mut miny = height as i32;
    let mut maxx = 0i32;
    let mut maxy = 0i32;

    for (y, row) in gray.rows().enumerate() {
        for (x, pixel) in row.enumerate() {
            let v = pixel[0];
            // treat dark regions as marker (pixel value less than threshold)
            if v < thresh {
                if (x as i32) < minx { minx = x as i32; }
                if (y as i32) < miny { miny = y as i32; }
                if (x as i32) > maxx { maxx = x as i32; }
                if (y as i32) > maxy { maxy = y as i32; }
            }
        }
    }

    // Fallback: if no dark region found, use a centered square roughly covering the marker area
    if maxx == 0 && maxy == 0 {
        let side = ((width.min(height) as f32) * 0.6) as i32;
        let cx = (width as i32) / 2;
        let cy = (height as i32) / 2;
        minx = cx - side/2; if minx < 0 { minx = 0 }
        miny = cy - side/2; if miny < 0 { miny = 0 }
        maxx = cx + side/2; if maxx >= width as i32 { maxx = width as i32 - 1 }
        maxy = cy + side/2; if maxy >= height as i32 { maxy = height as i32 - 1 }
    }

    println!("Detected marker region: minx={}, miny={}, maxx={}, maxy={}", minx, miny, maxx, maxy);

    let mut marker = ARMarkerInfo::default();
    marker.vertex = [
        [minx as f64, miny as f64], // TL
        [maxx as f64, miny as f64], // TR
        [maxx as f64, maxy as f64], // BR
        [minx as f64, maxy as f64], // BL
    ];


    // For debugging: extract the normalized pattern image that will be saved
    let mut ext_patt = vec![0u8; (patt_size * patt_size * 3) as usize];
    if let Err(e) = ar_patt_get_image2(0, /* frame image */
                                      ARPixelFormat::RGB as i32, /* detect mode not used here but keep consistent */
                                      patt_size as usize,
                                      (patt_size * 4) as usize,
                                      &img,
                                      xsize,
                                      ysize,
                                      pixel_format,
                                      &param_ltf,
                                      &marker.vertex,
                                      patt_ratio,
                                      &mut ext_patt) {
        eprintln!("Errore durante l'estrazione del pattern (debug): {}", e);
    } else {
        // Save extracted pattern to PNG for inspection (convert BGR->RGB as needed)
        let mut out = image::RgbImage::new(patt_size as u32, patt_size as u32);
        for y in 0..patt_size as usize {
            for x in 0..patt_size as usize {
                let idx = (y * patt_size as usize + x) * 3;
                // ext_patt stores in B,G,R order for ARPixelFormat::RGB path in our implementation
                let b = ext_patt[idx];
                let g = ext_patt[idx + 1];
                let r = ext_patt[idx + 2];
                out.put_pixel(x as u32, y as u32, image::Rgb([r, g, b]));
            }
        }
        let debug_path = Path::new("./crates/core/examples/Data/extracted_pattern.png");
        let _ = out.save(debug_path);
        println!("Saved extracted pattern to {:?}", debug_path);
    }

    // Salvataggio del pattern nel file (new signature expects image-first)
    let filename_path = Path::new(output_path);
    if let Err(e) = ar_patt_save(&img, xsize, ysize, pixel_format, &param_ltf, 0, &marker, patt_ratio, patt_size as usize, filename_path) {
        eprintln!("Errore durante il salvataggio del pattern: {}", e);
        return;
    }

    println!("Pattern salvato con successo in: {}", output_path);

    // Verifica del contenuto salvato
    let saved_content = fs::read_to_string(output_path).expect("Impossibile leggere il file salvato");
    println!("Contenuto del pattern salvato:\n{}", saved_content);
}
