use std::fs;
use std::path::Path;
use image::ImageReader;
use webarkitlib_rs::pattern::{ar_patt_save};
use webarkitlib_rs::types::{ARPixelFormat, ARParamLTf, ARMarkerInfo};

/*fn generate_dummy_image(patt_size: i32) -> Vec<u8> {
    let size = (4 * patt_size * patt_size * 3) as usize; // 4 orientamenti, 3 canali di colore
    (0..size).map(|i| (i % 256) as u8).collect()
}*/

fn main() {
    // Configurazione del file di output
    let output_path = "./crates/core/examples/Data/generated.patt";
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let img_path = Path::new(manifest_dir)
        .join("examples")
        .join("Data")
        .join("HIRO-test.jpg");
    let patt_size = 16;

    //let image = generate_dummy_image(patt_size);
    let image = ImageReader::open(img_path).unwrap().decode().unwrap();
    let image_buf = image.to_rgb8();
    let img = image_buf.into_raw();
    println!("image: {:?}", img);
    let xsize = patt_size as usize;
    let ysize = patt_size as usize;
    let pixel_format = ARPixelFormat::RGB;
    //let vertex = &[[0.0, 0.0], [patt_size as f64, 0.0], [patt_size as f64, patt_size as f64], [0.0, patt_size as f64]];
    let patt_ratio = 1.0;

    // Build a simple ARParamLTf identity mapping for the small generated image
    let param_ltf = ARParamLTf::new_basic(xsize as i32, ysize as i32);

    // Build a simple ARMarkerInfo with the desired vertices
    let mut marker = ARMarkerInfo::default();
    marker.vertex = [
        [0.0, 0.0],
        [patt_size as f64, 0.0],
        [patt_size as f64, patt_size as f64],
        [0.0, patt_size as f64],
    ];

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
