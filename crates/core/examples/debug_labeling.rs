/*
 *  debug_labeling.rs
 *  WebARKitLib-rs
 *
 *  Diagnostic tool to visualize the results of ar_labeling.
 */

use webarkitlib_rs::{
    labeling::{ar_labeling, LabelingMode, ImageProcMode},
    types::ARLabelInfo,
};
use image::{ImageReader, GenericImageView, RgbImage, Rgb};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    std::env::set_var("RUST_LOG", "trace");
    env_logger::init();

    let image_path = "crates/core/examples/Data/gem_05_3x3.png";
    let image_path_buf = std::path::PathBuf::from(image_path);
    
    // Attempt rescue if file not found (for CI run from crates/core)
    let final_image_path = if !image_path_buf.exists() {
        let alt_path = std::path::Path::new("examples/Data/gem_05_3x3.png");
        if alt_path.exists() { alt_path.to_path_buf() } else { image_path_buf }
    } else {
        image_path_buf
    };

    println!("Loading image {}...", final_image_path.display());
    let full_img = ImageReader::open(&final_image_path).expect(&format!("Failed to open {}", final_image_path.display())).decode().expect("Failed to decode image");
    let luma_img = full_img.to_luma8();
    let (width, height) = luma_img.dimensions();

    let mut label_info = ARLabelInfo::default();
    
    ar_labeling(
        luma_img.as_raw(),
        width as i32,
        height as i32,
        LabelingMode::BlackRegion,
        80,
        ImageProcMode::FrameImage,
        &mut label_info,
        false
    ).expect("Labeling failed");

    println!("Found {} labels.", label_info.label_num);

    // Save as colorized image
    let lxsize = width as usize + 2;
    let mut out_img = RgbImage::new(width, height);
    
    // Generate colors
    let mut colors = Vec::new();
    colors.push(Rgb([0, 0, 0])); // Label 0 (Background)
    for i in 1..=label_info.label_num {
        let r = (i * 123) % 256;
        let g = (i * 456) % 256;
        let b = (i * 789) % 256;
        colors.push(Rgb([r as u8, g as u8, b as u8]));
    }

    for y in 0..height {
        for x in 0..width {
            let p_idx = (y as usize + 1) * lxsize + (x as usize + 1);
            let label = label_info.label_image[p_idx] as usize;
            if label < colors.len() {
                out_img.put_pixel(x, y, colors[label]);
            } else {
                out_img.put_pixel(x, y, Rgb([255, 255, 255])); // Error
            }
        }
    }

    out_img.save("crates/core/examples/Data/debug_labels.png")?;
    println!("Saved debug_labels.png");
    
    // Print stats for largest components
    let mut labels: Vec<usize> = (0..label_info.label_num as usize).collect();
    labels.sort_by_key(|&idx| std::cmp::Reverse(label_info.area[idx]));
    
    for &idx in labels.iter().take(10) {
        println!("Label #{} : Area={}, Clip={:?}", idx+1, label_info.area[idx], label_info.clip[idx]);
    }

    Ok(())
}
