//! Connected Component Labeling Utilities
//! Translated from ARToolKit C headers (arLabeling.c, arLabelingSub.h)

use crate::types::{ARLabelInfo, ARdouble};
use log::debug;

pub const AR_LABELING_WORK_SIZE: usize = 1024 * 32;

pub enum LabelingMode {
    BlackRegion,
    WhiteRegion,
}

pub enum ImageProcMode {
    FrameImage,
    FieldImage, // Handles interlaced half-resolution fields
}

/// Perform connected-component analysis and labeling on an 8-bit luma image.
///
/// This collapses the ~14 macro variations of `arLabelingSub*` into one generic safe Rust function,
/// parameterizing the behavior using enums and generic pixel-checking closures.
pub fn ar_labeling(
    image: &[u8],
    xsize: i32,
    ysize: i32,
    mode: LabelingMode,
    thresh: u8,
    proc_mode: ImageProcMode,
    label_info: &mut ARLabelInfo,
    _debug_mode: bool,
) -> Result<(), &'static str> {
    
    let is_region = |pixel: u8| -> bool {
        match mode {
            LabelingMode::BlackRegion => pixel <= thresh,
            LabelingMode::WhiteRegion => pixel > thresh,
        }
    };

    let lxsize: usize;
    let lysize: usize;
    let row_stride: usize;
    let _col_stride: usize;

    match proc_mode {
        ImageProcMode::FrameImage => {
            lxsize = xsize as usize;
            lysize = ysize as usize;
            row_stride = xsize as usize;
            _col_stride = 1;
        }
        ImageProcMode::FieldImage => {
            lxsize = (xsize / 2) as usize;
            lysize = (ysize / 2) as usize;
            row_stride = (xsize * 2) as usize; // skip every other row
            if ysize < 480 { // Assuming 'height' in the snippet refers to ysize
                _col_stride = 1;
            } else {
                _col_stride = 2; // skip every other column
            }
        }
    }

    if image.len() < (ysize as usize * xsize as usize) {
        return Err("Image buffer is too small for given dimensions.");
    }

    // Ensure the label_image buffer is perfectly sized
    if label_info.label_image.len() < lxsize * lysize {
        label_info.label_image.resize(lxsize * lysize, 0);
    }
    label_info.label_image.fill(0);

    // Initialize work equivalence arrays
    if label_info.work.len() < AR_LABELING_WORK_SIZE {
        label_info.work.resize(AR_LABELING_WORK_SIZE, 0);
    }
    if label_info.work2.len() < AR_LABELING_WORK_SIZE * 7 {
        label_info.work2.resize(AR_LABELING_WORK_SIZE * 7, 0);
    }

    let mut wk_max: usize = 0;
    let work = &mut label_info.work;
    let work2 = &mut label_info.work2;
    let label_img = &mut label_info.label_image;

    // Helper for Union-Find find with path compression
    fn find(work: &mut [i32], mut i: i32) -> i32 {
        let mut root = i;
        while work[root as usize - 1] != root {
            root = work[root as usize - 1];
        }
        // Path compression
        let mut curr = i;
        while work[curr as usize - 1] != root {
            let next = work[curr as usize - 1];
            work[curr as usize - 1] = root;
            curr = next;
        }
        root
    }

    // Helper for Union-Find union
    fn do_union(work: &mut [i32], m: i32, n: i32) -> i32 {
        let root_m = find(work, m);
        let root_n = find(work, n);
        if root_m < root_n {
            work[root_n as usize - 1] = root_m;
            root_m
        } else {
            work[root_m as usize - 1] = root_n;
            root_n
        }
    }

    // Scan the inner pixel region (skip boundaries 0, and max-1)
    for j in 1..lysize - 1 {
        for i in 1..lxsize - 1 {
            let source_idx = match proc_mode {
                ImageProcMode::FrameImage => j * row_stride + i,
                ImageProcMode::FieldImage => (j * 2 + 1) * xsize as usize + (i * 2),
            };
            let pixel = image[source_idx];

            let p_idx = j * lxsize + i;

            if is_region(pixel) {
                let left_val = label_img[p_idx - 1];
                let up_val = label_img[p_idx - lxsize];
                let up_left = label_img[p_idx - lxsize - 1];
                let up_right = label_img[p_idx - lxsize + 1];

                if up_val > 0 {
                    label_img[p_idx] = up_val;
                    let l = (find(work, up_val as i32) as usize - 1) * 7;
                    work2[l + 0] += 1;
                    work2[l + 1] += i as i32;
                    work2[l + 2] += j as i32;
                    work2[l + 6] = j as i32;
                } else if up_right > 0 {
                    if up_left > 0 {
                        let final_label = do_union(work, up_right as i32, up_left as i32);
                        label_img[p_idx] = final_label as crate::types::ARLabelingLabelType;

                        let l = (final_label as usize - 1) * 7;
                        work2[l + 0] += 1;
                        work2[l + 1] += i as i32;
                        work2[l + 2] += j as i32;
                        work2[l + 6] = j as i32;
                    } else if left_val > 0 {
                        let final_label = do_union(work, up_right as i32, left_val as i32);
                        label_img[p_idx] = final_label as crate::types::ARLabelingLabelType;

                        let l = (final_label as usize - 1) * 7;
                        work2[l + 0] += 1;
                        work2[l + 1] += i as i32;
                        work2[l + 2] += j as i32;
                        work2[l + 6] = j as i32;
                    } else {
                        label_img[p_idx] = up_right;
                        let l = (find(work, up_right as i32) as usize - 1) * 7;
                        work2[l + 0] += 1;
                        work2[l + 1] += i as i32;
                        work2[l + 2] += j as i32;
                        if work2[l + 3] > i as i32 { work2[l + 3] = i as i32; }
                        work2[l + 6] = j as i32;
                    }
                } else if up_left > 0 {
                    label_img[p_idx] = up_left;
                    let l = (find(work, up_left as i32) as usize - 1) * 7;
                    work2[l + 0] += 1;
                    work2[l + 1] += i as i32;
                    work2[l + 2] += j as i32;
                    if work2[l + 4] < i as i32 { work2[l + 4] = i as i32; }
                    work2[l + 6] = j as i32;
                } else if left_val > 0 {
                    label_img[p_idx] = left_val;
                    let l = (find(work, left_val as i32) as usize - 1) * 7;
                    work2[l + 0] += 1;
                    work2[l + 1] += i as i32;
                    work2[l + 2] += j as i32;
                    if work2[l + 4] < i as i32 { work2[l + 4] = i as i32; }
                } else {
                    wk_max += 1;
                    if wk_max > AR_LABELING_WORK_SIZE {
                        return Err("Labeling work array overflow");
                    }
                    work[wk_max - 1] = wk_max as i32;
                    label_img[p_idx] = wk_max as crate::types::ARLabelingLabelType;

                    let l = (wk_max - 1) * 7;
                    work2[l + 0] = 1;         // area
                    work2[l + 1] = i as i32;  // pos[0]
                    work2[l + 2] = j as i32;  // pos[1]
                    work2[l + 3] = i as i32;  // clip[0] (xmin)
                    work2[l + 4] = i as i32;  // clip[1] (xmax)
                    work2[l + 5] = j as i32;  // clip[2] (ymin)
                    work2[l + 6] = j as i32;  // clip[3] (ymax)
                }
            } else {
                label_img[p_idx] = 0;
            }
        }
    }

    // Pass 2: Map equivalence table down to dense sequential indexes
    let mut num_labels = 0;
    for i in 1..=wk_max {
        if work[i - 1] == i as i32 {
            num_labels += 1;
            work[i - 1] = num_labels;
        } else {
            work[i - 1] = work[work[i - 1] as usize - 1]; // This is fine for one level, but let's be thorough
        }
    }
    
    // Thoroughly flatten the equivalence table
    for i in 1..=wk_max {
        let mut root = i as i32;
        while work[root as usize - 1] > num_labels || work[work[root as usize - 1] as usize - 1] != work[root as usize - 1] {
             // If not yet a finalized label index, follow parent
             root = work[root as usize - 1];
        }
        work[i - 1] = work[root as usize - 1];
    }
    
    label_info.label_num = num_labels;
    if label_info.label_num == 0 {
        return Ok(());
    }

    // Allocate memory and reset results for the second pass
    if label_info.area.len() < num_labels as usize { label_info.area.resize(num_labels as usize, 0); }
    if label_info.pos.len() < num_labels as usize { label_info.pos.resize(num_labels as usize, [0.0; 2]); }
    if label_info.clip.len() < num_labels as usize { label_info.clip.resize(num_labels as usize, [0; 4]); }

    label_info.area.fill(0);
    label_info.pos.fill([0.0; 2]);
    for clip in label_info.clip.iter_mut() {
        clip[0] = lxsize as i32;
        clip[1] = 0;
        clip[2] = lysize as i32;
        clip[3] = 0;
    }

    for i in 0..wk_max {
        let dest_label = work[i] as usize - 1;
        
        let area = work2[i * 7 + 0];
        let pos_x = work2[i * 7 + 1];
        let pos_y = work2[i * 7 + 2];
        let clip_xmin = work2[i * 7 + 3];
        let clip_xmax = work2[i * 7 + 4];
        let clip_ymin = work2[i * 7 + 5];
        let clip_ymax = work2[i * 7 + 6];

        label_info.area[dest_label] += area;
        label_info.pos[dest_label][0] += pos_x as ARdouble;
        label_info.pos[dest_label][1] += pos_y as ARdouble;
        
        if label_info.clip[dest_label][0] > clip_xmin { label_info.clip[dest_label][0] = clip_xmin; }
        if label_info.clip[dest_label][1] < clip_xmax { label_info.clip[dest_label][1] = clip_xmax; }
        if label_info.clip[dest_label][2] > clip_ymin { label_info.clip[dest_label][2] = clip_ymin; }
        if label_info.clip[dest_label][3] < clip_ymax { label_info.clip[dest_label][3] = clip_ymax; }
    }

    for i in 0..num_labels as usize {
        if label_info.area[i] > 0 {
            label_info.pos[i][0] /= label_info.area[i] as ARdouble;
            label_info.pos[i][1] /= label_info.area[i] as ARdouble;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ar_labeling_simple_square() {
        // Create an 8x8 image where center 4x4 is black (value 0), rest is white (value 255)
        let mut image = vec![255u8; 64];
        for j in 2..6 {
            for i in 2..6 {
                image[j * 8 + i] = 0;
            }
        }

        let mut info = ARLabelInfo::default();

        ar_labeling(
            &image,
            8,
            8,
            LabelingMode::BlackRegion,
            100, // Threshold 100
            ImageProcMode::FrameImage,
            &mut info,
            false
        ).unwrap();

        assert_eq!(info.label_num, 1);
        
        // The square is 4x4, so area should be 16
        assert_eq!(info.area[0], 16);
        
        // Pos should be the center: average of 2, 3, 4, 5 = 3.5
        assert!((info.pos[0][0] - 3.5).abs() < f64::EPSILON);
        assert!((info.pos[0][1] - 3.5).abs() < f64::EPSILON);
        
        // Clip coords
        assert_eq!(info.clip[0][0], 2); // xmin
        assert_eq!(info.clip[0][1], 5); // xmax
        assert_eq!(info.clip[0][2], 2); // ymin
        assert_eq!(info.clip[0][3], 5); // ymax
    }
}
