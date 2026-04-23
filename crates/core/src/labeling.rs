/*
 *  labeling.rs
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

//! Connected Component Labeling Utilities
//! Translated from ARToolKit C headers (arLabeling.c, arLabelingSub.h)

use crate::arlog_e;
use crate::types::{ARLabelInfo, ARdouble};
use log::trace;

pub const AR_LABELING_WORK_SIZE: usize = 1024 * 32;

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum LabelingMode {
    BlackRegion,
    WhiteRegion,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum ImageProcMode {
    FrameImage,
    FieldImage, // Handles interlaced half-resolution fields
}

/// Perform connected-component labeling on a grayscale (luma) image.
///
/// Ported from the family of `arLabelingSub*` C macros in ARToolKit.
/// All ~14 macro instantiations (black/white × frame/field × pixel-depth) are
/// collapsed into a single generic Rust function parameterised by `mode` and
/// `proc_mode`.
///
/// ## Algorithm
/// Single-pass raster scan with union-find equivalence table:
/// 1. Each pixel that satisfies the threshold test (`≤ thresh` for
///    [`LabelingMode::BlackRegion`], `> thresh` for [`LabelingMode::WhiteRegion`])
///    is provisionally assigned the label of its already-scanned neighbour, or a new
///    label if none exist.
/// 2. Conflicting neighbour labels are merged via a union-find structure stored in
///    `label_info.work`.
/// 3. A second pass resolves equivalences and accumulates per-label statistics
///    (area, centroid, bounding clip) into `label_info`.
///
/// ## Parameters
/// - `image` — row-major 8-bit luma buffer of length `xsize * ysize`.
/// - `mode` — whether dark or bright regions are treated as foreground.
/// - `thresh` — binarisation threshold (0–255).
/// - `proc_mode` — `FrameImage` (full resolution) or `FieldImage`
///   (half-resolution interlaced; dimensions are halved internally).
/// - `label_info` — output struct; resized automatically if needed.
///
/// # Example
/// ```rust,no_run
/// use webarkitlib_rs::arlog_i;
/// use webarkitlib_rs::labeling::{ar_labeling, LabelingMode, ImageProcMode};
/// use webarkitlib_rs::types::ARLabelInfo;
///
/// let width = 320i32;
/// let height = 240i32;
/// let luma = vec![0u8; (width * height) as usize];
/// let mut label_info = ARLabelInfo::default();
/// ar_labeling(&luma, width, height, LabelingMode::BlackRegion, 100,
///             ImageProcMode::FrameImage, &mut label_info, false).unwrap();
/// arlog_i!("{} regions found", label_info.label_num);
/// ```
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
            if ysize < 480 {
                // Assuming 'height' in the snippet refers to ysize
                _col_stride = 1;
            } else {
                _col_stride = 2; // skip every other column
            }
        }
    }

    if image.len() < (ysize as usize * xsize as usize) {
        arlog_e!(
            "ar_labeling: image buffer too small (len={}, expected>={})",
            image.len(),
            ysize as usize * xsize as usize
        );
        return Err("Image buffer is too small for given dimensions.");
    }

    // Ensure the label_image buffer is perfectly sized
    if label_info.label_image.len() < lxsize * lysize {
        label_info.label_image.resize(lxsize * lysize, 0);
    }
    label_info.label_image.fill(0);
    trace!(
        "ar_labeling: lxsize={}, lysize={}, len={}",
        lxsize,
        lysize,
        label_info.label_image.len()
    );

    // Initialize work equivalence arrays
    if label_info.work.len() < AR_LABELING_WORK_SIZE {
        label_info.work.resize(AR_LABELING_WORK_SIZE, 0);
    }
    let mut wk_max: usize = 0;
    let work = &mut label_info.work;
    // We'll use a local work2 of i64 to avoid overflow during summation
    let mut work2 = vec![0i64; AR_LABELING_WORK_SIZE * 7];

    // Initialize xmin, ymin to large values
    for k in 0..AR_LABELING_WORK_SIZE {
        work2[k * 7 + 3] = xsize as i64; // xmin
        work2[k * 7 + 5] = ysize as i64; // ymin
    }

    let label_img = &mut label_info.label_image;

    // Helper for Union-Find find with path compression
    fn find(work: &mut [i32], i: i32) -> i32 {
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

    // Helper for Union-Find union with immediate data transfer
    fn do_union(work: &mut [i32], work2: &mut [i64], m: i32, n: i32) -> i32 {
        let root_m = find(work, m);
        let root_n = find(work, n);
        if root_m != root_n {
            if root_m < root_n {
                work[root_n as usize - 1] = root_m;
                // Transfer data
                let m_idx = (root_m as usize - 1) * 7;
                let n_idx = (root_n as usize - 1) * 7;
                for i in 0..3 {
                    work2[m_idx + i] += work2[n_idx + i];
                }
                if work2[m_idx + 3] > work2[n_idx + 3] {
                    work2[m_idx + 3] = work2[n_idx + 3];
                }
                if work2[m_idx + 4] < work2[n_idx + 4] {
                    work2[m_idx + 4] = work2[n_idx + 4];
                }
                if work2[m_idx + 5] > work2[n_idx + 5] {
                    work2[m_idx + 5] = work2[n_idx + 5];
                }
                if work2[m_idx + 6] < work2[n_idx + 6] {
                    work2[m_idx + 6] = work2[n_idx + 6];
                }
                // Clear old root (optional)
                for i in 0..7 {
                    work2[n_idx + i] = 0;
                }
                root_m
            } else {
                work[root_m as usize - 1] = root_n;
                // Transfer data
                let m_idx = (root_m as usize - 1) * 7;
                let n_idx = (root_n as usize - 1) * 7;
                for i in 0..3 {
                    work2[n_idx + i] += work2[m_idx + i];
                }
                if work2[n_idx + 3] > work2[m_idx + 3] {
                    work2[n_idx + 3] = work2[m_idx + 3];
                }
                if work2[n_idx + 4] < work2[m_idx + 4] {
                    work2[n_idx + 4] = work2[m_idx + 4];
                }
                if work2[n_idx + 5] > work2[m_idx + 5] {
                    work2[n_idx + 5] = work2[m_idx + 5];
                }
                if work2[n_idx + 6] < work2[m_idx + 6] {
                    work2[n_idx + 6] = work2[m_idx + 6];
                }
                // Clear old root
                for i in 0..7 {
                    work2[m_idx + i] = 0;
                }
                root_n
            }
        } else {
            root_m
        }
    }

    // Scan the inner pixel region (skip boundaries 0, and max-1)
    for j in 1..lysize - 1 {
        for i in 1..lxsize - 1 {
            let source_idx = match proc_mode {
                ImageProcMode::FrameImage => j * row_stride + i,
                ImageProcMode::FieldImage => (j * 2) * xsize as usize + (i * 2),
            };
            let pixel = image[source_idx];

            let p_idx = j * lxsize + i;

            if is_region(pixel) {
                let left_val = label_img[p_idx - 1];
                let up_left_val = label_img[p_idx - lxsize - 1];
                let up_val = label_img[p_idx - lxsize];
                let up_right_val = label_img[p_idx - lxsize + 1];

                let mut neighbors = [0; 4];
                let mut n_count = 0;
                if left_val > 0 {
                    neighbors[n_count] = left_val as i32;
                    n_count += 1;
                }
                if up_left_val > 0 {
                    neighbors[n_count] = up_left_val as i32;
                    n_count += 1;
                }
                if up_val > 0 {
                    neighbors[n_count] = up_val as i32;
                    n_count += 1;
                }
                if up_right_val > 0 {
                    neighbors[n_count] = up_right_val as i32;
                    n_count += 1;
                }

                if n_count > 0 {
                    // Sort and unique neighbors
                    neighbors[0..n_count].sort_unstable();
                    let mut unique_count = 1;
                    for k in 1..n_count {
                        if neighbors[k] != neighbors[k - 1] {
                            neighbors[unique_count] = neighbors[k];
                            unique_count += 1;
                        }
                    }
                    n_count = unique_count;

                    let mut final_label = neighbors[0];
                    for k in 1..n_count {
                        if neighbors[k] != final_label {
                            final_label = do_union(work, &mut work2, final_label, neighbors[k]);
                        }
                    }
                    label_img[p_idx] = final_label as crate::types::ARLabelingLabelType;

                    let l = (final_label as usize - 1) * 7;
                    work2[l + 0] += 1;
                    work2[l + 1] += i as i64;
                    work2[l + 2] += j as i64;
                    if work2[l + 3] > i as i64 {
                        work2[l + 3] = i as i64;
                    }
                    if work2[l + 4] < i as i64 {
                        work2[l + 4] = i as i64;
                    }
                    if work2[l + 5] > j as i64 {
                        work2[l + 5] = j as i64;
                    }
                    if work2[l + 6] < j as i64 {
                        work2[l + 6] = j as i64;
                    }
                } else {
                    wk_max += 1;
                    if wk_max > AR_LABELING_WORK_SIZE {
                        arlog_e!(
                            "ar_labeling: work array overflow (wk_max={}, limit={})",
                            wk_max,
                            AR_LABELING_WORK_SIZE
                        );
                        return Err("Labeling work array overflow");
                    }
                    work[wk_max - 1] = wk_max as i32;
                    label_img[p_idx] = wk_max as crate::types::ARLabelingLabelType;

                    let l = (wk_max - 1) * 7;
                    work2[l + 0] = 1; // area
                    work2[l + 1] = i as i64; // pos[0]
                    work2[l + 2] = j as i64; // pos[1]
                    work2[l + 3] = i as i64; // clip[0] (xmin)
                    work2[l + 4] = i as i64; // clip[1] (xmax)
                    work2[l + 5] = j as i64; // clip[2] (ymin)
                    work2[l + 6] = j as i64; // clip[3] (ymax)

                    if wk_max == 32 || wk_max == 41 {
                        trace!("Label {} created at ({}, {})", wk_max, i, j);
                    }
                }
            } else {
                label_img[p_idx] = 0;
            }
        }
    }

    // Pass 2: Map equivalence table down to dense sequential indexes
    let mut num_labels = 0;
    // label_map[original_label - 1] = dense_id
    let mut label_map = vec![0; wk_max];

    // Pass 2a: Assign dense IDs to roots
    for i in 1..=wk_max {
        if work[i - 1] == i as i32 {
            // It's a root
            num_labels += 1;
            label_map[i - 1] = num_labels;
        }
    }

    // Pass 2b: Map all original labels back to dense IDs via their roots
    for i in 1..=wk_max {
        let root = find(work, i as i32) as usize;
        label_map[i - 1] = label_map[root - 1];
    }

    label_info.label_num = num_labels;
    if label_info.label_num == 0 {
        return Ok(());
    }

    // Pass 3: Update label_image with final labels
    for i in 0..label_img.len() {
        if label_img[i] > 0 {
            label_img[i] =
                label_map[label_img[i] as usize - 1] as crate::types::ARLabelingLabelType;
        }
    }

    // Finalize label info
    // Transfer from work2 back to label_info

    // Actually, we can just iterate the work2 table and match by finalized label
    if label_info.area.len() < num_labels as usize {
        label_info.area.resize(num_labels as usize, 0);
    }
    if label_info.pos.len() < num_labels as usize {
        label_info.pos.resize(num_labels as usize, [0.0; 2]);
    }
    if label_info.clip.len() < num_labels as usize {
        label_info.clip.resize(num_labels as usize, [0; 4]);
    }

    label_info.area.fill(0);
    for v in label_info.pos.iter_mut() {
        v.fill(0.0);
    }
    for v in label_info.clip.iter_mut() {
        v[0] = lxsize as i32;
        v[1] = 0;
        v[2] = lysize as i32;
        v[3] = 0;
    }

    // Let's redo the finalization loop more cleanly.
    // We'll iterate all ROOTs and move their data to the dense slots.
    // Redo finalization: only process ROOTS
    for k in 1..=wk_max {
        if work[k - 1] == k as i32 {
            // It's a root
            let src = (k - 1) * 7;
            if work2[src + 0] > 0 {
                let dense_idx = label_map[k - 1];
                if dense_idx > 0 {
                    let i = (dense_idx - 1) as usize;
                    label_info.area[i] = work2[src + 0] as i32;
                    label_info.pos[i][0] = work2[src + 1] as ARdouble;
                    label_info.pos[i][1] = work2[src + 2] as ARdouble;
                    label_info.clip[i][0] = work2[src + 3] as i32;
                    label_info.clip[i][1] = work2[src + 4] as i32;
                    label_info.clip[i][2] = work2[src + 5] as i32;
                    label_info.clip[i][3] = work2[src + 6] as i32;
                }
            }
        }
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
    fn test_ar_labeling_two_squares() {
        // Create an 20x20 image where two 5x5 squares are separate.
        // S1: (2,2) to (6,6)
        // S2: (12,12) to (16,16)
        let mut image = vec![255u8; 400];
        for j in 2..7 {
            for i in 2..7 {
                image[j * 20 + i] = 0;
            }
        }
        for j in 12..17 {
            for i in 12..17 {
                image[j * 20 + i] = 0;
            }
        }

        let mut info = crate::types::ARLabelInfo::default();

        ar_labeling(
            &image,
            20,
            20,
            LabelingMode::BlackRegion,
            100,
            ImageProcMode::FrameImage,
            &mut info,
            false,
        )
        .unwrap();

        assert_eq!(info.label_num, 2);

        let mut found_s1 = false;
        let mut found_s2 = false;

        for i in 0..info.label_num as usize {
            if info.area[i] == 25 {
                if info.clip[i][0] == 2
                    && info.clip[i][1] == 6
                    && info.clip[i][2] == 2
                    && info.clip[i][3] == 6
                {
                    found_s1 = true;
                } else if info.clip[i][0] == 12
                    && info.clip[i][1] == 16
                    && info.clip[i][2] == 12
                    && info.clip[i][3] == 16
                {
                    found_s2 = true;
                }
            }
        }

        assert!(
            found_s1,
            "Square 1 (2,2)-(6,6) not correctly found. Area={:?}, Clip={:?}",
            info.area, info.clip
        );
        assert!(
            found_s2,
            "Square 2 (12,12)-(16,16) not correctly found. Area={:?}, Clip={:?}",
            info.area, info.clip
        );
    }
}
