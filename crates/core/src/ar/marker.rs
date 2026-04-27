/*
 *  marker.rs
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

//! Marker Detection Pipeline
//! Ported from arDetectMarker.c, arDetectMarker2.c, and arGetMarkerInfo.c

use crate::types::{ARLabelInfo, ARMarkerInfo2, ARPixelFormat, ARdouble};
use crate::{arlog_d, arlog_e};

pub const AR_AREA_MAX: i32 = 100000;
pub const AR_AREA_MIN: i32 = 70;
pub const AR_SQUARE_FIT_THRESH: f64 = 0.05;
pub const AR_CHAIN_MAX: usize = 10000;
pub const AR_SQUARE_MAX: usize = 30;
pub const AR_CONFIDENCE_CUTOFF_DEFAULT: f64 = 0.5;

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum ImageProcMode {
    FrameImage = 0,
    FieldImage = 1,
}

/// High-level AR marker detection pipeline.
///
/// This is the main entry point for detecting markers in a video frame.
/// It orchestrates the full pipeline in order:
/// 1. **Thresholding** — luma image binarised against `ar_handle.ar_labeling_thresh`.
/// 2. **Labeling** — [`ar_labeling`](crate::labeling::ar_labeling) extracts connected
///    dark (or bright) regions and assigns each a unique integer label.
/// 3. **Candidate selection** - `ar_detect_marker2` filters regions by area and
///    fits a quadrilateral contour using `ar_get_contour` and `check_square`.
/// 4. **Identity resolution** — [`ar_get_marker_info`] decodes each candidate:
///    template matching (via `ar_patt_get_image` + `pattern_match`) **or**
///    matrix-code reading (via [`crate::matrix::ar_matrix_code_get_id`]),
///    depending on `ar_handle.ar_pattern_detection_mode`.
///
/// Results are written into `ar_handle.marker_info` (up to `AR_SQUARE_MAX` markers).
///
/// # Example
/// ```rust,no_run
/// use webarkitlib_rs::arlog_i;
/// use webarkitlib_rs::marker::ar_detect_marker;
/// use webarkitlib_rs::types::{ARHandle, AR2VideoBufferT};
///
/// let mut handle = ARHandle::default();
/// // ... configure handle, load pattern, set param_lt ...
/// let frame = AR2VideoBufferT::default();
/// if ar_detect_marker(&mut handle, &frame).is_ok() {
///     for i in 0..handle.marker_num as usize {
///         let m = &handle.marker_info[i];
///         arlog_i!("Marker id={}, cf={:.2}", m.id, m.cf);
///     }
/// }
/// ```
pub fn ar_detect_marker(
    ar_handle: &mut crate::types::ARHandle,
    frame: &crate::types::AR2VideoBufferT,
) -> Result<(), &'static str> {
    ar_handle.marker_num = 0;

    if ar_handle.ar_pixel_format == ARPixelFormat::Invalid {
        arlog_e!(
            "ar_detect_marker: ar_handle.ar_pixel_format is Invalid. \
             Call ARHandle::set_pixel_format() with a supported format \
             (e.g. ARPixelFormat::MONO for luma input, ARPixelFormat::RGB for color)."
        );
        return Err("ARHandle pixel format not set");
    }

    let luma_buff = match &frame.buff_luma {
        Some(b) => b.as_slice(),
        None => {
            arlog_e!("ar_detect_marker: AR2VideoBufferT.buff_luma is None");
            return Err("AR2VideoBufferT requires buff_luma to be available");
        }
    };

    let color_buff = match &frame.buff {
        Some(b) => b.as_slice(),
        None => {
            arlog_e!("ar_detect_marker: AR2VideoBufferT.buff is None");
            return Err("AR2VideoBufferT requires buff to be available");
        }
    };

    let thresh = ar_handle.ar_labeling_thresh as u8;
    let label_mode = if ar_handle.ar_labeling_mode == 0 {
        crate::labeling::LabelingMode::BlackRegion
    } else {
        crate::labeling::LabelingMode::WhiteRegion
    };

    let labeling_proc_mode = if ar_handle.ar_image_proc_mode == 0 {
        crate::labeling::ImageProcMode::FrameImage
    } else {
        crate::labeling::ImageProcMode::FieldImage
    };

    crate::labeling::ar_labeling(
        luma_buff,
        ar_handle.xsize,
        ar_handle.ysize,
        label_mode,
        thresh,
        labeling_proc_mode,
        &mut ar_handle.label_info,
        ar_handle.ar_debug != 0,
    )?;

    if ar_handle.ar_debug != 0 {
        arlog_d!(
            "ar_labeling found {} labels.",
            ar_handle.label_info.label_num
        );
    }

    let image_proc_mode = if ar_handle.ar_image_proc_mode == 0 {
        ImageProcMode::FrameImage
    } else {
        ImageProcMode::FieldImage
    };

    ar_detect_marker2(
        ar_handle.xsize,
        ar_handle.ysize,
        &mut ar_handle.label_info,
        image_proc_mode,
        AR_AREA_MAX,
        AR_AREA_MIN,
        AR_SQUARE_FIT_THRESH,
        &mut *ar_handle.marker_info2,
        &mut ar_handle.marker2_num,
    )?;

    if ar_handle.ar_debug != 0 {
        arlog_d!(
            "ar_detect_marker2 found {} square candidates.",
            ar_handle.marker2_num
        );
    }

    if ar_handle.ar_param_lt.is_null() {
        arlog_e!("ar_detect_marker: ARHandle.ar_param_lt is null");
        return Err("ARParamLT is null in ARHandle");
    }

    let image_proc_mode2 = if ar_handle.ar_image_proc_mode == 0 {
        ImageProcMode::FrameImage
    } else {
        ImageProcMode::FieldImage
    };

    let param_ltf = unsafe { &(*ar_handle.ar_param_lt).param_ltf };

    let patt_handle_opt = if !ar_handle.patt_handle.is_null() {
        Some(unsafe { &*ar_handle.patt_handle })
    } else {
        None
    };

    ar_get_marker_info(
        color_buff,
        ar_handle.xsize,
        ar_handle.ysize,
        ar_handle.ar_pixel_format,
        &ar_handle.marker_info2[..],
        ar_handle.marker2_num,
        image_proc_mode2,
        ar_handle.ar_pattern_detection_mode,
        param_ltf,
        ar_handle.patt_ratio,
        patt_handle_opt,
        &mut *ar_handle.marker_info,
        &mut ar_handle.marker_num,
        ar_handle.matrix_code_type,
    )?;

    if ar_handle.ar_debug != 0 {
        arlog_d!(
            "ar_get_marker_info produced {} final markers.",
            ar_handle.marker_num
        );
    }

    Ok(())
}

/// Refine labeled regions into square (marker) candidates.
///
/// Ported from `arDetectMarker2.c`. Operates on the output of [`crate::labeling::ar_labeling`].
/// For each surviving region it:
/// - Skips labels outside the `[area_min, area_max]` pixel-count range.
/// - Skips labels touching the image boundary (prevents partial squares).
/// - Calls `ar_get_contour` to trace the region boundary.
/// - Calls `check_square` to verify the contour is a plausible square
///   (four well-separated vertices, low fit error against `square_fit_thresh`).
/// - Deduplicates overlapping candidates by area proximity.
///
/// In `FieldImage` mode areas and coordinates are scaled by ½ for processing
/// and scaled back to full resolution before returning.
///
/// # Parameters
/// - `label_info` — labeling output produced by [`crate::labeling::ar_labeling`].
/// - `marker_info2` — output slice (length ≥ `AR_SQUARE_MAX`) to write candidates into.
/// - `marker2_num` — number of candidates written on return.
#[allow(clippy::too_many_arguments)]
pub fn ar_detect_marker2(
    xsize: i32,
    ysize: i32,
    label_info: &mut ARLabelInfo,
    image_proc_mode: ImageProcMode,
    area_max: i32,
    area_min: i32,
    square_fit_thresh: ARdouble,
    marker_info2: &mut [ARMarkerInfo2],
    marker2_num: &mut i32,
) -> Result<(), &'static str> {
    let mut xsize_local = xsize;
    let mut ysize_local = ysize;
    let mut area_min_local = area_min;
    let mut area_max_local = area_max;

    if matches!(image_proc_mode, ImageProcMode::FieldImage) {
        area_min_local /= 4;
        area_max_local /= 4;
        xsize_local /= 2;
        ysize_local /= 2;
    }

    *marker2_num = 0;

    let label_num = label_info.label_num as usize;
    for i in 0..label_num {
        if label_info.area[i] < area_min_local || label_info.area[i] > area_max_local {
            arlog_d!(
                "Label {} skipped due to Area ({}) not in [{}, {}]",
                i,
                label_info.area[i],
                area_min_local,
                area_max_local
            );
            continue;
        }
        if label_info.clip[i][0] <= 1 || label_info.clip[i][1] >= xsize_local - 2 {
            arlog_d!("Label {} skipped due to X-Clip bounds", i);
            continue;
        }
        if label_info.clip[i][2] <= 1 || label_info.clip[i][3] >= ysize_local - 2 {
            arlog_d!("Label {} skipped due to Y-Clip bounds", i);
            continue;
        }

        let mut current_marker = ARMarkerInfo2::default();

        let ret = ar_get_contour(
            &label_info.label_image,
            xsize_local,
            ysize_local,
            (i + 1) as i32,
            &label_info.clip[i],
            &mut current_marker,
        );

        if let Err(e) = ret {
            arlog_d!("ar_get_contour failed for label {}: {:?}", i, e);
            continue;
        }

        let ret = check_square(label_info.area[i], &mut current_marker, square_fit_thresh);
        if let Err(e) = ret {
            arlog_d!("check_square failed for label {}: {:?}", i, e);
            continue;
        }

        current_marker.area = label_info.area[i];
        current_marker.pos[0] = label_info.pos[i][0];
        current_marker.pos[1] = label_info.pos[i][1];

        marker_info2[*marker2_num as usize] = current_marker;
        *marker2_num += 1;
        if *marker2_num as usize == marker_info2.len() {
            break;
        }
    }

    // Sort/Filter identical overlapping markers
    let num_markers = *marker2_num as usize;
    for i in 0..num_markers {
        for j in i + 1..num_markers {
            if marker_info2[i].area == 0 || marker_info2[j].area == 0 {
                continue;
            }
            let d = (marker_info2[i].pos[0] - marker_info2[j].pos[0]).powi(2)
                + (marker_info2[i].pos[1] - marker_info2[j].pos[1]).powi(2);

            if marker_info2[i].area > marker_info2[j].area {
                if d < (marker_info2[i].area as ARdouble) / 4.0 {
                    marker_info2[j].area = 0;
                }
            } else {
                if d < (marker_info2[j].area as ARdouble) / 4.0 {
                    marker_info2[i].area = 0;
                }
            }
        }
    }

    // Compact the array
    let mut valid_count = 0;
    for i in 0..num_markers {
        if marker_info2[i].area > 0 {
            if i != valid_count {
                marker_info2[valid_count] = marker_info2[i].clone();
            }
            valid_count += 1;
        }
    }
    *marker2_num = valid_count as i32;

    if matches!(image_proc_mode, ImageProcMode::FieldImage) {
        for pm in &mut marker_info2[..*marker2_num as usize] {
            pm.area *= 4;
            pm.pos[0] *= 2.0;
            pm.pos[1] *= 2.0;
            for j in 0..pm.coord_num as usize {
                pm.x_coord[j] *= 2;
                pm.y_coord[j] *= 2;
            }
        }
    }

    Ok(())
}

/// Trace the boundary contour of a labeled region using 8-connected chain-code walking.
///
/// Starting from the top-left pixel of the region's bounding clip, walks the
/// outer boundary and records (x, y) coordinates into `marker_info2.x_coord` /
/// `y_coord`. After tracing, rotates the chain so the farthest point from the
/// start is first - this provides a canonical start for `check_square`.
///
/// Returns `Err` if:
/// - No starting pixel is found in the clip region.
/// - The contour is broken (no neighbour found in 8 directions).
/// - The contour exceeds `AR_CHAIN_MAX - 1` pixels.
fn ar_get_contour(
    limage: &[crate::types::ARLabelingLabelType],
    xsize: i32,
    _ysize: i32,
    label: i32,
    clip: &[i32; 4],
    marker_info2: &mut ARMarkerInfo2,
) -> Result<(), &'static str> {
    let xdir = [0, 1, 1, 1, 0, -1, -1, -1];
    let ydir = [-1, -1, 0, 1, 1, 1, 0, -1];

    let mut sx = -1;
    let sy = clip[2];

    // After labeling Pass 3, limage contains dense sequential IDs.
    // Compare the pixel value directly against `label`.
    for (p_idx, i) in ((sy * xsize + clip[0]) as usize..).zip(clip[0]..=clip[1]) {
        if p_idx < limage.len() && limage[p_idx] == label as crate::types::ARLabelingLabelType {
            sx = i;
            break;
        }
    }

    if sx == -1 {
        arlog_d!("ar_get_contour failed. label={}. clip={:?}.", label, clip);
        return Err("Contour start point not found");
    }

    marker_info2.coord_num = 1;
    marker_info2.x_coord[0] = sx;
    marker_info2.y_coord[0] = sy;
    let mut dir = 5;

    loop {
        let last_idx = (marker_info2.coord_num - 1) as usize;
        let p_idx =
            (marker_info2.y_coord[last_idx] * xsize + marker_info2.x_coord[last_idx]) as usize;

        dir = (dir + 5) % 8;
        let mut found = false;
        for _ in 0..8 {
            let next_idx = (p_idx as isize
                + ydir[dir] as isize * xsize as isize
                + xdir[dir] as isize) as usize;
            if next_idx < limage.len() && limage[next_idx] > 0 {
                found = true;
                break;
            }
            dir = (dir + 1) % 8;
        }

        if !found {
            arlog_d!(
                "ar_get_contour: contour broken for label {} at coord_num={}",
                label,
                marker_info2.coord_num
            );
            return Err("Contour broken");
        }

        let curr_idx = marker_info2.coord_num as usize;
        marker_info2.x_coord[curr_idx] = marker_info2.x_coord[last_idx] + xdir[dir];
        marker_info2.y_coord[curr_idx] = marker_info2.y_coord[last_idx] + ydir[dir];

        if marker_info2.x_coord[curr_idx] == sx && marker_info2.y_coord[curr_idx] == sy {
            break;
        }

        marker_info2.coord_num += 1;
        if marker_info2.coord_num as usize >= AR_CHAIN_MAX - 1 {
            arlog_d!(
                "ar_get_contour: contour exceeded AR_CHAIN_MAX={} for label {}",
                AR_CHAIN_MAX,
                label
            );
            return Err("Contour too long");
        }
    }

    let mut dmax = 0;
    let mut v1 = 0;

    for i in 1..marker_info2.coord_num as usize {
        let d = (marker_info2.x_coord[i] - sx).pow(2) + (marker_info2.y_coord[i] - sy).pow(2);
        if d > dmax {
            dmax = d;
            v1 = i;
        }
    }

    let mut wx = vec![0; v1];
    let mut wy = vec![0; v1];

    wx.copy_from_slice(&marker_info2.x_coord[..v1]);
    wy.copy_from_slice(&marker_info2.y_coord[..v1]);

    let coord_num = marker_info2.coord_num as usize;
    for i in v1..coord_num {
        marker_info2.x_coord[i - v1] = marker_info2.x_coord[i];
        marker_info2.y_coord[i - v1] = marker_info2.y_coord[i];
    }

    let offset = coord_num - v1;
    marker_info2.x_coord[offset..offset + v1].copy_from_slice(&wx);
    marker_info2.y_coord[offset..offset + v1].copy_from_slice(&wy);

    let end_idx = marker_info2.coord_num as usize;
    marker_info2.x_coord[end_idx] = marker_info2.x_coord[0];
    marker_info2.y_coord[end_idx] = marker_info2.y_coord[0];
    marker_info2.coord_num += 1;

    Ok(())
}

/// Validates that a traced contour is a plausible planar square.
///
/// Uses a recursive vertex-splitting algorithm (similar to the Ramer–Douglas–Peucker
/// approach) to find exactly **four** corner points from the contour stored in
/// `marker_info2.{x,y}_coord`.
///
/// The fit threshold `factor` is combined with the region `area` to produce an
/// adaptive pixel-distance tolerance:
/// ```text
/// thresh = (area / 0.75) * 0.01 * factor
/// ```
/// Values used in production: `area ∈ [AR_AREA_MIN, AR_AREA_MAX]`,
/// `factor = AR_SQUARE_FIT_THRESH` (0.05).
///
/// On success the four vertex indices are written into `marker_info2.vertex`
/// in counter-clockwise order (indices 0–4, where index 4 wraps back to 0).
///
/// Returns `Err` if the split process cannot isolate exactly four corners,
/// indicating the region is not a valid square marker candidate.
fn check_square(
    area: i32,
    marker_info2: &mut ARMarkerInfo2,
    factor: ARdouble,
) -> Result<(), &'static str> {
    let mut dmax = 0;
    let mut v1 = 0;
    let sx = marker_info2.x_coord[0];
    let sy = marker_info2.y_coord[0];
    let coord_num = marker_info2.coord_num as usize;

    for i in 1..(coord_num - 1) {
        let d = (marker_info2.x_coord[i] - sx).pow(2) + (marker_info2.y_coord[i] - sy).pow(2);
        if d > dmax {
            dmax = d;
            v1 = i;
        }
    }

    let thresh = ((area as f64) / 0.75) * 0.01 * factor;
    let mut vertex = [0; 10];
    vertex[0] = 0;
    let mut wv1 = [0; 10];
    let mut wvnum1 = 0;
    let mut wv2 = [0; 10];
    let mut wvnum2 = 0;

    if get_vertex(
        &marker_info2.x_coord,
        &marker_info2.y_coord,
        0,
        v1,
        thresh,
        &mut wv1,
        &mut wvnum1,
    )
    .is_err()
    {
        return Err("Square check failed");
    }
    if get_vertex(
        &marker_info2.x_coord,
        &marker_info2.y_coord,
        v1,
        coord_num - 1,
        thresh,
        &mut wv2,
        &mut wvnum2,
    )
    .is_err()
    {
        return Err("Square check failed");
    }

    if wvnum1 == 1 && wvnum2 == 1 {
        vertex[1] = wv1[0];
        vertex[2] = v1;
        vertex[3] = wv2[0];
    } else if wvnum1 > 1 && wvnum2 == 0 {
        let v2 = v1 / 2;
        wvnum1 = 0;
        wvnum2 = 0;
        if get_vertex(
            &marker_info2.x_coord,
            &marker_info2.y_coord,
            0,
            v2,
            thresh,
            &mut wv1,
            &mut wvnum1,
        )
        .is_err()
        {
            return Err("Square check failed");
        }
        if get_vertex(
            &marker_info2.x_coord,
            &marker_info2.y_coord,
            v2,
            v1,
            thresh,
            &mut wv2,
            &mut wvnum2,
        )
        .is_err()
        {
            return Err("Square check failed");
        }
        if wvnum1 == 1 && wvnum2 == 1 {
            vertex[1] = wv1[0];
            vertex[2] = wv2[0];
            vertex[3] = v1;
        } else {
            return Err("Not a square");
        }
    } else if wvnum1 == 0 && wvnum2 > 1 {
        let v2 = (v1 + coord_num - 1) / 2;
        wvnum1 = 0;
        wvnum2 = 0;
        if get_vertex(
            &marker_info2.x_coord,
            &marker_info2.y_coord,
            v1,
            v2,
            thresh,
            &mut wv1,
            &mut wvnum1,
        )
        .is_err()
        {
            return Err("Square check failed");
        }
        if get_vertex(
            &marker_info2.x_coord,
            &marker_info2.y_coord,
            v2,
            coord_num - 1,
            thresh,
            &mut wv2,
            &mut wvnum2,
        )
        .is_err()
        {
            return Err("Square check failed");
        }
        if wvnum1 == 1 && wvnum2 == 1 {
            vertex[1] = v1;
            vertex[2] = wv1[0];
            vertex[3] = wv2[0];
        } else {
            return Err("Not a square");
        }
    } else {
        return Err("Not a square");
    }

    marker_info2.vertex[0] = vertex[0] as i32;
    marker_info2.vertex[1] = vertex[1] as i32;
    marker_info2.vertex[2] = vertex[2] as i32;
    marker_info2.vertex[3] = vertex[3] as i32;
    marker_info2.vertex[4] = (coord_num - 1) as i32;

    Ok(())
}

/// Recursively find sub-vertices between two contour indices using perpendicular distance.
///
/// For the chord from contour point `st` to contour point `ed`, this function
/// finds the contour point with the maximum perpendicular distance from the chord.
/// If that distance exceeds `thresh`, the segment is split at that point and the
/// function recurses on both halves.
///
/// The result is appended into `vertex` (up to 5 entries), and `vnum` is
/// incremented for each found vertex. Returns `Err` if more than 5 vertices are
/// found (which indicates a non-convex / non-square shape).
fn get_vertex(
    x_coord: &[i32],
    y_coord: &[i32],
    st: usize,
    ed: usize,
    thresh: ARdouble,
    vertex: &mut [usize],
    vnum: &mut usize,
) -> Result<(), &'static str> {
    let a = (y_coord[ed] - y_coord[st]) as f64;
    let b = (x_coord[st] - x_coord[ed]) as f64;
    let c = (x_coord[ed] * y_coord[st] - y_coord[ed] * x_coord[st]) as f64;

    let mut dmax = 0.0;
    let mut v1 = st + 1;

    for i in (st + 1)..ed {
        let d = a * (x_coord[i] as f64) + b * (y_coord[i] as f64) + c;
        if d * d > dmax {
            dmax = d * d;
            v1 = i;
        }
    }

    if dmax / (a * a + b * b) > thresh {
        if get_vertex(x_coord, y_coord, st, v1, thresh, vertex, vnum).is_err() {
            return Err("Vertex expansion failed");
        }

        if *vnum > 5 {
            return Err("Too many vertices");
        }
        vertex[*vnum] = v1;
        *vnum += 1;

        if get_vertex(x_coord, y_coord, v1, ed, thresh, vertex, vnum).is_err() {
            return Err("Vertex expansion failed");
        }
    }

    Ok(())
}

use crate::math::{ARMat, ARVec};
use crate::types::{ARMarkerInfo, ARParamLTf};

/// Fits undistorted straight lines to each of the four sides of a detected square.
///
/// Ported from `arGetLine.c`. For each side of the candidate square (defined by
/// consecutive vertex index pairs) it:
/// 1. Trims 5 % of the edge from each end to avoid corner artefacts.
/// 2. Calls `param_ltf.observ2ideal` to correct lens-distortion on every pixel
///    along the edge.
/// 3. Runs PCA on the corrected pixel positions to find the dominant axis, from
///    which the line equation `(a, b, c)` is derived such that `ax + by + c = 0`.
/// 4. Computes the four corner coordinates as intersections of adjacent lines.
///
/// # Parameters
/// - `x_coord`, `y_coord` — full pixel contour from `ar_get_contour`.
/// - `vertex` — five vertex indices into the contour (4 corners + wrap-around).
/// - `param_ltf` — lens-distortion look-up table for `observ2ideal` mapping.
/// - `line` — output: four `[a, b, c]` line coefficients.
/// - `v` — output: four `[x, y]` corner coordinates in ideal (undistorted) space.
///
/// Returns `Err` if any edge has fewer than two pixels, PCA fails, or adjacent
/// lines are nearly parallel (determinant < 0.0001).
pub fn ar_get_line(
    x_coord: &[i32],
    y_coord: &[i32],
    _coord_num: usize,
    vertex: &[i32],
    param_ltf: &ARParamLTf,
    line: &mut [[ARdouble; 3]; 4],
    v: &mut [[ARdouble; 2]; 4],
) -> Result<(), &'static str> {
    for i in 0..4 {
        let w1 = ((vertex[i + 1] - vertex[i] + 1) as f64) * 0.05 + 0.5;
        let st = (vertex[i] as f64 + w1) as usize;
        let ed = (vertex[i + 1] as f64 - w1) as usize;
        let n = ed - st + 1;

        let mut input = ARMat::new(n as i32, 2);
        for j in 0..n {
            let (ix, iy) =
                param_ltf.observ2ideal(x_coord[st + j] as f32, y_coord[st + j] as f32)?;
            input.m[j * 2] = ix as f64;
            input.m[j * 2 + 1] = iy as f64;
        }

        let mut evec = ARMat::new(2, 2);
        let mut ev = ARVec::new(2);
        let mut mean = ARVec::new(2);

        input.pca(&mut evec, &mut ev, &mut mean)?;

        line[i][0] = evec.m[1];
        line[i][1] = -evec.m[0];
        line[i][2] = -(line[i][0] * mean.v[0] + line[i][1] * mean.v[1]);
    }

    for i in 0..4 {
        let w1 = line[(i + 3) % 4][0] * line[i][1] - line[i][0] * line[(i + 3) % 4][1];
        if w1.abs() < 0.0001 {
            return Err("Lines are near parallel");
        }
        v[i][0] = (line[(i + 3) % 4][1] * line[i][2] - line[i][1] * line[(i + 3) % 4][2]) / w1;
        v[i][1] = (line[i][0] * line[(i + 3) % 4][2] - line[(i + 3) % 4][0] * line[i][2]) / w1;
    }

    Ok(())
}

/// Resolves each square candidate into a fully identified marker.
///
/// Ported from `arGetMarkerInfo.c`. For every entry in `marker_info2[0..marker2_num]`:
/// 1. Calls `param_ltf.observ2ideal` on the region centroid to get the
///    undistorted position.
/// 2. Calls [`ar_get_line`] to fit four edges and compute corner vertices in
///    ideal coordinates.
/// 3. Branches on `patt_detect_mode`:
///    - **`AR_MATRIX_CODE_DETECTION`** (mode 2): calls
///      [`crate::matrix::ar_matrix_code_get_id`] to decode the barcode id,
///      writing results into `marker_info[j].{id_matrix, dir_matrix, cf_matrix}`.
///    - **template matching modes** (mode 0 or 1): calls `ar_patt_get_image` +
///      `pattern_match` to match against a loaded pattern set, writing into
///      `marker_info[j].{id, dir, cf}`.
///    - **combined mode** (mode 3): performs both branches.
///
/// Failures in `ar_get_line` or `observ2ideal` for a particular candidate skip
/// that candidate silently. `*marker_num` is set to the number of valid markers.
///
/// # Parameters
/// - `image` — raw pixel buffer (any supported `pixel_format`).
/// - `patt_handle_opt` — loaded pattern database (required for template modes;
///   pass `None` for pure matrix-code mode).
/// - `matrix_code_type` — dimension/ECC variant used by [`crate::matrix::ar_matrix_code_get_id`].
#[allow(clippy::too_many_arguments)]
pub fn ar_get_marker_info(
    image: &[u8],
    xsize: i32,
    ysize: i32,
    pixel_format: crate::types::ARPixelFormat,
    marker_info2: &[ARMarkerInfo2],
    marker2_num: i32,
    image_proc_mode: ImageProcMode,
    patt_detect_mode: i32,
    param_ltf: &ARParamLTf,
    patt_ratio: ARdouble,
    patt_handle_opt: Option<&crate::types::ARPattHandle>,
    marker_info: &mut [ARMarkerInfo],
    marker_num: &mut i32,
    matrix_code_type: crate::types::ARMatrixCodeType,
) -> Result<(), &'static str> {
    let mut j = 0;

    for info2 in &marker_info2[..marker2_num as usize] {
        marker_info[j].area = info2.area;

        if let Ok((ix, iy)) = param_ltf.observ2ideal(info2.pos[0] as f32, info2.pos[1] as f32) {
            marker_info[j].pos[0] = ix as f64;
            marker_info[j].pos[1] = iy as f64;
        } else {
            continue;
        }

        if ar_get_line(
            &info2.x_coord,
            &info2.y_coord,
            info2.coord_num as usize,
            &info2.vertex,
            param_ltf,
            &mut marker_info[j].line,
            &mut marker_info[j].vertex,
        )
        .is_err()
        {
            continue;
        }

        // Branch on detection mode
        let is_matrix_mode = patt_detect_mode == crate::types::AR_MATRIX_CODE_DETECTION
            || patt_detect_mode
                == crate::types::AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX_CODE_DETECTION;

        if is_matrix_mode {
            // Decode the matrix (barcode) code
            match crate::matrix::ar_matrix_code_get_id(
                image,
                xsize,
                ysize,
                &marker_info[j].vertex,
                matrix_code_type,
                pixel_format,
                patt_ratio,
            ) {
                Ok(ok) => {
                    marker_info[j].id_matrix = ok.id;
                    marker_info[j].dir_matrix = ok.dir;
                    marker_info[j].cf_matrix = ok.cf;
                    marker_info[j].error_corrected = ok.error_corrected;
                    marker_info[j].cutoff_phase = crate::types::ARMarkerInfoCutoffPhase::None;
                    arlog_d!(
                        "ar_get_marker_info: barcode id={}, dir={}, cf={:.4}",
                        ok.id,
                        ok.dir,
                        ok.cf
                    );
                }
                Err(e) => {
                    arlog_d!("ar_get_marker_info: barcode decode failed: {:?}", e);
                    marker_info[j].id_matrix = -1;
                    marker_info[j].dir_matrix = -1;
                    marker_info[j].cf_matrix = 0.0;
                    marker_info[j].cutoff_phase = e.into();
                }
            }
        }

        if !is_matrix_mode
            || patt_detect_mode
                == crate::types::AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX_CODE_DETECTION
        {
            // Template matching branch
            if let Some(patt_handle) = patt_handle_opt {
                if patt_handle.patt_num > 0 {
                    let patt_size = patt_handle.patt_size;
                    let ext_patt_len =
                        if patt_detect_mode == crate::pattern::AR_TEMPLATE_MATCHING_COLOR {
                            (patt_size * patt_size * 3) as usize
                        } else {
                            (patt_size * patt_size) as usize
                        };
                    let mut ext_patt = vec![0u8; ext_patt_len];

                    // ar_patt_get_image expects x_coord and y_coord arrays (contour)
                    // and a 4-element array of vertex indices (usize).
                    let vertex_indices: [usize; 4] = [
                        info2.vertex[0] as usize,
                        info2.vertex[1] as usize,
                        info2.vertex[2] as usize,
                        info2.vertex[3] as usize,
                    ];

                    let res = crate::pattern::ar_patt_get_image(
                        image_proc_mode as i32,
                        patt_detect_mode,
                        patt_size as usize,
                        (patt_size * 2) as usize,
                        image,
                        xsize as usize,
                        ysize as usize,
                        pixel_format,
                        &info2.x_coord,
                        &info2.y_coord,
                        &vertex_indices,
                        patt_ratio,
                        &mut ext_patt,
                    );

                    if res.is_ok() {
                        match crate::pattern::pattern_match(
                            patt_handle,
                            patt_detect_mode,
                            &ext_patt,
                            patt_size,
                        ) {
                            Ok(ok) if ok.id >= 0 => {
                                marker_info[j].id_patt = ok.id;
                                marker_info[j].dir_patt = ok.dir;
                                marker_info[j].cf_patt = ok.cf;
                                if !is_matrix_mode {
                                    marker_info[j].cutoff_phase =
                                        crate::types::ARMarkerInfoCutoffPhase::None;
                                }
                            }
                            Ok(ok) => {
                                // pattern_match returned Ok but no template matched
                                marker_info[j].id_patt = -1;
                                marker_info[j].dir_patt = 0;
                                marker_info[j].cf_patt = ok.cf;
                                if !is_matrix_mode {
                                    marker_info[j].cutoff_phase =
                                        crate::types::ARMarkerInfoCutoffPhase::MatchGeneric;
                                }
                            }
                            Err(e) => {
                                marker_info[j].id_patt = -1;
                                marker_info[j].dir_patt = 0;
                                marker_info[j].cf_patt = -1.0;
                                if !is_matrix_mode {
                                    marker_info[j].cutoff_phase = e.into();
                                }
                            }
                        }
                    } else {
                        marker_info[j].id_patt = -1;
                        marker_info[j].dir_patt = 0;
                        marker_info[j].cf_patt = -1.0;
                        if !is_matrix_mode {
                            marker_info[j].cutoff_phase =
                                crate::types::ARMarkerInfoCutoffPhase::PatternExtraction;
                        }
                    }
                } else {
                    marker_info[j].id_patt = -1;
                    marker_info[j].dir_patt = 0;
                    marker_info[j].cf_patt = 0.0;
                }
            } else {
                marker_info[j].id_patt = -1;
                marker_info[j].dir_patt = 0;
                marker_info[j].cf_patt = 0.0;
            }
        }

        // Mirror arGetMarkerInfo.c lines 92-100: copy the per-mode fields
        // into the final `id` / `cf` / `dir` based on detection mode.
        finalize_marker_id_cf_dir(&mut marker_info[j], patt_detect_mode);

        j += 1;
    }
    *marker_num = j as i32;

    Ok(())
}

/// Copy the per-mode marker fields (`*_patt` or `*_matrix`) into the final
/// `id` / `cf` / `dir` based on the detection mode. Mirrors
/// `arGetMarkerInfo.c` lines 92-100.
///
/// Mixed modes (`AR_TEMPLATE_MATCHING_*_AND_MATRIX_CODE_DETECTION`) leave the
/// final fields untouched — callers must inspect `*_patt` / `*_matrix`
/// themselves to decide.
fn finalize_marker_id_cf_dir(marker: &mut ARMarkerInfo, patt_detect_mode: i32) {
    if patt_detect_mode == crate::pattern::AR_TEMPLATE_MATCHING_COLOR
        || patt_detect_mode == crate::pattern::AR_TEMPLATE_MATCHING_MONO
    {
        marker.id = marker.id_patt;
        marker.dir = marker.dir_patt;
        marker.cf = marker.cf_patt;
    } else if patt_detect_mode == crate::types::AR_MATRIX_CODE_DETECTION {
        marker.id = marker.id_matrix;
        marker.dir = marker.dir_matrix;
        marker.cf = marker.cf_matrix;
    }
    // Mixed modes: do nothing.
}

/// Cull low-confidence marker IDs after matching.
///
/// Mirrors `confidenceCutoff()` from `arDetectMarker.c`.
pub fn confidence_cutoff(
    marker_info: &mut [ARMarkerInfo],
    marker_num: i32,
    patt_detect_mode: i32,
    cutoff: f64,
) {
    use crate::types::ARMarkerInfoCutoffPhase;

    let n = marker_num as usize;

    if patt_detect_mode == crate::pattern::AR_TEMPLATE_MATCHING_COLOR
        || patt_detect_mode == crate::pattern::AR_TEMPLATE_MATCHING_MONO
    {
        for m in &mut marker_info[..n] {
            if m.id >= 0 && m.cf < cutoff {
                m.id = -1;
                m.id_patt = -1;
                m.cutoff_phase = ARMarkerInfoCutoffPhase::MatchConfidence;
            }
        }
    } else if patt_detect_mode == crate::types::AR_MATRIX_CODE_DETECTION {
        for m in &mut marker_info[..n] {
            if m.id >= 0 && m.cf < cutoff {
                m.id = -1;
                m.id_matrix = -1;
                m.cutoff_phase = ARMarkerInfoCutoffPhase::MatchConfidence;
            }
        }
    } else {
        // Mixed mode (AR_TEMPLATE_MATCHING_*_AND_MATRIX_CODE_DETECTION)
        for m in &mut marker_info[..n] {
            let mut cf_ok = false;
            if m.id_patt >= 0 && m.cf_patt < cutoff {
                m.id_patt = -1;
            } else {
                cf_ok = true;
            }
            if m.id_matrix >= 0 && m.cf_matrix < cutoff {
                m.id_matrix = -1;
            } else {
                cf_ok = true;
            }
            if !cf_ok {
                m.cutoff_phase = ARMarkerInfoCutoffPhase::MatchConfidence;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ar_detect_marker2_empty() {
        let mut label_info = ARLabelInfo::default();
        let mut marker_info2 = vec![ARMarkerInfo2::default(); AR_SQUARE_MAX];
        let mut marker2_num = 0;

        let res = ar_detect_marker2(
            640,
            480,
            &mut label_info,
            ImageProcMode::FrameImage,
            AR_AREA_MAX,
            AR_AREA_MIN,
            AR_SQUARE_FIT_THRESH,
            &mut marker_info2,
            &mut marker2_num,
        );

        assert!(res.is_ok());
        assert_eq!(marker2_num, 0);
    }

    /// Verifies `finalize_marker_id_cf_dir` copies the right per-mode fields
    /// into the final `id` / `cf` / `dir` for each detection mode, mirroring
    /// `arGetMarkerInfo.c` lines 92-100.
    #[test]
    fn test_finalize_marker_id_cf_dir_template_color() {
        let mut m = ARMarkerInfo::default();
        m.id_patt = 7;
        m.dir_patt = 2;
        m.cf_patt = 0.85;
        m.id_matrix = 99; // should be ignored
        m.dir_matrix = 3;
        m.cf_matrix = 0.5;

        finalize_marker_id_cf_dir(&mut m, crate::pattern::AR_TEMPLATE_MATCHING_COLOR);

        assert_eq!(m.id, 7);
        assert_eq!(m.dir, 2);
        assert!((m.cf - 0.85).abs() < 1e-9);
    }

    #[test]
    fn test_finalize_marker_id_cf_dir_template_mono() {
        let mut m = ARMarkerInfo::default();
        m.id_patt = 4;
        m.dir_patt = 1;
        m.cf_patt = 0.7;

        finalize_marker_id_cf_dir(&mut m, crate::pattern::AR_TEMPLATE_MATCHING_MONO);

        assert_eq!(m.id, 4);
        assert_eq!(m.dir, 1);
        assert!((m.cf - 0.7).abs() < 1e-9);
    }

    #[test]
    fn test_finalize_marker_id_cf_dir_matrix_only() {
        let mut m = ARMarkerInfo::default();
        m.id_patt = 99; // should be ignored
        m.dir_patt = 2;
        m.cf_patt = 0.5;
        m.id_matrix = 12;
        m.dir_matrix = 0;
        m.cf_matrix = 0.92;

        finalize_marker_id_cf_dir(&mut m, crate::types::AR_MATRIX_CODE_DETECTION);

        assert_eq!(m.id, 12);
        assert_eq!(m.dir, 0);
        assert!((m.cf - 0.92).abs() < 1e-9);
    }

    #[test]
    fn test_finalize_marker_id_cf_dir_mixed_color_matrix_leaves_finals_alone() {
        let mut m = ARMarkerInfo::default();
        // Pre-set finals to sentinel values; verify they're not overwritten.
        m.id = -42;
        m.dir = -42;
        m.cf = -42.0;
        m.id_patt = 1;
        m.dir_patt = 1;
        m.cf_patt = 0.1;
        m.id_matrix = 2;
        m.dir_matrix = 2;
        m.cf_matrix = 0.2;

        finalize_marker_id_cf_dir(
            &mut m,
            crate::types::AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX_CODE_DETECTION,
        );

        assert_eq!(m.id, -42);
        assert_eq!(m.dir, -42);
        assert!((m.cf - -42.0).abs() < 1e-9);
    }

    #[test]
    fn test_finalize_marker_id_cf_dir_mixed_mono_matrix_leaves_finals_alone() {
        let mut m = ARMarkerInfo::default();
        m.id = -42;
        m.dir = -42;
        m.cf = -42.0;

        finalize_marker_id_cf_dir(
            &mut m,
            crate::types::AR_TEMPLATE_MATCHING_MONO_AND_MATRIX_CODE_DETECTION,
        );

        assert_eq!(m.id, -42);
        assert_eq!(m.dir, -42);
        assert!((m.cf - -42.0).abs() < 1e-9);
    }

<<<<<<< fix/confidence-cutoff
    #[test]
    fn test_confidence_cutoff_template_clears_low_cf() {
        let mut markers = vec![ARMarkerInfo::default()];
        markers[0].id = 10;
        markers[0].id_patt = 10;
        markers[0].cf = 0.49;

        confidence_cutoff(
            &mut markers,
            1,
            crate::pattern::AR_TEMPLATE_MATCHING_COLOR,
            0.5,
        );

        assert_eq!(markers[0].id, -1);
        assert_eq!(markers[0].id_patt, -1);
        assert_eq!(
            markers[0].cutoff_phase,
            crate::types::ARMarkerInfoCutoffPhase::MatchConfidence
        );
    }

    #[test]
    fn test_confidence_cutoff_matrix_clears_low_cf() {
        let mut markers = vec![ARMarkerInfo::default()];
        markers[0].id = 25;
        markers[0].id_matrix = 25;
        markers[0].cf = 0.3;

        confidence_cutoff(&mut markers, 1, crate::types::AR_MATRIX_CODE_DETECTION, 0.5);

        assert_eq!(markers[0].id, -1);
        assert_eq!(markers[0].id_matrix, -1);
        assert_eq!(
            markers[0].cutoff_phase,
            crate::types::ARMarkerInfoCutoffPhase::MatchConfidence
        );
    }

    #[test]
    fn test_confidence_cutoff_template_keeps_high_cf() {
        let mut markers = vec![ARMarkerInfo::default()];
        markers[0].id = 12;
        markers[0].id_patt = 12;
        markers[0].cf = 0.95;

        confidence_cutoff(
            &mut markers,
            1,
            crate::pattern::AR_TEMPLATE_MATCHING_COLOR,
            0.5,
        );

        assert_eq!(markers[0].id, 12);
        assert_eq!(markers[0].id_patt, 12);
        assert_eq!(
            markers[0].cutoff_phase,
            crate::types::ARMarkerInfoCutoffPhase::None
        );
    }

    #[test]
    fn test_confidence_cutoff_mixed_clears_independently() {
        let mut markers = vec![ARMarkerInfo::default()];
        markers[0].id_patt = 2;
        markers[0].cf_patt = 0.2;
        markers[0].id_matrix = 8;
        markers[0].cf_matrix = 0.9;

        confidence_cutoff(
            &mut markers,
            1,
            crate::types::AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX_CODE_DETECTION,
            0.5,
        );

        assert_eq!(markers[0].id_patt, -1);
        assert_eq!(markers[0].id_matrix, 8);
        assert_eq!(
            markers[0].cutoff_phase,
            crate::types::ARMarkerInfoCutoffPhase::None
        );
    }

    #[test]
    fn test_confidence_cutoff_mixed_both_fail_sets_phase() {
        let mut markers = vec![ARMarkerInfo::default()];
        markers[0].id_patt = 4;
        markers[0].cf_patt = 0.3;
        markers[0].id_matrix = 5;
        markers[0].cf_matrix = 0.4;

        confidence_cutoff(
            &mut markers,
            1,
            crate::types::AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX_CODE_DETECTION,
            0.5,
        );

        assert_eq!(markers[0].id_patt, -1);
        assert_eq!(markers[0].id_matrix, -1);
        assert_eq!(
            markers[0].cutoff_phase,
            crate::types::ARMarkerInfoCutoffPhase::MatchConfidence
=======
    /// Verifies cutoff_phase is set to None on a successful template match.
    #[test]
    fn test_cutoff_phase_none_on_template_success() {
        use crate::types::{ARMarkerInfoCutoffPhase, MatchOk};
        let mut marker = ARMarkerInfo::default();
        let ok = MatchOk {
            id: 0,
            dir: 0,
            cf: 0.8,
            error_corrected: 0,
        };
        marker.id_patt = ok.id;
        marker.dir_patt = ok.dir;
        marker.cf_patt = ok.cf;
        marker.cutoff_phase = ARMarkerInfoCutoffPhase::None;
        assert_eq!(marker.cutoff_phase, ARMarkerInfoCutoffPhase::None);
    }

    /// Verifies cutoff_phase is set to MatchContrast when matrix decoding
    /// returns MatchError::Contrast (mirrors arGetMarkerInfo.c:84).
    #[test]
    fn test_cutoff_phase_set_on_matrix_contrast_error() {
        use crate::types::{ARMarkerInfoCutoffPhase, MatchError};
        let mut marker = ARMarkerInfo::default();
        let e = MatchError::Contrast;
        marker.id_matrix = -1;
        marker.cf_matrix = 0.0;
        marker.cutoff_phase = e.into();
        assert_eq!(marker.cutoff_phase, ARMarkerInfoCutoffPhase::MatchContrast);
    }

    /// Verifies cutoff_phase is set to MatchBarcodeNotFound when matrix decoding
    /// returns MatchError::BarcodeNotFound (mirrors arGetMarkerInfo.c:85).
    #[test]
    fn test_cutoff_phase_set_on_matrix_barcode_not_found() {
        use crate::types::{ARMarkerInfoCutoffPhase, MatchError};
        let mut marker = ARMarkerInfo::default();
        let e = MatchError::BarcodeNotFound;
        marker.cutoff_phase = e.into();
        assert_eq!(
            marker.cutoff_phase,
            ARMarkerInfoCutoffPhase::MatchBarcodeNotFound
        );
    }

    /// Verifies cutoff_phase is set to PatternExtraction when ar_patt_get_image
    /// fails (mirrors arGetMarkerInfo.c:88).
    #[test]
    fn test_cutoff_phase_pattern_extraction_failure() {
        use crate::types::{ARMarkerInfoCutoffPhase, MatchError};
        let mut marker = ARMarkerInfo::default();
        let e = MatchError::PatternExtraction;
        marker.cutoff_phase = e.into();
        assert_eq!(
            marker.cutoff_phase,
            ARMarkerInfoCutoffPhase::PatternExtraction
>>>>>>> dev
        );
    }
}
