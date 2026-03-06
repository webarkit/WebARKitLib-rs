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

use crate::types::{ARLabelInfo, ARMarkerInfo2, ARdouble};
use log::debug;

pub const AR_AREA_MAX: i32 = 100000;
pub const AR_AREA_MIN: i32 = 70;
pub const AR_SQUARE_FIT_THRESH: f64 = 0.05;
pub const AR_CHAIN_MAX: usize = 10000;
pub const AR_SQUARE_MAX: usize = 30;

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum ImageProcMode {
    FrameImage = 0,
    FieldImage = 1,
}

/// High-level marker detection pipeline ported from arDetectMarker.
/// This method handles thresholding, region extraction, subpixel refinement, and template matching.
pub fn ar_detect_marker(
    ar_handle: &mut crate::types::ARHandle,
    frame: &crate::types::AR2VideoBufferT,
) -> Result<(), &'static str> {
    ar_handle.marker_num = 0;
    
    let luma_buff = match &frame.buff_luma {
        Some(b) => b.as_slice(),
        None => return Err("AR2VideoBufferT requires buff_luma to be available"),
    };
    
    let color_buff = match &frame.buff {
        Some(b) => b.as_slice(),
        None => return Err("AR2VideoBufferT requires buff to be available"),
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
        ar_handle.ar_debug != 0
    )?;
    
    if ar_handle.ar_debug != 0 {
        debug!("ar_labeling found {} labels.", ar_handle.label_info.label_num);
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
        debug!("ar_detect_marker2 found {} square candidates.", ar_handle.marker2_num);
    }

    if ar_handle.ar_param_lt.is_null() {
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
        debug!("ar_get_marker_info produced {} final markers.", ar_handle.marker_num);
    }

    Ok(())
}

/// Ported from arDetectMarker2 in arDetectMarker2.c
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
            debug!("Label {} skipped due to Area ({}) not in [{}, {}]", i, label_info.area[i], area_min_local, area_max_local); 
            continue;
        }
        if label_info.clip[i][0] <= 1 || label_info.clip[i][1] >= xsize_local - 2 {
            debug!("Label {} skipped due to X-Clip bounds", i);
            continue;
        }
        if label_info.clip[i][2] <= 1 || label_info.clip[i][3] >= ysize_local - 2 {
            debug!("Label {} skipped due to Y-Clip bounds", i);
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
        
        if ret.is_err() {
            debug!("ar_get_contour failed for label {}: {:?}", i, ret.unwrap_err());
            continue;
        }

        let ret = check_square(label_info.area[i], &mut current_marker, square_fit_thresh);
        if ret.is_err() {
            debug!("check_square failed for label {}: {:?}", i, ret.unwrap_err());
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
        for i in 0..(*marker2_num as usize) {
            let pm = &mut marker_info2[i];
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
    let mut p_idx = (sy * xsize + clip[0]) as usize;
    for i in clip[0]..=clip[1] {
        if p_idx < limage.len() {
            if limage[p_idx] == label as crate::types::ARLabelingLabelType {
                sx = i;
                break;
            }
        }
        p_idx += 1;
    }
    
    if sx == -1 {
        debug!("ar_get_contour failed. label={}. clip={:?}.", label, clip);
        return Err("Contour start point not found");
    }

    marker_info2.coord_num = 1;
    marker_info2.x_coord[0] = sx;
    marker_info2.y_coord[0] = sy;
    let mut dir = 5;
    
    loop {
        let last_idx = (marker_info2.coord_num - 1) as usize;
        let p_idx = (marker_info2.y_coord[last_idx] * xsize + marker_info2.x_coord[last_idx]) as usize;
        
        dir = (dir + 5) % 8;
        let mut found = false;
        for _ in 0..8 {
            let next_idx = (p_idx as isize + ydir[dir] as isize * xsize as isize + xdir[dir] as isize) as usize;
            if next_idx < limage.len() && limage[next_idx] > 0 {
                found = true;
                break;
            }
            dir = (dir + 1) % 8;
        }
        
        if !found {
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
    
    for i in 0..v1 {
        wx[i] = marker_info2.x_coord[i];
        wy[i] = marker_info2.y_coord[i];
    }
    
    let coord_num = marker_info2.coord_num as usize;
    for i in v1..coord_num {
        marker_info2.x_coord[i - v1] = marker_info2.x_coord[i];
        marker_info2.y_coord[i - v1] = marker_info2.y_coord[i];
    }
    
    let offset = coord_num - v1;
    for i in 0..v1 {
        marker_info2.x_coord[offset + i] = wx[i];
        marker_info2.y_coord[offset + i] = wy[i];
    }
    
    let end_idx = marker_info2.coord_num as usize;
    marker_info2.x_coord[end_idx] = marker_info2.x_coord[0];
    marker_info2.y_coord[end_idx] = marker_info2.y_coord[0];
    marker_info2.coord_num += 1;

    Ok(())
}

fn check_square(area: i32, marker_info2: &mut ARMarkerInfo2, factor: ARdouble) -> Result<(), &'static str> {
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
    
    if get_vertex(&marker_info2.x_coord, &marker_info2.y_coord, 0, v1, thresh, &mut wv1, &mut wvnum1).is_err() {
        return Err("Square check failed");
    }
    if get_vertex(&marker_info2.x_coord, &marker_info2.y_coord, v1, coord_num - 1, thresh, &mut wv2, &mut wvnum2).is_err() {
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
        if get_vertex(&marker_info2.x_coord, &marker_info2.y_coord, 0, v2, thresh, &mut wv1, &mut wvnum1).is_err() {
            return Err("Square check failed");
        }
        if get_vertex(&marker_info2.x_coord, &marker_info2.y_coord, v2, v1, thresh, &mut wv2, &mut wvnum2).is_err() {
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
        if get_vertex(&marker_info2.x_coord, &marker_info2.y_coord, v1, v2, thresh, &mut wv1, &mut wvnum1).is_err() {
            return Err("Square check failed");
        }
        if get_vertex(&marker_info2.x_coord, &marker_info2.y_coord, v2, coord_num - 1, thresh, &mut wv2, &mut wvnum2).is_err() {
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
use crate::types::{ARParamLTf, ARMarkerInfo};

/// Ports arGetLine from arGetLine.c
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
            let (ix, iy) = param_ltf.observ2ideal(x_coord[st + j] as f32, y_coord[st + j] as f32)?;
            input.m[j * 2 + 0] = ix as f64;
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

/// Ports arGetMarkerInfo from arGetMarkerInfo.c
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
    
    for i in 0..marker2_num as usize {
        marker_info[j].area = marker_info2[i].area;
        
        if let Ok((ix, iy)) = param_ltf.observ2ideal(marker_info2[i].pos[0] as f32, marker_info2[i].pos[1] as f32) {
            marker_info[j].pos[0] = ix as f64;
            marker_info[j].pos[1] = iy as f64;
        } else {
            continue;
        }

        if ar_get_line(
            &marker_info2[i].x_coord,
            &marker_info2[i].y_coord,
            marker_info2[i].coord_num as usize,
            &marker_info2[i].vertex,
            param_ltf,
            &mut marker_info[j].line,
            &mut marker_info[j].vertex,
        ).is_err() {
            continue;
        }

        // Branch on detection mode
        let is_matrix_mode = patt_detect_mode == crate::types::AR_MATRIX_CODE_DETECTION
            || patt_detect_mode == crate::types::AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX_CODE_DETECTION;

        if is_matrix_mode {
            // Decode the matrix (barcode) code
            let mut mc_id = -1i32;
            let mut mc_dir = -1i32;
            let mut mc_cf = 0.0f64;
            let mut mc_err = 0i32;
            match crate::matrix::ar_matrix_code_get_id(
                image,
                xsize,
                ysize,
                &marker_info[j].vertex,
                matrix_code_type,
                pixel_format,
                patt_ratio,
                &mut mc_id,
                &mut mc_dir,
                &mut mc_cf,
                &mut mc_err,
            ) {
                Ok(()) => {
                    marker_info[j].id_matrix = mc_id;
                    marker_info[j].dir_matrix = mc_dir;
                    marker_info[j].cf_matrix = mc_cf;
                    marker_info[j].error_corrected = mc_err;
                    debug!("ar_get_marker_info: barcode id={}, dir={}, cf={:.4}", mc_id, mc_dir, mc_cf);
                }
                Err(e) => {
                    debug!("ar_get_marker_info: barcode decode failed: {}", e);
                    marker_info[j].id_matrix = -1;
                    marker_info[j].dir_matrix = -1;
                    marker_info[j].cf_matrix = 0.0;
                }
            }
        }

        if !is_matrix_mode || patt_detect_mode == crate::types::AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX_CODE_DETECTION {
            // Template matching branch
            if let Some(patt_handle) = patt_handle_opt {
                if patt_handle.patt_num > 0 {
                    let patt_size = patt_handle.patt_size;
                    let ext_patt_len = if patt_detect_mode == crate::pattern::AR_TEMPLATE_MATCHING_COLOR {
                        (patt_size * patt_size * 3) as usize
                    } else {
                        (patt_size * patt_size) as usize
                    };
                    let mut ext_patt = vec![0u8; ext_patt_len];
                    
                    let res = crate::pattern::ar_patt_get_image(
                        image_proc_mode as i32,
                        patt_detect_mode,
                        patt_size,
                        patt_size * 2,
                        image,
                        xsize,
                        ysize,
                        pixel_format,
                        &marker_info[j].vertex,
                        patt_ratio,
                        &mut ext_patt,
                    );
                    
                    if res.is_ok() {
                        let mut p_code = -1;
                        let mut p_dir = 0;
                        let mut p_cf = -1.0;
                        let match_res = crate::pattern::pattern_match(
                            patt_handle,
                            patt_detect_mode,
                            &ext_patt,
                            patt_size,
                            &mut p_code,
                            &mut p_dir,
                            &mut p_cf,
                        );
                        
                        if match_res.is_ok() && p_code >= 0 {
                            marker_info[j].id = p_code;
                            marker_info[j].dir = p_dir;
                            marker_info[j].cf = p_cf;
                        } else {
                            marker_info[j].id = -1;
                            marker_info[j].dir = 0;
                            marker_info[j].cf = p_cf;
                        }
                    } else {
                        marker_info[j].id = -1;
                        marker_info[j].dir = 0;
                        marker_info[j].cf = -1.0;
                    }
                } else {
                    marker_info[j].id = -1;
                    marker_info[j].dir = 0;
                    marker_info[j].cf = 0.0;
                }
            } else {
                marker_info[j].id = -1;
                marker_info[j].dir = 0;
                marker_info[j].cf = 0.0;
            }
        }

        j += 1;
    }
    *marker_num = j as i32;

    Ok(())
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
}
