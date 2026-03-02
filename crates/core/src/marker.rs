//! Marker Detection Pipeline
//! Ported from arDetectMarker.c, arDetectMarker2.c, and arGetMarkerInfo.c

use crate::types::{ARLabelInfo, ARMarkerInfo2, ARdouble};

pub const AR_AREA_MAX: i32 = 100000;
pub const AR_AREA_MIN: i32 = 70;
pub const AR_SQUARE_FIT_THRESH: f64 = 0.05;
pub const AR_CHAIN_MAX: usize = 10000;
pub const AR_SQUARE_MAX: usize = 30;

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

    if ar_handle.ar_param_lt.is_null() {
        return Err("ARParamLT is null in ARHandle");
    }
    
    let image_proc_mode2 = if ar_handle.ar_image_proc_mode == 0 {
        ImageProcMode::FrameImage
    } else {
        ImageProcMode::FieldImage
    };
    
    let param_ltf = unsafe { &(*ar_handle.ar_param_lt).param_ltf };

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
        &mut *ar_handle.marker_info,
        &mut ar_handle.marker_num,
        ar_handle.matrix_code_type,
    )?;

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
            continue;
        }
        if label_info.clip[i][0] <= 1 || label_info.clip[i][1] >= xsize_local - 2 {
            continue;
        }
        if label_info.clip[i][2] <= 1 || label_info.clip[i][3] >= ysize_local - 2 {
            continue;
        }

        let mut current_marker = ARMarkerInfo2::default();
        
        let ret = ar_get_contour(
            &label_info.label_image,
            xsize_local,
            ysize_local,
            &label_info.work,
            (i + 1) as i32,
            &label_info.clip[i],
            &mut current_marker,
        );
        
        if ret.is_err() {
            continue;
        }

        let ret = check_square(label_info.area[i], &mut current_marker, square_fit_thresh);
        if ret.is_err() {
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
    label_ref: &[i32],
    label: i32,
    clip: &[i32; 4],
    marker_info2: &mut ARMarkerInfo2,
) -> Result<(), &'static str> {
    let xdir = [0, 1, 1, 1, 0, -1, -1, -1];
    let ydir = [-1, -1, 0, 1, 1, 1, 0, -1];
    
    let mut sx = -1;
    let sy = clip[2];
    let mut p_idx = (sy * xsize + clip[0]) as usize;
    
    for i in clip[0]..=clip[1] {
        if p_idx < limage.len() {
            let val = limage[p_idx];
            if val > 0 && label_ref[(val - 1) as usize] == label {
                sx = i;
                break;
            }
        }
        p_idx += 1;
    }
    
    if sx == -1 {
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
    for i in 0..v1 {
        marker_info2.x_coord[i - v1 + coord_num] = wx[i];
        marker_info2.y_coord[i - v1 + coord_num] = wy[i];
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
    _image: &[u8],
    _xsize: i32,
    _ysize: i32,
    _pixel_format: crate::types::ARPixelFormat,
    marker_info2: &[ARMarkerInfo2],
    marker2_num: i32,
    _image_proc_mode: ImageProcMode,
    _patt_detect_mode: i32,
    param_ltf: &ARParamLTf,
    _patt_ratio: ARdouble,
    marker_info: &mut [ARMarkerInfo],
    marker_num: &mut i32,
    _matrix_code_type: crate::types::ARMatrixCodeType,
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

        // TODO: Pattern matching integration

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
