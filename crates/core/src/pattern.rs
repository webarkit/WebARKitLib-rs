/*
 *  pattern.rs
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

//! Pattern Template matching logic
//! Ported natively to safe Rust from arPattLoad.c and arPattGetID.c

use crate::types::*;
use std::fs::File;
use std::io::{BufWriter, Write, Result as IoResult, Error, ErrorKind};
use std::path::Path;



pub const AR_PATT_NUM_MAX: i32 = 50;
pub const AR_PATT_SIZE1: i32 = 16;

// Standard sample factor used in ARToolKit for pattern extraction
const AR_PATT_SAMPLE_FACTOR1: usize = 4;

// Constants for image processing and matching modes.
// Ensure these match your actual definitions in WebARKitLib-rs.
const AR_IMAGE_PROC_FRAME_IMAGE: i32 = 0;
const AR_IMAGE_PROC_FIELD_IMAGE: i32 = 1;
pub const AR_TEMPLATE_MATCHING_COLOR: i32 = 0;
pub const AR_TEMPLATE_MATCHING_MONO: i32 = 1;
pub const AR_PATT_CONTRAST_THRESH1: ARdouble = 5.0;

impl ARPattHandle {
    /// Creates a new Handle for pattern matching with pre-allocated buffer capacities.
    pub fn new(patt_size: i32, pattern_count_max: i32) -> Self {
        let max_alloc = (pattern_count_max * 4) as usize;
        Self {
            patt_num: 0,
            patt_num_max: pattern_count_max,
            pattf: vec![0; pattern_count_max as usize],
            patt: vec![vec![0i16; (patt_size * patt_size * 3) as usize]; max_alloc],
            pattpow: vec![0.0; max_alloc],
            patt_bw: vec![vec![0i16; (patt_size * patt_size) as usize]; max_alloc],
            pattpow_bw: vec![0.0; max_alloc],
            patt_size,
            active: vec![false; pattern_count_max as usize],
        }
    }
}

/// Loads a pattern into the provided ARPattHandle from a text-based buffer.
/// Returns the index of the loaded pattern.
pub fn ar_patt_load_from_buffer(patt_handle: &mut ARPattHandle, buffer: &str) -> Result<i32, &'static str> {
    let mut tokens = buffer.split_whitespace();
    
    let mut patno = -1;
    for i in 0..patt_handle.patt_num_max as usize {
        if patt_handle.pattf[i] == 0 {
            patno = i as i32;
            break;
        }
    }
    
    if patno == -1 {
        return Err("Maximum pattern limit reached");
    }

    let p_idx = patno as usize;
    let size = patt_handle.patt_size as usize;

    for h in 0..4 {
        let mut l = 0;
        let mut i_out = 0;
        
        let mut read_tokens = Vec::with_capacity(size * size * 3);
        
        for _ in 0..(size * size * 3) {
            if let Some(t) = tokens.next() {
                if let Ok(val) = t.parse::<i32>() {
                    read_tokens.push(val);
                } else {
                    return Err("Failed to parse pattern number");
                }
            } else {
                return Err("Pattern data read error (unexpected EOF)");
            }
        }
        
        for i3 in 0..3 {
            for i2 in 0..size {
                for i1 in 0..size {
                    let mut j = read_tokens[i_out];
                    i_out += 1;
                    
                    j = 255 - j;
                    patt_handle.patt[p_idx * 4 + h][(i2 * size + i1) * 3 + i3] = j as i16;
                    
                    if i3 == 0 {
                        patt_handle.patt_bw[p_idx * 4 + h][i2 * size + i1] = j as i16;
                    } else {
                        patt_handle.patt_bw[p_idx * 4 + h][i2 * size + i1] += j as i16;
                    }
                    
                    if i3 == 2 {
                        patt_handle.patt_bw[p_idx * 4 + h][i2 * size + i1] /= 3;
                    }
                    l += j;
                }
            }
        }
        
        l /= (size * size * 3) as i32;

        let mut m_col = 0i64;
        let l16 = l as i16;
        for i in 0..(size * size * 3) {
            patt_handle.patt[p_idx * 4 + h][i] -= l16;
            let val = patt_handle.patt[p_idx * 4 + h][i] as i32;
            m_col += (val * val) as i64;
        }
        
        patt_handle.pattpow[p_idx * 4 + h] = (m_col as ARdouble).sqrt();
        if patt_handle.pattpow[p_idx * 4 + h] == 0.0 {
            patt_handle.pattpow[p_idx * 4 + h] = 0.0000001;
        }

        let mut m_bw = 0i64;
        for i in 0..(size * size) {
            patt_handle.patt_bw[p_idx * 4 + h][i] -= l16;
            let val = patt_handle.patt_bw[p_idx * 4 + h][i] as i32;
            m_bw += (val * val) as i64;
        }
        
        patt_handle.pattpow_bw[p_idx * 4 + h] = (m_bw as ARdouble).sqrt();
        if patt_handle.pattpow_bw[p_idx * 4 + h] == 0.0 {
            patt_handle.pattpow_bw[p_idx * 4 + h] = 0.0000001;
        }
    }

    patt_handle.pattf[p_idx] = 1;
    patt_handle.patt_num += 1;

    Ok(patno)
}

/// Loads a pattern from a specified file path.
pub fn ar_patt_load(patt_handle: &mut ARPattHandle, filename: &str) -> Result<i32, String> {
    use std::fs;
    
    let buffer = fs::read_to_string(filename)
        .map_err(|e| format!("Error reading pattern file '{}': {}", filename, e))?;
        
    ar_patt_load_from_buffer(patt_handle, &buffer).map_err(|e| e.to_string())
}

/// Saves the extracted pattern from an image to a file.
///
/// # Arguments
/// * `image` - Raw image buffer.
/// * `xsize` - Image width.
/// * `ysize` - Image height.
/// * `pixel_format` - Format of the raw image pixels.
/// * `param_ltf` - Camera parameter struct for distortion correction.
/// * `image_proc_mode` - Mode of image processing (Frame or Field).
/// * `marker_info` - Information about the detected marker, including its vertices.
/// * `patt_ratio` - Ratio of the pattern relative to the marker size.
/// * `patt_size` - Size of the pattern grid.
/// * `filename` - Destination path for the saved pattern file.
pub fn ar_patt_save(
    image: &[u8],
    xsize: usize,
    ysize: usize,
    pixel_format: ARPixelFormat,
    param_ltf: &ARParamLTf,
    image_proc_mode: i32,
    marker_info: &ARMarkerInfo,
    patt_ratio: ARdouble,
    patt_size: usize,
    filename: &Path,
) -> IoResult<()> {
    // Allocate 4 buffers for the 4 possible rotations of the pattern
    let mut ext_pat = vec![vec![0u8; patt_size * patt_size * 3]; 4];

    // Extract the pattern for each of the 4 rotations
    for j in 0..4 {
        let mut vertex = [[0.0; 2]; 4];
        for k in 0..4 {
            // Shift the vertices to simulate rotation: (k + j + 2) % 4
            vertex[k][0] = marker_info.vertex[(k + j + 2) % 4][0];
            vertex[k][1] = marker_info.vertex[(k + j + 2) % 4][1];
        }

        // Call the previously generated extraction function.
        // We map the static string error to a standard std::io::Error for consistent error handling.
        ar_patt_get_image2(
            image_proc_mode,
            AR_TEMPLATE_MATCHING_COLOR,
            patt_size,
            patt_size * AR_PATT_SAMPLE_FACTOR1,
            image,
            xsize,
            ysize,
            pixel_format,
            param_ltf,
            &vertex,
            patt_ratio,
            &mut ext_pat[j],
        ).map_err(|err_msg| Error::new(ErrorKind::InvalidData, err_msg))?;
    }

    // Open the file for writing
    let file = File::create(filename)?;
    // Use a BufWriter for efficient batching of small I/O writes
    let mut writer = BufWriter::new(file);

    // Write out in order: 4 orientations -> 3 colours -> rows (Y) -> columns (X)
    for i in 0..4 {
        for j in 0..3 {
            for y in 0..patt_size {
                for x in 0..patt_size {
                    let idx = (y * patt_size + x) * 3 + j;
                    // Format the pixel value to match C's "%4d"
                    write!(writer, "{:4}", ext_pat[i][idx])?;
                }
                writeln!(writer)?; // Newline after each row
            }
        }
        writeln!(writer)?; // Extra newline after each orientation block
    }

    // BufWriter automatically flushes when dropped, but we can explicitly flush to catch potential disk full errors
    writer.flush()?;

    Ok(())
}

/// Activates a pattern for detection.
pub fn ar_patt_activate(patt_handle: &mut ARPattHandle, patno: i32) -> Result<(), &'static str> {
    if patno < 0 || patno as usize >= patt_handle.pattf.len() {
        return Err("Invalid pattern index");
    }
    if patt_handle.pattf[patno as usize] == 0 {
        return Err("Pattern not loaded");
    }
    patt_handle.active[patno as usize] = true;
    Ok(())
}

/// Deactivates a pattern from detection.
pub fn ar_patt_deactivate(patt_handle: &mut ARPattHandle, patno: i32) -> Result<(), &'static str> {
    if patno < 0 || patno as usize >= patt_handle.pattf.len() {
        return Err("Invalid pattern index");
    }
    if patt_handle.pattf[patno as usize] == 0 {
        return Err("Pattern not loaded");
    }
    patt_handle.active[patno as usize] = false;
    Ok(())
}

/// Frees a pattern slot, making it available for a new pattern.
pub fn ar_patt_free(patt_handle: &mut ARPattHandle, patno: i32) -> Result<(), &'static str> {
    if patno < 0 || patno as usize >= patt_handle.pattf.len() {
        return Err("Invalid pattern index");
    }
    if patt_handle.pattf[patno as usize] != 0 {
        patt_handle.pattf[patno as usize] = 0;
        patt_handle.active[patno as usize] = false;
        patt_handle.patt_num -= 1;
    }
    Ok(())
}

/// Returns the number of currently loaded patterns.
pub fn ar_patt_count(patt_handle: &ARPattHandle) -> i32 {
    patt_handle.patt_num
}

/// Deletes the handle and all its resources.
pub fn ar_patt_delete_handle(patt_handle: ARPattHandle) {
    // Rust's RAII will handle vector cleanup when patt_handle is dropped.
    drop(patt_handle);
}

/// Matches the unwarped square marker against loaded templates via NCC (Normalized Cross Correlation).
pub fn pattern_match(
    patt_handle: &ARPattHandle,
    mode: i32,
    data: &[u8],
    size: i32,
    code: &mut i32,
    dir: &mut i32,
    cf: &mut ARdouble,
) -> Result<(), &'static str> {
    if size <= 0 {
        *code = 0;
        *dir = 0;
        *cf = -1.0;
        return Err("Invalid size");
    }

    let size_u = size as usize;

    if mode == AR_TEMPLATE_MATCHING_COLOR {
        let size_sqd_x3 = size_u * size_u * 3;
        if data.len() < size_sqd_x3 {
            return Err("Data array too small");
        }
        let mut input = vec![0i16; size_sqd_x3];

        let mut ave = 0i32;
        for i in 0..size_sqd_x3 {
            ave += (255 - data[i]) as i32;
        }
        ave /= size_sqd_x3 as i32;

        let mut sum = 0i64;
        let ave16 = ave as i16;
        for i in 0..size_sqd_x3 {
            input[i] = ((255 - data[i]) as i16) - ave16;
            let val = input[i] as i32;
            sum += (val * val) as i64;
        }

        let datapow = (sum as ARdouble).sqrt();
        if datapow / ((size as ARdouble) * 3.0f64.sqrt()) < AR_PATT_CONTRAST_THRESH1 {
            *code = 0;
            *dir = 0;
            *cf = -1.0;
            return Err("Insufficient contrast");
        }

        let mut res1 = -1;
        let mut res2 = -1;
        let mut max = 0.0;
        
        let mut k = -1isize;
        for _ in 0..patt_handle.patt_num {
            k += 1;
            while (k as usize) < patt_handle.pattf.len() && patt_handle.pattf[k as usize] == 0 {
                k += 1;
            }
            if k as usize >= patt_handle.pattf.len() { break; }
            if patt_handle.pattf[k as usize] == 2 {
                continue;
            }
            
            for j in 0..4 {
                let pattern_ref = &patt_handle.patt[k as usize * 4 + j];
                let sum_cc = dot_product(&input, pattern_ref);
                
                let sum2 = (sum_cc as ARdouble) / patt_handle.pattpow[k as usize * 4 + j] / datapow;
                if sum2 > max {
                    max = sum2;
                    res1 = j as i32;
                    res2 = k as i32;
                }
            }
        }

        *dir = res1;
        *code = res2;
        *cf = max;
        Ok(())
    } else if mode == AR_TEMPLATE_MATCHING_MONO {
        let size_sqd = size_u * size_u;
        if data.len() < size_sqd {
            return Err("Data array too small");
        }
        
        let mut input = vec![0i16; size_sqd];

        let mut ave = 0i32;
        for i in 0..size_sqd {
            ave += (255 - data[i]) as i32;
        }
        ave /= size_sqd as i32;

        let mut sum = 0i64;
        let ave16 = ave as i16;
        for i in 0..size_sqd {
            input[i] = ((255 - data[i]) as i16) - ave16;
            let val = input[i] as i32;
            sum += (val * val) as i64;
        }

        let datapow = (sum as ARdouble).sqrt();
        if datapow / (size as ARdouble) < AR_PATT_CONTRAST_THRESH1 {
            *code = 0;
            *dir = 0;
            *cf = -1.0;
            return Err("Insufficient contrast");
        }

        let mut res1 = -1;
        let mut res2 = -1;
        let mut max = 0.0;
        
        let mut k = -1isize;
        for _ in 0..patt_handle.patt_num {
            k += 1;
            while (k as usize) < patt_handle.pattf.len() && patt_handle.pattf[k as usize] == 0 {
                k += 1;
            }
            if k as usize >= patt_handle.pattf.len() { break; }
            if patt_handle.pattf[k as usize] == 2 {
                continue;
            }
            
            for j in 0..4 {
                let pattern_ref = &patt_handle.patt_bw[k as usize * 4 + j];
                let sum_cc = dot_product(&input, pattern_ref);
                
                let sum2 = (sum_cc as ARdouble) / patt_handle.pattpow_bw[k as usize * 4 + j] / datapow;
                if sum2 > max {
                    max = sum2;
                    res1 = j as i32;
                    res2 = k as i32;
                }
            }
        }

        *dir = res1;
        *code = res2;
        *cf = max;
        Ok(())
    } else {
        Err("Unsupported matching mode")
    }
}

pub fn get_cpara(world: &[[ARdouble; 2]; 4], vertex: &[[ARdouble; 2]; 4], para: &mut [[ARdouble; 3]; 3]) -> Result<(), &'static str> {
    use crate::math::{ARMat};
    let mut a = ARMat::new(8, 8);
    let mut b = ARMat::new(8, 1);

    for i in 0..4 {
        a.m[i * 16 + 0] = world[i][0];
        a.m[i * 16 + 1] = world[i][1];
        a.m[i * 16 + 2] = 1.0;
        a.m[i * 16 + 3] = 0.0;
        a.m[i * 16 + 4] = 0.0;
        a.m[i * 16 + 5] = 0.0;
        a.m[i * 16 + 6] = -world[i][0] * vertex[i][0];
        a.m[i * 16 + 7] = -world[i][1] * vertex[i][0];
        
        a.m[i * 16 + 8] = 0.0;
        a.m[i * 16 + 9] = 0.0;
        a.m[i * 16 + 10] = 0.0;
        a.m[i * 16 + 11] = world[i][0];
        a.m[i * 16 + 12] = world[i][1];
        a.m[i * 16 + 13] = 1.0;
        a.m[i * 16 + 14] = -world[i][0] * vertex[i][1];
        a.m[i * 16 + 15] = -world[i][1] * vertex[i][1];

        b.m[i * 2 + 0] = vertex[i][0];
        b.m[i * 2 + 1] = vertex[i][1];
    }

    a.self_inv()?;
    let c = (&a * &b)?;

    for i in 0..2 {
        para[i][0] = c.m[i * 3 + 0];
        para[i][1] = c.m[i * 3 + 1];
        para[i][2] = c.m[i * 3 + 2];
    }
    para[2][0] = c.m[6];
    para[2][1] = c.m[7];
    para[2][2] = 1.0;

    Ok(())
}

/// Extracts an image from a warped pattern space without lens distortion correction.
///
/// # Arguments
/// * `image_proc_mode` - Mode of image processing (Frame or Field).
/// * `patt_detect_mode` - Mode for pattern detection (e.g., Color or Mono).
/// * `patt_size` - Size of the pattern grid.
/// * `sample_size` - Maximum sample size for supersampling.
/// * `image` - Raw image buffer.
/// * `xsize` - Image width.
/// * `ysize` - Image height.
/// * `pixel_format` - Format of the raw image pixels.
/// * `x_coord` - Array of X coordinates for contour perimeter pixels.
/// * `y_coord` - Array of Y coordinates for contour perimeter pixels.
/// * `vertex` - Array of 4 indices pointing to the corners in the coord arrays.
/// * `patt_ratio` - Ratio of the pattern relative to the marker size.
/// * `ext_patt` - Output buffer where the extracted pattern will be written.
pub fn ar_patt_get_image(
    image_proc_mode: i32,
    patt_detect_mode: i32,
    patt_size: usize,
    sample_size: usize,
    image: &[u8],
    xsize: usize,
    ysize: usize,
    pixel_format: ARPixelFormat,
    x_coord: &[i32],
    y_coord: &[i32],
    vertex: &[usize; 4],
    patt_ratio: ARdouble,
    ext_patt: &mut [u8],
) -> Result<(), &'static str> {
    let mut world = [[0.0; 2]; 4];
    let mut local = [[0.0; 2]; 4];
    let mut para = [[0.0; 3]; 3];

    // Define the ideal world coordinates of the marker corners
    world[0][0] = 100.0;
    world[0][1] = 100.0;
    world[1][0] = 100.0 + 10.0;
    world[1][1] = 100.0;
    world[2][0] = 100.0 + 10.0;
    world[2][1] = 100.0 + 10.0;
    world[3][0] = 100.0;
    world[3][1] = 100.0 + 10.0;

    // Map local coordinates using the provided vertex indices into the coordinate arrays
    for i in 0..4 {
        local[i][0] = x_coord[vertex[i]] as f64;
        local[i][1] = y_coord[vertex[i]] as f64;
    }

    // Compute the perspective transformation matrix.
    // Assuming get_cpara is defined in scope or imported.
    crate::get_cpara(&world, &local, &mut para);

    // Calculate the square of the lengths of the polygon sides
    let mut lx1 = ((local[0][0] - local[1][0]).powi(2) + (local[0][1] - local[1][1]).powi(2)) as usize;
    let lx2 = ((local[2][0] - local[3][0]).powi(2) + (local[2][1] - local[3][1]).powi(2)) as usize;
    let mut ly1 = ((local[1][0] - local[2][0]).powi(2) + (local[1][1] - local[2][1]).powi(2)) as usize;
    let ly2 = ((local[3][0] - local[0][0]).powi(2) + (local[3][1] - local[0][1]).powi(2)) as usize;

    if lx2 > lx1 {
        lx1 = lx2;
    }
    if ly2 > ly1 {
        ly1 = ly2;
    }

    let lx_patt = (lx1 as ARdouble * patt_ratio * patt_ratio) as usize;
    let ly_patt = (ly1 as ARdouble * patt_ratio * patt_ratio) as usize;

    // Work out how many samples ("divisions") to take of the pattern space.
    let mut xdiv2 = patt_size;
    let mut ydiv2 = patt_size;

    if image_proc_mode == AR_IMAGE_PROC_FRAME_IMAGE {
        while xdiv2 * xdiv2 < lx_patt && xdiv2 < sample_size {
            xdiv2 *= 2;
        }
        while ydiv2 * ydiv2 < ly_patt && ydiv2 < sample_size {
            ydiv2 *= 2;
        }
    } else {
        while xdiv2 * xdiv2 * 4 < lx_patt && xdiv2 < sample_size {
            xdiv2 *= 2;
        }
        while ydiv2 * ydiv2 * 4 < ly_patt && ydiv2 < sample_size {
            ydiv2 *= 2;
        }
    }

    if xdiv2 > sample_size {
        xdiv2 = sample_size;
    }
    if ydiv2 > sample_size {
        ydiv2 = sample_size;
    }

    let xdiv = xdiv2 / patt_size;
    let ydiv = ydiv2 / patt_size;
    let patt_ratio1 = (1.0 - patt_ratio) / 2.0 * 10.0;
    let patt_ratio2 = patt_ratio * 10.0;

    let is_color_matching = patt_detect_mode == AR_TEMPLATE_MATCHING_COLOR;
    let channels = if is_color_matching { 3 } else { 1 };

    // Allocate temporary accumulation buffer
    let mut ext_patt2 = vec![0u32; patt_size * patt_size * channels];

    for j in 0..ydiv2 {
        let yw = (100.0 + patt_ratio1) + patt_ratio2 * (j as ARdouble + 0.5) / ydiv2 as ARdouble;
        for i in 0..xdiv2 {
            let xw = (100.0 + patt_ratio1) + patt_ratio2 * (i as ARdouble + 0.5) / xdiv2 as ARdouble;
            let d = para[2][0] * xw + para[2][1] * yw + para[2][2];

            if d == 0.0 {
                return Err("Matrix denominator is zero");
            }

            // Raw perspective coordinates (no ideal2observ lens distortion step here)
            let xc2 = (para[0][0] * xw + para[0][1] * yw + para[0][2]) / d;
            let yc2 = (para[1][0] * xw + para[1][1] * yw + para[1][2]) / d;

            let (xc, yc) = if image_proc_mode == AR_IMAGE_PROC_FIELD_IMAGE {
                ((((xc2 as i32) + 1) / 2) * 2, (((yc2 as i32) + 1) / 2) * 2)
            } else {
                (xc2 as i32, yc2 as i32)
            };

            // Ensure the pixel is within the image bounds
            if xc >= 0 && xc < xsize as i32 && yc >= 0 && yc < ysize as i32 {
                let xc = xc as usize;
                let yc = yc as usize;
                let idx_patt = (j / ydiv) * patt_size + (i / xdiv);

                if is_color_matching {
                    let dest_idx = idx_patt * 3;
                    match pixel_format {
                        ARPixelFormat::RGB => {
                            let src_idx = (yc * xsize + xc) * 3;
                            ext_patt2[dest_idx] += image[src_idx + 2] as u32;
                            ext_patt2[dest_idx + 1] += image[src_idx + 1] as u32;
                            ext_patt2[dest_idx + 2] += image[src_idx] as u32;
                        }
                        ARPixelFormat::RGBA => {
                            let src_idx = (yc * xsize + xc) * 4;
                            ext_patt2[dest_idx] += image[src_idx + 2] as u32;
                            ext_patt2[dest_idx + 1] += image[src_idx + 1] as u32;
                            ext_patt2[dest_idx + 2] += image[src_idx] as u32;
                        }
                        ARPixelFormat::MONO | ARPixelFormat::FourTwoZeroV | ARPixelFormat::FourTwoZeroF | ARPixelFormat::NV21 => {
                            let src_idx = yc * xsize + xc;
                            let val = image[src_idx] as u32;
                            ext_patt2[dest_idx] += val;
                            ext_patt2[dest_idx + 1] += val;
                            ext_patt2[dest_idx + 2] += val;
                        }
                        // Note: BGR, BGRA, 2vuy, yuvs, rgb565, etc. can be expanded here.
                        _ => return Err("Unsupported pixel format for color matching"),
                    }
                } else {
                    match pixel_format {
                        ARPixelFormat::RGB | ARPixelFormat::BGR => {
                            let src_idx = (yc * xsize + xc) * 3;
                            let val = (image[src_idx] as u32 + image[src_idx + 1] as u32 + image[src_idx + 2] as u32) / 3;
                            ext_patt2[idx_patt] += val;
                        }
                        ARPixelFormat::RGBA | ARPixelFormat::BGRA => {
                            let src_idx = (yc * xsize + xc) * 4;
                            let val = (image[src_idx] as u32 + image[src_idx + 1] as u32 + image[src_idx + 2] as u32) / 3;
                            ext_patt2[idx_patt] += val;
                        }
                        ARPixelFormat::MONO | ARPixelFormat::FourTwoZeroV | ARPixelFormat::FourTwoZeroF | ARPixelFormat::NV21 => {
                            let src_idx = yc * xsize + xc;
                            ext_patt2[idx_patt] += image[src_idx] as u32;
                        }
                        // Note: Other specific mono format decodings can be added here.
                        _ => return Err("Unsupported pixel format for mono matching"),
                    }
                }
            }
        }
    }

    // Average the accumulated samples and write back to the 8-bit output buffer
    let total_div = (xdiv * ydiv) as u32;
    for (i, val) in ext_patt2.iter().enumerate() {
        ext_patt[i] = (val / total_div) as u8;
    }

    Ok(())
}

/// Extracts an image from a warped pattern space.
///
/// # Arguments
/// * `image_proc_mode` - Mode of image processing (Frame or Field).
/// * `patt_detect_mode` - Mode for pattern detection (e.g., Color or Mono).
/// * `patt_size` - Size of the pattern grid.
/// * `sample_size` - Maximum sample size for supersampling.
/// * `image` - Raw image buffer.
/// * `xsize` - Image width.
/// * `ysize` - Image height.
/// * `pixel_format` - Format of the raw image pixels.
/// * `param_ltf` - Camera parameter struct for distortion correction.
/// * `vertex` - 4 corners of the detected marker.
/// * `patt_ratio` - Ratio of the pattern relative to the marker size.
/// * `ext_patt` - Output buffer where the extracted pattern will be written.
pub fn ar_patt_get_image2(
    image_proc_mode: i32,
    patt_detect_mode: i32,
    patt_size: usize,
    sample_size: usize,
    image: &[u8],
    xsize: usize,
    ysize: usize,
    pixel_format: ARPixelFormat,
    param_ltf: &ARParamLTf,
    vertex: &[[ARdouble; 2]; 4],
    patt_ratio: ARdouble,
    ext_patt: &mut [u8],
) -> Result<(), &'static str> {
    let mut world = [[0.0; 2]; 4];
    let mut local = [[0.0; 2]; 4];
    let mut para = [[0.0; 3]; 3];

    // Define the ideal world coordinates of the marker corners
    world[0][0] = 100.0;
    world[0][1] = 100.0;
    world[1][0] = 100.0 + 10.0;
    world[1][1] = 100.0;
    world[2][0] = 100.0 + 10.0;
    world[2][1] = 100.0 + 10.0;
    world[3][0] = 100.0;
    world[3][1] = 100.0 + 10.0;

    for i in 0..4 {
        local[i][0] = vertex[i][0];
        local[i][1] = vertex[i][1];
    }

    // Compute the perspective transformation matrix.
    get_cpara(&world, &local, &mut para)?;

    // The square roots of lx1, lx2, ly1, and ly2 are the lengths of the sides of the polygon.
    let mut lx1 = ((local[0][0] - local[1][0]).powi(2) + (local[0][1] - local[1][1]).powi(2)) as usize;
    let lx2 = ((local[2][0] - local[3][0]).powi(2) + (local[2][1] - local[3][1]).powi(2)) as usize;
    let mut ly1 = ((local[1][0] - local[2][0]).powi(2) + (local[1][1] - local[2][1]).powi(2)) as usize;
    let ly2 = ((local[3][0] - local[0][0]).powi(2) + (local[3][1] - local[0][1]).powi(2)) as usize;

    // Take the longest two adjacent sides, and calculate the square of the length in pattern space.
    if lx2 > lx1 {
        lx1 = lx2;
    }
    if ly2 > ly1 {
        ly1 = ly2;
    }
    let lx_patt = (lx1 as ARdouble * patt_ratio * patt_ratio) as usize;
    let ly_patt = (ly1 as ARdouble * patt_ratio * patt_ratio) as usize;

    // Work out how many samples ("divisions") to take of the pattern space.
    let mut xdiv2 = patt_size;
    let mut ydiv2 = patt_size;

    if image_proc_mode == AR_IMAGE_PROC_FRAME_IMAGE {
        while xdiv2 * xdiv2 < lx_patt && xdiv2 < sample_size {
            xdiv2 *= 2;
        }
        while ydiv2 * ydiv2 < ly_patt && ydiv2 < sample_size {
            ydiv2 *= 2;
        }
    } else {
        while xdiv2 * xdiv2 * 4 < lx_patt && xdiv2 < sample_size {
            xdiv2 *= 2;
        }
        while ydiv2 * ydiv2 * 4 < ly_patt && ydiv2 < sample_size {
            ydiv2 *= 2;
        }
    }

    if xdiv2 > sample_size {
        xdiv2 = sample_size;
    }
    if ydiv2 > sample_size {
        ydiv2 = sample_size;
    }

    let xdiv = xdiv2 / patt_size;
    let ydiv = ydiv2 / patt_size;
    let patt_ratio1 = (1.0 - patt_ratio) / 2.0 * 10.0; // borderSize * 10.0
    let patt_ratio2 = patt_ratio * 10.0;

    let is_color_matching = patt_detect_mode == AR_TEMPLATE_MATCHING_COLOR;
    let channels = if is_color_matching { 3 } else { 1 };

    // Allocate temporary accumulation buffer
    let mut ext_patt2 = vec![0u32; patt_size * patt_size * channels];

    for j in 0..ydiv2 {
        let yw = (100.0 + patt_ratio1) + patt_ratio2 * (j as ARdouble + 0.5) / ydiv2 as ARdouble;
        for i in 0..xdiv2 {
            let xw = (100.0 + patt_ratio1) + patt_ratio2 * (i as ARdouble + 0.5) / xdiv2 as ARdouble;
            let d = para[2][0] * xw + para[2][1] * yw + para[2][2];

            if d == 0.0 {
                return Err("Matrix denominator is zero");
            }

            let xc2_ideal = ((para[0][0] * xw + para[0][1] * yw + para[0][2]) / d) as f32;
            let yc2_ideal = ((para[1][0] * xw + para[1][1] * yw + para[1][2]) / d) as f32;

            // Apply lens distortion correction using the lookup table.
            // If the coordinates are out of bounds, we skip this sample.
            let (xc2, yc2) = match param_ltf.ideal2observ(xc2_ideal, yc2_ideal) {
                Ok(coords) => coords,
                Err(_) => continue,
            };

            // Calculate final pixel coordinates based on field/frame processing mode.
            let (xc, yc) = if image_proc_mode == AR_IMAGE_PROC_FIELD_IMAGE {
                ((((xc2 + 1.0) as i32) / 2) * 2, (((yc2 + 1.0) as i32) / 2) * 2)
            } else {
                ((xc2 + 0.5) as i32, (yc2 + 0.5) as i32)
            };

            // Ensure the pixel is within the image bounds before sampling.
            if xc >= 0 && xc < xsize as i32 && yc >= 0 && yc < ysize as i32 {
                let xc = xc as usize;
                let yc = yc as usize;
                let idx_patt = (j / ydiv) * patt_size + (i / xdiv);

                if is_color_matching {
                    let dest_idx = idx_patt * 3;
                    match pixel_format {
                        ARPixelFormat::RGB => {
                            let src_idx = (yc * xsize + xc) * 3;
                            ext_patt2[dest_idx] += image[src_idx + 2] as u32;
                            ext_patt2[dest_idx + 1] += image[src_idx + 1] as u32;
                            ext_patt2[dest_idx + 2] += image[src_idx] as u32;
                        }
                        ARPixelFormat::RGBA => {
                            let src_idx = (yc * xsize + xc) * 4;
                            ext_patt2[dest_idx] += image[src_idx + 2] as u32;
                            ext_patt2[dest_idx + 1] += image[src_idx + 1] as u32;
                            ext_patt2[dest_idx + 2] += image[src_idx] as u32;
                        }
                        ARPixelFormat::MONO | ARPixelFormat::FourTwoZeroV | ARPixelFormat::FourTwoZeroF | ARPixelFormat::NV21 => {
                            let src_idx = yc * xsize + xc;
                            let val = image[src_idx] as u32;
                            ext_patt2[dest_idx] += val;
                            ext_patt2[dest_idx + 1] += val;
                            ext_patt2[dest_idx + 2] += val;
                        }
                        // Note: Add other specific color format decodings (e.g., RGB_565, YUVS) here as needed.
                        _ => return Err("Unsupported pixel format for color matching"),
                    }
                } else {
                    match pixel_format {
                        ARPixelFormat::RGB | ARPixelFormat::BGR => {
                            let src_idx = (yc * xsize + xc) * 3;
                            let val = (image[src_idx] as u32 + image[src_idx + 1] as u32 + image[src_idx + 2] as u32) / 3;
                            ext_patt2[idx_patt] += val;
                        }
                        ARPixelFormat::RGBA | ARPixelFormat::BGRA => {
                            let src_idx = (yc * xsize + xc) * 4;
                            let val = (image[src_idx] as u32 + image[src_idx + 1] as u32 + image[src_idx + 2] as u32) / 3;
                            ext_patt2[idx_patt] += val;
                        }
                        ARPixelFormat::MONO | ARPixelFormat::FourTwoZeroV | ARPixelFormat::FourTwoZeroF | ARPixelFormat::NV21 => {
                            let src_idx = yc * xsize + xc;
                            ext_patt2[idx_patt] += image[src_idx] as u32;
                        }
                        // Note: Add other specific mono format decodings here as needed.
                        _ => return Err("Unsupported pixel format for mono matching"),
                    }
                }
            }
        }
    }

    // Average the accumulated samples and write back to the 8-bit output buffer
    let total_div = (xdiv * ydiv) as u32;
    for (i, val) in ext_patt2.iter().enumerate() {
        ext_patt[i] = (val / total_div) as u8;
    }

    Ok(())
}

pub fn ar_patt_get_id(
    patt_handle: &ARPattHandle,
    patt_detect_mode: i32,
    data: &[u8], // Data from the normalized region of interest (square)
    patt_size: i32,
    code: &mut i32,
    dir: &mut i32,
    cf: &mut f64,
) -> Result<(), &'static str> {
    pattern_match(patt_handle, patt_detect_mode, data, patt_size, code, dir, cf)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pattern_load_and_match() {
        let mut handle = ARPattHandle::new(AR_PATT_SIZE1, AR_PATT_NUM_MAX);
        
        let mut mock_patt = String::new();
        // 4 orientations * size * size * 3 colors
        for _ in 0..(4 * AR_PATT_SIZE1 * AR_PATT_SIZE1 * 3) {
            mock_patt.push_str("128 ");
        }
        
        let patno = ar_patt_load_from_buffer(&mut handle, &mock_patt).unwrap();
        assert_eq!(patno, 0);
        assert_eq!(handle.patt_num, 1);
        
        // Mock extracted pattern with high contrast
        let mut mock_data = vec![0; (AR_PATT_SIZE1 * AR_PATT_SIZE1 * 3) as usize];
        for i in 0..mock_data.len() {
            if i % 2 == 0 {
                mock_data[i] = 10;
            } else {
                mock_data[i] = 240;
            }
        }
        
        let mut code = 0;
        let mut dir = 0;
        let mut cf = 0.0;
        let result = pattern_match(&handle, AR_TEMPLATE_MATCHING_COLOR, &mock_data, AR_PATT_SIZE1, &mut code, &mut dir, &mut cf);
        assert!(result.is_ok());
    }

    #[test]
    fn test_ar_patt_save_and_load() {
        use std::fs;
        use image::ImageReader;
        let filename = "test_pattern_save.patt";
        let patt_size_i32: i32 = 16;
        let patt_size: usize = 16;

        // Load the image from examples/Data relative to this crate (CARGO_MANIFEST_DIR)
        let manifest_dir = env!("CARGO_MANIFEST_DIR");
        let img_path = std::path::Path::new(manifest_dir)
            .join("examples")
            .join("Data")
            .join("HIRO-test.jpg");
        let img = ImageReader::open(img_path).unwrap().decode().unwrap();
        // Use actual image dimensions to avoid out-of-bounds when sampling
        let xsize_i32 = img.width() as i32;
        let ysize_i32 = img.height() as i32;
        let xsize = img.width() as usize;
        let ysize = img.height() as usize;
        let image_buf = img.to_rgb8();
        // consume image buffer to obtain owned Vec<u8> for API which expects &[u8]
        let image = image_buf.into_raw();

        // Define a simple square quad (in image coordinates)
        let vertex = [
            [100.0, 100.0],
            [200.0, 100.0],
            [200.0, 200.0],
            [100.0, 200.0],
        ];

        // Build a simple ARParamLTf lookup table (identity mapping) to satisfy ar_patt_get_image2
        let param_ltf = ARParamLTf::new_basic(xsize_i32, ysize_i32);

        // Build a simple marker_info with the quad we defined
        let mut marker = ARMarkerInfo::default();
        marker.vertex = vertex;

        // Save using the image-first signature of ar_patt_save
        let filename_path = std::path::Path::new(filename);
        let res = ar_patt_save(
            &image,
            xsize,
            ysize,
            ARPixelFormat::RGB,
            &param_ltf,
            AR_IMAGE_PROC_FRAME_IMAGE,
            &marker,
            0.5, // patt_ratio
            patt_size,
            filename_path,
        );

        assert!(res.is_ok(), "ar_patt_save failed: {:?}", res.err());
        assert!(fs::metadata(filename).is_ok(), "Pattern file was not created");

        // Test loading it back using the buffer version since load_from_file is not available
        let content = fs::read_to_string(filename).expect("Failed to read saved pattern file");
        let mut handle = ARPattHandle::new(patt_size_i32, 1);
        let load_res = ar_patt_load_from_buffer(&mut handle, &content);

        assert!(load_res.is_ok(), "ar_patt_load_from_buffer failed: {:?}", load_res.err());

        // Compare with crates/core/examples/Data/patt.hiro (left commented for now)
        let _reference_content = fs::read_to_string("../core/examples/Data/patt.hiro").expect("Failed to read reference pattern file");
        // assert_eq!(content, _reference_content, "Saved pattern does not match the reference pattern");

        // Cleanup
        //let _ = fs::remove_file(filename);
    }
}

#[inline]
fn dot_product(a: &[i16], b: &[i16]) -> i64 {
    #[cfg(feature = "simd-pattern")]
    {
        #[cfg(all(target_arch = "wasm32", target_feature = "simd128"))]
        {
            return unsafe { dot_product_simd_wasm(a, b) };
        }
        #[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
        {
            if is_x86_feature_detected!("sse4.1") {
                return unsafe { dot_product_simd_x86(a, b) };
            }
        }
    }

    dot_product_scalar(a, b)
}

pub fn dot_product_scalar(a: &[i16], b: &[i16]) -> i64 {
    let mut sum = 0i64;
    for i in 0..a.len() {
        sum += (a[i] as i32 * b[i] as i32) as i64;
    }
    sum
}

#[cfg(all(feature = "simd-pattern", target_arch = "wasm32", target_feature = "simd128"))]
#[target_feature(enable = "simd128")]
unsafe fn dot_product_simd_wasm(a: &[i16], b: &[i16]) -> i64 {
    use std::arch::wasm32::*;
    
    let mut sum_v = i32x4_splat(0);
    let chunks_len = a.len() / 8;
    
    let mut a_ptr = a.as_ptr();
    let mut b_ptr = b.as_ptr();
    
    for _ in 0..chunks_len {
        let va = v128_load(a_ptr as *const v128);
        let vb = v128_load(b_ptr as *const v128);
        
        // i32x4_dot_i16x8 computes (a0*b0 + a1*b1), (a2*b2 + a3*b3), ...
        let dot = i32x4_dot_i16x8(va, vb);
        sum_v = i32x4_add(sum_v, dot);
        
        a_ptr = a_ptr.add(8);
        b_ptr = b_ptr.add(8);
    }
    
    // Sum the 4 horizontal parts into i64x2
    let low = i64x2_extend_low_i32x4(sum_v);
    let high = i64x2_extend_high_i32x4(sum_v);
    let sum64 = i64x2_add(low, high);

    let mut res = [0i64; 2];
    v128_store(res.as_mut_ptr() as *mut v128, sum64);
    let mut total = res[0] + res[1];
    
    let rem_start = chunks_len * 8;
    for i in rem_start..a.len() {
        total += (a[i] as i32 * b[i] as i32) as i64;
    }
    
    total
}

/// Calculates the dot product of two i16 slices using SSE4.1 intrinsics.
///
/// Processes 8 elements at a time using `_mm_madd_epi16`.
///
/// # Safety
/// This function is unsafe because it uses SIMD intrinsics and raw pointer arithmetic.
/// The caller must ensure that the target CPU supports SSE4.1.
#[cfg(all(feature = "simd-pattern", target_arch = "x86_64", target_feature = "sse4.1"))]
#[target_feature(enable = "sse4.1")]
pub unsafe fn dot_product_simd_x86(a: &[i16], b: &[i16]) -> i64 {
    use std::arch::x86_64::*;
    
    let mut sum_v = _mm_setzero_si128(); // i32x4
    let chunks_len = a.len() / 8;
    
    let mut a_ptr = a.as_ptr();
    let mut b_ptr = b.as_ptr();
    
    for _ in 0..chunks_len {
        let va = _mm_loadu_si128(a_ptr as *const __m128i);
        let vb = _mm_loadu_si128(b_ptr as *const __m128i);
        
        // Multiply and add adjacent pairs.
        let m = _mm_madd_epi16(va, vb);
        sum_v = _mm_add_epi32(sum_v, m);
        
        a_ptr = a_ptr.add(8);
        b_ptr = b_ptr.add(8);
    }
    
    let mut res = [0i32; 4];
    _mm_storeu_si128(res.as_mut_ptr() as *mut __m128i, sum_v);
    let mut total = (res[0] as i64) + (res[1] as i64) + (res[2] as i64) + (res[3] as i64);
    
    let rem_start = chunks_len * 8;
    for i in rem_start..a.len() {
        total += (a[i] as i32 * b[i] as i32) as i64;
    }
    
    total
}
