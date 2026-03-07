/*
 *  matrix.rs
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
 *  Copyright 2026 WebARKit.
 *
 *  Author(s): Walter Perdan @kalwalt https://github.com/kalwalt
 *
 */

//! Matrix Code (Barcode) Marker Decoding
//! Ported from arGetMatrixCode.c and associated ECC logic.

use crate::types::{ARMatrixCodeType, ARdouble, ARHandle, ARMarkerInfo, ARMarkerInfo2};
use crate::marker::ar_get_line;
use log::{debug, trace};

/// Results of a matrix code decoding attempt.
#[derive(Debug, Clone, PartialEq)]
pub struct MatrixCodeResult {
    pub id: i32,
    pub dir: i32,
    pub cf: f64,
    pub error_corrected: i32,
}

/// Decodes a matrix (barcode) marker code from the raw frame.
///
/// This is the main entry point for barcode marker decoding. Given the four
/// corner vertices of a detected square candidate, it:
/// 1. Calls `sample_grid` to project `grid_size × grid_size` cells from the
///    image onto a regular grid using a homography (`get_cpara`).
/// 2. Computes the adaptive binarisation threshold from the sampled min/max
///    intensity. Returns `Err("Low contrast…")` if the range < 30.
/// 3. Scans the four corners of the inner `dim × dim` core to detect the
///    L-shaped locator pattern (two adjacent `1`-bits and one `0`-bit), which
///    determines the orientation (`matched_dir ∈ 0..3`).
/// 4. Reads the data bits from the core in the correct rotation order and
///    passes them to `decode_matrix_raw`.
///
/// # Parameters
/// - `vertex` — four `[x, y]` corner coordinates in ideal (undistorted) space,
///   as produced by [`crate::marker::ar_get_line`].
/// - `code_type` — selects the square dimension (`3..=6`) and ECC scheme.
/// - `id` / `dir` / `cf` / `error_corrected` — output values.
///
/// # Returns
/// `Ok(())` on successful decode. `Err` on low contrast, missing locator pattern,
/// or unsupported `code_type`.
///
/// # Example
/// ```rust,no_run
/// use webarkitlib_rs::matrix::ar_matrix_code_get_id;
/// use webarkitlib_rs::types::ARMatrixCodeType;
///
/// let image = vec![0u8; 640 * 480 * 3];
/// let vertex = [[100.0, 100.0], [200.0, 100.0], [200.0, 200.0], [100.0, 200.0]];
/// let mut id = -1i32;
/// let mut dir = -1i32;
/// let mut cf = 0.0f64;
/// let mut err = 0i32;
/// if ar_matrix_code_get_id(&image, 640, 480, &vertex, ARMatrixCodeType::default(),
///         webarkitlib_rs::types::ARPixelFormat::RGB, 0.5,
///         &mut id, &mut dir, &mut cf, &mut err).is_ok() {
///     println!("Decoded barcode id={}, dir={}, cf={:.2}", id, dir, cf);
/// }
/// ```
pub fn ar_matrix_code_get_id(
    image: &[u8],
    xsize: i32,
    ysize: i32,
    vertex: &[[ARdouble; 2]; 4],
    code_type: ARMatrixCodeType,
    pixel_format: crate::types::ARPixelFormat,
    patt_ratio: f64,
    id: &mut i32,
    dir: &mut i32,
    cf: &mut f64,
    error_corrected: &mut i32,
) -> Result<(), &'static str> {
    let dim = (code_type as i32) & 0xFF;
    if dim < 3 || dim > 6 {
        return Err("Unsupported matrix dimension");
    }

    let grid_size = dim;
    let mut bits = vec![0u8; (grid_size * grid_size) as usize];

    sample_grid(image, xsize, ysize, vertex, grid_size, pixel_format, patt_ratio, &mut bits)?;
    
    debug!("ar_matrix_code_get_id: vertices v0=({:.1},{:.1}) v1=({:.1},{:.1}) v2=({:.1},{:.1}) v3=({:.1},{:.1})",
        vertex[0][0], vertex[0][1], vertex[1][0], vertex[1][1],
        vertex[2][0], vertex[2][1], vertex[3][0], vertex[3][1]);


    let mut min_val = 255u8;
    let mut max_val = 0u8;
    for &b in &bits {
        if b < min_val { min_val = b; }
        if b > max_val { max_val = b; }
    }
    
    let mid_thresh = ((min_val as u32 + max_val as u32) / 2) as u8;
    debug!("ar_matrix_code_get_id: min={}, max={}, thresh={}", min_val, max_val, mid_thresh);

    if (max_val as i32 - min_val as i32) < 30 {
        return Err("Low contrast in matrix grid");
    }

    let mut core_bits = vec![0u8; (grid_size * grid_size) as usize];
    for i in 0..bits.len() {
        core_bits[i] = if bits[i] < mid_thresh { 1 } else { 0 };
    }
    
    trace!("ar_matrix_code_get_id: grid_size={}, core_bits={:?}", grid_size, core_bits);
    debug!("ar_matrix_code_get_id: core_bits={:?}", core_bits);

    // The C reference `get_matrix_code` receives only the dim×dim inner data.
    // Check orientation based on corners of the core data:
    // An unrotated pattern has:
    //   top-left (0,0) = 1, bottom-left (0, dim-1) = 1
    //   bottom-right (dim-1, dim-1) = 0.
    let size = dim;
    let corners = [
        0,                          // top-left
        (size - 1) * size,          // bottom-left
        size * size - 1,            // bottom-right
        size - 1,                   // top-right
    ];
    
    let mut dir_code = [0u8; 4];
    for i in 0..4 {
        dir_code[i] = core_bits[corners[i] as usize];
    }
    
    let mut matched_dir = -1;
    for i in 0..4 {
        if dir_code[i] == 1 && dir_code[(i+1)%4] == 1 && dir_code[(i+2)%4] == 0 {
            matched_dir = i as i32;
            break;
        }
    }
    
    if matched_dir == -1 {
        return Err("Barcode locator pattern not found in grid corners");
    }
    
    // Extract code from the dim×dim core bits.
    // The 3 pixels forming the locator corners are ignored.
    let mut code_raw = 0u64;
    
    if matched_dir == 0 {
        for j in 0..size {
            for i in 0..size {
                if i == 0 && j == 0 { continue; }
                if i == 0 && j == size - 1 { continue; }
                if i == size - 1 && j == size - 1 { continue; }
                code_raw <<= 1;
                if core_bits[(j * size + i) as usize] == 1 { code_raw += 1; }
            }
        }
    } else if matched_dir == 1 {
        for i in 0..size {
            for j in (0..size).rev() {
                if i == 0 && j == size - 1 { continue; }
                if i == size - 1 && j == size - 1 { continue; }
                if i == size - 1 && j == 0 { continue; }
                code_raw <<= 1;
                if core_bits[(j * size + i) as usize] == 1 { code_raw += 1; }
            }
        }
    } else if matched_dir == 2 {
        for j in (0..size).rev() {
            for i in (0..size).rev() {
                if i == size - 1 && j == size - 1 { continue; }
                if i == size - 1 && j == 0 { continue; }
                if i == 0 && j == 0 { continue; }
                code_raw <<= 1;
                if core_bits[(j * size + i) as usize] == 1 { code_raw += 1; }
            }
        }
    } else if matched_dir == 3 {
        for i in (0..size).rev() {
            for j in 0..size {
                if i == size - 1 && j == 0 { continue; }
                if i == 0 && j == 0 { continue; }
                if i == 0 && j == size - 1 { continue; }
                code_raw <<= 1;
                if core_bits[(j * size + i) as usize] == 1 { code_raw += 1; }
            }
        }
    }
    
    debug!("ar_matrix_code_get_id: code_raw={}, dir={}", code_raw, matched_dir);
    
    *dir = matched_dir;
    *cf = 1.0; 

    // Handle decoding logic mapped from pattern types!
    if let Ok((decoded_id, err)) = decode_matrix_raw(code_raw, code_type) {
        *id = decoded_id;
        *error_corrected = err;
        Ok(())
    } else {
        Err("Failed to decode extracted matrix code string")
    }
}


/// Higher-level helper: trace contour lines then decode a barcode marker.
///
/// Alternative path for barcode detection not yet wired into [`crate::marker::ar_get_marker_info`].
/// Given a raw `ARMarkerInfo2` candidate, this function:
/// 1. Calls [`crate::marker::ar_get_line`] via the `ARHandle`'s lens-distortion
///    table to undistort the square edge lines and compute corner vertices.
/// 2. Calls [`ar_matrix_code_get_id`] to decode the barcode.
/// 3. Builds and returns a populated [`crate::types::ARMarkerInfo`] struct.
///
/// Returns `Err` if `ar_handle.ar_param_lt` is null, if line fitting fails,
/// or if the barcode decode fails.
pub fn ar_get_barcode_marker(
    image: &[u8],
    ar_handle: &mut ARHandle, 
    marker_info2: &mut ARMarkerInfo2
) -> Result<ARMarkerInfo, &'static str> {
    trace!("ar_get_barcode_marker: started for area={}, coord_num={}", marker_info2.area, marker_info2.coord_num);
    
    let mut marker_info = ARMarkerInfo::default();
    let mut id = -1;
    let mut dir = -1;
    let mut cf = 0.0;
    let mut error_corrected = 0;

    // Resolve corner coordinates from indices
    let mut resolved_vertex = [[0.0; 2]; 4];
    let mut line = [[0.0; 3]; 4];

    if ar_handle.ar_param_lt.is_null() {
        return Err("arParamLT is null");
    }
    let param_lt = unsafe { &*ar_handle.ar_param_lt };

    ar_get_line(
        &marker_info2.x_coord,
        &marker_info2.y_coord,
        marker_info2.coord_num as usize,
        &marker_info2.vertex,
        &param_lt.param_ltf,
        &mut line,
        &mut resolved_vertex
    )?;

    ar_matrix_code_get_id(
        image,
        ar_handle.xsize,
        ar_handle.ysize,
        &resolved_vertex,
        ar_handle.get_matrix_code_type(),
        ar_handle.ar_pixel_format,
        ar_handle.patt_ratio,
        &mut id,
        &mut dir,
        &mut cf,
        &mut error_corrected
    )?;

    marker_info.id_matrix = id;
    marker_info.dir_matrix = dir;
    marker_info.cf_matrix = cf;
    marker_info.area = marker_info2.area;
    marker_info.pos = marker_info2.pos;
    marker_info.vertex = resolved_vertex;
    marker_info.line = line;
    marker_info.error_corrected = error_corrected;

    Ok(marker_info)
}


/// Project image pixels onto a regular grid using a homography.
///
/// Samples `grid_size × grid_size` evenly-spaced points inside the square
/// defined by `vertex`. Uses the same world-coordinate system as `arPattGetImage2`
/// (i.e. nominal corners at (100,100)…(110,110)) so that `patt_ratio` correctly
/// controls what fraction of the square area is sampled.
///
/// For each grid cell `(x, y)` the homography (`para`) maps world coordinates to
/// image pixel `(xc, yc)` and reads into `bits` the intensity (G channel for
/// multi-channel formats, first byte for luma).
///
/// # Parameters
/// - `grid_size` — total grid side length including the one-cell border ring;
///   equal to `dim + 2` where `dim = code_type & 0xFF`.
/// - `patt_ratio` — fraction of the square covered by data cells (0.5–0.9).
/// - `bits` — output: `grid_size * grid_size` raw intensity values.
fn sample_grid(
    image: &[u8],
    xsize: i32,
    ysize: i32,
    vertex: &[[ARdouble; 2]; 4],
    grid_size: i32,
    pixel_format: crate::types::ARPixelFormat,
    patt_ratio: f64,
    bits: &mut [u8],
) -> Result<(), &'static str> {
    let nc = match pixel_format {
        crate::types::ARPixelFormat::MONO => 1,
        crate::types::ARPixelFormat::RGB | crate::types::ARPixelFormat::BGR => 3,
        crate::types::ARPixelFormat::RGBA | crate::types::ARPixelFormat::BGRA | crate::types::ARPixelFormat::ARGB => 4,
        _ => return Err("Unsupported pixel format in sample_grid"),
    };

    let mut world = [[0.0; 2]; 4];
    let mut para = [[0.0; 3]; 3];

    // Match C reference arPattGetImage2: world coords (100,100)→(110,110)
    world[0][0] = 100.0; world[0][1] = 100.0;
    world[1][0] = 110.0; world[1][1] = 100.0;
    world[2][0] = 110.0; world[2][1] = 110.0;
    world[3][0] = 100.0; world[3][1] = 110.0;

    crate::pattern::get_cpara(&world, vertex, &mut para)?;

    let patt_ratio1 = (1.0 - patt_ratio) / 2.0 * 10.0;
    let patt_ratio2 = patt_ratio * 10.0;

    for y in 0..grid_size {
        let yw = (100.0 + patt_ratio1) + patt_ratio2 * (y as f64 + 0.5) / grid_size as f64;
        for x in 0..grid_size {
            let xw = (100.0 + patt_ratio1) + patt_ratio2 * (x as f64 + 0.5) / grid_size as f64;
            
            let d = para[2][0] * xw + para[2][1] * yw + para[2][2];
            if d == 0.0 { continue; }
            
            let xc = ((para[0][0] * xw + para[0][1] * yw + para[0][2]) / d) as i32;
            let yc = ((para[1][0] * xw + para[1][1] * yw + para[1][2]) / d) as i32;

            if xc >= 0 && xc < xsize && yc >= 0 && yc < ysize {
                if y == 0 && (x == 0 || x == grid_size - 1) || y == grid_size - 1 && (x == 0 || x == grid_size - 1) {
                    trace!("sample_grid: grid({},{}) -> image({},{})", x, y, xc, yc);
                }
                let idx = ((yc * xsize + xc) * nc) as usize;
                if idx < image.len() {
                    // For RGB, averaging R, G, B gives luma. Or we can just take Green channel for simplicity.
                    // We'll take the G channel (idx + 1) for RGB, or just first byte (idx) if luma
                    bits[(y * grid_size + x) as usize] = if nc >= 3 {
                        // Approximation: Read the second byte (G) which usually represents contrast well, 
                        // or do a simple average if safe
                        image[idx + 1]
                    } else {
                        image[idx]
                    };
                }
            }
        }
    }

    Ok(())
}



/// Rotates a `dim × dim` bit-grid by `dir * 90°` counter-clockwise.
///
/// Used to bring a barcode into a canonical orientation before extracting the
/// data-bit sequence. Directions follow the ARToolKit convention:
/// - `0` — no rotation (identity)
/// - `1` — 90° CCW
/// - `2` — 180°
/// - `3` — 270° CCW
#[allow(dead_code)]
fn rotate_bits(bits: &[u8], dim: i32, dir: i32) -> Vec<u8> {
    let mut rotated = vec![0u8; bits.len()];
    for y in 0..dim {
        for x in 0..dim {
            let (nx, ny) = match dir {
                0 => (x, y),
                1 => (y, dim - 1 - x),
                2 => (dim - 1 - x, dim - 1 - y),
                3 => (dim - 1 - y, x),
                _ => (x, y),
            };
            rotated[(ny * dim + nx) as usize] = bits[(y * dim + x) as usize];
        }
    }
    rotated
}

/// Decodes a raw bit-word into a marker ID, applying ECC if the `code_type` supports it.
///
/// Maps `code_raw` (a `u64` bit-field extracted from the core grid) to a marker
/// ID using the appropriate lookup table for the given `code_type`:
///
/// | `code_type`                  | ECC scheme                              |
/// |------------------------------|-----------------------------------------|
/// | `Code3x3`                    | None — raw 6-bit value                  |
/// | `Code3x3Parity65`            | Single-bit parity (table lookup)        |
/// | `Code3x3Hamming63`           | Hamming (6,3) (table lookup)            |
/// | `Code4x4` / `Code5x5`       | BCH (39,12) / BCH (51,12)               |
fn decode_matrix_raw(code_raw: u64, code_type: ARMatrixCodeType) -> Result<(i32, i32), &'static str> {
    match code_type {
        ARMatrixCodeType::Code3x3 => {
            // Simple 3x3 has no ECC in ARToolKit5! The bits strictly form the value ranging up to 63 since it's 9 bits minus corners?
            // Actually it's 9 bits, so it ranges up to 511.
            Ok((code_raw as i32, 0))
        },
        ARMatrixCodeType::Code3x3Parity65 => {
            let code = crate::bch::decode_parity65(code_raw)?;
            Ok((code as i32, 0))
        },
        ARMatrixCodeType::Code3x3Hamming63 => {
            let (code, err) = crate::bch::decode_hamming63(code_raw)?;
            Ok((code as i32, err))
        },
        ARMatrixCodeType::Code4x4BCH1393 | ARMatrixCodeType::Code4x4BCH1355 |
        ARMatrixCodeType::Code5x5BCH22125 | ARMatrixCodeType::Code5x5BCH2277 => {
            let (code, err) = crate::bch::decode_bch(code_type, code_raw)?;
            Ok((code as i32, err))
        },
        _ => {
            // The unhandled types include GLOBAL_ID, etc.
            Ok((code_raw as i32, 0))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rotate_bits() {
        let dim = 3;
        let bits = vec![
            1, 0, 0,
            1, 1, 0,
            1, 0, 0,
        ];
        
        let rot1 = rotate_bits(&bits, dim, 1);
        let expected1 = vec![
            0, 0, 0,
            0, 1, 0,
            1, 1, 1,
        ];
        assert_eq!(rot1, expected1);
    }
}
