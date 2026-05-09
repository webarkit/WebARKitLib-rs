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

use super::marker::ar_get_line;
use crate::types::{ARHandle, ARMarkerInfo, ARMarkerInfo2, ARMatrixCodeType, ARdouble, MatchError};
use crate::{arlog_d, arlog_e};
use log::trace;

/// Outer dimension (in cells) of the AR_MATRIX_CODE_GLOBAL_ID grid sampled
/// from the marker. Mirrors `AR_GLOBAL_ID_OUTER_SIZE` in the C code
/// (`arPattGetID.c:98`).
pub(crate) const AR_GLOBAL_ID_OUTER_SIZE: usize = 14;

/// Border thickness (in cells) of the GlobalID grid. Cells with both indices
/// inside the inner `OUTER - 2*INNER = 8`-cell core are *not* used for data;
/// only the border ring contributes bits. Mirrors `AR_GLOBAL_ID_INNER_SIZE`
/// in the C code (`arPattGetID.c:99`).
pub(crate) const AR_GLOBAL_ID_INNER_SIZE: usize = 3;

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
///
/// # Returns
/// `Ok(MatchOk { id, dir, cf, error_corrected, global_id })` on successful decode.
/// `Err(MatchError::*)` on low contrast, missing locator pattern, ECC failure,
/// or unsupported `code_type`.
///
/// # Example
/// ```rust,no_run
/// use webarkitlib_rs::arlog_i;
/// use webarkitlib_rs::matrix::ar_matrix_code_get_id;
/// use webarkitlib_rs::types::ARMatrixCodeType;
///
/// let image = vec![0u8; 640 * 480 * 3];
/// let vertex = [[100.0, 100.0], [200.0, 100.0], [200.0, 200.0], [100.0, 200.0]];
/// if let Ok(ok) = ar_matrix_code_get_id(
///     &image, 640, 480, &vertex,
///     ARMatrixCodeType::default(),
///     webarkitlib_rs::types::ARPixelFormat::RGB,
///     0.5,
/// ) {
///     arlog_i!("Decoded barcode id={}, dir={}, cf={:.2}", ok.id, ok.dir, ok.cf);
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
) -> Result<crate::types::MatchOk, crate::types::MatchError> {
    use crate::types::{MatchError, MatchOk};

    // GlobalID uses a different grid size (14×14), bit layout, and ECC scheme;
    // route it to the dedicated decoder mirroring `arPattGetID.c:200-218`.
    if code_type == ARMatrixCodeType::GlobalID {
        return ar_matrix_code_get_id_global(image, xsize, ysize, vertex, pixel_format);
    }

    let dim = (code_type as i32) & 0xFF;
    if !(3..=6).contains(&dim) {
        arlog_e!(
            "ar_matrix_code_get_id: unsupported dim={} (expected 3..=6)",
            dim
        );
        return Err(MatchError::Generic);
    }

    let grid_size = dim;
    let mut bits = vec![0u8; (grid_size * grid_size) as usize];

    sample_grid(
        image,
        xsize,
        ysize,
        vertex,
        grid_size,
        pixel_format,
        patt_ratio,
        &mut bits,
    )
    .map_err(|_| MatchError::PatternExtraction)?;

    arlog_d!("ar_matrix_code_get_id: vertices v0=({:.1},{:.1}) v1=({:.1},{:.1}) v2=({:.1},{:.1}) v3=({:.1},{:.1})",
        vertex[0][0], vertex[0][1], vertex[1][0], vertex[1][1],
        vertex[2][0], vertex[2][1], vertex[3][0], vertex[3][1]);

    let mut min_val = 255u8;
    let mut max_val = 0u8;
    for &b in &bits {
        if b < min_val {
            min_val = b;
        }
        if b > max_val {
            max_val = b;
        }
    }

    let mid_thresh = ((min_val as u32 + max_val as u32) / 2) as u8;
    arlog_d!(
        "ar_matrix_code_get_id: min={}, max={}, thresh={}",
        min_val,
        max_val,
        mid_thresh
    );

    if (max_val as i32 - min_val as i32) < 30 {
        arlog_d!(
            "ar_matrix_code_get_id: low contrast range={}",
            max_val as i32 - min_val as i32
        );
        return Err(MatchError::Contrast);
    }

    let mut core_bits = vec![0u8; (grid_size * grid_size) as usize];
    for i in 0..bits.len() {
        core_bits[i] = if bits[i] < mid_thresh { 1 } else { 0 };
    }

    trace!(
        "ar_matrix_code_get_id: grid_size={}, core_bits={:?}",
        grid_size,
        core_bits
    );
    arlog_d!("ar_matrix_code_get_id: core_bits={:?}", core_bits);

    // The C reference `get_matrix_code` receives only the dim×dim inner data.
    // Check orientation based on corners of the core data:
    // An unrotated pattern has:
    //   top-left (0,0) = 1, bottom-left (0, dim-1) = 1
    //   bottom-right (dim-1, dim-1) = 0.
    let size = dim;
    let corners = [
        0,                 // top-left
        (size - 1) * size, // bottom-left
        size * size - 1,   // bottom-right
        size - 1,          // top-right
    ];

    let mut dir_code = [0u8; 4];
    for i in 0..4 {
        dir_code[i] = core_bits[corners[i] as usize];
    }

    let mut matched_dir = -1;
    for i in 0..4 {
        if dir_code[i] == 1 && dir_code[(i + 1) % 4] == 1 && dir_code[(i + 2) % 4] == 0 {
            matched_dir = i as i32;
            break;
        }
    }

    if matched_dir == -1 {
        arlog_d!(
            "ar_matrix_code_get_id: locator pattern not found, dir_code={:?}",
            dir_code
        );
        return Err(MatchError::BarcodeNotFound);
    }

    // Extract code from the dim×dim core bits.
    // The 3 pixels forming the locator corners are ignored.
    let mut code_raw = 0u64;

    if matched_dir == 0 {
        for j in 0..size {
            for i in 0..size {
                if i == 0 && j == 0 {
                    continue;
                }
                if i == 0 && j == size - 1 {
                    continue;
                }
                if i == size - 1 && j == size - 1 {
                    continue;
                }
                code_raw <<= 1;
                if core_bits[(j * size + i) as usize] == 1 {
                    code_raw += 1;
                }
            }
        }
    } else if matched_dir == 1 {
        for i in 0..size {
            for j in (0..size).rev() {
                if i == 0 && j == size - 1 {
                    continue;
                }
                if i == size - 1 && j == size - 1 {
                    continue;
                }
                if i == size - 1 && j == 0 {
                    continue;
                }
                code_raw <<= 1;
                if core_bits[(j * size + i) as usize] == 1 {
                    code_raw += 1;
                }
            }
        }
    } else if matched_dir == 2 {
        for j in (0..size).rev() {
            for i in (0..size).rev() {
                if i == size - 1 && j == size - 1 {
                    continue;
                }
                if i == size - 1 && j == 0 {
                    continue;
                }
                if i == 0 && j == 0 {
                    continue;
                }
                code_raw <<= 1;
                if core_bits[(j * size + i) as usize] == 1 {
                    code_raw += 1;
                }
            }
        }
    } else if matched_dir == 3 {
        for i in (0..size).rev() {
            for j in 0..size {
                if i == size - 1 && j == 0 {
                    continue;
                }
                if i == 0 && j == 0 {
                    continue;
                }
                if i == 0 && j == size - 1 {
                    continue;
                }
                code_raw <<= 1;
                if core_bits[(j * size + i) as usize] == 1 {
                    code_raw += 1;
                }
            }
        }
    }

    arlog_d!(
        "ar_matrix_code_get_id: code_raw={}, dir={}",
        code_raw,
        matched_dir
    );

    // Handle decoding logic mapped from pattern types!
    if let Ok((decoded_id, err)) = decode_matrix_raw(code_raw, code_type) {
        Ok(MatchOk {
            id: decoded_id,
            dir: matched_dir,
            cf: 1.0,
            error_corrected: err,
            global_id: 0,
        })
    } else {
        arlog_d!(
            "ar_matrix_code_get_id: decode failed for code_raw={} (dir={})",
            code_raw,
            matched_dir
        );
        Err(MatchError::BarcodeEdcFail)
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
    marker_info2: &mut ARMarkerInfo2,
) -> Result<ARMarkerInfo, &'static str> {
    trace!(
        "ar_get_barcode_marker: started for area={}, coord_num={}",
        marker_info2.area,
        marker_info2.coord_num
    );

    let mut marker_info = ARMarkerInfo::default();

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
        &mut resolved_vertex,
    )?;

    let ok = ar_matrix_code_get_id(
        image,
        ar_handle.xsize,
        ar_handle.ysize,
        &resolved_vertex,
        ar_handle.get_matrix_code_type(),
        ar_handle.ar_pixel_format,
        ar_handle.patt_ratio,
    )
    .map_err(|_| "matrix code decode failed")?;

    marker_info.id_matrix = ok.id;
    marker_info.dir_matrix = ok.dir;
    marker_info.cf_matrix = ok.cf;
    marker_info.area = marker_info2.area;
    marker_info.pos = marker_info2.pos;
    marker_info.vertex = resolved_vertex;
    marker_info.line = line;
    marker_info.error_corrected = ok.error_corrected;

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
#[allow(clippy::too_many_arguments)]
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
        crate::types::ARPixelFormat::RGBA
        | crate::types::ARPixelFormat::BGRA
        | crate::types::ARPixelFormat::ARGB => 4,
        _ => {
            arlog_e!("sample_grid: unsupported pixel format {:?}", pixel_format);
            return Err("Unsupported pixel format in sample_grid");
        }
    };

    let mut world = [[0.0; 2]; 4];
    let mut para = [[0.0; 3]; 3];

    // Match C reference arPattGetImage2: world coords (100,100)→(110,110)
    world[0][0] = 100.0;
    world[0][1] = 100.0;
    world[1][0] = 110.0;
    world[1][1] = 100.0;
    world[2][0] = 110.0;
    world[2][1] = 110.0;
    world[3][0] = 100.0;
    world[3][1] = 110.0;

    crate::pattern::get_cpara(&world, vertex, &mut para)?;

    let patt_ratio1 = (1.0 - patt_ratio) / 2.0 * 10.0;
    let patt_ratio2 = patt_ratio * 10.0;

    for y in 0..grid_size {
        let yw = (100.0 + patt_ratio1) + patt_ratio2 * (y as f64 + 0.5) / grid_size as f64;
        for x in 0..grid_size {
            let xw = (100.0 + patt_ratio1) + patt_ratio2 * (x as f64 + 0.5) / grid_size as f64;

            let d = para[2][0] * xw + para[2][1] * yw + para[2][2];
            if d == 0.0 {
                continue;
            }

            let xc = ((para[0][0] * xw + para[0][1] * yw + para[0][2]) / d) as i32;
            let yc = ((para[1][0] * xw + para[1][1] * yw + para[1][2]) / d) as i32;

            if xc >= 0 && xc < xsize && yc >= 0 && yc < ysize {
                if (y == 0 || y == grid_size - 1) && (x == 0 || x == grid_size - 1) {
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
fn decode_matrix_raw(
    code_raw: u64,
    code_type: ARMatrixCodeType,
) -> Result<(i32, i32), &'static str> {
    match code_type {
        ARMatrixCodeType::Code3x3 => {
            // Simple 3x3 has no ECC in ARToolKit5! The bits strictly form the value ranging up to 63 since it's 9 bits minus corners?
            // Actually it's 9 bits, so it ranges up to 511.
            Ok((code_raw as i32, 0))
        }
        ARMatrixCodeType::Code3x3Parity65 => {
            let code = crate::bch::decode_parity65(code_raw)?;
            Ok((code as i32, 0))
        }
        ARMatrixCodeType::Code3x3Hamming63 => {
            let (code, err) = crate::bch::decode_hamming63(code_raw)?;
            Ok((code as i32, err))
        }
        ARMatrixCodeType::Code4x4BCH1393
        | ARMatrixCodeType::Code4x4BCH1355
        | ARMatrixCodeType::Code5x5BCH22125
        | ARMatrixCodeType::Code5x5BCH2277 => {
            let (code, err) = crate::bch::decode_bch(code_type, code_raw)?;
            Ok((code as i32, err))
        }
        _ => {
            // The unhandled types include GLOBAL_ID, etc.
            Ok((code_raw as i32, 0))
        }
    }
}

/// Decodes an `AR_MATRIX_CODE_GLOBAL_ID` marker — 14×14 grid with BCH(127, 64,
/// 22) error correction and a 64-bit identifier.
///
/// Mirrors the GlobalID branch of `arPattGetIDGlobal` in
/// `arPattGetID.c:200-218`:
///   1. Sample a 14×14 grid using `patt_ratio = 14 / (14 + 2) = 0.875`.
///   2. Extract 120 bits via [`extract_global_id_bits`].
///   3. BCH-decode via [`crate::bch::decode_bch_global_id`].
///   4. Reject `u64::MAX` as a frequently-misrecognised pattern (heuristic).
///   5. For backward compatibility with `id_matrix`, set the lower 31 bits of
///      `global_id` as the regular matrix `id` when the upper 33 bits are zero
///      (mirrors `arPattGetID.c:214`).
fn ar_matrix_code_get_id_global(
    image: &[u8],
    xsize: i32,
    ysize: i32,
    vertex: &[[ARdouble; 2]; 4],
    pixel_format: crate::types::ARPixelFormat,
) -> Result<crate::types::MatchOk, MatchError> {
    use crate::types::MatchOk;

    // Sample a 14×14 grid. The C code passes pattRatio = 14 / (14 + 2) = 0.875
    // to `arPattGetImage2`, which corresponds to a 1-cell border around the
    // 14-cell pattern (16-cell total marker space).
    let mut grid = vec![0u8; AR_GLOBAL_ID_OUTER_SIZE * AR_GLOBAL_ID_OUTER_SIZE];
    let patt_ratio = AR_GLOBAL_ID_OUTER_SIZE as f64 / (AR_GLOBAL_ID_OUTER_SIZE as f64 + 2.0);
    sample_grid(
        image,
        xsize,
        ysize,
        vertex,
        AR_GLOBAL_ID_OUTER_SIZE as i32,
        pixel_format,
        patt_ratio,
        &mut grid,
    )
    .map_err(|_| {
        arlog_d!("ar_matrix_code_get_id_global: sample_grid failed");
        MatchError::PatternExtraction
    })?;

    // Extract the 120 data bits (orientation-aware).
    let (mut recd127, dir, cf) = extract_global_id_bits(&grid)?;

    // BCH(127, 64, 22) decode. Up to 9 bit errors are correctable.
    let (global_id, error_corrected) =
        crate::bch::decode_bch_global_id(&mut recd127).map_err(|_| {
            arlog_d!("ar_matrix_code_get_id_global: BCH decode failed");
            MatchError::BarcodeEdcFail
        })?;

    // Heuristic: `u64::MAX` is a known false-positive pattern. The C code maps
    // this to error code -5, which is `MatchError::HeuristicTroublesomeMatrixCodes`.
    if global_id == u64::MAX {
        arlog_d!("ar_matrix_code_get_id_global: heuristic rejected (UINT64_MAX)");
        return Err(MatchError::HeuristicTroublesomeMatrixCodes);
    }

    // Backward-compat: when the upper 33 bits are zero, populate `id` with the
    // lower 31 bits as if this were a regular matrix code.
    let id = if (global_id & 0xFFFF_8000_u64) == 0 {
        (global_id & 0x0000_7FFF_u64) as i32
    } else {
        0
    };

    Ok(MatchOk {
        id,
        dir,
        cf,
        error_corrected,
        global_id,
    })
}

/// Extracts the 120 GlobalID bits from a 14×14 sampled grid.
///
/// Mirrors C `get_global_id_code()` in `arPattGetID.c:2282-2404`. Detects the
/// orientation L-pattern from the four corner cells, then walks the border
/// ring (skipping the inner 8×8 zone and three of the four corner 2×2
/// blocks) to read 120 bits in a canonical order regardless of marker
/// rotation.
///
/// # Parameters
/// - `data` — 14×14 sampled grayscale buffer (row-major:
///   `data[j * OUTER + i]` is the cell at column `i`, row `j`).
///
/// # Returns
/// - `Ok((recd127, dir, cf))` — `recd127[0..120]` holds the read bits
///   (bit 119 = first read, bit 0 = last read); `dir` is 0..=3; `cf` is the
///   confidence (`min_contrast / 30.0`, capped at 1.0).
/// - `Err(MatchError::Contrast)` if the corner contrast is < 30.
/// - `Err(MatchError::BarcodeNotFound)` if no `(1, 1, 0)` L-pattern is found
///   in the four corner cells.
pub(crate) fn extract_global_id_bits(data: &[u8]) -> Result<([u8; 127], i32, f64), MatchError> {
    const SIZE: usize = AR_GLOBAL_ID_OUTER_SIZE;
    debug_assert_eq!(data.len(), SIZE * SIZE);

    // 1. Compute threshold from the four corner cells.
    //    Linear positions match C: 0, (S-1)*S, S*S-1, S-1.
    let corner = [0usize, (SIZE - 1) * SIZE, SIZE * SIZE - 1, SIZE - 1];
    let mut max = 0u8;
    let mut min = 255u8;
    for &c in &corner {
        let p = data[c];
        if p > max {
            max = p;
        }
        if p < min {
            min = p;
        }
    }
    if (max as i32 - min as i32) < 30 {
        arlog_d!(
            "extract_global_id_bits: insufficient contrast (max-min={})",
            max as i32 - min as i32
        );
        return Err(MatchError::Contrast);
    }
    let thresh = ((max as u16 + min as u16) / 2) as u8;

    // 2. Detect orientation. An unrotated marker has corners (1, 1, 0, ?)
    //    where 1 means "darker than threshold". Rotations cycle the
    //    L-pattern through positions [i, i+1, i+2] (mod 4).
    let dir_code: [u8; 4] = [
        if data[corner[0]] < thresh { 1 } else { 0 },
        if data[corner[1]] < thresh { 1 } else { 0 },
        if data[corner[2]] < thresh { 1 } else { 0 },
        if data[corner[3]] < thresh { 1 } else { 0 },
    ];
    let dir = (0..4)
        .find(|&i| dir_code[i] == 1 && dir_code[(i + 1) % 4] == 1 && dir_code[(i + 2) % 4] == 0)
        .ok_or_else(|| {
            arlog_d!(
                "extract_global_id_bits: locator pattern not found (dirCode={:?})",
                dir_code
            );
            MatchError::BarcodeNotFound
        })?;

    // 3. Walk the border ring in the order dictated by `dir`, reading 120
    //    bits into `recd127[0..120]` (MSB-first: bit 119 is the first cell
    //    visited, bit 0 the last).
    let mut recd127 = [0u8; 127];
    let mut bit: i32 = 119;
    let mut contrast_min: i32 = 255;

    let mut read_pixel = |i: usize, j: usize, bit: &mut i32, cmin: &mut i32| {
        let contrast = data[j * SIZE + i] as i32 - thresh as i32;
        recd127[*bit as usize] = if contrast < 0 { 1 } else { 0 };
        *bit -= 1;
        let abs_c = contrast.abs();
        if abs_c < *cmin {
            *cmin = abs_c;
        }
    };

    match dir {
        0 => {
            for j in 0..SIZE {
                for i in 0..SIZE {
                    if should_skip_global_id_cell(i, j, 0) {
                        continue;
                    }
                    read_pixel(i, j, &mut bit, &mut contrast_min);
                }
            }
        }
        1 => {
            for i in 0..SIZE {
                for j in (0..SIZE).rev() {
                    if should_skip_global_id_cell(i, j, 1) {
                        continue;
                    }
                    read_pixel(i, j, &mut bit, &mut contrast_min);
                }
            }
        }
        2 => {
            for j in (0..SIZE).rev() {
                for i in (0..SIZE).rev() {
                    if should_skip_global_id_cell(i, j, 2) {
                        continue;
                    }
                    read_pixel(i, j, &mut bit, &mut contrast_min);
                }
            }
        }
        3 => {
            for i in (0..SIZE).rev() {
                for j in 0..SIZE {
                    if should_skip_global_id_cell(i, j, 3) {
                        continue;
                    }
                    read_pixel(i, j, &mut bit, &mut contrast_min);
                }
            }
        }
        _ => unreachable!(),
    }

    debug_assert_eq!(bit, -1, "expected exactly 120 bits to be read");

    // 4. Confidence based on minimum observed contrast: full confidence at
    //    >= 30, scaled below.
    let cf = if contrast_min > 30 {
        1.0
    } else {
        contrast_min as f64 / 30.0
    };

    Ok((recd127, dir as i32, cf))
}

/// Returns `true` if the `(i, j)` cell of a 14×14 GlobalID grid should be
/// skipped during bit extraction for the given `dir`.
///
/// Two skip categories:
/// 1. The interior 8×8 zone (cells where both `i` and `j` lie strictly between
///    `INNER - 1` and `OUTER - INNER`) carries no data.
/// 2. Three of the four 2×2 corner blocks per `dir`: the three blocks that
///    form the L-shape locator. The fourth corner is the "data corner" and
///    contributes 4 of the 120 bits. The corner cycle is:
///    - dir 0 → skip TL, BL, BR (data at TR)
///    - dir 1 → skip BL, BR, TR (data at TL)
///    - dir 2 → skip BR, TR, TL (data at BL)
///    - dir 3 → skip TR, TL, BL (data at BR)
fn should_skip_global_id_cell(i: usize, j: usize, dir: usize) -> bool {
    const SIZE: usize = AR_GLOBAL_ID_OUTER_SIZE;
    const INNER: usize = AR_GLOBAL_ID_INNER_SIZE;

    // Inner 8×8 skip zone.
    if i > INNER - 1 && i < SIZE - INNER && j > INNER - 1 && j < SIZE - INNER {
        return true;
    }

    // Round indices down to the nearest even number to identify the 2×2
    // corner blocks: top-left = (0,0), bottom-left = (0,12),
    // bottom-right = (12,12), top-right = (12,0). MAX_Q = SIZE - 2 = 12.
    let i_q = i & !1;
    let j_q = j & !1;
    const MAX_Q: usize = SIZE - 2;

    let tl = i_q == 0 && j_q == 0;
    let bl = i_q == 0 && j_q == MAX_Q;
    let br = i_q == MAX_Q && j_q == MAX_Q;
    let tr = i_q == MAX_Q && j_q == 0;

    match dir {
        0 => tl || bl || br,
        1 => bl || br || tr,
        2 => br || tr || tl,
        3 => tr || tl || bl,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rotate_bits() {
        let dim = 3;
        let bits = vec![1, 0, 0, 1, 1, 0, 1, 0, 0];

        let rot1 = rotate_bits(&bits, dim, 1);
        let expected1 = vec![0, 0, 0, 0, 1, 0, 1, 1, 1];
        assert_eq!(rot1, expected1);
    }

    #[test]
    fn test_ar_matrix_code_get_id_low_contrast_returns_match_error() {
        use crate::types::{ARMatrixCodeType, ARPixelFormat, MatchError};
        // 100x100 image, all the same value -> low contrast in the sampled grid
        let image = vec![128u8; 100 * 100];
        let vertex = [[10.0f64, 10.0], [90.0, 10.0], [90.0, 90.0], [10.0, 90.0]];
        let result = ar_matrix_code_get_id(
            &image,
            100,
            100,
            &vertex,
            ARMatrixCodeType::Code3x3,
            ARPixelFormat::MONO,
            0.5,
        );
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), MatchError::Contrast);
    }

    // -----------------------------------------------------------------
    // GlobalID bit extraction tests
    // -----------------------------------------------------------------

    /// Builds a 14×14 grid encoding 120 known bits at the data positions for a
    /// given direction, plus the L-shape locator pattern. This is the inverse
    /// of `extract_global_id_bits` for `dir = 0`.
    ///
    /// The data corner (the 2×2 NOT skipped by the dir's skip rules) receives
    /// 4 of the 120 bits; remaining bits fill border cells in the same
    /// iteration order as the extractor.
    fn make_global_id_grid_dir0(bits120: &[u8; 120]) -> [u8; 14 * 14] {
        const SIZE: usize = AR_GLOBAL_ID_OUTER_SIZE;
        let mut grid = [128u8; SIZE * SIZE]; // mid-gray default for interior

        // Establish the dir 0 L-pattern (TL=dark, BL=dark, BR=light, TR=data).
        // We'll set TR's corner pixel below as part of the bit fill.
        grid[0] = 0; // TL (dark = bit 1)
        grid[(SIZE - 1) * SIZE] = 0; // BL (dark)
        grid[SIZE * SIZE - 1] = 255; // BR (light = bit 0)

        // Walk the dir 0 iteration writing bits 119..0 to non-skip cells.
        let mut bit: i32 = 119;
        for j in 0..SIZE {
            for i in 0..SIZE {
                if should_skip_global_id_cell(i, j, 0) {
                    continue;
                }
                grid[j * SIZE + i] = if bits120[bit as usize] != 0 { 0 } else { 255 };
                bit -= 1;
            }
        }
        debug_assert_eq!(bit, -1);
        grid
    }

    /// Rotates a 14×14 grid 90° clockwise (used to produce dir=1, 2, 3 inputs
    /// from a dir=0-formatted base grid).
    fn rotate_grid_cw(grid: &[u8; 14 * 14]) -> [u8; 14 * 14] {
        const SIZE: usize = AR_GLOBAL_ID_OUTER_SIZE;
        let mut out = [0u8; SIZE * SIZE];
        for j in 0..SIZE {
            for i in 0..SIZE {
                // Rotated cell at (i', j') = (SIZE-1-j, i).
                let new_i = SIZE - 1 - j;
                let new_j = i;
                out[new_j * SIZE + new_i] = grid[j * SIZE + i];
            }
        }
        out
    }

    #[test]
    fn test_extract_global_id_bits_low_contrast() {
        // Uniform image -> max-min < 30 -> Contrast error.
        let data = [128u8; 14 * 14];
        let result = extract_global_id_bits(&data);
        assert_eq!(result.unwrap_err(), MatchError::Contrast);
    }

    #[test]
    fn test_extract_global_id_bits_no_locator_pattern() {
        // 4 corners high contrast but not in (1,1,0,?) cyclic L-shape.
        // Set all four corners to the same value -> can't form (1,1,0).
        let mut data = [128u8; 14 * 14];
        data[0] = 0; // TL dark
        data[(14 - 1) * 14] = 0; // BL dark
        data[14 * 14 - 1] = 0; // BR dark (breaks the L: needs to be light)
        data[14 - 1] = 0; // TR dark
                          // We also need contrast > 30, so introduce a light pixel somewhere.
        data[7 * 14 + 7] = 255;
        // But the corner check uses corner pixels for max/min. Make max=0 (all dark).
        // Actually max-min must be >= 30, and it's computed over corners only.
        // Force one corner light so contrast passes but no L is formed.
        data[14 * 14 - 1] = 255; // BR light
                                 // Now corners = (0, 0, 255, 0) -> dirCode = (1, 1, 0, 1).
                                 // Check L: i=0 -> (1,1,0) ✓ -> dir=0. So this DOES form L. We need a
                                 // different invalid pattern. (1,0,1,0) has no L: at any i, dirCode[i+2]
                                 // is the same as dirCode[i].
        data[(14 - 1) * 14] = 255; // BL light -> corners = (0, 255, 255, 0) -> (1,0,0,1)
                                   // Check: i=0 (1,0,0) no; i=1 (0,0,1) no; i=2 (0,1,1) no; i=3 (1,1,0) yes -> dir=3.
                                   // Still passes. Try (1,0,1,0): TL=dark, BL=light, BR=dark, TR=light.
        data[0] = 0; // TL dark
        data[(14 - 1) * 14] = 255; // BL light
        data[14 * 14 - 1] = 0; // BR dark
        data[14 - 1] = 255; // TR light
                            // dirCode = (1, 0, 1, 0): no three consecutive (1,1,0) exist.
        let result = extract_global_id_bits(&data);
        assert_eq!(result.unwrap_err(), MatchError::BarcodeNotFound);
    }

    #[test]
    fn test_extract_global_id_bits_dir0_detected() {
        // Build a grid with the dir 0 L-pattern (TL=1, BL=1, BR=0).
        let bits = [0u8; 120];
        let grid = make_global_id_grid_dir0(&bits);
        let (recd, dir, _cf) = extract_global_id_bits(&grid).unwrap();
        assert_eq!(dir, 0);
        // All non-corner cells were filled with 255 (light = bit 0).
        for &b in &recd[..120] {
            assert_eq!(b, 0);
        }
    }

    #[test]
    fn test_extract_global_id_bits_all_directions_recover_same_bits() {
        // Build a known 120-bit pattern and confirm each rotation of the
        // input grid yields the same `recd[0..120]` (different dir values).
        let mut bits = [0u8; 120];
        for i in 0..120 {
            // Pseudo-random pattern: prime-indexed positions are 1.
            bits[i] = if matches!(
                i,
                2 | 3
                    | 5
                    | 7
                    | 11
                    | 13
                    | 17
                    | 19
                    | 23
                    | 29
                    | 31
                    | 37
                    | 41
                    | 43
                    | 47
                    | 53
                    | 59
                    | 61
                    | 67
                    | 71
                    | 73
                    | 79
                    | 83
                    | 89
                    | 97
                    | 101
                    | 103
                    | 107
                    | 109
                    | 113
            ) {
                1
            } else {
                0
            };
        }

        // The four iteration patterns are designed to "undo" the rotation, so
        // each successive 90° CW image rotation yields `dir = N + 3 mod 4`
        // (i.e. counts down through 0 → 3 → 2 → 1) while the canonical bit
        // order in `recd[..120]` stays invariant.
        let grid0 = make_global_id_grid_dir0(&bits);

        let (recd0, dir0, _) = extract_global_id_bits(&grid0).unwrap();
        assert_eq!(dir0, 0);
        assert_eq!(recd0[..120], bits[..]);

        let grid_cw1 = rotate_grid_cw(&grid0);
        let (recd1, dir1, _) = extract_global_id_bits(&grid_cw1).unwrap();
        assert_eq!(dir1, 3);
        assert_eq!(recd1[..120], bits[..]);

        let grid_cw2 = rotate_grid_cw(&grid_cw1);
        let (recd2, dir2, _) = extract_global_id_bits(&grid_cw2).unwrap();
        assert_eq!(dir2, 2);
        assert_eq!(recd2[..120], bits[..]);

        let grid_cw3 = rotate_grid_cw(&grid_cw2);
        let (recd3, dir3, _) = extract_global_id_bits(&grid_cw3).unwrap();
        assert_eq!(dir3, 1);
        assert_eq!(recd3[..120], bits[..]);
    }

    #[test]
    fn test_should_skip_global_id_cell_inner_zone() {
        // Cells with both indices in [3, 10] are interior (skip) for any dir.
        for dir in 0..4 {
            for j in 3..=10 {
                for i in 3..=10 {
                    assert!(
                        should_skip_global_id_cell(i, j, dir),
                        "dir={} (i={}, j={}) should be skipped (interior)",
                        dir,
                        i,
                        j
                    );
                }
            }
        }
    }

    #[test]
    fn test_should_skip_global_id_cell_data_corner_per_dir() {
        // Data corner per dir (the 2×2 NOT in the skip set):
        //   dir 0 -> TR (i_q=12, j_q=0)
        //   dir 1 -> TL (i_q=0,  j_q=0)
        //   dir 2 -> BL (i_q=0,  j_q=12)
        //   dir 3 -> BR (i_q=12, j_q=12)
        let data_corner = [(12, 0), (0, 0), (0, 12), (12, 12)];
        for (dir, &(i_q, j_q)) in data_corner.iter().enumerate() {
            for di in 0..2 {
                for dj in 0..2 {
                    let i = i_q + di;
                    let j = j_q + dj;
                    assert!(
                        !should_skip_global_id_cell(i, j, dir),
                        "dir={} data corner cell ({}, {}) must NOT be skipped",
                        dir,
                        i,
                        j
                    );
                }
            }
        }
    }

    /// Embeds a 14×14 GlobalID grid into a 16×16 monochrome image at pixels
    /// `(1..=14, 1..=14)`, leaving the 1-pixel border at the marker default.
    /// Designed so that vertex `[[0,0],[16,0],[16,16],[0,16]]` with
    /// `patt_ratio = 0.875` samples exactly one image pixel per cell.
    fn embed_grid_in_16x16_image(grid: &[u8; 14 * 14]) -> Vec<u8> {
        let mut image = vec![128u8; 16 * 16];
        for j in 0..14 {
            for i in 0..14 {
                image[(1 + j) * 16 + (1 + i)] = grid[j * 14 + i];
            }
        }
        image
    }

    #[test]
    fn test_ar_matrix_code_get_id_global_id_routing_low_contrast() {
        // Uniform image with code_type = GlobalID must hit the GlobalID branch
        // and return MatchError::Contrast (not Generic from the dim check).
        use crate::types::{ARMatrixCodeType, ARPixelFormat, MatchError};
        let image = vec![128u8; 16 * 16];
        let vertex = [[0.0f64, 0.0], [16.0, 0.0], [16.0, 16.0], [0.0, 16.0]];
        let result = ar_matrix_code_get_id(
            &image,
            16,
            16,
            &vertex,
            ARMatrixCodeType::GlobalID,
            ARPixelFormat::MONO,
            // patt_ratio is ignored for GlobalID (the function uses 14/16).
            0.5,
        );
        assert_eq!(result.unwrap_err(), MatchError::Contrast);
    }

    #[test]
    fn test_ar_matrix_code_get_id_global_id_full_roundtrip() {
        // End-to-end: encode a known global_id with BCH(127, 64, 22), embed
        // its 120-bit payload into a 16×16 image, then run the full decode
        // pipeline and recover the same global_id.
        use crate::bch::test_helpers::encode_bch_global_id;
        use crate::types::{ARMatrixCodeType, ARPixelFormat};

        let original_id: u64 = 0x1234_5678_DEAD_BEEF;
        let codeword = encode_bch_global_id(original_id);

        // Take the first 120 bits of the 127-bit codeword (the shortened
        // tail at positions 120..127 is implicitly zero in the grid).
        let mut bits120 = [0u8; 120];
        bits120.copy_from_slice(&codeword[..120]);

        let grid = make_global_id_grid_dir0(&bits120);
        let image = embed_grid_in_16x16_image(&grid);
        let vertex = [[0.0f64, 0.0], [16.0, 0.0], [16.0, 16.0], [0.0, 16.0]];

        let ok = ar_matrix_code_get_id(
            &image,
            16,
            16,
            &vertex,
            ARMatrixCodeType::GlobalID,
            ARPixelFormat::MONO,
            0.5,
        )
        .expect("decode should succeed");

        assert_eq!(ok.global_id, original_id);
        assert_eq!(ok.dir, 0);
        assert_eq!(ok.error_corrected, 0);
        // Backward-compat: upper 33 bits are non-zero, so id must be 0.
        assert_eq!(ok.id, 0);
    }

    #[test]
    fn test_ar_matrix_code_get_id_global_id_lower_31_bit_id() {
        // When global_id fits in 31 bits, the backward-compat `id` field
        // should carry the lower 31 bits (mirrors arPattGetID.c:214).
        use crate::bch::test_helpers::encode_bch_global_id;
        use crate::types::{ARMatrixCodeType, ARPixelFormat};

        let original_id: u64 = 0x0000_0000_0000_002A; // 42 — fits in 31 bits.
        let codeword = encode_bch_global_id(original_id);
        let mut bits120 = [0u8; 120];
        bits120.copy_from_slice(&codeword[..120]);

        let image = embed_grid_in_16x16_image(&make_global_id_grid_dir0(&bits120));
        let vertex = [[0.0f64, 0.0], [16.0, 0.0], [16.0, 16.0], [0.0, 16.0]];

        let ok = ar_matrix_code_get_id(
            &image,
            16,
            16,
            &vertex,
            ARMatrixCodeType::GlobalID,
            ARPixelFormat::MONO,
            0.5,
        )
        .expect("decode should succeed");

        assert_eq!(ok.global_id, 42);
        assert_eq!(ok.id, 42);
    }

    #[test]
    fn test_should_skip_global_id_cell_count_120() {
        // For every direction, the iteration over the full grid must produce
        // exactly 120 readable cells.
        const SIZE: usize = AR_GLOBAL_ID_OUTER_SIZE;
        for dir in 0..4 {
            let mut count = 0;
            for j in 0..SIZE {
                for i in 0..SIZE {
                    if !should_skip_global_id_cell(i, j, dir) {
                        count += 1;
                    }
                }
            }
            assert_eq!(count, 120, "dir={} should yield 120 readable cells", dir);
        }
    }
}
