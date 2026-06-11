/*
 *  param.rs
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

//! Parameter loading and manipulation utilities
//! Translated from ARToolKit C headers (param.h)

use crate::arlog_e;
use crate::types::ARParam;
use byteorder::{BigEndian, ReadBytesExt};
use std::io::{self, Read};

/// Resize camera parameters to a new image size **in place**.
///
/// C equivalent: `arParamChangeSize`
///
/// Scales rows 0 and 1 of `param.mat` by `xsize / param.xsize` and
/// `ysize / param.ysize` respectively, then updates `param.xsize` and
/// `param.ysize`.  This preserves the principal-point and focal-length
/// ratios when the camera is used with an image of a different resolution.
///
/// This matches the C calling convention where source and destination are
/// the same pointer:
/// ```c
/// arParamChangeSize(&cparam, width, height, &cparam);
/// ```
///
/// # Arguments
///
/// * `param` — Camera parameters to resize (mutated in place).
/// * `xsize` — New image width in pixels.
/// * `ysize` — New image height in pixels.
///
/// # Errors
///
/// Returns `Err` if `param.xsize` or `param.ysize` is zero before the
/// resize (i.e. the parameter was never initialised).
///
/// # Example
///
/// ```rust,no_run
/// use webarkitlib_rs::ar::param::ar_param_change_size;
/// use webarkitlib_rs::types::ARParam;
///
/// let mut cparam = ARParam::default();
/// cparam.xsize = 640;
/// cparam.ysize = 480;
/// cparam.mat[0][0] = 700.0; // fx
/// cparam.mat[1][1] = 700.0; // fy
///
/// ar_param_change_size(&mut cparam, 1280, 960).unwrap();
/// assert_eq!(cparam.xsize, 1280);
/// assert_eq!(cparam.ysize, 960);
/// ```
pub fn ar_param_change_size(
    param: &mut ARParam,
    xsize: i32,
    ysize: i32,
) -> Result<(), &'static str> {
    if param.xsize == 0 || param.ysize == 0 {
        arlog_e!(
            "ar_param_change_size: source ARParam has zero image dimensions ({}x{})",
            param.xsize,
            param.ysize
        );
        return Err("ar_param_change_size: source ARParam has zero image dimensions");
    }

    let sx = xsize as f64 / param.xsize as f64;
    let sy = ysize as f64 / param.ysize as f64;

    for col in 0..4 {
        param.mat[0][col] *= sx;
        param.mat[1][col] *= sy;
    }
    param.xsize = xsize;
    param.ysize = ysize;

    Ok(())
}

impl ARParam {
    /// Load ARParam from a byte stream (Endian-safe cross-platform BigEndian deserialization)
    #[allow(clippy::field_reassign_with_default)]
    pub fn load<R: Read>(mut reader: R) -> io::Result<Self> {
        let mut param = ARParam::default();

        // ARToolKit 5 parameter files are encoded in BigEndian format.
        param.xsize = reader.read_i32::<BigEndian>()?;
        param.ysize = reader.read_i32::<BigEndian>()?;

        // Load 3x4 projection matrix
        for row in 0..3 {
            for col in 0..4 {
                param.mat[row][col] = reader.read_f64::<BigEndian>()?;
            }
        }

        // Load distortion factors. AR_DIST_FACTOR_NUM_MAX is 9.
        for i in 0..crate::types::AR_DIST_FACTOR_NUM_MAX {
            if let Ok(val) = reader.read_f64::<BigEndian>() {
                param.dist_factor[i] = val;
            } else {
                break; // End of file or buffer handled gracefully for older parameter files.
            }
        }

        Ok(param)
    }

    /// Legacy ARToolKit v1 specific finalizer (swaps dist_factor 2 and 3)
    pub fn finalize_version_1(&mut self) {
        self.dist_factor.swap(2, 3);
    }
}

impl crate::types::ARParamLTf {
    /// Applies 2D distortion correction lookup
    pub fn observ2ideal(&self, ox: f32, oy: f32) -> Result<(f32, f32), &'static str> {
        let px = (ox + 0.5) as i32 + self.x_off;
        let py = (oy + 0.5) as i32 + self.y_off;

        if px < 0 || px >= self.xsize || py < 0 || py >= self.ysize {
            arlog_e!(
                "param.rs observ2ideal bounds fail: ox={}, oy={}, px={}, py={}, xsize={}, ysize={}",
                ox,
                oy,
                px,
                py,
                self.xsize,
                self.ysize
            );
            return Err("Coordinates out of bounds in lookup table");
        }

        let idx = ((py * self.xsize + px) * 2) as usize;
        if idx + 1 >= self.o2i.len() {
            return Err("Lookup table not properly initialized");
        }

        let ix = self.o2i[idx];
        let iy = self.o2i[idx + 1];

        Ok((ix, iy))
    }

    /// Applies inverse distortion lookup (calibrated to measured)
    pub fn ideal2observ(&self, ix: f32, iy: f32) -> Result<(f32, f32), &'static str> {
        let px = (ix + 0.5) as i32 + self.x_off;
        let py = (iy + 0.5) as i32 + self.y_off;

        if px < 0 || px >= self.xsize || py < 0 || py >= self.ysize {
            return Err("Coordinates out of bounds in lookup table");
        }

        let idx = ((py * self.xsize + px) * 2) as usize;
        if idx + 1 >= self.i2o.len() {
            return Err("Lookup table not properly initialized");
        }

        let ox = self.i2o[idx];
        let oy = self.i2o[idx + 1];

        Ok((ox, oy))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn test_arparam_load_big_endian() {
        // Construct a dummy byte array representing BigEndian encoded param data
        // xsize (4), ysize(4), mat(3*4 * 8), dist_factor(9 * 8)

        let mut buffer = Vec::new();
        // xsize = 640
        buffer.extend_from_slice(&640i32.to_be_bytes());
        // ysize = 480
        buffer.extend_from_slice(&480i32.to_be_bytes());

        // Fill mat with 1.0
        for _ in 0..12 {
            buffer.extend_from_slice(&1.0f64.to_be_bytes());
        }

        // Fill dist_factor with 2.0
        for _ in 0..9 {
            buffer.extend_from_slice(&2.0f64.to_be_bytes());
        }

        let cursor = Cursor::new(buffer);
        let param = ARParam::load(cursor).expect("Failed to load params");

        assert_eq!(param.xsize, 640);
        assert_eq!(param.ysize, 480);
        assert_eq!(param.mat[0][0], 1.0);
        assert_eq!(param.dist_factor[0], 2.0);
    }

    #[test]
    fn test_arparam_finalize_version_1() {
        let mut param = ARParam::default();
        param.dist_factor[2] = 10.0;
        param.dist_factor[3] = 20.0;

        param.finalize_version_1();

        assert_eq!(param.dist_factor[2], 20.0);
        assert_eq!(param.dist_factor[3], 10.0);
    }

    #[test]
    fn test_ar_param_change_size_doubles_resolution() {
        let mut src = ARParam {
            xsize: 640,
            ysize: 480,
            ..Default::default()
        };
        // fx, skew=0, cx, tx
        src.mat[0] = [700.0, 0.0, 320.0, 0.0];
        // 0, fy, cy, ty
        src.mat[1] = [0.0, 700.0, 240.0, 0.0];
        // 0, 0, 1, 0
        src.mat[2] = [0.0, 0.0, 1.0, 0.0];

        let original_mat2 = src.mat[2];
        let original_dist = src.dist_factor;

        ar_param_change_size(&mut src, 1280, 960).unwrap();

        assert_eq!(src.xsize, 1280);
        assert_eq!(src.ysize, 960);
        // Row 0 scaled by 2x
        assert!((src.mat[0][0] - 1400.0).abs() < 1e-9, "fx should double");
        assert!((src.mat[0][2] - 640.0).abs() < 1e-9, "cx should double");
        // Row 1 scaled by 2x
        assert!((src.mat[1][1] - 1400.0).abs() < 1e-9, "fy should double");
        assert!((src.mat[1][2] - 480.0).abs() < 1e-9, "cy should double");
        // Row 2 unchanged
        assert_eq!(src.mat[2], original_mat2);
        // Distortion factors unchanged
        assert_eq!(src.dist_factor, original_dist);
    }

    #[test]
    fn test_ar_param_change_size_halves_resolution() {
        let mut src = ARParam {
            xsize: 1280,
            ysize: 960,
            ..Default::default()
        };
        src.mat[0] = [1400.0, 0.0, 640.0, 0.0];
        src.mat[1] = [0.0, 1400.0, 480.0, 0.0];
        src.mat[2] = [0.0, 0.0, 1.0, 0.0];

        ar_param_change_size(&mut src, 640, 480).unwrap();

        assert_eq!(src.xsize, 640);
        assert_eq!(src.ysize, 480);
        assert!((src.mat[0][0] - 700.0).abs() < 1e-9);
        assert!((src.mat[1][1] - 700.0).abs() < 1e-9);
    }

    #[test]
    fn test_ar_param_change_size_identity() {
        let mut src = ARParam {
            xsize: 640,
            ysize: 480,
            ..Default::default()
        };
        src.mat[0] = [700.0, 0.0, 320.0, 0.0];
        src.mat[1] = [0.0, 700.0, 240.0, 0.0];
        src.mat[2] = [0.0, 0.0, 1.0, 0.0];

        let original_mat = src.mat;

        ar_param_change_size(&mut src, 640, 480).unwrap();

        assert_eq!(src.mat, original_mat);
        assert_eq!(src.xsize, 640);
        assert_eq!(src.ysize, 480);
    }

    #[test]
    fn test_ar_param_change_size_zero_src_dims_returns_err() {
        let mut src = ARParam::default(); // xsize = ysize = 0
        assert!(ar_param_change_size(&mut src, 640, 480).is_err());
    }
}
