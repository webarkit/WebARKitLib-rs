/*
 *  param_gl.rs
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

//! OpenGL / projection helpers for ARToolKit camera parameters.
//!
//! C equivalent: `param.h` / `paramGL.c` in ARToolKit / ARToolKitX.
//!
//! This module converts `ARParam` calibration data into OpenGL-compatible
//! projection matrices so that virtual objects can be rendered with the
//! correct perspective matching the physical camera.

use crate::arlog_e;
use crate::types::{ARParam, ARdouble};

/// Compute an OpenGL right-handed frustum projection matrix from calibrated
/// camera parameters.
///
/// C equivalent: `arglCameraFrustumRH` in `paramGL.c`
///
/// Converts a calibrated `ARParam` camera model into a 4×4 column-major
/// OpenGL projection matrix suitable for right-handed coordinate systems.
/// The result can be passed directly to `glUniformMatrix4fv` with
/// `transpose = GL_FALSE`.
///
/// ## Matrix layout
///
/// The 16-element array is stored **column-major** (OpenGL convention),
/// i.e. `m[col * 4 + row]`:
///
/// ```text
/// [ 2fx/w       0      2cx/w − 1           0          ]
/// [   0      −2fy/h   1 − 2cy/h            0          ]
/// [   0         0   (f+n)/(n−f)   2fn/(n−f)           ]
/// [   0         0        −1                0          ]
/// ```
///
/// where `w = xsize − 1`, `h = ysize − 1`, `n = focal_min`,
/// `f = focal_max`, and `fx`, `fy`, `cx`, `cy` are the camera intrinsics
/// normalised by `mat[2][2]`.
///
/// ## Assumption
///
/// This implementation assumes the `ARParam::mat` is already in **standard
/// intrinsic form**:
///
/// ```text
/// mat = [ fx   0   cx   0 ]
///       [  0  fy   cy   0 ]
///       [  0   0    1   0 ]
/// ```
///
/// This is always the case for camera parameter files produced by
/// ARToolKit / ARToolKitX calibration tools.  If you have a general
/// projective matrix you will need to decompose it first (e.g. via
/// `arParamDecompMat`).
///
/// ## Y-axis convention
///
/// ARToolKit image coordinates place the origin at the **top-left** with y
/// increasing downward, while OpenGL places the origin at the
/// **bottom-left** with y increasing upward.  The y-axis flip is baked into
/// the returned matrix (row 1 uses `−fy` and `1 − 2cy/h`).
///
/// # Arguments
///
/// * `cparam`    — Calibrated camera parameters.
/// * `focal_min` — Near clipping plane distance in world units (must be > 0).
/// * `focal_max` — Far clipping plane distance in world units (must be > `focal_min`).
///
/// # Returns
///
/// A 16-element array containing the 4×4 projection matrix in column-major
/// order, or an `Err` string if the inputs are invalid.
///
/// # Errors
///
/// * `cparam.xsize ≤ 1` or `cparam.ysize ≤ 1` — image is too small.
/// * `cparam.mat[2][2] == 0.0` — degenerate camera matrix.
/// * `focal_min ≤ 0.0` — near plane must be positive.
/// * `focal_max ≤ focal_min` — far plane must be beyond the near plane.
///
/// # References
///
/// * ARToolKit `paramGL.c` → `arglCameraFrustumRH()`
/// * [WebARKitLib param.h](https://github.com/webarkit/WebARKitLib/blob/master/include/ARX/AR/param.h)
///
/// # Example
///
/// ```rust,no_run
/// use webarkitlib_rs::ar::param_gl::argl_camera_frustum_rh;
/// use webarkitlib_rs::types::ARParam;
///
/// let mut cparam = ARParam::default();
/// cparam.xsize = 640;
/// cparam.ysize = 480;
/// cparam.mat[0][0] = 700.0; // fx
/// cparam.mat[1][1] = 700.0; // fy
/// cparam.mat[0][2] = 320.0; // cx
/// cparam.mat[1][2] = 240.0; // cy
/// cparam.mat[2][2] = 1.0;
///
/// let m = argl_camera_frustum_rh(&cparam, 10.0, 10000.0).unwrap();
/// // m is now a 4×4 OpenGL right-handed projection matrix (column-major).
/// ```
pub fn argl_camera_frustum_rh(
    cparam: &ARParam,
    focal_min: ARdouble,
    focal_max: ARdouble,
) -> Result<[ARdouble; 16], &'static str> {
    if cparam.xsize <= 1 || cparam.ysize <= 1 {
        arlog_e!(
            "argl_camera_frustum_rh: invalid image dimensions {}x{} (must be > 1)",
            cparam.xsize,
            cparam.ysize
        );
        return Err("argl_camera_frustum_rh: image dimensions must be > 1");
    }
    if focal_min <= 0.0 {
        arlog_e!(
            "argl_camera_frustum_rh: focal_min must be > 0, got {}",
            focal_min
        );
        return Err("argl_camera_frustum_rh: focal_min must be > 0");
    }
    if focal_max <= focal_min {
        arlog_e!(
            "argl_camera_frustum_rh: focal_max ({}) must be > focal_min ({})",
            focal_max,
            focal_min
        );
        return Err("argl_camera_frustum_rh: focal_max must be > focal_min");
    }

    let m22 = cparam.mat[2][2];
    if m22 == 0.0 {
        arlog_e!("argl_camera_frustum_rh: mat[2][2] is zero — degenerate camera matrix");
        return Err("argl_camera_frustum_rh: mat[2][2] is zero");
    }

    // Pixel-space widths (ARToolKit uses xsize-1 / ysize-1 as the range).
    let w = (cparam.xsize - 1) as ARdouble;
    let h = (cparam.ysize - 1) as ARdouble;

    // Normalise camera intrinsics by mat[2][2] (= 1 for standard ARToolKit
    // calibration files, but we handle the general non-unit case).
    let fx = cparam.mat[0][0] / m22;
    let fy = cparam.mat[1][1] / m22;
    let cx = cparam.mat[0][2] / m22;
    let cy = cparam.mat[1][2] / m22;

    // Y-axis flip: image coords (y↓) → OpenGL NDC coords (y↑).
    // Equivalent to what arParamDecompMat + the y-flip loop in the C source
    // produce for a standard intrinsic-only camera matrix.
    let neg_fy = -fy;
    let cy_rh = h - cy; // flipped principal-point y

    let near = focal_min;
    let far = focal_max;
    // Depth terms for the right-handed frustum (same signs as the C source).
    let depth_a = (far + near) / (near - far);
    let depth_b = 2.0 * far * near / (near - far);

    // Build the 4×4 matrix in column-major order.
    // Conceptual row-major layout:
    //   row 0: [ 2fx/w,        0,    2cx/w − 1,       0 ]
    //   row 1: [    0,    −2fy/h,   1 − 2cy/h,        0 ]
    //   row 2: [    0,         0,    depth_a,    depth_b ]
    //   row 3: [    0,         0,         −1,          0 ]
    //
    // Column-major storage: m[col * 4 + row].
    let mut m = [0.0_f64; 16];

    // Column 0
    m[0] = 2.0 * fx / w; // row 0
                         // m[1..3] = 0

    // Column 1
    // m[4] = 0 // row 0
    m[5] = 2.0 * neg_fy / h; // row 1
                             // m[6..7] = 0

    // Column 2
    m[8] = 2.0 * cx / w - 1.0; // row 0
    m[9] = 2.0 * cy_rh / h - 1.0; // row 1
    m[10] = depth_a; // row 2
    m[11] = -1.0; // row 3 (perspective divide: w_clip = −z_eye)

    // Column 3
    // m[12..13] = 0
    m[14] = depth_b; // row 2
                     // m[15] = 0

    Ok(m)
}

/// Build an OpenGL right-handed model-view matrix from an ARToolKit pose matrix.
///
/// C equivalent: `arglCameraViewRH` in `paramGL.c`
///
/// Converts a 3×4 ARToolKit pose matrix (camera-from-world) returned by
/// `ar_get_trans_mat_square` into a 4×4 column-major OpenGL model-view matrix
/// for right-handed coordinate systems.
///
/// ARToolKit's camera convention places the origin top-left with Y↓ and
/// Z pointing into the scene.  OpenGL places the origin bottom-left with Y↑
/// and Z pointing out.  Rows 1 and 2 of `para` are therefore negated.
///
/// The optional `scale` parameter scales the translation column
/// (`m[12]`, `m[13]`, `m[14]`).  Pass `1.0` for no change; `0.0` disables
/// scaling (matches the C guard `if (scale != 0.0)`).
///
/// # Arguments
///
/// * `para`  — 3×4 pose matrix from `ar_get_trans_mat_square`.
/// * `scale` — World-unit scale applied to the translation column;
///             `0.0` leaves the translation unchanged.
///
/// # Returns
///
/// A 16-element column-major OpenGL model-view matrix.
///
/// # Example
///
/// ```rust
/// use webarkitlib_rs::ar::param_gl::argl_camera_view_rh;
///
/// let para = [
///     [1.0_f64, 0.0, 0.0, 50.0],
///     [0.0,     1.0, 0.0,  0.0],
///     [0.0,     0.0, 1.0, 100.0],
/// ];
/// let mv = argl_camera_view_rh(&para, 1.0);
/// assert_eq!(mv[15], 1.0); // bottom-right of homogeneous row
/// ```
pub fn argl_camera_view_rh(para: &[[ARdouble; 4]; 3], scale: ARdouble) -> [ARdouble; 16] {
    let mut m = [0.0_f64; 16];

    // Column-major layout: m[col * 4 + row]
    // Row 0 — kept as-is
    m[0] = para[0][0]; // col 0, row 0  (R1C1)
    m[4] = para[0][1]; // col 1, row 0  (R1C2)
    m[8] = para[0][2]; // col 2, row 0  (R1C3)
    m[12] = para[0][3]; // col 3, row 0  (tx)

    // Rows 1 and 2 — negated (Y/Z axis flip for OpenGL convention)
    m[1] = -para[1][0]; // col 0, row 1
    m[5] = -para[1][1]; // col 1, row 1
    m[9] = -para[1][2]; // col 2, row 1
    m[13] = -para[1][3]; // col 3, row 1

    m[2] = -para[2][0]; // col 0, row 2
    m[6] = -para[2][1]; // col 1, row 2
    m[10] = -para[2][2]; // col 2, row 2
    m[14] = -para[2][3]; // col 3, row 2

    // Row 3 — homogeneous row
    m[3] = 0.0; // col 0, row 3
    m[7] = 0.0; // col 1, row 3
    m[11] = 0.0; // col 2, row 3
    m[15] = 1.0; // col 3, row 3

    // Apply scale to the translation column (same guard as the C source).
    if scale != 0.0 {
        m[12] *= scale; // tx
        m[13] *= scale; // ty
        m[14] *= scale; // tz
    }

    m
}

/// Single-precision variant of [`argl_camera_frustum_rh`].
///
/// C equivalent: `arglCameraFrustumRHf` in `paramGL.c`
///
/// Identical logic to [`argl_camera_frustum_rh`] but uses `f32` throughout,
/// required for APIs (WebGL, Metal, most mobile GL drivers) that only accept
/// 32-bit matrices.  The `ARParam::mat` entries are always `f64`; they are
/// cast to `f32` internally, matching the C source which reads from the same
/// `ARParam` struct regardless of compile-time float precision.
///
/// # Arguments
///
/// * `cparam`    — Calibrated camera parameters.
/// * `focal_min` — Near clipping plane distance (must be > 0).
/// * `focal_max` — Far clipping plane distance (must be > `focal_min`).
///
/// # Returns
///
/// A 16-element `f32` array (column-major), or an `Err` string on bad input.
///
/// # Errors
///
/// Same conditions as [`argl_camera_frustum_rh`].
///
/// # Example
///
/// ```rust
/// use webarkitlib_rs::ar::param_gl::argl_camera_frustum_rh_f;
/// use webarkitlib_rs::types::ARParam;
///
/// let mut cparam = ARParam::default();
/// cparam.xsize = 640;
/// cparam.ysize = 480;
/// cparam.mat[0][0] = 700.0;
/// cparam.mat[1][1] = 700.0;
/// cparam.mat[0][2] = 320.0;
/// cparam.mat[1][2] = 240.0;
/// cparam.mat[2][2] = 1.0;
///
/// let m = argl_camera_frustum_rh_f(&cparam, 10.0_f32, 10_000.0_f32).unwrap();
/// assert_eq!(m[11], -1.0_f32);
/// ```
pub fn argl_camera_frustum_rh_f(
    cparam: &ARParam,
    focal_min: f32,
    focal_max: f32,
) -> Result<[f32; 16], &'static str> {
    if cparam.xsize <= 1 || cparam.ysize <= 1 {
        arlog_e!(
            "argl_camera_frustum_rh_f: invalid image dimensions {}x{} (must be > 1)",
            cparam.xsize,
            cparam.ysize
        );
        return Err("argl_camera_frustum_rh_f: image dimensions must be > 1");
    }
    if focal_min <= 0.0 {
        arlog_e!(
            "argl_camera_frustum_rh_f: focal_min must be > 0, got {}",
            focal_min
        );
        return Err("argl_camera_frustum_rh_f: focal_min must be > 0");
    }
    if focal_max <= focal_min {
        arlog_e!(
            "argl_camera_frustum_rh_f: focal_max ({}) must be > focal_min ({})",
            focal_max,
            focal_min
        );
        return Err("argl_camera_frustum_rh_f: focal_max must be > focal_min");
    }

    let m22 = cparam.mat[2][2] as f32;
    if m22 == 0.0 {
        arlog_e!("argl_camera_frustum_rh_f: mat[2][2] is zero — degenerate camera matrix");
        return Err("argl_camera_frustum_rh_f: mat[2][2] is zero");
    }

    let w = (cparam.xsize - 1) as f32;
    let h = (cparam.ysize - 1) as f32;

    let fx = cparam.mat[0][0] as f32 / m22;
    let fy = cparam.mat[1][1] as f32 / m22;
    let cx = cparam.mat[0][2] as f32 / m22;
    let cy = cparam.mat[1][2] as f32 / m22;

    let neg_fy = -fy;
    let cy_rh = h - cy; // Y-axis flip: image (y↓) → OpenGL NDC (y↑)

    let near = focal_min;
    let far = focal_max;
    let depth_a = (far + near) / (near - far);
    let depth_b = 2.0_f32 * far * near / (near - far);

    let mut m = [0.0_f32; 16];

    // Column 0
    m[0] = 2.0_f32 * fx / w;

    // Column 1
    m[5] = 2.0_f32 * neg_fy / h;

    // Column 2
    m[8] = 2.0_f32 * cx / w - 1.0_f32;
    m[9] = 2.0_f32 * cy_rh / h - 1.0_f32;
    m[10] = depth_a;
    m[11] = -1.0_f32;

    // Column 3
    m[14] = depth_b;

    Ok(m)
}

/// Single-precision variant of [`argl_camera_view_rh`].
///
/// C equivalent: `arglCameraViewRHf` in `paramGL.c`
///
/// Identical to [`argl_camera_view_rh`] but accepts and returns `f32` values,
/// matching the `#ifndef ARDOUBLE_IS_FLOAT` branch in the original C source.
///
/// # Arguments
///
/// * `para`  — 3×4 pose matrix (f32 entries).
/// * `scale` — Translation-column scale; `0.0` means no scale.
///
/// # Returns
///
/// A 16-element `f32` column-major OpenGL model-view matrix.
///
/// # Example
///
/// ```rust
/// use webarkitlib_rs::ar::param_gl::argl_camera_view_rh_f;
///
/// let para = [
///     [1.0_f32, 0.0, 0.0, 50.0],
///     [0.0,     1.0, 0.0,  0.0],
///     [0.0,     0.0, 1.0, 100.0],
/// ];
/// let mv = argl_camera_view_rh_f(&para, 1.0);
/// assert_eq!(mv[15], 1.0_f32);
/// ```
pub fn argl_camera_view_rh_f(para: &[[f32; 4]; 3], scale: f32) -> [f32; 16] {
    let mut m = [0.0_f32; 16];

    // Column-major layout: m[col * 4 + row]
    // Row 0 — kept as-is
    m[0] = para[0][0]; // col 0, row 0  (R1C1)
    m[4] = para[0][1]; // col 1, row 0  (R1C2)
    m[8] = para[0][2]; // col 2, row 0  (R1C3)
    m[12] = para[0][3]; // col 3, row 0  (tx)

    // Rows 1 and 2 — negated
    m[1] = -para[1][0]; // col 0, row 1
    m[5] = -para[1][1]; // col 1, row 1
    m[9] = -para[1][2]; // col 2, row 1
    m[13] = -para[1][3]; // col 3, row 1

    m[2] = -para[2][0]; // col 0, row 2
    m[6] = -para[2][1]; // col 1, row 2
    m[10] = -para[2][2]; // col 2, row 2
    m[14] = -para[2][3]; // col 3, row 2

    // Row 3 — homogeneous row
    m[3] = 0.0; // col 0, row 3
    m[7] = 0.0; // col 1, row 3
    m[11] = 0.0; // col 2, row 3
    m[15] = 1.0; // col 3, row 3

    if scale != 0.0 {
        m[12] *= scale;
        m[13] *= scale;
        m[14] *= scale;
    }

    m
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::ARParam;

    fn make_param(xsize: i32, ysize: i32, fx: f64, fy: f64, cx: f64, cy: f64) -> ARParam {
        let mut p = ARParam::default();
        p.xsize = xsize;
        p.ysize = ysize;
        p.mat[0] = [fx, 0.0, cx, 0.0];
        p.mat[1] = [0.0, fy, cy, 0.0];
        p.mat[2] = [0.0, 0.0, 1.0, 0.0];
        p
    }

    // ------------------------------------------------------------------ //
    // Happy-path: structural properties of the returned matrix            //
    // ------------------------------------------------------------------ //

    #[test]
    fn test_frustum_rh_returns_ok_for_valid_params() {
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        assert!(argl_camera_frustum_rh(&p, 10.0, 10_000.0).is_ok());
    }

    #[test]
    fn test_frustum_rh_homogeneous_column_is_minus_one() {
        // The right-handed perspective divide is encoded as m[11] = −1.
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        let m = argl_camera_frustum_rh(&p, 10.0, 10_000.0).unwrap();
        assert_eq!(m[11], -1.0, "m[11] must be −1 for RH perspective");
    }

    #[test]
    fn test_frustum_rh_zeros_in_expected_positions() {
        // All off-diagonal zeros must be present.
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        let m = argl_camera_frustum_rh(&p, 10.0, 10_000.0).unwrap();
        for idx in [1, 2, 3, 4, 6, 7, 12, 13, 15] {
            assert_eq!(m[idx], 0.0, "m[{idx}] should be 0, got {}", m[idx]);
        }
    }

    #[test]
    fn test_frustum_rh_fx_scales_correctly() {
        // m[0] = 2*fx / (xsize-1)
        let fx = 700.0_f64;
        let xsize = 640;
        let p = make_param(xsize, 480, fx, 700.0, 320.0, 240.0);
        let m = argl_camera_frustum_rh(&p, 10.0, 10_000.0).unwrap();
        let expected = 2.0 * fx / (xsize - 1) as f64;
        assert!(
            (m[0] - expected).abs() < 1e-12,
            "m[0]={} expected={}",
            m[0],
            expected
        );
    }

    #[test]
    fn test_frustum_rh_fy_is_negated() {
        // m[5] = 2 * (−fy) / (ysize−1) — must be negative for positive fy.
        let fy = 700.0_f64;
        let ysize = 480;
        let p = make_param(640, ysize, 700.0, fy, 320.0, 240.0);
        let m = argl_camera_frustum_rh(&p, 10.0, 10_000.0).unwrap();
        let expected = 2.0 * (-fy) / (ysize - 1) as f64;
        assert!(
            (m[5] - expected).abs() < 1e-12,
            "m[5]={} expected={} (y-flip)",
            m[5],
            expected
        );
        assert!(m[5] < 0.0, "m[5] must be negative (y-axis flip)");
    }

    #[test]
    fn test_frustum_rh_depth_terms() {
        // m[10] = (far+near)/(near-far), m[14] = 2*near*far/(near-far)
        let near = 10.0_f64;
        let far = 10_000.0_f64;
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        let m = argl_camera_frustum_rh(&p, near, far).unwrap();
        let expected_10 = (far + near) / (near - far);
        let expected_14 = 2.0 * far * near / (near - far);
        assert!((m[10] - expected_10).abs() < 1e-10);
        assert!((m[14] - expected_14).abs() < 1e-10);
    }

    // ------------------------------------------------------------------ //
    // Error cases                                                          //
    // ------------------------------------------------------------------ //

    #[test]
    fn test_frustum_rh_error_small_xsize() {
        let p = make_param(1, 480, 700.0, 700.0, 320.0, 240.0);
        assert!(argl_camera_frustum_rh(&p, 10.0, 10_000.0).is_err());
    }

    #[test]
    fn test_frustum_rh_error_small_ysize() {
        let p = make_param(640, 1, 700.0, 700.0, 320.0, 240.0);
        assert!(argl_camera_frustum_rh(&p, 10.0, 10_000.0).is_err());
    }

    #[test]
    fn test_frustum_rh_error_non_positive_focal_min() {
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        assert!(argl_camera_frustum_rh(&p, 0.0, 10_000.0).is_err());
        assert!(argl_camera_frustum_rh(&p, -1.0, 10_000.0).is_err());
    }

    #[test]
    fn test_frustum_rh_error_focal_max_le_focal_min() {
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        assert!(argl_camera_frustum_rh(&p, 100.0, 100.0).is_err());
        assert!(argl_camera_frustum_rh(&p, 100.0, 50.0).is_err());
    }

    #[test]
    fn test_frustum_rh_error_zero_mat22() {
        let mut p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        p.mat[2][2] = 0.0;
        assert!(argl_camera_frustum_rh(&p, 10.0, 10_000.0).is_err());
    }

    // ------------------------------------------------------------------ //
    // argl_camera_view_rh                                                  //
    // ------------------------------------------------------------------ //

    fn identity_para() -> [[f64; 4]; 3] {
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ]
    }

    #[test]
    fn test_camera_view_rh_homogeneous_row() {
        let m = argl_camera_view_rh(&identity_para(), 1.0);
        // Row 3 must be [0, 0, 0, 1] (column-major: m[3], m[7], m[11], m[15])
        assert_eq!(m[3], 0.0);
        assert_eq!(m[7], 0.0);
        assert_eq!(m[11], 0.0);
        assert_eq!(m[15], 1.0);
    }

    #[test]
    fn test_camera_view_rh_row0_unchanged() {
        let para = [
            [2.0, 3.0, 4.0, 5.0],
            [1.0, 1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0, 1.0],
        ];
        let m = argl_camera_view_rh(&para, 1.0);
        // Row 0, columns 0-3 (column-major: m[0], m[4], m[8], m[12])
        assert_eq!(m[0], 2.0);
        assert_eq!(m[4], 3.0);
        assert_eq!(m[8], 4.0);
        assert_eq!(m[12], 5.0);
    }

    #[test]
    fn test_camera_view_rh_rows_1_2_negated() {
        let para = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 2.0, 0.0, 6.0],
            [0.0, 0.0, 3.0, 7.0],
        ];
        let m = argl_camera_view_rh(&para, 1.0);
        // Row 1, col 1 (m[1 + 1*4] = m[5]) must be −2.0
        assert_eq!(m[5], -2.0);
        // Row 1, col 3 (m[1 + 3*4] = m[13]) must be −6.0
        assert_eq!(m[13], -6.0);
        // Row 2, col 2 (m[2 + 2*4] = m[10]) must be −3.0
        assert_eq!(m[10], -3.0);
        // Row 2, col 3 (m[2 + 3*4] = m[14]) must be −7.0
        assert_eq!(m[14], -7.0);
    }

    #[test]
    fn test_camera_view_rh_scale_applied_to_translation() {
        let para = [
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 1.0, 0.0, 20.0],
            [0.0, 0.0, 1.0, 30.0],
        ];
        let m = argl_camera_view_rh(&para, 2.0);
        // Translation column after scale×2: m[12]=10*2, m[13]=−20*2, m[14]=−30*2
        assert!((m[12] - 20.0).abs() < 1e-12);
        assert!((m[13] - (-40.0)).abs() < 1e-12);
        assert!((m[14] - (-60.0)).abs() < 1e-12);
    }

    #[test]
    fn test_camera_view_rh_scale_zero_no_change() {
        let para = [
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 1.0, 0.0, 20.0],
            [0.0, 0.0, 1.0, 30.0],
        ];
        // scale==0 disables the scale step — translation stays raw (negated for rows 1&2)
        let m = argl_camera_view_rh(&para, 0.0);
        assert!((m[12] - 10.0).abs() < 1e-12);
        assert!((m[13] - (-20.0)).abs() < 1e-12);
        assert!((m[14] - (-30.0)).abs() < 1e-12);
    }

    // ------------------------------------------------------------------ //
    // argl_camera_frustum_rh_f                                             //
    // ------------------------------------------------------------------ //

    #[test]
    fn test_frustum_rh_f_returns_ok_for_valid_params() {
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        assert!(argl_camera_frustum_rh_f(&p, 10.0_f32, 10_000.0_f32).is_ok());
    }

    #[test]
    fn test_frustum_rh_f_homogeneous_column_is_minus_one() {
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        let m = argl_camera_frustum_rh_f(&p, 10.0, 10_000.0).unwrap();
        assert_eq!(m[11], -1.0_f32, "m[11] must be −1 for RH perspective");
    }

    #[test]
    fn test_frustum_rh_f_zeros_in_expected_positions() {
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        let m = argl_camera_frustum_rh_f(&p, 10.0, 10_000.0).unwrap();
        for idx in [1, 2, 3, 4, 6, 7, 12, 13, 15] {
            assert_eq!(m[idx], 0.0_f32, "m[{idx}] should be 0, got {}", m[idx]);
        }
    }

    #[test]
    fn test_frustum_rh_f_fy_is_negated() {
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        let m = argl_camera_frustum_rh_f(&p, 10.0, 10_000.0).unwrap();
        assert!(m[5] < 0.0_f32, "m[5] must be negative (y-axis flip)");
    }

    #[test]
    fn test_frustum_rh_f_error_small_image() {
        let p = make_param(1, 480, 700.0, 700.0, 320.0, 240.0);
        assert!(argl_camera_frustum_rh_f(&p, 10.0, 10_000.0).is_err());
    }

    #[test]
    fn test_frustum_rh_f_error_non_positive_focal_min() {
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        assert!(argl_camera_frustum_rh_f(&p, 0.0, 10_000.0).is_err());
    }

    #[test]
    fn test_frustum_rh_f_error_focal_max_le_focal_min() {
        let p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        assert!(argl_camera_frustum_rh_f(&p, 100.0, 100.0).is_err());
    }

    #[test]
    fn test_frustum_rh_f_error_zero_mat22() {
        let mut p = make_param(640, 480, 700.0, 700.0, 320.0, 240.0);
        p.mat[2][2] = 0.0;
        assert!(argl_camera_frustum_rh_f(&p, 10.0, 10_000.0).is_err());
    }

    // ------------------------------------------------------------------ //
    // argl_camera_view_rh_f                                                //
    // ------------------------------------------------------------------ //

    fn identity_para_f() -> [[f32; 4]; 3] {
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ]
    }

    #[test]
    fn test_camera_view_rh_f_homogeneous_row() {
        let m = argl_camera_view_rh_f(&identity_para_f(), 1.0);
        assert_eq!(m[3], 0.0_f32);
        assert_eq!(m[7], 0.0_f32);
        assert_eq!(m[11], 0.0_f32);
        assert_eq!(m[15], 1.0_f32);
    }

    #[test]
    fn test_camera_view_rh_f_rows_1_2_negated() {
        let para: [[f32; 4]; 3] = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 2.0, 0.0, 6.0],
            [0.0, 0.0, 3.0, 7.0],
        ];
        let m = argl_camera_view_rh_f(&para, 1.0);
        assert_eq!(m[5], -2.0_f32);
        assert_eq!(m[13], -6.0_f32);
        assert_eq!(m[10], -3.0_f32);
        assert_eq!(m[14], -7.0_f32);
    }

    #[test]
    fn test_camera_view_rh_f_scale_applied_to_translation() {
        let para: [[f32; 4]; 3] = [
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 1.0, 0.0, 20.0],
            [0.0, 0.0, 1.0, 30.0],
        ];
        let m = argl_camera_view_rh_f(&para, 2.0);
        assert!((m[12] - 20.0_f32).abs() < 1e-6);
        assert!((m[13] - (-40.0_f32)).abs() < 1e-6);
        assert!((m[14] - (-60.0_f32)).abs() < 1e-6);
    }

    #[test]
    fn test_camera_view_rh_f_scale_zero_no_change() {
        let para: [[f32; 4]; 3] = [
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 1.0, 0.0, 20.0],
            [0.0, 0.0, 1.0, 30.0],
        ];
        let m = argl_camera_view_rh_f(&para, 0.0);
        assert!((m[12] - 10.0_f32).abs() < 1e-6);
        assert!((m[13] - (-20.0_f32)).abs() < 1e-6);
        assert!((m[14] - (-30.0_f32)).abs() < 1e-6);
    }
}
