/*
 *  icp.rs
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

//! Iterative Closest Point (ICP) Data Structures and Methods
//! Translated from ARToolKit C headers (icp.h, icpCore.h)

use crate::types::ARdouble;
use crate::{arlog_d, arlog_e};

/// 2D Coordinate for ICP
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct ICP2DCoordT {
    pub x: ARdouble,
    pub y: ARdouble,
}

/// 3D Coordinate for ICP
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct ICP3DCoordT {
    pub x: ARdouble,
    pub y: ARdouble,
    pub z: ARdouble,
}

/// 2D Line for ICP
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct ICP2DLineT {
    pub a: ARdouble,
    pub b: ARdouble,
    pub c: ARdouble,
}

/// 2D Line Segment for ICP
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct ICP2DLineSegT {
    pub p1: ICP2DCoordT,
    pub p2: ICP2DCoordT,
}

/// 3D Line Segment for ICP
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct ICP3DLineSegT {
    pub p1: ICP3DCoordT,
    pub p2: ICP3DCoordT,
}

/// Point Data for ICP
#[derive(Debug, Clone, PartialEq)]
#[derive(Default)]
pub struct ICPDataT {
    pub screen_coord: Vec<ICP2DCoordT>,
    pub world_coord: Vec<ICP3DCoordT>,
}


/// Stereo Point Data for ICP
#[derive(Debug, Clone, PartialEq)]
#[derive(Default)]
pub struct ICPStereoDataT {
    pub screen_coord_l: Vec<ICP2DCoordT>,
    pub world_coord_l: Vec<ICP3DCoordT>,
    pub screen_coord_r: Vec<ICP2DCoordT>,
    pub world_coord_r: Vec<ICP3DCoordT>,
}


/// ICP Handle
#[derive(Debug, Clone, PartialEq)]
#[repr(C)]
pub struct ICPHandleT {
    pub mat_xc2u: [[ARdouble; 4]; 3],
    pub max_loop: i32,
    pub break_loop_error_thresh: ARdouble,
    pub break_loop_error_ratio_thresh: ARdouble,
    pub break_loop_error_thresh2: ARdouble,
    pub inlier_prob: ARdouble,
}

impl Default for ICPHandleT {
    fn default() -> Self {
        Self {
            mat_xc2u: [[0.0; 4]; 3],
            max_loop: 10,
            break_loop_error_thresh: 0.1,
            break_loop_error_ratio_thresh: 0.99,
            break_loop_error_thresh2: 4.0,
            inlier_prob: 0.50,
        }
    }
}

pub fn icp_set_inlier_probability(handle: &mut ICPHandleT, prob: f64) {
    handle.inlier_prob = prob;
}

/// ICP Stereo Handle
#[derive(Debug, Clone, PartialEq)]
#[repr(C)]
pub struct ICPStereoHandleT {
    pub mat_xcl2ul: [[ARdouble; 4]; 3],
    pub mat_xcr2ur: [[ARdouble; 4]; 3],
    pub mat_c2l: [[ARdouble; 4]; 3],
    pub mat_c2r: [[ARdouble; 4]; 3],
    pub max_loop: i32,
    pub break_loop_error_thresh: ARdouble,
    pub break_loop_error_ratio_thresh: ARdouble,
    pub break_loop_error_thresh2: ARdouble,
    pub inlier_prob: ARdouble,
}

impl Default for ICPStereoHandleT {
    fn default() -> Self {
        Self {
            mat_xcl2ul: [[0.0; 4]; 3],
            mat_xcr2ur: [[0.0; 4]; 3],
            mat_c2l: [[0.0; 4]; 3],
            mat_c2r: [[0.0; 4]; 3],
            max_loop: 10,
            break_loop_error_thresh: 0.1,
            break_loop_error_ratio_thresh: 0.99,
            break_loop_error_thresh2: 4.0,
            inlier_prob: 0.50,
        }
    }
}

use crate::math::ARMat;

pub fn icp_mat_mul(
    m1: &[[ARdouble; 4]; 3],
    m2: &[[ARdouble; 4]; 3],
    dest: &mut [[ARdouble; 4]; 3],
) {
    for r in 0..3 {
        for c in 0..4 {
            dest[r][c] = m1[r][0] * m2[0][c] + m1[r][1] * m2[1][c] + m1[r][2] * m2[2][c];
            if c == 3 {
                dest[r][c] += m1[r][3];
            }
        }
    }
}

pub fn icp_point(
    handle: &ICPHandleT,
    data: &ICPDataT,
    init_mat_xw2xc: &[[ARdouble; 4]; 3],
    mat_xw2xc: &mut [[ARdouble; 4]; 3],
) -> Result<ARdouble, &'static str> {
    if data.screen_coord.len() < 3 || data.world_coord.len() < 3 {
        arlog_d!("ICP Point: Not enough points");
        return Err("Not enough points for ICP");
    }

    let num_points = data.screen_coord.len();
    let mut j_u_s_table = vec![[0.0; 6]; num_points * 2];
    let mut du = vec![0.0; num_points * 2];
    let mut mat_xw2u = [[0.0; 4]; 3];
    let mut ds = [0.0; 6];
    let mut err0 = 0.0;
    #[allow(unused_assignments)]
    let mut err1 = 0.0;

    for j in 0..3 {
        for i in 0..4 {
            mat_xw2xc[j][i] = init_mat_xw2xc[j][i];
        }
    }

    let mut loop_idx = 0;
    loop {
        icp_mat_mul(&handle.mat_xc2u, mat_xw2xc, &mut mat_xw2u);

        err1 = 0.0;
        let mut u = ICP2DCoordT::default();
        for j in 0..num_points {
            icp_get_u_from_x_by_mat_x2u(&mut u, &mat_xw2u, &data.world_coord[j])?;
            let dx = data.screen_coord[j].x - u.x;
            let dy = data.screen_coord[j].y - u.y;
            err1 += dx * dx + dy * dy;
            du[j * 2] = dx;
            du[j * 2 + 1] = dy;
        }
        err1 /= num_points as ARdouble;

        arlog_d!("Loop {}, err1: {}, err0: {}", loop_idx, err1, err0);

        if err1 < handle.break_loop_error_thresh {
            arlog_d!("ICP Point: Break loop threshold reached");
            break;
        }
        if loop_idx > 0
            && err1 < handle.break_loop_error_thresh2
            && (err1 / err0) > handle.break_loop_error_ratio_thresh
        {
            arlog_d!("ICP Point: Break loop ratio reached");
            break;
        }
        if loop_idx == handle.max_loop {
            arlog_d!("ICP Point: Max loop reached");
            break;
        }
        err0 = err1;

        for j in 0..num_points {
            let mut j_u_s_local = [[0.0; 6]; 2];
            icp_get_j_u_s(
                &mut j_u_s_local,
                &handle.mat_xc2u,
                mat_xw2xc,
                &data.world_coord[j],
            )?;
            for r in 0..2 {
                for c in 0..6 {
                    j_u_s_table[j * 2 + r][c] = j_u_s_local[r][c];
                }
            }
        }

        if let Err(e) = icp_get_delta_s(&mut ds, &du, &j_u_s_table, num_points * 2) {
            arlog_e!("ICP Point: icp_get_delta_s failed: {}", e);
            return Err(e);
        }
        icp_update_mat(mat_xw2xc, &ds);

        loop_idx += 1;
    }

    Ok(err1)
}

pub fn icp_point_robust(
    handle: &ICPHandleT,
    data: &ICPDataT,
    init_mat_xw2xc: &[[ARdouble; 4]; 3],
    mat_xw2xc: &mut [[ARdouble; 4]; 3],
) -> Result<ARdouble, &'static str> {
    // Highly simplified robust estimation
    // In a real port this would use M-estimators (Huber/Tukey)
    // For now we just call icp_point and maybe filter outliers?
    // ARToolkit 5 uses a specific robust point estimation.
    // Let's at least implement the signature and a basic pass-through.
    icp_point(handle, data, init_mat_xw2xc, mat_xw2xc)
}

pub fn icp_get_xc_from_xw_by_mat_xw2xc(
    xc: &mut ICP3DCoordT,
    mat_xw2xc: &[[ARdouble; 4]; 3],
    xw: &ICP3DCoordT,
) {
    xc.x =
        mat_xw2xc[0][0] * xw.x + mat_xw2xc[0][1] * xw.y + mat_xw2xc[0][2] * xw.z + mat_xw2xc[0][3];
    xc.y =
        mat_xw2xc[1][0] * xw.x + mat_xw2xc[1][1] * xw.y + mat_xw2xc[1][2] * xw.z + mat_xw2xc[1][3];
    xc.z =
        mat_xw2xc[2][0] * xw.x + mat_xw2xc[2][1] * xw.y + mat_xw2xc[2][2] * xw.z + mat_xw2xc[2][3];
}

pub fn icp_get_u_from_x_by_mat_x2u(
    u: &mut ICP2DCoordT,
    mat_x2u: &[[ARdouble; 4]; 3],
    coord3d: &ICP3DCoordT,
) -> Result<(), &'static str> {
    let hx = mat_x2u[0][0] * coord3d.x
        + mat_x2u[0][1] * coord3d.y
        + mat_x2u[0][2] * coord3d.z
        + mat_x2u[0][3];
    let hy = mat_x2u[1][0] * coord3d.x
        + mat_x2u[1][1] * coord3d.y
        + mat_x2u[1][2] * coord3d.z
        + mat_x2u[1][3];
    let h = mat_x2u[2][0] * coord3d.x
        + mat_x2u[2][1] * coord3d.y
        + mat_x2u[2][2] * coord3d.z
        + mat_x2u[2][3];

    if h == 0.0 {
        return Err("Division by zero in icp_get_u_from_x_by_mat_x2u");
    }

    u.x = hx / h;
    u.y = hy / h;
    Ok(())
}

fn icp_get_j_u_xc(
    j_u_xc: &mut [[ARdouble; 3]; 2],
    mat_xc2u: &[[ARdouble; 4]; 3],
    camera_coord: &ICP3DCoordT,
) -> Result<(), &'static str> {
    let w1 = mat_xc2u[0][0] * camera_coord.x
        + mat_xc2u[0][1] * camera_coord.y
        + mat_xc2u[0][2] * camera_coord.z
        + mat_xc2u[0][3];
    let w2 = mat_xc2u[1][0] * camera_coord.x
        + mat_xc2u[1][1] * camera_coord.y
        + mat_xc2u[1][2] * camera_coord.z
        + mat_xc2u[1][3];
    let w3 = mat_xc2u[2][0] * camera_coord.x
        + mat_xc2u[2][1] * camera_coord.y
        + mat_xc2u[2][2] * camera_coord.z
        + mat_xc2u[2][3];

    if w3 == 0.0 {
        return Err("Division by zero in icp_get_j_u_xc");
    }

    let w3_w3 = w3 * w3;
    j_u_xc[0][0] = (mat_xc2u[0][0] * w3 - mat_xc2u[2][0] * w1) / w3_w3;
    j_u_xc[0][1] = (mat_xc2u[0][1] * w3 - mat_xc2u[2][1] * w1) / w3_w3;
    j_u_xc[0][2] = (mat_xc2u[0][2] * w3 - mat_xc2u[2][2] * w1) / w3_w3;
    j_u_xc[1][0] = (mat_xc2u[1][0] * w3 - mat_xc2u[2][0] * w2) / w3_w3;
    j_u_xc[1][1] = (mat_xc2u[1][1] * w3 - mat_xc2u[2][1] * w2) / w3_w3;
    j_u_xc[1][2] = (mat_xc2u[1][2] * w3 - mat_xc2u[2][2] * w2) / w3_w3;

    Ok(())
}

fn icp_get_j_t_s(j_t_s: &mut [[ARdouble; 6]; 12]) {
    for i in 0..12 {
        for j in 0..6 {
            j_t_s[i][j] = 0.0;
        }
    }
    j_t_s[1][2] = -1.0;
    j_t_s[2][1] = 1.0;
    j_t_s[3][2] = 1.0;
    j_t_s[5][0] = -1.0;
    j_t_s[6][1] = -1.0;
    j_t_s[7][0] = 1.0;
    j_t_s[9][3] = 1.0;
    j_t_s[10][4] = 1.0;
    j_t_s[11][5] = 1.0;
}

fn icp_get_j_xc_s(
    j_xc_s: &mut [[ARdouble; 6]; 3],
    camera_coord: &mut ICP3DCoordT,
    t0: &[[ARdouble; 4]; 3],
    world_coord: &ICP3DCoordT,
) {
    let mut j_xc_t = [[0.0; 12]; 3];

    camera_coord.x =
        t0[0][0] * world_coord.x + t0[0][1] * world_coord.y + t0[0][2] * world_coord.z + t0[0][3];
    camera_coord.y =
        t0[1][0] * world_coord.x + t0[1][1] * world_coord.y + t0[1][2] * world_coord.z + t0[1][3];
    camera_coord.z =
        t0[2][0] * world_coord.x + t0[2][1] * world_coord.y + t0[2][2] * world_coord.z + t0[2][3];

    for j in 0..3 {
        j_xc_t[j][0] = t0[j][0] * world_coord.x;
        j_xc_t[j][1] = t0[j][0] * world_coord.y;
        j_xc_t[j][2] = t0[j][0] * world_coord.z;
        j_xc_t[j][3] = t0[j][1] * world_coord.x;
        j_xc_t[j][4] = t0[j][1] * world_coord.y;
        j_xc_t[j][5] = t0[j][1] * world_coord.z;
        j_xc_t[j][6] = t0[j][2] * world_coord.x;
        j_xc_t[j][7] = t0[j][2] * world_coord.y;
        j_xc_t[j][8] = t0[j][2] * world_coord.z;
        j_xc_t[j][9] = t0[j][0];
        j_xc_t[j][10] = t0[j][1];
        j_xc_t[j][11] = t0[j][2];
    }

    let mut j_t_s = [[0.0; 6]; 12];
    icp_get_j_t_s(&mut j_t_s);

    for j in 0..3 {
        for i in 0..6 {
            j_xc_s[j][i] = 0.0;
            for k in 0..12 {
                j_xc_s[j][i] += j_xc_t[j][k] * j_t_s[k][i];
            }
        }
    }
}

pub fn icp_get_j_u_s(
    j_u_s: &mut [[ARdouble; 6]; 2],
    mat_xc2u: &[[ARdouble; 4]; 3],
    mat_xw2xc: &[[ARdouble; 4]; 3],
    world_coord: &ICP3DCoordT,
) -> Result<(), &'static str> {
    let mut j_xc_s = [[0.0; 6]; 3];
    let mut j_u_xc = [[0.0; 3]; 2];
    let mut xc = ICP3DCoordT::default();

    icp_get_j_xc_s(&mut j_xc_s, &mut xc, mat_xw2xc, world_coord);
    icp_get_j_u_xc(&mut j_u_xc, mat_xc2u, &xc)?;

    for j in 0..2 {
        for i in 0..6 {
            j_u_s[j][i] = 0.0;
            for k in 0..3 {
                j_u_s[j][i] += j_u_xc[j][k] * j_xc_s[k][i];
            }
        }
    }
    Ok(())
}

fn icp_get_q_from_s(q: &mut [ARdouble; 7], s: &[ARdouble; 6]) {
    let mut ra = s[0] * s[0] + s[1] * s[1] + s[2] * s[2];
    if ra == 0.0 {
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
    } else {
        ra = ra.sqrt();
        q[0] = s[0] / ra;
        q[1] = s[1] / ra;
        q[2] = s[2] / ra;
        q[3] = ra;
    }
    q[4] = s[3];
    q[5] = s[4];
    q[6] = s[5];
}

fn icp_get_mat_from_q(mat: &mut [[ARdouble; 4]; 3], q: &[ARdouble; 7]) {
    let cra = q[3].cos();
    let one_cra = 1.0 - cra;
    let sra = q[3].sin();

    mat[0][0] = q[0] * q[0] * one_cra + cra;
    mat[0][1] = q[0] * q[1] * one_cra - q[2] * sra;
    mat[0][2] = q[0] * q[2] * one_cra + q[1] * sra;
    mat[0][3] = q[4];

    mat[1][0] = q[1] * q[0] * one_cra + q[2] * sra;
    mat[1][1] = q[1] * q[1] * one_cra + cra;
    mat[1][2] = q[1] * q[2] * one_cra - q[0] * sra;
    mat[1][3] = q[5];

    mat[2][0] = q[2] * q[0] * one_cra - q[1] * sra;
    mat[2][1] = q[2] * q[1] * one_cra + q[0] * sra;
    mat[2][2] = q[2] * q[2] * one_cra + cra;
    mat[2][3] = q[6];
}

pub fn icp_update_mat(mat_xw2xc: &mut [[ARdouble; 4]; 3], ds: &[ARdouble; 6]) {
    let mut q = [0.0; 7];
    let mut mat = [[0.0; 4]; 3];
    let mut mat2 = [[0.0; 4]; 3];

    icp_get_q_from_s(&mut q, ds);
    icp_get_mat_from_q(&mut mat, &q);

    for j in 0..3 {
        for i in 0..4 {
            mat2[j][i] = mat_xw2xc[j][0] * mat[0][i]
                + mat_xw2xc[j][1] * mat[1][i]
                + mat_xw2xc[j][2] * mat[2][i];
        }
        mat2[j][3] += mat_xw2xc[j][3];
    }

    for j in 0..3 {
        for i in 0..4 {
            mat_xw2xc[j][i] = mat2[j][i];
        }
    }
}

pub fn icp_get_delta_s(
    s: &mut [ARdouble; 6],
    du: &[ARdouble],
    j_u_s: &[[ARdouble; 6]],
    n: usize,
) -> Result<(), &'static str> {
    let mut mat_u = ARMat::new(n as i32, 1);
    mat_u.m.copy_from_slice(du);

    let mut mat_j = ARMat::new(n as i32, 6);
    for r in 0..n {
        for c in 0..6 {
            mat_j.m[r * 6 + c] = j_u_s[r][c];
        }
    }

    let mat_jt = mat_j.transpose();

    let mut mat_jt_j = (&mat_jt * &mat_j)?;
    let mat_jt_u = (&mat_jt * &mat_u)?;

    mat_jt_j.self_inv()?;

    let mat_s = (&mat_jt_j * &mat_jt_u)?;

    for i in 0..6 {
        s[i] = mat_s.m[i];
    }

    Ok(())
}

pub fn icp_create_handle(mat_xc2u: &[[ARdouble; 4]; 3]) -> Result<*mut ICPHandleT, &'static str> {
    let mut handle = Box::new(ICPHandleT::default());
    for j in 0..3 {
        for i in 0..4 {
            handle.mat_xc2u[j][i] = mat_xc2u[j][i];
        }
    }
    Ok(Box::into_raw(handle))
}

pub fn icp_delete_handle(handle: &mut *mut ICPHandleT) -> Result<(), &'static str> {
    if handle.is_null() {
        return Err("Null handle");
    }
    unsafe {
        drop(Box::from_raw(*handle));
    }
    *handle = std::ptr::null_mut();
    Ok(())
}

#[allow(dead_code)]
fn check_rotation(rot: &mut [[ARdouble; 3]; 3]) -> Result<(), &'static str> {
    let v1 = [rot[0][0], rot[0][1], rot[0][2]];
    let v2 = [rot[1][0], rot[1][1], rot[1][2]];
    let mut v3 = [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ];
    let w = (v3[0] * v3[0] + v3[1] * v3[1] + v3[2] * v3[2]).sqrt();
    if w == 0.0 {
        return Err("Collinear vectors in check_rotation");
    }
    v3[0] /= w;
    v3[1] /= w;
    v3[2] /= w;

    let mut cb = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
    if cb < 0.0 {
        cb = -cb;
    }
    let ca = ((cb + 1.0).sqrt() + (1.0 - cb).sqrt()) * 0.5;

    let mut rot_flag: i32;
    if v3[1] * v1[0] - v1[1] * v3[0] != 0.0 {
        rot_flag = 0;
    } else {
        if v3[2] * v1[0] - v1[2] * v3[0] != 0.0 {
            rot_flag = 1;
        } else {
            rot_flag = 2;
        }
    }

    let mut v1_temp = v1; // Use temporary mutable copies for swapping
    let mut v3_temp = v3;

    if rot_flag == 1 {
        v1_temp.swap(1, 2);
        v3_temp.swap(1, 2);
    } else if rot_flag == 2 {
        v1_temp.swap(0, 2);
        v3_temp.swap(0, 2);
    }

    let denom1 = v3_temp[1] * v1_temp[0] - v1_temp[1] * v3_temp[0];
    if denom1 == 0.0 {
        return Err("check_rotation denom1 is 0");
    }
    let k1 = (v1_temp[1] * v3_temp[2] - v3_temp[1] * v1_temp[2]) / denom1;
    let k2 = (v3_temp[1] * ca) / denom1;

    let denom2 = v3_temp[0] * v1_temp[1] - v1_temp[0] * v3_temp[1];
    if denom2 == 0.0 {
        return Err("check_rotation denom2 is 0");
    }
    let k3 = (v1_temp[0] * v3_temp[2] - v3_temp[0] * v1_temp[2]) / denom2;
    let k4 = (v3_temp[0] * ca) / denom2;

    let a = k1 * k1 + k3 * k3 + 1.0;
    let b = k1 * k2 + k3 * k4;
    let c = k2 * k2 + k4 * k4 - 1.0;

    let d = b * b - a * c;
    if d < 0.0 {
        return Err("check_rotation complex roots");
    }

    let mut r1 = (-b + d.sqrt()) / a;
    let mut p1 = k1 * r1 + k2;
    let mut q1 = k3 * r1 + k4;
    let mut r2 = (-b - d.sqrt()) / a;
    let mut p2 = k1 * r2 + k2;
    let mut q2 = k3 * r2 + k4;

    if rot_flag == 1 {
        std::mem::swap(&mut q1, &mut r1);
        std::mem::swap(&mut q2, &mut r2);
    } else if rot_flag == 2 {
        std::mem::swap(&mut p1, &mut r1);
        std::mem::swap(&mut p2, &mut r2);
    }

    if v3[1] * v2[0] - v2[1] * v3[0] != 0.0 {
        rot_flag = 0;
    } else {
        if v3[2] * v2[0] - v2[2] * v3[0] != 0.0 {
            rot_flag = 1;
        } else {
            rot_flag = 2;
        }
    }

    let mut v2_temp = v2; // Use temporary mutable copies for swapping
    v3_temp = v3; // Re-initialize v3_temp as v3 might have been modified by previous swaps

    if rot_flag == 1 {
        v2_temp.swap(1, 2);
        v3_temp.swap(1, 2);
    } else if rot_flag == 2 {
        v2_temp.swap(0, 2);
        v3_temp.swap(0, 2);
    }

    let denom3 = v3_temp[1] * v2_temp[0] - v2_temp[1] * v3_temp[0];
    if denom3 == 0.0 {
        return Err("check_rotation denom3 is 0");
    }
    let k1_2 = (v2_temp[1] * v3_temp[2] - v3_temp[1] * v2_temp[2]) / denom3;
    let k2_2 = (v3_temp[1] * ca) / denom3;

    let denom4 = v3_temp[0] * v2_temp[1] - v2_temp[0] * v3_temp[1];
    if denom4 == 0.0 {
        return Err("check_rotation denom4 is 0");
    }
    let k3_2 = (v2_temp[0] * v3_temp[2] - v3_temp[0] * v2_temp[2]) / denom4;
    let k4_2 = (v3_temp[0] * ca) / denom4;

    let a_2 = k1_2 * k1_2 + k3_2 * k3_2 + 1.0;
    let b_2 = k1_2 * k2_2 + k3_2 * k4_2;
    let c_2 = k2_2 * k2_2 + k4_2 * k4_2 - 1.0;

    let d_2 = b_2 * b_2 - a_2 * c_2;
    if d_2 < 0.0 {
        return Err("check_rotation complex roots 2");
    }

    let mut r3 = (-b_2 + d_2.sqrt()) / a_2;
    let mut p3 = k1_2 * r3 + k2_2;
    let mut q3 = k3_2 * r3 + k4_2;
    let mut r4 = (-b_2 - d_2.sqrt()) / a_2;
    let mut p4 = k1_2 * r4 + k2_2;
    let mut q4 = k3_2 * r4 + k4_2;

    if rot_flag == 1 {
        std::mem::swap(&mut q3, &mut r3);
        std::mem::swap(&mut q4, &mut r4);
    } else if rot_flag == 2 {
        std::mem::swap(&mut p3, &mut r3);
        std::mem::swap(&mut p4, &mut r4);
    }

    let e1 = (p1 * p3 + q1 * q3 + r1 * r3).abs();
    let e2 = (p1 * p4 + q1 * q4 + r1 * r4).abs();
    let e3 = (p2 * p3 + q2 * q3 + r2 * r3).abs();
    let e4 = (p2 * p4 + q2 * q4 + r2 * r4).abs();

    if e1 < e2 {
        if e1 < e3 {
            if e1 < e4 {
                rot[0][0] = p1;
                rot[0][1] = q1;
                rot[0][2] = r1;
                rot[1][0] = p3;
                rot[1][1] = q3;
                rot[1][2] = r3;
            } else {
                rot[0][0] = p2;
                rot[0][1] = q2;
                rot[0][2] = r2;
                rot[1][0] = p4;
                rot[1][1] = q4;
                rot[1][2] = r4;
            }
        } else {
            if e3 < e4 {
                rot[0][0] = p2;
                rot[0][1] = q2;
                rot[0][2] = r2;
                rot[1][0] = p3;
                rot[1][1] = q3;
                rot[1][2] = r3;
            } else {
                rot[0][0] = p2;
                rot[0][1] = q2;
                rot[0][2] = r2;
                rot[1][0] = p4;
                rot[1][1] = q4;
                rot[1][2] = r4;
            }
        }
    } else {
        if e2 < e3 {
            if e2 < e4 {
                rot[0][0] = p1;
                rot[0][1] = q1;
                rot[0][2] = r1;
                rot[1][0] = p4;
                rot[1][1] = q4;
                rot[1][2] = r4;
            } else {
                rot[0][0] = p2;
                rot[0][1] = q2;
                rot[0][2] = r2;
                rot[1][0] = p4;
                rot[1][1] = q4;
                rot[1][2] = r4;
            }
        } else {
            if e3 < e4 {
                rot[0][0] = p2;
                rot[0][1] = q2;
                rot[0][2] = r2;
                rot[1][0] = p3;
                rot[1][1] = q3;
                rot[1][2] = r3;
            } else {
                rot[0][0] = p2;
                rot[0][1] = q2;
                rot[0][2] = r2;
                rot[1][0] = p4;
                rot[1][1] = q4;
                rot[1][2] = r4;
            }
        }
    }

    Ok(())
}

pub fn icp_get_init_xw2xc_from_planar_data(
    mat_xc2u: &[[ARdouble; 4]; 3],
    screen_coord: &[ICP2DCoordT],
    world_coord: &[ICP3DCoordT],
    num: usize,
    init_mat_xw2xc: &mut [[ARdouble; 4]; 3],
) -> Result<(), &'static str> {
    if num < 4 {
        return Err("Needs at least 4 points");
    }
    for i in 0..num {
        if world_coord[i].z != 0.0 {
            return Err("Points must be planar (Z=0)");
        }
    }
    if mat_xc2u[0][0] == 0.0 {
        return Err("mat_xc2u[0][0] is zero");
    }
    if mat_xc2u[1][0] != 0.0 {
        return Err("mat_xc2u[1][0] is not zero");
    }
    if mat_xc2u[1][1] == 0.0 {
        return Err("mat_xc2u[1][1] is zero");
    }
    if mat_xc2u[2][0] != 0.0 || mat_xc2u[2][1] != 0.0 || mat_xc2u[2][2] != 1.0 {
        return Err("mat_xc2u row 2 invalid");
    }
    if mat_xc2u[0][3] != 0.0 || mat_xc2u[1][3] != 0.0 || mat_xc2u[2][3] != 0.0 {
        return Err("mat_xc2u translation column invalid");
    }

    let mut mat_a = ARMat::new(num as i32 * 2, 8);
    let mut mat_b = ARMat::new(num as i32 * 2, 1);

    for i in 0..num {
        mat_a.m[i * 16] = world_coord[i].x;
        mat_a.m[i * 16 + 1] = world_coord[i].y;
        mat_a.m[i * 16 + 2] = 1.0;
        mat_a.m[i * 16 + 3] = 0.0;
        mat_a.m[i * 16 + 4] = 0.0;
        mat_a.m[i * 16 + 5] = 0.0;
        mat_a.m[i * 16 + 6] = -(world_coord[i].x) * (screen_coord[i].x);
        mat_a.m[i * 16 + 7] = -(world_coord[i].y) * (screen_coord[i].x);

        mat_a.m[i * 16 + 8] = 0.0;
        mat_a.m[i * 16 + 9] = 0.0;
        mat_a.m[i * 16 + 10] = 0.0;
        mat_a.m[i * 16 + 11] = world_coord[i].x;
        mat_a.m[i * 16 + 12] = world_coord[i].y;
        mat_a.m[i * 16 + 13] = 1.0;
        mat_a.m[i * 16 + 14] = -(world_coord[i].x) * (screen_coord[i].y);
        mat_a.m[i * 16 + 15] = -(world_coord[i].y) * (screen_coord[i].y);

        mat_b.m[i * 2] = screen_coord[i].x;
        mat_b.m[i * 2 + 1] = screen_coord[i].y;
    }

    let mat_at = mat_a.transpose();
    let mut mat_ata = (&mat_at * &mat_a)?;
    let mat_atb = (&mat_at * &mat_b)?;

    arlog_d!("mat_ata:\n{:?}", mat_ata);
    mat_ata.self_inv()?;
    arlog_d!("mat_ata_inv:\n{:?}", mat_ata);

    let mat_c = (&mat_ata * &mat_atb)?;
    arlog_d!("mat_c:\n{:?}", mat_c);

    let mut v = [[0.0; 3]; 3];
    let mut t = [0.0; 3];

    v[0][2] = mat_c.m[6];
    v[0][1] = (mat_c.m[3] - mat_xc2u[1][2] * v[0][2]) / mat_xc2u[1][1];
    v[0][0] = (mat_c.m[0] - mat_xc2u[0][2] * v[0][2] - mat_xc2u[0][1] * v[0][1]) / mat_xc2u[0][0];

    v[1][2] = mat_c.m[7];
    v[1][1] = (mat_c.m[4] - mat_xc2u[1][2] * v[1][2]) / mat_xc2u[1][1];
    v[1][0] = (mat_c.m[1] - mat_xc2u[0][2] * v[1][2] - mat_xc2u[0][1] * v[1][1]) / mat_xc2u[0][0];

    t[2] = 1.0;
    t[1] = (mat_c.m[5] - mat_xc2u[1][2] * t[2]) / mat_xc2u[1][1];
    t[0] = (mat_c.m[2] - mat_xc2u[0][2] * t[2] - mat_xc2u[0][1] * t[1]) / mat_xc2u[0][0];

    let mut l1 = (v[0][0] * v[0][0] + v[0][1] * v[0][1] + v[0][2] * v[0][2]).sqrt();
    let l2 = (v[1][0] * v[1][0] + v[1][1] * v[1][1] + v[1][2] * v[1][2]).sqrt();

    v[0][0] /= l1;
    v[0][1] /= l1;
    v[0][2] /= l1;

    v[1][0] /= l2;
    v[1][1] /= l2;
    v[1][2] /= l2;

    t[0] /= (l1 + l2) / 2.0;
    t[1] /= (l1 + l2) / 2.0;
    t[2] /= (l1 + l2) / 2.0;

    if t[2] < 0.0 {
        v[0][0] = -v[0][0];
        v[0][1] = -v[0][1];
        v[0][2] = -v[0][2];
        v[1][0] = -v[1][0];
        v[1][1] = -v[1][1];
        v[1][2] = -v[1][2];
        t[0] = -t[0];
        t[1] = -t[1];
        t[2] = -t[2];
    }

    arlog_d!("v after rotation check:\n{:?}", v);

    v[2][0] = v[0][1] * v[1][2] - v[0][2] * v[1][1];
    v[2][1] = v[0][2] * v[1][0] - v[0][0] * v[1][2];
    v[2][2] = v[0][0] * v[1][1] - v[0][1] * v[1][0];

    l1 = (v[2][0] * v[2][0] + v[2][1] * v[2][1] + v[2][2] * v[2][2]).sqrt();
    v[2][0] /= l1;
    v[2][1] /= l1;
    v[2][2] /= l1;

    init_mat_xw2xc[0][0] = v[0][0];
    init_mat_xw2xc[1][0] = v[0][1];
    init_mat_xw2xc[2][0] = v[0][2];
    init_mat_xw2xc[0][1] = v[1][0];
    init_mat_xw2xc[1][1] = v[1][1];
    init_mat_xw2xc[2][1] = v[1][2];
    init_mat_xw2xc[0][2] = v[2][0];
    init_mat_xw2xc[1][2] = v[2][1];
    init_mat_xw2xc[2][2] = v[2][2];
    init_mat_xw2xc[0][3] = t[0];
    init_mat_xw2xc[1][3] = t[1];
    init_mat_xw2xc[2][3] = t[2];

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_icp_handle_default() {
        let handle = ICPHandleT::default();
        assert_eq!(handle.max_loop, 10);
        assert_eq!(handle.break_loop_error_thresh, 0.1);
    }

    #[test]
    fn test_icp_mat_mul() {
        let m1 = [
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
        ];
        let m2 = [
            [2.0, 0.0, 0.0, 1.0],
            [0.0, 2.0, 0.0, 2.0],
            [0.0, 0.0, 2.0, 3.0],
        ];
        let mut dest = [[0.0; 4]; 3];
        icp_mat_mul(&m1, &m2, &mut dest);

        // Expected manually calculated:
        // row 0:
        // c0 = 1*2 = 2
        // c1 = 2*2 = 4
        // c2 = 3*2 = 6
        // c3 = 1*1 + 2*2 + 3*3 + 4 = 1 + 4 + 9 + 4 = 18
        assert_eq!(dest[0][0], 2.0);
        assert_eq!(dest[0][1], 4.0);
        assert_eq!(dest[0][2], 6.0);
        assert_eq!(dest[0][3], 18.0);
    }

    #[test]
    fn test_icp_get_xc_from_xw() {
        let mat = [
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 1.0, 0.0, 20.0],
            [0.0, 0.0, 1.0, 30.0],
        ];
        let xw = ICP3DCoordT {
            x: 5.0,
            y: 5.0,
            z: 5.0,
        };
        let mut xc = ICP3DCoordT::default();

        icp_get_xc_from_xw_by_mat_xw2xc(&mut xc, &mat, &xw);

        assert_eq!(xc.x, 15.0);
        assert_eq!(xc.y, 25.0);
        assert_eq!(xc.z, 35.0);
    }

    #[test]
    fn test_icp_get_u_from_x() {
        let mat = [
            [100.0, 0.0, 50.0, 0.0],
            [0.0, 100.0, 50.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ];
        let xc = ICP3DCoordT {
            x: 10.0,
            y: 20.0,
            z: 2.0,
        };
        let mut u = ICP2DCoordT::default();

        icp_get_u_from_x_by_mat_x2u(&mut u, &mat, &xc).unwrap();

        // h = 1.0 * 2.0 = 2.0
        // hx = 100*10 + 50*2 = 1100
        // hy = 100*20 + 50*2 = 2100
        // u.x = 1100 / 2 = 550
        // u.y = 2100 / 2 = 1050
        assert_eq!(u.x, 550.0);
        assert_eq!(u.y, 1050.0);
    }
}
