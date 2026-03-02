//! Iterative Closest Point (ICP) Data Structures and Methods
//! Translated from ARToolKit C headers (icp.h, icpCore.h)

use crate::types::ARdouble;

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
pub struct ICPDataT {
    pub screen_coord: Vec<ICP2DCoordT>,
    pub world_coord: Vec<ICP3DCoordT>,
}

impl Default for ICPDataT {
    fn default() -> Self {
        Self {
            screen_coord: Vec::new(),
            world_coord: Vec::new(),
        }
    }
}

/// Stereo Point Data for ICP
#[derive(Debug, Clone, PartialEq)]
pub struct ICPStereoDataT {
    pub screen_coord_l: Vec<ICP2DCoordT>,
    pub world_coord_l: Vec<ICP3DCoordT>,
    pub screen_coord_r: Vec<ICP2DCoordT>,
    pub world_coord_r: Vec<ICP3DCoordT>,
}

impl Default for ICPStereoDataT {
    fn default() -> Self {
        Self {
            screen_coord_l: Vec::new(),
            world_coord_l: Vec::new(),
            screen_coord_r: Vec::new(),
            world_coord_r: Vec::new(),
        }
    }
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

pub fn icp_mat_mul(m1: &[[ARdouble; 4]; 3], m2: &[[ARdouble; 4]; 3], dest: &mut [[ARdouble; 4]; 3]) {
    for r in 0..3 {
        for c in 0..4 {
            dest[r][c] = m1[r][0] * m2[0][c]
                       + m1[r][1] * m2[1][c]
                       + m1[r][2] * m2[2][c];
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
            du[j * 2 + 0] = dx;
            du[j * 2 + 1] = dy;
        }
        err1 /= num_points as ARdouble;

        if err1 < handle.break_loop_error_thresh { break; }
        if loop_idx > 0 && err1 < handle.break_loop_error_thresh2 && (err1 / err0) > handle.break_loop_error_ratio_thresh { break; }
        if loop_idx == handle.max_loop { break; }
        err0 = err1;

        for j in 0..num_points {
            let mut j_u_s_local = [[0.0; 6]; 2];
            icp_get_j_u_s(&mut j_u_s_local, &handle.mat_xc2u, mat_xw2xc, &data.world_coord[j])?;
            for r in 0..2 {
                for c in 0..6 {
                    j_u_s_table[j * 2 + r][c] = j_u_s_local[r][c];
                }
            }
        }

        icp_get_delta_s(&mut ds, &du, &j_u_s_table, num_points * 2)?;
        icp_update_mat(mat_xw2xc, &ds);

        loop_idx += 1;
    }

    Ok(err1)
}

pub fn icp_get_xc_from_xw_by_mat_xw2xc(xc: &mut ICP3DCoordT, mat_xw2xc: &[[ARdouble; 4]; 3], xw: &ICP3DCoordT) {
    xc.x = mat_xw2xc[0][0] * xw.x + mat_xw2xc[0][1] * xw.y + mat_xw2xc[0][2] * xw.z + mat_xw2xc[0][3];
    xc.y = mat_xw2xc[1][0] * xw.x + mat_xw2xc[1][1] * xw.y + mat_xw2xc[1][2] * xw.z + mat_xw2xc[1][3];
    xc.z = mat_xw2xc[2][0] * xw.x + mat_xw2xc[2][1] * xw.y + mat_xw2xc[2][2] * xw.z + mat_xw2xc[2][3];
}

pub fn icp_get_u_from_x_by_mat_x2u(u: &mut ICP2DCoordT, mat_x2u: &[[ARdouble; 4]; 3], coord3d: &ICP3DCoordT) -> Result<(), &'static str> {
    let hx = mat_x2u[0][0] * coord3d.x + mat_x2u[0][1] * coord3d.y + mat_x2u[0][2] * coord3d.z + mat_x2u[0][3];
    let hy = mat_x2u[1][0] * coord3d.x + mat_x2u[1][1] * coord3d.y + mat_x2u[1][2] * coord3d.z + mat_x2u[1][3];
    let h  = mat_x2u[2][0] * coord3d.x + mat_x2u[2][1] * coord3d.y + mat_x2u[2][2] * coord3d.z + mat_x2u[2][3];

    if h == 0.0 {
        return Err("Division by zero in icp_get_u_from_x_by_mat_x2u");
    }

    u.x = hx / h;
    u.y = hy / h;
    Ok(())
}

fn icp_get_j_u_xc(j_u_xc: &mut [[ARdouble; 3]; 2], mat_xc2u: &[[ARdouble; 4]; 3], camera_coord: &ICP3DCoordT) -> Result<(), &'static str> {
    let w1 = mat_xc2u[0][0] * camera_coord.x + mat_xc2u[0][1] * camera_coord.y + mat_xc2u[0][2] * camera_coord.z + mat_xc2u[0][3];
    let w2 = mat_xc2u[1][0] * camera_coord.x + mat_xc2u[1][1] * camera_coord.y + mat_xc2u[1][2] * camera_coord.z + mat_xc2u[1][3];
    let w3 = mat_xc2u[2][0] * camera_coord.x + mat_xc2u[2][1] * camera_coord.y + mat_xc2u[2][2] * camera_coord.z + mat_xc2u[2][3];

    if w3 == 0.0 { return Err("Division by zero in icp_get_j_u_xc"); }

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
        for j in 0..6 { j_t_s[i][j] = 0.0; }
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

fn icp_get_j_xc_s(j_xc_s: &mut [[ARdouble; 6]; 3], camera_coord: &mut ICP3DCoordT, t0: &[[ARdouble; 4]; 3], world_coord: &ICP3DCoordT) {
    let mut j_xc_t_flat = [0.0; 36];
    
    let mut write_j_xc_t = |r: usize, c: usize, val: f64| j_xc_t_flat[r * 12 + c] = val;

    for j in 0..3 {
        write_j_xc_t(j, j * 3 + 0, world_coord.x);
        write_j_xc_t(j, j * 3 + 1, world_coord.y);
        write_j_xc_t(j, j * 3 + 2, world_coord.z);
        write_j_xc_t(j, 9, if j == 0 { 1.0 } else { 0.0 });
        write_j_xc_t(j, 10, if j == 1 { 1.0 } else { 0.0 });
        write_j_xc_t(j, 11, if j == 2 { 1.0 } else { 0.0 });
    }
    
    icp_get_xc_from_xw_by_mat_xw2xc(camera_coord, t0, world_coord);

    let mut j_t_s = [[0.0; 6]; 12];
    icp_get_j_t_s(&mut j_t_s);

    for j in 0..3 {
        for i in 0..6 {
            j_xc_s[j][i] = 0.0;
            for k in 0..12 {
                j_xc_s[j][i] += j_xc_t_flat[j * 12 + k] * j_t_s[k][i];
            }
        }
    }
}

pub fn icp_get_j_u_s(j_u_s: &mut [[ARdouble; 6]; 2], mat_xc2u: &[[ARdouble; 4]; 3], mat_xw2xc: &[[ARdouble; 4]; 3], world_coord: &ICP3DCoordT) -> Result<(), &'static str> {
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
    let mut ra = s[0]*s[0] + s[1]*s[1] + s[2]*s[2];
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

    mat[0][0] = q[0]*q[0]*one_cra + cra;
    mat[0][1] = q[0]*q[1]*one_cra - q[2]*sra;
    mat[0][2] = q[0]*q[2]*one_cra + q[1]*sra;
    mat[0][3] = q[4];
    
    mat[1][0] = q[1]*q[0]*one_cra + q[2]*sra;
    mat[1][1] = q[1]*q[1]*one_cra + cra;
    mat[1][2] = q[1]*q[2]*one_cra - q[0]*sra;
    mat[1][3] = q[5];
    
    mat[2][0] = q[2]*q[0]*one_cra - q[1]*sra;
    mat[2][1] = q[2]*q[1]*one_cra + q[0]*sra;
    mat[2][2] = q[2]*q[2]*one_cra + cra;
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

pub fn icp_get_delta_s(s: &mut [ARdouble; 6], du: &[ARdouble], j_u_s: &[[ARdouble; 6]], n: usize) -> Result<(), &'static str> {
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
        let xw = ICP3DCoordT { x: 5.0, y: 5.0, z: 5.0 };
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
        let xc = ICP3DCoordT { x: 10.0, y: 20.0, z: 2.0 };
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
