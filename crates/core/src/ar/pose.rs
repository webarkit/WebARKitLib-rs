/*
 *  pose.rs
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
//! # Pose Estimation - core functions for estimating the pose of AR markers in 3D space.
use crate::icp::{
    icp_create_handle, icp_delete_handle, icp_get_init_xw2xc_from_planar_data, icp_point,
    ICP2DCoordT, ICP3DCoordT, ICPDataT,
};
use crate::types::{AR3DHandle, ARMarkerInfo, ARParam, ARdouble};
// use log::debug; // Removed unused import

/// Allocate an `AR3DHandle` for pose estimation from the given camera parameters.
pub fn ar_3d_create_handle(ar_param: &ARParam) -> Result<*mut AR3DHandle, &'static str> {
    let icp_handle = icp_create_handle(&ar_param.mat)?;
    let mut handle = Box::new(AR3DHandle::default());
    handle.icp_handle = icp_handle;
    Ok(Box::into_raw(handle))
}

/// Free an `AR3DHandle` and null the pointer.
pub fn ar_3d_delete_handle(handle: &mut *mut AR3DHandle) -> Result<(), &'static str> {
    if handle.is_null() {
        return Err("Null AR3DHandle");
    }
    unsafe {
        let mut bx = Box::from_raw(*handle);
        icp_delete_handle(&mut bx.icp_handle)?;
        drop(bx);
    }
    *handle = std::ptr::null_mut();
    Ok(())
}

/// Estimate the 3×4 pose of a square marker of the given side length.
pub fn ar_get_trans_mat_square(
    handle: &AR3DHandle,
    marker_info: &ARMarkerInfo,
    width: ARdouble,
    conv: &mut [[ARdouble; 4]; 3],
) -> Result<ARdouble, &'static str> {
    if handle.icp_handle.is_null() {
        return Err("Null ICPHandleT within AR3DHandle");
    }

    let dir = if marker_info.id_matrix < 0 {
        marker_info.dir_patt
    } else if marker_info.id_patt < 0 {
        marker_info.dir_matrix
    } else {
        marker_info.dir
    };

    let mut screen_coord = vec![ICP2DCoordT::default(); 4];
    screen_coord[0].x = marker_info.vertex[((4 - dir) % 4) as usize][0];
    screen_coord[0].y = marker_info.vertex[((4 - dir) % 4) as usize][1];
    screen_coord[1].x = marker_info.vertex[((5 - dir) % 4) as usize][0];
    screen_coord[1].y = marker_info.vertex[((5 - dir) % 4) as usize][1];
    screen_coord[2].x = marker_info.vertex[((6 - dir) % 4) as usize][0];
    screen_coord[2].y = marker_info.vertex[((6 - dir) % 4) as usize][1];
    screen_coord[3].x = marker_info.vertex[((7 - dir) % 4) as usize][0];
    screen_coord[3].y = marker_info.vertex[((7 - dir) % 4) as usize][1];

    let mut world_coord = vec![ICP3DCoordT::default(); 4];
    world_coord[0].x = -width / 2.0;
    world_coord[0].y = width / 2.0;
    world_coord[0].z = 0.0;
    world_coord[1].x = width / 2.0;
    world_coord[1].y = width / 2.0;
    world_coord[1].z = 0.0;
    world_coord[2].x = width / 2.0;
    world_coord[2].y = -width / 2.0;
    world_coord[2].z = 0.0;
    world_coord[3].x = -width / 2.0;
    world_coord[3].y = -width / 2.0;
    world_coord[3].z = 0.0;

    let mut init_mat_xw2xc = [[0.0; 4]; 3];

    let icp_handle_ref = unsafe { &*handle.icp_handle };

    match icp_get_init_xw2xc_from_planar_data(
        &icp_handle_ref.mat_xc2u,
        &screen_coord,
        &world_coord,
        4,
        &mut init_mat_xw2xc,
    ) {
        Ok(_) => {}
        Err(_) => return Ok(100000000.0), // Arbitrary high error
    }

    let data = ICPDataT {
        screen_coord,
        world_coord,
    };

    match icp_point(icp_handle_ref, &data, &init_mat_xw2xc, conv) {
        Ok(err) => Ok(err),
        Err(_) => Ok(100000000.0),
    }
}

/// Estimate a square marker's pose using the previous frame's pose as a seed.
pub fn ar_get_trans_mat_square_cont(
    handle: &AR3DHandle,
    marker_info: &ARMarkerInfo,
    init_conv: &[[ARdouble; 4]; 3],
    width: ARdouble,
    conv: &mut [[ARdouble; 4]; 3],
) -> Result<ARdouble, &'static str> {
    if handle.icp_handle.is_null() {
        return Err("Null ICPHandleT within AR3DHandle");
    }

    let dir = if marker_info.id_matrix < 0 {
        marker_info.dir_patt
    } else if marker_info.id_patt < 0 {
        marker_info.dir_matrix
    } else {
        marker_info.dir
    };

    let mut screen_coord = vec![ICP2DCoordT::default(); 4];
    let mut world_coord = vec![ICP3DCoordT::default(); 4];

    screen_coord[0].x = marker_info.vertex[((4 - dir) % 4) as usize][0];
    screen_coord[0].y = marker_info.vertex[((4 - dir) % 4) as usize][1];
    screen_coord[1].x = marker_info.vertex[((5 - dir) % 4) as usize][0];
    screen_coord[1].y = marker_info.vertex[((5 - dir) % 4) as usize][1];
    screen_coord[2].x = marker_info.vertex[((6 - dir) % 4) as usize][0];
    screen_coord[2].y = marker_info.vertex[((6 - dir) % 4) as usize][1];
    screen_coord[3].x = marker_info.vertex[((7 - dir) % 4) as usize][0];
    screen_coord[3].y = marker_info.vertex[((7 - dir) % 4) as usize][1];

    world_coord[0].x = -width / 2.0;
    world_coord[0].y = width / 2.0;
    world_coord[0].z = 0.0;
    world_coord[1].x = width / 2.0;
    world_coord[1].y = width / 2.0;
    world_coord[1].z = 0.0;
    world_coord[2].x = width / 2.0;
    world_coord[2].y = -width / 2.0;
    world_coord[2].z = 0.0;
    world_coord[3].x = -width / 2.0;
    world_coord[3].y = -width / 2.0;
    world_coord[3].z = 0.0;

    let icp_handle_ref = unsafe { &*handle.icp_handle };

    let data = ICPDataT {
        screen_coord,
        world_coord,
    };

    match icp_point(icp_handle_ref, &data, init_conv, conv) {
        Ok(err) => Ok(err),
        Err(_) => Ok(100000000.0),
    }
}

/// Estimate a 3×4 pose from 2D–3D correspondences via iterative refinement.
pub fn ar_get_trans_mat(
    handle: &AR3DHandle,
    init_conv: &[[ARdouble; 4]; 3],
    pos2d: &[[ARdouble; 2]],
    pos3d: &[[ARdouble; 3]],
    num: usize,
    conv: &mut [[ARdouble; 4]; 3],
) -> Result<ARdouble, &'static str> {
    if handle.icp_handle.is_null() {
        return Err("Null ICPHandleT within AR3DHandle");
    }

    if pos2d.len() < num || pos3d.len() < num {
        return Err("Not enough coordinate data provided");
    }

    let mut screen_coord = Vec::with_capacity(num);
    let mut world_coord = Vec::with_capacity(num);

    for i in 0..num {
        screen_coord.push(ICP2DCoordT {
            x: pos2d[i][0],
            y: pos2d[i][1],
        });
        world_coord.push(ICP3DCoordT {
            x: pos3d[i][0],
            y: pos3d[i][1],
            z: pos3d[i][2],
        });
    }

    let data = ICPDataT {
        screen_coord,
        world_coord,
    };

    let icp_handle_ref = unsafe { &*handle.icp_handle };

    match icp_point(icp_handle_ref, &data, init_conv, conv) {
        Ok(err) => Ok(err),
        Err(_) => Ok(100000000.0),
    }
}
