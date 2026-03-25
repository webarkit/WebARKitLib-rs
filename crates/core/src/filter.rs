/*
 *  filter.rs
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

use crate::math::{ar_util_mat2_quat_pos, ar_util_quat_pos2_mat, ar_util_quat_slerp};
use crate::types::ARdouble;

/// Smooths a transformation matrix over time using a simple interpolation filter.
///
/// Port of arFilterTransMat.
///
/// # Arguments
/// * `m` - The new transformation matrix to be filtered.
/// * `m_prev` - The filtered transformation matrix from the previous step.
/// * `sample_rate` - The rate at which samples are processed (inverse of time step).
/// * `cutoff_freq` - The frequency above which changes are smoothed out.
///
/// # Returns
/// The new filtered transformation matrix.
pub fn ar_filter_trans_mat(
    m: &[[ARdouble; 4]; 3],
    m_prev: &[[ARdouble; 4]; 3],
    sample_rate: ARdouble,
    cutoff_freq: ARdouble,
) -> [[ARdouble; 4]; 3] {
    let mut q = [0.0; 4];
    let mut p = [0.0; 3];
    let mut q_prev = [0.0; 4];
    let mut p_prev = [0.0; 3];
    let mut m_filtered = [[0.0; 4]; 3];

    // Decompose matrices into quaternion and position
    ar_util_mat2_quat_pos(m, &mut q, &mut p);
    ar_util_mat2_quat_pos(m_prev, &mut q_prev, &mut p_prev);

    // Calculate smoothing factor alpha
    let mut alpha: ARdouble;
    if cutoff_freq <= 0.0 {
        alpha = 0.0;
    } else if sample_rate <= 0.0 {
        alpha = 1.0;
    } else {
        let dt = 1.0 / sample_rate;
        let rc = 1.0 / (2.0 * std::f64::consts::PI * cutoff_freq);
        alpha = dt / (rc + dt);
    }

    if alpha > 1.0 {
        alpha = 1.0;
    }
    if alpha < 0.0 {
        alpha = 0.0;
    }

    // Interpolate position
    let mut p_filtered = [0.0; 3];
    for i in 0..3 {
        p_filtered[i] = p_prev[i] + alpha * (p[i] - p_prev[i]);
    }

    // Interpolate rotation using SLERP
    let mut q_filtered = [0.0; 4];
    ar_util_quat_slerp(&mut q_filtered, &q_prev, &q, alpha);

    // Recompose into matrix
    ar_util_quat_pos2_mat(&q_filtered, &p_filtered, &mut m_filtered);

    m_filtered
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ar_filter_trans_mat_no_change() {
        let m = [
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 1.0, 0.0, 20.0],
            [0.0, 0.0, 1.0, 30.0],
        ];
        let m_prev = m;
        let filtered = ar_filter_trans_mat(&m, &m_prev, 30.0, 10.0);

        for i in 0..3 {
            for j in 0..4 {
                assert!((filtered[i][j] - m[i][j]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_ar_filter_trans_mat_interpolation() {
        let m0 = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ];
        let m1 = [
            [1.0, 0.0, 0.0, 100.0],
            [0.0, 1.0, 0.0, 100.0],
            [0.0, 0.0, 1.0, 100.0],
        ];

        // Alpha will be 0.5 if dt = rc
        // rc = 1 / (2 * PI * cutoff)
        // dt = 1 / sample_rate
        // dt = rc => sample_rate = 2 * PI * cutoff
        let cutoff = 1.0;
        let sample_rate = 2.0 * std::f64::consts::PI * cutoff;

        let filtered = ar_filter_trans_mat(&m1, &m0, sample_rate, cutoff);

        // Position should be exactly in the middle (50.0)
        assert!((filtered[0][3] - 50.0).abs() < 1e-10);
        assert!((filtered[1][3] - 50.0).abs() < 1e-10);
        assert!((filtered[2][3] - 50.0).abs() < 1e-10);
    }
}
