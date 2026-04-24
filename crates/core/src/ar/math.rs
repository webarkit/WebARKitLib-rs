/*
 *  math.rs
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

//! Matrix and Vector Data Structures for WebARKitLib
//! Translated from ARToolKit C headers (matrix.h)

use crate::types::ARdouble;
use std::ops::Mul;

/// Matrix structure
#[derive(Debug, Clone, PartialEq)]
#[repr(C)]
#[derive(Default)]
pub struct ARMat {
    pub m: Vec<ARdouble>,
    pub row: i32,
    pub clm: i32,
}


impl ARMat {
    /// Allocate a new matrix with specified dimensions
    pub fn new(row: i32, clm: i32) -> Self {
        let size = (row * clm) as usize;
        Self {
            m: vec![0.0; size],
            row,
            clm,
        }
    }

    /// Transpose the matrix
    pub fn transpose(&self) -> ARMat {
        let mut dest = ARMat::new(self.clm, self.row);
        let dest_clm = dest.clm as usize;
        let src_clm = self.clm as usize;

        for r in 0..(dest.row as usize) {
            for c in 0..(dest.clm as usize) {
                // dest(r, c) = source(c, r)
                dest.m[r * dest_clm + c] = self.m[c * src_clm + r];
            }
        }
        dest
    }

    /// Calculate the determinant of a square matrix
    pub fn det(&self) -> f64 {
        if self.row != self.clm {
            return 0.0; // ARToolKit returns 0.0 for non-square matrices
        }

        let dimen = self.row as usize;
        let mut ap = self.m.clone(); // Clone to avoid mutating self
        let mut det = 1.0;
        let mut is = 0;

        for k in 0..(dimen - 1) {
            let mut mmax = k;
            for i in (k + 1)..dimen {
                if ap[i * dimen + k].abs() > ap[mmax * dimen + k].abs() {
                    mmax = i;
                }
            }
            if mmax != k {
                for j in k..dimen {
                    ap.swap(k * dimen + j, mmax * dimen + j);
                }
                is += 1;
            }
            for i in (k + 1)..dimen {
                let work = ap[i * dimen + k] / ap[k * dimen + k];
                for j in (k + 1)..dimen {
                    ap[i * dimen + j] -= work * ap[k * dimen + j];
                }
            }
        }

        for i in 0..dimen {
            det *= ap[i * dimen + i];
        }
        for _ in 0..is {
            det *= -1.0;
        }

        det
    }

    /// Invert the matrix in place using Gauss-Jordan elimination with column shifting
    pub fn self_inv(&mut self) -> Result<(), &'static str> {
        let dimen = self.row as usize;
        if dimen != self.clm as usize {
            return Err("Matrix must be square");
        }
        if dimen > 500 {
            return Err("Matrix too large");
        }
        if dimen == 0 {
            return Err("Matrix is empty");
        }
        if dimen == 1 {
            self.m[0] = 1.0 / self.m[0];
            return Ok(());
        }

        let mut nos = vec![0; dimen];
        for n in 0..dimen {
            nos[n] = n;
        }

        for n in 0..dimen {
            let mut p = 0.0;
            let mut ip = -1isize;

            for i in n..dimen {
                let pbuf = self.m[i * dimen].abs();
                if p < pbuf {
                    p = pbuf;
                    ip = i as isize;
                }
            }

            if p <= 1.0e-10 || ip == -1 {
                return Err("Matrix is singular");
            }

            let ip = ip as usize;

            nos.swap(ip, n);

            for j in 0..dimen {
                self.m.swap(ip * dimen + j, n * dimen + j);
            }

            let work = self.m[n * dimen];
            for j in 1..dimen {
                self.m[n * dimen + j - 1] = self.m[n * dimen + j] / work;
            }
            self.m[n * dimen + dimen - 1] = 1.0 / work;

            for i in 0..dimen {
                if i != n {
                    let work = self.m[i * dimen];
                    for j in 1..dimen {
                        self.m[i * dimen + j - 1] =
                            self.m[i * dimen + j] - work * self.m[n * dimen + j - 1];
                    }
                    self.m[i * dimen + dimen - 1] = -work * self.m[n * dimen + dimen - 1];
                }
            }
        }

        for n in 0..dimen {
            let mut j = n;
            while j < dimen {
                if nos[j] == n {
                    break;
                }
                j += 1;
            }
            nos[j] = nos[n];
            for i in 0..dimen {
                self.m.swap(i * dimen + j, i * dimen + n);
            }
        }

        Ok(())
    }

    /// Return a new inverted matrix
    pub fn inv(&self) -> Result<ARMat, &'static str> {
        let mut dest = self.clone();
        dest.self_inv()?;
        Ok(dest)
    }

    /// Tridiagonalize symmetric matrix (port of arVecTridiagonalize)
    pub fn tridiagonalize(
        &mut self,
        d: &mut [ARdouble],
        e: &mut [ARdouble],
    ) -> Result<(), &'static str> {
        let dim = self.row as usize;
        if dim != self.clm as usize || dim != d.len() || dim != e.len() + 1 {
            return Err("Mismatched dimensions for tridiagonalize");
        }

        for k in 0..dim.saturating_sub(2) {
            d[k] = self.m[k * dim + k];

            e[k] = household(&mut self.m[k * dim + k + 1..k * dim + dim]);
            if e[k] == 0.0 {
                continue;
            }

            for i in (k + 1)..dim {
                let mut s = 0.0;
                for j in (k + 1)..i {
                    s += self.m[j * dim + i] * self.m[k * dim + j];
                }
                for j in i..dim {
                    s += self.m[i * dim + j] * self.m[k * dim + j];
                }
                d[i] = s;
            }

            let t = inner_product(&self.m[k * dim + k + 1..k * dim + dim], &d[k + 1..dim]) / 2.0;

            for i in (k + 1..dim).rev() {
                let p = self.m[k * dim + i];
                d[i] -= t * p;
                let q = d[i];
                for j in i..dim {
                    self.m[i * dim + j] -= p * d[j] + q * self.m[k * dim + j];
                }
            }
        }

        if dim >= 2 {
            d[dim - 2] = self.m[(dim - 2) * dim + (dim - 2)];
            e[dim - 2] = self.m[(dim - 2) * dim + (dim - 1)];
        }

        if dim >= 1 {
            d[dim - 1] = self.m[(dim - 1) * dim + (dim - 1)];
        }

        for k in (0..dim).rev() {
            if k < dim.saturating_sub(2) {
                for i in (k + 1)..dim {
                    let mut t = 0.0;
                    for j in (k + 1)..dim {
                        t += self.m[k * dim + j] * self.m[i * dim + j];
                    }
                    for j in (k + 1)..dim {
                        self.m[i * dim + j] -= t * self.m[k * dim + j];
                    }
                }
            }
            for i in 0..dim {
                self.m[k * dim + i] = 0.0;
            }
            self.m[k * dim + k] = 1.0;
        }

        Ok(())
    }
}

/// Helper for vector inner product
pub fn inner_product(x: &[ARdouble], y: &[ARdouble]) -> ARdouble {
    x.iter().zip(y.iter()).map(|(a, b)| a * b).sum()
}

/// Helper for vector householder reflection
pub fn household(x: &mut [ARdouble]) -> ARdouble {
    let mut s = inner_product(x, x).sqrt();
    if s != 0.0 {
        if x[0] < 0.0 {
            s = -s;
        }
        x[0] += s;
        let t = 1.0 / (x[0] * s).sqrt();
        for val in x.iter_mut() {
            *val *= t;
        }
    }
    -s
}

pub fn qrm(a: &mut ARMat, dv: &mut [ARdouble]) -> Result<(), &'static str> {
    let dim = a.row as usize;
    if dim != a.clm as usize || dim < 2 || dv.len() != dim {
        return Err("Invalid dimensions for QRM");
    }

    let mut ev = vec![0.0; dim];
    a.tridiagonalize(dv, &mut ev[1..])?;
    ev[0] = 0.0;

    let eps = 1e-6;
    let vzero = 1e-16;
    let max_iter = 100;

    for h in (1..dim).rev() {
        let mut j = h;
        while j > 0 && ev[j].abs() > eps * (dv[j - 1].abs() + dv[j].abs()) {
            j -= 1;
        }
        if j == h {
            continue;
        }

        let mut iter = 0;
        while ev[h].abs() > eps * (dv[h - 1].abs() + dv[h].abs()) {
            iter += 1;
            if iter > max_iter {
                break;
            }

            let mut w = (dv[h - 1] - dv[h]) / 2.0;
            let mut t = ev[h] * ev[h];
            let mut s = (w * w + t).sqrt();
            if w < 0.0 {
                s = -s;
            }
            let mut x = dv[j] - dv[h] + t / (w + s);
            let mut y = ev[j + 1];

            for k in j..h {
                let c: ARdouble;
                if x.abs() >= y.abs() {
                    if x.abs() > vzero {
                        t = -y / x;
                        c = 1.0 / (t * t + 1.0).sqrt();
                        s = t * c;
                    } else {
                        c = 1.0;
                        s = 0.0;
                    }
                } else {
                    t = -x / y;
                    s = 1.0 / (t * t + 1.0).sqrt();
                    c = t * s;
                }
                w = dv[k] - dv[k + 1];
                t = (w * s + 2.0 * c * ev[k + 1]) * s;
                dv[k] -= t;
                dv[k + 1] += t;
                if k > j {
                    ev[k] = c * ev[k] - s * y;
                }
                ev[k + 1] += s * (c * w - 2.0 * s * ev[k + 1]);

                for i in 0..dim {
                    let rx = a.m[k * dim + i];
                    let ry = a.m[(k + 1) * dim + i];
                    a.m[k * dim + i] = c * rx - s * ry;
                    a.m[(k + 1) * dim + i] = s * rx + c * ry;
                }
                if k < h - 1 {
                    x = ev[k + 1];
                    y = -s * ev[k + 2];
                    ev[k + 2] *= c;
                }
            }
        }
    }

    for k in 0..dim - 1 {
        let mut h = k;
        let mut t = dv[h];
        for i in k + 1..dim {
            if dv[i] > t {
                h = i;
                t = dv[h];
            }
        }
        dv[h] = dv[k];
        dv[k] = t;
        for i in 0..dim {
            a.m.swap(h * dim + i, k * dim + i);
        }
    }
    Ok(())
}

impl ARMat {
    pub fn ex(&self, mean: &mut [ARdouble]) -> Result<(), &'static str> {
        let row = self.row as usize;
        let clm = self.clm as usize;
        if row == 0 || clm == 0 || mean.len() != clm {
            return Err("Invalid dimensions for EX");
        }

        for i in 0..clm {
            mean[i] = 0.0;
        }

        for r in 0..row {
            for c in 0..clm {
                mean[c] += self.m[r * clm + c];
            }
        }

        for i in 0..clm {
            mean[i] /= row as ARdouble;
        }
        Ok(())
    }

    pub fn center(&mut self, mean: &[ARdouble]) -> Result<(), &'static str> {
        let row = self.row as usize;
        let clm = self.clm as usize;
        if mean.len() != clm {
            return Err("Invalid dimensions for CENTER");
        }

        for r in 0..row {
            for c in 0..clm {
                self.m[r * clm + c] -= mean[c];
            }
        }
        Ok(())
    }

    pub fn x_by_xt(&self, output: &mut ARMat) -> Result<(), &'static str> {
        let row = self.row as usize;
        let clm = self.clm as usize;
        if output.row as usize != row || output.clm as usize != row {
            return Err("Invalid dimensions for x_by_xt");
        }

        for i in 0..row {
            for j in 0..row {
                if j < i {
                    output.m[i * row + j] = output.m[j * row + i];
                } else {
                    let mut out = 0.0;
                    for k in 0..clm {
                        out += self.m[i * clm + k] * self.m[j * clm + k];
                    }
                    output.m[i * row + j] = out;
                }
            }
        }
        Ok(())
    }

    pub fn xt_by_x(&self, output: &mut ARMat) -> Result<(), &'static str> {
        let row = self.row as usize;
        let clm = self.clm as usize;
        if output.row as usize != clm || output.clm as usize != clm {
            return Err("Invalid dimensions for xt_by_x");
        }

        for i in 0..clm {
            for j in 0..clm {
                if j < i {
                    output.m[i * clm + j] = output.m[j * clm + i];
                } else {
                    let mut out = 0.0;
                    for k in 0..row {
                        out += self.m[k * clm + i] * self.m[k * clm + j];
                    }
                    output.m[i * clm + j] = out;
                }
            }
        }
        Ok(())
    }

    pub fn ev_create(
        &self,
        u: &ARMat,
        output: &mut ARMat,
        ev: &mut [ARdouble],
    ) -> Result<(), &'static str> {
        let row = self.row as usize;
        let clm = self.clm as usize;
        if row == 0
            || clm == 0
            || u.row as usize != row
            || u.clm as usize != row
            || output.row as usize != row
            || output.clm as usize != clm
            || ev.len() != row
        {
            return Err("Invalid dimensions for EV_create");
        }

        let mut i = 0;
        while i < row {
            if ev[i] < 1e-16 {
                break;
            }
            let work = 1.0 / ev[i].abs().sqrt();
            for j in 0..clm {
                let mut sum = 0.0;
                for k in 0..row {
                    sum += u.m[i * row + k] * self.m[k * clm + j];
                }
                output.m[i * clm + j] = sum * work;
            }
            i += 1;
        }

        while i < row {
            ev[i] = 0.0;
            for j in 0..clm {
                output.m[i * clm + j] = 0.0;
            }
            i += 1;
        }
        Ok(())
    }

    pub fn pca_internal(
        &self,
        output: &mut ARMat,
        ev: &mut [ARdouble],
    ) -> Result<(), &'static str> {
        let row = self.row as usize;
        let clm = self.clm as usize;
        let min = row.min(clm);
        if row < 2
            || clm < 2
            || output.clm as usize != clm
            || output.row as usize != min
            || ev.len() != min
        {
            return Err("Invalid dimensions for PCA internal");
        }

        let mut u = ARMat::new(min as i32, min as i32);
        if row < clm {
            self.x_by_xt(&mut u)?;
        } else {
            self.xt_by_x(&mut u)?;
        }

        qrm(&mut u, ev)?;

        if row < clm {
            self.ev_create(&u, output, ev)?;
        } else {
            let mut i = 0;
            while i < min {
                if ev[i] < 1e-16 {
                    break;
                }
                for j in 0..min {
                    output.m[i * clm + j] = u.m[i * min + j];
                }
                i += 1;
            }
            while i < min {
                ev[i] = 0.0;
                for j in 0..min {
                    output.m[i * clm + j] = 0.0;
                }
                i += 1;
            }
        }
        Ok(())
    }

    pub fn pca(
        &self,
        evec: &mut ARMat,
        ev: &mut ARVec,
        mean: &mut ARVec,
    ) -> Result<(), &'static str> {
        let row = self.row as usize;
        let clm = self.clm as usize;
        let check = row.min(clm);
        if row < 2
            || clm < 2
            || evec.clm as usize != clm
            || evec.row as usize != check
            || ev.clm as usize != check
            || mean.clm as usize != clm
        {
            return Err("Invalid dimensions for PCA");
        }

        let mut work = self.clone();
        work.ex(&mut mean.v)?;
        work.center(&mean.v)?;

        let srow = (row as f64).sqrt();
        for val in work.m.iter_mut() {
            *val /= srow;
        }

        work.pca_internal(evec, &mut ev.v)?;

        let sum: f64 = ev.v.iter().sum();
        if sum != 0.0 {
            for val in ev.v.iter_mut() {
                *val /= sum;
            }
        }
        Ok(())
    }
}

impl<'b> Mul<&'b ARMat> for &ARMat {
    type Output = Result<ARMat, &'static str>;

    /// Multiplies two matrices (`self` * `rhs`).
    /// Port of `arMatrixMul` from `mMul.c`.
    fn mul(self, rhs: &'b ARMat) -> Self::Output {
        if self.clm != rhs.row {
            return Err("Matrix dimensions do not match for multiplication");
        }

        let mut dest = ARMat::new(self.row, rhs.clm);
        let dest_clm = dest.clm as usize;
        let a_clm = self.clm as usize;
        let b_clm = rhs.clm as usize;

        for r in 0..(dest.row as usize) {
            for c in 0..(dest.clm as usize) {
                let mut sum = 0.0;
                let mut p1_idx = r * a_clm;
                let mut p2_idx = c;
                for _ in 0..a_clm {
                    sum += self.m[p1_idx] * rhs.m[p2_idx];
                    p1_idx += 1;
                    p2_idx += b_clm;
                }
                dest.m[r * dest_clm + c] = sum;
            }
        }

        Ok(dest)
    }
}

/// Float Matrix structure (Explicit f32 variant)
#[derive(Debug, Clone, PartialEq)]
#[repr(C)]
#[derive(Default)]
pub struct ARMatf {
    pub m: Vec<f32>,
    pub row: i32,
    pub clm: i32,
}


impl ARMatf {
    /// Allocate a new matrix with specified dimensions
    pub fn new(row: i32, clm: i32) -> Self {
        let size = (row * clm) as usize;
        Self {
            m: vec![0.0; size],
            row,
            clm,
        }
    }
}

impl<'b> Mul<&'b ARMatf> for &ARMatf {
    type Output = Result<ARMatf, &'static str>;

    /// Multiplies two matrices (`self` * `rhs`).
    /// Port of `arMatrixMulf` from `mMul.c`.
    fn mul(self, rhs: &'b ARMatf) -> Self::Output {
        if self.clm != rhs.row {
            return Err("Matrix dimensions do not match for multiplication");
        }

        let mut dest = ARMatf::new(self.row, rhs.clm);
        let dest_clm = dest.clm as usize;
        let a_clm = self.clm as usize;
        let b_clm = rhs.clm as usize;

        for r in 0..(dest.row as usize) {
            for c in 0..(dest.clm as usize) {
                let mut sum = 0.0;
                let mut p1_idx = r * a_clm;
                let mut p2_idx = c;
                for _ in 0..a_clm {
                    sum += self.m[p1_idx] * rhs.m[p2_idx];
                    p1_idx += 1;
                    p2_idx += b_clm;
                }
                dest.m[r * dest_clm + c] = sum;
            }
        }

        Ok(dest)
    }
}

/// Vector structure
#[derive(Debug, Clone, PartialEq)]
#[repr(C)]
#[derive(Default)]
pub struct ARVec {
    pub v: Vec<ARdouble>,
    pub clm: i32,
}


impl ARVec {
    /// Allocate a new vector with specified columns
    pub fn new(clm: i32) -> Self {
        Self {
            v: vec![0.0; clm as usize],
            clm,
        }
    }
}

/// Convert a 3x4 transformation matrix to quaternion and position
/// Port of arUtilMat2QuatPos
pub fn ar_util_mat2_quat_pos(m: &[[ARdouble; 4]; 3], q: &mut [ARdouble; 4], p: &mut [ARdouble; 3]) {
    p[0] = m[0][3];
    p[1] = m[1][3];
    p[2] = m[2][3];

    let trace = m[0][0] + m[1][1] + m[2][2];
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        q[0] = 0.25 * s;
        q[1] = (m[2][1] - m[1][2]) / s;
        q[2] = (m[0][2] - m[2][0]) / s;
        q[3] = (m[1][0] - m[0][1]) / s;
    } else if (m[0][0] > m[1][1]) && (m[0][0] > m[2][2]) {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        q[0] = (m[2][1] - m[1][2]) / s;
        q[1] = 0.25 * s;
        q[2] = (m[0][1] + m[1][0]) / s;
        q[3] = (m[0][2] + m[2][0]) / s;
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        q[0] = (m[0][2] - m[2][0]) / s;
        q[1] = (m[0][1] + m[1][0]) / s;
        q[2] = 0.25 * s;
        q[3] = (m[1][2] + m[2][1]) / s;
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        q[0] = (m[1][0] - m[0][1]) / s;
        q[1] = (m[0][2] + m[2][0]) / s;
        q[2] = (m[1][2] + m[2][1]) / s;
        q[3] = 0.25 * s;
    }
}

/// Convert quaternion and position to a 3x4 transformation matrix
/// Port of arUtilQuatPos2Mat
pub fn ar_util_quat_pos2_mat(q: &[ARdouble; 4], p: &[ARdouble; 3], m: &mut [[ARdouble; 4]; 3]) {
    let x2 = q[1] + q[1];
    let y2 = q[2] + q[2];
    let z2 = q[3] + q[3];
    let xx = q[1] * x2;
    let xy = q[1] * y2;
    let xz = q[1] * z2;
    let yy = q[2] * y2;
    let yz = q[2] * z2;
    let zz = q[3] * z2;
    let wx = q[0] * x2;
    let wy = q[0] * y2;
    let wz = q[0] * z2;

    m[0][0] = 1.0 - (yy + zz);
    m[0][1] = xy - wz;
    m[0][2] = xz + wy;

    m[1][0] = xy + wz;
    m[1][1] = 1.0 - (xx + zz);
    m[1][2] = yz - wx;

    m[2][0] = xz - wy;
    m[2][1] = yz + wx;
    m[2][2] = 1.0 - (xx + yy);

    m[0][3] = p[0];
    m[1][3] = p[1];
    m[2][3] = p[2];
}

/// Spherical linear interpolation of quaternions
/// Port of arUtilQuatSlerp
pub fn ar_util_quat_slerp(
    q: &mut [ARdouble; 4],
    qy: &[ARdouble; 4],
    qz: &[ARdouble; 4],
    t: ARdouble,
) {
    let mut cos_omega = qy[0] * qz[0] + qy[1] * qz[1] + qy[2] * qz[2] + qy[3] * qz[3];

    let mut qz2 = [0.0; 4];
    if cos_omega < 0.0 {
        cos_omega = -cos_omega;
        qz2[0] = -qz[0];
        qz2[1] = -qz[1];
        qz2[2] = -qz[2];
        qz2[3] = -qz[3];
    } else {
        qz2 = *qz;
    }

    let (k0, k1);
    if cos_omega > 0.9999 {
        k0 = 1.0 - t;
        k1 = t;
    } else {
        let sin_omega = (1.0 - cos_omega * cos_omega).sqrt();
        let omega = sin_omega.atan2(cos_omega);
        let inv_sin_omega = 1.0 / sin_omega;
        k0 = ((1.0 - t) * omega).sin() * inv_sin_omega;
        k1 = (t * omega).sin() * inv_sin_omega;
    }

    q[0] = qy[0] * k0 + qz2[0] * k1;
    q[1] = qy[1] * k0 + qz2[1] * k1;
    q[2] = qy[2] * k0 + qz2[2] * k1;
    q[3] = qy[3] * k0 + qz2[3] * k1;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_armat_default_initialization() {
        let mat = ARMat::default();
        assert_eq!(mat.m.len(), 0);
        assert_eq!(mat.row, 0);
        assert_eq!(mat.clm, 0);
    }

    #[test]
    fn test_armat_multiplication() {
        let mut a = ARMat::new(2, 3);
        a.m = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        // [ 1 2 3 ]
        // [ 4 5 6 ]

        let mut b = ARMat::new(3, 2);
        b.m = vec![7.0, 8.0, 9.0, 10.0, 11.0, 12.0];
        // [ 7   8 ]
        // [ 9  10 ]
        // [ 11 12 ]

        let result = (&a * &b).unwrap();

        assert_eq!(result.row, 2);
        assert_eq!(result.clm, 2);
        // Calculation:
        // [0][0] = 1*7 + 2*9 + 3*11 = 7 + 18 + 33 = 58
        // [0][1] = 1*8 + 2*10 + 3*12 = 8 + 20 + 36 = 64
        // [1][0] = 4*7 + 5*9 + 6*11 = 28 + 45 + 66 = 139
        // [1][1] = 4*8 + 5*10 + 6*12 = 32 + 50 + 72 = 154
        assert_eq!(result.m, vec![58.0, 64.0, 139.0, 154.0]);
    }

    #[test]
    fn test_armat_transpose() {
        let mut a = ARMat::new(2, 3);
        a.m = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        // [ 1 2 3 ]
        // [ 4 5 6 ]

        let result = a.transpose();
        assert_eq!(result.row, 3);
        assert_eq!(result.clm, 2);
        // [ 1 4 ]
        // [ 2 5 ]
        // [ 3 6 ]
        assert_eq!(result.m, vec![1.0, 4.0, 2.0, 5.0, 3.0, 6.0]);
    }

    #[test]
    fn test_armat_det() {
        let mut a = ARMat::new(3, 3);
        a.m = vec![6.0, 1.0, 1.0, 4.0, -2.0, 5.0, 2.0, 8.0, 7.0];

        let det = a.det();
        // Determinant of A:
        // 6 * (-14 - 40) - 1 * (28 - 10) + 1 * (32 - (-4))
        // 6 * (-54) - 18 + 36
        // -324 - 18 + 36 = -306
        assert_eq!(det.round(), -306.0);
    }

    #[test]
    fn test_armat_inv() {
        let mut a = ARMat::new(2, 2);
        a.m = vec![4.0, 7.0, 2.0, 6.0];

        // Inverse of [4 7; 2 6] is 1/(24-14) * [6 -7; -2 4] = 0.1 * [6 -7; -2 4]
        // = [0.6, -0.7; -0.2, 0.4]

        let inv_a = a.inv().expect("Failed to invert matrix");

        assert_eq!(inv_a.row, 2);
        assert_eq!(inv_a.clm, 2);

        let epsilon = 1e-6;
        assert!((inv_a.m[0] - 0.6).abs() < epsilon);
        assert!((inv_a.m[1] - (-0.7)).abs() < epsilon);
        assert!((inv_a.m[2] - (-0.2)).abs() < epsilon);
        assert!((inv_a.m[3] - 0.4).abs() < epsilon);
    }

    #[test]
    fn test_armatf_default_initialization() {
        let matf = ARMatf::default();
        assert_eq!(matf.m.len(), 0);
        assert_eq!(matf.row, 0);
        assert_eq!(matf.clm, 0);
    }

    #[test]
    fn test_arvec_default_initialization() {
        let vec = ARVec::default();
        assert_eq!(vec.v.len(), 0);
        assert_eq!(vec.clm, 0);
    }

    #[test]
    fn test_quaternion_conversion() {
        // Translation + 90 deg rotation around X
        // R = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
        let m = [
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 0.0, -1.0, 20.0],
            [0.0, 1.0, 0.0, 30.0],
        ];
        let mut q = [0.0; 4];
        let mut p = [0.0; 3];
        ar_util_mat2_quat_pos(&m, &mut q, &mut p);

        assert_eq!(p[0], 10.0);
        assert_eq!(p[1], 20.0);
        assert_eq!(p[2], 30.0);

        // Check rotation (90 deg around X => q = [cos(45), sin(45), 0, 0])
        let half_sqrt2 = 2.0_f64.sqrt() / 2.0;
        assert!((q[0] - half_sqrt2).abs() < 1e-10);
        assert!((q[1] - half_sqrt2).abs() < 1e-10);
        assert!((q[2]).abs() < 1e-10);
        assert!((q[3]).abs() < 1e-10);

        let mut m2 = [[0.0; 4]; 3];
        ar_util_quat_pos2_mat(&q, &p, &mut m2);

        for i in 0..3 {
            for j in 0..4 {
                assert!((m[i][j] - m2[i][j]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_quat_slerp() {
        let q1 = [1.0, 0.0, 0.0, 0.0]; // Identity
        let q2 = [0.0, 1.0, 0.0, 0.0]; // 180 deg around X (q = [cos(90), sin(90), 0, 0])

        let mut qr = [0.0; 4];
        ar_util_quat_slerp(&mut qr, &q1, &q2, 0.5);

        // Midpoint should be 90 deg around X
        let half_sqrt2 = 2.0_f64.sqrt() / 2.0;
        assert!((qr[0] - half_sqrt2).abs() < 1e-10);
        assert!((qr[1] - half_sqrt2).abs() < 1e-10);
        assert!((qr[2]).abs() < 1e-10);
        assert!((qr[3]).abs() < 1e-10);
    }
}

/// Multiply a 3x4 double matrix by a 3x4 float matrix, outputting a 3x4 float matrix.
/// Port of arUtilMatMuldff.
pub fn mat_mul_dff(a: &[[f64; 4]; 3], b: &[[f32; 4]; 3], dest: &mut [[f32; 4]; 3]) {
    for i in 0..3 {
        for j in 0..3 {
            dest[i][j] = (a[i][0] * b[0][j] as f64
                + a[i][1] * b[1][j] as f64
                + a[i][2] * b[2][j] as f64) as f32;
        }
        dest[i][3] = (a[i][0] * b[0][3] as f64
            + a[i][1] * b[1][3] as f64
            + a[i][2] * b[2][3] as f64
            + a[i][3]) as f32;
    }
}

/// Multiply two 3x4 float matrices, outputting a 3x4 float matrix.
pub fn mat_mul_fff(a: &[[f32; 4]; 3], b: &[[f32; 4]; 3], dest: &mut [[f32; 4]; 3]) {
    for i in 0..3 {
        for j in 0..3 {
            dest[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
        dest[i][3] = a[i][0] * b[0][3] + a[i][1] * b[1][3] + a[i][2] * b[2][3] + a[i][3];
    }
}
