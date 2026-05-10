/*
 *  ar/mod.rs
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

//! # AR — Core Augmented Reality Module
//!
//! Ported from `lib/SRC/AR/` in the original WebARKitLib C/C++ codebase.
//!
//! This module bundles all algorithms that belong to the foundational ARToolKit
//! detection and tracking pipeline: image labeling, marker detection, pattern
//! matching, camera parameters, matrix/vector math, image processing, pose
//! estimation, and matrix-code (barcode) decoding.
//!
//! ## Submodules
//!
//! | Submodule | C/C++ origin | What it provides |
//! |---|---|---|
//! | [`bch`] | `ar.h` BCH tables | BCH / Hamming error-correction for matrix-code markers |
//! | [`filter`] | `arFilterTransMat.c` | One-Euro low-pass filter for smoothing transformation matrices over time |
//! | [`image_proc`] | `arImageProc.c` | RGBA→grayscale conversion, box filtering, adaptive thresholding (`ARImageProcInfo`) |
//! | [`labeling`] | `arLabeling.c`, `arLabelingSub/` | Connected-component labeling; produces `ARLabelInfo` used by the marker pipeline |
//! | [`marker`] | `arDetectMarker.c`, `arDetectMarker2.c`, `arGetLine.c`, `arGetMarkerInfo.c` | Full marker detection pipeline: contour extraction, corner fitting, candidate filtering |
//! | [`math`] | `m*.c` (16 files), `v*.c` (6 files) | `ARMat` / `ARVec` linear algebra: inversion, QR, PCA, quaternion conversion |
//! | [`matrix`] | `arGetMatrixCode.c` | Matrix-code (barcode) marker decoding and ECC |
//! | [`param`] | `param*.c` (10 files) | Camera intrinsic parameters: `ARParam`, `ARParamLTf`, lens-distortion helpers |
//! | [`param_gl`] | `paramGL.c` | OpenGL projection helpers: converts `ARParam` to a right-handed frustum matrix |
//! | [`pattern`] | `arPatt*.c` | Pattern template loading, normalisation, and ID matching |
//! | [`pose`] | `ar3DCreateHandle.c`, `arGetTransMat.c`, `arGetTransMatStereo.c` | 3-D pose estimation from marker corners; wraps into `AR3DHandle` |
//!
//! ## Dependency note
//!
//! All submodules share the common types from [`crate::types`] (kept at the
//! crate root because `ar2` and `kpm` depend on them too).  [`pose`] calls
//! into [`crate::icp`] for ICP-based pose refinement — mirroring the
//! `lib/SRC/AR/` → `lib/SRC/ARICP/` dependency that exists in the original C.

pub mod bch;
pub mod filter;
pub mod image_proc;
pub mod labeling;
pub mod marker;
pub mod math;
pub mod matrix;
pub mod param;
pub mod param_gl;
pub mod pattern;
pub mod pose;
