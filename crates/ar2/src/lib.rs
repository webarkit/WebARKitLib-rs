/*
 *  lib.rs
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

//! # webarkitlib-ar2
//!
//! AR2 NFT (Natural Feature Tracking) marker I/O for WebARKitLib-rs.
//!
//! This crate provides read support for the binary marker files used by
//! AR2-based NFT tracking:
//!
//! - **Image set I/O** — load `.iset` image pyramid files
//!   (ported from `AR2/imageSet.c`).
//! - **Feature set I/O** — load `.fset` feature files
//!   (ported from `AR2/featureSet.c`).
//!
//! ## NFT marker files
//!
//! | Extension | Contents | Crate |
//! |-----------|----------|-------|
//! | `.iset`   | Image pyramid (multiple DPI scales) | this crate |
//! | `.fset`   | AR2 feature points per scale | this crate |
//! | `.fset3`  | KPM FREAK descriptors | `webarkitlib-kpm` |
//!
//! The runtime tracking structs and algorithms remain in
//! `webarkitlib-rs` (the core crate) under `ar2::*`.

pub mod feature_set;
pub mod image_set;

pub use feature_set::AR2FeatureSetT;
pub use image_set::AR2ImageSetT;
