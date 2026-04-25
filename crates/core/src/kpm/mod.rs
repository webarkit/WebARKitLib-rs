/*
 *  kpm/mod.rs
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

//! KPM (Key-Point Matching) / NFT (Natural Feature Tracking) layer,
//! ported from the C++ FreakMatcher library.
//!
//! This module provides a Rust interface to the FREAK-descriptor-based
//! feature matching pipeline used for planar image tracking. It supports
//! two operation modes:
//!
//! - **`ffi-backend`** (default) — delegates to the compiled C++ FreakMatcher
//!   static library through a thin `extern "C"` wrapper (`kpm_c_api.h`).
//! - *(future)* **pure-Rust** — a native Rust re-implementation of the same
//!   pipeline, swappable via the [`FreakMatcherBackend`] trait.
//!
//! ## Quick start
//!
//! ```rust,ignore
//! use webarkitlib_rs::kpm::{CppFreakMatcher, KpmHandle};
//!
//! // Create a C++ backend for a 640x480 camera frame.
//! let backend = CppFreakMatcher::new(640, 480).unwrap();
//!
//! // Wrap it in a KpmHandle (the central coordinator).
//! let mut handle = KpmHandle::new(640, 480, None, Box::new(backend));
//! ```
//!
//! ## Module layout
//!
//! | Module           | Purpose |
//! |------------------|---------|
//! | [`backend`]      | `FreakMatcherBackend` trait and associated error / data types |
//! | [`handle`]       | [`KpmHandle`] struct — central coordinator for KPM operations |
//! | [`types`]        | Ported C structs (`KpmRefDataSet`, `KpmResult`, etc.) |
//! | [`cpp_backend`]  | [`CppFreakMatcher`] — FFI backend (feature-gated) |
//! | [`kpm_ffi`]      | Raw bindgen bindings (feature-gated) |
//! | [`matching`]     | [`MatchResult`](matching::MatchResult) wrapper |
//! | [`ref_data_set`] | [`RefDataSet`](ref_data_set::RefDataSet) collection |

pub mod backend;
pub mod freak;
pub mod handle;
pub mod kpm_ffi;
pub mod matching;
pub mod ref_data_set;
pub mod types;

#[cfg(feature = "ffi-backend")]
pub mod cpp_backend;

// Re-export key types for convenience.
pub use backend::QueryResult;
pub use backend::{FeaturePoint, FreakMatcherBackend, KpmError, Match, Point3d};
pub use handle::{KpmHandle, KpmPoseMode, KpmProcMode};
pub use ref_data_set::KpmCompMode;
pub use types::{Homography3x3, RefImage};

#[cfg(feature = "ffi-backend")]
pub use cpp_backend::CppFreakMatcher;
