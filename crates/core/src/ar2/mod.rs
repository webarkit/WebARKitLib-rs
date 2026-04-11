/*
 *  ar2/mod.rs
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

//! # AR2 — NFT Tracking and Marker I/O
//!
//! This module consolidates all AR2 (Natural Feature Tracking) functionality:
//!
//! | Sub-module       | Purpose |
//! |------------------|---------|
//! | [`tracking`]     | Runtime tracking structs and algorithms (ported from AR2) |
//! | [`surface`]      | Surface set loading and helpers (ported from `AR2/surface.c`) |
//! | [`image_set`]    | `.iset` image pyramid I/O (ported from `AR2/imageSet.c`) |
//! | [`feature_set`]  | `.fset` feature point I/O (ported from `AR2/featureSet.c`) |
//!
//! ## Backward compatibility
//!
//! All public items from [`tracking`] and [`surface`] are re-exported at this
//! module level, so existing code using e.g. `webarkitlib_rs::ar2::AR2Handle`
//! or `webarkitlib_rs::ar2::ar2_read_surface_set` continues to work.

pub mod feature_map;
pub mod feature_set;
pub mod image_set;
pub mod surface;
pub mod tracking;

// ---------------------------------------------------------------------------
// Shared error type
// ---------------------------------------------------------------------------

/// Errors returned by the AR2 generation pipeline.
///
/// Defined here (rather than in `feature_map` or `image_set`) to avoid a
/// circular module dependency: both sub-modules produce this error type.
#[derive(Debug)]
pub enum Ar2Error {
    /// The caller supplied invalid parameters.
    InvalidInput(&'static str),
}

impl std::fmt::Display for Ar2Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Ar2Error::InvalidInput(msg) => write!(f, "invalid input: {}", msg),
        }
    }
}

impl std::error::Error for Ar2Error {}

// ---------------------------------------------------------------------------
// Re-exports
// ---------------------------------------------------------------------------

// Re-export everything from tracking for backward compatibility.
// Before this consolidation, ar2.rs *was* the tracking module,
// so all its public items lived at crate::ar2::*.
pub use self::tracking::*;

// Re-export everything from surface for backward compatibility.
pub use self::surface::*;

// Convenience re-exports from the I/O sub-modules.
pub use feature_map::ar2_gen_feature_map;
pub use feature_set::AR2FeatureSetT;
pub use image_set::{ar2_gen_image_set, AR2ImageSetT};
