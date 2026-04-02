/*
 *  backend.rs
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

//! Feature-matching backend trait and associated types.
//!
//! This module defines [`FreakMatcherBackend`], the pluggable interface
//! that decouples the KPM pipeline from any specific implementation.
//! The crate ships one concrete backend —
//! [`CppFreakMatcher`](crate::CppFreakMatcher) — which delegates to
//! the compiled C++ FreakMatcher library via FFI. A future pure-Rust
//! backend can be dropped in by implementing the same trait.

use std::fmt;

// ---------------------------------------------------------------------------
// Backend-specific data types
// ---------------------------------------------------------------------------

/// A 2D feature point detected in an image.
///
/// Holds the pixel position together with the keypoint's orientation
/// and scale, as computed by the detector (e.g. DoG + orientation
/// assignment).
#[derive(Debug, Default, Clone, PartialEq)]
pub struct FeaturePoint {
    /// Horizontal position in pixels.
    pub x: f32,
    /// Vertical position in pixels.
    pub y: f32,
    /// Dominant orientation in radians.
    pub angle: f32,
    /// Scale (pyramid octave level) at which the keypoint was detected.
    pub scale: f32,
    /// Whether this keypoint is a scale-space maximum.
    pub maxima: bool,
}

/// A 3D point in world coordinates (millimetres).
///
/// Computed from 2D pixel positions and the reference image's DPI. The
/// Z coordinate is always `0.0` for planar targets.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Point3d {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// An index pair representing a single feature match.
///
/// Links a feature in the query (input) image to a feature in the
/// reference database.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Match {
    /// Index into the query feature-point list.
    pub ins: usize,
    /// Index into the reference feature-point list.
    pub ref_: usize,
}

/// Lightweight result returned by [`FreakMatcherBackend::query`].
///
/// Contains only the matched page ID and inlier count. For the full
/// pose matrix, see [`KpmResult`](crate::types::KpmResult).
#[derive(Debug, Default, Clone, PartialEq)]
pub struct QueryResult {
    /// Page number of the best-matching reference image, or `-1` if no
    /// match was found.
    pub matched_id: i32,
    /// Number of RANSAC inliers supporting the match.
    pub inlier_count: usize,
}

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors that can occur during KPM operations.
#[derive(Debug)]
pub enum KpmError {
    /// The internal C++ handle pointer is null (freed or never created).
    NullHandle,
    /// The caller supplied invalid arguments (e.g. wrong buffer size).
    InvalidInput(String),
    /// An unrecoverable error inside the backend.
    InternalError(String),
    /// Too few feature points were detected for reliable matching.
    NotEnoughPoints {
        /// Number of points actually detected.
        got: usize,
        /// Minimum number required by the algorithm.
        need: usize,
    },
}

impl fmt::Display for KpmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            KpmError::NullHandle => write!(f, "KPM handle is null"),
            KpmError::InvalidInput(msg) => write!(f, "invalid input: {msg}"),
            KpmError::InternalError(msg) => write!(f, "internal error: {msg}"),
            KpmError::NotEnoughPoints { got, need } => {
                write!(f, "not enough points: got {got}, need at least {need}")
            }
        }
    }
}

impl std::error::Error for KpmError {}

// ---------------------------------------------------------------------------
// Backend trait
// ---------------------------------------------------------------------------

/// Pluggable feature-matching backend for the KPM pipeline.
///
/// This trait mirrors the public API of the C++ `VisualDatabaseFacade`
/// and allows swapping the C++ FFI backend for a pure-Rust
/// implementation without changing any calling code.
///
/// All implementations must be [`Send`] so that a [`KpmHandle`](crate::KpmHandle)
/// can be moved between threads.
///
/// # Implementors
///
/// * [`CppFreakMatcher`](crate::CppFreakMatcher) — FFI backend (default,
///   feature-gated behind `ffi-backend`).
pub trait FreakMatcherBackend: Send {
    /// Adds a grayscale reference image to the database.
    ///
    /// The backend extracts FREAK features internally, computes 3D world
    /// coordinates, and stores them under the given `image_id`.
    ///
    /// # Arguments
    ///
    /// * `image`    — grayscale pixel data (one byte per pixel, row-major).
    /// * `width`    — image width in pixels.
    /// * `height`   — image height in pixels.
    /// * `image_id` — unique identifier for this reference image.
    fn add_image(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
        image_id: usize,
    ) -> Result<(), KpmError>;

    /// Adds pre-extracted FREAK features and descriptors to the database.
    ///
    /// Use this when features have already been computed externally
    /// (e.g. loaded from a serialised file). The C++ FFI backend does
    /// **not** support this path and returns `Ok(())` as a no-op.
    ///
    /// # Arguments
    ///
    /// * `points`      — 2D feature-point positions.
    /// * `descriptors` — packed binary FREAK descriptors (96 bytes each).
    /// * `points_3d`   — corresponding 3D world coordinates.
    /// * `width`       — reference image width.
    /// * `height`      — reference image height.
    /// * `db_id`       — database slot identifier.
    fn add_freak_features(
        &mut self,
        points: &[FeaturePoint],
        descriptors: &[u8],
        points_3d: &[Point3d],
        width: usize,
        height: usize,
        db_id: usize,
    ) -> Result<(), KpmError>;

    /// Runs a query against the database using a grayscale camera frame.
    ///
    /// Returns a [`QueryResult`] with the best-matching page ID and
    /// inlier count. A `matched_id` of `-1` indicates no match.
    ///
    /// # Arguments
    ///
    /// * `image`  — grayscale pixel data of the camera frame.
    /// * `width`  — frame width in pixels.
    /// * `height` — frame height in pixels.
    fn query(&mut self, image: &[u8], width: usize, height: usize)
        -> Result<QueryResult, KpmError>;

    /// Returns the inlier matches from the most recent query.
    ///
    /// Each [`Match`] links a query feature index to a reference feature
    /// index. Returns an empty slice if the backend does not expose
    /// inlier data (e.g. the C++ FFI backend).
    fn inliers(&self) -> &[Match];

    /// Returns the page ID of the last successful match, or `-1`.
    fn matched_id(&self) -> i32;

    /// Returns the 2D feature points detected in the most recent query frame.
    ///
    /// Returns an empty slice if not available.
    fn query_feature_points(&self) -> &[FeaturePoint];

    /// Returns the 3D feature points for the given reference image.
    ///
    /// Returns an empty slice if not available.
    fn get_3d_feature_points(&self, image_id: usize) -> &[Point3d];
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kpm_error_display() {
        let errors: Vec<KpmError> = vec![
            KpmError::NullHandle,
            KpmError::InvalidInput("bad image".to_string()),
            KpmError::InternalError("segfault".to_string()),
            KpmError::NotEnoughPoints { got: 2, need: 4 },
        ];
        for err in &errors {
            let msg = format!("{err}");
            assert!(!msg.is_empty(), "Display for {err:?} should be non-empty");
        }
    }

    #[test]
    fn test_query_result_default() {
        let qr = QueryResult {
            matched_id: -1,
            inlier_count: 0,
        };
        assert_eq!(qr.matched_id, -1);
        assert_eq!(qr.inlier_count, 0);
    }
}
