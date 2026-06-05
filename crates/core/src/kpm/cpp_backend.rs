/*
 *  cpp_backend.rs
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

//! C++ FFI backend for the KPM pipeline.
//!
//! [`CppFreakMatcher`] implements [`FreakMatcherBackend`] by delegating
//! to the compiled FreakMatcher C++ static library through the thin
//! `extern "C"` wrapper defined in `kpm_c_api.h` / `kpm_c_api.cpp`.
//!
//! This module is only compiled when the **`ffi-backend`** feature is
//! enabled (the default).
//!
//! # Safety
//!
//! All FFI calls are guarded by a null-pointer check on the internal
//! [`KpmOpaqueHandle`](crate::kpm::kpm_ffi::KpmOpaqueHandle) pointer, and
//! every `unsafe` block carries a `// SAFETY:` comment explaining why
//! the call is sound.

use crate::kpm::backend::{
    FeaturePoint, FreakMatcherBackend, KpmError, Match, Point3d, QueryResult,
};
use crate::kpm::kpm_ffi;
use std::cell::UnsafeCell;

/// C++ FFI backend implementing [`FreakMatcherBackend`].
///
/// Wraps an opaque `KpmOpaqueHandle` pointer allocated on the C++ side.
/// The handle is freed automatically when the struct is dropped.
///
/// # Thread safety
///
/// The struct implements [`Send`] because the C++ handle is not shared;
/// ownership is transferred with the Rust struct. All trait methods take
/// `&mut self`, preventing concurrent access.
///
/// # Example
///
/// ```rust,no_run
/// use webarkitlib_rs::kpm::CppFreakMatcher;
///
/// let matcher = CppFreakMatcher::new(640, 480).expect("failed to create backend");
/// // Use `matcher` through the `FreakMatcherBackend` trait…
/// // Automatically freed on drop.
/// ```
pub struct CppFreakMatcher {
    /// Raw pointer to the C++ `KpmOpaqueHandle`.
    ptr: *mut kpm_ffi::KpmOpaqueHandle,
    /// Cached inlier matches from the most recent query.
    cached_inliers: Vec<Match>,
    /// Cached query feature points from the most recent query.
    cached_query_points: Vec<FeaturePoint>,
    /// Cached 3D feature points (populated on demand per image_id).
    /// Wrapped in `UnsafeCell` because `get_3d_feature_points` takes `&self`
    /// but needs to lazily populate the cache.
    cached_3d_points: UnsafeCell<Vec<Point3d>>,
    /// The image_id for which `cached_3d_points` was last populated.
    cached_3d_image_id: UnsafeCell<Option<usize>>,
    /// 3x3 row-major homography from the most recent successful query.
    /// `None` if the last query produced no match.
    ///
    /// Populated from `kpm_query`'s `pose_out[0..9]` (see
    /// `kpm_c_api.cpp:156-166` — that field carries the matched geometry
    /// as a 3x3 homography in the first 9 slots, with the trailing 3
    /// slots zero-padded for FFI convenience). Used by
    /// [`DualFreakMatcher`](crate::kpm::DualFreakMatcher) for the tier-2
    /// corner-reprojection divergence check (M9-2 #141, D16).
    cached_homography: Option<[f32; 9]>,
}

impl Drop for CppFreakMatcher {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            // SAFETY: `self.ptr` was allocated by `kpm_create` and has not been
            // freed yet. After this call the pointer is invalid, so we null it.
            unsafe {
                kpm_ffi::kpm_destroy(self.ptr);
            }
            self.ptr = std::ptr::null_mut();
        }
    }
}

// SAFETY: KpmOpaqueHandle is not shared between threads; ownership is
// transferred with the struct. All trait methods take `&mut self`, so no
// concurrent access is possible.
unsafe impl Send for CppFreakMatcher {}

impl CppFreakMatcher {
    /// Creates a new C++ FreakMatcher backend for the given frame size.
    ///
    /// # Arguments
    ///
    /// * `xsize` — expected camera frame width in pixels.
    /// * `ysize` — expected camera frame height in pixels.
    ///
    /// # Errors
    ///
    /// Returns [`KpmError::NullHandle`] if the C++ allocation fails.
    pub fn new(xsize: i32, ysize: i32) -> Result<Self, KpmError> {
        // SAFETY: `kpm_create` allocates a new handle on the C++ side.
        // It returns null only on allocation failure.
        let ptr = unsafe { kpm_ffi::kpm_create(xsize, ysize) };
        if ptr.is_null() {
            return Err(KpmError::NullHandle);
        }
        Ok(Self {
            ptr,
            cached_inliers: Vec::new(),
            cached_query_points: Vec::new(),
            cached_3d_points: UnsafeCell::new(Vec::new()),
            cached_3d_image_id: UnsafeCell::new(None),
            cached_homography: None,
        })
    }

    /// Borrow the 3x3 row-major homography from the most recent successful
    /// query. Returns `None` if the last query produced no match (or no
    /// query has been run yet).
    ///
    /// Same data the C++ `vision::VisualDatabase::matchedGeometry()`
    /// returns; cached on the Rust side after each
    /// [`FreakMatcherBackend::query`] call from `pose_out[0..9]`.
    ///
    /// Used by [`DualFreakMatcher`](crate::kpm::DualFreakMatcher) for the
    /// tier-2 corner-reprojection divergence check (M9-2 #141, D16). Not
    /// part of the [`FreakMatcherBackend`] trait — concrete-impl-only
    /// accessor.
    pub fn matched_geometry(&self) -> Option<&[f32; 9]> {
        self.cached_homography.as_ref()
    }

    /// Returns an error if the internal pointer is null (use-after-free guard).
    fn check_ptr(&self) -> Result<(), KpmError> {
        if self.ptr.is_null() {
            Err(KpmError::NullHandle)
        } else {
            Ok(())
        }
    }
}

impl FreakMatcherBackend for CppFreakMatcher {
    fn add_image(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
        image_id: usize,
    ) -> Result<(), KpmError> {
        self.check_ptr()?;

        let expected = width * height;
        if image.len() < expected {
            return Err(KpmError::InvalidInput(format!(
                "buffer too small: got {} bytes, need {expected}",
                image.len()
            )));
        }

        // SAFETY: `self.ptr` is valid (checked above). `image` points to at
        // least `width * height` bytes. The C++ side copies the data it needs.
        let rc = unsafe {
            kpm_ffi::kpm_add_ref_image(
                self.ptr,
                image.as_ptr(),
                width as i32,
                height as i32,
                72.0, // default DPI
                image_id as i32,
                0,
            )
        };

        if rc < 0 {
            Err(KpmError::InternalError(
                "kpm_add_ref_image failed".to_string(),
            ))
        } else {
            Ok(())
        }
    }

    fn add_freak_features(
        &mut self,
        points: &[FeaturePoint],
        descriptors: &[u8],
        points_3d: &[Point3d],
        width: usize,
        height: usize,
        db_id: usize,
    ) -> Result<(), KpmError> {
        self.check_ptr()?;

        let num = points.len();
        if num == 0 {
            return Ok(());
        }
        if descriptors.len() < num * 96 {
            return Err(KpmError::InvalidInput(format!(
                "descriptors too short: got {} bytes, need {}",
                descriptors.len(),
                num * 96
            )));
        }
        if points_3d.len() < num {
            return Err(KpmError::InvalidInput(format!(
                "points_3d too short: got {}, need {num}",
                points_3d.len()
            )));
        }

        // Pack into flat arrays for the C API.
        let mut flat_pts = vec![0.0f32; num * 4];
        let mut maxima = vec![0i32; num];
        let mut flat_3d = vec![0.0f32; num * 3];

        for (i, pt) in points.iter().enumerate() {
            flat_pts[i * 4] = pt.x;
            flat_pts[i * 4 + 1] = pt.y;
            flat_pts[i * 4 + 2] = pt.angle;
            flat_pts[i * 4 + 3] = pt.scale;
            maxima[i] = if pt.maxima { 1 } else { 0 };
        }
        for (i, p3) in points_3d.iter().enumerate() {
            flat_3d[i * 3] = p3.x;
            flat_3d[i * 3 + 1] = p3.y;
            flat_3d[i * 3 + 2] = p3.z;
        }

        // SAFETY: `self.ptr` is valid (checked above). All flat arrays have
        // the correct sizes. The C++ side copies the data it needs.
        let rc = unsafe {
            kpm_ffi::kpm_add_freak_features(
                self.ptr,
                flat_pts.as_ptr(),
                maxima.as_ptr(),
                descriptors.as_ptr(),
                flat_3d.as_ptr(),
                num as i32,
                width as i32,
                height as i32,
                db_id as i32,
            )
        };

        if rc < 0 {
            Err(KpmError::InternalError(
                "kpm_add_freak_features failed".to_string(),
            ))
        } else {
            Ok(())
        }
    }

    fn query(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
    ) -> Result<QueryResult, KpmError> {
        self.check_ptr()?;

        let expected = width * height;
        if image.len() < expected {
            return Err(KpmError::InvalidInput(format!(
                "buffer too small: got {} bytes, need {expected}",
                image.len()
            )));
        }

        let mut pose_out = [0.0f32; 12];
        let mut error_out: f32 = 0.0;
        let mut page_no_out: i32 = -1;

        // SAFETY: `self.ptr` is valid (checked above). `image` has at least
        // `width * height` bytes. Output pointers are stack-local and valid.
        let rc = unsafe {
            kpm_ffi::kpm_query(
                self.ptr,
                image.as_ptr(),
                width as i32,
                height as i32,
                pose_out.as_mut_ptr(),
                &mut error_out,
                &mut page_no_out,
            )
        };

        // Invalidate 3D cache — a new query may match a different image.
        // SAFETY: We have `&mut self`, so no other references exist.
        unsafe {
            *self.cached_3d_image_id.get() = None;
            (*self.cached_3d_points.get()).clear();
        }

        if rc < 0 {
            self.cached_inliers.clear();
            self.cached_query_points.clear();
            self.cached_homography = None;
            return Ok(QueryResult {
                matched_id: -1,
                inlier_count: 0,
            });
        }

        // Cache the 3x3 homography from `pose_out[0..9]` (M9-2 #141, D16).
        // The trailing `pose_out[9..12]` slots are FFI zero-padding per
        // `kpm_c_api.cpp:156-166`.
        self.cached_homography = Some([
            pose_out[0],
            pose_out[1],
            pose_out[2],
            pose_out[3],
            pose_out[4],
            pose_out[5],
            pose_out[6],
            pose_out[7],
            pose_out[8],
        ]);

        // Populate inlier cache.
        // SAFETY: `self.ptr` is valid. The C function writes at most `count` pairs.
        let inlier_count = unsafe { kpm_ffi::kpm_get_inlier_count(self.ptr) } as usize;
        self.cached_inliers.clear();
        if inlier_count > 0 {
            let mut ins = vec![0i32; inlier_count];
            let mut refs = vec![0i32; inlier_count];
            unsafe {
                kpm_ffi::kpm_get_inliers(self.ptr, ins.as_mut_ptr(), refs.as_mut_ptr());
            }
            self.cached_inliers = ins
                .iter()
                .zip(refs.iter())
                .map(|(&i, &r)| Match {
                    ins: i as usize,
                    ref_: r as usize,
                })
                .collect();
        }

        // Populate query feature point cache.
        let qf_count = unsafe { kpm_ffi::kpm_get_query_feature_count(self.ptr) } as usize;
        self.cached_query_points.clear();
        if qf_count > 0 {
            let mut xs = vec![0.0f32; qf_count];
            let mut ys = vec![0.0f32; qf_count];
            unsafe {
                kpm_ffi::kpm_get_query_feature_points(self.ptr, xs.as_mut_ptr(), ys.as_mut_ptr());
            }
            self.cached_query_points = xs
                .iter()
                .zip(ys.iter())
                .map(|(&x, &y)| FeaturePoint {
                    x,
                    y,
                    ..Default::default()
                })
                .collect();
        }

        Ok(QueryResult {
            matched_id: page_no_out,
            inlier_count,
        })
    }

    fn inliers(&self) -> &[Match] {
        &self.cached_inliers
    }

    fn matched_id(&self) -> i32 {
        if self.ptr.is_null() {
            return -1;
        }
        // SAFETY: `self.ptr` is valid (null-checked above).
        unsafe { kpm_ffi::kpm_matched_id(self.ptr) }
    }

    fn query_feature_points(&self) -> &[FeaturePoint] {
        &self.cached_query_points
    }

    fn extract_features(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
    ) -> Result<(Vec<FeaturePoint>, Vec<u8>), KpmError> {
        self.check_ptr()?;

        let expected = width * height;
        if image.len() < expected {
            return Err(KpmError::InvalidInput(format!(
                "buffer too small: got {} bytes, need {expected}",
                image.len()
            )));
        }

        // Step 1: count-only call (NULL output arrays).
        // SAFETY: `self.ptr` is valid. Passing null output pointers is the
        // documented "count only" mode defined in kpm_c_api.h.
        let count = unsafe {
            kpm_ffi::kpm_extract_features(
                self.ptr,
                image.as_ptr(),
                width as i32,
                height as i32,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                0,
            )
        };

        if count < 0 {
            return Err(KpmError::InternalError(
                "kpm_extract_features failed".to_string(),
            ));
        }
        if count == 0 {
            return Ok((vec![], vec![]));
        }

        let n = count as usize;
        let mut xs = vec![0.0f32; n];
        let mut ys = vec![0.0f32; n];
        let mut angles = vec![0.0f32; n];
        let mut scales = vec![0.0f32; n];
        let mut maxima = vec![0i32; n];
        let mut descs = vec![0u8; n * 96];

        // Step 2: fill all output arrays.
        // SAFETY: `self.ptr` is valid. All output buffers have exactly `n`
        // elements / n*96 bytes. FREAK extraction is deterministic on the
        // same image, so the count cannot increase between the two calls.
        unsafe {
            kpm_ffi::kpm_extract_features(
                self.ptr,
                image.as_ptr(),
                width as i32,
                height as i32,
                xs.as_mut_ptr(),
                ys.as_mut_ptr(),
                angles.as_mut_ptr(),
                scales.as_mut_ptr(),
                maxima.as_mut_ptr(),
                descs.as_mut_ptr(),
                n as i32,
            );
        }

        let points = (0..n)
            .map(|i| FeaturePoint {
                x: xs[i],
                y: ys[i],
                angle: angles[i],
                scale: scales[i],
                maxima: maxima[i] != 0,
            })
            .collect();

        Ok((points, descs))
    }

    fn get_3d_feature_points(&self, image_id: usize) -> &[Point3d] {
        // SAFETY: No other thread can access `self` concurrently — the struct
        // is `Send` but not `Sync`, and all `&mut self` methods are exclusive.
        // The `UnsafeCell` provides interior mutability for this lazy cache.
        unsafe {
            // Return cached data if we already fetched for this image_id.
            if *self.cached_3d_image_id.get() == Some(image_id) {
                return &*self.cached_3d_points.get();
            }

            if self.ptr.is_null() {
                return &[];
            }

            let count = kpm_ffi::kpm_get_3d_feature_count(self.ptr, image_id as i32) as usize;

            if count == 0 {
                return &[];
            }

            let mut xs = vec![0.0f32; count];
            let mut ys = vec![0.0f32; count];
            let mut zs = vec![0.0f32; count];
            kpm_ffi::kpm_get_3d_feature_points(
                self.ptr,
                image_id as i32,
                xs.as_mut_ptr(),
                ys.as_mut_ptr(),
                zs.as_mut_ptr(),
            );

            let pts = self.cached_3d_points.get();
            *pts = xs
                .iter()
                .zip(ys.iter())
                .zip(zs.iter())
                .map(|((&x, &y), &z)| Point3d { x, y, z })
                .collect();
            *self.cached_3d_image_id.get() = Some(image_id);

            &*pts
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cpp_freak_matcher_new_and_drop() {
        let matcher = CppFreakMatcher::new(640, 480);
        assert!(matcher.is_ok());
        // Drop runs implicitly — should not panic.
    }

    #[test]
    fn test_cpp_freak_matcher_send() {
        fn assert_send<T: Send>() {}
        assert_send::<CppFreakMatcher>();
    }
}
