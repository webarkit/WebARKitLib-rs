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
//! [`KpmOpaqueHandle`](crate::kpm_ffi::KpmOpaqueHandle) pointer, and
//! every `unsafe` block carries a `// SAFETY:` comment explaining why
//! the call is sound.

use crate::backend::{FeaturePoint, FreakMatcherBackend, KpmError, Match, Point3d, QueryResult};
use crate::kpm_ffi;

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
/// use webarkitlib_kpm::CppFreakMatcher;
///
/// let matcher = CppFreakMatcher::new(640, 480).expect("failed to create backend");
/// // Use `matcher` through the `FreakMatcherBackend` trait…
/// // Automatically freed on drop.
/// ```
pub struct CppFreakMatcher {
    /// Raw pointer to the C++ `KpmOpaqueHandle`.
    ptr: *mut kpm_ffi::KpmOpaqueHandle,
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
        Ok(Self { ptr })
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

    /// No-op for the C++ backend.
    ///
    /// The C API wrapper handles feature extraction internally inside
    /// `kpm_add_ref_image`, so pre-extracted features cannot be injected
    /// through the FFI.
    fn add_freak_features(
        &mut self,
        _points: &[FeaturePoint],
        _descriptors: &[u8],
        _points_3d: &[Point3d],
        _width: usize,
        _height: usize,
        _db_id: usize,
    ) -> Result<(), KpmError> {
        self.check_ptr()?;
        Ok(())
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

        if rc < 0 {
            Ok(QueryResult {
                matched_id: -1,
                inlier_count: 0,
            })
        } else {
            Ok(QueryResult {
                matched_id: page_no_out,
                inlier_count: 0,
            })
        }
    }

    /// Returns an empty slice — the C API does not expose inlier data.
    fn inliers(&self) -> &[Match] {
        &[]
    }

    /// Returns `-1` — the thin C wrapper does not cache the last matched ID.
    ///
    /// Callers should use the [`QueryResult`] returned by [`query`](FreakMatcherBackend::query) instead.
    fn matched_id(&self) -> i32 {
        -1
    }

    /// Returns an empty slice — the C API does not expose query feature points.
    fn query_feature_points(&self) -> &[FeaturePoint] {
        &[]
    }

    /// Returns an empty slice — the C API does not expose per-image 3D feature points.
    fn get_3d_feature_points(&self, _image_id: usize) -> &[Point3d] {
        &[]
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
