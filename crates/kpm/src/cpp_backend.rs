use crate::backend::{FeaturePoint, FreakMatcherBackend, KpmError, Match, Point3d, QueryResult};
use crate::kpm_ffi;

/// C++ FFI backend implementing `FreakMatcherBackend` by delegating to the
/// compiled FreakMatcher static library through `kpm_c_api.h` bindings.
pub struct CppFreakMatcher {
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
        // The C API wrapper handles feature extraction internally inside
        // kpm_add_ref_image, so pre-extracted features cannot be injected
        // through the FFI. This is a no-op for the C++ backend.
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

    fn inliers(&self) -> &[Match] {
        // The C API does not expose inlier data; return empty slice.
        &[]
    }

    fn matched_id(&self) -> i32 {
        // Without cached state the last matched ID is not available through
        // the thin C wrapper. Callers should use the QueryResult instead.
        -1
    }

    fn query_feature_points(&self) -> &[FeaturePoint] {
        // The C API does not expose query feature points.
        &[]
    }

    fn get_3d_feature_points(&self, _image_id: usize) -> &[Point3d] {
        // The C API does not expose per-image 3D feature points.
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
