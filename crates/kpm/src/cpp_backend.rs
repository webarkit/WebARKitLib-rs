use crate::backend::{FeaturePoint, FreakMatcherBackend, KpmError, Match, Point3d, QueryResult};
use crate::kpm_ffi;

/// C++ FFI backend that delegates to the FreakMatcher library.
pub struct CppBackend {
    handle: *mut kpm_ffi::KpmOpaqueHandle,
    last_inliers: Vec<Match>,
    last_matched_id: i32,
    last_query_points: Vec<FeaturePoint>,
}

impl CppBackend {
    pub fn new(width: usize, height: usize) -> Result<Self, KpmError> {
        let handle = unsafe { kpm_ffi::kpm_create(width as i32, height as i32) };
        if handle.is_null() {
            return Err(KpmError::NullHandle);
        }
        Ok(Self {
            handle,
            last_inliers: Vec::new(),
            last_matched_id: -1,
            last_query_points: Vec::new(),
        })
    }
}

impl FreakMatcherBackend for CppBackend {
    fn add_image(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
        image_id: usize,
    ) -> Result<(), KpmError> {
        let expected = width * height;
        if image.len() < expected {
            return Err(KpmError::InvalidInput(format!(
                "buffer too small: got {} bytes, need {expected}",
                image.len()
            )));
        }
        let rc = unsafe {
            kpm_ffi::kpm_add_ref_image(
                self.handle,
                image.as_ptr(),
                width as i32,
                height as i32,
                72.0, // default DPI
                image_id as i32,
                0,
            )
        };
        if rc == 0 {
            Ok(())
        } else {
            Err(KpmError::InternalError(
                "kpm_add_ref_image failed".to_string(),
            ))
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
        // The C API wrapper handles feature extraction internally in
        // kpm_add_ref_image, so this is a no-op for the FFI backend.
        Ok(())
    }

    fn query(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
    ) -> Result<QueryResult, KpmError> {
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

        let rc = unsafe {
            kpm_ffi::kpm_query(
                self.handle,
                image.as_ptr(),
                width as i32,
                height as i32,
                pose_out.as_mut_ptr(),
                &mut error_out,
                &mut page_no_out,
            )
        };

        if rc == 0 {
            self.last_matched_id = page_no_out;
            Ok(QueryResult {
                matched_id: page_no_out,
                inlier_count: 0,
            })
        } else {
            self.last_matched_id = -1;
            Ok(QueryResult {
                matched_id: -1,
                inlier_count: 0,
            })
        }
    }

    fn inliers(&self) -> &[Match] {
        &self.last_inliers
    }

    fn matched_id(&self) -> i32 {
        self.last_matched_id
    }

    fn query_feature_points(&self) -> &[FeaturePoint] {
        &self.last_query_points
    }

    fn get_3d_feature_points(&self, _image_id: usize) -> &[Point3d] {
        &[]
    }
}

impl Drop for CppBackend {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe {
                kpm_ffi::kpm_destroy(self.handle);
            }
            self.handle = std::ptr::null_mut();
        }
    }
}

// Safety: The C++ handle is not thread-safe, but Send is fine
// since we only access it from one thread at a time via &mut self.
unsafe impl Send for CppBackend {}
