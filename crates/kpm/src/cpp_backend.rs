use crate::backend::KpmBackend;
use crate::kpm_ffi;
use crate::types::{Homography3x3, QueryResult, RefImage};

/// C++ FFI backend that delegates to the FreakMatcher library.
pub struct CppBackend {
    handle: *mut kpm_ffi::KpmOpaqueHandle,
}

impl KpmBackend for CppBackend {
    fn new(width: i32, height: i32) -> Self {
        let handle = unsafe { kpm_ffi::kpm_create(width, height) };
        assert!(!handle.is_null(), "kpm_create returned null");
        Self { handle }
    }

    fn add_ref_image(&mut self, image: &RefImage) -> Result<(), String> {
        let rc = unsafe {
            kpm_ffi::kpm_add_ref_image(
                self.handle,
                image.data.as_ptr(),
                image.width,
                image.height,
                image.dpi,
                image.page_no,
                image.image_no,
            )
        };
        if rc == 0 {
            Ok(())
        } else {
            Err("kpm_add_ref_image failed".to_string())
        }
    }

    fn query(
        &mut self,
        gray_image: &[u8],
        width: i32,
        height: i32,
    ) -> Result<Option<QueryResult>, String> {
        let expected_len = (width as usize) * (height as usize);
        if gray_image.len() < expected_len {
            return Err(format!(
                "buffer too small: got {} bytes, need {}",
                gray_image.len(),
                expected_len
            ));
        }

        let mut pose_out = [0.0f32; 12];
        let mut error_out: f32 = 0.0;
        let mut page_no_out: i32 = -1;

        let rc = unsafe {
            kpm_ffi::kpm_query(
                self.handle,
                gray_image.as_ptr(),
                width,
                height,
                pose_out.as_mut_ptr(),
                &mut error_out,
                &mut page_no_out,
            )
        };

        if rc == 0 {
            let mut h = [0.0f32; 9];
            h.copy_from_slice(&pose_out[..9]);
            Ok(Some(QueryResult {
                page_no: page_no_out,
                homography: Homography3x3(h),
                error: error_out,
            }))
        } else {
            Ok(None)
        }
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
