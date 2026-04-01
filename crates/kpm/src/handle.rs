use crate::backend::KpmBackend;
use crate::types::{QueryResult, RefImage};

/// Central handle for KPM operations, generic over the backend implementation.
pub struct KpmHandle<B: KpmBackend> {
    backend: B,
}

impl<B: KpmBackend> KpmHandle<B> {
    pub fn new(width: i32, height: i32) -> Self {
        Self {
            backend: B::new(width, height),
        }
    }

    pub fn add_ref_image(&mut self, image: &RefImage) -> Result<(), String> {
        self.backend.add_ref_image(image)
    }

    pub fn query(
        &mut self,
        gray_image: &[u8],
        width: i32,
        height: i32,
    ) -> Result<Option<QueryResult>, String> {
        self.backend.query(gray_image, width, height)
    }
}
