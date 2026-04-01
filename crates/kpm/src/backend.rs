use crate::types::{QueryResult, RefImage};

/// Trait abstracting the KPM feature-matching backend.
///
/// This allows swapping between a C++ FFI backend and a future pure-Rust
/// implementation behind the same interface.
pub trait KpmBackend {
    /// Create a new backend for images of the given dimensions.
    fn new(width: i32, height: i32) -> Self
    where
        Self: Sized;

    /// Add a reference image to the feature database.
    fn add_ref_image(&mut self, image: &RefImage) -> Result<(), String>;

    /// Query the database with a grayscale image.
    fn query(&mut self, gray_image: &[u8], width: i32, height: i32) -> Result<Option<QueryResult>, String>;
}
