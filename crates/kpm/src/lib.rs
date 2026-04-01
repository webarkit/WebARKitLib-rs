pub mod backend;
pub mod handle;
pub mod kpm_ffi;
pub mod matching;
pub mod ref_data_set;
pub mod types;

#[cfg(feature = "ffi-backend")]
pub mod cpp_backend;

// Re-export key types for convenience.
pub use backend::KpmBackend;
pub use handle::KpmHandle;
pub use types::{Homography3x3, QueryResult, RefImage};

#[cfg(feature = "ffi-backend")]
pub use cpp_backend::CppBackend;

#[cfg(feature = "ffi-backend")]
pub type DefaultKpmHandle = KpmHandle<CppBackend>;
