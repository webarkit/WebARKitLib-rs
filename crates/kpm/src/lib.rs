pub mod kpm_ffi;
pub mod types;
pub mod backend;
pub mod handle;
pub mod ref_data_set;
pub mod matching;

#[cfg(feature = "ffi-backend")]
pub mod cpp_backend;

// Re-export key types for convenience.
pub use types::{Homography3x3, KpmConfig, QueryResult, RefImage};
pub use backend::KpmBackend;
pub use handle::KpmHandle;

#[cfg(feature = "ffi-backend")]
pub use cpp_backend::CppBackend;

#[cfg(feature = "ffi-backend")]
pub type DefaultKpmHandle = KpmHandle<CppBackend>;
