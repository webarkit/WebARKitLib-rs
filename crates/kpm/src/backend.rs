use std::fmt;

// --- Backend-specific types ---

#[derive(Debug, Default, Clone, PartialEq)]
pub struct FeaturePoint {
    pub x: f32,
    pub y: f32,
    pub angle: f32,
    pub scale: f32,
    pub maxima: bool,
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Point3d {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Match {
    pub ins: usize,
    pub ref_: usize,
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct QueryResult {
    pub matched_id: i32,
    pub inlier_count: usize,
}

// --- Error type ---

#[derive(Debug)]
pub enum KpmError {
    NullHandle,
    InvalidInput(String),
    InternalError(String),
    NotEnoughPoints { got: usize, need: usize },
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

// --- Backend trait ---

/// Abstraction over the feature-matching backend.
///
/// This trait mirrors the `VisualDatabaseFacade` C++ API and allows
/// swapping the C++ FFI backend for a pure-Rust implementation without
/// changing any calling code.
pub trait FreakMatcherBackend: Send {
    fn add_image(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
        image_id: usize,
    ) -> Result<(), KpmError>;

    fn add_freak_features(
        &mut self,
        points: &[FeaturePoint],
        descriptors: &[u8],
        points_3d: &[Point3d],
        width: usize,
        height: usize,
        db_id: usize,
    ) -> Result<(), KpmError>;

    fn query(&mut self, image: &[u8], width: usize, height: usize)
        -> Result<QueryResult, KpmError>;

    fn inliers(&self) -> &[Match];

    fn matched_id(&self) -> i32;

    fn query_feature_points(&self) -> &[FeaturePoint];

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
