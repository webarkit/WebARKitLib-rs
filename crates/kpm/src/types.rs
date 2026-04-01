/// Configuration for the KPM subsystem.
pub struct KpmConfig {
    pub width: i32,
    pub height: i32,
}

/// A 3x3 homography matrix stored in row-major order.
#[derive(Debug, Clone, Copy)]
pub struct Homography3x3(pub [f32; 9]);

impl Default for Homography3x3 {
    fn default() -> Self {
        Self([0.0; 9])
    }
}

/// Result from a query operation.
#[derive(Debug, Clone)]
pub struct QueryResult {
    pub page_no: i32,
    pub homography: Homography3x3,
    pub error: f32,
}

/// A reference image to be added to the database.
pub struct RefImage {
    pub data: Vec<u8>,
    pub width: i32,
    pub height: i32,
    pub dpi: f32,
    pub page_no: i32,
    pub image_no: i32,
}
