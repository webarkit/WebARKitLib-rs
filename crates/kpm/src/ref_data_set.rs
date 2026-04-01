use crate::types::RefImage;

/// Collection of reference images for the KPM database.
pub struct RefDataSet {
    images: Vec<RefImage>,
}

impl RefDataSet {
    pub fn new() -> Self {
        Self { images: Vec::new() }
    }

    pub fn add(&mut self, image: RefImage) {
        self.images.push(image);
    }

    pub fn len(&self) -> usize {
        self.images.len()
    }

    pub fn is_empty(&self) -> bool {
        self.images.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &RefImage> {
        self.images.iter()
    }
}

impl Default for RefDataSet {
    fn default() -> Self {
        Self::new()
    }
}
