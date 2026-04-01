use crate::backend::{FreakMatcherBackend, KpmError, QueryResult};

/// Central handle for KPM operations, generic over the backend implementation.
pub struct KpmHandle<B: FreakMatcherBackend> {
    backend: B,
}

impl<B: FreakMatcherBackend> KpmHandle<B> {
    pub fn with_backend(backend: B) -> Self {
        Self { backend }
    }

    pub fn query(
        &mut self,
        image: &[u8],
        width: usize,
        height: usize,
    ) -> Result<QueryResult, KpmError> {
        self.backend.query(image, width, height)
    }

    pub fn backend(&self) -> &B {
        &self.backend
    }

    pub fn backend_mut(&mut self) -> &mut B {
        &mut self.backend
    }
}
