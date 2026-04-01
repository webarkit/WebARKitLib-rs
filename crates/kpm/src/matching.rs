use crate::types::QueryResult;

/// Result of a matching operation.
pub struct MatchResult {
    inner: Option<QueryResult>,
}

impl MatchResult {
    pub fn found(result: QueryResult) -> Self {
        Self {
            inner: Some(result),
        }
    }

    pub fn not_found() -> Self {
        Self { inner: None }
    }

    pub fn is_match(&self) -> bool {
        self.inner.is_some()
    }

    pub fn result(&self) -> Option<&QueryResult> {
        self.inner.as_ref()
    }

    pub fn into_result(self) -> Option<QueryResult> {
        self.inner
    }
}
