/*
 *  matching.rs
 *  WebARKitLib-rs
 *
 *  This file is part of WebARKitLib-rs - WebARKit.
 *
 *  WebARKitLib-rs is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  WebARKitLib-rs is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with WebARKitLib-rs.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2026 WebARKit.
 *
 *  Author(s): Walter Perdan @kalwalt https://github.com/kalwalt
 *
 */

//! High-level matching result wrapper.
//!
//! [`MatchResult`] provides a convenient `Option`-like API around a
//! [`QueryResult`](crate::types::QueryResult), making it easy to check
//! whether a match was found and to extract the result.

use crate::types::QueryResult;

/// Outcome of a single matching operation.
///
/// Wraps an `Option<QueryResult>` and exposes helper methods to inspect
/// the match without pattern matching.
///
/// # Examples
///
/// ```rust
/// use webarkitlib_kpm::matching::MatchResult;
///
/// let miss = MatchResult::not_found();
/// assert!(!miss.is_match());
/// assert!(miss.result().is_none());
/// ```
pub struct MatchResult {
    inner: Option<QueryResult>,
}

impl MatchResult {
    /// Creates a successful match result.
    pub fn found(result: QueryResult) -> Self {
        Self {
            inner: Some(result),
        }
    }

    /// Creates a "no match" result.
    pub fn not_found() -> Self {
        Self { inner: None }
    }

    /// Returns `true` if a match was found.
    pub fn is_match(&self) -> bool {
        self.inner.is_some()
    }

    /// Returns a reference to the inner [`QueryResult`], if any.
    pub fn result(&self) -> Option<&QueryResult> {
        self.inner.as_ref()
    }

    /// Consumes `self` and returns the inner [`QueryResult`], if any.
    pub fn into_result(self) -> Option<QueryResult> {
        self.inner
    }
}
