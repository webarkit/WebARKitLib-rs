/*
 *  matcher.rs
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
 *  Author(s): Walter Perdan <@kalwalt> https://github.com/kalwalt
 *
 */

//! Binary feature matching using ratio test on FREAK descriptors.
//!
//! This module ports the C++ `BinaryFeatureStore` and `BinaryFeatureMatcher` classes
//! from `feature_store.h`, `feature_matcher.h`, and `feature_matcher-inline.h`.
//! The matcher implements three variants:
//! - [`FeatureMatcher::match_all`]: brute force comparison
//! - [`FeatureMatcher::match_indexed`]: BHC-backed candidate lookup
//! - [`FeatureMatcher::match_guided`]: homography-guided spatial filter
//!
//! All three apply a ratio test (1st best / 2nd best < threshold) and filter
//! by `FeaturePoint::maxima` for parity with the C++ implementation.

use crate::kpm::backend::KpmError;
use crate::{arlog_d, arlog_e};

use super::clustering::{hamming_distance_96, BinaryHierarchicalClustering};
use super::homography::matrix_inverse_3x3;
use super::hough::{FeaturePoint, Match};

/// Default ratio test threshold (matches C++ `BinaryFeatureMatcher::mThreshold`).
const DEFAULT_THRESHOLD: f32 = 0.7;

/// Number of bytes per FREAK descriptor.
const FEATURE_SIZE: usize = 96;

// ============================================================================
// FeatureStore
// ============================================================================

/// Container for feature points and their FREAK descriptors.
///
/// C equivalent: `vision::BinaryFeatureStore`
pub struct FeatureStore {
    points: Vec<FeaturePoint>,
    descriptors: Vec<u8>,
    bytes_per_feature: usize,
}

impl FeatureStore {
    /// Creates an empty feature store with descriptors sized to `bytes_per_feature`.
    pub fn new(bytes_per_feature: usize) -> Result<Self, KpmError> {
        if bytes_per_feature == 0 {
            arlog_e!("FeatureStore::new: bytes_per_feature must be > 0");
            return Err(KpmError::InvalidInput(
                "bytes_per_feature must be > 0".into(),
            ));
        }
        Ok(Self {
            points: Vec::new(),
            descriptors: Vec::new(),
            bytes_per_feature,
        })
    }

    /// Adds a feature point and its descriptor to the store.
    ///
    /// Returns an error if `descriptor.len() != bytes_per_feature`.
    pub fn add(&mut self, point: FeaturePoint, descriptor: &[u8]) -> Result<(), KpmError> {
        if descriptor.len() != self.bytes_per_feature {
            arlog_e!(
                "FeatureStore::add: descriptor length {} does not match bytes_per_feature {}",
                descriptor.len(),
                self.bytes_per_feature
            );
            return Err(KpmError::InvalidInput(
                "descriptor length does not match bytes_per_feature".into(),
            ));
        }
        self.points.push(point);
        self.descriptors.extend_from_slice(descriptor);
        Ok(())
    }

    /// Returns the number of features in the store.
    pub fn num_features(&self) -> usize {
        self.points.len()
    }

    /// Returns the descriptor at index `i` as a byte slice.
    ///
    /// # Panics
    /// Panics if `i >= num_features()`.
    pub fn descriptor(&self, i: usize) -> &[u8] {
        let start = i * self.bytes_per_feature;
        &self.descriptors[start..start + self.bytes_per_feature]
    }

    /// Returns the feature point at index `i`.
    ///
    /// # Panics
    /// Panics if `i >= num_features()`.
    pub fn point(&self, i: usize) -> &FeaturePoint {
        &self.points[i]
    }

    /// Returns the number of bytes per feature descriptor.
    pub fn bytes_per_feature(&self) -> usize {
        self.bytes_per_feature
    }
}

// ============================================================================
// FeatureMatcher
// ============================================================================

/// Matches feature stores using a ratio test on Hamming distance.
///
/// C equivalent: `vision::BinaryFeatureMatcher<96>`
pub struct FeatureMatcher {
    index: Option<BinaryHierarchicalClustering>,
    threshold: f32,
    matches: Vec<Match>,
}

impl Default for FeatureMatcher {
    fn default() -> Self {
        Self::new()
    }
}

impl FeatureMatcher {
    /// Creates a new matcher with default threshold (0.7) and no built index.
    pub fn new() -> Self {
        Self {
            index: None,
            threshold: DEFAULT_THRESHOLD,
            matches: Vec::new(),
        }
    }

    /// Sets the ratio test threshold (typical values: 0.6–0.8).
    pub fn set_threshold(&mut self, threshold: f32) {
        self.threshold = threshold;
    }

    /// Returns the current ratio test threshold.
    pub fn threshold(&self) -> f32 {
        self.threshold
    }

    /// Returns the matches from the last successful call to a `match_*` method.
    pub fn matches(&self) -> &[Match] {
        &self.matches
    }

    /// Builds the BHC index from a reference feature store.
    ///
    /// Required before calling [`Self::match_indexed`].
    pub fn build(&mut self, store: &FeatureStore) -> Result<(), KpmError> {
        if store.bytes_per_feature() != FEATURE_SIZE {
            arlog_e!(
                "FeatureMatcher::build: only {}-byte features supported, got {}",
                FEATURE_SIZE,
                store.bytes_per_feature()
            );
            return Err(KpmError::InvalidInput(
                "Only 96-byte features supported for indexed matching".into(),
            ));
        }
        if store.num_features() == 0 {
            arlog_e!("FeatureMatcher::build: empty store");
            return Err(KpmError::InvalidInput(
                "Cannot build index from empty store".into(),
            ));
        }

        let descriptors: Vec<&[u8; 96]> = (0..store.num_features())
            .map(|i| {
                let bytes = store.descriptor(i);
                <&[u8; 96]>::try_from(bytes).expect("descriptor size verified above")
            })
            .collect();

        let mut bhc = BinaryHierarchicalClustering::new()?;
        bhc.build(&descriptors)?;
        self.index = Some(bhc);

        arlog_d!(
            "FeatureMatcher::build: index built from {} reference features",
            store.num_features()
        );
        Ok(())
    }

    /// Brute force match: compare every query feature against every reference.
    ///
    /// C equivalent: `match(features1, features2)`
    #[inline(never)]
    pub fn match_all(
        &mut self,
        query: &FeatureStore,
        reference: &FeatureStore,
    ) -> Result<usize, KpmError> {
        self.matches.clear();
        if query.num_features() == 0 || reference.num_features() == 0 {
            return Ok(0);
        }
        Self::ensure_feature_size(query, reference)?;

        self.matches.reserve(query.num_features());

        for i in 0..query.num_features() {
            let mut first_best = u32::MAX;
            let mut second_best = u32::MAX;
            let mut best_index: i32 = -1;
            let f1 = descriptor_96(query, i);
            let p1 = query.point(i);

            for j in 0..reference.num_features() {
                if p1.maxima != reference.point(j).maxima {
                    continue;
                }
                let f2 = descriptor_96(reference, j);
                let d = hamming_distance_96(f1, f2);
                if d < first_best {
                    second_best = first_best;
                    first_best = d;
                    best_index = j as i32;
                } else if d < second_best {
                    second_best = d;
                }
            }

            self.apply_ratio_test(i, best_index, first_best, second_best);
        }

        arlog_d!(
            "FeatureMatcher::match_all: {} matches from {} queries",
            self.matches.len(),
            query.num_features()
        );
        Ok(self.matches.len())
    }

    /// BHC-indexed match: uses the prebuilt index for fast candidate lookup.
    ///
    /// [`Self::build`] must have been called before this method.
    /// C equivalent: `match(features1, features2, index2)`
    #[inline(never)]
    pub fn match_indexed(
        &mut self,
        query: &FeatureStore,
        reference: &FeatureStore,
    ) -> Result<usize, KpmError> {
        self.matches.clear();
        if query.num_features() == 0 || reference.num_features() == 0 {
            return Ok(0);
        }
        Self::ensure_feature_size(query, reference)?;

        if self.index.is_none() {
            arlog_e!("FeatureMatcher::match_indexed: index not built");
            return Err(KpmError::InternalError("index not built".into()));
        }

        self.matches.reserve(query.num_features());

        for i in 0..query.num_features() {
            let mut first_best = u32::MAX;
            let mut second_best = u32::MAX;
            let mut best_index: i32 = -1;
            let f1 = descriptor_96(query, i);
            let p1 = query.point(i);

            // Tight scope: index borrow released after query() returns.
            let candidates = self.index.as_ref().expect("checked above").query(f1)?;

            for &j in candidates.iter() {
                if p1.maxima != reference.point(j).maxima {
                    continue;
                }
                let f2 = descriptor_96(reference, j);
                let d = hamming_distance_96(f1, f2);
                if d < first_best {
                    second_best = first_best;
                    first_best = d;
                    best_index = j as i32;
                } else if d < second_best {
                    second_best = d;
                }
            }

            self.apply_ratio_test(i, best_index, first_best, second_best);
        }

        arlog_d!(
            "FeatureMatcher::match_indexed: {} matches from {} queries",
            self.matches.len(),
            query.num_features()
        );
        Ok(self.matches.len())
    }

    /// Homography-guided match: restricts candidates to those near the warped query position.
    ///
    /// `h` is the 3x3 homography from query to reference (row-major).
    /// `tr` is the spatial threshold in pixels.
    /// C equivalent: `match(features1, features2, H, tr)`
    #[inline(never)]
    pub fn match_guided(
        &mut self,
        query: &FeatureStore,
        reference: &FeatureStore,
        h: &[f32; 9],
        tr: f32,
    ) -> Result<usize, KpmError> {
        self.matches.clear();
        if query.num_features() == 0 || reference.num_features() == 0 {
            return Ok(0);
        }
        Self::ensure_feature_size(query, reference)?;

        // 1e-20 preserves the historic match_guided behavior (almost never reject).
        let h_inv = matrix_inverse_3x3(h, 1e-20)?;
        let tr_sqr = tr * tr;

        self.matches.reserve(query.num_features());

        for i in 0..query.num_features() {
            let mut first_best = u32::MAX;
            let mut second_best = u32::MAX;
            let mut best_index: i32 = -1;
            let f1 = descriptor_96(query, i);
            let p1 = query.point(i);

            // Map p1 to reference space via inverse homography.
            let (xp1, yp1) = multiply_point_homography_inhomogenous(&h_inv, p1.x, p1.y);

            for j in 0..reference.num_features() {
                let p2 = reference.point(j);
                if p1.maxima != p2.maxima {
                    continue;
                }
                let dx = xp1 - p2.x;
                let dy = yp1 - p2.y;
                if dx * dx + dy * dy > tr_sqr {
                    continue;
                }
                let f2 = descriptor_96(reference, j);
                let d = hamming_distance_96(f1, f2);
                if d < first_best {
                    second_best = first_best;
                    first_best = d;
                    best_index = j as i32;
                } else if d < second_best {
                    second_best = d;
                }
            }

            self.apply_ratio_test(i, best_index, first_best, second_best);
        }

        arlog_d!(
            "FeatureMatcher::match_guided: {} matches from {} queries",
            self.matches.len(),
            query.num_features()
        );
        Ok(self.matches.len())
    }

    fn apply_ratio_test(&mut self, query_idx: usize, best_idx: i32, first: u32, second: u32) {
        if first == u32::MAX {
            return;
        }
        debug_assert!(
            best_idx >= 0,
            "best_idx should be set when first_best is set"
        );
        let accept = if second == u32::MAX {
            true
        } else {
            (first as f32 / second as f32) < self.threshold
        };
        if accept {
            self.matches.push(Match {
                ins: query_idx,
                ref_: best_idx as usize,
            });
        }
    }

    fn ensure_feature_size(query: &FeatureStore, reference: &FeatureStore) -> Result<(), KpmError> {
        if query.bytes_per_feature() != FEATURE_SIZE
            || reference.bytes_per_feature() != FEATURE_SIZE
        {
            arlog_e!(
                "FeatureMatcher: only {}-byte features supported (query={}, ref={})",
                FEATURE_SIZE,
                query.bytes_per_feature(),
                reference.bytes_per_feature()
            );
            return Err(KpmError::InvalidInput(
                "FeatureMatcher only supports 96-byte features".into(),
            ));
        }
        Ok(())
    }
}

// ============================================================================
// Helpers
// ============================================================================

/// Returns the i-th descriptor as a fixed-size array reference.
///
/// Caller must ensure `store.bytes_per_feature() == 96`.
fn descriptor_96(store: &FeatureStore, i: usize) -> &[u8; 96] {
    <&[u8; 96]>::try_from(store.descriptor(i)).expect("bytes_per_feature == 96")
}

// `matrix_inverse_3x3` was promoted to `pub fn` in `homography.rs` in M9-1
// (issue #140) so `visual_database::check_homography_heuristics` can reuse it.

/// Applies a 3x3 row-major homography to a 2D point (inhomogeneous output).
///
/// C equivalent: `MultiplyPointHomographyInhomogenous`
fn multiply_point_homography_inhomogenous(h: &[f32; 9], x: f32, y: f32) -> (f32, f32) {
    let xp = h[0] * x + h[1] * y + h[2];
    let yp = h[3] * x + h[4] * y + h[5];
    let w = h[6] * x + h[7] * y + h[8];
    (xp / w, yp / w)
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn fp(x: f32, y: f32, maxima: bool) -> FeaturePoint {
        FeaturePoint {
            x,
            y,
            angle: 0.0,
            scale: 1.0,
            maxima,
        }
    }

    fn make_descriptor(seed: u8) -> [u8; 96] {
        let mut d = [0u8; 96];
        for (i, b) in d.iter_mut().enumerate() {
            *b = seed.wrapping_add(i as u8);
        }
        d
    }

    // ----- FeatureStore tests -----

    #[test]
    fn test_feature_store_new_invalid_bytes() {
        assert!(FeatureStore::new(0).is_err());
    }

    #[test]
    fn test_feature_store_add_and_retrieve() {
        let mut s = FeatureStore::new(96).unwrap();
        let d0 = make_descriptor(0);
        let d1 = make_descriptor(1);
        let d2 = make_descriptor(2);
        s.add(fp(0.0, 0.0, true), &d0).unwrap();
        s.add(fp(1.0, 1.0, true), &d1).unwrap();
        s.add(fp(2.0, 2.0, false), &d2).unwrap();

        assert_eq!(s.num_features(), 3);
        assert_eq!(s.descriptor(1), &d1[..]);
        assert!(!s.point(2).maxima);
    }

    #[test]
    fn test_feature_store_add_wrong_size() {
        let mut s = FeatureStore::new(96).unwrap();
        assert!(s.add(fp(0.0, 0.0, true), &[0u8; 32]).is_err());
    }

    #[test]
    fn test_feature_store_empty() {
        let s = FeatureStore::new(96).unwrap();
        assert_eq!(s.num_features(), 0);
        assert_eq!(s.bytes_per_feature(), 96);
    }

    // ----- FeatureMatcher constructor/config tests -----

    #[test]
    fn test_matcher_new_defaults() {
        let m = FeatureMatcher::new();
        assert_eq!(m.threshold(), 0.7);
        assert!(m.matches().is_empty());
    }

    #[test]
    fn test_matcher_set_threshold_roundtrip() {
        let mut m = FeatureMatcher::new();
        m.set_threshold(0.5);
        assert_eq!(m.threshold(), 0.5);
    }

    #[test]
    fn test_matcher_build_empty_store() {
        let mut m = FeatureMatcher::new();
        let s = FeatureStore::new(96).unwrap();
        assert!(m.build(&s).is_err());
    }

    #[test]
    fn test_matcher_build_wrong_byte_size() {
        let mut m = FeatureMatcher::new();
        let mut s = FeatureStore::new(64).unwrap();
        s.add(fp(0.0, 0.0, true), &[0u8; 64]).unwrap();
        assert!(m.build(&s).is_err());
    }

    #[test]
    fn test_match_indexed_before_build() {
        let mut m = FeatureMatcher::new();
        let mut q = FeatureStore::new(96).unwrap();
        let mut r = FeatureStore::new(96).unwrap();
        q.add(fp(0.0, 0.0, true), &make_descriptor(0)).unwrap();
        r.add(fp(0.0, 0.0, true), &make_descriptor(0)).unwrap();
        assert!(m.match_indexed(&q, &r).is_err());
    }

    // ----- Functional tests -----

    fn build_stores_with_identical_descriptors(n: usize) -> (FeatureStore, FeatureStore) {
        let mut q = FeatureStore::new(96).unwrap();
        let mut r = FeatureStore::new(96).unwrap();
        for i in 0..n {
            let d = make_descriptor(i as u8);
            q.add(fp(i as f32, i as f32, true), &d).unwrap();
            r.add(fp(i as f32, i as f32, true), &d).unwrap();
        }
        (q, r)
    }

    #[test]
    fn test_match_all_finds_identical_descriptors() {
        let (q, r) = build_stores_with_identical_descriptors(5);
        let mut m = FeatureMatcher::new();
        let n = m.match_all(&q, &r).unwrap();
        assert_eq!(n, 5);
        // Each query should match its index in reference (identical).
        for mat in m.matches() {
            assert_eq!(mat.ins, mat.ref_);
        }
    }

    #[test]
    fn test_match_all_rejects_distant_descriptors() {
        // All query descriptors are "distance halfway" from every reference,
        // so first_best == second_best → ratio = 1.0, never accepted.
        let mut q = FeatureStore::new(96).unwrap();
        let mut r = FeatureStore::new(96).unwrap();
        // Query: all zeros.
        for _ in 0..3 {
            q.add(fp(0.0, 0.0, true), &[0u8; 96]).unwrap();
        }
        // Reference: descriptors equally distant from query (alternate bit pattern).
        let half = [0xAAu8; 96];
        for _ in 0..3 {
            r.add(fp(0.0, 0.0, true), &half).unwrap();
        }
        let mut m = FeatureMatcher::new();
        let n = m.match_all(&q, &r).unwrap();
        assert_eq!(n, 0);
    }

    #[test]
    fn test_match_all_maxima_filtering() {
        let mut q = FeatureStore::new(96).unwrap();
        let mut r = FeatureStore::new(96).unwrap();
        let d0 = make_descriptor(0);
        // Query is maxima.
        q.add(fp(0.0, 0.0, true), &d0).unwrap();
        // Reference is minima — should be filtered out.
        r.add(fp(0.0, 0.0, false), &d0).unwrap();

        let mut m = FeatureMatcher::new();
        let n = m.match_all(&q, &r).unwrap();
        assert_eq!(n, 0);
    }

    #[test]
    fn test_match_indexed_finds_identical() {
        // Use enough features so BHC builds a real tree (>16 leaf threshold).
        let (q, r) = build_stores_with_identical_descriptors(20);
        let mut m = FeatureMatcher::new();
        m.build(&r).unwrap();
        let n = m.match_indexed(&q, &r).unwrap();
        // BHC may not retrieve every identical descriptor due to tree pruning,
        // but should find a substantial portion.
        assert!(n > 0, "match_indexed should find at least some matches");
    }

    #[test]
    fn test_match_guided_spatial_filter() {
        let mut q = FeatureStore::new(96).unwrap();
        let mut r = FeatureStore::new(96).unwrap();
        let d0 = make_descriptor(0);

        // Query at (10, 10).
        q.add(fp(10.0, 10.0, true), &d0).unwrap();
        // Reference at (10, 10) — within radius.
        r.add(fp(10.0, 10.0, true), &d0).unwrap();
        // Reference at (1000, 1000) — far away, should be filtered.
        r.add(fp(1000.0, 1000.0, true), &d0).unwrap();

        // Identity homography: warped query stays at (10, 10).
        let h_identity: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

        let mut m = FeatureMatcher::new();
        let n = m.match_guided(&q, &r, &h_identity, 5.0).unwrap();
        assert_eq!(n, 1, "should find only the close reference within tr=5");
        assert_eq!(m.matches()[0].ref_, 0);
    }

    #[test]
    fn test_match_guided_singular_homography() {
        let q = FeatureStore::new(96).unwrap();
        let r = FeatureStore::new(96).unwrap();
        let h_singular: [f32; 9] = [0.0; 9];
        let mut m = FeatureMatcher::new();
        // Empty stores short-circuit before homography inversion, so add features.
        let mut q = q;
        let mut r = r;
        q.add(fp(0.0, 0.0, true), &make_descriptor(0)).unwrap();
        r.add(fp(0.0, 0.0, true), &make_descriptor(0)).unwrap();
        assert!(m.match_guided(&q, &r, &h_singular, 5.0).is_err());
    }

    #[test]
    fn test_match_empty_stores() {
        let mut m = FeatureMatcher::new();
        let q = FeatureStore::new(96).unwrap();
        let r = FeatureStore::new(96).unwrap();
        assert_eq!(m.match_all(&q, &r).unwrap(), 0);
    }

    // ----- Helper tests -----
    //
    // Tests for matrix_inverse_3x3 live in `homography.rs` after the function
    // moved there in M9-1 (issue #140).

    #[test]
    fn test_homography_point_identity() {
        let id: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let (x, y) = multiply_point_homography_inhomogenous(&id, 5.0, 7.0);
        assert!((x - 5.0).abs() < 1e-6);
        assert!((y - 7.0).abs() < 1e-6);
    }
}

// ============================================================================
// Dual-mode validation: bridges to C++ BinaryFeatureMatcher (M7-3, #111)
// ============================================================================
//
// When the `dual-mode` feature is enabled (which transitively enables
// `ffi-backend`), the C++ matcher in WebARKitLib is linked in via the
// `webarkit_cpp_match_features_*` wrappers in `kpm_c_api.cpp`. The tests
// below run identical inputs through both the Rust and C++ matchers and
// assert that the match counts agree within a small tolerance (BHC RNG
// differences and minor floating-point order can cause small variation).

#[cfg(feature = "dual-mode")]
extern "C" {
    fn webarkit_cpp_match_features_brute(
        query_descs: *const u8,
        query_points: *const f32,
        query_maxima: *const i32,
        num_query: i32,
        ref_descs: *const u8,
        ref_points: *const f32,
        ref_maxima: *const i32,
        num_ref: i32,
        threshold: f32,
        ins_out: *mut i32,
        ref_out: *mut i32,
    ) -> i32;

    fn webarkit_cpp_match_features_indexed(
        query_descs: *const u8,
        query_points: *const f32,
        query_maxima: *const i32,
        num_query: i32,
        ref_descs: *const u8,
        ref_points: *const f32,
        ref_maxima: *const i32,
        num_ref: i32,
        threshold: f32,
        ins_out: *mut i32,
        ref_out: *mut i32,
    ) -> i32;

    fn webarkit_cpp_match_features_guided(
        query_descs: *const u8,
        query_points: *const f32,
        query_maxima: *const i32,
        num_query: i32,
        ref_descs: *const u8,
        ref_points: *const f32,
        ref_maxima: *const i32,
        num_ref: i32,
        h: *const f32,
        tr: f32,
        threshold: f32,
        ins_out: *mut i32,
        ref_out: *mut i32,
    ) -> i32;
}

#[cfg(all(test, feature = "dual-mode"))]
mod dual_mode_tests {
    use super::*;

    /// Maximum allowed |count_rust - count_cpp| for matcher dual-mode tests.
    /// Small variance is expected due to BHC RNG differences (documented in
    /// PR #114) and floating-point order in the ratio test.
    const MATCH_COUNT_TOLERANCE: i32 = 2;

    /// Asserts per-match invariants that hold INDEPENDENTLY of the C++ baseline.
    ///
    /// These catch bugs that count-only comparisons would miss:
    /// 1. Indices are in bounds.
    /// 2. The maxima flags of the matched query/reference points agree
    ///    (matches the explicit `p1.maxima != p2.maxima` filter in C++).
    fn assert_match_invariants(matches: &[Match], query: &FeatureStore, reference: &FeatureStore) {
        for (k, m) in matches.iter().enumerate() {
            assert!(
                m.ins < query.num_features(),
                "match[{}].ins={} out of bounds (num_query={})",
                k,
                m.ins,
                query.num_features()
            );
            assert!(
                m.ref_ < reference.num_features(),
                "match[{}].ref_={} out of bounds (num_ref={})",
                k,
                m.ref_,
                reference.num_features()
            );
            assert_eq!(
                query.point(m.ins).maxima,
                reference.point(m.ref_).maxima,
                "match[{}] ({},{}) violates maxima filter",
                k,
                m.ins,
                m.ref_
            );
        }
    }

    /// Asserts that for every match, the matched reference is the GLOBAL best
    /// (minimum Hamming distance) among references with the same maxima flag,
    /// AND that the ratio (best / second_best) < threshold.
    ///
    /// This is an absolute correctness check independent of the C++ baseline:
    /// it verifies that brute-force matching produces the optimal pair under
    /// the ratio test rules — independent of any other implementation.
    fn assert_brute_force_optimality(
        matches: &[Match],
        query: &FeatureStore,
        reference: &FeatureStore,
        threshold: f32,
    ) {
        for m in matches {
            let f1 = descriptor_96(query, m.ins);
            let p1 = query.point(m.ins);

            // Recompute first_best / second_best across all valid references.
            let mut first_best = u32::MAX;
            let mut second_best = u32::MAX;
            let mut best_idx: i32 = -1;
            for j in 0..reference.num_features() {
                if p1.maxima != reference.point(j).maxima {
                    continue;
                }
                let f2 = descriptor_96(reference, j);
                let d = hamming_distance_96(f1, f2);
                if d < first_best {
                    second_best = first_best;
                    first_best = d;
                    best_idx = j as i32;
                } else if d < second_best {
                    second_best = d;
                }
            }

            assert!(
                best_idx >= 0,
                "no valid reference for match query={}",
                m.ins
            );
            // The matched reference must be one of the global-best ties.
            // (Floating-point ratio order in C++ vs Rust can pick a different
            // index when distances tie; we accept any reference at the min.)
            let f_matched = descriptor_96(reference, m.ref_);
            let d_matched = hamming_distance_96(f1, f_matched);
            assert_eq!(
                d_matched, first_best,
                "match query={}: matched ref={} has dist={}, but global min is {}",
                m.ins, m.ref_, d_matched, first_best
            );

            // Verify ratio test was actually applied.
            if second_best != u32::MAX {
                let ratio = first_best as f32 / second_best as f32;
                assert!(
                    ratio < threshold,
                    "match query={}: ratio {} >= threshold {} (should have been rejected)",
                    m.ins,
                    ratio,
                    threshold
                );
            }
        }
    }

    /// Returns sorted match pairs as `(ins, ref)` tuples for stable comparison.
    fn sorted_pairs(matches: &[Match]) -> Vec<(usize, usize)> {
        let mut v: Vec<(usize, usize)> = matches.iter().map(|m| (m.ins, m.ref_)).collect();
        v.sort_unstable();
        v
    }

    /// Same, but for raw FFI output: takes parallel ins/ref arrays + count.
    fn sorted_pairs_ffi(ins: &[i32], refs: &[i32], count: usize) -> Vec<(usize, usize)> {
        let mut v: Vec<(usize, usize)> = (0..count)
            .map(|i| (ins[i] as usize, refs[i] as usize))
            .collect();
        v.sort_unstable();
        v
    }

    /// Pseudo-random 96-byte descriptors derived from a u64 seed (xorshift).
    fn gen_descriptor(seed: u64) -> [u8; 96] {
        let mut state = seed.wrapping_mul(0x9E3779B97F4A7C15);
        let mut d = [0u8; 96];
        for chunk in d.chunks_mut(8) {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let bytes = state.to_le_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }
        d
    }

    /// Build paired Rust + flat-array data for both stores.
    /// Returns (query_store, ref_store, query_flat, ref_flat).
    #[allow(clippy::type_complexity)]
    fn make_dual_inputs(
        num_query: usize,
        num_ref: usize,
    ) -> (
        FeatureStore,
        FeatureStore,
        (Vec<u8>, Vec<f32>, Vec<i32>),
        (Vec<u8>, Vec<f32>, Vec<i32>),
    ) {
        let mut q = FeatureStore::new(96).unwrap();
        let mut r = FeatureStore::new(96).unwrap();
        let mut q_descs = Vec::with_capacity(num_query * 96);
        let mut q_points = Vec::with_capacity(num_query * 4);
        let mut q_max = Vec::with_capacity(num_query);
        let mut r_descs = Vec::with_capacity(num_ref * 96);
        let mut r_points = Vec::with_capacity(num_ref * 4);
        let mut r_max = Vec::with_capacity(num_ref);

        // Half the query descriptors share a descriptor with a reference
        // (so we expect real matches), the other half are random noise.
        for i in 0..num_query {
            let d = if i < num_ref {
                gen_descriptor(i as u64) // shared seed → identical to ref[i]
            } else {
                gen_descriptor((i as u64).wrapping_add(0xDEAD_BEEF))
            };
            let x = (i as f32) * 1.5;
            let y = (i as f32) * 0.7;
            let maxima = i % 2 == 0;
            let pt = FeaturePoint {
                x,
                y,
                angle: 0.0,
                scale: 1.0,
                maxima,
            };
            q.add(pt, &d).unwrap();
            q_descs.extend_from_slice(&d);
            q_points.extend_from_slice(&[x, y, 0.0, 1.0]);
            q_max.push(if maxima { 1 } else { 0 });
        }
        for i in 0..num_ref {
            let d = gen_descriptor(i as u64);
            let x = (i as f32) * 2.0;
            let y = (i as f32) * 1.3;
            let maxima = i % 2 == 0;
            let pt = FeaturePoint {
                x,
                y,
                angle: 0.0,
                scale: 1.0,
                maxima,
            };
            r.add(pt, &d).unwrap();
            r_descs.extend_from_slice(&d);
            r_points.extend_from_slice(&[x, y, 0.0, 1.0]);
            r_max.push(if maxima { 1 } else { 0 });
        }
        (q, r, (q_descs, q_points, q_max), (r_descs, r_points, r_max))
    }

    #[test]
    fn dual_mode_match_all_within_tolerance() {
        let (q, r, q_flat, r_flat) = make_dual_inputs(50, 100);
        let mut matcher = FeatureMatcher::new();
        let count_rust = matcher.match_all(&q, &r).unwrap() as i32;
        let rust_matches = matcher.matches().to_vec();

        let mut ins_out = vec![0i32; 50];
        let mut ref_out = vec![0i32; 50];
        let count_cpp = unsafe {
            webarkit_cpp_match_features_brute(
                q_flat.0.as_ptr(),
                q_flat.1.as_ptr(),
                q_flat.2.as_ptr(),
                50,
                r_flat.0.as_ptr(),
                r_flat.1.as_ptr(),
                r_flat.2.as_ptr(),
                100,
                0.7,
                ins_out.as_mut_ptr(),
                ref_out.as_mut_ptr(),
            )
        };

        assert!(count_cpp >= 0, "C++ FFI returned error: {}", count_cpp);

        // Invariant B: independent correctness checks (no C++ comparison needed).
        assert_match_invariants(&rust_matches, &q, &r);
        assert_brute_force_optimality(&rust_matches, &q, &r, 0.7);

        // Invariant A: count and pair equality vs C++ baseline. Brute force is
        // deterministic — match pairs should be identical (modulo floating-point
        // ratio-order ties allowed within MATCH_COUNT_TOLERANCE).
        let diff = (count_rust - count_cpp).abs();
        assert!(
            diff <= MATCH_COUNT_TOLERANCE,
            "match_all dual-mode count mismatch: rust={}, cpp={}, diff={}",
            count_rust,
            count_cpp,
            diff
        );

        // Strongest check: sorted pair equality. If counts agree exactly,
        // pairs must agree exactly (no C++ vs Rust divergence in brute-force
        // because the algorithm is deterministic).
        if count_rust == count_cpp {
            let rust_pairs = sorted_pairs(&rust_matches);
            let cpp_pairs = sorted_pairs_ffi(&ins_out, &ref_out, count_cpp as usize);
            assert_eq!(
                rust_pairs, cpp_pairs,
                "match_all: same count but different pairs — Rust ports diverged from C++ semantics"
            );
        }
    }

    #[test]
    fn dual_mode_match_indexed_within_tolerance() {
        let (q, r, q_flat, r_flat) = make_dual_inputs(50, 100);
        let mut matcher = FeatureMatcher::new();
        matcher.build(&r).unwrap();
        let count_rust = matcher.match_indexed(&q, &r).unwrap() as i32;
        let rust_matches = matcher.matches().to_vec();

        let mut ins_out = vec![0i32; 50];
        let mut ref_out = vec![0i32; 50];
        let count_cpp = unsafe {
            webarkit_cpp_match_features_indexed(
                q_flat.0.as_ptr(),
                q_flat.1.as_ptr(),
                q_flat.2.as_ptr(),
                50,
                r_flat.0.as_ptr(),
                r_flat.1.as_ptr(),
                r_flat.2.as_ptr(),
                100,
                0.7,
                ins_out.as_mut_ptr(),
                ref_out.as_mut_ptr(),
            )
        };

        assert!(count_cpp >= 0, "C++ FFI returned error: {}", count_cpp);

        // Invariant B: pair-by-pair correctness independent of C++.
        // We can't verify global optimality here because BHC's tree pruning
        // means not every reference is considered for every query — but we
        // can verify bounds and the maxima filter.
        assert_match_invariants(&rust_matches, &q, &r);

        // Invariant A: count and pair equality vs C++.
        //
        // Now that #116 ports `ArrayShuffle` exactly (verified byte-identical
        // by the dual-mode tests in clustering.rs), the BHC tree topology and
        // traversal order match C++ exactly. The indexed matcher should now
        // produce the same match pairs as C++.
        let diff = (count_rust - count_cpp).abs();
        assert!(
            diff <= MATCH_COUNT_TOLERANCE,
            "match_indexed dual-mode count mismatch: rust={}, cpp={}, diff={}",
            count_rust,
            count_cpp,
            diff
        );

        if count_rust == count_cpp {
            let rust_pairs = sorted_pairs(&rust_matches);
            let cpp_pairs = sorted_pairs_ffi(&ins_out, &ref_out, count_cpp as usize);
            assert_eq!(
                rust_pairs, cpp_pairs,
                "match_indexed: same count but different pairs — BHC tree or matcher diverged from C++"
            );
        }
    }

    #[test]
    fn dual_mode_match_guided_within_tolerance() {
        let (q, r, q_flat, r_flat) = make_dual_inputs(50, 100);
        let mut matcher = FeatureMatcher::new();

        // Identity homography + large radius so spatial filter accepts most pairs.
        let h_identity: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let tr = 1000.0;

        let count_rust = matcher.match_guided(&q, &r, &h_identity, tr).unwrap() as i32;
        let rust_matches = matcher.matches().to_vec();

        let mut ins_out = vec![0i32; 50];
        let mut ref_out = vec![0i32; 50];
        let count_cpp = unsafe {
            webarkit_cpp_match_features_guided(
                q_flat.0.as_ptr(),
                q_flat.1.as_ptr(),
                q_flat.2.as_ptr(),
                50,
                r_flat.0.as_ptr(),
                r_flat.1.as_ptr(),
                r_flat.2.as_ptr(),
                100,
                h_identity.as_ptr(),
                tr,
                0.7,
                ins_out.as_mut_ptr(),
                ref_out.as_mut_ptr(),
            )
        };

        assert!(count_cpp >= 0, "C++ FFI returned error: {}", count_cpp);

        // Invariant B: independent correctness checks (maxima + bounds).
        assert_match_invariants(&rust_matches, &q, &r);

        // Invariant A: count agreement vs C++.
        let diff = (count_rust - count_cpp).abs();
        assert!(
            diff <= MATCH_COUNT_TOLERANCE,
            "match_guided dual-mode count mismatch: rust={}, cpp={}, diff={}",
            count_rust,
            count_cpp,
            diff
        );

        // Strongest check: sorted pair equality when counts match. The guided
        // matcher is deterministic (no RNG), so pairs should agree exactly.
        if count_rust == count_cpp {
            let rust_pairs = sorted_pairs(&rust_matches);
            let cpp_pairs = sorted_pairs_ffi(&ins_out, &ref_out, count_cpp as usize);
            assert_eq!(
                rust_pairs, cpp_pairs,
                "match_guided: same count but different pairs"
            );
        }
    }
}
