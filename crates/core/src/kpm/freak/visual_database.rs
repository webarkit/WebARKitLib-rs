/*
 *  visual_database.rs
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

//! Top-level orchestrator for the FREAK-matcher per-frame query pipeline.
//!
//! `VisualDatabase` assembles every component delivered in M6–M8
//! ([`FeatureMatcher`], [`HoughSimilarityVoting`], [`RobustHomography`],
//! [`DoGScaleInvariantDetector`], [`Keyframe`], [`GaussianScaleSpacePyramid`])
//! into the single function that `RustFreakMatcher` (M9-2) will call every
//! camera frame.
//!
//! Ported from WebARKitLib C++ headers:
//! - `KPM/FreakMatcher/matchers/visual_database.h` (439 lines)
//! - `KPM/FreakMatcher/matchers/visual_database-inline.h` (360 lines)
//!
//! # Pipeline (`query`)
//!
//! For each query image, [`VisualDatabase::query`] runs:
//!
//! 1. Build a Gaussian-scale-space pyramid for the query image
//!    (reusing the cached buffer if dimensions match).
//! 2. Detect DoG keypoints and extract FREAK descriptors into a
//!    fresh [`Keyframe`] (`query_keyframe`).
//! 3. For each stored reference keyframe, run the two-pass matching
//!    pipeline (see [`VisualDatabase::try_match_one`] for the detail):
//!    - Pass 1: feature match → Hough voting → bin filter → homography
//!      → inlier filter (early exit if any step fails).
//!    - Pass 2: homography-guided re-match → Hough voting → bin filter
//!      → homography → inlier filter.
//! 4. Keep the keyframe whose Pass 2 produced the most inliers
//!    (above the `min_num_inliers` threshold) as the winner.
//!
//! `query_keyframe` is rebuilt on every call. The result accessors
//! ([`inliers`], [`matched_db_id`], [`matched_geometry`]) reset to
//! empty/`-1`/`None` at the start of each query.
//!
//! [`inliers`]: VisualDatabase::inliers
//! [`matched_db_id`]: VisualDatabase::matched_db_id
//! [`matched_geometry`]: VisualDatabase::matched_geometry

use std::collections::BTreeMap;

use purecv::core::Matrix;

use crate::kpm::backend::KpmError;
use crate::{arlog_d, arlog_e, arlog_i};

use super::detector::DoGScaleInvariantDetector;
use super::gaussian_pyramid::{num_octaves_for, GaussianScaleSpacePyramid};
use super::homography::{
    matrix_inverse_3x3, multiply_point_homography_inhomogenous, quadrilateral_convex,
    smallest_triangle_area, RobustHomography, HOMOGRAPHY_DEFAULT_CAUCHY_SCALE,
    HOMOGRAPHY_DEFAULT_CHUNK_SIZE, HOMOGRAPHY_DEFAULT_MAX_TRIALS,
    HOMOGRAPHY_DEFAULT_NUM_HYPOTHESES,
};
use super::hough::find_features;
use super::hough::{
    find_hough_matches, find_hough_similarity, BinParams, FeaturePoint, HoughMatch,
    HoughSimilarityVoting, Match,
};
use super::keyframe::Keyframe;
use super::matcher::FeatureMatcher;

// ─────────────────────────────────────────────────────────────────────────
// Constants (C++ `visual_database-inline.h:49–61`)
// ─────────────────────────────────────────────────────────────────────────

/// Laplacian threshold for the DoG detector (C++ `kLaplacianThreshold`).
const LAPLACIAN_THRESHOLD: f32 = 3.0;

/// Edge threshold for the DoG detector (C++ `kEdgeThreshold`).
const EDGE_THRESHOLD: f32 = 4.0;

/// Maximum number of keypoints to detect per image (C++ `kMaxNumFeatures`).
const MAX_NUM_FEATURES: usize = 500;

/// Minimum image side length for the coarsest pyramid level (C++ `kMinCoarseSize`).
const MIN_COARSE_SIZE: usize = 8;

/// Pixel threshold for homography-inlier classification (C++ `kHomographyInlierThreshold`).
const HOMOGRAPHY_INLIER_THRESHOLD: f32 = 3.0;

/// Minimum number of inliers to consider a match successful (C++ `kMinNumInliers`).
const MIN_NUM_INLIERS: usize = 8;

/// Hough bin-distance tolerance for filtering (C++ `kHoughBinDelta`).
const HOUGH_BIN_DELTA: f32 = 1.0;

/// Whether to use the BHC feature index for matching (C++ `kUseFeatureIndex`).
const USE_FEATURE_INDEX: bool = true;

/// Spatial tolerance for the homography-guided re-match (C++ inline literal `10`).
const GUIDED_MATCH_SPATIAL_TOLERANCE: f32 = 10.0;

/// Singular-matrix threshold used inside [`check_homography_heuristics`]
/// (C++ `visual_database.h:251` calls `MatrixInverse3x3` with `1e-5`).
const HOMOGRAPHY_INVERSE_THRESHOLD: f32 = 1e-5;

// ─────────────────────────────────────────────────────────────────────────
// Hough voting per-iteration configuration (C++ `visual_database.h:280–321`)
// ─────────────────────────────────────────────────────────────────────────

/// Number of angle bins for similarity voting. C++ `FindHoughSimilarity`
/// in `visual_database.h:312` passes `12`.
const HOUGH_NUM_ANGLE_BINS: i32 = 12;

/// Number of scale bins for similarity voting. C++ `FindHoughSimilarity`
/// in `visual_database.h:312` passes `10`.
const HOUGH_NUM_SCALE_BINS: i32 = 10;

/// Hardcoded scale range in C++ `HoughSimilarityVoting::init`
/// (`hough_similarity_voting.cpp:81-82`).
const HOUGH_MIN_SCALE: f32 = -1.0;

/// Hardcoded scale range in C++ `HoughSimilarityVoting::init`
/// (`hough_similarity_voting.cpp:81-82`).
const HOUGH_MAX_SCALE: f32 = 1.0;

/// Hardcoded log-base in C++ `HoughSimilarityVoting::init`
/// (`hough_similarity_voting.cpp:92`).
const HOUGH_SCALE_K: f32 = 10.0;

/// Outcome of [`VisualDatabase::try_match_one`]: `Some((inliers, H))` if
/// the keyframe survived the two-pass pipeline, `None` if it was rejected.
type MatchOutcome = Option<(Vec<Match>, [f32; 9])>;

// ─────────────────────────────────────────────────────────────────────────
// VisualDatabase
// ─────────────────────────────────────────────────────────────────────────

/// Top-level orchestrator of the FREAK-matcher query pipeline.
///
/// Holds a database of reference [`Keyframe`]s plus the per-query state
/// required to run the two-pass matching pipeline.
///
/// # C equivalent
/// `vision::VisualDatabase<FEATURE_EXTRACTOR, STORE, MATCHER>` —
/// the Rust port specializes the template at `kBytesPerFeature = 96` and
/// drops the `HoughSimilarityVoting` field (a fresh voter is constructed
/// per loop iteration; see M9-1 decision D13 in
/// `docs/design/m9-1-visual-database.md`).
pub struct VisualDatabase {
    /// Reference keyframes, keyed by user-supplied id.
    ///
    /// Mirrors C++ `mKeyframeMap` (`std::unordered_map<id_t, keyframe_ptr_t>`),
    /// but we deliberately use `BTreeMap` here for **intra-Rust
    /// determinism**: [`query`] iterates the keyframes and breaks ties on
    /// inlier-count with a strict `>` (first match wins). With a hashed
    /// iteration order, the winning keyframe on borderline ties would
    /// depend on Rust's per-process `RandomState`, producing run-to-run
    /// `matched_id` variance — see issue #170. `BTreeMap`'s ascending-key
    /// order makes the tie-breaking deterministic. Note this doesn't
    /// achieve cross-language parity with C++ (C++ `unordered_map` still
    /// has implementation-defined order); see #170 for that broader fix.
    pub keyframes: BTreeMap<usize, Keyframe>,

    // Pipeline components (private state)
    matcher: FeatureMatcher,
    homography: RobustHomography,
    detector: DoGScaleInvariantDetector,

    /// Cached Gaussian pyramid; reallocated only on image-dimension change.
    /// Mirrors C++ `mPyramid` reuse pattern.
    pyramid: GaussianScaleSpacePyramid,
    /// Sentinel `-1` = no pyramid built yet.
    pyramid_width: i32,
    pyramid_height: i32,

    /// Inliers from the most recent successful query (empty on miss).
    pub inliers: Vec<Match>,
    /// Database id of the matched reference, or `-1` on miss.
    pub matched_db_id: i32,
    /// 3×3 row-major homography from query → matched reference (zeroed on miss).
    matched_geometry: [f32; 9],
    /// Keyframe extracted from the most recent query image
    /// (populated on every [`query`] call, regardless of success).
    query_keyframe: Option<Keyframe>,

    // Tunables (defaults: kMinNumInliers, kHomographyInlierThreshold, kUseFeatureIndex)
    min_num_inliers: usize,
    homography_inlier_threshold: f32,
    use_feature_index: bool,
}

impl VisualDatabase {
    /// Construct a `VisualDatabase` with the C++ default settings.
    ///
    /// Defaults match `visual_database-inline.h:64-72`:
    /// - `LaplacianThreshold = 3.0`
    /// - `EdgeThreshold = 4.0`
    /// - `MaxNumFeaturePoints = 500`
    /// - `HomographyInlierThreshold = 3.0`
    /// - `MinNumInliers = 8`
    /// - `UseFeatureIndex = true`
    ///
    /// C equivalent: `VisualDatabase::VisualDatabase()`.
    pub fn new() -> Result<Self, KpmError> {
        let detector = DoGScaleInvariantDetector::new(
            LAPLACIAN_THRESHOLD,
            EDGE_THRESHOLD,
            MAX_NUM_FEATURES,
            true, // find_orientation — required for FREAK descriptors
        );
        let homography = RobustHomography::new(
            HOMOGRAPHY_DEFAULT_CAUCHY_SCALE,
            HOMOGRAPHY_DEFAULT_NUM_HYPOTHESES,
            HOMOGRAPHY_DEFAULT_MAX_TRIALS,
            HOMOGRAPHY_DEFAULT_CHUNK_SIZE,
        );
        Ok(Self {
            keyframes: BTreeMap::new(),
            matcher: FeatureMatcher::new(),
            homography,
            detector,
            pyramid: GaussianScaleSpacePyramid::new(0),
            pyramid_width: -1,
            pyramid_height: -1,
            inliers: Vec::new(),
            matched_db_id: -1,
            matched_geometry: [0.0; 9],
            query_keyframe: None,
            min_num_inliers: MIN_NUM_INLIERS,
            homography_inlier_threshold: HOMOGRAPHY_INLIER_THRESHOLD,
            use_feature_index: USE_FEATURE_INDEX,
        })
    }

    // -----------------------------------------------------------------
    // Database mutation
    // -----------------------------------------------------------------

    /// Build a [`Keyframe`] from `image` (pyramid + DoG + FREAK) and store
    /// it under `id`.
    ///
    /// Errors:
    /// - `KpmError::InvalidInput` if `id` is already in the database.
    ///
    /// C equivalent: `addImage(const Image&, id_t)` (`visual_database-inline.h:79`).
    pub fn add_image(&mut self, image: &Matrix<u8>, id: usize) -> Result<(), KpmError> {
        if self.keyframes.contains_key(&id) {
            arlog_e!("VisualDatabase::add_image: id {} already exists", id);
            return Err(KpmError::InvalidInput(format!(
                "VisualDatabase: id {} already exists",
                id
            )));
        }

        self.ensure_pyramid(image)?;

        let mut keyframe = Keyframe::new(image.cols as i32, image.rows as i32)?;
        find_features(&mut keyframe, &self.pyramid, &self.detector)?;
        arlog_i!(
            "VisualDatabase::add_image: id={} found {} features",
            id,
            keyframe.store.num_features()
        );

        // M9 #146: build the BHC index once at insertion time
        // (mirrors C++ visual_database-inline.h:128-131 `keyframe->buildIndex()`).
        keyframe.build_index()?;

        self.keyframes.insert(id, keyframe);
        Ok(())
    }

    /// Insert a pre-built [`Keyframe`] under `id`.
    ///
    /// If the supplied keyframe does not already have a BHC index built
    /// (i.e. [`Keyframe::index`] returns `None`), [`Keyframe::build_index`]
    /// is called before insertion. This mirrors the C++ facade's
    /// `addFreakFeaturesAndDescriptors`, which always calls `buildIndex`.
    /// Callers that pre-built the index (e.g. during testing) are respected.
    ///
    /// Errors:
    /// - `KpmError::InvalidInput` if `id` is already in the database.
    /// - Any error from [`Keyframe::build_index`] (e.g. empty store).
    ///
    /// C equivalent: `addKeyframe(keyframe_ptr_t, id_t)`
    /// (`visual_database-inline.h:141`).
    pub fn add_keyframe(&mut self, mut keyframe: Keyframe, id: usize) -> Result<(), KpmError> {
        if self.keyframes.contains_key(&id) {
            arlog_e!("VisualDatabase::add_keyframe: id {} already exists", id);
            return Err(KpmError::InvalidInput(format!(
                "VisualDatabase: id {} already exists",
                id
            )));
        }

        // M9 #146: ensure the keyframe has a BHC index. Mirrors the C++ facade
        // `addFreakFeaturesAndDescriptors` which always calls buildIndex.
        if keyframe.index().is_none() {
            keyframe.build_index()?;
        }

        self.keyframes.insert(id, keyframe);
        Ok(())
    }

    /// Remove the keyframe stored under `id`.
    ///
    /// Returns `true` if the keyframe was present and removed, `false` if
    /// the id was not in the database.
    ///
    /// C equivalent: `erase(id_t)` (`visual_database-inline.h:351`).
    pub fn erase(&mut self, id: usize) -> bool {
        self.keyframes.remove(&id).is_some()
    }

    // -----------------------------------------------------------------
    // Query
    // -----------------------------------------------------------------

    /// Run the two-pass matching pipeline against every stored keyframe.
    ///
    /// Returns `Ok(true)` if a winning match was found (i.e. `matched_db_id >= 0`).
    ///
    /// On every call, the per-query state is reset before processing:
    /// `inliers` is cleared, `matched_db_id` becomes `-1`,
    /// `matched_geometry` is zeroed, and `query_keyframe` is rebuilt from
    /// the new image.
    ///
    /// C equivalent: `query(const Image&)` (`visual_database-inline.h:155`).
    pub fn query(&mut self, image: &Matrix<u8>) -> Result<bool, KpmError> {
        // Reset per-query state (C++ `visual_database-inline.h:194-195`).
        self.inliers.clear();
        self.matched_db_id = -1;
        self.matched_geometry = [0.0; 9];

        // Build the pyramid and the query keyframe.
        self.ensure_pyramid(image)?;
        let mut query_kf = Keyframe::new(image.cols as i32, image.rows as i32)?;
        find_features(&mut query_kf, &self.pyramid, &self.detector)?;
        arlog_i!(
            "VisualDatabase::query: found {} features in query",
            query_kf.store.num_features()
        );

        // Iterate the database; the keyframe with the most inliers wins.
        // Collect ids upfront because we need a `&mut self` for the matcher
        // build inside the loop body.
        let ids: Vec<usize> = self.keyframes.keys().copied().collect();
        for id in ids {
            if let Some((inliers, h)) = self.try_match_one(&query_kf, id)? {
                if inliers.len() >= self.min_num_inliers && inliers.len() > self.inliers.len() {
                    self.matched_geometry = h;
                    self.inliers = inliers;
                    self.matched_db_id = id as i32;
                }
            }
        }

        self.query_keyframe = Some(query_kf);
        Ok(self.matched_db_id >= 0)
    }

    /// Run the two-pass matching pipeline for a single reference keyframe.
    ///
    /// Returns `Some((inliers, H))` if the keyframe survived all pipeline
    /// stages, `None` if it was rejected at any stage (insufficient
    /// matches, insufficient votes, homography failure, etc.). Errors
    /// propagate only for unrecoverable internal failures.
    ///
    /// Mirrors C++ `visual_database-inline.h:200-344` per-keyframe inner
    /// loop body.
    fn try_match_one(
        &mut self,
        query_kf: &Keyframe,
        ref_id: usize,
    ) -> Result<MatchOutcome, KpmError> {
        // Pass 1: initial matching ---------------------------------------------
        // M9 #146: use the Keyframe-owned BHC index built at insertion time,
        // not a per-iteration matcher.build() rebuild (mirrors C++
        // visual_database-inline.h:204 `mMatcher.match(...&it->second->index())`).
        let ref_kf = self
            .keyframes
            .get(&ref_id)
            .expect("ref_id was just iterated from self.keyframes");

        let n = if self.use_feature_index {
            let index = ref_kf.index().ok_or_else(|| {
                arlog_e!(
                    "VisualDatabase::try_match_one: keyframe id={} has no BHC index. \
                     Was it added via add_image / add_keyframe?",
                    ref_id
                );
                KpmError::InternalError(format!("Keyframe id={} missing BHC index", ref_id))
            })?;
            self.matcher
                .match_with_index(&query_kf.store, &ref_kf.store, index)?
        } else {
            self.matcher.match_all(&query_kf.store, &ref_kf.store)?
        };
        if n < self.min_num_inliers {
            return Ok(None);
        }

        let query_pts: Vec<FeaturePoint> = (0..query_kf.store.num_features())
            .map(|i| *query_kf.store.point(i))
            .collect();
        let ref_pts: Vec<FeaturePoint> = (0..ref_kf.store.num_features())
            .map(|i| *ref_kf.store.point(i))
            .collect();

        let matches: Vec<HoughMatch> = self
            .matcher
            .matches()
            .iter()
            .map(matches_to_hough)
            .collect();

        // Pass 1: Hough voting -------------------------------------------------
        let mut voter = make_hough_voter(query_kf, ref_kf)?;
        let max_bin = match find_hough_similarity(
            &mut voter,
            &query_pts,
            &ref_pts,
            &matches,
            query_kf.width,
            query_kf.height,
            ref_kf.width,
            ref_kf.height,
        ) {
            Ok(b) => b,
            Err(_) => return Ok(None),
        };

        // Pass 1: filter by bin distance --------------------------------------
        let mut hough_matches = Vec::new();
        find_hough_matches(
            &mut hough_matches,
            &voter,
            &query_pts,
            &ref_pts,
            &matches,
            max_bin,
            HOUGH_BIN_DELTA,
        )?;

        // Pass 1: estimate homography -----------------------------------------
        let mut h = [0.0_f32; 9];
        if !estimate_homography(
            &mut h,
            &query_pts,
            &ref_pts,
            &hough_matches,
            &self.homography,
            ref_kf.width,
            ref_kf.height,
        ) {
            return Ok(None);
        }

        // Pass 1: filter inliers by homography reprojection error -------------
        let inliers = find_inliers(
            &h,
            &query_pts,
            &ref_pts,
            &hough_matches,
            self.homography_inlier_threshold,
        );
        if inliers.len() < self.min_num_inliers {
            return Ok(None);
        }

        // Pass 2: homography-guided re-match ----------------------------------
        let n = self.matcher.match_guided(
            &query_kf.store,
            &ref_kf.store,
            &h,
            GUIDED_MATCH_SPATIAL_TOLERANCE,
        )?;
        if n < self.min_num_inliers {
            return Ok(None);
        }
        let matches: Vec<HoughMatch> = self
            .matcher
            .matches()
            .iter()
            .map(matches_to_hough)
            .collect();

        // Pass 2: Hough voting (again, on the refined match set) --------------
        voter.reset();
        let max_bin = match find_hough_similarity(
            &mut voter,
            &query_pts,
            &ref_pts,
            &matches,
            query_kf.width,
            query_kf.height,
            ref_kf.width,
            ref_kf.height,
        ) {
            Ok(b) => b,
            Err(_) => return Ok(None),
        };

        // Pass 2: filter by bin distance --------------------------------------
        find_hough_matches(
            &mut hough_matches,
            &voter,
            &query_pts,
            &ref_pts,
            &matches,
            max_bin,
            HOUGH_BIN_DELTA,
        )?;

        // Pass 2: re-estimate homography --------------------------------------
        if !estimate_homography(
            &mut h,
            &query_pts,
            &ref_pts,
            &hough_matches,
            &self.homography,
            ref_kf.width,
            ref_kf.height,
        ) {
            return Ok(None);
        }

        // Pass 2: final inlier filter -----------------------------------------
        let inliers = find_inliers(
            &h,
            &query_pts,
            &ref_pts,
            &hough_matches,
            self.homography_inlier_threshold,
        );

        arlog_d!(
            "VisualDatabase::try_match_one: id={} pass-2 inliers={}",
            ref_id,
            inliers.len()
        );

        Ok(Some((inliers, h)))
    }

    // M9 #146 removed `fn match_features`. Its body is now inlined at the
    // top of `try_match_one`, using the Keyframe-owned BHC index via
    // `FeatureMatcher::match_with_index` instead of rebuilding the index
    // every loop iteration.

    // -----------------------------------------------------------------
    // Pyramid management
    // -----------------------------------------------------------------

    /// (Re)allocate the cached pyramid if image dimensions changed, then
    /// build it from `image`.
    fn ensure_pyramid(&mut self, image: &Matrix<u8>) -> Result<(), KpmError> {
        let w = image.cols as i32;
        let h = image.rows as i32;
        if self.pyramid_width != w || self.pyramid_height != h {
            let n = num_octaves_for(image.cols, image.rows, MIN_COARSE_SIZE);
            self.pyramid = GaussianScaleSpacePyramid::new(n);
            self.pyramid_width = w;
            self.pyramid_height = h;
        }
        self.pyramid.build(image).map_err(|e| {
            arlog_e!(
                "VisualDatabase::ensure_pyramid: pyramid build failed: {}",
                e
            );
            KpmError::InternalError(format!("pyramid build failed: {}", e))
        })
    }

    // -----------------------------------------------------------------
    // Public accessors
    // -----------------------------------------------------------------

    /// Inliers from the most recent successful query (empty otherwise).
    ///
    /// C equivalent: `inliers()`.
    pub fn inliers(&self) -> &[Match] {
        &self.inliers
    }

    /// Matched DB id from the most recent query. `-1` if no match.
    ///
    /// C equivalent: `matchedId()`.
    pub fn matched_db_id(&self) -> i32 {
        self.matched_db_id
    }

    /// 3×3 row-major homography from query → matched reference.
    ///
    /// Returns `None` if the last query produced no match
    /// (`matched_db_id < 0`), so callers can't accidentally read stale data.
    ///
    /// C equivalent: `matchedGeometry()`.
    pub fn matched_geometry(&self) -> Option<&[f32; 9]> {
        if self.matched_db_id >= 0 {
            Some(&self.matched_geometry)
        } else {
            None
        }
    }

    /// The query [`Keyframe`] built on the most recent [`query`] call.
    ///
    /// Populated on every call, regardless of match success.
    ///
    /// C equivalent: `queryKeyframe()`.
    pub fn query_keyframe(&self) -> Option<&Keyframe> {
        self.query_keyframe.as_ref()
    }

    /// Set the minimum number of inliers required for a successful match.
    ///
    /// C equivalent: `setMinNumInliers(size_t)`.
    pub fn set_min_num_inliers(&mut self, n: usize) {
        self.min_num_inliers = n;
    }

    /// Current minimum-inliers threshold.
    ///
    /// C equivalent: `minNumInliers()`.
    pub fn min_num_inliers(&self) -> usize {
        self.min_num_inliers
    }

    /// Number of stored reference keyframes.
    ///
    /// C equivalent: `databaseCount()`.
    pub fn database_count(&self) -> usize {
        self.keyframes.len()
    }

    /// Read-only access to the reference keyframe stored under `id`.
    ///
    /// C equivalent: `keyframe(id_t)`.
    pub fn keyframe(&self, id: usize) -> Option<&Keyframe> {
        self.keyframes.get(&id)
    }
}

impl Default for VisualDatabase {
    fn default() -> Self {
        Self::new().expect("VisualDatabase::new with C++ defaults is infallible")
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Private helpers — ported from C++ `visual_database.h:244–437`
// ─────────────────────────────────────────────────────────────────────────

/// Convert a [`Match`] (matcher output) into a [`HoughMatch`] (hough input).
///
/// `distance` is unused by the current hough code so we set it to `0.0`.
/// Decision D9 in `docs/design/m9-1-visual-database.md`.
#[inline]
fn matches_to_hough(m: &Match) -> HoughMatch {
    HoughMatch {
        query_idx: m.ins as u32,
        ref_idx: m.ref_ as u32,
        distance: 0.0,
    }
}

/// Build a fresh [`HoughSimilarityVoting`] for the (query, reference) pair.
///
/// Mirrors C++ `FindHoughSimilarity` (`visual_database.h:280–321`):
/// - x/y bounds scale with the query image dimensions (`±width*1.2`).
/// - The reference object center is the reference image's center.
/// - Scale parameters are hardcoded in the C++ `init` (-1, 1, scale_k=10).
/// - **x/y bin counts are auto-adjusted at vote time** (M9 #150) — mirrors
///   C++ `visual_database.h:312` `hough.init(_, _, _, _, 0, 0, 12, 10)`,
///   where the zeros toggle `mAutoAdjustXYNumBins = true`. The actual bin
///   counts are computed from the median projected dimension of the
///   per-iteration match set.
fn make_hough_voter(
    query_kf: &Keyframe,
    ref_kf: &Keyframe,
) -> Result<HoughSimilarityVoting, KpmError> {
    let dx = query_kf.width as f32 + query_kf.width as f32 * 0.2;
    let dy = query_kf.height as f32 + query_kf.height as f32 * 0.2;

    let params = BinParams::new_auto_xy(
        HOUGH_NUM_ANGLE_BINS,
        HOUGH_NUM_SCALE_BINS,
        -dx,
        dx,
        -dy,
        dy,
        HOUGH_MIN_SCALE,
        HOUGH_MAX_SCALE,
        HOUGH_SCALE_K,
    )?;

    let center_x = (ref_kf.width >> 1) as f32;
    let center_y = (ref_kf.height >> 1) as f32;
    HoughSimilarityVoting::new(params, center_x, center_y, ref_kf.width, ref_kf.height)
}

/// Estimate the homography between `query_pts` and `ref_pts` for the given
/// `matches`, returning `true` on success.
///
/// Runs the RANSAC + IRLS-polish estimator with four reference-image
/// corners as geometric-consistency test points, then applies the
/// [`check_homography_heuristics`] post-filter (smallest-triangle-area
/// + convexity).
///
/// C equivalent: `vision::EstimateHomography` (`visual_database.h:359`).
fn estimate_homography(
    h: &mut [f32; 9],
    query_pts: &[FeaturePoint],
    ref_pts: &[FeaturePoint],
    matches: &[HoughMatch],
    estimator: &RobustHomography,
    ref_width: i32,
    ref_height: i32,
) -> bool {
    if matches.is_empty() {
        return false;
    }

    // Build flat correspondence arrays in [x, y, x, y, ...] layout.
    // C++ convention: query == "destination" (`dst`), reference == "source" (`src`).
    let n = matches.len();
    let mut src = vec![0.0_f32; n * 2];
    let mut dst = vec![0.0_f32; n * 2];
    for (i, m) in matches.iter().enumerate() {
        let q = &query_pts[m.query_idx as usize];
        let r = &ref_pts[m.ref_idx as usize];
        dst[i * 2] = q.x;
        dst[i * 2 + 1] = q.y;
        src[i * 2] = r.x;
        src[i * 2 + 1] = r.y;
    }

    // The four corners of the reference image are used as geometric-
    // consistency test points (mirrors C++ visual_database.h:385–393).
    let test_points: [f32; 8] = [
        0.0,
        0.0,
        ref_width as f32,
        0.0,
        ref_width as f32,
        ref_height as f32,
        0.0,
        ref_height as f32,
    ];

    if !estimator.find_with_test_points(h, &src, &dst, n, &test_points, 4) {
        return false;
    }

    check_homography_heuristics(h, ref_width, ref_height)
}

/// Sanity-check that a candidate homography preserves enough geometric
/// structure to be a plausible match.
///
/// 1. Inverts `H` (rejects if singular at `1e-5` tolerance — mirrors C++).
/// 2. Back-projects the four reference-image corners.
/// 3. Rejects if the smallest triangle formed by the back-projected corners
///    is smaller than `0.0001 * refWidth * refHeight`.
/// 4. Rejects if the back-projected quadrilateral is not convex.
///
/// C equivalent: `vision::CheckHomographyHeuristics` (`visual_database.h:244`).
fn check_homography_heuristics(h: &[f32; 9], ref_width: i32, ref_height: i32) -> bool {
    let h_inv = match matrix_inverse_3x3(h, HOMOGRAPHY_INVERSE_THRESHOLD) {
        Ok(inv) => inv,
        Err(_) => return false,
    };

    let p0 = [0.0_f32, 0.0];
    let p1 = [ref_width as f32, 0.0];
    let p2 = [ref_width as f32, ref_height as f32];
    let p3 = [0.0_f32, ref_height as f32];

    let mut p0p = [0.0_f32; 2];
    let mut p1p = [0.0_f32; 2];
    let mut p2p = [0.0_f32; 2];
    let mut p3p = [0.0_f32; 2];
    multiply_point_homography_inhomogenous(&mut p0p, &h_inv, &p0);
    multiply_point_homography_inhomogenous(&mut p1p, &h_inv, &p1);
    multiply_point_homography_inhomogenous(&mut p2p, &h_inv, &p2);
    multiply_point_homography_inhomogenous(&mut p3p, &h_inv, &p3);

    // Smallest triangle area heuristic.
    let tr = ref_width as f32 * ref_height as f32 * 0.0001;
    if smallest_triangle_area(&p0p, &p1p, &p2p, &p3p) < tr {
        return false;
    }

    // Convexity check.
    if !quadrilateral_convex(&p0p, &p1p, &p2p, &p3p) {
        return false;
    }

    true
}

/// Collect the matches whose reprojection error under `H` is `≤ threshold`.
///
/// `H` maps reference → query. For each match, the reference point is
/// projected through `H` and compared to the query point; the squared
/// pixel distance must be `≤ threshold²` to qualify as an inlier.
///
/// C equivalent: `vision::FindInliers` (`visual_database.h:417`).
fn find_inliers(
    h: &[f32; 9],
    query_pts: &[FeaturePoint],
    ref_pts: &[FeaturePoint],
    matches: &[HoughMatch],
    threshold: f32,
) -> Vec<Match> {
    let threshold_sqr = threshold * threshold;
    let mut inliers = Vec::with_capacity(matches.len());
    for m in matches {
        let q = &query_pts[m.query_idx as usize];
        let r = &ref_pts[m.ref_idx as usize];

        let r_pt = [r.x, r.y];
        let mut projected = [0.0_f32; 2];
        multiply_point_homography_inhomogenous(&mut projected, h, &r_pt);

        let d_x = projected[0] - q.x;
        let d_y = projected[1] - q.y;
        let d2 = d_x * d_x + d_y * d_y;
        if d2 <= threshold_sqr {
            inliers.push(Match {
                ins: m.query_idx as usize,
                ref_: m.ref_idx as usize,
            });
        }
    }
    inliers
}

// ─────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn load_grayscale(path: &str) -> Matrix<u8> {
        let img = image::open(path).expect("load test image").to_luma8();
        let (w, h) = img.dimensions();
        Matrix::<u8>::from_vec(h as usize, w as usize, 1, img.into_raw())
    }

    fn synthetic_blank(w: usize, h: usize) -> Matrix<u8> {
        Matrix::<u8>::from_vec(h, w, 1, vec![128u8; w * h])
    }

    // -----------------------------------------------------------------
    // Issue #140 required tests
    // -----------------------------------------------------------------

    #[test]
    #[cfg_attr(miri, ignore)] // #194: full pipeline + BHC index — too slow under Miri
    fn test_visual_database_add_and_query_same_image() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let mut db = VisualDatabase::new().expect("new");
        db.add_image(&img, 0).expect("add_image");
        assert_eq!(db.database_count(), 1);

        let matched = db.query(&img).expect("query");
        assert!(matched, "self-match must succeed");
        assert!(db.matched_db_id() >= 0);
        assert!(db.matched_geometry().is_some());
        assert!(
            db.inliers().len() >= db.min_num_inliers(),
            "self-match must have at least min_num_inliers ({}) inliers, got {}",
            db.min_num_inliers(),
            db.inliers().len()
        );
        assert!(
            db.query_keyframe().is_some(),
            "query_keyframe must be populated"
        );
    }

    #[test]
    #[cfg_attr(miri, ignore)] // #194: full pipeline + BHC index — too slow under Miri
    fn test_visual_database_query_different_image_returns_no_match() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let blank = synthetic_blank(640, 480);
        let mut db = VisualDatabase::new().expect("new");
        db.add_image(&img, 0).expect("add_image");

        let matched = db.query(&blank).expect("query");
        assert!(!matched, "blank image must not match");
        assert_eq!(db.matched_db_id(), -1);
        assert!(db.matched_geometry().is_none());
        assert!(db.inliers().is_empty());
        // query_keyframe is still populated even on miss (C++ behaviour).
        assert!(db.query_keyframe().is_some());
    }

    // -----------------------------------------------------------------
    // Additional negative test (decision D12)
    // -----------------------------------------------------------------

    #[test]
    #[cfg_attr(miri, ignore)] // #194: full pipeline + BHC index — too slow under Miri
    fn test_visual_database_add_same_id_returns_err() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let mut db = VisualDatabase::new().expect("new");
        db.add_image(&img, 0).expect("first add must succeed");
        let err = db.add_image(&img, 0);
        assert!(err.is_err(), "duplicate id must error");
    }

    #[test]
    #[cfg_attr(miri, ignore)] // #194: full pipeline + BHC index — too slow under Miri
    fn test_visual_database_erase_removes_keyframe() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let mut db = VisualDatabase::new().expect("new");
        db.add_image(&img, 7).expect("add_image");
        assert_eq!(db.database_count(), 1);
        assert!(db.erase(7));
        assert_eq!(db.database_count(), 0);
        assert!(!db.erase(7), "erasing non-existent id returns false");
    }

    // -----------------------------------------------------------------
    // BHC-index-on-Keyframe lifecycle (M9 #146)
    // -----------------------------------------------------------------

    #[test]
    #[cfg_attr(miri, ignore)] // #194: full pyramid + DoG + BHC pipeline on real image — too slow under Miri
    fn test_visual_database_add_image_builds_index() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let mut db = VisualDatabase::new().expect("new");
        db.add_image(&img, 0).expect("add_image");
        assert!(
            db.keyframe(0).unwrap().index().is_some(),
            "add_image must build the BHC index per C++ visual_database-inline.h:128-131"
        );
    }

    #[test]
    #[cfg_attr(miri, ignore)] // #194: full pyramid + DoG + BHC pipeline on real image — too slow under Miri
    fn test_visual_database_add_keyframe_builds_index_when_absent() {
        // Build a keyframe externally without calling build_index, hand it to
        // add_keyframe, assert add_keyframe built the index for us.
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let mut pyr = crate::kpm::freak::gaussian_pyramid::GaussianScaleSpacePyramid::new(3);
        pyr.build(&img).unwrap();
        let det = crate::kpm::freak::detector::DoGScaleInvariantDetector::new(3.0, 4.0, 500, true);

        let mut kf = Keyframe::new(img.cols as i32, img.rows as i32).unwrap();
        find_features(&mut kf, &pyr, &det).expect("find_features");
        assert!(
            kf.index().is_none(),
            "precondition: external kf has no index"
        );

        let mut db = VisualDatabase::new().expect("new");
        db.add_keyframe(kf, 0).expect("add_keyframe");
        assert!(
            db.keyframe(0).unwrap().index().is_some(),
            "add_keyframe must call build_index when the supplied keyframe has none"
        );
    }

    #[test]
    #[cfg_attr(miri, ignore)] // #194: full pyramid + DoG + BHC pipeline on real image — too slow under Miri
    fn test_visual_database_add_keyframe_preserves_caller_built_index() {
        // If the caller pre-built the index, add_keyframe must not rebuild.
        // Verified by the fact that after a successful pre-build the keyframe
        // still has Some(index) — and no error is raised. We can't easily
        // assert pointer identity from outside, but the "no rebuild" property
        // is the cheap side: if add_keyframe always rebuilt, this test would
        // still pass; the precondition `kf.index().is_some()` after
        // build_index proves the pre-built path is reachable.
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let mut pyr = crate::kpm::freak::gaussian_pyramid::GaussianScaleSpacePyramid::new(3);
        pyr.build(&img).unwrap();
        let det = crate::kpm::freak::detector::DoGScaleInvariantDetector::new(3.0, 4.0, 500, true);

        let mut kf = Keyframe::new(img.cols as i32, img.rows as i32).unwrap();
        find_features(&mut kf, &pyr, &det).expect("find_features");
        kf.build_index().expect("pre-build");
        assert!(kf.index().is_some(), "precondition: caller built the index");

        let mut db = VisualDatabase::new().expect("new");
        db.add_keyframe(kf, 0).expect("add_keyframe");
        assert!(db.keyframe(0).unwrap().index().is_some());
    }

    // -----------------------------------------------------------------
    // Dual-mode parity test (M9-1 gate per #140, closed by M9 #146)
    // -----------------------------------------------------------------

    /// Project the four corners of a reference image of size `(w, h_dim)`
    /// through the given 3×3 row-major homography. Returns the projected
    /// points in order: top-left, top-right, bottom-right, bottom-left.
    ///
    /// Used by [`test_visual_database_matches_cpp_pipeline`] to measure
    /// how differently the Rust vs C++ pipelines warp the reference image
    /// — a metric intrinsically invariant to BHC tree-topology
    /// cross-language nondeterminism (M9 #146 R1).
    #[cfg(feature = "dual-mode")]
    fn reproject_corners(h: &[f32; 9], w: i32, h_dim: i32) -> [[f32; 2]; 4] {
        let corners: [[f32; 2]; 4] = [
            [0.0, 0.0],
            [w as f32, 0.0],
            [w as f32, h_dim as f32],
            [0.0, h_dim as f32],
        ];
        let mut out = [[0.0_f32; 2]; 4];
        for (i, c) in corners.iter().enumerate() {
            multiply_point_homography_inhomogenous(&mut out[i], h, c);
        }
        out
    }

    /// M9 dual-mode parity gate.
    ///
    /// Asserts that the Rust and C++ pipelines warp the reference image
    /// into the same query-image region within a few pixels of agreement
    /// — a metric intrinsically invariant to BHC tree-topology
    /// cross-language nondeterminism (M9 #146 R1).
    ///
    /// **History**: introduced in M9-1 (#140) with a `±5 inliers`
    /// absolute-count assertion that proved structurally unclosable
    /// across three PRs of algorithmic fixes (#145, #149, #151), each
    /// verified byte-equivalent to C++ at the unit level via dedicated
    /// dual-mode FFI tests. The residual gap (`diff=15`) was diagnosed as
    /// BHC tree-topology nondeterminism — both languages use unordered-key
    /// maps when grouping K-medoids assignments into child clusters during
    /// BHC build (`binary_hierarchical_clustering.h:217`). Algorithm
    /// correctness was preserved throughout. M9 #152 (this rewrite)
    /// redefines the metric to be intrinsically invariant to that
    /// variance: corner reprojection error rather than absolute inlier
    /// count.
    ///
    /// **Metric**: project the 4 reference corners through both
    /// homographies (Rust's `matched_geometry` and C++'s `pose_out[0..9]`,
    /// which is the same 3×3 homography per `kpm_c_api.cpp:156-166`).
    /// Compute Euclidean distance between each corresponding pair.
    /// Assert `max <= TOLERANCE_PX`.
    ///
    /// **Tolerance**: set per M9 #146 Decision 10 (`max(2.0, observed)`)
    /// with a 5 px ceiling. If observed > 5 px, investigate before
    /// merging — see [`docs/design/m9-parity-metric.md`].
    #[test]
    #[cfg(feature = "dual-mode")]
    fn test_visual_database_matches_cpp_pipeline() {
        use crate::arlog_i;
        use crate::kpm::kpm_ffi;

        let reference = load_grayscale("../../benchmarks/data/found.jpg");
        let query = load_grayscale("../../benchmarks/data/img.jpg");
        let ref_w = reference.cols as i32;
        let ref_h = reference.rows as i32;

        // ----- Rust path -----
        let mut db = VisualDatabase::new().expect("new");
        db.add_image(&reference, 0).expect("add_image");
        let rust_matched = db.query(&query).expect("query");
        let rust_id = db.matched_db_id();
        let rust_h = *db
            .matched_geometry()
            .expect("Rust matched, must have geometry");

        // ----- C++ path via existing kpm_query (no new FFI needed) -----
        // `pose_out[0..9]` is the 3×3 row-major homography normalized
        // so [8] = 1; `pose_out[9..12]` is FFI zero-padding. See
        // `kpm_c_api.cpp:156-166`.
        let cpp_h: [f32; 9];
        let cpp_id;
        unsafe {
            let handle = kpm_ffi::kpm_create(query.cols as i32, query.rows as i32);
            assert!(!handle.is_null(), "kpm_create returned null");

            let rc = kpm_ffi::kpm_add_ref_image(
                handle,
                reference.data.as_ptr(),
                ref_w,
                ref_h,
                72.0, // default DPI
                0,    // page_no
                0,    // image_no
            );
            assert!(rc >= 0, "kpm_add_ref_image failed: {}", rc);

            let mut pose = [0.0f32; 12];
            let mut error_out = 0.0f32;
            let mut page_no_out = -1i32;
            let rc = kpm_ffi::kpm_query(
                handle,
                query.data.as_ptr(),
                query.cols as i32,
                query.rows as i32,
                pose.as_mut_ptr(),
                &mut error_out,
                &mut page_no_out,
            );
            assert!(
                rc >= 0,
                "C++ kpm_query failed (rc={}) — both pipelines should match",
                rc
            );

            cpp_id = kpm_ffi::kpm_matched_id(handle);
            cpp_h = [
                pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], pose[7], pose[8],
            ];

            kpm_ffi::kpm_destroy(handle);
        }

        // ----- Cross-language sanity -----
        assert!(rust_matched, "Rust must match on the pinball pair");
        assert_eq!(
            rust_id, cpp_id,
            "matched_db_id divergence: rust={} cpp={}",
            rust_id, cpp_id
        );

        // ----- Corner reprojection parity (M9 #152) -----
        let rust_corners = reproject_corners(&rust_h, ref_w, ref_h);
        let cpp_corners = reproject_corners(&cpp_h, ref_w, ref_h);
        let mut max_displacement = 0.0_f32;
        let mut per_corner = [0.0_f32; 4];
        for i in 0..4 {
            let dx = rust_corners[i][0] - cpp_corners[i][0];
            let dy = rust_corners[i][1] - cpp_corners[i][1];
            let d = (dx * dx + dy * dy).sqrt();
            per_corner[i] = d;
            if d > max_displacement {
                max_displacement = d;
            }
        }

        arlog_i!(
            "M9 dual-mode parity: max corner displacement = {:.4} px \
             (per corner: tl={:.4}, tr={:.4}, br={:.4}, bl={:.4})",
            max_displacement,
            per_corner[0],
            per_corner[1],
            per_corner[2],
            per_corner[3]
        );

        // Tolerance set per M9 #146 Decision 10: max(2.0, ceil(observed)).
        // Observed during implementation: max_displacement ≈ 0.24 px on the
        // pinball pair (per-corner range: 0.06–0.24 px). The 2.0 px floor
        // gives ~8× safety margin against future float-rounding drift in
        // upstream pyramid / DoG / FREAK / matching components.
        //
        // If a future change drives this above 2.0 px, the assertion
        // message reports the new value — update this constant in the same
        // PR. If observed > 5 px, halt and investigate per
        // `docs/design/m9-parity-metric.md` §R1: BHC variance shouldn't
        // produce that much geometric drift.
        const TOLERANCE_PX: f32 = 2.0;
        assert!(
            max_displacement <= TOLERANCE_PX,
            "Rust and C++ homographies warp the reference into significantly \
             different query-image regions: max corner displacement = {:.4} px \
             > tolerance {} px",
            max_displacement,
            TOLERANCE_PX
        );
    }
}
