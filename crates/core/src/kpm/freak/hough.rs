/*
 *  hough.rs
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

//! Hough voting for similarity transformation matching.
//!
//! This module implements a discretized Hough voting scheme for finding consistent
//! similarity transformations (translation, rotation, scale) between matched feature pairs.
//! The 4D transformation space is binned, and matches vote for their corresponding bins.
//! The bin with the most votes determines the winning transformation.

use crate::kpm::backend::KpmError;
use crate::{arlog_d, arlog_e, arlog_i};
use std::collections::BTreeMap;

use super::math::{fast_median_f32, safe_division_f32};

/// Minimum number of votes required for a bin to be considered a valid result.
const MIN_VOTES_THRESHOLD: i32 = 3;

/// Encapsulates bin discretization parameters and provides bin calculation utilities.
#[derive(Clone)]
pub struct BinParams {
    /// Number of bins for x translation. Private since M9 #150 because
    /// auto-adjust mutates this and the dependent `a` / `b` strides must
    /// stay in sync — see [`Self::set_xy_bins`]. Read via [`Self::num_x_bins`].
    num_x_bins: i32,
    /// Number of bins for y translation. See [`num_x_bins`] for the
    /// visibility rationale.
    num_y_bins: i32,
    /// Number of bins for angle (rotation).
    pub num_angle_bins: i32,
    /// Number of bins for scale.
    pub num_scale_bins: i32,
    /// Minimum x translation value.
    pub min_x: f32,
    /// Maximum x translation value.
    pub max_x: f32,
    /// Minimum y translation value.
    pub min_y: f32,
    /// Maximum y translation value.
    pub max_y: f32,
    /// Minimum scale value.
    pub min_scale: f32,
    /// Maximum scale value.
    pub max_scale: f32,
    /// Log base for scale discretization.
    pub scale_k: f32,
    /// Precomputed 1.0 / ln(scale_k) for scale calculation.
    pub scale_one_over_log_k: f32,
    /// Precomputed stride: num_x_bins * num_y_bins
    a: i32,
    /// Precomputed stride: a * num_angle_bins
    b: i32,
    /// When true, [`HoughSimilarityVoting::recompute_xy_bins_from_matches`]
    /// recomputes `num_x_bins` / `num_y_bins` from the median projected
    /// dimension of input matches before each voting pass. Mirrors C++
    /// `mAutoAdjustXYNumBins` (set by `init` when both numXBins and
    /// numYBins are 0). Set by [`Self::new_auto_xy`], cleared by [`Self::new`].
    pub(crate) auto_adjust_xy: bool,
}

impl BinParams {
    /// Creates a new `BinParams` with the given discretization parameters.
    ///
    /// # Arguments
    /// * `num_x_bins` - Number of bins for x translation (must be > 0)
    /// * `num_y_bins` - Number of bins for y translation (must be > 0)
    /// * `num_angle_bins` - Number of bins for angle (must be > 0)
    /// * `num_scale_bins` - Number of bins for scale (must be > 0)
    /// * `min_x`, `max_x` - Translation x range (min_x < max_x)
    /// * `min_y`, `max_y` - Translation y range (min_y < max_y)
    /// * `min_scale`, `max_scale` - Scale range (min_scale < max_scale)
    /// * `scale_k` - Log base for scale (typically 2.0 or e)
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        num_x_bins: i32,
        num_y_bins: i32,
        num_angle_bins: i32,
        num_scale_bins: i32,
        min_x: f32,
        max_x: f32,
        min_y: f32,
        max_y: f32,
        min_scale: f32,
        max_scale: f32,
        scale_k: f32,
    ) -> Result<Self, KpmError> {
        // Validate bin counts
        if num_x_bins <= 0 {
            arlog_e!("BinParams::new: num_x_bins must be > 0, got {}", num_x_bins);
            return Err(KpmError::InvalidInput("num_x_bins must be > 0".into()));
        }
        if num_y_bins <= 0 {
            arlog_e!("BinParams::new: num_y_bins must be > 0, got {}", num_y_bins);
            return Err(KpmError::InvalidInput("num_y_bins must be > 0".into()));
        }
        if num_angle_bins <= 0 {
            arlog_e!(
                "BinParams::new: num_angle_bins must be > 0, got {}",
                num_angle_bins
            );
            return Err(KpmError::InvalidInput("num_angle_bins must be > 0".into()));
        }
        if num_scale_bins <= 0 {
            arlog_e!(
                "BinParams::new: num_scale_bins must be > 0, got {}",
                num_scale_bins
            );
            return Err(KpmError::InvalidInput("num_scale_bins must be > 0".into()));
        }

        // Validate ranges
        if min_x >= max_x {
            arlog_e!(
                "BinParams::new: invalid x range: min={}, max={}",
                min_x,
                max_x
            );
            return Err(KpmError::InvalidInput("min_x must be < max_x".into()));
        }
        if min_y >= max_y {
            arlog_e!(
                "BinParams::new: invalid y range: min={}, max={}",
                min_y,
                max_y
            );
            return Err(KpmError::InvalidInput("min_y must be < max_y".into()));
        }
        if min_scale >= max_scale {
            arlog_e!(
                "BinParams::new: invalid scale range: min={}, max={}",
                min_scale,
                max_scale
            );
            return Err(KpmError::InvalidInput(
                "min_scale must be < max_scale".into(),
            ));
        }

        // Validate scale_k
        if scale_k <= 0.0 || (scale_k - 1.0).abs() < 1e-6 {
            arlog_e!("BinParams::new: invalid scale_k: {}", scale_k);
            return Err(KpmError::InvalidInput(
                "scale_k must be > 0 and != 1.0".into(),
            ));
        }

        let scale_one_over_log_k = 1.0 / scale_k.ln();
        let a = num_x_bins * num_y_bins;
        let b = a * num_angle_bins;

        Ok(Self {
            num_x_bins,
            num_y_bins,
            num_angle_bins,
            num_scale_bins,
            min_x,
            max_x,
            min_y,
            max_y,
            min_scale,
            max_scale,
            scale_k,
            scale_one_over_log_k,
            a,
            b,
            auto_adjust_xy: false,
        })
    }

    /// Construct a `BinParams` with auto-adjusting x/y bins.
    ///
    /// `num_x_bins` and `num_y_bins` are initialised to 5 (the minimum
    /// clamp value used by [`HoughSimilarityVoting::recompute_xy_bins_from_matches`])
    /// so the BinParams is in a valid state even before any votes are cast.
    /// Before each voting pass, the voter recomputes both bin counts from
    /// the median projected dimension of the input matches.
    ///
    /// Mirrors C++ `HoughSimilarityVoting::init(min_x, max_x, min_y, max_y,
    /// 0, 0, num_angle_bins, num_scale_bins)` from
    /// `hough_similarity_voting.cpp:95-99`, where `numXBins == 0 &&
    /// numYBins == 0` toggles `mAutoAdjustXYNumBins = true`.
    #[allow(clippy::too_many_arguments)]
    pub fn new_auto_xy(
        num_angle_bins: i32,
        num_scale_bins: i32,
        min_x: f32,
        max_x: f32,
        min_y: f32,
        max_y: f32,
        min_scale: f32,
        max_scale: f32,
        scale_k: f32,
    ) -> Result<Self, KpmError> {
        // Initial bin count = clamp floor. recompute_xy_bins_from_matches will
        // replace these before the first vote.
        let mut params = Self::new(
            5,
            5,
            num_angle_bins,
            num_scale_bins,
            min_x,
            max_x,
            min_y,
            max_y,
            min_scale,
            max_scale,
            scale_k,
        )?;
        params.auto_adjust_xy = true;
        Ok(params)
    }

    /// Number of x bins (read-only accessor; mutated atomically with
    /// [`num_y_bins`] via [`set_xy_bins`]).
    #[inline]
    pub fn num_x_bins(&self) -> i32 {
        self.num_x_bins
    }

    /// Number of y bins (read-only accessor).
    #[inline]
    pub fn num_y_bins(&self) -> i32 {
        self.num_y_bins
    }

    /// Atomically update both x/y bin counts and recompute the dependent
    /// strides `a` and `b`. Crate-private — only called by
    /// [`HoughSimilarityVoting::recompute_xy_bins_from_matches`].
    pub(crate) fn set_xy_bins(&mut self, num_x_bins: i32, num_y_bins: i32) {
        self.num_x_bins = num_x_bins;
        self.num_y_bins = num_y_bins;
        self.a = num_x_bins * num_y_bins;
        self.b = self.a * self.num_angle_bins;
    }

    /// Maps a transformation vote to floating-point bin coordinates.
    ///
    /// C++ equivalent: `mapVoteToBin`
    fn map_to_bin(&self, x: f32, y: f32, angle: f32, scale: f32) -> (f32, f32, f32, f32) {
        let fb_x = self.num_x_bins as f32 * (x - self.min_x) / (self.max_x - self.min_x);
        let fb_y = self.num_y_bins as f32 * (y - self.min_y) / (self.max_y - self.min_y);

        // Angle is in (-π, π]; map to [0, num_angle_bins)
        const PI: f32 = std::f32::consts::PI;
        let fb_angle = self.num_angle_bins as f32 * ((angle + PI) / (2.0 * PI));

        let fb_scale = self.num_scale_bins as f32 * (scale - self.min_scale)
            / (self.max_scale - self.min_scale);

        (fb_x, fb_y, fb_angle, fb_scale)
    }

    /// Computes a linear index from 4D bin coordinates.
    ///
    /// C++ equivalent: `getBinIndex`
    fn bin_index(&self, bx: i32, by: i32, ba: i32, bs: i32) -> i32 {
        bx + (by * self.num_x_bins) + (ba * self.a) + (bs * self.b)
    }

    /// Decomposes a linear bin index back to 4D coordinates.
    ///
    /// C++ equivalent: `getBinsFromIndex`
    fn bins_from_index(&self, index: i32) -> (i32, i32, i32, i32) {
        let bx = ((index % self.b) % self.a) % self.num_x_bins;
        let by = (((index - bx) % self.b) % self.a) / self.num_x_bins;
        let ba = ((index - bx - (by * self.num_x_bins)) % self.b) / self.a;
        let bs = (index - bx - (by * self.num_x_bins) - (ba * self.a)) / self.b;
        (bx, by, ba, bs)
    }
}

/// Accumulates votes for similarity transformations in discretized 4D space.
pub struct HoughSimilarityVoting {
    params: BinParams,
    /// Object center in reference image.
    pub center_x: f32,
    pub center_y: f32,
    /// Reference image dimensions.
    pub ref_image_width: i32,
    pub ref_image_height: i32,
    /// Vote map: bin_index → vote count.
    ///
    /// `BTreeMap` (not `HashMap`) guarantees deterministic iteration order
    /// across runs. With `HashMap`, Rust's per-process `RandomState` would
    /// make `get_maximum_votes`' tie-breaking (`max_by_key` returns the
    /// last equal element in iteration order) non-deterministic across
    /// runs — a documented source of intra-Rust matcher variance, see
    /// issue #170.
    votes: BTreeMap<i32, i32>,
}

impl HoughSimilarityVoting {
    /// Creates a new `HoughSimilarityVoting` voter with the given parameters.
    pub fn new(
        params: BinParams,
        center_x: f32,
        center_y: f32,
        ref_image_width: i32,
        ref_image_height: i32,
    ) -> Result<Self, KpmError> {
        if ref_image_width <= 0 || ref_image_height <= 0 {
            arlog_e!(
                "HoughSimilarityVoting::new: invalid image dimensions: {}x{}",
                ref_image_width,
                ref_image_height
            );
            return Err(KpmError::InvalidInput(
                "image dimensions must be > 0".into(),
            ));
        }

        Ok(Self {
            params,
            center_x,
            center_y,
            ref_image_width,
            ref_image_height,
            votes: BTreeMap::new(),
        })
    }

    /// Clears the vote map for a fresh voting round.
    pub fn reset(&mut self) {
        self.votes.clear();
    }

    /// Casts a vote for the given similarity transformation.
    ///
    /// Returns `Ok(true)` if the vote was successfully cast.
    /// Returns `Ok(false)` if the vote parameters are out of bounds (expected at runtime).
    /// Returns `Err(...)` if configuration is invalid.
    pub fn vote(&mut self, x: f32, y: f32, angle: f32, scale: f32) -> Result<bool, KpmError> {
        const PI: f32 = std::f32::consts::PI;

        // Bounds checking
        if x < self.params.min_x
            || x >= self.params.max_x
            || y < self.params.min_y
            || y >= self.params.max_y
            || angle <= -PI
            || angle > PI
            || scale < self.params.min_scale
            || scale >= self.params.max_scale
        {
            arlog_d!(
                "vote out of bounds: x={}, y={}, angle={}, scale={}",
                x,
                y,
                angle,
                scale
            );
            return Ok(false);
        }

        // Map to floating-point bin coordinates
        let (fb_x, fb_y, fb_angle, fb_scale) = self.params.map_to_bin(x, y, angle, scale);

        // Floor to integer bin indices with offset -0.5 (as in C++)
        let bx = (fb_x - 0.5).floor() as i32;
        let by = (fb_y - 0.5).floor() as i32;
        let mut ba = (fb_angle - 0.5).floor() as i32;
        let bs = (fb_scale - 0.5).floor() as i32;

        // Wrap angle bin (circular)
        ba = (ba + self.params.num_angle_bins) % self.params.num_angle_bins;

        // Check bounds for 2×2×2×2 interpolation neighborhood
        if bx < 0
            || bx + 1 >= self.params.num_x_bins
            || by < 0
            || by + 1 >= self.params.num_y_bins
            || bs < 0
            || bs + 1 >= self.params.num_scale_bins
        {
            arlog_d!(
                "vote neighborhood out of bounds: bx={}, by={}, bs={}",
                bx,
                by,
                bs
            );
            return Ok(false);
        }

        // Cast 16 votes (bilinear interpolation across 4D)
        for dsx in 0..=1 {
            for dsy in 0..=1 {
                for dsa in 0..=1 {
                    for dss in 0..=1 {
                        let idx = self.params.bin_index(
                            bx + dsx,
                            by + dsy,
                            (ba + dsa) % self.params.num_angle_bins,
                            bs + dss,
                        );
                        *self.votes.entry(idx).or_insert(0) += 1;
                    }
                }
            }
        }

        Ok(true)
    }

    /// Finds the bin with the maximum number of votes.
    ///
    /// Returns `Some((bin_index, vote_count))` if there is at least one vote.
    /// Returns `None` if no votes have been cast.
    pub fn get_maximum_votes(&self) -> Option<(i32, i32)> {
        self.votes
            .iter()
            .max_by_key(|&(_, &count)| count)
            .map(|(&idx, &count)| (idx, count))
    }

    /// Recompute `num_x_bins` / `num_y_bins` from the median projected
    /// dimension of the input matches.
    ///
    /// For each match, the "projected dimension" is
    /// `safe_division(query_scale, ref_scale) * max(ref_image_width, ref_image_height)`.
    /// The bin size is `0.25 × median(projected_dim)`, and the bin count
    /// along each axis is `ceil((max - min) / bin_size)`, clamped to a
    /// minimum of 5.
    ///
    /// Mirrors C++ `HoughSimilarityVoting::autoAdjustXYNumBins`
    /// (`hough_similarity_voting.cpp:204-236`). Called from
    /// [`find_hough_similarity`] when [`BinParams::auto_adjust_xy`] is true.
    ///
    /// A no-op if `matches` is empty (preserves the current 5×5 default).
    pub(crate) fn recompute_xy_bins_from_matches(
        &mut self,
        query_points: &[FeaturePoint],
        ref_points: &[FeaturePoint],
        matches: &[HoughMatch],
    ) -> Result<(), KpmError> {
        if matches.is_empty() {
            arlog_d!("recompute_xy_bins_from_matches: no matches, keeping current bin count");
            return Ok(());
        }

        let max_dim = self.ref_image_width.max(self.ref_image_height) as f32;
        let mut projected_dim: Vec<f32> = matches
            .iter()
            .map(|m| {
                let q_scale = query_points[m.query_idx as usize].scale;
                let r_scale = ref_points[m.ref_idx as usize].scale;
                let scale = safe_division_f32(q_scale, r_scale);
                scale * max_dim
            })
            .collect();

        let median = fast_median_f32(&mut projected_dim);
        let bin_size = 0.25 * median;
        if bin_size <= 0.0 || !bin_size.is_finite() {
            arlog_d!(
                "recompute_xy_bins_from_matches: degenerate bin_size={}, keeping current bins",
                bin_size
            );
            return Ok(());
        }

        let raw_x = ((self.params.max_x - self.params.min_x) / bin_size).ceil() as i32;
        let raw_y = ((self.params.max_y - self.params.min_y) / bin_size).ceil() as i32;
        let new_x = raw_x.max(5);
        let new_y = raw_y.max(5);
        self.params.set_xy_bins(new_x, new_y);

        arlog_d!(
            "recompute_xy_bins_from_matches: median={}, bin_size={}, bins=({}, {})",
            median,
            bin_size,
            new_x,
            new_y
        );
        Ok(())
    }

    /// Converts a bin index to the corresponding similarity transformation.
    ///
    /// C++ equivalent: `getSimilarityFromIndex`
    pub fn get_similarity_from_index(&self, index: i32) -> Result<(f32, f32, f32, f32), KpmError> {
        let (bx, by, ba, bs) = self.params.bins_from_index(index);

        // Map bin centers back to transformation space
        let x = self.params.min_x
            + (bx as f32 + 0.5) * (self.params.max_x - self.params.min_x)
                / self.params.num_x_bins as f32;
        let y = self.params.min_y
            + (by as f32 + 0.5) * (self.params.max_y - self.params.min_y)
                / self.params.num_y_bins as f32;

        const PI: f32 = std::f32::consts::PI;
        let angle = (ba as f32 + 0.5) * (2.0 * PI) / self.params.num_angle_bins as f32 - PI;

        let scale = self.params.min_scale
            + (bs as f32 + 0.5) * (self.params.max_scale - self.params.min_scale)
                / self.params.num_scale_bins as f32;

        Ok((x, y, angle, scale))
    }
}

/// A feature point with position, orientation, scale, and extremum type.
/// C equivalent: vision::FeaturePoint
#[derive(Clone, Debug, Copy)]
pub struct FeaturePoint {
    pub x: f32,
    pub y: f32,
    pub angle: f32,
    pub scale: f32,
    /// True if this is a maxima, false if a minima (used to filter matches).
    pub maxima: bool,
}

impl Default for FeaturePoint {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            angle: 0.0,
            scale: 0.0,
            maxima: true,
        }
    }
}

/// A correspondence between a query feature and a reference feature.
/// C equivalent: vision::match_t
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Match {
    /// Index into the query feature-point list.
    pub ins: usize,
    /// Index into the reference feature-point list.
    pub ref_: usize,
}

/// A scored match used internally by Hough voting (carries distance for ranking).
#[derive(Clone, Copy, Debug)]
pub struct HoughMatch {
    pub query_idx: u32,
    pub ref_idx: u32,
    pub distance: f32,
}

/// Detect keypoints in the Gaussian pyramid and extract FREAK descriptors
/// into `keyframe.store`.
///
/// Caller responsibilities:
/// - `pyramid` is already built (via `pyramid.build(image)?`).
/// - `detector` is configured with `find_orientation = true`. Otherwise
///   keypoints have `angle = 0.0` and the resulting FREAK descriptors are
///   rotation-variant, defeating the FREAK design.
///
/// C equivalent: `vision::FindFeatures` (`visual_database.h` lines 207–239).
pub fn find_features(
    keyframe: &mut super::keyframe::Keyframe,
    pyramid: &super::gaussian_pyramid::GaussianScaleSpacePyramid,
    detector: &super::detector::DoGScaleInvariantDetector,
) -> Result<(), KpmError> {
    // 1. Detect keypoints (M8-3).
    let dog_points = detector.detect(pyramid);

    // 2. Project rich DoG keypoints to persistent FeaturePoint (M8-3 From impl).
    let points: Vec<FeaturePoint> = dog_points.iter().map(FeaturePoint::from).collect();

    // 3. Extract FREAK descriptors as a flat Vec<u8>.
    let mut buf =
        Vec::<u8>::with_capacity(points.len() * super::descriptor::FREAK_DESCRIPTOR_BYTES);
    super::descriptor::extract_freak_descriptors(pyramid, &points, &mut buf);

    // 4. Populate the keyframe's FeatureStore.
    for (i, point) in points.iter().enumerate() {
        let start = i * super::descriptor::FREAK_DESCRIPTOR_BYTES;
        let desc = &buf[start..start + super::descriptor::FREAK_DESCRIPTOR_BYTES];
        keyframe.store.add(*point, desc)?;
    }

    Ok(())
}

/// Finds the Hough bin with the most consistent similarity transformation votes.
///
/// # Arguments
/// * `voting` - Voter to accumulate votes into
/// * `query_points` - Feature points from query image
/// * `ref_points` - Feature points from reference image
/// * `matches` - Correspondences between query and reference features
/// * `query_width`, `query_height` - Query image dimensions
/// * `ref_width`, `ref_height` - Reference image dimensions
///
/// # Returns
/// The bin index with the most votes (>= MIN_VOTES_THRESHOLD).
/// Returns `Err(InvalidInput("insufficient votes for feature matching".into()))` if no bin reaches the threshold.
#[allow(clippy::too_many_arguments)]
pub fn find_hough_similarity(
    voting: &mut HoughSimilarityVoting,
    query_points: &[FeaturePoint],
    ref_points: &[FeaturePoint],
    matches: &[HoughMatch],
    _query_width: i32,
    _query_height: i32,
    _ref_width: i32,
    _ref_height: i32,
) -> Result<i32, KpmError> {
    if matches.is_empty() {
        arlog_d!("find_hough_similarity: no matches provided");
        return Err(KpmError::InvalidInput(
            "insufficient votes for feature matching".into(),
        ));
    }

    // M9 #150: auto-adjust x/y bin counts from the median projected
    // dimension of the input matches. Mirrors C++ vote() invoking
    // autoAdjustXYNumBins when mAutoAdjustXYNumBins is true.
    if voting.params.auto_adjust_xy {
        voting.recompute_xy_bins_from_matches(query_points, ref_points, matches)?;
    }

    voting.reset();

    let mut vote_count = 0i32;
    for m in matches {
        let q_pt = &query_points[m.query_idx as usize];
        let r_pt = &ref_points[m.ref_idx as usize];

        // Compute similarity transformation from correspondence
        let (x, y, angle, scale) =
            compute_similarity(q_pt, r_pt, voting.center_x, voting.center_y)?;

        if voting.vote(x, y, angle, scale)? {
            vote_count += 1;
        }
    }

    if let Some((max_index, max_votes)) = voting.get_maximum_votes() {
        if max_votes >= MIN_VOTES_THRESHOLD {
            arlog_i!(
                "find_hough_similarity: found winner at bin {} with {} votes",
                max_index,
                max_votes
            );
            return Ok(max_index);
        }
    }

    arlog_d!(
        "find_hough_similarity: insufficient votes after {} attempted votes",
        vote_count
    );
    Err(KpmError::InvalidInput(
        "insufficient votes for feature matching".into(),
    ))
}

/// Filters matches to those consistent with the winning Hough bin.
///
/// For each input match, recomputes the similarity transformation from its
/// query/reference feature points (mirroring [`compute_similarity`], which is
/// also what [`HoughSimilarityVoting::vote`] does internally), maps it to
/// floating-point bin coordinates, and retains the match if its bin-space
/// distance to the winning bin's center is `< bin_delta` in all four
/// dimensions (with circular wrap-around on the angle axis).
///
/// # Arguments
/// * `out_matches` — Output vector for matches that fall within the winning bin.
/// * `voting` — Voter containing the winning bin index.
/// * `query_points` — Feature points from the query image.
/// * `ref_points` — Feature points from the reference image.
/// * `in_matches` — All matches to filter (one [`HoughMatch`] per pair).
/// * `max_hough_index` — The winning bin's linear index (from
///   [`find_hough_similarity`]).
/// * `bin_delta` — Maximum bin-space distance to be considered an inlier
///   (typically `kHoughBinDelta = 1.0` in C++).
///
/// # C++ equivalent
/// `vision::FindHoughMatches` from `visual_database.h:327`. The Rust port
/// recomputes the float bin position per match rather than caching
/// `mSubBinLocations` / `mSubBinLocationIndices` during [`vote`]
/// (M9-1 decision D15: recomputation cost is trivial; keeps state simpler).
pub fn find_hough_matches(
    out_matches: &mut Vec<HoughMatch>,
    voting: &HoughSimilarityVoting,
    query_points: &[FeaturePoint],
    ref_points: &[FeaturePoint],
    in_matches: &[HoughMatch],
    max_hough_index: i32,
    bin_delta: f32,
) -> Result<(), KpmError> {
    out_matches.clear();
    out_matches.reserve(in_matches.len());

    let (max_bx, max_by, max_ba, max_bs) = voting.params.bins_from_index(max_hough_index);
    let ref_bin_x = max_bx as f32 + 0.5;
    let ref_bin_y = max_by as f32 + 0.5;
    let ref_bin_a = max_ba as f32 + 0.5;
    let ref_bin_s = max_bs as f32 + 0.5;
    let num_angle_bins = voting.params.num_angle_bins as f32;

    const PI: f32 = std::f32::consts::PI;

    for m in in_matches {
        let q_pt = &query_points[m.query_idx as usize];
        let r_pt = &ref_points[m.ref_idx as usize];

        let (x, y, angle, scale) =
            compute_similarity(q_pt, r_pt, voting.center_x, voting.center_y)?;

        // Skip transformations that fall outside the voting volume — they
        // contributed no vote, so they cannot be near the winning bin.
        if x < voting.params.min_x
            || x >= voting.params.max_x
            || y < voting.params.min_y
            || y >= voting.params.max_y
            || angle <= -PI
            || angle > PI
            || scale < voting.params.min_scale
            || scale >= voting.params.max_scale
        {
            continue;
        }

        let (fb_x, fb_y, fb_angle, fb_scale) = voting.params.map_to_bin(x, y, angle, scale);

        let d_x = (fb_x - ref_bin_x).abs();
        let d_y = (fb_y - ref_bin_y).abs();
        let d_s = (fb_scale - ref_bin_s).abs();

        // Angle is circular: shortest distance wraps at num_angle_bins.
        let d_a_raw = (fb_angle - ref_bin_a).abs();
        let d_a = d_a_raw.min(num_angle_bins - d_a_raw);

        if d_x < bin_delta && d_y < bin_delta && d_a < bin_delta && d_s < bin_delta {
            out_matches.push(*m);
        }
    }

    arlog_d!(
        "find_hough_matches: retained {} of {} matches within bin_delta={}",
        out_matches.len(),
        in_matches.len(),
        bin_delta
    );
    Ok(())
}

/// Computes a similarity transformation from two feature correspondences.
fn compute_similarity(
    query_pt: &FeaturePoint,
    ref_pt: &FeaturePoint,
    center_x: f32,
    center_y: f32,
) -> Result<(f32, f32, f32, f32), KpmError> {
    // Angle difference
    let mut angle = query_pt.angle - ref_pt.angle;
    const PI: f32 = std::f32::consts::PI;
    if angle <= -PI {
        angle += 2.0 * PI;
    } else if angle > PI {
        angle -= 2.0 * PI;
    }

    // Scale difference
    let scale = if ref_pt.scale.abs() < 1e-6 {
        1.0
    } else {
        query_pt.scale / ref_pt.scale
    };

    // Rotation and scale matrix application
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    let s_cos = scale * cos_a;
    let s_sin = scale * sin_a;

    // Transform reference center by similarity
    let tp_x = s_cos * ref_pt.x - s_sin * ref_pt.y;
    let tp_y = s_sin * ref_pt.x + s_cos * ref_pt.y;

    // Translation
    let tx = query_pt.x - tp_x;
    let ty = query_pt.y - tp_y;

    // Transform object center by similarity
    let center_tp_x = s_cos * center_x - s_sin * center_y + tx;
    let center_tp_y = s_sin * center_x + s_cos * center_y + ty;

    // Log scale for discretization
    let log_scale = scale.ln() / 2.0_f32.ln();

    Ok((center_tp_x, center_tp_y, angle, log_scale))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bin_params_creation_valid() {
        let params = BinParams::new(10, 10, 8, 5, 0.0, 100.0, 0.0, 100.0, 0.5, 2.0, 2.0)
            .expect("valid params");
        assert_eq!(params.a, 100);
        assert_eq!(params.b, 800);
        assert!(!params.auto_adjust_xy, "new() must keep auto-adjust off");
    }

    // ------------------------------------------------------------------
    // M9 #150: auto-adjust factory + atomic stride mutator
    // ------------------------------------------------------------------

    #[test]
    fn test_bin_params_new_auto_xy_initial_state() {
        let p = BinParams::new_auto_xy(12, 10, -100.0, 100.0, -100.0, 100.0, -1.0, 1.0, 10.0)
            .expect("valid auto-xy params");
        assert_eq!(p.num_x_bins(), 5, "initial x bins = clamp floor");
        assert_eq!(p.num_y_bins(), 5, "initial y bins = clamp floor");
        assert!(p.auto_adjust_xy, "new_auto_xy must set the flag");
        // Strides reflect the initial 5×5 grid.
        assert_eq!(p.a, 25);
        assert_eq!(p.b, 25 * 12);
    }

    #[test]
    fn test_bin_params_set_xy_bins_updates_strides_atomically() {
        let mut p =
            BinParams::new_auto_xy(12, 10, -100.0, 100.0, -100.0, 100.0, -1.0, 1.0, 10.0).unwrap();
        p.set_xy_bins(13, 17);
        assert_eq!(p.num_x_bins(), 13);
        assert_eq!(p.num_y_bins(), 17);
        assert_eq!(p.a, 13 * 17, "a stride must update");
        assert_eq!(p.b, 13 * 17 * 12, "b stride must update");
    }

    #[test]
    fn test_recompute_xy_bins_from_matches_known_input() {
        // Construct a voter with simple parameters:
        //   ref_image_width = ref_image_height = 100 → max_dim = 100
        //   min_x..max_x = -100..100 (span = 200)
        //   min_y..max_y = -100..100 (span = 200)
        // All matches have ins_scale == ref_scale == 1.0, so
        //   safe_division = 1.0, projected_dim = 1.0 * 100 = 100 for each.
        // C++ FastMedian on [100, 100, 100, 100, 100] (n=5) returns the
        //   2nd smallest = 100.
        // bin_size = 0.25 * 100 = 25
        // num_x_bins = ceil(200 / 25) = 8, num_y_bins = 8.
        let params =
            BinParams::new_auto_xy(12, 10, -100.0, 100.0, -100.0, 100.0, -1.0, 1.0, 10.0).unwrap();
        let mut voter = HoughSimilarityVoting::new(params, 50.0, 50.0, 100, 100).unwrap();

        let pt = FeaturePoint {
            x: 0.0,
            y: 0.0,
            angle: 0.0,
            scale: 1.0,
            maxima: true,
        };
        let query = vec![pt; 5];
        let refs = vec![pt; 5];
        let matches: Vec<HoughMatch> = (0..5)
            .map(|i| HoughMatch {
                query_idx: i,
                ref_idx: i,
                distance: 0.0,
            })
            .collect();

        voter
            .recompute_xy_bins_from_matches(&query, &refs, &matches)
            .unwrap();
        assert_eq!(voter.params.num_x_bins(), 8);
        assert_eq!(voter.params.num_y_bins(), 8);
    }

    #[test]
    fn test_recompute_xy_bins_from_matches_clamps_at_5() {
        // Degenerate case: huge projected dimensions → ceil((max-min)/bin_size)
        // becomes < 5. Verify clamp to 5.
        // Use a tiny range relative to max_dim:
        //   ref_image dims = 10000 → max_dim = 10000
        //   range = 100 (small)
        //   all scales = 1.0 → projected_dim = 10000
        //   bin_size = 2500
        //   raw_x = ceil(100 / 2500) = 1 → clamps to 5.
        let params =
            BinParams::new_auto_xy(12, 10, -50.0, 50.0, -50.0, 50.0, -1.0, 1.0, 10.0).unwrap();
        let mut voter = HoughSimilarityVoting::new(params, 0.0, 0.0, 10000, 10000).unwrap();

        let pt = FeaturePoint {
            x: 0.0,
            y: 0.0,
            angle: 0.0,
            scale: 1.0,
            maxima: true,
        };
        let query = vec![pt; 3];
        let refs = vec![pt; 3];
        let matches: Vec<HoughMatch> = (0..3)
            .map(|i| HoughMatch {
                query_idx: i,
                ref_idx: i,
                distance: 0.0,
            })
            .collect();

        voter
            .recompute_xy_bins_from_matches(&query, &refs, &matches)
            .unwrap();
        assert_eq!(voter.params.num_x_bins(), 5, "must clamp to floor of 5");
        assert_eq!(voter.params.num_y_bins(), 5, "must clamp to floor of 5");
    }

    #[test]
    fn test_recompute_xy_bins_from_matches_empty_is_noop() {
        let params =
            BinParams::new_auto_xy(12, 10, -100.0, 100.0, -100.0, 100.0, -1.0, 1.0, 10.0).unwrap();
        let mut voter = HoughSimilarityVoting::new(params, 50.0, 50.0, 100, 100).unwrap();

        let initial_x = voter.params.num_x_bins();
        let initial_y = voter.params.num_y_bins();

        voter.recompute_xy_bins_from_matches(&[], &[], &[]).unwrap();

        assert_eq!(
            voter.params.num_x_bins(),
            initial_x,
            "empty must not mutate"
        );
        assert_eq!(
            voter.params.num_y_bins(),
            initial_y,
            "empty must not mutate"
        );
    }

    #[test]
    fn test_bin_params_invalid_x_bins() {
        let result = BinParams::new(0, 10, 8, 5, 0.0, 100.0, 0.0, 100.0, 0.5, 2.0, 2.0);
        assert!(result.is_err());
    }

    #[test]
    fn test_bin_params_invalid_range() {
        let result = BinParams::new(10, 10, 8, 5, 100.0, 0.0, 0.0, 100.0, 0.5, 2.0, 2.0);
        assert!(result.is_err());
    }

    #[test]
    fn test_bin_params_invalid_scale_k() {
        let result = BinParams::new(10, 10, 8, 5, 0.0, 100.0, 0.0, 100.0, 0.5, 2.0, 1.0);
        assert!(result.is_err());
    }

    #[test]
    fn test_bin_index_and_inverse() {
        let params =
            BinParams::new(5, 5, 4, 3, 0.0, 100.0, 0.0, 100.0, 0.5, 2.0, 2.0).expect("valid");
        let idx = params.bin_index(2, 1, 1, 1);
        let (bx, by, ba, bs) = params.bins_from_index(idx);
        assert_eq!((bx, by, ba, bs), (2, 1, 1, 1));
    }

    #[test]
    fn test_hough_voting_creation() {
        let params =
            BinParams::new(10, 10, 8, 5, 0.0, 100.0, 0.0, 100.0, 0.5, 2.0, 2.0).expect("valid");
        let voting = HoughSimilarityVoting::new(params, 50.0, 50.0, 200, 200).expect("valid");
        assert!(voting.get_maximum_votes().is_none());
    }

    #[test]
    fn test_hough_voting_single_vote() {
        let params =
            BinParams::new(10, 10, 8, 5, 0.0, 100.0, 0.0, 100.0, 0.5, 2.0, 2.0).expect("valid");
        let mut voting = HoughSimilarityVoting::new(params, 50.0, 50.0, 200, 200).expect("valid");

        let result = voting.vote(50.0, 50.0, 0.0, 1.0);
        assert!(result.is_ok());
        assert!(result.unwrap());

        let (_, max_votes) = voting.get_maximum_votes().expect("has votes");
        assert!(max_votes > 0);
    }

    #[test]
    fn test_hough_voting_out_of_bounds() {
        let params =
            BinParams::new(10, 10, 8, 5, 0.0, 100.0, 0.0, 100.0, 0.5, 2.0, 2.0).expect("valid");
        let mut voting = HoughSimilarityVoting::new(params, 50.0, 50.0, 200, 200).expect("valid");

        let result = voting.vote(-10.0, 50.0, 0.0, 1.0);
        assert!(result.is_ok());
        assert!(!result.unwrap());
    }

    #[test]
    fn test_hough_voting_reset() {
        let params =
            BinParams::new(10, 10, 8, 5, 0.0, 100.0, 0.0, 100.0, 0.5, 2.0, 2.0).expect("valid");
        let mut voting = HoughSimilarityVoting::new(params, 50.0, 50.0, 200, 200).expect("valid");

        voting.vote(50.0, 50.0, 0.0, 1.0).expect("vote ok");
        assert!(voting.get_maximum_votes().is_some());

        voting.reset();
        assert!(voting.get_maximum_votes().is_none());
    }

    #[test]
    fn test_find_hough_similarity_no_matches() {
        let params =
            BinParams::new(10, 10, 8, 5, 0.0, 100.0, 0.0, 100.0, 0.5, 2.0, 2.0).expect("valid");
        let mut voting = HoughSimilarityVoting::new(params, 50.0, 50.0, 200, 200).expect("valid");
        let query_points = [];
        let ref_points = [];
        let matches = [];

        let result = find_hough_similarity(
            &mut voting,
            &query_points,
            &ref_points,
            &matches,
            100,
            100,
            100,
            100,
        );
        assert!(result.is_err());
    }

    #[test]
    fn test_find_hough_matches_empty_input() {
        // With no input matches the output is also empty regardless of bin_delta.
        let params =
            BinParams::new(10, 10, 8, 5, 0.0, 100.0, 0.0, 100.0, 0.5, 2.0, 2.0).expect("valid");
        let voting = HoughSimilarityVoting::new(params, 50.0, 50.0, 200, 200).expect("valid");
        let query_points: [FeaturePoint; 0] = [];
        let ref_points: [FeaturePoint; 0] = [];
        let in_matches: [HoughMatch; 0] = [];

        let mut out = Vec::new();
        find_hough_matches(
            &mut out,
            &voting,
            &query_points,
            &ref_points,
            &in_matches,
            0,
            1.0,
        )
        .expect("ok");
        assert!(out.is_empty());
    }

    #[test]
    fn test_find_hough_matches_filters_by_bin_delta() {
        // Build two matches:
        //   - match A: query and ref points coincide exactly → similarity is
        //     pure identity → falls into a specific bin.
        //   - match B: ref point shifted far enough to land in a completely
        //     different bin.
        // Run voting once to discover the winning bin (which is A's bin), then
        // verify that find_hough_matches retains A and rejects B.

        let params = BinParams::new(
            12, 12, 12, 10, // bins
            -100.0, 100.0, // x range
            -100.0, 100.0, // y range
            -2.0, 2.0, // log-scale range
            2.0, // scale_k
        )
        .expect("valid");
        let mut voting = HoughSimilarityVoting::new(params, 0.0, 0.0, 200, 200).expect("valid");

        // Match A: identity transform.
        let qa = FeaturePoint {
            x: 10.0,
            y: 20.0,
            angle: 0.0,
            scale: 1.0,
            maxima: true,
        };
        let ra = FeaturePoint {
            x: 10.0,
            y: 20.0,
            angle: 0.0,
            scale: 1.0,
            maxima: true,
        };
        // Match B: large translation → different bin.
        let qb = FeaturePoint {
            x: 80.0,
            y: 80.0,
            angle: 0.0,
            scale: 1.0,
            maxima: true,
        };
        let rb = FeaturePoint {
            x: 10.0,
            y: 20.0,
            angle: 0.0,
            scale: 1.0,
            maxima: true,
        };

        let query_pts = vec![qa, qb];
        let ref_pts = vec![ra, rb];

        // Vote for both (only A succeeds; B may also vote into a different bin).
        let (xa, ya, aa, sa) =
            compute_similarity(&qa, &ra, voting.center_x, voting.center_y).unwrap();
        voting.vote(xa, ya, aa, sa).unwrap();
        let (xb, yb, ab, sb) =
            compute_similarity(&qb, &rb, voting.center_x, voting.center_y).unwrap();
        let _ = voting.vote(xb, yb, ab, sb); // may be in-bounds or not; we don't care here.

        let in_matches = vec![
            HoughMatch {
                query_idx: 0,
                ref_idx: 0,
                distance: 0.0,
            },
            HoughMatch {
                query_idx: 1,
                ref_idx: 1,
                distance: 0.0,
            },
        ];

        // Pick A's bin as the winner: recompute its bin index.
        let (fb_x, fb_y, fb_angle, fb_scale) = voting.params.map_to_bin(xa, ya, aa, sa);
        let bx = (fb_x - 0.5).floor() as i32;
        let by = (fb_y - 0.5).floor() as i32;
        let mut ba = (fb_angle - 0.5).floor() as i32;
        let bs = (fb_scale - 0.5).floor() as i32;
        ba = (ba + voting.params.num_angle_bins) % voting.params.num_angle_bins;
        let winning_idx = voting.params.bin_index(bx, by, ba, bs);

        let mut out = Vec::new();
        find_hough_matches(
            &mut out,
            &voting,
            &query_pts,
            &ref_pts,
            &in_matches,
            winning_idx,
            1.0,
        )
        .expect("ok");

        // A must be retained; B must not.
        assert_eq!(out.len(), 1, "expected exactly 1 retained match");
        assert_eq!(out[0].query_idx, 0);
    }
}

// ============================================================================
// Dual-mode validation against the C++ baseline (Milestone 9, #150)
// ============================================================================
//
// Verifies that the Rust `recompute_xy_bins_from_matches` produces
// byte-identical `(num_x_bins, num_y_bins)` to C++ `autoAdjustXYNumBins`
// for the same seeded random inputs. Isolates the auto-adjust parity at
// the algorithm level, separately from the end-to-end VisualDatabase
// parity gate in `visual_database.rs`.

#[cfg(feature = "dual-mode")]
extern "C" {
    /// M9 #150: reimplementation of `HoughSimilarityVoting::autoAdjustXYNumBins`
    /// in `kpm_c_api.cpp` (the C++ method is private, so the shim ports the
    /// formula using public `vision::SafeDivision` + `vision::FastMedian`).
    /// See `kpm_c_api.h` for full doc.
    #[allow(clippy::too_many_arguments)]
    fn webarkit_cpp_auto_adjust_xy_num_bins(
        ins: *const f32,
        ref_pts: *const f32,
        size: i32,
        ref_image_width: i32,
        ref_image_height: i32,
        min_x: f32,
        max_x: f32,
        min_y: f32,
        max_y: f32,
        num_angle_bins: i32,
        num_scale_bins: i32,
        out_num_x_bins: *mut i32,
        out_num_y_bins: *mut i32,
    ) -> i32;
}

#[cfg(all(test, feature = "dual-mode"))]
mod dual_mode_tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    /// Sweep seeded random match pairs through both Rust and C++
    /// auto-adjust paths and assert byte-identical `(num_x_bins, num_y_bins)`.
    #[test]
    fn auto_adjust_xy_num_bins_matches_cpp() {
        let mut rng = StdRng::seed_from_u64(0xABCDEF01u64);
        let num_angle_bins = 12i32;
        let num_scale_bins = 10i32;

        for trial in 0..40 {
            let size = rng.random_range(5..=300);
            let ref_w = rng.random_range(200..=2000);
            let ref_h = rng.random_range(200..=2000);
            let half_dx = rng.random_range(100.0_f32..3000.0);
            let half_dy = rng.random_range(100.0_f32..3000.0);

            // Build seeded match pairs. We only read [3] (scale) so the
            // other fields are arbitrary noise. ins and ref are flat
            // arrays of (x, y, angle, scale) — 4 floats per match.
            let mut ins = vec![0.0_f32; (size * 4) as usize];
            let mut refs = vec![0.0_f32; (size * 4) as usize];
            for i in 0..size as usize {
                ins[i * 4 + 3] = rng.random_range(0.1_f32..10.0);
                refs[i * 4 + 3] = rng.random_range(0.1_f32..10.0);
            }

            // ----- Rust path -----
            // We exercise the Rust code via the public-ish hooks: build a
            // BinParams with new_auto_xy, a HoughSimilarityVoting, then
            // call recompute_xy_bins_from_matches directly with synthetic
            // FeaturePoint + HoughMatch arrays whose `scale` reads pull
            // from the same source data.
            let params = BinParams::new_auto_xy(
                num_angle_bins,
                num_scale_bins,
                -half_dx,
                half_dx,
                -half_dy,
                half_dy,
                -1.0,
                1.0,
                10.0,
            )
            .unwrap();
            let mut voter = HoughSimilarityVoting::new(params, 0.0, 0.0, ref_w, ref_h).unwrap();

            // Inflate ins/refs into FeaturePoint arrays (only `scale` matters).
            let query_pts: Vec<FeaturePoint> = (0..size as usize)
                .map(|i| FeaturePoint {
                    x: 0.0,
                    y: 0.0,
                    angle: 0.0,
                    scale: ins[i * 4 + 3],
                    maxima: true,
                })
                .collect();
            let ref_pts: Vec<FeaturePoint> = (0..size as usize)
                .map(|i| FeaturePoint {
                    x: 0.0,
                    y: 0.0,
                    angle: 0.0,
                    scale: refs[i * 4 + 3],
                    maxima: true,
                })
                .collect();
            let matches: Vec<HoughMatch> = (0..size as u32)
                .map(|i| HoughMatch {
                    query_idx: i,
                    ref_idx: i,
                    distance: 0.0,
                })
                .collect();
            voter
                .recompute_xy_bins_from_matches(&query_pts, &ref_pts, &matches)
                .unwrap();
            let rust_x = voter.params.num_x_bins();
            let rust_y = voter.params.num_y_bins();

            // ----- C++ path -----
            let mut cpp_x = 0i32;
            let mut cpp_y = 0i32;
            let rc = unsafe {
                webarkit_cpp_auto_adjust_xy_num_bins(
                    ins.as_ptr(),
                    refs.as_ptr(),
                    size,
                    ref_w,
                    ref_h,
                    -half_dx,
                    half_dx,
                    -half_dy,
                    half_dy,
                    num_angle_bins,
                    num_scale_bins,
                    &mut cpp_x,
                    &mut cpp_y,
                )
            };
            assert_eq!(rc, 0, "trial {}: C++ shim returned error", trial);

            assert_eq!(
                rust_x, cpp_x,
                "trial {} (size={}, ref={}x{}): num_x_bins rust={} cpp={}",
                trial, size, ref_w, ref_h, rust_x, cpp_x
            );
            assert_eq!(
                rust_y, cpp_y,
                "trial {} (size={}, ref={}x{}): num_y_bins rust={} cpp={}",
                trial, size, ref_w, ref_h, rust_y, cpp_y
            );
        }
    }
}
