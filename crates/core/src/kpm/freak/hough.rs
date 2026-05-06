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
 *  Copyright 2025 WebARKit.
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

use crate::{arlog_d, arlog_e, arlog_i};
use crate::kpm::backend::KpmError;
use std::collections::HashMap;

/// Minimum number of votes required for a bin to be considered a valid result.
const MIN_VOTES_THRESHOLD: i32 = 3;

/// Encapsulates bin discretization parameters and provides bin calculation utilities.
#[derive(Clone)]
pub struct BinParams {
    /// Number of bins for x translation.
    pub num_x_bins: i32,
    /// Number of bins for y translation.
    pub num_y_bins: i32,
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
            return Err(KpmError::InvalidInput(
                "num_x_bins must be > 0".into(),
            ));
        }
        if num_y_bins <= 0 {
            arlog_e!("BinParams::new: num_y_bins must be > 0, got {}", num_y_bins);
            return Err(KpmError::InvalidInput(
                "num_y_bins must be > 0".into(),
            ));
        }
        if num_angle_bins <= 0 {
            arlog_e!(
                "BinParams::new: num_angle_bins must be > 0, got {}",
                num_angle_bins
            );
            return Err(KpmError::InvalidInput(
                "num_angle_bins must be > 0".into(),
            ));
        }
        if num_scale_bins <= 0 {
            arlog_e!(
                "BinParams::new: num_scale_bins must be > 0, got {}",
                num_scale_bins
            );
            return Err(KpmError::InvalidInput(
                "num_scale_bins must be > 0".into(),
            ));
        }

        // Validate ranges
        if min_x >= max_x {
            arlog_e!(
                "BinParams::new: invalid x range: min={}, max={}",
                min_x,
                max_x
            );
            return Err(KpmError::InvalidInput(
                "min_x must be < max_x".into(),
            ));
        }
        if min_y >= max_y {
            arlog_e!(
                "BinParams::new: invalid y range: min={}, max={}",
                min_y,
                max_y
            );
            return Err(KpmError::InvalidInput(
                "min_y must be < max_y".into(),
            ));
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
        })
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

        let fb_scale =
            self.num_scale_bins as f32 * (scale - self.min_scale) / (self.max_scale - self.min_scale);

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
    votes: HashMap<i32, i32>,
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
            votes: HashMap::new(),
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
        let mut bx = (fb_x - 0.5).floor() as i32;
        let mut by = (fb_y - 0.5).floor() as i32;
        let mut ba = (fb_angle - 0.5).floor() as i32;
        let mut bs = (fb_scale - 0.5).floor() as i32;

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
            arlog_d!("vote neighborhood out of bounds: bx={}, by={}, bs={}", bx, by, bs);
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

/// Stub types for M8 (to be implemented in Milestone 8).
/// These are placeholders to allow M7 to compile independently.

/// Placeholder for DoGScaleInvariantDetector from M8.
#[derive(Clone, Debug)]
pub struct DoGScaleInvariantDetector;

/// Placeholder for GaussianScaleSpacePyramid from M8.
#[derive(Clone, Debug)]
pub struct GaussianScaleSpacePyramid;

/// Placeholder for Keyframe from M8.
#[derive(Clone, Debug)]
pub struct Keyframe {
    pub features: Vec<FeaturePoint>,
}

/// A feature point with position, orientation, and scale.
#[derive(Clone, Debug, Copy)]
pub struct FeaturePoint {
    pub x: f32,
    pub y: f32,
    pub angle: f32,
    pub scale: f32,
}

/// A correspondence between a query and reference feature.
#[derive(Clone, Copy, Debug)]
pub struct Match {
    pub query_idx: u32,
    pub ref_idx: u32,
    pub distance: f32,
}

/// Stub for FindFeatures (M8 will implement).
pub fn find_features(
    _keyframe: &mut Keyframe,
    _detector: &DoGScaleInvariantDetector,
    _pyramid: &GaussianScaleSpacePyramid,
) -> Result<(), KpmError> {
    arlog_i!("find_features: stub implementation (M8 pending)");
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
pub fn find_hough_similarity(
    voting: &mut HoughSimilarityVoting,
    query_points: &[FeaturePoint],
    ref_points: &[FeaturePoint],
    matches: &[Match],
    _query_width: i32,
    _query_height: i32,
    _ref_width: i32,
    _ref_height: i32,
) -> Result<i32, KpmError> {
    if matches.is_empty() {
        arlog_d!("find_hough_similarity: no matches provided");
        return Err(KpmError::InvalidInput("insufficient votes for feature matching".into()));
    }

    voting.reset();

    let mut vote_count = 0i32;
    for m in matches {
        let q_pt = &query_points[m.query_idx as usize];
        let r_pt = &ref_points[m.ref_idx as usize];

        // Compute similarity transformation from correspondence
        let (x, y, angle, scale) = compute_similarity(q_pt, r_pt, voting.center_x, voting.center_y)?;

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
    Err(KpmError::InvalidInput("insufficient votes for feature matching".into()))
}

/// Filters matches to those consistent with the winning Hough bin.
///
/// # Arguments
/// * `out_matches` - Output vector for inlier matches
/// * `voting` - Voter containing the winning bin index
/// * `all_matches` - All matches to filter
/// * `max_hough_index` - The winning bin index
/// * `bin_delta` - Maximum bin distance to be considered an inlier
#[allow(unused_variables)]
pub fn find_hough_matches(
    out_matches: &mut Vec<Match>,
    voting: &HoughSimilarityVoting,
    all_matches: &[Match],
    max_hough_index: i32,
    bin_delta: i32,
) -> Result<(), KpmError> {
    let (_max_bx, _max_by, _max_ba, _max_bs) = voting.params.bins_from_index(max_hough_index);
    out_matches.clear();

    for m in all_matches {
        // TODO: Implement actual filtering once feature point data is available.
        // Currently accepts all matches; should filter by bin_delta proximity.
        out_matches.push(*m);
    }

    arlog_d!(
        "find_hough_matches: found {} inliers from {} total matches",
        out_matches.len(),
        all_matches.len()
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

        let result = find_hough_similarity(&mut voting, &query_points, &ref_points, &matches, 100, 100, 100, 100);
        assert!(result.is_err());
    }

    #[test]
    fn test_find_hough_matches() {
        let params =
            BinParams::new(10, 10, 8, 5, 0.0, 100.0, 0.0, 100.0, 0.5, 2.0, 2.0).expect("valid");
        let voting = HoughSimilarityVoting::new(params, 50.0, 50.0, 200, 200).expect("valid");

        let all_matches = vec![
            Match { query_idx: 0, ref_idx: 0, distance: 0.1 },
            Match { query_idx: 1, ref_idx: 1, distance: 0.2 },
        ];

        let mut out_matches = Vec::new();
        let result = find_hough_matches(&mut out_matches, &voting, &all_matches, 0, 2);
        assert!(result.is_ok());
        assert_eq!(out_matches.len(), all_matches.len());
    }
}
