/*
 *  freak/mod.rs
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

//! FREAK (Fast Retina Keypoint) descriptor-based feature matching utilities.
//!
//! This module contains the core math and homography utilities used by the
//! FreakMatcher feature detector. Functions are ported from the C++ WebARKitLib
//! implementation and optimized for performance.

pub mod clustering;
pub mod homography;
pub mod hough;
pub mod math;

// Public re-exports for convenience
pub use clustering::{hamming_distance_96, BhcNode, BinaryHierarchicalClustering, KMedoids};
pub use hough::{
    find_features, find_hough_matches, find_hough_similarity, BinParams, DoGScaleInvariantDetector,
    FeaturePoint, GaussianScaleSpacePyramid, HoughSimilarityVoting, Keyframe, Match,
};
