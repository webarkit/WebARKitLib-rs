/*
 *  keyframe.rs
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

//! Per-frame container holding FREAK descriptors and feature points.
//!
//! Mirrors C++ `vision::Keyframe<96>` from `keyframe.h`. The C++ template
//! parameter `NUM_BYTES_PER_FEATURE = 96` matches our
//! [`super::descriptor::FREAK_DESCRIPTOR_BYTES`].
//!
//! Keyframe is a **passive container**: it stores the per-frame results
//! but does not run the detection / extraction pipeline itself. The
//! orchestration that populates this container lives in
//! [`super::hough::find_features`] (mirroring C++ `vision::FindFeatures`
//! in `visual_database.h`).
//!
//! The C++ Keyframe also holds a `BinaryHierarchicalClustering` index for
//! fast nearest-neighbour matching. That index is **not** built here —
//! downstream KPM matching (already in main from M7) constructs its own
//! index when needed.

use crate::kpm::backend::KpmError;

use super::clustering::BinaryHierarchicalClustering;
use super::descriptor::FREAK_DESCRIPTOR_BYTES;
use super::matcher::FeatureStore;

// ─────────────────────────────────────────────────────────────────────────
// C++ Keyframe<>::buildIndex defaults (keyframe.h:116-122)
// ─────────────────────────────────────────────────────────────────────────

/// `Keyframe<>::buildIndex` setNumHypotheses argument.
const KEYFRAME_BUILD_INDEX_NUM_HYPOTHESES: usize = 128;

/// `Keyframe<>::buildIndex` setNumCenters argument.
const KEYFRAME_BUILD_INDEX_NUM_CENTERS: usize = 8;

/// `Keyframe<>::buildIndex` setMaxNodesToPop argument.
const KEYFRAME_BUILD_INDEX_MAX_NODES_TO_POP: usize = 8;

/// `Keyframe<>::buildIndex` setMinFeaturesPerNode argument.
const KEYFRAME_BUILD_INDEX_MIN_FEATURES_PER_NODE: usize = 16;

/// Per-frame container holding FREAK descriptors and feature points.
///
/// C equivalent: `vision::Keyframe<96>`.
pub struct Keyframe {
    /// FREAK descriptors and their associated feature points.
    /// `bytes_per_feature` is fixed at [`FREAK_DESCRIPTOR_BYTES`] = 96.
    pub store: FeatureStore,
    /// Width of the source image.
    pub width: i32,
    /// Height of the source image.
    pub height: i32,
    /// BHC feature index built from `store`, populated by [`build_index`].
    /// `None` until [`build_index`] is called.
    ///
    /// C equivalent: `Keyframe<NUM_BYTES_PER_FEATURE>::mIndex`
    /// (`keyframe.h:111`). M9 #146 moved ownership of the index from
    /// `FeatureMatcher` to here so it can be built once at insertion time
    /// rather than rebuilt per query.
    index: Option<BinaryHierarchicalClustering>,
}

impl Keyframe {
    /// Create an empty Keyframe sized for `(width, height)` source images.
    /// The internal [`FeatureStore`] is configured for 96-byte descriptors.
    /// The BHC index starts as `None`; call [`build_index`] after populating
    /// `store` (typically via `find_features`).
    pub fn new(width: i32, height: i32) -> Result<Self, KpmError> {
        Ok(Self {
            store: FeatureStore::new(FREAK_DESCRIPTOR_BYTES)?,
            width,
            height,
            index: None,
        })
    }

    /// Build the BHC feature index from `self.store`.
    ///
    /// Configures the [`BinaryHierarchicalClustering`] with the exact settings
    /// from C++ `Keyframe<NUM_BYTES_PER_FEATURE>::buildIndex`
    /// (`keyframe.h:116-122`): `num_hypotheses=128, num_centers=8,
    /// max_nodes_to_pop=8, min_features_per_node=16`.
    ///
    /// Idempotent: calling twice replaces the previous index. Errors
    /// propagate from [`BinaryHierarchicalClustering::build`] (e.g. empty
    /// store).
    pub fn build_index(&mut self) -> Result<(), KpmError> {
        let mut bhc = BinaryHierarchicalClustering::new()?;
        bhc.set_num_hypotheses(KEYFRAME_BUILD_INDEX_NUM_HYPOTHESES)?;
        bhc.set_num_centers(KEYFRAME_BUILD_INDEX_NUM_CENTERS)?;
        bhc.set_max_nodes_to_pop(KEYFRAME_BUILD_INDEX_MAX_NODES_TO_POP);
        bhc.set_min_features_per_leaf(KEYFRAME_BUILD_INDEX_MIN_FEATURES_PER_NODE)?;

        let descriptors: Vec<&[u8; 96]> = (0..self.store.num_features())
            .map(|i| {
                let bytes = self.store.descriptor(i);
                <&[u8; 96]>::try_from(bytes).expect("FREAK descriptors are always 96 bytes")
            })
            .collect();

        bhc.build(&descriptors)?;
        self.index = Some(bhc);
        Ok(())
    }

    /// Borrow the BHC index.
    ///
    /// Returns `None` until [`build_index`] is called. Used by
    /// `FeatureMatcher::match_with_index` and `VisualDatabase::try_match_one`.
    ///
    /// C equivalent: `Keyframe<>::index() const`.
    pub fn index(&self) -> Option<&BinaryHierarchicalClustering> {
        self.index.as_ref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kpm::freak::detector::DoGScaleInvariantDetector;
    use crate::kpm::freak::gaussian_pyramid::GaussianScaleSpacePyramid;
    use crate::kpm::freak::hough::find_features;
    use purecv::core::Matrix;

    fn load_grayscale(path: &str) -> Matrix<u8> {
        let img = image::open(path).expect("load test image").to_luma8();
        let (w, h) = img.dimensions();
        Matrix::<u8>::from_vec(h as usize, w as usize, 1, img.into_raw())
    }

    #[test]
    fn test_keyframe_new_stores_dimensions() {
        let kf = Keyframe::new(640, 480).unwrap();
        assert_eq!(kf.width, 640);
        assert_eq!(kf.height, 480);
        assert_eq!(kf.store.num_features(), 0);
    }

    #[test]
    #[cfg_attr(miri, ignore)] // #194: full pyramid + DoG pipeline on real image — too slow under Miri
    fn test_find_features_populates_keyframe() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let mut pyr = GaussianScaleSpacePyramid::new(3);
        pyr.build(&img).unwrap();
        let det = DoGScaleInvariantDetector::new(0.0, 10.0, 5000, true);

        let mut kf = Keyframe::new(img.cols as i32, img.rows as i32).unwrap();
        find_features(&mut kf, &pyr, &det).expect("find_features");
        assert!(
            kf.store.num_features() > 50,
            "expected > 50 features on found.jpg, got {}",
            kf.store.num_features()
        );
    }

    // ------------------------------------------------------------------
    // build_index lifecycle (M9 #146)
    // ------------------------------------------------------------------

    #[test]
    fn test_keyframe_index_is_none_before_build() {
        let kf = Keyframe::new(640, 480).unwrap();
        assert!(kf.index().is_none());
    }

    #[test]
    #[cfg_attr(miri, ignore)] // #194: full pyramid + DoG + BHC pipeline on real image — too slow under Miri
    fn test_keyframe_build_index_populates_index() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let mut pyr = GaussianScaleSpacePyramid::new(3);
        pyr.build(&img).unwrap();
        let det = DoGScaleInvariantDetector::new(0.0, 10.0, 5000, true);

        let mut kf = Keyframe::new(img.cols as i32, img.rows as i32).unwrap();
        find_features(&mut kf, &pyr, &det).expect("find_features");
        assert!(
            kf.index().is_none(),
            "find_features must not build the index"
        );

        kf.build_index().expect("build_index");
        assert!(kf.index().is_some(), "build_index must populate the index");
    }

    #[test]
    #[cfg_attr(miri, ignore)] // #194: full pipeline run twice — too slow under Miri
    fn test_keyframe_build_index_is_idempotent() {
        // Calling build_index twice should not error; the second call replaces
        // the first cleanly (mirrors C++ Keyframe::buildIndex which always
        // rebuilds from scratch).
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let mut pyr = GaussianScaleSpacePyramid::new(3);
        pyr.build(&img).unwrap();
        let det = DoGScaleInvariantDetector::new(0.0, 10.0, 5000, true);

        let mut kf = Keyframe::new(img.cols as i32, img.rows as i32).unwrap();
        find_features(&mut kf, &pyr, &det).expect("find_features");
        kf.build_index().expect("first build_index");
        kf.build_index().expect("second build_index");
        assert!(kf.index().is_some());
    }

    #[test]
    fn test_keyframe_build_index_empty_store_errors() {
        // BHC::build returns Err on empty input; Keyframe::build_index must propagate.
        let mut kf = Keyframe::new(640, 480).unwrap();
        assert!(kf.build_index().is_err());
    }
}
