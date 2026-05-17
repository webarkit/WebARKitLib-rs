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

use super::descriptor::FREAK_DESCRIPTOR_BYTES;
use super::matcher::FeatureStore;

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
}

impl Keyframe {
    /// Create an empty Keyframe sized for `(width, height)` source images.
    /// The internal [`FeatureStore`] is configured for 96-byte descriptors.
    pub fn new(width: i32, height: i32) -> Result<Self, KpmError> {
        Ok(Self {
            store: FeatureStore::new(FREAK_DESCRIPTOR_BYTES)?,
            width,
            height,
        })
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
}
