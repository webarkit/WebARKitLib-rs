/*
 *  ar2/surface.rs
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

//! AR2 Surface Set — loading, construction and helpers.
//!
//! Ported from the C `AR2/surface.c` file in ARToolKit5.
//! Contains [`AR2Surface`], [`AR2SurfaceSet`], and the
//! [`ar2_read_surface_set`] / [`ar2_read_surface_set_from_bytes`] loaders.

use crate::ar2::feature_set::AR2FeatureSetT;
use crate::ar2::image_set::AR2ImageSetT;
use crate::ar2::tracking::{
    AR2FeatureCoord, AR2FeaturePoints, AR2FeatureSet, AR2Image, AR2ImageSet, AR2TemplateCandidate,
    AR2_BLUR_IMAGE_MAX, AR2_SEARCH_FEATURE_MAX,
};
use std::path::Path;

// MarkerSet is ignored for now per user rules (no multi-marker / complex markers yet)
// but we might need a placeholder if it's tightly coupled.

/// One trackable surface of an NFT marker: its image pyramid, feature set,
/// and the pose transforms relating the surface to the marker.
#[derive(Debug, Default, Clone)]
pub struct AR2Surface {
    /// Multi-resolution image pyramid (`.iset`), if loaded.
    pub image_set: Option<AR2ImageSet>,
    /// Tracking feature set (`.fset`), if loaded.
    pub feature_set: Option<AR2FeatureSet>,
    /// Surface → marker transform (3×4).
    pub trans: [[f32; 4]; 3],
    /// Inverse of [`trans`](Self::trans) (marker → surface).
    pub itrans: [[f32; 4]; 3],
}

/// A set of trackable surfaces plus the per-frame AR2 tracking state
/// (recent pose history and previously matched features).
#[derive(Debug, Clone)]
pub struct AR2SurfaceSet {
    /// The individual trackable surfaces.
    pub surface: Vec<AR2Surface>,
    /// Most recent pose (3×4); the current tracking estimate.
    pub trans1: [[f32; 4]; 3],
    /// Pose from one frame ago.
    pub trans2: [[f32; 4]; 3],
    /// Pose from two frames ago.
    pub trans3: [[f32; 4]; 3],
    /// Number of consecutive frames tracked so far (0 = not currently tracked).
    pub cont_num: i32,
    /// Feature candidates carried over from the previous frame.
    pub prev_feature: Vec<AR2TemplateCandidate>,
}

impl Default for AR2SurfaceSet {
    fn default() -> Self {
        Self {
            surface: Vec::new(),
            trans1: [[0.0; 4]; 3],
            trans2: [[0.0; 4]; 3],
            trans3: [[0.0; 4]; 3],
            cont_num: 0,
            prev_feature: vec![AR2TemplateCandidate::default(); AR2_SEARCH_FEATURE_MAX + 1],
        }
    }
}

impl AR2SurfaceSet {
    /// Set the initial camera pose for tracking.
    ///
    /// Rust equivalent of `ar2SetInitTrans()` in the C API (`surface.c`).
    /// This sets `cont_num = 1` and stores the initial pose in `trans1`,
    /// enabling `ar2_tracking()` to run.
    pub fn set_init_trans(&mut self, trans: &[[f32; 4]; 3]) {
        self.cont_num = 1;
        self.trans1 = *trans;
        self.prev_feature[0].flag = -1;
    }
}

// ---------------------------------------------------------------------------
// ar2_read_surface_set — Rust equivalent of C `ar2ReadSurfaceSet()`
// ---------------------------------------------------------------------------

/// Read an NFT surface set from disk, given a base path name.
///
/// This is the Rust equivalent of the C function `ar2ReadSurfaceSet()` in
/// `AR2/surface.c`. It loads both the `.iset` (image pyramid) and `.fset`
/// (feature points) files from the given base name, constructs an
/// `AR2SurfaceSet` with a single surface, and applies an identity transform.
///
/// This mirrors the strategy used in nftSimple.c:
///
/// ```c
/// surfaceSet = ar2ReadSurfaceSet(markersNFT[i].datasetPathname, "fset", NULL);
/// ```
///
/// # Arguments
///
/// * `basename` — Base file path without extension (e.g. `"Data/pinball"`).
///   The function appends `.iset` and `.fset` to load both files.
///
/// # Example
///
/// ```no_run
/// use webarkitlib_rs::ar2::ar2_read_surface_set;
///
/// let surface_set = ar2_read_surface_set("examples/Data/pinball").unwrap();
/// assert_eq!(surface_set.surface.len(), 1);
/// ```
pub fn ar2_read_surface_set<P: AsRef<Path>>(basename: P) -> std::io::Result<AR2SurfaceSet> {
    let base = basename.as_ref();

    // Load .iset (image pyramid).
    let iset_path = base.with_extension("iset");
    let io_image_set = AR2ImageSetT::load(&iset_path)?;

    // Load .fset (feature points).
    let fset_path = base.with_extension("fset");
    let io_feature_set = AR2FeatureSetT::load(&fset_path)?;

    Ok(build_surface_set(&io_image_set, &io_feature_set))
}

/// Read an NFT surface set from in-memory byte slices.
///
/// This is the WASM-friendly variant of [`ar2_read_surface_set`] that
/// accepts pre-fetched `.iset` and `.fset` data as byte slices instead
/// of reading from the filesystem.
///
/// # Arguments
///
/// * `iset_bytes` — Contents of the `.iset` file (image pyramid).
/// * `fset_bytes` — Contents of the `.fset` file (feature points).
pub fn ar2_read_surface_set_from_bytes(
    iset_bytes: &[u8],
    fset_bytes: &[u8],
) -> std::io::Result<AR2SurfaceSet> {
    let io_image_set = AR2ImageSetT::from_bytes(iset_bytes)?;
    let io_feature_set = AR2FeatureSetT::from_bytes(fset_bytes)?;
    Ok(build_surface_set(&io_image_set, &io_feature_set))
}

/// Construct an `AR2SurfaceSet` from parsed I/O types.
///
/// Converts the I/O types (`AR2ImageSetT`, `AR2FeatureSetT`) to the
/// tracking types (`AR2ImageSet`, `AR2FeatureSet`) and creates a
/// single-surface set with an identity transform.
fn build_surface_set(
    io_image_set: &AR2ImageSetT,
    io_feature_set: &AR2FeatureSetT,
) -> AR2SurfaceSet {
    let tracking_image_set = convert_image_set(io_image_set);
    let tracking_feature_set = convert_feature_set(io_feature_set);

    // Identity transform (single marker, marker-local = world).
    let identity: [[f32; 4]; 3] = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ];

    let surface = AR2Surface {
        image_set: Some(tracking_image_set),
        feature_set: Some(tracking_feature_set),
        trans: identity,
        itrans: identity,
    };

    AR2SurfaceSet {
        surface: vec![surface],
        ..Default::default()
    }
}

/// Convert an `AR2ImageSetT` (I/O type) to `AR2ImageSet` (tracking type).
///
/// The tracking module expects `AR2Image` with `img_bw_blur` containing
/// multiple blur levels. We populate blur level 0 with the raw pixels
/// and leave higher levels as `None`.
fn convert_image_set(io_set: &AR2ImageSetT) -> AR2ImageSet {
    let mut scales = Vec::with_capacity(io_set.scale.len());

    for io_img in &io_set.scale {
        let mut blur_levels: Vec<Option<Vec<u8>>> = Vec::with_capacity(AR2_BLUR_IMAGE_MAX);

        // Level 0 = original image pixels.
        blur_levels.push(Some(io_img.img_bw.clone()));

        // Fill remaining levels with None.
        for _ in 1..AR2_BLUR_IMAGE_MAX {
            blur_levels.push(None);
        }

        scales.push(AR2Image {
            img_bw_blur: blur_levels,
            xsize: io_img.xsize,
            ysize: io_img.ysize,
            dpi: io_img.dpi,
        });
    }

    AR2ImageSet { scale: scales }
}

/// Convert an `AR2FeatureSetT` (I/O type) to `AR2FeatureSet` (tracking type).
fn convert_feature_set(io_set: &AR2FeatureSetT) -> AR2FeatureSet {
    let mut list = Vec::with_capacity(io_set.list.len());

    for io_fp in &io_set.list {
        let coord: Vec<AR2FeatureCoord> = io_fp
            .coord
            .iter()
            .map(|c| AR2FeatureCoord {
                x: c.x,
                y: c.y,
                mx: c.mx,
                my: c.my,
                max_sim: c.max_sim,
            })
            .collect();

        list.push(AR2FeaturePoints {
            coord,
            scale: io_fp.scale,
            maxdpi: io_fp.maxdpi,
            mindpi: io_fp.mindpi,
        });
    }

    AR2FeatureSet { list }
}

/// Get the marker dimensions from an `AR2SurfaceSet`.
///
/// Returns `(width, height, dpi)` from the first surface's image set
/// at scale 0, or `None` if the surface set is empty or has no image data.
pub fn ar2_surface_set_marker_info(surface_set: &AR2SurfaceSet) -> Option<(i32, i32, f32)> {
    let surface = surface_set.surface.first()?;
    let image_set = surface.image_set.as_ref()?;
    let base = image_set.scale.first()?;
    Some((base.xsize, base.ysize, base.dpi))
}
