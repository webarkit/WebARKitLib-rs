/*
 *  ref_data_set.rs
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

//! KpmRefDataSet I/O — generate, save, load, merge.
//!
//! Ported from the C++ `kpmRefDataSet.cpp` (BINARY_FEATURE=1 path) and
//! the image-resize helpers from `kpmUtil.cpp`.
//!
//! This module adds methods to [`KpmRefDataSet`] for binary
//! serialisation/deserialisation in the same little-endian format as the
//! original C code, as well as [`generate`](KpmRefDataSet::generate),
//! [`merge`](KpmRefDataSet::merge), and
//! [`change_page_no`](KpmRefDataSet::change_page_no).

use std::io::Write;
use std::path::Path;

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};

use crate::kpm::backend::{FreakMatcherBackend, KpmError};
use crate::kpm::handle::KpmProcMode;
use crate::kpm::types::{
    FreakFeature, KpmCoord2D, KpmImageInfo, KpmPageInfo, KpmRefData, KpmRefDataSet, RefImage,
    FREAK_SUB_DIMENSION,
};

// ---------------------------------------------------------------------------
// KpmCompMode
// ---------------------------------------------------------------------------

/// Compression mode applied to the reference image before feature
/// extraction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KpmCompMode {
    /// No compression — use the image as-is.
    None,
    /// Vertically compress by averaging pairs of rows (halves the
    /// effective height before detection, then doubles Y coordinates
    /// back).
    CompY,
}

// ---------------------------------------------------------------------------
// Sentinel for change_page_no "all pages"
// ---------------------------------------------------------------------------

/// Pass this as `old_page_no` to [`KpmRefDataSet::change_page_no`] to
/// rename **every** page whose `page_no >= 0`.
pub const KPM_CHANGE_PAGE_NO_ALL_PAGES: i32 = -1;

/// Standard file extension for KPM reference data sets (BINARY_FEATURE
/// / FREAK format).
///
/// The C++ code conventionally uses `"fset3"` as the extension argument
/// to `kpmSaveRefDataSet` / `kpmLoadRefDataSet`.
pub const KPM_REF_DATA_SET_EXT: &str = "fset3";

// ---------------------------------------------------------------------------
// Image resize helpers (ported from kpmUtil.cpp)
// ---------------------------------------------------------------------------

/// Returns a copy of the image (no resize).
fn gen_bw_image_full(image: &[u8], xsize: usize, ysize: usize) -> (Vec<u8>, usize, usize) {
    debug_assert!(image.len() >= xsize * ysize);
    let out = image[..xsize * ysize].to_vec();
    (out, xsize, ysize)
}

/// 2x2 box-filter downsample → half size.
fn gen_bw_image_half(image: &[u8], xsize: usize, ysize: usize) -> (Vec<u8>, usize, usize) {
    let xsize2 = xsize / 2;
    let ysize2 = ysize / 2;
    let mut out = Vec::with_capacity(xsize2 * ysize2);
    for j in 0..ysize2 {
        let row0 = j * 2;
        let row1 = row0 + 1;
        for i in 0..xsize2 {
            let col = i * 2;
            let v = (image[row0 * xsize + col] as u32
                + image[row0 * xsize + col + 1] as u32
                + image[row1 * xsize + col] as u32
                + image[row1 * xsize + col + 1] as u32)
                / 4;
            out.push(v as u8);
        }
    }
    (out, xsize2, ysize2)
}

/// 3x3 box-filter downsample → one-third size.
fn gen_bw_image_one_third(image: &[u8], xsize: usize, ysize: usize) -> (Vec<u8>, usize, usize) {
    let xsize2 = xsize / 3;
    let ysize2 = ysize / 3;
    let mut out = Vec::with_capacity(xsize2 * ysize2);
    for j in 0..ysize2 {
        let r0 = j * 3;
        for i in 0..xsize2 {
            let c = i * 3;
            let mut sum = 0u32;
            for dy in 0..3 {
                for dx in 0..3 {
                    sum += image[(r0 + dy) * xsize + c + dx] as u32;
                }
            }
            out.push((sum / 9) as u8);
        }
    }
    (out, xsize2, ysize2)
}

/// Weighted bilinear downsample → two-thirds size.
///
/// Matches the C++ `genBWImageTwoThird` exactly.
fn gen_bw_image_two_third(image: &[u8], xsize: usize, ysize: usize) -> (Vec<u8>, usize, usize) {
    let xsize2 = xsize / 3 * 2;
    let ysize2 = ysize / 3 * 2;
    let mut out = vec![0u8; xsize2 * ysize2];

    for j in 0..ysize2 / 2 {
        let p1_base = (j * 3) * xsize;
        let p2_base = (j * 3 + 1) * xsize;
        let p3_base = (j * 3 + 2) * xsize;
        let q1_base = (j * 2) * xsize2;
        let q2_base = (j * 2 + 1) * xsize2;

        let mut col_src = 0usize;
        let mut col_dst = 0usize;
        for _i in 0..xsize2 / 2 {
            let p1 = |d: usize| image[p1_base + col_src + d] as i32;
            let p2 = |d: usize| image[p2_base + col_src + d] as i32;
            let p3 = |d: usize| image[p3_base + col_src + d] as i32;

            out[q1_base + col_dst] = ((p1(0) + p1(1) / 2 + p2(0) / 2 + p2(1) / 4) * 4 / 9) as u8;
            out[q2_base + col_dst] = ((p2(0) / 2 + p2(1) / 4 + p3(0) + p3(1) / 2) * 4 / 9) as u8;
            col_src += 1;
            col_dst += 1;

            let p1 = |d: usize| image[p1_base + col_src + d] as i32;
            let p2 = |d: usize| image[p2_base + col_src + d] as i32;
            let p3 = |d: usize| image[p3_base + col_src + d] as i32;

            out[q1_base + col_dst] = ((p1(0) / 2 + p1(1) + p2(0) / 4 + p2(1) / 2) * 4 / 9) as u8;
            out[q2_base + col_dst] = ((p2(0) / 4 + p2(1) / 2 + p3(0) / 2 + p3(1)) * 4 / 9) as u8;
            col_src += 2;
            col_dst += 1;
        }
    }
    (out, xsize2, ysize2)
}

/// 4x4 box-filter downsample → quarter size.
fn gen_bw_image_quart(image: &[u8], xsize: usize, ysize: usize) -> (Vec<u8>, usize, usize) {
    let xsize2 = xsize / 4;
    let ysize2 = ysize / 4;
    let mut out = Vec::with_capacity(xsize2 * ysize2);
    for j in 0..ysize2 {
        let r0 = j * 4;
        for i in 0..xsize2 {
            let c = i * 4;
            let mut sum = 0u32;
            for dy in 0..4 {
                for dx in 0..4 {
                    sum += image[(r0 + dy) * xsize + c + dx] as u32;
                }
            }
            out.push((sum / 16) as u8);
        }
    }
    (out, xsize2, ysize2)
}

/// Resize a grayscale image according to the given [`KpmProcMode`].
///
/// Returns `(resized_pixels, new_width, new_height)`.
pub(crate) fn resize_image(
    image: &[u8],
    xsize: usize,
    ysize: usize,
    proc_mode: KpmProcMode,
) -> (Vec<u8>, usize, usize) {
    match proc_mode {
        KpmProcMode::FullSize => gen_bw_image_full(image, xsize, ysize),
        KpmProcMode::TwoThirdSize => gen_bw_image_two_third(image, xsize, ysize),
        KpmProcMode::HalfSize => gen_bw_image_half(image, xsize, ysize),
        KpmProcMode::OneThirdSize => gen_bw_image_one_third(image, xsize, ysize),
        KpmProcMode::QuarterSize => gen_bw_image_quart(image, xsize, ysize),
    }
}

// ---------------------------------------------------------------------------
// impl KpmRefDataSet — generate / save / load / merge / change_page_no
// ---------------------------------------------------------------------------

/// Constant used to convert pixels to millimetres: `25.4 mm / inch`.
const MM_PER_INCH: f32 = 25.4;

impl KpmRefDataSet {
    /// Generate a new [`KpmRefDataSet`] from a single reference image.
    ///
    /// This is the Rust equivalent of `kpmGenRefDataSet()` in the C++
    /// code (BINARY_FEATURE=1 path).
    ///
    /// # Arguments
    ///
    /// * `image`           — grayscale pixel data (one byte per pixel, row-major).
    /// * `xsize` / `ysize` — image dimensions in pixels.
    /// * `dpi`             — dots per inch (used to compute 3D world coordinates).
    /// * `proc_mode`       — processing resolution (see [`KpmProcMode`]).
    /// * `comp_mode`       — vertical compression mode (see [`KpmCompMode`]).
    /// * `_max_feature_num` — reserved for future use (SURF only in C++).
    /// * `page_no`         — page number to assign to all features.
    /// * `image_no`        — image number within the page.
    /// * `matcher`         — backend that will extract FREAK features.
    #[allow(clippy::too_many_arguments)]
    pub fn generate(
        image: &[u8],
        xsize: i32,
        ysize: i32,
        dpi: f32,
        proc_mode: KpmProcMode,
        comp_mode: KpmCompMode,
        _max_feature_num: i32,
        page_no: i32,
        image_no: i32,
        matcher: &mut dyn FreakMatcherBackend,
    ) -> Result<Self, KpmError> {
        if image.is_empty() || xsize <= 0 || ysize <= 0 || dpi == 0.0 {
            return Err(KpmError::InvalidInput(
                "invalid image/xsize/ysize/dpi".into(),
            ));
        }

        let xu = xsize as usize;
        let yu = ysize as usize;

        // 1. Resize image.
        let (mut resized, xsize2, ysize2) = resize_image(image, xu, yu, proc_mode);

        // 2. Optional vertical compression (CompY).
        if comp_mode == KpmCompMode::CompY {
            let half_h = ysize2 / 2;
            let mut compressed = Vec::with_capacity(xsize2 * half_h);
            for j in 0..half_h {
                for i in 0..xsize2 {
                    let a = resized[j * 2 * xsize2 + i] as u32;
                    let b = resized[(j * 2 + 1) * xsize2 + i] as u32;
                    compressed.push(((a + b) / 2) as u8);
                }
            }
            resized = compressed;
        }

        // 3. Extract FREAK features and descriptors (no database storage).
        //    Fix for issue #51: the previous code called add_image() then
        //    query_feature_points(), but query_feature_points() returns the
        //    query-side cache (populated only by query(), not add_image()).
        //    extract_features() returns the actual reference features with
        //    their full 96-byte descriptors.
        let (points, descriptors) = matcher.extract_features(&resized, xsize2, ysize2)?;
        let num = points.len();

        // 4. Compute scale factor and pixel-to-mm offset.
        //    Formula matches the original C++ kpmRefDataSet.cpp (kpmGenRefDataSet):
        //      coord3D.x = (x * scale + half_scale) / dpi * 25.4  [mm, top-left origin]
        //      coord3D.y = ((ysize - half_scale) - y * scale) / dpi * 25.4
        //    where x/y are in the resized image and ysize is the ORIGINAL height.
        let scale = proc_mode.scale_factor();
        let half_scale = scale / 2.0;

        // 5. Build ref_point array with real descriptors.
        let mut ref_point = Vec::with_capacity(num);
        for (i, pt) in points.iter().enumerate() {
            let mut y = pt.y;
            if comp_mode == KpmCompMode::CompY {
                y *= 2.0;
            }

            let coord3d_x = (pt.x * scale + half_scale) / dpi * MM_PER_INCH;
            let coord3d_y = ((ysize as f32 - half_scale) - pt.y * scale) / dpi * MM_PER_INCH;

            let desc_start = i * FREAK_SUB_DIMENSION;
            let mut v = [0u8; FREAK_SUB_DIMENSION];
            v.copy_from_slice(&descriptors[desc_start..desc_start + FREAK_SUB_DIMENSION]);

            ref_point.push(KpmRefData {
                coord2d: KpmCoord2D { x: pt.x, y },
                coord3d: KpmCoord2D {
                    x: coord3d_x,
                    y: coord3d_y,
                },
                feature_vec: FreakFeature {
                    v,
                    angle: pt.angle,
                    scale: pt.scale,
                    maxima: if pt.maxima { 1 } else { 0 },
                },
                page_no,
                ref_image_no: image_no,
            });
        }

        // 6. Build page info.
        let page_info = vec![KpmPageInfo {
            page_no,
            image_num: 1,
            image_info: vec![KpmImageInfo {
                image_no,
                width: xsize2 as i32,
                height: ysize2 as i32,
            }],
        }];

        Ok(KpmRefDataSet {
            num: num as i32,
            ref_point,
            page_num: 1,
            page_info,
        })
    }

    // ------------------------------------------------------------------
    // Binary I/O — little-endian, matching C++ layout exactly
    // ------------------------------------------------------------------

    /// Save this data set to a binary file.
    ///
    /// The format is identical to the C++ `kpmSaveRefDataSet` with
    /// `BINARY_FEATURE=1`, using native-width `i32` / `f32` in
    /// little-endian byte order.
    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        let file = std::fs::File::create(path)?;
        let mut w = std::io::BufWriter::new(file);

        // Number of ref points.
        w.write_i32::<LittleEndian>(self.num)?;

        for rp in &self.ref_point {
            // coord2D
            w.write_f32::<LittleEndian>(rp.coord2d.x)?;
            w.write_f32::<LittleEndian>(rp.coord2d.y)?;
            // coord3D
            w.write_f32::<LittleEndian>(rp.coord3d.x)?;
            w.write_f32::<LittleEndian>(rp.coord3d.y)?;
            // featureVec: 96 bytes + angle(f32) + scale(f32) + maxima(i32)
            w.write_all(&rp.feature_vec.v)?;
            w.write_f32::<LittleEndian>(rp.feature_vec.angle)?;
            w.write_f32::<LittleEndian>(rp.feature_vec.scale)?;
            w.write_i32::<LittleEndian>(rp.feature_vec.maxima)?;
            // pageNo, refImageNo
            w.write_i32::<LittleEndian>(rp.page_no)?;
            w.write_i32::<LittleEndian>(rp.ref_image_no)?;
        }

        // Page info.
        w.write_i32::<LittleEndian>(self.page_num)?;

        for pi in &self.page_info {
            w.write_i32::<LittleEndian>(pi.page_no)?;
            w.write_i32::<LittleEndian>(pi.image_num)?;
            for ii in &pi.image_info {
                // C++ KpmImageInfo layout: { int width; int height; int imageNo; }
                w.write_i32::<LittleEndian>(ii.width)?;
                w.write_i32::<LittleEndian>(ii.height)?;
                w.write_i32::<LittleEndian>(ii.image_no)?;
            }
        }

        w.flush()?;
        Ok(())
    }

    /// Load a data set from a binary file written by [`save`](Self::save).
    pub fn load(path: &Path) -> std::io::Result<Self> {
        let file = std::fs::File::open(path)?;
        Self::load_from_reader(std::io::BufReader::new(file))
    }

    /// Load a data set from in-memory bytes (e.g. a `.fset3` fetched in the
    /// browser, where there is no filesystem). Same format as [`load`](Self::load).
    pub fn load_from_bytes(bytes: &[u8]) -> std::io::Result<Self> {
        Self::load_from_reader(std::io::Cursor::new(bytes))
    }

    /// Parse a data set from any reader. Shared by [`load`](Self::load) and
    /// [`load_from_bytes`](Self::load_from_bytes).
    fn load_from_reader<R: std::io::Read>(mut r: R) -> std::io::Result<Self> {
        let num = r.read_i32::<LittleEndian>()?;
        if num <= 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "ref data set num <= 0",
            ));
        }

        let mut ref_point = Vec::with_capacity(num as usize);
        for _ in 0..num {
            let c2x = r.read_f32::<LittleEndian>()?;
            let c2y = r.read_f32::<LittleEndian>()?;
            let c3x = r.read_f32::<LittleEndian>()?;
            let c3y = r.read_f32::<LittleEndian>()?;

            let mut v = [0u8; FREAK_SUB_DIMENSION];
            r.read_exact(&mut v)?;
            let angle = r.read_f32::<LittleEndian>()?;
            let scale = r.read_f32::<LittleEndian>()?;
            let maxima = r.read_i32::<LittleEndian>()?;

            let page_no = r.read_i32::<LittleEndian>()?;
            let ref_image_no = r.read_i32::<LittleEndian>()?;

            ref_point.push(KpmRefData {
                coord2d: KpmCoord2D { x: c2x, y: c2y },
                coord3d: KpmCoord2D { x: c3x, y: c3y },
                feature_vec: FreakFeature {
                    v,
                    angle,
                    scale,
                    maxima,
                },
                page_no,
                ref_image_no,
            });
        }

        let page_num = r.read_i32::<LittleEndian>()?;
        if page_num <= 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "page_num <= 0",
            ));
        }

        let mut page_info = Vec::with_capacity(page_num as usize);
        for _ in 0..page_num {
            let page_no = r.read_i32::<LittleEndian>()?;
            let image_num = r.read_i32::<LittleEndian>()?;

            let mut image_info = Vec::with_capacity(image_num as usize);
            for _ in 0..image_num {
                // C++ KpmImageInfo layout: { int width; int height; int imageNo; }
                let width = r.read_i32::<LittleEndian>()?;
                let height = r.read_i32::<LittleEndian>()?;
                let image_no = r.read_i32::<LittleEndian>()?;
                image_info.push(KpmImageInfo {
                    image_no,
                    width,
                    height,
                });
            }

            page_info.push(KpmPageInfo {
                page_no,
                image_num,
                image_info,
            });
        }

        Ok(KpmRefDataSet {
            num,
            ref_point,
            page_num,
            page_info,
        })
    }

    // ------------------------------------------------------------------
    // Merge
    // ------------------------------------------------------------------

    /// Merge another data set into `self`.
    ///
    /// Ref-point arrays are concatenated. Page-info arrays are merged
    /// with deduplication by `page_no` — if both sets contain the same
    /// page number the image-info lists are combined.
    pub fn merge(&mut self, other: KpmRefDataSet) {
        // 1. Concatenate ref points.
        self.ref_point.extend(other.ref_point);
        self.num = self.ref_point.len() as i32;

        // 2. Merge page info with dedup by page_no.
        for other_page in other.page_info {
            if let Some(existing) = self
                .page_info
                .iter_mut()
                .find(|p| p.page_no == other_page.page_no)
            {
                // Same page_no — append image_info.
                existing.image_info.extend(other_page.image_info);
                existing.image_num = existing.image_info.len() as i32;
            } else {
                // New page — just push.
                self.page_info.push(other_page);
            }
        }
        self.page_num = self.page_info.len() as i32;
    }

    // ------------------------------------------------------------------
    // change_page_no
    // ------------------------------------------------------------------

    /// Rename all occurrences of `old_page_no` to `new_page_no`.
    ///
    /// If `old_page_no` is [`KPM_CHANGE_PAGE_NO_ALL_PAGES`] (`-1`),
    /// every page with a non-negative `page_no` is renamed.
    pub fn change_page_no(&mut self, old_page_no: i32, new_page_no: i32) {
        for rp in &mut self.ref_point {
            if rp.page_no == old_page_no
                || (old_page_no == KPM_CHANGE_PAGE_NO_ALL_PAGES && rp.page_no >= 0)
            {
                rp.page_no = new_page_no;
            }
        }
        for pi in &mut self.page_info {
            if pi.page_no == old_page_no
                || (old_page_no == KPM_CHANGE_PAGE_NO_ALL_PAGES && pi.page_no >= 0)
            {
                pi.page_no = new_page_no;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Legacy RefDataSet (simple image collection) — kept for compatibility
// ---------------------------------------------------------------------------

/// A simple growable collection of [`RefImage`] entries.
///
/// Used to stage reference images before they are submitted to the
/// backend for feature extraction.
pub struct RefDataSet {
    images: Vec<RefImage>,
}

impl RefDataSet {
    /// Creates an empty reference data set.
    pub fn new() -> Self {
        Self { images: Vec::new() }
    }

    /// Appends a reference image to the set.
    pub fn add(&mut self, image: RefImage) {
        self.images.push(image);
    }

    /// Returns the number of images in the set.
    pub fn len(&self) -> usize {
        self.images.len()
    }

    /// Returns `true` if the set contains no images.
    pub fn is_empty(&self) -> bool {
        self.images.is_empty()
    }

    /// Returns an iterator over the reference images.
    pub fn iter(&self) -> impl Iterator<Item = &RefImage> {
        self.images.iter()
    }
}

impl Default for RefDataSet {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a minimal KpmRefDataSet for testing.
    fn make_test_dataset(page_no: i32, num_points: usize) -> KpmRefDataSet {
        let mut ref_point = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let mut feat = FreakFeature::default();
            feat.v[0] = i as u8;
            feat.angle = i as f32 * 0.1;
            feat.scale = 1.0;
            feat.maxima = 1;

            ref_point.push(KpmRefData {
                coord2d: KpmCoord2D {
                    x: i as f32,
                    y: i as f32 * 2.0,
                },
                coord3d: KpmCoord2D {
                    x: i as f32 * 0.5,
                    y: i as f32 * 0.25,
                },
                feature_vec: feat,
                page_no,
                ref_image_no: 0,
            });
        }

        KpmRefDataSet {
            num: num_points as i32,
            ref_point,
            page_num: 1,
            page_info: vec![KpmPageInfo {
                page_no,
                image_num: 1,
                image_info: vec![KpmImageInfo {
                    image_no: 0,
                    width: 640,
                    height: 480,
                }],
            }],
        }
    }

    #[test]
    fn test_ref_data_roundtrip() {
        let ds = make_test_dataset(1, 2);

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.kpm");

        ds.save(&path).expect("save failed");
        let loaded = KpmRefDataSet::load(&path).expect("load failed");

        assert_eq!(loaded.num, ds.num);
        assert_eq!(loaded.page_num, ds.page_num);
        assert_eq!(loaded.ref_point.len(), ds.ref_point.len());

        // Verify first point round-trips exactly.
        assert_eq!(loaded.ref_point[0].coord2d, ds.ref_point[0].coord2d);
        assert_eq!(loaded.ref_point[0].coord3d, ds.ref_point[0].coord3d);
        assert_eq!(loaded.ref_point[0].feature_vec, ds.ref_point[0].feature_vec);
        assert_eq!(loaded.ref_point[0].page_no, ds.ref_point[0].page_no);

        // Verify page info round-trips.
        assert_eq!(loaded.page_info[0].page_no, ds.page_info[0].page_no);
        assert_eq!(
            loaded.page_info[0].image_info[0].width,
            ds.page_info[0].image_info[0].width
        );
    }

    #[test]
    fn test_ref_data_merge() {
        let mut ds1 = make_test_dataset(1, 3);
        let ds2 = make_test_dataset(2, 5);

        ds1.merge(ds2);

        assert_eq!(ds1.num, 8);
        assert_eq!(ds1.ref_point.len(), 8);
        assert_eq!(ds1.page_num, 2);
        assert_eq!(ds1.page_info.len(), 2);
    }

    #[test]
    fn test_ref_data_merge_same_page() {
        let mut ds1 = make_test_dataset(1, 2);
        let ds2 = make_test_dataset(1, 3);

        ds1.merge(ds2);

        // Points are concatenated.
        assert_eq!(ds1.num, 5);
        // Pages are deduplicated — still 1 page.
        assert_eq!(ds1.page_num, 1);
        // Image infos are combined.
        assert_eq!(ds1.page_info[0].image_num, 2);
    }

    #[test]
    fn test_change_page_no() {
        let mut ds = make_test_dataset(1, 4);

        ds.change_page_no(1, 5);

        for rp in &ds.ref_point {
            assert_eq!(rp.page_no, 5);
        }
        for pi in &ds.page_info {
            assert_eq!(pi.page_no, 5);
        }
    }

    #[test]
    fn test_change_page_no_all_pages() {
        let mut ds = make_test_dataset(1, 2);
        let ds2 = make_test_dataset(2, 2);
        ds.merge(ds2);

        ds.change_page_no(KPM_CHANGE_PAGE_NO_ALL_PAGES, 99);

        for rp in &ds.ref_point {
            assert_eq!(rp.page_no, 99);
        }
        for pi in &ds.page_info {
            assert_eq!(pi.page_no, 99);
        }
    }

    // NOTE: test_load_pinball_fset3 was removed during M3-1 consolidation.
    // The test data lives in crates/kpm/examples/data/ and CARGO_MANIFEST_DIR
    // now points to crates/core/. The test is covered by
    // crates/kpm/tests/regression.rs::test_fset3_load_structure.

    /// `load_from_bytes` (the wasm-friendly loader added for #161) must parse a
    /// real `.fset3` identically to the path-based `load`.
    #[cfg(not(target_arch = "wasm32"))]
    #[cfg_attr(miri, ignore)] // reads + parses a 625 KB .fset3 — too slow under Miri
    #[test]
    fn test_load_from_bytes_matches_load() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("examples")
            .join("Data")
            .join("pinball.fset3");
        let from_path = KpmRefDataSet::load(&path).expect("load(path) failed");
        let bytes = std::fs::read(&path).expect("read failed");
        let from_bytes = KpmRefDataSet::load_from_bytes(&bytes).expect("load_from_bytes failed");

        assert_eq!(from_path.num, from_bytes.num);
        assert_eq!(from_path.page_num, from_bytes.page_num);
        assert_eq!(from_path.ref_point.len(), from_bytes.ref_point.len());
        // Spot-check the first ref point round-trips identically.
        assert_eq!(
            from_path.ref_point[0].feature_vec.v,
            from_bytes.ref_point[0].feature_vec.v
        );
    }
}
