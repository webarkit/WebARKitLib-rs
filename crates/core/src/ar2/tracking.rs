/*
 *  ar2.rs
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

//! AR2 Robust Tracking Module
//! Ported and idiomatic Rust implementation of ARToolKit's AR2 NFT tracking logic.
//!
//! Focus: 2D region tracking and feature matching for robust pose estimation.

use crate::ar2::feature_set::AR2FeatureSetT;
use crate::ar2::image_set::AR2ImageSetT;
use crate::icp::ICPHandleT;
use crate::types::{ARParamLT, ARPixelFormat, ARdouble};
use std::path::Path;

pub const AR2_TRACKING_6DOF: i32 = 1;
pub const AR2_TRACKING_HOMOGRAPHY: i32 = 2;

pub const AR2_SEARCH_FEATURE_MAX: usize = 500;
pub const AR2_TRACKING_CANDIDATE_MAX: usize = 1000;
pub const AR2_TRACKING_SURFACE_MAX: usize = 10;
pub const AR2_THREAD_MAX: usize = 8; // Adjust as needed for Rust safety/concurrency
pub const AR2_BLUR_IMAGE_MAX: usize = 5;
pub const AR2_TEMP_SCALE: i32 = 1;
pub const AR2_TEMPLATE_NULL_PIXEL: u16 = 0xFFFF;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AR2TemplateCandidate {
    pub snum: i32,
    pub level: i32,
    pub num: i32,
    pub flag: i32, // -1: end of list, 0: valid, 1: selected
    pub sx: f32,
    pub sy: f32,
}

impl Default for AR2TemplateCandidate {
    fn default() -> Self {
        Self {
            snum: 0,
            level: 0,
            num: 0,
            flag: -1,
            sx: 0.0,
            sy: 0.0,
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct AR2Image {
    pub img_bw_blur: Vec<Option<Vec<u8>>>,
    pub xsize: i32,
    pub ysize: i32,
    pub dpi: f32,
}

#[derive(Debug, Default, Clone)]
pub struct AR2ImageSet {
    pub scale: Vec<AR2Image>,
}

#[derive(Debug, Default, Clone)]
pub struct AR2FeatureCoord {
    pub x: i32,
    pub y: i32,
    pub mx: f32,
    pub my: f32,
    pub max_sim: f32,
}

#[derive(Debug, Default, Clone)]
pub struct AR2FeaturePoints {
    pub coord: Vec<AR2FeatureCoord>,
    pub scale: i32,
    pub maxdpi: f32,
    pub mindpi: f32,
}

#[derive(Debug, Default, Clone)]
pub struct AR2FeatureSet {
    pub list: Vec<AR2FeaturePoints>,
}

#[derive(Debug, Default, Clone)]
pub struct AR2Template {
    pub xts1: i32,
    pub xts2: i32,
    pub yts1: i32,
    pub yts2: i32,
    pub xsize: i32,
    pub ysize: i32,
    pub img1: Vec<u16>,
    pub vlen: i32,
    pub sum: i32,
    pub valid_num: usize,
}

// MarkerSet is ignored for now per user rules (no multi-marker / complex markers yet)
// but we might need a placeholder if it's tightly coupled.

#[derive(Debug, Default, Clone)]
pub struct AR2Surface {
    pub image_set: Option<AR2ImageSet>,
    pub feature_set: Option<AR2FeatureSet>,
    pub trans: [[f32; 4]; 3],
    pub itrans: [[f32; 4]; 3],
}

#[derive(Debug, Clone)]
pub struct AR2SurfaceSet {
    pub surface: Vec<AR2Surface>,
    pub trans1: [[f32; 4]; 3],
    pub trans2: [[f32; 4]; 3],
    pub trans3: [[f32; 4]; 3],
    pub cont_num: i32,
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
    /// Rust equivalent of `ar2SetInitTrans()` in the C API.
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
pub fn ar2_read_surface_set<P: AsRef<Path>>(
    basename: P,
) -> std::io::Result<AR2SurfaceSet> {
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

pub struct AR2Handle {
    pub tracking_mode: i32,
    pub xsize: i32,
    pub ysize: i32,
    pub cparam_lt: *mut ARParamLT, // Pointer for now consistent with C API, or Box?
    pub icp_handle: *mut ICPHandleT,
    pub pix_format: ARPixelFormat,

    pub blur_method: i32,
    pub blur_level: i32,
    pub search_size: i32,
    pub template_size1: i32,
    pub template_size2: i32,
    pub search_feature_num: i32,
    pub sim_thresh: f32,
    pub tracking_thresh: f32,

    pub wtrans1: Vec<[[f32; 4]; 3]>,
    pub wtrans2: Vec<[[f32; 4]; 3]>,
    pub wtrans3: Vec<[[f32; 4]; 3]>,

    pub pos: Vec<[f32; 2]>,
    pub pos2d: Vec<[f32; 2]>,
    pub pos3d: Vec<[f32; 3]>,

    pub candidate: Vec<AR2TemplateCandidate>,
    pub candidate2: Vec<AR2TemplateCandidate>,
    pub used_feature: Vec<AR2TemplateCandidate>,

    pub templ: Vec<AR2Template>, // For sequential tracking
    pub mf_image: Vec<u8>,

    pub thread_num: i32,
}

impl AR2Handle {
    pub fn new(xsize: i32, ysize: i32, pix_format: ARPixelFormat) -> Self {
        Self {
            tracking_mode: AR2_TRACKING_6DOF,
            xsize,
            ysize,
            cparam_lt: std::ptr::null_mut(),
            icp_handle: std::ptr::null_mut(),
            pix_format,
            blur_method: 0,
            blur_level: 1,
            search_size: 6,
            template_size1: 16,
            template_size2: 16,
            search_feature_num: 10,
            sim_thresh: 0.5,
            tracking_thresh: 2.0,

            wtrans1: vec![[[0.0; 4]; 3]; AR2_TRACKING_SURFACE_MAX],
            wtrans2: vec![[[0.0; 4]; 3]; AR2_TRACKING_SURFACE_MAX],
            wtrans3: vec![[[0.0; 4]; 3]; AR2_TRACKING_SURFACE_MAX],

            pos: vec![[0.0; 2]; AR2_SEARCH_FEATURE_MAX + AR2_THREAD_MAX],
            pos2d: vec![[0.0; 2]; AR2_SEARCH_FEATURE_MAX],
            pos3d: vec![[0.0; 3]; AR2_SEARCH_FEATURE_MAX],

            candidate: vec![AR2TemplateCandidate::default(); AR2_TRACKING_CANDIDATE_MAX + 1],
            candidate2: vec![AR2TemplateCandidate::default(); AR2_TRACKING_CANDIDATE_MAX + 1],
            used_feature: vec![AR2TemplateCandidate::default(); AR2_SEARCH_FEATURE_MAX],

            templ: vec![
                AR2Template {
                    xts1: 16,
                    xts2: 16,
                    yts1: 16,
                    yts2: 16,
                    xsize: 33,
                    ysize: 33,
                    ..Default::default()
                };
                AR2_THREAD_MAX
            ],
            mf_image: vec![0; (xsize * ysize) as usize],

            thread_num: 1,
        }
    }
}

pub fn marker_coord_to_screen_coord(
    cparam_lt: Option<&ARParamLT>,
    trans: &[[f32; 4]; 3],
    mx: f32,
    my: f32,
) -> Result<(f32, f32), &'static str> {
    let (ix, iy) = if let Some(lt) = cparam_lt {
        let mut wtrans = [[0.0f32; 4]; 3];
        crate::math::mat_mul_dff(&lt.param.mat, trans, &mut wtrans);

        let hx = wtrans[0][0] * mx + wtrans[0][1] * my + wtrans[0][3];
        let hy = wtrans[1][0] * mx + wtrans[1][1] * my + wtrans[1][3];
        let h = wtrans[2][0] * mx + wtrans[2][1] * my + wtrans[2][3];
        (hx / h, hy / h)
    } else {
        let hx = trans[0][0] * mx + trans[0][1] * my + trans[0][3];
        let hy = trans[1][0] * mx + trans[1][1] * my + trans[1][3];
        let h = trans[2][0] * mx + trans[2][1] * my + trans[2][3];
        return Ok((hx / h, hy / h));
    };

    if let Some(lt) = cparam_lt {
        let (sx, sy) = lt.param_ltf.ideal2observ(ix, iy)?;
        Ok((sx, sy))
    } else {
        Ok((ix, iy))
    }
}

pub fn marker_coord_to_screen_coord2(
    cparam_lt: Option<&ARParamLT>,
    trans: &[[f32; 4]; 3],
    mx: f32,
    my: f32,
) -> Result<(f32, f32), &'static str> {
    let (sx, sy) = marker_coord_to_screen_coord(cparam_lt, trans, mx, my)?;

    if let Some(lt) = cparam_lt {
        let (ix1, iy1) = lt.param_ltf.observ2ideal(sx, sy)?;
        // Original C code does it for verification but we might just return it
        // if( (ix-ix1)*(ix-ix1) + (iy-iy1)*(iy-iy1) > 1.0F ) return -1;
        // In Rust we trust the LT for now or re-project?
        // Let's stick to the C logic if ix, iy are needed
        let (ix, iy) = {
            let mut wtrans = [[0.0f32; 4]; 3];
            crate::math::mat_mul_dff(&lt.param.mat, trans, &mut wtrans);
            let hx = wtrans[0][0] * mx + wtrans[0][1] * my + wtrans[0][3];
            let hy = wtrans[1][0] * mx + wtrans[1][1] * my + wtrans[1][3];
            let h = wtrans[2][0] * mx + wtrans[2][1] * my + wtrans[2][3];
            (hx / h, hy / h)
        };
        if (ix - ix1).powi(2) + (iy - iy1).powi(2) > 1.0 {
            return Err("Reprojection error too high");
        }
    }

    Ok((sx, sy))
}

pub fn ar2_screen_coord_2_marker_coord(
    cparam_lt: Option<&ARParamLT>,
    trans: &[[f32; 4]; 3],
    sx: f32,
    sy: f32,
) -> Result<(f32, f32), &'static str> {
    if let Some(lt) = cparam_lt {
        let (ix, iy) = lt.param_ltf.observ2ideal(sx, sy)?;
        let mut wtrans = [[0.0f32; 4]; 3];
        crate::math::mat_mul_dff(&lt.param.mat, trans, &mut wtrans);

        let c11 = wtrans[2][0] * ix - wtrans[0][0];
        let c12 = wtrans[2][1] * ix - wtrans[0][1];
        let c21 = wtrans[2][0] * iy - wtrans[1][0];
        let c22 = wtrans[2][1] * iy - wtrans[1][1];
        let b1 = wtrans[0][3] - wtrans[2][3] * ix;
        let b2 = wtrans[1][3] - wtrans[2][3] * iy;

        let m = c11 * c22 - c12 * c21;
        if m.abs() < 1e-10 {
            return Err("Singular matrix in coord conversion");
        }

        let mx = (b1 * c22 - b2 * c12) / m;
        let my = (c11 * b2 - c21 * b1) / m;
        Ok((mx, my))
    } else {
        let c11 = trans[2][0] * sx - trans[0][0];
        let c12 = trans[2][1] * sx - trans[0][1];
        let c21 = trans[2][0] * sy - trans[1][0];
        let c22 = trans[2][1] * sy - trans[1][1];
        let b1 = trans[0][3] - trans[2][3] * sx;
        let b2 = trans[1][3] - trans[2][3] * sy;

        let m = c11 * c22 - c12 * c21;
        if m.abs() < 1e-10 {
            return Err("Singular matrix in coord conversion");
        }

        let mx = (b1 * c22 - b2 * c12) / m;
        let my = (c11 * b2 - c21 * b1) / m;
        Ok((mx, my))
    }
}

pub fn ar2_marker_coord_2_image_coord(
    _xsize: i32,
    ysize: i32,
    dpi: f32,
    mx: f32,
    my: f32,
) -> (f32, f32) {
    let iix = mx * dpi / 25.4;
    let iiy = (ysize as f32) - my * dpi / 25.4;
    (iix, iiy)
}

pub fn ar2_get_image_value(
    cparam_lt: Option<&ARParamLT>,
    trans: &[[f32; 4]; 3],
    image: &AR2Image,
    sx: f32,
    sy: f32,
    blur_level: i32,
) -> Result<u8, &'static str> {
    let (mx, my) = ar2_screen_coord_2_marker_coord(cparam_lt, trans, sx, sy)?;
    let (iix, iiy) = ar2_marker_coord_2_image_coord(image.xsize, image.ysize, image.dpi, mx, my);

    let ix = (iix + 0.5) as i32;
    let iy = (iiy + 0.5) as i32;

    if ix < 0 || ix >= image.xsize || iy < 0 || iy >= image.ysize {
        return Err("Coord out of image bounds");
    }

    if let Some(blur_vec) = &image.img_bw_blur[blur_level as usize] {
        Ok(blur_vec[(iy * image.xsize + ix) as usize])
    } else {
        Err("Blur level not available")
    }
}

pub fn ar2_set_template_sub(
    cparam_lt: Option<&ARParamLT>,
    trans: &[[f32; 4]; 3],
    image_set: &AR2ImageSet,
    feature_points: &AR2FeaturePoints,
    num: usize,
    blur_level: i32,
    templ: &mut AR2Template,
) -> Result<(), &'static str> {
    let mx = feature_points.coord[num].mx;
    let my = feature_points.coord[num].my;

    let (sx, sy) = marker_coord_to_screen_coord(cparam_lt, trans, mx, my)?;
    let ix = (sx + 0.5) as i32;
    let iy = (sy + 0.5) as i32;

    let mut sum = 0;
    let mut sum2 = 0;
    let mut k = 0;

    templ.img1.clear();

    let image = &image_set.scale[feature_points.scale as usize];

    for j in -templ.yts1..=templ.yts2 {
        let iy2 = iy + j * AR2_TEMP_SCALE;
        for i in -templ.xts1..=templ.xts2 {
            let ix2 = ix + i * AR2_TEMP_SCALE;

            let res = if let Some(lt) = cparam_lt {
                match lt.param_ltf.observ2ideal(ix2 as f32, iy2 as f32) {
                    Ok((isx, isy)) => {
                        ar2_get_image_value(cparam_lt, trans, image, isx, isy, blur_level)
                    }
                    Err(_) => Err("Ideal error"),
                }
            } else {
                ar2_get_image_value(cparam_lt, trans, image, ix2 as f32, iy2 as f32, blur_level)
            };

            match res {
                Ok(pixel) => {
                    templ.img1.push(pixel as u16);
                    sum += pixel as i32;
                    sum2 += (pixel as i32) * (pixel as i32);
                    k += 1;
                }
                Err(_) => {
                    templ.img1.push(AR2_TEMPLATE_NULL_PIXEL);
                }
            }
        }
    }

    if k == 0 {
        return Err("No valid pixels in template");
    }

    let vlen = (sum2 - sum * sum / (k as i32)) as f32;
    templ.vlen = vlen.sqrt() as i32;
    templ.sum = sum;
    templ.valid_num = k;

    Ok(())
}

pub fn get_resolution(
    cparam_lt: Option<&ARParamLT>,
    trans: &[[f32; 4]; 3],
    pos: [f32; 2],
) -> Result<[f32; 2], &'static str> {
    let cparam = cparam_lt.map(|lt| &lt.param);
    get_resolution2(cparam, trans, pos)
}

pub fn get_resolution2(
    cparam: Option<&crate::types::ARParam>,
    trans: &[[f32; 4]; 3],
    pos: [f32; 2],
) -> Result<[f32; 2], &'static str> {
    let mut mat = [[0.0f32; 4]; 3];
    let (x0, y0, x1, y1, x2, y2);

    if let Some(cp) = cparam {
        crate::math::mat_mul_dff(&cp.mat, trans, &mut mat);

        let project = |px, py| {
            let hx = mat[0][0] * px + mat[0][1] * py + mat[0][3];
            let hy = mat[1][0] * px + mat[1][1] * py + mat[1][3];
            let h = mat[2][0] * px + mat[2][1] * py + mat[2][3];
            (hx / h, hy / h)
        };

        let (p0x, p0y) = project(pos[0], pos[1]);
        x0 = p0x;
        y0 = p0y;
        let (p1x, p1y) = project(pos[0] + 10.0, pos[1]);
        x1 = p1x;
        y1 = p1y;
        let (p2x, p2y) = project(pos[0], pos[1] + 10.0);
        x2 = p2x;
        y2 = p2y;
    } else {
        let project = |px, py| {
            let hx = trans[0][0] * px + trans[0][1] * py + trans[0][3];
            let hy = trans[1][0] * px + trans[1][1] * py + trans[1][3];
            let h = trans[2][0] * px + trans[2][1] * py + trans[2][3];
            (hx / h, hy / h)
        };

        let (p0x, p0y) = project(pos[0], pos[1]);
        x0 = p0x;
        y0 = p0y;
        let (p1x, p1y) = project(pos[0] + 10.0, pos[1]);
        x1 = p1x;
        y1 = p1y;
        let (p2x, p2y) = project(pos[0], pos[1] + 10.0);
        x2 = p2x;
        y2 = p2y;
    }

    let d1 = (x1 - x0).powi(2) + (y1 - y0).powi(2);
    let d2 = (x2 - x0).powi(2) + (y2 - y0).powi(2);

    let mut dpi = [0.0; 2];
    if d1 < d2 {
        dpi[0] = d2.sqrt() * 2.54;
        dpi[1] = d1.sqrt() * 2.54;
    } else {
        dpi[0] = d1.sqrt() * 2.54;
        dpi[1] = d2.sqrt() * 2.54;
    }

    Ok(dpi)
}

pub fn extract_visible_features(
    cparam_lt: &ARParamLT,
    wtrans1: &[[[f32; 4]; 3]],
    surface_set: &AR2SurfaceSet,
    candidate: &mut [AR2TemplateCandidate],
    candidate2: &mut [AR2TemplateCandidate],
) -> Result<(), &'static str> {
    let xsize = cparam_lt.param.xsize;
    let ysize = cparam_lt.param.ysize;

    let mut l = 0;
    let mut l2 = 0;

    for i in 0..surface_set.surface.len() {
        let surface = &surface_set.surface[i];
        let feature_set = match &surface.feature_set {
            Some(fs) => fs,
            None => continue,
        };

        let trans2 = &wtrans1[i];

        for j in 0..feature_set.list.len() {
            let feature_points = &feature_set.list[j];
            for k in 0..feature_points.coord.len() {
                let coord = &feature_points.coord[k];

                let (sx, sy) = match marker_coord_to_screen_coord2(
                    Some(cparam_lt),
                    trans2,
                    coord.mx,
                    coord.my,
                ) {
                    Ok(res) => res,
                    Err(_) => continue,
                };

                if sx < 0.0 || sx >= xsize as f32 || sy < 0.0 || sy >= ysize as f32 {
                    continue;
                }

                // Normal vector check (backface culling)
                let vdir0 = trans2[0][0] * coord.mx + trans2[0][1] * coord.my + trans2[0][3];
                let vdir1 = trans2[1][0] * coord.mx + trans2[1][1] * coord.my + trans2[1][3];
                let vdir2 = trans2[2][0] * coord.mx + trans2[2][1] * coord.my + trans2[2][3];
                let vlen = (vdir0 * vdir0 + vdir1 * vdir1 + vdir2 * vdir2).sqrt();
                let vd0 = vdir0 / vlen;
                let vd1 = vdir1 / vlen;
                let vd2 = vdir2 / vlen;

                if vd0 * trans2[0][2] + vd1 * trans2[1][2] + vd2 * trans2[2][2] > -0.1 {
                    continue;
                }

                let w = get_resolution(Some(cparam_lt), trans2, [coord.mx, coord.my])?;

                if w[1] <= feature_points.maxdpi && w[1] >= feature_points.mindpi {
                    if l >= AR2_TRACKING_CANDIDATE_MAX {
                        candidate[l].flag = -1;
                        return Err("Feature candidates overflow");
                    }
                    candidate[l] = AR2TemplateCandidate {
                        snum: i as i32,
                        level: j as i32,
                        num: k as i32,
                        sx,
                        sy,
                        flag: 0,
                    };
                    l += 1;
                } else if w[1] <= feature_points.maxdpi * 2.0 && w[1] >= feature_points.mindpi / 2.0
                {
                    if l2 < AR2_TRACKING_CANDIDATE_MAX {
                        candidate2[l2] = AR2TemplateCandidate {
                            snum: i as i32,
                            level: j as i32,
                            num: k as i32,
                            sx,
                            sy,
                            flag: 0,
                        };
                        l2 += 1;
                    }
                }
            }
        }
    }
    candidate[l].flag = -1;
    candidate2[l2].flag = -1;

    Ok(())
}

pub fn extract_visible_features_homography(
    xsize: i32,
    ysize: i32,
    wtrans1: &[[[f32; 4]; 3]],
    surface_set: &AR2SurfaceSet,
    candidate: &mut [AR2TemplateCandidate],
    candidate2: &mut [AR2TemplateCandidate],
) -> Result<(), &'static str> {
    let mut l = 0;
    let mut l2 = 0;

    for i in 0..surface_set.surface.len() {
        let surface = &surface_set.surface[i];
        let feature_set = match &surface.feature_set {
            Some(fs) => fs,
            None => continue,
        };

        let trans2 = &wtrans1[i];

        for j in 0..feature_set.list.len() {
            let feature_points = &feature_set.list[j];
            for k in 0..feature_points.coord.len() {
                let coord = &feature_points.coord[k];

                let (sx, sy) = match marker_coord_to_screen_coord2(None, trans2, coord.mx, coord.my)
                {
                    Ok(res) => res,
                    Err(_) => continue,
                };

                if sx < 0.0 || sx >= xsize as f32 || sy < 0.0 || sy >= ysize as f32 {
                    continue;
                }

                let w = get_resolution(None, trans2, [coord.mx, coord.my])?;

                if w[1] <= feature_points.maxdpi && w[1] >= feature_points.mindpi {
                    if l >= AR2_TRACKING_CANDIDATE_MAX {
                        candidate[l].flag = -1;
                        return Err("Feature candidates overflow");
                    }
                    candidate[l] = AR2TemplateCandidate {
                        snum: i as i32,
                        level: j as i32,
                        num: k as i32,
                        sx,
                        sy,
                        flag: 0,
                    };
                    l += 1;
                } else if w[1] <= feature_points.maxdpi * 2.0 && w[1] >= feature_points.mindpi / 2.0
                {
                    if l2 < AR2_TRACKING_CANDIDATE_MAX {
                        candidate2[l2] = AR2TemplateCandidate {
                            snum: i as i32,
                            level: j as i32,
                            num: k as i32,
                            sx,
                            sy,
                            flag: 0,
                        };
                        l2 += 1;
                    }
                }
            }
        }
    }
    candidate[l].flag = -1;
    candidate2[l2].flag = -1;

    Ok(())
}

pub fn ar2_select_template(
    candidate: &mut [AR2TemplateCandidate],
    prev_feature: &mut [AR2TemplateCandidate],
    num: i32,
    pos: &mut [[f32; 2]; 4],
    xsize: i32,
    ysize: i32,
) -> i32 {
    if num < 0 {
        return -1;
    }

    if num == 0 {
        let mut dmax = 0.0;
        let mut j = -1;
        for (i, cand) in candidate.iter().enumerate() {
            if cand.flag == -1 {
                break;
            }
            if cand.flag != 0 {
                continue;
            }
            if cand.sx < (xsize / 8) as f32
                || cand.sx > (xsize * 7 / 8) as f32
                || cand.sy < (ysize / 8) as f32
                || cand.sy > (ysize * 7 / 8) as f32
            {
                continue;
            }

            let d = (cand.sx - (xsize / 2) as f32).powi(2) + (cand.sy - (ysize / 2) as f32).powi(2);
            if d > dmax {
                dmax = d;
                j = i as i32;
            }
        }

        if j != -1 {
            candidate[j as usize].flag = 1;
        }
        return j;
    } else if num == 1 {
        let mut dmax = 0.0;
        let mut j = -1;
        for (i, cand) in candidate.iter().enumerate() {
            if cand.flag == -1 {
                break;
            }
            if cand.flag != 0 {
                continue;
            }
            if cand.sx < (xsize / 8) as f32
                || cand.sx > (xsize * 7 / 8) as f32
                || cand.sy < (ysize / 8) as f32
                || cand.sy > (ysize * 7 / 8) as f32
            {
                continue;
            }

            let d = (cand.sx - pos[0][0]).powi(2) + (cand.sy - pos[0][1]).powi(2);
            if d > dmax {
                dmax = d;
                j = i as i32;
            }
        }

        if j != -1 {
            candidate[j as usize].flag = 1;
        }
        return j;
    } else if num == 2 {
        let mut dmax = 0.0;
        let mut j = -1;
        for (i, cand) in candidate.iter().enumerate() {
            if cand.flag == -1 {
                break;
            }
            if cand.flag != 0 {
                continue;
            }
            if cand.sx < (xsize / 8) as f32
                || cand.sx > (xsize * 7 / 8) as f32
                || cand.sy < (ysize / 8) as f32
                || cand.sy > (ysize * 7 / 8) as f32
            {
                continue;
            }

            let d = (cand.sx - pos[0][0]) * (pos[1][1] - pos[0][1])
                - (cand.sy - pos[0][1]) * (pos[1][0] - pos[0][0]);
            let d_sq = d * d;
            if d_sq > dmax {
                dmax = d_sq;
                j = i as i32;
            }
        }

        if j != -1 {
            candidate[j as usize].flag = 1;
        }
        return j;
    } else if num == 3 {
        let (mut p2sinf, mut p2cosf) = (0.0, 0.0);
        let (mut p3sinf, mut p3cosf) = (0.0, 0.0);
        let (mut p4sinf, mut p4cosf) = (0.0, 0.0);

        if get_vector_angle(pos[0], pos[1], &mut p2sinf, &mut p2cosf).is_err() {
            return -1;
        }
        if get_vector_angle(pos[0], pos[2], &mut p3sinf, &mut p3cosf).is_err() {
            return -1;
        }

        let mut j = -1;
        let mut smax = 0.0;
        for (i, cand) in candidate.iter().enumerate() {
            if cand.flag == -1 {
                break;
            }
            if cand.flag != 0 {
                continue;
            }
            if cand.sx < (xsize / 8) as f32
                || cand.sx > (xsize * 7 / 8) as f32
                || cand.sy < (ysize / 8) as f32
                || cand.sy > (ysize * 7 / 8) as f32
            {
                continue;
            }

            pos[3][0] = cand.sx;
            pos[3][1] = cand.sy;
            if get_vector_angle(pos[0], pos[3], &mut p4sinf, &mut p4cosf).is_err() {
                continue;
            }

            let q1: usize;
            let r1: usize;
            let r2: usize;

            if (p3sinf * p2cosf - p3cosf * p2sinf >= 0.0)
                && (p4sinf * p2cosf - p4cosf * p2sinf >= 0.0)
            {
                if p4sinf * p3cosf - p4cosf * p3sinf >= 0.0 {
                    q1 = 1;
                    r1 = 2;
                    r2 = 3;
                } else {
                    q1 = 1;
                    r1 = 3;
                    r2 = 2;
                }
            } else if (p4sinf * p3cosf - p4cosf * p3sinf >= 0.0)
                && (p2sinf * p3cosf - p2cosf * p3sinf >= 0.0)
            {
                if p4sinf * p2cosf - p4cosf * p2sinf >= 0.0 {
                    q1 = 2;
                    r1 = 1;
                    r2 = 3;
                } else {
                    q1 = 2;
                    r1 = 3;
                    r2 = 1;
                }
            } else if (p2sinf * p4cosf - p2cosf * p4sinf >= 0.0)
                && (p3sinf * p4cosf - p3cosf * p4sinf >= 0.0)
            {
                if p3sinf * p2cosf - p3cosf * p2sinf >= 0.0 {
                    q1 = 3;
                    r1 = 1;
                    r2 = 2;
                } else {
                    q1 = 3;
                    r1 = 2;
                    r2 = 1;
                }
            } else {
                continue;
            }

            let s = get_region_area(pos, q1, r1, r2);
            if s > smax {
                smax = s;
                j = i as i32;
            }
        }

        if j != -1 {
            candidate[j as usize].flag = 1;
        }
        return j;
    } else {
        // Find previous feature if still valid
        for p_cand in prev_feature.iter_mut() {
            if p_cand.flag == -1 {
                break;
            }
            if p_cand.flag != 0 {
                continue;
            }
            p_cand.flag = 1;

            for (i, cand) in candidate.iter_mut().enumerate() {
                if cand.flag == -1 {
                    break;
                }
                if cand.flag == 0
                    && p_cand.snum == cand.snum
                    && p_cand.level == cand.level
                    && p_cand.num == cand.num
                {
                    cand.flag = 1;
                    return i as i32;
                }
            }
        }

        // Random fallback if no previous feature found
        let mut available = Vec::new();
        for (i, cand) in candidate.iter().enumerate() {
            if cand.flag == -1 {
                break;
            }
            if cand.flag == 0 {
                available.push(i);
            }
        }

        if available.is_empty() {
            return -1;
        }

        use rand::Rng;
        let mut rng = rand::thread_rng();
        let idx = available[rng.gen_range(0..available.len())];
        candidate[idx].flag = 1;
        idx as i32
    }
}

pub fn get_vector_angle(
    p1: [f32; 2],
    p2: [f32; 2],
    psinf: &mut f32,
    pcosf: &mut f32,
) -> Result<(), &'static str> {
    let l = ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2)).sqrt();
    if l == 0.0 {
        return Err("Zero length vector");
    }
    *psinf = (p2[1] - p1[1]) / l;
    *pcosf = (p2[0] - p1[0]) / l;
    Ok(())
}

pub fn get_region_area(pos: &[[f32; 2]; 4], q1: usize, r1: usize, r2: usize) -> f32 {
    get_triangle_area(pos[0], pos[q1], pos[r1]) + get_triangle_area(pos[0], pos[r1], pos[r2])
}

pub fn get_triangle_area(p1: [f32; 2], p2: [f32; 2], p3: [f32; 2]) -> f32 {
    let x1 = p2[0] - p1[0];
    let y1 = p2[1] - p1[1];
    let x2 = p3[0] - p1[0];
    let y2 = p3[1] - p1[1];
    ((x1 * y2 - x2 * y1) / 2.0).abs()
}

pub fn ar2_get_search_point(
    cparam_lt: Option<&ARParamLT>,
    trans1: Option<&[[f32; 4]; 3]>,
    trans2: Option<&[[f32; 4]; 3]>,
    trans3: Option<&[[f32; 4]; 3]>,
    feature: &AR2FeatureCoord,
    search: &mut [[i32; 2]; 3],
) {
    let mx = feature.mx;
    let my = feature.my;

    let ox1 = if let Some(t1) = trans1 {
        match marker_coord_to_screen_coord(cparam_lt, t1, mx, my) {
            Ok((x, y)) => {
                search[0][0] = x as i32;
                search[0][1] = y as i32;
                Some((x, y))
            }
            Err(_) => None,
        }
    } else {
        None
    };

    if ox1.is_none() {
        search[0][0] = -1;
        search[0][1] = -1;
        search[1][0] = -1;
        search[1][1] = -1;
        search[2][0] = -1;
        search[2][1] = -1;
        return;
    }
    let (ox1x, ox1y) = ox1.unwrap();

    let ox2 = if let Some(t2) = trans2 {
        match marker_coord_to_screen_coord(cparam_lt, t2, mx, my) {
            Ok((x, y)) => {
                search[1][0] = (2.0 * ox1x - x) as i32;
                search[1][1] = (2.0 * ox1y - y) as i32;
                Some((x, y))
            }
            Err(_) => None,
        }
    } else {
        None
    };

    if ox2.is_none() {
        search[1][0] = -1;
        search[1][1] = -1;
        search[2][0] = -1;
        search[2][1] = -1;
        return;
    }
    let (ox2x, ox2y) = ox2.unwrap();

    if let Some(t3) = trans3 {
        match marker_coord_to_screen_coord(cparam_lt, t3, mx, my) {
            Ok((x, y)) => {
                search[2][0] = (3.0 * ox1x - 3.0 * ox2x + x) as i32;
                search[2][1] = (3.0 * ox1y - 3.0 * ox2y + y) as i32;
            }
            Err(_) => {
                search[2][0] = -1;
                search[2][1] = -1;
            }
        }
    } else {
        search[2][0] = -1;
        search[2][1] = -1;
    }
}

pub struct AR2Tracking2DResult {
    pub sim: f32,
    pub pos2d: [f32; 2],
    pub pos3d: [f32; 3],
}

pub fn ar2_tracking_2d_sub(
    cparam_lt: Option<&ARParamLT>,
    pix_format: ARPixelFormat,
    xsize: i32,
    ysize: i32,
    blur_level: i32,
    search_size: i32,
    wtrans1: &[[f32; 4]; 3],
    wtrans2: Option<&[[f32; 4]; 3]>,
    wtrans3: Option<&[[f32; 4]; 3]>,
    surface_set: &AR2SurfaceSet,
    candidate: &AR2TemplateCandidate,
    data_ptr: &[u8],
    mf_image: &mut [u8],
    templ: &mut AR2Template,
) -> Result<AR2Tracking2DResult, &'static str> {
    let snum = candidate.snum as usize;
    let level = candidate.level as usize;
    let fnum = candidate.num as usize;

    let surface = &surface_set.surface[snum];
    let feature_set = surface.feature_set.as_ref().ok_or("No feature set")?;
    let feature_points = &feature_set.list[level];

    ar2_set_template_sub(
        cparam_lt,
        wtrans1,
        &surface.image_set.as_ref().ok_or("No image set")?.clone(),
        feature_points,
        fnum,
        blur_level,
        templ,
    )?;

    if ((templ.vlen * templ.vlen) as f32) < ((templ.xsize * templ.ysize) as f32 * 2.0 * 2.0) {
        // AR2_DEFAULT_TRACKING_SD_THRESH placeholder
        return Err("Low variance template");
    }

    let mut search = [[0i32; 2]; 3];
    let feature_coord = &feature_points.coord[fnum];

    if surface_set.cont_num == 1 {
        ar2_get_search_point(
            cparam_lt,
            Some(wtrans1),
            None,
            None,
            feature_coord,
            &mut search,
        );
    } else if surface_set.cont_num == 2 {
        ar2_get_search_point(
            cparam_lt,
            Some(wtrans1),
            wtrans2,
            None,
            feature_coord,
            &mut search,
        );
    } else {
        ar2_get_search_point(
            cparam_lt,
            Some(wtrans1),
            wtrans2,
            wtrans3,
            feature_coord,
            &mut search,
        );
    }

    let mut bx = 0;
    let mut by = 0;
    let mut sim = 0.0;

    ar2_get_best_matching(
        data_ptr,
        mf_image,
        xsize,
        ysize,
        pix_format,
        templ,
        search_size,
        search_size,
        search,
        &mut bx,
        &mut by,
        &mut sim,
    )?;

    let mut result = AR2Tracking2DResult {
        sim,
        pos2d: [bx as f32, by as f32],
        pos3d: [0.0; 3],
    };

    result.pos3d[0] = surface.trans[0][0] * feature_coord.mx
        + surface.trans[0][1] * feature_coord.my
        + surface.trans[0][3];
    result.pos3d[1] = surface.trans[1][0] * feature_coord.mx
        + surface.trans[1][1] * feature_coord.my
        + surface.trans[1][3];
    result.pos3d[2] = surface.trans[2][0] * feature_coord.mx
        + surface.trans[2][1] * feature_coord.my
        + surface.trans[2][3];

    Ok(result)
}

pub fn ar2_tracking(
    handle: &mut AR2Handle,
    surface_set: &mut AR2SurfaceSet,
    data_ptr: &[u8],
    trans: &mut [[f32; 4]; 3],
    err: &mut f32,
) -> Result<(), i32> {
    if surface_set.cont_num <= 0 {
        return Err(-2);
    }

    *err = 0.0;

    for i in 0..surface_set.surface.len() {
        crate::math::mat_mul_fff(
            &surface_set.trans1,
            &surface_set.surface[i].trans,
            &mut handle.wtrans1[i],
        );
        if surface_set.cont_num > 1 {
            crate::math::mat_mul_fff(
                &surface_set.trans2,
                &surface_set.surface[i].trans,
                &mut handle.wtrans2[i],
            );
        }
        if surface_set.cont_num > 2 {
            crate::math::mat_mul_fff(
                &surface_set.trans3,
                &surface_set.surface[i].trans,
                &mut handle.wtrans3[i],
            );
        }
    }

    let cparam_lt = unsafe { handle.cparam_lt.as_ref() }.ok_or(-1)?;

    if handle.tracking_mode == AR2_TRACKING_6DOF {
        extract_visible_features(
            cparam_lt,
            &handle.wtrans1,
            surface_set,
            &mut handle.candidate,
            &mut handle.candidate2,
        )
        .map_err(|_| -1)?;
    } else {
        extract_visible_features_homography(
            handle.xsize,
            handle.ysize,
            &handle.wtrans1,
            surface_set,
            &mut handle.candidate,
            &mut handle.candidate2,
        )
        .map_err(|_| -1)?;
    }

    let mut candidate_ptr_idx = 0; // 0 for candidate, 1 for candidate2
    let mut i = 0;
    let mut num = 0;

    // Sequential tracking loop
    while i < handle.search_feature_num {
        let mut pos = [[0.0f32; 2]; 4];
        let mut k = ar2_select_template(
            if candidate_ptr_idx == 0 {
                &mut handle.candidate
            } else {
                &mut handle.candidate2
            },
            &mut surface_set.prev_feature,
            num,
            &mut pos,
            handle.xsize,
            handle.ysize,
        );

        if k < 0 {
            if candidate_ptr_idx == 0 {
                candidate_ptr_idx = 1;
                k = ar2_select_template(
                    &mut handle.candidate2,
                    &mut surface_set.prev_feature,
                    num,
                    &mut pos,
                    handle.xsize,
                    handle.ysize,
                );
                if k < 0 {
                    break;
                }
            } else {
                break;
            }
        }

        let cand = if candidate_ptr_idx == 0 {
            &handle.candidate[k as usize]
        } else {
            &handle.candidate2[k as usize]
        };

        let cparam_lt = unsafe { handle.cparam_lt.as_ref() };
        let snum = cand.snum as usize;

        match ar2_tracking_2d_sub(
            cparam_lt,
            handle.pix_format,
            handle.xsize,
            handle.ysize,
            handle.blur_level,
            handle.search_size,
            &handle.wtrans1[snum],
            Some(&handle.wtrans2[snum]),
            Some(&handle.wtrans3[snum]),
            surface_set,
            cand,
            data_ptr,
            &mut handle.mf_image,
            &mut handle.templ[0], // Using index 0 for sequential
        ) {
            Ok(res) => {
                if res.sim > handle.sim_thresh {
                    if handle.tracking_mode == AR2_TRACKING_6DOF {
                        let (ix, iy) = cparam_lt
                            .unwrap()
                            .param_ltf
                            .observ2ideal(res.pos2d[0], res.pos2d[1])
                            .map_err(|_| -1)?;
                        handle.pos2d[num as usize][0] = ix;
                        handle.pos2d[num as usize][1] = iy;
                    } else {
                        handle.pos2d[num as usize][0] = res.pos2d[0];
                        handle.pos2d[num as usize][1] = res.pos2d[1];
                    }
                    handle.pos3d[num as usize][0] = res.pos3d[0];
                    handle.pos3d[num as usize][1] = res.pos3d[1];
                    handle.pos3d[num as usize][2] = res.pos3d[2];

                    handle.used_feature[num as usize].snum = cand.snum;
                    handle.used_feature[num as usize].level = cand.level;
                    handle.used_feature[num as usize].num = cand.num;
                    handle.used_feature[num as usize].flag = 0;

                    num += 1;
                }
            }
            Err(_) => {}
        }
        i += 1;
    }

    for idx in 0..num {
        surface_set.prev_feature[idx as usize] = handle.used_feature[idx as usize];
    }
    surface_set.prev_feature[num as usize].flag = -1;

    if num < 3 {
        surface_set.cont_num = 0;
        return Err(-3);
    }

    if handle.tracking_mode == AR2_TRACKING_6DOF {
        unsafe {
            let icp_handle = handle.icp_handle.as_mut().ok_or(-1)?;
            *err = ar2_get_trans_mat(
                icp_handle,
                surface_set.trans1,
                &handle.pos2d,
                &handle.pos3d,
                num as usize,
                trans,
                false,
            );
            if *err > handle.tracking_thresh {
                crate::icp::icp_set_inlier_probability(icp_handle, 0.8);
                *err = ar2_get_trans_mat(
                    icp_handle,
                    *trans,
                    &handle.pos2d,
                    &handle.pos3d,
                    num as usize,
                    trans,
                    true,
                );
                if *err > handle.tracking_thresh {
                    crate::icp::icp_set_inlier_probability(icp_handle, 0.0);
                    *err = ar2_get_trans_mat(
                        icp_handle,
                        *trans,
                        &handle.pos2d,
                        &handle.pos3d,
                        num as usize,
                        trans,
                        true,
                    );
                    if *err > handle.tracking_thresh {
                        surface_set.cont_num = 0;
                        return Err(-4);
                    }
                }
            }
        }
    } else {
        // Homography pose refinement (simplified or robust)
        // For now, return error if not implemented, or implement a basic version
        return Err(-5); // Homography refinement placeholder
    }

    surface_set.cont_num += 1;
    surface_set.trans3 = surface_set.trans2;
    surface_set.trans2 = surface_set.trans1;
    surface_set.trans1 = *trans;

    Ok(())
}

pub fn ar2_get_trans_mat(
    icp_handle: &mut ICPHandleT,
    init_conv: [[f32; 4]; 3],
    pos2d: &[[f32; 2]],
    pos3d: &[[f32; 3]],
    num: usize,
    conv: &mut [[f32; 4]; 3],
    robust_mode: bool,
) -> f32 {
    use crate::icp::{ICP2DCoordT, ICP3DCoordT, ICPDataT};

    let mut dx = 0.0;
    let mut dy = 0.0;
    let mut dz = 0.0;
    for i in 0..num {
        dx += pos3d[i][0];
        dy += pos3d[i][1];
        dz += pos3d[i][2];
    }
    dx /= num as f32;
    dy /= num as f32;
    dz /= num as f32;

    let mut data_2d = vec![ICP2DCoordT::default(); num];
    let mut data_3d = vec![ICP3DCoordT::default(); num];

    for i in 0..num {
        data_2d[i].x = pos2d[i][0] as ARdouble;
        data_2d[i].y = pos2d[i][1] as ARdouble;
        data_3d[i].x = (pos3d[i][0] - dx) as ARdouble;
        data_3d[i].y = (pos3d[i][1] - dy) as ARdouble;
        data_3d[i].z = (pos3d[i][2] - dz) as ARdouble;
    }

    let data = ICPDataT {
        screen_coord: data_2d,
        world_coord: data_3d,
    };

    let mut init_mat = [[0.0 as ARdouble; 4]; 3];
    for j in 0..3 {
        for i in 0..3 {
            init_mat[j][i] = init_conv[j][i] as ARdouble;
        }
    }
    init_mat[0][3] =
        (init_conv[0][0] * dx + init_conv[0][1] * dy + init_conv[0][2] * dz + init_conv[0][3])
            as ARdouble;
    init_mat[1][3] =
        (init_conv[1][0] * dx + init_conv[1][1] * dy + init_conv[1][2] * dz + init_conv[1][3])
            as ARdouble;
    init_mat[2][3] =
        (init_conv[2][0] * dx + init_conv[2][1] * dy + init_conv[2][2] * dz + init_conv[2][3])
            as ARdouble;

    let mut mat = [[0.0 as ARdouble; 4]; 3];
    let mut err = 0.0;

    if !robust_mode {
        crate::icp::icp_point(icp_handle, &data, &init_mat, &mut mat)
            .map(|e| err = e as f32)
            .ok();
    } else {
        crate::icp::icp_point_robust(icp_handle, &data, &init_mat, &mut mat)
            .map(|e| err = e as f32)
            .ok();
    }

    for j in 0..3 {
        for i in 0..3 {
            conv[j][i] = mat[j][i] as f32;
        }
    }
    conv[0][3] = (mat[0][3]
        - mat[0][0] * dx as ARdouble
        - mat[0][1] * dy as ARdouble
        - mat[0][2] * dz as ARdouble) as f32;
    conv[1][3] = (mat[1][3]
        - mat[1][0] * dx as ARdouble
        - mat[1][1] * dy as ARdouble
        - mat[1][2] * dz as ARdouble) as f32;
    conv[2][3] = (mat[2][3]
        - mat[2][0] * dx as ARdouble
        - mat[2][1] * dy as ARdouble
        - mat[2][2] * dz as ARdouble) as f32;

    err as f32
}

pub const KEEP_NUM: usize = 3;
pub const SKIP_INTERVAL: i32 = 3;

pub fn ar2_get_best_matching(
    img: &[u8],
    mf_image: &mut [u8],
    xsize: i32,
    ysize: i32,
    pix_format: ARPixelFormat,
    mtemp: &AR2Template,
    rx: i32,
    ry: i32,
    search: [[i32; 2]; 3],
    bx: &mut i32,
    by: &mut i32,
    val: &mut f32,
) -> Result<(), &'static str> {
    // Initialise mf_image for search areas
    for ii in 0..3 {
        if search[ii][0] < 0 {
            break;
        }

        let px =
            (search[ii][0] / (SKIP_INTERVAL + 1)) * (SKIP_INTERVAL + 1) + (SKIP_INTERVAL + 1) / 2;
        let py =
            (search[ii][1] / (SKIP_INTERVAL + 1)) * (SKIP_INTERVAL + 1) + (SKIP_INTERVAL + 1) / 2;

        let sx = (px - rx).max(0);
        let ex = (px + rx).min(xsize - 1);
        let sy = (py - ry).max(0);
        let ey = (py + ry).min(ysize - 1);

        for j in sy..=ey {
            for i in sx..=ex {
                mf_image[(j * xsize + i) as usize] = 0;
            }
        }
    }

    let mut keep_num = 0;
    let mut cx = [0; KEEP_NUM];
    let mut cy = [0; KEEP_NUM];
    let mut cval = [0; KEEP_NUM];
    let mut ret = true;

    for ii in 0..3 {
        if search[ii][0] < 0 {
            if ret {
                return Err("No valid search point");
            }
            break;
        }

        let px =
            (search[ii][0] / (SKIP_INTERVAL + 1)) * (SKIP_INTERVAL + 1) + (SKIP_INTERVAL + 1) / 2;
        let py =
            (search[ii][1] / (SKIP_INTERVAL + 1)) * (SKIP_INTERVAL + 1) + (SKIP_INTERVAL + 1) / 2;

        for j in (py - ry..=py + ry).step_by((SKIP_INTERVAL + 1) as usize) {
            if j - mtemp.yts1 * AR2_TEMP_SCALE < 0 {
                continue;
            }
            if j + mtemp.yts2 * AR2_TEMP_SCALE >= ysize {
                break;
            }

            for i in (px - rx..=px + rx).step_by((SKIP_INTERVAL + 1) as usize) {
                if i - mtemp.xts1 * AR2_TEMP_SCALE < 0 {
                    continue;
                }
                if i + mtemp.xts2 * AR2_TEMP_SCALE >= xsize {
                    break;
                }

                if mf_image[(j * xsize + i) as usize] != 0 {
                    continue;
                }
                mf_image[(j * xsize + i) as usize] = 1;

                let mut wval = 0;
                if ar2_get_best_matching_sub_fine(
                    img, xsize, ysize, pix_format, mtemp, i, j, &mut wval,
                )
                .is_ok()
                {
                    ret = false;
                    update_candidate(i, j, wval, &mut keep_num, &mut cx, &mut cy, &mut cval);
                }
            }
        }
    }

    let mut wval2 = 0;
    let mut final_ret = Err("No match found");

    // Simplified third pass (ignoring optimized SAT for now to keep it clear, can be optimized later)
    for l in 0..keep_num {
        for j in cy[l] - SKIP_INTERVAL..=cy[l] + SKIP_INTERVAL {
            if j - mtemp.yts1 * AR2_TEMP_SCALE < 0 {
                continue;
            }
            if j + mtemp.yts2 * AR2_TEMP_SCALE >= ysize {
                break;
            }

            for i in cx[l] - SKIP_INTERVAL..=cx[l] + SKIP_INTERVAL {
                if i - mtemp.xts1 * AR2_TEMP_SCALE < 0 {
                    continue;
                }
                if i + mtemp.xts2 * AR2_TEMP_SCALE >= xsize {
                    break;
                }

                let mut wval = 0;
                if ar2_get_best_matching_sub_fine(
                    img, xsize, ysize, pix_format, mtemp, i, j, &mut wval,
                )
                .is_ok()
                {
                    if wval > wval2 {
                        *bx = i;
                        *by = j;
                        wval2 = wval;
                        *val = wval as f32 / 10000.0;
                        final_ret = Ok(());
                    }
                }
            }
        }
    }

    final_ret
}

pub fn ar2_get_best_matching_sub_fine(
    img: &[u8],
    xsize: i32,
    _ysize: i32,
    pix_format: ARPixelFormat,
    mtemp: &AR2Template,
    sx: i32,
    sy: i32,
    val: &mut i32,
) -> Result<(), &'static str> {
    let mut sum1 = 0;
    let mut sum2 = 0;
    let mut sum3 = 0;

    let mut p1_idx = 0;

    match pix_format {
        ARPixelFormat::MONO
        | ARPixelFormat::NV21
        | ARPixelFormat::FourTwoZeroV
        | ARPixelFormat::FourTwoZeroF => {
            let ssx = -mtemp.xts1;
            let eex = mtemp.xts2;
            let ssy = -mtemp.yts1;
            let eey = mtemp.yts2;

            for j in ssy..=eey {
                let row_start = (sy + j * AR2_TEMP_SCALE) * xsize + sx + ssx * AR2_TEMP_SCALE;
                for _i in ssx..=eex {
                    let pixel_val = img
                        [row_start as usize + (_i - ssx) as usize * AR2_TEMP_SCALE as usize]
                        as i32;
                    let template_val = mtemp.img1[p1_idx];
                    if template_val != AR2_TEMPLATE_NULL_PIXEL {
                        sum1 += pixel_val;
                        sum2 += pixel_val * pixel_val;
                        sum3 += pixel_val * template_val as i32;
                    }
                    p1_idx += 1;
                }
            }
        }
        _ => {
            // Placeholder for other pixel formats (RGB, etc.)
            // We can add them later as needed. Mono is the usual target for NFT.
            return Err("Unsupported pixel format for matching");
        }
    }

    sum3 -= sum1 * mtemp.sum / (mtemp.valid_num as i32);
    let vlen = sum2 - sum1 * sum1 / (mtemp.valid_num as i32);
    if vlen == 0 {
        *val = 0;
    } else {
        *val = (sum3 * 100 / mtemp.vlen) * 100 / (vlen as f32).sqrt() as i32;
    }

    Ok(())
}

fn update_candidate(
    x: i32,
    y: i32,
    wval: i32,
    keep_num: &mut usize,
    cx: &mut [i32; KEEP_NUM],
    cy: &mut [i32; KEEP_NUM],
    cval: &mut [i32; KEEP_NUM],
) {
    if *keep_num == 0 {
        cx[0] = x;
        cy[0] = y;
        cval[0] = wval;
        *keep_num = 1;
        return;
    }

    let mut insert_idx = *keep_num;
    for l in 0..*keep_num {
        if cval[l] < wval {
            insert_idx = l;
            break;
        }
    }

    if insert_idx == *keep_num {
        if insert_idx < KEEP_NUM {
            cx[insert_idx] = x;
            cy[insert_idx] = y;
            cval[insert_idx] = wval;
            *keep_num += 1;
        }
        return;
    }

    let m = if *keep_num == KEEP_NUM {
        KEEP_NUM - 1
    } else {
        *keep_num
    };
    if *keep_num < KEEP_NUM {
        *keep_num += 1;
    }

    for n in (insert_idx + 1..=m).rev() {
        cx[n] = cx[n - 1];
        cy[n] = cy[n - 1];
        cval[n] = cval[n - 1];
    }
    cx[insert_idx] = x;
    cy[insert_idx] = y;
    cval[insert_idx] = wval;
}
