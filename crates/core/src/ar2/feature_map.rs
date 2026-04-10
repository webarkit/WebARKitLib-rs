/*
 *  ar2/feature_map.rs
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

//! AR2 Feature Map generation.
//!
//! Ported from `AR2/featureMap.c`. Combines feature map generation and
//! feature selection into a single multi-scale pipeline that returns an
//! [`AR2FeatureSetT`].

use super::feature_set::{AR2FeatureCoordT, AR2FeaturePointsT, AR2FeatureSetT};
use super::image_set::gen_image_layer2;

// ---------------------------------------------------------------------------
// Constants (from AR2/config.h)
// ---------------------------------------------------------------------------

const AR2_DEFAULT_TS1: i32 = 11;
const AR2_DEFAULT_TS2: i32 = 11;
/// Default search radius for feature map generation (used as fallback).
#[allow(dead_code)]
const AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE1: i32 = 10;
const AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE2: i32 = 2;
const AR2_DEFAULT_OCCUPANCY_SIZE: i32 = 24;
const AR2_DEFAULT_MAX_SIM_THRESH2: f32 = 0.95;
const AR2_DEFAULT_SD_THRESH2: f32 = 5.0;

/// Per-level thresholds (L0 = coarsest, L3 = finest).
const MAX_SIM_THRESH: [f32; 4] = [0.80, 0.85, 0.90, 0.98];
const MIN_SIM_THRESH: [f32; 4] = [0.70, 0.65, 0.55, 0.45];
const SD_THRESH: [f32; 4] = [12.0, 10.0, 8.0, 6.0];

/// Minimum pyramid dimension — stop building when either axis < 8.
const MIN_PYRAMID_DIM: i32 = 8;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors returned by the feature map pipeline.
#[derive(Debug)]
pub enum Ar2Error {
    /// The caller supplied invalid parameters.
    InvalidInput(&'static str),
}

impl std::fmt::Display for Ar2Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Ar2Error::InvalidInput(msg) => write!(f, "invalid input: {}", msg),
        }
    }
}

impl std::error::Error for Ar2Error {}

// ---------------------------------------------------------------------------
// Pyramid
// ---------------------------------------------------------------------------

struct PyramidLevel {
    data: Vec<u8>,
    width: i32,
    height: i32,
    dpi: f32,
}

/// Build a multi-resolution image pyramid.
///
/// Level 0 is the original image at `dpi`. Each subsequent level halves the
/// DPI. Stops when the next level would be smaller than 8×8.
fn build_pyramid(image: &[u8], xsize: i32, ysize: i32, dpi: f32) -> Vec<PyramidLevel> {
    let mut levels = Vec::new();

    // Level 0: original
    levels.push(PyramidLevel {
        data: image.to_vec(),
        width: xsize,
        height: ysize,
        dpi,
    });

    let mut current_dpi = dpi;
    loop {
        let next_dpi = current_dpi / 2.0;
        let scale = next_dpi / dpi;
        let next_w = ((xsize as f32) * scale) as i32;
        let next_h = ((ysize as f32) * scale) as i32;
        if next_w < MIN_PYRAMID_DIM || next_h < MIN_PYRAMID_DIM {
            break;
        }
        let layer = gen_image_layer2(image, xsize, ysize, dpi, next_dpi);
        levels.push(PyramidLevel {
            data: layer.img_bw,
            width: layer.xsize,
            height: layer.ysize,
            dpi: layer.dpi,
        });
        current_dpi = next_dpi;
    }

    levels
}

// ---------------------------------------------------------------------------
// Template helpers (ported from featureMap.c static functions)
// ---------------------------------------------------------------------------

/// Create a zero-mean template patch centred at `(cx, cy)`.
///
/// Returns `Some(vlen)` on success, `None` if the patch is out of bounds or
/// has insufficient variance.
fn make_template(
    image: &[u8],
    xsize: i32,
    ysize: i32,
    cx: i32,
    cy: i32,
    ts1: i32,
    ts2: i32,
    sd_thresh: f32,
    template: &mut [f32],
) -> Option<f32> {
    if cy - ts1 < 0 || cy + ts2 >= ysize || cx - ts1 < 0 || cx + ts2 >= xsize {
        return None;
    }

    let patch_size = (ts1 + ts2 + 1) as usize;

    // Compute mean
    let mut ave: f32 = 0.0;
    for j in -ts1..=ts2 {
        let row = ((cy + j) * xsize + (cx - ts1)) as usize;
        for i in 0..patch_size {
            ave += image[row + i] as f32;
        }
    }
    ave /= (patch_size * patch_size) as f32;

    // Build zero-mean template and compute squared magnitude
    let mut vlen1: f32 = 0.0;
    let mut idx = 0;
    for j in -ts1..=ts2 {
        let row = ((cy + j) * xsize + (cx - ts1)) as usize;
        for i in 0..patch_size {
            let v = image[row + i] as f32 - ave;
            template[idx] = v;
            vlen1 += v * v;
            idx += 1;
        }
    }

    if vlen1 == 0.0 {
        return None;
    }
    let n = (patch_size * patch_size) as f32;
    if vlen1 / n < sd_thresh * sd_thresh {
        return None;
    }

    Some(vlen1.sqrt())
}

/// Compute the normalised cross-correlation between `template` and the image
/// patch centred at `(cx, cy)`.
///
/// Returns `None` if the patch is out of bounds or has zero variance.
fn get_similarity(
    image: &[u8],
    xsize: i32,
    ysize: i32,
    template: &[f32],
    vlen: f32,
    ts1: i32,
    ts2: i32,
    cx: i32,
    cy: i32,
) -> Option<f32> {
    if cy - ts1 < 0 || cy + ts2 >= ysize || cx - ts1 < 0 || cx + ts2 >= xsize {
        return None;
    }

    let patch_size = (ts1 + ts2 + 1) as usize;
    let n = (patch_size * patch_size) as f32;

    let mut sx: f32 = 0.0;
    let mut sxx: f32 = 0.0;
    let mut sxy: f32 = 0.0;
    let mut idx = 0;

    for j in -ts1..=ts2 {
        let row = ((cy + j) * xsize + (cx - ts1)) as usize;
        for i in 0..patch_size {
            let p = image[row + i] as f32;
            sx += p;
            sxx += p * p;
            sxy += p * template[idx];
            idx += 1;
        }
    }

    let vlen2_sq = sxx - sx * sx / n;
    if vlen2_sq <= 0.0 {
        return None;
    }
    let vlen2 = vlen2_sq.sqrt();

    Some(sxy / (vlen * vlen2))
}

// ---------------------------------------------------------------------------
// Feature map generation (per-level)
// ---------------------------------------------------------------------------

/// Generate a per-pixel similarity map for a single image.
///
/// Ported from `ar2GenFeatureMap` in the C source.
///
/// The returned `Vec<f32>` has length `xsize * ysize`. Values of `1.0`
/// indicate invalid / suppressed pixels; lower values indicate better
/// (more unique) features.
fn gen_feature_map_for_level(
    image: &[u8],
    xsize: i32,
    ysize: i32,
    search_size1: i32,
    search_size2: i32,
    max_sim_thresh: f32,
    sd_thresh: f32,
) -> Vec<f32> {
    let w = xsize as usize;
    let h = ysize as usize;
    let total = w * h;

    let ts1 = AR2_DEFAULT_TS1;
    let ts2 = AR2_DEFAULT_TS2;
    let template_area = ((ts1 + ts2 + 1) * (ts1 + ts2 + 1)) as usize;

    // Stage 1: Sobel gradient magnitude
    let mut grad = vec![-1.0f32; total];
    for j in 1..(h - 1) {
        for i in 1..(w - 1) {
            let idx = j * w + i;
            let p = |di: isize, dj: isize| -> f32 {
                image[((j as isize + dj) as usize) * w + ((i as isize + di) as usize)] as f32
            };
            let dx =
                (p(1, -1) - p(-1, -1) + p(1, 0) - p(-1, 0) + p(1, 1) - p(-1, 1)) / (3.0 * 256.0);
            let dy =
                (p(1, 1) - p(1, -1) + p(0, 1) - p(0, -1) + p(-1, 1) - p(-1, -1)) / (3.0 * 256.0);
            grad[idx] = ((dx * dx + dy * dy) / 2.0).sqrt();
        }
    }

    // Stage 2: 4-neighbor NMS + histogram threshold (keep top 2%)
    let mut hist = [0u32; 1000];
    let mut sum = 0u32;
    for j in 1..(h - 1) {
        for i in 1..(w - 1) {
            let idx = j * w + i;
            let g = grad[idx];
            if g > grad[idx - 1] && g > grad[idx + 1] && g > grad[idx - w] && g > grad[idx + w] {
                let k = ((g * 1000.0) as usize).min(999);
                hist[k] += 1;
                sum += 1;
            }
        }
    }
    let _ = sum; // used implicitly via the 2% threshold
    let threshold_pixels = (total as f32 * 0.02) as u32;
    let mut acc = 0u32;
    let mut thresh_bin = 0usize;
    for i in (0..1000).rev() {
        acc += hist[i];
        if acc >= threshold_pixels {
            thresh_bin = i;
            break;
        }
    }

    // Stage 3: For each pixel passing NMS + threshold, compute template
    // similarity in the annular search region.
    let mut fmap = vec![1.0f32; total];
    let mut tmpl = vec![0.0f32; template_area];

    for j in 1..(h - 1) {
        for i in 1..(w - 1) {
            let idx = j * w + i;
            let g = grad[idx];

            // NMS check
            if g <= grad[idx - 1] || g <= grad[idx + 1] || g <= grad[idx - w] || g <= grad[idx + w]
            {
                continue;
            }
            // Threshold check
            if (g * 1000.0) as usize <= thresh_bin {
                continue;
            }

            let ci = i as i32;
            let cj = j as i32;

            let vlen =
                match make_template(image, xsize, ysize, ci, cj, ts1, ts2, sd_thresh, &mut tmpl) {
                    Some(v) => v,
                    None => continue,
                };

            let mut max = -1.0f32;
            let mut early_exit = false;
            for jj in -search_size1..=search_size1 {
                for ii in -search_size1..=search_size1 {
                    if ii * ii + jj * jj <= search_size2 * search_size2 {
                        continue;
                    }
                    if let Some(sim) =
                        get_similarity(image, xsize, ysize, &tmpl, vlen, ts1, ts2, ci + ii, cj + jj)
                    {
                        if sim > max {
                            max = sim;
                            if max > max_sim_thresh {
                                early_exit = true;
                                break;
                            }
                        }
                    }
                }
                if early_exit {
                    break;
                }
            }
            fmap[idx] = max;
        }
    }

    fmap
}

// ---------------------------------------------------------------------------
// Feature selection
// ---------------------------------------------------------------------------

/// Greedily select features from a feature map.
///
/// Ported from `ar2SelectFeature` in the C source.
fn select_features(
    image: &[u8],
    xsize: i32,
    ysize: i32,
    dpi: f32,
    fmap: &[f32],
    max_feature_num: i32,
    max_sim_thresh: f32,
    min_sim_thresh: f32,
    sd_thresh: f32,
    occ_size: i32,
) -> Vec<AR2FeatureCoordT> {
    let w = xsize as usize;
    let h = ysize as usize;
    let ts1 = AR2_DEFAULT_TS1;
    let ts2 = AR2_DEFAULT_TS2;
    let search_size2 = AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE2;
    let template_area = ((ts1 + ts2 + 1) * (ts1 + ts2 + 1)) as usize;

    let mut work = fmap.to_vec();
    let mut tmpl = vec![0.0f32; template_area];
    let mut coords = Vec::new();

    while (coords.len() as i32) < max_feature_num {
        // Find pixel with minimum similarity score
        let mut min_sim = max_sim_thresh;
        let mut cx: i32 = -1;
        let mut cy: i32 = -1;
        for j in 0..h {
            for i in 0..w {
                let v = work[j * w + i];
                if v < min_sim {
                    min_sim = v;
                    cx = i as i32;
                    cy = j as i32;
                }
            }
        }
        if cx == -1 {
            break;
        }

        // Validate: re-create template and check variance
        let vlen = match make_template(image, xsize, ysize, cx, cy, ts1, ts2, 0.0, &mut tmpl) {
            Some(v) => v,
            None => {
                work[(cy as usize) * w + (cx as usize)] = 1.0;
                continue;
            }
        };
        let patch_size = (ts1 + ts2 + 1) as f32;
        if vlen / patch_size < sd_thresh {
            work[(cy as usize) * w + (cx as usize)] = 1.0;
            continue;
        }

        // Search within search_size2 radius for too-similar or too-uniform
        let mut local_min: f32 = 1.0;
        let mut local_max: f32 = -1.0;
        let mut reject = false;
        'outer: for j in -search_size2..=search_size2 {
            for i in -search_size2..=search_size2 {
                if i * i + j * j > search_size2 * search_size2 {
                    continue;
                }
                if i == 0 && j == 0 {
                    continue;
                }
                if let Some(sim) =
                    get_similarity(image, xsize, ysize, &tmpl, vlen, ts1, ts2, cx + i, cy + j)
                {
                    if sim < local_min {
                        local_min = sim;
                        if local_min < min_sim_thresh && local_min < min_sim {
                            reject = true;
                            break 'outer;
                        }
                    }
                    if sim > local_max {
                        local_max = sim;
                        if local_max > 0.99 {
                            reject = true;
                            break 'outer;
                        }
                    }
                }
            }
        }
        if reject {
            work[(cy as usize) * w + (cx as usize)] = 1.0;
            continue;
        }

        // Accept feature — convert to marker-space millimetres
        let mx = cx as f32 / dpi * 25.4;
        let my = (ysize - cy) as f32 / dpi * 25.4;
        coords.push(AR2FeatureCoordT {
            x: cx,
            y: cy,
            mx,
            my,
            max_sim: min_sim,
        });

        // Suppress occupancy region
        for j in -occ_size..=occ_size {
            for i in -occ_size..=occ_size {
                let ny = cy + j;
                let nx = cx + i;
                if ny >= 0 && ny < ysize && nx >= 0 && nx < xsize {
                    work[(ny as usize) * w + (nx as usize)] = 1.0;
                }
            }
        }
    }

    coords
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Generate a multi-scale feature set from a grayscale image.
///
/// This combines the C functions `ar2GenFeatureMap`, `ar2SelectFeature`, and
/// the image pyramid builder into a single pipeline.
///
/// # Arguments
///
/// * `image` — Row-major grayscale pixel data (`xsize * ysize` bytes).
/// * `xsize` — Image width in pixels.
/// * `ysize` — Image height in pixels.
/// * `dpi` — Image resolution in dots per inch.
/// * `search_size` — Search window radius (C default: `AR2_DEFAULT_SEARCH_SIZE` = 25).
/// * `search_feature_num` — Maximum number of features per pyramid level.
///
/// # Errors
///
/// Returns [`Ar2Error::InvalidInput`] if the image is empty, dimensions are
/// non-positive, or the buffer length does not match `xsize * ysize`.
pub fn ar2_gen_feature_map(
    image: &[u8],
    xsize: i32,
    ysize: i32,
    dpi: f32,
    search_size: i32,
    search_feature_num: i32,
) -> Result<AR2FeatureSetT, Ar2Error> {
    // Validate inputs
    if image.is_empty() || xsize <= 0 || ysize <= 0 {
        return Err(Ar2Error::InvalidInput(
            "image is empty or has zero dimensions",
        ));
    }
    if image.len() != (xsize as usize) * (ysize as usize) {
        return Err(Ar2Error::InvalidInput(
            "image buffer length does not match xsize * ysize",
        ));
    }
    if dpi <= 0.0 {
        return Err(Ar2Error::InvalidInput("dpi must be positive"));
    }

    let pyramid = build_pyramid(image, xsize, ysize, dpi);
    let num_levels = pyramid.len();
    let mut list = Vec::new();

    for (i, level) in pyramid.iter().enumerate() {
        let level_idx = i.min(3);
        let max_sim = MAX_SIM_THRESH[level_idx];
        let min_sim = MIN_SIM_THRESH[level_idx];
        let sd = SD_THRESH[level_idx];

        // Generate per-pixel feature map.
        // The caller's `search_size` overrides the default search radius.
        let fmap = gen_feature_map_for_level(
            &level.data,
            level.width,
            level.height,
            search_size,
            AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE2,
            AR2_DEFAULT_MAX_SIM_THRESH2,
            AR2_DEFAULT_SD_THRESH2,
        );

        // Select features
        let coords = select_features(
            &level.data,
            level.width,
            level.height,
            level.dpi,
            &fmap,
            search_feature_num,
            max_sim,
            min_sim,
            sd,
            AR2_DEFAULT_OCCUPANCY_SIZE,
        );

        if coords.is_empty() {
            continue;
        }

        let maxdpi = level.dpi;
        let mindpi = if i + 1 < num_levels {
            pyramid[i + 1].dpi
        } else {
            level.dpi / 2.0
        };

        list.push(AR2FeaturePointsT {
            coord: coords,
            scale: i as i32,
            maxdpi,
            mindpi,
        });
    }

    Ok(AR2FeatureSetT { list })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore] // Heavy computation — run explicitly with `cargo test -- --ignored`
    fn test_feature_map_produces_points() {
        let img = image::open("examples/Data/pinball.jpg").expect("failed to open test image");
        let gray = img.to_luma8();
        let w = gray.width() as i32;
        let h = gray.height() as i32;
        let data = gray.into_raw();

        let result = ar2_gen_feature_map(&data, w, h, 72.0, 16, 64).expect("feature map failed");
        assert!(!result.list.is_empty(), "should produce at least one scale");

        let total: usize = result.list.iter().map(|p| p.coord.len()).sum();
        assert!(total > 0, "should produce at least one feature");

        for pts in &result.list {
            for c in &pts.coord {
                assert!(c.x >= 0 && c.x < w, "x out of bounds: {}", c.x);
                assert!(c.y >= 0 && c.y < h, "y out of bounds: {}", c.y);
                assert!(c.mx >= 0.0, "mx should be >= 0: {}", c.mx);
                assert!(c.my >= 0.0, "my should be >= 0: {}", c.my);
            }
        }
    }

    /// Lightweight version of the integration test using a small synthetic image.
    #[test]
    fn test_feature_map_small_synthetic() {
        // 64×64 image with a checkerboard pattern (strong features).
        let w: i32 = 64;
        let h: i32 = 64;
        let mut data = vec![0u8; (w * h) as usize];
        for y in 0..h {
            for x in 0..w {
                data[(y * w + x) as usize] = if ((x / 4) + (y / 4)) % 2 == 0 {
                    200
                } else {
                    50
                };
            }
        }

        let result = ar2_gen_feature_map(&data, w, h, 72.0, 6, 16);
        // Should succeed (even if no features survive the thresholds, it should not error).
        assert!(result.is_ok());
    }

    #[test]
    fn test_feature_map_rejects_empty_image() {
        let err = ar2_gen_feature_map(&[], 0, 0, 72.0, 16, 64);
        assert!(matches!(err, Err(Ar2Error::InvalidInput(_))));
    }

    #[test]
    fn test_build_pyramid_levels() {
        let w = 128;
        let h = 128;
        let data = vec![128u8; w * h];
        let levels = build_pyramid(&data, w as i32, h as i32, 200.0);
        // 200 → 100 → 50 → 25 (at 25 DPI: 128*25/200 = 16 ≥ 8)
        // → 12.5 (128*12.5/200 = 8 ≥ 8) → 6.25 (128*6.25/200 = 4 < 8, stop)
        assert!(
            levels.len() >= 3,
            "should have at least 3 levels, got {}",
            levels.len()
        );
        assert_eq!(levels[0].width, w as i32);
        assert_eq!(levels[0].height, h as i32);
        for i in 1..levels.len() {
            assert!(
                levels[i].width < levels[i - 1].width || levels[i].height < levels[i - 1].height
            );
        }
    }

    #[test]
    fn test_make_template_out_of_bounds() {
        let data = vec![100u8; 10 * 10];
        let mut tmpl = vec![0.0f32; 529]; // (11+11+1)^2 = 529
                                          // Centre at (0,0) with ts1=11 → out of bounds
        assert!(make_template(&data, 10, 10, 0, 0, 11, 11, 0.0, &mut tmpl).is_none());
    }
}
