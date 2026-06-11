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
use super::image_set::AR2ImageSetT;
use super::Ar2Error;
use crate::{arlog_e, arlog_i};
use rayon::prelude::*;

// ---------------------------------------------------------------------------
// Constants (from AR2/config.h)
// ---------------------------------------------------------------------------

const AR2_DEFAULT_TS1: i32 = 11;
const AR2_DEFAULT_TS2: i32 = 11;
/// Default search radius for stage-1 feature map generation.
#[allow(dead_code)]
const AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE1: i32 = 10;
const AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE2: i32 = 2;
const AR2_DEFAULT_OCCUPANCY_SIZE: i32 = 24;
/// Stage-2 max-similarity threshold used in `gen_feature_map_for_level`.
const AR2_DEFAULT_MAX_SIM_THRESH2: f32 = 0.95;
/// Stage-2 standard-deviation threshold used in `gen_feature_map_for_level`.
const AR2_DEFAULT_SD_THRESH2: f32 = 5.0;

/// Max-similarity thresholds indexed by tracking extraction level (0–3).
///
/// Matches `AR2_DEFAULT_MAX_SIM_THRESH_L0..L3` in `AR2/config.h`.
const MAX_SIM_THRESH: [f32; 4] = [0.80, 0.85, 0.90, 0.98];
/// Min-similarity thresholds indexed by tracking extraction level (0–3).
const MIN_SIM_THRESH: [f32; 4] = [0.70, 0.65, 0.55, 0.45];
/// Standard-deviation thresholds indexed by tracking extraction level (0–3).
const SD_THRESH: [f32; 4] = [12.0, 10.0, 8.0, 6.0];

/// Parameters derived from the tracking extraction level (0–4).
///
/// All pyramid levels in a single `ar2_gen_feature_map` call share the
/// same parameter set — matching `markerCreator.cpp`'s `tracking_extraction_level`.
struct ExtractionParams {
    max_sim_thresh: f32,
    min_sim_thresh: f32,
    sd_thresh: f32,
    occ_size: i32,
}

/// Look up extraction parameters for the given level (0–4).
///
/// Levels 0–3 map directly to the threshold arrays; level 4 uses the same
/// thresholds as level 3 but with a smaller occupancy region (matching C).
fn extraction_params(level: u8) -> ExtractionParams {
    let idx = (level as usize).min(3);
    let occ_size = match level {
        0 | 1 => AR2_DEFAULT_OCCUPANCY_SIZE,
        2 | 3 => AR2_DEFAULT_OCCUPANCY_SIZE * 2 / 3,
        _ => AR2_DEFAULT_OCCUPANCY_SIZE / 2,
    };
    ExtractionParams {
        max_sim_thresh: MAX_SIM_THRESH[idx],
        min_sim_thresh: MIN_SIM_THRESH[idx],
        sd_thresh: SD_THRESH[idx],
        occ_size,
    }
}

// ---------------------------------------------------------------------------
// Template helpers (ported from featureMap.c static functions)
// ---------------------------------------------------------------------------

/// Create a zero-mean template patch centred at `(cx, cy)`.
///
/// Returns `Some(vlen)` on success, `None` if the patch is out of bounds or
/// has insufficient variance.
#[allow(clippy::too_many_arguments)]
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
///
/// Dispatches to AVX2/FMA or SSE4.1 SIMD kernels when the corresponding
/// feature flags are enabled and the CPU supports them at runtime. Falls
/// back to the scalar implementation otherwise. All three paths produce
/// near-identical results; SIMD paths may differ from scalar by tiny FP
/// rounding due to lane-parallel summation of `sxy`.
///
/// `sx` (sum of u8 values) and `sxx` (sum of squared u8 values) are
/// accumulated as `u64` integers in all paths — no FP drift on those.
#[inline]
#[allow(clippy::too_many_arguments)]
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
    #[cfg(all(feature = "simd-x86-avx2", target_arch = "x86_64"))]
    {
        if is_x86_feature_detected!("avx2") && is_x86_feature_detected!("fma") {
            return unsafe {
                get_similarity_avx2(image, xsize, ysize, template, vlen, ts1, ts2, cx, cy)
            };
        }
    }
    #[cfg(all(feature = "simd-x86-sse41", target_arch = "x86_64"))]
    {
        if is_x86_feature_detected!("sse4.1") {
            return unsafe {
                get_similarity_sse41(image, xsize, ysize, template, vlen, ts1, ts2, cx, cy)
            };
        }
    }
    get_similarity_scalar(image, xsize, ysize, template, vlen, ts1, ts2, cx, cy)
}

/// Scalar reference implementation of [`get_similarity`].
///
/// `sx`/`sxx` use `u64` integer accumulation to avoid f32 rounding past
/// 2^24. `sxy` is accumulated left-to-right in row-major order.
#[inline]
#[allow(clippy::too_many_arguments)]
fn get_similarity_scalar(
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

    let mut sx_i: u64 = 0;
    let mut sxx_i: u64 = 0;
    let mut sxy: f32 = 0.0;
    let mut idx = 0;

    for j in -ts1..=ts2 {
        let row = ((cy + j) * xsize + (cx - ts1)) as usize;
        for i in 0..patch_size {
            let p_u = image[row + i] as u64;
            sx_i += p_u;
            sxx_i += p_u * p_u;
            sxy += (p_u as f32) * template[idx];
            idx += 1;
        }
    }

    let sx = sx_i as f32;
    let sxx = sxx_i as f32;

    let vlen2_sq = sxx - sx * sx / n;
    if vlen2_sq <= 0.0 {
        return None;
    }
    let vlen2 = vlen2_sq.sqrt();

    Some(sxy / (vlen * vlen2))
}

/// SSE4.1 SIMD implementation of [`get_similarity`].
///
/// Vectorises all three accumulators:
/// - `sx`: `_mm_sad_epu8` (16 bytes/iter) — integer-exact
/// - `sxx`: `_mm_cvtepu8_epi16` + `_mm_madd_epi16` — integer-exact
/// - `sxy`: 4-wide f32 multiply-accumulate (`_mm_cvtepi32_ps` + `_mm_mul_ps`
///   + `_mm_add_ps`), horizontal reduce once at the end
///
/// # Safety
///
/// Caller must ensure SSE4.1 is available. Only called from [`get_similarity`]
/// after `is_x86_feature_detected!`.
#[cfg(all(feature = "simd-x86-sse41", target_arch = "x86_64"))]
#[target_feature(enable = "sse4.1")]
// rationale: SIMD variant of get_similarity; signature locked to match
// the scalar fallback and sibling SIMD impl for runtime dispatch via
// is_x86_feature_detected! — bundling args into a struct would force
// every caller to materialize it on a hot path with no design win.
#[allow(clippy::too_many_arguments)]
unsafe fn get_similarity_sse41(
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
    use std::arch::x86_64::*;

    if cy - ts1 < 0 || cy + ts2 >= ysize || cx - ts1 < 0 || cx + ts2 >= xsize {
        return None;
    }

    let patch_size = (ts1 + ts2 + 1) as usize;
    let n = (patch_size * patch_size) as f32;

    let mut sx_i: u64 = 0;
    let mut sxx_i: u64 = 0;
    let mut sxy_acc = _mm_setzero_ps(); // 4 f32 lanes
    let zero = _mm_setzero_si128();

    let mut idx: usize = 0;
    for j in -ts1..=ts2 {
        let row = ((cy + j) * xsize + (cx - ts1)) as usize;
        let img_row = &image[row..row + patch_size];
        let tmpl_row = &template[idx..idx + patch_size];

        let mut i = 0usize;

        // Process 16 bytes at a time for sx/sxx
        while i + 16 <= patch_size {
            let v = _mm_loadu_si128(img_row.as_ptr().add(i) as *const __m128i);

            // sx: sum of u8 via SAD against zero
            let sad = _mm_sad_epu8(v, zero);
            sx_i += _mm_cvtsi128_si64(sad) as u64;
            sx_i += _mm_extract_epi64(sad, 1) as u64;

            // sxx: expand to i16, self-multiply → i32 sums
            let lo16 = _mm_cvtepu8_epi16(v);
            let hi16 = _mm_cvtepu8_epi16(_mm_srli_si128(v, 8));
            let sq = _mm_add_epi32(_mm_madd_epi16(lo16, lo16), _mm_madd_epi16(hi16, hi16));
            let sq_hi = _mm_unpackhi_epi64(sq, sq);
            let sum2 = _mm_add_epi32(sq, sq_hi);
            let sum1 = _mm_add_epi32(sum2, _mm_shuffle_epi32(sum2, 0b01));
            sxx_i += _mm_cvtsi128_si32(sum1) as u32 as u64;

            i += 16;
        }
        // Tail for sx/sxx (scalar)
        for &b in &img_row[i..patch_size] {
            let p = b as u64;
            sx_i += p;
            sxx_i += p * p;
        }

        // sxy: 4-wide f32 multiply-accumulate
        let mut i = 0usize;
        while i + 4 <= patch_size {
            // Load 4 u8 → i32 → f32 (unaligned read — u8 slice has no
            // alignment guarantee for i32).
            let v4 = _mm_cvtepu8_epi32(_mm_cvtsi32_si128(std::ptr::read_unaligned(
                img_row.as_ptr().add(i) as *const i32,
            )));
            let pf = _mm_cvtepi32_ps(v4);
            let tf = _mm_loadu_ps(tmpl_row.as_ptr().add(i));
            sxy_acc = _mm_add_ps(sxy_acc, _mm_mul_ps(pf, tf));
            i += 4;
        }
        // Tail for sxy (scalar)
        for ii in i..patch_size {
            let p = img_row[ii] as f32;
            sxy_acc = _mm_add_ps(sxy_acc, _mm_set_ss(p * tmpl_row[ii]));
        }

        idx += patch_size;
    }

    // Horizontal sum of sxy_acc (4 lanes → 1)
    let shuf = _mm_movehdup_ps(sxy_acc); // [1,1,3,3]
    let s1 = _mm_add_ps(sxy_acc, shuf); // [0+1, ?, 2+3, ?]
    let s2 = _mm_movehl_ps(s1, s1); // [2+3, ?, ?, ?]
    let sxy = _mm_cvtss_f32(_mm_add_ss(s1, s2));

    let sx = sx_i as f32;
    let sxx = sxx_i as f32;

    let vlen2_sq = sxx - sx * sx / n;
    if vlen2_sq <= 0.0 {
        return None;
    }
    let vlen2 = vlen2_sq.sqrt();

    Some(sxy / (vlen * vlen2))
}

/// AVX2/FMA SIMD implementation of [`get_similarity`].
///
/// 8-wide f32 vectorisation of `sxy` using `_mm256_fmadd_ps`. Integer
/// accumulators `sx`/`sxx` use the same SSE4.1 ops (AVX2 doesn't improve
/// 16-byte SAD/MADD meaningfully for our 23-byte rows).
///
/// # Safety
///
/// Caller must ensure AVX2 and FMA are available. Only called from
/// [`get_similarity`] after `is_x86_feature_detected!`.
#[cfg(all(feature = "simd-x86-avx2", target_arch = "x86_64"))]
#[target_feature(enable = "avx2,fma")]
// rationale: SIMD variant of get_similarity; signature locked to match
// the scalar fallback and sibling SIMD impl for runtime dispatch via
// is_x86_feature_detected! — bundling args into a struct would force
// every caller to materialize it on a hot path with no design win.
#[allow(clippy::too_many_arguments)]
unsafe fn get_similarity_avx2(
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
    use std::arch::x86_64::*;

    if cy - ts1 < 0 || cy + ts2 >= ysize || cx - ts1 < 0 || cx + ts2 >= xsize {
        return None;
    }

    let patch_size = (ts1 + ts2 + 1) as usize;
    let n = (patch_size * patch_size) as f32;

    let mut sx_i: u64 = 0;
    let mut sxx_i: u64 = 0;
    let mut sxy_acc = _mm256_setzero_ps(); // 8 f32 lanes
    let zero128 = _mm_setzero_si128();

    let mut idx: usize = 0;
    for j in -ts1..=ts2 {
        let row = ((cy + j) * xsize + (cx - ts1)) as usize;
        let img_row = &image[row..row + patch_size];
        let tmpl_row = &template[idx..idx + patch_size];

        // sx/sxx: SSE4.1 integer ops (16 bytes at a time)
        let mut i = 0usize;
        while i + 16 <= patch_size {
            let v = _mm_loadu_si128(img_row.as_ptr().add(i) as *const __m128i);
            let sad = _mm_sad_epu8(v, zero128);
            sx_i += _mm_cvtsi128_si64(sad) as u64;
            sx_i += _mm_extract_epi64(sad, 1) as u64;

            let lo16 = _mm_cvtepu8_epi16(v);
            let hi16 = _mm_cvtepu8_epi16(_mm_srli_si128(v, 8));
            let sq = _mm_add_epi32(_mm_madd_epi16(lo16, lo16), _mm_madd_epi16(hi16, hi16));
            let sq_hi = _mm_unpackhi_epi64(sq, sq);
            let sum2 = _mm_add_epi32(sq, sq_hi);
            let sum1 = _mm_add_epi32(sum2, _mm_shuffle_epi32(sum2, 0b01));
            sxx_i += _mm_cvtsi128_si32(sum1) as u32 as u64;
            i += 16;
        }
        for &b in &img_row[i..patch_size] {
            let p = b as u64;
            sx_i += p;
            sxx_i += p * p;
        }

        // sxy: 8-wide FMA (load 8 u8 → 2×4 i32 → 8 f32, fmadd)
        let mut i = 0usize;
        while i + 8 <= patch_size {
            // Load 8 u8 bytes via unaligned u64 read → lower 64 bits of xmm.
            let v8 = _mm_cvtsi64_si128(std::ptr::read_unaligned(
                img_row.as_ptr().add(i) as *const i64
            ));
            // Expand lower 4 u8 → i32 and upper 4 u8 → i32
            let lo4 = _mm_cvtepu8_epi32(v8);
            let hi4 = _mm_cvtepu8_epi32(_mm_srli_si128(v8, 4));
            // Combine into 256-bit i32 vector and convert to f32
            let v8_i32 = _mm256_set_m128i(hi4, lo4);
            let pf = _mm256_cvtepi32_ps(v8_i32);
            let tf = _mm256_loadu_ps(tmpl_row.as_ptr().add(i));
            sxy_acc = _mm256_fmadd_ps(pf, tf, sxy_acc);
            i += 8;
        }
        // 4-wide tail
        while i + 4 <= patch_size {
            let v4 = _mm_cvtepu8_epi32(_mm_cvtsi32_si128(std::ptr::read_unaligned(
                img_row.as_ptr().add(i) as *const i32,
            )));
            let pf = _mm_cvtepi32_ps(v4);
            let tf = _mm_loadu_ps(tmpl_row.as_ptr().add(i));
            let prod = _mm_mul_ps(pf, tf);
            // Add 128-bit product into lower lane of 256-bit accumulator
            sxy_acc = _mm256_add_ps(sxy_acc, _mm256_castps128_ps256(prod));
            i += 4;
        }
        // Scalar tail
        for ii in i..patch_size {
            let p = img_row[ii] as f32;
            let s = _mm_set_ss(p * tmpl_row[ii]);
            sxy_acc = _mm256_add_ps(sxy_acc, _mm256_castps128_ps256(s));
        }

        idx += patch_size;
    }

    // Horizontal sum of sxy_acc (8 lanes → 1)
    let hi128 = _mm256_extractf128_ps(sxy_acc, 1);
    let lo128 = _mm256_castps256_ps128(sxy_acc);
    let sum128 = _mm_add_ps(lo128, hi128);
    let shuf = _mm_movehdup_ps(sum128);
    let s1 = _mm_add_ps(sum128, shuf);
    let s2 = _mm_movehl_ps(s1, s1);
    let sxy = _mm_cvtss_f32(_mm_add_ss(s1, s2));

    let sx = sx_i as f32;
    let sxx = sxx_i as f32;

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
#[allow(clippy::needless_range_loop)]
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

    arlog_i!("         ImageSize = {:7}[pixel]", total);
    arlog_i!("Extracted features = {:7}[pixel]", sum);
    arlog_i!(" Filtered features = {:7}[pixel]", acc);

    // Stage 3: For each pixel passing NMS + threshold, compute template
    // similarity in the annular search region.
    //
    // Parallelised across pyramid rows via Rayon. Each row of `fmap` is
    // written only by its own thread; `image`, `grad`, and the template-area
    // are read-only, so this is a pure data-parallel kernel.
    let mut fmap = vec![1.0f32; total];

    fmap.par_chunks_mut(w).enumerate().for_each(|(j, row_out)| {
        if j == 0 || j >= h - 1 {
            return;
        }

        // Per-thread template buffer — allocated once per row.
        let mut tmpl = vec![0.0f32; template_area];

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
            row_out[i] = max;
        }
    });

    fmap
}

// ---------------------------------------------------------------------------
// Feature selection
// ---------------------------------------------------------------------------

/// Greedily select features from a feature map.
///
/// Ported from `ar2SelectFeature` in the C source.
#[allow(clippy::too_many_arguments, clippy::needless_range_loop)]
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

    arlog_i!("Max feature = {}", max_feature_num);

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

/// Generate a multi-scale feature set from a pre-built AR2 image pyramid.
///
/// Ports the per-scale feature generation loop from `markerCreator.cpp`,
/// combining `ar2GenFeatureMap` and `ar2SelectFeature2` calls for every
/// scale in the image set.
///
/// # Arguments
///
/// * `image_set` — A multi-resolution image pyramid produced by
///   [`ar2_gen_image_set()`](super::image_set::ar2_gen_image_set).
/// * `search_size` — Stage-1 search window half-width
///   (`AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE1` = 10 in C).
/// * `search_feature_num` — Maximum features to select per pyramid level.
/// * `level` — Tracking extraction level (0–4; default 2). Controls the
///   similarity and standard-deviation thresholds and the occupancy region
///   size, matching `markerCreator.cpp`'s `-level=n` flag.
///
/// # Returns
///
/// An [`AR2FeatureSetT`] containing feature points at each pyramid level,
/// with `mindpi`/`maxdpi` ranges computed exactly as the C tool does.
///
/// # Errors
///
/// Returns [`Ar2Error::InvalidInput`] if the image set is empty.
pub fn ar2_gen_feature_map(
    image_set: &AR2ImageSetT,
    search_size: i32,
    search_feature_num: i32,
    level: u8,
) -> Result<AR2FeatureSetT, Ar2Error> {
    if image_set.scale.is_empty() {
        arlog_e!("ar2_gen_feature_map: image set has no scales");
        return Err(Ar2Error::InvalidInput("image set has no scales"));
    }

    let params = extraction_params(level);
    let scales = &image_set.scale;
    let num = scales.len();
    let mut list = Vec::new();

    for (i, img) in scales.iter().enumerate() {
        // Generate per-pixel feature similarity map (stage-1 constants).
        let fmap = gen_feature_map_for_level(
            &img.img_bw,
            img.xsize,
            img.ysize,
            search_size,
            AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE2,
            AR2_DEFAULT_MAX_SIM_THRESH2,
            AR2_DEFAULT_SD_THRESH2,
        );

        // Greedily select features using level-dependent thresholds.
        let coords = select_features(
            &img.img_bw,
            img.xsize,
            img.ysize,
            img.dpi,
            &fmap,
            search_feature_num,
            params.max_sim_thresh,
            params.min_sim_thresh,
            params.sd_thresh,
            params.occ_size,
        );

        // Compute mindpi: next lower DPI in the set, or current × 0.5.
        // Ports the scale1 loop in markerCreator.cpp.
        let mindpi = scales
            .iter()
            .filter(|s| s.dpi < img.dpi)
            .map(|s| s.dpi)
            .fold(0.0f32, f32::max);
        let mindpi = if mindpi == 0.0 { img.dpi * 0.5 } else { mindpi };

        // Compute maxdpi: 0.8×current + 0.2×next-higher, or current × 2.0.
        // Ports the second scale1 loop in markerCreator.cpp.
        let next_higher = scales
            .iter()
            .filter(|s| s.dpi > img.dpi)
            .map(|s| s.dpi)
            .fold(f32::MAX, f32::min);
        let maxdpi = if next_higher == f32::MAX {
            img.dpi * 2.0
        } else {
            img.dpi * 0.8 + next_higher * 0.2
        };

        list.push(AR2FeaturePointsT {
            coord: coords,
            scale: i as i32,
            maxdpi,
            mindpi,
        });
    }

    // Suppress unused-variable lint: `num` is only needed for bounds logic
    // above, which we now compute via iterators. Confirm it equals scale count.
    let _ = num;

    Ok(AR2FeatureSetT { list })
}

// Re-export AR2ImageT for use in tests below.
#[allow(unused_imports)]
use super::image_set::AR2ImageT as _AR2ImageT;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::super::image_set::{ar2_gen_image_set, AR2ImageT};
    use super::*;

    /// Helper: build a minimal single-scale AR2ImageSetT directly.
    fn make_image_set(data: Vec<u8>, w: i32, h: i32, dpi: f32) -> AR2ImageSetT {
        AR2ImageSetT {
            scale: vec![AR2ImageT {
                img_bw: data,
                xsize: w,
                ysize: h,
                dpi,
            }],
        }
    }

    #[test]
    #[ignore] // Heavy computation — run explicitly with `cargo test --release -- --ignored`
    fn test_feature_map_produces_points() {
        let img = image::open("examples/Data/pinball.jpg").expect("failed to open test image");
        let gray = img.to_luma8();
        let w = gray.width() as i32;
        let h = gray.height() as i32;
        let data = gray.into_raw();

        let image_set = ar2_gen_image_set(&data, w, h, 1, 72.0).expect("ar2_gen_image_set failed");
        let result =
            ar2_gen_feature_map(&image_set, 16, 64, 2).expect("ar2_gen_feature_map failed");
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

    /// Lightweight version using a small synthetic image.
    #[test]
    fn test_feature_map_small_synthetic() {
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
        let image_set = make_image_set(data, w, h, 72.0);
        let result = ar2_gen_feature_map(&image_set, 6, 16, 2);
        assert!(result.is_ok());
    }

    #[test]
    fn test_feature_map_rejects_empty_image_set() {
        let empty_set = AR2ImageSetT { scale: vec![] };
        let err = ar2_gen_feature_map(&empty_set, 16, 64, 2);
        assert!(matches!(err, Err(Ar2Error::InvalidInput(_))));
    }

    /// Verify mindpi/maxdpi are computed correctly for a 3-level pyramid.
    #[test]
    fn test_feature_map_mindpi_maxdpi() {
        // Build a 3-scale set manually: 72, 57, 45 dpi
        let make = |dpi: f32| AR2ImageT {
            img_bw: vec![128u8; 32 * 32],
            xsize: 32,
            ysize: 32,
            dpi,
        };
        let image_set = AR2ImageSetT {
            scale: vec![make(72.0), make(57.0), make(45.0)],
        };
        let result = ar2_gen_feature_map(&image_set, 4, 8, 2).unwrap();
        // Verify that any produced feature points have sane dpi ranges
        for pts in &result.list {
            assert!(pts.mindpi > 0.0, "mindpi should be > 0");
            assert!(pts.maxdpi > pts.mindpi, "maxdpi should be > mindpi");
        }
    }

    #[test]
    fn test_make_template_out_of_bounds() {
        let data = vec![100u8; 10 * 10];
        let mut tmpl = vec![0.0f32; 529]; // (11+11+1)^2 = 529
                                          // Centre at (0,0) with ts1=11 → out of bounds
        assert!(make_template(&data, 10, 10, 0, 0, 11, 11, 0.0, &mut tmpl).is_none());
    }

    /// Helper: build a deterministic 64×64 pseudo-random image and template
    /// for SIMD correctness tests.
    fn make_deterministic_test_data() -> (Vec<u8>, Vec<f32>, f32) {
        let w: i32 = 64;
        let h: i32 = 64;
        let mut image = vec![0u8; (w * h) as usize];
        let mut s: u32 = 0x9E3779B9;
        for b in image.iter_mut() {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            *b = (s & 0xFF) as u8;
        }
        let ts1 = AR2_DEFAULT_TS1;
        let ts2 = AR2_DEFAULT_TS2;
        let patch = (ts1 + ts2 + 1) as usize;
        let mut template = vec![0.0f32; patch * patch];
        for (i, t) in template.iter_mut().enumerate() {
            *t = ((i as f32) * 0.123).sin();
        }
        let vlen: f32 = template.iter().map(|v| v * v).sum::<f32>().sqrt();
        (image, template, vlen)
    }

    /// Helper: assert SIMD result matches scalar within relative tolerance.
    fn assert_similarity_close(scalar: Option<f32>, simd: Option<f32>, cx: i32, cy: i32) {
        match (scalar, simd) {
            (Some(a), Some(b)) => {
                let tol = 1e-5 * a.abs().max(1.0);
                assert!(
                    (a - b).abs() <= tol,
                    "mismatch at ({cx},{cy}): scalar={a} simd={b} diff={}",
                    (a - b).abs()
                );
            }
            (None, None) => {}
            other => panic!("Option mismatch at ({cx},{cy}): {other:?}"),
        }
    }

    /// SSE4.1 SIMD `get_similarity` produces results within FP tolerance
    /// of the scalar path across all valid centres on a deterministic image.
    #[cfg(all(feature = "simd-x86-sse41", target_arch = "x86_64"))]
    #[test]
    fn test_get_similarity_sse41_tolerance() {
        if !is_x86_feature_detected!("sse4.1") {
            eprintln!("SSE4.1 not available — skipping");
            return;
        }

        let (image, template, vlen) = make_deterministic_test_data();
        let w: i32 = 64;
        let h: i32 = 64;
        let ts1 = AR2_DEFAULT_TS1;
        let ts2 = AR2_DEFAULT_TS2;

        let mut checked = 0usize;
        for cy in ts1..(h - ts2) {
            for cx in ts1..(w - ts2) {
                let scalar = get_similarity_scalar(&image, w, h, &template, vlen, ts1, ts2, cx, cy);
                let simd = unsafe {
                    get_similarity_sse41(&image, w, h, &template, vlen, ts1, ts2, cx, cy)
                };
                assert_similarity_close(scalar, simd, cx, cy);
                checked += 1;
            }
        }
        assert!(checked > 0, "no centres were tested");
    }

    /// AVX2/FMA SIMD `get_similarity` produces results within FP tolerance
    /// of the scalar path across all valid centres on a deterministic image.
    #[cfg(all(feature = "simd-x86-avx2", target_arch = "x86_64"))]
    #[test]
    fn test_get_similarity_avx2_tolerance() {
        if !is_x86_feature_detected!("avx2") || !is_x86_feature_detected!("fma") {
            eprintln!("AVX2/FMA not available — skipping");
            return;
        }

        let (image, template, vlen) = make_deterministic_test_data();
        let w: i32 = 64;
        let h: i32 = 64;
        let ts1 = AR2_DEFAULT_TS1;
        let ts2 = AR2_DEFAULT_TS2;

        let mut checked = 0usize;
        for cy in ts1..(h - ts2) {
            for cx in ts1..(w - ts2) {
                let scalar = get_similarity_scalar(&image, w, h, &template, vlen, ts1, ts2, cx, cy);
                let simd =
                    unsafe { get_similarity_avx2(&image, w, h, &template, vlen, ts1, ts2, cx, cy) };
                assert_similarity_close(scalar, simd, cx, cy);
                checked += 1;
            }
        }
        assert!(checked > 0, "no centres were tested");
    }

    /// Pipeline stability: `ar2_gen_feature_map` on pinball.jpg must produce
    /// the same per-level feature counts regardless of SIMD backend. This
    /// catches FP drift that might change feature selection at decision
    /// boundaries.
    #[test]
    #[ignore] // Loads pinball.jpg — run with `cargo test --release -- --ignored`
    fn test_feature_counts_stable_pinball() {
        let expected_counts: &[usize] = &[
            250, 250, 250, 250, 250, 223, 159, 102, 73, 43, 29, 12, 8, 4, 4, 4, 2, 1, 0,
        ];

        let img = image::open("examples/Data/pinball.jpg").expect("failed to open pinball.jpg");
        let gray = img.to_luma8();
        let w = gray.width() as i32;
        let h = gray.height() as i32;
        let data = gray.into_raw();

        let image_set = ar2_gen_image_set(&data, w, h, 1, 220.0).expect("ar2_gen_image_set failed");
        let result =
            ar2_gen_feature_map(&image_set, 10, 250, 2).expect("ar2_gen_feature_map failed");

        assert_eq!(
            result.list.len(),
            expected_counts.len(),
            "pyramid level count mismatch: got {} expected {}",
            result.list.len(),
            expected_counts.len()
        );

        for (i, (pts, &expected)) in result.list.iter().zip(expected_counts.iter()).enumerate() {
            assert_eq!(
                pts.coord.len(),
                expected,
                "feature count mismatch at level {i}: got {} expected {expected}",
                pts.coord.len()
            );
        }
    }
}
