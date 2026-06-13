/*
 *  descriptor.rs
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

//! FREAK (Fast Retina Keypoint) descriptor extraction.
//!
//! Ported from `WebARKitLib/.../matchers/freak.{h,cpp}` and `matchers/freak84-inline.h`.
//!
//! The descriptor samples the Gaussian scale-space pyramid at a rotated set of receptors
//! (6 rings × 6 points + 1 center = 37 receptors), compares all `C(37, 2) = 666` pairs,
//! and packs the bits into a 96-byte slot. The first 84 bytes carry the packed bits;
//! the trailing 12 bytes are zero padding. The padding matches the C++
//! `BinaryFeatureStore` layout used by M7's `hamming_distance_96`.
//!
//! C equivalent: `vision::FREAKExtractor::extract`, `ExtractFREAK84`,
//! `SamplePyramidFREAK84`, `CompareFREAK84`.

use super::gaussian_pyramid::GaussianScaleSpacePyramid;
use super::hough::FeaturePoint;
use super::interpolate::{bilinear_downsample_point, bilinear_interpolate_f32};

// ─────────────────────────────────────────────────────────────────────
// Public constants
// ─────────────────────────────────────────────────────────────────────

/// Storage size in bytes per FREAK descriptor.
///
/// Matches C++ `setNumBytesPerFeature(96)` in `FREAKExtractor::extract`.
/// Of these 96 bytes, the first [`FREAK_DESCRIPTOR_DATA_BYTES`] = 84 carry
/// the packed bits (`C(37, 2) = 666` pairwise comparisons, LSB-first);
/// the trailing 12 bytes are zero padding required for compatibility with
/// M7's `hamming_distance_96` and the C++ `BinaryFeatureStore` layout.
pub const FREAK_DESCRIPTOR_BYTES: usize = 96;

// ─────────────────────────────────────────────────────────────────────
// Private constants
// ─────────────────────────────────────────────────────────────────────

/// Number of bytes that carry actual descriptor data within each 96-byte
/// slot. Bytes `FREAK_DESCRIPTOR_DATA_BYTES..FREAK_DESCRIPTOR_BYTES` are
/// zero padding.
const FREAK_DESCRIPTOR_DATA_BYTES: usize = 84;

/// Number of (sample[i] < sample[j]) comparisons per descriptor.
/// Matches the C++ `ASSERT(pos == 666, "...")` in `CompareFREAK84`.
const FREAK_NUM_PAIRS: usize = 666; // = 37 * 36 / 2

/// Number of receptors per descriptor: 1 center + 6 rings × 6 points.
const FREAK_NUM_RECEPTORS: usize = 37;

/// Scale multiplier applied to keypoint scale to obtain the similarity
/// transform scale. Matches C++ `mExpansionFactor = 7` in
/// `FREAKExtractor::FREAKExtractor()`.
const FREAK_EXPANSION_FACTOR: f32 = 7.0;

/// Sigma values for the center receptor and each of the 6 rings.
/// Ported verbatim from `freak84-inline.h`.
const FREAK_SIGMA_CENTER: f32 = 0.100_000;
const FREAK_SIGMA_RING0: f32 = 0.175_000;
const FREAK_SIGMA_RING1: f32 = 0.250_000;
const FREAK_SIGMA_RING2: f32 = 0.325_000;
const FREAK_SIGMA_RING3: f32 = 0.400_000;
const FREAK_SIGMA_RING4: f32 = 0.475_000;
const FREAK_SIGMA_RING5: f32 = 0.550_000;

/// `(x, y)` coordinates for the 6 receptors in each ring, in canonical
/// (pre-similarity-transform) units. Each ring has 6 receptors × 2 coords
/// = 12 floats. Ported verbatim from `freak84-inline.h`.
const FREAK_POINTS_RING0: [f32; 12] = [
    0.000_000, 0.362_783, -0.314_179, 0.181_391, -0.314_179, -0.181_391, -0.000_000, -0.362_783,
    0.314_179, -0.181_391, 0.314_179, 0.181_391,
];
const FREAK_POINTS_RING1: [f32; 12] = [
    -0.595_502, 0.000_000, -0.297_751, -0.515_720, 0.297_751, -0.515_720, 0.595_502, -0.000_000,
    0.297_751, 0.515_720, -0.297_751, 0.515_720,
];
const FREAK_POINTS_RING2: [f32; 12] = [
    -0.000_000, -0.741_094, 0.641_806, -0.370_547, 0.641_806, 0.370_547, 0.000_000, 0.741_094,
    -0.641_806, 0.370_547, -0.641_806, -0.370_547,
];
const FREAK_POINTS_RING3: [f32; 12] = [
    0.847_306, -0.000_000, 0.423_653, 0.733_789, -0.423_653, 0.733_789, -0.847_306, 0.000_000,
    -0.423_653, -0.733_789, 0.423_653, -0.733_789,
];
const FREAK_POINTS_RING4: [f32; 12] = [
    0.000_000, 0.930_969, -0.806_243, 0.465_485, -0.806_243, -0.465_485, -0.000_000, -0.930_969,
    0.806_243, -0.465_485, 0.806_243, 0.465_485,
];
const FREAK_POINTS_RING5: [f32; 12] = [
    -1.000_000, 0.000_000, -0.500_000, -0.866_025, 0.500_000, -0.866_025, 1.000_000, -0.000_000,
    0.500_000, 0.866_025, -0.500_000, 0.866_025,
];

// ─────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────

/// Compute FREAK descriptors for each keypoint and append to `out`.
///
/// After the call, `out.len()` has grown by
/// `keypoints.len() * FREAK_DESCRIPTOR_BYTES`. Each 96-byte slot is
/// zero-initialized; the first 84 bytes are filled with 666 packed bit
/// comparisons; bytes 84..96 remain zero (matches C++
/// `BinaryFeatureStore` layout).
///
/// Caller responsibility: `keypoint.angle` should be set by
/// `OrientationAssignment` (M8-3 via `DoGScaleInvariantDetector::detect`
/// with `find_orientation = true`). If `angle == 0.0`, the descriptor is
/// rotation-variant.
///
/// C equivalent: `vision::FREAKExtractor::extract` →
/// `ExtractFREAK84` (per-keypoint) → `SamplePyramidFREAK84` +
/// `CompareFREAK84`.
pub fn extract_freak_descriptors(
    pyramid: &GaussianScaleSpacePyramid,
    keypoints: &[FeaturePoint],
    out: &mut Vec<u8>,
) {
    out.reserve(keypoints.len() * FREAK_DESCRIPTOR_BYTES);

    let mut samples = [0.0f32; FREAK_NUM_RECEPTORS];

    for kp in keypoints {
        sample_pyramid_freak84(&mut samples, pyramid, kp);

        // Reserve the 96-byte slot, zero-initialized.
        let start = out.len();
        out.resize(start + FREAK_DESCRIPTOR_BYTES, 0);
        // Pack 666 bits into the first 84 bytes. Bytes 84..96 stay zero.
        let desc = &mut out[start..start + FREAK_DESCRIPTOR_DATA_BYTES];
        let mut pos = 0usize;
        for i in 0..FREAK_NUM_RECEPTORS {
            for j in (i + 1)..FREAK_NUM_RECEPTORS {
                if samples[i] < samples[j] {
                    desc[pos / 8] |= 1u8 << (pos % 8);
                }
                pos += 1;
            }
        }
        debug_assert_eq!(pos, FREAK_NUM_PAIRS);
    }
}

// ─────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────

/// Sample the 37 receptors for one keypoint. Matches the C++ sample order
/// `ring5 → ring4 → ring3 → ring2 → ring1 → ring0 → center` from
/// `SamplePyramidFREAK84`. Reordering would produce a fundamentally
/// different bit pattern incompatible with the C++ baseline.
fn sample_pyramid_freak84(
    samples: &mut [f32; FREAK_NUM_RECEPTORS],
    pyramid: &GaussianScaleSpacePyramid,
    kp: &FeaturePoint,
) {
    // Similarity transform: clamped at 1.0 to match C++.
    let raw_scale = kp.scale * FREAK_EXPANSION_FACTOR;
    let transform_scale = if raw_scale < 1.0 { 1.0 } else { raw_scale };
    let cs = transform_scale * kp.angle.cos();
    let sn = transform_scale * kp.angle.sin();

    // Transform sigmas.
    let sc = FREAK_SIGMA_CENTER * transform_scale;
    let s0 = FREAK_SIGMA_RING0 * transform_scale;
    let s1 = FREAK_SIGMA_RING1 * transform_scale;
    let s2 = FREAK_SIGMA_RING2 * transform_scale;
    let s3 = FREAK_SIGMA_RING3 * transform_scale;
    let s4 = FREAK_SIGMA_RING4 * transform_scale;
    let s5 = FREAK_SIGMA_RING5 * transform_scale;

    // Sample in C++ order (ring5 → ring4 → … → ring0 → center).
    sample_ring(
        &mut samples[0..6],
        pyramid,
        s5,
        &FREAK_POINTS_RING5,
        kp,
        cs,
        sn,
    );
    sample_ring(
        &mut samples[6..12],
        pyramid,
        s4,
        &FREAK_POINTS_RING4,
        kp,
        cs,
        sn,
    );
    sample_ring(
        &mut samples[12..18],
        pyramid,
        s3,
        &FREAK_POINTS_RING3,
        kp,
        cs,
        sn,
    );
    sample_ring(
        &mut samples[18..24],
        pyramid,
        s2,
        &FREAK_POINTS_RING2,
        kp,
        cs,
        sn,
    );
    sample_ring(
        &mut samples[24..30],
        pyramid,
        s1,
        &FREAK_POINTS_RING1,
        kp,
        cs,
        sn,
    );
    sample_ring(
        &mut samples[30..36],
        pyramid,
        s0,
        &FREAK_POINTS_RING0,
        kp,
        cs,
        sn,
    );

    // Center receptor: single sample at the (untransformed) keypoint location.
    let (octave_c, scale_c) = pyramid.locate(sc);
    samples[36] = sample_pyramid_at(pyramid, kp.x, kp.y, octave_c, scale_c);
}

/// Sample the 6 receptors of one ring. All 6 share the same `(octave, scale)`
/// determined by `pyramid.locate(sigma)` (matches C++ — locating per receptor
/// would risk landing different receptors of the same ring at different
/// octaves due to FP variance, breaking C++ parity).
fn sample_ring(
    out: &mut [f32],
    pyramid: &GaussianScaleSpacePyramid,
    sigma: f32,
    ring: &[f32; 12],
    kp: &FeaturePoint,
    cs: f32,
    sn: f32,
) {
    let (octave, scale_idx) = pyramid.locate(sigma);
    for i in 0..6 {
        let px = ring[i * 2];
        let py = ring[i * 2 + 1];
        // S · (px, py): [cs·px − sn·py + kp.x ; sn·px + cs·py + kp.y]
        let rx = kp.x + cs * px - sn * py;
        let ry = kp.y + sn * px + cs * py;
        out[i] = sample_pyramid_at(pyramid, rx, ry, octave, scale_idx);
    }
}

/// Downsample fine-image `(x, y)` to octave-local coordinates, clip to
/// `[0, w-2] × [0, h-2]` (matches C++ `SampleReceptorBilinear`), and
/// sample with bilinear interpolation on the Gaussian pyramid level.
fn sample_pyramid_at(
    pyramid: &GaussianScaleSpacePyramid,
    x: f32,
    y: f32,
    octave: usize,
    scale: usize,
) -> f32 {
    let level = pyramid.level(octave, scale);
    let (xp, yp) = bilinear_downsample_point(x, y, octave as i32);
    let xc = xp.clamp(0.0, (level.cols as f32) - 2.0);
    let yc = yp.clamp(0.0, (level.rows as f32) - 2.0);
    bilinear_interpolate_f32(level, xc, yc)
}

// ═════════════════════════════════════════════════════════════════════
// Tests
// ═════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use purecv::core::Matrix;

    fn load_grayscale(path: &str) -> Matrix<u8> {
        let img = image::open(path).expect("load test image").to_luma8();
        let (w, h) = img.dimensions();
        Matrix::<u8>::from_vec(h as usize, w as usize, 1, img.into_raw())
    }

    fn build_pyramid(img: &Matrix<u8>) -> GaussianScaleSpacePyramid {
        let mut p = GaussianScaleSpacePyramid::new(3);
        p.build(img).expect("build pyramid");
        p
    }

    fn synthetic_keypoint(x: f32, y: f32, sigma: f32) -> FeaturePoint {
        FeaturePoint {
            x,
            y,
            angle: 0.0,
            scale: sigma,
            maxima: true,
        }
    }

    // ── Length / shape ────────────────────────────────────────────────

    #[test]
    #[cfg_attr(miri, ignore)] // #194: real-image FREAK extraction — too slow under Miri
    fn test_freak_descriptor_length_one_keypoint() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let pyr = build_pyramid(&img);
        let kps = vec![synthetic_keypoint(100.0, 100.0, 1.5)];
        let mut out = Vec::<u8>::new();
        extract_freak_descriptors(&pyr, &kps, &mut out);
        assert_eq!(out.len(), FREAK_DESCRIPTOR_BYTES);
        assert_eq!(out.len(), 96);
    }

    #[test]
    #[cfg_attr(miri, ignore)] // #194: real-image FREAK extraction — too slow under Miri
    fn test_freak_descriptor_length_multiple_keypoints() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let pyr = build_pyramid(&img);
        let kps: Vec<_> = (0..5)
            .map(|i| synthetic_keypoint(50.0 + 30.0 * i as f32, 100.0, 1.5))
            .collect();
        let mut out = Vec::<u8>::new();
        extract_freak_descriptors(&pyr, &kps, &mut out);
        assert_eq!(out.len(), 5 * FREAK_DESCRIPTOR_BYTES);
        assert_eq!(out.len(), 480);
    }

    #[test]
    #[cfg_attr(miri, ignore)] // #194: real-image FREAK extraction — too slow under Miri
    fn test_freak_descriptor_padding_bytes_are_zero() {
        // Bytes 84..96 of each descriptor must be zero (matches C++ store layout).
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let pyr = build_pyramid(&img);
        let kps = vec![synthetic_keypoint(150.0, 150.0, 1.5)];
        let mut out = Vec::<u8>::new();
        extract_freak_descriptors(&pyr, &kps, &mut out);
        for (i, &b) in out[FREAK_DESCRIPTOR_DATA_BYTES..FREAK_DESCRIPTOR_BYTES]
            .iter()
            .enumerate()
        {
            assert_eq!(
                b,
                0,
                "padding byte at offset {} must be zero",
                FREAK_DESCRIPTOR_DATA_BYTES + i
            );
        }
    }

    #[test]
    #[cfg_attr(miri, ignore)] // #194: real-image FREAK extraction — too slow under Miri
    fn test_freak_descriptor_empty_input() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let pyr = build_pyramid(&img);
        let mut out = Vec::<u8>::new();
        extract_freak_descriptors(&pyr, &[], &mut out);
        assert!(out.is_empty());
    }

    #[test]
    #[cfg_attr(miri, ignore)] // #194: real-image FREAK extraction (×2) — too slow under Miri
    fn test_freak_descriptor_is_reproducible() {
        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let pyr = build_pyramid(&img);
        let kps = vec![
            synthetic_keypoint(100.0, 100.0, 1.5),
            synthetic_keypoint(200.0, 150.0, 2.0),
        ];
        let mut a = Vec::<u8>::new();
        let mut b = Vec::<u8>::new();
        extract_freak_descriptors(&pyr, &kps, &mut a);
        extract_freak_descriptors(&pyr, &kps, &mut b);
        assert_eq!(a, b, "extraction must be reproducible byte-for-byte");
    }

    // ── Dual-mode: per-descriptor Hamming distance vs C++ ─────────────

    #[cfg(feature = "dual-mode")]
    extern "C" {
        fn webarkit_cpp_extract_freak_descriptors(
            src: *const u8,
            src_w: i32,
            src_h: i32,
            num_octaves: i32,
            keypoints: *const f32,
            num_keypoints: i32,
            dst_out: *mut u8,
            dst_capacity_bytes: i32,
        ) -> i32;
    }

    #[cfg(feature = "dual-mode")]
    fn cpp_extract_freak_descriptors(
        img: &Matrix<u8>,
        num_octaves: usize,
        keypoints: &[FeaturePoint],
    ) -> Vec<u8> {
        let kp_flat: Vec<f32> = keypoints
            .iter()
            .flat_map(|p| [p.x, p.y, p.angle, p.scale])
            .collect();
        let mut dst = vec![0u8; keypoints.len() * FREAK_DESCRIPTOR_BYTES];
        // SAFETY: pointers are valid for declared lengths; shim validates
        // dst_capacity_bytes >= num_keypoints * 96.
        let rc = unsafe {
            webarkit_cpp_extract_freak_descriptors(
                img.as_slice().as_ptr(),
                img.cols as i32,
                img.rows as i32,
                num_octaves as i32,
                kp_flat.as_ptr(),
                keypoints.len() as i32,
                dst.as_mut_ptr(),
                dst.len() as i32,
            )
        };
        assert_eq!(rc, 0, "C++ shim returned error {rc}");
        dst
    }

    #[cfg(feature = "dual-mode")]
    fn hamming_distance(a: &[u8], b: &[u8]) -> u32 {
        a.iter().zip(b).map(|(x, y)| (x ^ y).count_ones()).sum()
    }

    #[test]
    #[cfg(feature = "dual-mode")]
    fn test_freak_descriptors_match_cpp_baseline() {
        use crate::kpm::freak::detector::DoGScaleInvariantDetector;

        // Tolerance allows for ~2 bits of libm/FMA cross-platform variance
        // across 666 sign decisions per descriptor (~0.3%).
        //
        // Empirical local result (Windows MSVC, clean rebuild): all 10
        // descriptors match C++ byte-for-byte (Hamming = 0). The
        // tolerance is kept at 2 to absorb potential cross-platform
        // variance (Linux GCC / macOS Apple clang) before tightening
        // to exact match in a follow-up if CI shows consistent equality.
        const MAX_HAMMING_PER_DESCRIPTOR: u32 = 2;

        let img = load_grayscale("../../benchmarks/data/found.jpg");
        let num_octaves = 3;
        let pyr = build_pyramid(&img);
        let det = DoGScaleInvariantDetector::new(0.0, 10.0, 5000, true);

        // 1. Run Rust detection.
        let dog_points = det.detect(&pyr);
        let points: Vec<FeaturePoint> = dog_points.iter().map(FeaturePoint::from).collect();

        // 2. Take top-10 by |raw DoG score| (before projection).
        let mut indexed: Vec<(usize, f32)> = dog_points
            .iter()
            .enumerate()
            .map(|(i, p)| (i, p.score.abs()))
            .collect();
        indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        let top10: Vec<FeaturePoint> = indexed.iter().take(10).map(|&(i, _)| points[i]).collect();
        assert_eq!(top10.len(), 10, "expected >= 10 keypoints on found.jpg");

        // 3. Rust descriptors.
        let mut rust_descs = Vec::<u8>::new();
        extract_freak_descriptors(&pyr, &top10, &mut rust_descs);

        // 4. C++ descriptors via FFI shim (Rust supplies the keypoints).
        let cpp_descs = cpp_extract_freak_descriptors(&img, num_octaves, &top10);

        // 5. Per-descriptor Hamming distance.
        assert_eq!(rust_descs.len(), cpp_descs.len());
        for i in 0..10 {
            let rs = &rust_descs[i * FREAK_DESCRIPTOR_BYTES..(i + 1) * FREAK_DESCRIPTOR_BYTES];
            let cs = &cpp_descs[i * FREAK_DESCRIPTOR_BYTES..(i + 1) * FREAK_DESCRIPTOR_BYTES];
            let h = hamming_distance(rs, cs);
            assert!(
                h <= MAX_HAMMING_PER_DESCRIPTOR,
                "keypoint #{i}: Hamming distance {h} > {MAX_HAMMING_PER_DESCRIPTOR}"
            );
        }
    }
}
