/*
 *  kpm_c_api.h
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

#ifndef KPM_C_API_H
#define KPM_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct KpmOpaqueHandle KpmOpaqueHandle;

KpmOpaqueHandle* kpm_create(int xsize, int ysize);
void             kpm_destroy(KpmOpaqueHandle* handle);
int  kpm_add_ref_image(KpmOpaqueHandle* handle, const unsigned char* image,
                       int w, int h, float dpi, int page_no, int image_no);
int  kpm_query(KpmOpaqueHandle* handle, const unsigned char* gray_image,
               int xsize, int ysize,
               float pose_out[12], float* error_out, int* page_no_out);

/* ---- New accessors for kpm_matching orchestration (#36) ---- */

/* Feed pre-extracted FREAK features into the database (one image slot).
 * points:      flat array [x,y,angle,scale, ...] — 4 floats per point.
 * maxima:      array of ints (0 or 1) per point.
 * descriptors: packed binary descriptors, 96 bytes per point.
 * points_3d:   flat array [x,y,z, ...] — 3 floats per point.
 * Returns 0 on success, -1 on error. */
int kpm_add_freak_features(KpmOpaqueHandle* handle,
                           const float* points, const int* maxima,
                           const unsigned char* descriptors,
                           const float* points_3d,
                           int num_points,
                           int width, int height, int db_id);

/* Return the number of inlier matches from the most recent query. */
int kpm_get_inlier_count(KpmOpaqueHandle* handle);

/* Copy inlier match pairs into caller-provided arrays.
 * ins_out / ref_out must have room for at least kpm_get_inlier_count() ints.
 * Returns number of pairs written, or -1 on error. */
int kpm_get_inliers(KpmOpaqueHandle* handle, int* ins_out, int* ref_out);

/* Return the number of feature points detected in the most recent query. */
int kpm_get_query_feature_count(KpmOpaqueHandle* handle);

/* Copy query feature points into caller-provided arrays.
 * x_out / y_out must have room for kpm_get_query_feature_count() floats.
 * Returns number written, or -1 on error. */
int kpm_get_query_feature_points(KpmOpaqueHandle* handle,
                                 float* x_out, float* y_out);

/* Return the number of 3D feature points for the given image_id. */
int kpm_get_3d_feature_count(KpmOpaqueHandle* handle, int image_id);

/* Copy 3D feature points into caller-provided arrays.
 * x/y/z_out must have room for kpm_get_3d_feature_count() floats.
 * Returns number written, or -1 on error. */
int kpm_get_3d_feature_points(KpmOpaqueHandle* handle, int image_id,
                              float* x_out, float* y_out, float* z_out);

/* Return the matched image ID from the most recent query, or -1. */
int kpm_matched_id(KpmOpaqueHandle* handle);

/* ---- Feature extraction without database storage (#51) ---- */

/* Extract FREAK features and descriptors from an image without storing them.
 *
 * If x_out is NULL: returns the feature count and ignores all output arrays.
 * If x_out is not NULL: fills at most min(count, max_features) entries into
 *   each output array and returns the actual feature count.
 *
 * Output array sizes (when x_out != NULL):
 *   x_out / y_out / angle_out / scale_out: max_features floats each.
 *   maxima_out: max_features ints.
 *   desc_out:   max_features * 96 unsigned chars.
 *
 * Returns feature count (>= 0) on success, -1 on error. */
int kpm_extract_features(KpmOpaqueHandle* handle,
                         const unsigned char* image, int w, int h,
                         float* x_out, float* y_out,
                         float* angle_out, float* scale_out,
                         int* maxima_out,
                         unsigned char* desc_out,
                         int max_features);

/* ---- Dual-mode validation: math function bridges (Milestone 6, #63) ---- */

/* Thin wrappers around vision::fastatan2, vision::fastsqrt1, vision::fastexp6
 * from FreakMatcher/math/math_utils.h. Used by Rust dual-mode tests to verify
 * that the pure-Rust ports in crates/core/src/kpm/freak/math.rs produce the
 * same outputs as the C++ baseline across a sweep of inputs.
 *
 * These are always exposed when ffi-backend is built; they are zero-cost when
 * unused (link-time dead-code elimination removes them). */
float webarkit_cpp_fast_atan2(float y, float x);
float webarkit_cpp_fast_sqrt1(float x);
float webarkit_cpp_fast_exp6_f32(float x);

/* ---- Dual-mode validation: linear-algebra bridges (Milestone 6, #64) ---- */

/* Thin wrappers around vision::SolveLinearSystem2x2,
 * vision::SolveSymmetricLinearSystem3x3, vision::SolveNullVector8x9Destructive
 * from FreakMatcher/math/linear_algebra.h and linear_solvers.h. Used by the
 * M6-2 dual-mode tests to validate the pure-Rust ports against the C++
 * baseline.
 *
 * Return 1 on success, 0 on failure (mapping C++ bool to int for portability
 * across the C ABI). */
int webarkit_cpp_solve_linear_system_2x2(float x[2], const float a[4], const float b[2]);
int webarkit_cpp_solve_symmetric_linear_system_3x3(float x[3], const float a[9], const float b[3]);
int webarkit_cpp_solve_null_vector_8x9_destructive(float x[9], float a[72]);

/* ---- Dual-mode validation: homography bridges (Milestone 6, #65) ---- */

/* Thin wrappers around the homography pipeline in
 * homography_estimation/robust_homography.h. Used by the M6-3 dual-mode
 * tests to validate the pure-Rust ports against the C++ baseline.
 *
 * `webarkit_cpp_mat3_exp_pade_via_eigen` calls Eigen's MatrixExp
 * (`eigenMat.exp()`) and writes the 9-element row-major result into `out`.
 * This is the ground-truth oracle that `mat3_exp_pade` is validated against.
 *
 * `webarkit_cpp_preemptive_robust_homography` runs only the RANSAC step
 * (no IRLS polish). `webarkit_cpp_robust_homography_find` runs the full
 * `RobustHomography<float>::find()` (RANSAC + polish).
 *
 * Return 1 on success, 0 on failure. */
void webarkit_cpp_mat3_exp_pade_via_eigen(float out[9], const float in[9]);

int webarkit_cpp_preemptive_robust_homography(
    float h[9],
    const float* p,
    const float* q,
    int num_points,
    float scale,
    int num_hypotheses,
    int max_trials,
    int chunk_size);

int webarkit_cpp_robust_homography_find(
    float h[9],
    const float* p,
    const float* q,
    int num_points,
    float scale,
    int num_hypotheses,
    int max_trials,
    int chunk_size);

/* ---- Dual-mode validation: feature matcher bridges (Milestone 7, #111) ---- */

/* Thin wrappers around vision::BinaryFeatureMatcher<96>::match() variants.
 * Used by M7-3 dual-mode tests to validate the pure-Rust matcher against
 * the C++ baseline.
 *
 * Inputs (common to all 3 variants):
 *   query_descs:  num_query * 96 bytes packed (FREAK descriptors)
 *   query_points: num_query * 4 floats (x, y, angle, scale per point)
 *   query_maxima: num_query ints (0 or 1)
 *   ref_descs / ref_points / ref_maxima: same for reference store
 *   threshold:    ratio test threshold (0.7 == C++ default)
 *
 * Outputs:
 *   ins_out, ref_out: caller-provided arrays of size >= num_query.
 *     Filled with the match pairs in order.
 *
 * Returns: number of matches found (>= 0), or -1 on error. */
int webarkit_cpp_match_features_brute(
    const unsigned char* query_descs, const float* query_points,
    const int* query_maxima, int num_query,
    const unsigned char* ref_descs, const float* ref_points,
    const int* ref_maxima, int num_ref,
    float threshold,
    int* ins_out, int* ref_out);

int webarkit_cpp_match_features_indexed(
    const unsigned char* query_descs, const float* query_points,
    const int* query_maxima, int num_query,
    const unsigned char* ref_descs, const float* ref_points,
    const int* ref_maxima, int num_ref,
    float threshold,
    int* ins_out, int* ref_out);

int webarkit_cpp_match_features_guided(
    const unsigned char* query_descs, const float* query_points,
    const int* query_maxima, int num_query,
    const unsigned char* ref_descs, const float* ref_points,
    const int* ref_maxima, int num_ref,
    const float* h, float tr, float threshold,
    int* ins_out, int* ref_out);

/* ---- Dual-mode validation: PRNG bridges (Milestone 7, #116) ---- */

/* Thin wrappers around vision::FastRandom and vision::ArrayShuffle from
 * math/rand.h. Used by the M7 ArrayShuffle parity tests to verify that the
 * pure-Rust PRNG produces a byte-identical sequence to the C++ baseline.
 *
 * webarkit_cpp_fast_random:
 *   Mutates *seed in place using the LCG step; returns a 15-bit value in
 *   [0, 32767].
 *
 * webarkit_cpp_array_shuffle:
 *   Shuffles the first sample_size elements of v[] using FastRandom.
 *   Both v and seed are mutated. */
int  webarkit_cpp_fast_random(int* seed);
void webarkit_cpp_array_shuffle(int* v, int pop_size, int sample_size, int* seed);

#ifdef __cplusplus
}
#endif

#endif /* KPM_C_API_H */
