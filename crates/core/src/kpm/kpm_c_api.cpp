/*
 *  kpm_c_api.cpp
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

// Standard library headers MUST come first: hamming.h (transitively included
// from the matchers/ headers below) uses std::numeric_limits without including
// <limits> itself. MSVC pulls it in transitively, but GCC on Linux is strict.
#include <cstdlib>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

#include "kpm_c_api.h"
#include <facade/visual_database_facade.h>
#include <matchers/feature_point.h>
#include <matchers/matcher_types.h>
#include <matchers/feature_store.h>
#include <matchers/feature_matcher.h>
#include <matchers/feature_matcher-inline.h>
#include <matchers/binary_hierarchical_clustering.h>
#include <math/math_utils.h>
#include <math/linear_algebra.h>
#include <math/linear_solvers.h>
#include <math/rand.h>
#include <homography_estimation/robust_homography.h>
#include <detectors/DoG_scale_invariant_detector.h>
#include <detectors/gaussian_scale_space_pyramid.h>
#include <framework/image.h>
#include <matchers/freak.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

struct KpmOpaqueHandle {
    vision::VisualDatabaseFacade* db;
    int xsize;
    int ysize;
    int next_image_id;
};

extern "C" {

KpmOpaqueHandle* kpm_create(int xsize, int ysize) {
    KpmOpaqueHandle* h = new (std::nothrow) KpmOpaqueHandle();
    if (!h) return nullptr;
    h->db = new (std::nothrow) vision::VisualDatabaseFacade();
    if (!h->db) {
        delete h;
        return nullptr;
    }
    h->xsize = xsize;
    h->ysize = ysize;
    h->next_image_id = 0;
    return h;
}

void kpm_destroy(KpmOpaqueHandle* handle) {
    if (!handle) return;
    delete handle->db;
    delete handle;
}

int kpm_add_ref_image(KpmOpaqueHandle* handle, const unsigned char* image,
                      int w, int h, float dpi, int page_no, int image_no) {
    if (!handle || !handle->db || !image || w <= 0 || h <= 0 || dpi <= 0.0f) {
        return -1;
    }

    // Extract FREAK features and descriptors from the reference image.
    std::vector<vision::FeaturePoint> featurePoints;
    std::vector<unsigned char> descriptors;
    handle->db->computeFreakFeaturesAndDescriptors(
        const_cast<unsigned char*>(image),
        static_cast<size_t>(w), static_cast<size_t>(h),
        featurePoints, descriptors);

    if (featurePoints.empty()) {
        return -1;
    }

    // Compute 3D world coordinates from pixel positions and DPI.
    // Convert pixel coords to millimeters, centered at image center.
    float cx = static_cast<float>(w) / 2.0f;
    float cy = static_cast<float>(h) / 2.0f;
    float px_to_mm = 25.4f / dpi;

    std::vector<vision::Point3d<float>> points3D(featurePoints.size());
    for (size_t i = 0; i < featurePoints.size(); i++) {
        points3D[i].x = (featurePoints[i].x - cx) * px_to_mm;
        points3D[i].y = (featurePoints[i].y - cy) * px_to_mm;
        points3D[i].z = 0.0f;
    }

    // Use page_no as the image_id for the visual database.
    int image_id = page_no;
    handle->db->addFreakFeaturesAndDescriptors(
        featurePoints, descriptors, points3D,
        static_cast<size_t>(w), static_cast<size_t>(h), image_id);

    handle->next_image_id++;
    return 0;
}

int kpm_query(KpmOpaqueHandle* handle, const unsigned char* gray_image,
              int xsize, int ysize,
              float pose_out[12], float* error_out, int* page_no_out) {
    if (!handle || !handle->db || !gray_image || xsize <= 0 || ysize <= 0 ||
        !pose_out || !error_out || !page_no_out) {
        return -1;
    }

    bool matched = handle->db->query(
        const_cast<unsigned char*>(gray_image),
        static_cast<size_t>(xsize), static_cast<size_t>(ysize));

    if (!matched) {
        *page_no_out = -1;
        *error_out = -1.0f;
        std::memset(pose_out, 0, 12 * sizeof(float));
        return -1;
    }

    // Copy the 3x3 homography matrix into the first 9 elements of pose_out.
    const float* geom = handle->db->matchedGeometry();
    if (geom) {
        std::memcpy(pose_out, geom, 9 * sizeof(float));
    } else {
        std::memset(pose_out, 0, 9 * sizeof(float));
    }
    // Zero the remaining 3 elements.
    pose_out[9] = 0.0f;
    pose_out[10] = 0.0f;
    pose_out[11] = 0.0f;

    *page_no_out = handle->db->matchedId();
    *error_out = 0.0f;

    return 0;
}

// ---- New accessors for kpm_matching orchestration (#36) ----

int kpm_add_freak_features(KpmOpaqueHandle* handle,
                           const float* points, const int* maxima,
                           const unsigned char* descriptors,
                           const float* points_3d,
                           int num_points,
                           int width, int height, int db_id) {
    if (!handle || !handle->db || !points || !maxima || !descriptors ||
        !points_3d || num_points <= 0 || width <= 0 || height <= 0) {
        return -1;
    }

    std::vector<vision::FeaturePoint> fps(num_points);
    std::vector<unsigned char> descs(num_points * 96);
    std::vector<vision::Point3d<float>> pts3d(num_points);

    for (int i = 0; i < num_points; i++) {
        fps[i] = vision::FeaturePoint(
            points[i * 4 + 0],   // x
            points[i * 4 + 1],   // y
            points[i * 4 + 2],   // angle
            points[i * 4 + 3],   // scale
            maxima[i] != 0       // maxima
        );
        pts3d[i].x = points_3d[i * 3 + 0];
        pts3d[i].y = points_3d[i * 3 + 1];
        pts3d[i].z = points_3d[i * 3 + 2];
    }
    std::memcpy(descs.data(), descriptors, num_points * 96);

    handle->db->addFreakFeaturesAndDescriptors(
        fps, descs, pts3d,
        static_cast<size_t>(width), static_cast<size_t>(height), db_id);

    return 0;
}

int kpm_get_inlier_count(KpmOpaqueHandle* handle) {
    if (!handle || !handle->db) return 0;
    return static_cast<int>(handle->db->inliers().size());
}

int kpm_get_inliers(KpmOpaqueHandle* handle, int* ins_out, int* ref_out) {
    if (!handle || !handle->db || !ins_out || !ref_out) return -1;
    const vision::matches_t& m = handle->db->inliers();
    for (size_t i = 0; i < m.size(); i++) {
        ins_out[i] = m[i].ins;
        ref_out[i] = m[i].ref;
    }
    return static_cast<int>(m.size());
}

int kpm_get_query_feature_count(KpmOpaqueHandle* handle) {
    if (!handle || !handle->db) return 0;
    return static_cast<int>(handle->db->getQueryFeaturePoints().size());
}

int kpm_get_query_feature_points(KpmOpaqueHandle* handle,
                                 float* x_out, float* y_out) {
    if (!handle || !handle->db || !x_out || !y_out) return -1;
    const auto& pts = handle->db->getQueryFeaturePoints();
    for (size_t i = 0; i < pts.size(); i++) {
        x_out[i] = pts[i].x;
        y_out[i] = pts[i].y;
    }
    return static_cast<int>(pts.size());
}

int kpm_get_3d_feature_count(KpmOpaqueHandle* handle, int image_id) {
    if (!handle || !handle->db) return 0;
    return static_cast<int>(handle->db->get3DFeaturePoints(image_id).size());
}

int kpm_get_3d_feature_points(KpmOpaqueHandle* handle, int image_id,
                              float* x_out, float* y_out, float* z_out) {
    if (!handle || !handle->db || !x_out || !y_out || !z_out) return -1;
    const auto& pts = handle->db->get3DFeaturePoints(image_id);
    for (size_t i = 0; i < pts.size(); i++) {
        x_out[i] = pts[i].x;
        y_out[i] = pts[i].y;
        z_out[i] = pts[i].z;
    }
    return static_cast<int>(pts.size());
}

int kpm_matched_id(KpmOpaqueHandle* handle) {
    if (!handle || !handle->db) return -1;
    return handle->db->matchedId();
}

int kpm_extract_features(KpmOpaqueHandle* handle,
                         const unsigned char* image, int w, int h,
                         float* x_out, float* y_out,
                         float* angle_out, float* scale_out,
                         int* maxima_out,
                         unsigned char* desc_out,
                         int max_features) {
    if (!handle || !handle->db || !image || w <= 0 || h <= 0) {
        return -1;
    }

    std::vector<vision::FeaturePoint> featurePoints;
    std::vector<unsigned char> descriptors;
    handle->db->computeFreakFeaturesAndDescriptors(
        const_cast<unsigned char*>(image),
        static_cast<size_t>(w), static_cast<size_t>(h),
        featurePoints, descriptors);

    int count = static_cast<int>(featurePoints.size());

    // Count-only mode: caller passes NULL for output arrays.
    if (x_out == nullptr) {
        return count;
    }

    int n = (count < max_features) ? count : max_features;
    for (int i = 0; i < n; i++) {
        x_out[i]      = featurePoints[i].x;
        y_out[i]      = featurePoints[i].y;
        angle_out[i]  = featurePoints[i].angle;
        scale_out[i]  = featurePoints[i].scale;
        maxima_out[i] = featurePoints[i].maxima ? 1 : 0;
    }
    if (desc_out && !descriptors.empty()) {
        std::memcpy(desc_out, descriptors.data(),
                    static_cast<size_t>(n) * 96);
    }

    return n;
}

/* ---- Dual-mode validation: math function bridges (Milestone 6, #63) ---- */

float webarkit_cpp_fast_atan2(float y, float x) {
    return vision::fastatan2(y, x);
}

float webarkit_cpp_fast_sqrt1(float x) {
    return vision::fastsqrt1(x);
}

float webarkit_cpp_fast_exp6_f32(float x) {
    return vision::fastexp6<float>(x);
}

/* ---- Dual-mode validation: linear-algebra bridges (Milestone 6, #64) ---- */

int webarkit_cpp_solve_linear_system_2x2(float x[2], const float a[4], const float b[2]) {
    return vision::SolveLinearSystem2x2<float>(x, a, b) ? 1 : 0;
}

int webarkit_cpp_solve_symmetric_linear_system_3x3(float x[3], const float a[9], const float b[3]) {
    return vision::SolveSymmetricLinearSystem3x3<float>(x, a, b) ? 1 : 0;
}

int webarkit_cpp_solve_null_vector_8x9_destructive(float x[9], float a[72]) {
    return vision::SolveNullVector8x9Destructive<float>(x, a) ? 1 : 0;
}

/* ---- Dual-mode validation: homography bridges (Milestone 6, #65) ---- */

/* Compute exp(M) for a 3x3 matrix using Eigen's MatrixExp.
 * This is the ground-truth oracle for `mat3_exp_pade` (the Rust Padé(3,3)
 * replacement that eliminates Eigen from the pure-Rust path).
 * Input/output: row-major 9-element float arrays. */
void webarkit_cpp_mat3_exp_pade_via_eigen(float out[9], const float in[9]) {
    Eigen::Matrix<float, 3, 3> m;
    // Eigen stores column-major by default; copy element-by-element so
    // that `m(row, col)` matches the row-major `in[row*3 + col]` layout.
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            m(row, col) = in[row * 3 + col];
        }
    }
    Eigen::Matrix<float, 3, 3> e = m.exp();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out[row * 3 + col] = e(row, col);
        }
    }
}

/* Run the RANSAC step of robust homography estimation (no IRLS polish).
 * Allocates the std::vector scratch buffers internally. */
int webarkit_cpp_preemptive_robust_homography(float h[9],
                                              const float* p,
                                              const float* q,
                                              int num_points,
                                              float scale,
                                              int num_hypotheses,
                                              int max_trials,
                                              int chunk_size) {
    if (!h || !p || !q || num_points < 4 || num_hypotheses <= 0) {
        return 0;
    }
    std::vector<float> hyp(static_cast<size_t>(9) * static_cast<size_t>(num_hypotheses));
    std::vector<int> tmp_i(static_cast<size_t>(num_points));
    std::vector<std::pair<float, int>> hyp_costs(static_cast<size_t>(num_hypotheses));

    bool ok = vision::PreemptiveRobustHomography<float>(
        h, p, q, num_points, /*test_points=*/nullptr, /*num_test_points=*/0,
        hyp, tmp_i, hyp_costs,
        scale, num_hypotheses, max_trials, chunk_size);
    return ok ? 1 : 0;
}

/* Run the full RobustHomography<float>::find pipeline (RANSAC + IRLS polish). */
int webarkit_cpp_robust_homography_find(float h[9],
                                        const float* p,
                                        const float* q,
                                        int num_points,
                                        float scale,
                                        int num_hypotheses,
                                        int max_trials,
                                        int chunk_size) {
    if (!h || !p || !q || num_points < 4 || num_hypotheses <= 0) {
        return 0;
    }
    vision::RobustHomography<float> estimator(scale, num_hypotheses, max_trials, chunk_size);
    return estimator.find(h, p, q, num_points) ? 1 : 0;
}

/* ---- Dual-mode validation: feature matcher bridges (Milestone 7, #111) ---- */

/* Helper: build a BinaryFeatureStore from flat C arrays (96-byte features). */
static void build_store(vision::BinaryFeatureStore& store,
                        const unsigned char* descs, const float* points,
                        const int* maxima, int n) {
    store.setNumBytesPerFeature(96);
    store.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; i++) {
        store.point(i) = vision::FeaturePoint(
            points[i * 4 + 0],
            points[i * 4 + 1],
            points[i * 4 + 2],
            points[i * 4 + 3],
            maxima[i] != 0);
        std::memcpy(store.feature(i), descs + i * 96, 96);
    }
}

/* Helper: copy matches into output arrays, returning count. */
static int copy_matches(const vision::matches_t& m, int* ins_out, int* ref_out) {
    for (size_t i = 0; i < m.size(); i++) {
        ins_out[i] = m[i].ins;
        ref_out[i] = m[i].ref;
    }
    return static_cast<int>(m.size());
}

int webarkit_cpp_match_features_brute(
    const unsigned char* query_descs, const float* query_points,
    const int* query_maxima, int num_query,
    const unsigned char* ref_descs, const float* ref_points,
    const int* ref_maxima, int num_ref,
    float threshold,
    int* ins_out, int* ref_out) {
    if (!query_descs || !query_points || !query_maxima || num_query <= 0 ||
        !ref_descs || !ref_points || !ref_maxima || num_ref <= 0 ||
        !ins_out || !ref_out) {
        return -1;
    }

    vision::BinaryFeatureStore q_store, r_store;
    build_store(q_store, query_descs, query_points, query_maxima, num_query);
    build_store(r_store, ref_descs, ref_points, ref_maxima, num_ref);

    vision::BinaryFeatureMatcher<96> matcher;
    matcher.setThreshold(threshold);
    matcher.match(&q_store, &r_store);

    return copy_matches(matcher.matches(), ins_out, ref_out);
}

int webarkit_cpp_match_features_indexed(
    const unsigned char* query_descs, const float* query_points,
    const int* query_maxima, int num_query,
    const unsigned char* ref_descs, const float* ref_points,
    const int* ref_maxima, int num_ref,
    float threshold,
    int* ins_out, int* ref_out) {
    if (!query_descs || !query_points || !query_maxima || num_query <= 0 ||
        !ref_descs || !ref_points || !ref_maxima || num_ref <= 0 ||
        !ins_out || !ref_out) {
        return -1;
    }

    vision::BinaryFeatureStore q_store, r_store;
    build_store(q_store, query_descs, query_points, query_maxima, num_query);
    build_store(r_store, ref_descs, ref_points, ref_maxima, num_ref);

    vision::BinaryHierarchicalClustering<96> index;
    index.build(r_store.features().data(), num_ref);

    vision::BinaryFeatureMatcher<96> matcher;
    matcher.setThreshold(threshold);
    matcher.match(&q_store, &r_store, index);

    return copy_matches(matcher.matches(), ins_out, ref_out);
}

int webarkit_cpp_match_features_guided(
    const unsigned char* query_descs, const float* query_points,
    const int* query_maxima, int num_query,
    const unsigned char* ref_descs, const float* ref_points,
    const int* ref_maxima, int num_ref,
    const float* h, float tr, float threshold,
    int* ins_out, int* ref_out) {
    if (!query_descs || !query_points || !query_maxima || num_query <= 0 ||
        !ref_descs || !ref_points || !ref_maxima || num_ref <= 0 ||
        !h || !ins_out || !ref_out) {
        return -1;
    }

    vision::BinaryFeatureStore q_store, r_store;
    build_store(q_store, query_descs, query_points, query_maxima, num_query);
    build_store(r_store, ref_descs, ref_points, ref_maxima, num_ref);

    vision::BinaryFeatureMatcher<96> matcher;
    matcher.setThreshold(threshold);
    matcher.match(&q_store, &r_store, h, tr);

    return copy_matches(matcher.matches(), ins_out, ref_out);
}

/* ---- Dual-mode validation: BHC bridge with Keyframe::buildIndex settings
 *      (Milestone 9, #146) ---- */

int webarkit_cpp_bhc_build_and_query_with_settings(
    const unsigned char* features, int num_features,
    int num_hypotheses, int num_centers,
    int max_nodes_to_pop, int min_features_per_node,
    const unsigned char* query_feat,
    int* out_indices) {
    if (!features || num_features <= 0 ||
        !query_feat || !out_indices ||
        num_hypotheses <= 0 || num_centers <= 0 || min_features_per_node <= 0) {
        return -1;
    }

    vision::BinaryHierarchicalClustering<96> bhc;
    bhc.setNumHypotheses(num_hypotheses);
    bhc.setNumCenters(num_centers);
    bhc.setMaxNodesToPop(max_nodes_to_pop);
    bhc.setMinFeaturesPerNode(min_features_per_node);

    bhc.build(features, num_features);
    bhc.query(query_feat);

    const std::vector<int>& indices = bhc.reverseIndex();
    int n = static_cast<int>(indices.size());
    if (n > num_features) n = num_features; // safety cap on caller-allocated buffer
    for (int i = 0; i < n; i++) {
        out_indices[i] = indices[i];
    }
    return n;
}

/* ---- Dual-mode validation: PRNG bridges (Milestone 7, #116) ---- */

/* Thin wrappers around vision::FastRandom and vision::ArrayShuffle from
 * math/rand.h. Used by the M7 ArrayShuffle parity tests to verify that the
 * pure-Rust PRNG produces a byte-identical sequence to the C++ baseline. */
int webarkit_cpp_fast_random(int* seed) {
    return vision::FastRandom(*seed);
}

void webarkit_cpp_array_shuffle(int* v, int pop_size, int sample_size, int* seed) {
    vision::ArrayShuffle<int>(v, pop_size, sample_size, *seed);
}

/* ---- Dual-mode validation: FREAKExtractor bridge (Milestone 8, #129) ---- */

int webarkit_cpp_extract_freak_descriptors(
    const unsigned char* src,
    int src_w,
    int src_h,
    int num_octaves,
    const float* keypoints,
    int num_keypoints,
    unsigned char* dst_out,
    int dst_capacity_bytes) {

    if (!src || (!keypoints && num_keypoints > 0) || !dst_out
        || src_w < 5 || src_h < 5 || num_octaves < 1 || num_keypoints < 0) {
        return 1;
    }
    // Verify all octaves fit the binomial filter (M8-2 invariant: each octave >= 5x5).
    {
        int w = src_w;
        int h = src_h;
        for (int o = 0; o < num_octaves; ++o) {
            if (w < 5 || h < 5) {
                return 2;
            }
            w >>= 1;
            h >>= 1;
        }
    }
    if (dst_capacity_bytes < num_keypoints * 96) {
        return 3;
    }

    try {
        vision::Image image(
            const_cast<unsigned char*>(src), vision::IMAGE_UINT8,
            src_w, src_h, src_w /* step */, 1);

        vision::BinomialPyramid32f pyr;
        pyr.alloc(src_w, src_h, num_octaves);
        pyr.build(image);

        std::vector<vision::FeaturePoint> points;
        points.reserve(static_cast<size_t>(num_keypoints));
        for (int i = 0; i < num_keypoints; ++i) {
            const float* k = &keypoints[i * 4];
            // maxima is unused by FREAK extraction; pass true.
            points.emplace_back(k[0], k[1], k[2], k[3], /*maxima=*/true);
        }

        vision::BinaryFeatureStore store;
        // store.setNumBytesPerFeature(96) is called inside extract().
        vision::FREAKExtractor extractor;
        extractor.extract(store, &pyr, points);

        if (num_keypoints > 0) {
            std::memcpy(
                dst_out,
                store.feature(0),
                static_cast<size_t>(num_keypoints) * 96);
        }
        return 0;
    } catch (...) {
        return 4;
    }
}

/* ---- Dual-mode validation: DoGScaleInvariantDetector bridge (Milestone 8, #128) ---- */

int webarkit_cpp_dog_detect_count(
    const unsigned char* src,
    int src_w,
    int src_h,
    int num_octaves,
    float laplacian_threshold,
    float edge_threshold,
    int max_num_feature_points,
    int find_orientation,
    int* count_out) {

    if (!src || !count_out || src_w < 5 || src_h < 5 || num_octaves < 1
        || edge_threshold <= 0.0f || max_num_feature_points <= 0) {
        return 1;
    }

    // Ensure all octaves are at least 5x5 (binomial filter requirement).
    {
        int w = src_w;
        int h = src_h;
        for (int o = 0; o < num_octaves; ++o) {
            if (w < 5 || h < 5) {
                return 2;
            }
            w >>= 1;
            h >>= 1;
        }
    }

    try {
        vision::Image image(
            const_cast<unsigned char*>(src), vision::IMAGE_UINT8,
            src_w, src_h, src_w /* step */, 1);

        vision::BinomialPyramid32f pyr;
        pyr.alloc(src_w, src_h, num_octaves);
        pyr.build(image);

        vision::DoGScaleInvariantDetector det;
        det.alloc(&pyr);
        det.setLaplacianThreshold(laplacian_threshold);
        det.setEdgeThreshold(edge_threshold);
        det.setMaxNumFeaturePoints(static_cast<size_t>(max_num_feature_points));
        det.setFindOrientation(find_orientation != 0);

        det.detect(&pyr);

        *count_out = static_cast<int>(det.features().size());
        return 0;
    } catch (...) {
        return 4;
    }
}

/* ---- Dual-mode validation: BinomialPyramid32f bridge (Milestone 8, #127) ---- */

int webarkit_cpp_binomial_pyramid_build_level(
    const unsigned char* src,
    int src_w,
    int src_h,
    int num_octaves,
    int target_octave,
    int target_scale,
    float* dst_out,
    int dst_capacity_floats) {

    if (!src || !dst_out || src_w < 5 || src_h < 5 || num_octaves < 1
        || target_octave < 0 || target_octave >= num_octaves
        || target_scale < 0 || target_scale >= 3) {
        return 1;
    }

    const int lvl_w = src_w >> target_octave;
    const int lvl_h = src_h >> target_octave;
    if (lvl_w < 5 || lvl_h < 5) {
        return 2;
    }
    if (dst_capacity_floats < lvl_w * lvl_h) {
        return 3;
    }

    try {
        vision::Image image(
            const_cast<unsigned char*>(src), vision::IMAGE_UINT8,
            src_w, src_h, src_w /* step */, 1);

        vision::BinomialPyramid32f pyr;
        pyr.alloc(src_w, src_h, num_octaves);
        pyr.build(image);

        const vision::Image& lvl = pyr.get(target_octave, target_scale);
        const float* lvl_ptr = reinterpret_cast<const float*>(lvl.get());
        const int lvl_step_floats = static_cast<int>(lvl.step() / sizeof(float));
        for (int row = 0; row < lvl_h; ++row) {
            std::memcpy(
                dst_out + row * lvl_w,
                lvl_ptr + row * lvl_step_floats,
                static_cast<size_t>(lvl_w) * sizeof(float));
        }
        return 0;
    } catch (...) {
        return 4;
    }
}

} // extern "C"
