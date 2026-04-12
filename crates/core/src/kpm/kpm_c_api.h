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

#ifdef __cplusplus
}
#endif

#endif /* KPM_C_API_H */
