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

#ifdef __cplusplus
}
#endif

#endif /* KPM_C_API_H */
