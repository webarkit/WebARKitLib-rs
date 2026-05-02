/*
 *  dump_patt.c
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
 *  executable, regardless of the license terms of these independent modules,
 *  and to copy and distribute the resulting executable under terms of your
 *  choice, provided that you also meet, for each linked independent module,
 *  the terms and conditions of the license of that module. An independent
 *  module is a module which is neither derived from nor based on this
 *  library. If you modify this library, you may extend this exception to
 *  your version of the library, but you are not obligated to do so. If you
 *  do not wish to do so, delete this exception statement from your version.
 *
 *  Copyright 2026 WebARKit.
 *
 *  Author(s): Walter Perdan @kalwalt https://github.com/kalwalt
 *
 */

/*
 * Diagnostic harness for issue #103 (low CF from template matching).
 *
 * Runs `arDetectMarker` once on a luma raw image, then re-extracts the same
 * pattern patch via `arPattGetImage` using the contour data preserved in
 * `arHandle->markerInfo2[0]`. The extracted patch is dumped to disk as raw
 * bytes for byte-level comparison with the Rust port.
 *
 * NOTE: this is a sibling of `main.c`; it does NOT modify the timing
 * benchmark. Outputs:
 *   - <out_dir>/c_ext_patt.bin       raw ext_patt bytes (patt_size^2 * channels)
 *   - <out_dir>/c_ext_patt_meta.txt  patt_size, channels, mode, vertices, cf, id, dir
 */

#include <AR/ar.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Compatibility helpers required by some WebARKitLib builds. */
char *cat(const char *file, size_t *bufSize_p) {
  FILE *fp;
  size_t len;
  char *s;
  if (!file)
    return NULL;
  fp = fopen(file, "rb");
  if (!fp)
    return NULL;
  fseek(fp, 0, SEEK_END);
  len = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  if (!(s = (char *)malloc(len + 1))) {
    fclose(fp);
    return NULL;
  }
  if (fread(s, len, 1, fp) < 1 && len > 0) {
    free(s);
    fclose(fp);
    return NULL;
  }
  s[len] = '\0';
  fclose(fp);
  if (bufSize_p)
    *bufSize_p = len + 1;
  return s;
}
int test_d(const char *dir) { (void)dir; return 0; }
int mkdir_p(const char *path) { (void)path; return 0; }

static const char *pixfmt_name(AR_PIXEL_FORMAT f) {
  switch (f) {
  case AR_PIXEL_FORMAT_RGB:  return "AR_PIXEL_FORMAT_RGB";
  case AR_PIXEL_FORMAT_BGR:  return "AR_PIXEL_FORMAT_BGR";
  case AR_PIXEL_FORMAT_RGBA: return "AR_PIXEL_FORMAT_RGBA";
  case AR_PIXEL_FORMAT_BGRA: return "AR_PIXEL_FORMAT_BGRA";
  case AR_PIXEL_FORMAT_ABGR: return "AR_PIXEL_FORMAT_ABGR";
  case AR_PIXEL_FORMAT_ARGB: return "AR_PIXEL_FORMAT_ARGB";
  case AR_PIXEL_FORMAT_MONO: return "AR_PIXEL_FORMAT_MONO";
  default:                   return "<unknown>";
  }
}

static const char *mode_name(int m) {
  switch (m) {
  case AR_TEMPLATE_MATCHING_COLOR:                      return "AR_TEMPLATE_MATCHING_COLOR";
  case AR_TEMPLATE_MATCHING_MONO:                       return "AR_TEMPLATE_MATCHING_MONO";
  case AR_MATRIX_CODE_DETECTION:                        return "AR_MATRIX_CODE_DETECTION";
  case AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX:           return "AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX";
  case AR_TEMPLATE_MATCHING_MONO_AND_MATRIX:            return "AR_TEMPLATE_MATCHING_MONO_AND_MATRIX";
  default:                                              return "<unknown>";
  }
}

int main(int argc, char *argv[]) {
  if (argc < 5) {
    printf("Usage: %s <camera_para.dat> <patt.hiro> <luma.raw> <out_dir> [width] [height]\n",
           argv[0]);
    printf("  out_dir defaults expected: ../data\n");
    return 1;
  }

  const char *cparam_name = argv[1];
  const char *patt_name   = argv[2];
  const char *luma_name   = argv[3];
  const char *out_dir     = argv[4];
  int width  = (argc > 5) ? atoi(argv[5]) : 429;
  int height = (argc > 6) ? atoi(argv[6]) : 317;

  /* ---- ARParam + ARHandle setup, mirrors main.c ---- */
  ARParam cparam;
  if (arParamLoad(cparam_name, 1, &cparam) < 0) {
    fprintf(stderr, "ERROR: arParamLoad failed for %s\n", cparam_name);
    return 1;
  }
  arParamChangeSize(&cparam, width, height, &cparam);
  ARParamLT *cparamLT = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET);

  ARHandle *arHandle = arCreateHandle(cparamLT);
  if (!arHandle) {
    fprintf(stderr, "ERROR: arCreateHandle failed\n");
    return 1;
  }
  arHandle->arPixelFormat = AR_PIXEL_FORMAT_MONO;
  /* arPatternDetectionMode is left at the default (AR_TEMPLATE_MATCHING_COLOR), which
     matches the simple Rust example; do not override. */

  ARPattHandle *pattHandle = arPattCreateHandle();
  if (!pattHandle || arPattLoad(pattHandle, patt_name) < 0) {
    fprintf(stderr, "ERROR: arPattLoad failed for %s\n", patt_name);
    return 1;
  }
  arPattAttach(arHandle, pattHandle);

  /* ---- Load raw luma image ---- */
  size_t data_size = (size_t)width * (size_t)height;
  unsigned char *luma_buffer = (unsigned char *)malloc(data_size);
  FILE *f = fopen(luma_name, "rb");
  if (!f || fread(luma_buffer, 1, data_size, f) != data_size) {
    fprintf(stderr, "ERROR: could not read %zu bytes from %s\n", data_size, luma_name);
    if (f) fclose(f);
    free(luma_buffer);
    return 1;
  }
  fclose(f);

  AR2VideoBufferT vbuf;
  memset(&vbuf, 0, sizeof(vbuf));
  vbuf.buff     = luma_buffer;
  vbuf.buffLuma = luma_buffer;
  vbuf.fillFlag = 1;

  /* ---- Run detection once ---- */
  if (arDetectMarker(arHandle, &vbuf) < 0) {
    fprintf(stderr, "ERROR: arDetectMarker failed\n");
    return 1;
  }

  int marker_num = arGetMarkerNum(arHandle);
  if (marker_num <= 0) {
    fprintf(stderr, "ERROR: no markers detected (marker_num=%d)\n", marker_num);
    return 1;
  }
  ARMarkerInfo *minfo = arGetMarker(arHandle);
  /* Pick the first marker with id >= 0; otherwise fall back to index 0. */
  int sel = 0;
  for (int i = 0; i < marker_num; i++) {
    if (minfo[i].id >= 0) { sel = i; break; }
  }

  /* The candidate's contour data lives in markerInfo2[sel] (assumes marker_info[i]
     corresponds to markerInfo2[i] for our single-detection case — true when no
     candidates were dropped between detection stages). */
  ARMarkerInfo2 *m2 = &arHandle->markerInfo2[sel];

  int patt_size      = pattHandle->pattSize;
  int patt_max_size  = patt_size * 2;
  int channels       = (arHandle->arPatternDetectionMode == AR_TEMPLATE_MATCHING_COLOR) ? 3 : 1;
  size_t ext_len     = (size_t)patt_size * (size_t)patt_size * (size_t)channels;
  unsigned char *ext_patt = (unsigned char *)malloc(ext_len);
  memset(ext_patt, 0, ext_len);

  /* Re-extract using the same contour data the matcher used internally.
     If the prototype differs in your AR/ar.h variant, adjust here. */
  int rv = arPattGetImage(arHandle->arImageProcMode, arHandle->arPatternDetectionMode,
                          patt_size, patt_max_size,
                          luma_buffer, width, height, arHandle->arPixelFormat,
                          m2->x_coord, m2->y_coord, m2->vertex,
                          arHandle->pattRatio, ext_patt);
  if (rv < 0) {
    fprintf(stderr, "ERROR: arPattGetImage returned %d\n", rv);
    free(ext_patt);
    return 1;
  }

  /* ---- Write outputs ---- */
  char path_bin[1024], path_meta[1024];
  snprintf(path_bin,  sizeof(path_bin),  "%s/c_ext_patt.bin",       out_dir);
  snprintf(path_meta, sizeof(path_meta), "%s/c_ext_patt_meta.txt", out_dir);

  FILE *fbin = fopen(path_bin, "wb");
  if (!fbin) {
    fprintf(stderr, "ERROR: could not open %s for writing\n", path_bin);
    return 1;
  }
  fwrite(ext_patt, 1, ext_len, fbin);
  fclose(fbin);

  FILE *fmeta = fopen(path_meta, "w");
  if (!fmeta) {
    fprintf(stderr, "ERROR: could not open %s for writing\n", path_meta);
    return 1;
  }
  fprintf(fmeta, "patt_size = %d\n",         patt_size);
  fprintf(fmeta, "channels  = %d\n",         channels);
  fprintf(fmeta, "mode      = %s\n",         mode_name(arHandle->arPatternDetectionMode));
  fprintf(fmeta, "pixfmt    = %s\n",         pixfmt_name(arHandle->arPixelFormat));
  fprintf(fmeta, "xsize     = %d\n",         width);
  fprintf(fmeta, "ysize     = %d\n",         height);
  fprintf(fmeta, "patt_ratio = %.6f\n",      arHandle->pattRatio);
  fprintf(fmeta, "selected  = %d (of %d markers)\n", sel, marker_num);
  /* The 4 contour vertices used by arPattGetImage. */
  for (int i = 0; i < 4; i++) {
    int idx = m2->vertex[i];
    fprintf(fmeta, "vertex[%d] = (%d, %d)\n", i, m2->x_coord[idx], m2->y_coord[idx]);
  }
  /* Ideal-space marker corners (post-distortion-correction). */
  for (int i = 0; i < 4; i++) {
    fprintf(fmeta, "marker_vertex[%d] = (%.4f, %.4f)\n", i, minfo[sel].vertex[i][0], minfo[sel].vertex[i][1]);
  }
  fprintf(fmeta, "id        = %d\n",         minfo[sel].id);
  fprintf(fmeta, "id_patt   = %d\n",         minfo[sel].idPatt);
  fprintf(fmeta, "cf        = %.6f\n",       minfo[sel].cf);
  fprintf(fmeta, "cf_patt   = %.6f\n",       minfo[sel].cfPatt);
  fprintf(fmeta, "dir       = %d\n",         minfo[sel].dir);
  fclose(fmeta);

  printf("Wrote %s (%zu bytes)\n", path_bin, ext_len);
  printf("Wrote %s\n", path_meta);
  printf("cf=%.4f id=%d dir=%d\n", minfo[sel].cf, minfo[sel].id, minfo[sel].dir);

  /* Cleanup */
  free(ext_patt);
  free(luma_buffer);
  arPattDetach(arHandle);
  arPattDeleteHandle(pattHandle);
  arDeleteHandle(arHandle);
  arParamLTFree(&cparamLT);

  return 0;
}
