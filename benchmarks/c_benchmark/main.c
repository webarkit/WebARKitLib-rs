/*
 *  main.c
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
 * and to copy and distribute the resulting executable under terms of your
 * choice, provided that you also meet, for each linked independent module, the
 * terms and conditions of the license of that module. An independent module is
 * a module which is neither derived from nor based on this library. If you
 * modify this library, you may extend this exception to your version of the
 * library, but you are not obligated to do so. If you do not wish to do so,
 * delete this exception statement from your version.
 *
 *  Copyright 2026 WebARKit.
 *
 *  Author(s): Walter Perdan @kalwalt https://github.com/kalwalt
 *
 */

#include <AR/ar.h>
#include <AR/icp.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifdef _WIN32
#include <errno.h>
#include <windows.h>

double get_time_ms() {
  LARGE_INTEGER count, freq;
  QueryPerformanceCounter(&count);
  QueryPerformanceFrequency(&freq);
  return (double)count.QuadPart * 1000.0 / freq.QuadPart;
}
#else
#include <time.h>
double get_time_ms() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0;
}
#endif

// Compatibility function for arPattLoad which uses cat()
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

// More dummy functions for ARUtil
int test_d(const char *dir) { return 0; }
int mkdir_p(const char *path) { return 0; }

int main(int argc, char *argv[]) {
  if (argc < 4) {
    printf(
        "Usage: %s <camera_para.dat> <patt.hiro> <luma.raw> [width] [height]\n",
        argv[0]);
    return 1;
  }

  const char *cparam_name = argv[1];
  const char *patt_name = argv[2];
  const char *luma_name = argv[3];
  int width = (argc > 4) ? atoi(argv[4]) : 640;
  int height = (argc > 5) ? atoi(argv[5]) : 480;

  // Load Camera Parameters
  ARParam cparam;
  if (arParamLoad(cparam_name, 1, &cparam) < 0) {
    printf("Error: Could not load camera parameters %s\n", cparam_name);
    return 1;
  }
  arParamChangeSize(&cparam, width, height, &cparam);

  ARParamLT *cparamLT = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET);

  // Initialize AR Handle
  ARHandle *arHandle = arCreateHandle(cparamLT);
  arHandle->arPixelFormat = AR_PIXEL_FORMAT_MONO;

  // Pattern loading (Simplified for benchmark)
  ARPattHandle *pattHandle = arPattCreateHandle();
  if (arPattLoad(pattHandle, patt_name) < 0) {
    printf("Error: Could not load pattern %s\n", patt_name);
    return 1;
  }
  arPattAttach(arHandle, pattHandle);

  // Initialize AR3D Handle
  AR3DHandle *ar3DHandle = ar3DCreateHandle(&cparam);

  // Load Raw Luma Image
  size_t data_size = (size_t)width * height;
  unsigned char *luma_buffer = (unsigned char *)malloc(data_size);
  FILE *f = fopen(luma_name, "rb");
  if (!f) {
    printf("Error: Could not open %s\n", luma_name);
    return 1;
  }
  fread(luma_buffer, 1, data_size, f);
  fclose(f);

  AR2VideoBufferT video_buffer;
  video_buffer.buff = luma_buffer;
  video_buffer.buffLuma = luma_buffer;
  video_buffer.fillFlag = 1;

  // Benchmark loop
  const int iterations = 100;
  printf("Starting C benchmark (%d iterations)...\n", iterations);

  double start = get_time_ms();
  for (int i = 0; i < iterations; i++) {
    arDetectMarker(arHandle, &video_buffer);
    int num_markers = arGetMarkerNum(arHandle);
    if (num_markers > 0) {
      ARMarkerInfo *marker_info = arGetMarker(arHandle);
      // Find hiro marker (ID 0 usually)
      for (int j = 0; j < num_markers; j++) {
        if (marker_info[j].id == 0) {
          ARdouble trans[3][4];
          arGetTransMatSquare(ar3DHandle, &marker_info[j], 80.0, trans);
          break;
        }
      }
    }
  }
  double end = get_time_ms();

  printf("Detection + Pose total: %.2f ms (Avg: %.4f ms)\n", end - start,
         (end - start) / iterations);

  // Cleanup
  free(luma_buffer);
  arPattDetach(arHandle);
  arPattDeleteHandle(pattHandle);
  arDeleteHandle(arHandle);
  ar3DDeleteHandle(&ar3DHandle);
  arParamLTFree(&cparamLT);

  return 0;
}
