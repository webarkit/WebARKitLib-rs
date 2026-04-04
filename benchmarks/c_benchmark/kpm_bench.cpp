/*
 *  kpm_bench.c
 *
 *  KPM/NFT matching performance benchmark.
 *  Runs kpmMatching on pinball-demo.jpg in a loop and reports timing.
 *
 *  Build:  cmake . && cmake --build .
 *  Run:    ./kpm_bench [iterations]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
extern "C" {
#include <AR/ar.h>
#include <AR/param.h>
}
#include <KPM/kpm.h>

/* stb_image for JPEG loading — implementation already in kpm_dump_fixtures.c,
 * but we need it here too as a separate translation unit. */
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

/* Paths relative to the benchmarks/c_benchmark/ directory */
#define CAMERA_PARAM_FILE   "../../benchmarks/data/camera_para.dat"
#define PINBALL_FSET3       "../../crates/kpm/examples/data/pinball"
#define PINBALL_DEMO_JPG    "../../crates/kpm/examples/data/pinball-demo.jpg"

#define DEFAULT_ITERATIONS  100

static ARUint8 *load_jpeg_as_luma(const char *path, int *out_w, int *out_h)
{
    int w, h, channels;
    unsigned char *rgb = stbi_load(path, &w, &h, &channels, 3);
    if (!rgb) {
        fprintf(stderr, "ERROR: stbi_load failed for %s: %s\n", path, stbi_failure_reason());
        return NULL;
    }

    ARUint8 *luma = (ARUint8 *)malloc(w * h);
    if (!luma) { stbi_image_free(rgb); return NULL; }

    for (int i = 0; i < w * h; i++) {
        int r = rgb[i * 3 + 0];
        int g = rgb[i * 3 + 1];
        int b = rgb[i * 3 + 2];
        luma[i] = (ARUint8)((r * 77 + g * 150 + b * 29) >> 8);
    }

    stbi_image_free(rgb);
    *out_w = w;
    *out_h = h;
    return luma;
}

int main(int argc, char *argv[])
{
    int iterations = DEFAULT_ITERATIONS;
    if (argc > 1) {
        iterations = atoi(argv[1]);
        if (iterations <= 0) iterations = DEFAULT_ITERATIONS;
    }

    ARParam         cparam;
    ARParamLT      *cparamLT = NULL;
    KpmHandle      *kpmHandle = NULL;
    KpmRefDataSet  *refDataSet = NULL;
    KpmResult      *kpmResult = NULL;
    int             kpmResultNum = 0;

    printf("=== KPM Performance Benchmark ===\n");
    printf("Iterations: %d\n\n", iterations);

    /* Load camera parameters */
    if (arParamLoad(CAMERA_PARAM_FILE, 1, &cparam) < 0) {
        fprintf(stderr, "ERROR: arParamLoad failed\n");
        return 2;
    }

    /* Load query image */
    int imgW, imgH;
    ARUint8 *lumaImage = load_jpeg_as_luma(PINBALL_DEMO_JPG, &imgW, &imgH);
    if (!lumaImage) return 2;
    printf("Image: %dx%d\n", imgW, imgH);

    arParamChangeSize(&cparam, imgW, imgH, &cparam);
    cparamLT = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET);
    if (!cparamLT) { free(lumaImage); return 2; }

    /* Create KPM handle */
    kpmHandle = kpmCreateHandle(cparamLT);
    if (!kpmHandle) { arParamLTFree(&cparamLT); free(lumaImage); return 2; }

    /* Load reference data */
    if (kpmLoadRefDataSet(PINBALL_FSET3, "fset3", &refDataSet) < 0) {
        fprintf(stderr, "ERROR: kpmLoadRefDataSet failed\n");
        kpmDeleteHandle(&kpmHandle);
        arParamLTFree(&cparamLT);
        free(lumaImage);
        return 2;
    }
    kpmChangePageNoOfRefDataSet(refDataSet, KpmChangePageNoAllPages, 0);
    kpmSetRefDataSet(kpmHandle, refDataSet);
    kpmDeleteRefDataSet(&refDataSet);

    printf("Setup complete. Starting benchmark...\n\n");

    /* Benchmark loop */
    int match_count = 0;
    clock_t start = clock();

    for (int i = 0; i < iterations; i++) {
        if (kpmMatching(kpmHandle, lumaImage) < 0) {
            fprintf(stderr, "WARNING: kpmMatching failed on iteration %d\n", i);
            continue;
        }
        kpmGetResult(kpmHandle, &kpmResult, &kpmResultNum);
        for (int j = 0; j < kpmResultNum; j++) {
            if (kpmResult[j].camPoseF == 0) {
                match_count++;
                break;
            }
        }
    }

    clock_t end = clock();
    double elapsed_ms = (double)(end - start) / CLOCKS_PER_SEC * 1000.0;

    printf("=== Results ===\n");
    printf("Total time   : %.1f ms\n", elapsed_ms);
    printf("Avg per iter : %.2f ms\n", elapsed_ms / iterations);
    printf("Matches      : %d / %d\n", match_count, iterations);

    /* Cleanup */
    free(lumaImage);
    kpmDeleteHandle(&kpmHandle);
    arParamLTFree(&cparamLT);

    return 0;
}
