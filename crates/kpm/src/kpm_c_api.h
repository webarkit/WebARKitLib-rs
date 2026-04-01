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
