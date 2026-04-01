#include "kpm_c_api.h"
#include <facade/visual_database_facade.h>
#include <cstdlib>
#include <cstring>
#include <vector>

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

} // extern "C"
