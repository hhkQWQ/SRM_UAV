#ifndef RM_VO_CORE_H
#define RM_VO_CORE_H
#include <stddef.h>
#include <stdint.h>

/* Portable single-core QVGA core. No driver, interpreter or allocator dependency.
 * All images are GRAY8. Offsets/strides are size_t, never uint16_t. */
#define VO_WIDTH 320
#define VO_HEIGHT 240
#define VO_PIXELS (VO_WIDTH * VO_HEIGHT)
#define VO_MAX_FEATURES 64
#define VO_PATCH 8
#define VO_META_FLOATS 13 /* R_NC[9], camera lever arm in NED[3], camera height */
enum {
    V_FX, V_FY, V_CX, V_CY, V_K1, V_K2, V_P1, V_P2, V_K3,
    V_FEATURES, V_FAST, V_MATCHES, V_INLIERS, V_RATIO, V_CELLS, V_RAY,
    V_GATE, V_YAW, V_SEARCH_MIN, V_SEARCH_MAX, V_KLT_ITERS, V_KLT_ERROR,
    V_FB, V_KEY_DISTANCE, V_KEY_HEIGHT, V_KEY_ROTATION, V_NEAR_FLOW,
    V_TOPUP, V_MIN_EIG, V_RANSAC_ITERS, V_KEY_RATIO, VO_CONFIG_FLOATS
};
enum {
    O_STATUS, O_N, O_E, O_DN, O_DE, O_YAW, O_MATCHES, O_INLIERS,
    O_RATIO, O_CELLS, O_SPREAD, O_RESIDUAL, O_VAR_N, O_VAR_E,
    O_KEY_ID, O_KEY_CHANGED, O_BASE, O_REF_HEIGHT, O_REF_DT,
    O_FEATURES, O_KLT, O_SAD, O_ITERATIONS, VO_RESULT_FLOATS
};
typedef struct { float x, y, score; } vo_point_t;
typedef struct {
    float meta[VO_META_FLOATS], position[2];
    int64_t stamp;
    vo_point_t points[VO_MAX_FEATURES];
    int count;
} vo_frame_t;
typedef struct { float au, av, bu, bv, ax, ay, bx, by; } vo_pair_t;
typedef struct {
    /* Persistent images and feature data: owned, never camera DMA buffers. */
    uint8_t key_image[VO_PIXELS], previous_image[VO_PIXELS];
    vo_frame_t key, previous;
    float cfg[VO_CONFIG_FLOATS];
    uint32_t frame_id, key_id, random;
    int ready;
    /* Reused per-frame workspace; no allocation during step/recover. */
    vo_pair_t pairs[VO_MAX_FEATURES];
    vo_point_t current[VO_MAX_FEATURES];
    uint8_t inlier[VO_MAX_FEATURES];
    uint8_t patch[VO_PATCH * VO_PATCH];
    float output[VO_RESULT_FLOATS];
} vo_context_t;

int vo_init(vo_context_t *ctx, const float *config);
void vo_reset(vo_context_t *ctx);
/* prediction is absolute N/E, used ONLY for matching. radius is in QVGA px.
 * step status: 2 measured, 1 initialized (no measurement), 0 lost,
 * -1 insufficient features, -2 invalid input. No state commit on failure. */
void vo_step(vo_context_t *ctx, const uint8_t *image, size_t stride,
             const float *meta, int64_t stamp, const float *prediction, float radius);
/* ORB recovery supplies packed au,av,bu,bv pairs against the retained key. */
void vo_recover(vo_context_t *ctx, const uint8_t *image, size_t stride,
                const float *meta, int64_t stamp, const float *pairs, size_t count);
size_t vo_context_bytes(void);
const float *vo_get_result(const vo_context_t *ctx);
#endif
