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
#define VO_TEMPLATE 10
#define VO_ABI_VERSION 3
#define VO_META_FLOATS 13 /* R_NC[9], camera lever arm in NED[3], camera height */
enum {
    V_FX, V_FY, V_CX, V_CY, V_K1, V_K2, V_P1, V_P2, V_K3,
    V_FEATURES, V_FAST, V_MATCHES, V_INLIERS, V_RATIO, V_RAY,
    V_GATE, V_SEARCH_MIN, V_SEARCH_MAX, V_KLT_ITERS, V_KLT_ERROR,
    V_FB, V_KEY_DISTANCE, V_KEY_HEIGHT, V_KEY_ROTATION,
    V_TOPUP, V_MIN_EIG, V_KEY_RATIO, V_TOPUP_CELLS, V_TOPUP_POINTS,
    V_KEY_INTERVAL, V_WARP_ANGLE, V_WARP_HEIGHT, V_UNIQUENESS, VO_CONFIG_FLOATS
};
enum {
    O_STATUS, O_N, O_E, O_DN, O_DE, O_MATCHES, O_INLIERS,
    O_RATIO, O_CELLS, O_RESIDUAL, O_VAR_N, O_VAR_E,
    O_KEY_ID, O_KEY_CHANGED, O_BASE, O_REF_HEIGHT,
    O_SPARSE, O_REASON, O_EFFECTIVE, O_ATTEMPTS, O_SAD_TESTS,
    O_DETECT_CELLS, O_ADDED,
    VO_RESULT_FLOATS
};
enum { T_WARP, T_KLT, T_SAD, T_DETECT, T_FIT, T_COPY, VO_PHASES };
enum { VO_REASON_OK, VO_REASON_COUNT, VO_REASON_RATIO, VO_REASON_AMBIGUOUS, VO_REASON_INPUT, VO_REASON_INIT };
typedef struct { float x, y, score; } vo_point_t;
typedef struct {
    float meta[VO_META_FLOATS], position[2];
    int64_t stamp;
    vo_point_t points[VO_MAX_FEATURES];
    int count;
} vo_frame_t;
typedef struct {
    float au, av, bu, bv, ax, ay, bx, by, weight;
    float inverse[4]; /* Reuse the accepted warp for sparse uniqueness checks. */
} vo_pair_t;
typedef struct {
    /* Persistent images and feature data: owned, never camera DMA buffers. */
    uint8_t key_image[VO_PIXELS], previous_image[VO_PIXELS];
    vo_frame_t key, previous;
    float cfg[VO_CONFIG_FLOATS];
    uint32_t frame_id, key_id;
    int ready, grid_cursor, initializing, refresh_key;
    /* Reused per-frame workspace; no allocation during step. */
    vo_pair_t pairs[VO_MAX_FEATURES];
    vo_point_t current[VO_MAX_FEATURES];
    uint8_t inlier[VO_MAX_FEATURES];
    uint8_t patch[VO_TEMPLATE * VO_TEMPLATE];
    float gradient[2][VO_PATCH * VO_PATCH];
    /* Optional phase clock: disabled in production; uint32 wrap is intentional. */
    uint32_t (*clock_us)(void);
    uint32_t timing[VO_PHASES];
    float output[VO_RESULT_FLOATS];
} vo_context_t;

int vo_init(vo_context_t *ctx, const float *config);
void vo_reset(vo_context_t *ctx);
void vo_profile(vo_context_t *ctx, uint32_t (*clock_us)(void));
/* prediction is absolute N/E, used ONLY for matching. radius is in QVGA px.
 * step status: 2 measured, 1 initialized (no measurement), 0 lost,
 * -1 insufficient features, -2 invalid input. No state commit on failure. */
void vo_step(vo_context_t *ctx, const uint8_t *image, size_t stride,
             const float *meta, int64_t stamp, const float *prediction, float radius);
size_t vo_context_bytes(void);
const float *vo_get_result(const vo_context_t *ctx);
const uint32_t *vo_get_timings(const vo_context_t *ctx);
#endif
