#ifndef RM_VO_FAST_H
#define RM_VO_FAST_H
#include <stdint.h>
/* p must have valid radius-three neighbours at all supplied offsets. */
int vo_fast_test(const uint8_t *p, const int offsets[16], int threshold);
#endif
