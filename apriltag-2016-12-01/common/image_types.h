#ifndef _IMAGE_TYPES_H
#define _IMAGE_TYPES_H

#include <stdint.h>

// to support conversions between different types, we define all image
// types at once. Type-specific implementations can then #include this
// file, assured that the basic types of each image are known.

typedef struct image_u8 image_u8_t;
struct image_u8
{
    const int32_t width;
    const int32_t height;
    const int32_t stride;

    uint8_t *buf;
};

typedef struct image_u8x3 image_u8x3_t;
struct image_u8x3
{
    const int32_t width;//图片的宽度
    const int32_t height;//图片的高度
    const int32_t stride; // bytes per line表示每一列的bytes数

    uint8_t *buf;  //索引时候可通过 buf[y*stride + x]
};

typedef struct image_u8x4 image_u8x4_t;
struct image_u8x4
{
    const int32_t width;
    const int32_t height;
    const int32_t stride; // bytes per line

    uint8_t *buf;
};

typedef struct image_f32 image_f32_t;
struct image_f32
{
    const int32_t width;
    const int32_t height;
    const int32_t stride; // floats per line

    float *buf; // indexed as buf[y*stride + x]
};

typedef struct image_u32 image_u32_t;
struct image_u32
{
    const int32_t width;
    const int32_t height;
    const int32_t stride; // int32_ts per line

    uint32_t *buf; // indexed as buf[y*stride + x]
};

#endif
