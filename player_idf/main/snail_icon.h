// Snail icon — 24×13 px, 1-bit, faces right (shell←, head→)
// palette[0]=white(bg), palette[1]=black(ink)
//
// Preview:
//   ...................█.█..   antennae
//   ...................█.█..   antennae
//   ..████████........████..   shell top arc + head top
//   .█........█.......█...█.   shell outer + head sides
//   .█..████..█.......█...█.   inner spiral arc
//   .█.█....█.█.......█.█.█.   inner spiral sides + eye
//   .█.█....█.█.......█...█.   inner spiral sides
//   .█..████..█.......█...█.   inner spiral arc + head sides
//   .█........█.......████..   shell bot + head bottom
//   ..████████..............   shell bottom arc
//   █████████████████████...   body/foot
//   █...................█...   body underside
//   .██..██.................   feet
#pragma once
#include <stdint.h>
#include <lvgl.h>

#define SNAIL_W      24
#define SNAIL_H      13
#define SNAIL_STRIDE  3   // ceil(24/8)

static const uint8_t snail_data[] = {
    // palette[0]=white(bg)  palette[1]=black(ink)
    0xFF,0xFF,0xFF,0xFF,  0x00,0x00,0x00,0xFF,
    // 24-wide rows, MSB-first, 3 bytes each:
    0x00, 0x00, 0x14,  // ...................█.█..
    0x00, 0x00, 0x14,  // ...................█.█..
    0x3F, 0xC0, 0x3C,  // ..████████........████..
    0x40, 0x20, 0x22,  // .█........█.......█...█.
    0x4F, 0x20, 0x22,  // .█..████..█.......█...█.
    0x50, 0xA0, 0x2A,  // .█.█....█.█.......█.█.█.
    0x50, 0xA0, 0x22,  // .█.█....█.█.......█...█.
    0x4F, 0x20, 0x22,  // .█..████..█.......█...█.
    0x40, 0x20, 0x3C,  // .█........█.......████..
    0x3F, 0xC0, 0x00,  // ..████████..............
    0xFF, 0xFF, 0xF8,  // █████████████████████...
    0x80, 0x00, 0x08,  // █...................█...
    0x66, 0x00, 0x00,  // .██..██.................
};

static const lv_image_dsc_t snail_dsc = {
    .header = {
        .magic  = LV_IMAGE_HEADER_MAGIC,
        .cf     = LV_COLOR_FORMAT_I1,
        .flags  = 0,
        .w      = SNAIL_W,
        .h      = SNAIL_H,
        .stride = SNAIL_STRIDE,
    },
    .data_size = sizeof(snail_data),
    .data      = snail_data,
};
