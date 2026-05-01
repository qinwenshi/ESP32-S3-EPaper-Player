// Snail icon — 12×8 px, 1-bit, faces right (shell←, head→)
// palette[0]=white(bg), palette[1]=black(ink)
#pragma once
#include <stdint.h>
#include <lvgl.h>

#define SNAIL_W      12
#define SNAIL_H       8
#define SNAIL_STRIDE  2   // ceil(12/8)

//  Pixel map (1=black, snail faces right):
//   001100000101   shell-top   + antennae
//   010010000101   shell-sides + antennae
//   010110001110   shell+dot   + head-top
//   010010001001   shell-sides + head-side
//   001100001101   shell-bot   + head+eye
//   111111111001   body strip  + head-lower
//   100000001110   body-under  + head-bottom
//   010100000000   feet

static const uint8_t snail_data[] = {
    // palette[0] = white  (BGRA)
    0xFF, 0xFF, 0xFF, 0xFF,
    // palette[1] = black  (BGRA)
    0x00, 0x00, 0x00, 0xFF,
    // pixel rows (MSB-first, 2 bytes per row, 4 padding bits at end)
    0x30, 0x50,   // row 0
    0x48, 0x50,   // row 1
    0x58, 0xE0,   // row 2
    0x48, 0x90,   // row 3
    0x30, 0xD0,   // row 4
    0xFF, 0x90,   // row 5
    0x80, 0xE0,   // row 6
    0x50, 0x00,   // row 7
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
