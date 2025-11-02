/*
 * The MIT License (MIT)
 *
 * C port of PixelBuf (CircuitPython) by Roy Hooper et al.
 * Adapted by [Your Name or Project] from Python source
 */

#ifndef PIXELBUF_H
#define PIXELBUF_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define DOTSTAR_LED_START_FULL_BRIGHT 0xFF
#define DOTSTAR_LED_START 0b11100000
#define DOTSTAR_LED_BRIGHTNESS 0b00011111

/*
typedef enum {
    ORDER_RGB,
    ORDER_GRB,
    ORDER_BGR,
    ORDER_RGBW,
    ORDER_GRBW,
    ORDER_DOTSTAR
} pixel_order_t;
*/

typedef struct {
    uint8_t* buffer;               // Brightness-applied pixel buffer
    uint8_t* pre_brightness_buf;  // Original buffer if brightness < 1.0
    int num_pixels;
    int bpp;                      // Bytes per pixel (3 or 4)
    float brightness;             // Brightness value 0.0 - 1.0
    bool auto_write;
    pixel_order_t order;
    bool has_white;
    bool dotstar_mode;
    int byteorder[4];             // Channel order indexes (e.g., R=0, G=1, B=2, W/P=3)
    int pixel_step;
    int offset;
    uint8_t* header;
    int header_len;
    uint8_t* trailer;
    int trailer_len;
} PixelBuf;

static void parse_byteorder(const char* str, PixelBuf* pb) {
    pb->bpp = strlen(str);
    pb->dotstar_mode = false;
    pb->has_white = false;

    for (int i = 0; i < 4; ++i) pb->byteorder[i] = -1;

    for (int i = 0; i < pb->bpp; ++i) {
        switch (str[i]) {
            case 'R': pb->byteorder[0] = i; break;
            case 'G': pb->byteorder[1] = i; break;
            case 'B': pb->byteorder[2] = i; break;
            case 'W': pb->byteorder[3] = i; pb->has_white = true; break;
            case 'P': pb->byteorder[3] = i; pb->dotstar_mode = true; break;
            default: break;
        }
    }

    if (pb->dotstar_mode) {
        for (int i = 0; i < 3; ++i) pb->byteorder[i]++;
        pb->byteorder[3] = 0; // brightness byte for DotStar
    }

    pb->pixel_step = pb->dotstar_mode ? 4 : pb->bpp;
}

bool pixelbuf_init(PixelBuf* pb, int n, const char* byteorder_str, float brightness, bool auto_write,
                   const uint8_t* header, int header_len,
                   const uint8_t* trailer, int trailer_len) {
    if (!pb) return false;

    parse_byteorder(byteorder_str, pb);

    pb->num_pixels = n;
    pb->brightness = brightness;
    pb->auto_write = auto_write;
    pb->offset = header_len;
    pb->header_len = header_len;
    pb->trailer_len = trailer_len;

    int total_len = pb->pixel_step * n + header_len + trailer_len;

    pb->buffer = (uint8_t*)calloc(total_len, 1);
    if (!pb->buffer) return false;

    if (brightness < 0.999) {
        pb->pre_brightness_buf = (uint8_t*)calloc(total_len, 1);
        if (!pb->pre_brightness_buf) {
            free(pb->buffer);
            return false;
        }
    } else {
        pb->pre_brightness_buf = NULL;
    }

    if (header && header_len > 0) {
        memcpy(pb->buffer, header, header_len);
        if (pb->pre_brightness_buf) memcpy(pb->pre_brightness_buf, header, header_len);
    }

    if (trailer && trailer_len > 0) {
        memcpy(pb->buffer + header_len + pb->pixel_step * n, trailer, trailer_len);
        if (pb->pre_brightness_buf)
            memcpy(pb->pre_brightness_buf + header_len + pb->pixel_step * n, trailer, trailer_len);
    }

    return true;
}

void pixelbuf_deinit(PixelBuf* pb) {
    if (!pb) return;
    free(pb->buffer);
    free(pb->pre_brightness_buf);
    pb->buffer = NULL;
    pb->pre_brightness_buf = NULL;
}

static void apply_brightness(PixelBuf* pb, int offset, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    if (pb->dotstar_mode) {
        w = (uint8_t)(((int)(w * 31)) & DOTSTAR_LED_BRIGHTNESS) | DOTSTAR_LED_START;
    } else if (pb->has_white && r == g && g == b) {
        w = r; r = g = b = 0;
    } else if (!pb->dotstar_mode) {
        w = (uint8_t)(w * pb->brightness);
    }

    pb->buffer[offset + pb->byteorder[0]] = (uint8_t)(r * pb->brightness);
    pb->buffer[offset + pb->byteorder[1]] = (uint8_t)(g * pb->brightness);
    pb->buffer[offset + pb->byteorder[2]] = (uint8_t)(b * pb->brightness);
    if (pb->bpp == 4)
        pb->buffer[offset + pb->byteorder[3]] = w;
}

void pixelbuf_set_pixel(PixelBuf* pb, int index, uint32_t color) {
    if (index < 0 || index >= pb->num_pixels) return;

    int offset = pb->offset + index * pb->pixel_step;

    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    uint8_t w = (color >> 24) & 0xFF;

    if (pb->pre_brightness_buf) {
        pb->pre_brightness_buf[offset + pb->byteorder[0]] = r;
        pb->pre_brightness_buf[offset + pb->byteorder[1]] = g;
        pb->pre_brightness_buf[offset + pb->byteorder[2]] = b;
        if (pb->bpp == 4)
            pb->pre_brightness_buf[offset + pb->byteorder[3]] = w;
    }

    apply_brightness(pb, offset, r, g, b, w);
}

void pixelbuf_fill(PixelBuf* pb, uint32_t color) {
    for (int i = 0; i < pb->num_pixels; i++) {
        pixelbuf_set_pixel(pb, i, color);
    }
}

void pixelbuf_set_brightness(PixelBuf* pb, float value) {
    if (fabs(pb->brightness - value) < 0.001f) return;

    value = fminf(fmaxf(value, 0.0f), 1.0f);
    pb->brightness = value;

    if (!pb->pre_brightness_buf) return;

    for (int i = 0; i < pb->num_pixels; i++) {
        int offset = pb->offset + i * pb->pixel_step;
        uint8_t r = pb->pre_brightness_buf[offset + pb->byteorder[0]];
        uint8_t g = pb->pre_brightness_buf[offset + pb->byteorder[1]];
        uint8_t b = pb->pre_brightness_buf[offset + pb->byteorder[2]];
        uint8_t w = pb->bpp == 4 ? pb->pre_brightness_buf[offset + pb->byteorder[3]] : 0;
        apply_brightness(pb, offset, r, g, b, w);
    }
}

uint32_t pixelbuf_get_pixel(PixelBuf* pb, int index) {
    if (index < 0 || index >= pb->num_pixels) return 0;

    int offset = pb->offset + index * pb->pixel_step;
    uint8_t* src = pb->pre_brightness_buf ? pb->pre_brightness_buf : pb->buffer;
    uint8_t r = src[offset + pb->byteorder[0]];
    uint8_t g = src[offset + pb->byteorder[1]];
    uint8_t b = src[offset + pb->byteorder[2]];
    uint8_t w = (pb->bpp == 4) ? src[offset + pb->byteorder[3]] : 0;

    return ((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

uint8_t* pixelbuf_get_transmit_buffer(PixelBuf* pb, int* len) {
    *len = pb->header_len + pb->pixel_step * pb->num_pixels + pb->trailer_len;
    return pb->buffer;
}

uint32_t wheel(uint8_t pos) {
    if (pos < 85) {
        return ((255 - pos * 3) << 16) | ((pos * 3) << 8);
    } else if (pos < 170) {
        pos -= 85;
        return ((255 - pos * 3) << 8) | (pos * 3);
    } else {
        pos -= 170;
        return (pos * 3 << 16) | (255 - pos * 3);
    }
}

#endif // PIXELBUF_H
