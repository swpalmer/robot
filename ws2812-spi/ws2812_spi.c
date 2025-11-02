#include "ws2812_spi.h"
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdio.h>

#define FREQ_HZ 800000
#define SPI_BITS_PER_COLOR_BIT 3 // 4 SPI bits per neopixel bit

static void apply_brightness(uint8_t *r, uint8_t *g, uint8_t *b, float brightness) {
    *r = (uint8_t)((*r) * brightness);
    *g = (uint8_t)((*g) * brightness);
    *b = (uint8_t)((*b) * brightness);
}

#define HEADER_BYTES 0 // 4 should be enough
#define TRAILER_BYTES 40
#define WS2812_SPI_ONE  0b110 // Approx. 800ns high, 450ns low
#define WS2812_SPI_ZERO 0b100 // Approx. 400ns high, 850ns low

bool neopixel_spi_init(neopixel_spi_t *np, const char *spi_path, int num_pixels, int bpp,
                       float brightness, bool auto_write, pixel_order_t order) {
    if (bpp != 3 && bpp != 4) return false;

    np->spi_fd = open(spi_path, O_WRONLY);
    if (np->spi_fd < 0) return false;

    uint8_t mode = 0;
    ioctl(np->spi_fd, SPI_IOC_WR_MODE, &mode);
    uint8_t bits = 8;
    ioctl(np->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    uint8_t lsb = 0; // write MSB first
    ioctl(np->spi_fd, SPI_IOC_WR_LSB_FIRST, &lsb);
    uint32_t speed = FREQ_HZ * SPI_BITS_PER_COLOR_BIT;
    ioctl(np->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    printf("Set SPI clock speed to %dHz\n", speed);

    np->num_pixels = num_pixels;
    np->bytes_per_pixel = bpp;
    np->brightness = brightness;
    np->auto_write = auto_write;
    np->order = order;
    np->pixel_buf = calloc(num_pixels * bpp, sizeof(uint8_t));
    return (np->pixel_buf != NULL);
}

void neopixel_spi_deinit(neopixel_spi_t *np) {
    if (!np) return;
    neopixel_spi_fill(np, 0);
    neopixel_spi_show(np);
    close(np->spi_fd);
    free(np->pixel_buf);
    np->pixel_buf = NULL;
}

void neopixel_spi_set_pixel(neopixel_spi_t *np, int index, uint32_t color) {
    if (index < 0 || index >= np->num_pixels) return;

    int offset = index * np->bytes_per_pixel;
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    uint8_t w = (color >> 24) & 0xFF;

    apply_brightness(&r, &g, &b, np->brightness);

    switch (np->order) {
        case ORDER_GRB:
            np->pixel_buf[offset] = g;
            np->pixel_buf[offset + 1] = r;
            np->pixel_buf[offset + 2] = b;
            break;
        case ORDER_RGB:
            np->pixel_buf[offset] = r;
            np->pixel_buf[offset + 1] = g;
            np->pixel_buf[offset + 2] = b;
            break;
        case ORDER_RGBW:
            np->pixel_buf[offset] = r;
            np->pixel_buf[offset + 1] = g;
            np->pixel_buf[offset + 2] = b;
            np->pixel_buf[offset + 3] = w;
            break;
        case ORDER_GRBW:
            np->pixel_buf[offset] = g;
            np->pixel_buf[offset + 1] = r;
            np->pixel_buf[offset + 2] = b;
            np->pixel_buf[offset + 3] = w;
            break;
    }

    if (np->auto_write) {
        neopixel_spi_show(np);
    }
}

void neopixel_spi_fill(neopixel_spi_t *np, uint32_t color) {
    for (int i = 0; i < np->num_pixels; i++) {
        neopixel_spi_set_pixel(np, i, color);
    }
}

void neopixel_spi_encode_byte(uint8_t byte, uint8_t *encoded) {
    // WS2812 encoding for SPI: each bit is translated into 3 SPI bits
    uint32_t spi_bytes = 0;
    for (int i = 7; i >= 0 ; i--) {
        spi_bytes |= ((byte & (1 << i)) ? WS2812_SPI_ONE : WS2812_SPI_ZERO) << (i*3);
    }
    *encoded = (uint8_t) (spi_bytes >> 16);
    encoded++;
    *encoded = (uint8_t) ((spi_bytes >> 8) & 0xff);
    encoded++;
    *encoded = (uint8_t) (spi_bytes & 0xff);

    // for (int i = 0; i < 4; i++) {
    //     encoded[i] = ((byte >> (2 * i + 1)) & 1) * 0x60 +
    //                  ((byte >> (2 * i + 0)) & 1) * 0x06 + 0x88;
    // }
}

void print_binary(uint8_t b) {
    for (int i = 7; i >= 0; i--) {
        printf((b & (1 << i)) ? "1" : "0");
    }
}
void neopixel_spi_show(neopixel_spi_t *np) {
    int encoded_len = HEADER_BYTES + TRAILER_BYTES + np->num_pixels * np->bytes_per_pixel * SPI_BITS_PER_COLOR_BIT;
    uint8_t *encoded_buf = malloc(encoded_len);
    if (!encoded_buf) return;

    memset(encoded_buf, 0, HEADER_BYTES);
    int i = 0;
    for (; i < np->num_pixels * np->bytes_per_pixel; ++i) {
        //printf("%3d: %02x\n     ", i, np->pixel_buf[i]);
        neopixel_spi_encode_byte(np->pixel_buf[i], &encoded_buf[HEADER_BYTES + i * SPI_BITS_PER_COLOR_BIT]);
        //print_binary(*(&encoded_buf[HEADER_BYTES + i * SPI_BITS_PER_COLOR_BIT])); printf(" ");
        //print_binary(*(&encoded_buf[HEADER_BYTES + i * SPI_BITS_PER_COLOR_BIT + 1])); printf(" ");
        //print_binary(*(&encoded_buf[HEADER_BYTES + i * SPI_BITS_PER_COLOR_BIT + 2])); printf("\n");
    }
    for (int j = 0; j < TRAILER_BYTES; j++) {
        encoded_buf[HEADER_BYTES + i * SPI_BITS_PER_COLOR_BIT + j] = 0;
    }

    printf("Encode buffer length for %d pixels is %d at %d bytes-per-pixel, and %d SPI bits per source bit with %d header bytes and % d trailer bytes\n",
        np->num_pixels, encoded_len,
        np->bytes_per_pixel, SPI_BITS_PER_COLOR_BIT,
        HEADER_BYTES, TRAILER_BYTES
    );
    if (false) {
        for (int i = 0; i < encoded_len; i += np->bytes_per_pixel) {
            printf("%3d: ");
            for (int j = 0; j < np->bytes_per_pixel; j++) {
                uint8_t *ptr = &encoded_buf[i+j];
                print_binary(*ptr); printf(" ");
            }
            printf("\n");
        }
    }

    write(np->spi_fd, encoded_buf, encoded_len);
    free(encoded_buf);
}
