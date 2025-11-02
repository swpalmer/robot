#ifndef NEOPIXEL_SPI_H
#define NEOPIXEL_SPI_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ORDER_RGB,
    ORDER_GRB,
    ORDER_RGBW,
    ORDER_GRBW
} pixel_order_t;

typedef struct {
    int spi_fd;
    int num_pixels;
    int bytes_per_pixel;
    float brightness;
    bool auto_write;
    pixel_order_t order;
    uint8_t *pixel_buf;
} neopixel_spi_t;

bool neopixel_spi_init(neopixel_spi_t *np, const char *spi_path, int num_pixels, int bpp,
                       float brightness, bool auto_write, pixel_order_t order);
void neopixel_spi_deinit(neopixel_spi_t *np);

void neopixel_spi_set_pixel(neopixel_spi_t *np, int index, uint32_t color);
void neopixel_spi_fill(neopixel_spi_t *np, uint32_t color);
void neopixel_spi_show(neopixel_spi_t *np);

void print_binary(uint8_t b);
void neopixel_spi_encode_byte(uint8_t byte, uint8_t *encoded);
#endif // NEOPIXEL_SPI_H
