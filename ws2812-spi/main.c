#include <unistd.h>
#include <stdio.h>
#include "ws2812_spi.h"
#include "pixelbuf.h"

int main1();
int main2();

int main() {
    return main2();
}

int main1() {
    uint8_t buf[] = { 0, 0, 0 };
    neopixel_spi_encode_byte(0xff, buf);
    print_binary(buf[0]);
    printf(" ");
    print_binary(buf[1]);
    printf(" ");
    print_binary(buf[2]);
    printf("\n");
    return 0;
}

int main2() {
    int numPixels = 144;
    int bpp= 3;
    float brightness = 1.0;
    neopixel_spi_t pixels;
    if (!neopixel_spi_init(&pixels, "/dev/spidev0.0", numPixels, bpp, brightness, false, ORDER_GRB)) {
        return 1;
    }

    neopixel_spi_fill(&pixels, 0xff0000); // Red
    neopixel_spi_show(&pixels);
    sleep(1);
    neopixel_spi_fill(&pixels, 0x00ff00); // Green
    neopixel_spi_show(&pixels);
    sleep(1);
    neopixel_spi_fill(&pixels, 0x0000ff); // Blue
    neopixel_spi_show(&pixels);
    sleep(1);



    neopixel_spi_deinit(&pixels);
    return 0;
}
