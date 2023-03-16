#include "hardware/i2c.h"

int i2c_start() {
    i2c_init(i2c1, 200 * 1000); //i2c frequency
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCK, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCK);
    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCK, GPIO_FUNC_I2C));
    return 0;
}

int i2c_scan() {
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
        int ret;
        uint8_t rxdata;
        ret = i2c_read_blocking(i2c1, addr, &rxdata, 1, false);
        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    return 0;
}


