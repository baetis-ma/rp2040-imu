#include "hardware/spi.h"
#include <math.h>
#define PIN_MISO 4
#define PIN_CS   1
#define PIN_SCK  2
#define PIN_MOSI 3

#define SPI_PORT spi0
#define READ_BIT 0x80

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}
static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    sleep_us(100); //this delay must be at least 50us (blocking stall)
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
    //sleep_us(30);
}

static void mpu6500_reset() {
    uint8_t buf[] = {0x68, 0x00};
    cs_select(); spi_write_blocking(SPI_PORT, buf, 2); cs_deselect(); sleep_us(100);
    buf[0] = 0x1b; buf[1] = 0x10;
    cs_select(); spi_write_blocking(SPI_PORT, buf, 2); cs_deselect(); sleep_us(100);
    buf[0] = 0x1c; buf[1] = 0x08;
    cs_select(); spi_write_blocking(SPI_PORT, buf, 2); cs_deselect(); sleep_us(100);
}

void imu_startup() {
    printf("Hello, MPU6500! Reading raw data from registers via SPI...\n");

    // This example will use SPI0 at 0.5MHz.
    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PIN_CS, "SPI CS"));

    mpu6500_reset();

    // See if SPI is working - interrograte the device for its I2C ID number, should be 0x71
    uint8_t id;
    read_registers(0x75, &id, 1);
    printf("I2C address is 0x%x\n", id);
    read_registers(0x1b, &id, 1);
    printf("gyro cntl 0x1b  0x%x\n", id);
    read_registers(0x1c, &id, 1);
    printf("accl cntl 0x1c  0x%x\n", id);
}

void spi_read() {
    float accel[3], gyro[3], temp;
    int16_t  daccel[3], dgyro[3];
    //imu_init();

    uint8_t buffer[6];
    static float caccel[3], cgyro[3];
    static float frac;
    frac = (tau/rate) / (1.0 + (tau/rate));
    static float ifrac;
    ifrac = 1.0 - frac;

    absolute_time_t start = get_absolute_time(), spi_read, trig_calc;
    // Start reading acceleration registers from register 0x3B for 6 bytes
    // reading exec time could be halved by reading both accel,  gyro and temp at same time
    read_registers(0x3B, buffer, 6);
    for (int i = 0; i < 3; i++) {
        daccel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        accel[i] = 4 * (float)daccel[i]/(1<<15) - caccel[i];
    }

    // Now gyro data from reg 0x43 for 6 bytes
    read_registers(0x43, buffer, 6);
    for (int i = 0; i < 3; i++) {
        dgyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]) ;
        gyro[i] = 1000 * (float)dgyro[i]/(1<<15) - cgyro[i];
    }

    if (count < 2) {
       caccel[0] = 0;
       caccel[1] = 0;
       caccel[2] = 0;
       cgyro[0] = 0;
       cgyro[1] = 0;
       cgyro[2] = 0;
    }
    if (count == 3) {
       caccel[0] = accel[0];
       caccel[1] = accel[1];
       caccel[2] = accel[2] - 1;
       cgyro[0] = gyro[0];
       cgyro[1] = gyro[1];
       cgyro[2] = gyro[2];
       pitch = 0;
       roll = 0;
    }

    //Now temperature from reg 0x41 for 2 bytes
    //read_registers(0x41, buffer, 2);
    //temp = 32 + 1.8 * (float) (16.53 + (buffer[0] << 8 | buffer[1]) / 340.0);

    //spi_read = get_absolute_time() - start;
    //start = get_absolute_time();

    if(accel[0] > 1.00) accel[0] = 1.0; if(accel[0] < -1.00) accel[0] = -1.0;
    if(accel[1] > 1.00) accel[1] = 1.0; if(accel[1] < -1.00) accel[1] = -1.0;

    pitch = frac * pitch + rate * gyro[1] - ifrac * 57.3 * asin(accel[0]);
    roll  = frac * roll  + rate * gyro[0] + ifrac * 57.3 * asin(accel[1]);

    //trig_calc = get_absolute_time() - start;
    //printf("%8.6f %8.6f", 0.000001 *spi_read,  0.000001 *trig_calc);
    //printf("  %9.4f %9.4f    %9.4f %9.4f\n", accel[0], gyro[1], pitch, roll);
}
