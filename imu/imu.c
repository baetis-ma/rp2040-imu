#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
//#include "led3.pio.h"

#define QMC5883L_I2C_ADDR  0x0d
#define BMP280_I2C_ADDR       0x76
#define SSD1306_I2C_ADDR       0x3c
int digT1, digT2, digT3;
int digP1, digP2, digP3, digP4, digP5, digP6, digP7, digP8, digP9;
#define I2C_SDA 6
#define I2C_SCK 7
#define I2C_POWER 0
#define LED_BLUE 25
#define LED_GREEN 16
#define LED_RED 17
//#define WS2812_PIN 16
float pres, radius, theta, psi;
#include "./include/i2c.h"
#include "./include/ssd1306.h"
#include "./include/bmp280.c"
#include "./include/qmc5883l.c"

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("waited 2 seconds\n");
    absolute_time_t systimenext = 0;

    //gpios init
    gpio_init(LED_BLUE); gpio_set_dir(LED_BLUE, GPIO_OUT); gpio_put(LED_BLUE, 1);
    gpio_init(LED_GREEN); gpio_set_dir(LED_GREEN, GPIO_OUT); gpio_put(LED_GREEN, 1);
    gpio_init(LED_RED); gpio_set_dir(LED_RED, GPIO_OUT); gpio_put(LED_RED, 1);
    //set ws2812 pio driver
    //uint offset = pio_add_program(0, &ws2812_program);
    //ws2812_program_init(0, 0, offset, WS2812_PIN);
    gpio_init(I2C_POWER); gpio_set_dir(I2C_POWER, GPIO_OUT); gpio_put(I2C_POWER, 1);

    i2c_start();
    sleep_ms(10);
    ssd1306_init();
    char disp_string[256] = "Hi There";
    ssd1306_text(disp_string);
    qmc5883_init();   
    bmp280_cal();
    //spi interface setup
    //sdk spi, dma, ipterupt service
    float pitch, roll, yaw;
    
    int count = 0;
    float prescal;
    while(1) {
        if(get_absolute_time() > systimenext) {
           printf("%11.4f   ", 0.000001 * get_absolute_time());
           systimenext = systimenext + 500000;
           qmc5883_read();   
           bmp280_read();
	   if (count < 5)  prescal = pres;
           //i2c_scan();
           //    pio_sm_put_blocking(0, 0, 0x808080);
           int blue = gpio_get(LED_BLUE);
           if (blue == 0) blue = 1; else blue = 0;
           gpio_put(LED_BLUE, blue);

           sprintf(disp_string, "4 IMU %5d||1 %4.2f %4.2f %4.2f||1pres=%5dmb  %6.1f", 
              count/2, radius, theta, psi, (int)pres, (prescal-pres)/.038);
           ssd1306_text(disp_string);
	   ++count;
	}
	sleep_us(1000); //just in case the compiler doesn't
    }
    return 0;
}
