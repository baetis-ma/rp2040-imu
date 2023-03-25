#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
//#include "led3.pio.h"
int count = 0, count1 = 0;

//setup hardware
#define LED_BLUE   25
#define LED_GREEN  16
#define LED_RED    17
#define SW         29
//#define WS2812_PIN 16

//init i2c
#define I2C_SDA    6
#define I2C_SCK    7
#include "./include/i2c.h"

//oled display
#define I2C_POWER  0
#define SSD1306_I2C_ADDR   0x3c
#include "./include/ssd1306.h"

//bmp280
int digT1, digT2, digT3;
int digP1, digP2, digP3, digP4, digP5, digP6, digP7, digP8, digP9;
float pres, radius, theta, psi;
#define BMP280_I2C_ADDR    0x76
#include "./include/bmp280.c"

//magnetometer
#define QMC5883L_I2C_ADDR  0x0d
int direction;
int degrees;
float declination =  -13.9;   //noaa declination
float azoffset = -11.5;       //azimuth/2 offset
char *azimuthstr[16] = {"N  ", "NNE", "NE ", "ENE", "E  ", "ESE", "SE ", "SSE", "S  ", "SSW",
                        "SW ", "WSW", "W  ", "WNW", "NW ", "NNW" };
#include "./include/qmc5883l.c"

//intertal measurement unit
float pitch = 0, roll = 0;
float tau = 0.5;         //imu fusion filter time constant
float rate = 0.001;      //rate of mpu6500 measurements
#include "./include/attitude.c"

void core1_entry() {
    sleep_ms(1000);
    absolute_time_t systimenext0 = 5000000;
    imu_startup();
    sleep_ms(1000);
    int looprate0 = (int)(rate / 0.000001);
    while(1) {
        if(get_absolute_time() > systimenext0) {
           systimenext0 = systimenext0 + looprate0;
	   //printf("core1 %f\n", 0.000001*systimenext0);
	   spi_read();
	   ++count1;
        }
	//sleep_us(10); //just in case the compiler doesn't
    }
}

int mode = 0;
int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("waited 2 seconds\n");
    multicore_launch_core1(core1_entry);

    int looprate = (int)(0.5 / 0.000001);
    absolute_time_t systimenext = 4000000;
    float prescal;

    //gpios init
    gpio_init(LED_BLUE); gpio_set_dir(LED_BLUE, GPIO_OUT); gpio_put(LED_BLUE, 1);
    gpio_init(LED_GREEN); gpio_set_dir(LED_GREEN, GPIO_OUT); gpio_put(LED_GREEN, 1);
    gpio_init(LED_RED); gpio_set_dir(LED_RED, GPIO_OUT); gpio_put(LED_RED, 1);
    //set ws2812 pio driver
    //uint offset = pio_add_program(0, &ws2812_program);
    //ws2812_program_init(0, 0, offset, WS2812_PIN);
    gpio_init(I2C_POWER); gpio_set_dir(I2C_POWER, GPIO_OUT); gpio_put(I2C_POWER, 1);
    gpio_init(SW); gpio_set_dir(SW, GPIO_IN); gpio_pull_up(SW);

    //i2c_start();
    //sleep_us(1000);
    //ssd1306_init();
    char disp_string[256] = "Hi There";
    //ssd1306_text(disp_string);
    //qmc5883_init();   
    //bmp280_cal();
    while(1) {
        if(get_absolute_time() > systimenext) {
           systimenext = systimenext + looprate;
           //i2c_scan();
           //qmc5883_read();   
           //bmp280_read();
           //if (count < 5)  prescal = pres;
	   //spi_read();
           //    pio_sm_put_blocking(0, 0, 0x808080);
	   
           int blue = gpio_get(LED_BLUE);
           if (blue == 0) blue = 1; else blue = 0;
           gpio_put(LED_BLUE, blue);

           if(mode == 0) sprintf(disp_string, "4    IMU||1mag field vetor=|1       %4.2f %4.2f %4.2f\
                       |1pres=  %5dmb %5.1f\'|pitch =        -12.3*|roll =          10.3*|systime =  %7.1fsec", 
                       radius, theta, psi, (int)pres, (prescal-pres)/.038,  0.000001*get_absolute_time());
	            else sprintf(disp_string, "4%3s    %3d*||elev %5.1f\'||pitch 12.3*||roll -11.5*", 
                       azimuthstr[direction], degrees, (prescal-pres)/.038);
           //ssd1306_text(disp_string);
           printf("%11.4f  %4d %4d ", 0.000001 * get_absolute_time(), count, count1);
           printf ("pitch=%4d roll=%4d    radius=%5.2f theta=%5.2f psi=%5.2f    dir=%3s %3ddeg   pres=%.1f  %5.1f\n",
               (int)pitch, (int)roll, radius, theta, psi, azimuthstr[direction], degrees, pres, (prescal-pres)/.038);

	   count1 = 0;
           ++count;
        }
	if(0 == gpio_get(SW) && count > 5) { mode = (++mode) % 2; count = 0; }
	//sleep_ms(10); //just in case the compiler doesn't
    }
    return 0;
}
