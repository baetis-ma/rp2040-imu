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
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    sleep_ms(1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
    sleep_ms(1);
}

static void mpu9250_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x68, 0x00};
    cs_select(); spi_write_blocking(SPI_PORT, buf, 2); cs_deselect(); sleep_us(10);
    buf[0] = 0x1b; buf[1] = 0x10;
    cs_select(); spi_write_blocking(SPI_PORT, buf, 2); cs_deselect(); sleep_us(10);
    buf[0] = 0x1c; buf[1] = 0x08;
    cs_select(); spi_write_blocking(SPI_PORT, buf, 2); cs_deselect(); sleep_us(10);
}




void imu_startup() {
    printf("Hello, MPU9250! Reading raw data from registers via SPI...\n");

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

    mpu9250_reset();

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

    absolute_time_t start = get_absolute_time(), spi_read, trig_calc;
    // Start reading acceleration registers from register 0x3B for 6 bytes
    read_registers(0x3B, buffer, 6);

    for (int i = 0; i < 3; i++) {
        daccel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        //accel[i] = 2 * (float)daccel[i]/(1<<15);
        accel[i] = 4 * (float)daccel[i]/(1<<15) - caccel[i];
    }

    // Now gyro data from reg 0x43 for 6 bytes
    read_registers(0x43, buffer, 6);

    for (int i = 0; i < 3; i++) {
        dgyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]) ;
        gyro[i] = 1000 * (float)dgyro[i]/(1<<15) - cgyro[i];
        //gyro[i] = 500 * (float)dgyro[i]/(1<<15) - cgyro[i];
    }

    if (count == 10) {
       caccel[0] = accel[0];
       caccel[1] = accel[1];
       caccel[2] = accel[2] - 1;
       cgyro[0] = gyro[0];
       cgyro[1] = gyro[1];
       cgyro[2] = gyro[2];
    }
    // Now temperature from reg 0x41 for 2 bytes
    read_registers(0x41, buffer, 2);
    temp = 32 + 1.8 * (float) (16.53 + (buffer[0] << 8 | buffer[1]) / 340.0);

    spi_read = get_absolute_time() - start;
    start = get_absolute_time();



    trig_calc = get_absolute_time() - start;

    printf("%5.4f %5.4f   ", 0.000001 *spi_read,  0.000001 *trig_calc);
    printf("  %9.4f %9.4f\n", accel[0], gyro[1]);
    //printf("  %9.4f %9.4f\n", accel[0], gyro[1]);
    //printf("  %9.4f %9.4f %9.4f    ", accel[0],accel[1], accel[2]);
    //printf("    %9.4f %9.4f %9.4f\n", gyro[0], gyro[1],gyro[2]);
    //printf("Temp. = %5.2fF\n", temp );

}

struct IMU {
       int nsamp;
       int cal_cnt;
       float pitch;
       float roll;
} imu;

//static void imu_init () 
//{
//   cs_select();
//   printf("started\n");
//   uint8_t buf[] = {0x19, 0x00};
//   spi_write_blocking(SPI_PORT, buf, 2);
//   printf("first write completed\n");

//   buf[0] = 0x1a; buf[1] = 0x01;
//   spi_write_blocking(SPI_PORT, buf, 2);
//   buf[0] = 0x1b; buf[1] = 0x00;
//   spi_write_blocking(SPI_PORT, buf, 2);
//   buf[0] = 0x1c; buf[1] = 0x00;
//   spi_write_blocking(SPI_PORT, buf, 2);
//   buf[0] = 0x1d; buf[1] = 0x00;
//   spi_write_blocking(SPI_PORT, buf, 2);
//   buf[0] = 0x23; buf[1] = 0x78;
//   spi_write_blocking(SPI_PORT, buf, 2);
//   buf[0] = 0x6b; buf[1] = 0x00;
//   spi_write_blocking(SPI_PORT, buf, 2);
//   buf[0] = 0x6c; buf[1] = 0x00;
//   spi_write_blocking(SPI_PORT, buf, 2);
//   buf[0] = 0x6a; buf[1] = 0x40;
//   spi_write_blocking(SPI_PORT, buf, 2);
//   buf[0] = 0x68; buf[1] = 0x07;
//   spi_write_blocking(SPI_PORT, buf, 2);
//   cs_deselect();

   //spi_write_byte( device, 0x19, 0x00); /* Sample Rate Div not used in rates >= 1K */
   //spi_write_byte( device, 0x1a, 0x01); /* Config - fifo overflow, no sync*/
   //spi_write_byte( device, 0x1b, 0x00); /* Gyro Config - range +/-250dps, bypass dlpf 8Ksamp 250LP*/
   //spi_write_byte( device, 0x1c, 0x00); /* Accel Config - range +/-2g */
   //spi_write_byte( device, 0x1d, 0x00); /* Accel Config 1 -x00 1khz/1ksamp 0x08=200hz x0a=100 x0e=5 x0f=420 */
   //spi_write_byte( device, 0x23, 0x78); /* fifo enable - gyroscope and accel */
   //spi_write_byte( device, 0x6b, 0x00); /* Power Management 1 */
   //spi_write_byte( device, 0x6c, 0x00); /* Power Management 2 */
   //spi_write_byte( device, 0x6a, 0x40); /* User Control - enable fifo*/
   //spi_write_byte( device, 0x68, 0x07); /* Signal Path Reset */
   //read back
//   int8_t ibuf[64];
//   read_registers(0x19, ibuf, 16);
//   for (int n = 0; n < 16; n++) printf("0x %02x ", ibuf[n]); printf("\n");
//   read_registers(0x68, ibuf, 8);
//   for (int n = 0; n < 8; n++) printf("0x %02x ", ibuf[n]); printf("\n");
//}

//void imu_read()
//{
//    uint8_t temp_buffer[32];
//    uint8_t buffer[1024];
//    float calx=0, caly=0, calz=0, gcalx=0, gcaly=0;
//    float pitch_last=0, roll_last=0;
//    float xAccl_cal=0, yAccl_cal=0, zAccl_cal=0, xGyro_cal=0, yGyro_cal=0;
//    float xAccl, yAccl, zAccl, xGyro, yGyro;
//
//    float rate = 0.001;
//    float xFusion_T = (0.50/rate)/(1+0.50/rate); 
//    float yFusion_T = (0.50/rate)/(1+0.50/rate); 
//
//    imu.nsamp = 0;
//    //spi_write_byte(vspi, 0x6a, 0x44); /* reset fifo */
//    while (1) {
//        //read_registers(0x19, ibuf, 16);
//        //spi_read_bytes(vspi, 0x72, 2, temp_buffer);
//        int samples = 256 * temp_buffer[0] + temp_buffer[1];
//        if(samples > 256)samples=256;  //max size of system
//        samples = samples / 12;
//        if(samples>0){ //read_registers(0x74, ibuf, 12);
//        //if(samples>0){ spi_read_bytes(vspi, 0x74, 12 * samples, buffer); }
//        for (int n = 0; n <samples; n++){
//            imu.nsamp++;
//            //accelerometers raw calibrated and filtered - 1K sample rate (1/8 of gyro)
//            xAccl = (float)(256 * buffer[12*n+0] + buffer[12*n+1]);
//            if(xAccl>0x8000)xAccl = -1.0*(0xffff - xAccl) ; 
//            xAccl = xAccl / 0x4000;
//            xAccl = xAccl - xAccl_cal;
//
//            yAccl = (float)(256 * buffer[12*n+2] + buffer[12*n+3]);
//            if(yAccl>0x8000)yAccl = -1.0 * (0xffff - yAccl);
//            yAccl = yAccl / 0x4000;
//            yAccl = yAccl - yAccl_cal;
//
//            zAccl = (float)(256 * buffer[12*n+4] + buffer[12*n+5]);
//            if(zAccl>0x8000)zAccl = -1.0 * (0xffff - zAccl );
//            zAccl = zAccl / 0x4000;
//            zAccl = zAccl - zAccl_cal;
//
//            //Gyros raw calibrated and Integral at full 8K sample rate
//            xGyro = (float)(256 * buffer[12*n+6] + buffer[12*n+7]);
//            if(xGyro>0x8000)xGyro = -1.0*(0xffff - xGyro) ; 
//            xGyro = 0.001 * xGyro * 250.00 / 0x8000;
//
//            yGyro = (float)(256 * buffer[12*n+8] + buffer[12*n+9]);
//            if(yGyro>0x8000)yGyro = -1.0 * (0xffff - yGyro);
//            yGyro = 0.001 * yGyro * 250.00 / 0x8000;
//
//            //sensor fusion 'complimentary filter' LP filter on Accelerometer and HP filter on gyro
//            if(xAccl< -1)xAccl = -1; if(xAccl> 1)xAccl = 1;
//            if(yAccl< -1)yAccl = -1; if(yAccl> 1)yAccl = 1;
//            imu.pitch = xFusion_T * pitch_last + (1 - xFusion_T) * 57.3 * asin(yAccl) + 1* (xGyro - xGyro_cal) ;
//            pitch_last = imu.pitch;  
//            imu.roll = yFusion_T * roll_last + (1 - yFusion_T) * -57.3 * asin(xAccl) + 1* (yGyro - yGyro_cal) ;
//            roll_last = imu.roll;
//            //if(imu.nsamp%1000==0){
//            //    printf("%6d  %7.2f %7.2f %7.2f %7.2f %7.2f  ",imu.nsamp,xAccl,yAccl,zAccl,xGyro,yGyro);
//            //    printf("    %7.2f %7.2f  ",imu.pitch, imu.roll);
//            //    printf("\n");
//            //}
//
//            //calibration routine runs for about 1/4sec averages 500 measurements
//            if (imu.cal_cnt <= 2000){
//                if(imu.cal_cnt == 0 ) {
//                   xAccl = 0; yAccl = 0; zAccl = 0; xGyro = 0; yGyro = 0;
//                   xAccl_cal=0; yAccl_cal=0; zAccl_cal=0; xGyro_cal=0; yGyro_cal=0; 
//                   calx = 0.0; caly = 0.0; calz = 0.0; gcalx = 0.0; gcaly = 0.0; } 
//                else if(imu.cal_cnt >= 1500 && imu.cal_cnt < 2000) {
//                   calx = calx + xAccl; caly = caly + yAccl; calz = calz + zAccl;
//                   gcalx = gcalx + xGyro; gcaly = gcaly + yGyro; 
//                }
//                else if(imu.cal_cnt ==  2000) {
//                   xAccl_cal = calx / 500; yAccl_cal = caly / 500; zAccl_cal = calz / 500;
//                   xGyro_cal = gcalx / 500; yGyro_cal = gcaly / 500; 
//                   zAccl_cal = zAccl_cal - 1.0;  //gravity orientation
//                   printf( "cal   %f %f %f   %f %f\n", calx/500,caly/500,calz/500,gcalx/500,gcaly/500);
//                }
//                ++imu.cal_cnt;
//            } //end of cal
//        } //end of samples
//    } //end of while 1
//} //end of task
