# rp2040-imu
RPi PICO / RP2040 IMU

Rp2040 sdk framework
bmp280 atmospheric pressure sensor
qmc5883 magnetometer 
mcp8005 three axis accelerometer and gyroscope
Ssd1306 oled display

RPi PICO/RP2040 SSD FRAMEWORK
This project uses the PICO SDK environment (in a linux environment), for simplicity, without any specific IDE. The installation of the SDK is well documented elsewhere, varies by OS and is a half dozen steps. The github repository contains all other c programs, cmake files, shell scripts and include files required. The I2C SCK is on RP2040 gpio pin 7 and SDA is on gpio pin 6.

The steps required to run the project from my linux directory are:
```
cd Desktop
git clone https://github.com/baetis-ma/rp2040-imu
cd imu
chmod +x *.sh
mkdir build
cd build
. ../../setenv.sh     (include first dot, . is source - will apply to current window shell process)
cmake ..
make
../../flash.sh monitor  ( | tee data)
```
Note: the shell scripts setenv.sh and flash.sh may contain paths or files specific to your build, for example I installed SDK at ~/pico-sdk and my serial port shows up at /dev/ttyACM0.
The setenv.sh script is used to install environment variables into the current terminal window shell. The flash.sh file 1)waits for /dev/ttyACM0 to be present, 2)stty’s ‘magic’ baud rate of 1200 which puts the rp2040 into boot mode and presents flash device as mass storage device, 3) wait for mass storage device to mount at /media/<name>/RPI-RP2, 4) writes picocode.ef2 file to /media/<name>/RPI-RP2, 5)waits for /dev/ttyACM0 to reconnect and 6)sets up monitor with correct stty settings, cntl C exits monitor. Piping output to | tee data1 will display the output and save it to a file named ‘data’.

BMP280 ATMOSPHERIC PRESSURE SENSOR
Calibration of this device requires transferring 24 bytes of calibration offset data fused into the device by the manufacturer. The pressure and temperature data read from the device along with the 24 calibration numbers are used to calculate the actual pressure value in hPa (=millibar). The device can be used to measure fluctuations in atmospheric pressure (which has interesting subtleties); additionally, once calibrated to local pressure can calculate altitude changes accurately to unit feet (10 feet = .38 millibar).

Once the 24 bytes of calibration data are read and converted into 12 calibration variables (digT1-3 and digP1-9) the calculation of ctemp and pres and proceed (from datasheet):
```C
    float ctemp, pres;
    double var1, var2, ctemp;
    
    //read 6 measurement bytes 
    regdata[0] = 0xf7;
    i2c_write_blocking(i2c1, BMP280_I2C_ADDR, regdata, 1, false); 
    i2c_read_blocking(i2c1, BMP280_I2C_ADDR, regdata, 6, false);
    int adcp = 4096 * regdata[0] + 16*regdata[1] + regdata[2]/16;
    int adct = 4096 * regdata[3] + 16*regdata[4] + regdata[5]/16;
    //calculate temperature
    int tfine;
    var1 = (((double)adct)/16384.0 - ((double)digT1)/1024.0)*((double)digT2);
    var2 = ((((double)adct)/131072.0 - ((double)digT1)/8192.0)*
            (((double)adct)/131072.0 - ((double)digT1)/8192.0)*(double)digT3);
    tfine = (int) var1+var2;
    ctemp = (float) tfine/5120.0;
    //calculate pressure
    double p;
    var1 = ((double)tfine/2.0) - 64000.0;
    var2 = var1*var1*((double)digP6)/32768.0;
    var2 = var2 + var1*((double)digP5)*2.0;
    var2 = var2/4 + ((double)digP4)*65536.0;
    var1 = (((double)digP3)*var1*var1/524288.0 + ((double)digP2)*var1)/524288.0;
    var1 = (1 + var1/32768.0)*((double)digP1);
    p = 1048576.0-((double)adcp);
    p = (p - var2/4096.0)*6250.0/var1;
    var1 = ((double)digP9)*p*p/2147483648.0;
    var2 = p*((double)digP8)/32768.0;
    pres = (p + (var1 + var2 + ((double)digP7))/16.0)/100;
```

SSD1306 OLED DISPLAY
This project uses a custom software that has been with me a long time. An 8x5 bit font array is loaded that is used to extract eight single or double sized pixel images to be transferred to the display. The configuration of the SSD1306 is completed with the ssd1306_init() command. The command ssd1306_text(str_text) writes text to the display - examples:
```C
   sprintf(disp_string, “4Top Line 2X|||1  value1=%d|   value2=%4.2fmsec||4LastLine”, value1, value2);
   ssd1306(disp_sting);
```
This will cause Top Line to be displayed at beginning of top line in double size, then it advances three lines - two lines to make up for the two lines used and a blank line, the next line is in small text and displays <space>value1=<value1>, then another line advance for the line used and <3xspace>value2=<value2 float> and finally LastLine is printed out in double size. Basically, any ‘|’ resets column pointer, each ‘|’ advances row pointer, when number 1 or 4 is in column 0 position text is either printed single or doubled size. It looks like this:

<img src="https://user-images.githubusercontent.com/32702163/225639441-afbaed19-081f-4ff9-8ac0-c9b88f4c7ab9.png" width="200"/>

One of the RP2040 pins (RP2040 gpio pin 0) powers the SSD1306 VCC pin as a means of resetting the SSD1306 memory pointer; the device requires reconfiguration after reset.

MAGNETOMETER
The Qmc5883 is a three axis magneto-resistive (hall effect) sensor, with 16 bit dacs with resolution to 2milli-gauss resolution with a 200KHz i2c interface, with a 200Hz maximum sample rate. In addition to providing compass direction readings (degrees from north) it also provides position data much like the three axis accelerometer, instead of measuring vector toward the center of earth it measures the vector of earth's local magnetic field.

The magnetic field of the earth at any one point is not large compared to say a magnet or a laptop computer a foot away, the field is also easily perturbed by ferroelectric materials. Many of these effects can be counteracted by calibrating after final placement in circuit, but the best use of the device is to isolate it from anything magnetic, ferroelectric, massive  or carrying current. Avoid breadboards and the 100mm header pins. A non-magnetized iron nail four inches away changes the measured fields by about 10% - leading to big changes in local field and vector calculations (falls off inverse cube?). A quarter, glass or a hand has basically no affect.

Calibration of the magnetometer requires the collection of a fairly substantial amount of data collected from the magnetometer while the device is being basically pitched, rolled and yawed over 360 degrees on three axes (there must be a trick to this). The data should be stored in a file with three columns (x, y and z) - lets say a data file is named ‘data’. An easy way to analyze and extract calibration parameters is by using Gnuplot. The gnuplot command `splot ‘data’ 1:2:3` will show a 3 dimensional plot of the data points, rotating the plot with the four arrow keys will show if the data has a basic spherical shape with all corners of the sphere represented. 
    
<img src="https://user-images.githubusercontent.com/32702163/225654806-caa31fa8-141b-4957-ada6-b45d7e906bfa.png" width="400" />   
The gnuplot command `stats ‘data’ using 1` will show statistical calculations for column 1 including min and max. The min and max numbers for each of the axes are the calibration data - which will need to be entered into the program. Once the calibration data is entered and new calibration data collected, the new data displayed with gnuplot should show a sphere of radius 1 centered at 0,0,0.
```C
#define PI 3.14159
int qmc5883_read () {
    //calibrate data
    int xmax = 7152; int xmin = -5632;
    int ymax = 6327; int ymin = -7112;
    int zmax = 5785; int zmin = -6085;
    //read 6 bytes of i2c data from qmc5883
    uint8_t regdata[6];
    regdata[0] = 0x00;
    i2c_write_blocking (i2c1, QMC5883L_I2C_ADDR, regdata, 1, false);
    i2c_read_blocking (i2c1, QMC5883L_I2C_ADDR, regdata, 6, false); 
    //assemble bytes
    int x = 256 * regdata[1] + regdata[0]; if(regdata[1]>=128) x = x - (1 << 16);
    int y = 256 * regdata[3] + regdata[2]; if(regdata[3]>=128) y = y - (1 << 16);
    int z = 256 * regdata[5] + regdata[4]; if(regdata[5]>=128) z = z - (1 << 16);
    printf(" %7d   %7d   %7d        ", x, y, z);   
    //apply calibration data + normalize 
    float xmag = ((float) (x - 0.5 * (xmax + xmin)) / ( 0.5 * (xmax - xmin)));
    float ymag = ((float) (y - 0.5 * (ymax + ymin)) / ( 0.5 * (ymax - ymin)));   
    float zmag = ((float) (z - 0.5 * (zmax + zmin)) / ( 0.5 * (zmax - zmin)));    
    printf(" %6.3f   %6.3f   %6.3f     ", xmag, ymag, zmag);
    //calulate field magnitude
    radius = sqrt ((xmag * xmag) + (ymag * ymag) + (zmag * zmag));
    //calculate vertical component of field vector 
    if (zmag > 0) theta = atan ((xmag * xmag + ymag * ymag) / zmag);
    else if (zmag < 0) theta = PI + atan ((xmag * xmag + ymag * ymag) / zmag);
    else theta = PI / 2;
    //calculate horizontal component of field vector
    if(xmag > 0) psi = atan (ymag / xmag);
    else if(xmag < 0 && ymag >=0) psi = PI + atan(ymag / xmag);
    else if(xmag < 0 && ymag < 0) psi = -1.0 * PI + atan(ymag / xmag);
    else if(xmag == 0 && ymag >= 0) psi = PI/2;
    else psi = -1.0 * PI / 2;
    printf ("radius=%5.2f theta=%5.2f psi=%5.2f pres=%.1f\n", radius, theta, psi,pres); 
    return (0);
}
```
If the Qmc5883 is mounted free of major field influencers seems to calculate very resonable values of Declination (hoizontal offset from true north - positive east) and Inclination (from horizontal) compared to the https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm website. In this case the calculation of compass direction works out pretty well just calculating the direction from the value of psi (north = o radians). If the local geomagnetic field is pretty distorted in any given setup, I could imagine things get pretty complicated, however, I did have about 10 degrees accuracy on a buggy project I did a while ago calibrating only the x and y axes - in this case the qmd5883 was kept within a couple degrees of flat.


    
