# rp2040-imu

## RPi PICO / RP2040 IMU

#### The IMU shown here uses two native rp2040 buses, one SPI and one I2C, to support a 6 axis accelerometer/gyroscope, a 3 axis magnetometer, an atmospheric  pressure measurement chip, a 1 inch OLED display and a momentary contact switch. This implementation uses the PICO SDK and programs written in C, a Seeedi np2040, mpu6500, qmc5883, bmp280 and a ssd1306 module breadboard assemblys. OLED screen updates are made twice a second and include fused filtered accelerometer/gyroscope pitch/roll, magnetic heading and a pressure based altitude measurements. A usb serial port can supply higher data measurement rates: magnetic readings up to 200HZ, altitude to 100Hz and pitch/roll at over 1KHz. 
#### The OLED Display of the summary data is shown below, it includes direction azimuth, direction in degrees, elevation in feet, pitch in degrees and roll in degrees.
![IMG_20230323_105721661](https://user-images.githubusercontent.com/32702163/227251581-0e927ba0-ca8e-4c49-a5fd-5fabf607c2ee.jpg)
#### Pushing the momentary contact SW0 switch changes to an alternate display and zeros out the accelerometer, gyroscope and altitude calibration offsets. This display provides a few more details; the magnetic field vector components - magnitude (normalized to 1g) as well as the calculated theta and psi in radians. The pressure is shown in millibars as well as altitude change since last calibrate. The pitch and roll are from the mpu6500 are shown in degrees and the system time in seconds.
![IMG_20230323_105805364](https://user-images.githubusercontent.com/32702163/227251731-f382c114-112a-45ad-b3fe-02ca2322ffc2.jpg)
## Sections
## Gyroscope and Accelerometer
## Magnetometer
## BMP280 Atmospheric Pressure Sensor
## SSD1306 OLED Display
## RPi PICO/RP2040 SSD Framework
#

## Gyroscope and Accelerometer

#### The mpu6500 is a combined three axis gyroscope and three axis accelerometer. Which is used to supply a 1000 samples of each measurement every second. The gyroscope measurements from this device have a strong tendency to drift and the accelerometer measurements tend to be relatively noisy in the short term and stable in the long term. The gyroscope and accelerometer measurements are filtered (high pass for gyroscope and low pass for accelerometer) and combined. The results are pitch (forward/backward tilt) and roll (left/right tilt) measurements that filter out the low frequency drift of the gyroscope measurements and the frequency noise of the accelerometers.
#### The following code lines show the basis of the pitch and roll calculation. The rate is the number of spi (12 byte) reads per second. The tau is the time constant of the gyroscope high frequency and also the time constant of the accelerometer low pass filter in seconds. Fusion_T is the digital filter multiplier for the high frequency dsp filter and (1 - Fusion_T) is the dsp multiplier for the low frequency dsp filter.
```C
    float rate = 0.01;      //sample rate in seconds
    float tau = .5;         //lp and hp filter time constant (sec)
    float Fusion_T = (tau/rate)/(1+tau/rate);
    
    pitch = Fusion_T * (pitch + xGyro) + (1 - Fusion_T) *  57.3 * asin(yAccl);
    roll =  Fusion_T * (roll  + yGyro) + (1 - Fusion_T) * -57.3 * asin(xAccl);
```
   
#### The pitch and roll calculations need to be implemented with forethought. The spi measurements must be calculated at a constant rate with no imteruptions and relatively frequently (at least 100 times per second). Each measurement requires several spi reads and about a dozen floating point and two trigonometric calculations per loop (which are slow on this device - no fpu). These measurements can not be in the main() while loop with the i2c measurements because of the time required to serve the i2c devices (especially the OLED). Off loading the spi measurements to the second core is probably the least painful solution (it might also ne possible by using mpu6500 fifo, to dma with isr).

#### Here is how a sample of data collected by rotating the circuit board through about 90 degree rotations on my desk looks:

![raw](https://user-images.githubusercontent.com/32702163/226621528-23cd7ea7-f89f-480c-a736-9c17a7f96dde.png)

#### The graph above is time vs yaxis gyroscope and `(360/2*pi) * asin(x axis accelerometer)`, the arcsin is used to convert the normalized x axis accelerometer gravitation measurement to an angle in degrees. Notice that the integrated gyroscope measurement gives a good rendition of the shape of the action, but the dc (drift) component is a problem. The Accelerometer is no better behaved by being very noisy. 

##### *The max/min clipping on the accelerometer derived clipping is induced by the simple 'one point' calibration. Which calibrates a simple offset for each acceleration axis, calibrating out each axis gain is painful and would probably only increase accuracy on order or 20% (and that only when pitch/roll > 60 degrees).
##### *For angles less than about 10 degrees pitch = asin(Accelx) ~ Accelx. 

#
#### This next graph plots the same signals above but with simple dsp filters with a 0.2 second time constant, the gyroscope measurements are high passed filtered to remove the dc component and the acceleration measurement are lp passed filtered to average the noise.

 ![filter](https://user-images.githubusercontent.com/32702163/226628739-1f5bbe45-8375-4f88-8c93-83385981b177.png)
   
#### This last graph shows the sum of the two plots in the above graph and is the result of the pitch angle measurement. Roll is very similar but with y axis acceleration and x axis gyroscope.

![pitch](https://user-images.githubusercontent.com/32702163/226628863-3cab0088-eb16-4a52-90a1-b1ca5994b614.png)

#
#### The execution time result for the mpu6500 read/calculation loop was about 500 usec for the spi data read; the pitch and roll floating point and trigonometric calculations clock in at about 60usec. With loop execution time of 1msec, running a 1KHz pitch/roll calculation rate should be no problem (it did work fine to 1.8KHz).

##### *Higher rates - up to 8K - are likely possible by optimizing program; 2K easy, higher rates would likely require using mcu6500 internal fifo. 

#
## MAGNETOMETER

#### The Qmc5883 is a three axis magneto-resistive (hall effect) sensor, with 16 bit dacs with 2milli-gauss resolution, with a 200Hz maximum sample rate. In addition to providing compass direction readings (degrees from north) it can also provides position data much like the three axis accelerometer, instead of measuring vector toward the center of earth it measures the vector toward the earth's local magnetic field.

#### The magnetic field of the earth at any one point is not large compared to say a magnet or a laptop computer a foot away, the field is also easily perturbed by ferroelectric materials. Many of these effects can be counteracted to an extent by calibrating after final placement in circuit, but the nicest use of the device is to isolate it from anything magnetic, ferroelectric, massive  or carrying current. Avoid breadboards and the 100mm header pins. A non-magnetized iron nail four inches away changes the measured fields by  >10% - leading to big changes in local field and vector calculations (falls off inverse cube?). A quarter, glass or a hand has basically no affect.

#### Calibration of the magnetometer requires the collection of a fairly substantial amount of data collected from the magnetometer while the device is being physically pitched, rolled and yawed over 360 degrees on three axes (there must be a trick to this). The data stored in a file with three columns (x, y and z) - lets say a data file is named ‘data’ - is easily reviewed with gnuplot to analyze and extract calibration parameters. The gnuplot command `splot 'data’ 1:2:3` will show a 3 dimensional plot of the data points, rotating the plot with the four arrow keys will show if the data has a basic spherical shape with all corners of the sphere represented. 
    
<img src="https://user-images.githubusercontent.com/32702163/225654806-caa31fa8-141b-4957-ada6-b45d7e906bfa.png" width="400" />  

#### The gnuplot command `stats ‘data’ using 1` will show statistical calculations for column 1 including min and max. The min and max numbers for each of the axes are the calibration data - which will be entered into the program. Once the calibration data is entered and new calibration data collected, the new data displayed with gnuplot should show a sphere of radius 1 centered at 0,0,0.
 
 #### This is the code that runs each time new magnetic heading data is requested. The degrees is the number of degrees from north increasing in clockwise direction until wrapping at 360 degrees. The integer direction is used to look up one of 16 azimuth strings (N, NNE, NE, ENE, etc).
```C
#define PI 3.14159
int qmc5883_read () {
    int xmax = 7152; int xmin = -5632;
    int ymax = 6327; int ymin = -7112;
    int zmax = 5785; int zmin = -6085;   
    uint8_t regdata[8];
    //read six bytes starting at 0x00
    regdata[0] = 0x00;
    i2c_write_blocking (i2c1, QMC5883L_I2C_ADDR, regdata, 1, false);
    i2c_read_blocking (i2c1, QMC5883L_I2C_ADDR, regdata, 6, false);

    int x = 256 * regdata[1] + regdata[0]; if(regdata[1]>=128) x = x - (1 << 16);
    int y = 256 * regdata[3] + regdata[2]; if(regdata[3]>=128) y = y - (1 << 16);
    int z = 256 * regdata[5] + regdata[4]; if(regdata[5]>=128) z = z - (1 << 16);
    
    float xmag = ((float) (x - 0.5 * (xmax + xmin)) / ( 0.5 * (xmax - xmin)));
    float ymag = ((float) (y - 0.5 * (ymax + ymin)) / ( 0.5 * (ymax - ymin)));
    float zmag = ((float) (z - 0.5 * (zmax + zmin)) / ( 0.5 * (zmax - zmin)));

    radius = sqrt ((xmag * xmag) + (ymag * ymag) + (zmag * zmag));
    
    if (zmag > 0) theta = atan ((xmag * xmag + ymag * ymag) / zmag);
    else if (zmag < 0) theta = PI + atan ((xmag * xmag + ymag * ymag) / zmag);
    else theta = PI / 2;
    
    if(xmag > 0) psi = atan (ymag / xmag);
    else if(xmag < 0 && ymag >=0) psi = PI + atan(ymag / xmag);
    else if(xmag < 0 && ymag < 0) psi = -1.0 * PI + atan(ymag / xmag);
    else if(xmag == 0 && ymag >= 0) psi = PI/2;
    else psi = -1.0 * PI / 2;

    degrees = ((int)(360 + declination + (57.3 * psi)) % 360);
    direction = (int) ((azoffset + degrees) / 22.5);
}
```

##### The qmc5883 mounted free of major field influences seems to calculate very reasonable values of Declination (horizontal offset from true north - positive east) and Inclination (from horizontal) compared to the https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm website. In this case the calculation of compass direction works out pretty well just calculating the direction from the value of psi (north = 0 radians + declination). 
##### *If the local geomagnetic field is pretty distorted in any given setup, I could imagine things get pretty complicated, however, I did have about 10 degrees accuracy on a buggy project I did a while ago calibrating only the x and y axes - in this case the qmc5883 was kept within a couple degrees of flat.

#
## BMP280 ATMOSPHERIC PRESSURE SENSOR
#### Calibration, to factory settings, of this device requires transferring 24 bytes of calibration offset data fused into the device by the manufacturer. The pressure and temperature data read from the device along with the 24 calibration numbers are used to calculate the actual pressure value in hPa (=millibar). The device can be used to measure fluctuations in atmospheric pressure (which has interesting subtleties); additionally, once calibrated to local pressure, it can calculate altitude changes accurately to unit feet (10 feet = .38 millibar).

#### Once the 24 bytes of calibration data are read and converted into 12 calibration variables (digT1-3 and digP1-9) the calculation of ctemp and pres and proceed (from datasheet):
#

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
#
## SSD1306 OLED DISPLAY
#### This project uses a custom software that has been with me a long time. An 8x5 bit font array is loaded that is used to extract eight single or double sized pixel images to be transferred to the display. The configuration of the SSD1306 is completed with the ssd1306_init() command. The command ssd1306_text(str_text) writes text to the display - examples:
```C
   sprintf(disp_string, “4Top Line 2X|||1  value1=%d|   value2=%4.2fmsec||4LastLine”, 17, 2.71828);
   ssd1306(disp_sting);
```

#### This will cause Top Line to be displayed at beginning of top line in double size, then it advances three lines - two lines to make up for the two lines used and a blank line, the next line is in small text and displays <space>value1=<value1>, then another line advance for the line used and <3xspace>value2=<value2 float> and finally LastLine is printed out in double size. Basically, any ‘|’ resets column pointer, each ‘|’ advances row pointer, when number 1 or 4 is in column 0 position text is either printed single or doubled size. It looks like this:

<img src="https://user-images.githubusercontent.com/32702163/225639441-afbaed19-081f-4ff9-8ac0-c9b88f4c7ab9.png" width="200"/>

#### One of the RP2040 pins (RP2040 gpio pin 0) powers the SSD1306 VCC pin as a means of resetting the SSD1306 memory pointer; the device requires reconfiguration after reset.

#
## RPi PICO/RP2040 SSD FRAMEWORK

#### This project uses the PICO SDK (in a linux environment), for simplicity without any specific IDE. The installation of the SDK is well documented here https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html. This github repository contains all other c programs, cmake files, shell scripts and include files required. The I2C SCK is on RP2040 gpio pin 7 and SDA is on gpio pin 6.

#### The steps required to run the project from my linux directory are:

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

##### Note: the shell scripts setenv.sh and flash.sh may contain paths or files specific to your build, for example I installed SDK at ~/pico-sdk and my serial port shows up at /dev/ttyACM0.
# 

#### The setenv.sh script is used to install environment variables into the current terminal window shell. The flash.sh file 1)waits for /dev/ttyACM0 to be present, 2)stty’s ‘magic’ baud rate of 1200 which puts the rp2040 into boot mode and presents flash device as mass storage device, 3) wait for mass storage device to mount at /media/<name>/RPI-RP2, 4) writes picocode.ef2 file to /media/<name>/RPI-RP2, 5)waits for /dev/ttyACM0 to reconnect and 6)sets up monitor with correct stty settings, cntl C exits monitor. Piping output to | tee data1 will display the output and save it to a file named ‘data’.

##### *this file is cut and paste from README.md from https://github.com/baetis-ma/rp2040-imu

