
void qmc5883_init() {
    uint8_t data[] = { 0x09, 0x01, 0x00, 0x01 };
    i2c_write_blocking (i2c1, QMC5883L_I2C_ADDR, data, 4, false);
    //read back
    data[0] = 0x09;
    i2c_write_blocking (i2c1, QMC5883L_I2C_ADDR, data, 1, false);
    i2c_read_blocking (i2c1, QMC5883L_I2C_ADDR, data, 4, false); 
    //printf("9 %x %x %x %x", data[0], data[1], data[2], data[3]);
}

#define PI 3.14159
int qmc5883_read () {
    int xmax = 7152; int xmin = -5632;
    int ymax = 6327; int ymin = -7112;
    int zmax = 5785; int zmin = -6085;

    uint8_t regdata[8];
    regdata[0] = 0x00;
    i2c_write_blocking (i2c1, QMC5883L_I2C_ADDR, regdata, 1, false);
    i2c_read_blocking (i2c1, QMC5883L_I2C_ADDR, regdata, 6, false); 
    //for (int a =0; a<6;a++)printf("0x%02x ",regdata[a]);// printf("\n");
    
    int x = 256 * regdata[1] + regdata[0]; if(regdata[1]>=128) x = x - (1 << 16);
    int y = 256 * regdata[3] + regdata[2]; if(regdata[3]>=128) y = y - (1 << 16);
    int z = 256 * regdata[5] + regdata[4]; if(regdata[5]>=128) z = z - (1 << 16);
    //printf(" %7d   %7d   %7d        ", x, y, z);   
     
    float xmag = ((float) (x - 0.5 * (xmax + xmin)) / ( 0.5 * (xmax - xmin)));
    float ymag = ((float) (y - 0.5 * (ymax + ymin)) / ( 0.5 * (ymax - ymin)));   
    float zmag = ((float) (z - 0.5 * (zmax + zmin)) / ( 0.5 * (zmax - zmin)));    
    //printf(" %6.3f   %6.3f   %6.3f     ", xmag, ymag, zmag);
    
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
    direction = (int) (((int)(360 + degrees + 11.25)  % 360) / 22.5);
    return (0);
}

