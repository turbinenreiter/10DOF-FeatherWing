/* based on:
  BMX055_MS5637_t3 Basic Example Code
  by: Kris Winer
  date: August 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.

  Modified for the 10DofOne by Pontus Oldberg
  date: January, 2016
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.

  Modified for the 10-DOF WING for Adalooger M0 by Sebastian Plamauer
  date: February, 2016
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
*/

void getGres()
{
    switch (Gscale)
    {
        // Possible gyro scales (and their register bit settings) are:
        // 125 DPS (100), 250 DPS (011), 500 DPS (010), 1000 DPS (001), and 2000 DPS (000).
    case GFS_125DPS:
        gRes = 124.87/32768.0; // per data sheet, not exactly 125!?
        break;
    case GFS_250DPS:
        gRes = 249.75/32768.0;
        break;
    case GFS_500DPS:
        gRes = 499.5/32768.0;
        break;
    case GFS_1000DPS:
        gRes = 999.0/32768.0;
        break;
    case GFS_2000DPS:
        gRes = 1998.0/32768.0;
        break;
    }
}

void getAres()
{
    switch (Ascale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (0011), 4 Gs (0101), 8 Gs (1000), and 16 Gs  (1100).
        // BMX055 ACC data is signed 12 bit
    case AFS_2G:
        aRes = 2.0/2048.0;
        break;
    case AFS_4G:
        aRes = 4.0/2048.0;
        break;
    case AFS_8G:
        aRes = 8.0/2048.0;
        break;
    case AFS_16G:
        aRes = 16.0/2048.0;
        break;
    }
}


void readAccelData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(BMX055_ACC_ADDRESS, BMX055_ACC_D_X_LSB, 6, &rawData[0]);       // Read the six raw data registers into data array
    if((rawData[0] & 0x01) && (rawData[2] & 0x01) && (rawData[4] & 0x01))    // Check that all 3 axes have new data
    {
        destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 4;  // Turn the MSB and LSB into a signed 12-bit value
        destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 4;
        destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 4;
    }
}

void readGyroData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(BMX055_GYRO_ADDRESS, BMX055_GYRO_RATE_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);
}

void readMagData(int16_t * magData)
{
    int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
    uint16_t data_r = 0;
    uint8_t rawData[8];  // x/y/z hall magnetic field data, and Hall resistance data

    readBytes(BMX055_MAG_ADDRESS, BMX055_MAG_XOUT_LSB, 8, &rawData[0]);  // Read the eight raw data registers sequentially into data array
    if(rawData[6] & 0x01)   // Check if data ready status bit is set
    {
        mdata_x = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 3;  // 13-bit signed integer for x-axis field
        mdata_y = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 3;  // 13-bit signed integer for y-axis field
        mdata_z = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 1;  // 15-bit signed integer for z-axis field
        data_r = (uint16_t) (((uint16_t)rawData[7] << 8) | rawData[6]) >> 2;  // 14-bit unsigned integer for Hall resistance
        // calculate temperature compensated 16-bit magnetic fields
        temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
        magData[0] = ((int16_t)((((int32_t)mdata_x) *
                     ((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
                     (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
                     ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_x2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
                     (((int16_t)dig_x1) << 3);
        temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
        magData[1] = ((int16_t)((((int32_t)mdata_y) *
                     ((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
                     (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
                     ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_y2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
                     (((int16_t)dig_y1) << 3);
        magData[2] = (((((int32_t)(mdata_z - dig_z4)) << 15) - ((((int32_t)dig_z3) * ((int32_t)(((int16_t)data_r) -
                     ((int16_t)dig_xyz1))))>>2))/(dig_z2 + ((int16_t)(((((int32_t)dig_z1) * ((((int16_t)data_r) << 1)))+(1<<15))>>16))));
    }
}

int16_t readACCTempData()
{
    uint8_t c =  readByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_TEMP);  // Read the raw data register
    return ((int16_t)((int16_t)c << 8)) >> 8 ;  // Turn the byte into a signed 8-bit integer
}

void trimBMX055()  // get trim values for magnetometer sensitivity
{
    uint8_t rawData[2];  //placeholder for 2-byte trim data
    dig_x1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X1);
    dig_x2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X2);
    dig_y1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y1);
    dig_y2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y2);
    dig_xy1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY1);
    dig_xy2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY2);

    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z1_LSB, 2, &rawData[0]);
    dig_z1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);

    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z2_LSB, 2, &rawData[0]);
    dig_z2 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);

    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z3_LSB, 2, &rawData[0]);
    dig_z3 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);

    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z4_LSB, 2, &rawData[0]);
    dig_z4 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);

    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_XYZ1_LSB, 2, &rawData[0]);
    dig_xyz1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);
}


void initBMX055()
{
    // start with all sensors in default mode with all registers reset
    writeByte(BMX055_ACC_ADDRESS,  BMX055_ACC_BGW_SOFTRESET, 0xB6);  // reset accelerometer
    delay(1000); // Wait for all registers to reset

    // Configure accelerometer
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_RANGE, Ascale & 0x0F); // Set accelerometer full range
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_BW, ACCBW & 0x0F);     // Set accelerometer bandwidth
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_HBW, 0x00);              // Use filtered data

//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_EN_1, 0x10);           // Enable ACC data ready interrupt
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_OUT_CTRL, 0x04);       // Set interrupts push-pull, active high for INT1 and INT2
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_MAP_1, 0x02);        // Define INT1 (intACC1) as ACC data ready interrupt
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_MAP_1, 0x80);          // Define INT2 (intACC2) as ACC data ready interrupt

//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_BGW_SPI3_WDT, 0x06);       // Set watchdog timer for 50 ms

// Configure Gyro
// start by resetting gyro, better not since it ends up in sleep mode?!
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BGW_SOFTRESET, 0xB6); // reset gyro
// delay(100);
// Three power modes, 0x00 Normal,
// set bit 7 to 1 for suspend mode, set bit 5 to 1 for deep suspend mode
// sleep duration in fast-power up from suspend mode is set by bits 1 - 3
// 000 for 2 ms, 111 for 20 ms, etc.
//  writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM1, 0x00);  // set GYRO normal mode
//  set GYRO sleep duration for fast power-up mode to 20 ms, for duty cycle of 50%
//  writeByte(BMX055_ACC_ADDRESS, BMX055_GYRO_LPM1, 0x0E);
// set bit 7 to 1 for fast-power-up mode,  gyro goes quickly to normal mode upon wake up
// can set external wake-up interrupts on bits 5 and 4
// auto-sleep wake duration set in bits 2-0, 001 4 ms, 111 40 ms
//  writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM2, 0x00);  // set GYRO normal mode
// set gyro to fast wake up mode, will sleep for 20 ms then run normally for 20 ms
// and collect data for an effective ODR of 50 Hz, other duty cycles are possible but there
// is a minimum wake duration determined by the bandwidth duration, e.g.,  > 10 ms for 23Hz gyro bandwidth
//  writeByte(BMX055_ACC_ADDRESS, BMX055_GYRO_LPM2, 0x87);

    writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_RANGE, Gscale);  // set GYRO FS range
    writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BW, GODRBW);     // set GYRO ODR and Bandwidth

// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_EN_0, 0x80);  // enable data ready interrupt
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_EN_1, 0x04);  // select push-pull, active high interrupts
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_MAP_1, 0x80); // select INT3 (intGYRO1) as GYRO data ready interrupt

// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BGW_SPI3_WDT, 0x06); // Enable watchdog timer for I2C with 50 ms window


// Configure magnetometer
    writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x82);  // Softreset magnetometer, ends up in sleep mode
    delay(100);
    writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // Wake up magnetometer
    delay(100);

    writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3); // Normal mode
//writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3 | 0x02); // Forced mode

//writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_INT_EN_2, 0x84); // Enable data ready pin interrupt, active high

// Set up four standard configurations for the magnetometer
    switch (Mmode)
    {
    case lowPower:
        // Low-power
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x01);  // 3 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x02);  // 3 repetitions (oversampling)
        break;
    case Regular:
        // Regular
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x04);  //  9 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x16);  // 15 repetitions (oversampling)
        break;
    case enhancedRegular:
        // Enhanced Regular
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x07);  // 15 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x22);  // 27 repetitions (oversampling)
        break;
    case highAccuracy:
        // High Accuracy
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x17);  // 47 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x51);  // 83 repetitions (oversampling)
        break;
    }
}

void fastcompaccelBMX055(float * dest1)
{
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x80); // set all accel offset compensation registers to zero
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_SETTING, 0x20);  // set offset targets to 0, 0, and +1 g for x, y, z axes
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x20); // calculate x-axis offset

    byte c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
    while(!(c & 0x10))     // check if fast calibration complete
    {
        c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        delay(10);
    }
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x40); // calculate y-axis offset

    c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
    while(!(c & 0x10))     // check if fast calibration complete
    {
        c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        delay(10);
    }
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x60); // calculate z-axis offset

    c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
    while(!(c & 0x10))     // check if fast calibration complete
    {
        c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        delay(10);
    }

    int8_t compx = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_X);
    int8_t compy = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Y);
    int8_t compz = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Z);

    dest1[0] = (float) compx/128.; // accleration bias in g
    dest1[1] = (float) compy/128.; // accleration bias in g
    dest1[2] = (float) compz/128.; // accleration bias in g
}

void magcalBMX055(float * dest1)
{
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0};
    int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

    Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    delay(1);

    sample_count = 16;
    for(ii = 0; ii < sample_count; ii++)
    {
        int16_t mag_temp[3] = {0, 0, 0};
        readMagData(mag_temp);
        for (int jj = 0; jj < 3; jj++)
        {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
    }

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes;
    dest1[2] = (float) mag_bias[2]*mRes;

    Serial.println("Mag Calibration done!");
}

// I2C read/write functions for the IMU

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data;                    // `data` will store the register data

    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
    data = Wire.read();               // Fill Rx buffer with result
    return data;                     // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
    while (Wire.available())
    {
        dest[i++] = Wire.read();      // Put read results in the Rx buffer
    }
}

