/* flightSW_Feather
  including:
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

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#include "BMX055.h"
#include "Wire.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SD.h>
#include <Servo.h>

#define DEBUG
#define LOG

Servo myservo;

const int chipSelect = 4;
const int myLed = 15; // red
const int ServoLed = 16; //green

float t, ax, ay, az, gx, gy, gz, mx, my, mz, T, p, alt; // variables to hold latest sensor data values
float out[] = {t, ax, ay, az, gx, gy, gz, mx, my, mz, T, p, alt};

int t0 = 0;
int para_flag = 0;

File dataFile;

Adafruit_BME280 baro; // on add 76 - change in lib

// BMX055 stuff
// Specify sensor full scale
uint8_t Gscale = GFS_2000DPS;       // set gyro full scale
uint8_t GODRBW = G_200Hz23Hz;      // set gyro ODR and bandwidth
uint8_t Ascale = AFS_16G;           // set accel full scale
uint8_t ACCBW  = 0x08 | ABW_63Hz;  // Choose bandwidth for accelerometer, need bit 3 = 1 to enable bandwidth choice in enum
uint8_t Mmode  = Regular;          // Choose magnetometer operation mode
uint8_t MODR   = MODR_10Hz;        // set magnetometer data rate
float aRes, gRes, mRes;            // scale resolutions per LSB for the sensors

// Parameters to hold BMX055 trim values
signed char   dig_x1;
signed char   dig_y1;
signed char   dig_x2;
signed char   dig_y2;
uint16_t      dig_z1;
int16_t       dig_z2;
int16_t       dig_z3;
int16_t       dig_z4;
unsigned char dig_xy1;
signed char   dig_xy2;
uint16_t      dig_xyz1;

// Pin definitions
// The BMX055 has three sensors, and two interrupt pins per device!
int intACC1   =  28;  // These are fixed on the BMX055 Mini Add-On for Teensy 3.1
int intACC2   =  29;
int intGYRO1  = 211;
int intGYRO2  = 210;
int intMAG1   = 212;
int intDRDYM  = 215;

// BMX055 variables
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 13/15-bit signed magnetometer sensor output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the BMX055 internal chip temperature in degrees Celsius
float SelfTest[6];            // holds results of gyro and accelerometer self test
int zerogcount = 0;

void setup()
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println(F("\nStarting BMX055 tests !"));

  pinMode(ServoLed, OUTPUT);
  digitalWrite(ServoLed, LOW);

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  delay(250);
  digitalWrite(myLed, LOW);
  delay(250);
  digitalWrite(myLed, HIGH);
  delay(250);
  digitalWrite(myLed, LOW);

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  byte c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_WHOAMI);  // Read ACC WHO_AM_I register for BMX055
  byte d = readByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_WHOAMI);  // Read GYRO WHO_AM_I register for BMX055
  writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // wake up magnetometer first thing
  delay(100);
  byte e = readByte(BMX055_MAG_ADDRESS, BMX055_MAG_WHOAMI);  // Read MAG WHO_AM_I register for BMX055

  if ((c == 0xFA) && (d == 0x0F) && (e == 0x32)) // WHO_AM_I should always be ACC = 0xFA, GYRO = 0x0F, MAG = 0x32
  {
    Serial.println("BMX055 is online...");

    initBMX055();
    // get sensor resolutions, only need to do this once
    getAres();
    getGres();
    // magnetometer resolution is 1 microTesla/16 counts or 1/1.6 milliGauss/count
    mRes = 1. / 1.6;
    trimBMX055();  // read the magnetometer calibration data

    fastcompaccelBMX055(accelBias);
    magcalBMX055(magBias);

  }
  else
  {
    Serial.print("Could not connect to BMX055: 0x");
    while (1) ; // Loop forever if communication doesn't happen
  }

  if (!baro.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  File dataFile = SD.open("log.csv", FILE_WRITE);

  if (dataFile) {
    dataFile.println("t, ax, ay, az, gx, gy, gz, mx, my, mz, T, p, alt");
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening log.csv");
    while (1);
  }

#ifndef DEBUG
  Serial.end();
#endif
  myservo.attach(5);
  myservo.write(90);
  delay(150);
  digitalWrite(myLed, HIGH);
}

void loop()
{
#ifdef LOG
  File dataFile = SD.open("log.csv", FILE_WRITE);
#endif
  t0 = millis();

  while (millis() < t0 + 1000) {

    t = millis() / 1000.0;

    readAccelData(accelCount);  // Read the x/y/z adc values
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes + accelBias[0]; // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes + accelBias[1];
    az = (float)accelCount[2] * aRes + accelBias[2];

    readGyroData(gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    readMagData(magCount);  // Read the x/y/z adc values
    // Calculate the magnetometer values in milliGauss
    // Temperature-compensated magnetic field is in 16 LSB/microTesla
    mx = (float)magCount[0] * mRes - magBias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes - magBias[1];
    mz = (float)magCount[2] * mRes - magBias[2];

    T = baro.readTemperature();
    p = baro.readPressure();
    alt = baro.readAltitude(1013.25);

    float out[] = {t, ax, ay, az, gx, gy, gz, mx, my, mz, T, p, alt};

    if (az < 0.1 && para_flag == 0 && zerogcount > 100) {
      myservo.write(45);              // tell servo to go to position in variable 'pos'
      para_flag = 1;
      digitalWrite(ServoLed, HIGH);
      delay(15);                       // waits 15ms for the servo to reach the position
    } else if (az < 0.1 && para_flag == 0) {
      zerogcount++;
    }

#ifdef DEBUG
    for (int i = 0; i < 13; i++) {
      Serial.print(out[i]);
      if (i < 12) {
        Serial.print(", ");
      }
    }
    Serial.println("");
#endif

#ifdef LOG
    for (int i = 0; i < 13; i++) {
      dataFile.print(out[i]);
      if (i < 12) {
        dataFile.print(",");
      }
    }
    dataFile.println("");
#endif
  }
#ifdef LOG
  dataFile.close();
#endif
}
