#include <Arduino.h>
#include <Wire.h>
#include <SSC.h>
#include <SPI.h>
#include <SdFat.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Adafruit_MAX31855.h"
#include <dht.h>
#include <SparkFunLSM9DS1.h>

/**********************************GPS*****************************************/
static const int RXPin = 10, TXPin = 8;
static const uint32_t GPSBaud = 4800;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

static void smartDelay(unsigned long);
void printAttitude(float, float, float, float, float, float);

/****************************Pressure Sensor***********************************/
// Create an SSC sensor with I2C address 0x28 and power pin 53.
// I picked an unused pin because I want the sensor to always be on.
SSC ssc(0x28, 53);

/******************************Thermocouple************************************/
#define MAXVIN  49
#define MAXCS   47

Adafruit_MAX31855 thermocouple(MAXCS);

/********************************Humidity**************************************/
#define DHTVIN  31
#define DHTPIN  33
dht DHT;

/**********************************9DoF****************************************/
LSM9DS1 imu;
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

#define DECLINATION 3.61965

/********************************SD Card***************************************/
// SD chip select pin.
const uint8_t chipSelect = 4;

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Data"

// File system object.
SdFat sd;

// Log file.
SdFile data;

/*********************************Setup****************************************/
void setupGyro()
{
    imu.settings.gyro.enabled = true;  // Enable the gyro
    // [scale] sets the full-scale range of the gyroscope.
    // scale can be set to either 245, 500, or 2000
    imu.settings.gyro.scale = 2000; // Set scale to +/-2000dps
}

void setupAccel()
{
    // [enabled] turns the acclerometer on or off.
    imu.settings.accel.enabled = true; // Enable accelerometer
    // [enableX], [enableY], and [enableZ] can turn on or off
    // select axes of the acclerometer.
    imu.settings.accel.enableX = true; // Enable X
    imu.settings.accel.enableY = true; // Enable Y
    imu.settings.accel.enableZ = true; // Enable Z
    // [scale] sets the full-scale range of the accelerometer.
    // accel scale can be 2, 4, 8, or 16
    imu.settings.accel.scale = 16; // Set accel scale to +/-16g.
}

void setupMag()
{
    // [enabled] turns the magnetometer on or off.
    imu.settings.mag.enabled = true; // Enable magnetometer
    // [scale] sets the full-scale range of the magnetometer
    // mag scale can be 4, 8, 12, or 16
    imu.settings.mag.scale = 16; // Set mag scale to +/-16 Gs
    // [tempCompensationEnable] enables or disables
    // temperature compensation of the magnetometer.
    imu.settings.mag.tempCompensationEnable = true;
}

void setupTemperature()
{
  // [enabled] turns the temperature sensor on or off.
  imu.settings.temp.enabled = true;
}

void setup()
{
  Serial.begin(230400);

///////////////// SD Card ///////////////////
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    sd.initErrorHalt();
  }

  if (!data.open("data.csv", O_RDWR | O_CREAT | O_APPEND)) {
    sd.errorHalt("opening test.txt for write failed");
  }

  data.println("TIME,SATS,LATITUDE,LONGITUDE,ALT,COURSE,SPEED,PRESSURE,INTTEMP,EXTTEMP,HUMIDITY,GX,GY,GZ,AX,AY,AZ,MX,MY,MZ");
  Serial.println("TIME,SATS,LATITUDE,LONGITUDE,ALT,COURSE,SPEED,PRESSURE,INTTEMP,EXTTEMP,HUMIDITY,GX,GY,GZ,AX,AY,AZ,MX,MY,MZ");

  /////////////////// GPS ///////////////////
  ss.begin(GPSBaud);

  /////////////////// Sensor ///////////////////
  Wire.begin();

  //  set min / max reading and pressure, see datasheet for the values for your sensor
  ssc.setMinRaw(1638);
  ssc.setMaxRaw(14746);
  ssc.setMinPressure(0.0);
  ssc.setMaxPressure(1034.21);

  //  start the sensor
  ssc.start();

  // Set MAXVIN to HIGH. This provides +5v to power our thermocouple
  pinMode(MAXVIN, OUTPUT);
  digitalWrite(MAXVIN, HIGH);

  pinMode(DHTVIN, OUTPUT);
  digitalWrite(DHTVIN, HIGH);

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  setupGyro();          // Set up gyroscope parameters
  setupAccel();         // Set up accelerometer parameters
  setupMag();           // Set up magnetometer parameters
  setupTemperature();   // Set up temp sensor parameter

  imu.begin();
}

/*********************************LOOP*****************************************/
void loop()
{
  //  update pressure
  ssc.update();

  // update humidity
  DHT.read22(DHTPIN);

  // update IMU data
  if (imu.accelAvailable())
  {
    imu.readAccel();
  }

  // imu.gyroAvailable() returns 1 if new gyroscope
  // data is ready to be read. 0 otherwise.
  if (imu.gyroAvailable())
  {
    imu.readGyro();
  }

  // imu.magAvailable() returns 1 if new magnetometer
  // data is ready to be read. 0 otherwise.
  if (imu.magAvailable())
  {
    imu.readMag();
  }

  Serial.print(gps.time.value()); // Raw time in HHMMSSCC format (u32)
  Serial.print(",");
  Serial.print(gps.satellites.value()); // Number of satellites in use (u32)
  Serial.print(",");
  Serial.print(gps.location.lat(), 6); // Latitude in degrees (double)
  Serial.print(",");
  Serial.print(gps.location.lng(), 6); // Longitude in degrees (double)
  Serial.print(",");
  Serial.print(gps.altitude.feet()); // Altitude in feet (double)
  Serial.print(",");
  Serial.print(gps.course.deg()); // Course in degrees (double)
  Serial.print(",");
  Serial.print(gps.speed.mph()); // Speed in miles per hour (double)
  Serial.print(",");
  Serial.print(ssc.pressure() + 27.6);
  Serial.print(",");
  Serial.print(thermocouple.readInternal());
  Serial.print(",");
  Serial.print(thermocouple.readCelsius());
  Serial.print(",");
  Serial.print(DHT.humidity);
  Serial.print(",");
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(",");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(",");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.print(",");
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(",");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(",");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.print(",");
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(",");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(",");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println("");

  data.print(gps.time.value()); // Raw time in HHMMSSCC format (u32)
  data.print(",");
  data.print(gps.satellites.value()); // Number of satellites in use (u32)
  data.print(",");
  data.print(gps.location.lat(), 6); // Latitude in degrees (double)
  data.print(",");
  data.print(gps.location.lng(), 6); // Longitude in degrees (double)
  data.print(",");
  data.print(gps.altitude.feet()); // Altitude in feet (double)
  data.print(",");
  data.print(gps.course.deg()); // Course in degrees (double)
  data.print(",");
  data.print(gps.speed.mph()); // Speed in miles per hour (double)
  data.print(",");
  data.print(ssc.pressure() + 27.6);
  data.print(",");
  data.print(thermocouple.readInternal());
  data.print(",");
  data.print(thermocouple.readCelsius());
  data.print(",");
  data.print(DHT.humidity);
  data.print(",");
  data.print(imu.calcGyro(imu.gx), 2);
  data.print(",");
  data.print(imu.calcGyro(imu.gy), 2);
  data.print(",");
  data.print(imu.calcGyro(imu.gz), 2);
  data.print(",");
  data.print(imu.calcAccel(imu.ax), 2);
  data.print(",");
  data.print(imu.calcAccel(imu.ay), 2);
  data.print(",");
  data.print(imu.calcAccel(imu.az), 2);
  data.print(",");
  data.print(imu.calcMag(imu.mx), 2);
  data.print(",");
  data.print(imu.calcMag(imu.my), 2);
  data.print(",");
  data.print(imu.calcMag(imu.mz), 2);
  data.println("");

  if (!data.sync() || data.getWriteError()) {
    Serial.println("write error");
  }

  smartDelay(500);
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
