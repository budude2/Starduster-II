#include <Arduino.h>
#include <Wire.h>
#include <SdFat.h>
#include <SparkFunLSM9DS1.h>

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

void serial_printAttitude(float, float, float, float, float, float);
void data_printAttitude(float, float, float, float, float, float);

void setupGyro()
{
    imu.settings.gyro.enabled = true;  // Enable the gyro
    // [scale] sets the full-scale range of the gyroscope.
    // scale can be set to either 245, 500, or 2000
    imu.settings.gyro.scale = 2000; // Set scale to +/-2000dps
    // [sampleRate] sets the output data rate (ODR) of the gyro
    // sampleRate can be set between 1-6
    // 1 = 14.9    4 = 238
    // 2 = 59.5    5 = 476
    // 3 = 119     6 = 952
    imu.settings.gyro.sampleRate = 3; // 59.5Hz ODR
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
    // [highResEnable] enables or disables high resolution
    // mode for the acclerometer.
    imu.settings.accel.highResEnable = true; // Disable HR
}

void setupMag()
{
    // [enabled] turns the magnetometer on or off.
    imu.settings.mag.enabled = true; // Enable magnetometer
    // [scale] sets the full-scale range of the magnetometer
    // mag scale can be 4, 8, 12, or 16
    imu.settings.mag.scale = 16; // Set mag scale to +/-16 Gs
    // [sampleRate] sets the output data rate (ODR) of the
    // magnetometer.
    // mag data rate can be 0-7:
    // 0 = 0.625 Hz  4 = 10 Hz
    // 1 = 1.25 Hz   5 = 20 Hz
    // 2 = 2.5 Hz    6 = 40 Hz
    // 3 = 5 Hz      7 = 80 Hz
    imu.settings.mag.sampleRate = 7; // Set OD rate to 80Hz
    // [tempCompensationEnable] enables or disables
    // temperature compensation of the magnetometer.
    imu.settings.mag.tempCompensationEnable = true;
    // [XYPerformance] sets the x and y-axis performance of the
    // magnetometer to either:
    // 0 = Low power mode      2 = high performance
    // 1 = medium performance  3 = ultra-high performance
    imu.settings.mag.XYPerformance = 3; // Ultra-high perform.
    // [ZPerformance] does the same thing, but only for the z
    imu.settings.mag.ZPerformance = 3; // Ultra-high perform.
    // [lowPowerEnable] enables or disables low power mode in
    // the magnetometer.
    imu.settings.mag.lowPowerEnable = false;
    // [operatingMode] sets the operating mode of the
    // magnetometer. operatingMode can be 0-2:
    // 0 = continuous conversion
    // 1 = single-conversion
    // 2 = power down
    imu.settings.mag.operatingMode = 0; // Continuous mode
}

void setupTemperature()
{
  // [enabled] turns the temperature sensor on or off.
  imu.settings.temp.enabled = true;
}

/*********************************Setup****************************************/
void setup()
{
  Serial.begin(115200);

///////////////// SD Card ///////////////////
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    sd.initErrorHalt();
  }

  if (!data.open("data.csv", O_RDWR | O_CREAT | O_APPEND)) {
    sd.errorHalt("opening data.txt for write failed");
  }

  data.println("GX,GY,GZ,AX,AY,AZ,MX,MY,MZ,PITCH,ROLL,HEADING");
  Serial.println("GX,GY,GZ,AX,AY,AZ,MX,MY,MZ,PITCH,ROLL,HEADING");

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  setupGyro(); // Set up gyroscope parameters
  setupAccel(); // Set up accelerometer parameters
  setupMag(); // Set up magnetometer parameters
  setupTemperature(); // Set up temp sensor parameter

  imu.begin();
}

/*********************************LOOP*****************************************/
void loop()
{
  // update IMU data
  // imu.readGyro();
  // imu.readAccel();
  // imu.readMag();

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

  // imu.tempAvailable() returns 1 if new temperature sensor
  // data is ready to be read. 0 otherwise.
  if (imu.tempAvailable())
  {
    imu.readTemp();
  }

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
  Serial.print(",");
  serial_printAttitude(imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz);

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
  data.print(",");
  data_printAttitude(imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz);

  if (!data.sync() || data.getWriteError()) {
    Serial.println("write error");
  }

}

void serial_printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.println(heading, 2);
}

void data_printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  data.print(pitch, 2);
  data.print(",");
  data.print(roll, 2);
  data.print(",");
  data.println(heading, 2);
}
