#include <Arduino.h>

#include <Wire.h>
#include <SSC.h>
#include <SPI.h>
#include <SdFat.h>

// Create an SSC sensor with I2C address 0x28 and power pin 53.
// I picked an unused pin because I want the sensor to always be on.
SSC ssc(0x28, 53);

// SD chip select pin.
const uint8_t chipSelect = 4;

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Data"

// File system object.
SdFat sd;

// Log file.
SdFile data;

long unsigned iterator = 0;
float current_time = 0;

void setup()
{
  Serial.begin(9600);

/////////////////// SD Card ///////////////////
  char fileName = "d.csv";

  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    sd.initErrorHalt();
  }

  if (!data.open("data.csv", O_RDWR | O_CREAT | O_TRUNC)) {
    sd.errorHalt("opening test.txt for write failed");
  }

  data.println("TIME,PRESSURE (mb)");

/////////////////// Sensor ///////////////////

  Wire.begin();

  //  set min / max reading and pressure, see datasheet for the values for your sensor
  ssc.setMinRaw(1638);
  ssc.setMaxRaw(14746);
  ssc.setMinPressure(0.0);
  ssc.setMaxPressure(1034.21);

  //  start the sensor
  Serial.println(ssc.start());
}

void loop()
{
  current_time = 0.5 * iterator;

  //  update pressure
  ssc.update();

  Serial.print("Time \t");
  Serial.println(current_time);
  data.print(current_time);
  data.print(",");

  // print pressure
  Serial.print("pressure(mb)\t");
  Serial.println(ssc.pressure() + 18.0);
  data.print(ssc.pressure() + 18.0);
  data.println();

  Serial.println("");

  iterator++;

  if (!data.sync() || data.getWriteError()) {
    Serial.println("write error");
  }

  delay(500);
}
