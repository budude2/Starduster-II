#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SdFat.h>

//// SD chip select pin.
//const uint8_t chipSelect = 4;
//
//// Log file base name.  Must be six characters or less.
//#define FILE_BASE_NAME "Data"
//
//// File system object.
//SdFat sd;
//
//// Log file.
//SdFile data;

/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 10, TXPin = 8;
static const uint32_t GPSBaud = 4800;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  
//  /////////////////// SD Card ///////////////////
//  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
//    sd.initErrorHalt();
//  }
//
//  if (!data.open("data.csv", O_RDWR | O_CREAT | O_APPEND)) {
//    sd.errorHalt("opening test.txt for write failed");
//  }
  ss.begin(GPSBaud);


  Serial.println("Sats,Latitude,Longitude,Time,Alt,Course,Speed");

}

void loop()
{
  Serial.print(gps.satellites.value()); // Number of satellites in use (u32)
  Serial.print(",");
  Serial.print(gps.location.lat(), 6); // Latitude in degrees (double)
  Serial.print(",");
  Serial.print(gps.location.lng(), 6); // Longitude in degrees (double)
  Serial.print(",");
  Serial.print(gps.time.value()); // Raw time in HHMMSSCC format (u32)
  Serial.print(",");
  Serial.print(gps.altitude.feet()); // Altitude in feet (double)
  Serial.print(",");
  Serial.print(gps.course.deg()); // Course in degrees (double)
  Serial.print(",");
  Serial.println(gps.speed.mph()); // Speed in miles per hour (double)

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

//  if (!data.sync() || data.getWriteError()) {
//    Serial.println("write error");
//  }
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
