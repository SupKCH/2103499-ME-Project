#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include "RTClib.h"

#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 12
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress A0_thermometer = { 0x28, 0x4E, 0x12, 0xFF, 0x0C, 0x00, 0x00, 0xD7 };
DeviceAddress A1_thermometer = { 0x28, 0xBC, 0x54, 0xFE, 0x0C, 0x00, 0x00, 0x31 };
DeviceAddress A2_thermometer = { 0x28, 0xA6, 0xF9, 0x71, 0x15, 0x21, 0x01, 0xA6 };
DeviceAddress A3_thermometer = { 0x28, 0x88, 0x78, 0x38, 0x11, 0x20, 0x06, 0xAF };
float A0_temp, A1_temp, A2_temp, A3_temp;

#define DHTPIN 3
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
float h, t;

RTC_DS3231 rtc;
File myFile2;
//String filename_general = "temptest.txt";
String filename_max = "temptest.txt";
const byte chipSelect = 53; // same as SS

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Wire.begin();
  sensors.begin();
  //if (!sensors.getAddress(A0_thermometer, 0)) Serial.println("Unable to find address for Device 0");
  //if (!sensors.getAddress(A1_thermometer, 1)) Serial.println("Unable to find address for Device 1");
  sensors.setResolution(A0_thermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(A1_thermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(A2_thermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(A3_thermometer, TEMPERATURE_PRECISION);
  dht.begin();
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.print("Initializing SD card...");
  pinMode(SS, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

}

void loop() {
  // put your main code here, to run repeatedly:

  sensors.requestTemperatures();
  A0_temp = read_surface_temperature(A0_thermometer);
  A1_temp = read_surface_temperature(A1_thermometer);
  A2_temp = read_surface_temperature(A2_thermometer);
  A3_temp = read_surface_temperature(A3_thermometer);
  read_DHT();
  DateTime now = rtc.now();
  myFile2 = SD.open(filename_max, FILE_WRITE);
  if (myFile2) {
    String entire_line = String(now.year()) + ";"
                         + String(now.month()) + ";"
                         + String(now.day()) + ";"
                         + String(now.hour()) + ";"
                         + String(now.minute()) + ";"
                         + String(now.second()) + ";"
                         + String(t) + ";"
                         + String(h) + ";"
                         + String(A0_temp) + ";"
                         + String(A1_temp) + ";"
                         + String(A2_temp) + ";"
                         + String(A3_temp);
    myFile2.println(entire_line);
    myFile2.close();
    Serial.println(entire_line);
  } else Serial.println("error opening" + filename_max);

}


void read_DHT() {
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
}

float read_surface_temperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  return tempC;
}
