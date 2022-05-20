/* FOR ONLY BOX-B HARDWARE */

#include <Wire.h>
#include <INA226_WE.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>

/* -------------------------------- OneWire (DS18B20) Configuration -------------------------------- */
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 12
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress B0_thermometer = { 0x28, 0x50, 0x84, 0xFE, 0x0C, 0x00, 0x00, 0x3B };
DeviceAddress B1_thermometer = { 0x28, 0xA2, 0x83, 0xFE, 0x0C, 0x00, 0x00, 0x76 };
DeviceAddress B2_thermometer = { 0x28, 0xFE, 0xC8, 0xB1, 0x10, 0x20, 0x06, 0x2B };
DeviceAddress B3_thermometer = { 0x28, 0xFD, 0x4F, 0x8D, 0x15, 0x21, 0x01, 0xC0 };
float B0_temp, B1_temp, B2_temp, B3_temp;

/* -------------------------------- DHT22 Configuration -------------------------------- */
#define DHTPIN 3
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
float h, t;
//unsigned long lastDHTreading = 0;
//int DHT_delay_ms = 300;

/* -------------------------------- 2x INA226 (Voltmeters) / INA219 (low-Amp) Configuration -------------------------------- */
#define I2C_ADDRESS 0x40
#define I2C_ADDRESS_TRIG 0x44
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);
INA226_WE ina226_trig = INA226_WE(I2C_ADDRESS_TRIG);
Adafruit_INA219 ina219(0x41);
float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0;
float trig_shuntVoltage_mV = 0.0;
float trig_loadVoltage_V = 0.0;
float trig_busVoltage_V = 0.0;

/* -------------------------------- RTC & SD Card Configuration -------------------------------- */
RTC_DS3231 rtc;
DateTime now;
File myFile, myFile2, myFile3, myFile4;
String filename_general = "gendemoB.txt";
String filename_max = "maxcalBf.txt";
String filename_triggerVolt = "trig_Bf.txt";
String filename_log = "log_B.txt";
const byte chipSelect = 53; // same as SS

/* -------------------------------- MPPT Parameters -------------------------------- */
const byte switch_pin = 30;
const byte relay_pin = 22;
float power_mW = 0;
float P_max = 0;
byte k = 0;
unsigned int j = 0;
String buff = "";
unsigned int overload = 0;
byte found_peak_cycles = 0;

/* DO NOT CHANGE THESE VALUES */
bool high = false;
bool discharging = false;
bool firsttime = true;
bool rested = true;
/* DO NOT CHANGE THESE VALUES */

const bool use_hall_effect = true;
const byte trigger_threshold = 25;
const byte limit_volt = 7;
const byte limit_k = 50;
const byte peaks_before_reboot = 10;
//const byte dischargeTime_before_reboot = 2;  // 2 mins

/* -------------------------------- MPPT - Hall effect -------------------------------- */
const int ammeter_pin = A15;
byte sensitive = 100;   // for 20A
float offset = 2480;    // last used: 2492.5;

/* -------------------------------- MPPT - MovAvg for Hall effect (no longer used) --------------------------------
  const byte numReadings = 1;
  float readings[numReadings];      // the readings from the analog input
  byte readIndex = 0;              // the index of the current reading
  float total = 0;                  // the running total
  float average = 0;                // the average
  //float current = 0;
*/

void setup() {
  /* ========================== Initialization ========================== */
  pinMode(switch_pin, INPUT_PULLUP);
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW);

  Serial.begin(9600);
  Wire.begin();

  /* ========================== OneWire (DS18B20) ========================== */
  sensors.begin();
  sensors.setResolution(B0_thermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(B1_thermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(B2_thermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(B3_thermometer, TEMPERATURE_PRECISION);
  /* ========================== DHT22 ========================== */
  dht.begin();
  /* ========================== INA226 ========================== */
  ina226.init();
  Serial.println("INA226 Voltage Sensor - Initialized");
  ina226.waitUntilConversionCompleted();
  /* ========================== INA219 ========================== */
  if (!use_hall_effect) {
    if (!ina219.begin()) {
      Serial.println("Failed to find INA219 chip");
      while (1) {
        delay(10);
      }
    }
  }
  /* ========================== RTC ========================== */
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  /* ========================== SD Card ========================== */
  Serial.print("Initializing SD card...");
  pinMode(SS, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  /* ========================== MovAvg (no longer used) ========================== */
  /*
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
    }*/
}

/* =================================== MPPT OPERATION =================================== */
void loop() {
  read_trigger_voltage();

  while (trig_loadVoltage_V * 2 < trigger_threshold) {
    if (!rested) digitalWrite(relay_pin, LOW); // open circuit
    rested = true;
    delay(5000); // idle for 5 seconds
    read_trigger_voltage();
    SD_card_triggerVolt();
    Serial.print(F("Waiting for open-circuit voltage to reach threshold, current volt: "));
    Serial.println(trig_loadVoltage_V * 2);

    if (trig_loadVoltage_V * 2 >= trigger_threshold) break;
  }


  while (true) {
    //Serial.println(String(millis() - checkpoint));
    //checkpoint = millis();
    if (digitalRead(switch_pin) == HIGH && (!high || rested)) {
      Serial.println(F("START"));
      discharging = false;
      buff = "";
      high = true;
      firsttime = true;
      rested = false;
      //start_of_cycle = millis();
      digitalWrite(relay_pin, HIGH);
    }
    else if (digitalRead(switch_pin) == LOW && high) {
      digitalWrite(relay_pin, LOW);
      buff = "";
      high = false;
      j = 0;
    }

    ina226.readAndClearFlags();
    shuntVoltage_mV = ina226.getShuntVoltage_mV();
    busVoltage_V = ina226.getBusVoltage_V();
    loadVoltage_V  = busVoltage_V + (shuntVoltage_mV / 1000);

    if (!use_hall_effect) current_mA = ina219.getCurrent_mA();
    else if (use_hall_effect) current_mA = getCurrentAvg() * 1000;

    power_mW = (loadVoltage_V * 2) * current_mA;

    if (high && !discharging) {
      if (firsttime) {
        buff = "-9;0\n";
        buff += String(loadVoltage_V * 2) + ";" + String(current_mA, 1) + "\n";
        firsttime = false;
        j++;
        P_max = 0;
        k = 0;
        overload = 0;
      }
      else if (j < 250) {
        buff += String(loadVoltage_V * 2) + ";" + String(current_mA, 1) + "\n";
        j++;
      }
      else if (j == 250) {
        byte newline_index = buff.indexOf("\n", 5);
        buff.remove(5, newline_index + 1 - 5);
        buff.concat(String(loadVoltage_V * 2) + ";" + String(current_mA, 1) + "\n");
        overload++;
        if (overload == 300 * 10) {
          digitalWrite(relay_pin, LOW);
          Serial.println(F("Cannot find MPPT peaks within 3000 rows ... Discharge & Reboot ..."));
          SD_card_log();
          delay(120000);
          Serial.println(F("REBOOT\n"));
          software_Reboot();
        }
      }
      if (loadVoltage_V * 2 >= limit_volt) {
        if (power_mW > P_max) {
          P_max = power_mW;
          k = 0;
          continue;
        }
        else {
          if (power_mW < 0.8 * P_max) {
            k++;
            if (k > limit_k) {
              digitalWrite(relay_pin, LOW);
              sensors.requestTemperatures();
              B0_temp = read_surface_temperature(B0_thermometer);
              B1_temp = read_surface_temperature(B1_thermometer);
              B2_temp = read_surface_temperature(B2_thermometer);
              B3_temp = read_surface_temperature(B3_thermometer);
              read_DHT();
              now = rtc.now();
              SD_card_timePowerAmbTempAtMAX();
              buff.concat(String(now.year()) + ";"
                          + String(now.month()) + ";"
                          + String(now.day()) + ";"
                          + String(now.hour()) + ";"
                          + String(now.minute()) + ";"
                          + String(now.second()));
              SD_card_allAtOnce();
              P_max = 0;
              k = 0;
              j = 0;
              discharging = true;
              firsttime = true;
              found_peak_cycles++;
              Serial.println(overload);
              Serial.println(found_peak_cycles);
              continue;
            }
            else continue;
          }
          else continue;
        }
      }
      else continue;
    }
    else if (high && discharging) {
      delay(200);
      if (found_peak_cycles == peaks_before_reboot) {
        Serial.println(F("Numbers of peaks reach threshold ... Discharge & Reboot ..."));
        delay(120000);
        Serial.println(F("REBOOT\n"));
        software_Reboot();
      }
      Serial.println(String(loadVoltage_V * 2) + ";" + String(current_mA, 1));
      if (loadVoltage_V * 2 < limit_volt) {
        read_trigger_voltage();
        SD_card_triggerVolt();
        if (trig_loadVoltage_V * 2 >= trigger_threshold) {
          Serial.println(F("START"));
          discharging = false;
          digitalWrite(relay_pin, HIGH);
          continue;
        }
        else {
          rested = false;
          break;
        }
      }
    }
    else if (!high) {
      read_trigger_voltage();
      SD_card_triggerVolt();  // <-- already includes DHT22 and Time readings!
      Serial.println(String(now.year()) + ";"
                     + String(now.month()) + ";"
                     + String(now.day()) + ";"
                     + String(now.hour()) + ";"
                     + String(now.minute()) + ";"
                     + String(now.second()) + ";" + " "
                     + String(trig_loadVoltage_V * 2) + ";"
                     + String(loadVoltage_V * 2) + ";" + " "
                     + String(current_mA) + ";"
                     + String(t) + ";"
                     + String(h) + ";"
                     + String(B0_temp) + ";"
                     + String(B1_temp) + ";"
                     + String(B2_temp) + ";"
                     + String(B3_temp));
      delay(100);
      if (trig_loadVoltage_V * 2 < trigger_threshold) {
        rested = false;
        break;
      }
    }
  }
}

/* =================================== FUNCTION ZONE =================================== */

float getCurrentAvg() {
  int count = 4;
  float sum = 0;
  for (int i = 0; i < count; i++) {
    sum += getCurrent();
    //delay(1);
  }
  float val = sum / count;
  return val;
}

float getCurrent() {
  int a = analogRead(ammeter_pin);
  float v = (a / 1024.0) * 5000;
  float c = (v - offset) / sensitive;
  return c;
}

/*
  void movAvgHallEffect() {
  total = total - readings[readIndex];
  current_mA = getCurrentAvg() * 1000;
  readings[readIndex] = current_mA;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  current_mA = total / numReadings;
  }
*/

void SD_card_timePowerAmbTempAtMAX() {
  myFile2 = SD.open(filename_max, FILE_WRITE);
  if (myFile2) {
    String entire_line = String(now.year()) + ";"
                         + String(now.month()) + ";"
                         + String(now.day()) + ";"
                         + String(now.hour()) + ";"
                         + String(now.minute()) + ";"
                         + String(now.second()) + ";"
                         + String(P_max) + ";"
                         + String(t) + ";"
                         + String(h) + ";"
                         + String(B0_temp) + ";"
                         + String(B1_temp) + ";"
                         + String(B2_temp) + ";"
                         + String(B3_temp);
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

void SD_card_allAtOnce() {
  myFile = SD.open(filename_general, FILE_WRITE);
  if (myFile) {
    myFile.println(buff);
    myFile.close();
    //Serial.println(buff);
  } else Serial.println("error opening" + filename_general);
}

void software_Reboot()
{
  wdt_enable(WDTO_15MS);
  delay(100);
}

void read_trigger_voltage() {
  ina226_trig.readAndClearFlags();
  trig_shuntVoltage_mV = ina226_trig.getShuntVoltage_mV();
  trig_busVoltage_V = ina226_trig.getBusVoltage_V();
  trig_loadVoltage_V  = trig_busVoltage_V + (trig_shuntVoltage_mV / 1000);
}

void SD_card_triggerVolt() {
  sensors.requestTemperatures();
  B0_temp = read_surface_temperature(B0_thermometer);
  B1_temp = read_surface_temperature(B1_thermometer);
  B2_temp = read_surface_temperature(B2_thermometer);
  B3_temp = read_surface_temperature(B3_thermometer);
  read_DHT();
  now = rtc.now();
  myFile3 = SD.open(filename_triggerVolt, FILE_WRITE);
  if (myFile3) {
    myFile3.println(String(now.year()) + ";"
                    + String(now.month()) + ";"
                    + String(now.day()) + ";"
                    + String(now.hour()) + ";"
                    + String(now.minute()) + ";"
                    + String(now.second()) + ";"
                    + String(trig_loadVoltage_V * 2) + ";"
                    + String(t) + ";"
                    + String(h) + ";"
                    + String(B0_temp) + ";"
                    + String(B1_temp) + ";"
                    + String(B2_temp) + ";"
                    + String(B3_temp));
    myFile3.close();
  } else Serial.println("error opening" + filename_triggerVolt);
}

void SD_card_log() {
  now = rtc.now();
  buff.concat(String(now.year()) + ";"
              + String(now.month()) + ";"
              + String(now.day()) + ";"
              + String(now.hour()) + ";"
              + String(now.minute()) + ";"
              + String(now.second()));
  myFile4 = SD.open(filename_log, FILE_WRITE);
  if (myFile4) {
    myFile4.println(buff);
    myFile4.close();
  } else Serial.println("error opening " + filename_log);
}
/* =================================== End of this file =================================== */
