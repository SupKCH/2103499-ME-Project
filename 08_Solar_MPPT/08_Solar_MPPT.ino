#include <Wire.h>
#include <INA226_WE.h>

#define I2C_ADDRESS 0x40
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);

const byte switch_pin = 30;
const byte relay_pin = 22;
bool high = false;

int sensitive = 100; // สำหรับ 20A
float offset = 2490.5;

const int numReadings = 32;

double readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
double total = 0;                  // the running total
double average = 0;                // the average
double current = 0;

void setup() {
  Wire.begin();
  ina226.init();
  //ina226.setAverage(AVERAGE_16); // choose mode and uncomment for change of default; default 1
  //ina226.setConversionTime(CONV_TIME_1100); //choose conversion time and uncomment for change of default
  //ina226.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default

  //ina226.setCurrentRange(MA_800); // choose gain and uncomment for change of default
  //ina226.setCorrectionFactor(0.95);

  Serial.println("INA226 Current Sensor Example Sketch - Continuous");
  ina226.waitUntilConversionCompleted(); //if you comment this line the first data might be zero

  pinMode(switch_pin, INPUT_PULLUP);
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW);
  Serial.begin(9600);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop() {
  if (digitalRead(switch_pin) == HIGH && !high) {
    digitalWrite(relay_pin, HIGH);
    //Serial.println(digitalRead(switch_pin));
    high = true;
  }
  else if (digitalRead(switch_pin) == LOW && high) {
    digitalWrite(relay_pin, LOW);
    //Serial.println(digitalRead(switch_pin));
    high = false;
  }

  float shuntVoltage_mV = 0.0;
  float loadVoltage_V = 0.0;
  float busVoltage_V = 0.0; // <--- use only this to measure voltage on VBS-GND pin
  //  float current_mA = 0.0;
  //  float power_mW = 0.0;

  ina226.readAndClearFlags();
  shuntVoltage_mV = ina226.getShuntVoltage_mV();
  busVoltage_V = ina226.getBusVoltage_V();
  //  current_mA = ina226.getCurrent_mA();
  //  power_mW = ina226.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV / 1000);

  //Serial.print("Shunt Voltage [mV]: ");
  //Serial.println(shuntVoltage_mV);
  //Serial.print("Bus Voltage [V]: ");
  //Serial.println(busVoltage_V);
  //Serial.print("Load Voltage [V] x 2: ");
  //Serial.println(loadVoltage_V * 2);
  //  Serial.print("Current[mA]: "); Serial.println(current_mA);
  //  Serial.print("Bus Power [mW]: "); Serial.println(power_mW);
  //  if (!ina226.overflow) {
  //    Serial.println("Values OK - no overflow");
  //  }
  //  else {
  //    Serial.println("Overflow! Choose higher current range");
  //  }
  //Serial.println();
  //double current = getCurrentAvg();
  //Serial.println(current);

  Serial.print(loadVoltage_V * 2);
  Serial.print(",");
  movAvg();
  Serial.print(current*1000);
  Serial.print(",");
  Serial.print(average*1000);
  double power_mW = (loadVoltage_V * 2) * average*1000;
  Serial.print(",");
  Serial.println(power_mW);

  delay(100);
}


double getCurrentAvg() {
  int count = 1;
  double sum = 0;
  for (int i = 0; i < count; i++) {
    sum += getCurrent();
    //delay(1);
  }
  double val = sum / count;
  return val;
}

double getCurrent() {
  int a = analogRead(A15);
  double v = (a / 1024.0) * 5000;
  double c = (v - offset) / sensitive;
  return c;
}

void movAvg() {
  total = total - readings[readIndex];
  current = getCurrentAvg();
  readings[readIndex] = current;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  average = total / numReadings;
}
