int sensitive = 100; // สำหรับ 20A
// int sensitive = 185; // สำหรับ 5A
// int sensitive = 66; // สำหรับ 30A

int offset = 2494; // ค่าเริ่มต้น 2500 ปรับค่าตรงนี้เพื่อให้ค่ายังไม่มีโหลดเป็น 0.00

void setup() {
  Serial.begin(9600);
  pinMode(22, OUTPUT); // <--- relay
  digitalWrite(22, LOW);
}

void loop() {
  double c = getCurrentAvg();
  Serial.println(c);
  delay(1000);
}


double getCurrentAvg() {
  int count = 20;
  double sum = 0;
  for (int i = 0; i < count; i++) {
    sum += getCurrent();
    delay(1);
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
