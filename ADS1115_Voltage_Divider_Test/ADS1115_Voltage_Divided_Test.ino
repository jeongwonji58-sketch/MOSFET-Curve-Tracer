#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // ESP32-S3 I2C 핀
  Wire.begin(8, 9);   // SDA = GPIO8, SCL = GPIO9

  if (!ads.begin(0x48, &Wire)) {
    Serial.println("ADS1115 연결 실패");
    while (1);
  }

  // 측정 범위 ±4.096V
  ads.setGain(GAIN_ONE);

  Serial.println("ADS1115 연결 성공!");
}

void loop() {
  int16_t raw = ads.readADC_SingleEnded(0);  // A0 측정
  float voltage = ads.computeVolts(raw);

  Serial.print("RAW = ");
  Serial.print(raw);

  Serial.print("   A0 = ");
  Serial.print(voltage, 4);
  Serial.println(" V");

  delay(1000);
}