#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(8, 9);  // SDA = GPIO8, SCL = GPIO9

  if (!ads.begin(0x48, &Wire)) {
    Serial.println("ADS1115 연결 실패");
    while (1);
  }

  // 작은 션트 전압 측정을 위해 ±0.256V 범위 사용
  ads.setGain(GAIN_SIXTEEN);

  Serial.println("ADS1115 연결 성공!");
}

void loop() {
  int16_t raw = ads.readADC_SingleEnded(0);
  float vShunt = ads.computeVolts(raw);

  // Rshunt = 1Ω
  float current_A = vShunt / 1.0;
  float current_mA = current_A * 1000.0;

  Serial.print("RAW = ");
  Serial.print(raw);

  Serial.print(" | Vshunt = ");
  Serial.print(vShunt * 1000.0, 3);
  Serial.print(" mV");

  Serial.print(" | Current = ");
  Serial.print(current_mA, 3);
  Serial.println(" mA");

  delay(1000);
}