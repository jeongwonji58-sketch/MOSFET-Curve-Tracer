#include <Wire.h>
#include <Adafruit_MCP4728.h>

Adafruit_MCP4728 mcp;

// 원하는 전압을 Channel A에 출력하는 함수
void setVoltage(float voltage) {

  // MCP4728 전원전압을 3.3V로 가정
  uint16_t value = (uint16_t)((voltage / 3.3) * 4095);

  mcp.setChannelValue(
    MCP4728_CHANNEL_A,
    value,
    MCP4728_VREF_VDD,
    MCP4728_GAIN_1X
  );

  Serial.print("설정 전압: ");
  Serial.print(voltage, 1);
  Serial.println(" V");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(8, 9);  // SDA=8, SCL=9

  if (!mcp.begin(0x60, &Wire)) {
    Serial.println("MCP4728 연결 실패!");
    while (1);
  }

  Serial.println("MCP4728 연결 성공!");
}

void loop() {

  setVoltage(0.5);
  delay(5000);

  setVoltage(1.0);
  delay(5000);

  setVoltage(2.0);
  delay(5000);

  setVoltage(3.0);
  delay(5000);
}