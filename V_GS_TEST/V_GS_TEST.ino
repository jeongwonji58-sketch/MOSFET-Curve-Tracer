
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

// ======================================
// OBJECTS
// ======================================
Adafruit_MCP4728 mcp;
Adafruit_ADS1115 ads;

// ======================================
// PIN SETTINGS
// ======================================
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// ======================================
// HARDWARE SETTINGS
// ======================================
const float DAC_VDD = 3.3f;
const float R_SHUNT = 1.0f;

// Initial test limit: 0.5V
const float DAC_B_MAX = 0.5f;

// ======================================
// DAC
// ======================================
uint16_t voltageToCode(float voltage) {
  voltage = constrain(voltage, 0.0f, DAC_VDD);

  return (uint16_t)roundf(
    voltage / DAC_VDD * 4095.0f
  );
}

float dacB = 0.0f;

void setDAC_B(float voltage) {

  dacB = constrain(
    voltage,
    0.0f,
    DAC_B_MAX
  );

  mcp.setChannelValue(
    MCP4728_CHANNEL_B,
    voltageToCode(dacB),
    MCP4728_VREF_VDD,
    MCP4728_GAIN_1X
  );
}

void allOff() {

  setDAC_B(0.0f);

  mcp.setChannelValue(
    MCP4728_CHANNEL_A,
    0,
    MCP4728_VREF_VDD,
    MCP4728_GAIN_1X
  );

  mcp.setChannelValue(
    MCP4728_CHANNEL_C,
    0,
    MCP4728_VREF_VDD,
    MCP4728_GAIN_1X
  );

  mcp.setChannelValue(
    MCP4728_CHANNEL_D,
    0,
    MCP4728_VREF_VDD,
    MCP4728_GAIN_1X
  );
}

// ======================================
// ADC MEASUREMENTS
// ======================================

// Gate voltage: ADS1115 A2
float readGateVoltage() {

  ads.setGain(GAIN_ONE);

  int16_t raw =
    ads.readADC_SingleEnded(2);

  return ads.computeVolts(raw);
}

// Source voltage: ADS1115 A0
// A3 is connected to GND
float readSourceVoltage() {

  ads.setGain(GAIN_ONE);

  int16_t raw =
    ads.readADC_Differential_0_3();

  return ads.computeVolts(raw);
}

// VGS = VG - VS
float readVGS() {

  float vg = readGateVoltage();
  float vs = readSourceVoltage();

  return vg - vs;
}

// ======================================
// MEASUREMENT DISPLAY
// ======================================
void showMeasurements() {

  float vg = readGateVoltage();
  float vs = readSourceVoltage();
  float vgs = vg - vs;

  Serial.println();
  Serial.println("======== VGS MEASUREMENTS ========");

  Serial.print("DAC B Command : ");
  Serial.print(dacB, 4);
  Serial.println(" V");

  Serial.print("Gate VG       : ");
  Serial.print(vg, 6);
  Serial.println(" V");

  Serial.print("Source VS     : ");
  Serial.print(vs, 6);
  Serial.println(" V");

  Serial.print("VGS           : ");
  Serial.print(vgs, 6);
  Serial.println(" V");

  Serial.print("Target Error  : ");
  Serial.print(dacB - vgs, 6);
  Serial.println(" V");

  Serial.println("==================================");
}

// ======================================
// SERIAL COMMAND
// ======================================
void printHelp() {

  Serial.println();
  Serial.println("===== COMMANDS =====");
  Serial.println("B 0.05  : Set DAC B to 0.05V");
  Serial.println("B 0.10  : Set DAC B to 0.10V");
  Serial.println("B 0.20  : Set DAC B to 0.20V");
  Serial.println("B 0.50  : Set DAC B to 0.50V");
  Serial.println("M       : Show measurements");
  Serial.println("OFF     : Set DAC outputs to zero");
  Serial.println("HELP    : Show commands");
  Serial.println("====================");
}

void processCommand(String command) {

  command.trim();
  command.toUpperCase();

  if (command == "M") {

    showMeasurements();

  }
  else if (command == "OFF") {

    allOff();

    Serial.println("DAC OUTPUTS OFF");

    showMeasurements();

  }
  else if (command == "HELP") {

    printHelp();

  }
  else if (command.startsWith("B ")) {

    float voltage =
      command.substring(2).toFloat();

    if (voltage < 0.0f ||
        voltage > DAC_B_MAX) {

      Serial.println(
        "ERROR: DAC B range is 0~0.5V"
      );

      return;
    }

    setDAC_B(voltage);

    Serial.print("DAC B SET: ");
    Serial.print(dacB, 4);
    Serial.println(" V");

    delay(300);

    showMeasurements();

  }
  else {

    Serial.println("UNKNOWN COMMAND");
    printHelp();

  }
}

// ======================================
// SETUP
// ======================================
void setup() {

  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  delay(500);

  Serial.println();
  Serial.println("MOSFET VGS TEST");
  Serial.println("====================");

  // MCP4728
  if (!mcp.begin(0x60, &Wire)) {

    Serial.println("MCP4728 ERROR");

    while (1) delay(100);

  }

  // ADS1115
  if (!ads.begin(0x48, &Wire)) {

    Serial.println("ADS1115 ERROR");

    while (1) delay(100);

  }

  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_128SPS);

  // Initial DAC output
  allOff();

  Serial.println("MCP4728 OK");
  Serial.println("ADS1115 OK");
  Serial.println("DAC B INITIALIZED TO 0V");

  printHelp();

}

// ======================================
// LOOP
// ======================================
void loop() {

  if (Serial.available()) {

    String command =
      Serial.readStringUntil('\n');

    processCommand(command);

  }

}
