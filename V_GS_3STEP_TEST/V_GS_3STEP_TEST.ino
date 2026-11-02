
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

const float DAC_B_MAX = 0.60f;
const float TARGET_VGS_MAX = 0.50f;

// ======================================
// ZERO CALIBRATION
// ======================================
const int ZERO_SAMPLE_COUNT = 100;
const int ZERO_SAMPLE_DELAY_MS = 20;

float VGS_OFFSET = 0.0f;
bool zeroCalibrated = false;

// ======================================
// FILTER SETTINGS
// ======================================
const int FILTER_SAMPLE_COUNT = 9;
const int FILTER_DELAY_MS = 10;

// ======================================
// CLOSED LOOP SETTINGS
// ======================================
const float VGS_TOL = 0.001f;       // 1mV
const float FEEDBACK_GAIN = 0.5f;
const float DIVIDER_RATIO = 0.9091f;

const float MAX_DAC_CHANGE = 0.005f;

const int MAX_ITERATIONS = 250;
const int REQUIRED_STABLE_COUNT = 5;

const int CAL_WAIT_MS = 300;

// ======================================
// DAC VARIABLE
// ======================================
float dacB = 0.0f;

// ======================================
// DAC CONVERSION
// ======================================
uint16_t voltageToCode(float voltage) {

  voltage = constrain(voltage, 0.0f, DAC_VDD);

  return (uint16_t)roundf(
    voltage / DAC_VDD * 4095.0f
  );
}

// ======================================
// DAC OUTPUT
// ======================================
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

// Source voltage: ADS1115 A0 - A3
float readSourceVoltage() {

  ads.setGain(GAIN_ONE);

  int16_t raw =
    ads.readADC_Differential_0_3();

  return ads.computeVolts(raw);
}

// ======================================
// RAW VGS
// ======================================
float readVGSRaw() {

  float vg = readGateVoltage();
  float vs = readSourceVoltage();

  return vg - vs;
}

// ======================================
// CORRECTED VGS
// ======================================
float readVGSCorrected() {

  return readVGSRaw() - VGS_OFFSET;
}

// ======================================
// AVERAGE
// ======================================
float readVGSRawAverage(int count) {

  double sum = 0.0;

  for (int i = 0; i < count; i++) {

    sum += readVGSRaw();

    delay(ZERO_SAMPLE_DELAY_MS);
  }

  return (float)(sum / count);
}

// ======================================
// MEDIAN FILTER
// ======================================
float readVGSFiltered() {

  float values[FILTER_SAMPLE_COUNT];

  for (int i = 0; i < FILTER_SAMPLE_COUNT; i++) {

    values[i] = readVGSRaw();

    delay(FILTER_DELAY_MS);
  }

  // Sort values
  for (int i = 0; i < FILTER_SAMPLE_COUNT - 1; i++) {

    for (int j = i + 1; j < FILTER_SAMPLE_COUNT; j++) {

      if (values[i] > values[j]) {

        float temp = values[i];

        values[i] = values[j];

        values[j] = temp;
      }
    }
  }

  // Return median
  return values[FILTER_SAMPLE_COUNT / 2];
}

// ======================================
// AUTOMATIC ZERO CALIBRATION
// ======================================
bool calibrateZero() {

  Serial.println();
  Serial.println("================================");
  Serial.println(" AUTOMATIC ZERO CALIBRATION");
  Serial.println("================================");

  // DAC B = 0V
  setDAC_B(0.0f);

  Serial.println("DAC B set to 0V");

  Serial.println("Waiting for stabilization...");

  delay(2000);

  // Discard initial ADC conversions
  for (int i = 0; i < 10; i++) {

    readVGSRaw();

    delay(20);
  }

  Serial.println("Collecting 100 samples...");

  // Measure zero offset
  VGS_OFFSET =
    readVGSRawAverage(ZERO_SAMPLE_COUNT);

  if (!isfinite(VGS_OFFSET)) {

    Serial.println("ZERO CALIBRATION ERROR");

    zeroCalibrated = false;

    return false;
  }

  zeroCalibrated = true;

  Serial.println();
  Serial.println("ZERO CALIBRATION COMPLETE");

  Serial.print("Measured VGS Offset: ");

  Serial.print(VGS_OFFSET, 6);

  Serial.println(" V");

  Serial.println("================================");

  return true;
}

// ======================================
// DISPLAY MEASUREMENTS
// ======================================
void showMeasurements() {

  float vg = readGateVoltage();

  float vs = readSourceVoltage();

  float raw = vg - vs;

  float corrected = raw - VGS_OFFSET;

  Serial.println();

  Serial.println("======== VGS MEASUREMENTS ========");

  Serial.print("DAC B Command : ");

  Serial.print(dacB, 6);

  Serial.println(" V");

  Serial.print("Gate VG       : ");

  Serial.print(vg, 6);

  Serial.println(" V");

  Serial.print("Source VS     : ");

  Serial.print(vs, 6);

  Serial.println(" V");

  Serial.print("VGS RAW       : ");

  Serial.print(raw, 6);

  Serial.println(" V");

  Serial.print("VGS OFFSET    : ");

  Serial.print(VGS_OFFSET, 6);

  Serial.println(" V");

  Serial.print("VGS CORRECTED : ");

  Serial.print(corrected, 6);

  Serial.println(" V");

  Serial.println("==================================");
}

// ======================================
// CLOSED LOOP CALIBRATION
// ======================================
bool calibrateVGS(float targetVGS) {

  if (!zeroCalibrated) {

    Serial.println("ERROR: ZERO NOT CALIBRATED");

    return false;
  }

  Serial.println();

  Serial.println("================================");

  Serial.println(" VGS CLOSED LOOP CALIBRATION");

  Serial.println("================================");

  Serial.print("Target VGS: ");

  Serial.print(targetVGS, 6);

  Serial.println(" V");

  // Cannot reduce the physical voltage below
  // the zero-output baseline using positive DAC B.
  if (targetVGS < VGS_OFFSET - VGS_TOL) {

    Serial.println("ABORT: TARGET BELOW ZERO BASELINE");

    Serial.print("Zero baseline: ");

    Serial.println(VGS_OFFSET, 6);

    return false;
  }

  int stableCount = 0;

  // Start from zero DAC output
  setDAC_B(0.0f);

  delay(500);

  for (int i = 0; i < MAX_ITERATIONS; i++) {

    delay(CAL_WAIT_MS);

    // Use RAW VGS for physical voltage control
    float measuredVGS = readVGSFiltered();

    if (!isfinite(measuredVGS)) {

      Serial.println("ERROR: INVALID ADC READING");

      allOff();

      return false;
    }

    float correctedVGS =
      measuredVGS - VGS_OFFSET;

    // Physical target uses RAW measurement
    float error =
      targetVGS - measuredVGS;

    Serial.print("Iteration: ");

    Serial.print(i + 1);

    Serial.print(" | DAC B: ");

    Serial.print(dacB, 6);

    Serial.print(" | RAW VGS: ");

    Serial.print(measuredVGS, 6);

    Serial.print(" | Corrected: ");

    Serial.print(correctedVGS, 6);

    Serial.print(" | Error: ");

    Serial.println(error, 6);

    // ==================================
    // STABILITY CHECK
    // ==================================
    if (fabsf(error) <= VGS_TOL) {

      stableCount++;

      Serial.print("Stable: ");

      Serial.print(stableCount);

      Serial.print("/");

      Serial.println(REQUIRED_STABLE_COUNT);

      if (stableCount >= REQUIRED_STABLE_COUNT) {

        Serial.println();

        Serial.println("VGS CALIBRATION COMPLETE");

        showMeasurements();

        return true;
      }

      continue;
    }

    stableCount = 0;

    // ==================================
    // FEEDBACK CORRECTION
    // ==================================
    float correction =
      FEEDBACK_GAIN * error / DIVIDER_RATIO;

    correction = constrain(
      correction,
      -MAX_DAC_CHANGE,
      MAX_DAC_CHANGE
    );

    float nextDAC = dacB + correction;

    // ==================================
    // DAC LIMIT CHECK
    // ==================================
    if (nextDAC > DAC_B_MAX ||
        nextDAC < 0.0f) {

      Serial.println("ABORT: DAC LIMIT REACHED");

      allOff();

      return false;
    }

    if (fabsf(nextDAC - dacB) < 0.000001f) {

      Serial.println("ABORT: NO DAC PROGRESS");

      allOff();

      return false;
    }

    setDAC_B(nextDAC);
  }

  Serial.println();

  Serial.println("CALIBRATION FAILED");

  Serial.println("MAXIMUM ITERATIONS REACHED");

  allOff();

  return false;
}

// ======================================
// SERIAL COMMANDS
// ======================================
void printHelp() {

  Serial.println();

  Serial.println("========= COMMANDS =========");

  Serial.println("T 0.05 : Target VGS 0.05V");

  Serial.println("T 0.10 : Target VGS 0.10V");

  Serial.println("T 0.20 : Target VGS 0.20V");

  Serial.println("T 0.30 : Target VGS 0.30V");

  Serial.println("T 0.50 : Target VGS 0.50V");

  Serial.println("M      : Show measurements");

  Serial.println("ZERO   : Recalibrate zero offset");

  Serial.println("OFF    : Set DAC outputs to zero");

  Serial.println("HELP   : Show commands");

  Serial.println("============================");
}

void processCommand(String command) {

  command.trim();

  command.toUpperCase();

  if (command == "M") {

    showMeasurements();

  }

  else if (command == "OFF") {

    allOff();

    Serial.println("ALL DAC OUTPUTS OFF");

    showMeasurements();

  }

  else if (command == "ZERO") {

    allOff();

    calibrateZero();

  }

  else if (command == "HELP") {

    printHelp();

  }

  else if (command.startsWith("T ")) {

    float target =
      command.substring(2).toFloat();

    if (target < 0.0f ||
        target > TARGET_VGS_MAX) {

      Serial.println("ERROR: TARGET RANGE 0~0.5V");

      return;
    }

    calibrateVGS(target);

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

  Serial.println("MOSFET VGS ZERO + CLOSED LOOP TEST");

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

  // All outputs zero
  allOff();

  Serial.println("MCP4728 OK");

  Serial.println("ADS1115 OK");

  // Automatic zero calibration
  if (!calibrateZero()) {

    Serial.println("STARTUP FAILED: ZERO CALIBRATION");

    allOff();

    while (1) delay(100);
  }

  Serial.println();

  Serial.println("SYSTEM READY");

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
