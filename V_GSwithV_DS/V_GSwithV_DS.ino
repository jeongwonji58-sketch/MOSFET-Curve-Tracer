#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include "Measurement.h"

Adafruit_MCP4728 mcp;
Adafruit_ADS1115 ads;
Adafruit_ADS1115 adsBE;

// =====================================
// HARDWARE
// =====================================
const int SDA_PIN = 8;
const int SCL_PIN = 9;

const float DAC_VDD = 3.3f;
const float DRAIN_GAIN = 4.0f;
const float BE_DIVIDER_GAIN = 4.0f;

// =====================================
// VGS CONTROL
// =====================================
const float TARGET_VGS = 1.0f;
const float VGS_TOL = 0.020f;

const float FEEDBACK_GAIN = 0.3f;
const float MAX_DAC_CHANGE = 0.010f;

const int REQUIRED_STABLE_COUNT = 10;
const int MAX_ITERATIONS = 500;

const unsigned long SAMPLE_WAIT_MS = 150;

// =====================================
// DAC C AUTO SWEEP
// 0 ~ 2.5V / 0.025V / 101 points
// =====================================
const float DAC_C_START = 0.000f;
const float DAC_C_END = 2.500f;
const float DAC_C_STEP = 0.025f;

const int NUM_STEPS = 101;

const unsigned long SWEEP_SETTLE_MS = 1000;

int currentStep = -1;
float currentDACC = 0.0f;

bool sweepRunning = false;
unsigned long lastSweepChange = 0;

// =====================================
// STATE
// =====================================
float dacB = 0.0f;

int iteration = 0;
int stableCount = 0;

bool vgsRunning = false;
bool vgsStable = false;

// =====================================
// DAC
// =====================================
uint16_t voltageToCode(float voltage) {

  voltage = constrain(voltage, 0.0f, DAC_VDD);

  return (uint16_t)roundf(
    voltage / DAC_VDD * 4095.0f
  );
}

void setDAC(
  MCP4728_channel_t channel,
  float voltage
) {

  mcp.setChannelValue(
    channel,
    voltageToCode(voltage),
    MCP4728_VREF_VDD,
    MCP4728_GAIN_1X
  );
}

// =====================================
// ADC READ FUNCTIONS
// =====================================
float readADC(uint8_t channel) {

  ads.setGain(GAIN_ONE);

  float sum = 0.0f;

  for (int i = 0; i < 10; i++) {

    int16_t raw =
      ads.readADC_SingleEnded(channel);

    sum += ads.computeVolts(raw);

    delay(5);
  }

  return sum / 10.0f;
}

float readSource() {

  ads.setGain(GAIN_SIXTEEN);

  float sum = 0.0f;

  for (int i = 0; i < 10; i++) {

    int16_t raw =
      ads.readADC_Differential_0_3();

    sum += ads.computeVolts(raw);

    delay(5);
  }

  return sum / 10.0f;
}

float readBE_ADC(uint8_t channel) {

  adsBE.setGain(GAIN_ONE);

  float sum = 0.0f;

  for (int i = 0; i < 10; i++) {

    int16_t raw =
      adsBE.readADC_SingleEnded(channel);

    sum += adsBE.computeVolts(raw);

    delay(5);
  }

  return sum / 10.0f;
}

// =====================================
// MEASUREMENT STRUCT
// =====================================


Measurement readAllMeasurements() {

  Measurement m;

  m.gate = readADC(2);
  m.source = readSource();
  m.drainADC = readADC(1);

  m.vgs = m.gate - m.source;

  m.drain = m.drainADC * DRAIN_GAIN;

  m.vds = m.drain - m.source;

  m.baseADC = readBE_ADC(0);
  m.emitterADC = readBE_ADC(1);

  // ADS1115 #2 (0x49) A2: OP AMP output through 30k/10k divider
  m.opampADC = readBE_ADC(2);

  m.base = m.baseADC * BE_DIVIDER_GAIN;
  m.emitter = m.emitterADC * BE_DIVIDER_GAIN;
  m.opamp = m.opampADC * 4.0f;

  m.vbe = m.base - m.emitter;

  // 1k ohm resistor: voltage drop in volts equals current in mA
  m.baseResDrop = m.opamp - m.base;
  m.ib_mA = m.baseResDrop;  // I_B(mA) = (VOP - VB) / 1kOhm

  // Theoretical VDS using the measured node voltages
  m.vdsTheory = m.opamp - m.baseResDrop - m.vbe - m.source;

  return m;
}

// =====================================
// PRINT MEASUREMENTS
// =====================================
void printMeasurements() {

  Measurement m = readAllMeasurements();

  Serial.println();
  Serial.println("======= MEASUREMENTS =======");

  Serial.print("DAC B: ");
  Serial.println(dacB, 5);

  Serial.print("DAC C: ");
  Serial.println(currentDACC, 5);

  Serial.print("Gate: ");
  Serial.println(m.gate, 5);

  Serial.print("Source: ");
  Serial.println(m.source, 6);

  Serial.print("VGS: ");
  Serial.println(m.vgs, 5);

  Serial.print("Drain ADC: ");
  Serial.println(m.drainADC, 5);

  Serial.print("Drain reconstructed: ");
  Serial.println(m.drain, 5);

  Serial.print("VDS reconstructed: ");
  Serial.println(m.vds, 5);

  Serial.print("Base: ");
  Serial.println(m.base, 5);

  Serial.print("Emitter: ");
  Serial.println(m.emitter, 5);

  Serial.print("VBE: ");
  Serial.println(m.vbe, 5);

  Serial.print("OP AMP ADC A2: ");
  Serial.println(m.opampADC, 5);

  Serial.print("OP AMP output (x4): ");
  Serial.println(m.opamp, 5);

  Serial.print("Base resistor drop: ");
  Serial.println(m.baseResDrop, 5);

  Serial.print("Estimated IB (mA): ");
  Serial.println(m.ib_mA, 5);

  Serial.print("Theoretical VDS: ");
  Serial.println(m.vdsTheory, 5);

  Serial.print("VDS error (measured - theory): ");
  Serial.println(m.vds - m.vdsTheory, 5);

  if (
    m.baseADC > 3.0f ||
    m.emitterADC > 3.0f ||
    m.baseADC < 0.0f ||
    m.emitterADC < 0.0f
  ) {

    Serial.println(
      "WARNING: BASE/EMITTER ADC INPUT RANGE CHECK"
    );
  }

  Serial.println("============================");
}

// =====================================
// TIP41C MEASUREMENT
// =====================================
void printTIP41C() {

  Measurement m = readAllMeasurements();

  Serial.println();
  Serial.println("======= MAIN TIP41C =======");

  Serial.print("Base ADC: ");
  Serial.println(m.baseADC, 5);

  Serial.print("Emitter ADC: ");
  Serial.println(m.emitterADC, 5);

  Serial.print("Base Voltage: ");
  Serial.println(m.base, 5);

  Serial.print("Emitter Voltage: ");
  Serial.println(m.emitter, 5);

  Serial.print("VBE: ");
  Serial.println(m.vbe, 5);

  Serial.print("OP AMP output: ");
  Serial.println(m.opamp, 5);

  Serial.print("Estimated IB (mA): ");
  Serial.println(m.ib_mA, 5);

  Serial.print("Theoretical VDS: ");
  Serial.println(m.vdsTheory, 5);

  Serial.println("===========================");
}

// =====================================
// CSV OUTPUT
// =====================================
void printCSVHeader() {

  Serial.println();

  Serial.println(
    "STEP,DAC_C_V,OPAMP_ADC_V,OPAMP_V,BASE_V,EMITTER_V,"
    "VBE_V,BASE_RES_DROP_V,IB_mA,VDS_THEORY_V,"
    "VDS_MEASURED_V,VDS_ERROR_V,GATE_V,SOURCE_V,VGS_V,"
    "DRAIN_ADC_V,DRAIN_RECON_V"
  );
}

void printCSVRow(
  int step,
  float dacC,
  const Measurement &m
) {
  Serial.print(step); Serial.print(",");
  Serial.print(dacC, 5); Serial.print(",");
  Serial.print(m.opampADC, 5); Serial.print(",");
  Serial.print(m.opamp, 5); Serial.print(",");
  Serial.print(m.base, 5); Serial.print(",");
  Serial.print(m.emitter, 5); Serial.print(",");
  Serial.print(m.vbe, 5); Serial.print(",");
  Serial.print(m.baseResDrop, 5); Serial.print(",");
  Serial.print(m.ib_mA, 5); Serial.print(",");
  Serial.print(m.vdsTheory, 5); Serial.print(",");
  Serial.print(m.vds, 5); Serial.print(",");
  Serial.print(m.vds - m.vdsTheory, 5); Serial.print(",");
  Serial.print(m.gate, 5); Serial.print(",");
  Serial.print(m.source, 6); Serial.print(",");
  Serial.print(m.vgs, 5); Serial.print(",");
  Serial.print(m.drainADC, 5); Serial.print(",");
  Serial.println(m.drain, 5);
}

// =====================================
// START VGS
// =====================================
void startVGS() {

  dacB = 0.0f;

  iteration = 0;
  stableCount = 0;

  currentStep = -1;
  currentDACC = 0.0f;

  vgsStable = false;
  vgsRunning = true;

  sweepRunning = false;

  setDAC(MCP4728_CHANNEL_C, 0.0f);
  setDAC(MCP4728_CHANNEL_B, 0.0f);

  delay(1000);

  Serial.println();
  Serial.println("VGS CLOSED LOOP STARTED");
  Serial.println("TARGET VGS = 1.0V");
}

// =====================================
// VGS FEEDBACK
// =====================================
void updateVGS() {

  if (!vgsRunning) return;

  if (iteration >= MAX_ITERATIONS) {

    Serial.println("VGS TIMEOUT");

    vgsRunning = false;

    setDAC(MCP4728_CHANNEL_B, 0.0f);

    return;
  }

  delay(SAMPLE_WAIT_MS);

  float gate = readADC(2);
  float source = readSource();

  float actualVGS = gate - source;

  float error = TARGET_VGS - actualVGS;

  if (
    !isfinite(actualVGS) ||
    gate < -0.05f ||
    gate > 3.35f
  ) {

    Serial.println("INVALID ADC");

    vgsRunning = false;

    setDAC(MCP4728_CHANNEL_B, 0.0f);

    return;
  }

  iteration++;

  Serial.print("Iteration: ");
  Serial.print(iteration);

  Serial.print(" | DAC B: ");
  Serial.print(dacB, 5);

  Serial.print(" | VGS: ");
  Serial.print(actualVGS, 5);

  Serial.print(" | Error: ");
  Serial.println(error, 5);

  if (fabsf(error) <= VGS_TOL) {

    stableCount++;

    Serial.print("Stable: ");
    Serial.print(stableCount);
    Serial.print("/");
    Serial.println(REQUIRED_STABLE_COUNT);

    if (stableCount >= REQUIRED_STABLE_COUNT) {

      vgsRunning = false;
      vgsStable = true;

      Serial.println();
      Serial.println("VGS STABLE");

      Serial.print("Final VGS: ");
      Serial.println(actualVGS, 5);

      Serial.println("DAC B HOLD ENABLED");
      Serial.println("Enter a to start automatic DAC C sweep");
      Serial.println("Sweep: 0.000V to 2.500V, step 0.025V");
    }

    return;
  }

  stableCount = 0;

  float correction = FEEDBACK_GAIN * error;

  correction = constrain(
    correction,
    -MAX_DAC_CHANGE,
    MAX_DAC_CHANGE
  );

  float nextDAC = constrain(
    dacB + correction,
    0.0f,
    DAC_VDD
  );

  if (fabsf(nextDAC - dacB) < 0.000001f) {

    Serial.println("DAC B LIMIT");

    vgsRunning = false;

    setDAC(MCP4728_CHANNEL_B, 0.0f);

    return;
  }

  dacB = nextDAC;

  setDAC(MCP4728_CHANNEL_B, dacB);
}

// =====================================
// MANUAL DAC C STEP
// =====================================
void nextDACCTest() {

  if (!vgsStable) {

    Serial.println("ERROR: VGS NOT STABLE");

    return;
  }

  if (sweepRunning) {

    Serial.println("AUTO SWEEP RUNNING. Enter x to stop/reset.");

    return;
  }

  if (currentStep + 1 >= NUM_STEPS) {

    Serial.println("ALL DAC C STEPS COMPLETED");

    return;
  }

  currentStep++;

  currentDACC =
    DAC_C_START + currentStep * DAC_C_STEP;

  setDAC(MCP4728_CHANNEL_C, currentDACC);

  delay(SWEEP_SETTLE_MS);

  Serial.println();
  Serial.println("======= DAC C STEP =======");

  Serial.print("STEP: ");
  Serial.println(currentStep + 1);

  Serial.print("DAC C: ");
  Serial.print(currentDACC, 4);
  Serial.println(" V");

  Serial.print("Expected x4 signal: ");
  Serial.print(currentDACC * DRAIN_GAIN, 4);
  Serial.println(" V");

  Serial.println("VGS HOLD ACTIVE");

  printMeasurements();
}

// =====================================
// START AUTOMATIC SWEEP
// =====================================
void startAutoSweep() {

  if (!vgsStable || vgsRunning) {

    Serial.println("ERROR: VGS MUST BE STABLE BEFORE SWEEP");

    return;
  }

  if (sweepRunning) {

    Serial.println("AUTO SWEEP ALREADY RUNNING");

    return;
  }

  currentStep = 0;

  currentDACC = DAC_C_START;

  setDAC(MCP4728_CHANNEL_C, currentDACC);

  sweepRunning = true;

  lastSweepChange = millis();

  Serial.println();
  Serial.println("AUTO DAC C SWEEP STARTED");

  Serial.println("Range: 0.000V to 2.500V");
  Serial.println("Step: 0.025V | Total: 101 points");

  Serial.println("VGS feedback is holding its last value.");
  Serial.println("CSV output begins after settling.");

  printCSVHeader();
}

// =====================================
// UPDATE AUTOMATIC SWEEP
// =====================================
void updateAutoSweep() {

  if (!sweepRunning) return;

  if (millis() - lastSweepChange < SWEEP_SETTLE_MS) {
    return;
  }

  Measurement m = readAllMeasurements();

  if (
    m.baseADC > 3.0f ||
    m.emitterADC > 3.0f ||
    m.baseADC < 0.0f ||
    m.emitterADC < 0.0f
  ) {

    Serial.println(
      "WARNING: BASE/EMITTER ADC INPUT RANGE CHECK"
    );
  }

  printCSVRow(
    currentStep + 1,
    currentDACC,
    m
  );

  if (currentStep >= NUM_STEPS - 1) {

    sweepRunning = false;

    Serial.println("AUTO SWEEP COMPLETE");

    Serial.println(
      "Copy the CSV rows from STEP through the final row."
    );

    return;
  }

  currentStep++;

  currentDACC =
    DAC_C_START + currentStep * DAC_C_STEP;

  setDAC(MCP4728_CHANNEL_C, currentDACC);

  lastSweepChange = millis();
}

// =====================================
// RESET / STOP
// =====================================
void resetOutputs() {

  sweepRunning = false;

  vgsRunning = false;
  vgsStable = false;

  setDAC(MCP4728_CHANNEL_B, 0.0f);
  setDAC(MCP4728_CHANNEL_C, 0.0f);

  dacB = 0.0f;

  currentDACC = 0.0f;

  currentStep = -1;

  Serial.println("AUTO SWEEP STOPPED");
  Serial.println("DAC B/C RESET");
}

// =====================================
// SETUP
// =====================================
void setup() {

  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.setClock(100000);

  delay(2000);

  // MCP4728
  if (!mcp.begin(0x60, &Wire)) {

    Serial.println("MCP4728 ERROR");

    while (1) delay(100);
  }

  // ADS1115 #1
  if (!ads.begin(0x48, &Wire)) {

    Serial.println("ADS1115 #1 ERROR");

    while (1) delay(100);
  }

  // ADS1115 #2
  if (!adsBE.begin(0x49, &Wire)) {

    Serial.println("ADS1115 #2 ERROR");

    while (1) delay(100);
  }

  ads.setDataRate(RATE_ADS1115_128SPS);
  adsBE.setDataRate(RATE_ADS1115_128SPS);

  ads.setGain(GAIN_ONE);
  adsBE.setGain(GAIN_ONE);

  setDAC(MCP4728_CHANNEL_B, 0.0f);
  setDAC(MCP4728_CHANNEL_C, 0.0f);

  Serial.println();
  Serial.println("MOSFET CONTROL SYSTEM");

  Serial.println("MCP4728 OK");
  Serial.println("ADS1115 #1 OK (0x48)");
  Serial.println("ADS1115 #2 OK (0x49)");

  Serial.println();
  Serial.println("Commands:");

  Serial.println("s : Start VGS feedback");
  Serial.println("a : Start automatic DAC C sweep");
  Serial.println("n : Next single DAC C step");
  Serial.println("m : Measure all");
  Serial.println("t : Measure TIP41C Base/Emitter");
  Serial.println("x : Stop sweep and reset DAC B/C");

  Serial.println();
  Serial.println("SYSTEM READY");
}

// =====================================
// LOOP
// =====================================
void loop() {

  while (Serial.available() > 0) {

    char command = Serial.read();

    if (command == 's' || command == 'S') {

      if (!vgsRunning && !vgsStable) {

        startVGS();
      }
      else {

        Serial.println("VGS ALREADY RUNNING/STABLE");
      }
    }

    else if (command == 'a' || command == 'A') {

      startAutoSweep();
    }

    else if (command == 'n' || command == 'N') {

      nextDACCTest();
    }

    else if (command == 'm' || command == 'M') {

      printMeasurements();
    }

    else if (command == 't' || command == 'T') {

      printTIP41C();
    }

    else if (command == 'x' || command == 'X') {

      resetOutputs();
    }
  }

  if (vgsRunning) {

    updateVGS();
  }

  if (sweepRunning) {

    updateAutoSweep();
  }
}