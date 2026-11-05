
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include <stdlib.h>

// ======================================================
// ESP32-S3 MOSFET 자동 I-V 측정
//
// 1. VGS 최초 안정화
// 2. DAC C 예비 측정 (0~1.5V)
// 3. 베이스/이미터 전달률 측정
// 4. 이동 중앙값 필터로 전달률 이상치 탐색
// 5. 전달률 95~105% 선형 구간 탐색
// 6. 구간 시작점부터 끝까지 선형 회귀
// 7. 회귀식 예측 + 이중 폐루프 본 측정
// 8. 그래프용 CSV 데이터 출력
//
// 시리얼 모니터 안내 메시지: 한국어
//
// 주의:
// 소프트웨어 전류 및 전력 감시는
// 하드웨어 전류 제한을 대체하지 못함.
// ======================================================


// ======================================================
// 1. 자료형 정의
// ======================================================

enum ControlMode {
  MODE_FEEDBACK,
  MODE_REGRESSION,
  MODE_EXTRAPOLATION
};

struct Measurement {
  float gate;
  float source;
  float drainADC;
  float drain;

  float vgs;
  float vds;
  float id_mA;

  float base;
  float emitter;
  float opamp;

  float vbe;
  float ib_mA;
  float powerW;
};


// ======================================================
// 2. 함수 선언
// ======================================================

const char* modeName(ControlMode mode);

ControlMode predictDacC(
  float targetRel,
  float &predicted
);


// ======================================================
// 3. 하드웨어 설정
// ======================================================

Adafruit_MCP4728 mcp;
Adafruit_ADS1115 ads;
Adafruit_ADS1115 adsBE;

const int SDA_PIN = 8;
const int SCL_PIN = 9;

const float DAC_VDD = 3.3f;
const float DAC_B_MAX = 3.3f;

const float PRE_DAC_C_MAX = 1.5f;
const float MAIN_DAC_C_MAX = 2.0f;

const float DRAIN_GAIN = 4.0f;
const float BE_GAIN = 4.0f;

const float SHUNT_OHMS = 1.0f;
const float BASE_RESISTOR_OHMS = 1000.0f;


// ======================================================
// 4. 소프트웨어 감시 설정
// ======================================================

// 실제 하드웨어 전류 제한 장치가 아님.
const float MAX_CURRENT_A = 0.100f;

// 앞서 설정한 300mW 유지
const float MAX_DUT_POWER_W = 0.300f;

const float MAX_VDS_RAW = 10.20f;
const float MAX_GATE_V = 3.35f;


// ======================================================
// 5. 예비 측정 설정
// ======================================================

const int PRE_STEPS = 61;
const float PRE_DAC_STEP = 0.025f;


// ======================================================
// 6. 본 측정 설정
// ======================================================

const int MAIN_STEPS = 61;
const float TARGET_VDS_STEP = 0.1f;


// ======================================================
// 7. 전달률 설정
// ======================================================

// 전달률: delta VE / delta VB
const float TRANSFER_MIN = 0.95f;
const float TRANSFER_MAX = 1.05f;

// 4개 간격으로 전달률 계산
const int TRANSFER_SPAN = 4;

// 연속 5구간 확인
const int TRANSFER_CONFIRM = 5;

// 분모가 너무 작으면 계산하지 않음
const float MIN_BASE_DELTA_V = 0.050f;

// 이동 중앙값 필터
const int MEDIAN_WINDOW = 5;

// 1.5%p = 비율 0.015
const float TRANSFER_OUTLIER_LIMIT = 0.015f;


// ======================================================
// 8. 회귀식 설정
// ======================================================

const int MIN_REG_POINTS = 10;

const float MIN_R2 = 0.995f;
const float MAX_REG_ERROR_V = 0.050f;

// VDS 단독 이상치 판정 기준
const float VDS_OUTLIER_LIMIT_V = 0.050f;

// 회귀에서 자동 제외할 수 있는
// 고립된 VDS 이상치의 최대 개수
const int MAX_VDS_EXCLUSIONS = 3;

const bool ALLOW_GUARDED_EXTRAPOLATION = true;
const float MAX_EXTRAPOLATION_DAC_V = 0.50f;


// ======================================================
// 9. 폐루프 설정
// ======================================================

const float VGS_TOL = 0.020f;
const float VDS_TOL = 0.020f;

const float VGS_KP = 0.35f;
const float VDS_KP = 0.50f;

const float MAX_B_STEP = 0.010f;
const float MAX_C_STEP = 0.020f;

const int STABLE_REQUIRED = 5;

const int MAX_VGS_ITER = 500;
const int MAX_DUAL_ITER = 200;

const unsigned long SETTLE_MS = 60;

const float MAX_BASELINE_DRIFT_V = 0.050f;


// ======================================================
// 10. 전역 상태
// ======================================================

float dacB = 0.0f;
float dacC = 0.0f;

float targetVgs = 2.0f;

float vdsZeroOffset = 0.0f;

bool stopRequested = false;

String commandLine;


// ======================================================
// 11. 예비 측정 데이터
// ======================================================

float preDac[PRE_STEPS];
float preVgs[PRE_STEPS];

float preVdsRaw[PRE_STEPS];
float preVdsRel[PRE_STEPS];

float preId[PRE_STEPS];

float preBase[PRE_STEPS];
float preEmitter[PRE_STEPS];
float preVbe[PRE_STEPS];

float preTransferRaw[PRE_STEPS];
float preTransferFiltered[PRE_STEPS];

bool preTransferOutlier[PRE_STEPS];
bool preVdsExcluded[PRE_STEPS];

int preCount = 0;


// ======================================================
// 12. 본 측정 데이터
// ======================================================

float mainTarget[MAIN_STEPS];
float mainDacC[MAIN_STEPS];

float mainVgs[MAIN_STEPS];
float mainVdsRaw[MAIN_STEPS];
float mainVdsRel[MAIN_STEPS];

float mainId[MAIN_STEPS];
float mainPower[MAIN_STEPS];

float mainBase[MAIN_STEPS];
float mainEmitter[MAIN_STEPS];

ControlMode mainMode[MAIN_STEPS];

int mainCount = 0;


// ======================================================
// 13. 선형 회귀 결과
// ======================================================

bool regressionValid = false;

float regA = 0.0f;
float regB = 0.0f;

float regR2 = 0.0f;

float regMaxError = 0.0f;
float regMaxErrorDac = 0.0f;

float regMinDac = 0.0f;
float regMaxDac = 0.0f;

int regStartIndex = -1;
int regPointCount = 0;
int regExcludedCount = 0;


// ======================================================
// 14. DAC 제어
// ======================================================

uint16_t toCode(float v) {
  return (uint16_t)lroundf(
    constrain(v, 0.0f, DAC_VDD) *
    4095.0f / DAC_VDD
  );
}

void writeDAC(
  MCP4728_channel_t channel,
  float v
) {
  mcp.setChannelValue(
    channel,
    toCode(v),
    MCP4728_VREF_VDD,
    MCP4728_GAIN_1X
  );
}

void setDacB(float v) {
  dacB = constrain(v, 0.0f, DAC_B_MAX);

  writeDAC(
    MCP4728_CHANNEL_B,
    dacB
  );
}

void setDacC(float v) {
  dacC = constrain(
    v,
    0.0f,
    MAIN_DAC_C_MAX
  );

  writeDAC(
    MCP4728_CHANNEL_C,
    dacC
  );
}

void outputsOff() {
  setDacC(0.0f);
  setDacB(0.0f);
}


// ======================================================
// 15. 중단 및 오류 처리
// ======================================================

void pollStop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'x' || c == 'X') {
      stopRequested = true;
      outputsOff();

      Serial.println(
        "[사용자 중단] DAC B/C 출력을 0V로 설정했습니다."
      );
    }
  }
}

bool fault(const char *reason) {
  stopRequested = true;
  outputsOff();

  Serial.print("[오류로 중단] ");
  Serial.println(reason);

  return false;
}

bool stopForLimit(
  const char *name,
  float value
) {
  stopRequested = true;
  outputsOff();

  Serial.print("[설정 한계 도달] ");
  Serial.print(name);
  Serial.print(" = ");
  Serial.println(value, 5);

  return false;
}


// ======================================================
// 16. ADC 측정
// ======================================================

float readShuntVoltage() {
  ads.setGain(GAIN_SIXTEEN);

  int16_t raw =
    ads.readADC_Differential_0_3();

  return ads.computeVolts(raw);
}

float readADC1(uint8_t channel) {
  ads.setGain(GAIN_ONE);

  float sum = 0.0f;

  for (int i = 0; i < 3; i++) {
    int16_t raw =
      ads.readADC_SingleEnded(channel);

    sum += ads.computeVolts(raw);
  }

  return sum / 3.0f;
}

float readADC2(uint8_t channel) {
  adsBE.setGain(GAIN_ONE);

  float sum = 0.0f;

  for (int i = 0; i < 3; i++) {
    int16_t raw =
      adsBE.readADC_SingleEnded(channel);

    sum += adsBE.computeVolts(raw);
  }

  return sum / 3.0f;
}


// ======================================================
// 17. 전류 감시
// ======================================================

bool checkCurrent() {
  pollStop();

  if (stopRequested)
    return false;

  float shuntV = readShuntVoltage();

  if (!isfinite(shuntV)) {
    return fault(
      "션트 ADC 측정값이 유효하지 않습니다."
    );
  }

  float id =
    shuntV / SHUNT_OHMS;

  if (fabsf(id) >= MAX_CURRENT_A) {
    return stopForLimit(
      "드레인 전류(mA)",
      id * 1000.0f
    );
  }

  return true;
}


// ======================================================
// 18. 전체 측정
// ======================================================

bool measure(Measurement &m) {
  if (!checkCurrent())
    return false;

  m.source = readShuntVoltage();

  if (!isfinite(m.source)) {
    return fault(
      "션트 전압 측정 오류"
    );
  }

  m.id_mA =
    m.source * 1000.0f /
    SHUNT_OHMS;

  if (fabsf(m.id_mA) >=
      MAX_CURRENT_A * 1000.0f) {

    return stopForLimit(
      "드레인 전류(mA)",
      m.id_mA
    );
  }

  m.gate = readADC1(2);
  m.drainADC = readADC1(1);

  m.drain =
    m.drainADC * DRAIN_GAIN;

  m.vgs =
    m.gate - m.source;

  m.vds =
    m.drain - m.source;

  if (!isfinite(m.gate) ||
      !isfinite(m.vgs) ||
      !isfinite(m.vds)) {

    return fault(
      "게이트 또는 드레인 전압 측정 오류"
    );
  }

  if (m.drainADC < -0.030f ||
      m.vds < -0.080f) {

    return fault(
      "드레인 ADC에서 비정상적인 음수 전압이 측정되었습니다."
    );
  }

  if (m.gate > MAX_GATE_V) {
    return stopForLimit(
      "게이트 전압(V)",
      m.gate
    );
  }

  m.powerW =
    fmaxf(m.vds, 0.0f) *
    fabsf(m.id_mA / 1000.0f);

  if (m.powerW >=
      MAX_DUT_POWER_W) {

    return stopForLimit(
      "MOSFET 소비 전력(mW)",
      m.powerW * 1000.0f
    );
  }

  if (m.vds > MAX_VDS_RAW) {
    return stopForLimit(
      "실제 VDS(V)",
      m.vds
    );
  }

  m.base =
    readADC2(0) * BE_GAIN;

  m.emitter =
    readADC2(1) * BE_GAIN;

  m.opamp =
    readADC2(2) * BE_GAIN;

  m.vbe =
    m.base - m.emitter;

  m.ib_mA =
    (m.opamp - m.base) /
    BASE_RESISTOR_OHMS *
    1000.0f;

  if (!isfinite(m.base) ||
      !isfinite(m.emitter) ||
      !isfinite(m.opamp)) {

    return fault(
      "베이스 또는 이미터 측정 오류"
    );
  }

  return checkCurrent();
}


// ======================================================
// 19. 감시 대기
// ======================================================

bool waitMonitored(unsigned long ms) {
  unsigned long started = millis();

  do {
    pollStop();

    if (stopRequested)
      return false;

    Measurement m = {};

    if (!measure(m))
      return false;

    delay(5);

  } while (millis() - started < ms);

  return true;
}


// ======================================================
// 20. VGS 안정화
// ======================================================

bool stabilizeVGS(
  float target,
  bool initialReset
) {
  if (initialReset)
    outputsOff();

  if (initialReset) {
    Serial.println(
      "\n[1단계] VGS 최초 안정화"
    );
  } else {
    Serial.println(
      "\n[확인] 현재 DAC B에서 VGS 미세 보정"
    );
  }

  int stable = 0;

  for (int i = 0;
       i < MAX_VGS_ITER;
       i++) {

    Measurement m = {};

    if (!measure(m))
      return false;

    float error =
      target - m.vgs;

    if (i % 10 == 0) {
      Serial.print(
        "[VGS 조정] 반복="
      );

      Serial.print(i);

      Serial.print(" / DAC B=");
      Serial.print(dacB, 5);

      Serial.print("V / VGS=");
      Serial.print(m.vgs, 5);

      Serial.print("V / 오차=");
      Serial.println(error, 5);
    }

    if (fabsf(error) <= VGS_TOL) {
      stable++;

      if (stable >= STABLE_REQUIRED) {
        Serial.print(
          "[VGS 안정화 완료] "
        );

        Serial.println(m.vgs, 5);
        return true;
      }
    } else {
      stable = 0;

      float change = constrain(
        VGS_KP * error,
        -MAX_B_STEP,
        MAX_B_STEP
      );

      float next = constrain(
        dacB + change,
        0.0f,
        DAC_B_MAX
      );

      if (fabsf(next - dacB) <
          0.0001f) {

        return fault(
          "DAC B 출력 한계에 도달했습니다."
        );
      }

      setDacB(next);
    }

    if (!waitMonitored(SETTLE_MS))
      return false;
  }

  return fault(
    "VGS 안정화 반복 횟수를 초과했습니다."
  );
}


// ======================================================
// 21. 전달률 계산
// ======================================================

float calculateTransferRatio(int i) {
  if (i < TRANSFER_SPAN)
    return NAN;

  int first =
    i - TRANSFER_SPAN;

  float dB =
    preBase[i] - preBase[first];

  float dE =
    preEmitter[i] - preEmitter[first];

  if (!isfinite(dB) ||
      !isfinite(dE) ||
      dB < MIN_BASE_DELTA_V ||
      dE < 0.0f) {

    return NAN;
  }

  return dE / dB;
}


// ======================================================
// 22. 예비 측정
// ======================================================

bool preliminarySweep() {
  Serial.println(
    "\n[2단계] DAC C 예비 측정"
  );

  preCount = 0;
  regressionValid = false;

  setDacC(0.0f);

  if (!waitMonitored(300))
    return false;

  Measurement zero = {};

  if (!measure(zero))
    return false;

  vdsZeroOffset = zero.vds;

  Serial.print(
    "[초기 VDS 기준값] "
  );

  Serial.println(vdsZeroOffset, 5);

  Serial.println(
    "예비순번,DAC_C_V,VGS_V,VDS_실측_V,"
    "VDS_상대_V,ID_mA,베이스_V,이미터_V,"
    "VBE_V,전달률_원본_퍼센트,전력_mW"
  );

  for (int step = 0;
       step < PRE_STEPS;
       step++) {

    float command = fminf(
      step * PRE_DAC_STEP,
      PRE_DAC_C_MAX
    );

    setDacC(command);

    if (!waitMonitored(100))
      return false;

    int gsStable = 0;
    bool gsOK = false;

    for (int iter = 0;
         iter < 100;
         iter++) {

      Measurement t = {};

      if (!measure(t))
        return false;

      float error =
        targetVgs - t.vgs;

      if (fabsf(error) <= VGS_TOL) {
        gsStable++;

        if (gsStable >= 3) {
          gsOK = true;
          break;
        }
      } else {
        gsStable = 0;

        setDacB(
          dacB + constrain(
            VGS_KP * error,
            -MAX_B_STEP,
            MAX_B_STEP
          )
        );
      }

      if (!waitMonitored(SETTLE_MS))
        return false;
    }

    if (!gsOK) {
      return fault(
        "예비 측정 중 VGS 안정화 실패"
      );
    }

    Measurement m = {};

    if (!measure(m))
      return false;

    int i = preCount;

    preDac[i] = dacC;
    preVgs[i] = m.vgs;

    preVdsRaw[i] = m.vds;

    preVdsRel[i] =
      m.vds - vdsZeroOffset;

    preId[i] = m.id_mA;

    preBase[i] = m.base;
    preEmitter[i] = m.emitter;
    preVbe[i] = m.vbe;

    preTransferRaw[i] = NAN;
    preTransferFiltered[i] = NAN;

    preTransferOutlier[i] = false;
    preVdsExcluded[i] = false;

    preCount++;

    preTransferRaw[i] =
      calculateTransferRatio(i);

    Serial.print(i + 1);
    Serial.print(",");

    Serial.print(preDac[i], 5);
    Serial.print(",");

    Serial.print(preVgs[i], 5);
    Serial.print(",");

    Serial.print(preVdsRaw[i], 5);
    Serial.print(",");

    Serial.print(preVdsRel[i], 5);
    Serial.print(",");

    Serial.print(preId[i], 5);
    Serial.print(",");

    Serial.print(preBase[i], 5);
    Serial.print(",");

    Serial.print(preEmitter[i], 5);
    Serial.print(",");

    Serial.print(preVbe[i], 5);
    Serial.print(",");

    if (isfinite(preTransferRaw[i])) {
      Serial.print(
        preTransferRaw[i] * 100.0f,
        3
      );
    } else {
      Serial.print("계산불가");
    }

    Serial.print(",");
    Serial.println(
      m.powerW * 1000.0f,
      5
    );
  }

  Serial.print(
    "[예비 측정 완료] 데이터 "
  );

  Serial.print(preCount);
  Serial.println("개");

  return true;
}


// ======================================================
// 23. 5개 중앙값 계산
// ======================================================

float medianOfFive(
  float a,
  float b,
  float c,
  float d,
  float e
) {
  float v[5] = {a, b, c, d, e};

  for (int i = 1; i < 5; i++) {
    float key = v[i];
    int j = i - 1;

    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }

    v[j + 1] = key;
  }

  return v[2];
}


// ======================================================
// 24. 전달률 중앙값 필터
//
// 98,99,105,99,98 -> 중앙값 99
//
// 원본은 보존하고 필터값을 별도로 저장.
//
// 중앙값 필터 결과만으로
// VDS 측정점을 삭제하지 않음.
// ======================================================

void filterTransferRatios() {
  Serial.println(
    "\n[3단계] 전달률 이동 중앙값 필터"
  );

  int outliers = 0;

  for (int i = 0;
       i < preCount;
       i++) {

    preTransferFiltered[i] =
      preTransferRaw[i];

    preTransferOutlier[i] = false;
  }

  for (int i = 2;
       i < preCount - 2;
       i++) {

    bool windowOK = true;

    for (int k = i - 2;
         k <= i + 2;
         k++) {

      if (!isfinite(preTransferRaw[k])) {
        windowOK = false;
        break;
      }
    }

    if (!windowOK)
      continue;

    float median = medianOfFive(
      preTransferRaw[i - 2],
      preTransferRaw[i - 1],
      preTransferRaw[i],
      preTransferRaw[i + 1],
      preTransferRaw[i + 2]
    );

    float difference =
      fabsf(
        preTransferRaw[i] - median
      );

    if (difference >
        TRANSFER_OUTLIER_LIMIT) {

      preTransferOutlier[i] = true;
      preTransferFiltered[i] = median;

      outliers++;

      Serial.print(
        "[전달률 이상치 후보] DAC C="
      );

      Serial.print(preDac[i], 5);

      Serial.print("V / 원본=");
      Serial.print(
        preTransferRaw[i] * 100.0f,
        3
      );

      Serial.print("% / 중앙값=");
      Serial.print(
        median * 100.0f,
        3
      );

      Serial.println("%");
    }
  }

  Serial.print(
    "[필터 완료] 이상치 후보 "
  );

  Serial.print(outliers);
  Serial.println("개");

  Serial.println(
    "※ 전달률 이상치만으로 VDS 데이터는 삭제하지 않습니다."
  );
}


// ======================================================
// 25. 회귀 시작점 탐색
// ======================================================

int findTransferStart() {
  Serial.println(
    "\n[4단계] 선형 구간 시작점 탐색"
  );

  int consecutive = 0;
  int firstEnd = -1;

  for (int i = TRANSFER_SPAN;
       i < preCount;
       i++) {

    float ratio =
      preTransferFiltered[i];

    bool valid =
      isfinite(ratio) &&
      ratio >= TRANSFER_MIN &&
      ratio <= TRANSFER_MAX;

    if (valid) {
      if (consecutive == 0)
        firstEnd = i;

      consecutive++;

      if (consecutive >= TRANSFER_CONFIRM) {

        // 전달률 판정이 충족된 첫 번째
        // 측정 위치부터 회귀 구간 시작
        int start = firstEnd;

        Serial.print(
          "[회귀 시작점] DAC C="
        );

        Serial.print(
          preDac[start],
          5
        );

        Serial.println("V");

        Serial.print(
          "[회귀 구간] 시작 순번="
        );

        Serial.println(start + 1);

        return start;
      }
    } else {
      consecutive = 0;
      firstEnd = -1;
    }
  }

  Serial.println(
    "[판정 실패] 전달률 조건을 만족한 "
    "연속 구간이 없습니다."
  );

  return -1;
}


// ======================================================
// 26. VDS 고립 이상치 탐색
//
// 주변 두 점은 증가 추세인데
// 가운데 한 점만 갑자기 튀는 경우 검출.
//
// 실제 비선형 구간을 임의로 삭제하지 않도록
// 회귀 전 고립형 이상치만 판정.
//
// 제거 여부를 CSV에도 기록.
// ======================================================

void detectVdsOutliers(int start) {
  int excluded = 0;

  for (int i = 0;
       i < preCount;
       i++) {

    preVdsExcluded[i] = false;
  }

  for (int i = start + 1;
       i < preCount - 1;
       i++) {

    if (excluded >= MAX_VDS_EXCLUSIONS)
      break;

    float y0 = preVdsRel[i - 1];
    float y1 = preVdsRel[i];
    float y2 = preVdsRel[i + 1];

    float expected =
      (y0 + y2) * 0.5f;

    float deviation =
      fabsf(y1 - expected);

    // 특정 점만 튀고,
    // 해당 점에서 부자연스러운 역전이
    // 발생했는지 확인
    bool isolated =
      (y1 < y0 && y2 > y1) ||
      (y1 > y2 && y1 > y0);

    if (isolated &&
        deviation > VDS_OUTLIER_LIMIT_V) {

      preVdsExcluded[i] = true;
      excluded++;

      Serial.print(
        "[VDS 고립 이상치] DAC C="
      );

      Serial.print(preDac[i], 5);

      Serial.print("V / 실측 상대 VDS=");
      Serial.print(y1, 5);

      Serial.print("V / 주변 예상값=");
      Serial.print(expected, 5);

      Serial.println("V");
    }
  }

  Serial.print(
    "[VDS 이상치 검사] 회귀 제외 후보="
  );

  Serial.println(excluded);
}


// ======================================================
// 27. 선형 회귀 계산
//
// 전달률 시작점 ~ 예비 측정 마지막.
//
// 고립된 VDS 이상치만 제외하고
// 나머지 모든 데이터를 회귀에 사용.
// ======================================================

bool calculateRegression() {
  regressionValid = false;

  filterTransferRatios();

  int start = findTransferStart();

  if (start < 0 ||
      preCount - start < MIN_REG_POINTS) {

    Serial.println(
      "[회귀 사용 보류] 데이터가 부족합니다."
    );

    return true;
  }

  detectVdsOutliers(start);

  double sx = 0.0;
  double sy = 0.0;
  double sxx = 0.0;
  double sxy = 0.0;

  int n = 0;
  int excluded = 0;

  for (int i = start;
       i < preCount;
       i++) {

    if (preVdsExcluded[i]) {
      excluded++;
      continue;
    }

    double x = preDac[i];
    double y = preVdsRel[i];

    sx += x;
    sy += y;
    sxx += x * x;
    sxy += x * y;

    n++;
  }

  if (n < MIN_REG_POINTS) {
    Serial.println(
      "[회귀 사용 보류] 유효 데이터 수가 부족합니다."
    );

    return true;
  }

  double denominator =
    n * sxx - sx * sx;

  if (fabs(denominator) < 1e-12) {
    Serial.println(
      "[회귀 사용 보류] 계산 분모가 너무 작습니다."
    );

    return true;
  }

  double a =
    (n * sxy - sx * sy) /
    denominator;

  double b =
    (sy - a * sx) / n;

  if (!isfinite(a) ||
      !isfinite(b) ||
      a <= 0.0) {

    Serial.println(
      "[회귀 사용 보류] 회귀 계수가 유효하지 않습니다."
    );

    return true;
  }

  double meanY = sy / n;

  double ssTot = 0.0;
  double ssRes = 0.0;

  regMaxError = 0.0f;
  regMaxErrorDac = preDac[start];

  for (int i = start;
       i < preCount;
       i++) {

    if (preVdsExcluded[i])
      continue;

    double y = preVdsRel[i];

    double expected =
      a * preDac[i] + b;

    double residual =
      y - expected;

    ssTot +=
      (y - meanY) * (y - meanY);

    ssRes +=
      residual * residual;

    if (fabs(residual) >
        regMaxError) {

      regMaxError =
        (float)fabs(residual);

      regMaxErrorDac =
        preDac[i];
    }
  }

  if (ssTot < 1e-12) {
    Serial.println(
      "[회귀 사용 보류] VDS 변화량이 부족합니다."
    );

    return true;
  }

  regA = (float)a;
  regB = (float)b;

  regR2 =
    1.0f - (float)(ssRes / ssTot);

  regMinDac = preDac[start];
  regMaxDac = preDac[preCount - 1];

  regStartIndex = start;
  regPointCount = n;
  regExcludedCount = excluded;

  Serial.println(
    "\n[5단계] 선형 회귀 결과"
  );

  Serial.print(
    "회귀 시작 DAC C(V): "
  );
  Serial.println(regMinDac, 5);

  Serial.print(
    "회귀 종료 DAC C(V): "
  );
  Serial.println(regMaxDac, 5);

  Serial.print(
    "회귀 사용 데이터 수: "
  );
  Serial.println(regPointCount);

  Serial.print(
    "회귀 제외 데이터 수: "
  );
  Serial.println(regExcludedCount);

  Serial.print("기울기 a: ");
  Serial.println(regA, 7);

  Serial.print("절편 b: ");
  Serial.println(regB, 7);

  Serial.print("결정계수 R²: ");
  Serial.println(regR2, 7);

  Serial.print("최대 잔차(V): ");
  Serial.println(regMaxError, 6);

  Serial.print(
    "최대 잔차 발생 DAC C(V): "
  );
  Serial.println(regMaxErrorDac, 5);

  Serial.println(
    "회귀식: VDS_상대 = a × DAC C + b"
  );

  if (regR2 < MIN_R2 ||
      regMaxError > MAX_REG_ERROR_V) {

    Serial.println(
      "[회귀 사용 보류] R² 또는 잔차 기준 미충족"
    );

    Serial.println(
      "[제어 방식] 실측 폐루프만 사용합니다."
    );

    return true;
  }

  regressionValid = true;

  Serial.println(
    "[회귀식 승인] 예측 제어에 사용합니다."
  );

  return true;
}


// ======================================================
// 28. 제어 방식 표시
// ======================================================

const char* modeName(ControlMode mode) {
  if (mode == MODE_REGRESSION)
    return "회귀예측_폐루프";

  if (mode == MODE_EXTRAPOLATION)
    return "외삽예측_폐루프";

  return "실측폐루프";
}


// ======================================================
// 29. DAC C 예측
// ======================================================

ControlMode predictDacC(
  float targetRel,
  float &predicted
) {
  predicted = dacC;

  if (!regressionValid ||
      fabsf(regA) < 1e-6f) {

    return MODE_FEEDBACK;
  }

  float estimate =
    (targetRel - regB) / regA;

  if (!isfinite(estimate) ||
      estimate < regMinDac) {

    return MODE_FEEDBACK;
  }

  if (estimate <= regMaxDac) {
    predicted = constrain(
      estimate,
      0.0f,
      MAIN_DAC_C_MAX
    );

    return MODE_REGRESSION;
  }

  if (ALLOW_GUARDED_EXTRAPOLATION &&
      estimate <=
        regMaxDac +
        MAX_EXTRAPOLATION_DAC_V &&
      estimate <= MAIN_DAC_C_MAX) {

    predicted = estimate;

    return MODE_EXTRAPOLATION;
  }

  return MODE_FEEDBACK;
}


// ======================================================
// 30. 예비 측정 후 DAC C 복귀
//
// DAC B는 유지.
// DAC C만 단계적으로 0V까지 감소.
// ======================================================

bool returnDacCToZeroKeepingVGS() {
  Serial.println(
    "\n[전환 단계] VGS 유지 및 DAC C 복귀"
  );

  int count = 0;

  while (dacC > 0.001f &&
         count < 150) {

    setDacC(
      fmaxf(
        0.0f,
        dacC - MAX_C_STEP
      )
    );

    if (!waitMonitored(SETTLE_MS))
      return false;

    Measurement m = {};

    if (!measure(m))
      return false;

    float error =
      targetVgs - m.vgs;

    if (fabsf(error) > VGS_TOL) {
      setDacB(
        dacB + constrain(
          VGS_KP * error,
          -MAX_B_STEP,
          MAX_B_STEP
        )
      );

      if (!waitMonitored(SETTLE_MS))
        return false;
    }

    if (count % 10 == 0) {
      Serial.print(
        "[DAC C 복귀] DAC C="
      );

      Serial.print(dacC, 5);

      Serial.print("V / VGS=");
      Serial.println(m.vgs, 5);
    }

    count++;
  }

  if (dacC > 0.001f) {
    return fault(
      "DAC C 복귀 반복 횟수 초과"
    );
  }

  if (!stabilizeVGS(targetVgs, false))
    return false;

  Serial.println(
    "[전환 완료] DAC C=0V, VGS 유지"
  );

  return true;
}


// ======================================================
// 31. 이중 폐루프 제어
// ======================================================

bool stabilizeDual(
  float gsTarget,
  float dsTarget
) {
  int stable = 0;
  int noMove = 0;

  for (int i = 0;
       i < MAX_DUAL_ITER;
       i++) {

    Measurement m = {};

    if (!measure(m))
      return false;

    float vdsRel =
      m.vds - vdsZeroOffset;

    float errorGS =
      gsTarget - m.vgs;

    float errorDS =
      dsTarget - vdsRel;

    bool gsOK =
      fabsf(errorGS) <= VGS_TOL;

    bool dsOK =
      fabsf(errorDS) <= VDS_TOL;

    if (gsOK && dsOK) {
      stable++;

      if (stable >= STABLE_REQUIRED)
        return true;
    } else {
      stable = 0;
    }

    float prevB = dacB;
    float prevC = dacC;

    if (!gsOK) {
      setDacB(
        dacB + constrain(
          VGS_KP * errorGS,
          -MAX_B_STEP,
          MAX_B_STEP
        )
      );
    }

    if (!dsOK) {
      setDacC(
        dacC + constrain(
          VDS_KP * errorDS / 4.0f,
          -MAX_C_STEP,
          MAX_C_STEP
        )
      );
    }

    if (i % 10 == 0) {
      Serial.print(
        "[이중 폐루프] 반복="
      );

      Serial.print(i);

      Serial.print(" / DAC C=");
      Serial.print(dacC, 5);

      Serial.print("V / VGS=");
      Serial.print(m.vgs, 5);

      Serial.print("V / VDS 상대=");
      Serial.print(vdsRel, 5);

      Serial.print("V / ID=");
      Serial.print(m.id_mA, 5);

      Serial.println("mA");
    }

    bool blocked =
      (!gsOK &&
       fabsf(prevB - dacB) < 0.0001f) ||
      (!dsOK &&
       fabsf(prevC - dacC) < 0.0001f);

    if (blocked) {
      noMove++;

      if (noMove >= 10) {
        return fault(
          "DAC 출력 한계로 목표 전압에 도달하지 못했습니다."
        );
      }
    } else {
      noMove = 0;
    }

    if (!waitMonitored(SETTLE_MS))
      return false;
  }

  return fault(
    "이중 폐루프 반복 횟수 초과"
  );
}


// ======================================================
// 32. 예측 DAC C 접근
// ======================================================

bool approachPredictedDac(float target) {
  target = constrain(
    target,
    0.0f,
    MAIN_DAC_C_MAX
  );

  for (int i = 0; i < 150; i++) {
    float difference =
      target - dacC;

    if (fabsf(difference) <= 0.001f)
      return true;

    setDacC(
      dacC + constrain(
        difference,
        -MAX_C_STEP,
        MAX_C_STEP
      )
    );

    if (!waitMonitored(SETTLE_MS))
      return false;
  }

  return fault(
    "예측 DAC C 값 접근 실패"
  );
}


// ======================================================
// 33. 최종 측정 검증
// ======================================================

bool verifyFinalPoint(
  float gsTarget,
  float dsTarget,
  Measurement &m
) {
  for (int attempt = 0;
       attempt < 3;
       attempt++) {

    if (!measure(m))
      return false;

    float rel =
      m.vds - vdsZeroOffset;

    bool gsOK =
      fabsf(m.vgs - gsTarget) <= VGS_TOL;

    bool dsOK =
      fabsf(rel - dsTarget) <= VDS_TOL;

    if (gsOK && dsOK)
      return true;

    Serial.print(
      "[최종 측정 재보정] 시도 "
    );

    Serial.println(attempt + 1);

    if (!stabilizeDual(
          gsTarget,
          dsTarget
        )) {

      return false;
    }
  }

  return fault(
    "최종 VGS/VDS 허용오차 초과"
  );
}


// ======================================================
// 34. 본 측정
// ======================================================

bool mainSweep() {
  Serial.println(
    "\n[6단계] 목표 VDS 본 측정"
  );

  if (!returnDacCToZeroKeepingVGS())
    return false;

  Measurement zero = {};

  if (!measure(zero))
    return false;

  float drift =
    zero.vds - vdsZeroOffset;

  Serial.print(
    "[영점 확인] 기준 변화(V): "
  );

  Serial.println(drift, 5);

  if (fabsf(drift) >
      MAX_BASELINE_DRIFT_V) {

    return fault(
      "VDS 초기 기준값 변화가 50mV를 초과했습니다."
    );
  }

  Serial.println(
    "본측정순번,VDS_목표_V,DAC_C_V,"
    "제어방식,VGS_V,VDS_실측_V,"
    "VDS_상대_V,ID_mA,베이스_V,"
    "이미터_V,OPAMP_V,VBE_V,"
    "IB_mA,전력_mW"
  );

  mainCount = 0;

  for (int step = 0;
       step < MAIN_STEPS;
       step++) {

    float dsTarget =
      step * TARGET_VDS_STEP;

    if (!checkCurrent())
      return false;

    float predicted = dacC;

    ControlMode mode =
      predictDacC(
        dsTarget,
        predicted
      );

    Serial.print("\n[본 측정 ");
    Serial.print(step + 1);

    Serial.print("/");
    Serial.print(MAIN_STEPS);

    Serial.print("] 목표 VDS=");
    Serial.println(dsTarget, 4);

    Serial.print("[제어 방식] ");
    Serial.println(modeName(mode));

    if (mode != MODE_FEEDBACK) {
      Serial.print(
        "[회귀 예상 DAC C] "
      );

      Serial.println(predicted, 5);

      if (!approachPredictedDac(predicted))
        return false;
    }

    if (!stabilizeDual(
          targetVgs,
          dsTarget
        )) {

      return false;
    }

    Measurement m = {};

    if (!verifyFinalPoint(
          targetVgs,
          dsTarget,
          m
        )) {

      return false;
    }

    // 데이터를 메모리에 저장
    int i = mainCount;

    mainTarget[i] = dsTarget;
    mainDacC[i] = dacC;

    mainVgs[i] = m.vgs;

    mainVdsRaw[i] = m.vds;

    mainVdsRel[i] =
      m.vds - vdsZeroOffset;

    mainId[i] = m.id_mA;
    mainPower[i] = m.powerW * 1000.0f;

    mainBase[i] = m.base;
    mainEmitter[i] = m.emitter;

    mainMode[i] = mode;

    mainCount++;

    // 본 측정 CSV 출력
    Serial.print(step + 1);
    Serial.print(",");

    Serial.print(dsTarget, 5);
    Serial.print(",");

    Serial.print(dacC, 5);
    Serial.print(",");

    Serial.print(modeName(mode));
    Serial.print(",");

    Serial.print(m.vgs, 5);
    Serial.print(",");

    Serial.print(m.vds, 5);
    Serial.print(",");

    Serial.print(
      m.vds - vdsZeroOffset,
      5
    );
    Serial.print(",");

    Serial.print(m.id_mA, 5);
    Serial.print(",");

    Serial.print(m.base, 5);
    Serial.print(",");

    Serial.print(m.emitter, 5);
    Serial.print(",");

    Serial.print(m.opamp, 5);
    Serial.print(",");

    Serial.print(m.vbe, 5);
    Serial.print(",");

    Serial.print(m.ib_mA, 5);
    Serial.print(",");

    Serial.println(
      m.powerW * 1000.0f,
      5
    );
  }

  return true;
}


// ======================================================
// 35. 그래프 데이터 출력
//
// 시리얼 명령어: g
//
// 예비 측정:
// DAC C - VDS
// DAC C - 베이스/이미터
// DAC C - 전달률
//
// 본 측정:
// VDS - ID
// VDS - VGS
// VDS - 소비 전력
//
// 출력된 CSV를 Excel 등에 붙여넣으면
// XY 산점도 그래프 작성 가능.
// ======================================================

void printGraphData() {
  Serial.println();
  Serial.println(
    "========== 그래프 데이터 추출 시작 =========="
  );

  if (preCount > 0) {
    Serial.println();
    Serial.println(
      "[그래프 1] 예비 측정 데이터"
    );

    Serial.println(
      "DAC_C_V,VDS_실측_V,VDS_상대_V,"
      "베이스_V,이미터_V,"
      "전달률_원본_퍼센트,"
      "전달률_필터_퍼센트,"
      "전달률_이상치,회귀_제외"
    );

    for (int i = 0;
         i < preCount;
         i++) {

      Serial.print(preDac[i], 5);
      Serial.print(",");

      Serial.print(preVdsRaw[i], 5);
      Serial.print(",");

      Serial.print(preVdsRel[i], 5);
      Serial.print(",");

      Serial.print(preBase[i], 5);
      Serial.print(",");

      Serial.print(preEmitter[i], 5);
      Serial.print(",");

      if (isfinite(preTransferRaw[i])) {
        Serial.print(
          preTransferRaw[i] * 100.0f,
          4
        );
      }

      Serial.print(",");

      if (isfinite(preTransferFiltered[i])) {
        Serial.print(
          preTransferFiltered[i] * 100.0f,
          4
        );
      }

      Serial.print(",");

      Serial.print(
        preTransferOutlier[i] ? 1 : 0
      );

      Serial.print(",");

      Serial.println(
        preVdsExcluded[i] ? 1 : 0
      );
    }
  }

  if (mainCount > 0) {
    Serial.println();
    Serial.println(
      "[그래프 2] 본 측정 I-V 데이터"
    );

    Serial.println(
      "VDS_실측_V,ID_mA,VGS_V,"
      "VDS_목표_V,VDS_상대_V,"
      "DAC_C_V,전력_mW,베이스_V,이미터_V"
    );

    for (int i = 0;
         i < mainCount;
         i++) {

      Serial.print(mainVdsRaw[i], 5);
      Serial.print(",");

      Serial.print(mainId[i], 5);
      Serial.print(",");

      Serial.print(mainVgs[i], 5);
      Serial.print(",");

      Serial.print(mainTarget[i], 5);
      Serial.print(",");

      Serial.print(mainVdsRel[i], 5);
      Serial.print(",");

      Serial.print(mainDacC[i], 5);
      Serial.print(",");

      Serial.print(mainPower[i], 5);
      Serial.print(",");

      Serial.print(mainBase[i], 5);
      Serial.print(",");

      Serial.println(mainEmitter[i], 5);
    }
  }

  if (regStartIndex >= 0) {
    Serial.println();
    Serial.println(
      "[그래프 3] 회귀식 비교 데이터"
    );

    Serial.println(
      "DAC_C_V,VDS_실측_상대_V,"
      "VDS_회귀예측_V,잔차_V,회귀_제외"
    );

    for (int i = regStartIndex;
         i < preCount;
         i++) {

      float prediction =
        regA * preDac[i] + regB;

      float residual =
        preVdsRel[i] - prediction;

      Serial.print(preDac[i], 5);
      Serial.print(",");

      Serial.print(preVdsRel[i], 5);
      Serial.print(",");

      Serial.print(prediction, 5);
      Serial.print(",");

      Serial.print(residual, 5);
      Serial.print(",");

      Serial.println(
        preVdsExcluded[i] ? 1 : 0
      );
    }
  }

  Serial.println();
  Serial.println(
    "========== 그래프 데이터 추출 완료 =========="
  );
}


// ======================================================
// 36. 전체 실행
// ======================================================

void runAll(float gsTarget) {
  stopRequested = false;
  regressionValid = false;

  preCount = 0;
  mainCount = 0;

  regStartIndex = -1;

  targetVgs = gsTarget;

  outputsOff();

  Serial.println();
  Serial.println(
    "=========================================="
  );

  Serial.println(
    "MOSFET 자동 I-V 측정 시작"
  );

  Serial.print("VGS 목표(V): ");
  Serial.println(targetVgs, 4);

  Serial.println(
    "예비 측정 DAC C: 0~1.5V, 간격 0.025V"
  );

  Serial.println(
    "본 측정 VDS 상대 목표: 0~6V, 간격 0.1V"
  );

  Serial.println(
    "전달률: 95~105%, 연속 5구간"
  );

  Serial.println(
    "이동 중앙값: 5개 측정점"
  );

  Serial.println(
    "전달률 이상치 기준: 1.5%p"
  );

  Serial.print(
    "소프트웨어 전력 감시(mW): "
  );

  Serial.println(
    MAX_DUT_POWER_W * 1000.0f,
    1
  );

  Serial.println(
    "주의: 하드웨어 전류 제한 장치가 필요합니다."
  );

  Serial.println(
    "=========================================="
  );

  if (!stabilizeVGS(
        targetVgs,
        true
      )) {

    outputsOff();
    return;
  }

  if (!preliminarySweep()) {
    outputsOff();

    Serial.println(
      "[측정 종료] 예비 측정 미완료"
    );

    return;
  }

  if (!calculateRegression()) {
    outputsOff();
    return;
  }

  bool success = mainSweep();

  outputsOff();

  if (success) {
    Serial.println();
    Serial.println(
      "[측정 완료] 본 스윕 61개 완료"
    );
  } else {
    Serial.println();
    Serial.println(
      "[측정 종료] 본 스윕 중단"
    );
  }

  Serial.println(
    "DAC B/C는 0V로 설정했습니다."
  );

  Serial.println(
    "주의: 외부 12V 전원은 별도로 차단해야 합니다."
  );

  Serial.println();
  Serial.println(
    "그래프 데이터를 다시 보려면 g를 입력하세요."
  );
}


// ======================================================
// 37. 초기 설정
// ======================================================

void setup() {
  Serial.begin(115200);

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(100000);
  Wire.setTimeOut(100);

  delay(500);

  if (!mcp.begin(0x60, &Wire)) {
    Serial.println(
      "[연결 오류] MCP4728(0x60)"
    );

    while (true)
      delay(100);
  }

  if (!ads.begin(0x48, &Wire)) {
    Serial.println(
      "[연결 오류] ADS1115 1번(0x48)"
    );

    while (true)
      delay(100);
  }

  if (!adsBE.begin(0x49, &Wire)) {
    Serial.println(
      "[연결 오류] ADS1115 2번(0x49)"
    );

    while (true)
      delay(100);
  }

  ads.setDataRate(
    RATE_ADS1115_128SPS
  );

  adsBE.setDataRate(
    RATE_ADS1115_128SPS
  );

  outputsOff();

  Serial.println();
  Serial.println(
    "[준비 완료] MOSFET 자동 측정 시스템"
  );

  Serial.println(
    "v2 : VGS 2V 자동 측정"
  );

  Serial.println(
    "v1 : VGS 1V 자동 측정"
  );

  Serial.println(
    "i : 현재 드레인 전류 확인"
  );

  Serial.println(
    "g : 저장된 그래프 데이터 출력"
  );

  Serial.println(
    "x : DAC B/C 출력 중단"
  );
}


// ======================================================
// 38. 시리얼 명령
// ======================================================

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r' || c == '\n') {
      commandLine.trim();

      if (commandLine == "i") {
        float current =
          readShuntVoltage() /
          SHUNT_OHMS *
          1000.0f;

        Serial.print(
          "현재 드레인 전류(mA): "
        );

        Serial.println(current, 5);
      }

      else if (commandLine == "g") {
        printGraphData();
      }

      else if (commandLine == "x") {
        stopRequested = true;
        outputsOff();

        Serial.println(
          "[사용자 중단] DAC B/C 출력을 0V로 설정했습니다."
        );
      }

      else if (
        commandLine.length() > 1 &&
        (
          commandLine[0] == 'v' ||
          commandLine[0] == 'V'
        )
      ) {

        String value =
          commandLine.substring(1);

        value.trim();

        char *endptr = nullptr;

        float target = strtof(
          value.c_str(),
          &endptr
        );

        if (
          endptr == value.c_str() ||
          *endptr != '\0' ||
          !isfinite(target) ||
          target < 0.0f ||
          target > 3.0f
        ) {

          Serial.println(
            "[입력 오류] VGS는 0~3V입니다."
          );

        } else {
          runAll(target);
        }
      }

      else if (commandLine.length() > 0) {
        Serial.println(
          "[입력 오류] 알 수 없는 명령어입니다."
        );
      }

      commandLine = "";

    } else if (commandLine.length() < 40) {
      commandLine += c;
    }
  }
}
