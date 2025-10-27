// --- Resistor Autorange with Standby (one-time print), Debounce, Hysteresis, Vcc auto (Mega 2560) ---

// Wiring (per range):
// Dx -- 100Ω -- R1x --+
//                      +---- Node ---- A0
//                  R_unknown
//                      |
//                     GND

// ====== USER SETTINGS ======
const uint8_t AIN = A0;
const uint8_t RANGE_PINS[4] = {2, 3, 4, 5};  // D2..D5 control ranges

// Calibrated high-side resistors (ohms):
const float R_KNOWN[4] = { 288.6f, 4756.0f, 69300.0f, 217500.0f };
const float R_SERIES = 100.0f;               // 100Ω per-pin series

// Standby / wake behaviour
const float   R_WAKE_OHMS = 700000.0f;       // wake when R_unknown <= this
const int     ADC_HYST    = 12;              // hysteresis (ADC counts; ~1.2%)
const uint8_t WAKE_DEBOUNCE_COUNT  = 3;      // consecutive hits to enter ACTIVE
const uint8_t SLEEP_DEBOUNCE_COUNT = 3;      // consecutive hits to return STANDBY
const unsigned STANDBY_CHECK_MS = 200;       // standby poll interval
const unsigned MEASUREMENT_PERIOD_MS = 1000; // cadence when active
// ===========================

// ---------- Range control / ADC settle ----------
static inline void selectRange(uint8_t idx) {
  for (uint8_t i = 0; i < 4; i++) pinMode(RANGE_PINS[i], (i == idx) ? OUTPUT : INPUT);
  digitalWrite(RANGE_PINS[idx], HIGH);
}
static inline void allRangesOff() {
  for (uint8_t i = 0; i < 4; i++) pinMode(RANGE_PINS[i], INPUT);
}
static inline int readADCSettled() {
  delayMicroseconds(300);
  analogRead(AIN);                 // throwaway after switching
  return analogRead(AIN);          // real read
}

// ---------- Measure Vcc using 1.1V bandgap (Mega 2560) ----------
long measureVcc_mV() {
  uint8_t admux_save  = ADMUX;
  uint8_t adcsrb_save = ADCSRB;
  uint8_t adcsra_save = ADCSRA;

  // AVcc as reference
  ADMUX  = (ADMUX & 0b00111111) | _BV(REFS0);  // REFS1=0, REFS0=1
  // Bandgap on Mega2560 is channel 30 (MUX5..0 = 11110)
  ADCSRB |= _BV(MUX5);
  ADMUX   = (ADMUX & 0b11100000) | 0b00011110;

  delayMicroseconds(300);
  ADCSRA |= _BV(ADSC); while (ADCSRA & _BV(ADSC)) {} // throwaway
  ADCSRA |= _BV(ADSC); while (ADCSRA & _BV(ADSC)) {} // real
  uint16_t adc = ADC;

  ADMUX  = admux_save;
  ADCSRB = adcsrb_save;
  ADCSRA = adcsra_save;

  if (adc == 0) return -1;
  return (1100L * 1023L) / adc;   // mV
}

// ---------- Autorange measurement (median-of-3 per range) ----------
float measureAutorange(uint8_t *bestIdxOut=nullptr, int *bestAdcOut=nullptr, float *vrefOut=nullptr) {
  long vcc_mV = measureVcc_mV();
  float Vref = (vcc_mV > 0) ? (vcc_mV / 1000.0f) : 5.00f; // fallback
  if (vrefOut) *vrefOut = Vref;

  int bestScore = 1e9;
  float bestR = -1.0f;
  uint8_t bestIdx = 255;
  int bestAdc = -1;

  for (uint8_t i = 0; i < 4; i++) {
    selectRange(i);
    int a = readADCSettled(), b = readADCSettled(), c = readADCSettled();
    int adc = (a > b) ? ((b > c) ? b : (a > c ? c : a))
                      : ((a > c) ? a : (b > c ? c : b)); // median-of-3

    if (adc > 1 && adc < 1022) {
      float vout = (adc / 1023.0f) * Vref;
      float R1eff = R_KNOWN[i] + R_SERIES;
      float runk  = R1eff * (vout / (Vref - vout));
      int score   = abs(adc - 512);  // prefer mid-scale

      if (score < bestScore) { bestScore = score; bestR = runk; bestIdx = i; bestAdc = adc; }
    }
  }

  allRangesOff();
  if (bestIdxOut) *bestIdxOut = bestIdx;
  if (bestAdcOut) *bestAdcOut = bestAdc;
  return bestR; // -1 if no valid range
}

// ---------- Top-range read & thresholds ----------
void computeAdcThresholds(int &adcWake, int &adcSleep) {
  const float R1eff = R_KNOWN[3] + R_SERIES;                   // ~219.2k
  const float frac  = R_WAKE_OHMS / (R1eff + R_WAKE_OHMS);     // ~0.762 @ 700k
  const int   adcT  = (int)(frac * 1023.0f + 0.5f);            // ~780
  adcWake  = adcT - ADC_HYST;  // go ACTIVE when adc < adcWake (lower)
  adcSleep = adcT + ADC_HYST;  // go STANDBY when adc > adcSleep (upper)
}

int readTopRangeADC_Avg6() {
  selectRange(3); // top range
  long acc = 0; for (uint8_t i=0; i<6; i++) acc += readADCSettled();
  allRangesOff();
  return (int)(acc / 6);
}

// ---------- State machine ----------
enum Mode : uint8_t { STANDBY, ACTIVE };

void setup() {
  Serial.begin(9600);
  analogReference(DEFAULT);   // use Vcc as ADC reference
}

void loop() {
  static Mode mode = STANDBY;
  static bool printedStandby = false;
  static uint8_t wakeHits = 0, sleepHits = 0;

  int adcWake, adcSleep;
  computeAdcThresholds(adcWake, adcSleep);

  if (mode == STANDBY) {
    // One-time "Standby..." on entry
    if (!printedStandby) { Serial.println("Standby..."); printedStandby = true; }

    int adc = readTopRangeADC_Avg6();

    if (adc < adcWake) {
      if (++wakeHits >= WAKE_DEBOUNCE_COUNT) {
        mode = ACTIVE;
        wakeHits = 0;
        sleepHits = 0;
        printedStandby = false;  // allow print next time we *re-enter* standby
      }
    } else {
      wakeHits = 0; // reset debounce if condition breaks
    }

    delay(STANDBY_CHECK_MS);
    return;
  }

  // ---- ACTIVE ----
  uint8_t idx; int adcChosen; float vref;
  float R = measureAutorange(&idx, &adcChosen, &vref);

  if (R > 0 && idx != 255) {
    Serial.print("Resistance: "); Serial.print(R, 1); Serial.print(" ohms  |  ");
    Serial.print("Range R1eff="); Serial.print(R_KNOWN[idx] + R_SERIES, 1); Serial.print("Ω  |  ");
    Serial.print("ADC="); Serial.print(adcChosen); Serial.print("  |  Vref="); Serial.print(vref, 3); Serial.println(" V");
  } else {
    Serial.println("Out of range");
  }

  // Decide if we should return to STANDBY (debounced on the *upper* threshold)
  int adcTop = readTopRangeADC_Avg6();
  if (adcTop > adcSleep) {
    if (++sleepHits >= SLEEP_DEBOUNCE_COUNT) {
      mode = STANDBY;
      sleepHits = 0;
      // "Standby..." will print once on next loop iteration
    }
  } else {
    sleepHits = 0;
  }

  delay(MEASUREMENT_PERIOD_MS);
}

