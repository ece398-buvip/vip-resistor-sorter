#include <Servo.h>

// ===================== PIN CONFIGURATION =====================
// Change these *only here* if wiring changes later.
const int X_DIR_PIN  = 47;
const int X_STEP_PIN = 3;

const int Y_DIR_PIN  = 46;
const int Y_STEP_PIN = 2;

// ===== FEED STEPPER (step/dir driver) =====
const int FEED_STEP_PIN        = 4;    // PULSE+
const int FEED_DIR_PIN         = 51;   // DIR+
const unsigned int FEED_STEP_DELAY_US = 800;  // step speed (tune if needed)


// Limit switches (active LOW: pressed = LOW)
const int XLIMIT_SWITCH_PINLEFT = 51;
const int YLIMIT_SWITCH_TOP     = 53;
const int YLIMIT_SWITCH_BOT     = 52;

// Prox sensors
const int FEEDER_PROX_PIN = A2;
const int GANTRY_PROX_PIN = A3;

// Gantry servos
const int SLIDE_SERVO_PIN = 6;   // Continuous (FS5103R)
const int FORKS_SERVO_PIN = 5;  // SG90

// Slide (continuous) servo calibration (microseconds)
const int SLIDE_STOP_US = 1468;   // <-- we'll tune this
const int SLIDE_FWD_US  = SLIDE_STOP_US + 500;  // was ~2000
const int SLIDE_REV_US  = SLIDE_STOP_US - 500;  // was ~1000

const int FORKS_UP_ANGLE = 0; 
const int FORKS_DOWN_ANGLE = 100; 

// Magnet relay
const int MAGNET_RELAY_PIN = 8;

// Homing behavior
const int X_HOME_BACKOFF_STEPS = 50;   // how many steps to move away from switch after hit
const int Y_HOME_BACKOFF_STEPS = 50;   // same idea for Y bottom

// Homing step delays (microseconds)
const unsigned int X_HOME_DELAY_US = 900; // tuned for smooth motion
const unsigned int Y_HOME_DELAY_US = 900;


// ===================== GLOBAL STATE =====================

Servo SlideServo;// ===== OHMMETER CONFIG (Mega 2560) =====
//  D   23–26 = PB4..PB7 = Arduino D10..D13
const uint8_t OHM_AIN = A1;  // analog input from resistor under test

const uint8_t OHM_RANGE_PINS[4] = {
  23, // PA1  -> 288.6 Ω range
  24, // PA2  -> 4.756 kΩ range
  25, // PA3  -> 69.3 kΩ range
  26  // PA4  -> 217.5 kΩ range
};

// Calibrated high-side resistors (ohms)
const float OHM_R_KNOWN[4] = {
  288.6f,
  4756.0f,
  69300.0f,
  217500.0f
};

const float OHM_R_SERIES = 100.0f;   // 100 Ω series in each branch

Servo ForksServo;

// Simple position/state tracking for safety / sanity
long currentXSteps = 0;   // relative position in steps (diagnostic only)
long currentYSteps = 0;   // relative position in steps (diagnostic only)

int forksAngleDeg   = 0;  // last written angle to forks servo
int slideMotionDir  = 0;  // -1 = backward, 0 = stopped, +1 = forward

// ===================== LOW-LEVEL HELPERS =====================

// X step profiles (same timing as your existing code)
void xstepping_seq_fast() {
  digitalWrite(X_STEP_PIN, HIGH);
  delayMicroseconds(800);
  digitalWrite(X_STEP_PIN, LOW);
  delayMicroseconds(800);
}

void xstepping_seq_medium() {
  digitalWrite(X_STEP_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(X_STEP_PIN, LOW);
  delayMicroseconds(1000);
}

void xstepping_seq_slow() {
  digitalWrite(X_STEP_PIN, HIGH);
  delayMicroseconds(2500);
  digitalWrite(X_STEP_PIN, LOW);
  delayMicroseconds(2500);
}

// Y step profiles (same timing as your existing code)
void ystepping_seq_fast() {
  digitalWrite(Y_STEP_PIN, HIGH);
  delayMicroseconds(800);
  digitalWrite(Y_STEP_PIN, LOW);
  delayMicroseconds(800);
}

void ystepping_seq_medium() {
  digitalWrite(Y_STEP_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(Y_STEP_PIN, LOW);
  delayMicroseconds(1000);
}

void ystepping_seq_slow() {
  digitalWrite(Y_STEP_PIN, HIGH);
  delayMicroseconds(2500);
  digitalWrite(Y_STEP_PIN, LOW);
  delayMicroseconds(2500);
}

// Check if ANY limit switch is currently pressed
bool anyLimitPressed() {
  bool xHit = (digitalRead(XLIMIT_SWITCH_PINLEFT) == LOW);
  bool yTop = (digitalRead(YLIMIT_SWITCH_TOP)     == LOW);
  bool yBot = (digitalRead(YLIMIT_SWITCH_BOT)     == LOW);
  return (xHit || yTop || yBot);
}

// Immediately stop anything that can move
void stopAllMotion() {
  // Stop steppers (just ensure step lines low; motion is blocking anyway)
  digitalWrite(X_STEP_PIN, LOW);
  digitalWrite(Y_STEP_PIN, LOW);

  // Stop slide servo and keep forks at last commanded angle
  SlideServo.writeMicroseconds(SLIDE_STOP_US);; // neutral/stop for continuous
  slideMotionDir = 0;

  // Do *not* change magnet here — "movement" only.
  Serial.println(F("[SAFETY] Motion stopped due to limit switch."));
}

// Safer wrapper to call whenever we’re in a motion loop
bool safetyCheck() {
  if (anyLimitPressed()) {
    stopAllMotion();
    return true;  // something is pressed
  }
  return false;   // all clear
}

// ===================== STEPPER HELPERS =====================

// X: positive steps => direction HIGH, negative => LOW (matches your unified logic)
void moveXSteps(long steps) {
  if (steps == 0) return;

  // +steps = move RIGHT, -steps = move LEFT
  bool dirRight = (steps > 0);
  digitalWrite(X_DIR_PIN, dirRight ? HIGH : LOW);

  Serial.print(F("[X] Moving "));
  Serial.print(steps);
  Serial.println(F(" steps."));

  for (long i = 0; i < labs(steps); i++) {
    // Only block motion when trying to move *into* the left limit.
    if (!dirRight && digitalRead(XLIMIT_SWITCH_PINLEFT) == LOW) {
      Serial.println(F("[X] Left limit pressed – cannot move further LEFT."));
      stopAllMotion();
      return;
    }

    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(800);
  }

  currentXSteps += steps;
}


// Y: direction HIGH = UP, LOW = DOWN (matches your fixed logic)
void moveYSteps(long steps) {
  if (steps == 0) return;

  // +steps = move UP, -steps = move DOWN
  bool dirUp = (steps > 0);
  digitalWrite(Y_DIR_PIN, dirUp ? HIGH : LOW);

  Serial.print(F("[Y] Moving "));
  Serial.print(steps);
  Serial.println(F(" steps."));

  for (long i = 0; i < labs(steps); i++) {
    // Only block motion when trying to move *into* the corresponding limit.
    if (dirUp && digitalRead(YLIMIT_SWITCH_TOP) == LOW) {
      Serial.println(F("[Y] Top limit pressed – cannot move further UP."));
      stopAllMotion();
      return;
    }
    if (!dirUp && digitalRead(YLIMIT_SWITCH_BOT) == LOW) {
      Serial.println(F("[Y] Bottom limit pressed – cannot move further DOWN."));
      stopAllMotion();
      return;
    }

    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(600);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(800);
  }

  currentYSteps += steps;
}
void moveFeedSteps(long steps) {
  if (steps == 0) return;

  bool forward = (steps > 0);          // convention: + = forward, - = backward
  digitalWrite(FEED_DIR_PIN, forward ? HIGH : LOW);

  Serial.print(F("[FEED] Moving "));
  Serial.print(steps);
  Serial.println(F(" steps."));

  for (long i = 0; i < labs(steps); i++) {
    // Optional: stop when feeder prox triggers while moving forward
   // if (forward && digitalRead(FEEDER_PROX_PIN) == LOW) {
  //   Serial.println(F("[FEED] FEEDER_PROX triggered, stopping early."));
  //  break;
   // }

    digitalWrite(FEED_STEP_PIN, HIGH);
    delayMicroseconds(FEED_STEP_DELAY_US);
    digitalWrite(FEED_STEP_PIN, LOW);
    delayMicroseconds(FEED_STEP_DELAY_US);
  }
}

void homeAxesSimple() {
  Serial.println(F("[HOME] Homing X to left, Y to bottom."));

  // -------- X HOME: move LEFT until switch, then back off --------
  digitalWrite(X_DIR_PIN, LOW);  // LOW = left
  Serial.println(F("[HOME] Homing X toward left limit..."));

  while (digitalRead(XLIMIT_SWITCH_PINLEFT) == HIGH) {
    // step left slowly for gentle contact
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(X_HOME_DELAY_US);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(X_HOME_DELAY_US);
  }
  Serial.println(F("[HOME] X limit switch hit. Backing off..."));

  // Back off a bit so we're off the switch
  digitalWrite(X_DIR_PIN, HIGH);  // RIGHT
  for (int i = 0; i < X_HOME_BACKOFF_STEPS; i++) {
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(X_HOME_DELAY_US);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(X_HOME_DELAY_US);
  }

  // Now treat this backed-off position as X = 0
  currentXSteps = 0;
  Serial.println(F("[HOME] X homed. Position set to X = 0."));

  // -------- Y HOME: move DOWN until bottom switch, then back off --------
  digitalWrite(Y_DIR_PIN, LOW);  // LOW = down
  Serial.println(F("[HOME] Homing Y toward bottom limit..."));

  while (digitalRead(YLIMIT_SWITCH_BOT) == HIGH) {
    // step down slowly
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(Y_HOME_DELAY_US);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(Y_HOME_DELAY_US);
  }
  Serial.println(F("[HOME] Y bottom limit hit. Backing off..."));

  // Back off a bit *up* so we're off the switch
  digitalWrite(Y_DIR_PIN, HIGH);  // HIGH = up
  for (int i = 0; i < Y_HOME_BACKOFF_STEPS; i++) {
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(Y_HOME_DELAY_US);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(Y_HOME_DELAY_US);
  }

  currentYSteps = 0;
  Serial.println(F("[HOME] Y homed. Position set to Y = 0."));

  // -------- Move to 500,500 offset --------
  Serial.println(F("[HOME] Moving to offset (500, 500)..."));
  moveXSteps(500);  // uses your direction-aware safe move
  moveYSteps(500);
  Serial.println(F("[HOME] Homing complete. Now at (500, 500)."));
}

// ---------- Ohmmeter helpers ----------
static inline void ohmSelectRange(uint8_t idx) {
  // Enable only the selected range pin, others high-Z
  for (uint8_t i = 0; i < 4; i++) {
    if (i == idx) {
      pinMode(OHM_RANGE_PINS[i], OUTPUT);
      digitalWrite(OHM_RANGE_PINS[i], HIGH);
    } else {
      pinMode(OHM_RANGE_PINS[i], INPUT);
    }
  }
}

static inline void ohmAllRangesOff() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(OHM_RANGE_PINS[i], INPUT);
  }
}

static inline int ohmReadADCSettled() {
  delayMicroseconds(300);
  analogRead(OHM_AIN);      // throwaway after switching range
  return analogRead(OHM_AIN);
}

// ===================== SERVO / MAGNET HELPERS =====================

void setForksAngle(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  ForksServo.write(angle);
  forksAngleDeg = angle;
  delay(250);  // small settle time
}

void setForksUp() {
  Serial.print(F("[FORKS] Up ("));
  Serial.print(FORKS_UP_ANGLE);
  Serial.println(F(" deg)."));
  setForksAngle(FORKS_UP_ANGLE);
}


void setForksDown() {
  Serial.print(F("[FORKS] Down ("));
  Serial.print(FORKS_DOWN_ANGLE);
  Serial.println(F(" deg)."));
  setForksAngle(FORKS_DOWN_ANGLE);
}

// dir = +1 forward, -1 backward, 0 stop
void slideSetDir(int dir) {
  if (dir > 0) {
    SlideServo.writeMicroseconds(SLIDE_FWD_US); // forward full speed
  } else if (dir < 0) {
    SlideServo.writeMicroseconds(SLIDE_REV_US); // backward full speed
  } else {
    SlideServo.writeMicroseconds(SLIDE_STOP_US); // stop
  }
  slideMotionDir = dir;
}

// Run slide servo in a direction for a fixed time, with limit-checking
void slideRunTimed(int dir, unsigned long ms) {
  if (ms == 0) return;
  Serial.print(F("[SLIDE] dir="));
  Serial.print(dir > 0 ? F("forward") : F("backward"));
  Serial.print(F(", ms="));
  Serial.println(ms);

  unsigned long start = millis();
  slideSetDir(dir);

  while (millis() - start < ms) {
    // Stop if any limit is pressed
    if (safetyCheck()) break;

    // Additionally, for forward motion we can stop when GANTRY_PROX triggers
    if (dir > 0 && digitalRead(GANTRY_PROX_PIN) == LOW) {
      Serial.println(F("[SLIDE] GANTRY_PROX triggered (forward). Stopping."));
      break;
    }
    // For backward motion you could add a similar check if you had a rear sensor.

    delay(5); // don’t spin too tight
  }

  slideSetDir(0); // stop
}

// Magnet helpers
void magnetOn() {
  digitalWrite(MAGNET_RELAY_PIN, HIGH);
  Serial.println(F("[MAGNET] ON"));
}

void magnetOff() {
  digitalWrite(MAGNET_RELAY_PIN, LOW);
  Serial.println(F("[MAGNET] OFF"));
}
long ohmMeasureVcc_mV() {
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
float ohmMeasureSingleRange(uint8_t idx, float *vrefOut, int *adcOut) {
  long vcc_mV = ohmMeasureVcc_mV();
  float Vref = (vcc_mV > 0) ? (vcc_mV / 1000.0f) : 5.00f;
  if (vrefOut) *vrefOut = Vref;

  ohmSelectRange(idx);
  int a = ohmReadADCSettled();
  int b = ohmReadADCSettled();
  int c = ohmReadADCSettled();
  int adc = (a > b) ? ((b > c) ? b : (a > c ? c : a))
                    : ((a > c) ? a : (b > c ? c : a));

  if (adcOut) *adcOut = adc;

  // treat rails as invalid
  if (adc <= 1 || adc >= 1022) {
    ohmAllRangesOff();
    return -1.0f;  // invalid for that range
  }

  float vout  = (adc / 1023.0f) * Vref;
  float R1eff = OHM_R_KNOWN[idx] + OHM_R_SERIES;
  float runk  = R1eff * (vout / (Vref - vout));

  ohmAllRangesOff();
  return runk;
}

float ohmMeasureAutorange(uint8_t *bestIdxOut = nullptr,
                          int *bestAdcOut = nullptr,
                          float *vrefOut = nullptr) {
  float Rvals[4];
  int   Adc[4];
  bool  valid[4];
  int   count = 0;

  // Measure all ranges using the same logic as OHMALL
  for (uint8_t i = 0; i < 4; i++) {
    float dummyVref;
    int adc;
    float R = ohmMeasureSingleRange(i, &dummyVref, &adc);

    Rvals[i] = R;
    Adc[i]   = adc;
    valid[i] = (R > 0);   // we already rejected rails inside singleRange()

    if (valid[i]) {
      count++;
      if (vrefOut) *vrefOut = dummyVref;  // last valid Vref is fine
    }
  }

  // No valid range at all -> out of range
  if (count == 0) {
    if (bestIdxOut) *bestIdxOut = 255;
    if (bestAdcOut) *bestAdcOut = -1;
    return -1.0f;
  }

  // Compute average of all *valid* estimates
  float sumR = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    if (valid[i]) {
      sumR += Rvals[i];
    }
  }
  float avgR = sumR / (float)count;

  // Select the range whose estimate is closest to the average
  // (this naturally prefers the "cluster" of sane values)
  uint8_t bestIdx = 255;
  float bestErr   = 1e30f;
  int bestAdc     = -1;
  float bestR     = -1.0f;

  for (uint8_t i = 0; i < 4; i++) {
    if (!valid[i]) continue;

    float err = fabs(Rvals[i] - avgR);
    if (err < bestErr) {
      bestErr   = err;
      bestIdx   = i;
      bestAdc   = Adc[i];
      bestR     = Rvals[i];
    }
  }

  if (bestIdxOut) *bestIdxOut = bestIdx;
  if (bestAdcOut) *bestAdcOut = bestAdc;

  return bestR;
}





// ===================== SENSOR / STATUS READOUT =====================

void printSensors() {
  int xLim  = digitalRead(XLIMIT_SWITCH_PINLEFT);
  int yTop  = digitalRead(YLIMIT_SWITCH_TOP);
  int yBot  = digitalRead(YLIMIT_SWITCH_BOT);
  int feedP = digitalRead(FEEDER_PROX_PIN);
  int ganP  = digitalRead(GANTRY_PROX_PIN);
  int mag   = digitalRead(MAGNET_RELAY_PIN);

  Serial.println(F("===== SENSOR STATUS ====="));
  Serial.print(F("X Left Limit        : ")); Serial.println(xLim  == LOW ? F("PRESSED") : F("released"));
  Serial.print(F("Y Top Limit         : ")); Serial.println(yTop  == LOW ? F("PRESSED") : F("released"));
  Serial.print(F("Y Bottom Limit      : ")); Serial.println(yBot  == LOW ? F("PRESSED") : F("released"));
  Serial.print(F("Feeder Prox (A0)    : ")); Serial.println(feedP == LOW ? F("ACTIVE")  : F("inactive"));
  Serial.print(F("Gantry Prox (A1)    : ")); Serial.println(ganP  == LOW ? F("ACTIVE")  : F("inactive"));
  Serial.print(F("Magnet Relay (A4)   : ")); Serial.println(mag   == HIGH ? F("ON")     : F("OFF"));
  Serial.println(F("========================="));
}

void printStatus() {
  Serial.println(F("===== MOTION STATUS ====="));
  Serial.print(F("X steps (relative)  : ")); Serial.println(currentXSteps);
  Serial.print(F("Y steps (relative)  : ")); Serial.println(currentYSteps);
  Serial.print(F("Forks angle (deg)   : ")); Serial.println(forksAngleDeg);
  Serial.print(F("Slide motion state  : "));
  if (slideMotionDir > 0)      Serial.println(F("forward"));
  else if (slideMotionDir < 0) Serial.println(F("backward"));
  else                         Serial.println(F("stopped"));
  Serial.println(F("========================="));
}

// ===================== SERIAL COMMAND HANDLING =====================

void printMenu() {
  Serial.println();
  Serial.println(F("===== RESISTOR SORTER DIAGNOSTIC MENU ====="));
  Serial.println(F("Type commands followed by <Enter>. Examples in [ ]."));
  Serial.println(F("General:"));
  Serial.println(F("  HELP or ?           - Show this menu"));
  Serial.println(F("  SENS or S           - Print sensor states"));
  Serial.println(F("  STAT                - Print motion/servo status"));
  Serial.println();
  Serial.println(F("Steppers (X/Y):"));
  Serial.println(F("  HOME or H           - Home X (left) and Y (down) to their limit switches"));
  Serial.println(F("  X <steps>           - Move X by <steps> (positive = right, negative = left)  [X 500]"));
  Serial.println(F("  Y <steps>           - Move Y by <steps> (positive = up,   negative = down)  [Y -400]"));
  Serial.println(F("Feeder stepper:"));
  Serial.println(F("  F <steps>          - Move feeder (+ forward, - backward)  [F 200]"));
  Serial.println();
  Serial.println(F("Gantry Servos & Magnet:"));
  Serial.println(F("  FU                  - Forks UP  (0 deg)"));
  Serial.println(F("  FD                  - Forks DOWN (100 deg)"));
  Serial.println(F("  SF <ms>             - Slide FORWARD for <ms> milliseconds   [SF 800]"));
  Serial.println(F("  SB <ms>             - Slide BACKWARD for <ms> milliseconds  [SB 800]"));
  Serial.println(F("  M ON                - Magnet ON"));
  Serial.println(F("  M OFF               - Magnet OFF"));
  Serial.println();
  Serial.println(F("Ohmmeter:"));
  Serial.println(F("  OHM                - Measure resistor on ohmmeter input (A1)"));
  Serial.println(F("  OHMDBG             - Debug: show ADC on each range"));
  Serial.println(F("  OHMALL             - Show candidate R for each ohmmeter range"));
  Serial.println(F("Safety:"));
  Serial.println(F("  If ANY limit switch is pressed, ALL motion stops immediately."));
  Serial.println(F("=============================================="));
  Serial.println();
}

// Parse and execute a single command line
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Uppercase for simpler parsing
  line.toUpperCase();

  // Simple full-word checks first
  if (line == "HELP" || line == "?") {
    printMenu();
    return;
  }
  if (line == "SENS" || line == "S") {
    printSensors();
    return;
  }
  if (line == "STAT") {
    printStatus();
    return;
  }
  if (line == "HOME" || line == "H") {
    homeAxesSimple();
    return;
  }

  // Multi-letter/argument commands
  // Format: <CMD> [argument]
  // e.g., "X 500", "SF 800", "M ON"
  int firstSpace = line.indexOf(' ');
  String cmd = (firstSpace == -1) ? line : line.substring(0, firstSpace);
  String arg = (firstSpace == -1) ? ""   : line.substring(firstSpace + 1);
  arg.trim();

  // ---- Steppers ----
  if (cmd == "X") {
    long steps = arg.toInt();
    moveXSteps(steps);
  }
  else if (cmd == "Y") {
    long steps = arg.toInt();
    moveYSteps(steps);
  }
  else if (cmd == "F") {
  long steps = arg.toInt();
  moveFeedSteps(steps);
  }
  // ---- Forks ----
  else if (cmd == "FU") {
    setForksUp();
  }
  else if (cmd == "FD") {
    setForksDown();
  }
  // ---- Slide ----
  else if (cmd == "SF") {
    unsigned long ms = (unsigned long)arg.toInt();
    slideRunTimed(+1, ms);
  }
  else if (cmd == "SB") {
    unsigned long ms = (unsigned long)arg.toInt();
    slideRunTimed(-1, ms);
  }
  // ---- Magnet ----
  else if (cmd == "M") {
    if (arg == "ON") {
      magnetOn();
    } else if (arg == "OFF") {
      magnetOff();
    } else {
      Serial.println(F("Usage: M ON  or  M OFF"));
    }
  }
  else if (cmd == "OHMDBG") {
  for (uint8_t i = 0; i < 4; i++) {
    ohmSelectRange(i);
    int adc = analogRead(OHM_AIN);
    Serial.print(F("[OHMDBG] Range "));
    Serial.print(i);
    Serial.print(F(" (pin "));
    Serial.print(OHM_RANGE_PINS[i]);
    Serial.print(F(") -> ADC="));
    Serial.println(adc);
    delay(200);
  }

  ohmAllRangesOff();
  } 

  else if (cmd == "OHMALL") {
  float Vref;
  int dummyAdc;
  ohmMeasureVcc_mV(); // just to be sure ADC is set up

  Serial.println(F("[OHMALL] Per-range readings:"));
  for (uint8_t i = 0; i < 4; i++) {
    int adc;
    float R = ohmMeasureSingleRange(i, &Vref, &adc);
    Serial.print(F("  Range "));
    Serial.print(i);
    Serial.print(F(" (pin "));
    Serial.print(OHM_RANGE_PINS[i]);
    Serial.print(F("): ADC="));
    Serial.print(adc);
    Serial.print(F("  R≈ "));
    if (R < 0) {
      Serial.println(F("invalid (near rail)"));
    } else {
      Serial.print(R, 1);
      Serial.println(F(" Ω"));
    }
  }
}

  // ---- Ohmmeter ----
  else if (cmd == "OHM") {
  uint8_t idx;
  int adc;
  float vref;
  float R = ohmMeasureAutorange(&idx, &adc, &vref);

  if (R < 0) {
    Serial.println(F("[OHM] Out of range or open circuit."));
  } else {
    Serial.print(F("[OHM] R ≈ "));
    Serial.print(R, 1);
    Serial.print(F(" Ω   (range idx="));
    Serial.print(idx);
    Serial.print(F(", ADC="));
    Serial.print(adc);
    Serial.print(F(", Vref≈"));
    Serial.print(vref, 3);
    Serial.println(F(" V)"));
  }
}


  else {
    Serial.print(F("Unknown command: "));
    Serial.println(line);
    Serial.println(F("Type HELP for a list of commands."));
  }
}

// ===================== SETUP / LOOP =====================

void setup() {
  Serial.begin(9600);

  // Stepper pins
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN,  OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN,  OUTPUT);
  pinMode(FEED_STEP_PIN, OUTPUT);
  pinMode(FEED_DIR_PIN,  OUTPUT);

  digitalWrite(FEED_STEP_PIN, LOW);
  digitalWrite(FEED_DIR_PIN,  LOW);


  // Limits
  pinMode(XLIMIT_SWITCH_PINLEFT, INPUT_PULLUP);
  pinMode(YLIMIT_SWITCH_TOP,     INPUT_PULLUP);
  pinMode(YLIMIT_SWITCH_BOT,     INPUT_PULLUP);

  // Prox sensors (assuming external pull-up or driver)
  pinMode(FEEDER_PROX_PIN, INPUT);
  pinMode(GANTRY_PROX_PIN, INPUT);

  // Magnet relay
  pinMode(MAGNET_RELAY_PIN, OUTPUT);
  magnetOff();

  // Servos
  SlideServo.attach(SLIDE_SERVO_PIN);
  ForksServo.attach(FORKS_SERVO_PIN);

  // Safe startup positions
  // Send calibrated neutral immediately on boot
  SlideServo.writeMicroseconds(SLIDE_STOP_US);
  slideMotionDir = 0;

  // tiny delay just to let it settle
  delay(200);

  Serial.println(F("Resistor Sorter – Diagnostic Firmware"));
  Serial.println(F("Serial @ 9600 baud. Type HELP and press Enter."));
  Serial.println();
  printMenu();
}

void loop() {
  // Read a full line from Serial Monitor and handle it
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  // You could optionally keep watching safety here too, but
  // all motion functions already do safety checks inside.
}
