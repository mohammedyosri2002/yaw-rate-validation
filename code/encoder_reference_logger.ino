#include <Arduino.h>

// ============================================================
// ENCODER REFERENCE LOGGER
// IBT-2 / BTS7960 + Quadrature Encoder
//
// Purpose: high-rate, timestamped reference logging for
//          camera-vs-encoder yaw / yaw-rate RMS comparison.
//
// Key idea: log raw (time_us, encoder_count) at high rate.
//           Compute the REFERENCE angular velocity OFFLINE from
//           those two columns. The on-board rpm_from_counts is a
//           smoothed convenience value for control/status only.
// ============================================================

// ---------------- PINS ----------------
const int ENCODER_A_PIN = 2;
const int ENCODER_B_PIN = 3;
const int RPWM_PIN      = 5;
const int LPWM_PIN      = 6;
const int LED_PIN       = 13;   // optional diagnostic sync marker; not used in reported analysis

// ---------------- ENCODER ----------------
const float COUNTS_PER_REV = 1496.0f;   // 4x decoding * gearing, as you measured
const int   MOTOR_SIGN     = 1;         // flip one of these if direction is wrong
const int   ENCODER_SIGN   = 1;

// ---------------- SERIAL ----------------
const unsigned long BAUD = 500000UL;    // high baud so logging never blocks the loop

// ---------------- TIMING (microsecond-based) ----------------
const unsigned long LOG_PERIOD_US     = 10000UL;  // 100 Hz logging (set 5000 for 200 Hz)
const unsigned long CONTROL_PERIOD_US = 20000UL;  // 50 Hz control loop

unsigned long lastLogUs     = 0;
unsigned long lastControlUs = 0;

// ---------------- QUADRATURE ----------------
volatile long    encoderCount = 0;
volatile uint8_t lastState    = 0;

// ---------------- VELOCITY RING BUFFER ----------------
// Live velocity is computed over a short window so it is smooth enough
// for the PID and the status line. It is NOT your final reference.
const uint8_t  RING_SIZE = 32;
const uint8_t  VEL_LAG   = 12;          // ~120 ms window at 100 Hz
unsigned long  ringTime[RING_SIZE];
long           ringCount[RING_SIZE];
uint8_t        ringHead   = 0;
uint16_t       ringFilled = 0;

// ---------------- STATE ----------------
long  signedCount     = 0;
float currentAngleDeg = 0.0f;
float currentRPM      = 0.0f;   // smoothed, from ring window (rpm_from_counts)
int   appliedPWM      = 0;

enum Mode { MODE_IDLE, MODE_RPM, MODE_PWM };
Mode  mode      = MODE_IDLE;
float targetRPM = 0.0f;
int   directPWM = 0;

bool  logging = true;

// sync LED (non-blocking flash)
unsigned long syncLedOffUs = 0;

// ---------------- RPM LIMITS ----------------
const float MIN_STABLE_RPM = 5.0f;
const float MAX_ALLOWED_RPM = 55.0f;

// ---------------- PID ----------------
float KP_RPM = 0.55f;
float KI_RPM = 0.10f;
float KD_RPM = 0.00f;
float rpmIntegral  = 0.0f;
float rpmPrevError = 0.0f;

// ---------------- PWM LIMITS ----------------
int PWM_MIN_LIMIT = 0;
int PWM_MAX_LIMIT = 80;

// ---------------- FEEDFORWARD MAP (midpoints of your latest tests) ----------------
struct MapPoint { float rpm; float pwm; };
MapPoint ffMap[] = {
  { 5.5f,   7.0f},
  { 6.5f,   8.0f},
  { 7.95f,  9.0f},
  { 9.5f,  10.0f},
  {16.95f, 15.0f},
  {24.65f, 20.0f},
  {39.4f,  30.0f},
  {54.15f, 40.0f}
};
const int FF_MAP_SIZE = sizeof(ffMap) / sizeof(ffMap[0]);

// ============================================================
// QUADRATURE ISR  (your decoding)
// ============================================================
void updateEncoder() {
  uint8_t a = digitalRead(ENCODER_A_PIN);
  uint8_t b = digitalRead(ENCODER_B_PIN);
  uint8_t currentState = (a << 1) | b;
  uint8_t transition = (lastState << 2) | currentState;
  switch (transition) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCount++; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCount--; break;
    default: break;
  }
  lastState = currentState;
}

// ============================================================
// MOTOR DRIVE
// ============================================================
void setMotorPWM(int pwmRaw) {
  int pwm = MOTOR_SIGN * pwmRaw;
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0)      { analogWrite(RPWM_PIN, pwm); analogWrite(LPWM_PIN, 0);    }
  else if (pwm < 0) { analogWrite(RPWM_PIN, 0);   analogWrite(LPWM_PIN, -pwm); }
  else              { analogWrite(RPWM_PIN, 0);   analogWrite(LPWM_PIN, 0);    }
}

void resetRPMController() { rpmIntegral = 0.0f; rpmPrevError = 0.0f; }

// ============================================================
// SAMPLE + RING BUFFER
// ============================================================
void pushSample(unsigned long ts, long c) {
  ringHead = (ringHead + 1) % RING_SIZE;
  ringTime[ringHead]  = ts;
  ringCount[ringHead] = c;
  if (ringFilled < RING_SIZE) ringFilled++;
}

// smoothed velocity over the ring window, in deg/s
float velocityDegPerSec() {
  if (ringFilled <= VEL_LAG) return 0.0f;
  uint8_t iNew = ringHead;
  uint8_t iOld = (uint8_t)((ringHead + RING_SIZE - VEL_LAG) % RING_SIZE);
  unsigned long dtus = ringTime[iNew] - ringTime[iOld];   // unsigned -> wrap-safe
  if (dtus == 0) return 0.0f;
  long dc = ringCount[iNew] - ringCount[iOld];
  return ((float)dc / COUNTS_PER_REV * 360.0f) / ((float)dtus / 1000000.0f);
}

// ============================================================
// FEEDFORWARD + PI CONTROL
// ============================================================
float getFeedforwardPWM(float rpmTarget) {
  if (rpmTarget <= ffMap[0].rpm)               return ffMap[0].pwm;
  if (rpmTarget >= ffMap[FF_MAP_SIZE - 1].rpm) return ffMap[FF_MAP_SIZE - 1].pwm;
  for (int i = 0; i < FF_MAP_SIZE - 1; i++) {
    float r1 = ffMap[i].rpm, r2 = ffMap[i + 1].rpm;
    float p1 = ffMap[i].pwm, p2 = ffMap[i + 1].pwm;
    if (rpmTarget >= r1 && rpmTarget <= r2) {
      float t = (rpmTarget - r1) / (r2 - r1);
      return p1 + t * (p2 - p1);
    }
  }
  return ffMap[FF_MAP_SIZE - 1].pwm;
}

int computeRPMControl(float dt) {
  float ffPWM = getFeedforwardPWM(targetRPM);
  float error = targetRPM - currentRPM;
  rpmIntegral += error * dt;
  rpmIntegral = constrain(rpmIntegral, -30.0f, 30.0f);
  float derivative = (dt > 0) ? (error - rpmPrevError) / dt : 0.0f;
  rpmPrevError = error;
  float out = ffPWM + KP_RPM * error + KI_RPM * rpmIntegral + KD_RPM * derivative;
  out = constrain(out, (float)PWM_MIN_LIMIT, (float)PWM_MAX_LIMIT);
  return (int)round(out);
}

// ============================================================
// OUTPUT
// CSV data rows are bare. Every human/status message starts with '#'
// so it is a comment line that never corrupts the CSV.
// (pandas: pd.read_csv(file, comment='#'))
// ============================================================
void printHeader() {
  Serial.println(F("time_us,encoder_count,angle_deg,rpm_from_counts,applied_pwm,target_rpm"));
}

void printDataRow(unsigned long ts) {
  Serial.print(ts);                Serial.print(',');
  Serial.print(signedCount);       Serial.print(',');
  Serial.print(currentAngleDeg, 2);Serial.print(',');
  Serial.print(currentRPM, 2);     Serial.print(',');
  Serial.print(appliedPWM);        Serial.print(',');
  Serial.println(targetRPM, 2);
}

// ============================================================
// OPTIONAL SYNC DIAGNOSTIC: not used by the reported onset-based analysis
// ============================================================
void doSync() {
  unsigned long ts = micros();
  digitalWrite(LED_PIN, HIGH);
  syncLedOffUs = ts + 150000UL;   // 150 ms flash, turned off non-blocking in loop
  Serial.print(F("# SYNC time_us=")); Serial.println(ts);
}

// ============================================================
// COMMANDS
// ============================================================
String cmd = "";

void processCommand(String s) {
  s.trim(); s.toLowerCase();
  if (s.length() == 0) return;

  if (s == "stop") {
    mode = MODE_IDLE; targetRPM = 0; directPWM = 0; appliedPWM = 0;
    resetRPMController(); setMotorPWM(0);
    Serial.println(F("# STOPPED")); return;
  }
  if (s == "zero") {
    noInterrupts(); encoderCount = 0; interrupts();
    signedCount = 0; currentAngleDeg = 0; currentRPM = 0;
    ringHead = 0; ringFilled = 0;
    Serial.println(F("# ZERO DONE")); return;
  }
  if (s == "status") {
    Serial.print(F("# STATUS mode="));
    Serial.print(mode == MODE_RPM ? F("RPM") : (mode == MODE_PWM ? F("PWM") : F("IDLE")));
    Serial.print(F(" target_rpm=")); Serial.print(targetRPM, 2);
    Serial.print(F(" meas_rpm="));   Serial.print(currentRPM, 2);
    Serial.print(F(" pwm="));        Serial.print(appliedPWM);
    Serial.print(F(" angle_deg="));  Serial.print(currentAngleDeg, 2);
    Serial.print(F(" count="));      Serial.println(signedCount);
    return;
  }
  if (s == "header")  { printHeader(); return; }
  if (s == "sync")    { doSync(); return; }
  if (s == "log on")  { logging = true;  Serial.println(F("# LOG ON"));  return; }
  if (s == "log off") { logging = false; Serial.println(F("# LOG OFF")); return; }

  if (s.startsWith("rpm ")) {
    float req = s.substring(4).toFloat();
    if (req < MIN_STABLE_RPM)  { Serial.print(F("# RPM too low, min "));  Serial.println(MIN_STABLE_RPM, 1);  return; }
    if (req > MAX_ALLOWED_RPM) { Serial.print(F("# RPM too high, max ")); Serial.println(MAX_ALLOWED_RPM, 1); return; }
    targetRPM = req; mode = MODE_RPM; resetRPMController();
    Serial.print(F("# NEW TARGET RPM = ")); Serial.println(targetRPM, 2); return;
  }
  if (s.startsWith("pwm ")) {
    int p = s.substring(4).toInt();
    p = constrain(p, -PWM_MAX_LIMIT, PWM_MAX_LIMIT);
    directPWM = p; mode = MODE_PWM; targetRPM = 0;
    Serial.print(F("# DIRECT PWM = ")); Serial.println(directPWM); return;
  }
  if (s.startsWith("kp ")) { KP_RPM = s.substring(3).toFloat(); Serial.print(F("# KP=")); Serial.println(KP_RPM, 4); return; }
  if (s.startsWith("ki ")) { KI_RPM = s.substring(3).toFloat(); Serial.print(F("# KI=")); Serial.println(KI_RPM, 4); return; }
  if (s.startsWith("kd ")) { KD_RPM = s.substring(3).toFloat(); Serial.print(F("# KD=")); Serial.println(KD_RPM, 4); return; }
  if (s.startsWith("pwmmax ")) { PWM_MAX_LIMIT = s.substring(7).toInt(); Serial.print(F("# PWM_MAX=")); Serial.println(PWM_MAX_LIMIT); return; }

  Serial.println(F("# UNKNOWN COMMAND"));
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(BAUD);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  setMotorPWM(0);

  delay(100);
  uint8_t a = digitalRead(ENCODER_A_PIN);
  uint8_t b = digitalRead(ENCODER_B_PIN);
  lastState = (a << 1) | b;

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), updateEncoder, CHANGE);

  unsigned long now = micros();
  lastLogUs = now; lastControlUs = now;

  Serial.println(F("# READY - ENCODER REFERENCE LOGGER"));
  Serial.println(F("# commands: rpm <5..55> | pwm <-80..80> | stop | zero | status | sync | header | log on | log off | kp/ki/kd <v> | pwmmax <v>"));
  printHeader();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // ---- serial command intake ----
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { processCommand(cmd); cmd = ""; }
    else                        { cmd += c; }
  }

  unsigned long now = micros();

  // ---- non-blocking sync LED off ----
  if (syncLedOffUs && (long)(now - syncLedOffUs) >= 0) {
    digitalWrite(LED_PIN, LOW);
    syncLedOffUs = 0;
  }

  // ---- high-rate sample + log ----
  if ((unsigned long)(now - lastLogUs) >= LOG_PERIOD_US) {
    // fixed cadence with catch-up guard (avoids runaway if ever delayed)
    if ((unsigned long)(now - lastLogUs) > 4UL * LOG_PERIOD_US) lastLogUs = now;
    else                                                        lastLogUs += LOG_PERIOD_US;

    // atomic count read, then timestamp as close as possible to the sample
    noInterrupts(); long rawCount = encoderCount; interrupts();
    unsigned long ts = micros();

    signedCount     = ENCODER_SIGN * rawCount;
    pushSample(ts, signedCount);
    currentAngleDeg = ((float)signedCount / COUNTS_PER_REV) * 360.0f;
    currentRPM      = velocityDegPerSec() / 6.0f;   // deg/s -> rpm

    if (logging) printDataRow(ts);
  }

  // ---- control loop ----
  if ((unsigned long)(now - lastControlUs) >= CONTROL_PERIOD_US) {
    float dt = (now - lastControlUs) / 1000000.0f;
    if ((unsigned long)(now - lastControlUs) > 4UL * CONTROL_PERIOD_US) lastControlUs = now;
    else                                                                lastControlUs += CONTROL_PERIOD_US;

    if (mode == MODE_RPM)      { appliedPWM = computeRPMControl(dt); setMotorPWM(appliedPWM); }
    else if (mode == MODE_PWM) { appliedPWM = directPWM;            setMotorPWM(appliedPWM); }
    else                       { appliedPWM = 0;                    setMotorPWM(0); }
  }
}
