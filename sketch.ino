// Fault State Machine with Structured Recovery (ESP32, Wokwi)
// States: NORMAL, DEGRADED, FAILSAFE, SHUTDOWN. Transitions come from ONE table.
// Faults are identified by ID and source (CELL / RELAY / COMM / ADC) and remembered.
// Recovery from FAILSAFE is a verification sequence, never a jump to NORMAL.
// No delay() anywhere. Every state transition is logged as one JSON line.
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <stdarg.h>

// ---------- Types (declared first so Arduino auto-prototypes can see them) ----------
enum State  : uint8_t { S_NORMAL, S_DEGRADED, S_FAILSAFE, S_SHUTDOWN, NUM_STATES };
enum Event  : uint8_t { EV_CLEAR, EV_MINOR, EV_MAJOR, EV_FATAL, EV_VERIFIED, EV_EXHAUSTED, EV_RESET, NUM_EVENTS };
enum Source : uint8_t { SRC_NONE, SRC_CELL, SRC_RELAY, SRC_COMM, SRC_ADC };
enum Sev    : uint8_t { SEV_NONE, SEV_DEG, SEV_FS, SEV_SD };
enum FaultId : uint8_t {
  F_NONE,
  F_C1_LOW, F_C2_LOW, F_C3_LOW,          // cell weak            -> DEGRADED
  F_C1_CRIT, F_C2_CRIT, F_C3_CRIT,       // cell under/over volt -> FAILSAFE
  F_IMBAL,                               // cell imbalance       -> DEGRADED
  F_RLY_OPEN, F_RLY_WELD,                // relay stuck open -> FAILSAFE, welded -> SHUTDOWN
  F_COMM_CRC, F_COMM_TMO,                // comm bad frames -> DEGRADED, timeout -> FAILSAFE
  F_ADC1_FRZ, F_ADC2_FRZ, F_ADC3_FRZ,    // frozen ADC channel   -> FAILSAFE
  NUM_FAULTS
};
enum Verify : uint8_t { V_WAIT_CLEAR, V_STABLE, V_RELAY_TEST, V_ADC_CHECK, V_COMM_CHECK };
enum Inject : uint8_t { INJ_NONE, INJ_ADC_FREEZE, INJ_RELAY_OPEN, INJ_RELAY_WELD, INJ_COMM_LOSS, INJ_COMM_NOISE, NUM_INJ };
struct FaultInfo { const char* name; uint8_t source; uint8_t sev; };

// ---------- Fault table: name, source, severity ----------
const FaultInfo FAULTS[NUM_FAULTS] = {
  {"NONE",     SRC_NONE,  SEV_NONE},
  {"C1_LOW",   SRC_CELL,  SEV_DEG}, {"C2_LOW",   SRC_CELL, SEV_DEG}, {"C3_LOW",   SRC_CELL, SEV_DEG},
  {"C1_CRIT",  SRC_CELL,  SEV_FS},  {"C2_CRIT",  SRC_CELL, SEV_FS},  {"C3_CRIT",  SRC_CELL, SEV_FS},
  {"IMBAL",    SRC_CELL,  SEV_DEG},
  {"RLY_OPEN", SRC_RELAY, SEV_FS},  {"RLY_WELD", SRC_RELAY, SEV_SD},
  {"COMM_CRC", SRC_COMM,  SEV_DEG}, {"COMM_TMO", SRC_COMM, SEV_FS},
  {"ADC1_FRZ", SRC_ADC,   SEV_FS},  {"ADC2_FRZ", SRC_ADC,  SEV_FS},  {"ADC3_FRZ", SRC_ADC,  SEV_FS}
};
const char* STATE_NAME[]  = {"NORMAL", "DEGRADED", "FAILSAFE", "SHUTDOWN"};
const char* SOURCE_NAME[] = {"NONE", "CELL", "RELAY", "COMM", "ADC"};
const char* STEP_NAME[]   = {"WAIT_CLEAR", "STABLE", "RELAY_TEST", "ADC_CHECK", "COMM_CHECK"};
const char* INJ_NAME[]    = {"NONE", "ADC_FREEZE", "RELAY_OPEN", "RELAY_WELD", "COMM_LOSS", "COMM_NOISE"};
const char* EVENT_REASON[] = {"stable, hold complete", "minor fault", "major fault", "fatal fault",
                              "verification passed", "recovery failed", "operator reset"};

// ---------- THE TRANSITION TABLE: next state = TRANS[current state][event] ----------
//                         CLEAR       MINOR       MAJOR       FATAL       VERIFIED    EXHAUSTED   RESET
const uint8_t TRANS[NUM_STATES][NUM_EVENTS] = {
  /* NORMAL   */ { S_NORMAL,   S_DEGRADED, S_FAILSAFE, S_SHUTDOWN, S_NORMAL,   S_NORMAL,   S_NORMAL   },
  /* DEGRADED */ { S_NORMAL,   S_DEGRADED, S_FAILSAFE, S_SHUTDOWN, S_DEGRADED, S_DEGRADED, S_DEGRADED },
  /* FAILSAFE */ { S_FAILSAFE, S_FAILSAFE, S_FAILSAFE, S_SHUTDOWN, S_DEGRADED, S_SHUTDOWN, S_FAILSAFE },
  /* SHUTDOWN */ { S_SHUTDOWN, S_SHUTDOWN, S_SHUTDOWN, S_SHUTDOWN, S_SHUTDOWN, S_SHUTDOWN, S_FAILSAFE }
};

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- Pins ----------
const uint8_t CELL_PIN[3] = {34, 35, 32};   // pots = 3 battery cells
#define RELAY_PIN 19                         // relay drive
#define RELAY_FB_PIN 33                      // relay feedback (COM-NO contact to 3V3)
#define RELAY_ACTIVE_LOW true                // flip if your relay module acts the other way
#define COMM_RX 16                           // Serial2 loopback: wire GPIO17 to GPIO16
#define COMM_TX 17
#define RED_LED 2
#define GREEN_LED 4
#define YELLOW_LED 5
#define BUZZER 18
#define BTN_RESET 25                         // leave SHUTDOWN
#define BTN_INJECT 26                        // cycle injected fault
#define BTN_REPORT 27                        // print the fault registry

// ---------- Tuning ----------
const uint32_t SAMPLE_MS = 50, LCD_MS = 250, STAT_MS = 5000;
const uint8_t  ASSERT_N = 3, CLEAR_N = 20;               // fault debounce: 150 ms set, 1 s clear
const uint8_t  FROZEN_N = 20;                            // identical readings in a row = frozen (1 s)
const uint32_t RELAY_SETTLE_MS = 150;                    // time allowed for feedback to follow
const uint32_t COMM_PERIOD_MS = 100, COMM_TIMEOUT_MS = 500;
const uint32_t DEGRADED_HOLD_MS = 5000;                  // clean time before DEGRADED -> NORMAL
const uint32_t VERIFY_STABLE_MS = 3000, RELAY_TEST_MS = 400, ADC_CHECK_MS = 1000, COMM_CHECK_MS = 2000;
const uint8_t  COMM_CHECK_GOOD = 8;
const uint8_t  MAX_VERIFY_ATTEMPTS = 3;
const uint32_t FAILSAFE_TIMEOUT_MS = 45000;              // FAILSAFE can never last longer
const float LOW_TRIP = 3.30, LOW_CLR = 3.40;
const float CRIT_LO = 3.00, CRIT_LO_CLR = 3.05, CRIT_HI = 4.25, CRIT_HI_CLR = 4.20;
const float IMBAL_TRIP = 0.25, IMBAL_CLR = 0.18;
const uint8_t FROZEN_CH = 1;                             // ADC_FREEZE injection freezes cell 2

// ---------- Global data ----------
uint8_t state = S_NORMAL, vstep = V_WAIT_CLEAR, inject = INJ_NONE;
uint8_t verifyAttempts = 0, limitPct = 100, lastFaultId = F_NONE, relayPhase = 0;
uint32_t stateSince = 0, vT = 0, failsafeSince = 0, seqNo = 0, transitions = 0;
const char* exhaustReason = "";
bool relayCmd = true, resetRequested = false, cleanOn = false;
uint32_t relayCmdSince = 0, cleanSince = 0;
float volts[3] = {3.6, 3.6, 3.6};
uint16_t heldRaw = 0, lastRaw[3] = {0, 0, 0}, adcMin[3], adcMax[3], rawNow[3] = {2048, 2048, 2048};
uint8_t same[3] = {0, 0, 0};

// Fault registry (kept for the whole run)
uint16_t activeMask = 0;
uint8_t  aCnt[NUM_FAULTS], cCnt[NUM_FAULTS];
uint16_t occur[NUM_FAULTS];
uint32_t firstMs[NUM_FAULTS], lastMs[NUM_FAULTS];

// Communication
uint8_t txSeq = 0, rxSeq = 0, rxBuf[4], rxIdx = 0, crcScore = 0, goodStreak = 0;
bool haveSeq = false;
uint32_t lastGoodMs = 0, crcErrors = 0, framesOk = 0;
uint32_t maxLoopUs = 0, stalls = 0;

// ---------- Structured log: one JSON object per line ----------
void logj(const char* ev, const char* fmt, ...) {
  char body[170];
  va_list ap; va_start(ap, fmt); vsnprintf(body, sizeof body, fmt, ap); va_end(ap);
  Serial.printf("{\"t_ms\":%lu,\"ev\":\"%s\",%s}\n", (unsigned long)millis(), ev, body);
}

// ---------- Relay ----------
void setRelayCmd(bool closed) {
  if (closed == relayCmd) return;
  relayCmd = closed; relayCmdSince = millis();
  digitalWrite(RELAY_PIN, (closed != RELAY_ACTIVE_LOW) ? HIGH : LOW);
  logj("RELAY", "\"cmd\":\"%s\"", closed ? "CLOSE" : "OPEN");
}
bool readFeedback() {
  if (inject == INJ_RELAY_OPEN) return false;      // injected: contact stuck open
  if (inject == INJ_RELAY_WELD) return true;       // injected: contact welded shut
  return digitalRead(RELAY_FB_PIN) == HIGH;
}

// ---------- Fault registry ----------
bool isActive(uint8_t id) { return (activeMask >> id) & 1; }

uint8_t worstSeverity() {
  uint8_t w = SEV_NONE;
  for (uint8_t id = 1; id < NUM_FAULTS; id++) if (isActive(id) && FAULTS[id].sev > w) w = FAULTS[id].sev;
  return w;
}
uint8_t dominantFault() {                          // most severe active fault
  uint8_t best = F_NONE, bs = SEV_NONE;
  for (uint8_t id = 1; id < NUM_FAULTS; id++) if (isActive(id) && FAULTS[id].sev > bs) { bs = FAULTS[id].sev; best = id; }
  return best;
}

// Debounced set/clear. A fault becomes active after ASSERT_N bad samples in a row
// and clears after CLEAR_N good samples in a row.
void faultReport(uint8_t id, bool cond) {
  uint32_t now = millis();
  if (cond) {
    cCnt[id] = 0;
    if (!isActive(id) && ++aCnt[id] >= ASSERT_N) {
      activeMask |= (uint16_t)(1u << id); aCnt[id] = 0;
      occur[id]++; if (occur[id] == 1) firstMs[id] = now;
      lastMs[id] = now; lastFaultId = id;
      logj("FAULT", "\"action\":\"SET\",\"fault_id\":%u,\"fault\":\"%s\",\"source\":\"%s\",\"severity\":%u,\"count\":%u",
           id, FAULTS[id].name, SOURCE_NAME[FAULTS[id].source], FAULTS[id].sev, occur[id]);
    }
  } else {
    aCnt[id] = 0;
    if (isActive(id) && ++cCnt[id] >= CLEAR_N) {
      activeMask &= (uint16_t)~(1u << id); cCnt[id] = 0; lastMs[id] = now;
      logj("FAULT", "\"action\":\"CLEAR\",\"fault_id\":%u,\"fault\":\"%s\",\"source\":\"%s\"",
           id, FAULTS[id].name, SOURCE_NAME[FAULTS[id].source]);
    }
  }
}

void dumpRegistry() {
  for (uint8_t id = 1; id < NUM_FAULTS; id++)
    logj("REGISTRY", "\"fault_id\":%u,\"fault\":\"%s\",\"source\":\"%s\",\"severity\":%u,\"active\":%s,\"count\":%u,\"first_ms\":%lu,\"last_ms\":%lu",
         id, FAULTS[id].name, SOURCE_NAME[FAULTS[id].source], FAULTS[id].sev, isActive(id) ? "true" : "false",
         occur[id], (unsigned long)firstMs[id], (unsigned long)lastMs[id]);
}

// ---------- State machine core ----------
void enterState(uint8_t next, uint8_t fid, const char* why) {
  uint8_t prev = state;
  seqNo++;
  logj("STATE", "\"seq\":%lu,\"from\":\"%s\",\"to\":\"%s\",\"fault_id\":%u,\"fault\":\"%s\",\"source\":\"%s\",\"reason\":\"%s\"",
       (unsigned long)seqNo, STATE_NAME[prev], STATE_NAME[next], fid, FAULTS[fid].name, SOURCE_NAME[FAULTS[fid].source], why);
  state = next; stateSince = millis(); transitions++;
  if (next == S_NORMAL)        { verifyAttempts = 0; limitPct = 100; setRelayCmd(true); }
  else if (next == S_DEGRADED) { limitPct = 50; setRelayCmd(true); }
  else if (next == S_FAILSAFE) {
    limitPct = 0; setRelayCmd(false);
    failsafeSince = stateSince; vstep = V_WAIT_CLEAR; vT = stateSince;
    if (prev == S_DEGRADED) verifyAttempts++;     // relapse after a recovery counts against us
  } else { limitPct = 0; setRelayCmd(false); }    // SHUTDOWN
}

void dispatch(uint8_t ev) {
  uint8_t next = TRANS[state][ev];
  if (next == state) return;
  uint8_t fid = F_NONE;
  if (ev == EV_MINOR || ev == EV_MAJOR || ev == EV_FATAL || ev == EV_EXHAUSTED) {
    fid = dominantFault(); if (!fid) fid = lastFaultId;
  }
  enterState(next, fid, ev == EV_EXHAUSTED ? exhaustReason : EVENT_REASON[ev]);
}

// ---------- FAILSAFE verification: a fixed sequence of checks ----------
void vgo(uint8_t step) {
  vstep = step; vT = millis();
  if (step == V_ADC_CHECK) for (uint8_t c = 0; c < 3; c++) { adcMin[c] = 4095; adcMax[c] = 0; }
  logj("VERIFY", "\"step\":\"%s\",\"result\":\"START\"", STEP_NAME[step]);
}
void vpass(const char* what) { logj("VERIFY", "\"step\":\"%s\",\"result\":\"PASS\"", what); }

uint8_t vfail(const char* why) {
  verifyAttempts++;
  setRelayCmd(false);
  logj("VERIFY", "\"step\":\"%s\",\"result\":\"FAIL\",\"reason\":\"%s\",\"attempt\":%u", STEP_NAME[vstep], why, verifyAttempts);
  vgo(V_WAIT_CLEAR);
  if (verifyAttempts >= MAX_VERIFY_ATTEMPTS) { exhaustReason = "verification failed repeatedly"; return EV_EXHAUSTED; }
  return 0xFF;
}

uint8_t verifyRun() {                              // returns an event, or 0xFF for none
  uint32_t now = millis();
  if (verifyAttempts >= MAX_VERIFY_ATTEMPTS) { exhaustReason = "too many failed recoveries"; return EV_EXHAUSTED; }
  if (now - failsafeSince >= FAILSAFE_TIMEOUT_MS) { exhaustReason = "failsafe timeout"; return EV_EXHAUSTED; }
  bool any = activeMask != 0;
  switch (vstep) {
    case V_WAIT_CLEAR:
      if (!any) vgo(V_STABLE);
      break;
    case V_STABLE:
      if (any) vgo(V_WAIT_CLEAR);
      else if (now - vT >= VERIFY_STABLE_MS) { vpass("STABLE"); vgo(V_RELAY_TEST); relayPhase = 0; setRelayCmd(true); }
      break;
    case V_RELAY_TEST:                             // close, check feedback; open, check feedback
      if (now - vT >= RELAY_TEST_MS) {
        bool fb = readFeedback();
        if (relayPhase == 0) {
          if (fb) { relayPhase = 1; setRelayCmd(false); vT = now; }
          else return vfail("relay did not close");
        } else {
          if (!fb) { vpass("RELAY_TEST"); vgo(V_ADC_CHECK); }
          else return vfail("relay did not open");
        }
      }
      break;
    case V_ADC_CHECK:                              // every channel must show live variation
      if (any) return vfail("fault during adc check");
      if (now - vT >= ADC_CHECK_MS) {
        bool live = true;
        for (uint8_t c = 0; c < 3; c++) if (adcMax[c] <= adcMin[c]) live = false;
        if (live) { vpass("ADC_CHECK"); vgo(V_COMM_CHECK); }
        else return vfail("adc channel not live");
      }
      break;
    case V_COMM_CHECK:                             // consecutive, in-sequence good frames
      if (any) return vfail("fault during comm check");
      if (goodStreak >= COMM_CHECK_GOOD) { vpass("COMM_CHECK"); return EV_VERIFIED; }
      if (now - vT >= COMM_CHECK_MS) return vfail("comm not healthy");
      break;
  }
  return 0xFF;
}

// ---------- Communication (UART loopback, 4-byte frame with CRC) ----------
uint8_t crc8(const uint8_t* d, uint8_t n) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) { c ^= d[i]; for (uint8_t b = 0; b < 8; b++) c = (c & 0x80) ? (c << 1) ^ 0x07 : (c << 1); }
  return c;
}
void commTxTask() {
  if (inject == INJ_COMM_LOSS) return;             // injected: sender silent
  uint8_t f[4] = {0xA5, txSeq++, 0x01, 0};
  f[3] = crc8(f, 3);
  if (inject == INJ_COMM_NOISE && (txSeq & 1)) f[2] ^= 0x10;   // corrupt every other frame
  Serial2.write(f, 4);
}
void commRxTask() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (rxIdx == 0 && b != 0xA5) continue;
    rxBuf[rxIdx++] = b;
    if (rxIdx < 4) continue;
    rxIdx = 0;
    if (crc8(rxBuf, 3) == rxBuf[3]) {
      framesOk++; lastGoodMs = millis();
      goodStreak = (haveSeq && rxBuf[1] == (uint8_t)(rxSeq + 1)) ? (goodStreak < 255 ? goodStreak + 1 : 255) : 1;
      rxSeq = rxBuf[1]; haveSeq = true;
      if (crcScore) crcScore--;
    } else {
      crcErrors++; goodStreak = 0; crcScore = min(10, crcScore + 2);
    }
  }
  if (millis() - lastGoodMs > COMM_TIMEOUT_MS) goodStreak = 0;
}

// ---------- Sensors and fault detection (50 ms task) ----------
uint16_t readAdc(uint8_t ch) {
  int v = constrain(analogRead(CELL_PIN[ch]) + random(-3, 4), 0, 4095);   // real ADCs are noisy
  if (inject == INJ_ADC_FREEZE && ch == FROZEN_CH) return heldRaw;        // injected: stuck value
  return (uint16_t)v;
}

void detectFaults() {
  uint32_t now = millis();
  // ADC: frozen value (identical reading many times in a row)
  for (uint8_t c = 0; c < 3; c++) {
    uint16_t raw = readAdc(c); rawNow[c] = raw;
    volts[c] = 2.8 + raw / 4095.0 * 1.6;
    if (raw == lastRaw[c]) { if (same[c] < 255) same[c]++; } else same[c] = 0;
    lastRaw[c] = raw;
    if (state == S_FAILSAFE && vstep == V_ADC_CHECK) { if (raw < adcMin[c]) adcMin[c] = raw; if (raw > adcMax[c]) adcMax[c] = raw; }
    faultReport(F_ADC1_FRZ + c, same[c] >= FROZEN_N);
  }
  // Relay: commanded state must match the feedback after the settle time
  bool fb = readFeedback(), settled = (now - relayCmdSince) >= RELAY_SETTLE_MS;
  faultReport(F_RLY_OPEN, settled && relayCmd && !fb);
  faultReport(F_RLY_WELD, settled && !relayCmd && fb);
  // Cells: a cell with a frozen ADC is isolated (its stale value is not trusted)
  float hi = -1, lo = 99; uint8_t n = 0;
  for (uint8_t c = 0; c < 3; c++) {
    if (isActive(F_ADC1_FRZ + c)) continue;
    float v = volts[c];
    faultReport(F_C1_LOW + c, isActive(F_C1_LOW + c) ? v < LOW_CLR : v < LOW_TRIP);
    faultReport(F_C1_CRIT + c, isActive(F_C1_CRIT + c) ? (v < CRIT_LO_CLR || v > CRIT_HI_CLR) : (v < CRIT_LO || v > CRIT_HI));
    if (v > hi) hi = v; if (v < lo) lo = v; n++;
  }
  if (n >= 2) faultReport(F_IMBAL, (hi - lo) > (isActive(F_IMBAL) ? IMBAL_CLR : IMBAL_TRIP));
  // Communication
  faultReport(F_COMM_TMO, now - lastGoodMs > COMM_TIMEOUT_MS);
  faultReport(F_COMM_CRC, isActive(F_COMM_CRC) ? crcScore > 0 : crcScore >= 6);
}

// ---------- One decision per sample: turn faults into an event ----------
void stateTask() {
  uint32_t now = millis();
  uint8_t worst = worstSeverity(), ev = 0xFF;
  if (worst == SEV_NONE) { if (!cleanOn) { cleanOn = true; cleanSince = now; } } else cleanOn = false;

  if (resetRequested) {
    resetRequested = false;
    if (state == S_SHUTDOWN) { verifyAttempts = 0; ev = EV_RESET; }
    else logj("INFO", "\"msg\":\"reset ignored, not in SHUTDOWN\"");
  } else if (worst == SEV_SD) ev = EV_FATAL;
  else if (state == S_FAILSAFE) ev = verifyRun();          // verification decides
  else if (worst == SEV_FS) ev = EV_MAJOR;
  else if (worst == SEV_DEG) ev = EV_MINOR;
  else if (state == S_DEGRADED && cleanOn) {                // probation counts from DEGRADED entry
    uint32_t from = cleanSince > stateSince ? cleanSince : stateSince;
    if (now - from >= DEGRADED_HOLD_MS) ev = EV_CLEAR;
  }
  if (ev != 0xFF) dispatch(ev);
}

// ---------- Buttons (polled, debounced) ----------
const uint8_t BTN_PIN[3] = {BTN_RESET, BTN_INJECT, BTN_REPORT};
bool bStable[3] = {HIGH, HIGH, HIGH}, bLast[3] = {HIGH, HIGH, HIGH};
uint32_t bT[3] = {0, 0, 0};
bool btnPressed(uint8_t k) {
  bool r = digitalRead(BTN_PIN[k]); uint32_t now = millis();
  if (r != bLast[k]) { bLast[k] = r; bT[k] = now; }
  if (r != bStable[k] && now - bT[k] >= 30) { bStable[k] = r; return r == LOW; }
  return false;
}
void buttonsTask() {
  if (btnPressed(0)) resetRequested = true;
  if (btnPressed(1)) {
    inject = (inject + 1) % NUM_INJ;
    if (inject == INJ_ADC_FREEZE) heldRaw = rawNow[FROZEN_CH];
    logj("INJECT", "\"mode\":\"%s\"", INJ_NAME[inject]);
  }
  if (btnPressed(2)) dumpRegistry();
}

// ---------- Outputs ----------
void outputsTask() {
  uint32_t now = millis();
  bool slow = (now / 500) % 2, r = false, g = false, y = false, bz = false;
  if (state == S_NORMAL) g = true;
  else if (state == S_DEGRADED) y = true;
  else if (state == S_FAILSAFE) { r = slow; bz = (now % 1000) < 100; }
  else { r = true; bz = (now % 300) < 150; }
  digitalWrite(RED_LED, r); digitalWrite(GREEN_LED, g); digitalWrite(YELLOW_LED, y); digitalWrite(BUZZER, bz);
}

void lcdLine(uint8_t row, const char* s) {          // rewrites a line only if it changed
  static char last[2][17];
  char b[17]; snprintf(b, sizeof b, "%-16s", s);
  if (strcmp(b, last[row]) == 0) return;
  strcpy(last[row], b); lcd.setCursor(0, row); lcd.print(b);
}
void lcdTask() {
  char a[24], b[24];
  uint8_t d = dominantFault(), n = 0;
  for (uint8_t id = 1; id < NUM_FAULTS; id++) n += isActive(id);
  snprintf(a, sizeof a, "%-8s Lim%3d%%", STATE_NAME[state], limitPct);
  if (state == S_FAILSAFE && n == 0) snprintf(b, sizeof b, "Verify:%s", STEP_NAME[vstep]);
  else if (state == S_SHUTDOWN && (millis() / 2000) % 2) snprintf(b, sizeof b, "Press RESET btn");
  else if (n) snprintf(b, sizeof b, "F%02u %-8s %s", d, FAULTS[d].name, n > 1 ? "+" : "");
  else snprintf(b, sizeof b, "No faults");
  lcdLine(0, a); lcdLine(1, b);
}

bool tableValid() {                                  // every table entry must be a real state
  for (uint8_t s = 0; s < NUM_STATES; s++) for (uint8_t e = 0; e < NUM_EVENTS; e++) if (TRANS[s][e] >= NUM_STATES) return false;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, COMM_RX, COMM_TX);
  pinMode(RED_LED, OUTPUT); pinMode(GREEN_LED, OUTPUT); pinMode(YELLOW_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT); pinMode(RELAY_PIN, OUTPUT); pinMode(RELAY_FB_PIN, INPUT_PULLDOWN);
  for (uint8_t i = 0; i < 3; i++) pinMode(BTN_PIN[i], INPUT_PULLUP);
  lcd.init(); lcd.backlight();
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH);   // start closed
  relayCmd = true; relayCmdSince = millis(); lastGoodMs = millis();
  logj("BOOT", "\"msg\":\"fault state machine started\",\"table_ok\":%s", tableValid() ? "true" : "false");
}

void loop() {
  static uint32_t tSample = 0, tComm = 0, tLcd = 0, tStat = 0, lastUs = 0;
  uint32_t now = millis(), us = micros();
  if (lastUs) { uint32_t dt = us - lastUs; if (dt > maxLoopUs) maxLoopUs = dt; if (dt > 50000) stalls++; }
  lastUs = us;

  commRxTask();
  if (now - tComm >= COMM_PERIOD_MS) { tComm += COMM_PERIOD_MS; if (now - tComm > 5 * COMM_PERIOD_MS) tComm = now; commTxTask(); }
  if (now - tSample >= SAMPLE_MS) {
    tSample += SAMPLE_MS; if (now - tSample > 5 * SAMPLE_MS) tSample = now;
    detectFaults();
    stateTask();
  }
  buttonsTask();
  outputsTask();
  if (now - tLcd >= LCD_MS) { tLcd = now; lcdTask(); }
  if (now - tStat >= STAT_MS) {
    tStat = now;
    logj("STAT", "\"state\":\"%s\",\"active_mask\":\"0x%04X\",\"limit_pct\":%u,\"transitions\":%lu,\"attempts\":%u,\"relay_cmd\":\"%s\",\"relay_fb\":\"%s\",\"frames_ok\":%lu,\"crc_errors\":%lu,\"max_loop_us\":%lu,\"stalls\":%lu",
         STATE_NAME[state], activeMask, limitPct, (unsigned long)transitions, verifyAttempts, relayCmd ? "CLOSED" : "OPEN",
         readFeedback() ? "CLOSED" : "OPEN", (unsigned long)framesOk, (unsigned long)crcErrors, (unsigned long)maxLoopUs, (unsigned long)stalls);
  }
}
