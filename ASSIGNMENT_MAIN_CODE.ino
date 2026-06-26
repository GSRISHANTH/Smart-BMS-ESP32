// ================================================================
// SMART BATTERY MANAGEMENT SYSTEM
// Author        : Srishanth G
// Organisation  : ElevanceSkills Technology Private Limited
// Platform      : ESP32 DevKit C v4 + Wokwi Simulator + Blynk Cloud
//
// CIRCUIT CONNECTIONS:
//   pot1 SIG → GPIO34  (Cell 1)   pot2 SIG → GPIO35  (Cell 2)
//   pot3 SIG → GPIO32  (Cell 3)   pot4 SIG → GPIO33  (Cell 4)
//   All pots: VCC → 3V3, GND → GND
//   LCD SDA → GPIO21, LCD SCL → GPIO22, LCD VCC → 3V3, LCD GND → GND
//   Relay IN → GPIO25 (active-LOW), Relay VCC → 3V3, Relay GND → GND
//   Relay NO → LED(A), Relay COM → 3V3, LED(C) → GND
//   Buzzer+ → GPIO26, Buzzer- → GND
//
// TASKS:
//   TASK 1 : Adaptive Multi-Cell Battery Intelligence Engine
//   TASK 2 : Event-Driven Safety Protection Kernel
//   TASK 3 : Intelligent Embedded HMI & Diagnostic Interface
//   TASK 4 : Fault-Tolerant Embedded Runtime System
//   TASK 5 : Intelligent Cloud Telemetry Architecture
//   TASK 6 : Executive Battery Intelligence Dashboard
// ================================================================

#define BLYNK_TEMPLATE_ID   "TMPL3wGv2b_vq"
#define BLYNK_TEMPLATE_NAME "BMS Dashboard"
#define BLYNK_AUTH_TOKEN    "Bi_3ZxWzZ8mE7rkVh36nprKqTYW3z9NN"
#define BLYNK_PRINT         Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>

// ================================================================
// BLYNK VIRTUAL PIN MAP
// ================================================================
// V0–V3  : Cell 1–4 Voltage (Double, 3.0–4.5)
// V4     : Pack Average Voltage (Double, 3.0–4.5)
// V5     : Imbalance % (Double, 0–100)
// V6     : Health Status (String)
// V7     : System Mode (String)
// V8     : Relay State (String)
// V9     : Protection State (String)
// V10    : Fault Count (Integer, 0–9999)
// V11    : Weakest Cell (String)
// V12    : Signal Quality % (Integer, 0–100)
// V13    : Recommendation (String)
// V14    : Risk Level (Integer, 0–3)
// V15    : Strongest Cell (String)
// V16    : Fault Log Terminal (String)
// V17    : Total Faults (Integer, 0–9999)
// V18–V21: Cell 1–4 History for SuperCharts (Double, 3.0–4.5)

// ================================================================
// HARDWARE PIN DEFINITIONS
// ================================================================
#define PIN_CELL1   34
#define PIN_CELL2   35
#define PIN_CELL3   32
#define PIN_CELL4   33
#define PIN_RELAY   25
#define PIN_BUZZER  26

// ================================================================
// BATTERY VOLTAGE THRESHOLDS
// ================================================================
#define NUM_CELLS           4
#define V_FULL              4.50f
#define V_MIN               3.00f
#define V_OVERVOLT          4.30f
#define V_WARN              3.30f
#define IMBALANCE_MINOR     2.0f
#define IMBALANCE_CRITICAL  5.0f
#define RAPID_CHANGE_THRESH 0.20f
#define ADC_MAX             4095.0f
#define ADC_SAMPLES         8
#define ADC_FAULT_LOW       3
#define ADC_FAULT_HIGH      4092
#define ADC_FAULT_CYCLES    3

// ================================================================
// TIMING CONSTANTS (millis-based, no delay() in loop)
// ================================================================
#define T_SENSOR          500UL
#define T_LCD_REFRESH     800UL
#define T_LCD_ROTATE      4000UL
#define T_CELL_FLIP       2000UL
#define T_RELAY_DEBOUNCE  1500UL
#define T_BUZZER_ON       300UL
#define T_BUZZER_OFF      700UL
#define T_WIFI_RETRY      8000UL
#define T_BLYNK_MIN       2000UL
#define T_BLYNK_RUN       100UL
#define T_QUEUE_REPLAY    300UL
#define T_FAULT_DEDUP     5000UL

// ================================================================
// RUNTIME CONSTANTS
// ================================================================
#define RECOVERY_CYCLES          6
#define SHUTDOWN_RECOVERY_CYCLES 12
#define STARTUP_SKIP_CYCLES      8
#define FAULT_LOG_MAX            10
#define EVENT_QUEUE_MAX          8
#define RELAY_MISMATCH_LIM       4
#define FROZEN_ADC_CYCLES        20

// ================================================================
// ENUMERATIONS
// ================================================================
enum BatteryHealth : uint8_t {
  HEALTH_HEALTHY            = 0,
  HEALTH_MINOR_IMBALANCE    = 1,
  HEALTH_CRITICAL_IMBALANCE = 2,
  HEALTH_PACK_FAILURE       = 3
};

enum SystemMode : uint8_t {
  MODE_NORMAL   = 0,
  MODE_DEGRADED = 1,
  MODE_FAILSAFE = 2,
  MODE_SHUTDOWN = 3
};

enum ProtectionEvent : uint8_t {
  PROT_NONE         = 0,
  PROT_WEAK_CELL    = 1,
  PROT_OVERVOLTAGE  = 2,
  PROT_SENSOR_FAULT = 3,
  PROT_RAPID_CHANGE = 4
};

enum LCDScreen : uint8_t {
  SCREEN_CELLS  = 0,
  SCREEN_PACK   = 1,
  SCREEN_SAFETY = 2,
  SCREEN_FAULTS = 3,
  SCREEN_COUNT  = 4
};

// ================================================================
// DATA STRUCTURES
// ================================================================
struct CellData {
  float    voltage;
  float    prevVoltage;
  uint16_t rawADC;
  bool     isFaulty;
  bool     isWeak;
  bool     isOverVoltage;
  uint8_t  faultCycles;
};

struct PackData {
  float         avgVoltage;
  float         minVoltage;
  float         maxVoltage;
  float         imbalancePct;
  uint8_t       weakestCell;
  uint8_t       strongestCell;
  uint8_t       faultyCells;
  BatteryHealth health;
};

struct SafetyData {
  ProtectionEvent activeEvent;
  ProtectionEvent prevEvent;
  bool            relayOpen;
  bool            buzzerActive;
  bool            buzzerPinState;
  unsigned long   lastRelayChange;
  unsigned long   lastBuzzerToggle;
  uint8_t         clearCycles;
  char            lcdWarning[17];
  uint8_t         relayMismatchCounter;
  bool            relayMismatchDetected;
};

struct FaultRecord {
  unsigned long timestamp;
  char          message[52];
  SystemMode    modeAtFault;
  float         cellSnapshot[NUM_CELLS];
};

struct FaultLog {
  FaultRecord   records[FAULT_LOG_MAX];
  uint8_t       writeIdx;
  uint32_t      totalFaults;
  char          lastMessage[52];
  unsigned long lastFaultTime;
  bool          newFaultReady;
  bool          logOverflowed;
};

struct RuntimeData {
  SystemMode mode;
  uint8_t    recoveryCycles;
  bool       frozenADC;
  bool       relayMismatch;
  uint8_t    startupCycles;
  uint8_t    frozenCounter;
};

struct QueuedEvent {
  float           voltages[NUM_CELLS];
  BatteryHealth   health;
  SystemMode      mode;
  ProtectionEvent event;
  unsigned long   timestamp;
};

struct TelemetryData {
  float           lastVoltages[NUM_CELLS];
  BatteryHealth   lastHealth;
  SystemMode      lastMode;
  ProtectionEvent lastEvent;
  unsigned long   lastSendTime;
  bool            offlineMode;
  int             rssiPercent;
  QueuedEvent     eventQueue[EVENT_QUEUE_MAX];
  uint8_t         queueHead;
  uint8_t         queueTail;
  uint8_t         queueCount;
  bool            replayActive;
  unsigned long   lastReplayTime;
  BatteryHealth   lastQueuedHealth;
  SystemMode      lastQueuedMode;
  ProtectionEvent lastQueuedEvent;
};

// ================================================================
// GLOBAL INSTANCES
// ================================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
CellData      cells[NUM_CELLS];
PackData      pack;
SafetyData    safety;
FaultLog      faults;
RuntimeData   runtime;
TelemetryData telemetry;

// ================================================================
// TIMING TRACKERS
// ================================================================
unsigned long lastSensorTime = 0;
unsigned long lastLCDRefresh = 0;
unsigned long lastLCDRotate  = 0;
unsigned long lastCellFlip   = 0;
unsigned long lastWiFiRetry  = 0;
unsigned long lastBlynkRun   = 0;

// ================================================================
// LCD STATE
// ================================================================
LCDScreen currentScreen  = SCREEN_CELLS;
LCDScreen prevScreen     = SCREEN_COUNT;
uint8_t   cellSubScreen  = 0;
char      lcdPrev0[17]   = "";
char      lcdPrev1[17]   = "";
bool      lcdForceUpdate = true;
bool      lcdWasCritical = false;
float     lcdLastVoltages[NUM_CELLS] = {0.0f, 0.0f, 0.0f, 0.0f};

// ================================================================
// NETWORK STATE
// ================================================================
bool wifiOK          = false;
bool blynkOK         = false;
bool blynkConfigured = false;
char ssid[]          = "Wokwi-GUEST";
char pass[]          = "";

// ================================================================
// FORWARD DECLARATIONS
// ================================================================
void        logFault(const char* msg);
void        task2_ControlRelay(bool open, unsigned long now);
void        task2_StartBuzzer(unsigned long now);
void        lcdInvalidate();
void        updateLCDVoltageSnapshot();
const char* getHealthStr();
const char* getHealthShort();
const char* getModeStr();
const char* getModeShort();
const char* getProtectionStr(ProtectionEvent e);
const char* getProtectionShort(ProtectionEvent e);
const char* getRiskLabel(uint8_t level);
String      task6_GetRecommendation();
uint8_t     task6_GetRiskLevel();
int         task5_SignalQuality();
bool        voltagesChangedForLCD();
bool        task5_StateChanged();
void        task5_QueueEvent();
void        task5_PushToBlynk();

// ================================================================
// SERIAL HELPERS
// ================================================================
void serialLine() { Serial.println(F("====================================================")); }
void serialDash() { Serial.println(F("----------------------------------------------------")); }

// ================================================================
// TASK 1 — ADAPTIVE MULTI-CELL BATTERY INTELLIGENCE ENGINE
// Read 4 cell voltages via ADC, classify each cell, compute pack
// statistics (avg, min, max, imbalance%) from healthy cells only,
// identify weakest/strongest cell, classify 4-state pack health.
// ================================================================

uint16_t readADCRaw(uint8_t pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) sum += analogRead(pin);
  return (uint16_t)(sum / ADC_SAMPLES);
}

float adcToVoltage(uint16_t raw) {
  return V_MIN + ((float)raw / ADC_MAX) * (V_FULL - V_MIN);
}

void task1_ReadVoltages() {
  const uint8_t pins[NUM_CELLS] = { PIN_CELL1, PIN_CELL2, PIN_CELL3, PIN_CELL4 };
  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    cells[i].prevVoltage = cells[i].voltage;
    cells[i].rawADC      = readADCRaw(pins[i]);
    cells[i].voltage     = adcToVoltage(cells[i].rawADC);
  }
}

void task1_ClassifyCells() {
  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    bool nearRail = (cells[i].rawADC <= ADC_FAULT_LOW || cells[i].rawADC >= ADC_FAULT_HIGH);
    if (nearRail) {
      cells[i].faultCycles++;
      if (cells[i].faultCycles >= ADC_FAULT_CYCLES) cells[i].isFaulty = true;
    } else {
      cells[i].faultCycles = 0;
      cells[i].isFaulty    = false;
    }
    cells[i].isWeak        = (!cells[i].isFaulty && cells[i].voltage < V_WARN);
    cells[i].isOverVoltage = (!cells[i].isFaulty && cells[i].voltage > V_OVERVOLT);
  }
}

void task1_CalculatePack() {
  float   sum    = 0.0f, minV = 99.0f, maxV = 0.0f;
  uint8_t minIdx = 0, maxIdx = 0, valid = 0;
  pack.faultyCells = 0;

  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    if (cells[i].isFaulty) { pack.faultyCells++; continue; }
    sum += cells[i].voltage;
    valid++;
    if (cells[i].voltage < minV) { minV = cells[i].voltage; minIdx = i; }
    if (cells[i].voltage > maxV) { maxV = cells[i].voltage; maxIdx = i; }
  }

  pack.avgVoltage = (valid > 0) ? sum / valid : 0.0f;
  pack.minVoltage = (valid > 0) ? minV        : 0.0f;
  pack.maxVoltage = (valid > 0) ? maxV        : 0.0f;

  if      (valid > 1 && pack.avgVoltage > 0.0f)
    pack.imbalancePct = ((maxV - minV) / pack.avgVoltage) * 100.0f;
  else if (valid == 1) pack.imbalancePct = 100.0f;
  else                 pack.imbalancePct = 0.0f;

  pack.weakestCell   = (valid > 0) ? minIdx + 1 : 0;
  pack.strongestCell = (valid > 0) ? maxIdx + 1 : 0;

  if      (pack.faultyCells >= 2 || valid == 0)                        pack.health = HEALTH_PACK_FAILURE;
  else if (pack.faultyCells == 1 || pack.imbalancePct >= IMBALANCE_CRITICAL) pack.health = HEALTH_CRITICAL_IMBALANCE;
  else if (pack.imbalancePct >= IMBALANCE_MINOR)                        pack.health = HEALTH_MINOR_IMBALANCE;
  else                                                                   pack.health = HEALTH_HEALTHY;
}

void runTask1() {
  task1_ReadVoltages();
  task1_ClassifyCells();
  task1_CalculatePack();
}

// ================================================================
// TASK 2 — EVENT-DRIVEN SAFETY PROTECTION KERNEL
// Detect 4 events (SENSOR_FAULT > OVERVOLTAGE > WEAK_CELL > RAPID_CHANGE).
// Relay opens for first three; buzzer fires for all four.
// Non-blocking 300ms ON / 700ms OFF buzzer via millis().
// Anti-chatter debounce 1500ms; 6-cycle recovery before relay closes.
// ================================================================

ProtectionEvent task2_EvaluateEvent() {
  for (uint8_t i = 0; i < NUM_CELLS; i++)
    if (cells[i].isFaulty)      return PROT_SENSOR_FAULT;
  for (uint8_t i = 0; i < NUM_CELLS; i++)
    if (cells[i].isOverVoltage) return PROT_OVERVOLTAGE;
  for (uint8_t i = 0; i < NUM_CELLS; i++)
    if (cells[i].isWeak)        return PROT_WEAK_CELL;
  if (runtime.startupCycles >= STARTUP_SKIP_CYCLES) {
    for (uint8_t i = 0; i < NUM_CELLS; i++) {
      if (!cells[i].isFaulty &&
          fabsf(cells[i].voltage - cells[i].prevVoltage) > RAPID_CHANGE_THRESH)
        return PROT_RAPID_CHANGE;
    }
  }
  return PROT_NONE;
}

void task2_ControlRelay(bool shouldOpen, unsigned long now) {
  if (shouldOpen == safety.relayOpen) return;
  if ((now - safety.lastRelayChange) < T_RELAY_DEBOUNCE) return;
  safety.relayOpen       = shouldOpen;
  safety.lastRelayChange = now;
  digitalWrite(PIN_RELAY, shouldOpen ? LOW : HIGH);
  char buf[52];
  if (shouldOpen) snprintf(buf, sizeof(buf), "RELAY OPEN-%s", getProtectionStr(safety.activeEvent));
  else            strncpy(buf, "RELAY CLOSED-Cleared", 51);
  buf[51] = '\0';
  logFault(buf);
}

// Called every loop() for precise 300ms ON / 700ms OFF timing
void task2_ControlBuzzer(unsigned long now) {
  if (!safety.buzzerActive) {
    if (safety.buzzerPinState) { digitalWrite(PIN_BUZZER, LOW); safety.buzzerPinState = false; }
    return;
  }
  unsigned long elapsed = now - safety.lastBuzzerToggle;
  if (safety.buzzerPinState && elapsed >= T_BUZZER_ON) {
    digitalWrite(PIN_BUZZER, LOW);
    safety.buzzerPinState   = false;
    safety.lastBuzzerToggle = now;
  } else if (!safety.buzzerPinState && elapsed >= T_BUZZER_OFF) {
    digitalWrite(PIN_BUZZER, HIGH);
    safety.buzzerPinState   = true;
    safety.lastBuzzerToggle = now;
  }
}

void task2_StartBuzzer(unsigned long now) {
  digitalWrite(PIN_BUZZER, HIGH);
  safety.buzzerPinState   = true;
  safety.lastBuzzerToggle = now;
  safety.buzzerActive     = true;
}

void task2_UpdateLCDWarning() {
  switch (safety.activeEvent) {
    case PROT_SENSOR_FAULT:  strncpy(safety.lcdWarning, "** SENSOR FAULT!", 16); break;
    case PROT_OVERVOLTAGE:   strncpy(safety.lcdWarning, "** OVERVOLTAGE! ", 16); break;
    case PROT_WEAK_CELL:     strncpy(safety.lcdWarning, "** WEAK CELL!   ", 16); break;
    case PROT_RAPID_CHANGE:  strncpy(safety.lcdWarning, "** RAPID CHANGE!", 16); break;
    default:                 safety.lcdWarning[0] = '\0'; break;
  }
  safety.lcdWarning[16] = '\0';
}

void runTask2(unsigned long now) {
  BatteryHealth prevHealth = pack.health;
  SystemMode    prevMode   = runtime.mode;

  safety.prevEvent   = safety.activeEvent;
  safety.activeEvent = task2_EvaluateEvent();

  if (safety.activeEvent != PROT_NONE && safety.activeEvent != safety.prevEvent)
    safety.clearCycles = 0;

  bool openRelay    = false;
  bool doBuzz       = false;
  bool buzzStarting = !safety.buzzerActive;

  switch (safety.activeEvent) {
    case PROT_SENSOR_FAULT:
    case PROT_OVERVOLTAGE:
    case PROT_WEAK_CELL:
      openRelay = true; doBuzz = true; safety.clearCycles = 0; break;
    case PROT_RAPID_CHANGE:
      openRelay = false; doBuzz = true; safety.clearCycles = 0; break;
    case PROT_NONE:
      safety.clearCycles++;
      if (safety.clearCycles >= RECOVERY_CYCLES) {
        openRelay = false; doBuzz = false;
        safety.clearCycles = RECOVERY_CYCLES;
      } else {
        openRelay = safety.relayOpen;
        doBuzz    = safety.buzzerActive;
      }
      break;
  }

  if (runtime.mode != MODE_SHUTDOWN)
    task2_ControlRelay(openRelay, now);

  if (doBuzz && buzzStarting && safety.activeEvent != PROT_NONE)
    task2_StartBuzzer(now);
  else
    safety.buzzerActive = doBuzz;

  task2_UpdateLCDWarning();

  if (pack.health != prevHealth || runtime.mode != prevMode)
    lcdInvalidate();
}

// ================================================================
// STRING HELPERS
// ================================================================
const char* getHealthStr() {
  switch (pack.health) {
    case HEALTH_HEALTHY:            return "HEALTHY";
    case HEALTH_MINOR_IMBALANCE:    return "MINOR IMBALANCE";
    case HEALTH_CRITICAL_IMBALANCE: return "CRITICAL IMBAL.";
    case HEALTH_PACK_FAILURE:       return "PACK FAILURE";
    default:                        return "UNKNOWN";
  }
}

const char* getHealthShort() {
  switch (pack.health) {
    case HEALTH_HEALTHY:            return "OK  ";
    case HEALTH_MINOR_IMBALANCE:    return "MINR";
    case HEALTH_CRITICAL_IMBALANCE: return "CRIT";
    case HEALTH_PACK_FAILURE:       return "FAIL";
    default:                        return "?   ";
  }
}

const char* getModeStr() {
  switch (runtime.mode) {
    case MODE_NORMAL:   return "NORMAL";
    case MODE_DEGRADED: return "DEGRADED";
    case MODE_FAILSAFE: return "FAILSAFE";
    case MODE_SHUTDOWN: return "SHUTDOWN";
    default:            return "UNKNOWN";
  }
}

const char* getModeShort() {
  switch (runtime.mode) {
    case MODE_NORMAL:   return "NORM";
    case MODE_DEGRADED: return "DEGD";
    case MODE_FAILSAFE: return "SAFE";
    case MODE_SHUTDOWN: return "SHTD";
    default:            return "????";
  }
}

const char* getProtectionStr(ProtectionEvent e) {
  switch (e) {
    case PROT_NONE:         return "NONE";
    case PROT_WEAK_CELL:    return "WEAK CELL";
    case PROT_OVERVOLTAGE:  return "OVERVOLTAGE";
    case PROT_SENSOR_FAULT: return "SENSOR FAULT";
    case PROT_RAPID_CHANGE: return "RAPID CHANGE";
    default:                return "UNKNOWN";
  }
}

const char* getProtectionShort(ProtectionEvent e) {
  switch (e) {
    case PROT_NONE:         return "NONE  ";
    case PROT_WEAK_CELL:    return "WEAK  ";
    case PROT_OVERVOLTAGE:  return "OVRV  ";
    case PROT_SENSOR_FAULT: return "SNSFLT";
    case PROT_RAPID_CHANGE: return "RAPID ";
    default:                return "UNK   ";
  }
}

const char* getRiskLabel(uint8_t level) {
  switch (level) {
    case 0: return "SAFE";
    case 1: return "CAUTION";
    case 2: return "WARNING";
    case 3: return "CRITICAL";
    default: return "UNKNOWN";
  }
}

// ================================================================
// TASK 3 — INTELLIGENT EMBEDDED HMI & DIAGNOSTIC INTERFACE
// 4 auto-rotating screens: CELLS → PACK → SAFETY → FAULTS (4s each).
// Cell sub-screen alternates C1/C2 ↔ C3/C4 every 2s.
// Any pot change: immediately jump to SCREEN_CELLS, reset sub-screen
// to C1/C2, restart rotation timer. Flicker-free differential refresh.
// Critical override halts rotation for SENSOR_FAULT/FAILSAFE/SHUTDOWN/PACK_FAILURE.
// ================================================================

static const char* cellStatusTag(uint8_t i) {
  if (cells[i].isFaulty)      return "[ERR]";
  if (cells[i].isOverVoltage) return "[HI] ";
  if (cells[i].isWeak)        return "[LOW]";
  return "[OK] ";
}

bool voltagesChangedForLCD() {
  for (uint8_t i = 0; i < NUM_CELLS; i++)
    if (fabsf(cells[i].voltage - lcdLastVoltages[i]) >= 0.01f) return true;
  return false;
}

void updateLCDVoltageSnapshot() {
  for (uint8_t i = 0; i < NUM_CELLS; i++)
    lcdLastVoltages[i] = cells[i].voltage;
}

void lcdWrite(const char* l0, const char* l1) {
  char b0[17], b1[17];
  snprintf(b0, sizeof(b0), "%-16s", l0);
  snprintf(b1, sizeof(b1), "%-16s", l1);
  b0[16] = b1[16] = '\0';

  bool ch0 = (strncmp(b0, lcdPrev0, 16) != 0);
  bool ch1 = (strncmp(b1, lcdPrev1, 16) != 0);
  if (!lcdForceUpdate && !ch0 && !ch1) return;

  if (lcdForceUpdate || ch0) { lcd.setCursor(0, 0); lcd.print(b0); }
  if (lcdForceUpdate || ch1) { lcd.setCursor(0, 1); lcd.print(b1); }

  memcpy(lcdPrev0, b0, 17);
  memcpy(lcdPrev1, b1, 17);
  lcdForceUpdate = false;
}

void lcdInvalidate() {
  memset(lcdPrev0, 0, sizeof(lcdPrev0));
  memset(lcdPrev1, 0, sizeof(lcdPrev1));
  lcdForceUpdate = true;
}

bool isCritical() {
  return (pack.health        == HEALTH_PACK_FAILURE  ||
          safety.activeEvent == PROT_SENSOR_FAULT     ||
          runtime.mode       == MODE_SHUTDOWN         ||
          runtime.mode       == MODE_FAILSAFE);
}

void screen_Cells(unsigned long now) {
  if ((now - lastCellFlip) >= T_CELL_FLIP) {
    cellSubScreen = (cellSubScreen + 1) % 2;
    lastCellFlip  = now;
    lcdInvalidate();
  }
  uint8_t b = cellSubScreen * 2;
  char l0[17], l1[17];
  snprintf(l0, sizeof(l0), "C%d:%4.2fV %s", b+1, cells[b].voltage,   cellStatusTag(b));
  snprintf(l1, sizeof(l1), "C%d:%4.2fV %s", b+2, cells[b+1].voltage, cellStatusTag(b+1));
  lcdWrite(l0, l1);
}

void screen_Pack() {
  char l0[17], l1[17];
  float imb = (pack.imbalancePct > 99.9f) ? 99.9f : pack.imbalancePct;
  snprintf(l0, sizeof(l0), "Avg:%4.2fV  %-4s", pack.avgVoltage, getHealthShort());
  snprintf(l1, sizeof(l1), "Ib:%4.1f W:%d S:%d", imb, pack.weakestCell, pack.strongestCell);
  lcdWrite(l0, l1);
}

void screen_Safety() {
  char l0[17], l1[17];
  snprintf(l0, sizeof(l0), "Prot:%-6s     ", getProtectionShort(safety.activeEvent));
  snprintf(l1, sizeof(l1), "Mode:%-4s R:%-4s", getModeShort(), safety.relayOpen ? "OPEN" : "OK  ");
  lcdWrite(l0, l1);
}

void screen_Faults() {
  if (faults.totalFaults == 0) { lcdWrite("Faults: 0       ", "  All Systems OK"); return; }
  char l0[17];
  uint32_t cnt = (faults.totalFaults > 9999) ? 9999 : faults.totalFaults;
  snprintf(l0, sizeof(l0), "Faults: %lu", (unsigned long)cnt);
  uint8_t last = (faults.writeIdx == 0) ? FAULT_LOG_MAX-1 : faults.writeIdx-1;
  char msg[17];
  strncpy(msg, faults.records[last].message, 15);
  msg[15] = '\0';
  if (strlen(faults.records[last].message) > 15) msg[14] = '~';
  lcdWrite(l0, msg);
}

void screen_Critical() {
  char l0[17], l1[17];
  if (safety.lcdWarning[0] != '\0') snprintf(l0, sizeof(l0), "%-16s", safety.lcdWarning);
  else                               snprintf(l0, sizeof(l0), "!!%-14s", getHealthShort());
  if      (runtime.mode == MODE_SHUTDOWN) strncpy(l1, "SHTDWN:CHK CELLS", 16);
  else if (runtime.mode == MODE_FAILSAFE) strncpy(l1, "FAILSAFE:CHK BAT", 16);
  else snprintf(l1, sizeof(l1), "%-4s %-4s ALERT!", getModeShort(), getHealthShort());
  l1[16] = '\0';
  lcdWrite(l0, l1);
}

void runTask3(unsigned long now) {
  bool nowCritical = isCritical();
  if (nowCritical != lcdWasCritical) { lcdInvalidate(); lcdWasCritical = nowCritical; }

  if (nowCritical) {
    screen_Critical();
    updateLCDVoltageSnapshot();
    return;
  }

  // Any pot change: reset to SCREEN_CELLS, C1/C2 sub-screen, restart timers
  if (voltagesChangedForLCD()) {
    currentScreen = SCREEN_CELLS;
    cellSubScreen = 0;
    lastCellFlip  = now;
    lastLCDRotate = now;
    lcdInvalidate();
  }

  if ((now - lastLCDRotate) >= T_LCD_ROTATE) {
    currentScreen = (LCDScreen)((currentScreen + 1) % SCREEN_COUNT);
    lastLCDRotate = now;
    lcdInvalidate();
    if (currentScreen == SCREEN_CELLS) { cellSubScreen = 0; lastCellFlip = now; }
  }

  if (currentScreen != prevScreen) { lcdInvalidate(); prevScreen = currentScreen; }

  switch (currentScreen) {
    case SCREEN_CELLS:  screen_Cells(now); break;
    case SCREEN_PACK:   screen_Pack();     break;
    case SCREEN_SAFETY: screen_Safety();   break;
    case SCREEN_FAULTS: screen_Faults();   break;
    default: break;
  }
  updateLCDVoltageSnapshot();
}

// ================================================================
// TASK 4 — FAULT-TOLERANT EMBEDDED RUNTIME SYSTEM
// Detects sensor disconnect, frozen ADC, relay mismatch.
// 4-mode FSM: NORMAL → DEGRADED → FAILSAFE → SHUTDOWN.
// Timestamped 10-entry ring-buffer fault log with deduplication.
// ================================================================

void logFault(const char* msg) {
  unsigned long now = millis();
  bool sameMsg      = (strncmp(faults.lastMessage, msg, 51) == 0);
  bool withinWindow = (faults.totalFaults > 0 && (now - faults.lastFaultTime) < T_FAULT_DEDUP);
  if (sameMsg && withinWindow) return;

  if (faults.writeIdx == 0 && faults.totalFaults >= FAULT_LOG_MAX && !faults.logOverflowed) {
    faults.logOverflowed = true;
    Serial.println(F("[FAULT LOG] Ring buffer wrapped"));
  }

  FaultRecord& r = faults.records[faults.writeIdx];
  r.timestamp   = now;
  strncpy(r.message, msg, 51); r.message[51] = '\0';
  r.modeAtFault = runtime.mode;
  for (uint8_t i = 0; i < NUM_CELLS; i++) r.cellSnapshot[i] = cells[i].voltage;

  faults.writeIdx = (faults.writeIdx + 1) % FAULT_LOG_MAX;
  faults.totalFaults++;
  strncpy(faults.lastMessage, msg, 51); faults.lastMessage[51] = '\0';
  faults.lastFaultTime = now;
  faults.newFaultReady = true;

  Serial.print(F("[FAULT] T=")); Serial.print(r.timestamp / 1000);
  Serial.print(F("s | Mode:")); Serial.print(getModeStr());
  Serial.print(F(" | ")); Serial.println(r.message);
  Serial.print(F("  Snapshot:"));
  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    Serial.print(F(" C")); Serial.print(i+1);
    Serial.print(':'); Serial.print(r.cellSnapshot[i], 2); Serial.print('V');
  }
  Serial.println();
}

bool task4_DetectFrozenADC() {
  if (runtime.startupCycles < STARTUP_SKIP_CYCLES) return false;
  uint8_t healthy = 0, frozen = 0;
  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    if (cells[i].isFaulty) continue;
    healthy++;
    if (fabsf(cells[i].voltage - cells[i].prevVoltage) < 0.005f) frozen++;
  }
  if (healthy < 2) return false;
  if (frozen == healthy) runtime.frozenCounter++;
  else                   runtime.frozenCounter = 0;
  return (runtime.frozenCounter >= FROZEN_ADC_CYCLES);
}

bool task4_DetectRelayMismatch() {
  if (safety.relayOpen && safety.activeEvent == PROT_NONE &&
      safety.clearCycles >= RECOVERY_CYCLES && runtime.mode != MODE_SHUTDOWN) {
    safety.relayMismatchCounter++;
    if (safety.relayMismatchCounter >= RELAY_MISMATCH_LIM) {
      safety.relayMismatchCounter = 0;
      return true;
    }
  } else {
    safety.relayMismatchCounter = 0;
  }
  return false;
}

void task4_TransitionMode() {
  int target = (int)MODE_NORMAL;

  if (pack.faultyCells == 1) {
    uint8_t fc = 0;
    for (uint8_t i = 0; i < NUM_CELLS; i++) if (cells[i].isFaulty) { fc = i + 1; break; }
    char buf[52]; snprintf(buf, sizeof(buf), "Sensor fault:Cell%d isolated", fc);
    logFault(buf); target = max(target, (int)MODE_DEGRADED);
  }
  if (runtime.frozenADC)    { logFault("Frozen ADC detected");      target = max(target, (int)MODE_DEGRADED); }
  if (runtime.relayMismatch) { logFault("Relay mismatch detected"); target = max(target, (int)MODE_DEGRADED); }

  if (pack.faultyCells >= 2) {
    char buf[52]; snprintf(buf, sizeof(buf), "Multi-cell fault:%d cells", pack.faultyCells);
    logFault(buf); target = max(target, (int)MODE_FAILSAFE);
  }
  if (pack.health == HEALTH_PACK_FAILURE) {
    logFault("Pack failure:critical health"); target = max(target, (int)MODE_FAILSAFE);
  }
  if (safety.activeEvent == PROT_OVERVOLTAGE && pack.health == HEALTH_CRITICAL_IMBALANCE) {
    logFault("CRITICAL:Overvoltage+CritImb"); target = max(target, (int)MODE_SHUTDOWN);
  }
  if (pack.faultyCells >= 2 && pack.health == HEALTH_PACK_FAILURE) {
    logFault("SHUTDOWN:MultiCell+PackFail"); target = max(target, (int)MODE_SHUTDOWN);
  }

  if (target > (int)runtime.mode) {
    char buf[52];
    snprintf(buf, sizeof(buf), "Mode: %s->%s", getModeStr(),
             target==1?"DEGRADED":target==2?"FAILSAFE":"SHUTDOWN");
    runtime.mode = (SystemMode)target; runtime.recoveryCycles = 0;
    logFault(buf); lcdInvalidate();
    Serial.print(F("[RUNTIME] Escalated -> ")); Serial.println(getModeStr());
    return;
  }

  if (target < (int)runtime.mode) {
    SystemMode prevMode = runtime.mode;
    runtime.recoveryCycles++;
    uint8_t needed = (runtime.mode == MODE_SHUTDOWN) ? SHUTDOWN_RECOVERY_CYCLES : RECOVERY_CYCLES;
    if (runtime.recoveryCycles >= needed) {
      char buf[52];
      snprintf(buf, sizeof(buf), "Mode recovered->%s",
               target==0?"NORMAL":target==1?"DEGRADED":"FAILSAFE");
      runtime.mode = (SystemMode)target; runtime.recoveryCycles = 0;
      logFault(buf); lcdInvalidate();
      if (prevMode == MODE_FAILSAFE || prevMode == MODE_SHUTDOWN) {
        safety.buzzerActive = false; safety.buzzerPinState = false;
        digitalWrite(PIN_BUZZER, LOW);
      }
      Serial.print(F("[RUNTIME] Recovered -> ")); Serial.println(getModeStr());
    }
  } else {
    runtime.recoveryCycles = 0;
  }
}

void runTask4(unsigned long now) {
  if (runtime.startupCycles < STARTUP_SKIP_CYCLES) runtime.startupCycles++;
  runtime.frozenADC     = task4_DetectFrozenADC();
  runtime.relayMismatch = task4_DetectRelayMismatch();
  task4_TransitionMode();

  if (runtime.mode == MODE_SHUTDOWN) {
    if (!safety.relayOpen) {
      safety.relayOpen = true; safety.lastRelayChange = now;
      digitalWrite(PIN_RELAY, LOW);
    }
    if (!safety.buzzerActive) task2_StartBuzzer(now);
  }
}

// ================================================================
// TASK 5 — INTELLIGENT CLOUD TELEMETRY ARCHITECTURE
// Event-driven Blynk sends on state change / threshold violation.
// Offline event queue (8 slots) with reconnect replay at 300ms pace.
// WiFi self-healing every 8s. RSSI mapped 0–100%.
// ================================================================

void task5_QueueEvent() {
  if (safety.activeEvent == telemetry.lastQueuedEvent &&
      pack.health        == telemetry.lastQueuedHealth &&
      runtime.mode       == telemetry.lastQueuedMode) return;

  QueuedEvent& ev = telemetry.eventQueue[telemetry.queueTail];
  for (uint8_t i = 0; i < NUM_CELLS; i++) ev.voltages[i] = cells[i].voltage;
  ev.health = pack.health; ev.mode = runtime.mode;
  ev.event  = safety.activeEvent; ev.timestamp = millis();

  telemetry.queueTail = (telemetry.queueTail + 1) % EVENT_QUEUE_MAX;
  if (telemetry.queueCount < EVENT_QUEUE_MAX) telemetry.queueCount++;
  else telemetry.queueHead = (telemetry.queueHead + 1) % EVENT_QUEUE_MAX;

  telemetry.lastQueuedEvent  = safety.activeEvent;
  telemetry.lastQueuedHealth = pack.health;
  telemetry.lastQueuedMode   = runtime.mode;
}

bool task5_StateChanged() {
  if (pack.health        != telemetry.lastHealth) return true;
  if (runtime.mode       != telemetry.lastMode)   return true;
  if (safety.activeEvent != telemetry.lastEvent)  return true;
  for (uint8_t i = 0; i < NUM_CELLS; i++)
    if (fabsf(cells[i].voltage - telemetry.lastVoltages[i]) > 0.05f) return true;
  return false;
}

bool task5_ThresholdViolation() {
  for (uint8_t i = 0; i < NUM_CELLS; i++)
    if (!cells[i].isFaulty && (cells[i].voltage < V_WARN || cells[i].voltage > V_OVERVOLT))
      return true;
  return false;
}

int task5_SignalQuality() {
  if (WiFi.status() != WL_CONNECTED) return 0;
  return (int)constrain(map((long)WiFi.RSSI(), -100L, -50L, 0L, 100L), 0L, 100L);
}

void task5_PushToBlynk() {
  if (!blynkOK) return;

  Blynk.virtualWrite(V0, cells[0].voltage); Blynk.virtualWrite(V1, cells[1].voltage);
  Blynk.virtualWrite(V2, cells[2].voltage); Blynk.virtualWrite(V3, cells[3].voltage);
  float sendImb = (pack.imbalancePct > 100.0f) ? 100.0f : pack.imbalancePct;
  Blynk.virtualWrite(V4, pack.avgVoltage); Blynk.virtualWrite(V5, sendImb);
  Blynk.virtualWrite(V6, getHealthStr()); Blynk.virtualWrite(V7, getModeStr());
  Blynk.virtualWrite(V8, safety.relayOpen ? "OPEN (Protection ON)" : "CLOSED (Normal)");
  Blynk.virtualWrite(V9, getProtectionStr(safety.activeEvent));
  Blynk.virtualWrite(V10, (int)min((uint32_t)9999, faults.totalFaults));
  char wk[12], st[12];
  snprintf(wk, sizeof(wk), "Cell %d", pack.weakestCell);
  snprintf(st, sizeof(st), "Cell %d", pack.strongestCell);
  Blynk.virtualWrite(V11, pack.weakestCell   > 0 ? wk : "None");
  Blynk.virtualWrite(V15, pack.strongestCell > 0 ? st : "None");
  Blynk.virtualWrite(V12, telemetry.rssiPercent);
  Blynk.virtualWrite(V13, task6_GetRecommendation());
  Blynk.virtualWrite(V14, (int)task6_GetRiskLevel());
  Blynk.virtualWrite(V18, cells[0].voltage); Blynk.virtualWrite(V19, cells[1].voltage);
  Blynk.virtualWrite(V20, cells[2].voltage); Blynk.virtualWrite(V21, cells[3].voltage);

  for (uint8_t i = 0; i < NUM_CELLS; i++) telemetry.lastVoltages[i] = cells[i].voltage;
  telemetry.lastHealth = pack.health; telemetry.lastMode = runtime.mode;
  telemetry.lastEvent  = safety.activeEvent; telemetry.lastSendTime = millis();

  if (faults.newFaultReady) {
    uint8_t last = (faults.writeIdx == 0) ? FAULT_LOG_MAX - 1 : faults.writeIdx - 1;
    FaultRecord& r = faults.records[last];
    String entry = "[" + String(r.timestamp / 1000) + "s][" +
                   String(r.modeAtFault==0?"NORM":r.modeAtFault==1?"DEGD":r.modeAtFault==2?"SAFE":"SHTD") +
                   "] " + String(r.message) + "\n";
    Blynk.virtualWrite(V16, entry);
    Blynk.virtualWrite(V17, (int)min((uint32_t)9999, faults.totalFaults));
    faults.newFaultReady = false;
  }

  Serial.print(F("[TELEMETRY] Sent | RSSI:")); Serial.print(telemetry.rssiPercent);
  Serial.print(F("% | ")); Serial.print(getHealthStr());
  Serial.print(F(" | ")); Serial.print(getModeStr());
  Serial.print(F(" | ")); Serial.println(getProtectionStr(safety.activeEvent));
}

void task5_ReplayStep(unsigned long now) {
  if (!telemetry.replayActive) return;
  if ((now - telemetry.lastReplayTime) < T_QUEUE_REPLAY) return;

  if (telemetry.queueCount == 0) {
    telemetry.replayActive = false;
    Serial.println(F("[TELEMETRY] Queue sync done — pushing live"));
    task5_PushToBlynk(); return;
  }

  QueuedEvent& ev = telemetry.eventQueue[telemetry.queueHead];
  Blynk.virtualWrite(V0, ev.voltages[0]); Blynk.virtualWrite(V1, ev.voltages[1]);
  Blynk.virtualWrite(V2, ev.voltages[2]); Blynk.virtualWrite(V3, ev.voltages[3]);
  Blynk.virtualWrite(V18, ev.voltages[0]); Blynk.virtualWrite(V19, ev.voltages[1]);
  Blynk.virtualWrite(V20, ev.voltages[2]); Blynk.virtualWrite(V21, ev.voltages[3]);
  Blynk.virtualWrite(V6, ev.health==0?"HEALTHY":ev.health==1?"MINOR IMBALANCE":ev.health==2?"CRITICAL IMBAL.":"PACK FAILURE");
  Blynk.virtualWrite(V7, ev.mode==0?"NORMAL":ev.mode==1?"DEGRADED":ev.mode==2?"FAILSAFE":"SHUTDOWN");
  Blynk.virtualWrite(V9, getProtectionStr(ev.event));

  Serial.print(F("  [QREPLAY] T=")); Serial.print(ev.timestamp / 1000);
  Serial.print(F("s H:")); Serial.print(ev.health);
  Serial.print(F(" M:")); Serial.println(ev.mode);

  telemetry.queueHead = (telemetry.queueHead + 1) % EVENT_QUEUE_MAX;
  telemetry.queueCount--;
  telemetry.lastReplayTime = now;
}

void runTask5(unsigned long now) {
  telemetry.rssiPercent = (WiFi.status() == WL_CONNECTED) ? task5_SignalQuality() : 0;

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiOK) { wifiOK = false; blynkOK = false; telemetry.offlineMode = true;
                  Serial.println(F("[WiFi] Disconnected — offline mode")); }
    if ((now - lastWiFiRetry) >= T_WIFI_RETRY) {
      lastWiFiRetry = now; WiFi.reconnect();
      Serial.println(F("[WiFi] Reconnect attempt..."));
    }
    return;
  }

  if (!wifiOK) {
    wifiOK = true; telemetry.offlineMode = false;
    Serial.print(F("[WiFi] Connected: ")); Serial.println(WiFi.localIP());
    if (!blynkConfigured) { Blynk.config(BLYNK_AUTH_TOKEN); blynkConfigured = true; }
    Blynk.connect(3000);
    if (telemetry.queueCount > 0) {
      telemetry.replayActive = true; telemetry.lastReplayTime = now;
      Serial.print(F("[TELEMETRY] Syncing ")); Serial.print(telemetry.queueCount);
      Serial.println(F(" queued events..."));
    }
  }

  if ((now - lastBlynkRun) >= T_BLYNK_RUN) { lastBlynkRun = now; Blynk.run(); }
  blynkOK = Blynk.connected();
  if (!blynkOK) return;

  if (telemetry.replayActive) { task5_ReplayStep(now); return; }

  bool shouldSend = task5_StateChanged() || task5_ThresholdViolation() ||
                    (safety.activeEvent != PROT_NONE);
  if (shouldSend && (now - telemetry.lastSendTime) >= T_BLYNK_MIN)
    task5_PushToBlynk();
}

// ================================================================
// TASK 6 — EXECUTIVE BATTERY INTELLIGENCE DASHBOARD
// Risk level 0–3, 9-scenario recommendation engine.
// ================================================================

String task6_GetRecommendation() {
  if (runtime.mode == MODE_SHUTDOWN)              return F("SHUTDOWN: Power off. Inspect all cells.");
  if (runtime.mode == MODE_FAILSAFE)              return "FAILSAFE: Disconnect load. Check C" + String(pack.weakestCell) + ".";
  if (safety.activeEvent == PROT_SENSOR_FAULT)    return F("SENSOR FAULT: Check ADC wiring/pot.");
  if (safety.activeEvent == PROT_OVERVOLTAGE)     return F("OVERVOLTAGE: Reduce pot. Stop charging.");
  if (safety.activeEvent == PROT_RAPID_CHANGE)    return "RAPID CHANGE on C" + String(pack.weakestCell) + ". Check connections.";
  if (safety.activeEvent == PROT_WEAK_CELL)       return "WEAK CELL C" + String(pack.weakestCell) + " at " + String(pack.minVoltage, 2) + "V. Balance.";
  if (pack.health == HEALTH_CRITICAL_IMBALANCE)   return "CRIT IMB: " + String(pack.imbalancePct, 1) + "%. Balance now.";
  if (pack.health == HEALTH_MINOR_IMBALANCE)      return "MINOR IMB: " + String(pack.imbalancePct, 1) + "%. Schedule balance.";
  if (runtime.mode == MODE_DEGRADED)              return F("DEGRADED: Reduce load 50%. Run diag.");
  return "HEALTHY: " + String(pack.avgVoltage, 2) + "V Imb:" + String(pack.imbalancePct, 1) + "% OK.";
}

uint8_t task6_GetRiskLevel() {
  if (runtime.mode == MODE_SHUTDOWN  || pack.health == HEALTH_PACK_FAILURE)       return 3;
  if (runtime.mode == MODE_FAILSAFE  || pack.health == HEALTH_CRITICAL_IMBALANCE) return 2;
  if (runtime.mode == MODE_DEGRADED  || pack.health == HEALTH_MINOR_IMBALANCE)    return 1;
  return 0;
}

// ================================================================
// SERIAL DASHBOARD — printed every 500ms sensor cycle
// ================================================================
void printSerialDashboard() {
  serialLine();
  Serial.println(F("  SMART BMS — LIVE DASHBOARD"));
  Serial.println(F("  Srishanth G | ElevanceSkills Technology Pvt Ltd"));
  serialLine();

  Serial.println(F("  CELL VOLTAGES:"));
  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    Serial.print(F("    Cell ")); Serial.print(i + 1);
    Serial.print(F(": ")); Serial.print(cells[i].voltage, 2); Serial.print(F("V  "));
    if      (cells[i].isFaulty)      Serial.print(F("[SENSOR FAULT]"));
    else if (cells[i].isOverVoltage) Serial.print(F("[OVERVOLTAGE ]"));
    else if (cells[i].isWeak)        Serial.print(F("[WEAK / LOW  ]"));
    else                             Serial.print(F("[OK          ]"));
    if (pack.weakestCell   > 0 && i+1 == pack.weakestCell)   Serial.print(F(" <-WEAKEST"));
    if (pack.strongestCell > 0 && i+1 == pack.strongestCell) Serial.print(F(" <-STRONGEST"));
    Serial.println();
  }
  serialDash();

  Serial.println(F("  PACK ANALYTICS:"));
  Serial.print(F("    Avg Voltage  : ")); Serial.print(pack.avgVoltage, 2);  Serial.println(F(" V"));
  Serial.print(F("    Min / Max    : ")); Serial.print(pack.minVoltage, 2);
  Serial.print(F("V / "));                Serial.print(pack.maxVoltage, 2);  Serial.println(F("V"));
  Serial.print(F("    Imbalance    : ")); Serial.print(pack.imbalancePct, 2); Serial.println(F(" %"));
  Serial.print(F("    Health       : ")); Serial.println(getHealthStr());
  Serial.print(F("    Faulty Cells : ")); Serial.print(pack.faultyCells);
  Serial.print(F(" / "));                  Serial.println(NUM_CELLS);
  serialDash();

  Serial.println(F("  SYSTEM STATUS:"));
  Serial.print(F("    Mode          : ")); Serial.println(getModeStr());
  Serial.print(F("    Protection    : ")); Serial.println(getProtectionStr(safety.activeEvent));
  Serial.print(F("    Relay         : ")); Serial.println(safety.relayOpen ? "OPEN (Protection ACTIVE)" : "CLOSED (Normal)");
  Serial.print(F("    Buzzer        : ")); Serial.println(safety.buzzerActive ? "ACTIVE (Beeping)" : "Silent");
  Serial.print(F("    Frozen ADC    : ")); Serial.println(runtime.frozenADC    ? "DETECTED" : "OK");
  Serial.print(F("    Relay Mismatch: ")); Serial.println(runtime.relayMismatch ? "DETECTED" : "OK");
  serialDash();

  Serial.println(F("  CLOUD TELEMETRY:"));
  Serial.print(F("    WiFi Signal  : ")); Serial.print(telemetry.rssiPercent); Serial.println(F(" %"));
  Serial.print(F("    Blynk        : ")); Serial.println(blynkOK ? "CONNECTED" : "OFFLINE");
  Serial.print(F("    Offline Mode : ")); Serial.println(telemetry.offlineMode ? "YES" : "NO");
  Serial.print(F("    Queue Count  : ")); Serial.println(telemetry.queueCount);
  serialDash();

  Serial.println(F("  EXECUTIVE DASHBOARD:"));
  Serial.print(F("    Total Faults  : ")); Serial.println(faults.totalFaults);
  uint8_t rl = task6_GetRiskLevel();
  Serial.print(F("    Risk Level    : ")); Serial.print(rl);
  Serial.print(F(" (")); Serial.print(getRiskLabel(rl)); Serial.println(F(")"));
  Serial.print(F("    Recommendation: ")); Serial.println(task6_GetRecommendation());
  serialLine();
  Serial.println();
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  serialLine();
  Serial.println(F("  SMART BMS — STARTING"));
  Serial.println(F("  Srishanth G | ElevanceSkills Technology Pvt Ltd"));
  serialLine();

  pinMode(PIN_RELAY,  OUTPUT); digitalWrite(PIN_RELAY,  HIGH);
  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);

  lcd.init();
  lcd.backlight();
  lcdWrite("  SMART BMS     ", "  Srishanth G   ");
  delay(1200);
  lcdWrite("ElevanceSkills  ", "  Loading...    ");
  delay(600);

  memset(cells,      0, sizeof(cells));
  memset(&pack,      0, sizeof(pack));
  memset(&safety,    0, sizeof(safety));
  memset(&faults,    0, sizeof(faults));
  memset(&runtime,   0, sizeof(runtime));
  memset(&telemetry, 0, sizeof(telemetry));

  safety.clearCycles         = RECOVERY_CYCLES;
  telemetry.lastQueuedEvent  = PROT_NONE;
  telemetry.lastQueuedHealth = HEALTH_HEALTHY;
  telemetry.lastQueuedMode   = MODE_NORMAL;

  Serial.println(F("[WiFi] Connecting to Wokwi-GUEST..."));
  lcdWrite("WiFi:Connecting ", "Please wait...  ");
  WiFi.begin(ssid, pass);

  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 12) { delay(500); Serial.print('.'); tries++; }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiOK = true;
    Serial.print(F("[WiFi] Connected | IP: ")); Serial.println(WiFi.localIP());
    Serial.print(F("[WiFi] RSSI: ")); Serial.print(WiFi.RSSI()); Serial.println(F(" dBm"));
    char ipStr[17];
    snprintf(ipStr, sizeof(ipStr), "%-16s", WiFi.localIP().toString().c_str());
    lcdWrite("WiFi: Connected!", ipStr);
    delay(600);
    Blynk.config(BLYNK_AUTH_TOKEN);
    blynkConfigured = true;
    Blynk.connect(3000);
    blynkOK = Blynk.connected();
    Serial.print(F("[Blynk] ")); Serial.println(blynkOK ? "Connected!" : "Not connected yet");
    lcdWrite("Blynk Cloud:    ", blynkOK ? "  Connected!    " : "  Offline Mode  ");
    delay(600);
  } else {
    Serial.println(F("[WiFi] Failed — BMS running offline"));
    lcdWrite("WiFi: Offline   ", "BMS Running OK  ");
    delay(600);
  }

  lcdWrite("  BMS  ONLINE   ", "  Mode: NORMAL  ");
  delay(500);

  unsigned long t = millis();
  lastSensorTime = lastLCDRefresh = lastLCDRotate =
  lastCellFlip   = lastWiFiRetry  = lastBlynkRun  = t;

  lcdInvalidate();
  prevScreen = SCREEN_COUNT;

  Serial.println(F("\n[BMS] All 6 tasks active — ready\n"));
}

// ================================================================
// MAIN LOOP — fully non-blocking, zero delay()
// Continuous: buzzer timing, Blynk telemetry
// 500ms:      sensor read, safety, fault detection, LCD on pot change
// 800ms:      periodic LCD refresh and screen rotation
// ================================================================
void loop() {
  unsigned long now = millis();

  task2_ControlBuzzer(now);  // continuous — must run every iteration for precise timing

  if ((now - lastSensorTime) >= T_SENSOR) {
    lastSensorTime = now;
    runTask1();
    runTask2(now);
    runTask4(now);
    if (telemetry.offlineMode || task5_StateChanged()) task5_QueueEvent();
    if (voltagesChangedForLCD()) { lastLCDRefresh = now; runTask3(now); }
    printSerialDashboard();
  }

  if ((now - lastLCDRefresh) >= T_LCD_REFRESH) {
    lastLCDRefresh = now;
    runTask3(now);
  }

  runTask5(now);
}
