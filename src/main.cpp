#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// Zunked parking monitor configuration. Replace the Wi-Fi placeholders before use.
constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
constexpr char FALLBACK_AP_SSID[] = "Zunked-Setup";
constexpr char FALLBACK_AP_PASSWORD[] = "parking123";

constexpr uint8_t PARKING_SLOT_COUNT = 4;
constexpr uint8_t ULTRASONIC_TRIG_PINS[PARKING_SLOT_COUNT] = {13, 14, 16, 17};
constexpr uint8_t ULTRASONIC_ECHO_PINS[PARKING_SLOT_COUNT] = {34, 35, 36, 39};
constexpr uint16_t OCCUPANCY_DISTANCE_CM = 20;
constexpr uint16_t RELEASE_DISTANCE_CM = OCCUPANCY_DISTANCE_CM + 5;
constexpr uint8_t DISTANCE_FILTER_SAMPLES = 3;
constexpr uint8_t OCCUPANCY_DEBOUNCE_SAMPLES = 3;

// Demo mode cycles through plausible parking patterns without requiring sensors.
constexpr bool SIMULATION_MODE = true;
constexpr bool USE_GROUND_SENSORS = true;
constexpr uint8_t GROUND_SENSOR_PINS[PARKING_SLOT_COUNT] = {32, 33, 25, 26};
constexpr uint32_t GROUND_SENSOR_SAMPLE_MS = 50;
constexpr uint8_t GATE_SERVO_PIN = 27;
constexpr uint8_t GATE_CLOSED_ANGLE = 0;
constexpr uint8_t GATE_OPEN_ANGLE = 90;
constexpr uint16_t GATE_SERVO_MIN_PULSE_US = 500;
constexpr uint16_t GATE_SERVO_MAX_PULSE_US = 2400;
constexpr uint32_t GATE_SERVO_STEP_MS = 20;
constexpr uint8_t GATE_SERVO_STEP_DEGREES = 2;
constexpr uint32_t GATE_OPEN_HOLD_MS = 5000;
constexpr uint32_t SIMULATION_SAMPLE_MS = 500;
constexpr uint32_t SIMULATION_PATTERN_MS = 6000;
constexpr uint32_t SENSOR_TIMEOUT_US = 30000;
constexpr uint32_t SENSOR_START_GAP_MS = 5;
constexpr uint32_t WIFI_TIMEOUT_MS = 15000;
constexpr uint32_t DEBUG_LOG_MS = 2000;

// GPIO 34, 35, 36 and 39 are input-only and are used for HC-SR04 Echo.
// Never connect a 5 V Echo output directly: use a divider to keep it at 3.3 V.

const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <meta name="color-scheme" content="dark">
  <title>Zunked Parking</title>
  <style>
    :root { color-scheme: dark; --bg:#08111f; --panel:#111d30; --panel-hi:#17263e; --text:#f2f6fc; --muted:#a5b4ca; --line:#2a3b56; --green:#44d7a2; --red:#ff7580; --blue:#86bfff; }
    * { box-sizing:border-box; }
    body { margin:0; min-height:100vh; background:radial-gradient(ellipse at 15% 0%,#172d49 0%,var(--bg) 52%); color:var(--text); font:16px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif; }
    header, main, footer { width:min(100% - 32px,1000px); margin-inline:auto; }
    header { padding:38px 0 14px; }
    h1 { margin:0; font-size:clamp(2rem,7vw,3.2rem); letter-spacing:-.04em; line-height:1.1; }
    h2 { margin:30px 0 14px; font-size:1.15rem; }
    .eyebrow { margin:0 0 8px; color:var(--blue); font-weight:700; letter-spacing:.12em; text-transform:uppercase; font-size:.78rem; }
    .lede { color:var(--muted); margin:10px 0 0; }
    .connection { display:flex; align-items:center; gap:9px; margin:18px 0 0; color:var(--muted); }
    .dot { width:10px; height:10px; border-radius:50%; background:var(--red); box-shadow:0 0 0 4px #ff75801c; }
    .dot.online { background:var(--green); box-shadow:0 0 0 4px #44d7a21c; }
    .summary { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:12px; }
    .metric { background:linear-gradient(145deg,var(--panel-hi),var(--panel)); border:1px solid var(--line); border-radius:16px; padding:16px 18px; }
    .metric strong { display:block; font-size:clamp(1.8rem,6vw,2.35rem); line-height:1.15; }
    .metric span { color:var(--muted); font-size:.9rem; }
    .slots { display:grid; grid-template-columns:repeat(auto-fit,minmax(min(100%,220px),1fr)); gap:14px; }
    .slot { background:var(--panel); border:1px solid var(--line); border-left:5px solid var(--green); border-radius:15px; padding:17px; min-width:0; }
    .slot.occupied { border-left-color:var(--red); }
    .slot h3 { margin:0 0 16px; font-size:1rem; color:var(--muted); }
    .slot-state { font-size:1.35rem; font-weight:750; letter-spacing:-.02em; }
    .slot:not(.occupied) .slot-state { color:var(--green); }
    .slot.occupied .slot-state { color:var(--red); }
    .distance { margin:8px 0 0; color:var(--muted); font-variant-numeric:tabular-nums; }
    .sensor-detail, .confidence { margin:7px 0 0; color:var(--muted); font-size:.86rem; }
    .confidence strong { color:var(--blue); text-transform:capitalize; }
    .gate-panel { display:flex; flex-wrap:wrap; align-items:center; gap:14px 24px; padding:17px; border:1px solid var(--line); border-radius:15px; background:var(--panel); }
    .gate-panel strong { display:block; font-size:1.2rem; }
    .gate-panel span { color:var(--muted); font-size:.88rem; }
    .mode { display:inline-flex; margin-top:14px; padding:4px 9px; border:1px solid var(--line); border-radius:999px; color:var(--blue); font-size:.78rem; }
    footer { padding:28px 0 36px; color:var(--muted); font-size:.85rem; }
    :focus-visible { outline:3px solid var(--blue); outline-offset:3px; }
    @media (min-width:620px) { header, main, footer { width:min(100% - 48px,1000px); } .summary { grid-template-columns:repeat(2,minmax(180px,240px)); } }
    @media (prefers-reduced-motion:reduce) { *,*::before,*::after { scroll-behavior:auto !important; animation-duration:.01ms !important; transition-duration:.01ms !important; } }
  </style>
</head>
<body>
  <header>
    <p class="eyebrow">Smart parking prototype</p>
    <h1>Zunked</h1>
    <p class="lede">Live availability for your parking area.</p>
    <p class="connection" aria-live="polite"><span id="connection-dot" class="dot" aria-hidden="true"></span><span id="connection-text">Connecting to ESP32…</span></p>
  </header>
  <main>
    <section aria-labelledby="summary-heading">
      <h2 id="summary-heading">Parking overview</h2>
      <div class="summary">
        <div class="metric"><strong id="free-count">—</strong><span>Available</span></div>
        <div class="metric"><strong id="occupied-count">—</strong><span>Occupied</span></div>
      </div>
    </section>
    <section aria-labelledby="slots-heading">
      <h2 id="slots-heading">Parking slots</h2>
      <div id="slot-cards" class="slots" aria-live="off"></div>
    </section>
    <section aria-labelledby="gate-heading">
      <h2 id="gate-heading">Entry / exit gate</h2>
      <div class="gate-panel">
        <div><span>Gate</span><strong id="gate-state">Waiting</strong></div>
        <div><span>Last vehicle event</span><strong id="gate-event">—</strong></div>
        <div><span>Position</span><strong id="gate-angle">—</strong></div>
      </div>
    </section>
    <p id="mode-label" class="mode">Waiting for status</p>
  </main>
  <footer>Updates automatically every second · Local ESP32 dashboard</footer>
  <script>
    'use strict';
    const slotCards = document.querySelector('#slot-cards');
    const freeCount = document.querySelector('#free-count');
    const occupiedCount = document.querySelector('#occupied-count');
    const connectionText = document.querySelector('#connection-text');
    const connectionDot = document.querySelector('#connection-dot');
    const modeLabel = document.querySelector('#mode-label');
    const gateState = document.querySelector('#gate-state');
    const gateEvent = document.querySelector('#gate-event');
    const gateAngle = document.querySelector('#gate-angle');

    function setConnection(online, text) {
      connectionDot.classList.toggle('online', online);
      connectionText.textContent = text;
    }

    function renderStatus(payload) {
      if (!Array.isArray(payload.slots)) throw new TypeError('Invalid parking status');
      slotCards.replaceChildren();
      let occupied = 0;
      payload.slots.forEach((slot, index) => {
        const isOccupied = Boolean(slot.occupied);
        if (isOccupied) occupied += 1;
        const card = document.createElement('article');
        card.className = `slot${isOccupied ? ' occupied' : ''}`;
        const title = document.createElement('h3');
        title.textContent = `Slot ${index + 1}`;
        const state = document.createElement('p');
        state.className = 'slot-state';
        state.textContent = isOccupied ? 'Occupied' : 'Available';
        const ultrasonic = slot.ultrasonic || {};
        const magnetic = slot.magnetic || {};
        const distance = document.createElement('p');
        distance.className = 'distance';
        distance.textContent = ultrasonic.valid && ultrasonic.distanceCm >= 0 ? `Ultrasonic: ${ultrasonic.distanceCm} cm · ${ultrasonic.occupied ? 'occupied' : 'clear'}` : 'Ultrasonic: no reading';
        const ground = document.createElement('p');
        ground.className = 'sensor-detail';
        ground.textContent = magnetic.enabled ? `Magnetic: ${magnetic.valid ? (magnetic.occupied ? 'occupied' : 'clear') : 'waiting'}` : 'Magnetic: disabled';
        const confidence = document.createElement('p');
        confidence.className = 'confidence';
        confidence.append('Sensor confidence: ');
        const confidenceValue = document.createElement('strong');
        confidenceValue.textContent = slot.confidence || 'none';
        confidence.append(confidenceValue, ` · ${slot.sensorState || 'waiting'}`);
        card.append(title, state, distance, ground, confidence);
        slotCards.append(card);
      });
      occupiedCount.textContent = String(occupied);
      freeCount.textContent = String(payload.slots.length - occupied);
      modeLabel.textContent = payload.simulationMode ? 'Simulation demo' : 'Physical sensors';
      if (payload.gate) {
        gateState.textContent = payload.gate.state || 'Unknown';
        gateEvent.textContent = payload.gate.lastEvent || '—';
        gateAngle.textContent = `${payload.gate.angle ?? 0}°`;
      }
    }

    async function pollStatus() {
      try {
        const response = await fetch('/api/status', { cache: 'no-store' });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        renderStatus(await response.json());
        setConnection(true, 'Connected to Zunked controller');
      } catch (error) {
        setConnection(false, 'Controller unavailable; retrying…');
      } finally {
        window.setTimeout(pollStatus, 1000);
      }
    }

    pollStatus();
  </script>
</body>
</html>
)HTML";

AsyncWebServer server(80);

struct SlotState {
  bool occupied = false;
  bool candidateOccupied = false;
  uint8_t stableSamples = 0;
  bool ultrasonicOccupied = false;
  bool ultrasonicCandidate = false;
  bool ultrasonicValid = false;
  uint8_t ultrasonicStableSamples = 0;
  bool magneticOccupied = false;
  bool magneticCandidate = false;
  bool magneticValid = false;
  uint8_t magneticStableSamples = 0;
  uint8_t historyCount = 0;
  uint8_t historyNext = 0;
  int16_t distanceCm = -1;
  int16_t distanceHistory[DISTANCE_FILTER_SAMPLES] = {};
};

SlotState slots[PARKING_SLOT_COUNT];
enum class MeasurePhase : uint8_t { Idle, TriggerHigh, WaitingForRise, WaitingForFall };
MeasurePhase measurePhase = MeasurePhase::Idle;
uint8_t measuringSlot = 0;
uint32_t phaseStartedUs = 0;
uint32_t echoStartedUs = 0;
uint32_t nextMeasurementMs = 0;
uint32_t wifiStartedMs = 0;
uint32_t lastGroundSampleMs = 0;
uint32_t lastDebugLogMs = 0;
uint32_t nextSimulationSampleMs = 0;
uint32_t nextSimulationPatternMs = 0;
uint8_t simulationPatternIndex = 0;
bool apMode = false;
bool wifiReadyLogged = false;
Servo gateServo;
bool gateServoAttached = false;
enum class GateMotion : uint8_t { Closed, Opening, Open, Closing };
GateMotion gateMotion = GateMotion::Closed;
uint8_t gateAngle = GATE_CLOSED_ANGLE;
uint32_t gateLastStepMs = 0;
uint32_t gateCloseDeadlineMs = 0;
bool occupancyBaselineReady = false;
uint8_t previousOccupiedSlots = 0;
const char *lastGateEvent = "None";

constexpr uint8_t SIMULATION_PATTERNS[] = {
  0b0000, 0b0001, 0b0101, 0b0111, 0b0110, 0b1110, 0b1010, 0b1000, 0b0000
};
constexpr uint8_t SIMULATION_PATTERN_COUNT = sizeof(SIMULATION_PATTERNS) / sizeof(SIMULATION_PATTERNS[0]);

void updateDebouncedSignal(bool sample, bool &candidate, bool &state, uint8_t &stableSamples) {
  if (sample == candidate) {
    if (stableSamples < OCCUPANCY_DEBOUNCE_SAMPLES) ++stableSamples;
  } else {
    candidate = sample;
    stableSamples = 1;
  }
  if (stableSamples >= OCCUPANCY_DEBOUNCE_SAMPLES) state = candidate;
}

const char *slotConfidence(const SlotState &slot) {
  if (slot.ultrasonicValid && slot.magneticValid)
    return slot.ultrasonicOccupied == slot.magneticOccupied ? "high" : "medium";
  if (slot.ultrasonicValid || slot.magneticValid) return "low";
  return "none";
}

const char *slotSensorState(const SlotState &slot) {
  if (slot.ultrasonicValid && slot.magneticValid)
    return slot.ultrasonicOccupied == slot.magneticOccupied ? "sensors_agree" : "sensors_disagree";
  if (slot.ultrasonicValid) return "ultrasonic_only";
  if (slot.magneticValid) return "magnetic_only";
  return "waiting";
}

void updateFusedOccupancy(SlotState &slot) {
  if (!slot.ultrasonicValid && !slot.magneticValid) return;
  const bool fusedOccupied = (slot.ultrasonicValid && slot.ultrasonicOccupied) ||
                             (slot.magneticValid && slot.magneticOccupied);
  if (fusedOccupied == slot.candidateOccupied) {
    if (slot.stableSamples < OCCUPANCY_DEBOUNCE_SAMPLES) ++slot.stableSamples;
  } else {
    slot.candidateOccupied = fusedOccupied;
    slot.stableSamples = 1;
  }
  if (slot.stableSamples >= OCCUPANCY_DEBOUNCE_SAMPLES) slot.occupied = slot.candidateOccupied;
}

int16_t medianDistance(const SlotState &slot) {
  int16_t sorted[DISTANCE_FILTER_SAMPLES];
  for (uint8_t i = 0; i < slot.historyCount; ++i) sorted[i] = slot.distanceHistory[i];
  for (uint8_t i = 1; i < slot.historyCount; ++i) {
    const int16_t value = sorted[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && sorted[j] > value) {
      sorted[j + 1] = sorted[j];
      --j;
    }
    sorted[j + 1] = value;
  }
  return sorted[slot.historyCount / 2];
}

void acceptDistance(SlotState &slot, int16_t distanceCm) {
  if (distanceCm < 1 || distanceCm > 500) {
    slot.ultrasonicValid = false;
    slot.historyCount = 0;
    slot.historyNext = 0;
    slot.ultrasonicStableSamples = 0;
    updateFusedOccupancy(slot);
    return;
  }
  slot.distanceHistory[slot.historyNext] = distanceCm;
  slot.historyNext = (slot.historyNext + 1) % DISTANCE_FILTER_SAMPLES;
  if (slot.historyCount < DISTANCE_FILTER_SAMPLES) ++slot.historyCount;
  slot.distanceCm = medianDistance(slot);
  slot.ultrasonicValid = true;

  const uint16_t threshold = slot.ultrasonicOccupied ? RELEASE_DISTANCE_CM : OCCUPANCY_DISTANCE_CM;
  updateDebouncedSignal(slot.distanceCm < threshold, slot.ultrasonicCandidate,
                        slot.ultrasonicOccupied, slot.ultrasonicStableSamples);
  updateFusedOccupancy(slot);
}

void finishMeasurement(SlotState &slot, int16_t distanceCm) {
  if (distanceCm >= 0) acceptDistance(slot, distanceCm);
  else {
    slot.ultrasonicValid = false;
    slot.historyCount = 0;
    slot.historyNext = 0;
    slot.ultrasonicStableSamples = 0;
    updateFusedOccupancy(slot);
  }
  measuringSlot = (measuringSlot + 1) % PARKING_SLOT_COUNT;
  measurePhase = MeasurePhase::Idle;
  nextMeasurementMs = millis() + SENSOR_START_GAP_MS;
}

void serviceUltrasonic() {
  const uint32_t nowUs = micros();
  const uint8_t trigPin = ULTRASONIC_TRIG_PINS[measuringSlot];
  const uint8_t echoPin = ULTRASONIC_ECHO_PINS[measuringSlot];
  if (measurePhase == MeasurePhase::Idle) {
    if (static_cast<int32_t>(millis() - nextMeasurementMs) < 0) return;
    digitalWrite(trigPin, HIGH);
    phaseStartedUs = nowUs;
    measurePhase = MeasurePhase::TriggerHigh;
    return;
  }
  if (measurePhase == MeasurePhase::TriggerHigh) {
    if (nowUs - phaseStartedUs < 12) return;
    digitalWrite(trigPin, LOW);
    phaseStartedUs = nowUs;
    measurePhase = MeasurePhase::WaitingForRise;
    return;
  }
  if (measurePhase == MeasurePhase::WaitingForRise) {
    if (digitalRead(echoPin) == HIGH) {
      echoStartedUs = nowUs;
      measurePhase = MeasurePhase::WaitingForFall;
    } else if (nowUs - phaseStartedUs >= SENSOR_TIMEOUT_US) finishMeasurement(slots[measuringSlot], -1);
    return;
  }
  if (digitalRead(echoPin) == LOW) {
    const int16_t distanceCm = static_cast<int16_t>((nowUs - echoStartedUs) / 58UL);
    finishMeasurement(slots[measuringSlot], distanceCm);
  } else if (nowUs - echoStartedUs >= SENSOR_TIMEOUT_US) finishMeasurement(slots[measuringSlot], -1);
}

void serviceGroundSensors() {
  if (!USE_GROUND_SENSORS || millis() - lastGroundSampleMs < GROUND_SENSOR_SAMPLE_MS) return;
  lastGroundSampleMs = millis();
  for (uint8_t i = 0; i < PARKING_SLOT_COUNT; ++i) {
    SlotState &slot = slots[i];
    const bool sampleOccupied = digitalRead(GROUND_SENSOR_PINS[i]) == LOW;
    updateDebouncedSignal(sampleOccupied, slot.magneticCandidate, slot.magneticOccupied,
                          slot.magneticStableSamples);
    slot.magneticValid = true;
    updateFusedOccupancy(slot);
  }
}

void serviceSimulation() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - nextSimulationPatternMs) >= 0) {
    simulationPatternIndex = (simulationPatternIndex + 1) % SIMULATION_PATTERN_COUNT;
    nextSimulationPatternMs = now + SIMULATION_PATTERN_MS;
  }
  if (static_cast<int32_t>(now - nextSimulationSampleMs) < 0) return;
  nextSimulationSampleMs = now + SIMULATION_SAMPLE_MS;
  const uint8_t pattern = SIMULATION_PATTERNS[simulationPatternIndex];
  for (uint8_t i = 0; i < PARKING_SLOT_COUNT; ++i) {
    SlotState &slot = slots[i];
    const bool occupied = (pattern & (1U << i)) != 0;
    const uint8_t variation = static_cast<uint8_t>((now / 1000 + i * 7) % 9);
    slot.distanceCm = occupied ? 12 + variation : 42 + variation * 3;
    slot.ultrasonicValid = true;
    slot.magneticValid = true;
    updateDebouncedSignal(occupied, slot.ultrasonicCandidate, slot.ultrasonicOccupied,
                          slot.ultrasonicStableSamples);
    updateDebouncedSignal(occupied, slot.magneticCandidate, slot.magneticOccupied,
                          slot.magneticStableSamples);
    updateFusedOccupancy(slot);
  }
}

const char *gateStateName() {
  switch (gateMotion) {
    case GateMotion::Opening: return "Opening";
    case GateMotion::Open: return "Open";
    case GateMotion::Closing: return "Closing";
    default: return "Closed";
  }
}

void startGateCycle(bool entryArrival) {
  lastGateEvent = entryArrival ? "Entry arrival" : "Exit departure";
  if (gateMotion == GateMotion::Open) {
    gateCloseDeadlineMs = millis() + GATE_OPEN_HOLD_MS;
    return;
  }
  gateMotion = GateMotion::Opening;
  Serial.printf("Gate opening for %s.\n", lastGateEvent);
}

void serviceVehicleEvents() {
  uint8_t occupiedNow = 0;
  for (const SlotState &slot : slots) occupiedNow += slot.occupied ? 1 : 0;
  if (!occupancyBaselineReady) {
    previousOccupiedSlots = occupiedNow;
    occupancyBaselineReady = true;
    return;
  }
  if (occupiedNow == previousOccupiedSlots) return;
  startGateCycle(occupiedNow > previousOccupiedSlots);
  previousOccupiedSlots = occupiedNow;
}

void serviceGate() {
  const uint32_t now = millis();
  if (gateMotion == GateMotion::Open && static_cast<int32_t>(now - gateCloseDeadlineMs) >= 0)
    gateMotion = GateMotion::Closing;
  if (now - gateLastStepMs < GATE_SERVO_STEP_MS) return;
  gateLastStepMs = now;

  if (gateMotion == GateMotion::Opening) {
    const uint16_t next = gateAngle + GATE_SERVO_STEP_DEGREES;
    gateAngle = next >= GATE_OPEN_ANGLE ? GATE_OPEN_ANGLE : next;
    if (gateServoAttached) gateServo.write(gateAngle);
    if (gateAngle == GATE_OPEN_ANGLE) {
      gateMotion = GateMotion::Open;
      gateCloseDeadlineMs = now + GATE_OPEN_HOLD_MS;
    }
  } else if (gateMotion == GateMotion::Closing) {
    gateAngle = gateAngle > GATE_SERVO_STEP_DEGREES ? gateAngle - GATE_SERVO_STEP_DEGREES : GATE_CLOSED_ANGLE;
    if (gateServoAttached) gateServo.write(gateAngle);
    if (gateAngle == GATE_CLOSED_ANGLE) gateMotion = GateMotion::Closed;
  }
}

String makeStatusJson() {
  JsonDocument doc;
  doc["type"] = "parking_status";
  doc["uptimeMs"] = millis();
  doc["simulationMode"] = SIMULATION_MODE;
  JsonArray values = doc["slots"].to<JsonArray>();
  uint8_t occupiedCount = 0;
  for (uint8_t i = 0; i < PARKING_SLOT_COUNT; ++i) {
    const SlotState &slot = slots[i];
    occupiedCount += slot.occupied ? 1 : 0;
    JsonObject item = values.add<JsonObject>();
    item["index"] = i + 1;
    item["occupied"] = slot.occupied;
    item["status"] = slot.occupied ? "Occupied" : "Available";
    item["confidence"] = slotConfidence(slot);
    item["sensorState"] = slotSensorState(slot);
    JsonObject ultrasonic = item["ultrasonic"].to<JsonObject>();
    ultrasonic["valid"] = slot.ultrasonicValid;
    ultrasonic["occupied"] = slot.ultrasonicOccupied;
    ultrasonic["distanceCm"] = slot.ultrasonicValid ? slot.distanceCm : -1;
    JsonObject magnetic = item["magnetic"].to<JsonObject>();
    magnetic["enabled"] = SIMULATION_MODE || USE_GROUND_SENSORS;
    magnetic["valid"] = slot.magneticValid;
    magnetic["occupied"] = slot.magneticOccupied;
  }
  doc["summary"]["total"] = PARKING_SLOT_COUNT;
  doc["summary"]["occupied"] = occupiedCount;
  doc["summary"]["available"] = PARKING_SLOT_COUNT - occupiedCount;
  JsonObject gate = doc["gate"].to<JsonObject>();
  gate["state"] = gateStateName();
  gate["angle"] = gateAngle;
  gate["lastEvent"] = lastGateEvent;
  gate["simulation"] = SIMULATION_MODE;
  String output;
  serializeJson(doc, output);
  return output;
}

void logStatus() {
  Serial.printf("Zunked %s |", SIMULATION_MODE ? "SIMULATION" : "SENSORS");
  for (uint8_t i = 0; i < PARKING_SLOT_COUNT; ++i) {
    const SlotState &slot = slots[i];
    Serial.printf(" slot%u=%s", i + 1, slot.occupied ? "OCCUPIED" : "AVAILABLE");
    if (slot.ultrasonicValid) Serial.printf("(%dcm)", slot.distanceCm);
    else Serial.print("(no reading)");
    Serial.printf("[U:%s M:%s %s]", slot.ultrasonicValid ? (slot.ultrasonicOccupied ? "occupied" : "clear") : "n/a",
                  slot.magneticValid ? (slot.magneticOccupied ? "occupied" : "clear") : "n/a",
                  slotConfidence(slot));
  }
  Serial.printf(" gate=%s@%u\n", gateStateName(), gateAngle);
}

void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiStartedMs = millis();
  Serial.println("Connecting to Wi-Fi...");
}

void serviceWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiReadyLogged) {
      wifiReadyLogged = true;
      Serial.print("Dashboard: http://");
      Serial.println(WiFi.localIP());
    }
    return;
  }
  if (apMode || millis() - wifiStartedMs < WIFI_TIMEOUT_MS) return;
  apMode = true;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  const bool started = WiFi.softAP(FALLBACK_AP_SSID, FALLBACK_AP_PASSWORD);
  Serial.println(started ? "Wi-Fi unavailable; starting setup access point." : "Failed to start setup access point.");
  if (started) {
    Serial.print("Dashboard: http://");
    Serial.println(WiFi.softAPIP());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Zunked parking monitor...");
  if (!SIMULATION_MODE) {
    for (uint8_t i = 0; i < PARKING_SLOT_COUNT; ++i) {
      pinMode(ULTRASONIC_TRIG_PINS[i], OUTPUT);
      pinMode(ULTRASONIC_ECHO_PINS[i], INPUT);
      digitalWrite(ULTRASONIC_TRIG_PINS[i], LOW);
      if (USE_GROUND_SENSORS) pinMode(GROUND_SENSOR_PINS[i], INPUT_PULLUP);
    }
    // Power the servo from a suitable external supply and connect grounds together.
    gateServoAttached = gateServo.attach(GATE_SERVO_PIN, GATE_SERVO_MIN_PULSE_US,
                                         GATE_SERVO_MAX_PULSE_US) != 0;
    if (gateServoAttached) gateServo.write(GATE_CLOSED_ANGLE);
    else Serial.println("Gate servo attach failed; gate state remains simulated.");
  }
  startWiFi();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html; charset=utf-8", DASHBOARD_HTML);
  });
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", makeStatusJson());
  });
  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "ok");
  });
  server.begin();
  Serial.println("Zunked dashboard server ready.");
}

void loop() {
  serviceWiFi();
  if (SIMULATION_MODE) serviceSimulation();
  else {
    serviceUltrasonic();
    serviceGroundSensors();
  }
  serviceVehicleEvents();
  serviceGate();
  if (millis() - lastDebugLogMs >= DEBUG_LOG_MS) {
    lastDebugLogMs = millis();
    logStatus();
  }
}
