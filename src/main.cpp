/*
  Smart Parking - one sensor (P1)
  ESP32 Dev Module / HC-SR04
  Match the user's wiring: TRIG -> GPIO 14; ECHO -> GPIO 34
  through a voltage divider. Optional LED -> GPIO 13 with a resistor to GND.
*/
#include <WiFi.h>
#include <WebServer.h>

// Match the wiring shown in the user's screenshot / supplied sketch.
constexpr uint8_t TRIG_PIN = 14;
constexpr uint8_t ECHO_PIN = 34;
constexpr uint8_t LED_PIN = 13; // Optional; set LED_ENABLED false if unwired.
constexpr bool LED_ENABLED = true;

// Adjust after measuring the empty slot and a parked vehicle/object.
constexpr float OCCUPIED_BELOW_CM = 20.0f;
constexpr float VACANT_ABOVE_CM = 25.0f;
// HC-SR04 needs a quiet interval between pings; 100 ms is responsive and safe.
constexpr uint32_t SAMPLE_INTERVAL_MS = 100;
constexpr uint8_t MEDIAN_SIZE = 3;
constexpr uint8_t DEBOUNCE_SAMPLES = 2;
constexpr uint8_t ERROR_AFTER_BAD_READS = 6;

constexpr char AP_SSID[] = "SmartParking-Setup";
constexpr char AP_PASSWORD[] = "parking123";

WebServer server(80);
float readings[MEDIAN_SIZE] = {};
uint8_t readingCount = 0, readingIndex = 0;
uint8_t candidateRun = 0, badReadRun = 0;
bool candidateOccupied = false, occupied = false, sensorError = false;
bool hasDistance = false;
float lastDistanceCm = 0.0f;
uint32_t lastSampleMs = 0, lastErrorReportMs = 0;

const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Parking P1</title><style>
body{font-family:Arial,sans-serif;background:#101820;color:#f5f7fa;text-align:center;padding:24px}
.card{max-width:360px;margin:30px auto;padding:34px 16px;border-radius:18px;background:#253746}
.free{background:#16845b}.occupied{background:#c43d42}.error{background:#626a73}
#state{font-size:30px;font-weight:bold}small{display:block;margin-top:12px;font-size:16px}
</style></head><body><h1>Smart Parking</h1><div id="card" class="card"><div id="state">Connecting…</div><small id="distance">Waiting for sensor</small></div>
<script>async function update(){try{const d=await(await fetch('/api/status')).json();const c=document.getElementById('card');
if(d.error){c.className='card error';state.textContent='SENSOR ERROR';distance.textContent='Check sensor power, pins and Echo divider';}
else{c.className='card '+(d.occupied?'occupied':'free');state.textContent=d.occupied?'OCCUPIED':'FREE';distance.textContent=d.distance_cm.toFixed(1)+' cm';}}
catch(e){state.textContent='Connection lost';distance.textContent='Reconnect to SmartParking-Setup';}}update();setInterval(update,1000);</script></body></html>
)rawliteral";

struct DistanceReading {
  float cm;
  uint32_t echoUs;
  bool timedOut;
};

DistanceReading readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  const uint32_t echoUs = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (echoUs == 0) return {-1.0f, 0, true};
  const float cm = echoUs * 0.0343f / 2.0f;
  if (cm < 2.0f || cm > 400.0f) return {-1.0f, echoUs, false};
  return {cm, echoUs, false};
}

float medianReading() {
  float sorted[MEDIAN_SIZE];
  for (uint8_t i = 0; i < readingCount; ++i) sorted[i] = readings[i];
  for (uint8_t i = 0; i + 1 < readingCount; ++i) {
    for (uint8_t j = i + 1; j < readingCount; ++j) {
      if (sorted[j] < sorted[i]) {
        const float temp = sorted[i]; sorted[i] = sorted[j]; sorted[j] = temp;
      }
    }
  }
  return sorted[readingCount / 2];
}

void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleStatus() {
  String json = "{\"occupied\":";
  json += occupied ? "true" : "false";
  json += ",\"error\":";
  json += sensorError ? "true" : "false";
  json += ",\"valid\":";
  json += hasDistance ? "true" : "false";
  json += ",\"distance_cm\":";
  json += hasDistance ? String(lastDistanceCm, 1) : "null";
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  if (LED_ENABLED) { pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW); }

  Serial.println("\nSmart Parking P1 starting");
  Serial.println("Wiring expected: TRIG=GPIO14, ECHO divider junction=GPIO34");

  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.printf("Wi-Fi: %s | Dashboard: http://%s/\n", AP_SSID,
                  WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("Could not start Wi-Fi access point.");
  }
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.begin();
  Serial.println("P1 ready. Serial Monitor: 115200 baud.");
}

void loop() {
  server.handleClient();
  const uint32_t now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  const DistanceReading reading = readDistance();
  if (reading.cm < 0.0f) {
    if (badReadRun < ERROR_AFTER_BAD_READS) ++badReadRun;
    if (badReadRun >= ERROR_AFTER_BAD_READS) sensorError = true;
    if (now - lastErrorReportMs >= 1000) {
      lastErrorReportMs = now;
      if (reading.timedOut) {
        Serial.printf("NO ECHO: no pulse on GPIO34 (pin level %s). Check sensor ECHO, divider junction, 5V and shared GND.\n",
                      digitalRead(ECHO_PIN) ? "HIGH" : "LOW");
      } else {
        Serial.printf("ECHO OUT OF RANGE: %lu us; move a flat target 20-50 cm away.\n",
                      static_cast<unsigned long>(reading.echoUs));
      }
    }
    return;
  }

  badReadRun = 0;
  sensorError = false;
  lastErrorReportMs = 0;
  readings[readingIndex] = reading.cm;
  readingIndex = (readingIndex + 1) % MEDIAN_SIZE;
  if (readingCount < MEDIAN_SIZE) ++readingCount;
  lastDistanceCm = medianReading();
  hasDistance = true;

  const bool nextCandidate = occupied
      ? !(lastDistanceCm >= VACANT_ABOVE_CM)
      : (lastDistanceCm < OCCUPIED_BELOW_CM);
  if (nextCandidate == candidateOccupied) {
    if (candidateRun < DEBOUNCE_SAMPLES) ++candidateRun;
  } else {
    candidateOccupied = nextCandidate;
    candidateRun = 1;
  }
  if (candidateRun >= DEBOUNCE_SAMPLES && occupied != candidateOccupied) {
    occupied = candidateOccupied;
    if (LED_ENABLED) digitalWrite(LED_PIN, occupied ? HIGH : LOW);
    Serial.printf("P1 is now %s\n", occupied ? "OCCUPIED" : "FREE");
  }
  Serial.printf("P1: %.1f cm | %s\n", lastDistanceCm, occupied ? "OCCUPIED" : "FREE");
}
