# Drone-codes
Includes various codes related to drone / embbeded projects
#include <WiFi.h>
#include <WebServer.h>

// -------------------------------------------------------------------
// 1. GLOBAL SETTINGS & WIFI
// Standardized WiFi credentials from TDS_TESTER.ino and PH_TESTER.ino
// -------------------------------------------------------------------

const char* ssid = "WIFI_JAM";       // Standardized [cite: 4, 35]
const char* password = "12345678";   // Standardized [cite: 4, 35]

WebServer server(80);

// Timing for loop functions
unsigned long lastTDSReadMs = 0;
unsigned long lastPHReadMs = 0;
// TDS sensor should sample frequently (e.g., 400ms)
const unsigned long TDS_INTERVAL_MS = 400; 
// pH sensor read involves delay(5) in the function, so it should be called less frequently
const unsigned long PH_READ_INTERVAL_MS = 2000; 

// Initializing variables for global access
float lastVoltage = 0.0;
float lastTDS = 0.0;
float lastPHVoltage = 0.0;
float lastPH = 0.0;
int moisturePercentage = 0;

// -------------------------------------------------------------------
// NEW: TEMPERATURE (fake but realistic-looking)
// -------------------------------------------------------------------
// Configuration: desired realistic range 20.0 - 35.0 °C
const float TEMP_MIN = 20.0;
const float TEMP_MAX = 35.0;

// Simulation parameters
const unsigned long TEMP_INTERVAL_MS = 3000; // update temperature every 3s
unsigned long lastTempReadMs = 0;
float lastTemp = 24.5;      // initial seed temperature (can be tuned)
float tempSmoothAlpha = 0.08; // smoothing factor for low-pass (0..1)
float tempDiurnalAmplitude = 3.0; // small diurnal swing amplitude
float tempDiurnalPeriodMs = 24UL * 60UL * 60UL * 1000UL; // 24 hours in ms

// Random-walk maximum delta per update (to keep it realistic)
const float TEMP_MAX_STEP = 0.35;

// Helper: safe clamp
float clampf(float v, float a, float b) {
  if (v < a) return a;
  if (v > b) return b;
  return v;
}

// Temperature simulator: uses diurnal sine + small random walk + smoothing
void readTemperature() {
  unsigned long now = millis();

  // diurnal component scaled to a convenient amplitude (based on millis)
  // This produces a slow sine wave over 24h. For short demos it will be almost flat,
  // but it's a realistic base pattern if the device runs long-term.
  float diurnal = tempDiurnalAmplitude * sinf( (2.0f * 3.14159265f * (float)now) / (float)tempDiurnalPeriodMs );

  // random walk step: small random between -TEMP_MAX_STEP..TEMP_MAX_STEP
  // Use built-in random() which must be seeded in setup()
  long r = random(-1000, 1001);
  float randStep = ((float)r / 1000.0f) * TEMP_MAX_STEP;

  // propose new raw temp
  float proposed = lastTemp + diurnal * 0.0f + randStep; 
  // note: diurnal multiplied by 0.0 above (keeps diurnal from double-counting on short demos).
  // We'll instead blend lastTemp towards a base that includes diurnal centered in the mid-range.

  float baseMid = (TEMP_MIN + TEMP_MAX) / 2.0f + diurnal * 0.4f; // diurnal nudges base
  float mixed = baseMid * 0.15f + proposed * 0.85f; // keep mostly lastTemp + rand

  // smoothing (simple low-pass)
  float smoothed = lastTemp + tempSmoothAlpha * (mixed - lastTemp);

  // clamp to allowed range
  smoothed = clampf(smoothed, TEMP_MIN, TEMP_MAX);

  lastTemp = smoothed;
}

// -------------------------------------------------------------------
// NEW: GPS (fake coordinates with small jitter)
// -------------------------------------------------------------------
// Base coordinate provided by user: 20.0136° N, 73.8223° E
const double GPS_BASE_LAT = 20.0136;
const double GPS_BASE_LON = 73.8223;

// jitter settings (in degrees). ~0.00001 deg ≈ 1.1 m at equator
const double GPS_JITTER_DEG = 0.00003; // up to ~3–4 m jitter

// Return fake lat/lon with a tiny random jitter to look real
void getFakeGPS(double &lat, double &lon) {
  long r1 = random(-10000, 10001); // -10000..10000
  long r2 = random(-10000, 10001);
  lat = GPS_BASE_LAT + ( (double)r1 / 10000.0 ) * (GPS_JITTER_DEG);
  lon = GPS_BASE_LON + ( (double)r2 / 10000.0 ) * (GPS_JITTER_DEG);
}

// -------------------------------------------------------------------
// 2. MOISTURE SENSOR (GPIO 35 - From _moisture_testing.ino)
// -------------------------------------------------------------------
const int MOISTURE_SENSOR_PIN = 35; // 
const int WET_VALUE = 1600;         // [cite: 54]
const int DRY_VALUE = 3900;         // [cite: 55]
const int MOISTURE_ALERT_THRESHOLD = 30; // [cite: 56]

void readMoisture() {
  int rawValue = analogRead(MOISTURE_SENSOR_PIN); // [cite: 57]
  // Map the raw sensor reading to a percentage (0-100)
  int mappedValue = map(rawValue, DRY_VALUE, WET_VALUE, 0, 100); // [cite: 58]
  // Constrain the value to be within 0 and 100
  moisturePercentage = constrain(mappedValue, 0, 100); // [cite: 59]
}

// -------------------------------------------------------------------
// 3. TDS SENSOR (GPIO 32 - From TDS_TESTER.ino)
// -------------------------------------------------------------------
const int TDS_PIN = 32;                 // 
const int ADC_BITS = 4095;              // 12-bit ADC 
const float VREF = 3.3;                 // [cite: 6]
const float calibrationFactor = 0.5;    // [cite: 7]

// Smoothing settings [cite: 7, 8]
const int SMOOTH_N = 8;
float tdsSmoothingBuffer[SMOOTH_N];
int tdsSmoothingIndex = 0;
bool tdsSmoothingFilled = false;

// Read ADC, smooth, convert to voltage and TDS (ppm)
void readTDS(){
  int raw = analogRead(TDS_PIN);         // 0 - 4095
  // smoothing buffer [cite: 19]
  tdsSmoothingBuffer[tdsSmoothingIndex++] = raw;
  if (tdsSmoothingIndex >= SMOOTH_N) { tdsSmoothingIndex = 0; tdsSmoothingFilled = true; }

  float sum = 0; int count = tdsSmoothingFilled ? SMOOTH_N : tdsSmoothingIndex;
  for (int i=0;i<count;i++) sum += tdsSmoothingBuffer[i];
  float avgRaw = (count>0) ? (sum / count) : raw;

  float voltage = (avgRaw / (float)ADC_BITS) * VREF; // voltage in volts (approx) [cite: 20]
  lastVoltage = voltage;
  
  // Convert voltage -> TDS (ppm) using standard polynomial [cite: 21, 22]
  float v = voltage;
  float tdsVal = (133.42 * v * v * v - 255.86 * v * v + 857.39 * v) * calibrationFactor;
  if (tdsVal < 0) tdsVal = 0; // safety clamp [cite: 23]

  lastTDS = tdsVal;
}

// -------------------------------------------------------------------
// 4. pH SENSOR (GPIO 34 - From PH_TESTER.ino)
// -------------------------------------------------------------------
const int PH_PIN = 34;                // [cite: 36, 34]
const int PH_ADC_WIDTH = 12;          // [cite: 37]
const int PH_ADC_MAX = (1 << PH_ADC_WIDTH) - 1; // [cite: 37]
const float PH_VREF = 3.3;            // [cite: 38]

float neutralVoltage = 2.50;      // voltage at pH 7.0 (calibrate this) [cite: 38]
float slopeVoltagePerPH = 0.177;  // expected V change per pH unit [cite: 39]

const int PH_SAMPLE_COUNT = 8; // Basic smoothing [cite: 39]

// Read smoothed ADC for pH
void readPH() {
  // read smoothed ADC
  long sum = 0;
  for(int i=0;i<PH_SAMPLE_COUNT;i++){
    sum += analogRead(PH_PIN);
    delay(5); // [cite: 47] - delay needed for stable sampling
  }
  float raw = (float)sum / PH_SAMPLE_COUNT;
  float voltage = (raw / PH_ADC_MAX) * PH_VREF;
  lastPHVoltage = voltage;
  float pH = 7.0 + (neutralVoltage - voltage) / slopeVoltagePerPH; // [cite: 39]

  // Clip pH to reasonable range
  if(pH < 0.0) pH = 0.0;
  if(pH > 14.0) pH = 14.0;

  lastPH = pH;
}


// -------------------------------------------------------------------
// 5. WEB SERVER HANDLERS & HTML/JSON ENDPOINTS
// -------------------------------------------------------------------

// Helper: Read all sensor data once for the index page
void updateAllSensorsForIndex() {
    readMoisture();
    readTDS();
    readPH();
    // read temperature (fake) to show on index
    readTemperature();
}

// Handler for the Main Index Page (/)
void handleRoot() {
  updateAllSensorsForIndex();
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'><title>ESP32 Multi-Sensor Monitor</title><style>body{text-align: center; font-family: sans-serif;} h1{color: #007BFF;}</style></head><body>";
  html += "<h1>Multi-Sensor Monitor Dashboard</h1>";
  html += "<h2>Current Readings (Refresh Page for Update):</h2>";
  html += "<ul>";
  html += "<li><a href='/moisture'>Soil Moisture: <b>" + String(moisturePercentage) + "%</b></a></li>";
  html += "<li><a href='/tds'>TDS: <b>" + String(lastTDS, 1) + " ppm</b></a></li>";
  html += "<li><a href='/ph'>pH: <b>" + String(lastPH, 2) + "</b></a></li>";
  // show temperature and GPS quick view
  html += "<li><a href='/temp'>Temperature: <b>" + String(lastTemp, 2) + " &deg;C</b></a></li>";
  double lat, lon;
  getFakeGPS(lat, lon);
  html += "<li><a href='/gps'>GPS: <b>" + String(lat, 6) + " N, " + String(lon, 6) + " E</b></a></li>";
  html += "</ul>";
  html += "<p><em>Individual pages (/moisture, /tds, /ph, /temp, /gps) provide live/auto-refreshing data.</em></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Handler for Moisture HTML Page (/moisture) - Uses content from _moisture_testing.ino [cite: 61, 62, 63, 64, 65, 66]
void handleMoistureRoot() {
  readMoisture(); // Read sensor before generating the page [cite: 61]
  
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'><meta http-equiv='refresh' content='5'>"; // Auto-refresh every 5s
  html += "<title>ESP32 Soil Moisture Monitor</title>"; // [cite: 62]
  html += "<style>body{text-align: center; font-family: sans-serif;} h1{color: #007BFF;} .alert{color: red; font-weight: bold;} .ok{color: green; font-weight: bold;}</style>"; // [cite: 63]
  html += "</head><body>";
  
  // Display the current moisture value
  html += "<h1>Soil Moisture Level</h1>";
  html += "<h2>" + String(moisturePercentage) + "%</h2>"; // [cite: 64]
  
  // Display the alert if moisture is low
  if (moisturePercentage < MOISTURE_ALERT_THRESHOLD) {
    html += "<p class='alert'>🚨 LOW MOISTURE ALERT: Below " + String(MOISTURE_ALERT_THRESHOLD) + "%! Please water the plant.</p>"; // [cite: 65]
  } else {
    html += "<p class='ok'>✅ Moisture is Optimal.</p>"; // [cite: 66]
  }

  html += "<p><a href='/'>&larr; Back to Dashboard</a></p>";
  html += "<p><em>Last Updated: " + String(millis()/1000) + "s ago</em></p>"; // [cite: 66]
  html += "</body></html>";
  
  server.send(200, "text/html", html); // [cite: 67]
}

// Handler for TDS HTML Page (/tds) - Uses content from TDS_TESTER.ino [cite: 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]
void handleTDSMonitor() {
  const char tds_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8" /><title>ESP32 TDS Monitor</title><style>body { font-family: Arial, sans-serif; margin: 20px; } .card { border-radius: 8px; padding: 16px; box-shadow: 0 2px 6px rgba(0,0,0,0.12); max-width:500px; } #value { font-size: 2.6em; margin:10px 0; } #chart { width:100%; height:120px; border:1px solid #ddd; }</style></head><body><div class="card"><h2>ESP32 TDS Monitor</h2><div>Live TDS (ppm): <div id="value">--</div></div><canvas id="chart"></canvas><div style="font-size:0.9em;color:#666;margin-top:8px">Voltage: <span id="voltage">--</span> V · Raw: <span id="raw">--</span></div><p><a href='/'>&larr; Back to Dashboard</a></p></div><script>let values = []; const maxPoints = 60; function addPoint(v) { values.push(v); if (values.length > maxPoints) values.shift(); draw(); } function draw() { const canvas = document.getElementById('chart'); canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; const ctx = canvas.getContext('2d'); ctx.clearRect(0,0,canvas.width,canvas.height); if (values.length === 0) return; const w = canvas.width, h = canvas.height; const max = Math.max(...values) * 1.1; const min = Math.min(...values); ctx.beginPath(); for (let i=0;i<values.length;i++){ const x = (i / (maxPoints-1)) * w; const y = h - ((values[i] - min) / ( (max-min)||1 )) * h; if (i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); } ctx.strokeStyle = '#007acc'; ctx.lineWidth = 2; ctx.stroke(); } async function fetchTDS(){ try { const res = await fetch('/tds-data'); const j = await res.json(); document.getElementById('value').textContent = j.tds.toFixed(1); document.getElementById('voltage').textContent = j.voltage.toFixed(3); document.getElementById('raw').textContent = j.raw; addPoint(j.tds); } catch(e){ console.log('fetch error', e); } } setInterval(fetchTDS, 1000); window.addEventListener('resize', draw); fetchTDS();</script></body></html>
)rawliteral";
  server.send_P(200, "text/html", tds_html);
}

// Handler for TDS JSON Data (/tds-data) - Renamed from /tds [cite: 16]
void handleTDSData(){
  // return last reading as JSON
  String json = "{";
  json += "\"tds\":" + String(lastTDS, 2) + ",";
  json += "\"voltage\":" + String(lastVoltage, 3) + ",";
  json += "\"raw\":" + String((int)analogRead(TDS_PIN)) + ",";
  json += "\"time\":" + String(millis());
  json += "}";
  server.send(200, "application/json", json);
}

// Handler for pH HTML Page (/ph) - Uses content from PH_TESTER.ino [cite: 40, 41, 42, 43, 44, 45, 46]
void handlePHMonitor() {
  const char ph_html[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8"><title>ESP32 pH Monitor</title><style>body{font-family: Arial, Helvetica, sans-serif; text-align:center; margin:20px;}#phVal{font-size:2.4rem; margin:10px;}canvas{border:1px solid #ddd; width:90%; max-width:600px; height:160px;}</style></head><body><h2>ESP32 pH Monitor</h2><div>pH: <span id="phVal">--</span></div><div>Voltage: <span id="voltVal">--</span> V</div><canvas id="chart"></canvas><p><a href='/'>&larr; Back to Dashboard</a></p><script>let history = [];const maxPoints = 40;const canvas = document.getElementById('chart');const ctx = canvas.getContext('2d');function fetchPH(){fetch('/ph-data').then(r=>r.json()).then(j=>{document.getElementById('phVal').innerText = j.pH.toFixed(2);document.getElementById('voltVal').innerText = j.voltage.toFixed(3);history.push(j.pH);if(history.length>maxPoints) history.shift();drawChart();}).catch(err=>console.error(err));}function drawChart(){const w = canvas.clientWidth;const h = canvas.clientHeight;canvas.width = w;canvas.height = h;ctx.clearRect(0,0,w,h);if(history.length<2) return;let min = Math.min(...history);let max = Math.max(...history);if(max - min < 0.5){ max += 0.25; min -= 0.25; }ctx.beginPath();history.forEach((v,i)=>{const x = i*(w/(maxPoints-1));const y = h - ((v - min)/(max-min))*h;if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);});ctx.stroke();if(7>=min && 7<=max){const y7 = h - ((7-min)/(max-min))*h;ctx.setLineDash([4,4]);ctx.beginPath();ctx.moveTo(0,y7);ctx.lineTo(w,y7);ctx.stroke();ctx.setLineDash([]);}}setInterval(fetchPH, 2000);fetchPH();</script></body></html>
)rawliteral";
  server.send_P(200, "text/html", ph_html); // [cite: 51]
}

// Handler for pH JSON Data (/ph-data) - Renamed from /ph [cite: 47]
void handlePHData() {
    readPH(); // Read pH data before sending it
    String json = "{\"pH\":" + String(lastPH,3) + ",\"voltage\":" + String(lastPHVoltage,4) + "}";
    server.send(200, "application/json", json);
}

// -------------------------------------------------------------------
// NEW: Temperature HTML & JSON handler (/temp, /temp-data)
// -------------------------------------------------------------------
void handleTempMonitor() {
  // simple HTML page with auto-refresh
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'><meta http-equiv='refresh' content='3'><title>ESP32 Temperature (simulated)</title>";
  html += "<style>body{text-align:center;font-family:Arial;}#val{font-size:2.4rem;margin:10px;}</style></head><body>";
  html += "<h2>Simulated Temperature</h2>";
  html += "<div id='val'>" + String(lastTemp, 2) + " &deg;C</div>";
  html += "<p><a href='/temp-data'>Get JSON</a> · <a href='/'>Back</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleTempData() {
  // ensure temperature is up-to-date
  readTemperature();
  String json = "{";
  json += "\"temperature\":" + String(lastTemp, 2) + ",";
  json += "\"unit\":\"C\",";
  json += "\"time\":" + String(millis());
  json += "}";
  server.send(200, "application/json", json);
}

// -------------------------------------------------------------------
// NEW: GPS HTML & JSON handler (/gps, /gps-data)
// -------------------------------------------------------------------
void handleGPSMonitor() {
  double lat, lon;
  getFakeGPS(lat, lon);

  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'><meta http-equiv='refresh' content='5'><title>Fake GPS</title></head><body style='font-family:Arial;text-align:center;'>";
  html += "<h2>Fake GPS Position</h2>";
  html += "<div>Latitude: <b>" + String(lat, 6) + " N</b></div>";
  html += "<div>Longitude: <b>" + String(lon, 6) + " E</b></div>";
  html += "<p><a href='/gps-data'>Get JSON</a> · <a href='/'>Back</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleGPSData() {
  double lat, lon;
  getFakeGPS(lat, lon);

  String json = "{";
  json += "\"lat\":" + String(lat, 6) + ",";
  json += "\"lon\":" + String(lon, 6) + ",";
  json += "\"source\":\"simulated\",";
  json += "\"time\":" + String(millis());
  json += "}";
  server.send(200, "application/json", json);
}

// -------------------------------------------------------------------
// 6. SETUP AND LOOP
// -------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(100);

  // --- Pin Modes ---
  pinMode(MOISTURE_SENSOR_PIN, INPUT); 

  // --- ADC Settings for all sensors ---
  analogReadResolution(12); // 12-bit (0-4095) [cite: 27, 28, 37]
  
  // Set 11dB attenuation for all pins to read full 0-3.3V range [cite: 3, 27, 34]
  analogSetPinAttenuation(MOISTURE_SENSOR_PIN, ADC_11db); 
  analogSetPinAttenuation(TDS_PIN, ADC_11db);
  analogSetPinAttenuation(PH_PIN, ADC_11db); 

  // Zero smoothing buffer for TDS [cite: 28]
  for (int i=0; i<SMOOTH_N; i++) tdsSmoothingBuffer[i] = 0;

  // --- WiFi Connection ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); // [cite: 24, 67]
  Serial.printf("Connecting to WiFi '%s' ...\n", ssid); // [cite: 48]
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) { // [cite: 48]
    delay(400); // [cite: 25]
    Serial.print("."); // [cite: 49]
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!"); // [cite: 50]
    Serial.print("Web Server IP address: "); // [cite: 69]
    Serial.println(WiFi.localIP()); // [cite: 25]
  } else {
    Serial.println("\nWiFi connect failed. Check credentials."); // [cite: 26, 50]
  }

  // Seed random with micros (simple, avoids needing extra pins)
  randomSeed(micros());

  // --- Web Server Setup ---
  server.on("/", handleRoot);
  server.on("/moisture", handleMoistureRoot); // HTML page with auto-refresh
  server.on("/tds", handleTDSMonitor);       // HTML page with JS chart
  server.on("/tds-data", handleTDSData);     // JSON data endpoint (renamed from /tds)
  server.on("/ph", handlePHMonitor);         // HTML page with JS chart
  server.on("/ph-data", handlePHData);       // JSON data endpoint (renamed from /ph)

  // new endpoints
  server.on("/temp", handleTempMonitor);
  server.on("/temp-data", handleTempData);
  server.on("/gps", handleGPSMonitor);
  server.on("/gps-data", handleGPSData);
  
  server.begin(); // [cite: 29, 70]
  Serial.println("HTTP server started");

  // Initial sensor reads to populate variables
  readTDS();
  readPH();
  readTemperature();
  lastTDSReadMs = millis(); // [cite: 30]
  lastPHReadMs = millis();
  lastTempReadMs = millis();
}

void loop() {
  server.handleClient();

  unsigned long now = millis();

  // TDS reading logic (sample every 400 ms) [cite: 31]
  if (now - lastTDSReadMs >= TDS_INTERVAL_MS) {
    readTDS();
    lastTDSReadMs = now;
  }

  // pH reading logic (sample every 2000 ms)
  if (now - lastPHReadMs >= PH_READ_INTERVAL_MS) {
    readPH();
    lastPHReadMs = now;
  }
  
  // Temperature update logic (simulate at interval)
  if (now - lastTempReadMs >= TEMP_INTERVAL_MS) {
    readTemperature();
    lastTempReadMs = now;
  }

  // Moisture reading is handled directly when the client requests the page (handleMoistureRoot)
}
