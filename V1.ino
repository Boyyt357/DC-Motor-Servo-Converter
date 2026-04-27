#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Preferences.h>

// --- Pins (ESP32-S3) ---
const int motorIN1 = 25; 
const int motorIN2 = 26; 
const int motorENA = 27; 
const int inputPin = 32; // PWM or Analog Input

// --- PID & Control Variables ---
float Kp, Ki, Kd, tolerance, maxDeg, gearRatio;
float targetDeg = 0.0, currentDeg = 0.0;
float integral = 0, lastError = 0;
bool servoEnabled = true;
int controlMode = 0; // 0: Web, 1: PWM, 2: Analog

// --- Smoothing/Filtering ---
float filterAlpha = 0.15; // 0.1 (very smooth/slow) to 0.8 (fast/noisy)

// --- Encoder Variables ---
int lastRaw = 0;
long totalRaw = 0;
long homeOffset = 0;
volatile unsigned long pulseStart = 0;
volatile int pulseWidth = 0;

Preferences prefs; 
AsyncWebServer server(80);

// --- Interrupt for External PWM ---
void IRAM_ATTR handlePWM() {
  if (digitalRead(inputPin) == HIGH) pulseStart = micros();
  else pulseWidth = micros() - pulseStart;
}

void updateEncoder() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0E); // AS5600 Angle Register
  if (Wire.endTransmission() != 0) return; // I2C Error check

  Wire.requestFrom(0x36, 2);
  if (Wire.available() >= 2) {
    int raw = (Wire.read() << 8) | Wire.read();
    int diff = raw - lastRaw;
    
    // Handle wrap-around for multi-turn
    if (diff > 2048) diff -= 4096;
    if (diff < -2048) diff += 4096;
    
    totalRaw += diff;
    lastRaw = raw;

    // Degrees = (Total Pulses - Offset) * (360 / 4096) * GearRatio
    currentDeg = ((totalRaw - homeOffset) * (360.0 / 4096.0)) * gearRatio;
  }
}

void resetPID() {
  integral = 0;
  lastError = 0;
}

void driveMotor(int output) {
  // Add this inside void setup()
  analogWriteFrequency(motorIN1, 20000); 
  analogWriteFrequency(motorIN2, 20000);
  float error = abs(targetDeg - currentDeg);

  // DEADZONE: Stop motor if within tolerance to prevent jitter/humming
  if (!servoEnabled || error < tolerance) {
    analogWrite(motorIN1, 0);
    analogWrite(motorIN2, 0);
    return;
  }

  // Minimum starting power (constrain between 45 and 255)
  int speed = constrain(abs(output), 45, 255); 
  
  if (output > 0) {
    analogWrite(motorIN1, speed);
    analogWrite(motorIN2, 0);
  } else {
    analogWrite(motorIN1, 0);
    analogWrite(motorIN2, speed);
  }
}

void runPID() {
  float error = targetDeg - currentDeg;
  
  // Anti-windup for Integral
  integral += error;
  integral = constrain(integral, -100, 100); 
  
  float derivative = error - lastError;
  int output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  driveMotor(output);
  lastError = error;
}

// --- HTML Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Pro Servo Panel</title>
<style>
  body { font-family: sans-serif; text-align: center; background: #1a1a1a; color: white; padding: 20px; }
  .card { background: #2d2d2d; padding: 20px; border-radius: 15px; display: inline-block; width: 380px; border: 1px solid #444; }
  .btn { padding: 12px; margin: 5px; cursor: pointer; border: none; border-radius: 8px; font-weight: bold; width: 45%; }
  .on { background: #2ecc71; color: white; }
  .save { background: #3498db; color: white; width: 93%; }
  input, select { width: 85%; padding: 10px; margin: 10px 0; border-radius: 5px; background: #444; color: white; border: none; }
  .row { display: flex; justify-content: space-around; align-items: center; }
  .row input { width: 60px; }
</style></head>
<body>
  <div class="card">
    <h2>Smart Servo Pro</h2>
    <h1 id="pos">0.0</h1><p>Output Degrees</p>
    <hr>
    <h4>Gearing & Limits</h4>
    <div class="row"> Ratio: <input type="text" id="ratio"> Max: <input type="text" id="mDeg"> </div>
    <select id="mode">
      <option value="0">Web Dashboard</option>
      <option value="1">External PWM</option>
      <option value="2">Analog Signal</option>
    </select>
    <hr>
    <h4>PID Tuning</h4>
    <div class="row"> P:<input type="text" id="p"> I:<input type="text" id="i"> D:<input type="text" id="d"> </div>
    Tol: <input type="text" id="t" style="width:60px;">
    <button class="btn save" onclick="saveSettings()">APPLY & SAVE</button>
    <hr>
    <button class="btn on" onclick="fetch('/sethome')">SET ZERO</button>
    <input type="number" id="moveVal" placeholder="Degrees">
    <button class="btn on" style="width:93%; background:#9b59b6" onclick="move()">WEB MOVE</button>
  </div>
<script>
  function move() { fetch('/move?val=' + document.getElementById('moveVal').value); }
  function saveSettings() {
    const p = document.getElementById('p').value, i = document.getElementById('i').value, d = document.getElementById('d').value;
    const t = document.getElementById('t').value, m = document.getElementById('mode').value, max = document.getElementById('mDeg').value, r = document.getElementById('ratio').value;
    fetch(`/save?p=${p}&i=${i}&d=${d}&t=${t}&m=${m}&max=${max}&r=${r}`).then(() => alert("Saved!"));
  }
  fetch('/getparams').then(r => r.json()).then(data => {
    document.getElementById('p').value = data.p; document.getElementById('i').value = data.i;
    document.getElementById('d').value = data.d; document.getElementById('t').value = data.t;
    document.getElementById('mode').value = data.m; document.getElementById('mDeg').value = data.max;
    document.getElementById('ratio').value = data.r;
  });
  setInterval(() => { fetch('/getpos').then(r => r.text()).then(t => { document.getElementById('pos').innerText = t; }); }, 300);
</script></body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(motorIN1, OUTPUT); pinMode(motorIN2, OUTPUT);
  pinMode(motorENA, OUTPUT); digitalWrite(motorENA, HIGH);
  pinMode(inputPin, INPUT);

  // Load Saved Data
  prefs.begin("servo-data", false);
  homeOffset = prefs.getLong("offset", 0);
  Kp = prefs.getFloat("kp", 2.0);
  Ki = prefs.getFloat("ki", 0.0);
  Kd = prefs.getFloat("kd", 0.1);
  tolerance = prefs.getFloat("tol", 1.0);
  maxDeg = prefs.getFloat("maxDeg", 180.0);
  gearRatio = prefs.getFloat("ratio", 1.0);
  controlMode = prefs.getInt("mode", 0);

  // INITIAL ENCODER READ
  Wire.beginTransmission(0x36);
  Wire.write(0x0E);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 2);
  if (Wire.available() >= 2) {
    lastRaw = (Wire.read() << 8) | Wire.read();
    totalRaw = lastRaw; 
  }

  // Calculate current pos and SYNC target to prevent startup jump
  updateEncoder();
  targetDeg = currentDeg; 

  if (controlMode == 1) attachInterrupt(digitalPinToInterrupt(inputPin), handlePWM, CHANGE);

  WiFi.softAP("SmartServo_Pro", "");

  // API Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *f){ f->send_P(200, "text/html", index_html); });
  
  server.on("/save", [](AsyncWebServerRequest *f){
    Kp = f->getParam("p")->value().toFloat();
    Ki = f->getParam("i")->value().toFloat();
    Kd = f->getParam("d")->value().toFloat();
    tolerance = f->getParam("t")->value().toFloat();
    maxDeg = f->getParam("max")->value().toFloat();
    gearRatio = f->getParam("r")->value().toFloat();
    int newMode = f->getParam("m")->value().toInt();

    if (newMode == 1 && controlMode != 1) attachInterrupt(digitalPinToInterrupt(inputPin), handlePWM, CHANGE);
    else if (newMode != 1 && controlMode == 1) detachInterrupt(digitalPinToInterrupt(inputPin));
    
    controlMode = newMode;
    resetPID();

    prefs.putFloat("kp", Kp); prefs.putFloat("ki", Ki); prefs.putFloat("kd", Kd);
    prefs.putFloat("tol", tolerance); prefs.putInt("mode", controlMode);
    prefs.putFloat("maxDeg", maxDeg); prefs.putFloat("ratio", gearRatio);
    f->send(200);
  });

  server.on("/move", [](AsyncWebServerRequest *f){ targetDeg = f->getParam("val")->value().toFloat(); f->send(200); });
  server.on("/getparams", [](AsyncWebServerRequest *f){
    String json = "{\"p\":"+String(Kp)+",\"i\":"+String(Ki)+",\"d\":"+String(Kd)+",\"t\":"+String(tolerance)+",\"m\":"+String(controlMode)+",\"max\":"+String(maxDeg)+",\"r\":"+String(gearRatio)+"}";
    f->send(200, "application/json", json);
  });
  server.on("/getpos", [](AsyncWebServerRequest *f){ f->send(200, "text/plain", String(currentDeg, 1)); });
  server.on("/sethome", [](AsyncWebServerRequest *f){ 
    homeOffset = totalRaw; 
    targetDeg = 0;
    prefs.putLong("offset", homeOffset); 
    f->send(200); 
  });

  server.begin();
}

void loop() {
  updateEncoder();

  float rawInput = targetDeg;

  if (controlMode == 1) { // PWM MODE
    if (pulseWidth > 800 && pulseWidth < 2200) {
      rawInput = map(pulseWidth, 1000, 2000, 0, maxDeg);
    }
  } 
  else if (controlMode == 2) { // ANALOG MODE
    int analogVal = 0;
    for(int i=0; i<8; i++) analogVal += analogRead(inputPin); // Multi-sample average
    rawInput = map(analogVal / 8, 0, 4095, 0, maxDeg);
  }

  // Apply smoothing only to external inputs (PWM/Analog)
  if (controlMode != 0) {
    // EMA Formula: Target = (Alpha * Raw) + ((1 - Alpha) * Target)
    targetDeg = (filterAlpha * rawInput) + ((1.0 - filterAlpha) * targetDeg);
  }

  runPID();
  delay(2); // Faster loop for better response
}