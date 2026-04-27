#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Preferences.h>

// --- Pins (Standard ESP32 Safe Pins) ---
const int motorIN1 = 25; // Direction / PWM 1
const int motorIN2 = 26; // Direction / PWM 2
const int motorENA = 27; // Enable Pin
const int inputPin = 32; // PWM or Analog Input

// I2C Pins (Default for standard ESP32)
// SDA = 21, SCL = 22

// --- Hardware PWM (LEDC) Configuration ---
const int pwmFreq = 20000; // 20kHz to eliminate motor whine
const int pwmResolution = 8; // 8-bit (0-255)

// --- PID & Kinematics Variables ---
float Kp, Ki, Kd, tolerance, maxDeg, gearRatio;
float targetDeg = 0.0, currentDeg = 0.0;
float integral = 0.0, lastError = 0.0;
bool servoEnabled = true;
int controlMode = 0; // 0: Web, 1: PWM, 2: Analog

// --- FreeRTOS & Time Variables ---
TaskHandle_t controlTaskHandle;
unsigned long lastLoopTime = 0;

// --- Motion Profiling & Safety ---
float maxVelocity = 300.0; // Max degrees per second allowed
float currentProfiledTarget = 0.0; 
unsigned long stallStartTime = 0;
bool isStalled = false;

// --- Smoothing/Filtering ---
float filterAlpha = 0.15; 

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

// --- Sensor Update ---
void updateEncoder() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0E); // AS5600 Angle Register
  if (Wire.endTransmission() != 0) return; // Prevent bus hang if sensor disconnects

  Wire.requestFrom(0x36, 2);
  if (Wire.available() >= 2) {
    int raw = (Wire.read() << 8) | Wire.read();
    int diff = raw - lastRaw;
    
    if (diff > 2048) diff -= 4096;
    if (diff < -2048) diff += 4096;
    
    totalRaw += diff;
    lastRaw = raw;

    currentDeg = ((totalRaw - homeOffset) * (360.0 / 4096.0)) * gearRatio;
  }
}

void resetPID() {
  integral = 0;
  lastError = 0;
  isStalled = false;
  stallStartTime = 0;
}

// --- Motor Output via Hardware PWM (Core v3.0.0+) ---
void driveMotor(int output) {
  if (!servoEnabled || isStalled) {
    ledcWrite(motorIN1, 0);
    ledcWrite(motorIN2, 0);
    return;
  }

  // Deadzone to prevent jitter
  if (abs(targetDeg - currentDeg) < tolerance) {
    ledcWrite(motorIN1, 0);
    ledcWrite(motorIN2, 0);
    return;
  }

  int speed = constrain(abs(output), 45, 255); 
  
  if (output > 0) {
    ledcWrite(motorIN1, speed);
    ledcWrite(motorIN2, 0);
  } else {
    ledcWrite(motorIN1, 0);
    ledcWrite(motorIN2, speed);
  }
}

// --- Time-Based PID Core ---
void runPID(float dt) {
  float error = currentProfiledTarget - currentDeg;
  
  // Anti-Windup: Only integrate if not outputting max power
  if (abs(error) < 50.0) { // Only build integral when close to target
    integral += (error * dt);
    integral = constrain(integral, -100.0, 100.0); 
  } else {
    integral = 0; // Clear integral during large moves
  }
  
  float derivative = (dt > 0) ? ((error - lastError) / dt) : 0;
  int output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // --- Stall Protection ---
  if (abs(output) >= 250 && abs(derivative) < 2.0) {
    if (stallStartTime == 0) stallStartTime = millis();
    else if (millis() - stallStartTime > 1500) { // 1.5 seconds of stall
      isStalled = true;
      Serial.println("STALL DETECTED! Motor Disabled.");
    }
  } else {
    stallStartTime = 0; 
  }

  driveMotor(output);
  lastError = error;
}

// --- Core 1 RTOS Task (Dedicated Control Loop) ---
void controlLoopTask(void * pvParameters) {
  lastLoopTime = micros();
  
  for(;;) {
    unsigned long now = micros();
    float dt = (now - lastLoopTime) / 1000000.0; // Delta time in seconds
    lastLoopTime = now;

    updateEncoder();

    // 1. Process Input
    float rawInput = targetDeg;
    if (controlMode == 1 && pulseWidth > 800 && pulseWidth < 2200) {
      rawInput = map(pulseWidth, 1000, 2000, 0, maxDeg);
    } else if (controlMode == 2) {
      int analogVal = 0;
      for(int i=0; i<4; i++) analogVal += analogRead(inputPin); 
      rawInput = map(analogVal / 4, 0, 4095, 0, maxDeg);
    }

    if (controlMode != 0) {
      targetDeg = (filterAlpha * rawInput) + ((1.0 - filterAlpha) * targetDeg);
    }

    // 2. Motion Profiling (Velocity Limiting)
    float maxChange = maxVelocity * dt;
    if (targetDeg > currentProfiledTarget + maxChange) {
      currentProfiledTarget += maxChange;
    } else if (targetDeg < currentProfiledTarget - maxChange) {
      currentProfiledTarget -= maxChange;
    } else {
      currentProfiledTarget = targetDeg;
    }

    // 3. Execute Control
    runPID(dt);

    vTaskDelay(2 / portTICK_PERIOD_MS); // Run at ~500Hz
  }
}

// --- HTML Dashboard (Classic V1 Style) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Pro Servo Panel</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: sans-serif; text-align: center; background: #1a1a1a; color: white; padding: 20px; }
  .card { background: #2d2d2d; padding: 20px; border-radius: 15px; display: inline-block; width: 380px; border: 1px solid #444; max-width: 100%; box-sizing: border-box; }
  .btn { padding: 12px; margin: 5px; cursor: pointer; border: none; border-radius: 8px; font-weight: bold; width: 45%; }
  .on { background: #2ecc71; color: white; }
  .save { background: #3498db; color: white; width: 93%; }
  .danger { background: #e74c3c; color: white; width: 93%; display: none; }
  input, select { width: 85%; padding: 10px; margin: 10px 0; border-radius: 5px; background: #444; color: white; border: none; }
  .row { display: flex; justify-content: space-around; align-items: center; }
  .row input { width: 60px; }
  .alert { background: #e74c3c; padding: 10px; border-radius: 5px; font-weight: bold; display: none; margin-bottom: 10px; }
</style></head>
<body>
  <div class="card">
    <h2>Smart Servo V2</h2>
    <div id="stallAlert" class="alert">⚠️ MOTOR STALLED</div>
    <h1 id="pos" style="margin-bottom: 0;">0.0</h1>
    <p style="margin-top: 5px; color: #aaa;">Target: <span id="tgt">0.0</span></p>
    <button id="resetBtn" class="btn danger" onclick="resetStall()">CLEAR FAULT</button>
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
    <input type="number" id="moveVal" placeholder="Degrees" style="width: 120px;">
    <button class="btn on" style="width:93%; background:#9b59b6" onclick="move()">WEB MOVE</button>
  </div>
<script>
  function move() { fetch('/move?val=' + document.getElementById('moveVal').value); }
  function resetStall() { 
    fetch('/resetstall'); 
    document.getElementById('stallAlert').style.display = 'none'; 
    document.getElementById('resetBtn').style.display = 'none'; 
  }
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
  setInterval(() => { 
    fetch('/status').then(r => r.json()).then(data => { 
      document.getElementById('pos').innerText = data.pos; 
      document.getElementById('tgt').innerText = data.tgt;
      if(data.stall) {
        document.getElementById('stallAlert').style.display = 'block';
        document.getElementById('resetBtn').style.display = 'inline-block';
      }
    }); 
  }, 250);
</script></body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  
  // Hardware Enable
  pinMode(motorENA, OUTPUT); 
  digitalWrite(motorENA, HIGH);
  pinMode(inputPin, INPUT);

  // Configure ESP32 LEDC (Hardware PWM) - Core 3.0.0+
  ledcAttach(motorIN1, pwmFreq, pwmResolution);
  ledcAttach(motorIN2, pwmFreq, pwmResolution);

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
  updateEncoder();
  targetDeg = currentDeg; 
  currentProfiledTarget = currentDeg;

  if (controlMode == 1) attachInterrupt(digitalPinToInterrupt(inputPin), handlePWM, CHANGE);

  // Start FreeRTOS Task on Core 1
  xTaskCreatePinnedToCore(
    controlLoopTask,   // Task function
    "PID_Loop",        // Name
    4096,              // Stack size
    NULL,              // Parameters
    1,                 // Priority
    &controlTaskHandle,// Handle
    1                  // Core Number (0 is WiFi, 1 is App)
  );

  WiFi.softAP("SmartServo_V2", "");

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
  server.on("/resetstall", [](AsyncWebServerRequest *f){ resetPID(); f->send(200); });
  
  server.on("/getparams", [](AsyncWebServerRequest *f){
    String json = "{\"p\":"+String(Kp)+",\"i\":"+String(Ki)+",\"d\":"+String(Kd)+",\"t\":"+String(tolerance)+",\"m\":"+String(controlMode)+",\"max\":"+String(maxDeg)+",\"r\":"+String(gearRatio)+"}";
    f->send(200, "application/json", json);
  });
  
  server.on("/status", [](AsyncWebServerRequest *f){ 
    String json = "{\"pos\":\"" + String(currentDeg, 1) + "\",\"tgt\":\"" + String(targetDeg, 1) + "\",\"stall\":" + (isStalled ? "true" : "false") + "}";
    f->send(200, "application/json", json);
  });
  
  server.on("/sethome", [](AsyncWebServerRequest *f){ 
    homeOffset = totalRaw; 
    targetDeg = 0; currentProfiledTarget = 0; currentDeg = 0;
    prefs.putLong("offset", homeOffset); 
    f->send(200); 
  });

  server.begin();
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
}