/*
 * ============================================================================
 * Circuit Documentation
 * ============================================================================
 * 
 * Summary
 * This circuit is designed to integrate various components including resistors, 
 * a microcontroller (ESP32), a power management system, and sensors. The primary 
 * function of this circuit is to manage power distribution from a solar panel 
 * and a battery, control a relay for switching purposes, and interface with a 
 * Purple Air Sensor for environmental monitoring. The ESP32 microcontroller 
 * serves as the central processing unit, handling data acquisition and control 
 * tasks.
 * 
 * Component List
 * 
 * Resistor (10k Ohms)
 *   Description: A resistor with a resistance of 10,000 Ohms.
 *   Purpose: Used for voltage division or current limiting.
 * 
 * Resistor (1k Ohms)
 *   Description: A resistor with a resistance of 1,000 Ohms.
 *   Purpose: Used for voltage division or current limiting.
 * 
 * ESP32 (30 pin)
 *   Description: A versatile microcontroller with Wi-Fi and Bluetooth capabilities.
 *   Purpose: Central processing unit for data acquisition and control.
 * 
 * Electrolytic Capacitor (10µF)
 *   Description: A capacitor with a capacitance of 10 microfarads.
 *   Purpose: Used for smoothing voltage fluctuations.
 * 
 * 5V Step Up/Down Converter
 *   Description: A power converter that can step up or step down voltage to 5V.
 *   Purpose: Provides a stable 5V output for powering components.
 * 
 * Purple Air Sensor
 *   Description: An air quality sensor.
 *   Purpose: Monitors environmental air quality.
 * 
 * 1 Channel Relay (5V)
 *   Description: A relay module that operates at 5V.
 *   Purpose: Used for switching high-power devices.
 * 
 * LiFEPO4 Battery (12.8V, 18Ah)
 *   Description: A rechargeable lithium iron phosphate battery.
 *   Purpose: Provides power storage for the circuit.
 * 
 * Solar Panel (380W)
 *   Description: A solar panel capable of generating 380 watts of power.
 *   Purpose: Provides renewable energy to the circuit.
 * 
 * Wiring Details
 * 
 * Resistor (10k Ohms)
 *   Pin 1: Connected to Pin 2 of the 1k Ohm Resistor.
 *   Pin 2: Connected to the positive pin of the Electrolytic Capacitor and 
 *          VP pin of the ESP32.
 * 
 * Resistor (1k Ohms)
 *   Pin 1: Connected to Pin 2 of the 10k Ohm Resistor.
 *   Pin 2: Connected to the ground net shared with the ESP32, Electrolytic 
 *          Capacitor, and other components.
 * 
 * ESP32 (30 pin)
 *   VP: Connected to Pin 2 of the 10k Ohm Resistor and the positive pin of 
 *       the Electrolytic Capacitor.
 *   GND: Connected to the ground net shared with the battery, solar panel, 
 *        and other components.
 *   Vin: Connected to the VOUT of the 5V Step Up/Down Converter.
 *   D27: Connected to the IN pin of the 1 Channel Relay.
 * 
 * Electrolytic Capacitor (10µF)
 *   Positive (+): Connected to Pin 2 of the 10k Ohm Resistor and VP pin of 
 *                 the ESP32.
 *   Negative (-): Connected to the ground net shared with the ESP32, battery, 
 *                 and other components.
 * 
 * 5V Step Up/Down Converter
 *   VIN: Connected to the positive terminals of the battery and solar panel.
 *   GND: Connected to the ground net shared with the ESP32, battery, and 
 *        other components.
 *   VOUT: Connected to the VCC of the 1 Channel Relay and Vin of the ESP32.
 * 
 * Purple Air Sensor
 *   Positive (+): Connected to the NC pin of the 1 Channel Relay.
 *   Negative (-): Connected to the ground net shared with the ESP32, battery, 
 *                 and other components.
 * 
 * 1 Channel Relay (5V)
 *   VCC: Connected to the VOUT of the 5V Step Up/Down Converter.
 *   GND: Connected to the ground net shared with the ESP32, battery, and 
 *        other components.
 *   IN: Connected to the D2 pin of the ESP32.
 *   NC: Connected to the positive pin of the Purple Air Sensor.
 *   COM: Connected to the positive terminals of the battery and solar panel.
 * 
 * LiFEPO4 Battery (12.8V, 18Ah)
 *   Positive (+): Connected to the VIN of the 5V Step Up/Down Converter and 
 *                 COM of the 1 Channel Relay.
 *   Negative (-): Connected to the ground net shared with the ESP32, solar 
 *                 panel, and other components.
 * 
 * Solar Panel (380W)
 *   Positive (+): Connected to the VIN of the 5V Step Up/Down Converter and 
 *                 COM of the 1 Channel Relay.
 *   Negative (-): Connected to the ground net shared with the ESP32, battery, 
 *                 and other components.
 * 
 * ============================================================================
 * Code Documentation
 * ============================================================================
 * 
 * PURPOSE:
 * Protects a 12.8V LiFePO4 battery by disconnecting a sensor load at low 
 * voltage and reconnecting after battery recovery. Hosts a web interface
 * showing live battery stats.
 * 
 * FEATURES:
 * - Measures battery voltage via voltage divider into ADC1 pin (GPIO36/VP)
 * - Controls NO relay to disconnect load at low voltage
 * - Simple logic: voltage above reconnect = ON, voltage below cutoff = OFF
 * - Web interface with live updates
 * - JSON API endpoint for external monitoring
 * - Dynamic threshold adjustment via web API
 * - Serial output for debugging (CSV format)
 * 
 * ============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>

// ============================================================================
// CONFIGURATION SECTION - MODIFY THESE VALUES FOR YOUR SETUP
// ============================================================================

// WiFi Credentials
// Replace with your WiFi network name and password
const char* WIFI_SSID = "ObieConnect";
const char* WIFI_PASS = "122ElmStreet";

// Pin Assignments
const int ADC_PIN = 36;      // GPIO36 (VP) - ADC1_CH0, works with WiFi enabled
const int RELAY_PIN = 27;    // GPIO27 (D27) - Controls relay module

// Relay Configuration
// Most relay modules are "active LOW" - setting pin LOW energizes the coil
// If your relay works opposite (HIGH = ON), set this to false
const bool RELAY_ACTIVE_LOW = true;

// Voltage Divider Configuration
// IMPORTANT: Choose resistor values that keep ADC voltage ≤ 3.3V
// 
// Recommended combinations for 12V LiFePO4:
// - Option A (best): RTOP=10kΩ, RBOT=1kΩ  → divides by 11, max input ~36V
// - Option B (okay): RTOP=100kΩ, RBOT=10kΩ → divides by 11, max input ~36V
// 
// Formula: Vadc = Vbat × (RBOT / (RTOP + RBOT))
// For safety: Vadc_max should be < 3.3V when Vbat is at maximum (14.6V)
//
// CURRENT WIRING: 100kΩ on top, 10kΩ on bottom (SAFE - same ratio as 10k/1k)
const float RTOP = 100000.0;  // Top resistor (Battery+ to ADC node) in Ohms  
const float RBOT = 10000.0;   // Bottom resistor (ADC node to GND) in Ohms

// ADC Configuration
const float VREF = 3.3;       // ESP32 reference voltage (typically 3.3V)
const int ADC_MAX = 4095;     // 12-bit ADC resolution (0-4095)
const int SAMPLES = 150;      // Number of samples to average for stable reading (increased to reduce tenths place noise)

// ============================================================================
// BATTERY PROTECTION THRESHOLDS - CUSTOMIZE THESE FOR YOUR NEEDS
// ============================================================================
//
// These values create HYSTERESIS to prevent relay chattering (rapid on/off)
//
// HOW IT WORKS:
// - When battery drops to or below V_CUTOFF → Load disconnects
// - Load stays OFF even as voltage rises slightly
// - When battery recovers to V_RECONNECT → Load reconnects
// - The gap between these values prevents rapid cycling
//
// EXAMPLE SCENARIOS:
//
// Scenario 1: CONSERVATIVE (Maximum Battery Protection)
//   const float V_CUTOFF = 12.4;     // Disconnect at 12.4V (~20% SOC)
//   const float V_RECONNECT = 13.0;  // Reconnect at 13.0V (~80% SOC)
//   USE WHEN: Battery longevity is priority, or battery is small/old
//
// Scenario 2: BALANCED (Default - Good for Most Users)
//   const float V_CUTOFF = 12.0;     // Disconnect at 12.0V (~5% SOC)
//   const float V_RECONNECT = 12.9;  // Reconnect at 12.9V (~70% SOC)
//   USE WHEN: Normal operation, good balance of protection and runtime
//
// Scenario 3: AGGRESSIVE (Maximum Runtime)
//   const float V_CUTOFF = 11.5;     // Disconnect at 11.5V (battery empty)
//   const float V_RECONNECT = 12.5;  // Reconnect at 12.5V (~40% SOC)
//   USE WHEN: Runtime is critical, battery is new/healthy
//   WARNING: May reduce battery lifespan if used frequently
//
// Scenario 4: TIGHT GAP (Fast Reconnection)
//   const float V_CUTOFF = 12.0;     // Disconnect at 12.0V
//   const float V_RECONNECT = 12.5;  // Reconnect at 12.5V (small gap)
//   USE WHEN: Battery recovers quickly, minimal voltage sag
//   WARNING: May cause relay chatter with high loads or weak batteries
//
// Scenario 5: WIDE GAP (Prevent Cycling)
//   const float V_CUTOFF = 12.0;     // Disconnect at 12.0V
//   const float V_RECONNECT = 13.5;  // Reconnect at 13.5V (large gap)
//   USE WHEN: Load causes significant voltage drop, or slow solar charging
//   BENEFIT: Ensures battery is nearly full before reconnecting
//
// IMPORTANT NOTES:
// - V_RECONNECT must ALWAYS be higher than V_CUTOFF
// - Minimum recommended gap: 0.3V (prevents chatter)
// - Typical gap: 0.5V - 1.0V (good for most applications)
// - Never set V_CUTOFF below 11.0V (damages LiFePO4 cells)
// - Never set V_RECONNECT above 14.0V (only reached during charging)
//
// TO CHANGE THESE VALUES:
// 1. Edit the two lines below
// 2. Upload the modified code to your ESP32
// 3. Check Serial Monitor to confirm new values are active
//
// QUICK REFERENCE:
// See THRESHOLD_QUICK_GUIDE.md for copy-paste examples and visual guide
// See README.md "Customizing Voltage Thresholds" section for detailed explanation
//
// ============================================================================

// Thresholds are changeable via web interface
float V_CUTOFF = 12.0;           // Disconnect load at or below this voltage (changeable)
float V_RECONNECT = 12.9;        // Reconnect load at or above this voltage (changeable)

// ============================================================================

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

WebServer server(80);         // Web server on port 80

bool loadEnabled = true;      // Current load state: true = load connected
bool autoMode = true;         // Control mode: true = automatic, false = manual

float lastVBat = 0.0;        // Last measured battery voltage
// int lastPct = 0;          // Percentage removed - not used

// Moving average for ultra-stable voltage display
const int DISPLAY_SAMPLES = 20;  // Increased for better stability in tenths place
float voltageHistory[20] = {0};
int voltageIndex = 0;
bool historyInitialized = false;  // Track if buffer is filled

// ============================================================================
// RELAY CONTROL FUNCTIONS
// ============================================================================

/**
 * Sets the relay energization state
 * 
 * @param energized true to energize relay coil, false to de-energize
 * 
 * Note: With NO (Normally Open) wiring:
 * - Relay NOT energized → NO open → Load has NO power
 * - Relay IS energized → NO closes → Load connected
 */
void setRelayEnergized(bool energized) {
  if (RELAY_ACTIVE_LOW) {
    // Active LOW relay: LOW = energized, HIGH = not energized
    digitalWrite(RELAY_PIN, energized ? LOW : HIGH);
  } else {
    // Active HIGH relay: HIGH = energized, LOW = not energized
    digitalWrite(RELAY_PIN, energized ? HIGH : LOW);
  }
}

/**
 * Applies the desired load state
 * 
 * @param wantLoadOn true to enable load (relay ON), false to disable (relay OFF)
 * 
 * This is the main function to control load power.
 * NO relay logic: To enable load, relay must be energized.
 */
void applyLoadState(bool wantLoadOn) {
  loadEnabled = wantLoadOn;
  // NO relay logic: To enable load, relay must be energized
  setRelayEnergized(wantLoadOn);
}

/**
 * Smooths voltage reading with moving average
 * Provides ultra-stable display value
 * 
 * Uses exponential moving average approach: fills buffer first, then uses
 * moving average to eliminate noise in tenths place
 */
float smoothVoltage(float newVoltage) {
  voltageHistory[voltageIndex] = newVoltage;
  voltageIndex = (voltageIndex + 1) % DISPLAY_SAMPLES;
  
  // Mark as initialized once buffer is filled
  if (voltageIndex == 0) {
    historyInitialized = true;
  }
  
  // If buffer not fully filled yet, use only available samples
  int samplesToUse = historyInitialized ? DISPLAY_SAMPLES : voltageIndex;
  
  float sum = 0;
  for (int i = 0; i < samplesToUse; i++) {
    sum += voltageHistory[i];
  }
  
  float average = sum / samplesToUse;
  
  // Round to nearest 0.01V to reduce flickering in tenths place
  // This helps stabilize the display while maintaining accuracy
  return round(average * 100.0) / 100.0;
}

// ============================================================================
// BATTERY VOLTAGE MEASUREMENT
// ============================================================================

/**
 * Reads and calculates the actual battery voltage
 * 
 * Process:
 * 1. Takes multiple ADC samples and averages them (reduces noise)
 * 2. Converts ADC value to voltage at the ADC pin
 * 3. Applies voltage divider formula to get actual battery voltage
 * 
 * @return Battery voltage in volts (float)
 * 
 * Formula breakdown:
 * - ADC reading → voltage at pin: Vadc = (ADC / ADC_MAX) × VREF
 * - Pin voltage → battery voltage: Vbat = Vadc × ((RTOP + RBOT) / RBOT)
 */
float readBatteryVoltage() {
  // Take multiple samples for averaging (reduces noise and spikes)
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(ADC_PIN);
    delayMicroseconds(200);  // Small delay between samples
  }
  
  // Calculate average ADC reading
  float avgADC = (float)sum / (float)SAMPLES;
  
  // Convert ADC value to voltage at the pin
  float vAdc = (avgADC / (float)ADC_MAX) * VREF;
  
  // Apply voltage divider formula to get actual battery voltage
  // Vbat = Vadc × (divider ratio)
  float vBat = vAdc * ((RTOP + RBOT) / RBOT);
  
  return vBat;
}

// ============================================================================
// BATTERY PERCENTAGE ESTIMATION
// ============================================================================

/**
 * Estimates LiFePO4 battery state of charge from voltage
 * 
 * @param v Battery voltage in volts
 * @return Estimated percentage (0-100)
 * 
 * LiFePO4 voltage curve (at rest, no load):
 * - 12.0V or below → 0% (empty)
 * - 12.0V - 12.4V → 0-20% (steep discharge curve)
 * - 12.4V - 12.8V → 20-70% (flat discharge plateau - typical LiFePO4)
 * - 12.8V - 13.2V → 70-90% (upper plateau)
 * - 13.2V - 13.6V → 90-100% (approaching full)
 * - 13.6V or above → 100% (full or charging)
 * 
 * Note: This is an approximation. Actual SOC depends on temperature,
 * load current, battery age, and rest time. Most accurate when battery
 * has been resting (no charge/discharge) for 30+ minutes.
 */
int lifepo4Percent(float v) {
  if (v <= 12.0) return 0;
  if (v >= 13.6) return 100;
  
  // 0-20%: 12.0V to 12.4V
  if (v < 12.4) return (int)((v - 12.0) / 0.4 * 20.0);
  
  // 20-70%: 12.4V to 12.8V (flat plateau region)
  if (v < 12.8) return 20 + (int)((v - 12.4) / 0.4 * 50.0);
  
  // 70-90%: 12.8V to 13.2V
  if (v < 13.2) return 70 + (int)((v - 12.8) / 0.4 * 20.0);
  
  // 90-100%: 13.2V to 13.6V
  return 90 + (int)((v - 13.2) / 0.4 * 10.0);
}

// ============================================================================
// WEB SERVER - HTML PAGE
// ============================================================================

/**
 * Generates the HTML page for the web interface
 * 
 * Features:
 * - Responsive design (works on mobile)
 * - Live updating display (fetches /status.json every second)
 * - Manual control buttons (Auto, Force On, Force Off)
 * - Shows voltage, load state, and control mode
 * 
 * @return HTML page as String
 */
String htmlPage() {
  String ip = WiFi.isConnected() ? WiFi.localIP().toString() : String("not connected");
  
  String s;
  
  // HTML head with responsive viewport and styling
  s += "<!doctype html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  s += "<title>Battery Monitor</title>";
  
  // Embedded CSS for clean, simple design
  s += "<style>";
  s += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Arial,sans-serif;padding:20px;background:#f8f9fa;margin:0}";
  s += ".card{border:none;border-radius:16px;padding:32px;max-width:500px;background:white;margin:0 auto;box-shadow:0 4px 12px rgba(0,0,0,0.08)}";
  s += "h1{font-size:24px;font-weight:600;color:#1a1a1a;margin:0 0 24px 0}";
  s += ".voltage{font-size:56px;font-weight:700;color:#2c3e50;margin:16px 0;text-align:center}";
  s += ".info{display:flex;justify-content:space-between;align-items:center;padding:16px;background:#f8f9fa;border-radius:12px;margin:12px 0;font-size:15px}";
  s += ".label{color:#6c757d;font-weight:500}";
  s += ".value{font-weight:600;color:#1a1a1a}";
  s += ".status{display:inline-block;padding:6px 14px;border-radius:8px;font-weight:600;font-size:14px}";
  s += ".status-on{background:#d4edda;color:#155724}";
  s += ".status-off{background:#f8d7da;color:#721c24}";
  s += ".status-auto{background:#fff3cd;color:#856404}";
  s += ".status-manual{background:#e2e3e5;color:#383d41}";
  s += ".threshold{color:#0066cc;font-weight:600}";
  s += ".buttons{display:flex;gap:8px;margin-top:24px}";
  s += "button{flex:1;padding:14px;border-radius:10px;border:none;background:#007bff;color:white;cursor:pointer;font-size:15px;font-weight:600;transition:all 0.2s}";
  s += "button:hover{background:#0056b3;transform:translateY(-1px)}";
  s += "button:active{transform:translateY(0)}";
  s += "small{color:#6c757d;display:block;text-align:center;margin-top:20px;font-size:13px}";
  s += "</style>";
  s += "</head><body>";
  
  // Main content card
  s += "<div class='card'>";
  s += "<h1>🔋 Battery Monitor</h1>";
  
  // Large voltage display
  s += "<div class='voltage' id='v'>--.-- V</div>";
  
  // Load status
  s += "<div class='info'>";
  s += "<span class='label'>Load Status</span>";
  s += "<span class='status' id='on'>--</span>";
  s += "</div>";
  
  // Control mode
  s += "<div class='info'>";
  s += "<span class='label'>Control Mode</span>";
  s += "<span class='status' id='mode'>--</span>";
  s += "</div>";
  
  // Thresholds
  s += "<div class='info'>";
  s += "<span class='label'>Turn OFF at</span>";
  s += "<span class='value'><span class='threshold' id='lower'>--</span> V</span>";
  s += "</div>";
  
  s += "<div class='info'>";
  s += "<span class='label'>Turn ON at</span>";
  s += "<span class='value'><span class='threshold' id='upper'>--</span> V</span>";
  s += "</div>";
  
  s += "<small style='text-align:center;color:#6c757d;margin:8px 0;display:block'>";
  s += "Hysteresis: OFF at ≤ lower, ON at ≥ upper, between = no change";
  s += "</small>";
  
  // Control buttons
  s += "<div class='buttons'>";
  s += "<button onclick=\"fetch('/relay?auto=1').then(()=>tick())\">Auto</button>";
  s += "<button onclick=\"fetch('/relay?on=1').then(()=>tick())\">Force ON</button>";
  s += "<button onclick=\"fetch('/relay?on=0').then(()=>tick())\">Force OFF</button>";
  s += "</div>";
  
  // Footer
  s += "<small>" + ip + " • Updates every second</small>";
  s += "</div>";
  
  // JavaScript for live updates
  s += "<script>";
  s += "async function tick(){";
  s += "  try{";
  s += "    const r = await fetch('/status.json',{cache:'no-store'});";
  s += "    const j = await r.json();";
  
  // Update voltage
  s += "    document.getElementById('v').textContent = j.voltage_v.toFixed(2) + ' V';";
  
  // Update load status
  s += "    const loadElem = document.getElementById('on');";
  s += "    loadElem.textContent = j.load_on ? 'ON' : 'OFF';";
  s += "    loadElem.className = 'status ' + (j.load_on ? 'status-on' : 'status-off');";
  
  // Update mode
  s += "    const modeElem = document.getElementById('mode');";
  s += "    modeElem.textContent = j.auto_mode ? 'AUTO' : 'MANUAL';";
  s += "    modeElem.className = 'status ' + (j.auto_mode ? 'status-auto' : 'status-manual');";
  
  // Update thresholds
  s += "    document.getElementById('lower').textContent = j.v_cutoff.toFixed(2);";
  s += "    document.getElementById('upper').textContent = j.v_reconnect.toFixed(2);";
  
  s += "  }catch(e){console.error('Fetch error:',e);}";
  s += "}";
  s += "setInterval(tick,1000); tick();";
  s += "</script>";
  
  s += "</body></html>";
  return s;
}

// ============================================================================
// WEB SERVER - REQUEST HANDLERS
// ============================================================================

/**
 * Handler for root path "/"
 * Serves the HTML web interface
 */
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

/**
 * Handler for "/status.json"
 * Returns current system status as JSON
 * 
 * JSON format:
 * {
 *   "voltage_v": 12.345,
 *   "load_on": true,
 *   "auto_mode": true,
 *   "v_cutoff": 12.0,
 *   "v_reconnect": 12.9,
 *   "uptime_ms": 123456
 * }
 */
void handleStatus() {
  String json = "{";
  json += "\"voltage_v\":" + String(lastVBat, 3) + ",";
  json += "\"load_on\":" + String(loadEnabled ? "true" : "false") + ",";
  json += "\"auto_mode\":" + String(autoMode ? "true" : "false") + ",";
  json += "\"v_cutoff\":" + String(V_CUTOFF, 2) + ",";
  json += "\"v_reconnect\":" + String(V_RECONNECT, 2) + ",";
  json += "\"uptime_ms\":" + String(millis());
  json += "}";
  server.send(200, "application/json", json);
}

/**
 * Handler for "/relay"
 * Allows manual control of the relay via query parameters
 * 
 * Query parameters:
 * - ?auto=1 → Enable automatic mode (hysteresis control)
 * - ?on=1   → Force load ON (disables auto mode)
 * - ?on=0   → Force load OFF (disables auto mode)
 * 
 * Examples:
 * - http://ESP32_IP/relay?auto=1
 * - http://ESP32_IP/relay?on=1
 * - http://ESP32_IP/relay?on=0
 */
void handleRelay() {
  // Check for auto mode request
  if (server.hasArg("auto") && server.arg("auto") == "1") {
    autoMode = true;
    // Don't change load state, let automatic control handle it
  }
  
  // Check for manual on/off request
  if (server.hasArg("on")) {
    autoMode = false;  // Disable automatic control
    bool turnOn = (server.arg("on") == "1");
    applyLoadState(turnOn);
  }
  
  server.send(200, "text/plain", "OK");
}

/**
 * Handler for "/settings"
 * Allows changing voltage thresholds dynamically
 * 
 * Query parameters:
 * - ?lower=12.0  → Set cutoff voltage (V_CUTOFF)
 * - ?upper=12.9  → Set reconnect voltage (V_RECONNECT)
 * 
 * Examples:
 * - http://ESP32_IP/settings?lower=11.5&upper=12.5
 * - http://ESP32_IP/settings?lower=12.0
 * - http://ESP32_IP/settings?upper=13.0
 * 
 * Returns current settings as JSON
 */
void handleSettings() {
  bool changed = false;
  
  // Accept threshold changes - can set one or both
  if (server.hasArg("lower")) {
    float newValue = server.arg("lower").toFloat();
    if (newValue > 8.0 && newValue < 15.0) {  // Safety limits
      V_CUTOFF = newValue;
      changed = true;
    }
  }
  
  if (server.hasArg("upper")) {
    float newValue = server.arg("upper").toFloat();
    if (newValue > 8.0 && newValue < 15.0) {  // Safety limits
      V_RECONNECT = newValue;
      changed = true;
    }
  }
  
  // Validate that upper >= lower (can be equal for no hysteresis)
  if (V_RECONNECT < V_CUTOFF) {
    V_RECONNECT = V_CUTOFF;  // Force upper to at least equal lower
  }
  
  // IMPORTANT: Re-evaluate load state immediately after threshold change
  // This ensures the system responds to new thresholds right away
  if (changed && autoMode) {
    Serial.println("! THRESHOLD CHANGE DETECTED - Re-evaluating load state");
    Serial.print("  New cutoff: ");
    Serial.print(V_CUTOFF, 2);
    Serial.print("V, New reconnect: ");
    Serial.print(V_RECONNECT, 2);
    Serial.print("V, Current voltage: ");
    Serial.print(lastVBat, 2);
    Serial.println("V");
    
    // Always check and apply the correct state based on new thresholds
    if (lastVBat <= V_CUTOFF) {
      // Voltage at or below cutoff - must be OFF
      if (loadEnabled) {
        Serial.println("  -> Turning load OFF (voltage at or below cutoff)");
        applyLoadState(false);
      } else {
        Serial.println("  -> Load already OFF (voltage at or below cutoff)");
      }
    }
    else if (lastVBat > V_CUTOFF) {
      // Voltage is above cutoff - should be ON
      // When thresholds change, if voltage > new cutoff, turn ON immediately
      // (ignore reconnect threshold when user manually changes thresholds)
      if (!loadEnabled) {
        Serial.println("  -> Turning load ON (voltage above new cutoff threshold)");
        applyLoadState(true);
      }
      else {
        // Load is ON and voltage > cutoff - keep it ON
        Serial.println("  -> Load stays ON (voltage above cutoff)");
      }
    }
    
    // Force relay update to ensure hardware state matches
    setRelayEnergized(loadEnabled);
    Serial.print("  -> Relay pin ");
    Serial.print(RELAY_PIN);
    Serial.print(" set to ");
    Serial.println(loadEnabled ? "LOW (energized)" : "HIGH (not energized)");
  }
  
  // Return current settings
  String json = "{";
  json += "\"v_cutoff\":" + String(V_CUTOFF, 2) + ",";
  json += "\"v_reconnect\":" + String(V_RECONNECT, 2) + ",";
  json += "\"changed\":" + String(changed ? "true" : "false");
  json += "}";
  
  server.send(200, "application/json", json);
  
  if (changed) {
    Serial.println("! SETTINGS CHANGED:");
    Serial.print("  Cutoff: ");
    Serial.print(V_CUTOFF, 2);
    Serial.print("V, Reconnect: ");
    Serial.print(V_RECONNECT, 2);
    Serial.print("V, Current voltage: ");
    Serial.print(lastVBat, 2);
    Serial.println("V");
  }
}

// ============================================================================
// SETUP FUNCTION - RUNS ONCE AT STARTUP
// ============================================================================

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  delay(300);  // Allow serial to stabilize
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("  Battery Cutoff Monitor Starting");
  Serial.println("========================================");
  
  // Configure relay pin as output
  pinMode(RELAY_PIN, OUTPUT);
  
  // Configure ADC settings
  analogReadResolution(12);                     // 12-bit resolution (0-4095)
  analogSetPinAttenuation(ADC_PIN, ADC_11db);  // 11dB attenuation (0-~3.3V, non-linear)
  
  Serial.println("ADC configured: 12-bit, 11dB attenuation");
  
  // Initialize system in automatic mode with load enabled
  autoMode = true;
  applyLoadState(true);  // Start with load connected
  
  Serial.println("Initial state: Load ON, Auto mode");
  Serial.print("Voltage divider: ");
  Serial.print(RTOP);
  Serial.print("Ω / ");
  Serial.print(RBOT);
  Serial.print("Ω (ratio: ");
  Serial.print((RTOP + RBOT) / RBOT, 2);
  Serial.println(")");
  
  Serial.print("Cutoff: ");
  Serial.print(V_CUTOFF);
  Serial.print("V, Reconnect: ");
  Serial.print(V_RECONNECT);
  Serial.println("V");
  
  // Connect to WiFi
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);  // Station mode (client)
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  // Wait up to 15 seconds for connection
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
    delay(250);
    Serial.print(".");
  }
  
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Open in browser: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed (timeout)");
    Serial.println("System will continue without WiFi");
  }
  
  // Configure web server routes
  server.on("/", handleRoot);              // Main page
  server.on("/status.json", handleStatus); // JSON API
  server.on("/relay", handleRelay);        // Manual control
  server.on("/settings", handleSettings);  // Change thresholds
  
  // Start web server
  server.begin();
  Serial.println("Web server started");
  Serial.println("========================================");
  Serial.println();
  Serial.println("CSV Output: Voltage,Percent,LoadState");
  Serial.println();
}

// ============================================================================
// MAIN LOOP - RUNS CONTINUOUSLY
// ============================================================================

void loop() {
  // Handle incoming web requests
  server.handleClient();
  
  // Perform periodic voltage measurement and control logic
  static unsigned long lastReadTime = 0;
  
  // Update every 250ms (4 times per second)
  if (millis() - lastReadTime >= 250) {
    lastReadTime = millis();
    
    // Read battery voltage with smoothing
    float rawVoltage = readBatteryVoltage();
    lastVBat = smoothVoltage(rawVoltage);  // Ultra-stable display value
    
    // Automatic control (only if in auto mode)
    // Logic: voltage > cutoff = ON, voltage <= cutoff = OFF
    // Reconnect threshold only prevents rapid cycling when recovering from OFF
    if (autoMode) {
      if (lastVBat <= V_CUTOFF) {
        // Voltage at or below cutoff - turn OFF
        if (loadEnabled) {
          Serial.print("! CUTOFF: Battery voltage (");
          Serial.print(lastVBat, 2);
          Serial.print("V) at or below cutoff (");
          Serial.print(V_CUTOFF, 2);
          Serial.println("V), turning load OFF");
          applyLoadState(false);
        }
      }
      else if (lastVBat > V_CUTOFF) {
        // Voltage is above cutoff - should be ON
        if (!loadEnabled) {
          // Load is currently OFF - check if we should turn it ON
          // Use reconnect threshold to prevent rapid cycling
          if (lastVBat >= V_RECONNECT) {
            Serial.print("! RECONNECT: Battery voltage (");
            Serial.print(lastVBat, 2);
            Serial.print("V) at or above reconnect (");
            Serial.print(V_RECONNECT, 2);
            Serial.println("V), turning load ON");
            applyLoadState(true);
          }
          // If voltage is between cutoff and reconnect, stay OFF (hysteresis)
        }
        else {
          // Load is ON and voltage > cutoff - keep it ON
          // This is the key fix: if voltage > cutoff, load should be ON
        }
      }
    }
    
    // Output CSV format to serial: voltage, load_state
    // This format is easy to parse and log externally
    Serial.print(lastVBat, 2);
    Serial.print(",");
    Serial.println(loadEnabled ? 1 : 0);
  }
}

// ============================================================================
// END OF CODE
// ============================================================================
