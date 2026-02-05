/*
 * ============================================================================
 * ESP32 Battery Cutoff Monitor - Power Optimized with Dashboard Control
 * ============================================================================
 * 
 * VERSION: 2.0
 * 
 * FEATURES:
 * - Deep sleep between readings (15 min default) for power savings
 * - HTTP POST to FastAPI backend for remote monitoring
 * - Two-way communication: receives commands from dashboard in POST response
 * - Remote calibration via dashboard
 * - Remote relay control (Force ON / Force OFF / Automatic)
 * - Remote threshold adjustment
 * - All settings persist in NVS across deep sleep and power cycles
 * 
 * POWER SAVINGS:
 * - Original: ~2,400 mAh/day (web server always on)
 * - Optimized: ~60 mAh/day (~97.5% reduction)
 * 
 * WORKFLOW PER WAKE CYCLE:
 * 1. Wake from deep sleep
 * 2. Read battery voltage
 * 3. Connect to WiFi
 * 4. POST data to backend
 * 5. Parse response for commands (relay mode, thresholds, calibration)
 * 6. Apply any changes
 * 7. Apply relay logic
 * 8. Enter deep sleep
 * 
 * ============================================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <time.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

// ============================================================================
// CONFIGURATION - MODIFY THESE FOR YOUR SETUP
// ============================================================================

// WiFi Credentials (safe placeholders for public code)
const char* WIFI_SSID = "YOUR_WIFI_SSID_HERE";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD_HERE";

// API Configuration
// NOTE: Keep real sensor IDs and tokens OUT of public repos.
//       Replace these placeholders with values from your dashboard in a private config.
const char* API_ENDPOINT = "https://sensor-dashboard.environmentaldashboard.org/api/esp32/voltage";
const char* SENSOR_ID = "REPLACE_WITH_SENSOR_ID_FROM_DASHBOARD";
const char* UPLOAD_TOKEN = "REPLACE_WITH_UPLOAD_TOKEN_FROM_DASHBOARD";

// Pin Assignments
const int ADC_PIN = 36;      // GPIO36 (VP) - ADC1_CH0
const int RELAY_PIN = 27;    // GPIO27 (D27) - Controls relay

// Relay Configuration (true = LOW energizes relay)
const bool RELAY_ACTIVE_LOW = false;

// Voltage Divider Configuration
const float RTOP = 10000.0;  // Top resistor in Ohms
const float RBOT = 1000.0;   // Bottom resistor in Ohms

// ADC Configuration
const float VREF = 3.3;
const int ADC_MAX = 4095;
const int SAMPLES = 200;

// ============================================================================
// DEFAULT VALUES (will be overridden by NVS or backend)
// ============================================================================

#define DEFAULT_WAKE_INTERVAL_MINUTES 15
#define DEFAULT_V_CUTOFF 12.0
#define DEFAULT_V_RECONNECT 12.6
#define DEFAULT_CALIBRATION_FACTOR 1.0
#define DEFAULT_RELAY_MODE "automatic"

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

Preferences preferences;

// Settings (loaded from NVS, updated by backend)
uint32_t wakeIntervalMinutes = DEFAULT_WAKE_INTERVAL_MINUTES;
float vCutoff = DEFAULT_V_CUTOFF;
float vReconnect = DEFAULT_V_RECONNECT;
float calibrationFactor = DEFAULT_CALIBRATION_FACTOR;
String relayMode = DEFAULT_RELAY_MODE;  // "automatic", "force_on", "force_off"

// Runtime state
float lastVBat = 0.0;
bool loadEnabled = true;

// Cycle counting (persisted in RTC memory to survive deep sleep)
RTC_DATA_ATTR unsigned long cycleCount = 0;
RTC_DATA_ATTR bool rtcLoadEnabled = true;

// ============================================================================
// NVS PERSISTENCE FUNCTIONS
// ============================================================================

/**
 * Load all settings from NVS
 */
void loadSettings() {
  preferences.begin("voltmeter", true);  // Read-only
  
  calibrationFactor = preferences.getFloat("cal_factor", DEFAULT_CALIBRATION_FACTOR);
  wakeIntervalMinutes = preferences.getUInt("wake_interval", DEFAULT_WAKE_INTERVAL_MINUTES);
  vCutoff = preferences.getFloat("v_cutoff", DEFAULT_V_CUTOFF);
  vReconnect = preferences.getFloat("v_reconnect", DEFAULT_V_RECONNECT);
  relayMode = preferences.getString("relay_mode", DEFAULT_RELAY_MODE);
  
  preferences.end();
  
  Serial.println("Settings loaded from NVS:");
  Serial.printf("  Calibration factor: %.4f\n", calibrationFactor);
  Serial.printf("  Wake interval: %d min\n", wakeIntervalMinutes);
  Serial.printf("  V_cutoff: %.2f V\n", vCutoff);
  Serial.printf("  V_reconnect: %.2f V\n", vReconnect);
  Serial.printf("  Relay mode: %s\n", relayMode.c_str());
}

/**
 * Save a single float setting to NVS
 */
void saveFloat(const char* key, float value) {
  preferences.begin("voltmeter", false);
  preferences.putFloat(key, value);
  preferences.end();
}

/**
 * Save a single uint setting to NVS
 */
void saveUInt(const char* key, uint32_t value) {
  preferences.begin("voltmeter", false);
  preferences.putUInt(key, value);
  preferences.end();
}

/**
 * Save a single string setting to NVS
 */
void saveString(const char* key, const String& value) {
  preferences.begin("voltmeter", false);
  preferences.putString(key, value);
  preferences.end();
}

// ============================================================================
// RELAY CONTROL FUNCTIONS
// ============================================================================

/**
 * Sets relay energization state
 */
void setRelayEnergized(bool energized) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_PIN, energized ? LOW : HIGH);
  } else {
    digitalWrite(RELAY_PIN, energized ? HIGH : LOW);
  }
}

/**
 * Applies load state and tracks changes
 */
void applyLoadState(bool wantLoadOn) {
  if (loadEnabled != wantLoadOn) {
    if (wantLoadOn) {
      cycleCount++;
      if (cycleCount >= 10000) cycleCount = 0;
      Serial.printf("Cycle count: %lu\n", cycleCount);
    }
  }
  
  loadEnabled = wantLoadOn;
  rtcLoadEnabled = wantLoadOn;
  setRelayEnergized(wantLoadOn);
}

/**
 * Apply relay logic based on mode and voltage
 */
void applyRelayLogic() {
  Serial.printf("Applying relay logic (mode: %s, voltage: %.2f V)\n", 
                relayMode.c_str(), lastVBat);
  
  if (relayMode == "force_on") {
    Serial.println("  -> Force ON (ignoring voltage)");
    applyLoadState(true);
  } 
  else if (relayMode == "force_off") {
    Serial.println("  -> Force OFF (ignoring voltage)");
    applyLoadState(false);
  } 
  else {
    // Automatic mode - use hysteresis logic
    if (lastVBat <= vCutoff) {
      if (loadEnabled) {
        Serial.printf("  -> Voltage LOW (<=%.2f) - turning OFF\n", vCutoff);
        applyLoadState(false);
      } else {
        Serial.println("  -> Voltage LOW - already OFF");
      }
    } 
    else if (lastVBat >= vReconnect) {
      if (!loadEnabled) {
        Serial.printf("  -> Voltage RECOVERED (>=%.2f) - turning ON\n", vReconnect);
        applyLoadState(true);
      } else {
        Serial.println("  -> Voltage OK - already ON");
      }
    } 
    else {
      Serial.println("  -> Voltage in hysteresis zone - no change");
    }
  }
  
  Serial.printf("Load state: %s\n", loadEnabled ? "ON" : "OFF");
}

/**
 * Holds relay GPIO during deep sleep
 */
void holdRelayPinDuringSleep() {
  rtc_gpio_hold_dis((gpio_num_t)RELAY_PIN);
  rtc_gpio_set_direction((gpio_num_t)RELAY_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
  int level = digitalRead(RELAY_PIN);
  rtc_gpio_set_level((gpio_num_t)RELAY_PIN, level);
  rtc_gpio_hold_en((gpio_num_t)RELAY_PIN);
  gpio_deep_sleep_hold_en();
}

// ============================================================================
// VOLTAGE MEASUREMENT FUNCTIONS
// ============================================================================

/**
 * Sort array for median calculation
 */
void sortArray(float arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

/**
 * Reads battery voltage with filtering and calibration
 */
float readBatteryVoltage() {
  float samples[SAMPLES];
  
  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = (float)analogRead(ADC_PIN);
    delayMicroseconds(500);
  }
  
  sortArray(samples, SAMPLES);
  
  // Trim top and bottom 10%
  int discard = SAMPLES / 10;
  int start = discard;
  int end = SAMPLES - discard;
  int count = end - start;
  
  long sum = 0;
  for (int i = start; i < end; i++) {
    sum += (long)samples[i];
  }
  
  float avgADC = (float)sum / (float)count;
  float vAdc = (avgADC / (float)ADC_MAX) * VREF;
  float vBat = vAdc * ((RTOP + RBOT) / RBOT);
  
  // Apply calibration
  vBat = vBat * calibrationFactor;
  
  return vBat;
}

/**
 * Reads raw battery voltage (no calibration) for calibration procedure
 */
float readBatteryVoltageRaw() {
  float samples[SAMPLES];
  
  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = (float)analogRead(ADC_PIN);
    delayMicroseconds(500);
  }
  
  sortArray(samples, SAMPLES);
  
  int discard = SAMPLES / 10;
  long sum = 0;
  for (int i = discard; i < SAMPLES - discard; i++) {
    sum += (long)samples[i];
  }
  
  float avgADC = (float)sum / (float)(SAMPLES - 2 * discard);
  float vAdc = (avgADC / (float)ADC_MAX) * VREF;
  
  // No calibration factor applied
  return vAdc * ((RTOP + RBOT) / RBOT);
}

/**
 * Performs calibration to target voltage
 */
void performCalibration(float targetVoltage) {
  Serial.println("\n========================================");
  Serial.println("  CALIBRATION PROCEDURE");
  Serial.println("========================================");
  Serial.printf("Target voltage: %.3f V\n", targetVoltage);
  
  // Take multiple raw readings for accuracy
  const int CAL_SAMPLES = 60;
  float sum = 0.0f;
  
  Serial.println("Taking raw voltage readings...");
  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum += readBatteryVoltageRaw();
    delay(30);
    if (i % 10 == 0) Serial.print(".");
  }
  Serial.println();
  
  float measuredRaw = sum / (float)CAL_SAMPLES;
  Serial.printf("Measured raw average: %.3f V\n", measuredRaw);
  
  if (measuredRaw > 0.1f) {
    float newFactor = targetVoltage / measuredRaw;
    
    if (newFactor > 0.5f && newFactor < 2.0f) {
      float oldFactor = calibrationFactor;
      calibrationFactor = newFactor;
      
      // Save to NVS
      saveFloat("cal_factor", calibrationFactor);
      
      Serial.printf("Old calibration factor: %.5f\n", oldFactor);
      Serial.printf("New calibration factor: %.5f\n", calibrationFactor);
      Serial.println("Calibration saved to NVS!");
      
      // Update lastVBat with new calibration
      lastVBat = readBatteryVoltage();
      Serial.printf("New calibrated reading: %.2f V\n", lastVBat);
    } else {
      Serial.printf("ERROR: Calculated factor %.5f is out of range (0.5-2.0)\n", newFactor);
    }
  } else {
    Serial.println("ERROR: Raw reading too low, calibration aborted");
  }
  
  Serial.println("========================================\n");
}

// ============================================================================
// WIFI AND HTTP FUNCTIONS
// ============================================================================

/**
 * Connects to WiFi with timeout
 */
bool connectWiFi() {
  Serial.print("Connecting to WiFi");
  
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
  } else {
    Serial.println("WiFi connection failed");
    return false;
  }
}

/**
 * Syncs time via NTP
 */
void syncTime() {
  setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0/2", 1);
  tzset();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  
  struct tm t;
  for (int i = 0; i < 5; i++) {
    if (getLocalTime(&t)) {
      Serial.printf("Time: %04d-%02d-%02d %02d:%02d\n",
        1900 + t.tm_year, 1 + t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min);
      return;
    }
    delay(200);
  }
  Serial.println("Time sync failed");
}

/**
 * Process commands received from backend
 */
void processCommands(JsonObject& commands) {
  Serial.println("\nProcessing commands from backend...");
  
  bool settingsChanged = false;
  
  // Check relay mode
  if (commands.containsKey("relay_mode")) {
    String newMode = commands["relay_mode"].as<String>();
    if (newMode != relayMode && 
        (newMode == "automatic" || newMode == "force_on" || newMode == "force_off")) {
      Serial.printf("  Relay mode: %s -> %s\n", relayMode.c_str(), newMode.c_str());
      relayMode = newMode;
      saveString("relay_mode", relayMode);
      settingsChanged = true;
    }
  }
  
  // Check voltage thresholds
  if (commands.containsKey("v_cutoff")) {
    float newCutoff = commands["v_cutoff"].as<float>();
    if (abs(newCutoff - vCutoff) > 0.01 && newCutoff >= 10.0 && newCutoff <= 14.0) {
      Serial.printf("  V_cutoff: %.2f -> %.2f\n", vCutoff, newCutoff);
      vCutoff = newCutoff;
      saveFloat("v_cutoff", vCutoff);
      settingsChanged = true;
    }
  }
  
  if (commands.containsKey("v_reconnect")) {
    float newReconnect = commands["v_reconnect"].as<float>();
    if (abs(newReconnect - vReconnect) > 0.01 && newReconnect >= 10.0 && newReconnect <= 14.0) {
      // Ensure minimum hysteresis gap
      if (newReconnect >= vCutoff + 0.3) {
        Serial.printf("  V_reconnect: %.2f -> %.2f\n", vReconnect, newReconnect);
        vReconnect = newReconnect;
        saveFloat("v_reconnect", vReconnect);
        settingsChanged = true;
      } else {
        Serial.printf("  V_reconnect %.2f rejected (must be >= %.2f)\n", 
                      newReconnect, vCutoff + 0.3);
      }
    }
  }
  
  // Check for calibration target
  if (commands.containsKey("calibration_target") && !commands["calibration_target"].isNull()) {
    float target = commands["calibration_target"].as<float>();
    if (target >= 10.0 && target <= 15.0) {
      Serial.printf("  Calibration requested: target = %.3f V\n", target);
      performCalibration(target);
      settingsChanged = true;
    }
  }
  
  if (!settingsChanged) {
    Serial.println("  No changes needed");
  }
}

/**
 * Posts voltage data to backend and processes response
 */
bool postVoltageData() {
  Serial.println("\nPosting data to backend...");
  
  // Build JSON payload using ArduinoJson
  StaticJsonDocument<512> doc;
  doc["sensor_id"] = SENSOR_ID;
  doc["voltage_v"] = round(lastVBat * 1000) / 1000.0;  // 3 decimal places
  doc["load_on"] = loadEnabled;
  doc["relay_mode"] = relayMode;
  doc["v_cutoff"] = vCutoff;
  doc["v_reconnect"] = vReconnect;
  doc["calibration_factor"] = calibrationFactor;
  doc["cycle_count"] = cycleCount;
  doc["wake_interval_minutes"] = wakeIntervalMinutes;
  doc["uptime_ms"] = millis();
  
  String jsonPayload;
  serializeJson(doc, jsonPayload);
  Serial.println("Payload: " + jsonPayload);
  
  // Determine if HTTPS
  bool useHTTPS = String(API_ENDPOINT).startsWith("https://");
  
  HTTPClient http;
  WiFiClientSecure *secureClient = nullptr;
  
  if (useHTTPS) {
    secureClient = new WiFiClientSecure;
    secureClient->setInsecure();  // NOTE: Insecure for simplicity; consider proper cert pinning for production
    if (!http.begin(*secureClient, API_ENDPOINT)) {
      Serial.println("HTTP begin failed");
      delete secureClient;
      return false;
    }
  } else {
    if (!http.begin(API_ENDPOINT)) {
      Serial.println("HTTP begin failed");
      return false;
    }
  }
  
  // Set headers
  http.addHeader("Content-Type", "application/json");
  http.addHeader("user-token", UPLOAD_TOKEN);
  http.setTimeout(10000);  // 10 second timeout
  
  // Send POST
  Serial.println("URL: " + String(API_ENDPOINT));
  int httpCode = http.POST(jsonPayload);
  bool success = false;
  
  if (httpCode > 0) {
    Serial.printf("HTTP Response: %d\n", httpCode);
    
    if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) {
      String response = http.getString();
      Serial.println("Response: " + response);
      
      // Parse response for commands
      StaticJsonDocument<512> responseDoc;
      DeserializationError error = deserializeJson(responseDoc, response);
      
      if (!error && responseDoc.containsKey("commands")) {
        JsonObject commands = responseDoc["commands"];
        processCommands(commands);
      }
      
      success = true;
    }
  } else {
    Serial.printf("HTTP Error: %s\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();
  if (secureClient) delete secureClient;
  
  return success;
}

// ============================================================================
// DEEP SLEEP FUNCTION
// ============================================================================

void goToDeepSleep() {
  Serial.println("\nPreparing for deep sleep...");
  
  // Hold relay state
  holdRelayPinDuringSleep();
  
  // Turn off WiFi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  // Set wake timer
  uint64_t sleepMicros = (uint64_t)wakeIntervalMinutes * 60ULL * 1000000ULL;
  esp_sleep_enable_timer_wakeup(sleepMicros);
  
  Serial.printf("Sleeping for %d minutes...\n", wakeIntervalMinutes);
  Serial.println("========================================\n");
  Serial.flush();
  
  esp_deep_sleep_start();
}

// ============================================================================
// SETUP - RUNS ONCE PER WAKE CYCLE
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("  Battery Monitor v2.0");
  Serial.println("  Power Optimized + Dashboard Control");
  Serial.println("========================================");
  
  // Configure pins FIRST
  pinMode(RELAY_PIN, OUTPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
  
  // Restore relay state from RTC memory immediately
  loadEnabled = rtcLoadEnabled;
  setRelayEnergized(loadEnabled);
  Serial.printf("Restored relay state: %s\n", loadEnabled ? "ON" : "OFF");
  
  // Load settings from NVS
  loadSettings();
  
  // ========================================
  // STEP 1: Read voltage
  // ========================================
  Serial.println("\n[1] Reading voltage...");
  lastVBat = readBatteryVoltage();
  Serial.printf("Battery: %.2f V (factor: %.4f)\n", lastVBat, calibrationFactor);
  
  // ========================================
  // STEP 2: Connect WiFi
  // ========================================
  Serial.println("\n[2] Connecting WiFi...");
  bool wifiConnected = connectWiFi();
  
  // ========================================
  // STEP 3: Sync time
  // ========================================
  if (wifiConnected) {
    Serial.println("\n[3] Syncing time...");
    syncTime();
  }
  
  // ========================================
  // STEP 4: POST data and get commands
  // ========================================
  if (wifiConnected) {
    Serial.println("\n[4] Communicating with backend...");
    if (postVoltageData()) {
      Serial.println("Communication successful!");
    } else {
      Serial.println("Communication failed (will retry next cycle)");
    }
  } else {
    Serial.println("\n[4] Skipping backend (no WiFi)");
  }
  
  // ========================================
  // STEP 5: Apply relay logic
  // ========================================
  Serial.println("\n[5] Applying relay logic...");
  Serial.printf("Thresholds: OFF at %.2f V, ON at %.2f V\n", vCutoff, vReconnect);
  applyRelayLogic();
  
  // ========================================
  // STEP 6: Enter deep sleep
  // ========================================
  Serial.println("\n[6] Entering deep sleep...");
  goToDeepSleep();
}

// ============================================================================
// LOOP - NOT USED
// ============================================================================

void loop() {
  // Deep sleep restarts from setup()
}

// ============================================================================
// END OF CODE
// ============================================================================

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
 *   IN: Connected to the D27 pin of the ESP32.
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
 * - Controls relay to disconnect load at low voltage
 * - Simple logic: voltage above reconnect = ON, voltage below cutoff = OFF
 * - Web interface with live updates
 * - JSON API endpoint for external monitoring
 * - Dynamic threshold adjustment via web API
 * - Serial output for debugging (CSV format)
 * 
 * NEW FEATURES ADDED (Day/Night profiles + UTF-8):
 * - Uses NTP time to select Day vs Night cutoff/reconnect thresholds automatically
 * - Day and Night thresholds are independently adjustable via existing /settings endpoint
 * - Web UI retained, plus UTF-8 charset enabled for correct emoji rendering
 * 
 * CALIBRATION FIX (NEW CHANGE YOU REQUESTED):
 * - Adds readBatteryVoltageRaw() (uncalibrated / unsmoothed)
 * - /settings?target=... now calibrates using RAW averaged readings (reliable)
 * 
 * NEW POWER-SAVING FEATURE (THIS UPDATE):
 * - ESP32 wakes periodically (e.g., every 1 minute), connects to WiFi, runs the web server
 *   for a short awake window (e.g., 60 seconds), then deep-sleeps to save power.
 * - Relay state is applied before sleeping and the relay GPIO is held during deep sleep.
 * 
 * IMPORTANT TRADEOFF:
 * - While the ESP32 is in deep sleep, the web UI will NOT be reachable.
 * - Battery/relay decisions are evaluated on wake (so switching resolution is the wake interval).
 * 
 * ============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>  // For persistent storage (replaces EEPROM on ESP32)
#include <time.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

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
const bool RELAY_ACTIVE_LOW = false;

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
const float RTOP = 10000.0;  // Top resistor (Battery+ to ADC node) in Ohms  
const float RBOT = 1000.0;   // Bottom resistor (ADC node to GND) in Ohms

// ADC Configuration
// ADC Calibration
// ESP32 ADC reference voltage varies from chip to chip (typically 1.0V to 1.2V)
// This calibration factor corrects for this variation
// To calibrate: Measure actual battery voltage with multimeter, then adjust CALIBRATION_FACTOR
// Formula: CALIBRATION_FACTOR = (MeasuredVoltage / DisplayedVoltage)
// Example: If multimeter shows 12.50V but display shows 12.30V, factor = 12.50/12.30 = 1.016
const float VREF = 3.3;       // ESP32 reference voltage (nominal, but varies per chip)
const int ADC_MAX = 4095;     // 12-bit ADC resolution (0-4095)
float CALIBRATION_FACTOR = 1.0;  // Calibration factor (1.0 = no calibration, adjust as needed)
Preferences preferences;  // For saving calibration to flash memory
const int SAMPLES = 200;      // Number of samples to average for stable reading (increased for maximum stability)

// ============================================================================
// POWER SAVING (NEW FEATURE)
// ============================================================================
//
// Wake up every WAKE_INTERVAL_MINUTES, connect to WiFi, serve UI for AWAKE_WINDOW_SECONDS,
// then go to deep sleep.
//
// NOTE: While sleeping, the web UI is offline.
// NOTE: Relay decisions are evaluated once per wake cycle.
// NOTE: These values are adjustable via web UI and saved to flash memory.
//
uint32_t WAKE_INTERVAL_MINUTES = 1;   // Wake interval in minutes (adjustable via UI)
uint32_t AWAKE_WINDOW_SECONDS  = 120; // Awake window in seconds (adjustable via UI)

// ============================================================================
// DAY / NIGHT PROFILE SETTINGS
// ============================================================================

const int NIGHT_START_HOUR = 22; // 10 PM
const int DAY_START_HOUR   = 8;  // 8 AM

float NIGHT_CUTOFF    = 11.5;
float NIGHT_RECONNECT = 12.5;

float DAY_CUTOFF      = 12.0;
float DAY_RECONNECT   = 12.6;

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

// Backward-compatible "active profile" thresholds (used by old UI/JSON keys)
// These are automatically set based on day/night profile
float V_CUTOFF = 12.0;           // Disconnect load at or below this voltage (changeable)
float V_RECONNECT = 12.6;        // Reconnect load at or above this voltage (changeable)

// ============================================================================

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

WebServer server(80);         // Web server on port 80

bool loadEnabled = true;      // Current load state: true = load connected
bool autoMode = true;         // Control mode: true = automatic, false = manual

float lastVBat = 0.0;        // Last measured battery voltage
// int lastPct = 0;          // Percentage removed - not used

// Cycle counting and history tracking
unsigned long cycleCount = 0;           // Number of times load has turned ON (resets at 10,000)
unsigned long lastSwitchTime = 0;       // Timestamp (millis) of last relay state change
unsigned long turnOnHistory[288];       // Store timestamps of turn-ON events (48 hours = 2880 minutes, but we'll use 288 entries for 10-minute resolution)
int historyIndex = 0;                   // Current index in history array
int historyCount = 0;                   // Number of entries in history
const int MAX_CYCLE_COUNT = 10000;      // Reset cycle count at this value
const unsigned long FORTY_EIGHT_HOURS_MS = 48UL * 60UL * 60UL * 1000UL; // 48 hours in milliseconds

// Moving average for ultra-stable voltage display
const int DISPLAY_SAMPLES = 30;  // Increased for maximum stability (30 samples = 7.5 seconds of history)
float voltageHistory[DISPLAY_SAMPLES] = {0};
int voltageIndex = 0;
bool historyInitialized = false;  // Track if buffer is filled

// Track awake window
unsigned long bootMillis = 0;

// ============================================================================
// DAY / NIGHT TIME HELPERS
// ============================================================================

/**
 * Determines if current time is night based on NIGHT_START_HOUR and DAY_START_HOUR
 * 
 * @return true if it's night time, false if day time (or if time not available)
 */
bool isNightTime() {
  struct tm t;
  if (!getLocalTime(&t)) return false; // If time not available, treat as day
  return (t.tm_hour >= NIGHT_START_HOUR || t.tm_hour < DAY_START_HOUR);
}

/**
 * Refreshes the active thresholds (V_CUTOFF, V_RECONNECT) based on current day/night profile
 * Enforces minimum hysteresis gap of 0.3V
 */
void refreshActiveThresholds() {
  bool night = isNightTime();

  float cutoff    = night ? NIGHT_CUTOFF    : DAY_CUTOFF;
  float reconnect = night ? NIGHT_RECONNECT : DAY_RECONNECT;

  // Enforce minimum hysteresis gap
  if (reconnect < cutoff + 0.3f) reconnect = cutoff + 0.3f;

  V_CUTOFF = cutoff;
  V_RECONNECT = reconnect;
}

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
 * Also tracks cycle count, last switch time, and turn-on history.
 */
void applyLoadState(bool wantLoadOn) {
  // Only track state changes (not redundant calls)
  if (loadEnabled != wantLoadOn) {
    // State is changing - record the timestamp
    lastSwitchTime = millis();
    
    // If turning ON, increment cycle count and record in history
    if (wantLoadOn) {
      cycleCount++;
      if (cycleCount >= MAX_CYCLE_COUNT) {
        cycleCount = 0;  // Reset at 10,000
        Serial.println("! Cycle count reset to 0 (reached 10,000)");
      }
      
      // Add to 48-hour history
      unsigned long currentTime = millis();
      turnOnHistory[historyIndex] = currentTime;
      historyIndex = (historyIndex + 1) % 288;  // Circular buffer
      if (historyCount < 288) {
        historyCount++;
      }
      
      Serial.print("! Cycle count: ");
      Serial.println(cycleCount);
    }
  }
  
  loadEnabled = wantLoadOn;
  // NO relay logic: To enable load, relay must be energized
  setRelayEnergized(wantLoadOn);
}

/**
 * Holds the relay GPIO state during deep sleep so the relay does not glitch.
 * GPIO27 is RTC-capable on ESP32, so RTC hold can be used 
 */
void holdRelayPinDuringSleep() {
  // Ensure the pin is RTC capable before using rtc_gpio_* (GPIO27 is RTC-capable)
  rtc_gpio_hold_dis((gpio_num_t)RELAY_PIN);          // disable hold while updating
  rtc_gpio_set_direction((gpio_num_t)RELAY_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);

  // Read current digital level and hold it
  int level = digitalRead(RELAY_PIN);
  rtc_gpio_set_level((gpio_num_t)RELAY_PIN, level);

  rtc_gpio_hold_en((gpio_num_t)RELAY_PIN);           // enable hold for the pin
  gpio_deep_sleep_hold_en();                         // enable deep-sleep hold globally
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
  
  // Round to nearest 0.05V for ultra-stable display (reduces minor fluctuations)
  // This helps stabilize the display significantly while maintaining reasonable accuracy
  return round(average * 20.0) / 20.0;
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
/**
 * Helper function to sort array for median calculation
 */
void sortArray(float arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

float readBatteryVoltage() {
  // Take multiple samples for median filtering (removes outliers) + averaging
  float samples[SAMPLES];
  
  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = (float)analogRead(ADC_PIN);
    delayMicroseconds(500);  // Increased delay to allow ADC to settle (reduces noise)
  }
  
  // Sort samples to find median (removes outliers from noise/spikes)
  sortArray(samples, SAMPLES);
  
  // Use median of middle 80% (discard top 10% and bottom 10% to remove extreme outliers)
  int discard = SAMPLES / 10;
  int start = discard;
  int end = SAMPLES - discard;
  int count = end - start;
  
  long sum = 0;
  for (int i = start; i < end; i++) {
    sum += (long)samples[i];
  }
  
  // Calculate average of trimmed samples
  float avgADC = (float)sum / (float)count;
  
  // Convert ADC value to voltage at the pin
  float vAdc = (avgADC / (float)ADC_MAX) * VREF;
  
  // Apply voltage divider formula to get actual battery voltage
  // Vbat = Vadc × (divider ratio)
  float vBat = vAdc * ((RTOP + RBOT) / RBOT);
  
  // Apply calibration factor to correct for ADC reference voltage variation
  // This compensates for differences in ESP32 chip VREF (typically 1.0V-1.2V)
  vBat = vBat * CALIBRATION_FACTOR;
  
  return vBat;
}

// ===========================
// NEW: RAW reading (NO calibration, NO smoothing) for target calibration
// ===========================
/**
 * Reads battery voltage without applying calibration factor or smoothing
 * Used for calibration purposes where we need the raw, uncalibrated reading
 * 
 * @return Raw battery voltage (uncalibrated, unsmoothed)
 */
float readBatteryVoltageRaw() {
  float samples[SAMPLES];

  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = (float)analogRead(ADC_PIN);
    delayMicroseconds(500);
  }

  sortArray(samples, SAMPLES);

  int discard = SAMPLES / 10;
  long sum = 0;
  for (int i = discard; i < SAMPLES - discard; i++) {
    sum += (long)samples[i];
  }

  float avgADC = (float)sum / (float)(SAMPLES - 2 * discard);
  float vAdc = (avgADC / (float)ADC_MAX) * VREF;

  // IMPORTANT: no calibration factor here
  return vAdc * ((RTOP + RBOT) / RBOT);
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
  s += "<!doctype html><html><head>";
  s += "<meta charset='UTF-8'>";
  s += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
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
  s += ".status-night{background:#343a40;color:#f8f9fa}";
  s += ".status-day{background:#ffeeba;color:#856404}";
  s += ".threshold{color:#0066cc;font-weight:600}";
  s += ".buttons{display:flex;gap:8px;margin-top:24px}";
  s += "button{flex:1;padding:14px;border-radius:10px;border:none;background:#007bff;color:white;cursor:pointer;font-size:15px;font-weight:600;transition:all 0.2s}";
  s += "button:hover{background:#0056b3;transform:translateY(-1px)}";
  s += "button:active{transform:translateY(0)}";
  s += "small{color:#6c757d;display:block;text-align:center;margin-top:20px;font-size:13px}";
  s += ".notification{position:fixed;top:20px;right:20px;background:#28a745;color:white;padding:16px 24px;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,0.3);z-index:10000;font-weight:600;font-size:15px;opacity:0;transform:translateY(-20px);transition:all 0.3s ease;pointer-events:none;max-width:400px;min-width:250px;display:block}";
  s += ".notification.show{opacity:1;transform:translateY(0);pointer-events:auto}";
  s += ".notification.error{background:#dc3545}";
  s += "</style>";
  s += "</head><body>";

  // Main content card
  s += "<div class='card'>";
  s += "<h1>🔋 Battery Monitor</h1>";

  // Large voltage display
  s += "<div class='voltage' id='v'>Loading...</div>";

  // Load status
  s += "<div class='info'>";
  s += "<span class='label'>Load Status</span>";
  s += "<span class='status' id='on'>Loading...</span>";
  s += "</div>";

  // Control mode
  s += "<div class='info'>";
  s += "<span class='label'>Control Mode</span>";
  s += "<span class='status' id='mode'>Loading...</span>";
  s += "</div>";

  s += "<div class='info'><span class='label'>Profile</span><span class='status' id='profile'>Loading...</span></div>";

  // Thresholds
  s += "<div class='info'>";
  s += "<span class='label'>Turn OFF at</span>";
  s += "<span class='value'><span class='threshold' id='lower'>--</span> V</span>";
  s += "</div>";

  s += "<div class='info'>";
  s += "<span class='label'>Turn ON at</span>";
  s += "<span class='value'><span class='threshold' id='upper'>--</span> V</span>";
  s += "</div>";

  s += "<div class='info'><span class='label'>Day OFF / ON</span><span class='value'><span class='threshold' id='day_lower'>--</span> / <span class='threshold' id='day_upper'>--</span> V</span></div>";
  s += "<div class='info'><span class='label'>Night OFF / ON</span><span class='value'><span class='threshold' id='night_lower'>--</span> / <span class='threshold' id='night_upper'>--</span> V</span></div>";

  s += "<small style='text-align:center;color:#6c757d;margin:8px 0;display:block'>";
  s += "Hysteresis: OFF at ≤ lower, ON at ≥ upper, between = no change";
  s += "</small>";

  // Power saving settings
  s += "<div style='margin-top:24px;padding-top:24px;border-top:1px solid #e9ecef'>";
  s += "<h2 style='font-size:18px;font-weight:600;color:#1a1a1a;margin:0 0 16px 0'>⚡ Power Saving</h2>";

  s += "<div class='info' style='flex-direction:column;align-items:flex-start;gap:8px'>";
  s += "<div style='display:flex;justify-content:space-between;width:100%;align-items:center'>";
  s += "<span class='label'>Wake Interval</span>";
  s += "<div style='display:flex;gap:8px;align-items:center'>";
  s += "<input type='number' id='wake_interval' min='1' max='1440' step='1' style='width:80px;padding:8px;border:1px solid #ced4da;border-radius:6px;font-size:14px;text-align:center' />";
  s += "<span style='color:#6c757d;font-size:14px'>minutes</span>";
  s += "<button id='save_wake' style='padding:8px 16px;margin-left:8px;font-size:13px;flex:none'>Save</button>";
  s += "</div></div>";
  s += "<small style='color:#6c757d;font-size:12px'>How often the device wakes up (1-1440 min)</small>";
  s += "</div>";

  s += "<div class='info' style='flex-direction:column;align-items:flex-start;gap:8px;margin-top:12px'>";
  s += "<div style='display:flex;justify-content:space-between;width:100%;align-items:center'>";
  s += "<span class='label'>Awake Window</span>";
  s += "<div style='display:flex;gap:8px;align-items:center'>";
  s += "<input type='number' id='awake_window' min='10' max='300' step='5' style='width:80px;padding:8px;border:1px solid #ced4da;border-radius:6px;font-size:14px;text-align:center' />";
  s += "<span style='color:#6c757d;font-size:14px'>seconds</span>";
  s += "<button id='save_awake' style='padding:8px 16px;margin-left:8px;font-size:13px;flex:none'>Save</button>";
  s += "</div></div>";
  s += "<small style='color:#6c757d;font-size:12px'>How long the device stays awake to serve web UI (10-300 sec)</small>";
  s += "</div>";
  s += "</div>";

  // Control buttons (NO inline onclick — wired in JS so tick() is in-scope)
  s += "<div class='buttons'>";
  s += "<button id='btn_auto'>Auto</button>";
  s += "<button id='btn_on'>Force ON</button>";
  s += "<button id='btn_off'>Force OFF</button>";
  s += "</div>";

  // Footer
  s += "<small>" + ip + " • Updates every 5 seconds (while awake)</small>";
  s += "</div>";

  // Single notification toast (ONLY ONCE)
  s += "<div id='notification' class='notification' style='display:none'></div>";

  // JavaScript for live updates
  s += "<script>";
  s += "(function(){";
  s += "  function byId(id){ return document.getElementById(id); }";
  s += "  function setText(id, text){ var el = byId(id); if(el) el.textContent = text; }";

  s += "  function showNotification(message, isError){";
  s += "    var notif = byId('notification');";
  s += "    if(!notif){ return; }";
  s += "    notif.textContent = message;";
  s += "    notif.className = 'notification' + (isError ? ' error' : '');";
  s += "    notif.style.display = 'block';";
  s += "    notif.style.visibility = 'visible';";
  s += "    setTimeout(function(){ if(notif.classList) notif.classList.add('show'); }, 10);";
  s += "    setTimeout(function(){";
  s += "      if(notif.classList) notif.classList.remove('show');";
  s += "      setTimeout(function(){ notif.textContent=''; notif.style.display='none'; }, 300);";
  s += "    }, 2500);";
  s += "  }";

  s += "  function xhrJson(url, timeoutMs, cb){";
  s += "    var x = new XMLHttpRequest();";
  s += "    x.open('GET', url, true);";
  s += "    x.timeout = timeoutMs;";
  s += "    x.onreadystatechange = function(){";
  s += "      if(x.readyState !== 4) return;";
  s += "      if(x.status >= 200 && x.status < 300){";
  s += "        try{ cb(null, JSON.parse(x.responseText)); }catch(e){ cb(e); }";
  s += "      } else { cb(new Error('HTTP ' + x.status)); }";
  s += "    };";
  s += "    x.ontimeout = function(){ cb(new Error('timeout')); };";
  s += "    x.onerror = function(){ cb(new Error('network')); };";
  s += "    x.send();";
  s += "  }";

  s += "  function xhrText(url, timeoutMs, cb){";
  s += "    var x = new XMLHttpRequest();";
  s += "    x.open('GET', url, true);";
  s += "    x.timeout = timeoutMs;";
  s += "    x.onreadystatechange = function(){";
  s += "      if(x.readyState !== 4) return;";
  s += "      if(x.status >= 200 && x.status < 300){ cb(null, x.responseText); }";
  s += "      else { cb(new Error('HTTP ' + x.status)); }";
  s += "    };";
  s += "    x.ontimeout = function(){ cb(new Error('timeout')); };";
  s += "    x.onerror = function(){ cb(new Error('network')); };";
  s += "    x.send();";
  s += "  }";

  s += "  function applyStatus(j){";
  s += "    if(j && j.voltage_v != null) setText('v', Number(j.voltage_v).toFixed(2) + ' V');";
  s += "    if(j && j.load_on != null){";
  s += "      var loadElem = byId('on');";
  s += "      if(loadElem){";
  s += "        loadElem.textContent = j.load_on ? 'ON' : 'OFF';";
  s += "        loadElem.className = 'status ' + (j.load_on ? 'status-on' : 'status-off');";
  s += "      }";
  s += "    }";
  s += "    if(j && j.auto_mode != null){";
  s += "      var modeElem = byId('mode');";
  s += "      if(modeElem){";
  s += "        modeElem.textContent = j.auto_mode ? 'AUTO' : 'MANUAL';";
  s += "        modeElem.className = 'status ' + (j.auto_mode ? 'status-auto' : 'status-manual');";
  s += "      }";
  s += "    }";
  s += "    if(j && j.night != null){";
  s += "      var profileElem = byId('profile');";
  s += "      if(profileElem){ profileElem.textContent = j.night ? '🌙 Night' : '☀️ Day'; }";
  s += "    }";
  s += "    if(j && j.v_cutoff != null) setText('lower', Number(j.v_cutoff).toFixed(2));";
  s += "    if(j && j.v_reconnect != null) setText('upper', Number(j.v_reconnect).toFixed(2));";
  s += "    if(j){";
  s += "      if(j.day_cutoff != null) setText('day_lower', Number(j.day_cutoff).toFixed(2));";
  s += "      if(j.day_reconnect != null) setText('day_upper', Number(j.day_reconnect).toFixed(2));";
  s += "      if(j.night_cutoff != null) setText('night_lower', Number(j.night_cutoff).toFixed(2));";
  s += "      if(j.night_reconnect != null) setText('night_upper', Number(j.night_reconnect).toFixed(2));";

  s += "      var wakeInput = byId('wake_interval');";
  s += "      var awakeInput = byId('awake_window');";
  s += "      var active = document.activeElement;";

  s += "      if(wakeInput && active !== wakeInput){";
  s += "        var lastSave = Number(wakeInput.getAttribute('data-last-save') || '0');";
  s += "        if(Date.now() - lastSave > 2000 && j.wake_interval_minutes != null){ wakeInput.value = j.wake_interval_minutes; }";
  s += "      }";
  s += "      if(awakeInput && active !== awakeInput){";
  s += "        var lastSave2 = Number(awakeInput.getAttribute('data-last-save') || '0');";
  s += "        if(Date.now() - lastSave2 > 2000 && j.awake_window_seconds != null){ awakeInput.value = j.awake_window_seconds; }";
  s += "      }";
  s += "    }";
  s += "  }";

  // tick() stays inside this closure; buttons are wired here too
  s += "  function tick(){";
  s += "    if(byId('v') && byId('v').textContent === 'Loading...'){ setText('v','Connecting...'); }";
  s += "    xhrJson('/status.json', 5000, function(err, j){";
  s += "      if(err){";
  s += "        setText('v', (err.message === 'timeout') ? 'Timeout' : ('Error: ' + err.message));";
  s += "        setText('on', 'Error');";
  s += "        setText('mode', 'Error');";
  s += "        return;";
  s += "      }";
  s += "      applyStatus(j);";
  s += "    });";
  s += "  }";

 s += "  function relayCmd(url){";
s += "    showNotification('Sending command...', false);";
s += "    var fullUrl = url + (url.indexOf('?') >= 0 ? '&' : '?') + '_=' + Date.now();";
s += "    xhrText(fullUrl, 5000, function(err){";
s += "      if(err){ showNotification('Command failed: ' + err.message, true); return; }";
s += "      showNotification('✓ Command sent', false);";
s += "      setTimeout(function(){ tick(); }, 150);";
s += "    });";
s += "  }";


  s += "  function updatePowerSettings(){";
  s += "    var wakeInput = byId('wake_interval');";
  s += "    var awakeInput = byId('awake_window');";
  s += "    if(!wakeInput || !awakeInput){ showNotification('UI error: inputs missing', true); return; }";
  s += "    var wake = parseInt(wakeInput.value, 10);";
  s += "    var awake = parseInt(awakeInput.value, 10);";
  s += "    if(isNaN(wake) || isNaN(awake)){ showNotification('Enter valid numbers', true); return; }";
  s += "    if(wake < 1 || wake > 1440){ showNotification('Wake interval must be 1–1440 min', true); return; }";
  s += "    if(awake < 10 || awake > 300){ showNotification('Awake window must be 10–300 sec', true); return; }";
  s += "    showNotification('Saving power settings...', false);";
  s += "    var url = '/settings?wake_interval=' + encodeURIComponent(wake) + '&awake_window=' + encodeURIComponent(awake);";
  s += "    xhrJson(url, 8000, function(err, j){";
  s += "      if(err){ showNotification('Save failed: ' + err.message, true); return; }";
  s += "      var now = String(Date.now());";
  s += "      wakeInput.setAttribute('data-last-save', now);";
  s += "      awakeInput.setAttribute('data-last-save', now);";
  s += "      if(j && j.wake_interval_minutes != null) wakeInput.value = j.wake_interval_minutes;";
  s += "      if(j && j.awake_window_seconds != null) awakeInput.value = j.awake_window_seconds;";
  s += "      showNotification('✓ Power settings saved', false);";
  s += "      setTimeout(function(){ tick(); }, 300);";
  s += "    });";
  s += "  }";

  s += "  function attach(){";
  s += "    var sw = byId('save_wake');";
  s += "    var sa = byId('save_awake');";
  s += "    if(sw) sw.onclick = function(e){ if(e && e.preventDefault) e.preventDefault(); updatePowerSettings(); };";
  s += "    if(sa) sa.onclick = function(e){ if(e && e.preventDefault) e.preventDefault(); updatePowerSettings(); };";

  s += "    var bAuto = byId('btn_auto');";
  s += "    var bOn   = byId('btn_on');";
  s += "    var bOff  = byId('btn_off');";
  s += "    if(bAuto) bAuto.onclick = function(e){ if(e && e.preventDefault) e.preventDefault(); relayCmd('/relay?auto=1'); };";
  s += "    if(bOn)   bOn.onclick   = function(e){ if(e && e.preventDefault) e.preventDefault(); relayCmd('/relay?on=1'); };";
  s += "    if(bOff)  bOff.onclick  = function(e){ if(e && e.preventDefault) e.preventDefault(); relayCmd('/relay?on=0'); };";
  s += "  }";

  // Init
  s += "  attach();";
  s += "  tick();";
  s += "  setInterval(function(){ tick(); }, 5000);";
  s += "})();";
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
  server.send(200, "text/html; charset=UTF-8", htmlPage());
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
  refreshActiveThresholds();
  bool night = isNightTime();

  // Calculate turn-ON count in last 48 hours
  unsigned long currentTime = millis();
  unsigned long fortyEightHoursAgo = currentTime - FORTY_EIGHT_HOURS_MS;
  int turnOnCount48h = 0;
  
  for (int i = 0; i < historyCount; i++) {
    // Handle wrap-around in circular buffer
    int idx = (historyIndex - historyCount + i + 288) % 288;
    if (turnOnHistory[idx] >= fortyEightHoursAgo) {
      turnOnCount48h++;
    }
  }
  
  String json = "{";
  json += "\"voltage_v\":" + String(lastVBat, 3) + ",";
  json += "\"load_on\":" + String(loadEnabled ? "true" : "false") + ",";
  json += "\"auto_mode\":" + String(autoMode ? "true" : "false") + ",";
  json += "\"night\":" + String(night ? "true" : "false") + ",";
  json += "\"v_cutoff\":" + String(V_CUTOFF, 2) + ",";
  json += "\"v_reconnect\":" + String(V_RECONNECT, 2) + ",";
  json += "\"day_cutoff\":" + String(DAY_CUTOFF, 2) + ",";
  json += "\"day_reconnect\":" + String(DAY_RECONNECT, 2) + ",";
  json += "\"night_cutoff\":" + String(NIGHT_CUTOFF, 2) + ",";
  json += "\"night_reconnect\":" + String(NIGHT_RECONNECT, 2) + ",";
  json += "\"calibration_factor\":" + String(CALIBRATION_FACTOR, 4) + ",";
  json += "\"cycle_count\":" + String(cycleCount) + ",";
  json += "\"turn_on_count_48h\":" + String(turnOnCount48h) + ",";
  json += "\"last_switch_time_ms\":" + String(lastSwitchTime) + ",";
  json += "\"wake_interval_minutes\":" + String(WAKE_INTERVAL_MINUTES) + ",";
  json += "\"awake_window_seconds\":" + String(AWAKE_WINDOW_SECONDS) + ",";
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
  bool requested = false;

  Serial.print("HTTP /relay args: ");
  Serial.println(server.args());

  // ?auto=1  -> enable automatic mode (no immediate change forced)
  if (server.hasArg("auto") && server.arg("auto") == "1") {
    autoMode = true;
    requested = true;
    Serial.println("-> Set autoMode = true");
  }

  // ?on=1 or ?on=0 -> manual override + apply relay
  if (server.hasArg("on")) {
    autoMode = false;
    bool turnOn = (server.arg("on") == "1");
    applyLoadState(turnOn);
    requested = true;

    Serial.print("-> Manual command: load ");
    Serial.println(turnOn ? "ON" : "OFF");
  }

  // Return JSON so UI can confirm state
  String json = "{";
  json += "\"ok\":true,";
  json += "\"requested\":" + String(requested ? "true" : "false") + ",";
  json += "\"load_on\":" + String(loadEnabled ? "true" : "false") + ",";
  json += "\"auto_mode\":" + String(autoMode ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}


/**
 * Handler for "/settings"
 * Allows changing voltage thresholds and calibration dynamically
 * 
 * Query parameters:
 * - ?lower=12.0  → Set cutoff voltage (V_CUTOFF)
 * - ?upper=12.9  → Set reconnect voltage (V_RECONNECT)
 * - ?calibrate=1.1937  → Set calibration factor directly
 * - ?target=13.5  → Auto-calibrate to target voltage (calculates factor automatically)
 * 
 * Examples:
 * - http://ESP32_IP/settings?lower=11.5&upper=12.5
 * - http://ESP32_IP/settings?calibrate=1.1937  (manual calibration)
 * - http://ESP32_IP/settings?target=13.5  (auto-calibration - easier!)
 * 
 * Returns current settings as JSON
 */
void handleSettings() {
  bool changed = false;

  refreshActiveThresholds();

  // Threshold updates apply to ACTIVE profile (day vs night)
  if (server.hasArg("lower")) {
    float newValue = server.arg("lower").toFloat();
    if (newValue > 8.0f && newValue < 15.0f) {
      if (isNightTime()) NIGHT_CUTOFF = newValue;
      else DAY_CUTOFF = newValue;
      changed = true;
    }
  }

  if (server.hasArg("upper")) {
    float newValue = server.arg("upper").toFloat();
    if (newValue > 8.0f && newValue < 15.0f) {
      if (isNightTime()) {
        if (newValue < NIGHT_CUTOFF + 0.3f) newValue = NIGHT_CUTOFF + 0.3f;
        NIGHT_RECONNECT = newValue;
      } else {
        if (newValue < DAY_CUTOFF + 0.3f) newValue = DAY_CUTOFF + 0.3f;
        DAY_RECONNECT = newValue;
      }
      changed = true;
    }
  }
  
  // Accept calibration factor changes
  // Method 1: Direct factor setting
  if (server.hasArg("calibrate")) {
    float newCal = server.arg("calibrate").toFloat();
    if (newCal > 0.5 && newCal < 2.0) {  // Extended range to handle larger calibration needs
      CALIBRATION_FACTOR = newCal;
      changed = true;
      
      //Save calibration factor to flash memory so it persists across reboots
      preferences.begin("voltmeter", false);
      preferences.putFloat("cal_factor", CALIBRATION_FACTOR);
      preferences.end();
      
      Serial.print("! CALIBRATION FACTOR CHANGED to: ");
      Serial.println(CALIBRATION_FACTOR, 4);
      Serial.println("  Calibration saved to flash memory (will persist after reboot)");
      Serial.println("  Calibration formula: factor = measured_voltage / displayed_voltage");
    }
  }
  
  // ===========================
  // FIXED Target calibration (uses RAW averaged readings)
  // ===========================
  if (server.hasArg("target")) {
    float targetVoltage = server.arg("target").toFloat();

    if (targetVoltage > 8.0f && targetVoltage < 20.0f) {
      bool prevAuto = autoMode;
      bool prevLoad = loadEnabled;
      autoMode = false;

      Serial.println("! AUTO-CALIBRATION START");
      Serial.print("  Target voltage: ");
      Serial.println(targetVoltage, 3);

      delay(300);

      const int CAL_SAMPLES = 60; // ~2 seconds
      float sum = 0.0f;
      for (int i = 0; i < CAL_SAMPLES; i++) {
        sum += readBatteryVoltageRaw();   // <-- uncalibrated, unsmoothed
        delay(30);
      }

      float measuredRaw = sum / (float)CAL_SAMPLES;

      Serial.print("  Measured RAW avg: ");
      Serial.println(measuredRaw, 3);

      if (measuredRaw > 0.1f) {
        float newFactor = targetVoltage / measuredRaw;

        if (newFactor > 0.5f && newFactor < 2.0f) {
          CALIBRATION_FACTOR = newFactor;
          changed = true;

          preferences.begin("voltmeter", false);
          preferences.putFloat("cal_factor", CALIBRATION_FACTOR);
          preferences.end();

          Serial.print("  -> New calibration factor: ");
          Serial.println(CALIBRATION_FACTOR, 5);
          Serial.println("  Calibration saved to flash memory");
        } else {
          Serial.println("! AUTO-CALIBRATION FAILED: factor out of range (0.5 to 2.0)");
        }
      } else {
        Serial.println("! AUTO-CALIBRATION FAILED: raw reading invalid/too small");
      }

      autoMode = prevAuto;
      applyLoadState(prevLoad);

      Serial.println("! AUTO-CALIBRATION END");
    }
  }

  // Power saving settings (wake interval and awake window)
  if (server.hasArg("wake_interval")) {
    uint32_t newWake = server.arg("wake_interval").toInt();
    if (newWake >= 1 && newWake <= 1440) {  // 1 minute to 24 hours
      WAKE_INTERVAL_MINUTES = newWake;
      changed = true;
      
      preferences.begin("voltmeter", false);
      preferences.putUInt("wake_interval", WAKE_INTERVAL_MINUTES);
      preferences.end();
      
      Serial.print("! WAKE INTERVAL CHANGED to: ");
      Serial.print(WAKE_INTERVAL_MINUTES);
      Serial.println(" minutes");
      Serial.println("  Changes take effect on next wake cycle");
    }
  }

  if (server.hasArg("awake_window")) {
    uint32_t newAwake = server.arg("awake_window").toInt();
    if (newAwake >= 10 && newAwake <= 300) {  // 10 seconds to 5 minutes
      AWAKE_WINDOW_SECONDS = newAwake;
      changed = true;
      
      preferences.begin("voltmeter", false);
      preferences.putUInt("awake_window", AWAKE_WINDOW_SECONDS);
      preferences.end();
      
      Serial.print("! AWAKE WINDOW CHANGED to: ");
      Serial.print(AWAKE_WINDOW_SECONDS);
      Serial.println(" seconds");
      Serial.println("  Changes take effect on next wake cycle");
    }
  }

  refreshActiveThresholds();
  
  // Immediate re-evaluation if thresholds changed and autoMode is on
  if (changed && autoMode) {
    if (lastVBat <= V_CUTOFF) applyLoadState(false);
    else applyLoadState(true);
  }
  
  // Return current settings (always return current values, even if not changed)
  String json = "{";
  json += "\"v_cutoff\":" + String(V_CUTOFF, 2) + ",";
  json += "\"v_reconnect\":" + String(V_RECONNECT, 2) + ",";
  json += "\"calibration_factor\":" + String(CALIBRATION_FACTOR, 4) + ",";
  json += "\"wake_interval_minutes\":" + String(WAKE_INTERVAL_MINUTES) + ",";
  json += "\"awake_window_seconds\":" + String(AWAKE_WINDOW_SECONDS) + ",";
  json += "\"changed\":" + String(changed ? "true" : "false");
  json += "}";
  
  Serial.print("Settings response: ");
  Serial.println(json);
  server.send(200, "application/json", json);
}

// ============================================================================
// POWER SAVING HELPERS (NEW FEATURE)
// ============================================================================

/**
 * Connects to WiFi (best-effort) with a short timeout.
 * This is called at each wake-up to enable the web server during the awake window.
 */
void connectWiFiBestEffort() {
  WiFi.setTxPower(WIFI_POWER_11dBm);  // keep your low-noise setting
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

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

  } else {
    Serial.println("WiFi connection failed (timeout)");
    Serial.println("System will continue without WiFi");
  }
}

/**
 * Syncs NTP time (best-effort). If WiFi is not connected, time may not update.
 */
void syncTimeBestEffort() {
  // Use US Eastern time rules.
  setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0/2", 1);
  tzset();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  // Best-effort wait a little for NTP on first wake (do not block too long)
  struct tm t;
  for (int i = 0; i < 10; i++) {
    if (getLocalTime(&t)) {
      Serial.print("Time synced: ");
      Serial.print(1900 + t.tm_year);
      Serial.print("-");
      Serial.print(1 + t.tm_mon);
      Serial.print("-");
      Serial.print(t.tm_mday);
      Serial.print(" ");
      Serial.print(t.tm_hour);
      Serial.print(":");
      Serial.println(t.tm_min);
      return;
    }
    delay(200);
  }
  Serial.println("Time not available yet (using DAY profile until time is available).");
}

/**
 * Prepares deep sleep and enters it.
 * - Applies relay hold so relay does not glitch.
 * - Turns off WiFi to save power.
 * - Sleeps for WAKE_INTERVAL_MINUTES.
 */
void goToDeepSleep() {
  Serial.println("Preparing for deep sleep...");

  // Hold relay output level during deep sleep
  holdRelayPinDuringSleep();

  // Turn off WiFi to reduce current draw before sleeping
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  // Stop web server (not strictly required; deep sleep resets anyway)
  server.stop();

  // Set timer wakeup
  uint64_t us = (uint64_t)WAKE_INTERVAL_MINUTES * 60ULL * 1000000ULL;
  esp_sleep_enable_timer_wakeup(us);

  Serial.print("Sleeping for ");
  Serial.print(WAKE_INTERVAL_MINUTES);
  Serial.println(" minutes...");

  Serial.flush();
  esp_deep_sleep_start();
}

// ============================================================================
// SETUP FUNCTION - RUNS ONCE AT STARTUP
// ============================================================================

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  delay(300);  // Allow serial to stabilize

  bootMillis = millis();
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("  Battery Cutoff Monitor Starting");
  Serial.println("========================================");
  
  // Load saved settings from flash memory
  preferences.begin("voltmeter", false);  // false = read/write mode
  CALIBRATION_FACTOR = preferences.getFloat("cal_factor", 1.0);
  
  // Load wake interval, but if it's the old default (10), update to new default (1)
  WAKE_INTERVAL_MINUTES = preferences.getUInt("wake_interval", 1);
  if (WAKE_INTERVAL_MINUTES == 10) {
    // Old default detected, update to new default
    WAKE_INTERVAL_MINUTES = 1;
    preferences.putUInt("wake_interval", 1);
    Serial.println("! Updated wake interval from old default (10 min) to new default (1 min)");
  }
  
  // Load awake window, but if it's the old default (60), update to new default (120)
  AWAKE_WINDOW_SECONDS = preferences.getUInt("awake_window", 120);
  if (AWAKE_WINDOW_SECONDS == 60) {
    // Old default detected, update to new default
    AWAKE_WINDOW_SECONDS = 120;
    preferences.putUInt("awake_window", 120);
    Serial.println("! Updated awake window from old default (60 sec) to new default (120 sec)");
  }
  
  preferences.end();
  
  Serial.print("Power saving settings: Wake every ");
  Serial.print(WAKE_INTERVAL_MINUTES);
  Serial.print(" min, stay awake ");
  Serial.print(AWAKE_WINDOW_SECONDS);
  Serial.println(" sec");
  
  // Configure relay pin as output
  pinMode(RELAY_PIN, OUTPUT);
  
  // Configure ADC settings for maximum stability
  analogReadResolution(12);                     // 12-bit resolution (0-4095)
  analogSetPinAttenuation(ADC_PIN, ADC_11db);  // 11dB attenuation (0-~3.3V, non-linear)
  
  // Initialize history array
  for (int i = 0; i < 288; i++) {
    turnOnHistory[i] = 0;
  }
  historyIndex = 0;
  historyCount = 0;
  
  // Start with load ON (state will be evaluated below)
  loadEnabled = true;
  setRelayEnergized(true);
  lastSwitchTime = millis();
  
  // Connect WiFi periodically (we are awake now)
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  connectWiFiBestEffort();
  
  // Time config (best-effort each wake)
  syncTimeBestEffort();
  
  // Read voltage ONCE immediately on boot/wake, set thresholds, and apply relay logic.
  // This ensures relay is correct even if the user never opens the web UI.
  float rawVoltage = readBatteryVoltage();
  lastVBat = smoothVoltage(rawVoltage);

  refreshActiveThresholds();

  if (autoMode) {
    if (lastVBat <= V_CUTOFF) {
      applyLoadState(false);
    } else if (lastVBat >= V_RECONNECT) {
      applyLoadState(true);
    }
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
  Serial.println("CSV Output: Voltage,LoadState");
  Serial.println();
}

// ============================================================================
// MAIN LOOP - RUNS CONTINUOUSLY
// ============================================================================

void loop() {
  // Handle incoming web requests while awake
  server.handleClient();

  // Keep your original periodic measurement while awake
  static unsigned long lastReadTime = 0;
  if (millis() - lastReadTime >= 250) {
    lastReadTime = millis();

    float rawVoltage = readBatteryVoltage();
    lastVBat = smoothVoltage(rawVoltage);

    refreshActiveThresholds();

    if (autoMode) {
      if (lastVBat <= V_CUTOFF) {
        if (loadEnabled) applyLoadState(false);
      } else if (lastVBat >= V_RECONNECT) {
        if (!loadEnabled) applyLoadState(true);
      }
    }

    Serial.print(lastVBat, 2);
    Serial.print(",");
    Serial.println(loadEnabled ? 1 : 0);
  }


  // After the awake window expires, go to deep sleep to save power
  if (millis() - bootMillis >= (AWAKE_WINDOW_SECONDS * 1000UL)) {
    goToDeepSleep();
  }
}

// ============================================================================
// END OF CODE
// ============================================================================

