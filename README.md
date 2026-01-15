# Battery Cutoff Monitor - ESP32

Automatic battery protection system that disconnects your load when voltage gets too low and reconnects when voltage recovers.

## Circuit Diagram

View the complete wiring diagram here: [Cirkit Designer - Battery Monitor Wiring](https://app.cirkitdesigner.com/project/de4b9459-3ebd-45a3-887c-bb5d5f185a88)

---

## What It Does

Monitors your battery voltage continuously. Simple logic: when voltage is above the cutoff threshold (default: 12.0V), load is ON. When voltage drops to or below the threshold, load turns OFF.

**Current Default Settings:**
- Cutoff threshold: 12.0V (load OFF if voltage is at or below this)
- Relay Type: NO (Normally Open)
- Relay Pin: GPIO27

---

## Parts Needed

| Part | Specification |
|------|---------------|
| ESP32 board | Any ESP32 development board |
| Relay module | 5V relay with NO contact |
| Top resistor | 100kΩ (Brown-Black-Yellow) |
| Bottom resistor | 10kΩ (Brown-Black-Orange) |
| Capacitor | 0.1µF to 1µF ceramic |
| Battery | 8V-16V (tested with 12.8V LiFePO4) |
| Wires | Jumper wires and power wires |

---

## Wiring Instructions

### A. Battery Voltage Measurement (Voltage Divider)

```
Battery + ──→ 100kΩ resistor ──→ [Connection Point] ──→ 10kΩ resistor ──→ GND
                                          ↓
                                      GPIO36 (VP)
                                          ↓
                                   Capacitor to GND
```

The resistors divide the battery voltage down to a safe level (0-3.3V) for the ESP32 ADC input.

### B. Relay Connection (NO Terminal)

```
Power Supply + ──→ Relay COM terminal
Relay NO terminal ──→ Your Load +
Load GND ──→ Common GND
```

**How NO (Normally Open) works:**
- Relay OFF = NO contact open = Load has no power
- Relay ON = NO contact closed = Load gets power

### C. Relay Control from ESP32

```
ESP32 GPIO27 ──→ Relay IN pin
ESP32 GND ──→ Relay GND
5V supply ──→ Relay VCC
```

### D. ESP32 Power

```
5V power supply ──→ ESP32 VIN
GND ──→ ESP32 GND
```

### IMPORTANT: Common Ground

All grounds must be connected together:
- Battery negative
- ESP32 GND
- Relay GND  
- Load GND
- Power supply GND

---

## Software Setup

### Step 1: Install Arduino IDE

Download from: https://www.arduino.cc/en/software

### Step 2: Add ESP32 Board Support

1. Open Arduino IDE
2. File → Preferences
3. Add this URL to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Tools → Board → Boards Manager
5. Search "ESP32" and install

### Step 3: Configure WiFi Credentials

Open `voltage_meter.ino` and edit line 61:

```cpp
const char* WIFI_SSID = "YourWiFiName";
const char* WIFI_PASS = "YourPassword";
```

### Step 4: Upload Code

1. Connect ESP32 to computer with USB cable
2. Tools → Board → "ESP32 Dev Module"
3. Tools → Port → Select your port (e.g., /dev/cu.usbserial-0001)
4. Tools → Upload Speed → "115200"
5. Click Upload button
6. If it says "Connecting...", hold the BOOT button on ESP32
7. Wait for "Done uploading"

### Step 5: Get IP Address

1. Tools → Serial Monitor
2. Set baud rate to 115200
3. Press RESET button on ESP32
4. Note the IP address printed (e.g., 10.17.195.65)
5. Open that IP in your web browser

---

## How To Change Voltage Thresholds

### Method 1: Edit the Code (Permanent Change)

Open `voltage_meter.ino` and find line 147:

```cpp
float V_CUTOFF = 12.0;     // Turn OFF at this voltage
float V_RECONNECT = 12.9;  // Turn ON at this voltage
```

Change the values to what you need, then upload the code again.

**Example 1: For 12V Lead Acid Battery**
```cpp
float V_CUTOFF = 11.5;     // Protect battery at 11.5V
float V_RECONNECT = 12.4;  // Reconnect at 12.4V
```

**Example 2: Allow More Battery Discharge**
```cpp
float V_CUTOFF = 11.0;     // Use more capacity before cutoff
float V_RECONNECT = 12.0;  // Resume at 12V
```

**Example 3: Conservative Protection**
```cpp
float V_CUTOFF = 12.5;     // Cut off early to preserve battery
float V_RECONNECT = 13.2;  // Wait for full charge
```

### Method 2: Change via Web API (Temporary Until Restart)

Send HTTP request to change the cutoff threshold on the fly:

**Set cutoff to 11V:**
```
http://[ESP32-IP]/settings?lower=11.0
```

**Example with curl:**
```bash
curl "http://10.17.195.65/settings?lower=11.0"
```

**Response:**
```json
{
  "v_cutoff": 11.00,
  "v_reconnect": 12.90,
  "changed": true
}
```

**The system will immediately apply the new threshold:**
- If voltage is now above the new cutoff, load turns ON instantly
- If voltage is now at or below the new cutoff, load turns OFF instantly

---

## Web Dashboard

Open your browser to the ESP32 IP address (e.g., http://10.17.195.65)

**Dashboard shows:**
- Current battery voltage (updates every second)
- Load status (ON or OFF with color indicator)
- Control mode (AUTO or MANUAL)
- Current cutoff threshold
- Current reconnect threshold

**Control buttons:**
- Auto - Automatic control based on voltage thresholds
- Force ON - Manually turn load ON (ignores voltage)
- Force OFF - Manually turn load OFF

---

## How The Control Logic Works

**In AUTO mode (Simple Logic):**
- If voltage is **above** cutoff threshold → Load is **ON**
- If voltage is **at or below** cutoff threshold → Load is **OFF**

**Example with cutoff at 11.0V:**
- Voltage at 13.0V → Load ON (above 11.0V)
- Voltage at 11.5V → Load ON (above 11.0V)
- Voltage drops to 11.0V → Load turns OFF (at threshold)
- Voltage drops to 10.8V → Load stays OFF (below threshold)
- Voltage rises to 11.1V → Load turns ON (above threshold)

The system continuously checks the voltage and immediately responds to changes.

---

## JSON API For External Programs

**Get current status:**
```
GET http://[ESP32-IP]/status.json
```

**Response:**
```json
{
  "voltage_v": 11.63,
  "load_on": false,
  "auto_mode": true,
  "uptime_ms": 123456,
  "v_cutoff": 11.00,
  "v_reconnect": 12.90
}
```

**Change thresholds:**
```
GET http://[ESP32-IP]/settings?lower=11.0&upper=12.5
```

**Response:**
```json
{
  "v_cutoff": 11.00,
  "v_reconnect": 12.50,
  "changed": true
}
```

**Control relay:**
```
GET http://[ESP32-IP]/relay?auto=1      (enable auto mode)
GET http://[ESP32-IP]/relay?on=1        (force load ON)
GET http://[ESP32-IP]/relay?on=0        (force load OFF)
```

---

## Troubleshooting

### Wrong Voltage Reading

**Problem:** Displayed voltage doesn't match multimeter

**Fix:** Verify resistor values. Measure each resistor with multimeter:
- Top resistor should be 100kΩ (Brown-Black-Yellow)
- Bottom resistor should be 10kΩ (Brown-Black-Orange)

If your resistors are different, update line 82 in the code:
```cpp
const float RTOP = 100000.0;  // Change to your measured value
const float RBOT = 10000.0;   // Change to your measured value
```

### Voltage Reading Too High (36V or maxed out)

**Problem:** Reading shows impossible voltage like 36V

**Fix:** The resistors are likely swapped. Check that:
- 100kΩ resistor connects Battery+ to ADC point
- 10kΩ resistor connects ADC point to GND
- NOT the other way around

### Voltage Jumps Around

**Problem:** Voltage reading is unstable or noisy

**Fix:**
- Add larger capacitor (1µF instead of 0.1µF)
- Keep voltage divider wires short
- Check all connections are solid
- Move voltage divider away from noisy components

### Relay Doesn't Click

**Problem:** Relay never activates

**Fix:**
- Check relay VCC pin has 5V power
- Verify GPIO27 connects to relay IN pin
- Check relay GND connects to ESP32 GND
- Test relay manually by connecting IN pin directly to GND

### Load Doesn't Turn On

**Problem:** Relay clicks but load has no power

**Fix:**
- Verify using NO terminal on relay, not NC
- Check COM terminal has power supply connected
- Check NO terminal connects to load positive
- Verify load GND connects to common ground
- Try "Force ON" button on web page

### Can't Upload Code

**Problem:** Upload fails with serial errors

**Fix:**
- Close Serial Monitor before uploading
- Try different USB cable
- Use /dev/tty.usbserial-* port instead of /dev/cu.*
- Hold BOOT button during upload
- Check nothing else (Chrome, etc.) is using the serial port:
  ```bash
  lsof | grep usbserial
  ```

---

## Technical Specifications

| Parameter | Value |
|-----------|-------|
| Input voltage range | 8V - 16V |
| ADC resolution | 12-bit (0-4095) |
| ADC pin | GPIO36 (VP, ADC1_CH0) |
| Relay control pin | GPIO27 |
| Relay type | NO (Normally Open) |
| Voltage measurement rate | 4 Hz (every 250ms) |
| Web update rate | 1 Hz (every 1 second) |
| Smoothing | 100 sample average + 10 point moving average |
| WiFi | 2.4GHz only (ESP32 limitation) |

---

## Voltage Divider Calculation

**Formula:**
```
V_adc = V_battery × (R_bottom / (R_top + R_bottom))
```

**With 100kΩ top and 10kΩ bottom:**
```
V_adc = V_battery × (10000 / (100000 + 10000))
V_adc = V_battery × 0.0909
V_adc = V_battery / 11
```

**Safety check for 14.6V max (charging voltage):**
```
V_adc_max = 14.6V / 11 = 1.33V
```

This is well below the 3.3V maximum for ESP32 ADC. Safe operation confirmed.

---

## Safety Notes

1. Check battery voltage rating before connecting
2. Connect all grounds together (battery, ESP32, relay, load)
3. Verify relay current rating matches your load
4. Use appropriate wire gauge for load current
5. Test voltage reading with multimeter before connecting load
6. Never short circuit battery terminals
7. Double-check wiring before powering on

---

## Files

- `voltage_meter.ino` - Main Arduino code
- `README.md` - This documentation  
- `circuit_diagram.png` - Circuit wiring photo
- `CIRCUIT_DOCUMENTATION.md` - Detailed technical documentation

---

## License

Open source. Free to use and modify.

---

**GitHub:** https://github.com/Environmental-Dashboard/-Battery-Cutoff-Monitor
