# Circuit Documentation - Battery Cutoff Monitor

## System Overview

This circuit manages power distribution from a solar panel and LiFePO4 battery, automatically controls a relay to protect the battery, and powers an air quality sensor (Purple Air Sensor). The ESP32 microcontroller monitors battery voltage and intelligently disconnects/reconnects the load based on configurable thresholds.

---

## Component List

### Power Components

| Component | Specification | Quantity | Purpose |
|-----------|--------------|----------|---------|
| **LiFePO4 Battery** | 12.8V, 18Ah | 1 | Main power storage |
| **Solar Panel** | 380W | 1 | Renewable energy source |
| **5V Step Up/Down Converter** | 12V → 5V | 1 | Powers ESP32 and relay |

### Control Components

| Component | Specification | Quantity | Purpose |
|-----------|--------------|----------|---------|
| **ESP32 (30 pin)** | WiFi + Bluetooth | 1 | Main controller |
| **1 Channel Relay** | 5V, NO contact | 1 | Load switching |

### Voltage Measurement Components

| Component | Specification | Quantity | Purpose |
|-----------|--------------|----------|---------|
| **Resistor (Top)** | 100kΩ, 1/4W | 1 | Voltage divider (Battery+ side) |
| **Resistor (Bottom)** | 10kΩ, 1/4W | 1 | Voltage divider (GND side) |
| **Electrolytic Capacitor** | 10µF | 1 | Smooths voltage readings |

### Load

| Component | Specification | Quantity | Purpose |
|-----------|--------------|----------|---------|
| **Purple Air Sensor** | Air quality sensor | 1 | Environmental monitoring |

---

## Circuit Diagram

View the complete wiring diagram here: [Cirkit Designer - Battery Monitor Wiring](https://app.cirkitdesigner.com/project/de4b9459-3ebd-45a3-887c-bb5d5f185a88)

---

## Wiring Details

### Common Ground ⚠️ CRITICAL

**All grounds MUST be connected together:**
- Battery Negative (-)
- Solar Panel Negative (-)
- 5V Converter GND
- ESP32 GND
- Relay GND
- Purple Air Sensor GND

### Power Flow

```
Solar Panel (380W)
    ↓
LiFePO4 Battery (12.8V, 18Ah) ← Stores energy
    ↓
5V Step Up/Down Converter → Powers ESP32 + Relay
    ↓
Relay (Controlled by ESP32)
    ↓
Purple Air Sensor (Load)
```

### Detailed Connections

#### 1. Battery + Solar Panel

**Battery Positive (+):**
- → 5V Converter VIN
- → Relay COM (Common)
- → 100kΩ Resistor (for voltage measurement)

**Solar Panel Positive (+):**
- ⚠️ **CRITICAL: MUST use a charge controller!**
- → Charge Controller INPUT
- Charge Controller OUTPUT → Battery Positive (+)
- **NEVER connect solar panel directly to battery!**
- A 380W solar panel can output 30-40V open circuit voltage
- This will destroy the 5V converter and ESP32!

**Battery Negative (-) & Solar Negative (-):**
- → Common Ground (all GNDs connected)

---

#### 2. Voltage Measurement Circuit (Voltage Divider)

```
Battery+ ──→ [100kΩ] ──→ MIDDLE POINT ──→ [10kΩ] ──→ GND
                              ↓
                          ESP32 VP (GPIO36)
                              ↓
                     [Capacitor 10µF]
                              ↓
                            GND
```

**100kΩ Resistor:**
- Pin 1: Battery Positive (+)
- Pin 2: Middle junction point

**10kΩ Resistor:**
- Pin 1: Middle junction point
- Pin 2: Common Ground

**Middle Junction Point connects to:**
- ESP32 VP pin (GPIO36)
- Capacitor Positive (+)

**Electrolytic Capacitor (10µF):**
- Positive (+): Middle junction point
- Negative (-): Common Ground

**⚠️ IMPORTANT:**
- Voltage divider ratio: (100k + 10k) / 10k = 11
- This reduces 12.8V battery to 1.16V at ESP32 (safe!)
- Maximum safe input: 36.3V
- Always verify resistor values with multimeter!

---

#### 3. ESP32 Microcontroller

**Power Pins:**
- **Vin**: ← 5V Converter VOUT (5V supply)
- **GND**: ← Common Ground

**Input Pins:**
- **VP (GPIO36)**: ← Voltage divider middle point (battery voltage sensing)

**Output Pins:**
- **D27 (GPIO27)**: → Relay IN (control signal)

**WiFi:**
- SSID: `ObieConnect`
- Password: Configured in code
- Web Interface: `http://[ESP32_IP]`

---

#### 4. 5V Step Up/Down Converter

**Input:**
- **VIN**: ← Battery Positive (+) & Solar Positive (+)
- **GND**: ← Common Ground

**Output:**
- **VOUT (5V)**: → ESP32 Vin
- **VOUT (5V)**: → Relay VCC
- **GND**: ← Common Ground

**Note:** Converter stays powered even when load is disconnected, allowing ESP32 to continue monitoring.

---

#### 5. 1 Channel Relay (5V, Normally Open)

**Control:**
- **VCC**: ← 5V Converter VOUT (power for relay coil)
- **GND**: ← Common Ground
- **IN**: ← ESP32 D27 (GPIO27) - control signal

**Switching:**
- **COM**: ← Battery Positive (+) / Solar Positive (+)
- **NO**: → Purple Air Sensor Positive (+)
- **NC**: Not used

**Relay Logic (Normally Open):**
```
Relay OFF (GPIO27 LOW):
  NO contact = OPEN → Sensor has NO power 🔴

Relay ON (GPIO27 HIGH):
  NO contact = CLOSED → Sensor has power 🟢
```

**Active-LOW Relay Modules:**
Most relay modules are active-LOW, meaning:
- GPIO LOW = Relay energized (closes NO contact)
- GPIO HIGH = Relay not energized (opens NO contact)

This is configured in code:
```cpp
const bool RELAY_ACTIVE_LOW = true;
```

---

#### 6. Purple Air Sensor (Load)

**Power:**
- **Positive (+)**: ← Relay NO (Normally Open) contact
- **Negative (-)**: ← Common Ground

**Operation:**
- When relay is ON → Sensor receives power
- When relay is OFF → Sensor is disconnected (battery protection)

---

## System Operation

### Automatic Battery Protection

**Default Thresholds:**
- **Turn OFF at:** 12.0V (battery low)
- **Turn ON at:** 12.9V (battery recovered)

**Operation Cycle:**

```
1. Battery Voltage: 13.0V
   └→ Load: ON (sensor running)

2. Battery drains to 12.0V
   └→ CUTOFF! Load turns OFF (relay opens)

3. Solar charges battery
   └→ Voltage rises: 12.1V → 12.2V → ... → 12.9V

4. Battery reaches 12.9V
   └→ RECONNECT! Load turns ON (relay closes)

5. Cycle repeats
```

### Hysteresis (Prevents Rapid Cycling)

**Why two different thresholds?**
- Gap between OFF (12.0V) and ON (12.9V) prevents relay "chatter"
- Ensures battery has time to properly recharge
- Avoids rapid on/off cycling that damages relays

---

## Voltage Divider Calculations

### Formula

```
Vadc = Vbattery × (R_bottom / (R_top + R_bottom))
```

### With Your Resistors (100kΩ / 10kΩ):

```
Ratio = (100,000 + 10,000) / 10,000 = 11

If battery = 12.8V:
  Vadc = 12.8V ÷ 11 = 1.16V ✅ Safe for ESP32

If battery = 14.6V (full charge):
  Vadc = 14.6V ÷ 11 = 1.33V ✅ Safe for ESP32

Maximum safe input = 3.3V × 11 = 36.3V ✅
```

**ESP32 ADC can only handle 0-3.3V!**

---

## Power Budget

### System Power Consumption

| Component | Power Draw | Notes |
|-----------|-----------|-------|
| **ESP32** | ~150mA @ 5V = 0.75W | With WiFi active |
| **Relay Module** | ~70mA @ 5V = 0.35W | When energized |
| **Voltage Divider** | ~0.12mA @ 12.8V = 0.0015W | Negligible |
| **Purple Air Sensor** | Varies | Check sensor specs |
| **Total (ESP32 + Relay)** | ~1.1W | Excluding sensor |

### Battery Runtime (No Solar)

```
Battery: 12.8V × 18Ah = 230.4 Wh

ESP32 + Relay only: 1.1W
Runtime = 230.4 Wh / 1.1W = 209 hours (8.7 days)

With Purple Air Sensor (~5W):
Runtime = 230.4 Wh / 6.1W = 37.7 hours (1.5 days)
```

---

## Safety Considerations

### ⚠️ CRITICAL SAFETY NOTES

1. **SOLAR PANEL PROTECTION - HIGHEST PRIORITY**
   - ⚠️ **MUST use a charge controller between solar panel and battery!**
   - A 380W solar panel can output 30-40V open circuit voltage
   - Direct connection will destroy the 5V converter and ESP32
   - Charge controller limits voltage to safe levels (typically 14-15V for 12V systems)
   - **Without a charge controller, the ESP32 WILL BURN when sun comes out!**
   - Recommended: MPPT or PWM charge controller rated for your panel voltage/current

2. **5V Converter Input Voltage Limits**
   - Check your converter's maximum input voltage rating
   - Most converters: 20-30V maximum input
   - Solar panel open circuit voltage can exceed this
   - Charge controller prevents overvoltage damage

3. **Common Ground Required**
   - All negative terminals MUST be connected together
   - Missing common ground = incorrect voltage readings and relay malfunction

4. **Resistor Values**
   - MUST use 100kΩ top, 10kΩ bottom (or equivalent 11:1 ratio)
   - Wrong values can damage ESP32 ADC (over-voltage)
   - Verify with multimeter before connecting

5. **Battery Protection**
   - Never let LiFePO4 drop below 10V (cell damage)
   - Default 12V cutoff is safe for most applications
   - Adjust thresholds based on your battery specifications

6. **Solar Panel Wiring**
   - Solar Panel → Charge Controller INPUT
   - Charge Controller OUTPUT → Battery Positive
   - Charge Controller GND → Common Ground
   - Battery → 5V Converter VIN
   - **NEVER bypass the charge controller!**
   - Direct connection can overcharge/damage battery
   - Charge controller not shown in simplified diagram

5. **Wire Gauge**
   - Use appropriate wire gauge for load current
   - Undersized wires can overheat
   - 18 AWG suitable for loads up to ~10A

6. **Relay Current Rating**
   - Ensure relay can handle your load current
   - Purple Air Sensor is low power (typically <1A)
   - For higher currents, use appropriately rated relay

---

## Web Interface

### Access Dashboard

**URL:** `http://[ESP32_IP_ADDRESS]`

**Example:** `http://10.17.195.65`

### Features

- 🔋 **Live Voltage Display** - Updates every second
- 🔴/🟢 **Load Status** - Shows if sensor is ON or OFF
- 🎛️ **Control Mode** - AUTO (automatic) or MANUAL
- ⚙️ **Current Thresholds** - Displays turn OFF/ON voltages
- 🔘 **Control Buttons:**
  - Auto Mode - Enable automatic battery protection
  - Force ON - Manually turn sensor on (overrides thresholds)
  - Force OFF - Manually turn sensor off

### JSON API

**Status Endpoint:**
```
GET http://[ESP32_IP]/status.json
```

**Response:**
```json
{
  "voltage_v": 12.35,
  "load_on": true,
  "auto_mode": true,
  "v_cutoff": 12.00,
  "v_reconnect": 12.90,
  "uptime_ms": 123456
}
```

**Settings Endpoint (Change Thresholds):**
```
GET http://[ESP32_IP]/settings?lower=11&upper=12

Response:
{
  "v_cutoff": 11.00,
  "v_reconnect": 12.00,
  "changed": true
}
```

**Examples:**
```bash
# Set cutoff to 11V, reconnect to 12V
curl "http://10.17.195.65/settings?lower=11&upper=12"

# Change only cutoff voltage
curl "http://10.17.195.65/settings?lower=11.5"

# Change only reconnect voltage
curl "http://10.17.195.65/settings?upper=13"

# Get current settings
curl "http://10.17.195.65/settings"

# Get system status
curl "http://10.17.195.65/status.json"
```

---

## Troubleshooting

### Incorrect Voltage Reading

**Symptom:** Displayed voltage doesn't match multimeter

**Solutions:**
1. Verify resistor values with multimeter (should be 100kΩ and 10kΩ)
2. Check that GPIO36 is connected to the MIDDLE point between resistors
3. Ensure capacitor is connected properly
4. Check for loose connections

### Relay Not Switching

**Symptom:** Relay doesn't click or sensor stays on/off

**Solutions:**
1. Verify relay has 5V power (VCC pin)
2. Check GPIO27 connection to relay IN
3. Verify relay module type (active-LOW vs active-HIGH)
4. Check relay current rating matches load

### Web Interface Not Accessible

**Symptom:** Can't connect to ESP32 IP address

**Solutions:**
1. Check ESP32 is powered and running
2. Verify WiFi credentials in code
3. Open Serial Monitor to see IP address
4. Ensure you're on same WiFi network (2.4GHz only)

### Voltage Reading 0V or 36.3V

**Symptom:** Shows exactly 0V or 36.3V

**Solutions:**
- **0V:** Voltage divider not connected
- **36.3V:** ADC is maxed out, resistors might be swapped or wrong values

---

## Maintenance

### Regular Checks

- **Weekly:** Check battery voltage via web interface
- **Monthly:** Verify relay is switching properly (check sensor operation)
- **Quarterly:** Inspect connections for corrosion or looseness
- **Yearly:** Test battery capacity and solar panel output

### Battery Care (LiFePO4)

- Store at 50-60% charge (12.8V-13.0V) if unused long-term
- Avoid deep discharges below 10V
- Keep battery in moderate temperatures (avoid extreme heat/cold)
- Expected lifespan: 2000-5000 cycles with proper care

---

## Future Enhancements

### Possible Upgrades

- [ ] Add temperature sensor for battery monitoring
- [ ] Implement MQTT for home automation integration
- [ ] Add current sensor to monitor actual load draw
- [ ] Solar panel voltage/current monitoring
- [ ] Battery state of charge estimation algorithm
- [ ] Email/SMS alerts for low battery
- [ ] Data logging to SD card or cloud
- [ ] Support for multiple load outputs

---

## Technical Specifications

| Parameter | Value |
|-----------|-------|
| **Input Voltage** | 10V - 15V (12.8V nominal) |
| **Maximum Measurable** | 36.3V |
| **ADC Resolution** | 12-bit (4096 steps) |
| **Voltage Accuracy** | ±2-3% (with quality resistors) |
| **Update Rate** | 4 Hz (250ms) |
| **Relay Response Time** | <50ms |
| **WiFi** | 2.4GHz (802.11 b/g/n) |
| **Operating Temperature** | 0°C to 50°C |

---

## References

- [ESP32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
- [LiFePO4 Battery Guide](https://batteryuniversity.com/)
- Voltage Divider Calculator: [calculator.net](https://www.calculator.net/voltage-divider-calculator.html)

---

**Last Updated:** 2026-01-15  
**Version:** 1.0  
**Author:** Battery Cutoff Monitor System
