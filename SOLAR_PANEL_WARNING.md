# ⚠️ CRITICAL: Solar Panel Protection Required

## What Happened

If your ESP32 burned out when the sun started shining, **you are missing a charge controller**.

## The Problem

A 380W solar panel can output:
- **30-40V open circuit voltage** (no load)
- **20-25V under load** (depending on conditions)

Your 5V step-up/down converter likely has a maximum input voltage of:
- **20-30V** (check your converter's datasheet)

When the sun comes out, the solar panel voltage spikes above the converter's limit, causing:
- ❌ 5V converter failure
- ❌ ESP32 burnout
- ❌ Component damage

## The Solution

**You MUST use a charge controller between the solar panel and battery.**

### Correct Wiring

```
Solar Panel Positive (+)
    ↓
Charge Controller INPUT (+)
    ↓
Charge Controller OUTPUT (+)
    ↓
Battery Positive (+)
    ↓
5V Converter VIN
    ↓
ESP32 Vin
```

**All grounds connected together:**
- Solar Panel Negative (-)
- Charge Controller GND
- Battery Negative (-)
- 5V Converter GND
- ESP32 GND

## Charge Controller Selection

For a 380W solar panel with a 12.8V battery:

**Recommended Specifications:**
- **Type:** MPPT (more efficient) or PWM (cheaper)
- **Input Voltage:** Must handle panel's open circuit voltage (30-40V)
- **Output Voltage:** 12V/14.4V (for 12.8V LiFePO4 battery)
- **Current Rating:** 380W ÷ 12.8V = ~30A minimum (use 40A+ for safety margin)

**Example Charge Controllers:**
- MPPT: 40A MPPT charge controller (handles 30-40V input)
- PWM: 40A PWM charge controller (handles 30-40V input)

## What NOT to Do

❌ **NEVER connect solar panel directly to battery**
❌ **NEVER connect solar panel directly to 5V converter**
❌ **NEVER bypass the charge controller**

## After Installing Charge Controller

1. Verify charge controller is working (LED indicators)
2. Check battery voltage stays in safe range (12-14.6V for LiFePO4)
3. Monitor with multimeter during sunny conditions
4. Replace burned ESP32 and 5V converter if damaged

## Prevention Checklist

- [ ] Charge controller installed between solar panel and battery
- [ ] Charge controller rated for panel's open circuit voltage
- [ ] Charge controller rated for panel's current output
- [ ] All connections secure and properly sized
- [ ] Ground connections all connected together
- [ ] Battery voltage monitored and in safe range
- [ ] 5V converter input voltage never exceeds its rating

---

**Remember: The charge controller is not optional - it's essential for protecting your entire system!**
