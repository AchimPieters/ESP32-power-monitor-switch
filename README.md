# ESP32 Power Plug with Energy Meter

HomeKit-compatible smart power plug firmware based on **ESP32** and **BL0942 energy metering IC**, with full **Eve Energy** support.

This project is designed for **ESP-IDF v5.4+** and focuses on **stable Home/Eve presentation**, **accurate energy accounting**, and **production-grade robustness**.

---

## Supported Hardware

- ESP32-WROOM-32D
- ESP8685-WROOM-03
- BL0942 energy metering chip

---

## Features

- HomeKit **Switch / Outlet** service
- Eve Energy **custom service** (Voltage, Current, Power, Total Energy)
- Stable graphs in **Apple Home** and **Eve.app**
- Energy consumption **persistent across reboots and power loss**
- BL0942 **calibration via NVS**
- Measurement **rounding & normalization** (Eve-proof)
- **EMA smoothing** to remove jitter
- Non-blocking **status LED** with error signaling

---

## Architecture Overview

- BL0942 sampled at ~200 ms
- Measurements aggregated at 1 second
- HomeKit notifications rate-limited and threshold-based
- Energy calculated as a monotonic accumulator (kWh)

---

## Step 12 – Eve / Home polish (Completed)

This step focuses on correct presentation, units, and update stability in Home.app and Eve.app.

### Services & Characteristics

- Eve Energy characteristics are implemented as **float values** (no string/float mismatch)
- Consistent characteristic definitions prevent Eve graph glitches

### Units & Rounding

| Measurement | Unit | Resolution |
|-----------|------|------------|
| Voltage | V | 0.1 |
| Current | A | 0.001 |
| Power | W | 1 |
| Energy | kWh | 0.001 |

All values are rounded **before** being published to HomeKit.

### Update & Notification Strategy

- BL0942 sampling: ~200 ms
- Aggregation: 1 s
- HomeKit notify rate: max ~1× per 2 s
- Notifications only sent when values change beyond defined thresholds

Result: **smooth, stable graphs without spikes or dropped history**.

---

## Step 13 – Validation & Robustness (Hardware validation required)

The firmware contains all required mechanisms for robust operation, but the following **hardware validation steps must be executed**.

### Required Tests

- USB flash
- OTA flash (if enabled)
- Software reboot (`esp_restart()`)
- Power-cycle (plug off/on)
- Load testing (e.g. 300–1000 W resistive load)

### Expected Behavior

- Total energy (kWh) is **monotonic** and never decreases
- No power or voltage spikes after reboot
- Home and Eve continue logging without interruption

### Status LED Behavior

| LED Pattern | Meaning |
|------------|---------|
| Solid ON | WiFi provisioning / not ready |
| OFF | Normal operation |
| 1 blink / 2 s | BL0942 error or timeout |

### Regression Checklist

- Switching works without metering active
- Metering works with zero load
- No NaN / infinite / negative values
- No heap growth over time

---

## Step 14 – Calibration, Normalization & Smoothing (Completed)

### Calibration

BL0942 calibration factors are configurable via **NVS**.

**Namespace:** `bl0942`

- `v_cal` – Voltage multiplier
- `i_cal` – Current multiplier
- `p_cal` – Power multiplier

Calibration values are clamped to safe ranges (0.5 – 2.0).

### Normalization

- Values are clamped and sanitized (no negative, NaN, or infinite values)
- Rounding ensures Eve/Home receive clean numeric data

### Smoothing

Exponential Moving Average (EMA) is applied to:

- Voltage
- Current
- Power

Energy is **not smoothed** (it is integrated over time).

---

## Energy Persistence

- Total energy is periodically stored in **NVS**
- No double-counting across reboots
- Energy continues seamlessly after flash or power loss

---

## Build & Flash

```bash
idf.py set-target esp32
idf.py build
idf.py flash monitor
```

### ESP-IDF Version

```text
>= v5.4
```

---

## Project Status

- Step 12 (Eve/Home polish): ✅ Completed
- Step 14 (Calibration & smoothing): ✅ Completed
- Step 13 (Validation): 🟨 Hardware testing required

After successful validation, this firmware is **production-ready** for HomeKit + Eve Energy usage.

---

## Possible Next Enhancements

- CLI or HTTP interface for live calibration
- Power factor (PF) reporting
- Overcurrent protection / automatic cut-off
- Eve history interval tuning

---
