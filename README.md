# ESP32 Power Monitor Switch (BL0942)

This project implements a **smart power plug / switch** based on an **ESP32** and the **BL0942 calibration‑free energy metering IC**.
It measures **voltage, current, active power, energy, and frequency**, controls a relay, and exposes the data to **HomeKit**.

The project is verified with **BL0942 (SSOP‑10)** using **UART packet mode**.

---

## Features

- BL0942 energy metering (UART)
- Voltage RMS, Current RMS, Active Power
- Energy accumulation with **NVS persistence**
- Line frequency reporting
- Relay control (on/off)
- Button handling (short / long / double press)
- HomeKit integration (Eve‑style characteristics)
- Smooth filtering and rate‑limited notifications

---

## Hardware Overview

### Main Components

- ESP32 (ESP‑IDF)
- BL0942 (SSOP‑10)
- Low‑ohmic shunt resistor (typically 1–2 mΩ)
- Mains voltage divider
- Relay + driver
- Momentary push button
- Isolated power supply

---

## BL0942 Pinout (SSOP‑10)

This project uses the BL0942 **exactly as described below**.

### Power and Ground

- **Pin 1 – VDD**  
  3.3 V supply. Decouple close to the pin.

- **Pin 5 – GND**  
  Ground reference.

---

### Current Measurement Channel

- **Pin 2 – IP**  
- **Pin 3 – IN**  

Differential current input across the shunt resistor.

- Max input: ±42 mV peak (30 mV RMS)
- High input impedance (~370 kΩ)

---

### Voltage Measurement Channel

- **Pin 4 – VP**  

Voltage input from mains via resistor divider.

- Max input: ±100 mV peak (70 mV RMS)
- High input impedance (~370 kΩ)

---

### Digital Interface and Control

- **Pin 6 – CF1**  
  Configurable logic output (energy pulse, zero‑crossing, or over‑current).
  *Not required for normal UART operation in this project.*

- **Pin 7 – SEL**  
  Interface select  
  - GND → UART mode (used here)  
  - VDD → SPI mode  

- **Pin 8 – SCLK_BPS**  
  UART baud‑rate configuration pin (UART mode).

- **Pin 9 – RX / SDI**  
  UART RX (BL0942 receives data from ESP32).  
  **External pull‑up required.**

- **Pin 10 – TX / SDO**  
  UART TX (BL0942 sends data to ESP32).  
  **External pull‑up required.**

---

## UART Wiring (ESP32 ↔ BL0942)

| BL0942 Pin | Function | ESP32 GPIO |
|-----------|---------|------------|
| RX / SDI  | UART RX | ESP32 TX |
| TX / SDO  | UART TX | ESP32 RX |
| SEL       | Mode    | GND |
| SCLK_BPS  | Baud    | GND or VDD |

### ⚠ Menuconfig Naming Note

In ESP‑IDF:
- `UART TX GPIO` = **ESP32 TX → BL0942 RX**
- `UART RX GPIO` = **ESP32 RX ← BL0942 TX**

This is electrically correct but easy to misread.

---

## UART Configuration

- Mode: UART (SEL = 0)
- Baud rate: 4800 / 9600 / 19200 / 38400 bps
- Packet mode enabled
- 24‑bit register data
- Checksum verified on every packet

The driver uses the **BL0942 packet reading mode** to fetch all parameters in one transaction.

---

## Measurement Model

### Read Values

- Voltage RMS → `V_RMS`
- Current RMS → `I_RMS`
- Active Power → `WATT`
- Energy → `CF_CNT`
- Frequency → `FREQ`
- Status → `STATUS`

### Energy Persistence

- Total energy is stored in **NVS (mWh)**
- On boot:
  - Load stored energy
  - Continue accumulation from BL0942 counter
- Prevents energy loss or double counting across resets

---

## Button Actions

- **Short press**: Toggle relay
- **Long press**: Factory reset (HomeKit + NVS)
- **Double press**: Reset accumulated energy

---

## Build & Flash

```bash
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py flash monitor
```

---

## Menuconfig Options

- BL0942 UART pins
- Baud rate selection
- Shunt value
- Voltage divider ratio
- HomeKit accessory configuration
- Relay GPIO
- Button GPIO

---

## Common Issues & Fixes

### No data / checksum errors
- Missing pull‑ups on RX/SDI or TX/SDO
- SEL not tied to GND
- Baud rate mismatch

### Power reads zero
- Shunt value incorrect
- Divider ratio wrong
- Anti‑creep active at very low loads

### RMS looks unstable
- Small load (<10–20 mA)
- Board‑level DC offset
- Normal for shunt‑based systems

---

## Notes

- BL0942 defaults are assumed:
  - CF_EN = enabled
  - CF_CNT does **not** clear on read
  - Absolute energy accumulation
- Changing MODE bits externally may break energy math unless handled in firmware.

---

## License

MIT (or your chosen license)
