# 🚁 Autonomous Disaster-Response Multirotor UAS
### Team UDAAN — AeroTHON 2025 | NIT Rourkela | Team ID: AT2025054

---

## Overview

A competition-grade autonomous quadcopter designed for rapid search, rescue, and relief operations in disaster-stricken environments. The system integrates real-time AI-based survivor detection, multi-sensor environmental monitoring, GPS-guided autonomous navigation, and precision payload delivery — all within a 2 kg MTOW micro UAS platform.

> *Presented at AeroTHON 2025 — National-level drone design competition*

---

## Key Capabilities

| Capability | Implementation |
|-----------|---------------|
| Survivor Detection | YOLOv8 real-time object detection on Raspberry Pi 5 |
| Area Coverage | 30 hectares scan via GPS waypoint navigation |
| Environmental Monitoring | Gas (MQ2), Temperature/Humidity (SHT31), Light (LDR), Ultrasonic |
| Autonomous Navigation | Pixhawk 4 + Neo M8N GPS + Mission Planner |
| Payload Delivery | 200g servo-triggered precision drop mechanism |
| Ground Communication | MAVLink over 3DR 433MHz telemetry radio |
| Flight Time | 12–15 minutes (88.8 Wh, 6000mAh 14.8V LiPo) |

<img width="823" height="569" alt="Screenshot 2026-05-26 at 5 33 00 PM" src="https://github.com/user-attachments/assets/8765d4a4-e12e-425d-b80e-54ca6fa06245" />


---

## System Architecture

```
┌─────────────────────────────────────────────────┐
│                  GROUND STATION                  │
│         Mission Planner + Telemetry UI           │
└──────────────────┬──────────────────────────────┘
                   │ MAVLink (3DR 433MHz)
┌──────────────────▼──────────────────────────────┐
│                   ONBOARD SYSTEM                 │
│                                                  │
│  ┌─────────────┐      ┌──────────────────────┐  │
│  │  Pixhawk 4  │◄────►│   Raspberry Pi 5     │  │
│  │  Flight     │ UART │   - YOLOv8 Detection  │  │
│  │  Controller │      │   - Sensor Fusion     │  │
│  │  PID Loops  │      │   - Decision Logic    │  │
│  └──────┬──────┘      └──────────┬───────────┘  │
│         │                        │               │
│  ┌──────▼──────┐      ┌──────────▼───────────┐  │
│  │ Neo M8N GPS │      │   Sensor Suite        │  │
│  │ ESCs (x4)   │      │   MQ2 | SHT31 | LDR  │  │
│  │ Motors (x4) │      │   Ultrasonic | Camera │  │
│  └─────────────┘      └──────────────────────┘  │
└─────────────────────────────────────────────────┘
```

---

## My Contribution — Sensor Integration & Calibration

> **Role:** Sensor selection, hardware calibration, and Raspberry Pi data acquisition pipeline

### Sensors Selected & Integrated

| Sensor | Model | Interface | Purpose |
|--------|-------|-----------|---------|
| Gas Sensor | MQ2 | Analog → ADS1115 ADC → I2C | Detects LPG, CO, CH₄, smoke |
| Temp/Humidity | SHT31 | I2C | Environmental monitoring |
| Ambient Light | GL5537 LDR | Analog → ADS1115 ADC | Camera exposure adaptation |
| ADC Converter | ADS1115 | I2C | 16-bit analog-to-digital conversion |
| Ultrasonic | HC-SR04 | GPIO | Flood-level detection, obstacle avoidance |
| Camera | Analog (YOLO input) | CSI/USB | Real-time object detection |

### Calibration Methodology

**MQ2 Gas Sensor:**
- Preheat time: 24–48 hours for stable baseline resistance (R0)
- Calibrated in clean air: R0 = Rs / 9.8 (per datasheet ratio)
- Applied load resistance (RL = 10kΩ) for voltage divider output
- Validated against known LPG concentration using ratio curves

**SHT31 Temperature & Humidity:**
- Two-point calibration: 0°C (ice bath) and 100°C (boiling water)
- Offset correction applied in software for ±0.3°C accuracy
- I2C address verified (0x44 default)

**ADS1115 ADC Setup:**
- Gain set to ±4.096V for full-range analog sensor compatibility
- Sampling rate: 128 SPS (samples per second)
- Differential mode used for noise rejection

**Ultrasonic (HC-SR04):**
- Temperature-compensated speed of sound: v = 331.3 + 0.606×T m/s
- Filtered using 5-sample rolling median to remove spike noise
- Validated against known distances (0.05m – 4m range)

### Data Acquisition Pipeline (Raspberry Pi 5)

```python
# Sensor fusion data collection — simplified flow
import board
import adafruit_sht31d
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO
import time

# I2C bus
i2c = board.I2C()
sht31 = adafruit_sht31d.SHT31D(i2c)        # Temp/humidity
ads   = ADS.ADS1115(i2c)                    # ADC for MQ2 + LDR
mq2   = AnalogIn(ads, ADS.P0)              # MQ2 on channel 0
ldr   = AnalogIn(ads, ADS.P1)              # LDR on channel 1

TRIG, ECHO = 23, 24
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def read_ultrasonic():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    return (pulse_end - pulse_start) * 17150  # cm

def read_all_sensors():
    return {
        "temperature_C": round(sht31.temperature, 2),
        "humidity_pct":  round(sht31.relative_humidity, 2),
        "gas_voltage_V": round(mq2.voltage, 3),
        "light_voltage_V": round(ldr.voltage, 3),
        "distance_cm":   round(read_ultrasonic(), 1),
        "timestamp":     time.time()
    }
```

---

## Flight Stability Analysis — MATLAB PID Simulation

> Performed as part of team's control system validation. Validated PID parameters for altitude regulation before hardware implementation.

### Vertical Dynamics Model

The simplified altitude dynamics of the UAS modelled as:

```
m * z̈ = T - m*g - b*ż
```

Where:
- `m` = 1.57 kg (UAS mass)
- `T` = Thrust input (controlled variable)
- `g` = 9.81 m/s²
- `b` = Air damping coefficient

### PID Controller — Altitude Hold

```matlab
% UAS Altitude PID Stability Simulation
% Team UDAAN — AeroTHON 2025

clear; clc;

% System parameters
m  = 1.57;   % mass (kg)
g  = 9.81;   % gravity (m/s²)
b  = 0.5;    % air damping coefficient

% PID Gains (tuned for stable altitude hold)
Kp = 30;
Ki = 50;
Kd = 10;

% Transfer function: Plant G(s) = 1 / (m*s^2 + b*s)
num = [1];
den = [m, b, 0];
plant = tf(num, den);

% PID Controller
pid_ctrl = pid(Kp, Ki, Kd);

% Closed-loop system
sys_cl = feedback(pid_ctrl * plant, 1);

% Step response (1 metre altitude command)
t = 0:0.01:5;
[y, t] = step(sys_cl, t);

% Plot
figure;
plot(t, y, 'b-', 'LineWidth', 2); hold on;
yline(1.0, 'r--', 'Setpoint (1m)', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Altitude (metres)');
title('UAS Altitude Step Response — PID Control (Kp=30, Ki=50, Kd=10)');
grid on;
legend('Altitude Response', 'Setpoint');

% Performance metrics
info = stepinfo(sys_cl);
fprintf('Rise Time:     %.3f s\n', info.RiseTime);
fprintf('Settling Time: %.3f s\n', info.SettlingTime);
fprintf('Overshoot:     %.2f %%\n', info.Overshoot);
fprintf('Steady-state:  %.4f m\n', dcgain(sys_cl));
```

### Simulation Results

| Parameter | Value |
|-----------|-------|
| Proportional Gain (Kp) | 30 |
| Integral Gain (Ki) | 50 |
| Derivative Gain (Kd) | 10 |
| Overshoot | ~0% (smooth response) |
| Thrust | Within safe limits |
| Steady-state error | ~0 (integral eliminates) |

**Insights:** The integral term (Ki=50) eliminates steady-state altitude error caused by gravity. The derivative term (Kd=10) provides damping to prevent oscillation. Final parameters were validated on Pixhawk 4 flight controller via Mission Planner.

---

## Hardware Stack

| Component | Model | Qty | Cost (₹) |
|-----------|-------|-----|----------|
| Flight Controller | Pixhawk 4 | 1 | 15,500 |
| Onboard Computer | Raspberry Pi 5 | 1 | 7,500 |
| Motors | EMAX ECOII 2807 | 4 | 8,000 |
| ESCs | 30A | 4 | 3,200 |
| GPS | Neo M8N | 1 | 2,800 |
| Telemetry | 3DR 433MHz | 1 | 4,500 |
| Battery | 14.8V 6000mAh 50C LiPo | 1 | 6,500 |
| Frame | Carbon Fiber + ABS (custom) | 1 | 12,000 |
| Sensors (all) | MQ2, SHT31, LDR, Ultrasonic, ADS1115 | — | 1,800 |
| **Total** | | | **~₹85,000** |

---

## UAS Final Specifications

| Parameter | Value |
|-----------|-------|
| Type | Quadcopter (X-frame) |
| MTOW | 1570 g |
| Payload | 200 g |
| Thrust-to-Weight | 3.4:1 |
| Flight Time | 15–20 min |
| Communication Range | >1 km (MAVLink) |
| Detection Model | YOLOv8 |
| Propulsion | Electric (14.8V LiPo) |

---

## Repository Structure

```
team-udaan-uas/
├── README.md                        ← This file
├── sensor_integration/
│   ├── sensor_calibration.py        ← Calibration scripts
│   ├── data_acquisition.py          ← Sensor fusion pipeline
│   └── calibration_notes.md        ← Calibration methodology
├── matlab_simulation/
│   ├── pid_altitude_simulation.m    ← MATLAB PID script
│   └── results/
│       └── step_response.png        ← Simulation output
├── autonomous_mission/
│   └── mission_notes.md            ← Mission Planner waypoint methodology
└── docs/
    └── AeroTHON2025_Presentation.pdf
```

---

## Team

**Team UDAAN** | NIT Rourkela | AeroTHON 2025
Team ID: AT2025054

---

## Competition Context

AeroTHON is a national-level drone design and innovation competition. This design was submitted for the **Disaster Relief** problem statement — requiring autonomous area scanning, survivor identification, and precision payload delivery within strict weight and power constraints.
