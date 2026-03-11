# Non-Deviate: Assistive Bus Stop Navigation

Assistive navigation system designed to help visually impaired commuters identify arriving buses at a bus stop using **UWB localisation and wearable audio feedback**.

The system detects bus locations in real time using **UWB anchors and trilateration**, filters noisy measurements with a **Kalman filter**, and determines whether a requested bus has arrived within a defined bus stop zone.

> Developed as part of the **SUTD Term 5 Engineering Systems Project**.

---

## System Overview

The localisation system uses **three UWB anchors (DW3000)** installed around a bus stop to measure distances to UWB tags mounted on buses. Distance measurements are processed to estimate the bus position relative to the bus stop, smoothed using a Kalman filter, and classified into predefined bus stop zones. When a requested bus enters the detection zone, the wearable device provides **audio feedback to the user**.

### System Pipeline

```
UWB Anchors
     ↓
Distance Measurements
     ↓
Trilateration Algorithm
     ↓
Kalman Filter (Position Smoothing)
     ↓
Zone Classification
     ↓
User Audio Notification
```

---

## Key Features

- Real-time **UWB localisation using DW3000 modules**
- **3-anchor trilateration algorithm** for 2D position estimation
- **2D Kalman filter** to reduce measurement noise
- Bus stop **zone classification** for arrival detection
- UDP + serial communication between localisation server and ESP devices
- Real-time localisation visualisation using Matplotlib

---

## Localisation Algorithm

### Trilateration

Bus position is estimated by solving the intersection of three circles centred at each UWB anchor:

```
(x − x₁)² + (y − y₁)² = r₁²
(x − x₂)² + (y − y₂)² = r₂²
(x − x₃)² + (y − y₃)² = r₃²
```

Where:
- `(x, y)` — estimated bus position
- `(xₙ, yₙ)` — anchor coordinates
- `rₙ` — measured distance from each anchor to the tag

### Kalman Filtering

UWB measurements are affected by multipath reflections and signal noise. A **2D Kalman filter** smooths position estimates using the state vector:

```
[x, y, vx, vy]
```

The filter predicts the next state and corrects it using incoming distance measurements, significantly improving localisation stability compared to raw trilateration output.

---

## Visualisation

The localisation server renders in real time:

- Anchor positions
- Bus stop zone boundaries
- Raw and filtered bus position estimates

```
A1        A2
  \      /
   \    /
    Bus
   /
  /
A3
```

---

## Repository Structure

```
nondeviate-bus-navigation/
│
├── README.md
│
├── report/
│   └── term-paper.pdf
│
├── localisation/
│   └── uwb_localisation_kalman.py
│
├── firmware/
│   ├── ss_twr_initiator.ino
│   └── ss_twr_responder.ino
│
└── figures/
    ├── localisation_result.png
    └── radiation_pattern.png
```

---

## Hardware Components

| Component | Role |
|---|---|
| DW3000 UWB modules | Distance measurement (anchors + tags) |
| ESP32 microcontrollers | Communication and control |
| UWB tags | Mounted on buses for ranging |
| Wearable feedback device | Audio output for visually impaired users |
| Bus stop anchor infrastructure | Fixed anchor mounting points |

---

## Installation

Clone the repository:

```bash
git clone https://github.com/yourusername/nondeviate-bus-navigation.git
cd nondeviate-bus-navigation
```

Install required Python libraries:

```bash
pip install numpy matplotlib pyserial
```

Run the localisation server:

```bash
python localisation/uwb_localisation_kalman.py
```

---

## Contributors

Developed as a team project for the SUTD Term 5 Engineering Systems Project.

| Contributor | Contributions |
|---|---|
| Nurin Diyanah Binte Awaludin | UWB trilateration model, 2D Kalman filter implementation, anchor placement analysis using radiation patterns |
| Benjamin Lee Yan Jie | Circuitry Design for handheld device, UWB hardware system testing and deployment |
| Chong Xing Xiao | ESP32 communication pipeline, system testing and deployment |
| Kristen Tan Kai Lih | UWB hardware integration, ESP32 communication pipeline, system testing and deployment |
| Lee Yi Xuan, Samuel | UWB hardware integration, ESP32 communication pipeline, system testing and deployment |
| Seth Choo Sze Wei | Circuitry design for handheld device, UWB hardware system testing and deployment |
| Sharmaine Koh Xin Yi | Design and fabrication of handheld device |
| Tan Pang Wei Matthew | UWB hardware integration, system testing and deployment |

---

## License

This repository is shared for academic and portfolio purposes. Please credit the original project team if using any part of this work.
