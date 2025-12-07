# ArduDrag
An Arduino-based drag race sytle timer for 0-60mph and 1/4 mile

This project is a lightweight Arduino/Teensy sketch for **evaluating straight-line car performance**. It measures:

* **0–60 mph elapsed time and speed**
* **1/4 mile elapsed time and trap speed**

A **10 Hz GPS** is the primary sensor used to measure **speed and distance**. An optional **BNO055 IMU** is included to provide orientation context (pitch/roll/heading), but the performance timing is intended to be GPS-driven.

---

## What you get

* Reliable GPS configuration for **RMC + GGA** at **10 Hz**
* A unified **pose** data structure (GPS + IMU)
* Simple serial outputs for logging
* A drag timing state machine with:

  * stop-to-arm logic
  * launch detection
  * **0–60 mph**
  * **1/4 mile**
  * distance tracking via:

    * integrated speed
    * haversine from GPS position

---

## Output

### Continuous log

```
POSE_CSV, time_sec, numSat, lat_deg, lon_deg, speed_mps, heading_deg, pitch_deg, roll_deg
```

### Run summary (when enabled)

```
DRAG_RESULT, t_start=..., t_0_60=..., t_quarter_int=..., t_quarter_hav=..., v_0_60_mph=..., v_quarter_mph=..., dist_int_m=..., dist_hav_m=..., maxSpeed_mph=...
```

---

## Hardware

* **Adafruit Ultimate GPS** (10 Hz capable)
* **Adafruit BNO055** IMU
* MCU with:
  * **Serial1** for GPS
  * **I2C** for IMU

---

## Intended use

Use this to capture repeatable, serial-logged performance metrics for vehicle testing in a controlled, safe environment with good GPS sky view.
