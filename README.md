# ArduDrag – GPS-Based Drag Timer with Kalman Filtering

ArduDrag is an Arduino/Teensy sketch that implements a GPS-based drag timer with basic sensor fusion. It takes raw NMEA data from an Adafruit GPS module, estimates distance and speed using a 1-D Kalman filter, and feeds those into a drag-race finite state machine (FSM) to detect launches and measure performance.

> Note: This project depends on `KalmanFilter.h` and `DragFSM.h`, which are part of this codebase but not shown in the snippet below. They are referenced throughout and described conceptually in this README.

---

# GPS Speed Latency / High-Acceleration Accuracy Disclaimer

This project uses an Adafruit “Ultimate GPS” (MTK33xx-class) NMEA receiver. I just happened to have it laying around from another project. However, it turns out to be a bad choice for this project. Under steady-state conditions the reported speed (RMC SOG) is typically accurate, but **during rapid acceleration/deceleration the GPS-reported speed may lag the vehicle’s true instantaneous speed**. The module’s internal navigation solution applies filtering/smoothing and the NMEA stream can reflect a “time-of-validity” solution that is inherently delayed. Basically, the GPS internal motion model used for filtering does not support high accelerations. It is likely setup up for more static (low acceleration) situations to help filter noise more effecitvely given its lower cost design and inteneded use cases. Implication is that peak speeds and short-interval acceleration metrics (e.g., 0–60, 60–130) may be **under-reported** when using GPS speed alone.   

**The underlying code is well writen with attention paid to both the Kalman Filter and DragFSM code. Both can easily be adopted to different GPS modules and projects.**

A better GPS option would be the **SparkFun GPS Breakout - NEO-M9N**. This is a true uBlox 25Hz GPS and able to use binary messages over a high speed UART (921600 baud) for faster performance. **The Kalman Filter would not be needed with this GPS** as you would just be reling on the built in uBlox filter. You should switch to using the UBX protocol and the UBX-NAV-PVT packet. UBX is an efficient, **binary protocol** and the UBX-NAV-PVT packet provides much more data than is available via NMEA. The additional data include North, East, and Down (NED) velocities; estimates of accuracy for horizontal and vertical position and velocity; and position dilution of precision (pDOP). You will also want to set the dynamic model/platform to **Automotive**. The easiest integration would be to use SparkFun’s u-blox GNSS Arduino library. You should be able to find the basics in SparkFun’s Example1_AutoPVT.
- https://www.sparkfun.com/sparkfun-gps-breakout-neo-m9n-chip-antenna-qwiic.html#content-features
- https://learn.sparkfun.com/tutorials/sparkfun-gps-neo-m9n-hookup-guide

---

## Features

- Uses an Adafruit GPS module (NMEA over UART) at 10 Hz.
- Automatically reconfigures the GPS from 9600 baud → 57600 baud for better throughput.
- Parses RMC and GGA sentences for:
  - Latitude / longitude
  - Speed over ground (SOG)
  - Fix quality, satellite count, HDOP
- Computes:
  - Great-circle distance traveled using the haversine formula.
  - Quality-dependent position uncertainty based on fix quality, satellites, and HDOP.
  - Minimum credible speed threshold based on satellite count.
- 1-D Kalman filter (`KalmanFilter`) to:
  - Estimate distance and speed at 20 Hz.
  - Fuse distance (integrated position) and speed from GPS.
  - Use a jerk-based process noise model for responsive acceleration tracking.
- Drag race FSM (`DragFSM`) to:
  - Detect launch from standstill using filtered speed.
  - Track run progression (e.g., WAIT_FOR_STOP → ARMED → RUNNING → FINISHED).
  - Use GPS quality metrics to gate measurements.
- Serial debug / logging:
  - Human-readable GPS quality summary.
  - Optional verbose timing and KF debug when `DEBUG` is enabled.
- Fully non-blocking main loop; GPS parsing runs continuously.

---

## Repository Layout

Expected top-level files:

- `ArduDrag.ino`  
  Main Arduino/Teensy sketch: GPS setup, GPS quality handling, haversine distance, Kalman filter wiring, and drag FSM integration.

- `KalmanFilter.h`  
  1-D Kalman filter for position and speed estimation. Exposes methods such as:
  - `init(dt, p0, v0, sigma_p0, sigma_v0)` – initialize state and covariance.
  - `setSigmaJ(sigma_j)` – configure jerk-based process noise.
  - `setR_measurement(sigma_p, sigma_v)` – configure measurement noise.
  - `predict(dt)` – propagate state forward in time.
  - `update(meas_position, meas_speed)` – fuse a new GPS measurement.
  - `position()`, `velocity()` – access filtered outputs.
  - `initP_state(sigma_p0, sigma_v0)` – reset state covariance (used when re-arming runs).

- `DragFSM.h`  
  Drag finite state machine:
  - Internal `enum class DragState { DRAG_WAIT_FOR_STOP, DRAG_ARMED, DRAG_RUNNING, DRAG_FINISHED }`.
  - Methods like:
    - `update(now_ms, gpsDataGood, v_est_mps, dist_est_m, gpsSpd_mps)`
    - `setStopThresholdMps(...)`, `setStartThresholdMps(...)`
    - `state()` – current state accessor.
  - Handles arming, run detection, aborts, and final result reporting via `Serial`.

If you re-upload these header files in the future and want this README to reflect exact function signatures and behavior, they can be incorporated verbatim.

---

## Hardware and Platform

The code is written for a Teensy board using `Serial1` for GPS:

- **MCU**: Teensy (e.g., Teensy 3.2/3.1) with hardware UART `Serial1`.
- **GPS**: Adafruit Ultimate GPS (NMEA) at 10Hz, or similar module supported by `Adafruit_GPS`.
  - NMEA output at 9600 baud by default; the sketch reconfigures it to 57600.
  - Configured to output RMC and GGA at 10 Hz.
- **LED**: Onboard LED on pin 13 used as a heartbeat.

### Wiring (typical)

- GPS TX → Teensy `RX1` (Serial1 RX)
- GPS RX → Teensy `TX1` (Serial1 TX)
- GPS VCC → 3.3V or 5V (depending on the module; check GPS board specs)
- GPS GND → Teensy GND

Ensure the GPS antenna has clear sky view for reliable satellite lock.

---

## Software Dependencies

Install the following in the Arduino IDE / PlatformIO environment:

- **Adafruit GPS Library**  
  `Adafruit_GPS` from Library Manager.

- **Teensy core / board package**  
  Install Teensy support via Teensyduino or appropriate board manager setup.

- **Project-local headers**  
  - `KalmanFilter.h`
  - `DragFSM.h`

These must reside in the same project directory or in a library folder so the compiler can find them.

---

## High-Level Design

### GPS Input and Quality Handling

- `setupGPS()`:
  - Starts `Serial1` at 9600 baud.
  - Sends `PMTK_SET_BAUD_57600` to the GPS.
  - Restarts `Serial1` at 57600 baud.
  - Configures NMEA output to `RMCGGA` and sets update rate to 10 Hz.

- `readGPS()`:
  - Called every loop iteration.
  - Feeds one character at a time into the `Adafruit_GPS` parser via `GPS.read()`.
  - Once an entire sentence is ready, calls `GPS.parse()`.
  - Returns:
    - `0` – no new sentence.
    - `1` – parse failed.
    - `2` – RMC sentence parsed.
    - `3` – GGA sentence parsed.
    - `4` – other sentences.

- `printGPS()`:
  - Logs satellite count, fix flag, fix quality string, and HDOP.
  - When fixed, logs lat/lon and SOG in mph.

The helper function `GpsFixQuality(uint8_t q)` converts `GPS.fixquality` into a human-readable label.

### Haversine Distance

`haversineDistance_m(lat1, lon1, lat2, lon2)` implements the standard great-circle distance formula, returning meters.

This is used to integrate total traveled distance:

- `_lat_prev`, `_lon_prev` store the previous GPS position.
- `gpsDist_m` accumulates absolute distance increments between updates.

### GPS Quality → Measurement Noise

Two helper functions map GPS quality metrics into an approximate position standard deviation:

- `baseSigmaFromFix(int fixQuality)`
- `computeSigmaP(int fixQuality, int sats, float hdop)`

Key ideas:

- Better fix quality (e.g., RTK fixed) → lower base sigma.
- More satellites → decrease sigma modestly.
- Higher HDOP (poorer geometry) → increase sigma.

The result is then low-pass filtered:

```cpp
const float alpha = 0.2f;
sigma_p_smoothed = (1.0f - alpha) * sigma_p_smoothed + alpha * sigma_p;
_kf.setR_measurement(sigma_p_smoothed, _sigma_v);
````

This stabilized `sigma_p_smoothed` becomes the position measurement uncertainty `R` for the Kalman filter.

### Minimum Credible GPS Speed

`minSpeedBasedonSatellites(numSat)` returns a minimum speed (m/s) below which GPS SOG is considered unreliable, based on satellite count.

This is used by `DragFSM` thresholds:

```cpp
float minGpsSpeed = minSpeedBasedonSatellites(GPS.satellites);
_dragFSM.setStopThresholdMps(minGpsSpeed + 0.0f);
_dragFSM.setStartThresholdMps(minGpsSpeed + 0.1f); // small hysteresis
```

---

## Kalman Filter (KalmanFilter.h)

The Kalman filter operates in 1D along the direction of travel:

* **State**: `[position, velocity]`.
* **Inputs**: No explicit control input; motion driven by process noise.
* **Process Model**: Constant-acceleration assumption, with jerk noise (`_sigma_j`) setting how fast acceleration can change.
* **Measurements**:

  * Position: `gpsDist_m` (monotonic integrated distance from GPS positions).
  * Speed: `gpsSpd_mps` (SOG from GPS).

Configuration parameters:

* `_kfRate_sec` – nominal KF update period, tied to process loop (20 Hz).
* `sigma_state_p0` – initial position uncertainty.
* `_sigma_state_v0` – initial speed uncertainty.
* `_sigma_v` – speed measurement uncertainty.
* `_sigma_j` – jerk noise; higher values make the estimate track changes more aggressively.

Main usage pattern in `loop()`:

1. On first valid GPS RMC:

   * Initialize state: `p0 = 0`, `v0 = 0`.
   * Call `_kf.init(_kfRate_sec, p0, v0, sigma_state_p0, _sigma_state_v0)`.
   * Call `_kf.setSigmaJ(_sigma_j)`.

2. On each new RMC:

   * Update GPS distance and speed.
   * Compute/update `sigma_p_smoothed` and call `_kf.setR_measurement(...)`.
   * Run `predict(dt)` if enough time elapsed.
   * Run `update(gpsDist_m, gpsSpd_mps)`.

3. On each 20 Hz process tick:

   * Run `predict(dt)` again if needed (to get continuous 20 Hz estimates).
   * Read `distKF_m = _kf.position()`, `spdKF_mps = _kf.velocity()`.
   * Pass these to `DragFSM`.

When `DragFSM` returns to `DRAG_ARMED`, the code calls `_kf.initP_state(...)` to enlarge state covariance again, improving responsiveness on a new run.

---

## Drag State Machine (DragFSM.h)

`DragFSM` encapsulates drag-run logic:

* **Inputs** (via `_dragFSM.update(...)`):

  * `now_ms` – current time in ms.
  * `_gpsDataGood` – boolean gating on satellite count and HDOP.
  * `spdKF_mps` – filtered speed estimate from Kalman filter.
  * `distKF_m` – filtered distance estimate.
  * `gpsSpd_mps` – raw GPS speed (for additional logic or diagnostics).

* **Config**:

  * `setStopThresholdMps(...)` – speed below which the car is considered stopped.
  * `setStartThresholdMps(...)` – speed above which a run is considered started.

* **States** (inferred from naming):

  * `DRAG_WAIT_FOR_STOP` – waiting for car to be stopped with good GPS.
  * `DRAG_ARMED` – ready to detect launch.
  * `DRAG_RUNNING` – run in progress; monitors distance and speed.
  * `DRAG_FINISHED` – run complete; waiting for reset.

The FSM handles:

* Arming when stopped for long enough.
* Detecting launch using filtered speed.
* Measuring distances and time to targets (e.g., 1/4 mile, 0–60 mph).
* Aborting runs if GPS quality drops or the car stops prematurely.
* Printing run summaries to `Serial`.

Since the exact implementation is in `DragFSM.h`, consult that file for precise behavior and event formats.

---

## Main Loop Timing and Structure

The main loop is fully non-blocking and organized around timers:

```cpp
// Rates
_processRate_hz      = 20.0f;
_processPeriod_sec   = 1.0f / _processRate_hz;
_processPeriod_ms    ≈ 50 ms;

_blinkPeriod_ms      = 1000; // LED heartbeat
_printPeriod_ms      = 1000; // GPS status print
```

Important timers:

* `_time_ms` – global timestamp each loop.
* `_timeProcess_ms` – last 20 Hz process tick.
* `_timeKF_ms` – last KF prediction update.
* `_timePrint_ms` – last print.
* `_time_RMC_ms` – last RMC reception.

Loop responsibilities:

1. **Always**:

   * Compute `etime_ms`, `etime_sec`, loop rate.
   * Call `readGPS()` every loop.

2. **When GPS fix available and at least one GGA received**:

   * On each RMC (`gpsRes == 2`):

     * Check GPS quality (satellites, HDOP).
     * Initialize KF on first RMC.
     * Update measurement noise and distances.
     * Run KF `predict()` (if `dt > 5 ms`) and `update()`.

   * On each 20 Hz process tick:

     * Run KF `predict()` again as needed.
     * Get filtered `distKF_m` and `spdKF_mps`.
     * Call `_dragFSM.update(...)`.
     * Optionally re-init KF covariance on transitions to ARMED.

3. **Every `_printPeriod_ms`**:

   * Print debug timing (if `DEBUG`).
   * Call `printGPS()`.

4. **Every `_blinkPeriod_ms`**:

   * Toggle onboard LED.

---

## Debugging and Test Mode

* `DEBUG` flag (bool):

  * When true, prints:

    * KF timing (`dt`), predict/update events.
    * GPS quality metrics (sigma, sigma_p_smoothed).
    * Distance and speed from both GPS and KF.
  * Useful for tuning `_sigma_j`, `_sigma_v`, and sigma calculation.

* `TEST` flag (bool):

  * When true, a synthetic test profile is injected in the RMC path:

    * Simulates a launch, acceleration, and deceleration sequence using `_testCnt`.
    * Operates on top of the real GPS data, so best used when GPS is stationary.
  * Allows testing of KF and drag FSM logic without driving.
