#pragma once
/**
 * Drag timing finite state machine for 0–60 mph and 1/4 mile performance measurement.
 *
 * This class implements a GPS-based drag timing state machine intended for Arduino projects.
 * It detects a valid “staged” condition (vehicle stopped with a valid GPS fix), arms itself,
 * then detects launch based on speed thresholds. Once a run is in progress, it measures:
 *
 *   1) 0–60 mph elapsed time (ET)
 *   2) 1/4 mile ET using two distance estimation methods:
 *        - Integrated distance from speed via trapezoidal integration
 *        - Direct start-to-current displacement via haversine distance
 *
 * The class also tracks:
 *   - Max speed during the run
 *   - Speed at the 0–60 threshold crossing
 *   - Speed at the 1/4 mile completion event
 *
 * Run lifecycle:
 *   - DRAG_WAIT_FOR_STOP: Waits for the vehicle to remain below the stop threshold for a
 *     minimum duration while GPS fix is valid.
 *   - DRAG_ARMED: Armed state; waits for speed to exceed the launch threshold.
 *   - DRAG_RUNNING: Integrates distance, checks 0–60 and 1/4 mile completion.
 *   - DRAG_FINISHED: Prints a single-line summary and waits for the vehicle to stop again,
 *     then auto-resets for the next run.
 *
 * Safety/robustness behaviors:
 *   - If GPS fix is lost during DRAG_RUNNING, the run is aborted and the FSM resets.
 *   - If the vehicle slows below the stop threshold for longer than the abort timeout
 *     before completing the 1/4 mile, the run is aborted and the FSM resets.
 *
 * Required external inputs:
 *   - A valid GPS fix boolean provided to update().
 *   - A Pose struct provided to update() containing:
 *       time_sec, lat_deg, lon_deg, speed_mps.
 *   - A haversineDistance_m() implementation.
 *
 * Time base assumptions:
 *   - pose.time_sec should be monotonic and consistent with dt_sec.
 *     If your system cannot guarantee this, consider deriving time from millis()
 *     or adapting the class accordingly.
 *
 * Debug/logging:
 *   - By default, output is written to Serial, but begin(Stream&) allows redirection.
 *   - The run summary is printed in a stable, comma-delimited format intended for
 *     easy parsing by external logging or telemetry tools.
 */
class DragTimingFSM
{
public:
  //==========================================================================================
  // Public types
  //==========================================================================================

  // Drag run states
  enum DragState
  {
    DRAG_WAIT_FOR_STOP = 0,   // waiting for vehicle to be fully stopped with GPS fix
    DRAG_ARMED         = 1,   // stopped and GPS OK, waiting for launch
    DRAG_RUNNING       = 2,   // run in progress
    DRAG_FINISHED      = 3    // run complete, waiting for reset
  };

  // Pose input required by this FSM
  struct Pose
  {
    float time_sec  = 0.0f;  // monotonic time in seconds
    float lat_deg   = 0.0f;
    float lon_deg   = 0.0f;
    float speed_mps = 0.0f;
  };

  // Results / internal bookkeeping
  struct DragTiming
  {
    DragState state = DRAG_WAIT_FOR_STOP;

    // Run bookkeeping
    float startTime_sec          = 0.0f;
    float distance_integrated_m  = 0.0f;
    float distance_haversine_m   = 0.0f;
    float startLat_deg           = 0.0f;
    float startLon_deg           = 0.0f;
    float lastSpeed_mps          = 0.0f;
    float maxSpeed_mps           = 0.0f;

    // 0-60 mph results
    bool  has_0_60     = false;
    float t_0_60_sec   = 0.0f;
    float v_0_60_mps   = 0.0f;

    // 1/4 mile results (two distance methods)
    bool  has_quarter        = false;
    float t_quarter_int_sec  = 0.0f; // using integrated speed distance
    float t_quarter_hav_sec  = 0.0f; // using haversine distance
    float v_quarter_mps      = 0.0f;

    // stop detection timer (for arming and abort/reset)
    uint32_t stopStart_ms    = 0;
    bool     stopTimerRunning = false;
  };

  //==========================================================================================
  // Construction / setup
  //==========================================================================================

  DragTimingFSM() = default;

  // Optional: route debug output somewhere other than Serial
  void begin(Stream& debugStream = Serial)
  {
    _dbg = &debugStream;
    reset();
  }

  //==========================================================================================
  // Configuration setters (optional)
  //==========================================================================================

  void setStopThresholdMps(float v)   { _speedStopThreshold_mps = v; }
  void setStartThresholdMps(float v)  { _speedStartThreshold_mps = v; }
  void setMinStopTimeMs(uint32_t ms)  { _minStopTime_ms = ms; }
  void setAbortStopTimeMs(uint32_t ms){ _abortStopTime_ms = ms; }

  //==========================================================================================
  // Core API
  //==========================================================================================

  // Reset state machine to initial state.
  void reset()
  {
    _drag.state = DRAG_WAIT_FOR_STOP;

    _drag.startTime_sec = 0.0f;
    _drag.distance_integrated_m = 0.0f;
    _drag.distance_haversine_m = 0.0f;
    _drag.startLat_deg = 0.0f;
    _drag.startLon_deg = 0.0f;
    _drag.lastSpeed_mps = 0.0f;
    _drag.maxSpeed_mps = 0.0f;

    _drag.has_0_60 = false;
    _drag.t_0_60_sec = 0.0f;
    _drag.v_0_60_mps = 0.0f;

    _drag.has_quarter = false;
    _drag.t_quarter_int_sec = 0.0f;
    _drag.t_quarter_hav_sec = 0.0f;
    _drag.v_quarter_mps = 0.0f;

    _drag.stopStart_ms = 0;
    _drag.stopTimerRunning = false;

    logln(F("DRAG: Reset, waiting for stop."));
  }

  // Main drag-timing state machine update
  // dt_sec is the time since the last update
  //
  // REQUIRED INPUTS:
  // - gpsFixValid: true if you have a valid GPS fix
  // - pose:        current fused pose data
  //
  // MISSING (external dependency):
  // - A correct monotonic pose.time_sec source.
  //
  void update(float dt_sec, bool gpsFixValid, const Pose& pose)
  {
    // Require a valid GPS fix for all run logic
    if (!gpsFixValid)
    {
      // If we lose GPS fix while running, safest is to abort and reset
      if (_drag.state == DRAG_RUNNING)
      {
        logln(F("DRAG: GPS fix lost, aborting run."));
        reset();
      }
      return;
    }

    const float speed_mps = pose.speed_mps;
    const float speed_mph = speed_mps * MPS2MPH;

    switch (_drag.state)
    {
      case DRAG_WAIT_FOR_STOP:
      {
        if (updateStopTimer(speed_mps, _minStopTime_ms))
        {
          _drag.state = DRAG_ARMED;
          logln(F("DRAG: Armed (vehicle stopped, GPS fix OK)."));
        }
        break;
      }

      case DRAG_ARMED:
      {
        if (speed_mps < _speedStopThreshold_mps)
        {
          // stay armed
        }
        else if (speed_mps >= _speedStartThreshold_mps)
        {
          // Launch detected -> start run
          _drag.state = DRAG_RUNNING;

          _drag.startTime_sec = pose.time_sec;
          _drag.startLat_deg  = pose.lat_deg;
          _drag.startLon_deg  = pose.lon_deg;
          _drag.lastSpeed_mps = speed_mps;
          _drag.maxSpeed_mps  = speed_mps;

          _drag.distance_integrated_m = 0.0f;
          _drag.distance_haversine_m  = 0.0f;

          _drag.has_0_60    = false;
          _drag.has_quarter = false;

          _drag.stopTimerRunning = false;

          log(F("DRAG: Run started at t = "));
          logFloat(_drag.startTime_sec, 3);
          log(F(" s, speed = "));
          logFloat(speed_mph, 2);
          logln(F(" mph."));
        }
        else
        {
          // small creep, but not enough to start; stay in ARMED
        }
        break;
      }

      case DRAG_RUNNING:
      {
        // Track max speed for info
        if (speed_mps > _drag.maxSpeed_mps)
        {
          _drag.maxSpeed_mps = speed_mps;
        }

        // 1) Distance from integrated speed (trapezoidal integration)
        const float v_prev = _drag.lastSpeed_mps;
        const float v_curr = speed_mps;
        _drag.distance_integrated_m += 0.5f * (v_prev + v_curr) * dt_sec;
        _drag.lastSpeed_mps = v_curr;

        // 2) Distance from haversine between start position and current position
        //
        // MISSING: haversineDistance_m implementation.
        // Provide your own function or replace with a library call.
        //
        _drag.distance_haversine_m = haversineDistance_m(
          _drag.startLat_deg, _drag.startLon_deg,
          pose.lat_deg, pose.lon_deg
        );

        // 0-60 mph timing
        if (!_drag.has_0_60 && (speed_mps >= TARGET_60_MPS))
        {
          _drag.has_0_60   = true;
          _drag.t_0_60_sec = pose.time_sec - _drag.startTime_sec;
          _drag.v_0_60_m
