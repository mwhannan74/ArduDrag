#pragma once
#include <Arduino.h>
/**
 * Drag timing finite state machine for 0–60 mph and 1/4 mile using external KF state.
 *
 * This class implements a lightweight drag timing state machine intended for Arduino projects.
 * It no longer computes distance internally. Instead, it consumes fused navigation outputs from
 * an external Kalman filter (KF) at a fixed update rate (e.g., 20 Hz).
 *
 * The FSM assumes Model A semantics:
 *   - distance_m is the estimated distance since the Kalman filter was most recently reset.
 *
 * Inputs per update:
 *   - now_ms: system time in milliseconds since Arduino power-on.
 *   - navDataGood: boolean indicating data quality (GPS/KF validity).
 *   - speed_mps: fused speed estimate.
 *   - distance_m: fused distance since KF reset (Model A).
 *
 * Outputs:
 *   - Internal state and timing results (0–60 and 1/4 mile).
 *   - A one-shot Kalman reset request when a run first starts.
 *
 * Run lifecycle:
 *   - DRAG_WAIT_FOR_STOP: Waits for vehicle to be stopped for MIN_STOP_TIME_MS with valid data.
 *   - DRAG_ARMED: Staged; waits for launch (speed >= start threshold).
 *   - DRAG_RUNNING: Tracks elapsed time using now_ms, captures 0–60 and 1/4 mile events.
 *   - DRAG_FINISHED: Waits for stop again, then auto-resets for next run.
 *
 * Robustness behaviors:
 *   - If navDataGood becomes false during DRAG_RUNNING, the run is aborted and the FSM resets.
 *   - If vehicle remains below stop threshold for ABORT_STOP_TIME_MS before 1/4 mile, abort.
 *
 * MISSING / External requirements:
 *   - External KF must honor reset requests emitted by consumeKalmanResetRequest().
 *   - distance_m must reliably represent distance since that KF reset.
 */

class DragFSM
{
public:
  //==========================================================================================
  // Public types
  //==========================================================================================

  enum DragState
  {
    DRAG_WAIT_FOR_STOP = 0,
    DRAG_ARMED         = 1,
    DRAG_RUNNING       = 2,
    DRAG_FINISHED      = 3
  };

  struct DragTiming
  {
    DragState state = DRAG_WAIT_FOR_STOP;

    // Run time bookkeeping (derived from now_ms)
    uint32_t startTime_ms = 0;        // system time at launch
    float    startDistance_m = 0.0f;  // global distance at launch
    float    maxSpeed_mps = 0.0f;

    // 0-60 mph results
    bool  has_0_60    = false;
    float t_0_60_sec  = 0.0f;
    float v_0_60_mps  = 0.0f;

    // 1/4 mile results
    bool  has_quarter       = false;
    float t_quarter_sec     = 0.0f;
    float v_quarter_mps     = 0.0f;
    float dist_quarter_m    = 0.0f; // captured input distance at completion

    // stop detection timer (arming/abort/reset)
    uint32_t stopStart_ms    = 0;
    bool     stopTimerRunning = false;
  };

  //==========================================================================================
  // Constants / conversions
  //==========================================================================================
  static constexpr float MPS2MPH = 2.23693629f;  
  static constexpr float MILE2MTR = 1609.34f;
  static constexpr float QUARTER_MILE_M = 402.336f;
  

  static constexpr float TARGET_60_MPH = 40.0f; //60.0f; // testing at 40mph because it is safer and easier
  static constexpr float TARGET_60_MPS = TARGET_60_MPH / MPS2MPH;

  //==========================================================================================
  // Construction / setup
  //==========================================================================================
  DragFSM(usb_serial_class* debugSerial)
  : _Serial(debugSerial)
  { 
    reset();   
  }

  //==========================================================================================
  // Configuration setters
  //==========================================================================================
  void setStopThresholdMps(float v)    { _speedStopThreshold_mps = v; }
  void setStartThresholdMps(float v)   { _speedStartThreshold_mps = v; }
  void setMinStopTimeMs(uint32_t ms)   { _minStopTime_ms = ms; }
  void setAbortStopTimeMs(uint32_t ms) { _abortStopTime_ms = ms; }

  //==========================================================================================
  // Core API
  //==========================================================================================
  void reset()
  {
    _drag = DragTiming{};
    _pendingKfReset = false;
    _Serial->println("DRAG: Reset, waiting for stop");
  }

  /**
   * @brief Update the drag FSM.
   *
   * @param now_ms       System time in milliseconds since power-on.
   * @param navDataGood  True if KF/GPS/navigation data is valid.
   * @param speed_mps    Fused speed estimate.
   * @param distance_m   Fused distance since KF reset (Model A).
   *
   * MISSING:
   * - Caller must provide now_ms consistently (do not mix with millis() inside other layers).
   * - External KF must reset when requested and publish distance_m from that reset origin.
   */
  void update(uint32_t now_ms, bool navDataGood, float speed_mps, float distance_m, float speedGPS_mps)
  {
     const float speed_mph = speed_mps * MPS2MPH;
     const float distanceRun_m = distance_m - _drag.startDistance_m;

    // Data quality gating
    if (!navDataGood)
    {
      _drag.stopTimerRunning = false;

      if (_drag.state == DRAG_RUNNING)
      {
        _Serial->println("-> Nav data invalid, aborting run");
        reset();
      }
      return;
    }

    switch (_drag.state)
    {
      case DRAG_WAIT_FOR_STOP:
      {
        _Serial->print("DRAG_WAIT_FOR_STOP:");
        _Serial->print(" time_ms "); _Serial->print(now_ms-_drag.stopStart_ms);
        _Serial->print(" timeout_ms "); _Serial->print(_minStopTime_ms);
        _Serial->println("");

        if (updateStopTimer(now_ms, speed_mps, _minStopTime_ms))
        {
          _drag.state = DRAG_ARMED;
          _Serial->println("**************************************************************");
          _Serial->println("-> Armed (vehicle stopped, nav data OK)");
          _Serial->println("**************************************************************");
        }
        break;
      }

      case DRAG_ARMED:
      {
        _Serial->println("DRAG_ARMED");
        if (speed_mps < _speedStopThreshold_mps)
        {
          // stay armed
        }
        else if (speed_mps >= _speedStartThreshold_mps)
        {
          // Launch detected -> start run
          _drag.state = DRAG_RUNNING;

          _drag.startTime_ms = now_ms;         // remember time we started
          _drag.startDistance_m = distance_m;  // remember where we started
          _drag.maxSpeed_mps = speed_mps;

          _drag.has_0_60 = false;
          _drag.has_quarter = false;

          _drag.stopTimerRunning = false;

          _Serial->print("-> Run started at t_ms = ");
          _Serial->print(now_ms);
          _Serial->print(", speed = ");
          _Serial->print(speed_mph, 2);
          _Serial->println(" mph");
        }
        else
        {
          // small creep, not enough to start
        }
        break;
      }

      case DRAG_RUNNING:
      {
        //const float distanceRun_m = distance_m - _drag.startDistance_m;

        _Serial->print("DRAG_RUNNING: ");
        _Serial->print(" time "); _Serial->print(elapsedSec(now_ms), 2);
        _Serial->print(" KF_mph "); _Serial->print(speed_mph, 2);
        _Serial->print(" GPS_mph "); _Serial->print(speedGPS_mps, 2);
        _Serial->print(" mile "); _Serial->print(distanceRun_m / MILE2MTR, 3);
        _Serial->println("");

        // Track max speed
        if (speed_mps > _drag.maxSpeed_mps)
        {
          _drag.maxSpeed_mps = speed_mps;
        }

        // 0-60 mph timing
        if (!_drag.has_0_60 && (speed_mps >= TARGET_60_MPS))
        {
          _drag.has_0_60 = true;
          _drag.t_0_60_sec = elapsedSec(now_ms);
          _drag.v_0_60_mps = speed_mps;

          _Serial->println("**************************************************************");
          _Serial->print("-> 0-60 mph in ");
          _Serial->print(_drag.t_0_60_sec, 3);
          _Serial->print(" s, speed = ");
          _Serial->print(speed_mph, 2);
          _Serial->println(" mph");
          _Serial->println("**************************************************************");
        }

        // 1/4 mile completion
        if (!_drag.has_quarter && (distanceRun_m >= QUARTER_MILE_M))
        {
          _drag.has_quarter = true;
          _drag.t_quarter_sec = elapsedSec(now_ms);
          _drag.v_quarter_mps = speed_mps;
          _drag.dist_quarter_m = distanceRun_m;

          _drag.state = DRAG_FINISHED;

          printRunSummary();
        }

        // Abort condition: stop too long before 1/4 mile
        if (!_drag.has_quarter)
        {
          if (updateStopTimer(now_ms, speed_mps, _abortStopTime_ms))
          {
            _Serial->println("**************************************************************");
            _Serial->println("-> !!! Run aborted (vehicle stopped/slowed before 1/4 mile) !!!");
            _Serial->println("**************************************************************");
            reset();
          }
        }

        break;
      }

      case DRAG_FINISHED:
      {
        _Serial->print("DRAG_FINISHED (waiting for stop): ");
        _Serial->print(" time "); _Serial->print(elapsedSec(now_ms), 2);
        _Serial->print(" mph "); _Serial->print(speed_mph, 2);
        _Serial->print(" mile "); _Serial->print(distanceRun_m / MILE2MTR, 3);
        _Serial->println("");

        // After finished run, wait for stop then auto-reset
        if (updateStopTimer(now_ms, speed_mps, _minStopTime_ms))
        {
          _Serial->println("**************************************************************");
          _Serial->println("-> Finished run, vehicle stopped. Ready for next run");
          _Serial->println("**************************************************************");
          reset();
        }
        break;
      }

      default:
        reset();
        break;
    }
  }

  //==========================================================================================
  // Accessors
  //==========================================================================================
  const DragTiming& data() const { return _drag; }
  DragState state() const { return _drag.state; }

private:
  //==========================================================================================
  // Internal configuration (defaults match original intent)
  //==========================================================================================
  float    _speedStopThreshold_mps  = 0.8f;
  float    _speedStartThreshold_mps = 1.2f;
  uint32_t _minStopTime_ms          = 2000;
  uint32_t _abortStopTime_ms        = 2000;

  //==========================================================================================
  // Internal state
  //==========================================================================================
  DragTiming _drag;
  bool _pendingKfReset = false;
  usb_serial_class* _Serial;

  //==========================================================================================
  // Helpers
  //==========================================================================================
  float elapsedSec(uint32_t now_ms) const
  {
    if (_drag.startTime_ms == 0) return 0.0f;
    const uint32_t dt_ms = now_ms - _drag.startTime_ms;
    return dt_ms * 0.001f;
  }

  bool updateStopTimer(uint32_t now_ms, float speed_mps, uint32_t timeout_ms)
  {
    if (speed_mps < _speedStopThreshold_mps)
    {
      if (!_drag.stopTimerRunning)
      {
        _drag.stopTimerRunning = true;
        _drag.stopStart_ms = now_ms;
      }
      else
      {
        if ((now_ms - _drag.stopStart_ms) > timeout_ms)
        {
          _drag.stopTimerRunning = false;
          return true;
        }
      }
    }
    else
    {
      _drag.stopTimerRunning = false;
    }
    return false;
  }

  void printRunSummary()
  {
    _Serial->println("");
    _Serial->println("**************************************************************");
    _Serial->println("**************************************************************");
    _Serial->println("-> Run complete");
    _Serial->print("-> DRAG_RESULT, ");
    _Serial->print(", t_0-60="); _Serial->print(_drag.has_0_60 ? _drag.t_0_60_sec : -1.0f, 3);
    _Serial->print(", t_quarter="); _Serial->print(_drag.t_quarter_sec, 3);
    _Serial->print(", v_0-60_mph=");
    _Serial->print(_drag.has_0_60 ? _drag.v_0_60_mps * MPS2MPH : 0.0f, 2);
    _Serial->print(", v_quarter_mph=");
    _Serial->print(_drag.v_quarter_mps * MPS2MPH, 2);
    _Serial->print(", dist_quarter_m=");
    _Serial->print(_drag.dist_quarter_m, 2);
    _Serial->print(", maxSpeed_mph=");
    _Serial->print(_drag.maxSpeed_mps * MPS2MPH, 2);
    _Serial->println("");
    _Serial->println("**************************************************************");
    _Serial->println("**************************************************************");
    _Serial->println("");
  }
};
