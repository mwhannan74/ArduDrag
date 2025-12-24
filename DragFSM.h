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

  struct DragStatus
  {
    // default state 
    DragState state = DRAG_WAIT_FOR_STOP; 

    // stop detection timer (arming/abort/reset)
    uint32_t stopStart_ms     = 0;
    bool     stopTimerRunning = false;
  };

  struct DragLog
  {
    // Run time bookkeeping
    uint32_t startTime_ms = 0;        // system time at launch
    float    startDistance_m = 0.0f;  // global distance at launch
    float    maxSpeed_mps = 0.0f;

    // 1) 0–60 mph time, mph, and distance
    bool  has_0_60   = false;
    float t_0_60_sec = 0.0f;
    float v_0_60_mps = 0.0f;
    float d_0_60_m   = 0.0f;

    // 2) 60 ft time and mph
    bool  has_60ft   = false;
    float t_60ft_sec = 0.0f;
    float v_60ft_mps = 0.0f;

    // 3) 330 ft time and mph
    bool  has_330ft   = false;
    float t_330ft_sec = 0.0f;
    float v_330ft_mps = 0.0f;

    // 4) 660 ft (1/8 mile ET) time and mph
    bool  has_660ft   = false;
    float t_660ft_sec = 0.0f;
    float v_660ft_mps = 0.0f;

    // 5) 1000 ft time and mph
    bool  has_1000ft   = false;
    float t_1000ft_sec = 0.0f;
    float v_1000ft_mps = 0.0f;

    // 6) 1320 ft (1/4 mile) time and mph
    bool  has_quarter    = false;
    float t_quarter_sec  = 0.0f;
    float v_quarter_mps  = 0.0f;
    float dist_quarter_m = 0.0f; // captured input distance at completion
  };

  //==========================================================================================
  // Constants / conversions
  //==========================================================================================
  static constexpr float MPS2MPH       = 2.23693629f;  
  static constexpr float MILE2MTR      = 1609.34f;
  static constexpr float MTR2FT        = 3.28084f;

  // Distances in feet
  static constexpr float DIST_60FT_FT   = 60.0f;
  static constexpr float DIST_330FT_FT  = 330.0f;
  static constexpr float DIST_660FT_FT  = 660.0f;
  static constexpr float DIST_1000FT_FT = 1000.0f;
  static constexpr float DIST_1320FT_FT = 1320.0f;

  // Same distances in meters
  static constexpr float DIST_60FT_M   = DIST_60FT_FT   / MTR2FT;
  static constexpr float DIST_330FT_M  = DIST_330FT_FT  / MTR2FT;
  static constexpr float DIST_660FT_M  = DIST_660FT_FT  / MTR2FT;
  static constexpr float DIST_1000FT_M = DIST_1000FT_FT / MTR2FT;
  static constexpr float QTR_MILE_IN_FT  = DIST_1320FT_FT;
  static constexpr float QTR_MILE_IN_MTR = DIST_1320FT_FT / MTR2FT;  

  static constexpr float TARGET_60_MPH = 60.0f;
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
    _dragStatus = DragStatus{}; // sets DRAG_WAIT_FOR_STOP and stopStart_ms = 0
    _dragLog = DragLog{}; // clear log data
    _Serial->println("DRAG::Reset()");
  }

  /**
   * @brief Update the drag FSM.
   *
   * @param now_ms       System time in milliseconds since power-on.
   * @param navDataGood  True if KF/GPS/navigation data is valid.
   * @param speedKF_mps  Fused speed estimate.
   * @param distance_m   Fused distance since KF reset (Model A).
   * @param speedGPS_mps Speed raw from GPS
   */
  void update(uint32_t now_ms, bool navDataGood, float speedKF_mps, float distance_m, float speedGPS_mps)
  {
    // Select which speed source to use
    float speed_mps = speedKF_mps; // Use KF speed
    //float speed_mps = speedGPS_mps;  // Use GPS speed

    // convert to mph
    const float speed_mph = speed_mps * MPS2MPH;
    const float speedKF_mph = speedKF_mps * MPS2MPH; 
    const float speedGPS_mph = speedGPS_mps * MPS2MPH;
    
    // distance
    const float distanceRun_m = distance_m - _dragLog.startDistance_m;
    //float distanceRun_ft = distanceRun_m * MTR2FT;

    // Data quality gating
    if (!navDataGood)
    {      
      _dragStatus.stopTimerRunning = false;

      // Stop and reset if we are actively running
      if (_dragStatus.state == DRAG_RUNNING)
      {
        // EVENT: GPS is bad
        _Serial->println("-> Nav data invalid, ABORTING RUN");
        reset(); // reset data and state back to DRAG_WAIT_FOR_STOP
      }
      return;
    }

    switch (_dragStatus.state)
    {
      case DRAG_WAIT_FOR_STOP:
      {
        if( DEBUG ) _Serial->print("DRAG_WAIT_FOR_STOP:");

        bool isStoppedLongEnough = updateStopTimer(now_ms, speed_mps, _minStopTime_ms);

        if( !isStoppedLongEnough )
        {
          if( _dragStatus.stopTimerRunning )
          {
            // stopped but not for long enough
            if( DEBUG )
            {              
              _Serial->print(" Stopped ");
              _Serial->print(" time_ms "); _Serial->print(now_ms-_dragStatus.stopStart_ms);
              _Serial->print(" timeout_ms "); _Serial->print(_minStopTime_ms);     
              _Serial->println("");         
            }
          }
          else
          {
            // not stopped yet
            if( DEBUG )
            {
              _Serial->print(" Moving ");
              _Serial->print(" KF_mph "); _Serial->print(speedKF_mph, 2);
              _Serial->print(" GPS_mph "); _Serial->print(speedGPS_mph, 2);
              _Serial->print(" SPD_mph "); _Serial->print(speed_mph, 2);
              _Serial->print(" Threshold_MPH "); _Serial->print(_speedStopThreshold_mps*MPS2MPH, 2);               
              _Serial->println("");
            }
          }          
        }
        else
        {
          // EVENT: detected complete stop for long enough
          _dragStatus.state = DRAG_ARMED;
          _Serial->println("");
          _Serial->println("**************************************************************");
          _Serial->println("-> Armed (vehicle stopped, nav data OK)");
          _Serial->println("**************************************************************");
        }

        break;
      }

      case DRAG_ARMED:
      {
        if( DEBUG )
        {
          _Serial->print("DRAG_ARMED:");
          _Serial->print(" KF_mph "); _Serial->print(speedKF_mph, 2);
          _Serial->print(" GPS_mph "); _Serial->print(speedGPS_mph, 2);
          _Serial->print(" SPD_mph "); _Serial->print(speed_mph, 2);
          _Serial->print(" Threshold_MPH "); _Serial->println(_speedStartThreshold_mps*MPS2MPH, 2);
        }

        if (speed_mps < _speedStopThreshold_mps)
        {
          // stay armed
        }
        else if (speed_mps >= _speedStartThreshold_mps)
        {
          // EVENT: detected movement
          _dragStatus.state = DRAG_RUNNING;
          _Serial->println("Movement Detected --> Changing to DRAG_RUNNING");

          // initialize drag info
          uint32_t timeOffset_ms = 200; // compenstate for the short time it took to cross the speed threshold
          float posOffset_m = 0.2;

          _dragLog = DragLog{};                           // clear previous run data
          _dragLog.startTime_ms = now_ms - timeOffset_ms; // remember time we started
          _dragLog.startDistance_m = distance_m - posOffset_m; // remember where we started
          _dragLog.maxSpeed_mps = speed_mps;

          // reset stop timer flag
          _dragStatus.stopTimerRunning = false;
        }
        else
        {
          // > stop speed but less than start threadhold (small creep, not enough to start)
        }
        break;
      }

      case DRAG_RUNNING:
      {
        if( DEBUG2 )
        {
          _Serial->print("DRAG_RUNNING:");
          _Serial->print(" time "); _Serial->print(elapsedSec(now_ms), 2);
          _Serial->print(" KF_mph "); _Serial->print(speedKF_mph, 2);
          _Serial->print(" GPS_mph "); _Serial->print(speedGPS_mph, 2);
          _Serial->print(" SPD_mph "); _Serial->print(speed_mph, 2);
          _Serial->print(" Dist_ft "); _Serial->print(distanceRun_m*MTR2FT, 1);
          _Serial->println("");
        }
        else
        {
          PRINT = !PRINT;
          if( PRINT )
          {
            _Serial->print("KF "); _Serial->print(speedKF_mph, 2);
            _Serial->print(" GPS "); _Serial->print(speedGPS_mph, 2);
            _Serial->println("");
          }
        }

        // Track max speed
        if (speed_mps > _dragLog.maxSpeed_mps)
        {
          _dragLog.maxSpeed_mps = speed_mps;
        }

        // 1) 0-60 mph timing
        if (!_dragLog.has_0_60 && (speed_mps >= TARGET_60_MPS))
        {
          // EVENT: Reached 60 mph (early exit for testing)
          //_dragStatus.state = DRAG_FINISHED;

          // data
          _dragLog.has_0_60   = true;
          _dragLog.t_0_60_sec = elapsedSec(now_ms);
          _dragLog.v_0_60_mps = speed_mps;
          _dragLog.d_0_60_m   = distanceRun_m;  // capture distance at 60 mph

          _Serial->println("**************************************************************");
          _Serial->print("-> 0-"); _Serial->print(TARGET_60_MPH,0); _Serial->print(" mph in ");
          _Serial->print(_dragLog.t_0_60_sec, 3);
          _Serial->print(" sec, dist = ");
          _Serial->print(_dragLog.d_0_60_m * MTR2FT);
          _Serial->println(" ft");
          _Serial->println("**************************************************************");
        }

        // 2) 60 ft time and mph
        if (!_dragLog.has_60ft && (distanceRun_m >= DIST_60FT_M))
        {
          _dragLog.has_60ft   = true;
          _dragLog.t_60ft_sec = elapsedSec(now_ms);
          _dragLog.v_60ft_mps = speed_mps;

          _Serial->println("**************************************************************");
          _Serial->print("-> 60 ft in ");
          _Serial->print(_dragLog.t_60ft_sec, 3);
          _Serial->print(" sec @ ");
          _Serial->print(_dragLog.v_60ft_mps * MPS2MPH, 2);
          _Serial->println(" mph");
          _Serial->println("**************************************************************");
        }

        // 3) 330 ft time and mph
        if (!_dragLog.has_330ft && (distanceRun_m >= DIST_330FT_M))
        {
          _dragLog.has_330ft   = true;
          _dragLog.t_330ft_sec = elapsedSec(now_ms);
          _dragLog.v_330ft_mps = speed_mps;

          _Serial->println("**************************************************************");
          _Serial->print("-> 330 ft in ");
          _Serial->print(_dragLog.t_330ft_sec, 3);
          _Serial->print(" sec @ ");
          _Serial->print(_dragLog.v_330ft_mps * MPS2MPH, 2);
          _Serial->println(" mph");
          _Serial->println("**************************************************************");
        }

        // 4) 660 ft (1/8 mile) time and mph
        if (!_dragLog.has_660ft && (distanceRun_m >= DIST_660FT_M))
        {
          // EVENT: Reached end of 1/8 mile
          // _dragStatus.state = DRAG_FINISHED;

          _dragLog.has_660ft   = true;
          _dragLog.t_660ft_sec = elapsedSec(now_ms);
          _dragLog.v_660ft_mps = speed_mps;

          _Serial->println("**************************************************************");
          _Serial->print("-> 1/8 mile (660 ft) in ");
          _Serial->print(_dragLog.t_660ft_sec, 3);
          _Serial->print(" sec @ ");
          _Serial->print(_dragLog.v_660ft_mps * MPS2MPH, 2);
          _Serial->println(" mph");
          _Serial->println("**************************************************************");
        }

        // 5) 1000 ft time and mph
        if (!_dragLog.has_1000ft && (distanceRun_m >= DIST_1000FT_M))
        {
          _dragLog.has_1000ft   = true;
          _dragLog.t_1000ft_sec = elapsedSec(now_ms);
          _dragLog.v_1000ft_mps = speed_mps;

          _Serial->println("**************************************************************");
          _Serial->print("-> 1000 ft in ");
          _Serial->print(_dragLog.t_1000ft_sec, 3);
          _Serial->print(" sec @ ");
          _Serial->print(_dragLog.v_1000ft_mps * MPS2MPH, 2);
          _Serial->println(" mph");
          _Serial->println("**************************************************************");
        }

        // 6) 1320 ft (1/4 mile) time and mph --> end of run no matter what
        if (!_dragLog.has_quarter && (distanceRun_m >= QTR_MILE_IN_MTR))
        {
          // EVENT: Reached end of 1/4 mile
          _dragStatus.state = DRAG_FINISHED;

          _dragLog.has_quarter    = true;
          _dragLog.t_quarter_sec  = elapsedSec(now_ms);
          _dragLog.v_quarter_mps  = speed_mps;
          _dragLog.dist_quarter_m = distanceRun_m;

          _Serial->println("**************************************************************");
          _Serial->print("-> 1/4 mile (1320 ft) in ");
          _Serial->print(_dragLog.t_quarter_sec, 3);
          _Serial->print(" sec @ ");
          _Serial->print(_dragLog.v_quarter_mps * MPS2MPH, 2);
          _Serial->println(" mph");
          _Serial->println("**************************************************************");
        }

        // Abort condition: stop too long before 1/4 mile
        if (!_dragLog.has_quarter)
        {
          // Check for stop and that stopped for longer than abortStopTime_ms
          if (updateStopTimer(now_ms, speed_mps, _abortStopTime_ms))
          {
            // EVENT: 
            _Serial->println("**************************************************************");
            _Serial->println("!!! Run aborted (vehicle stopped) !!!");
            printRunSummary();
            _Serial->println("**************************************************************");
            reset(); // reset data and state back to DRAG_WAIT_FOR_STOP
          }
        }

        break;
      }

      case DRAG_FINISHED:
      {
        // _Serial->print("DRAG_FINISHED (waiting for stop): ");
        // _Serial->print(" eTime "); _Serial->print(elapsedSec(now_ms), 2);
        // _Serial->print(" KF_mph "); _Serial->print(speedKF_mph, 2);
        // _Serial->print(" GPS_mph "); _Serial->print(speedGPS_mph, 2);
        // _Serial->print(" SPD_mph "); _Serial->print(speed_mph, 2);
        // _Serial->print(" Threshold_MPH "); _Serial->println(_speedStartThreshold_mps*MPS2MPH, 2); 

        // // After finished run, wait for stop then auto-reset
        // if (updateStopTimer(now_ms, speed_mps, _minStopTime_ms))
        // {
        //   _Serial->println("**************************************************************");
        //   _Serial->println("-> Finished run, vehicle stopped. Ready for next run");
        //   _Serial->println("**************************************************************");
        //   reset();
        // }

        _Serial->println("DRAG_FINISHED: Come to a stop to arm for another run");
        printRunSummary();      
        reset(); // sets DRAG_WAIT_FOR_STOP and stopStart_ms = 0
        updateStopTimer(now_ms, speed_mps, _minStopTime_ms);
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
  const DragLog& dataLog() const { return _dragLog; }
  DragState state() const { return _dragStatus.state; }

private:
  bool DEBUG = false;
  bool DEBUG2 = false;
  bool PRINT = false;

  //==========================================================================================
  // Internal configuration (defaults match original intent)
  //==========================================================================================
  float    _speedStopThreshold_mps  = 0.3f; // speed less than this is defined as "stopped" 
  float    _speedStartThreshold_mps = 0.4f; // speed greater than this is defined as "moving/started" if previously "stopped"
  uint32_t _minStopTime_ms          = 2000; // car has been "truly stopped" for at least this long, then reset/arm
  uint32_t _abortStopTime_ms        = 2000; // While a run is in progress, if car "slows down or stops" for this long without reaching 1/4 mile, consider the run invalid and reset

  //==========================================================================================
  // Internal state
  //==========================================================================================
  DragStatus _dragStatus;
  DragLog _dragLog;

  usb_serial_class* _Serial;

  //==========================================================================================
  // Helpers
  //==========================================================================================
  float elapsedSec(uint32_t now_ms) const
  {
    if (_dragLog.startTime_ms == 0) return 0.0f;
    const uint32_t dt_ms = now_ms - _dragLog.startTime_ms;
    return dt_ms * 0.001f;
  }

  /**
  * Stop-detection timer helper.
  *
  * Tracks how long the vehicle's speed has remained below the configured stop threshold.
  *
  * Returns true once the speed has been continuously below the threshold for at least
  * timeout_ms milliseconds. The internal timer is automatically started, stopped, and
  * reset based on the current speed.
  *
  * @param now_ms     Current system time in milliseconds.
  * @param speed_mps  Current vehicle speed in m/s.
  * @param timeout_ms Required duration below the stop threshold before returning true.
  *
  * @return true  if the vehicle has been below the stop threshold for at least timeout_ms.
  * @return false otherwise.
  */
  bool updateStopTimer(uint32_t now_ms, float speed_mps, uint32_t timeout_ms)
  {
    // Check if we are currently below the "stopped" speed threshold.
    if (speed_mps < _speedStopThreshold_mps)
    {
      // If the stop timer is not running yet, start it now.
      if (!_dragStatus.stopTimerRunning)
      {
        _dragStatus.stopTimerRunning = true;
        _dragStatus.stopStart_ms = now_ms;   // record when we first went "stopped"
      }
      else
      {
        // Timer is already running: check how long we've been "stopped".
        if ((now_ms - _dragStatus.stopStart_ms) > timeout_ms)
        {
          // We've been below the threshold long enough; stop the timer and signal timeout.
          _dragStatus.stopTimerRunning = false;
          return true;                 // condition satisfied
        }
      }
    }
    else
    {
      // Speed is above the stop threshold -> cancel any in-progress stop timing.
      _dragStatus.stopTimerRunning = false;
    }
    // Either not below threshold long enough, or not stopped at all.
    return false;
  }


  void printRunSummary()
  {
    _Serial->println("==============================================================");
    _Serial->println("TIME SLIP: ");
    
    _Serial->print("0-60  ... "); _Serial->println(_dragLog.has_0_60 ? _dragLog.t_0_60_sec : -1.0f, 3);
    _Serial->print("DIST  ... "); _Serial->println(_dragLog.has_0_60 ? _dragLog.d_0_60_m*MTR2FT : -1.0f, 1);

    _Serial->print("60'   ... "); _Serial->println(_dragLog.t_60ft_sec, 3);
    _Serial->print("MPH   ... "); _Serial->println(_dragLog.v_60ft_mps * MPS2MPH, 2);

    _Serial->print("330'  ... "); _Serial->println(_dragLog.t_330ft_sec, 3);
    _Serial->print("MPH   ... "); _Serial->println(_dragLog.v_330ft_mps * MPS2MPH, 2);

    _Serial->print("1/8   ... "); _Serial->println(_dragLog.t_660ft_sec, 3);
    _Serial->print("MPH   ... "); _Serial->println(_dragLog.v_660ft_mps * MPS2MPH, 2);

    _Serial->print("1000' ... "); _Serial->println(_dragLog.t_1000ft_sec, 3);
    _Serial->print("MPH   ... "); _Serial->println(_dragLog.v_1000ft_mps * MPS2MPH, 2);
    
    _Serial->print("1/4   ... "); _Serial->println(_dragLog.t_quarter_sec, 3);
    _Serial->print("MPH   ... "); _Serial->println(_dragLog.v_quarter_mps * MPS2MPH, 2);

    _Serial->println("==============================================================");
    _Serial->println("");
  }
};
