// namespace dragFSM {

// //============================================================================================
// // DRAG TIMING STATE MACHINE (0-60 mph and 1/4 mile)
// //============================================================================================

// // Distance for 1/4 mile in meters
// static const float QUARTER_MILE_M = 402.336;

// // Speed thresholds (m/s)
// static const float SPEED_STOP_THRESHOLD_MPS  = 0.5f; // below this ~ stopped (~1.1 mph)
// static const float SPEED_START_THRESHOLD_MPS = 1.0f; // above this = run starts (~2.2 mph)
// static const float TARGET_60_MPH             = 60.0f;
// static const float TARGET_60_MPS             = TARGET_60_MPH / MPS2MPH;

// // Timing thresholds (ms)
// static const uint32_t MIN_STOP_TIME_MS   = 2000; // must be stopped this long to arm
// static const uint32_t ABORT_STOP_TIME_MS = 2000; // slowed too long -> abort

// // Drag run states
// enum DragState
// {
//   DRAG_WAIT_FOR_STOP = 0,   // waiting for vehicle to be fully stopped with GPS fix
//   DRAG_ARMED         = 1,   // stopped and GPS OK, waiting for launch
//   DRAG_RUNNING       = 2,   // run in progress
//   DRAG_FINISHED      = 3    // run complete, waiting for reset
// };

// struct DragTiming
// {
//   DragState state = DRAG_WAIT_FOR_STOP;

//   // Run bookkeeping
//   float startTime_sec = 0.0f;
//   float distance_integrated_m = 0.0f;
//   float distance_haversine_m = 0.0f;
//   float startLat_deg = 0.0f;
//   float startLon_deg = 0.0f;
//   float lastSpeed_mps = 0.0f;
//   float maxSpeed_mps = 0.0f;

//   // 0-60 mph results
//   bool  has_0_60 = false;
//   float t_0_60_sec = 0.0f;
//   float v_0_60_mps = 0.0f;

//   // 1/4 mile results (two distance methods)
//   bool  has_quarter = false;
//   float t_quarter_int_sec = 0.0f; // using integrated speed distance
//   float t_quarter_hav_sec = 0.0f; // using haversine distance
//   float v_quarter_mps = 0.0f;

//   // stop detection timer (for arming and abort/reset)
//   uint32_t stopStart_ms = 0;
//   bool     stopTimerRunning = false;

// } _drag;


// // Haversine distance between two lat/lon points in meters
// float haversineDistance_m(float lat1_deg, float lon1_deg, float lat2_deg, float lon2_deg)
// {
//   const float R = 6371000.0f; // Earth radius in meters

//   float phi1 = lat1_deg * D2R;
//   float phi2 = lat2_deg * D2R;
//   float dphi = (lat2_deg - lat1_deg) * D2R;
//   float dlambda = (lon2_deg - lon1_deg) * D2R;

//   float sin_dphi_2    = sin(dphi * 0.5f);
//   float sin_dlambda_2 = sin(dlambda * 0.5f);

//   float a = sin_dphi_2 * sin_dphi_2 +
//             cos(phi1) * cos(phi2) * sin_dlambda_2 * sin_dlambda_2;

//   float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
//   float d = R * c;

//   return d;
// }

// void resetDragState()
// {
//   _drag.state = DRAG_WAIT_FOR_STOP;

//   _drag.startTime_sec = 0.0f;
//   _drag.distance_integrated_m = 0.0f;
//   _drag.distance_haversine_m = 0.0f;
//   _drag.startLat_deg = 0.0f;
//   _drag.startLon_deg = 0.0f;
//   _drag.lastSpeed_mps = 0.0f;
//   _drag.maxSpeed_mps = 0.0f;

//   _drag.has_0_60 = false;
//   _drag.t_0_60_sec = 0.0f;
//   _drag.v_0_60_mps = 0.0f;

//   _drag.has_quarter = false;
//   _drag.t_quarter_int_sec = 0.0f;
//   _drag.t_quarter_hav_sec = 0.0f;
//   _drag.v_quarter_mps = 0.0f;

//   _drag.stopStart_ms = 0;
//   _drag.stopTimerRunning = false;

//   Serial.println("DRAG: Reset, waiting for stop.");
// }

// // Print final run summary in a single line for easy parsing
// void printDragRunSummary()
// {
//   Serial.println("DRAG: Run complete.");
//   Serial.print("DRAG_RESULT, ");
//   Serial.print("t_start=");           Serial.print(_drag.startTime_sec, 3);
//   Serial.print(", t_0_60=");          Serial.print(_drag.has_0_60 ? _drag.t_0_60_sec : -1.0f, 3);
//   Serial.print(", t_quarter_int=");   Serial.print(_drag.t_quarter_int_sec, 3);
//   Serial.print(", t_quarter_hav=");   Serial.print(_drag.t_quarter_hav_sec, 3);
//   Serial.print(", v_0_60_mph=");
//   Serial.print(_drag.has_0_60 ? _drag.v_0_60_mps * MPS2MPH : 0.0f, 2);
//   Serial.print(", v_quarter_mph=");
//   Serial.print(_drag.v_quarter_mps * MPS2MPH, 2);
//   Serial.print(", dist_int_m=");
//   Serial.print(_drag.distance_integrated_m, 2);
//   Serial.print(", dist_hav_m=");
//   Serial.print(_drag.distance_haversine_m, 2);
//   Serial.print(", maxSpeed_mph=");
//   Serial.print(_drag.maxSpeed_mps * MPS2MPH, 2);
//   Serial.println("");
// }

// // Main drag-timing state machine update
// // dt_sec is the time since the last fusion update
// void updateDragTiming(float dt_sec)
// {
//   // Require a valid GPS fix for all run logic
//   if( !GPS.fix )
//   {
//     // If we lose GPS fix while running, safest is to abort and reset
//     if( _drag.state == DRAG_RUNNING )
//     {
//       Serial.println("DRAG: GPS fix lost, aborting run.");
//       resetDragState();
//     }
//     return;
//   }

//   float speed_mps = _pose.speed_mps;
//   float speed_mph = speed_mps * MPS2MPH;

//   // Generic stop-detection helper (used in several states)
//   auto updateStopTimer = [&](uint32_t timeout_ms) -> bool
//   {
//     if( speed_mps < SPEED_STOP_THRESHOLD_MPS )
//     {
//       if( !_drag.stopTimerRunning )
//       {
//         _drag.stopTimerRunning = true;
//         _drag.stopStart_ms = millis();
//       }
//       else
//       {
//         if( millis() - _drag.stopStart_ms > timeout_ms )
//         {
//           _drag.stopTimerRunning = false;
//           return true; // timeout reached
//         }
//       }
//     }
//     else
//     {
//       _drag.stopTimerRunning = false;
//     }
//     return false;
//   };

//   switch( _drag.state )
//   {
//     case DRAG_WAIT_FOR_STOP:
//     {
//       // Wait until the vehicle is stopped for a bit with a valid fix
//       if( updateStopTimer(MIN_STOP_TIME_MS) )
//       {
//         _drag.state = DRAG_ARMED;
//         Serial.println("DRAG: Armed (vehicle stopped, GPS fix OK).");
//       }
//       break;
//     }

//     case DRAG_ARMED:
//     {
//       // Still require vehicle to remain near-stopped until launch
//       if( speed_mps < SPEED_STOP_THRESHOLD_MPS )
//       {
//         // stay armed
//       }
//       else if( speed_mps >= SPEED_START_THRESHOLD_MPS )
//       {
//         // Launch detected -> start run
//         _drag.state = DRAG_RUNNING;

//         _drag.startTime_sec = _pose.time_sec;
//         _drag.startLat_deg  = _pose.lat_deg;
//         _drag.startLon_deg  = _pose.lon_deg;
//         _drag.lastSpeed_mps = speed_mps;
//         _drag.maxSpeed_mps  = speed_mps;

//         _drag.distance_integrated_m = 0.0f;
//         _drag.distance_haversine_m = 0.0f;

//         _drag.has_0_60 = false;
//         _drag.has_quarter = false;

//         _drag.stopTimerRunning = false;

//         Serial.print("DRAG: Run started at t = ");
//         Serial.print(_drag.startTime_sec, 3);
//         Serial.print(" s, speed = ");
//         Serial.print(speed_mph, 2);
//         Serial.println(" mph.");
//       }
//       else
//       {
//         // small creep, but not enough to start; stay in ARMED
//       }
//       break;
//     }

//     case DRAG_RUNNING:
//     {
//       // Track max speed for info
//       if( speed_mps > _drag.maxSpeed_mps )
//       {
//         _drag.maxSpeed_mps = speed_mps;
//       }

//       // 1) Distance from integrated speed (trapezoidal integration)
//       float v_prev = _drag.lastSpeed_mps;
//       float v_curr = speed_mps;
//       _drag.distance_integrated_m += 0.5f * (v_prev + v_curr) * dt_sec;
//       _drag.lastSpeed_mps = v_curr;

//       // 2) Distance from haversine between start position and current position
//       _drag.distance_haversine_m = haversineDistance_m(_drag.startLat_deg, _drag.startLon_deg,
//                                                       _pose.lat_deg, _pose.lon_deg);

//       // 0-60 mph timing
//       if( !_drag.has_0_60 && (speed_mps >= TARGET_60_MPS) )
//       {
//         _drag.has_0_60 = true;
//         _drag.t_0_60_sec = _pose.time_sec - _drag.startTime_sec;
//         _drag.v_0_60_mps = speed_mps;

//         Serial.print("DRAG: 0-60 mph in ");
//         Serial.print(_drag.t_0_60_sec, 3);
//         Serial.print(" s, speed = ");
//         Serial.print(speed_mph, 2);
//         Serial.println(" mph.");
//       }

//       // 1/4 mile (check both distance methods)
//       bool reachedInt = (_drag.distance_integrated_m  >= QUARTER_MILE_M);
//       bool reachedHav = (_drag.distance_haversine_m   >= QUARTER_MILE_M);

//       if( !_drag.has_quarter && (reachedInt || reachedHav) )
//       {
//         _drag.has_quarter = true;

//         // For now, use current time as both ETs.
//         // If you want more precision, you can interpolate back to the exact crossing.
//         _drag.t_quarter_int_sec = _pose.time_sec - _drag.startTime_sec;
//         _drag.t_quarter_hav_sec = _drag.t_quarter_int_sec;

//         _drag.v_quarter_mps = speed_mps;

//         _drag.state = DRAG_FINISHED;

//         printDragRunSummary();
//       }

//       // Abort condition: if we slow down below stop threshold for a while before 1/4 mile
//       if( !_drag.has_quarter )
//       {
//         if( updateStopTimer(ABORT_STOP_TIME_MS) )
//         {
//           Serial.println("DRAG: Run aborted (vehicle stopped/slowed before 1/4 mile).");
//           resetDragState();
//         }
//       }
//       break;
//     }

//     case DRAG_FINISHED:
//     {
//       // After a finished run, wait for the car to come to a stop, then auto-reset
//       if( updateStopTimer(MIN_STOP_TIME_MS) )
//       {
//         Serial.println("DRAG: Finished run, vehicle stopped. Ready for next run.");
//         resetDragState();
//       }
//       break;
//     }

//     default:
//       resetDragState();
//       break;
//   }
// }

// } // namespace