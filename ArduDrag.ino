// GPS
#include <Adafruit_GPS.h>
#include <vector>
#include <string>

#include "KalmanFilter.h"

//============================================================================================
// HELPER FUNCTIONS
//============================================================================================
static const float D2R = PI/180.0;
static const float R2D = 180.0/PI;
static const float MPS2MPH = 2.23693629;
static const float KNTS2MPS = 0.514444; // GPS returns speed in Knots
static const float KNTS2MPH = 1.15078;

float angleWrap_deg(float angle)
{
  float aw = angle;
  if( angle > 180.0 ) aw = angle - 360.0;
  else if( angle < -180.0 ) aw = angle + 360;
  return aw;
}


//============================================================================================
// Onboard LED
//============================================================================================
static const int _ledPin = 13;
bool _ledOn = true;


//============================================================================================
// Adafruit Ultimate GPS 
//============================================================================================
#define SerialGPS Serial1 // rename for convenience

Adafruit_GPS GPS(&SerialGPS);
static const std::vector<std::string> GpsFixQuality{"invalid", "SPS", "DGPS", "PPS", "RTK", "fRTK", "dead reck", "manual", "sim"};

void setupGPS()
{
  SerialGPS.begin(9600);
  delay(500);

  // Change to higher baud rate for robustness using 2 NMEA messages.
  GPS.sendCommand(PMTK_SET_BAUD_57600);
  SerialGPS.end();
  delay(200);  
  SerialGPS.begin(57600);
  delay(500);

  // Always request RMC + GGA (RMC = lat/lon/sog/cog/magVar, GGA = lat/lon/alt/qual/sats)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 
  // Set the update rate to 10Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  
  delay(200);
}

// Read data from the GPS serial port in the 'main loop'.
// Need to call as fast as possible as the GPS.read() only reads one character at at time from the serial port.
// Returns an int value that corresponds to parsing result: NA=0, FAIL=1, RMC=2, GGA=3, OTHER=4
int readGPS()
{  
  int rVal = 0;  // NA --> no new sentence parsed

  // reads one character from the GPS serial stream and feeds it into the library’s internal NMEA sentence buffer
  GPS.read();
  
  // Check if  NMEA message is ready to parse
  if( GPS.newNMEAreceived() ) 
  {
    // full NMEA sentence has been received and is read to be parsed --> NEW DATA    
    if( strcmp(GPS.thisSentence,"RMC") == 0 ) rVal = 2;      // RMC = lat/lon/sog/cog/magVar
    else if( strcmp(GPS.thisSentence,"GGA") == 0 ) rVal = 3; // GGA = lat/lon/alt/qual/sats
    else rVal = 4;

    // Now parse the message to get the data and store it in the GPS object
    if( !GPS.parse(GPS.lastNMEA()) ) // parse() sets the newNMEAreceived() flag to false
    {
      return 1; // Failed to parse sentence -->  just wait for another
    }
  }
  return rVal;
}

void printGPS()
{
  Serial.print("GPS: satellites "); Serial.print(GPS.satellites);
  Serial.print(" Fix=");
    if( (int)GPS.fix ) Serial.print("True");
    else Serial.print("False");
  Serial.print(" Fix_quality="); Serial.print( GpsFixQuality[(int)GPS.fixquality].c_str() ); //"invalid", "SPS", "DGPS", "PPS", "RTK", "fRTK", "dead reck", "manual", "sim"
  
  // Does the GPS have a "Fix"?
  if( GPS.fix ) 
  {
    Serial.print(" Position ");
    Serial.print(GPS.latitudeDegrees,11); Serial.print(", ");     
    Serial.print(GPS.longitudeDegrees,11);
    
    Serial.print(" Altitude MSL (m) "); Serial.print(GPS.altitude);
    Serial.print(" Geoid Height (m) "); Serial.print(GPS.geoidheight);

    Serial.print(" SOG (mph) "); Serial.print(GPS.speed*KNTS2MPH);
    Serial.print(" COG (deg) "); Serial.print(GPS.angle);
    Serial.print(" Mag Var "); Serial.print((int)GPS.magvariation);
    
    Serial.println("");
  }
  else
  {
    Serial.println("");
  }
}


//============================================================================================
// POSE DATA - Interpolated/filtered data that we do all calculations on
//============================================================================================
struct
{
  float time_sec = 0;
  unsigned int numSat = 0;
  float lat_deg = 0;
  float lon_deg = 0;
  float speed_mps = 0;
  float heading_deg = 0;
 

  // print in CSV format
  // time_sec, lat_deg, lon_deg, speed_mps, heading_deg, pitch_deg, roll_deg, gpsFixQual, numSat
  void printCSV()
  {
    Serial.print("POSE_CSV, ");
    Serial.print(time_sec); Serial.print(", ");
    Serial.print(numSat); Serial.print(", ");
    Serial.print(lat_deg,11); Serial.print(", ");
    Serial.print(lon_deg,11); Serial.print(", ");
    Serial.print(speed_mps); Serial.print(", ");
    Serial.print(heading_deg);
    Serial.println("");
  }
} _pose;



//============================================================================================
// DRAG TIMING STATE MACHINE (0-60 mph and 1/4 mile)
//============================================================================================

// Distance for 1/4 mile in meters
static const float QUARTER_MILE_M = 402.336;

// Speed thresholds (m/s)
static const float SPEED_STOP_THRESHOLD_MPS  = 0.5f; // below this ~ stopped (~1.1 mph)
static const float SPEED_START_THRESHOLD_MPS = 1.0f; // above this = run starts (~2.2 mph)
static const float TARGET_60_MPH             = 60.0f;
static const float TARGET_60_MPS             = TARGET_60_MPH / MPS2MPH;

// Timing thresholds (ms)
static const uint32_t MIN_STOP_TIME_MS   = 2000; // must be stopped this long to arm
static const uint32_t ABORT_STOP_TIME_MS = 2000; // slowed too long -> abort

// Drag run states
enum DragState
{
  DRAG_WAIT_FOR_STOP = 0,   // waiting for vehicle to be fully stopped with GPS fix
  DRAG_ARMED         = 1,   // stopped and GPS OK, waiting for launch
  DRAG_RUNNING       = 2,   // run in progress
  DRAG_FINISHED      = 3    // run complete, waiting for reset
};

struct DragTiming
{
  DragState state = DRAG_WAIT_FOR_STOP;

  // Run bookkeeping
  float startTime_sec = 0.0f;
  float distance_integrated_m = 0.0f;
  float distance_haversine_m = 0.0f;
  float startLat_deg = 0.0f;
  float startLon_deg = 0.0f;
  float lastSpeed_mps = 0.0f;
  float maxSpeed_mps = 0.0f;

  // 0-60 mph results
  bool  has_0_60 = false;
  float t_0_60_sec = 0.0f;
  float v_0_60_mps = 0.0f;

  // 1/4 mile results (two distance methods)
  bool  has_quarter = false;
  float t_quarter_int_sec = 0.0f; // using integrated speed distance
  float t_quarter_hav_sec = 0.0f; // using haversine distance
  float v_quarter_mps = 0.0f;

  // stop detection timer (for arming and abort/reset)
  uint32_t stopStart_ms = 0;
  bool     stopTimerRunning = false;

} _drag;


// Haversine distance between two lat/lon points in meters
float haversineDistance_m(float lat1_deg, float lon1_deg, float lat2_deg, float lon2_deg)
{
  const float R = 6371000.0f; // Earth radius in meters

  float phi1 = lat1_deg * D2R;
  float phi2 = lat2_deg * D2R;
  float dphi = (lat2_deg - lat1_deg) * D2R;
  float dlambda = (lon2_deg - lon1_deg) * D2R;

  float sin_dphi_2    = sin(dphi * 0.5f);
  float sin_dlambda_2 = sin(dlambda * 0.5f);

  float a = sin_dphi_2 * sin_dphi_2 +
            cos(phi1) * cos(phi2) * sin_dlambda_2 * sin_dlambda_2;

  float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
  float d = R * c;

  return d;
}

void resetDragState()
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

  Serial.println("DRAG: Reset, waiting for stop.");
}

// Print final run summary in a single line for easy parsing
void printDragRunSummary()
{
  Serial.println("DRAG: Run complete.");
  Serial.print("DRAG_RESULT, ");
  Serial.print("t_start=");           Serial.print(_drag.startTime_sec, 3);
  Serial.print(", t_0_60=");          Serial.print(_drag.has_0_60 ? _drag.t_0_60_sec : -1.0f, 3);
  Serial.print(", t_quarter_int=");   Serial.print(_drag.t_quarter_int_sec, 3);
  Serial.print(", t_quarter_hav=");   Serial.print(_drag.t_quarter_hav_sec, 3);
  Serial.print(", v_0_60_mph=");
  Serial.print(_drag.has_0_60 ? _drag.v_0_60_mps * MPS2MPH : 0.0f, 2);
  Serial.print(", v_quarter_mph=");
  Serial.print(_drag.v_quarter_mps * MPS2MPH, 2);
  Serial.print(", dist_int_m=");
  Serial.print(_drag.distance_integrated_m, 2);
  Serial.print(", dist_hav_m=");
  Serial.print(_drag.distance_haversine_m, 2);
  Serial.print(", maxSpeed_mph=");
  Serial.print(_drag.maxSpeed_mps * MPS2MPH, 2);
  Serial.println("");
}

// Main drag-timing state machine update
// dt_sec is the time since the last fusion update
void updateDragTiming(float dt_sec)
{
  // Require a valid GPS fix for all run logic
  if( !GPS.fix )
  {
    // If we lose GPS fix while running, safest is to abort and reset
    if( _drag.state == DRAG_RUNNING )
    {
      Serial.println("DRAG: GPS fix lost, aborting run.");
      resetDragState();
    }
    return;
  }

  float speed_mps = _pose.speed_mps;
  float speed_mph = speed_mps * MPS2MPH;

  // Generic stop-detection helper (used in several states)
  auto updateStopTimer = [&](uint32_t timeout_ms) -> bool
  {
    if( speed_mps < SPEED_STOP_THRESHOLD_MPS )
    {
      if( !_drag.stopTimerRunning )
      {
        _drag.stopTimerRunning = true;
        _drag.stopStart_ms = millis();
      }
      else
      {
        if( millis() - _drag.stopStart_ms > timeout_ms )
        {
          _drag.stopTimerRunning = false;
          return true; // timeout reached
        }
      }
    }
    else
    {
      _drag.stopTimerRunning = false;
    }
    return false;
  };

  switch( _drag.state )
  {
    case DRAG_WAIT_FOR_STOP:
    {
      // Wait until the vehicle is stopped for a bit with a valid fix
      if( updateStopTimer(MIN_STOP_TIME_MS) )
      {
        _drag.state = DRAG_ARMED;
        Serial.println("DRAG: Armed (vehicle stopped, GPS fix OK).");
      }
      break;
    }

    case DRAG_ARMED:
    {
      // Still require vehicle to remain near-stopped until launch
      if( speed_mps < SPEED_STOP_THRESHOLD_MPS )
      {
        // stay armed
      }
      else if( speed_mps >= SPEED_START_THRESHOLD_MPS )
      {
        // Launch detected -> start run
        _drag.state = DRAG_RUNNING;

        _drag.startTime_sec = _pose.time_sec;
        _drag.startLat_deg  = _pose.lat_deg;
        _drag.startLon_deg  = _pose.lon_deg;
        _drag.lastSpeed_mps = speed_mps;
        _drag.maxSpeed_mps  = speed_mps;

        _drag.distance_integrated_m = 0.0f;
        _drag.distance_haversine_m = 0.0f;

        _drag.has_0_60 = false;
        _drag.has_quarter = false;

        _drag.stopTimerRunning = false;

        Serial.print("DRAG: Run started at t = ");
        Serial.print(_drag.startTime_sec, 3);
        Serial.print(" s, speed = ");
        Serial.print(speed_mph, 2);
        Serial.println(" mph.");
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
      if( speed_mps > _drag.maxSpeed_mps )
      {
        _drag.maxSpeed_mps = speed_mps;
      }

      // 1) Distance from integrated speed (trapezoidal integration)
      float v_prev = _drag.lastSpeed_mps;
      float v_curr = speed_mps;
      _drag.distance_integrated_m += 0.5f * (v_prev + v_curr) * dt_sec;
      _drag.lastSpeed_mps = v_curr;

      // 2) Distance from haversine between start position and current position
      _drag.distance_haversine_m = haversineDistance_m(_drag.startLat_deg, _drag.startLon_deg,
                                                      _pose.lat_deg, _pose.lon_deg);

      // 0-60 mph timing
      if( !_drag.has_0_60 && (speed_mps >= TARGET_60_MPS) )
      {
        _drag.has_0_60 = true;
        _drag.t_0_60_sec = _pose.time_sec - _drag.startTime_sec;
        _drag.v_0_60_mps = speed_mps;

        Serial.print("DRAG: 0-60 mph in ");
        Serial.print(_drag.t_0_60_sec, 3);
        Serial.print(" s, speed = ");
        Serial.print(speed_mph, 2);
        Serial.println(" mph.");
      }

      // 1/4 mile (check both distance methods)
      bool reachedInt = (_drag.distance_integrated_m  >= QUARTER_MILE_M);
      bool reachedHav = (_drag.distance_haversine_m   >= QUARTER_MILE_M);

      if( !_drag.has_quarter && (reachedInt || reachedHav) )
      {
        _drag.has_quarter = true;

        // For now, use current time as both ETs.
        // If you want more precision, you can interpolate back to the exact crossing.
        _drag.t_quarter_int_sec = _pose.time_sec - _drag.startTime_sec;
        _drag.t_quarter_hav_sec = _drag.t_quarter_int_sec;

        _drag.v_quarter_mps = speed_mps;

        _drag.state = DRAG_FINISHED;

        printDragRunSummary();
      }

      // Abort condition: if we slow down below stop threshold for a while before 1/4 mile
      if( !_drag.has_quarter )
      {
        if( updateStopTimer(ABORT_STOP_TIME_MS) )
        {
          Serial.println("DRAG: Run aborted (vehicle stopped/slowed before 1/4 mile).");
          resetDragState();
        }
      }
      break;
    }

    case DRAG_FINISHED:
    {
      // After a finished run, wait for the car to come to a stop, then auto-reset
      if( updateStopTimer(MIN_STOP_TIME_MS) )
      {
        Serial.println("DRAG: Finished run, vehicle stopped. Ready for next run.");
        resetDragState();
      }
      break;
    }

    default:
      resetDragState();
      break;
  }
}



//============================================================================================
// GPS dead-reckoning state (speed + accel hold)
//============================================================================================
struct GpsDeadReck
{
  bool hasLast = false;
  bool hasPrev = false;

  uint32_t timestamp_ms = 0;
  uint32_t timestamp_ms_prev = 0;

  float spd_mps = 0.0f;
  float spd_mps_prev = 0.0f;

  float accel_mps2_calc = 0.0f;  // acceleration computed from last two GPS SOG updates

  // Optional debug
  float last_dt_sec = 0.0f;
} _gpsDR;



//============================================================================================
//============================================================================================
//                                        SETUP
//============================================================================================
//============================================================================================
KalmanFilter _kf;
float _kfRate_hz = 20.0;
float _kfRate_ms = 1.0 / _kfRate_hz;

unsigned long _timeStart_ms = 0;

void setup() 
{   
  pinMode(_ledPin, _ledOn);

  // Configure the USB Serial port
  Serial.begin(115200);

  // Configure GPS interface
  Serial.println("Setting Up GPS");
  setupGPS();
  Serial.println("Completed GPS Setup");

  // Kalman Filter Init
  _kf.init(_kfRate_ms);
  
  _timeStart_ms = millis();
}


//============================================================================================
//============================================================================================
//                                        LOOP
//============================================================================================
//============================================================================================
unsigned long _blinkPeriod_ms   = 1000; // 1Hz
unsigned long _processPeriod_ms = 50;  // 20Hz (IMU can go up to 100Hz)
unsigned long _printPeriod_ms   = 200;  // 5Hz

uint32_t _time_ms       = millis();  // number of milliseconds since the program started
uint32_t _timeLED_ms    = millis();  // 
uint32_t _timeProcess_ms = millis();
uint32_t _timePrint_ms  = millis();

bool DEBUG = true;

// This will loop as fast as it can. Timer periods set above will manage when diffent functionality runs.
void loop() 
{ 
  // Elapsed Time for each loop (each loop takes around 1-2ms to execute)
  unsigned long etime_ms  = millis() - _time_ms;
  float etime_sec         = 0.001f * float(etime_ms);
  float loopRate_Hz       = 1.0f / etime_sec;

  // get the current time in msec for this loop
  _time_ms = millis();
  

  //------------------------------------------------------------
  // Parse GPS continuously
  int gpsRes = readGPS(); // NA=0, FAIL=1, RMC=2, GGA=3, OTHER=4

  //------------------------------------------------------------
  // Update accel-hold model on new RMC message (RMC has lat, lon, and SOG)
  if( gpsRes == 2 && GPS.fix )
  {
    // new GPS speed and timestamp
    float spd_mps = GPS.speed * KNTS2MPS;
    uint32_t t_new_ms = _time_ms;

    if( !_gpsDR.hasLast )
    {
      _gpsDR.hasLast   = true;
      _gpsDR.timestamp_ms = t_new_ms;
      _gpsDR.spd_mps = spd_mps;

      // no accel estimate yet
      _gpsDR.accel_mps2_calc = 0.0f;
    }
    else
    {
      // Shift last -> prev
      _gpsDR.hasPrev   = true;
      _gpsDR.timestamp_ms_prev = _gpsDR.timestamp_ms;
      _gpsDR.spd_mps_prev = _gpsDR.spd_mps;

      // Store new as last
      _gpsDR.timestamp_ms = t_new_ms;
      _gpsDR.spd_mps = spd_mps;

      // Compute accel from two most recent GPS speeds
      float dt_sec = 0.001f * float(_gpsDR.timestamp_ms - _gpsDR.timestamp_ms_prev);
      _gpsDR.last_dt_sec = dt_sec;

      // Guard against weird dt
      if( dt_sec > 0.02f && dt_sec < 0.5f )
      {
        _gpsDR.accel_mps2_calc = (_gpsDR.spd_mps - _gpsDR.spd_mps_prev) / dt_sec;
      }
      else
      {
        _gpsDR.accel_mps2_calc = 0.0f;
      }
    }

    // Phase-lock 20 Hz processing to RMC arrival.
    // This encourages a 50ms "midpoint" predict tick after each GPS update.
    _timeProcess_ms = t_new_ms;
  }

  //------------------------------------------------------------
  // PROCESS (20 Hz)
  if( _time_ms - _timeProcess_ms > _processPeriod_ms )
  {
    //uint32_t dt_ms = _time_ms - _timeProcess_ms;
    _timeProcess_ms  = _time_ms;
    //float dt_sec   = 0.001f * float(dt_ms);

    //--------------------------------------------------------
    // predicted speed between GPS updates
    float v_pred_mps = GPS.speed * KNTS2MPS; // fallback to raw GPS speed

    if( _gpsDR.hasLast )
    {
      float dt_gps_sec = 0.001f * float(_time_ms - _gpsDR.timestamp_ms);
      v_pred_mps = _gpsDR.spd_mps + _gpsDR.accel_mps2_calc * dt_gps_sec;

      if( v_pred_mps < 0.0f ) v_pred_mps = 0.0f;
    }

    //--------------------------------------------------------
    // Pose fields (use predicted speed)
    _pose.time_sec    = 0.001f * float(_time_ms - _timeStart_ms);
    _pose.lat_deg     = GPS.latitudeDegrees;
    _pose.lon_deg     = GPS.longitudeDegrees;
    _pose.speed_mps   = v_pred_mps;                 // *** Tier A speed ***
    _pose.heading_deg = angleWrap_deg(GPS.angle);
    _pose.numSat      = GPS.satellites;

    // updateDragTiming(dt_sec);
  }

  //------------------------------------------------------------
  // Print data
  if( _time_ms - _timePrint_ms > _printPeriod_ms ) 
  {
    _timePrint_ms = _time_ms;

    if( DEBUG )
    {
      Serial.println("\n-------------------------------------------------");
      Serial.print("etime_ms: "); Serial.print(etime_ms);
      Serial.print(" etime_sec: "); Serial.print(etime_sec,5);
      Serial.print(" loopRate_Hz: "); Serial.println(loopRate_Hz);
      Serial.println("");

      printGPS();

      // DR debug
      Serial.print("DeadReck: spd_mps="); Serial.print(_gpsDR.spd_mps, 3);
      Serial.print(" accel_mps2_calc="); Serial.print(_gpsDR.accel_mps2_calc, 3);
      Serial.print(" last_dt_sec="); Serial.print(_gpsDR.last_dt_sec, 3);
      Serial.println("");
    }

    _pose.printCSV();
  }

  //------------------------------------------------------------
  // Blink LED
  if( _time_ms - _timeLED_ms > _blinkPeriod_ms ) 
  {
    _timeLED_ms = _time_ms;
    _ledOn = !_ledOn;
    digitalWrite(_ledPin, _ledOn);
  }

  // no delay because we need to run readGPS as fast as possible to parse NMEA sentences!
}
