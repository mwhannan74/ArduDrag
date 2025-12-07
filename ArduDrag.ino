// GPS
#include <Adafruit_GPS.h>
#include <vector>
#include <string>

// BNO055 Orient Sensor
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


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

unsigned long timeStart_ms = 0;


//============================================================================================
// Onboard LED
//============================================================================================
static const int ledPin = 13;
bool ledOn = true;


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
// Adafruit BNO055 IMU Orient Sensor
//============================================================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55);
struct
{
  bool isCalibrated = false;
  float hdg_mag = 0;
  float hdg_true = 0;
  float pitch = 0;
  float roll = 0;
  float declination = -10.8186;
} IMU;

// Display sensor calibration status if using full 9 dof with magnetometer
bool displayImuCalibStatus(void)
{  
  uint8_t sys, gyro, accel, mag;
  sys = gyro = accel = mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("IMU Calibartion: Move the sensor around! ");
  Serial.print("Sys = "); Serial.print(sys, DEC);
  Serial.print(" G = "); Serial.print(gyro, DEC);
  Serial.print(" A = "); Serial.print(accel, DEC);
  Serial.print(" M = "); Serial.println(mag, DEC);

  if( ((gyro+accel+mag) == 9) && (sys == 3)) return true;
  else return false;
}

// Display sensor calibration status if only using 6 dof without magnetometer
bool displayImuCalibStatus_IMUOnly(void)
{  
  uint8_t sys, gyro, accel, mag;
  sys = gyro = accel = mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("IMU Calibartion: ");
  Serial.print("Sys = "); Serial.print(sys, DEC);
  Serial.print(" G = "); Serial.print(gyro, DEC);
  Serial.print(" A = "); Serial.print(accel, DEC);
  Serial.print(" M = "); Serial.println(mag, DEC);

  // In IMUPLUS, magnetometer is not part of the solution.
  // Treat fully calibrated as gyro=3 and accel=3.
  if( gyro == 3 ) return true;
  else return false;
}

// Display the raw calibration offset and radius data
void displayImuCalibOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.println("\nBNO055 Calibration Data");
    Serial.print("calibData.accel_offset_x = "); Serial.print(calibData.accel_offset_x); Serial.println(";");
    Serial.print("calibData.accel_offset_y = "); Serial.print(calibData.accel_offset_y); Serial.println(";");
    Serial.print("calibData.accel_offset_z = "); Serial.print(calibData.accel_offset_z); Serial.println(";");
    Serial.print("calibData.accel_radius = "); Serial.print(calibData.accel_radius); Serial.println(";");

    Serial.print("calibData.gyro_offset_x = "); Serial.print(calibData.gyro_offset_x); Serial.println(";");
    Serial.print("calibData.gyro_offset_y = "); Serial.print(calibData.gyro_offset_y); Serial.println(";");
    Serial.print("calibData.gyro_offset_z = "); Serial.print(calibData.gyro_offset_z); Serial.println(";");
    Serial.print("calibData.mag_offset_x = "); Serial.print(calibData.mag_offset_x); Serial.println(";");
    Serial.print("calibData.mag_offset_y = "); Serial.print(calibData.mag_offset_y); Serial.println(";");
    Serial.print("calibData.mag_offset_z = "); Serial.print(calibData.mag_offset_z); Serial.println(";");
    Serial.print("calibData.mag_radius = "); Serial.print(calibData.mag_radius); Serial.println(";");

    Serial.println("");
}

// This will eliminate the need to calibrate the accelerometers!!!
// The magnetometers will still need to be calibrated
// The gyros still require the sensor to be motionless at start up
bool loadImuCalibration()
{
  adafruit_bno055_offsets_t calibData;
  bno.getSensorOffsets( calibData );
  displayImuCalibOffsets( calibData );
  
  Serial.println("Restoring calibration data to the BNO055");  
  calibData.accel_offset_x = 16;
  calibData.accel_offset_y = -23;
  calibData.accel_offset_z = -5;
  calibData.accel_radius = 1000;
  
  calibData.gyro_offset_x = 1;
  calibData.gyro_offset_y = -2;
  calibData.gyro_offset_z = 1;
  
  calibData.mag_offset_x = -122;
  calibData.mag_offset_y = 424;
  calibData.mag_offset_z = -163; 
  calibData.mag_radius = 778;  
  
  bno.setSensorOffsets(calibData);
  displayImuCalibOffsets( calibData );

  return true;
}

// Setup the IMU mode
void setupIMU()
{
  // Initialize the BNO055 sensor 
  if(!bno.begin())  
  {
    Serial.println("ERROR, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1)
    {
      delay(100);
    }
  }  
  delay(500);    

  // use high resolution crystal oscillator for its clock
  bno.setExtCrystalUse(true); 
  delay(50);

  // disable the magnetometer
  bno.setMode(OPERATION_MODE_IMUPLUS);
  delay(50);

  //*****************************************************************************************
  // For a fast calibration, uncomment this line and use the most recently saved calibration.
  loadImuCalibration();
  //*****************************************************************************************

  while( !IMU.isCalibrated )
  {
    //IMU.isCalibrated = displayImuCalibStatus();      
    IMU.isCalibrated = displayImuCalibStatus_IMUOnly();      
  }

  adafruit_bno055_offsets_t calibData; // struct to store data
  bno.getSensorOffsets( calibData );   // get offsets
  displayImuCalibOffsets( calibData ); // display them
}


// Data is always available on a call to read since the library directly requests it
void readIMU()
{
  // Possible vector values:
  // VECTOR_EULER -> degrees (100Hz)
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // P1 (default)
  IMU.hdg_mag = euler.x() + 90.0; // CW is positive; seems to be off 90 deg
  IMU.pitch   = euler.y();        // up is positive
  IMU.roll    = euler.z();        // CCW (left) is positive

  // angle wrap
  IMU.hdg_mag = angleWrap_deg(IMU.hdg_mag);
  IMU.pitch   = angleWrap_deg(IMU.pitch);
  IMU.roll    = angleWrap_deg(IMU.roll);

  // declination adjustment
  IMU.hdg_true = IMU.hdg_mag - IMU.declination;
  IMU.hdg_true = angleWrap_deg(IMU.hdg_true);
}

void printIMU()
{
  Serial.print("IMU: hdg_mag "); Serial.print(IMU.hdg_mag, 4);
  Serial.print(" hdg_true "); Serial.print(IMU.hdg_true, 4);
  Serial.print(" pitch "); Serial.print(IMU.pitch, 4);
  Serial.print(" roll "); Serial.print(IMU.roll, 4);
  Serial.println("");
}



//============================================================================================
// POSE DATA (GPS + IMU) - primary logging structure
//============================================================================================
struct
{
  float time_sec = 0;
  unsigned int numSat = 0;
  float lat_deg = 0;
  float lon_deg = 0;
  float speed_mps = 0;
  float heading_deg = 0;
  float pitch_deg = 0;
  float roll_deg = 0;
  

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
    Serial.print(heading_deg); Serial.print(", ");
    Serial.print(pitch_deg); Serial.print(", ");    
    Serial.print(roll_deg);
    Serial.println("");
  }
} pose;


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

} drag;


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
  drag.state = DRAG_WAIT_FOR_STOP;

  drag.startTime_sec = 0.0f;
  drag.distance_integrated_m = 0.0f;
  drag.distance_haversine_m = 0.0f;
  drag.startLat_deg = 0.0f;
  drag.startLon_deg = 0.0f;
  drag.lastSpeed_mps = 0.0f;
  drag.maxSpeed_mps = 0.0f;

  drag.has_0_60 = false;
  drag.t_0_60_sec = 0.0f;
  drag.v_0_60_mps = 0.0f;

  drag.has_quarter = false;
  drag.t_quarter_int_sec = 0.0f;
  drag.t_quarter_hav_sec = 0.0f;
  drag.v_quarter_mps = 0.0f;

  drag.stopStart_ms = 0;
  drag.stopTimerRunning = false;

  Serial.println("DRAG: Reset, waiting for stop.");
}

// Print final run summary in a single line for easy parsing
void printDragRunSummary()
{
  Serial.println("DRAG: Run complete.");
  Serial.print("DRAG_RESULT, ");
  Serial.print("t_start=");           Serial.print(drag.startTime_sec, 3);
  Serial.print(", t_0_60=");          Serial.print(drag.has_0_60 ? drag.t_0_60_sec : -1.0f, 3);
  Serial.print(", t_quarter_int=");   Serial.print(drag.t_quarter_int_sec, 3);
  Serial.print(", t_quarter_hav=");   Serial.print(drag.t_quarter_hav_sec, 3);
  Serial.print(", v_0_60_mph=");
  Serial.print(drag.has_0_60 ? drag.v_0_60_mps * MPS2MPH : 0.0f, 2);
  Serial.print(", v_quarter_mph=");
  Serial.print(drag.v_quarter_mps * MPS2MPH, 2);
  Serial.print(", dist_int_m=");
  Serial.print(drag.distance_integrated_m, 2);
  Serial.print(", dist_hav_m=");
  Serial.print(drag.distance_haversine_m, 2);
  Serial.print(", maxSpeed_mph=");
  Serial.print(drag.maxSpeed_mps * MPS2MPH, 2);
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
    if( drag.state == DRAG_RUNNING )
    {
      Serial.println("DRAG: GPS fix lost, aborting run.");
      resetDragState();
    }
    return;
  }

  float speed_mps = pose.speed_mps;
  float speed_mph = speed_mps * MPS2MPH;

  // Generic stop-detection helper (used in several states)
  auto updateStopTimer = [&](uint32_t timeout_ms) -> bool
  {
    if( speed_mps < SPEED_STOP_THRESHOLD_MPS )
    {
      if( !drag.stopTimerRunning )
      {
        drag.stopTimerRunning = true;
        drag.stopStart_ms = millis();
      }
      else
      {
        if( millis() - drag.stopStart_ms > timeout_ms )
        {
          drag.stopTimerRunning = false;
          return true; // timeout reached
        }
      }
    }
    else
    {
      drag.stopTimerRunning = false;
    }
    return false;
  };

  switch( drag.state )
  {
    case DRAG_WAIT_FOR_STOP:
    {
      // Wait until the vehicle is stopped for a bit with a valid fix
      if( updateStopTimer(MIN_STOP_TIME_MS) )
      {
        drag.state = DRAG_ARMED;
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
        drag.state = DRAG_RUNNING;

        drag.startTime_sec = pose.time_sec;
        drag.startLat_deg  = pose.lat_deg;
        drag.startLon_deg  = pose.lon_deg;
        drag.lastSpeed_mps = speed_mps;
        drag.maxSpeed_mps  = speed_mps;

        drag.distance_integrated_m = 0.0f;
        drag.distance_haversine_m = 0.0f;

        drag.has_0_60 = false;
        drag.has_quarter = false;

        drag.stopTimerRunning = false;

        Serial.print("DRAG: Run started at t = ");
        Serial.print(drag.startTime_sec, 3);
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
      if( speed_mps > drag.maxSpeed_mps )
      {
        drag.maxSpeed_mps = speed_mps;
      }

      // 1) Distance from integrated speed (trapezoidal integration)
      float v_prev = drag.lastSpeed_mps;
      float v_curr = speed_mps;
      drag.distance_integrated_m += 0.5f * (v_prev + v_curr) * dt_sec;
      drag.lastSpeed_mps = v_curr;

      // 2) Distance from haversine between start position and current position
      drag.distance_haversine_m = haversineDistance_m(drag.startLat_deg, drag.startLon_deg,
                                                      pose.lat_deg, pose.lon_deg);

      // 0-60 mph timing
      if( !drag.has_0_60 && (speed_mps >= TARGET_60_MPS) )
      {
        drag.has_0_60 = true;
        drag.t_0_60_sec = pose.time_sec - drag.startTime_sec;
        drag.v_0_60_mps = speed_mps;

        Serial.print("DRAG: 0-60 mph in ");
        Serial.print(drag.t_0_60_sec, 3);
        Serial.print(" s, speed = ");
        Serial.print(speed_mph, 2);
        Serial.println(" mph.");
      }

      // 1/4 mile (check both distance methods)
      bool reachedInt = (drag.distance_integrated_m  >= QUARTER_MILE_M);
      bool reachedHav = (drag.distance_haversine_m   >= QUARTER_MILE_M);

      if( !drag.has_quarter && (reachedInt || reachedHav) )
      {
        drag.has_quarter = true;

        // For now, use current time as both ETs.
        // If you want more precision, you can interpolate back to the exact crossing.
        drag.t_quarter_int_sec = pose.time_sec - drag.startTime_sec;
        drag.t_quarter_hav_sec = drag.t_quarter_int_sec;

        drag.v_quarter_mps = speed_mps;

        drag.state = DRAG_FINISHED;

        printDragRunSummary();
      }

      // Abort condition: if we slow down below stop threshold for a while before 1/4 mile
      if( !drag.has_quarter )
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
//============================================================================================
//                                        SETUP
//============================================================================================
//============================================================================================
void setup() 
{   
  pinMode(ledPin, ledOn);

  // Configure the USB Serial port
  Serial.begin(115200);

  Serial.println("Setting Up IMU");
  setupIMU();
  Serial.println("Completed IMU Setup");

  Serial.println("Setting Up GPS");
  setupGPS();
  Serial.println("Completed GPS Setup");
  
  timeStart_ms = millis();
}


//============================================================================================
//============================================================================================
//                                        LOOP
//============================================================================================
//============================================================================================
unsigned long blinkPeriod_ms  = 1000; // 1Hz
unsigned long processPeriod_ms = 50;  // 20Hz (IMU can go up to 100Hz)
unsigned long printPeriod_ms  = 200;  // 5Hz

uint32_t time_ms       = millis();
uint32_t timeLED_ms    = millis();
uint32_t timeProcess_ms = millis();
uint32_t timePrint_ms  = millis();

bool DEBUG = true;

// This will loop as fast as it can. Timer periods set above will manage when diffent functionality runs.
void loop() 
{  
  // Each loop takes around 1-2ms to execute
  unsigned long etime_ms  = millis() - time_ms;
  float etime_sec         = 0.001f * float(etime_ms);
  float loopRate_Hz       = 1.0f / etime_sec;

  // get the current time in msec for this loop
  time_ms = millis();
  

  //------------------------------------------------------------
  // Get GPS data from serial port on every loop and attempt to parse it (parsing as fast as the Teensy can loop!)
  // RMC = lat/lon/sog/cog/magVar
  // GGA = lat/lon/alt/qual/sats
  int gpsRes = readGPS(); // NA=0, FAIL=1, RMC=2, GGA=3, OTHER=4
  if( gpsRes == 2)
  {
    //if(DEBUG) Serial.println("GPS: Recieved RMC message (lat/lon/sog/cog/magVar)");
  }
  else if( gpsRes == 3 )
  {
    //if(DEBUG) Serial.println("GPS: Recieved GGA message (lat/lon/alt/qual/sats)"); 
  }

  //------------------------------------------------------------
  // PROCESS the IMU and GPS data
  if( time_ms - timeProcess_ms > processPeriod_ms )
  {
    uint32_t dt_ms = time_ms - timeProcess_ms;
    timeProcess_ms  = time_ms;
    float dt_sec   = 0.001f * float(dt_ms);

    // Get IMU data from I2C port (always ready, unlike the GPS)
    readIMU();
  
    // Pose fields
    pose.time_sec    = 0.001f * float(time_ms - timeStart_ms);
    pose.lat_deg     = GPS.latitudeDegrees;
    pose.lon_deg     = GPS.longitudeDegrees;
    pose.speed_mps   = GPS.speed * KNTS2MPS;
    pose.roll_deg    = IMU.roll;
    pose.pitch_deg   = IMU.pitch;    
    pose.heading_deg = angleWrap_deg(GPS.angle); //COG --> alternative IMU.hdg_true;
    pose.numSat      = GPS.satellites;
 
    // Update drag timing state machine
    //updateDragTiming(dt_sec);
  }


  //------------------------------------------------------------
  // Print data
  if( time_ms - timePrint_ms > printPeriod_ms ) 
  {
    timePrint_ms = time_ms;
   
    if( DEBUG )
    {
      Serial.println("\n-------------------------------------------------");
      Serial.print("etime_ms: ");   Serial.print(etime_ms);
      Serial.print(" etime_sec: "); Serial.print(etime_sec,5);
      Serial.print(" loopRate_Hz: "); Serial.println(loopRate_Hz);
      Serial.println("");  
  
      printGPS();
      printIMU();  
    }

    // Primary pose CSV output for logging
    pose.printCSV();    
  }


  //------------------------------------------------------------
  // Blink LED
  if( time_ms - timeLED_ms > blinkPeriod_ms ) 
  {
    timeLED_ms = time_ms;
    ledOn = !ledOn;
    digitalWrite(ledPin, ledOn);
  }

  // no delay because we need to run readGPS as fast as possible to parse NMEA sentences!
}
