#include <Adafruit_GPS.h>
#include "KalmanFilter.h"
#include "DragFSM.h"
#include "GpsHelper.h"

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
static uint8_t gps_rx_buffer[512];

Adafruit_GPS GPS(&SerialGPS);
const char* GpsFixQuality(uint8_t q)
{
  switch (q)
  {
    case 1: return "SPS";
    case 2: return "DGPS";
    case 3: return "PPS";
    case 4: return "RTK";
    case 5: return "fRTK";
    case 6: return "dead reck";
    case 7: return "manual";
    case 8: return "sim";
    default: return "invalid";
  }
}

void setupGPS()
{
  Serial.println("=======================================");
  Serial.println("Setting Up GPS");

  // increase buffer size on Teensy 3.2 so occasional USB serial stalls don’t cost bytes
  Serial1.addMemoryForRead(gps_rx_buffer, sizeof(gps_rx_buffer));

  // UART bring-up / baud switch
  SerialGPS.begin(9600);
  delay(200);

  GPS.sendCommand(PMTK_SET_BAUD_57600);
  delay(150);           // give GPS a moment to switch
  SerialGPS.end();
  delay(20);
  SerialGPS.begin(57600);
  delay(50);

  // IMPORTANT: do not print per-character during ACK waits
  const bool ECHO = false;
  const bool READABLE = true; // irrelevant when ECHO=false
  const uint32_t TMO = 800;

  int flag314=-1, flag869=-1, flag886=-1;

  // Quiet stream: disable all NMEA sentences (PMTK314 all zeros)
  // This is more deterministic than relying on library macros.
  GpsHelper::flushGPS(SerialGPS);
  GPS.sendCommand("$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  bool ok314 = GpsHelper::waitForPMTKAck(Serial, SerialGPS, 314, TMO, &flag314, ECHO, READABLE);

  // Disable EASY
  GpsHelper::flushGPS(SerialGPS);
  GPS.sendCommand("$PMTK869,1,0*34");
  bool ok869 = GpsHelper::waitForPMTKAck(Serial, SerialGPS, 869, TMO, &flag869, ECHO, READABLE);

  // Avionic
  GpsHelper::flushGPS(SerialGPS);
  GPS.sendCommand("$PMTK886,2*2A");
  bool ok886 = GpsHelper::waitForPMTKAck(Serial, SerialGPS, 886, TMO, &flag886, ECHO, READABLE);

  // Re-enable desired output + rate
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  // Print once at the end (safe)
  Serial.println("Completed GPS Setup");
  Serial.println("--- GPS SETUP SUMMARY ---");
  Serial.printf("PMTK314 quiet stream ok=%d flag=%d\n", ok314, flag314);
  Serial.printf("PMTK869 EASY disable ok=%d flag=%d\n", ok869, flag869);
  Serial.printf("PMTK886 avionic ok=%d flag=%d\n", ok886, flag886);
  Serial.println("=======================================");
}


// void setupGPS()
// {
//   Serial.println("=======================================");
//   Serial.println("Setting Up GPS");  

//   //------------------------------------------------------------------------  
//   // defualt GPS baud rate is 9600
//   SerialGPS.begin(9600);
//   delay(200);

//   // Change GPS to 57600 baud rate for robustness using 2 NMEA messages.
//   GPS.sendCommand(PMTK_SET_BAUD_57600);
//   SerialGPS.end();
//   delay(50);  
//   SerialGPS.begin(57600);
//   delay(50);

//   // Disable all output to quiet the stream output
//   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_OFF);
//   delay(50);

//   // Ensure we don't match a stale ACK
//   //GpsHelper::flushGPS(SerialGPS);


//   //------------------------------------------------------------------------
//   // FIX ACCELERATION ISSUE
//   // disable EASY
//   GPS.sendCommand("$PMTK869,1,0*34");
//   int flag869 = -1;
//   bool success869 = GpsHelper::waitForPMTKAck(Serial, SerialGPS, 869, 500, &flag869, false, true);
//   Serial.print("PMTK869 (EASY) success = "); Serial.println(success869);
//   if( !success869 ) {Serial.print("PMTK869 flag = "); Serial.println(flag869);} // 0 = invalid command, 1 = unsupported command, 2 = valid command, but failed, 3 = valid command, succeeded

//   // avionic (high dynamic)
//   GPS.sendCommand("$PMTK886,2*2A");
//   int flag886 = -1;
//   bool success886 = GpsHelper::waitForPMTKAck(Serial, SerialGPS, 886, 500, &flag886, true, true);
//   Serial.print("PMTK886 (avionic) success = "); Serial.println(success886);
//   if( !success886 ) {Serial.print("PMTK886 flag = "); Serial.println(flag886);}


//   //------------------------------------------------------------------------
//   // Always request RMC + GGA (RMC = lat/lon/sog/cog/magVar, GGA = lat/lon/alt/qual/sats)
//   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//   delay(50);
 
//   // Set the update rate to 10Hz
//   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
//   delay(50);

//   Serial.println("Completed GPS Setup");
//   Serial.println("=======================================");
// }

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
  // >= 10: Excelent
  // 8–9: Good
  // 6–7: Marginal
  // 5: Bad
  // 4: Unreliable -> don't use
  Serial.print("GPS: satellites "); Serial.print(GPS.satellites);
  
  Serial.print(" Fix=");
    if( (int)GPS.fix ) Serial.print("True");
    else Serial.print("False");

  // 0 = invalid
  // 1 = SPS 
  // 2 = DGPS
  // 4 = RTK fixed
  // 5 = RTK float
  Serial.print(" Fix_quality="); Serial.print( GpsFixQuality((int)GPS.fixquality) );

  // < 1.2:   Excellent -> near-centimeter accuracy (ideal for high-precision tasks).
  // 1.2-2.0: Good
  // 2.0-3.0: Marginal
  // > 3.0:   Bad -> should reject position updates
  // > 4:     Unreliable -> don't use
  Serial.print(" HDOP "); Serial.print(GPS.HDOP);

  // Does the GPS have a "Fix"?
  if( GPS.fix ) 
  {
    Serial.print(" Position ");
    Serial.print(GPS.latitudeDegrees,11); Serial.print(", ");     
    Serial.print(GPS.longitudeDegrees,11);
    
    //Serial.print(" Altitude MSL (m) "); Serial.print(GPS.altitude);
    //Serial.print(" Geoid Height (m) "); Serial.print(GPS.geoidheight);

    Serial.print(" SOG(mph) "); Serial.print(GPS.speed*KNTS2MPH);
    //Serial.print(" COG(deg) "); Serial.print(GPS.angle);
    //Serial.print(" MagVar "); Serial.print((int)GPS.magvariation);
    
    Serial.println("");
  }
  else
  {
    Serial.println("");
  }
}


//============================================================================================
// GPS HELPER FUNCTIONS
//============================================================================================
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

  float a = sin_dphi_2 * sin_dphi_2 + cos(phi1) * cos(phi2) * sin_dlambda_2 * sin_dlambda_2;

  float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
  float d = R * c;

  return d;
}

// Position std dev (meters) based on GPS fix quality
float baseSigmaFromFix(int fixQuality)
{
    // Common NMEA GGA quality encodings:
    // 0=invalid, 1=GPS(SPS), 2=DGPS, 4=RTK fixed, 5=RTK float
    switch (fixQuality) 
    {
        case 4:  return 0.05f;  // RTK fixed
        case 5:  return 0.25f;  // RTK float
        case 2:  return 0.8f;   // DGPS/SBAS
        case 1:  return 2.0f;   // SPS
        default: return 50.0f;  // invalid/unknown
    }
}

// State estimation covariance P std dev based on fix quality, num of sats, and hdop.
float computeSigmaP(int fixQuality, int sats, float hdop)
{
    float base = baseSigmaFromFix(fixQuality);

    // If no fix, blow up uncertainty
    if (fixQuality == 0) return 50.0f;

    // Clamp inputs to sane bounds
    if (sats < 0) sats = 0;
    if (hdop < 0.5f) hdop = 0.5f;
    if (hdop > 5.0f) hdop = 5.0f;

    // Satellite factor: more sats => lower sigma
    // 8 sats ~ nominal; fewer increases sigma modestly.
    float satFactor = 1.0f;
    if (sats > 0) 
    {
        satFactor = 8.0f / (float)sats;
        if (satFactor < 0.7f) satFactor = 0.7f;
        if (satFactor > 2.5f) satFactor = 2.5f;
    }

    // HDOP factor directly scales uncertainty
    float hdopFactor = hdop; // 1.0 good, 2.0 mediocre, 3-5 poor

    return base * satFactor * hdopFactor;
}

// Min measureable speed from GPS based on number of satellites (meters/second)
// 0.5 mps = 1.12 mph
float minSpeedBasedonSatellites(unsigned int numSat)
{
  if( numSat <= 6)                         return 1.2f;  // 1.2mps = 2.7 mph
  else if( (numSat > 6) && (numSat <= 9) ) return 0.8f;  // 0.8mps = 1.8 mph
  else                                     return 0.35f; // 0.35mps = 0.78mph
}


//============================================================================================
//============================================================================================
//                                        SETUP
//============================================================================================
//============================================================================================
void setup() 
{   
  // Configure DIO for LED
  pinMode(_ledPin, _ledOn);

  // Configure the USB Serial port
  Serial.begin(115200);

  // Configure GPS interface
  setupGPS();
}


//============================================================================================
//============================================================================================
//                                        LOOP
//============================================================================================
//============================================================================================
float _processRate_hz = 20.0f;
float _processPeriod_sec = 1.0f / _processRate_hz;
unsigned long _processPeriod_ms = 1000.0f * _processPeriod_sec;

unsigned long _blinkPeriod_ms = 1000; // 1Hz
unsigned long _printPeriod_ms = 2000; // 1Hz

uint32_t _time_ms        = millis();  // number of milliseconds since the program started
uint32_t _timeLED_ms     = millis(); 
uint32_t _timeProcess_ms = millis();
uint32_t _timeKF_ms      = millis();
uint32_t _timePrint_ms   = millis();
uint32_t _time_RMC_ms    = millis();

bool _isFirstTime = true;
bool _haveGpsFixInfo = false;
bool _enableKF = false;
bool _gpsDataGood = false;

double _lat_prev;
double _lon_prev;

float gpsSpdMax_mps = 0.0f; // max speed recorded from GPS
float gpsDist_m = 0.0f;
float gpsSpd_mps = 0.0f;
unsigned int minSatellites = 6;

DragFSM _dragFSM(&Serial);

bool DEBUG = false;
bool TEST = false;
uint32_t _testCnt = 0;
float _testDist = 0.0f;

//------------------------------------------------------------
KalmanFilter _kf;

float _kfRate_sec = _processPeriod_sec;

float sigma_state_p0 = 2.0f;             // (static m) initial position STATE uncertanty P (filter reset) -> start large and let KF learn over time
float sigma_p_smoothed = sigma_state_p0; // (dynamic m) position MEASUREMENT uncertainty R of GPS -> updates in real time

float _sigma_state_v0 = 1.0f; // (static m/s) initial speed STATE uncertanty P (filter reset) -> start large and let KF learn over time
float _sigma_v = 0.2f;        // (static m/s) speed MEASUREMENT uncertainty R of GPS -> reduce to trust GPS more

float _sigma_j = 2.0f; // (static m/s^3) jerk noise uncertainty used to compute Q -> (0.4f to 2.0f) where higher makes model more responsive to real world accel changes


//------------------------------------------------------------------------------------------------------------------------
// This will loop as fast as it can. Timer periods set above will manage when different functionality runs.
//------------------------------------------------------------------------------------------------------------------------
void loop() 
{ 
  // Elapsed Time for each loop (each loop takes around 1ms to execute)
  uint32_t etime_ms  = millis() - _time_ms;
  float etime_sec    = 0.001f * float(etime_ms);
  float loopRate_Hz  = 1.0f / etime_sec;

  // get the current time in msec for this loop (must come after etime calc)
  _time_ms = millis();
  
  //------------------------------------------------------------
  // Parse GPS continuously
  int gpsRes = readGPS(); // NA=0, FAIL=1, RMC=2, GGA=3, OTHER=4

  //------------------------------------------------------------  
  // Only run processing if we have a GPS fix
  if( GPS.fix )
  {
    // Did we get sat fix info yet? 
    // gpsRes = 3 = GGA --> satillite information
    if( gpsRes == 3 && !_haveGpsFixInfo )
    {
      _haveGpsFixInfo = true;
    }
  }

  //------------------------------------------------------------  
  // Do nothing unless we have a GPS fix and have recieved GPS satillite info (fix)
  if( GPS.fix && _haveGpsFixInfo )
  {    
    //===============================================================
    // KF update only on new RMC message (RMC has lat, lon, and SOG)
    // Event driven with a rate around 10Hz 
    //===============================================================
    if( gpsRes == 2 )
    {   
      if(DEBUG) Serial.println("=== RMC Message Received ==="); 
      
      // how much time has passed since last RMC message
      uint32_t time_since_last_RMC_ms = _time_ms - _time_RMC_ms;
      float time_since_last_RMC_sec = (float)time_since_last_RMC_ms / 1000.0f;
      float freq_since_last_RMC_sec = 0.0;
      if(time_since_last_RMC_sec > 0.0) freq_since_last_RMC_sec = 1.0f / time_since_last_RMC_sec;

      _time_RMC_ms = _time_ms;
      if(DEBUG) 
      {
        Serial.print("time_since_last_RMC_ms = "); Serial.println(time_since_last_RMC_ms);
        Serial.print("freq_since_last_RMC_sec = "); Serial.println(freq_since_last_RMC_sec);
      }


      //------------------------------------------------------------
      // Check for sufficient quality of GPS data
      if( GPS.satellites < minSatellites )
      {
        _gpsDataGood = false;

        Serial.print("***** Insufficent Satellites! Require at least ");
        Serial.print(minSatellites);
        Serial.print(" but only have ");
        Serial.print(GPS.satellites);
        Serial.print(" -> HDOP "); Serial.print(GPS.HDOP);
        Serial.println(" *****");
      }
      else
      {
        _gpsDataGood = true;
      }


      //------------------------------------------------------------
      // On the first time we receive a GPS message
      if( _isFirstTime )
      {
        _isFirstTime = false;
        _enableKF = true;

        // GPS position
        _lat_prev = GPS.latitudeDegrees;
        _lon_prev = GPS.longitudeDegrees;

        // Kalman Filter Init
        float p0 = 0.0f;
        float v0 = 0.0f;        
        _kf.init(_kfRate_sec, p0, v0, sigma_state_p0, _sigma_state_v0); // Sets inital state uncertainty P

        // Set Jerk and this Model uncertainty Q
        _kf.setSigmaJ(_sigma_j);
      }


      //------------------------------------------------------------
      // Compute pos std dev based on GPS quality
      float sigma_p = computeSigmaP(GPS.fixquality, GPS.satellites, GPS.HDOP);
      if(DEBUG) {Serial.print("sigma_p = "); Serial.println(sigma_p);}

      // lowpass filter sigma to keep filter stable
      const float alpha = 0.2f;
      sigma_p_smoothed = (1.0f - alpha) * sigma_p_smoothed + alpha * sigma_p;
      if(DEBUG) {Serial.print("sigma_p_smoothed = "); Serial.println(sigma_p_smoothed);}
      
      // update measurement uncertainty R
      _kf.setR_measurement(sigma_p_smoothed, _sigma_v);


      //------------------------------------------------------------
      // GPS Speed
      gpsSpd_mps = GPS.speed * KNTS2MPS;      
      if(DEBUG) {Serial.print("gpsSpd_mps = "); Serial.println(gpsSpd_mps);}      

      // max speed we have seen so far
      if( gpsSpd_mps > gpsSpdMax_mps )
      {
        gpsSpdMax_mps = gpsSpd_mps;
      }
      if(DEBUG) {Serial.print("gpsSpdMax_mps = "); Serial.println(gpsSpdMax_mps);}

      // minimum GPS speed that is reliable given the number of satellites
      float minGpsSpeed = minSpeedBasedonSatellites(GPS.satellites);
 
      // Update Drag FSM speeds for the different states
      //_dragFSM.setStopThresholdMps( minGpsSpeed + 0.0 );
      //_dragFSM.setStartThresholdMps(minGpsSpeed + 0.1 ); // a little higher to provide some hystersis

      // use a consistant fixed speed threshold, especially for starting to eliminate any roll out issues. DragFSM will compensate for this in timing.
      float speedThresh_mph = 2.2;
      _dragFSM.setStopThresholdMps( speedThresh_mph/MPS2MPH - 0.1 );
      _dragFSM.setStartThresholdMps(speedThresh_mph/MPS2MPH + 0.0 ); 
      

      //------------------------------------------------------------
      // Compute change in distance between subsequent GPS measurements
      float gpsDeltaDist_m = haversineDistance_m(_lat_prev, _lon_prev, GPS.latitudeDegrees, GPS.longitudeDegrees);

      // Update previous position every time
      _lat_prev = GPS.latitudeDegrees;
      _lon_prev = GPS.longitudeDegrees;

      // Filter out big GPS jumps (anti-spike)
      const float MAX_STEP_M = 20.0f;
      if (gpsDeltaDist_m >= 0.0f && gpsDeltaDist_m < MAX_STEP_M)
      {
        // Only accumulate distance if we believe we’re actually moving
        if (gpsSpd_mps > minGpsSpeed)
        {
          gpsDist_m += gpsDeltaDist_m;  // monotonic
        }
      } 
      else 
      {
          Serial.print("GPS jump ignored: "); Serial.println(gpsDeltaDist_m);
      }      
      if(DEBUG) {Serial.print("gpsDist_m = "); Serial.println(gpsDist_m);}
      

      //------------------------------------------------------------
      // This section allows for testing the code even if the GPS is stationary
      if( TEST )
      {
        unsigned long t1 = 25;
        //unsigned long t2 = t1 + 50; // 0-60 test
        unsigned long t2 = t1 + 150; // 1/4 mile test
        unsigned long t3 = t2 + 25;
        const float dV = 0.5; 
        float testSpeed = 0.0;
        float dt_sec = 0.1; // 10 Hz
        if (_testCnt < t1)
        {
          testSpeed = 0.0; // stop
        }
        else if (_testCnt < t2)
        {
          // Accelerate
          unsigned long k = _testCnt - t1;
          testSpeed = k * dV;
        }
        else if (_testCnt < t3)
        {
          // Stop
          testSpeed = 0.0;   
        }
        else // _testCnt > t3
        {
          // Done: restart cycle
          _testCnt = 0;
        }        
        _testCnt++;
        //Serial.print("_testCnt = "); Serial.print(_testCnt); 
        //Serial.print("  testSpeed = "); Serial.print(testSpeed);
        //Serial.print("  _testDist = "); Serial.println(_testDist);
                
        _testDist += testSpeed * dt_sec; // integrate
        gpsDist_m = _testDist;
        
        gpsSpd_mps = testSpeed;
      }


      //------------------------------------------------------------
      // Run KF PREDICT if enough time has passed
      float dt = (_time_ms - _timeKF_ms) / 1000.0f;
      if(DEBUG) {Serial.print("dt = "); Serial.println(dt);}
      if( dt > 0.005 ) // ignore if we just did a predict
      {
        _timeKF_ms = _time_ms;
        _kf.predict(dt);
        if(DEBUG) Serial.println("KF Predict");
      }
      else
      {
        if(DEBUG) Serial.println("Skipping KF predict");
      }


      //------------------------------------------------------------
      // Run KF UPDATE to fuse new measurements
      _kf.update(gpsDist_m, gpsSpd_mps);      
      if(DEBUG) Serial.println("KF Update");
    }


    //===============================================================
    //===============================================================
    // PROCESS (20 Hz timer)
    //===============================================================
    //===============================================================
    if( _time_ms - _timeProcess_ms > _processPeriod_ms )
    {
      if(DEBUG) Serial.println("=== Process Loop ==="); 
      _timeProcess_ms  = _time_ms;

      //------------------------------------------------------------
      // Only run if KF has been initialized with the a GPS message
      if( _enableKF )
      {
        //------------------------------------------------------------
        // Kalman Filter Predict
        float dt = (_time_ms - _timeKF_ms) / 1000.0f;
        if(DEBUG) {Serial.print("dt = "); Serial.println(dt);}
        if( dt > 0.005 ) // ignore if we just did a predict
        {      
          _timeKF_ms = _time_ms;
          _kf.predict(dt);
          if(DEBUG) Serial.println("KF Predict"); 
        }
        else
        {
          if(DEBUG) Serial.println("Skipping KF predict");
        }
        float distKF_m = _kf.position();
        float spdKF_mps = _kf.velocity();
        if(DEBUG) {Serial.print("distKF_m = "); Serial.println(distKF_m);}
        if(DEBUG) {Serial.print("spdKF_mps = "); Serial.println(spdKF_mps);}

      
        //--------------------------------------------------------
        // Run the Drag FSM logic
        _dragFSM.update(_time_ms, _gpsDataGood, spdKF_mps, distKF_m, gpsSpd_mps);
        if( _dragFSM.state() == DragFSM::DragState::DRAG_ARMED )
        {
          // Reset state uncertainty to be larger to help with start from zero speed again
          _kf.initP_state(sigma_p_smoothed, _sigma_state_v0);
        }

      } // if( _enableKF )  
    } // if( _time_ms - _timeProcess_ms > _processPeriod_ms )
  } // if( GPS.fix && _haveGpsFixInfo )


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
    }
    printGPS();
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
