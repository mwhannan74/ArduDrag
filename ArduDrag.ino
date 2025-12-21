// GPS
#include <Adafruit_GPS.h>
#include <vector>
#include <string>

#include "KalmanFilter.h"
#include "DragFSM.h"

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
  // Having issues reliably connecting to GPS
  bool initConnectWithDefaultBaud = true;
  if( initConnectWithDefaultBaud )
  {
    // defualt GPS baud rate is 9600
    SerialGPS.begin(9600);
    delay(500);

    // Change GPS to 57600 baud rate for robustness using 2 NMEA messages.
    GPS.sendCommand(PMTK_SET_BAUD_57600);
    SerialGPS.end();
    delay(500);  
  }
  SerialGPS.begin(57600);
  delay(500);

  // SerialGPS.begin(57600);

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
  Serial.print(" Fix_quality="); Serial.print( GpsFixQuality[(int)GPS.fixquality].c_str() ); 

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

  float a = sin_dphi_2 * sin_dphi_2 +
            cos(phi1) * cos(phi2) * sin_dlambda_2 * sin_dlambda_2;

  float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
  float d = R * c;

  return d;
}

float baseSigmaFromFix(int fixQuality)
{
    // Common NMEA GGA quality encodings:
    // 0=invalid, 1=GPS(SPS), 2=DGPS, 4=RTK fixed, 5=RTK float
    switch (fixQuality) {
        case 4:  return 0.05f;  // RTK fixed
        case 5:  return 0.25f;  // RTK float
        case 2:  return 0.8f;   // DGPS/SBAS
        case 1:  return 2.0f;   // SPS
        default: return 50.0f;  // invalid/unknown
    }
}

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
    if (sats > 0) {
        satFactor = 8.0f / (float)sats;
        if (satFactor < 0.7f) satFactor = 0.7f;
        if (satFactor > 2.5f) satFactor = 2.5f;
    }

    // HDOP factor directly scales uncertainty
    float hdopFactor = hdop; // 1.0 good, 2.0 mediocre, 3-5 poor

    return base * satFactor * hdopFactor;
}

// Min measureable speed from GPS based on number of satellites
// 0.5 mps = 1.12 mph
float minSpeedBasedonSatellites(unsigned int numSat)
{
  if( numSat <= 6)      return 1.2f;
  else if( (numSat > 6) && (numSat <= 9) ) return 0.9f;
  else                 return 0.5f;
}


//============================================================================================
//============================================================================================
//                                        SETUP
//============================================================================================
//============================================================================================
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

  _timeStart_ms = millis();
}


//============================================================================================
//============================================================================================
//                                        LOOP
//============================================================================================
//============================================================================================
KalmanFilter _kf;
float _kfRate_hz = 20.0; // this needs to match processPeriod
float _kfRate_sec = 1.0 / _kfRate_hz;

unsigned long _blinkPeriod_ms   = 1000; // 1Hz
unsigned long _processPeriod_ms = 1000.0f * _kfRate_sec;
unsigned long _printPeriod_ms   = 1000;  // 1Hz

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
bool DEBUG2 = false;
bool TEST = false;

uint32_t _counter = 0;


//------------------------------------------------------------
float sigma_state_p0 = 2.0f;             // (static m) initial position STATE uncertanty P (filter reset) -> start large and let KF learn over time
float sigma_p_smoothed = sigma_state_p0; // (dynamic m) position MEASUREMENT uncertainty R of GPS -> updates in real time

float _sigma_state_v0 = 1.0f; // (static m/s) initial speed STATE uncertanty P (filter reset) -> start large and let KF learn over time
float _sigma_v = 0.2f;        // (static m/s) speed MEASUREMENT uncertainty R of GPS -> reduce to trust GPS more

float _sigma_j = 2.0f; // (static m/s^3) jerk noise uncertainty used to compute Q -> (0.4f to 2.0f) where higher makes model more responsive to real world accel changes



//------------------------------------------------------------
// This will loop as fast as it can. Timer periods set above will manage when different functionality runs.
void loop() 
{ 
  // Elapsed Time for each loop (each loop takes around 1-2ms to execute)
  uint32_t etime_ms  = millis() - _time_ms;
  float etime_sec    = 0.001f * float(etime_ms);
  float loopRate_Hz  = 1.0f / etime_sec;

  // get the current time in msec for this loop
  _time_ms = millis();
  
  //------------------------------------------------------------
  // Parse GPS continuously
  int gpsRes = readGPS(); // NA=0, FAIL=1, RMC=2, GGA=3, OTHER=4

  //------------------------------------------------------------  
  // Only run processing if we have a GPS fix
  if( GPS.fix )
  {
    // did we get sat fix info yet?
    if( gpsRes == 3 && !_haveGpsFixInfo )
    {
      _haveGpsFixInfo = true;
    }
  }

  
  if( GPS.fix && _haveGpsFixInfo )
  {    
    //===============================================================
    //===============================================================
    // KF update only on new RMC message (RMC has lat, lon, and SOG)
    // Event driven with a rate around 10Hz 
    //===============================================================
    //===============================================================    
    if( gpsRes == 2 )
    {   
      uint32_t time_gps_process_start_ms =  millis();

      if(DEBUG2) Serial.println("=== RMC Message Received ==="); 
      
      // how much time has passed since last RMC message
      uint32_t time_since_last_RMC_ms = _time_ms - _time_RMC_ms;
      float time_since_last_RMC_sec = (float)time_since_last_RMC_ms / 1000.0f;
      float freq_since_last_RMC_sec = 1.0f / time_since_last_RMC_sec;
      _time_RMC_ms = _time_ms;
      if(DEBUG2) 
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

        // GPS ORIGIN
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
      if(DEBUG2) {Serial.print("sigma_p = "); Serial.println(sigma_p);}

      // lowpass filter sigma to keep filter stable
      const float alpha = 0.2f;
      sigma_p_smoothed = (1.0f - alpha) * sigma_p_smoothed + alpha * sigma_p;
      if(DEBUG2) {Serial.print("sigma_p_smoothed = "); Serial.println(sigma_p_smoothed);}
      
      // update measurement uncertainty R
      _kf.setR_measurement(sigma_p_smoothed, _sigma_v);
      

      //------------------------------------------------------------
      // compute change in linear distance from GPS measurements
      float gpsDeltaDist_m = haversineDistance_m(_lat_prev, _lon_prev, GPS.latitudeDegrees, GPS.longitudeDegrees);
      gpsDist_m += abs(gpsDeltaDist_m); // always increasing
      _lat_prev = GPS.latitudeDegrees;
      _lon_prev = GPS.longitudeDegrees;
      

      //------------------------------------------------------------
      // GPS Speed
      gpsSpd_mps = GPS.speed * KNTS2MPS;
      if(DEBUG2) {Serial.print("gpsDist_m = "); Serial.println(gpsDist_m);}
      if(DEBUG2) {Serial.print("gpsSpd_mps = "); Serial.println(gpsSpd_mps);}

      // max speed we have seen so far
      if( gpsSpd_mps > gpsSpdMax_mps )
      {
        gpsSpdMax_mps = gpsSpd_mps;
      }
      if(DEBUG2) {Serial.print("gpsSpdMax_mps = "); Serial.println(gpsSpdMax_mps);}

      // deadband for practical GPS speed (it never goes to zero with real GPS measurements)
      float minGpsSpeed = minSpeedBasedonSatellites(GPS.satellites);
      // if( gpsSpd_mps < minGpsSpeed )
      // {
      //   gpsSpd_mps = 0.0f;
      // }
      // if(DEBUG2) {Serial.print("gpsSpd_mps = "); Serial.println(gpsSpd_mps);}

      // Update Drag FSM speeds for the different states
      _dragFSM.setStopThresholdMps( minGpsSpeed + 0.1 );
      _dragFSM.setStartThresholdMps(minGpsSpeed + 0.2 );


      //------------------------------------------------------------
      if( TEST )
      {
        unsigned long t1 = 25;
        unsigned long t2 = t1 + 75;
        unsigned long t3 = t2 + 75;
        const float DV = 0.4; 
        float speed;
        if (_counter < t1)
        {
          speed = 0.0; // stop
          gpsDist_m = 0.0;
        }
        else if (_counter < t2)
        {
          // Accelerate
          unsigned long k = _counter - t1;
          speed = k * DV;
          gpsDist_m = k * DV * 0.05;
        }
        else if (_counter < t3)
        {
          // Decelerate
          //unsigned long k = _counter - t2;
          //speed = -k * DV;
          //if (speed < 0.0) speed = 0.0;
          speed = 0.0;   
          gpsDist_m = (t3-t2) * DV * 0.05;
        }
        else // _counter > t3
        {
          // Done: restart cycle
          Serial.print("COUNTER REST");
          _counter = 0;
          speed = 0.0;
        }
        _counter++;
        //Serial.print("_counter = "); Serial.print(_counter); Serial.print("  speed = "); Serial.println(speed);
        gpsSpd_mps += speed;
      }


      //------------------------------------------------------------
      // Run KF PREDICT if enough time has passed
      float dt = (_time_ms - _timeKF_ms) / 1000.0f;
      if(DEBUG2) {Serial.print("dt = "); Serial.println(dt);}
      if( dt > 0.005 ) // ignore if we just did a predict
      {
        _timeKF_ms = _time_ms;
        _kf.predict(dt);
        if(DEBUG2) Serial.println("KF Predict");
      }
      else
      {
        if(DEBUG2) Serial.println("Skipping KF predict");
      }


      //------------------------------------------------------------
      // Run KF UPDATE to fuse new measurements
      _kf.update(gpsDist_m, gpsSpd_mps);      
      if(DEBUG2) Serial.println("KF Update");


      if(DEBUG2)
      {
        uint32_t time_gps_process_stop_ms =  millis();
        uint32_t time_gps_process_total_ms =  time_gps_process_stop_ms - time_gps_process_start_ms;
        Serial.print("time_gps_process_total_ms = "); Serial.println(time_gps_process_total_ms);
      }
    }


    //===============================================================
    //===============================================================
    // PROCESS (20 Hz timer)
    //===============================================================
    //===============================================================
    if( _time_ms - _timeProcess_ms > _processPeriod_ms )
    {
      if(DEBUG2) Serial.println("=== Process Loop ==="); 
      _timeProcess_ms  = _time_ms;

      //------------------------------------------------------------
      // Only run if KF has been initialized with the a GPS message
      if( _enableKF )
      {
        //------------------------------------------------------------
        // Kalman Filter Predict
        float dt = (_time_ms - _timeKF_ms) / 1000.0f;
        if(DEBUG2) {Serial.print("dt = "); Serial.println(dt);}
        if( dt > 0.005 ) // ignore if we just did a predict
        {      
          _timeKF_ms = _time_ms;
          _kf.predict(dt);
          if(DEBUG2) Serial.println("KF Predict"); 
        }
        else
        {
          if(DEBUG2) Serial.println("Skipping KF predict");
        }
        float distKF_m = _kf.position();
        float spdKF_mps = _kf.velocity();
        if(DEBUG2) {Serial.print("distKF_m = "); Serial.println(distKF_m);}
        if(DEBUG2) {Serial.print("spdKF_mps = "); Serial.println(spdKF_mps);}

      
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
