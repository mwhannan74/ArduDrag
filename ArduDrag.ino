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
  float distance_m = 0;
 

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
    Serial.print(distance_m);
    Serial.println("");
  }
} _dragState;


//============================================================================================
// Haversine distance between two lat/lon points in meters
//============================================================================================
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


//============================================================================================
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


//============================================================================================
//============================================================================================
//                                        SETUP
//============================================================================================
//============================================================================================
KalmanFilter _kf;
float _kfRate_hz = 20.0; // this needs to match processPeriod
float _kfRate_sec = 1.0 / _kfRate_hz;

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
  _kf.init(_kfRate_sec);
  _kf.setSigmaJ(0.45f); // 0.4f to 2.0f --> higher for poor GPS
  _kf.setR(/*sigma_p*/1.0f, /*sigma_v*/0.2f); // std dev (meters, mps)
  
  _timeStart_ms = millis();
}


//============================================================================================
//============================================================================================
//                                        LOOP
//============================================================================================
//============================================================================================
unsigned long _blinkPeriod_ms   = 1000; // 1Hz
unsigned long _processPeriod_ms = 1000.0f * _kfRate_sec;
unsigned long _printPeriod_ms   = 200;  // 5Hz

uint32_t _time_ms        = millis();  // number of milliseconds since the program started
uint32_t _timeLED_ms     = millis();  // 
uint32_t _timeProcess_ms = millis();
uint32_t _timeKF_ms      = millis();
uint32_t _timePrint_ms   = millis();

bool _isFirstTime = true;
bool _enableKF = false;

double lat0;
double lon0;
float sigma_p_smoothed = 2.0f;

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
  // Update on new RMC message (RMC has lat, lon, and SOG)
  if( GPS.fix && gpsRes == 2)
  {
    if( _isFirstTime )
    {
      _isFirstTime = false;
      _enableKF = true;

      lat0 = GPS.latitudeDegrees;
      lon0 = GPS.longitudeDegrees;
    }
    else
    {
      Serial.println("RMC Message"); 

      float gpsDist_m = haversineDistance_m(lat0, lon0, GPS.latitudeDegrees, GPS.longitudeDegrees);
      float gpsSpd_mps = GPS.speed * KNTS2MPS;
      Serial.print("gpsDist_m = "); Serial.println(gpsDist_m);
      Serial.print("gpsSpd_mps = "); Serial.println(gpsSpd_mps);

      // compute pos std dev based on GPS quality
      float sigma_p = computeSigmaP(GPS.fixquality, GPS.satellites, GPS.HDOP);
      Serial.print("sigma_p = "); Serial.println(sigma_p);

      // lowpass filter
      const float alpha = 0.2f;
      sigma_p_smoothed = (1.0f - alpha) * sigma_p_smoothed + alpha * sigma_p;
      Serial.print("sigma_p_smoothed = "); Serial.println(sigma_p_smoothed);

      // apply 
      _kf.setR(/*sigma_p*/sigma_p_smoothed, /*sigma_v*/0.2f); // keep speed fixed
      

      float dt = (_time_ms - _timeKF_ms) / 1000.0f;
      Serial.print("dt = "); Serial.println(dt);
      if( dt > 0.005 ) // ignore if we just did a predict
      {
        _timeKF_ms = _time_ms;
        _kf.predict(dt);
        Serial.println("KF Predict");
      }
      else
      {
        Serial.println("Skipping KF predict");
      }
      _kf.update(gpsDist_m, gpsSpd_mps);
      
      Serial.println("KF Update");
    }
  }

  //------------------------------------------------------------
  // PROCESS (20 Hz)
  if( _enableKF && (_time_ms - _timeProcess_ms > _processPeriod_ms) )
  {
    Serial.println("Process Loop"); 
    _timeProcess_ms  = _time_ms;

    // Kalman Filter
    float dt = (_time_ms - _timeKF_ms) / 1000.0f;
    Serial.print("dt = "); Serial.println(dt);
    if( dt > 0.005 ) // ignore if we just did a predict
    {      
      _timeKF_ms = _time_ms;
      _kf.predict(dt);
      Serial.println("KF Predict"); 
    }
    else
    {
      Serial.println("Skipping KF predict");
    }
    float distKF = _kf.position();
    float spdKF = _kf.velocity();

    Serial.print("distKF = "); Serial.println(distKF);
    Serial.print("spdKF = "); Serial.println(spdKF);

    //--------------------------------------------------------
    // Update the Drag State
    _dragState.time_sec    = float(_time_ms - _timeStart_ms) / 1000.0f;
    _dragState.numSat      = GPS.satellites;
    _dragState.lat_deg     = GPS.latitudeDegrees;
    _dragState.lon_deg     = GPS.longitudeDegrees;
    _dragState.speed_mps   = spdKF;
    _dragState.heading_deg = angleWrap_deg(GPS.angle);
    _dragState.distance_m  = distKF;
    

    //--------------------------------------------------------
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
    }

    _dragState.printCSV();
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
