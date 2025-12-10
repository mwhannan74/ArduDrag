#include "DragFSM.h"

// Fake KF state for testing
struct FakeDRagKF
{
  bool     navGood = true;
  float    speed_mps = 0.0f;
  float    distance_m = 0.0f;   // Model A: distance since KF reset
  uint32_t last_ms = 0;

  // Scenario control
  enum Phase { STOP1, ACCEL, HOLD, DECEL, STOP2, DONE } phase = STOP1;
  uint32_t phaseStart_ms = 0;

  // Tunables
  float accel_mps2 = 4.5f;     // ~0.4g-ish
  float accel2_mps2 = 1.5f;
  float decel_mps2 = 8.0f;
  float hold_speed_mps = 30.0f; // 30 mps = 67.1081 mph

  void begin(uint32_t now_ms)
  {
    last_ms = now_ms;
    phaseStart_ms = now_ms;
    phase = STOP1;
    speed_mps = 0.0f;
    distance_m = 0.0f;
    navGood = true;
  }

  // Call this each loop with the current time
  void update(uint32_t now_ms)
  {
    float dt = (now_ms - last_ms) * 0.001f;
    if (dt < 0) dt = 0;
    last_ms = now_ms;

    uint32_t phaseElapsed = now_ms - phaseStart_ms;

    switch (phase)
    {
      case STOP1:
        speed_mps = 0.2f;
        // Wait long enough for FSM to arm
        if (phaseElapsed > 3000)
        {
          phase = ACCEL;
          phaseStart_ms = now_ms;
        }
        break;

      case ACCEL:
        speed_mps += accel_mps2 * dt;
        if (speed_mps >= hold_speed_mps)
        {
          speed_mps = hold_speed_mps;
          phase = HOLD;
          phaseStart_ms = now_ms;
        }
        break;

      case HOLD:
        // hold
        //speed_mps = hold_speed_mps;

        // accel
        speed_mps += accel2_mps2 * dt;
        
        // Once we are clearly beyond the quarter mile, start decel
        if (distance_m >= 420.0f)
        {
          phase = DECEL;
          phaseStart_ms = now_ms;
        }
        break;

      case DECEL:
        speed_mps -= decel_mps2 * dt;
        if (speed_mps <= 0.0f)
        {
          speed_mps = 0.0f;
          phase = STOP2;
          phaseStart_ms = now_ms;
        }
        break;

      case STOP2:
        speed_mps = 0.0f;
        if (phaseElapsed > 3000)
        {
          phase = DONE;
        }
        break;

      case DONE:
        speed_mps = 0.0f;
        break;
    }

    // "Fake KF" distance integration for the test harness only
    distance_m += speed_mps * dt;
  }

  // Your DragFSM requests a KF reset at run start.
  // In the real system you'd reset the filter here.
  void resetDistance()
  {
    distance_m = 0.0f;
  }
};


