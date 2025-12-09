#pragma once
/*
 * 1D Kalman filter for ground vehicle distance-from-start tracking.
 *
 * This filter estimates a physically consistent kinematic state using a
 * constant-acceleration model with jerk-driven process noise:
 *
 *   State vector:
 *     x = [ p; v; a ]
 *       p = position / distance from start (m)
 *       v = velocity / speed (m/s)
 *       a = acceleration (m/s^2)
 *
 *   Measurement vector (when available):
 *     z = [ p_meas; v_meas ]
 *       typically from GPS-derived distance-from-start and a speed source.
 *
 * The filter runs in two steps:
 *   1) predict(dt):
 *      Propagates state and covariance forward by the actual elapsed time dt
 *      (seconds). The state transition A(dt) implements standard kinematics:
 *        p += v*dt + 0.5*a*dt^2
 *        v += a*dt
 *        a stays constant over the interval
 *
 *      Process noise Q(dt) is recomputed every predict using a white-jerk model
 *      (jerk = da/dt). The tuning parameter sigma_j (m/s^3) controls how
 *      quickly acceleration is allowed to change:
 *        higher sigma_j -> more responsive, noisier accel estimate
 *        lower sigma_j  -> smoother, more model-trusting behavior
 *
 *   2) update(z_pos, z_vel):
 *      Incorporates position and velocity measurements via:
 *        H maps state to measurements (p and v)
 *        R is measurement noise covariance
 *
 * Key matrices:
 *   A (3x3): state transition, rebuilt from dt each predict
 *   H (2x3): measurement model for [p, v]
 *   Q (3x3): process noise covariance from sigma_j and dt
 *   R (2x2): measurement noise covariance set from sensor std devs
 *   P (3x3): state estimate covariance
 *
 * Intended usage:
 *   - Call init(dt_nominal) once.
 *   - Call predict(dt_actual) at your preferred estimate rate (e.g., 20 Hz).
 *   - Call update() whenever new measurements arrive (e.g., ~10 Hz with jitter).
 *
 * This implementation avoids dynamic allocation and uses fixed-size arrays,
 * making it suitable for small MCUs such as Arduino.
 */

#include <Arduino.h>
#include <stdint.h>
#include <math.h>   // for fabsf, sqrtf

class KalmanFilter
{
public:
    KalmanFilter()
    : dt_nominal(0.0f),
      sigma_j(1.0f)
    {
        // Provide a safe default initialization.
        // Users should still call init(dt_nominal) before use.
        zeroMatricesAndState();
    }

    // Initialize filter parameters and state
    // dt_nominal is the design / nominal sample time (e.g., 0.05 for 20 Hz).
    //   --> The actual dt passed to predict() can vary around this.
    // Initial state: p0 (poistion) and v0 (velocity)
    // Uncertainty in state: std dev (sigma) for each state (pos and vel)
    void init(float dt_nominal_in, 
              float p0, float v0,
              float sigma_p0, float sigma_v0)
    {
        float dt2;

        dt_nominal = dt_nominal_in;

        // State transition for constant acceleration model:
        // p_k+1 = p_k + v_k*dt + 0.5*a_k*dt^2
        // v_k+1 = v_k + a_k*dt
        // a_k+1 = a_k
        dt2 = dt_nominal * dt_nominal * 0.5f;

        A[0][0] = 1.0f;      A[0][1] = dt_nominal;  A[0][2] = dt2;
        A[1][0] = 0.0f;      A[1][1] = 1.0f;        A[1][2] = dt_nominal;
        A[2][0] = 0.0f;      A[2][1] = 0.0f;        A[2][2] = 1.0f;

        // Measurement matrix: we measure position and velocity
        // z_pos = p, z_vel = v
        H[0][0] = 1.0f;  H[0][1] = 0.0f;  H[0][2] = 0.0f;
        H[1][0] = 0.0f;  H[1][1] = 1.0f;  H[1][2] = 0.0f;

        // Set sigma_j (jerk noise std dev).
        //
        // Previous version used diagonal Q with:
        //   const float q_a = 1e-2f;
        // tuned assuming dt_nominal.
        //
        // To preserve similar "acceleration uncertainty growth" at dt_nominal,
        // we map that legacy tuning to sigma_j using:
        //   Q_aa = sigma_j^2 * dt
        // => sigma_j^2 = q_a / dt_nominal
        //
        // You should still empirically tune sigma_j for your system.
        {
            const float q_a_legacy = 1e-2f;
            if (dt_nominal > 0.0f) {
                sigma_j = sqrtf(q_a_legacy / dt_nominal);
            } else {
                sigma_j = 1.0f; // safe fallback; dt_nominal should not be zero
            }
        }

        // Initialize Q using dt_nominal so the struct starts consistent.
        compute_Q_from_jerk(dt_nominal);

        // Set R (measurement noise covariance).
        // Tune these to reflect your sensor standard deviation.
        // Example: GPS position std dev ~1 m, speed std dev ~0.2 m/s.
        {
            const float sigma_p = 1.0f;
            const float sigma_v = 0.2f;

            R[0][0] = sigma_p * sigma_p;   R[0][1] = 0.0f;
            R[1][0] = 0.0f;               R[1][1] = sigma_v * sigma_v;
        }

        // Initialize state
        x[0] = p0;  // position
        x[1] = v0;  // velocity
        x[2] = 0;   // acceleration

        // Initialize covariance P with variance = sigma^2
        P[0][0] = sigma_p0*sigma_p0; P[0][1] = 0.0f;              P[0][2] = 0.0f;
        P[1][0] = 0.0f;              P[1][1] = sigma_v0*sigma_v0; P[1][2] = 0.0f;
        P[2][0] = 0.0f;              P[2][1] = 0.0f;              P[2][2] = 10.0f;
    }

    // Prediction step: x_k|k-1 = A(dt) x_{k-1|k-1}, P_k|k-1 = A(dt) P A(dt)^T + Q
    //
    // dt is the ACTUAL elapsed time (seconds) since the last state update.
    // This allows the filter to handle jitter and asynchronous sensor arrivals.
    void predict(float dt)
    {
        float x_pred[3];
        float P_pred[3][3];
        float AP[3][3];
        float dt2;
        float max_dt;
        int i, j, k;

        // Sanity-check / clamp dt.
        //
        // Rationale:
        // Very large dt (e.g., from loop stalls or sensor hiccups) can cause
        // large prediction jumps and overly inflated uncertainty. Clamping
        // helps maintain stability. Adjust the multiplier as appropriate.
        if (dt < 0.0f) {
            return;
        }

        max_dt = (dt_nominal > 0.0f) ? (5.0f * dt_nominal) : 0.5f;
        if (dt > max_dt) {
            dt = max_dt;
        }

        // Update A’s time-dependent elements using the actual dt.
        // This is the core change to support variable dt.
        dt2 = dt * dt * 0.5f;

        A[0][0] = 1.0f;  A[0][1] = dt;    A[0][2] = dt2;
        A[1][0] = 0.0f;  A[1][1] = 1.0f;  A[1][2] = dt;
        A[2][0] = 0.0f;  A[2][1] = 0.0f;  A[2][2] = 1.0f;

        // Compute a jerk-based, dt-scaled Q.
        compute_Q_from_jerk(dt);

        // x_pred = A * x
        for (i = 0; i < 3; i++) {
            x_pred[i] = 0.0f;
            for (j = 0; j < 3; j++) {
                x_pred[i] += A[i][j] * x[j];
            }
        }

        // AP = A * P
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                AP[i][j] = 0.0f;
                for (k = 0; k < 3; k++) {
                    AP[i][j] += A[i][k] * P[k][j];
                }
            }
        }

        // P_pred = AP * A^T + Q
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                P_pred[i][j] = 0.0f;
                for (k = 0; k < 3; k++) {
                    P_pred[i][j] += AP[i][k] * A[j][k];  // A^T[k][j] = A[j][k]
                }
                P_pred[i][j] += Q[i][j];
            }
        }

        // Store back
        for (i = 0; i < 3; i++) {
            x[i] = x_pred[i];
            for (j = 0; j < 3; j++) {
                P[i][j] = P_pred[i][j];
            }
        }
    }

    // Update step: use new measurement z = [z_pos, z_vel]
    void update(float z_pos, float z_vel)
    {
        float z[2];
        float y[2];        // innovation: y = z - H x_pred
        float S[2][2];     // innovation covariance: S = H P H^T + R
        float S_inv[2][2];
        float PHt[3][2];   // P H^T
        float K[3][2];     // Kalman gain: K = P H^T S^{-1}
        float hx[2];       // H x
        float P_old[3][3];
        float KH[3][3];
        float IminusKH[3][3];
        float P_new[3][3];
        float HP[2][3];
        float det;
        float inv_det;
        int i, j, k;

        z[0] = z_pos;
        z[1] = z_vel;

        // Save old P for later use in P update
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                P_old[i][j] = P[i][j];
            }
        }

        // hx = H * x
        for (i = 0; i < 2; i++) {
            hx[i] = 0.0f;
            for (j = 0; j < 3; j++) {
                hx[i] += H[i][j] * x[j];
            }
        }

        // Innovation y = z - hx
        for (i = 0; i < 2; i++) {
            y[i] = z[i] - hx[i];
        }

        // Compute S = H P H^T + R
        // First: HP = H * P  (2x3 = 2x3 * 3x3)
        for (i = 0; i < 2; i++) {
            for (j = 0; j < 3; j++) {
                HP[i][j] = 0.0f;
                for (k = 0; k < 3; k++) {
                    HP[i][j] += H[i][k] * P_old[k][j];
                }
            }
        }

        // S = HP * H^T + R  (2x2 = 2x3 * 3x2)
        for (i = 0; i < 2; i++) {
            for (j = 0; j < 2; j++) {
                S[i][j] = 0.0f;
                for (k = 0; k < 3; k++) {
                    S[i][j] += HP[i][k] * H[j][k]; // H^T[k][j] = H[j][k]
                }
                S[i][j] += R[i][j];
            }
        }

        // Invert S (2x2 matrix) with epsilon robustness
        det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        if (fabsf(det) < EPS) {
            // Degenerate or near-degenerate case.
            // In practice you could:
            //  - inflate R temporarily,
            //  - add a small diagonal to S,
            //  - or skip update as done here.
            return;
        }

        inv_det = 1.0f / det;

        S_inv[0][0] =  S[1][1] * inv_det;
        S_inv[0][1] = -S[0][1] * inv_det;
        S_inv[1][0] = -S[1][0] * inv_det;
        S_inv[1][1] =  S[0][0] * inv_det;

        // Compute PHt = P_old * H^T  (3x2 = 3x3 * 3x2)
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 2; j++) {
                PHt[i][j] = 0.0f;
                for (k = 0; k < 3; k++) {
                    PHt[i][j] += P_old[i][k] * H[j][k]; // H^T[k][j] = H[j][k]
                }
            }
        }

        // Kalman gain: K = PHt * S_inv  (3x2 = 3x2 * 2x2)
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 2; j++) {
                K[i][j] = 0.0f;
                for (k = 0; k < 2; k++) {
                    K[i][j] += PHt[i][k] * S_inv[k][j];
                }
            }
        }

        // Update state: x = x + K * y
        for (i = 0; i < 3; i++) {
            x[i] += K[i][0] * y[0] + K[i][1] * y[1];
        }

        // Update covariance: P = (I - K H) * P_old
        //
        // This is the "fast" form. For maximum numerical robustness, consider
        // switching to the Joseph form:
        //   P = (I-KH)P(I-KH)^T + K R K^T
        //
        // We add a symmetry enforcement step afterward to reduce numeric drift.

        // First compute KH = K * H  (3x3 = 3x2 * 2x3)
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                KH[i][j] = 0.0f;
                for (k = 0; k < 2; k++) {
                    KH[i][j] += K[i][k] * H[k][j];
                }
            }
        }

        // IminusKH = I - KH
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                float Iij = (i == j) ? 1.0f : 0.0f;
                IminusKH[i][j] = Iij - KH[i][j];
            }
        }

        // P_new = IminusKH * P_old  (3x3 = 3x3 * 3x3)
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                P_new[i][j] = 0.0f;
                for (k = 0; k < 3; k++) {
                    P_new[i][j] += IminusKH[i][k] * P_old[k][j];
                }
            }
        }

        // Store back P_new
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                P[i][j] = P_new[i][j];
            }
        }

        // Cheap numeric stability: enforce symmetry after update.
        symmetrize3(P);
    }

    // Optional convenience accessors for Arduino sketches.
    float position() const { return x[0]; }
    float velocity() const { return x[1]; }
    float acceleration() const { return x[2]; }

    // Expose state/covariance if you prefer direct access.
    // These are kept public-accessible via getters to avoid breaking encapsulation
    // too aggressively, while still supporting fixed-size, no-allocation behavior.
    const float* state() const { return x; }
    const float (*covariance() const)[3] { return P; }

    // Sets jerk noise std dev (m/s^3).
    //   Higher -> more responsive to accel changes
    //   Lower -> smoother/more model-trusting (more filtering)
    void setSigmaJ(float sj = 0.45f) // 0.4f to 2.0f
    { sigma_j = sj; }

    // Sets measurement noise *std devs* for position (m) and velocity (m/s).
    //   Higher -> trust measurements less (more filtering) 
    //   Lower -> trust measurements more
    void setR(float sigma_p=1.0f, float sigma_v=0.2f)
    {
        R[0][0] = sigma_p * sigma_p;   R[0][1] = 0.0f;
        R[1][0] = 0.0f;               R[1][1] = sigma_v * sigma_v;
    }

private:
    float EPS = 1e-9f; // epsilon near zero
    float dt_nominal = 0.05; // nominal sample time used for initial tuning (seconds)
    float sigma_j; // Jerk noise std dev for process model (not measurements!)

    // State transition matrix A (3x3)
    // NOTE: A is updated inside predict() based on the actual dt used.
    float A[3][3];

    // Measurement matrix H (2x3)
    // z = H * x  (z is [pos; vel])
    float H[2][3];

    // Process noise covariance Q (3x3)
    //
    // We assume "jerk" (time-derivative of acceleration) is white noise.
    // For state [p, v, a], the discrete-time Q derived from white jerk with variance sigma_j^2 is:
    //
    //   Q = sigma_j^2 * [dt^5/20, dt^4/8,  dt^3/6
    //                    dt^4/8,  dt^3/3,  dt^2/2
    //                    dt^3/6,  dt^2/2,  dt]
    //
    // This Q is recomputed each predict step using the actual dt.
    float Q[3][3];

    // Measurement noise covariance R (2x2)
    float R[2][2];

    // State estimate x = [p, v, a]^T
    float x[3];

    // Estimate covariance P (3x3)
    float P[3][3];

    // Small helper to symmetrize a 3x3 matrix in-place (cheap numeric stability).
    static void symmetrize3(float M[3][3])
    {
        int i, j;
        for (i = 0; i < 3; i++) {
            for (j = i + 1; j < 3; j++) {
                float sym = 0.5f * (M[i][j] + M[j][i]);
                M[i][j] = sym;
                M[j][i] = sym;
            }
        }
    }

    // Recompute Q for a given dt using the jerk-driven model.
    void compute_Q_from_jerk(float dt)
    {
        float dt2, dt3, dt4, dt5;
        float s2;

        dt2 = dt * dt;
        dt3 = dt2 * dt;
        dt4 = dt3 * dt;
        dt5 = dt4 * dt;

        s2 = sigma_j * sigma_j;

        Q[0][0] = s2 * (dt5 * (1.0f / 20.0f));
        Q[0][1] = s2 * (dt4 * (1.0f / 8.0f));
        Q[0][2] = s2 * (dt3 * (1.0f / 6.0f));

        Q[1][0] = Q[0][1];
        Q[1][1] = s2 * (dt3 * (1.0f / 3.0f));
        Q[1][2] = s2 * (dt2 * 0.5f);

        Q[2][0] = Q[0][2];
        Q[2][1] = Q[1][2];
        Q[2][2] = s2 * dt;
    }

    void zeroMatricesAndState()
    {
        int i, j;
        for (i = 0; i < 3; i++) {
            x[i] = 0.0f;
            for (j = 0; j < 3; j++) {
                A[i][j] = 0.0f;
                Q[i][j] = 0.0f;
                P[i][j] = 0.0f;
            }
        }
        for (i = 0; i < 2; i++) {
            for (j = 0; j < 3; j++) {
                H[i][j] = 0.0f;
            }
            for (j = 0; j < 2; j++) {
                R[i][j] = 0.0f;
            }
        }
    }
};


//=====================================================================================
// Example usage pattern (updated for this C++ class):
//=====================================================================================
/*
#include <Arduino.h>
#include "KalmanFilter1D.h"  // whatever you name this file

KF::KalmanFilter kf;

uint32_t last_state_ms = 0;

void setup() {
    float dt_nominal = 0.05f;
    kf.init(dt_nominal);
    last_state_ms = millis();
}

void loop() {
    uint32_t now_ms = millis();

    if (now_ms - last_state_ms >= 50) {
        float dt = (now_ms - last_state_ms) / 1000.0f;
        last_state_ms = now_ms;

        kf.predict(dt);

        // Now kf.position(), kf.velocity(), kf.acceleration()
        // at time 'now_ms'.
    }

    // On measurement arrival:
    // uint32_t meas_ms = millis();
    // float dt_meas = (meas_ms - last_state_ms) / 1000.0f;
    // if (dt_meas > 0.0f) {
    //     kf.predict(dt_meas);
    //     last_state_ms = meas_ms;
    // }
    // kf.update(z_pos, z_vel);
}
*/
