#include <Arduino.h>

class PID
{
public:
  PID(unsigned long dt_ms) : dt_ms_(dt_ms) {recomputeCoeffs();}

  [[nodiscard]] float update(float measurement)
  {
    float FilteredMeasurement = lowPassFilter(measurement);
    float error = setpoint_ - measurement;
    p_ = kp_ * error;
    i_ = constrain(i_ + (error * ki_dt_), -100, 100);
    d_ = (FilteredMeasurement - previousFilteredMeasurement_) * kd_invdt_;

    previousMeasurement_ = measurement;
    previousFilteredMeasurement_ = FilteredMeasurement;
    return constrain(p_ + i_ + d_, -100, 100);
  }

   // --- Gains (set in "per-second" units) ---
  void setKp(float kp) { kp_ = kp; }
  void setKi(float ki) { ki_ = ki; recomputeCoeffs(); }
  void setKd(float kd) { kd_ = kd; recomputeCoeffs(); }

  void setTarget(float setpoint) { setpoint_ = setpoint; }

  /**
   * @brief Sets the low-pass filter gain for derivative smoothing.
   *
   * A gain of 1.0 applies the full derivative, while 0.0 disables it.
   */
  void setLpfGain(float lpfGain)
  {
    lpfGain_ = constrain(lpfGain, 0, 1);
    oneMinusLpfGain_ = 1 - lpfGain_;
  }

  float getKp() { return kp_;}
  float getKi() { return ki_;}
  float getKd() { return kd_;}
  float getSp() { return setpoint_;}
  float getLpfGain() { return lpfGain_;}

  float lowPassFilter(float measurement)
  {
    return (oneMinusLpfGain_ * previousFilteredMeasurement_) +
           (lpfGain_ * measurement);
  }

  void paceLoop()
  {
    static unsigned long previousTime = 0;
    long delayTime = dt_ms_ - (millis() - previousTime);
    if (delayTime < 0)
    {
      Serial.println("WARNING, Loop can not execute at set rate");
      Serial.print("delayTime = ");
      Serial.println(delayTime);
    }
    else
    {
      delay(delayTime);
    }
    previousTime = millis();
  }

private:
  unsigned long dt_ms_ = 0;

  // User-set gains (per-second units)
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;

  // Precomputed fast-path constants
  float ki_dt_ = 0.0f;     // ki * dt_sec
  float kd_invdt_ = 0.0f;  // kd / dt_sec

  float p_ = 0;
  float i_ = 0;
  float d_ = 0;

  float lpfGain_ = 1;
  float oneMinusLpfGain_ = 0;

  float setpoint_ = 0;
  float previousMeasurement_ = 0;
  float previousFilteredMeasurement_ = 0;

   void recomputeCoeffs()
  {
    // Convert ms -> sec
    float dt_sec_ = (float)dt_ms_ * 0.001f;

    // Precomputed constants for efficient PID update()
    ki_dt_     = ki_ * dt_sec_;      
    kd_invdt_  = kd_ / dt_sec_;   
  }
};