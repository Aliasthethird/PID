#include <Arduino.h>

class PID
{
public:
  PID(unsigned long dt) : dt_(dt) {}

  [[nodiscard]] float update(float measurement)
  {
    float FilteredMeasurement = lowPassFilter(measurement);
    float error = setpoint_ - measurement;
    p_ = kp_ * error;
    i_ = constrain(i_ + (ki_ * error * dt_), -100, 100);
    d_ = kd_ * (FilteredMeasurement - previousFilteredMeasurement_) / dt_;

    previousMeasurement_ = measurement;
    previousFilteredMeasurement_ = FilteredMeasurement;
    return constrain(p_ + i_ + d_, -100, 100);
  }

  void setKd(float kd)
  {
    kd_ = kd;
  }

  void setKp(float kp)
  {
    kp_ = kp;
  }

  void setKi(float ki)
  {
    ki_ = ki;
  }

  void setTarget(float setpoint)
  {
    setpoint_ = setpoint;
  }

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

  float getKp()
  {
    return kp_;
  }
  float getKi()
  {
    return ki_;
  }
  float getKd()
  {
    return kd_;
  }
  float getSp()
  {
    return setpoint_;
  }
  float getLpfGain()
  {
    return lpfGain_;
  }

  float lowPassFilter(float measurement)
  {
    return (oneMinusLpfGain_ * previousFilteredMeasurement_) +
           (lpfGain_ * measurement);
  }

  void paceLoop()
  {
    static unsigned long previousTime = 0;
    long delayTime = dt_ - (millis() - previousTime);
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
  unsigned long dt_ = 0;
  float kp_ = 0;
  float ki_ = 0;
  float kd_ = 0;
  float p_ = 0;
  float i_ = 0;
  float d_ = 0;

  float lpfGain_ = 1;
  float oneMinusLpfGain_ = 0;

  float setpoint_ = 0;
  float previousMeasurement_ = 0;
  float previousFilteredMeasurement_ = 0;
};