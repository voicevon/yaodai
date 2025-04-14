#include "motor.h"

Motor::Motor(int pin_num)
{
  pinMode(pin_num, OUTPUT);
  digitalWrite(pin_num_, LOW);

  pin_num_ = pin_num;
  is_enabled_ = false;
  viberate_ms_ = 500;
  total_period_ = 700; // 默认总周期为700ms
  state_ = IDLE;
  last_timestamp_ = 0;
}

void Motor::SpinOnce()
{
  if (state_ == IDLE)
  {
      digitalWrite(pin_num_, LOW);
      return;
  }
  else if (state_ == VIBERATING)
  {
    if (millis() - last_timestamp_ > viberate_ms_)
    {
      last_timestamp_ = millis();
      state_ = SILIENT;
      digitalWrite(pin_num_, LOW);
    }
  }
  else if (state_ == SILIENT)
  {
    if (millis() - last_timestamp_ > silient_ms_)
    {
      last_timestamp_ = millis();
      state_ = VIBERATING;
      digitalWrite(pin_num_, HIGH);
    }
  }
}



void Motor::SetVibrate(int viberate_ms)
{
  viberate_ms_ = viberate_ms;
  if (viberate_ms == 0) { // 如果设置的振动时间大于总周期，强制设置为总周期
    state_ = IDLE;
    return;
  }
  silient_ms_ = total_period_ - viberate_ms_; // 自动计算静音时间
  if (viberate_ms_ > 0) {
    state_ = VIBERATING;
    last_timestamp_ = millis();
  }
}

// 注意：  viberate_ms 将强制设置为 0 ！
void Motor::SetPeriod(int total_period)
{
  total_period_ = total_period;
  viberate_ms_ = 0; // 强制设置为 0
  silient_ms_ = total_period ;
  state_ = IDLE;
}