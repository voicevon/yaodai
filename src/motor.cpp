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

void Motor::Start(){
    state_ = VIBERATING;
    digitalWrite(pin_num_, HIGH);
    last_timestamp_ = millis();
}

void Motor::Stop(){
    state_ = IDLE;
    digitalWrite(pin_num_, LOW);
    last_timestamp_ = millis();
}
void Motor::Enable_(bool is_enable)
{
  if (is_enable)
  {
    state_ = VIBERATING;
    digitalWrite(pin_num_, LOW);
  }
  else
  {
    state_ = IDLE;
    digitalWrite(pin_num_, HIGH);
  }
  last_timestamp_ = millis();
}

void Motor::SetVibrate(int viberate_ms, int total_period)
{
  viberate_ms_ = viberate_ms;
  total_period_ = total_period;
  silient_ms_ = total_period_ - viberate_ms_; // 自动计算静音时间
}