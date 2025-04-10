#include "motor.h"

void Motor::SpinOnce(){
    if (state_==IDLE){
        return;
    } else if (state_== VIBERATING){
        if (millis() - last_timestamp_ > viberate_ms_){
            last_timestamp_ = millis();
            state_ = SILIENT;
            digitalWrite(pin_num_, LOW); 
        }
    } else if (state_== SILIENT){
        if (millis() - last_timestamp_ > silient_ms_){
            last_timestamp_ = millis();
            state_ = VIBERATING;
            digitalWrite(pin_num_, HIGH); 
        } 
    }
    {
        /* code */
    }
    
    
}

Motor::Motor(int pin_num){
  pinMode(pin_num, OUTPUT);
  digitalWrite(pin_num_, LOW);

  pin_num_ = pin_num;
  is_enabled_ = false;
  viberate_ms_ = 500;
  silient_ms_ = 200;
  state_ = IDLE;
  last_timestamp_ = 0;
}

void Motor::Start(){
  Enable_(true);
}

void Motor::Stop(){
  Enable_(false);  
}
void Motor::Enable_(bool is_enable){
  if(is_enable){
    state_ = VIBERATING;
    digitalWrite(pin_num_, HIGH);
  } else {
    state_ = IDLE;
    digitalWrite(pin_num_, LOW); 
  }
  last_timestamp_ = millis();
}


void Motor::SetVibrate(int viberate_ms, int silient_ms){
    viberate_ms_ = viberate_ms; 
    silient_ms_ = silient_ms;
}