#include "motor.h"

#define ENABLE_MOTOR LOW
#define DISABLE_MOTOR HIGH


void Motor::SpinOnce(){
    // if (state_==IDLE){
    //     return;
    // } //else if (state_== VIBERATING){
    //     if (millis() - last_timestamp_ > viberate_ms_){
    //         last_timestamp_ = millis();
    //         state_ = SILIENT;
    //         digitalWrite(pin_num_, LOW); 
    //     }
    // } else if (state_== SILIENT){
    //     if (millis() - last_timestamp_ > silient_ms_){
    //         last_timestamp_ = millis();
    //         state_ = VIBERATING;
    //         digitalWrite(pin_num_, HIGH); 
    //     } 
    // }
    // {
    //     /* code */
    // }
    
    int xx = millis() % period_ms_;
    if (xx < viberate_ms_){
        digitalWrite(pin_num_, ENABLE_MOTOR); 
    } else {
        digitalWrite(pin_num_, DISABLE_MOTOR);
    }
    
}

Motor::Motor(int pin_num, int period_ms){
  pin_num_ = pin_num;
  pinMode(pin_num, OUTPUT);
  digitalWrite(pin_num, LOW);

  period_ms_ = period_ms;
  viberate_ms_ = 0;
  // silient_ms_ = period_ms;
  // state_ = IDLE;
  // last_timestamp_ = 0;
}

void Motor::Start(){
    digitalWrite(pin_num_, ENABLE_MOTOR);
}

void Motor::Stop(){
    digitalWrite(pin_num_, DISABLE_MOTOR);
}




void Motor::SetVibrate(int viberate_ms){
    viberate_ms_ = viberate_ms; 
    // if (viberate_ms_ <  10 ){
    //     state_ = SILIENT;
    //     digitalWrite(pin_num_, DISABLE_MOTOR);
    // }
}