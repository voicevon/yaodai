#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    enum State {
        IDLE,
        VIBERATING,
        SILIENT
    };

    Motor(int pin_num);
    void SpinOnce();
    void SetVibrate(int viberate_ms); 
    void SetPeriod(int total_period_ms); 

private:
    
    int pin_num_;
    bool is_enabled_;
    int viberate_ms_;
    int total_period_;  // 总周期
    int silient_ms_;    // 静音时间
    State state_;
    unsigned long last_timestamp_;
};

#endif