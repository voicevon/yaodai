#pragma once
#include <Arduino.h>


class Motor {
    public:
        enum State{
           IDLE = 1,
           VIBERATING = 2,
           SILIENT = 3, 
        };

        Motor(int pin_num);
        void Start();
        void Stop();
        void SetVibrate(int viberate_ms, int silient_ms);
        void SpinOnce();
        State GetState(){return state_;}


    private:
        void Enable_(bool is_enable);
        int pin_num_;
        bool is_enabled_;
        int viberate_ms_;
        int silient_ms_;
        State state_;
        unsigned long last_timestamp_;
};