#pragma once
#include <Arduino.h>


class Motor {
    public:
        // enum State{
        //    IDLE = 1,
        //    VIBERATING = 2,
        //    SILIENT = 3, 
        // };

        Motor(int pin_num, int period_ms);
        void Start();
        void Stop();
        void SetVibrate(int viberate_ms);
        void SpinOnce();
        // State GetState(){return state_;}


    private:
        int pin_num_;
        int viberate_ms_;
        // int silient_ms_;
		int period_ms_;
        // State state_;
        // unsigned long last_timestamp_;
};