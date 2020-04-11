#ifndef MY_TIMER_H
#define MY_TIMER_H

#include "Arduino.h"
#define NONE 0

class MyTimer
{
public:
    MyTimer();
    bool ok();

    void set(unsigned long int time);

    void start(unsigned long int time = NONE);

    void stop();

    void resume();

    bool is_active();

private:
    bool active;
    unsigned long int l_time;
    unsigned long int l_wait_config;
    int l_wait;
    bool l_ok;
};

#endif