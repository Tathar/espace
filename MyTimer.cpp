
#include "MyTimer.h"

MyTimer::MyTimer()
{
    l_ok = false;
    active = false;
}

bool MyTimer::ok()
{
    if (active)
    {
        int temp = millis() - l_time;
        l_time = millis();
        if (temp > 0 && l_wait > temp)
        {
            l_wait -= temp;
        }
        else if (temp > 0)
        {
            active = false;
            return true;
        }
    }
    return false;
}

void MyTimer::set(unsigned long int time)
{
    l_wait_config = time;
}

void MyTimer::start(unsigned long int time = NONE)
{
    if (time != NONE)
        l_wait_config = time;

    l_wait = l_wait_config;
    l_time = millis();
    active = true;
    l_ok = false;
}

void MyTimer::stop()
{
    active = false;
}

void MyTimer::resume()
{
    l_time = millis();
    active = true;
}

bool MyTimer::is_active()
{
    return active;
}
