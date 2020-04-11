#include "NeoTimer.h"

//default constructor
NeoTimer::NeoTimer(unsigned long _t = 1000) //Default 1 second interval if not specified
{
    this->_timer.time = _t;
    this->_timer.started = false;
    this->_timer.last = 0;
    this->_timer.remaining = 0;
}

//Default destructor
NeoTimer::~NeoTimer()
{
}

/*
 * Repeats a timer x times
 * Useful to execute a task periodically.
 * Usage:
 * if(timer.repeat(10)){
 * 	  do something 10 times, every second (default)
 * }
 */
boolean NeoTimer::repeat(int times)
{
    if (times != NEOTIMER_UNLIMITED)
    {
        // First repeat
        if (this->repetitions == NEOTIMER_UNLIMITED)
        {
            this->repetitions = times;
        }
        // Stop
        if (this->repetitions == 0)
        {
            return false;
        }

        if (this->repeat())
        {
            this->repetitions--;
            return true;
        }
        return false;
    }
    return this->repeat();
}

/*
 * Repeats a timer x times with a defined period
 * Useful to execute a task periodically.
 * Usage:
 * if(timer.repeat(10,5000)){
 * 	  do something 10 times, every 5 seconds
 * }
 */
boolean NeoTimer::repeat(int times, unsigned long _t)
{
    this->_timer.time = _t;
    return this->repeat(times);
}

/*
 * Repeats a timer indefinetely
 * Useful to execute a task periodically.
 * Usage:
 * if(timer.repeat()){
 * 	  do something indefinetely, every second (default)
 * }
 */
boolean NeoTimer::repeat()
{
    if (this->done())
    {
        this->reset();
        return true;
    }
    if (!this->_timer.started)
    {
        this->_timer.last = millis();
        this->_timer.started = true;
    }
    return false;
}

void NeoTimer::repeatReset()
{
    this->repetitions = -1;
}

/*
 * Checks if timer has finished
 * Returns true if it finished
 */
boolean NeoTimer::done()
{
    if (!this->_timer.started)
    {
        // Serial.println("NeoTimer::done false");
        return false;
    }

    unsigned long elapse = millis() - this->_timer.last;
    this->_timer.last = millis();
    if (elapse > 0)
    {
        this->_timer.remaining -= elapse;
    }

    if (elapse > 0 && this->_timer.remaining <= 0)
    {
        this->_timer.done = true;
        // Serial.println("NeoTimer::done true");
        return true;
    }

    // Serial.println("NeoTimer::done false");
    return false;
}

boolean NeoTimer::front()
{
    if (!this->_timer.done && done())
    {
        //Serial.println("NeoTimer::front true");
        return true;
    }
    else if (this->_timer.done)
    {
        done(); // update remaining
    }

    // Serial.println("NeoTimer::front false");
    return false;
}

/*
 * Sets a timer preset
 */
void NeoTimer::set(unsigned long t)
{
    this->_timer.time = t;
}

/*
 * Gets the timer preset
 */
unsigned long NeoTimer::get()
{
    return this->_timer.time;
}

/*
 * resets a timer
 */
void NeoTimer::reset()
{
    this->_timer.started = false;
    this->_timer.done = false;
    this->_timer.remaining = this->_timer.time;
}

/*
 * Start a timer
 */
void NeoTimer::start(unsigned long times = NEOTIMER_INDEFINITE)
{
    if (this->_timer.started == false)
    {
        if (times != NEOTIMER_INDEFINITE)
            this->set(times);
        this->reset();
        this->_timer.last = millis();
        this->_timer.started = true;
    }
}

/*
 * Stops a timer
 */
long NeoTimer::stop()
{
    this->_timer.started = false;
    return this->elapsed();
}
/*
 * Continues a stopped timer
 */
void NeoTimer::resume()
{
    if (!this->done())
    {
        this->start();
    }
}

/*
 * Indicates if the timer is active
 * but has not yet finished.
 */
boolean NeoTimer::waiting()
{
    return (this->_timer.started && !this->done()) ? true : false;
}

boolean NeoTimer::started()
{
    return this->_timer.started;
}

/*
 * return elapsed time
 */
unsigned long NeoTimer::elapsed()
{
    if (this->_timer.remaining > 0)
        return this->_timer.time - this->_timer.remaining;
    else
        return (this->_timer.remaining * -1) + this->_timer.time;
}

/*
 * return remaining time
 */
long NeoTimer::remaining()
{
    return this->_timer.remaining;
}
