#ifndef TTFRONT_H
#define TTFRONT_H

class TtFront
{
    bool _front;

public:
    TtFront(bool value)
    {
        this->_front = value;
    }

    bool Front(bool value)
    {
        if (value != _front)
        {
            _front = value;
            // Serial.println("TtFront::Front = True");
            return true;
        }
        return false;
    }

    bool True(bool value)
    {
        if (value == true && value != _front)
        {
            _front = value;
            // Serial.println("TtFront::True = True");
            return true;
        }
        return false;
    }

    bool False(bool value)
    {
        if (value == false && value != _front)
        {
            _front = value;
            // Serial.println("TtFront::False = True");
            return true;
        }
        return false;
    }
};

#endif