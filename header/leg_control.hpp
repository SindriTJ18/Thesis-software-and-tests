#ifndef LEG_CONTROL_HPP
#define LEG_CONTROL_HPP
#include <stdint.h>


class leg{
    int speedPin,dirPin1,dirPin2, endstop, legbutton;

    public:
    leg(int speedPin, int dirPin1, int dirPin2, int endstop,int legbutton);
    void extend();
    void retract();
    void stop();
    void lock();
    void home();
    void land();
    };
#endif