#ifndef RECEIVER_CONTROL_HPP
#define RECEIVER_CONTROL_HPP
#include <stdint.h>

class receiver{
    int receiver_pin, switch_state;
    uint32_t uptick;
    uint32_t diff;
    
    void scan(int gpio, int level, uint32_t tick);
    static void scanEx(int gpio, int level, uint32_t tick,void *user);
    public:
    receiver(int receiver_pin);
    int getPos();
    };
#endif