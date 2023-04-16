#include <pigpio.h>

#include <iostream>

#include "../header/receiver_control.hpp"

receiver::receiver(int rx_pin) {
  receiver_pin = rx_pin;
  uptick = 0;
  diff = 0;

  gpioSetMode(receiver_pin, PI_INPUT);
  gpioSetAlertFuncEx(receiver_pin, scanEx, this);
}

void receiver::scanEx(int gpio, int level, uint32_t tick, void *user) {
  /*
        Need a static callback to link with C.
     */

  receiver *mySelf = (receiver *)user;

  mySelf->scan(gpio, level, tick); /* Call the instance callback. */
}

void receiver::scan(int gpio, int level, uint32_t tick) {
  if (level == 1) {
    uptick = tick;
  }

  if (level == 0) {
    diff = tick - uptick;
  }
}

int receiver::getPos() {
  if (diff > 1900) {
    return 0;
  } else if (diff < 1100) {
    return 2;
  } else {
    return 1;
  }
}