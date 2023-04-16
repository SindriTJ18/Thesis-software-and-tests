#include <iostream>
#include "../header/leg_control.hpp"
#include <pigpio.h>

leg::leg(int speed, int dir1, int dir2, int end, int legbut)
{
   gpioInitialise();
   speedPin = speed;
   dirPin1 = dir1;
   dirPin2 = dir2;
   endstop = end;
   legbutton = legbut;
   
   gpioSetMode(speedPin, PI_OUTPUT);
   gpioSetMode(dirPin1, PI_OUTPUT);
   gpioSetMode(dirPin2, PI_OUTPUT);
   gpioSetMode(endstop, PI_INPUT);
   gpioSetMode(legbutton, PI_INPUT);
   gpioSetPullUpDown(legbutton, PI_PUD_UP); 
   gpioSetPullUpDown(endstop, PI_PUD_UP); 

}

void leg::extend(){
   gpioWrite(speedPin, 1);
   gpioWrite(dirPin1, 1);
   gpioWrite(dirPin2, 0);

}

void leg::retract(){
   gpioWrite(speedPin, 1);
   gpioWrite(dirPin1, 0);
   gpioWrite(dirPin2, 1);

}

void leg::stop(){
   gpioWrite(speedPin, 0);
   gpioWrite(dirPin1, 0);
   gpioWrite(dirPin2, 0);

}

void leg::lock(){
   gpioWrite(speedPin, 1);
   gpioWrite(dirPin1, 1);
   gpioWrite(dirPin2, 1);

}

void leg::home(){
   leg::extend();
   while(gpioRead(leg::endstop) == 1){
      time_sleep(0.01);
   }
   leg::retract();
   time_sleep(0.3);
   leg::extend();
   while(gpioRead(leg::endstop) == 1){
      time_sleep(0.01);
   }
   leg::retract();
   time_sleep(0.2);
   leg::lock();
   time_sleep(0.2);
   leg::stop();
}


void leg::land(){
   leg::extend();
   while(gpioRead(leg::legbutton) == 1){
      if (gpioRead(leg::endstop) != 1){
         break;
      }
      time_sleep(0.01);
   }
   std::cout << "touch";
   leg::lock();
   time_sleep(0.3);
   leg::stop();
}
